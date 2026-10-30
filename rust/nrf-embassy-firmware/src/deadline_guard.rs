//! ARM-private TIMER4/PPI owner. Scheduled TX is installed, never called by RX.

use core::cell::RefCell;
use core::ptr::NonNull;
use critical_section::Mutex;
use embassy_nrf::interrupt::typelevel::{Interrupt, RADIO, TIMER4};
use embassy_nrf::ppi::{Ppi, PpiGroup, Task};
use embassy_nrf::timer::{Frequency, Timer};
use embassy_nrf::{Peri, pac, peripherals};
use embassy_nrf_esb::{
    CompletionPolicy as VendorCompletionPolicy, DeadlineGuardError, OperationEpoch,
    TxDeadlineGuard, TxLaunch,
};
use nrf_embassy_firmware::deadline::{Error, Registers, State};

use crate::shared_clock::SharedClock;

#[cfg(feature = "sync-trace")]
use embassy_nrf::ppi::Event;
#[cfg(feature = "sync-trace")]
use embassy_nrf_esb::{TxEndCapture, TxEndCaptureReason};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::tx_end_capture::{Action, CandidateValidity, ExclusiveOwner, Frozen};

/// Unique deadline leases moved together into the same static owner.
pub(crate) struct Resources {
    pub(crate) timer: Peri<'static, peripherals::TIMER4>,
    pub(crate) ch3: Peri<'static, peripherals::PPI_CH3>,
    pub(crate) ch4: Peri<'static, peripherals::PPI_CH4>,
    pub(crate) group0: Peri<'static, peripherals::PPI_GROUP0>,
    #[cfg(feature = "sync-trace")]
    pub(crate) ch5: Peri<'static, peripherals::PPI_CH5>,
}

#[cfg(feature = "sync-trace")]
enum CaptureRoute {
    Ready(Ppi<'static, peripherals::PPI_CH5, 1, 1>),
    // Retain the unique lease, with inert endpoints, if another group inherited
    // CH5. Observing membership does not authorize modifying that group.
    Unavailable {
        _lease: Peri<'static, peripherals::PPI_CH5>,
    },
}

struct Hardware {
    timer: Timer<'static>,
    cutoff: Ppi<'static, peripherals::PPI_CH3, 1, 2>,
    _start: Ppi<'static, peripherals::PPI_CH4, 1, 1>,
    group: PpiGroup<'static, peripherals::PPI_GROUP0>,
    disable: Task<'static>,
    #[cfg(feature = "sync-trace")]
    capture_route: CaptureRoute,
    #[cfg(feature = "sync-trace")]
    latest_pair: Option<(u32, u64)>,
}

impl Registers for Hardware {
    fn pair(&mut self) -> Option<(u32, u64)> {
        // Conservative early mapping: T4 capture precedes T0 capture/extension.
        let raw = self.timer.cc(3).capture();
        let now = SharedClock::read_static().ok()?;
        let pair = (raw, u64::try_from(now).ok()?);
        #[cfg(feature = "sync-trace")]
        {
            self.latest_pair = Some(pair);
        }
        Some(pair)
    }
    fn group_off(&mut self) {
        self.group.disable_all();
        let _ = pac::PPI.chen().read();
    }
    fn group_on(&mut self) {
        self.group.enable_all();
        let _ = pac::PPI.chen().read();
    }
    fn group_enabled(&mut self) -> bool {
        pac::PPI.chen().read().ch(4)
    }
    fn cutoff_route(&mut self, enabled: bool) {
        if enabled {
            self.cutoff.enable();
        } else {
            self.cutoff.disable();
        }
        let _ = pac::PPI.chen().read();
    }
    fn deadline_interrupt(&mut self, enabled: bool) {
        if enabled {
            pac::TIMER4.intenset().write(|w| w.set_compare(0, true));
        } else {
            pac::TIMER4
                .intenclr()
                .write_value(pac::timer::regs::Int(0xffff_ffff));
        }
    }
    fn clear_pending(&mut self) {
        TIMER4::unpend();
    }
    fn event(&mut self, index: usize) -> bool {
        pac::TIMER4.events_compare(index).read() != 0
    }
    fn clear_event(&mut self, index: usize) {
        self.timer.cc(index).clear_events();
    }
    fn write_cc(&mut self, index: usize, raw: u32) {
        self.timer.cc(index).write(raw);
    }
    fn read_cc(&mut self, index: usize) -> u32 {
        self.timer.cc(index).read()
    }
    fn radio_disable(&mut self) {
        // Only overdue/fault cutoff, never CPU TXEN. DMA stays with the facade.
        self.disable.trigger();
        cortex_m::asm::dsb();
    }
}

struct Owner {
    hardware: Hardware,
    state: State<OperationEpoch>,
    #[cfg(feature = "sync-trace")]
    capture: ExclusiveOwner,
    #[cfg(feature = "sync-trace")]
    anchor_now: Option<(u64, u64)>,
    #[cfg(feature = "sync-trace")]
    diagnostic_failures: u32,
}

#[cfg(feature = "sync-trace")]
impl Owner {
    fn diagnostic_failure(&mut self) {
        self.diagnostic_failures = self.diagnostic_failures.saturating_add(1);
    }

    fn capture_off(&mut self) {
        match &mut self.hardware.capture_route {
            CaptureRoute::Ready(route) => route.disable(),
            CaptureRoute::Unavailable { .. } => {
                pac::PPI.chenclr().write(|w| w.set_ch(5, true));
            }
        }
        let _ = pac::PPI.chen().read();
        cortex_m::asm::dsb();
        // Ordering of CPU MMIO only: neither CHEN readback nor DSB proves a
        // drained PPI pipeline. Equal CC reads are observational candidates.
    }

    fn prepare_capture(&mut self, epoch: OperationEpoch) {
        if !matches!(&self.hardware.capture_route, CaptureRoute::Ready(_)) {
            self.diagnostic_failure();
            return;
        }
        let Some((raw, now)) = self.hardware.latest_pair else {
            self.diagnostic_failure();
            return;
        };
        let Some(cutoff) = self.state.raw_cutoff(epoch) else {
            self.diagnostic_failure();
            return;
        };
        let Ok(horizon) = u32::try_from(embassy_nrf_esb::esb::SEND_DEADLINE_US) else {
            self.diagnostic_failure();
            return;
        };
        match self.capture.prepare(epoch.value(), raw, cutoff, horizon) {
            Ok(Action::Seed {
                epoch: action_epoch,
                raw: seed,
            }) if action_epoch == epoch.value()
                && self
                    .capture
                    .current()
                    .is_some_and(|s| s.epoch == action_epoch) =>
            {
                self.capture_off();
                self.hardware.timer.cc(2).write(seed);
                self.anchor_now = Some((action_epoch, now));
            }
            _ => self.diagnostic_failure(),
        }
    }

    fn arm_capture(&mut self, epoch: OperationEpoch) {
        // Vendor RADIO preparation has already cleared END; no RADIO register
        // is read/cleared here. This runs before State can program START/TXEN.
        if self
            .capture
            .current()
            .is_none_or(|s| s.epoch != epoch.value())
        {
            return;
        }
        match self.capture.arm(epoch.value()) {
            Ok(Action::EnableRoute {
                epoch: action_epoch,
            }) if action_epoch == epoch.value()
                && self
                    .capture
                    .current()
                    .is_some_and(|s| s.epoch == action_epoch) =>
            {
                if let CaptureRoute::Ready(route) = &mut self.hardware.capture_route {
                    route.enable();
                }
            }
            _ => self.diagnostic_failure(),
        }
    }

    fn freeze_capture(&mut self, epoch: u64, completion: bool) -> Option<Frozen> {
        // Reject stale operations BEFORE any diagnostic MMIO, even while a
        // newer route is active. A retained frozen copy requires no MMIO.
        if self.capture.current().is_none_or(|s| s.epoch != epoch) {
            return self.capture.frozen().filter(|f| f.snapshot.epoch == epoch);
        }
        match self.capture.disarm(epoch) {
            Ok(Action::DisableRoute {
                epoch: action_epoch,
            }) if action_epoch == epoch
                && self.capture.current().is_some_and(|s| s.epoch == epoch) => {}
            _ => {
                self.diagnostic_failure();
                return None;
            }
        }
        self.capture_off();
        let samples = [
            self.hardware.timer.cc(2).read(),
            self.hardware.timer.cc(2).read(),
        ];
        // Extra DIAGNOSTIC TIMER4 CC3 + SharedClock capture under the owner/IRQ
        // CS. Its cost and clock/capture latency are explicitly unqualified.
        let observation = self.hardware.pair().and_then(|(raw, now)| {
            let (anchor_epoch, anchor_now) = self.anchor_now?;
            (anchor_epoch == epoch).then_some(())?;
            Some((raw, now.checked_sub(anchor_now)?))
        });
        self.anchor_now = None;
        let Some((raw, elapsed)) = observation else {
            // DiagnosticUnavailable: no fabricated observation or send fault.
            let _ = self.capture.retire(epoch);
            self.diagnostic_failure();
            return None;
        };
        match self
            .capture
            .freeze_at(epoch, samples, raw, elapsed, completion)
        {
            Ok(frozen) => Some(frozen),
            Err(_) => {
                let _ = self.capture.retire(epoch);
                self.diagnostic_failure();
                None
            }
        }
    }

    fn retire_current_capture(&mut self) {
        if let Some(current) = self.capture.current() {
            let _ = self.freeze_capture(current.epoch, false);
        }
    }
}

#[cfg(feature = "sync-trace")]
fn capture_report(epoch: OperationEpoch, frozen: Frozen) -> Option<TxEndCapture> {
    if frozen.snapshot.epoch != epoch.value() {
        return None;
    }
    let (reason, cutoff_delta_us) = match frozen.validity {
        CandidateValidity::Missing => (TxEndCaptureReason::Missing, None),
        CandidateValidity::Unstable => (TxEndCaptureReason::Unstable, None),
        CandidateValidity::NoCompletionEvidence => (TxEndCaptureReason::NoCompletionEvidence, None),
        CandidateValidity::OutsideEpochRange => (TxEndCaptureReason::OutsideEpochRange, None),
        CandidateValidity::ObservationOutsideHorizon => {
            (TxEndCaptureReason::ObservationOutsideHorizon, None)
        }
        CandidateValidity::NotArmed => (TxEndCaptureReason::NotArmed, None),
        CandidateValidity::Candidate {
            cutoff_delta_us, ..
        } => (TxEndCaptureReason::Candidate, Some(cutoff_delta_us)),
        CandidateValidity::WrongEpoch => return None,
    };
    Some(TxEndCapture {
        epoch,
        anchor_raw: frozen.snapshot.anchor_raw,
        cutoff_raw: frozen.snapshot.cutoff_raw,
        sampled_cc: frozen.sampled_cc,
        observation_raw: frozen.actual_observation_raw,
        elapsed_us: frozen.elapsed_us,
        horizon_us: frozen.snapshot.horizon_us,
        reason,
        cutoff_delta_us,
    })
}
impl Drop for Owner {
    fn drop(&mut self) {
        self.state.disarm(&mut self.hardware);
        #[cfg(feature = "sync-trace")]
        self.capture_off();
        TIMER4::disable();
        TIMER4::unpend();
        // Hardware's Timer lease Drop stops the counter only at owner teardown.
    }
}
static OWNER: Mutex<RefCell<Option<Owner>>> = Mutex::new(RefCell::new(None));
pub(crate) struct Provider;
static PROVIDER: Provider = Provider;

fn with_owner<R>(f: impl FnOnce(&mut Owner) -> R) -> R {
    use cortex_m::peripheral::scb::{Exception, VectActive};
    // PRIMASK cannot serialize NMI/HardFault. Contract violations fail-stop
    // before taking CS or a state loan; no facade/actor call is made here.
    assert!(!matches!(
        cortex_m::peripheral::SCB::vect_active(),
        VectActive::Exception(Exception::NonMaskableInt | Exception::HardFault)
    ));
    critical_section::with(|cs| {
        let mut owner = OWNER.borrow(cs).borrow_mut();
        f(owner.as_mut().expect("deadline owner installed"))
    })
}

fn error(error: Error) -> DeadlineGuardError {
    match error {
        Error::Expired => DeadlineGuardError::Expired,
        Error::Unavailable => DeadlineGuardError::Unavailable,
        Error::HardwareFault => DeadlineGuardError::HardwareFault,
    }
}

// SAFETY: same uniquely consumed RADIO as the static facade; raw endpoints are
// private validated MMIO tasks. TIMER4/CH3/CH4/GROUP0 are consumed once and held
// in static CS Mutex/RefCell state. All hooks serialize on one core, never enter
// facade/Runtime/TDMA or poll wakers; NMI/fault use is forbidden. TIMER4/RADIO/T2
// share nonnesting P2. Only arm enables GROUP0; errors disable pending routes
// before returning. IRQ may only pend RADIO after releasing the owner loan.
// Source programming latency and half-wrap stall liveness remain unqualified.
// With guarded-completion-retention only, the strengthened unsafe obligation is
// that every successful arm establishes an autonomous current-TX cutoff effective
// no later than the requested monotonic deadline and maintained until completion
// disarm, preserving every reached matching expiry, including disarm/restoration
// races. TIMER4 runs continuously at 1 MHz; pair() captures TIMER4 before reading
// the current SharedClock TIMER0 physical microsecond domain used by admission.
// Adding deadline - now to that earlier TIMER4 capture is a conservative early
// source mapping under the current clock contract, independent of TDMA profiles.
// CC0/CH3 drives RADIO DISABLE and GROUP0 DISABLE; CH3 is outside GROUP0, and only
// arm enables launch gating. The existing disarm samples latched compare expiry
// before clearing events and retains it until the next arm. These source mappings
// and protocol checks do not prove physical propagation latency, clock agreement,
// or expiry preservation in actual hardware races. Hardware qualification of the
// strengthened obligation remains unmet. This feature is for controlled
// experimental validation, not release qualification or an attested timing proof.
// With sync-trace only, the same owner also consumes CH5 and TIMER4 CC2; its
// private END endpoint is address-only and capture diagnostics never decide a
// send outcome. Route-disable and extra clock work may perturb trace timing.
unsafe impl TxDeadlineGuard for Provider {
    fn completion_policy(&self) -> VendorCompletionPolicy {
        #[cfg(feature = "guarded-completion-retention")]
        {
            VendorCompletionPolicy::RetainGuardedCompletion
        }
        #[cfg(not(feature = "guarded-completion-retention"))]
        {
            VendorCompletionPolicy::StrictSoftwareDeadline
        }
    }

    fn arm(&self, epoch: OperationEpoch, deadline_us: u64) -> Result<(), DeadlineGuardError> {
        with_owner(|owner| {
            #[cfg(feature = "sync-trace")]
            owner.retire_current_capture();
            let result = owner.state.arm(&mut owner.hardware, epoch, deadline_us);
            #[cfg(feature = "sync-trace")]
            if result.is_ok() {
                owner.prepare_capture(epoch);
            }
            result
        })
        .map_err(error)
    }
    fn launch(&self, epoch: OperationEpoch) -> Result<TxLaunch, DeadlineGuardError> {
        with_owner(|owner| {
            #[cfg(feature = "sync-trace")]
            {
                owner.arm_capture(epoch);
                let result = owner
                    .state
                    .launch_before_cleanup(&mut owner.hardware, epoch);
                if let Err(error) = result {
                    // Preserve State::fail ordering, including overdue RADIO
                    // disable, before diagnostic reads can delay guard cleanup.
                    let error = owner.state.cleanup_launch_error(&mut owner.hardware, error);
                    // CC2 survives guard cleanup; freeze without completion
                    // evidence before the facade can restore RX.
                    if owner
                        .capture
                        .current()
                        .is_some_and(|s| s.epoch == epoch.value())
                    {
                        let _ = owner.freeze_capture(epoch.value(), false);
                    } else {
                        owner.capture_off();
                    }
                    return Err(error);
                }
                result
            }
            #[cfg(not(feature = "sync-trace"))]
            owner.state.launch(&mut owner.hardware, epoch)
        })
        .map(|()| TxLaunch::Scheduled)
        .map_err(error)
    }
    fn disarm(&self) {
        with_owner(|owner| {
            // Guard routes/expiry first. Compare-event clearing preserves CC2;
            // diagnostic freeze still finishes before the facade restores RX.
            owner.state.disarm(&mut owner.hardware);
            #[cfg(feature = "sync-trace")]
            owner.retire_current_capture();
        });
    }
    #[cfg(feature = "sync-trace")]
    fn capture_end(&self, epoch: OperationEpoch) -> Option<TxEndCapture> {
        // Only the vendor's actual current-TX predicate supplies this evidence.
        // Reclassify a copy, never sample or mutate the retained/newer owner.
        with_owner(|owner| {
            owner
                .capture
                .frozen()
                .filter(|f| f.snapshot.epoch == epoch.value())
        })
        .map(Frozen::with_completion_evidence)
        .and_then(|frozen| capture_report(epoch, frozen))
    }
    fn expired(&self, epoch: OperationEpoch) -> bool {
        with_owner(|owner| owner.state.expired(&mut owner.hardware, epoch))
    }
    fn on_interrupt(&self) {
        if with_owner(|owner| owner.state.on_interrupt(&mut owner.hardware)) {
            RADIO::pend();
        }
    }
}

/// Consume the resource bundle by value; all leases remain with one static owner.
pub(crate) fn initialize(
    radio: &Peri<'static, peripherals::RADIO>,
    resources: Resources,
) -> &'static Provider {
    let Resources {
        timer,
        ch3,
        ch4,
        group0,
        #[cfg(feature = "sync-trace")]
        ch5,
    } = resources;
    let _same_radio = radio;
    #[cfg(feature = "sync-trace")]
    let inherited_capture_group = {
        // Neutralize our newly consumed CH5 before any endpoint/timer changes.
        pac::PPI.chenclr().write(|w| w.set_ch(5, true));
        let _ = pac::PPI.chen().read();
        cortex_m::asm::dsb();
        pac::PPI.fork(5).tep().write_value(0);
        pac::PPI.ch(5).eep().write_value(0);
        pac::PPI.ch(5).tep().write_value(0);
        (0..6).any(|group| pac::PPI.chg(group).read().ch(5))
    };
    TIMER4::disable();
    TIMER4::unpend();
    // Neutralize inherited owned routes before changing timer/group/endpoints.
    pac::PPI.chenclr().write(|w| {
        w.set_ch(3, true);
        w.set_ch(4, true);
    });
    let _ = pac::PPI.chen().read();
    pac::PPI.fork(4).tep().write_value(0);
    pac::TIMER4
        .intenclr()
        .write_value(pac::timer::regs::Int(0xffff_ffff));
    // No steal, clone or task getter: borrow the real RADIO token only during
    // private endpoint validation, then let the caller move it into Esb.
    let txen_ptr = NonNull::new(pac::RADIO.tasks_txen().as_ptr()).expect("RADIO TXEN pointer");
    let disable_ptr =
        NonNull::new(pac::RADIO.tasks_disable().as_ptr()).expect("RADIO DISABLE pointer");
    assert_eq!(txen_ptr.as_ptr() as usize, 0x4000_1000);
    assert_eq!(disable_ptr.as_ptr() as usize, 0x4000_1010);
    // SAFETY: validated nRF52840 TASKS registers of the borrowed unique RADIO.
    let txen = unsafe { Task::new_unchecked(txen_ptr) };
    let disable = unsafe { Task::new_unchecked(disable_ptr) };
    let timer = Timer::new(timer);
    timer.set_frequency(Frequency::F1MHz);
    pac::TIMER4
        .shorts()
        .write_value(pac::timer::regs::Shorts(0));
    let mut group = PpiGroup::new(group0);
    let mut start = Ppi::new_one_to_one(ch4, timer.cc(1).event_compare(), txen);
    start.disable();
    group.add_channel(&start); // CH3 deliberately outside the group.
    group.disable_all();
    let mut cutoff = Ppi::new_one_to_two(
        ch3,
        timer.cc(0).event_compare(),
        disable,
        group.task_disable_all(),
    );
    cutoff.disable();
    #[cfg(feature = "sync-trace")]
    let capture_route = if inherited_capture_group {
        CaptureRoute::Unavailable { _lease: ch5 }
    } else {
        // Address construction ONLY: never read/clear RADIO EVENTS_END. The
        // real unique RADIO loan above is returned to the facade, never stolen.
        let end_ptr = NonNull::new(pac::RADIO.events_end().as_ptr()).expect("RADIO END pointer");
        assert_eq!(end_ptr.as_ptr() as usize, 0x4000_110c);
        // SAFETY: validated event endpoint of the borrowed unique RADIO; only
        // the owned CH5 EEP uses it. CC2 has no compare IRQ/shortcut/PPI route.
        let end = unsafe { Event::new_unchecked(end_ptr) };
        let mut route = Ppi::new_one_to_one(ch5, end, timer.cc(2).task_capture());
        route.disable();
        CaptureRoute::Ready(route) // Never added to GROUP0 or another group.
    };
    let mut hardware = Hardware {
        timer,
        cutoff,
        _start: start,
        group,
        disable,
        #[cfg(feature = "sync-trace")]
        capture_route,
        #[cfg(feature = "sync-trace")]
        latest_pair: None,
    };
    let mut state = State::new();
    state.disarm(&mut hardware);
    hardware.timer.start(); // Continuous 32-bit 1 MHz, no runtime CLR/STOP.
    critical_section::with(|cs| {
        let mut owner = OWNER.borrow(cs).borrow_mut();
        assert!(owner.is_none(), "deadline tokens consumed exactly once");
        *owner = Some(Owner {
            hardware,
            state,
            #[cfg(feature = "sync-trace")]
            capture: ExclusiveOwner::new(),
            #[cfg(feature = "sync-trace")]
            anchor_now: None,
            #[cfg(feature = "sync-trace")]
            diagnostic_failures: u32::from(inherited_capture_group),
        });
    });
    &PROVIDER
}

/// Called only after the exact static facade IRQ reference is published.
pub(crate) fn enable_irq() {
    use embassy_nrf::interrupt::Priority;
    TIMER4::set_priority(Priority::P2);
    TIMER4::unpend();
    // SAFETY: custom vector bound, static owner/facade installed, compares masked.
    unsafe {
        TIMER4::enable();
    }
}
