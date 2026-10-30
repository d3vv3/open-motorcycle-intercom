//! Exclusive NoAck role owner with cancellation-safe tickets and consuming wait.
//! Async wait uses deferred IRQ notification and a nominal software timer fallback,
//! not a hard RF cutoff. Callers supply an independent monotonic microsecond clock.

use core::cell::UnsafeCell;
use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::pin::pin;
use core::task::{Poll, Waker};

use cortex_m::peripheral::scb::VectActive;
use embassy_sync::waitqueue::AtomicWaker;

use crate::addresses::EsbAddresses;
use crate::config::{Channel, EsbConfig};
use crate::error::Error;
use crate::isr::{DriverParts, EsbPrx, EsbPtx, IrqMask, ReceivedPacket, unpend_irqs};
use crate::payload::PacketPool;
use crate::radio::StopOutcome;
use crate::rx_metadata::{ReceiveControl, RxEpoch, SharedReceiverContext};
use crate::state_machine::ExclusiveSendError;
use crate::timer::TimerInstance;

/// Caller-serviced send window in monotonic microseconds.
pub const SEND_DEADLINE_US: u64 = 1450;

/// Opaque generation. Values never wrap or get reused by an owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperationEpoch(u64);

impl OperationEpoch {
    /// Diagnostic identity; does not construct or authorize a ticket.
    pub const fn value(self) -> u64 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeadlineGuardError {
    Unavailable,
    Expired,
    HardwareFault,
}

/// How the prepared exclusive NoAck transmission is launched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxLaunch {
    Cpu,
    Scheduled,
}

/// Provider contract used to evaluate a normally completed current transmission.
/// Neither variant qualifies RF timing or diagnostic capture data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompletionPolicy {
    /// Service at or after the software deadline times out even with completion.
    StrictSoftwareDeadline,
    /// Retain current normal completion under the strengthened unsafe provider
    /// contract, unless matching expiry or a clock/reset/restoration fault wins.
    RetainGuardedCompletion,
}

/// Optional static deadline resource owner, not an arbitrary safe callback.
///
/// # Safety
/// Implementors must uniquely own their timer/PPI capabilities, bound to the
/// SAME unique RADIO consumed by this facade, initially disarmed at attachment.
/// Only the bound RADIO TXEN and DISABLE tasks may be written through owned
/// hardware PPI; raw task endpoints must remain private to the guard. The bound
/// RADIO EVENTS_END address may also be used solely as a PPI EEP to capture an
/// owned TIMER for diagnostics. This grants no event register clear, read/modify,
/// shared RADIO access, or competing owner. The diagnostic owner must always be
/// the same provider bound to that RADIO; no callback registration is permitted.
/// The monotonic microsecond domain must match begin's `now_us`.
/// State must be single-core ISR-safe using critical sections/atomics (`Sync`).
/// EVERY hook, including arm/disarm before activation, must NEVER synchronously
/// reenter Esb, drivers, Runtime, TDMA, borrowed actor state, or task polling.
/// Access only independent clock/resource state. No NMI/fault access is allowed.
/// Arm must reject unavailable, already-expired or faulty resources. Disarm
/// must be idempotent and synchronously disable the autonomous route before
/// returning (including after failed arm), before claim release/RX; it must preserve matching expiry
/// until the next arm; no hook other than arm may enable launch gating. Expired must
/// report only the requested epoch, and must
/// include latched compare expiry even if its IRQ has not yet run. On interrupt
/// may update expiry and pend RADIO/deferred work, never synchronously service
/// owner/actor state. IRQ setup belongs to the provider, not Esb activation.
/// Launch is called only after the DMA claim, TxNoAck state, register preparation
/// and release fence/barrier. Scheduled MUST arrange TXEN through owned hardware
/// PPI, never CPU fallback, and MUST NEVER re-enable gating after cutoff (in
/// particular, launch must not enable a group disabled by the deadline). An Err
/// must synchronously disable every pending launch route before returning;
/// the facade additionally disarms before stopping hardware and releasing claims.
/// A provider opting into RetainGuardedCompletion additionally guarantees that
/// each successful arm establishes an autonomous current-TX cutoff effective no
/// later than the requested monotonic deadline, maintained until completion
/// disarm. Every reached matching expiry must be observable and preserved,
/// including expiry racing completion disarm or restoration. This is an unsafe
/// provider obligation, not hardware qualification supplied by this facade.
pub unsafe trait TxDeadlineGuard: Sync {
    fn arm(&self, epoch: OperationEpoch, deadline_us: u64) -> Result<(), DeadlineGuardError>;
    /// Bounded, static capability declaration, queried once after successful arm
    /// and before driver launch. All hook no-reentry obligations apply. Must not
    /// mutate resources, expiry, launch gating or owner state.
    fn completion_policy(&self) -> CompletionPolicy {
        CompletionPolicy::StrictSoftwareDeadline
    }
    fn launch(&self, _epoch: OperationEpoch) -> Result<TxLaunch, DeadlineGuardError> {
        Ok(TxLaunch::Cpu)
    }
    /// Observe diagnostics only at the first existing current-TX END + DISABLED
    /// completion hook, after completion disarm and its existing expiry query,
    /// before stop and PRX restoration. Query only an already frozen copy: no
    /// hardware capture, clock reads or owner-state mutation.
    /// Must be bounded and ISR-safe, with the same stable owner and no reentry
    /// required by this provider's safety contract. Must not mutate diagnostic
    /// capture PPI, bound RADIO tasks/registers, launch or
    /// deadline routes, expiry state, or operation/deadline outcomes.
    /// A returned epoch mismatch is discarded as diagnostics only.
    fn capture_end(&self, _epoch: OperationEpoch) -> Option<TxEndCapture> {
        None
    }
    fn disarm(&self);
    fn expired(&self, epoch: OperationEpoch) -> bool;
    fn on_interrupt(&self);
}

fn guard_fault(error: DeadlineGuardError) -> TxFault {
    match error {
        DeadlineGuardError::Expired => TxFault::Deadline,
        DeadlineGuardError::Unavailable | DeadlineGuardError::HardwareFault => {
            TxFault::DeadlineGuardFault
        }
    }
}

/// Private witness: only a successful actual arm and explicit opt-in set it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct GuardArm(bool);

/// Shared production orchestration; the launch closure is never run on failure.
fn arm_before_launch<R>(
    provider: Option<&dyn TxDeadlineGuard>,
    epoch: OperationEpoch,
    deadline_us: u64,
    launch: impl FnOnce(GuardArm) -> R,
) -> Result<R, TxFault> {
    let armed = if let Some(provider) = provider {
        provider.arm(epoch, deadline_us).map_err(guard_fault)?;
        GuardArm(provider.completion_policy() == CompletionPolicy::RetainGuardedCompletion)
    } else {
        GuardArm(false)
    };
    Ok(launch(armed))
}

fn disarm_before_restore<R>(
    provider: Option<&dyn TxDeadlineGuard>,
    restore: impl FnOnce() -> R,
) -> R {
    if let Some(provider) = provider {
        provider.disarm();
    }
    restore()
}

fn restore_current<R>(
    current: bool,
    provider: Option<&dyn TxDeadlineGuard>,
    restore: impl FnOnce() -> R,
) -> Option<R> {
    current.then(|| disarm_before_restore(provider, restore))
}

fn guard_expired(provider: Option<&dyn TxDeadlineGuard>, epoch: OperationEpoch) -> bool {
    provider.is_some_and(|guard| guard.expired(epoch))
}

fn disarm_completion(provider: Option<&dyn TxDeadlineGuard>, epoch: OperationEpoch) -> bool {
    disarm_before_restore(provider, || guard_expired(provider, epoch))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxFault {
    /// This call was rejected without accessing state. Not a terminal TX fault.
    WrongContext,
    /// A receive control call cannot alter an in-flight send.
    Busy,
    /// IRQ activation must precede a receive start request.
    NotReady,
    TimedOut,
    /// Matching provider expiry, including END latched before late service.
    Deadline,
    DeadlineGuardFault,
    Cancelled,
    PowerCycled,
    ClockRegressed,
    ReceiverUnavailable,
    StaleEpoch,
    /// Receive-session counter exhausted; this owner cannot start RX again.
    RxEpochOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxOutcome {
    Pending,
    /// Current-TX END and DISABLED were observed, stop did not reset RADIO, and
    /// the PRX role was restored. RX may remain explicitly stopped.
    PhysicalComplete,
    Fault(TxFault),
}

/// Terminal arbitration after restoration and the existing matching expiry queries.
fn finalize_after_restore(
    proposed: TxOutcome,
    restoration_fault: Option<TxFault>,
    matching_expiry: bool,
) -> TxOutcome {
    if matching_expiry || proposed == TxOutcome::Fault(TxFault::Deadline) {
        TxOutcome::Fault(TxFault::Deadline)
    } else if let Some(fault) = restoration_fault {
        TxOutcome::Fault(fault)
    } else {
        proposed
    }
}

/// Where the existing current-TX END + DISABLED completion predicate was observed.
/// This is not a timestamp or proof of success before an RF deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxCompletionOrigin {
    RadioIrq,
    TaskService,
}

/// Numeric TIMER-domain diagnostic classification only. Even Candidate does not
/// qualify an RF/IRQ timestamp, latency, or a drained PPI capture pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxEndCaptureReason {
    Missing,
    Unstable,
    NoCompletionEvidence,
    OutsideEpochRange,
    ObservationOutsideHorizon,
    NotArmed,
    Candidate,
}

/// Provider-owned numeric TIMER-domain observation, with no absolute SharedClock
/// mapping. Raw values and the signed cutoff delta are diagnostic candidates,
/// not qualified RF/IRQ timestamps, latency, or PPI pipeline evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxEndCapture {
    pub epoch: OperationEpoch,
    pub anchor_raw: u32,
    pub cutoff_raw: u32,
    pub sampled_cc: [u32; 2],
    pub observation_raw: u32,
    pub elapsed_us: u64,
    pub horizon_us: u32,
    pub reason: TxEndCaptureReason,
    pub cutoff_delta_us: Option<i32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxObservations {
    /// First invocation of the driver's existing before-completion hook.
    pub completion: Option<TxCompletionOrigin>,
    /// At least one existing matching provider expiry query returned true.
    /// False includes an absent provider; it is not proof that TX was unexpired.
    pub provider_expiry_observed: bool,
    /// Matching provider diagnostics from the first completion hook only.
    pub end_capture: Option<TxEndCapture>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxReport {
    pub epoch: OperationEpoch,
    pub outcome: TxOutcome,
    /// Matching flight evidence, including pending flights. None means no matching
    /// flight evidence (unknown/stale epoch or context rejection).
    pub observations: Option<TxObservations>,
}

/// Pure context policy: the access closure cannot run in any exception class.
fn with_thread_context<R>(active: VectActive, access: impl FnOnce() -> R) -> Result<R, TxFault> {
    match active {
        VectActive::ThreadMode => Ok(access()),
        _ => Err(TxFault::WrongContext),
    }
}

/// Keep the architectural MMIO query separate from the host-testable policy.
fn task_access<R>(access: impl FnOnce() -> R) -> Result<R, TxFault> {
    with_thread_context(cortex_m::peripheral::SCB::vect_active(), access)
}

fn ticket_finished(outcome: TxOutcome) -> bool {
    !matches!(
        outcome,
        TxOutcome::Pending | TxOutcome::Fault(TxFault::WrongContext)
    )
}

/// Private adapter boundary: no endpoint method may retain an inner state loan.
trait WaitEndpoint {
    fn epoch(&self) -> OperationEpoch;
    fn check_context(&self) -> Result<(), TxFault>;
    fn register(&self, waker: &Waker);
    fn poll(&mut self, now_us: u64) -> TxReport;
}

fn wait_step(
    endpoint: &mut impl WaitEndpoint,
    now_us: &mut impl FnMut() -> u64,
    waker: &Waker,
) -> Option<TxReport> {
    let report = match endpoint.check_context() {
        Ok(()) => {
            // RawWaker clone/drop and the clock run outside any Inner/IrqMask loan.
            // Register first: an interrupt between registration and poll is not lost.
            endpoint.register(waker);
            let now = now_us();
            endpoint.poll(now)
        }
        Err(fault) => TxReport {
            epoch: endpoint.epoch(),
            outcome: TxOutcome::Fault(fault),
            observations: None,
        },
    };
    (report.outcome != TxOutcome::Pending).then_some(report)
}

/// Production wait protocol, also exercised by software-only endpoints/timers.
/// Owning the endpoint preserves the real ticket's cancellation Drop across await.
async fn wait_with<E: WaitEndpoint, F: Future<Output = ()>>(
    mut endpoint: E,
    mut now_us: impl FnMut() -> u64,
    mut fallback: impl FnMut() -> F,
) -> TxReport {
    if let Some(report) =
        poll_fn(|cx| Poll::Ready(wait_step(&mut endpoint, &mut now_us, cx.waker()))).await
    {
        return report;
    }
    loop {
        // Construct only after a gated Pending; never call the timer in IRQ context.
        let mut timer = pin!(fallback());
        let report = poll_fn(|cx| {
            if let Some(report) = wait_step(&mut endpoint, &mut now_us, cx.waker()) {
                return Poll::Ready(Some(report));
            }
            if timer.as_mut().poll(cx).is_ready() {
                // Resample the independent clock, then replace an exhausted timer
                // if still pending. Never sleep on a Ready timer or spin on it.
                return Poll::Ready(wait_step(&mut endpoint, &mut now_us, cx.waker()));
            }
            Poll::Pending
        })
        .await;
        if let Some(report) = report {
            return report;
        }
    }
}

struct Flight {
    epoch: OperationEpoch,
    deadline_us: u64,
    signal: Option<StopOutcome>,
    completion: Option<TxCompletionOrigin>,
    guard_expired: bool,
    guarded_completion_retention: bool,
    end_capture: Option<TxEndCapture>,
}

impl Flight {
    fn observations(&self) -> TxObservations {
        TxObservations {
            completion: self.completion,
            provider_expiry_observed: self.guard_expired,
            end_capture: self.end_capture,
        }
    }

    fn query_expiry(&mut self, provider: Option<&dyn TxDeadlineGuard>) -> bool {
        let expired = guard_expired(provider, self.epoch);
        self.guard_expired |= expired;
        expired
    }
}

/// The hook alone supplies completion evidence; a stop result does not.
fn observe_tx_event(
    flight: &mut Flight,
    provider: Option<&dyn TxDeadlineGuard>,
    origin: TxCompletionOrigin,
    event: impl FnOnce(&mut dyn FnMut()) -> Option<StopOutcome>,
) -> Option<StopOutcome> {
    let mut expired = guard_expired(provider, flight.epoch);
    let mut completed = false;
    let capture_first = flight.completion.is_none();
    let mut end_capture = None;
    let stop = event(&mut || {
        expired |= disarm_completion(provider, flight.epoch);
        if capture_first && !completed {
            end_capture = provider
                .and_then(|guard| guard.capture_end(flight.epoch))
                .filter(|capture| capture.epoch == flight.epoch);
        }
        completed = true;
    });
    flight.guard_expired |= expired;
    if completed {
        flight.completion.get_or_insert(origin);
        if capture_first {
            flight.end_capture = end_capture;
        }
    }
    stop
}

/// Pure policy, shared by production transitions and host-only tests.
struct Control {
    sequence: u64,
    clock_us: u64,
    active: Option<Flight>,
    last: Option<TxReport>,
}

impl Control {
    fn new() -> Self {
        Self {
            sequence: 0,
            clock_us: 0,
            active: None,
            last: None,
        }
    }

    fn can_begin(&self, now_us: u64) -> Result<(), Error> {
        if self.active.is_some() {
            return Err(Error::Busy);
        }
        if now_us < self.clock_us || now_us.checked_add(SEND_DEADLINE_US).is_none() {
            return Err(Error::InvalidParam);
        }
        if self.sequence == u64::MAX {
            return Err(Error::NotReady);
        }
        Ok(())
    }

    #[cfg(test)]
    fn begin(&mut self, now_us: u64) -> OperationEpoch {
        self.begin_until(now_us, now_us + SEND_DEADLINE_US)
    }

    fn can_begin_until(&self, now_us: u64, until_us: u64) -> Result<(), Error> {
        self.can_begin(now_us)?;
        if until_us <= now_us || until_us > now_us + SEND_DEADLINE_US {
            return Err(Error::InvalidParam);
        }
        Ok(())
    }

    fn begin_until(&mut self, now_us: u64, until_us: u64) -> OperationEpoch {
        // Caller checked under the same exclusive guard, before moving owners.
        assert!(self.can_begin_until(now_us, until_us).is_ok());
        self.sequence += 1;
        self.clock_us = now_us;
        let epoch = OperationEpoch(self.sequence);
        self.active = Some(Flight {
            epoch,
            deadline_us: until_us,
            signal: None,
            completion: None,
            guard_expired: false,
            guarded_completion_retention: false,
            end_capture: None,
        });
        epoch
    }

    fn matches(&self, epoch: OperationEpoch) -> bool {
        self.active
            .as_ref()
            .is_some_and(|flight| flight.epoch == epoch)
    }

    fn signal(&mut self, epoch: OperationEpoch, stop: StopOutcome) {
        if let Some(flight) = self.active.as_mut().filter(|flight| flight.epoch == epoch) {
            // A later normal event cannot erase a reset fault.
            if flight.signal != Some(StopOutcome::PowerCycled) {
                flight.signal = Some(stop);
            }
        }
    }

    fn evaluate(&mut self, now_us: u64) -> TxOutcome {
        let regressed = now_us < self.clock_us;
        self.clock_us = self.clock_us.max(now_us);
        let Some(flight) = &self.active else {
            return TxOutcome::Pending;
        };
        if flight.guard_expired {
            return TxOutcome::Fault(TxFault::Deadline);
        }
        if flight.signal == Some(StopOutcome::PowerCycled) {
            return TxOutcome::Fault(TxFault::PowerCycled);
        }
        if regressed {
            return TxOutcome::Fault(TxFault::ClockRegressed);
        }
        // Explicitly armed provider contract can retain current normal completion
        // across late service. Diagnostics and completion origin do not authorize it.
        if flight.signal == Some(StopOutcome::Normal)
            && flight.completion.is_some()
            && flight.guarded_completion_retention
        {
            return TxOutcome::PhysicalComplete;
        }
        // Strict default: late service remains a timeout.
        if now_us >= flight.deadline_us {
            return TxOutcome::Fault(TxFault::TimedOut);
        }
        if flight.signal == Some(StopOutcome::Normal) {
            TxOutcome::PhysicalComplete
        } else {
            TxOutcome::Pending
        }
    }

    fn finish(&mut self, outcome: TxOutcome) -> TxReport {
        assert!(outcome != TxOutcome::Pending);
        let flight = self.active.take().expect("finish requires an operation");
        let report = TxReport {
            epoch: flight.epoch,
            outcome,
            observations: Some(flight.observations()),
        };
        self.last = Some(report);
        report
    }

    fn outcome(&self, epoch: OperationEpoch) -> TxOutcome {
        self.report(epoch).outcome
    }

    fn report(&self, epoch: OperationEpoch) -> TxReport {
        if let Some(flight) = self.active.as_ref().filter(|flight| flight.epoch == epoch) {
            return TxReport {
                epoch,
                outcome: TxOutcome::Pending,
                observations: Some(flight.observations()),
            };
        }
        match self.last {
            Some(report) if report.epoch == epoch => report,
            _ => TxReport {
                epoch,
                outcome: TxOutcome::Fault(TxFault::StaleEpoch),
                observations: None,
            },
        }
    }
}

enum Role<T: TimerInstance, const N: usize, const SIZE: usize> {
    Prx(EsbPrx<T, N, SIZE>),
    Ptx(EsbPtx<T, N, SIZE>),
}

struct Inner<T: TimerInstance, const N: usize, const SIZE: usize> {
    // None exists only during a masked, synchronous, consuming conversion.
    role: Option<Role<T, N, SIZE>>,
    config: EsbConfig,
    addresses: EsbAddresses,
    next_pid: u8,
    activated: bool,
    receive_control: ReceiveControl,
    control: Control,
    last_fault: Option<TxFault>,
    receiver: SharedReceiverContext,
    deadline: Option<&'static dyn TxDeadlineGuard>,
}

impl<T: TimerInstance, const N: usize, const SIZE: usize> Inner<T, N, SIZE> {
    fn recv_start(&mut self) -> Result<RxEpoch, TxFault> {
        let role = &mut self.role;
        let deadline = self.deadline;
        let result = self.receive_control.start(
            &mut self.receiver,
            self.config.channel,
            self.activated,
            self.control.active.is_some(),
            |context| {
                let Some(Role::Prx(driver)) = role.as_mut() else {
                    return Err(TxFault::NotReady);
                };
                if driver.take_stop_fault() {
                    return Err(TxFault::PowerCycled);
                }
                if driver.receiver_idle() {
                    disarm_before_restore(deadline, || {
                        driver.set_rx_metadata_context(context);
                        driver
                            .start_exclusive_rx()
                            .map_err(|_| TxFault::ReceiverUnavailable)
                    })?;
                }
                Ok(())
            },
        );
        let result = if result == Err(TxFault::PowerCycled) {
            // A faulted ISR receiver needs register reconstruction, not merely
            // an idle starvation retry. Preserve the fault even if rearm succeeds.
            Err(self.restore_prx(false).unwrap_or(TxFault::PowerCycled))
        } else {
            result
        };
        if let Err(fault) = result
            && !matches!(fault, TxFault::Busy | TxFault::NotReady)
        {
            self.last_fault = Some(fault);
        }
        result
    }

    fn recv_stop(&mut self) -> Result<(), TxFault> {
        let role = &mut self.role;
        let deadline = self.deadline;
        let result = self
            .receive_control
            .stop(self.control.active.is_some(), || {
                let stop = disarm_before_restore(deadline, || {
                    match role.as_mut().expect("owner must be published") {
                        Role::Prx(driver) => driver.quiesce(),
                        Role::Ptx(driver) => driver.quiesce(),
                    }
                });
                if stop == StopOutcome::PowerCycled {
                    Err(TxFault::PowerCycled)
                } else {
                    Ok(())
                }
            });
        if result == Err(TxFault::PowerCycled) {
            // Stop reset the RADIO configuration. Reconstruct a quiescent PRX
            // while desire is false so a later explicit start is safe.
            let _ = self.restore_prx(false);
        }
        if let Err(fault) = result
            && fault != TxFault::Busy
        {
            self.last_fault = Some(fault);
        }
        result
    }

    fn quiesce(&mut self) -> StopOutcome {
        match self.role.as_mut().expect("owner must be published") {
            Role::Prx(driver) => driver.quiesce(),
            Role::Ptx(driver) => driver.quiesce(),
        }
    }

    /// Only called after quiesce with RADIO/TIMER + PRIMASK masked. These
    /// consuming helpers enforce Idle/no DMA indices and move the real owners;
    /// the temporary empty role is never visible to an IRQ or an await point.
    fn take_parts(&mut self) -> DriverParts<T, N, SIZE> {
        match self.role.take().expect("owner must be published") {
            Role::Prx(driver) => driver.into_parts(),
            Role::Ptx(driver) => {
                let (parts, next_pid) = driver.into_parts();
                self.next_pid = next_pid;
                parts
            }
        }
    }

    fn restore_prx(&mut self, reset_duplicates: bool) -> Option<TxFault> {
        disarm_before_restore(self.deadline, || {
            self.restore_prx_disarmed(reset_duplicates)
        })
    }

    fn restore_prx_disarmed(&mut self, reset_duplicates: bool) -> Option<TxFault> {
        let stop = self.quiesce();
        // Dormant config spends no epoch; active RX exhaustion fails before
        // consuming/reconstructing owners or starting another receiver.
        let context = match self.receive_control.prepare_session(
            &mut self.receiver,
            self.config.channel,
            self.activated,
        ) {
            Ok(context) => context,
            Err(fault) => {
                self.last_fault = Some(fault);
                return Some(fault);
            }
        };
        let mut parts = self.take_parts();
        parts.prepare(&self.config, &self.addresses, reset_duplicates);
        self.role = Some(Role::Prx(EsbPrx::from_parts(
            parts,
            &self.config,
            &self.addresses,
        )));
        let Role::Prx(driver) = self.role.as_mut().expect("PRX was published") else {
            unreachable!()
        };
        driver.set_rx_metadata_context(context);
        let started = if self.receive_control.wanted(self.activated) {
            driver.start_exclusive_rx()
        } else {
            Ok(())
        };
        if stop == StopOutcome::PowerCycled {
            Some(TxFault::PowerCycled)
        } else if started.is_err() {
            Some(TxFault::ReceiverUnavailable)
        } else {
            None
        }
    }

    fn poll_tx_event(&mut self, origin: TxCompletionOrigin) {
        if let (Some(Role::Ptx(driver)), Some(flight)) =
            (self.role.as_mut(), self.control.active.as_mut())
        {
            let epoch = flight.epoch;
            let stop = observe_tx_event(flight, self.deadline, origin, |before_completion| {
                driver.exclusive_radio_event_with_completion(before_completion)
            });
            if let Some(stop) = stop {
                self.control.signal(epoch, stop);
            }
        }
    }

    fn service(&mut self, now_us: u64) -> Option<TxReport> {
        if !self.activated || self.receiver.overflowed() {
            return None;
        }
        if self.control.active.is_none() {
            self.control.evaluate(now_us);
            if let Some(Role::Prx(driver)) = self.role.as_mut() {
                if driver.take_stop_fault() {
                    self.last_fault = Some(self.restore_prx(false).unwrap_or(TxFault::PowerCycled));
                } else if self.receive_control.wanted(self.activated) && driver.receiver_idle() {
                    let _ = self.recv_start();
                }
            }
            return None;
        }
        self.poll_tx_event(TxCompletionOrigin::TaskService);
        let mut outcome = self.control.evaluate(now_us);
        if outcome == TxOutcome::Pending {
            return None;
        }
        let restored = self.restore_prx(false);
        // Disarm preserves a compare racing the earlier event sample. Check it
        // before publishing even a timeout/clock/reset restoration result.
        let flight = self.control.active.as_mut().expect("active TX");
        let expired = flight.query_expiry(self.deadline);
        outcome = finalize_after_restore(outcome, restored, expired || flight.guard_expired);
        if let TxOutcome::Fault(fault) = outcome {
            self.last_fault = Some(fault);
        }
        Some(self.control.finish(outcome))
    }
}

/// A single static RADIO/TIMER owner, initially PRX. Inner drivers and their
/// addresses are never exposed; IRQs bind to this stable outer address only.
/// Safe state/control and receive calls reject every exception context, including
/// NMI, before masking IRQs or borrowing state/pool, even before activation.
pub struct Esb<T: TimerInstance, const N: usize = 4, const SIZE: usize = 256> {
    inner: UnsafeCell<Inner<T, N, SIZE>>,
    pool: &'static PacketPool<N, SIZE>,
    tx_notifier: AtomicWaker,
}

// SAFETY: on the single Cortex-M core, every safe state/pool access first rejects
// all exception contexts, even before unsafe activation. Thread-mode inner access
// then holds IrqMask (including PRIMASK). An NMI cannot reborrow through safe APIs
// while that guard is held. Unsafe IRQ access requires the activation/dispatch
// contract (bound, non-NMI handlers, no nesting); no inner borrow escapes.
// Deadline providers are Sync and their unsafe implementor contract forbids
// synchronous hook reentry even in ThreadMode and before activation.
// TX Waker clone/drop/wake always run outside Inner/IrqMask loans. Activation
// additionally requires deferred polling on this same core's ThreadMode executor.
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Send for Esb<T, N, SIZE> {}
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Sync for Esb<T, N, SIZE> {}

impl<T: TimerInstance, const N: usize, const SIZE: usize> Esb<T, N, SIZE> {
    /// Checks thread mode, then validates the NoAck config and pool before
    /// RADIO/TIMER MMIO. Leaves IRQs masked; receiving begins only after static
    /// placement and unsafe activation. Exception context returns Busy.
    pub fn new(
        timer: embassy_nrf::Peri<'static, T>,
        radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
    ) -> Result<Self, Error> {
        Self::new_inner(timer, radio, pool, config, addresses, None, None)
    }

    /// Like new, with an actor-independent static shared-clock accessor.
    /// The hook is called only in the bound RADIO ISR for valid new NoAck RX,
    /// after normal quiescence and before publication, never by task methods.
    /// None or a negative time means unavailable, not a fabricated zero.
    /// This hook does not timestamp RADIO END or service TX deadlines.
    pub fn new_with_rx_clock(
        timer: embassy_nrf::Peri<'static, T>,
        radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        clock: fn() -> Option<i64>,
    ) -> Result<Self, Error> {
        Self::new_inner(timer, radio, pool, config, addresses, Some(clock), None)
    }

    /// Like new_with_rx_clock, with a static unsafe-implemented deadline owner.
    /// No TIMER4/PPI driver is supplied here. The existing RADIO/TIMER tokens
    /// are consumed unchanged; no additional peripheral token is fabricated.
    pub fn new_with_rx_clock_and_deadline(
        timer: embassy_nrf::Peri<'static, T>,
        radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        clock: fn() -> Option<i64>,
        deadline: &'static dyn TxDeadlineGuard,
    ) -> Result<Self, Error> {
        Self::new_inner(
            timer,
            radio,
            pool,
            config,
            addresses,
            Some(clock),
            Some(deadline),
        )
    }

    fn new_inner(
        timer: embassy_nrf::Peri<'static, T>,
        radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        clock: Option<fn() -> Option<i64>>,
        deadline: Option<&'static dyn TxDeadlineGuard>,
    ) -> Result<Self, Error> {
        task_access(|| {
            let mut driver = EsbPrx::new(timer, radio, pool, config, addresses)?;
            let _guard = IrqMask::<T>::new();
            if let Some(deadline) = deadline {
                deadline.disarm();
            }
            if driver.quiesce() == StopOutcome::PowerCycled {
                return Err(Error::RadioFault);
            }
            Ok(Self {
                inner: UnsafeCell::new(Inner {
                    role: Some(Role::Prx(driver)),
                    config: config.clone(),
                    addresses: addresses.clone(),
                    next_pid: 0,
                    activated: false,
                    receive_control: ReceiveControl::new(),
                    control: Control::new(),
                    last_fault: None,
                    receiver: SharedReceiverContext::new(clock),
                    deadline,
                }),
                pool,
                tx_notifier: AtomicWaker::new(),
            })
        })
        .unwrap_or(Err(Error::Busy))
    }

    /// Bind only this outer facade, never references to its movable drivers.
    ///
    /// # Safety
    /// On a single Cortex-M core, both vectors must dispatch exclusively to this
    /// exact static owner. Configure clocks and equal preemption priorities so
    /// RADIO/TIMER never nest/reenter. No raw peripheral/state borrows or other
    /// owners may coexist. Waker clone/drop/wake must NEVER synchronously poll
    /// tasks or reenter facade/driver/actor state, and must schedule only this
    /// same core's ThreadMode executor. NMI/fault handlers must not access it.
    /// Activation itself must run in thread mode. Safe state/control methods
    /// enforce thread mode independently of this contract, including before setup.
    /// Complete all setup before activation and service send deadlines thereafter.
    /// An installed RX clock must be single-core/ISR-safe and must not reenter
    /// the facade, any driver, Runtime or TDMA actor, or poll tasks synchronously.
    /// It may access only independent clock state, never aliased owner state.
    /// An installed deadline IRQ must also never nest RADIO/protocol TIMER;
    /// its provider controls setup/enabling. Activation never unmasks TIMER4.
    pub unsafe fn activate(&'static self) -> Result<(), Error> {
        unsafe { self.activate_with_receive(true) }
    }

    /// Publish/unmask the same IRQ owner without starting RX or spending an epoch.
    /// Call `recv_start` in ThreadMode to request the first receive session.
    ///
    /// # Safety
    /// All static placement, single-core, non-nesting RADIO/protocol TIMER/deadline
    /// vector, independent-clock and deferred-waker obligations of `activate` apply.
    pub unsafe fn activate_stopped(&'static self) -> Result<(), Error> {
        unsafe { self.activate_with_receive(false) }
    }

    unsafe fn activate_with_receive(&'static self, request_rx: bool) -> Result<(), Error> {
        let mut guard = IrqMask::<T>::new();
        let inner = unsafe { &mut *self.inner.get() };
        if inner.activated {
            return Err(Error::Busy);
        }
        unpend_irqs::<T>();
        if let Some(deadline) = inner.deadline {
            deadline.disarm();
        }
        if inner.receiver.overflowed() {
            return Err(Error::NotReady);
        }
        if !matches!(&inner.role, Some(Role::Prx(_))) {
            return Err(Error::NotReady);
        }
        inner.activated = true;
        let result = if request_rx {
            inner.recv_start().map(|_| ()).map_err(|fault| match fault {
                TxFault::RxEpochOverflow | TxFault::NotReady => Error::NotReady,
                TxFault::PowerCycled => Error::RadioFault,
                _ => Error::OutOfMemory,
            })
        } else {
            Ok(())
        };
        if inner.receiver.overflowed() {
            inner.activated = false;
        } else {
            guard.enable_on_drop();
        }
        result
    }

    /// Request PRX in ThreadMode. Healthy already-requested PRX is idempotent;
    /// an idle retry keeps its epoch. Active TX returns Busy without touching it.
    /// ReceiverUnavailable retains desire for service recovery; an adapter whose
    /// START failed must call `recv_stop` to withdraw that desire. Overflow is
    /// sticky and fail-closed, requiring a new owner. No clock callback is invoked.
    pub fn recv_start(&self) -> Result<RxEpoch, TxFault> {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            unsafe { &mut *self.inner.get() }.recv_start()
        })?
    }

    /// Withdraw PRX desire before disarm-first quiescence. Does not cancel TX:
    /// Busy leaves all state unchanged. Queued/held packets remain available.
    /// PowerCycled is sticky but desire remains false; service cannot restart RX.
    /// Safe in ThreadMode even before activation; repeated stops are idempotent.
    pub fn recv_stop(&self) -> Result<(), TxFault> {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            unsafe { &mut *self.inner.get() }.recv_stop()
        })?
    }

    /// ThreadMode diagnostic: None when stopped, otherwise the last requested
    /// PRX epoch, including during PTX. Valid even with legacy metadata disabled;
    /// it is a session capability, not proof of a timestamp or healthy RX.
    /// Backends must track a minimum accepted epoch after explicit stop/channel
    /// changes, not compare every queued packet to this value: ordinary TX restore
    /// advances it without invalidating older queued/held records.
    pub fn receiver_epoch(&self) -> Result<Option<RxEpoch>, TxFault> {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            let inner = unsafe { &*self.inner.get() };
            inner.receive_control.epoch(&inner.receiver)
        })
    }

    /// Task-only coherent pool counters; retained across PRX/PTX reconstruction.
    #[cfg(feature = "rx-blocking-trace")]
    pub fn rx_diagnostics(&self) -> Result<crate::payload::RxDiagnostics, TxFault> {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            self.pool.rx_diagnostics()
        })
    }

    /// Starts hardware TX, not merely an enqueue. The default absolute deadline
    /// is `now_us + 1450`, using a caller-supplied nonwrapping monotonic clock.
    /// Allowed after IRQ activation even when RX is explicitly stopped. Completion
    /// restores the PRX role, but starts reception only while it is requested.
    /// Call service/poll before that deadline; Drop synchronously cancels.
    /// Exception context returns Busy before touching state or hardware.
    pub fn begin_send_no_ack_to(
        &self,
        pipe: u8,
        payload: &[u8],
        now_us: u64,
    ) -> Result<SendTicket<'_, T, N, SIZE>, Error> {
        let until_us = now_us.saturating_add(SEND_DEADLINE_US);
        self.begin_send_no_ack_to_until(pipe, payload, now_us, until_us)
    }

    /// Absolute caller cutoff, no later than the default 1450 us deadline.
    /// Invalid bounds reject before IRQ masking, claims, counters or MMIO.
    pub fn begin_send_no_ack_to_until(
        &self,
        pipe: u8,
        payload: &[u8],
        now_us: u64,
        until_us: u64,
    ) -> Result<SendTicket<'_, T, N, SIZE>, Error> {
        task_access(|| {
            let maximum = now_us
                .checked_add(SEND_DEADLINE_US)
                .ok_or(Error::InvalidParam)?;
            if until_us <= now_us || until_us > maximum {
                return Err(Error::InvalidParam);
            }
            let _guard = IrqMask::<T>::new();
            let inner = unsafe { &mut *self.inner.get() };
            if !inner.activated || inner.receiver.overflowed() {
                return Err(Error::NotReady);
            }
            if pipe >= 8
                || payload.is_empty()
                || payload.len() > usize::from(inner.config.payload_length)
            {
                return Err(Error::InvalidParam);
            }
            inner.control.can_begin_until(now_us, until_us)?;
            if disarm_before_restore(inner.deadline, || inner.quiesce()) == StopOutcome::PowerCycled
            {
                inner.last_fault = Some(inner.restore_prx(false).unwrap_or(TxFault::PowerCycled));
                return Err(Error::RadioFault);
            }
            let mut parts = inner.take_parts();
            parts.prepare(&inner.config, &inner.addresses, false);
            inner.role = Some(Role::Ptx(EsbPtx::from_parts(
                parts,
                &inner.config,
                inner.next_pid,
            )));
            let epoch = inner.control.begin_until(now_us, until_us);
            let deadline_us = inner
                .control
                .active
                .as_ref()
                .expect("prepared TX")
                .deadline_us;
            let provider = inner.deadline;
            let Some(Role::Ptx(driver)) = inner.role.as_mut() else {
                unreachable!()
            };
            let flight = inner.control.active.as_mut().expect("prepared TX");
            let launched = arm_before_launch(inner.deadline, epoch, deadline_us, |armed| {
                assert_eq!(flight.epoch, epoch);
                flight.guarded_completion_retention = armed.0;
                driver.begin_exclusive_no_ack_with_launch(pipe, payload, || {
                    provider.map_or(Ok(TxLaunch::Cpu), |provider| provider.launch(epoch))
                })
            });
            let launched = match launched {
                Ok(result) => result,
                Err(fault) => {
                    let _ = inner.restore_prx(false);
                    inner.control.finish(TxOutcome::Fault(fault));
                    inner.last_fault = Some(fault);
                    return Err(Error::RadioFault);
                }
            };
            if let Err(error) = launched {
                if let ExclusiveSendError::Guard(error) = error {
                    let fault = guard_fault(error);
                    let _ = inner.restore_prx(false);
                    inner.control.finish(TxOutcome::Fault(fault));
                    inner.last_fault = Some(fault);
                    return Err(Error::RadioFault);
                }
                let ExclusiveSendError::Driver(error) = error else {
                    unreachable!()
                };
                let fault = inner
                    .restore_prx(false)
                    .unwrap_or(TxFault::ReceiverUnavailable);
                inner.control.finish(TxOutcome::Fault(fault));
                inner.last_fault = Some(fault);
                return Err(if fault == TxFault::PowerCycled {
                    Error::RadioFault
                } else {
                    error
                });
            }
            Ok(SendTicket {
                esb: self,
                epoch,
                finished: false,
                _thread_owner: PhantomData,
            })
        })
        .unwrap_or(Err(Error::Busy))
    }

    /// MUST be driven by the caller's timer/task even if a ticket is forgotten.
    /// No hardware deadline timer is installed by this facade. Late service times
    /// out by default; an explicitly opted-in, successfully armed provider can
    /// retain current normal completion, subject to expiry and restoration faults.
    /// Retention does not qualify an RF timestamp or hardware cutoff.
    /// Exception-context calls return None without servicing or accessing state.
    pub fn service(&self, now_us: u64) -> Option<TxReport> {
        let report = task_access(|| {
            let _guard = IrqMask::<T>::new();
            unsafe { &mut *self.inner.get() }.service(now_us)
        })
        .unwrap_or(None);
        if report.is_some() {
            self.tx_notifier.wake();
        }
        report
    }

    fn poll_epoch(&self, epoch: OperationEpoch, now_us: u64) -> TxReport {
        let (report, published) = task_access(|| {
            let _guard = IrqMask::<T>::new();
            let inner = unsafe { &mut *self.inner.get() };
            // Keep the generation check and service in one critical section. A
            // retained stale ticket must not service/timeout a newer operation.
            let published = inner.control.matches(epoch) && inner.service(now_us).is_some();
            (inner.control.report(epoch), published)
        })
        .unwrap_or_else(|fault| {
            (
                TxReport {
                    epoch,
                    outcome: TxOutcome::Fault(fault),
                    observations: None,
                },
                false,
            )
        });
        if published {
            self.tx_notifier.wake();
        }
        report
    }

    /// A completion is published only by task service after PRX restoration.
    /// Only the latest terminal report is retained; older epochs become stale.
    /// WrongContext rejects this call only; it does not terminate the operation.
    pub fn check_tx_outcome(&self, epoch: OperationEpoch) -> TxOutcome {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            unsafe { &*self.inner.get() }.control.outcome(epoch)
        })
        .unwrap_or_else(TxOutcome::Fault)
    }

    /// Stops before freeing DMA claims; stale epochs cannot cancel a new send.
    /// WrongContext leaves the operation pending and must be retried in task context.
    pub fn cancel(&self, epoch: OperationEpoch) -> TxOutcome {
        let outcome = task_access(|| {
            let _guard = IrqMask::<T>::new();
            let inner = unsafe { &mut *self.inner.get() };
            let current = inner.control.matches(epoch);
            let expired = current
                && (inner
                    .control
                    .active
                    .as_mut()
                    .expect("active TX")
                    .query_expiry(inner.deadline)
                    || inner
                        .control
                        .active
                        .as_ref()
                        .is_some_and(|flight| flight.guard_expired));
            let Some(restored) = restore_current(current, inner.deadline, || {
                inner.restore_prx_disarmed(false)
            }) else {
                return inner.control.outcome(epoch);
            };
            let fault = if expired
                || inner
                    .control
                    .active
                    .as_mut()
                    .expect("active TX")
                    .query_expiry(inner.deadline)
            {
                TxFault::Deadline
            } else {
                restored.unwrap_or(TxFault::Cancelled)
            };
            inner.last_fault = Some(fault);
            inner.control.finish(TxOutcome::Fault(fault)).outcome
        })
        .unwrap_or_else(TxOutcome::Fault);
        if ticket_finished(outcome) {
            self.tx_notifier.wake();
        }
        outcome
    }

    /// Changes channel/addresses only when no send is in flight. Received packet
    /// handles and queued RX data remain valid. Only a channel change resets the
    /// RX duplicate history; ordinary role switches and address updates do not.
    /// A valid requested configuration remains applied on a reset/rearm error;
    /// service retries the receiver when packet buffers become available, only
    /// while reception is requested. Dormant/stopped configuration spends no epoch.
    /// Exception context returns Busy, including before activation.
    pub fn configure_idle(&self, channel: Channel, addresses: &EsbAddresses) -> Result<(), Error> {
        task_access(|| {
            if channel.0 > Channel::MAX {
                return Err(Error::InvalidParam);
            }
            let _guard = IrqMask::<T>::new();
            let inner = unsafe { &mut *self.inner.get() };
            if inner.control.active.is_some() {
                return Err(Error::Busy);
            }
            if inner.receiver.overflowed() {
                return Err(Error::NotReady);
            }
            let stop = disarm_before_restore(inner.deadline, || inner.quiesce());
            let reset_duplicates = channel != inner.config.channel;
            inner.config.channel = channel;
            inner.addresses = addresses.clone();
            let restored = inner.restore_prx(reset_duplicates);
            let fault = if restored == Some(TxFault::RxEpochOverflow) {
                restored
            } else if stop == StopOutcome::PowerCycled {
                Some(TxFault::PowerCycled)
            } else {
                restored
            };
            match fault {
                Some(fault) => {
                    inner.last_fault = Some(fault);
                    Err(if fault == TxFault::RxEpochOverflow {
                        Error::NotReady
                    } else if fault == TxFault::PowerCycled {
                        Error::RadioFault
                    } else {
                        Error::OutOfMemory
                    })
                }
                None => Ok(()),
            }
        })
        .unwrap_or(Err(Error::Busy))
    }

    /// Sticky diagnostic, including receiver faults that have no send epoch.
    /// In exception context returns Some(WrongContext) without reading/changing it.
    pub fn last_fault(&self) -> Option<TxFault> {
        task_access(|| {
            let _guard = IrqMask::<T>::new();
            unsafe { &*self.inner.get() }.last_fault
        })
        .unwrap_or(Some(TxFault::WrongContext))
    }

    /// Task-only dequeue. Exception context returns Busy without accessing the pool.
    pub fn try_receive(&self) -> Result<Option<ReceivedPacket<'_, N, SIZE>>, Error> {
        task_access(|| {
            self.pool
                .try_receive_rx()
                .map(|idx| ReceivedPacket::from_pool(self.pool, idx))
        })
        .map_err(|_| Error::Busy)
    }

    /// Task-only receive; every poll rejects exception context with Busy before
    /// polling the pool's channel. Recreate the future in task context on rejection.
    pub async fn receive(&self) -> Result<ReceivedPacket<'_, N, SIZE>, Error> {
        let receive = task_access(|| self.pool.receive_rx()).map_err(|_| Error::Busy)?;
        let mut receive = pin!(receive);
        poll_fn(|cx| {
            task_access(|| {
                receive
                    .as_mut()
                    .poll(cx)
                    .map(|idx| Ok(ReceivedPacket::from_pool(self.pool, idx)))
            })
            .unwrap_or(Poll::Ready(Err(Error::Busy)))
        })
        .await
    }

    /// # Safety
    /// Only the bound RADIO IRQ may call this after activation. RADIO/TIMER
    /// must not nest or overlap any facade/driver/peripheral state borrow.
    /// Never call from NMI/fault handlers. This intentionally has no task gate.
    /// The RX clock hook must obey the single-core, independent-clock and no
    /// facade/Runtime/TDMA reentry contract of activate. Wakers must defer polling.
    pub unsafe fn on_radio_interrupt(&self) {
        {
            let inner = unsafe { &mut *self.inner.get() };
            if inner.receiver.overflowed() {
                return;
            }
            match inner
                .role
                .as_mut()
                .expect("role moves only with IRQs masked")
            {
                Role::Prx(driver) if inner.receive_control.wanted(inner.activated) => {
                    driver.exclusive_radio_event()
                }
                Role::Prx(_) => {}
                Role::Ptx(_) => inner.poll_tx_event(TxCompletionOrigin::RadioIrq),
            }
            // TX stays disabled/Idle until task service consumes the role. RX stays
            // in the same role; no conversion or epoch rollover here.
            cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::RADIO);
        }
        // Driver completion/disarm/claim cleanup precedes deferred notification.
        self.tx_notifier.wake();
    }

    /// # Safety
    /// Only the bound TIMER IRQ may call this after activation, without nesting,
    /// reentrancy or overlapping RADIO/TIMER/facade state borrows.
    /// Never call from NMI/fault handlers. This intentionally has no task gate.
    pub unsafe fn on_timer_interrupt(&self) {
        {
            let inner = unsafe { &mut *self.inner.get() };
            match inner
                .role
                .as_mut()
                .expect("role moves only with IRQs masked")
            {
                Role::Prx(driver) if inner.receive_control.wanted(inner.activated) => {
                    driver.exclusive_timer_event()
                }
                Role::Prx(_) => {}
                Role::Ptx(driver) => driver.exclusive_timer_event(),
            }
        }
        self.tx_notifier.wake();
    }

    /// Dispatch to the provider if present, then notify the waiter outside state
    /// loans. Task service still publishes TX reports. No provider means only a
    /// spurious notification. This hook alone does not enforce a hard deadline:
    /// a concrete TIMER4/PPI-to-RADIO_DISABLE guard is not implemented here.
    ///
    /// # Safety
    /// Bind the deadline vector to this exact static outer owner on one core.
    /// RADIO, protocol TIMER (e.g. TIMER2) and deadline TIMER (e.g. TIMER4) must
    /// have equal preemption priorities and never nest/reenter or overlap ANY
    /// facade/driver/Runtime/TDMA borrow. No NMI/fault calls. Complete provider
    /// setup before enabling its IRQ; activate does not unmask the deadline IRQ.
    /// Provider notification must pend RADIO or deferred work, not poll tasks.
    pub unsafe fn on_deadline_interrupt(&'static self) {
        {
            let inner = unsafe { &*self.inner.get() };
            if let Some(provider) = inner.deadline {
                provider.on_interrupt();
            }
        }
        self.tx_notifier.wake();
    }
}

/// Send operation borrowing only the outer owner. Dropping a pending ticket in
/// ThreadMode quiesces TX and restores PRX, including when owned by `wait`.
/// Neither Send nor Sync: keep the ticket in its creating task, not IRQ handlers.
///
/// ```compile_fail
/// use embassy_nrf_esb::{SendTicket, timer::TimerInstance};
/// fn require_send<T: TimerInstance>() {
///     fn assert_send<S: Send>() {}
///     assert_send::<SendTicket<'static, T, 4, 256>>();
/// }
/// ```
///
/// ```compile_fail
/// use embassy_nrf_esb::{SendTicket, timer::TimerInstance};
/// fn require_sync<T: TimerInstance>() {
///     fn assert_sync<S: Sync>() {}
///     assert_sync::<SendTicket<'static, T, 4, 256>>();
/// }
/// ```
#[must_use = "poll/service the send deadline or drop the ticket to cancel"]
pub struct SendTicket<'a, T: TimerInstance, const N: usize, const SIZE: usize> {
    esb: &'a Esb<T, N, SIZE>,
    epoch: OperationEpoch,
    finished: bool,
    _thread_owner: PhantomData<*mut ()>,
}

impl<T: TimerInstance, const N: usize, const SIZE: usize> SendTicket<'_, T, N, SIZE> {
    pub fn epoch(&self) -> OperationEpoch {
        self.epoch
    }

    /// WrongContext does not mark this ticket finished; later task polling or
    /// task-context Drop must still service/cancel its pending TX.
    pub fn poll(&mut self, now_us: u64) -> TxOutcome {
        let outcome = self.esb.poll_epoch(self.epoch, now_us).outcome;
        self.finished |= ticket_finished(outcome);
        outcome
    }

    /// Consume this ticket and await its physical terminal report after task
    /// service restores PRX (or publishes a restoration fault), never enqueue or
    /// launch success. The future owns the ticket, not the caller's payload, which
    /// begin already copied into the owned pool. Pending Drop uses the real cancel.
    ///
    /// Poll and drop only in the creating core's ThreadMode executor. WrongContext
    /// returns a diagnostic without registration or clock calls; it does not finish
    /// TX, and the ensuing Drop's cancellation is also rejected in IRQ context.
    /// ThreadMode service/cancel is then still owed; the pool keeps the DMA claim.
    ///
    /// `now_us` must be an infallible, independent monotonic microsecond accessor
    /// in begin's clock domain (e.g. SharedClock TIMER0), not the Embassy RTC clock.
    /// It runs outside Inner loans/masks, must not dismantle/change operation epochs
    /// or reenter facade/actor state. Use a fail-stop wrapper for a fallible clock.
    /// An Embassy time driver is required. Nominal 250 us fallback ticks service the
    /// 1450 us deadline without IRQs; RTC quantization/executor latency are not a
    /// hard deadline guarantee. Notifications may be spurious; poll checks epochs.
    ///
    /// ```compile_fail
    /// use embassy_nrf_esb::{SendTicket, timer::TimerInstance};
    /// fn require_send<T: TimerInstance>(ticket: SendTicket<'_, T, 4, 256>) {
    ///     fn assert_send<S: Send>(_: S) {}
    ///     assert_send(ticket.wait(|| 0));
    /// }
    /// ```
    ///
    /// ```compile_fail
    /// use embassy_nrf_esb::{SendTicket, timer::TimerInstance};
    /// fn require_sync<T: TimerInstance>(ticket: SendTicket<'_, T, 4, 256>) {
    ///     fn assert_sync<S: Sync>(_: S) {}
    ///     assert_sync(ticket.wait(|| 0));
    /// }
    /// ```
    pub async fn wait(self, now_us: fn() -> u64) -> TxReport {
        wait_with(self, now_us, || embassy_time::Timer::after_micros(250)).await
    }

    /// Like `wait`, with a capturing clock callback. All of `wait`'s clock,
    /// context and cancellation restrictions apply. The callback may retain an
    /// already sampled value in local diagnostic state, but must not reenter
    /// facade/actor state or change operation epochs. This is a task evaluation
    /// sample, not an RF timestamp.
    pub async fn wait_with_clock(self, now_us: impl FnMut() -> u64) -> TxReport {
        wait_with(self, now_us, || embassy_time::Timer::after_micros(250)).await
    }

    /// Consumes the ticket. WrongContext does not mark it finished, but its Drop
    /// cancellation is also rejected there; task service/Esb::cancel is still owed.
    pub fn cancel(mut self) -> TxOutcome {
        let outcome = self.esb.cancel(self.epoch);
        self.finished |= ticket_finished(outcome);
        outcome
    }
}

impl<T: TimerInstance, const N: usize, const SIZE: usize> WaitEndpoint
    for SendTicket<'_, T, N, SIZE>
{
    fn epoch(&self) -> OperationEpoch {
        self.epoch
    }

    fn check_context(&self) -> Result<(), TxFault> {
        task_access(|| ())
    }

    fn register(&self, waker: &Waker) {
        self.esb.tx_notifier.register(waker);
    }

    fn poll(&mut self, now_us: u64) -> TxReport {
        let report = self.esb.poll_epoch(self.epoch, now_us);
        self.finished |= ticket_finished(report.outcome);
        report
    }
}

impl<T: TimerInstance, const N: usize, const SIZE: usize> Drop for SendTicket<'_, T, N, SIZE> {
    fn drop(&mut self) {
        if !self.finished {
            self.esb.cancel(self.epoch);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::{
        CompletionPolicy, Control, DeadlineGuardError, OperationEpoch, SEND_DEADLINE_US,
        TxDeadlineGuard, TxFault, TxLaunch, TxOutcome, arm_before_launch, disarm_before_restore,
        disarm_completion, finalize_after_restore, guard_expired, restore_current, ticket_finished,
        with_thread_context,
    };
    use super::{
        TxCompletionOrigin, TxEndCapture, TxEndCaptureReason, TxObservations, TxReport,
        observe_tx_event,
    };
    use crate::error::Error;
    use crate::isr::ReceivedPacket;
    use crate::payload::PacketPool;
    use crate::radio::StopOutcome;
    use core::cell::{Cell, RefCell};
    use core::task::Poll;
    use cortex_m::peripheral::scb::VectActive;
    use std::sync::Mutex;
    use std::vec::Vec;

    fn production_source() -> &'static str {
        let source = include_str!("esb.rs");
        let end = source
            .find("\n#[cfg(test)]\nmod tests")
            .expect("esb.rs must contain the final test-module boundary");
        &source[..end]
    }

    struct WaitModel {
        control: RefCell<Control>,
        notifier: embassy_sync::waitqueue::AtomicWaker,
        context: Cell<VectActive>,
        now: Cell<u64>,
        timer_ready: Cell<bool>,
        arms: Cell<usize>,
        registrations: Cell<usize>,
        clocks: Cell<usize>,
        drops: Cell<usize>,
        cancels: Cell<usize>,
        restored: Cell<bool>,
        on_register: Cell<Option<StopOutcome>>,
    }

    impl WaitModel {
        fn new() -> Self {
            let mut control = Control::new();
            control.begin(100);
            Self {
                control: RefCell::new(control),
                notifier: embassy_sync::waitqueue::AtomicWaker::new(),
                context: Cell::new(VectActive::ThreadMode),
                now: Cell::new(100),
                timer_ready: Cell::new(false),
                arms: Cell::new(0),
                registrations: Cell::new(0),
                clocks: Cell::new(0),
                drops: Cell::new(0),
                cancels: Cell::new(0),
                restored: Cell::new(false),
                on_register: Cell::new(None),
            }
        }

        fn irq(&self, epoch: OperationEpoch, stop: StopOutcome) {
            let mut control = self.control.borrow_mut();
            if let Some(flight) = control
                .active
                .as_mut()
                .filter(|flight| flight.epoch == epoch)
            {
                let _ = observe_tx_event(flight, None, TxCompletionOrigin::RadioIrq, |hook| {
                    if stop == StopOutcome::Normal {
                        hook(); // Modeled current-TX END + DISABLED predicate.
                    }
                    Some(stop)
                });
            }
            control.signal(epoch, stop);
            drop(control);
            self.notifier.wake(); // Signal loan is gone; not a terminal publication.
        }

        fn clock(&self) -> u64 {
            assert!(self.control.try_borrow_mut().is_ok());
            self.clocks.set(self.clocks.get() + 1);
            self.now.get()
        }

        fn fallback(&self) -> ModelTimer<'_> {
            self.arms.set(self.arms.get() + 1);
            self.timer_ready.set(false);
            ModelTimer(self)
        }

        fn endpoint(&self, epoch: OperationEpoch) -> ModelTicket<'_> {
            ModelTicket {
                model: self,
                epoch,
                finished: false,
                _pinned: core::marker::PhantomPinned,
            }
        }
    }

    struct ModelTimer<'a>(&'a WaitModel);

    impl core::future::Future for ModelTimer<'_> {
        type Output = ();
        fn poll(self: core::pin::Pin<&mut Self>, _: &mut core::task::Context<'_>) -> Poll<()> {
            if self.0.timer_ready.get() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    // Software adapter Drop only: this does NOT execute real MMIO ticket Drop.
    struct ModelTicket<'a> {
        model: &'a WaitModel,
        epoch: OperationEpoch,
        finished: bool,
        _pinned: core::marker::PhantomPinned,
    }

    impl super::WaitEndpoint for ModelTicket<'_> {
        fn epoch(&self) -> OperationEpoch {
            self.epoch
        }

        fn check_context(&self) -> Result<(), TxFault> {
            with_thread_context(self.model.context.get(), || ())
        }

        fn register(&self, waker: &core::task::Waker) {
            assert!(self.model.control.try_borrow_mut().is_ok());
            self.model
                .registrations
                .set(self.model.registrations.get() + 1);
            self.model.notifier.register(waker);
            if let Some(stop) = self.model.on_register.take() {
                self.model.irq(self.epoch, stop);
            }
        }

        fn poll(&mut self, now: u64) -> TxReport {
            let (report, published) = {
                let mut control = self.model.control.borrow_mut();
                let mut published = false;
                if control.matches(self.epoch) {
                    let outcome = control.evaluate(now);
                    if ticket_finished(outcome) {
                        self.model.restored.set(true); // Modeled PRX restoration.
                        control.finish(outcome);
                        published = true;
                    }
                }
                (control.report(self.epoch), published)
            };
            if published {
                assert!(self.model.restored.get());
                self.model.notifier.wake();
            }
            self.finished |= ticket_finished(report.outcome);
            report
        }
    }

    impl Drop for ModelTicket<'_> {
        fn drop(&mut self) {
            self.model.drops.set(self.model.drops.get() + 1);
            if !self.finished && self.check_context_for_drop() {
                let mut control = self.model.control.borrow_mut();
                if control.matches(self.epoch) {
                    self.model.cancels.set(self.model.cancels.get() + 1);
                    self.model.restored.set(true);
                    control.finish(TxOutcome::Fault(TxFault::Cancelled));
                }
            }
        }
    }

    impl ModelTicket<'_> {
        fn check_context_for_drop(&self) -> bool {
            self.model.context.get() == VectActive::ThreadMode
        }
    }

    #[derive(Default)]
    struct WakeCount(core::sync::atomic::AtomicUsize);

    impl std::task::Wake for WakeCount {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        fn wake_by_ref(self: &std::sync::Arc<Self>) {
            self.0.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
    }

    fn model_waker() -> (std::sync::Arc<WakeCount>, core::task::Waker) {
        let count = std::sync::Arc::new(WakeCount::default());
        (count.clone(), core::task::Waker::from(count))
    }

    #[test]
    fn wait_registration_race_does_not_lose_irq_and_restores_before_report() {
        use core::future::Future;
        let model = WaitModel::new();
        model.on_register.set(Some(StopOutcome::Normal));
        let (count, waker) = model_waker();
        let future = super::wait_with(
            model.endpoint(OperationEpoch(1)),
            || model.clock(),
            || model.fallback(),
        );
        let mut future = core::pin::pin!(future);
        let report = future
            .as_mut()
            .poll(&mut core::task::Context::from_waker(&waker));
        assert_eq!(
            report,
            Poll::Ready(super::TxReport {
                epoch: OperationEpoch(1),
                outcome: TxOutcome::PhysicalComplete,
                observations: Some(TxObservations {
                    completion: Some(TxCompletionOrigin::RadioIrq),
                    provider_expiry_observed: false,
                    end_capture: None,
                }),
            })
        );
        assert!(model.restored.get());
        assert!(count.0.load(core::sync::atomic::Ordering::Relaxed) >= 1);
        assert_eq!(model.arms.get(), 0);
        assert_eq!(model.drops.get(), 1);
        assert_eq!(model.cancels.get(), 0);
    }

    #[test]
    fn wait_two_pending_polls_one_irq_and_only_terminal_completion() {
        use core::future::Future;
        let model = WaitModel::new();
        let (count, waker) = model_waker();
        let mut samples = Vec::new();
        {
            let mut future = core::pin::pin!(super::wait_with(
                model.endpoint(OperationEpoch(1)),
                || {
                    let now = model.clock();
                    samples.push(now);
                    now
                },
                || model.fallback()
            ));
            let mut cx = core::task::Context::from_waker(&waker);
            assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
            assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
            assert!(!model.restored.get());
            model.irq(OperationEpoch(1), StopOutcome::Normal);
            assert_eq!(count.0.load(core::sync::atomic::Ordering::Relaxed), 1);
            assert_eq!(
                model.control.borrow().outcome(OperationEpoch(1)),
                TxOutcome::Pending
            );
            model.now.set(200);
            let Poll::Ready(report) = future.as_mut().poll(&mut cx) else {
                panic!("terminal report expected")
            };
            assert_eq!(report.outcome, TxOutcome::PhysicalComplete);
            assert_eq!(
                report.observations.unwrap().completion,
                Some(TxCompletionOrigin::RadioIrq)
            );
            assert!(model.restored.get());
        }
        assert_eq!(samples, [100, 100, 100, 200]);
        assert_eq!(
            samples.last().copied(),
            Some(model.control.borrow().clock_us)
        );
    }

    #[test]
    fn wait_fallback_resets_ready_timer_and_services_deadline_without_irq() {
        use core::future::Future;
        let model = WaitModel::new();
        let (_, waker) = model_waker();
        let mut samples = Vec::new();
        let mut terminal_samples = [1549, 1550].into_iter();
        {
            let mut future = core::pin::pin!(super::wait_with(
                model.endpoint(OperationEpoch(1)),
                || {
                    let now = model.clock();
                    let now = if now == 1549 {
                        terminal_samples.next().expect("only two terminal samples")
                    } else {
                        now
                    };
                    samples.push(now);
                    now
                },
                || model.fallback()
            ));
            let mut cx = core::task::Context::from_waker(&waker);
            assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
            for now in [350, 600, 850, 1100, 1350] {
                model.now.set(now);
                model.timer_ready.set(true);
                let arms = model.arms.get();
                assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
                assert_eq!(model.arms.get(), arms + 1);
                assert!(!model.timer_ready.get());
            }
            model.now.set(1549);
            model.timer_ready.set(true);
            let Poll::Ready(report) = future.as_mut().poll(&mut cx) else {
                panic!("deadline report expected")
            };
            assert_eq!(report.outcome, TxOutcome::Fault(TxFault::TimedOut));
            assert!(model.restored.get());
            assert_eq!(model.cancels.get(), 0);
        }
        assert_eq!(&samples[samples.len() - 2..], &[1549, 1550]);
        assert_eq!(samples.len(), model.clocks.get());
        assert_eq!(
            samples.last().copied(),
            Some(model.control.borrow().clock_us)
        );
    }

    #[test]
    fn wait_fault_is_terminal_and_stale_notification_cannot_complete_new_epoch() {
        use core::future::Future;
        let model = WaitModel::new();
        model
            .control
            .borrow_mut()
            .finish(TxOutcome::Fault(TxFault::Cancelled));
        let current = model.control.borrow_mut().begin(100);
        let (_, waker) = model_waker();
        let mut future = core::pin::pin!(super::wait_with(
            model.endpoint(current),
            || model.clock(),
            || model.fallback()
        ));
        let mut cx = core::task::Context::from_waker(&waker);
        assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
        model.irq(OperationEpoch(1), StopOutcome::Normal);
        assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
        model.irq(current, StopOutcome::PowerCycled);
        let Poll::Ready(report) = future.as_mut().poll(&mut cx) else {
            panic!("fault report expected")
        };
        assert_eq!(report.epoch, current);
        assert_eq!(report.outcome, TxOutcome::Fault(TxFault::PowerCycled));
        assert_eq!(report.observations.unwrap().completion, None);
        assert!(model.restored.get());
    }

    #[test]
    fn wait_pending_future_drop_owns_one_adapter_cancellation() {
        use core::future::Future;
        let model = WaitModel::new();
        let (_, waker) = model_waker();
        {
            let mut future = core::pin::pin!(super::wait_with(
                model.endpoint(OperationEpoch(1)),
                || model.clock(),
                || model.fallback()
            ));
            assert_eq!(
                future
                    .as_mut()
                    .poll(&mut core::task::Context::from_waker(&waker)),
                Poll::Pending
            );
        }
        assert_eq!(model.drops.get(), 1);
        assert_eq!(model.cancels.get(), 1);
        assert!(model.restored.get());
        // AtomicWaker owns a valid Rust Waker after future Drop, no ticket pointer.
        model.notifier.wake();
        assert_eq!(model.cancels.get(), 1);
    }

    #[test]
    fn wait_wrong_context_gates_initial_and_resumed_registration_clock_and_timer() {
        use core::future::Future;
        for initially_wrong in [true, false] {
            let model = WaitModel::new();
            let (_, waker) = model_waker();
            let mut future = core::pin::pin!(super::wait_with(
                model.endpoint(OperationEpoch(1)),
                || model.clock(),
                || model.fallback()
            ));
            let mut cx = core::task::Context::from_waker(&waker);
            if !initially_wrong {
                assert_eq!(future.as_mut().poll(&mut cx), Poll::Pending);
            }
            model.context.set(VectActive::from(2).unwrap());
            let before = (
                model.registrations.get(),
                model.clocks.get(),
                model.arms.get(),
            );
            let interrupted_loan = model.control.borrow_mut();
            let Poll::Ready(report) = future.as_mut().poll(&mut cx) else {
                panic!("context rejection expected")
            };
            assert_eq!(report.outcome, TxOutcome::Fault(TxFault::WrongContext));
            assert_eq!(report.observations, None);
            assert_eq!(
                (
                    model.registrations.get(),
                    model.clocks.get(),
                    model.arms.get()
                ),
                before
            );
            drop(interrupted_loan);
            assert!(model.control.borrow().matches(OperationEpoch(1)));
            assert_eq!(model.drops.get(), 1);
            assert_eq!(model.cancels.get(), 0);
        }
    }

    #[test]
    fn wait_stale_ticket_does_not_service_or_cancel_current_epoch() {
        use core::future::Future;
        let model = WaitModel::new();
        model
            .control
            .borrow_mut()
            .finish(TxOutcome::Fault(TxFault::Cancelled));
        let current = model.control.borrow_mut().begin(100);
        model
            .control
            .borrow_mut()
            .finish(TxOutcome::PhysicalComplete);
        let newest = model.control.borrow_mut().begin(100);
        let (_, waker) = model_waker();
        let mut future = core::pin::pin!(super::wait_with(
            model.endpoint(OperationEpoch(1)),
            || model.clock(),
            || model.fallback()
        ));
        let Poll::Ready(report) = future
            .as_mut()
            .poll(&mut core::task::Context::from_waker(&waker))
        else {
            panic!("stale report expected")
        };
        assert_eq!(report.outcome, TxOutcome::Fault(TxFault::StaleEpoch));
        assert_eq!(report.observations, None);
        assert_ne!(current, newest);
        assert!(model.control.borrow().matches(newest));
        assert_eq!(model.cancels.get(), 0);
        assert!(!model.restored.get());
    }

    #[derive(Default)]
    struct GuardState {
        epoch: Option<OperationEpoch>,
        deadline: u64,
        error: Option<DeadlineGuardError>,
        expired: bool,
        expire_on_disarm: bool,
        expiry_samples: std::collections::VecDeque<bool>,
        routed: bool,
        launch_result: Option<Result<TxLaunch, DeadlineGuardError>>,
        capture: Option<TxEndCapture>,
        calls: Vec<&'static str>,
    }

    #[derive(Default)]
    struct FakeGuard(Mutex<GuardState>);

    // SAFETY: host-only independent Mutex state, no MMIO, actors or reentry.
    // No timer/PPI/RADIO capability is fabricated by this orchestration model.
    unsafe impl TxDeadlineGuard for FakeGuard {
        fn arm(&self, epoch: OperationEpoch, deadline_us: u64) -> Result<(), DeadlineGuardError> {
            let mut state = self.0.lock().unwrap();
            state.calls.push("arm");
            state.epoch = Some(epoch);
            state.deadline = deadline_us;
            state.routed = true; // Model partially armed hardware even on error.
            state.expired = state.error == Some(DeadlineGuardError::Expired);
            state.error.map_or(Ok(()), Err)
        }

        fn disarm(&self) {
            let mut state = self.0.lock().unwrap();
            state.calls.push("disarm");
            state.routed = false;
            state.expired |= state.expire_on_disarm;
        }

        fn launch(&self, epoch: OperationEpoch) -> Result<TxLaunch, DeadlineGuardError> {
            let mut state = self.0.lock().unwrap();
            assert_eq!(state.epoch, Some(epoch));
            state.calls.push("guard_launch");
            let result = state.launch_result.unwrap_or(Ok(TxLaunch::Cpu));
            if result.is_err() {
                state.routed = false;
            }
            result
        }

        fn expired(&self, epoch: OperationEpoch) -> bool {
            let mut state = self.0.lock().unwrap();
            state.calls.push("expired");
            let expired = state.expiry_samples.pop_front().unwrap_or(state.expired);
            state.epoch == Some(epoch) && expired
        }

        fn on_interrupt(&self) {
            let mut state = self.0.lock().unwrap();
            state.calls.push("irq");
            state.expired = true;
        }
    }

    #[test]
    fn deadline_arm_precedes_launch_with_the_prepared_absolute_deadline() {
        let guard = FakeGuard::default();
        let mut control = Control::new();
        let epoch = control.begin(100);
        let deadline = control.active.as_ref().unwrap().deadline_us;
        arm_before_launch(Some(&guard), epoch, deadline, |armed| {
            assert!(!armed.0);
            let mut state = guard.0.lock().unwrap();
            assert_eq!(state.epoch, Some(epoch));
            assert_eq!(state.deadline, 100 + SEND_DEADLINE_US);
            assert!(state.routed);
            state.calls.push("launch");
        })
        .unwrap();
        assert_eq!(guard.0.lock().unwrap().calls, ["arm", "launch"]);
    }

    #[test]
    fn caller_cutoff_is_stored_and_armed_instead_of_default_deadline() {
        let guard = FakeGuard::default();
        let mut control = Control::new();
        let epoch = control.begin_until(16_500, 17_500);
        let deadline = control.active.as_ref().unwrap().deadline_us;
        arm_before_launch(Some(&guard), epoch, deadline, |_| {
            assert_eq!(guard.0.lock().unwrap().deadline, 17_500);
        })
        .unwrap();
        assert_eq!(control.evaluate(17_499), TxOutcome::Pending);
        assert_eq!(
            control.evaluate(17_500),
            TxOutcome::Fault(TxFault::TimedOut)
        );
    }

    #[test]
    fn invalid_caller_cutoffs_do_not_claim_or_increment_epoch() {
        let mut control = Control::new();
        for until in [0, 100, 1551, u64::MAX] {
            assert_eq!(
                control.can_begin_until(100, until),
                Err(Error::InvalidParam)
            );
            assert_eq!(control.sequence, 0);
            assert_eq!(control.clock_us, 0);
            assert!(control.active.is_none());
        }
        assert_eq!(
            control.can_begin_until(u64::MAX, u64::MAX),
            Err(Error::InvalidParam)
        );
        assert_eq!(control.can_begin_until(100, 101), Ok(()));
        assert_eq!(control.can_begin_until(100, 1550), Ok(()));
        control.begin_until(100, 101);
        assert_eq!(control.evaluate(101), TxOutcome::Fault(TxFault::TimedOut));
    }

    fn prepared_launch_model(result: Result<TxLaunch, DeadlineGuardError>) {
        use crate::state_machine::{StatePtx, prepare_and_launch};
        use core::sync::atomic::{Ordering, compiler_fence};

        // Real pool claim plus a register/state model: no fabricated peripherals,
        // physical DMA, barrier, stop or ISR execution is asserted here.
        let pool = PacketPool::<1, 256>::new();
        let guard = FakeGuard::default();
        let epoch = Control::new().begin(100);
        guard.0.lock().unwrap().launch_result = Some(result);
        guard.arm(epoch, 1550).unwrap();
        let idx = pool.alloc_tx().unwrap();
        unsafe {
            pool.header_mut(idx).set_no_ack(true);
            pool.buf_mut(idx)[crate::header::EsbHeader::PAYLOAD_OFFSET] = 42;
        }
        pool.claim_allocated_tx(idx);
        let state = Cell::new(StatePtx::TxNoAck);
        let dma = unsafe { pool.dma_ptr(idx) };
        let prepared = Cell::new(false);
        let cpu_starts = Cell::new(0);
        let actual = prepare_and_launch(
            &mut (),
            |_| {
                assert_eq!(state.get(), StatePtx::TxNoAck);
                assert_eq!(pool.alloc_tx(), None);
                guard.0.lock().unwrap().calls.push("prepare_dma");
                compiler_fence(Ordering::Release);
                guard.0.lock().unwrap().calls.push("fence");
                prepared.set(true);
            },
            || {
                assert!(prepared.get());
                assert_eq!(state.get(), StatePtx::TxNoAck);
                assert_eq!(unsafe { pool.dma_ptr(idx) }, dma);
                assert_eq!(pool.alloc_tx(), None);
                guard.launch(epoch)
            },
            |_| {
                cpu_starts.set(cpu_starts.get() + 1);
                guard.0.lock().unwrap().calls.push("cpu_start");
            },
        );
        assert_eq!(actual, result.map(|_| ()));
        assert_eq!(cpu_starts.get(), usize::from(result == Ok(TxLaunch::Cpu)));
        assert_eq!(pool.alloc_tx(), None); // Even Err retains the DMA claim.
        assert_eq!(state.get(), StatePtx::TxNoAck);
        let mut expected = std::vec!["arm", "prepare_dma", "fence", "guard_launch"];
        if result == Ok(TxLaunch::Cpu) {
            expected.push("cpu_start");
        }
        assert_eq!(guard.0.lock().unwrap().calls, expected);
        if result.is_err() {
            assert!(!guard.0.lock().unwrap().routed);
        }
        disarm_before_restore(Some(&guard), || {
            assert!(!guard.0.lock().unwrap().routed);
            assert_eq!(pool.alloc_tx(), None);
            guard.0.lock().unwrap().calls.push("stop"); // Quiescent-stop model.
            pool.release_tx(idx);
            state.set(StatePtx::Idle);
            guard.0.lock().unwrap().calls.push("release");
        });
        expected.extend(["disarm", "stop", "release"]);
        assert_eq!(guard.0.lock().unwrap().calls, expected);
        assert_eq!(pool.alloc_tx(), Some(idx));
    }

    #[test]
    fn prepared_cpu_launch_starts_exactly_once() {
        prepared_launch_model(Ok(TxLaunch::Cpu));
    }

    #[test]
    fn prepared_scheduled_launch_never_starts_cpu() {
        prepared_launch_model(Ok(TxLaunch::Scheduled));
    }

    #[test]
    fn existing_provider_without_launch_override_defaults_to_cpu() {
        struct LegacyGuard;
        // SAFETY: independent no-resource host stub, no reentry or hardware.
        unsafe impl TxDeadlineGuard for LegacyGuard {
            fn arm(&self, _: OperationEpoch, _: u64) -> Result<(), DeadlineGuardError> {
                Ok(())
            }
            fn disarm(&self) {}
            fn expired(&self, _: OperationEpoch) -> bool {
                false
            }
            fn on_interrupt(&self) {}
        }
        let provider: &dyn TxDeadlineGuard = &LegacyGuard;
        assert_eq!(provider.launch(OperationEpoch(1)), Ok(TxLaunch::Cpu));
        let driver = include_str!("isr.rs");
        assert!(driver.contains(
            "fn begin_exclusive_no_ack(&mut self, pipe: u8, payload: &[u8]) -> Result<(), Error>"
        ));
        assert!(driver.contains(
            "self.begin_exclusive_no_ack_with_launch(pipe, payload, || Ok(TxLaunch::Cpu))"
        ));
    }

    #[test]
    fn prepared_expired_launch_retains_claim_until_disarm_stop_release() {
        prepared_launch_model(Err(DeadlineGuardError::Expired));
        assert_eq!(
            super::guard_fault(DeadlineGuardError::Expired),
            TxFault::Deadline
        );
    }

    #[test]
    fn prepared_faulty_launch_retains_claim_until_disarm_stop_release() {
        prepared_launch_model(Err(DeadlineGuardError::HardwareFault));
        prepared_launch_model(Err(DeadlineGuardError::Unavailable));
        assert_eq!(
            super::guard_fault(DeadlineGuardError::HardwareFault),
            TxFault::DeadlineGuardFault
        );
    }

    #[test]
    fn exclusive_source_prepares_owned_state_before_provider_and_cpu_launch() {
        // Source-order evidence only, not MMIO or hardware timing qualification.
        let sm = include_str!("state_machine.rs")
            .split("#[cfg(test)]")
            .next()
            .unwrap();
        let exclusive = sm
            .split("fn start_exclusive_no_ack_with_launch")
            .nth(1)
            .unwrap()
            .split("fn take_no_ack_completion")
            .next()
            .unwrap();
        let claim = exclusive.find("pool.claim_allocated_tx(idx)").unwrap();
        let index = exclusive.find("self.tx_idx = idx").unwrap();
        let state = exclusive.find("self.state = StatePtx::TxNoAck").unwrap();
        let helper = exclusive.find("prepare_and_launch(").unwrap();
        assert!(claim < index && index < state && state < helper);
        assert!(!exclusive.contains("release_tx"));
        assert!(!exclusive.contains("StatePtx::Idle;"));
        let helper = sm.split("fn prepare_and_launch<C>").nth(1).unwrap();
        assert!(helper.find("prepare(context)").unwrap() < helper.find("match launch()?").unwrap());
        assert!(helper.contains("TxLaunch::Scheduled => {}"));
        let radio = include_str!("radio.rs");
        let prepare = radio
            .split("fn prepare_transmit")
            .nth(1)
            .unwrap()
            .split("fn launch_transmit_cpu")
            .next()
            .unwrap();
        assert!(
            prepare.find("packetptr()").unwrap()
                < prepare.find("compiler_fence(Ordering::Release)").unwrap()
        );
        assert!(
            prepare.find("compiler_fence(Ordering::Release)").unwrap()
                < prepare.find("cortex_m::asm::dmb()").unwrap()
        );
        assert!(!prepare.contains("tasks_txen"));
        let driver = include_str!("isr.rs");
        assert!(driver.contains(".start_exclusive_no_ack_with_launch("));
        let facade = production_source();
        assert!(facade.contains("provider.launch(epoch)"));
        let contract = facade
            .split("pub unsafe trait TxDeadlineGuard")
            .next()
            .unwrap();
        assert!(contract.contains("including arm/disarm before activation"));
        assert!(contract.contains("NEVER synchronously"));
        assert!(contract.contains("MUST NEVER re-enable gating after cutoff"));
    }

    #[test]
    fn every_arm_error_is_terminal_without_launch_and_disarms_before_restore() {
        for (error, fault) in [
            (DeadlineGuardError::Unavailable, TxFault::DeadlineGuardFault),
            (DeadlineGuardError::Expired, TxFault::Deadline),
            (
                DeadlineGuardError::HardwareFault,
                TxFault::DeadlineGuardFault,
            ),
        ] {
            let guard = FakeGuard::default();
            guard.0.lock().unwrap().error = Some(error);
            let mut control = Control::new();
            let epoch = control.begin(100);
            let result =
                arm_before_launch::<()>(Some(&guard), epoch, 1550, |_| panic!("must not launch"));
            assert_eq!(result, Err(fault));
            disarm_before_restore(Some(&guard), || {
                let mut state = guard.0.lock().unwrap();
                assert!(!state.routed);
                // Ordered model, not executed RADIO stop or real claim release.
                state
                    .calls
                    .extend(["quiesce", "disabled", "release", "take", "rx_epoch", "rx"]);
            });
            let report = control.finish(TxOutcome::Fault(fault));
            assert_eq!(
                report.observations,
                Some(TxObservations {
                    completion: None,
                    provider_expiry_observed: false,
                    end_capture: None,
                })
            );
            assert!(ticket_finished(report.outcome));
            assert_ne!(report.outcome, TxOutcome::Fault(TxFault::WrongContext));
            assert_eq!(
                guard.0.lock().unwrap().calls,
                [
                    "arm", "disarm", "quiesce", "disabled", "release", "take", "rx_epoch", "rx"
                ]
            );
        }
    }

    #[test]
    fn matching_expiry_overrides_latched_end_completion_and_late_service() {
        for now in [101, 1550, 2000] {
            let guard = FakeGuard::default();
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            guard.on_interrupt();
            assert!(!guard_expired(Some(&guard), OperationEpoch(0)));
            control.signal(epoch, StopOutcome::Normal); // Model END + DISABLED.
            control.active.as_mut().unwrap().guard_expired = disarm_completion(Some(&guard), epoch);
            assert!(guard_expired(Some(&guard), epoch)); // Disarm preserves expiry.
            assert_eq!(control.evaluate(now), TxOutcome::Fault(TxFault::Deadline));
        }
    }

    #[test]
    fn first_completion_origin_survives_repoll_and_late_timeout() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            let mut control = Control::new();
            let epoch = control.begin(100);
            let stop = observe_tx_event(control.active.as_mut().unwrap(), None, origin, |hook| {
                hook();
                Some(StopOutcome::Normal)
            })
            .unwrap();
            control.signal(epoch, stop);
            assert_eq!(control.evaluate(101), TxOutcome::PhysicalComplete);
            // An Idle driver can return a retained stop without invoking the predicate.
            let _ = observe_tx_event(
                control.active.as_mut().unwrap(),
                None,
                TxCompletionOrigin::TaskService,
                |_| Some(StopOutcome::Normal),
            );
            let outcome = control.evaluate(1550);
            assert_eq!(outcome, TxOutcome::Fault(TxFault::TimedOut));
            let report = control.finish(outcome);
            assert_eq!(
                report.observations,
                Some(TxObservations {
                    completion: Some(origin),
                    provider_expiry_observed: false,
                    end_capture: None,
                })
            );
            assert_eq!(control.report(epoch), report);
        }
    }

    struct CaptureGuard(FakeGuard);

    // SAFETY: delegates to independent host-only Mutex state; no capabilities,
    // hardware, actor callbacks or reentry are exercised by this model.
    unsafe impl TxDeadlineGuard for CaptureGuard {
        fn arm(&self, epoch: OperationEpoch, deadline_us: u64) -> Result<(), DeadlineGuardError> {
            self.0.arm(epoch, deadline_us)
        }
        fn launch(&self, epoch: OperationEpoch) -> Result<TxLaunch, DeadlineGuardError> {
            self.0.launch(epoch)
        }
        fn disarm(&self) {
            self.0.disarm();
        }
        fn expired(&self, epoch: OperationEpoch) -> bool {
            self.0.expired(epoch)
        }
        fn on_interrupt(&self) {
            self.0.on_interrupt();
        }
        fn capture_end(&self, epoch: OperationEpoch) -> Option<TxEndCapture> {
            let mut state = self.0.0.lock().unwrap();
            assert_eq!(state.epoch, Some(epoch));
            assert!(!state.routed); // Original guard disarm and expiry query come first.
            assert_eq!(state.calls.last(), Some(&"expired"));
            state.calls.push("capture");
            state.capture
        }
    }

    fn candidate(epoch: OperationEpoch) -> TxEndCapture {
        TxEndCapture {
            epoch,
            anchor_raw: 100,
            cutoff_raw: 1550,
            sampled_cc: [200; 2],
            observation_raw: 250,
            elapsed_us: 150,
            horizon_us: 1450,
            reason: TxEndCaptureReason::Candidate,
            cutoff_delta_us: Some(-1350),
        }
    }

    struct RetainingGuard(CaptureGuard);

    // SAFETY: software-only orchestration model with independent Mutex state.
    // The modeled contract is not evidence of autonomous physical cutoff timing.
    unsafe impl TxDeadlineGuard for RetainingGuard {
        fn arm(&self, epoch: OperationEpoch, deadline_us: u64) -> Result<(), DeadlineGuardError> {
            self.0.arm(epoch, deadline_us)
        }
        fn completion_policy(&self) -> CompletionPolicy {
            self.0.0.0.lock().unwrap().calls.push("policy");
            CompletionPolicy::RetainGuardedCompletion
        }
        fn launch(&self, epoch: OperationEpoch) -> Result<TxLaunch, DeadlineGuardError> {
            self.0.launch(epoch)
        }
        fn disarm(&self) {
            self.0.disarm();
        }
        fn expired(&self, epoch: OperationEpoch) -> bool {
            self.0.expired(epoch)
        }
        fn on_interrupt(&self) {
            self.0.on_interrupt();
        }
        fn capture_end(&self, epoch: OperationEpoch) -> Option<TxEndCapture> {
            self.0.capture_end(epoch)
        }
    }

    fn arm_flight(
        control: &mut Control,
        provider: Option<&dyn TxDeadlineGuard>,
    ) -> Result<(), TxFault> {
        let flight = control.active.as_mut().unwrap();
        let epoch = flight.epoch;
        arm_before_launch(provider, epoch, flight.deadline_us, |armed| {
            assert_eq!(flight.epoch, epoch);
            flight.guarded_completion_retention = armed.0;
        })
    }

    fn complete_flight(
        control: &mut Control,
        provider: Option<&dyn TxDeadlineGuard>,
        origin: TxCompletionOrigin,
        stop: StopOutcome,
    ) {
        let flight = control.active.as_mut().unwrap();
        let epoch = flight.epoch;
        let observed = observe_tx_event(flight, provider, origin, |hook| {
            hook();
            Some(stop)
        })
        .unwrap();
        control.signal(epoch, observed);
    }

    #[test]
    fn explicit_successful_arm_retains_both_completion_origins_at_and_after_cutoff() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            for now in [499, 500, 501, 10_000] {
                for captured in [false, true] {
                    let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
                    let mut control = Control::new();
                    let epoch = control.begin_until(100, 500);
                    assert!(
                        !control
                            .active
                            .as_ref()
                            .unwrap()
                            .guarded_completion_retention
                    );
                    arm_flight(&mut control, Some(&guard)).unwrap();
                    assert!(
                        control
                            .active
                            .as_ref()
                            .unwrap()
                            .guarded_completion_retention
                    );
                    if captured {
                        guard.0.0.0.lock().unwrap().capture = Some(candidate(epoch));
                    }
                    assert_eq!(guard.0.0.0.lock().unwrap().calls, ["arm", "policy"]);
                    complete_flight(&mut control, Some(&guard), origin, StopOutcome::Normal);
                    assert_eq!(control.evaluate(now), TxOutcome::PhysicalComplete);
                    // Model the unchanged service restoration and final expiry query.
                    disarm_before_restore(Some(&guard), || {});
                    control.active.as_mut().unwrap().query_expiry(Some(&guard));
                    assert_eq!(control.evaluate(now), TxOutcome::PhysicalComplete);
                    let report = control.finish(TxOutcome::PhysicalComplete);
                    assert_eq!(report.observations.unwrap().completion, Some(origin));
                    assert_eq!(
                        guard.0.0.0.lock().unwrap().calls,
                        [
                            "arm", "policy", "expired", "disarm", "expired", "capture", "disarm",
                            "expired",
                        ]
                    );
                }
            }
        }
    }

    #[test]
    fn absent_default_and_unarmed_providers_cannot_authorize_late_completion() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            let strict = CaptureGuard(FakeGuard::default());
            let retaining = RetainingGuard(CaptureGuard(FakeGuard::default()));
            for (provider, armed) in [
                (None, true),
                (Some(&strict as &dyn TxDeadlineGuard), true),
                (Some(&retaining as &dyn TxDeadlineGuard), false),
            ] {
                let mut control = Control::new();
                let epoch = control.begin_until(100, 500);
                if armed {
                    arm_flight(&mut control, provider).unwrap();
                } else {
                    // Even an actual provider arm outside the flight orchestration
                    // cannot grant this flight a witness from pointer presence.
                    retaining.arm(epoch, 500).unwrap();
                }
                strict.0.0.lock().unwrap().capture = Some(candidate(epoch));
                complete_flight(&mut control, provider, origin, StopOutcome::Normal);
                assert!(
                    !control
                        .active
                        .as_ref()
                        .unwrap()
                        .guarded_completion_retention
                );
                assert_eq!(control.evaluate(500), TxOutcome::Fault(TxFault::TimedOut));
                assert_eq!(control.evaluate(501), TxOutcome::Fault(TxFault::TimedOut));
            }
        }
        assert_eq!(
            FakeGuard::default().completion_policy(),
            CompletionPolicy::StrictSoftwareDeadline
        );
    }

    #[test]
    fn retained_completion_finalizes_restoration_faults_and_expiry_before_publication() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            for restoration_fault in [
                Some(TxFault::PowerCycled),
                Some(TxFault::ReceiverUnavailable),
                Some(TxFault::RxEpochOverflow),
                None,
            ] {
                for expired_after_restore in [false, true] {
                    for captured in [false, true] {
                        let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
                        let mut control = Control::new();
                        let epoch = control.begin_until(100, 500);
                        arm_flight(&mut control, Some(&guard)).unwrap();
                        let capture = captured.then(|| candidate(epoch));
                        {
                            let mut state = guard.0.0.0.lock().unwrap();
                            state.capture = capture;
                            state
                                .expiry_samples
                                .extend([false, false, expired_after_restore]);
                        }
                        complete_flight(&mut control, Some(&guard), origin, StopOutcome::Normal);
                        let proposed = control.evaluate(10_000);
                        assert_eq!(proposed, TxOutcome::PhysicalComplete);
                        // Supply the restoration result without MMIO. Execute the same
                        // production arbitration as service, then publish its result.
                        let restored = disarm_before_restore(Some(&guard), || restoration_fault);
                        let flight = control.active.as_mut().unwrap();
                        let expired = flight.query_expiry(Some(&guard));
                        let outcome = finalize_after_restore(
                            proposed,
                            restored,
                            expired || flight.guard_expired,
                        );
                        let expected = if expired_after_restore {
                            TxOutcome::Fault(TxFault::Deadline)
                        } else {
                            restoration_fault.map_or(TxOutcome::PhysicalComplete, TxOutcome::Fault)
                        };
                        let report = control.finish(outcome);
                        assert_eq!(report.epoch, epoch);
                        assert_eq!(report.outcome, expected);
                        assert_eq!(
                            report.observations,
                            Some(TxObservations {
                                completion: Some(origin),
                                provider_expiry_observed: expired_after_restore,
                                end_capture: capture,
                            })
                        );
                        assert!(control.active.is_none());
                        assert_eq!(control.report(epoch), report);
                        assert_eq!(
                            guard.0.0.0.lock().unwrap().calls,
                            [
                                "arm", "policy", "expired", "disarm", "expired", "capture",
                                "disarm", "expired",
                            ]
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn successful_restoration_does_not_promote_strict_late_completion() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            let guard = CaptureGuard(FakeGuard::default());
            let mut control = Control::new();
            let epoch = control.begin_until(100, 500);
            arm_flight(&mut control, Some(&guard)).unwrap();
            let capture = Some(candidate(epoch));
            guard.0.0.lock().unwrap().capture = capture;
            complete_flight(&mut control, Some(&guard), origin, StopOutcome::Normal);
            let proposed = control.evaluate(500);
            assert_eq!(proposed, TxOutcome::Fault(TxFault::TimedOut));
            let restored = disarm_before_restore(Some(&guard), || None);
            let flight = control.active.as_mut().unwrap();
            let expired = flight.query_expiry(Some(&guard));
            let outcome =
                finalize_after_restore(proposed, restored, expired || flight.guard_expired);
            let report = control.finish(outcome);
            assert_eq!(report.epoch, epoch);
            assert_eq!(report.outcome, TxOutcome::Fault(TxFault::TimedOut));
            assert_eq!(
                report.observations,
                Some(TxObservations {
                    completion: Some(origin),
                    provider_expiry_observed: false,
                    end_capture: capture,
                })
            );
            assert_eq!(control.report(epoch), report);
        }
    }

    #[test]
    fn failed_opt_in_arm_never_queries_policy_or_grants_a_witness() {
        for error in [
            DeadlineGuardError::Unavailable,
            DeadlineGuardError::Expired,
            DeadlineGuardError::HardwareFault,
        ] {
            let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
            guard.0.0.0.lock().unwrap().error = Some(error);
            let mut control = Control::new();
            control.begin_until(100, 500);
            assert_eq!(
                arm_flight(&mut control, Some(&guard)),
                Err(super::guard_fault(error))
            );
            assert!(
                !control
                    .active
                    .as_ref()
                    .unwrap()
                    .guarded_completion_retention
            );
            assert_eq!(guard.0.0.0.lock().unwrap().calls, ["arm"]);
            disarm_before_restore(Some(&guard), || {});
            assert_eq!(
                control
                    .finish(TxOutcome::Fault(super::guard_fault(error)))
                    .outcome,
                TxOutcome::Fault(super::guard_fault(error))
            );
        }
    }

    #[test]
    fn opted_in_flight_still_requires_current_normal_completion() {
        for evidence in 0..3 {
            let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
            let mut control = Control::new();
            let epoch = control.begin_until(100, 500);
            arm_flight(&mut control, Some(&guard)).unwrap();
            if evidence == 1 {
                // Hook without the driver's normal quiescence signal is insufficient.
                let _ = observe_tx_event(
                    control.active.as_mut().unwrap(),
                    Some(&guard),
                    TxCompletionOrigin::TaskService,
                    |before| {
                        before();
                        None
                    },
                );
            } else if evidence == 2 {
                assert_eq!(control.evaluate(499), TxOutcome::Pending);
                control.signal(OperationEpoch(0), StopOutcome::Normal);
                assert_eq!(control.evaluate(499), TxOutcome::Pending);
                control.signal(epoch, StopOutcome::Normal); // No current completion hook.
            } else {
                assert_eq!(control.evaluate(499), TxOutcome::Pending);
            }
            assert_eq!(control.evaluate(500), TxOutcome::Fault(TxFault::TimedOut));
        }
    }

    #[test]
    fn opted_in_expiry_at_each_existing_query_and_disarm_race_overrides_capture() {
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            for samples in [
                [true, false, false],
                [false, true, false],
                [false, false, true],
            ] {
                let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
                let mut control = Control::new();
                let epoch = control.begin_until(100, 500);
                arm_flight(&mut control, Some(&guard)).unwrap();
                {
                    let mut state = guard.0.0.0.lock().unwrap();
                    state.capture = Some(candidate(epoch)); // Negative cutoff delta.
                    state.expiry_samples.extend(samples);
                }
                complete_flight(&mut control, Some(&guard), origin, StopOutcome::Normal);
                let tentative = control.evaluate(500);
                assert_eq!(
                    tentative,
                    if samples[0] || samples[1] {
                        TxOutcome::Fault(TxFault::Deadline)
                    } else {
                        TxOutcome::PhysicalComplete
                    }
                );
                disarm_before_restore(Some(&guard), || {});
                let flight = control.active.as_mut().unwrap();
                let expired = flight.query_expiry(Some(&guard));
                let outcome =
                    finalize_after_restore(tentative, None, expired || flight.guard_expired);
                assert_eq!(outcome, TxOutcome::Fault(TxFault::Deadline));
                assert!(
                    control
                        .finish(outcome)
                        .observations
                        .unwrap()
                        .provider_expiry_observed
                );
            }
            let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
            let mut control = Control::new();
            control.begin_until(100, 500);
            arm_flight(&mut control, Some(&guard)).unwrap();
            guard.0.0.0.lock().unwrap().expire_on_disarm = true;
            complete_flight(&mut control, Some(&guard), origin, StopOutcome::Normal);
            assert_eq!(control.evaluate(500), TxOutcome::Fault(TxFault::Deadline));
        }
    }

    #[test]
    fn retention_preserves_reset_clock_faults_and_new_flight_starts_strict() {
        for fault in [TxFault::PowerCycled, TxFault::ClockRegressed] {
            let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
            let mut control = Control::new();
            let epoch = control.begin_until(100, 500);
            arm_flight(&mut control, Some(&guard)).unwrap();
            complete_flight(
                &mut control,
                Some(&guard),
                TxCompletionOrigin::RadioIrq,
                if fault == TxFault::PowerCycled {
                    StopOutcome::PowerCycled
                } else {
                    StopOutcome::Normal
                },
            );
            if fault == TxFault::PowerCycled {
                control.signal(epoch, StopOutcome::Normal);
            }
            let now = if fault == TxFault::ClockRegressed {
                99
            } else {
                500
            };
            assert_eq!(control.evaluate(now), TxOutcome::Fault(fault));
            // Matching expiry keeps precedence over both reset and regression.
            control.active.as_mut().unwrap().guard_expired = true;
            assert_eq!(control.evaluate(now), TxOutcome::Fault(TxFault::Deadline));
            control.finish(TxOutcome::Fault(TxFault::Deadline));
            let next = control.begin(500);
            assert_ne!(epoch, next);
            assert!(
                !control
                    .active
                    .as_ref()
                    .unwrap()
                    .guarded_completion_retention
            );
            complete_flight(
                &mut control,
                None,
                TxCompletionOrigin::TaskService,
                StopOutcome::Normal,
            );
            assert_eq!(control.evaluate(1950), TxOutcome::Fault(TxFault::TimedOut));
        }
    }

    #[test]
    fn successful_arm_with_failed_launch_finishes_fault_and_never_returns_ticket() {
        let guard = RetainingGuard(CaptureGuard(FakeGuard::default()));
        let mut control = Control::new();
        let epoch = control.begin_until(100, 500);
        let flight = control.active.as_mut().unwrap();
        let result = arm_before_launch(Some(&guard), epoch, 500, |armed| {
            flight.guarded_completion_retention = armed.0;
            Err::<(), _>(TxFault::ReceiverUnavailable) // Driver launch failure model.
        })
        .unwrap();
        assert!(
            control
                .active
                .as_ref()
                .unwrap()
                .guarded_completion_retention
        );
        disarm_before_restore(Some(&guard), || {});
        let fault = result.unwrap_err();
        assert_eq!(
            control.finish(TxOutcome::Fault(fault)).outcome,
            TxOutcome::Fault(TxFault::ReceiverUnavailable)
        );
        assert!(control.active.is_none());
        // Source wiring supplements the model; no real facade/MMIO is constructed.
        let source = production_source();
        let begin = source
            .split("pub fn begin_send_no_ack_to_until")
            .nth(1)
            .unwrap()
            .split("pub fn service")
            .next()
            .unwrap();
        assert!(
            begin
                .find("flight.guarded_completion_retention = armed.0")
                .unwrap()
                < begin
                    .find("driver.begin_exclusive_no_ack_with_launch(")
                    .unwrap()
        );
        assert!(
            begin.find("if let Err(error) = launched").unwrap()
                < begin.find("Ok(SendTicket").unwrap()
        );
        let service = source
            .split("fn service(&mut self")
            .nth(1)
            .unwrap()
            .split("/// A single static")
            .next()
            .unwrap();
        assert!(
            service
                .find("let restored = self.restore_prx(false)")
                .unwrap()
                < service.find("flight.query_expiry(self.deadline)").unwrap()
        );
        assert!(
            service.find("flight.query_expiry(self.deadline)").unwrap()
                < service.find("outcome = finalize_after_restore(").unwrap()
        );
        assert!(service.contains(
            "finalize_after_restore(outcome, restored, expired || flight.guard_expired)"
        ));
        assert!(
            service.find("outcome = finalize_after_restore(").unwrap()
                < service
                    .find("if let TxOutcome::Fault(fault) = outcome")
                    .unwrap()
        );
        assert!(
            service.find("outcome = finalize_after_restore(").unwrap()
                < service.find("self.control.finish(outcome)").unwrap()
        );
    }

    #[test]
    fn default_capture_observer_is_inert_and_preserves_completion_query_sequence() {
        let guard = FakeGuard::default(); // Intentionally no capture_end override.
        let mut control = Control::new();
        let epoch = control.begin(100);
        assert_eq!(guard.capture_end(epoch), None);
        assert!(guard.0.lock().unwrap().calls.is_empty());
        guard.arm(epoch, 1550).unwrap();
        let stop = observe_tx_event(
            control.active.as_mut().unwrap(),
            Some(&guard),
            TxCompletionOrigin::RadioIrq,
            |hook| {
                hook();
                guard.0.lock().unwrap().calls.push("stop");
                Some(StopOutcome::Normal)
            },
        )
        .unwrap();
        control.signal(epoch, stop);
        assert_eq!(control.evaluate(101), TxOutcome::PhysicalComplete);
        assert_eq!(
            control.report(epoch).observations.unwrap().end_capture,
            None
        );
        assert_eq!(
            guard.0.lock().unwrap().calls,
            ["arm", "expired", "disarm", "expired", "stop"]
        );
    }

    #[test]
    fn matching_capture_survives_late_timeout_finish_poll_and_wait_per_epoch() {
        use core::future::Future;
        let model = WaitModel::new();
        let guard = CaptureGuard(FakeGuard::default());
        for origin in [
            TxCompletionOrigin::RadioIrq,
            TxCompletionOrigin::TaskService,
        ] {
            let epoch = model.control.borrow().active.as_ref().unwrap().epoch;
            let start = model.control.borrow().clock_us;
            let deadline = model.control.borrow().active.as_ref().unwrap().deadline_us;
            let capture = candidate(epoch);
            guard.arm(epoch, deadline).unwrap();
            guard.0.0.lock().unwrap().capture = Some(capture);
            {
                let mut control = model.control.borrow_mut();
                let stop = observe_tx_event(
                    control.active.as_mut().unwrap(),
                    Some(&guard),
                    origin,
                    |hook| {
                        hook();
                        guard.0.0.lock().unwrap().calls.push("stop_release");
                        Some(StopOutcome::Normal)
                    },
                )
                .unwrap();
                control.signal(epoch, stop);
                disarm_before_restore(Some(&guard), || {
                    guard.0.0.lock().unwrap().calls.push("restore")
                });
                control.active.as_mut().unwrap().query_expiry(Some(&guard));
                assert_eq!(control.evaluate(start + 1), TxOutcome::PhysicalComplete);
                assert_eq!(
                    control.report(epoch).observations.unwrap().end_capture,
                    Some(capture)
                );
                let outcome = control.evaluate(deadline);
                assert_eq!(outcome, TxOutcome::Fault(TxFault::TimedOut));
                let report = control.finish(outcome);
                assert_eq!(
                    report.observations,
                    Some(TxObservations {
                        completion: Some(origin),
                        provider_expiry_observed: false,
                        end_capture: Some(capture),
                    })
                );
                assert_eq!(control.report(epoch), report);
            }
            let retained = model.control.borrow().report(epoch);
            let current = model.control.borrow_mut().begin(deadline);
            assert_eq!(
                model
                    .control
                    .borrow()
                    .report(current)
                    .observations
                    .unwrap()
                    .end_capture,
                None
            );
            let (_, waker) = model_waker();
            {
                let mut future = core::pin::pin!(super::wait_with(
                    model.endpoint(epoch),
                    || model.clock(),
                    || model.fallback()
                ));
                assert_eq!(
                    future
                        .as_mut()
                        .poll(&mut core::task::Context::from_waker(&waker)),
                    Poll::Ready(retained)
                );
            }
            assert!(model.control.borrow().matches(current));
            assert_eq!(
                guard.0.0.lock().unwrap().calls,
                [
                    "arm",
                    "expired",
                    "disarm",
                    "expired",
                    "capture",
                    "stop_release",
                    "disarm",
                    "restore",
                    "expired"
                ]
            );
            guard.0.0.lock().unwrap().calls.clear();
        }
    }

    #[test]
    fn wrong_capture_epoch_is_diagnostic_only_with_existing_outcome_precedence() {
        for (expired, now, expected) in [
            (false, 101, TxOutcome::PhysicalComplete),
            (false, 1550, TxOutcome::Fault(TxFault::TimedOut)),
            (true, 101, TxOutcome::Fault(TxFault::Deadline)),
        ] {
            let guard = CaptureGuard(FakeGuard::default());
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            {
                let mut state = guard.0.0.lock().unwrap();
                state.capture = Some(candidate(OperationEpoch(0)));
                state.expired = expired;
            }
            let stop = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::RadioIrq,
                |hook| {
                    hook();
                    Some(StopOutcome::Normal)
                },
            )
            .unwrap();
            control.signal(epoch, stop);
            let outcome = control.evaluate(now);
            assert_eq!(outcome, expected);
            assert_eq!(
                control.finish(outcome).observations,
                Some(TxObservations {
                    completion: Some(TxCompletionOrigin::RadioIrq),
                    provider_expiry_observed: expired,
                    end_capture: None,
                })
            );
        }
    }

    #[test]
    fn retained_capture_does_not_leak_into_wrong_context_or_stale_reports() {
        use core::future::Future;
        let model = WaitModel::new();
        let guard = CaptureGuard(FakeGuard::default());
        let epoch = OperationEpoch(1);
        guard.arm(epoch, 1550).unwrap();
        guard.0.0.lock().unwrap().capture = Some(candidate(epoch));
        {
            let mut control = model.control.borrow_mut();
            let _ = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::RadioIrq,
                |hook| {
                    hook();
                    Some(StopOutcome::Normal)
                },
            );
            let retained = control.finish(TxOutcome::PhysicalComplete);
            assert_eq!(
                retained.observations.unwrap().end_capture,
                Some(candidate(epoch))
            );
        }
        model.context.set(VectActive::from(11).unwrap());
        let (_, waker) = model_waker();
        {
            let mut future = core::pin::pin!(super::wait_with(
                model.endpoint(epoch),
                || model.clock(),
                || model.fallback()
            ));
            assert_eq!(
                future
                    .as_mut()
                    .poll(&mut core::task::Context::from_waker(&waker)),
                Poll::Ready(TxReport {
                    epoch,
                    outcome: TxOutcome::Fault(TxFault::WrongContext),
                    observations: None,
                })
            );
        }
        assert_eq!(model.clocks.get(), 0);
        assert_eq!(model.registrations.get(), 0);
        assert_eq!(model.arms.get(), 0);
        let mut control = model.control.borrow_mut();
        assert_eq!(
            control.report(epoch).observations.unwrap().end_capture,
            Some(candidate(epoch))
        );
        control.begin(200);
        control.finish(TxOutcome::Fault(TxFault::Cancelled));
        assert_eq!(control.report(epoch).observations, None);
        assert_eq!(control.report(OperationEpoch(0)).observations, None);
    }

    #[test]
    fn capture_requires_completion_hook_even_with_power_cycle_stop_result() {
        for stop in [
            None,
            Some(StopOutcome::Normal),
            Some(StopOutcome::PowerCycled),
        ] {
            let guard = CaptureGuard(FakeGuard::default());
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            guard.0.0.lock().unwrap().capture = Some(candidate(epoch));
            let observed = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::TaskService,
                |_| stop,
            );
            if let Some(stop) = observed {
                control.signal(epoch, stop);
            }
            assert_eq!(
                control.report(epoch).observations,
                Some(TxObservations {
                    completion: None,
                    provider_expiry_observed: false,
                    end_capture: None,
                })
            );
            assert_eq!(guard.0.0.lock().unwrap().calls, ["arm", "expired"]);
        }
    }

    #[test]
    fn first_capture_and_origin_are_immutable_including_absent_or_wrong_epoch_capture() {
        for first in [
            None,
            Some(candidate(OperationEpoch(1))),
            Some(candidate(OperationEpoch(0))),
        ] {
            let guard = CaptureGuard(FakeGuard::default());
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            guard.0.0.lock().unwrap().capture = first;
            let _ = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::RadioIrq,
                |hook| {
                    hook();
                    hook(); // Even repeated invocations within the same event.
                    Some(StopOutcome::Normal)
                },
            );
            guard.0.0.lock().unwrap().capture = Some(TxEndCapture {
                sampled_cc: [300; 2],
                ..candidate(epoch)
            });
            let _ = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::TaskService,
                |hook| {
                    hook();
                    Some(StopOutcome::Normal)
                },
            );
            assert_eq!(
                control.report(epoch).observations,
                Some(TxObservations {
                    completion: Some(TxCompletionOrigin::RadioIrq),
                    provider_expiry_observed: false,
                    end_capture: first.filter(|capture| capture.epoch == epoch),
                })
            );
            assert_eq!(
                guard.0.0.lock().unwrap().calls,
                [
                    "arm", "expired", "disarm", "expired", "capture", "disarm", "expired",
                    "expired", "disarm", "expired"
                ]
            );
        }
    }

    #[test]
    fn stop_results_without_completion_predicate_do_not_fabricate_origin() {
        for stop in [
            None,
            Some(StopOutcome::Normal),
            Some(StopOutcome::PowerCycled),
        ] {
            let mut control = Control::new();
            let epoch = control.begin(100);
            let observed = observe_tx_event(
                control.active.as_mut().unwrap(),
                None,
                TxCompletionOrigin::TaskService,
                |_| stop,
            );
            if let Some(stop) = observed {
                control.signal(epoch, stop);
            }
            assert_eq!(control.report(epoch).observations.unwrap().completion, None);
        }
    }

    #[test]
    fn completion_observation_does_not_erase_reset_signal_precedence() {
        let mut control = Control::new();
        let epoch = control.begin(100);
        let stop = observe_tx_event(
            control.active.as_mut().unwrap(),
            None,
            TxCompletionOrigin::RadioIrq,
            |hook| {
                hook();
                Some(StopOutcome::PowerCycled)
            },
        )
        .unwrap();
        control.signal(epoch, stop);
        control.signal(epoch, StopOutcome::Normal);
        let outcome = control.evaluate(101);
        assert_eq!(outcome, TxOutcome::Fault(TxFault::PowerCycled));
        assert_eq!(
            control.finish(outcome).observations.unwrap().completion,
            Some(TxCompletionOrigin::RadioIrq)
        );
    }

    #[test]
    fn production_observation_dispatch_and_report_gate_remain_scoped() {
        // Source wiring checks complement the executed helper models, not MMIO.
        let source = production_source();
        assert_eq!(source.matches(".poll_tx_event(").count(), 2);
        assert!(source.contains("self.poll_tx_event(TxCompletionOrigin::TaskService)"));
        assert!(source.contains("inner.poll_tx_event(TxCompletionOrigin::RadioIrq)"));
        let poll = source
            .split("fn poll_epoch(")
            .nth(1)
            .unwrap()
            .split("pub fn check_tx_outcome")
            .next()
            .unwrap();
        assert!(poll.find("task_access(||").unwrap() < poll.find("IrqMask::<T>::new()").unwrap());
        assert!(poll.find("IrqMask::<T>::new()").unwrap() < poll.find("self.inner.get()").unwrap());
        assert!(poll.contains("inner.control.matches(epoch) && inner.service(now_us).is_some()"));
        assert!(poll.contains("inner.control.report(epoch)"));
        assert!(poll.contains("observations: None"));
        let event = source
            .split("fn observe_tx_event(")
            .nth(1)
            .unwrap()
            .split("struct Control")
            .next()
            .unwrap();
        assert!(
            event.find("let stop = event(").unwrap()
                < event
                    .find("flight.completion.get_or_insert(origin)")
                    .unwrap()
        );
        assert!(event.contains("if completed"));
        assert!(
            event
                .find("expired |= disarm_completion(provider, flight.epoch)")
                .unwrap()
                < event.find("guard.capture_end(flight.epoch)").unwrap()
        );
    }

    #[test]
    fn event_and_post_restore_queries_latch_only_observed_expiry_in_order() {
        // True at initial sample, at completion/disarm, or only after restoration.
        for samples in [
            [true, false, false],
            [false, true, false],
            [false, false, true],
            [false, false, false],
        ] {
            let guard = FakeGuard::default();
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            guard.0.lock().unwrap().expiry_samples.extend(samples);
            let stop = observe_tx_event(
                control.active.as_mut().unwrap(),
                Some(&guard),
                TxCompletionOrigin::RadioIrq,
                |hook| {
                    hook();
                    guard.0.lock().unwrap().calls.push("stop_release");
                    Some(StopOutcome::Normal)
                },
            )
            .unwrap();
            control.signal(epoch, stop);
            disarm_before_restore(Some(&guard), || {
                guard.0.lock().unwrap().calls.push("restore")
            });
            control.active.as_mut().unwrap().query_expiry(Some(&guard));
            let outcome = control.evaluate(101);
            let report = control.finish(outcome);
            assert_eq!(
                report.observations.unwrap().provider_expiry_observed,
                samples.contains(&true)
            );
            assert_eq!(
                outcome,
                if samples.contains(&true) {
                    TxOutcome::Fault(TxFault::Deadline)
                } else {
                    TxOutcome::PhysicalComplete
                }
            );
            assert_eq!(
                guard.0.lock().unwrap().calls,
                [
                    "arm",
                    "expired",
                    "disarm",
                    "expired",
                    "stop_release",
                    "disarm",
                    "restore",
                    "expired"
                ]
            );
        }
    }

    #[test]
    fn cancellation_queries_keep_short_circuit_and_disarm_expiry_evidence() {
        for (initial, latched, at_disarm) in [
            (true, false, false),
            (false, true, false),
            (false, false, true),
            (false, false, false),
        ] {
            let guard = FakeGuard::default();
            let mut control = Control::new();
            let epoch = control.begin(100);
            guard.arm(epoch, 1550).unwrap();
            {
                let mut state = guard.0.lock().unwrap();
                state.expired = initial;
                state.expire_on_disarm = at_disarm;
            }
            let flight = control.active.as_mut().unwrap();
            flight.guard_expired = latched;
            let expired = flight.query_expiry(Some(&guard)) || flight.guard_expired;
            restore_current(true, Some(&guard), || {
                guard.0.lock().unwrap().calls.push("restore")
            })
            .unwrap();
            let expired = expired || flight.query_expiry(Some(&guard));
            let report = control.finish(TxOutcome::Fault(if expired {
                TxFault::Deadline
            } else {
                TxFault::Cancelled
            }));
            assert_eq!(
                report.observations,
                Some(TxObservations {
                    completion: None,
                    provider_expiry_observed: initial || latched || at_disarm,
                    end_capture: None,
                })
            );
            let mut calls = std::vec!["arm", "expired", "disarm", "restore"];
            if !initial && !latched {
                calls.push("expired");
            }
            assert_eq!(guard.0.lock().unwrap().calls, calls);
        }
    }

    #[test]
    fn retained_old_report_and_unknown_epoch_do_not_leak_new_flight_evidence() {
        let mut control = Control::new();
        let old = control.begin(100);
        let _ = observe_tx_event(
            control.active.as_mut().unwrap(),
            None,
            TxCompletionOrigin::RadioIrq,
            |hook| {
                hook();
                Some(StopOutcome::Normal)
            },
        );
        let old_report = control.finish(TxOutcome::Fault(TxFault::Cancelled));
        let current = control.begin(200);
        control.active.as_mut().unwrap().guard_expired = true;
        assert_eq!(control.report(old), old_report);
        assert_eq!(control.report(OperationEpoch(0)).observations, None);
        assert_eq!(
            control.report(current).observations,
            Some(TxObservations {
                completion: None,
                provider_expiry_observed: true,
                end_capture: None,
            })
        );
        assert_eq!(control.clock_us, 200);
        control.finish(TxOutcome::Fault(TxFault::Deadline));
        assert_eq!(control.report(old).observations, None);
    }

    #[test]
    fn wait_propagates_full_retained_report_without_servicing_new_flight() {
        use core::future::Future;
        let model = WaitModel::new();
        let old = OperationEpoch(1);
        model.irq(old, StopOutcome::Normal);
        model
            .control
            .borrow_mut()
            .active
            .as_mut()
            .unwrap()
            .guard_expired = true;
        let retained = model
            .control
            .borrow_mut()
            .finish(TxOutcome::Fault(TxFault::Deadline));
        let current = model.control.borrow_mut().begin(200);
        model.now.set(10_000); // Would time out the newer flight if serviced.
        let (_, waker) = model_waker();
        let mut future = core::pin::pin!(super::wait_with(
            model.endpoint(old),
            || model.clock(),
            || model.fallback()
        ));
        assert_eq!(
            future
                .as_mut()
                .poll(&mut core::task::Context::from_waker(&waker)),
            Poll::Ready(retained)
        );
        assert!(model.control.borrow().matches(current));
        assert_eq!(model.control.borrow().clock_us, 200);
        assert_eq!(model.cancels.get(), 0);
    }

    #[test]
    fn provider_expiry_without_physical_completion_is_a_deadline_fault() {
        let guard = FakeGuard::default();
        let mut control = Control::new();
        let epoch = control.begin(100);
        guard.arm(epoch, 1550).unwrap();
        guard.on_interrupt();
        control.active.as_mut().unwrap().guard_expired = guard_expired(Some(&guard), epoch);
        assert_eq!(control.evaluate(101), TxOutcome::Fault(TxFault::Deadline));
        assert_eq!(control.active.as_ref().unwrap().signal, None);
        assert_eq!(
            control
                .finish(TxOutcome::Fault(TxFault::Deadline))
                .observations,
            Some(TxObservations {
                completion: None,
                provider_expiry_observed: true,
                end_capture: None,
            })
        );
    }

    #[test]
    fn cancellation_and_pending_drop_model_disarm_but_stale_epochs_do_not() {
        // Both real Esb::cancel and SendTicket::drop reach restore_current.
        // This executes that helper, NOT physical facade/ticket Drop or MMIO.
        for via_pending_drop in [false, true] {
            let guard = FakeGuard::default();
            let mut control = Control::new();
            let old = control.begin(10);
            control.finish(TxOutcome::Fault(TxFault::Cancelled));
            let current = control.begin(20);
            guard.arm(current, 1470).unwrap();
            assert_eq!(
                restore_current::<()>(control.matches(old), Some(&guard), || {
                    panic!("stale restore")
                }),
                None
            );
            assert_eq!(guard.0.lock().unwrap().calls, ["arm"]);
            let finished = ticket_finished(control.outcome(current));
            if !via_pending_drop || !finished {
                restore_current(control.matches(current), Some(&guard), || {
                    let mut state = guard.0.lock().unwrap();
                    assert!(!state.routed);
                    state.calls.push("restore");
                })
                .unwrap();
                control.finish(TxOutcome::Fault(TxFault::Cancelled));
            }
            assert_eq!(guard.0.lock().unwrap().calls, ["arm", "disarm", "restore"]);
        }
    }

    #[test]
    fn unguarded_helpers_preserve_legacy_launch_completion_and_timeout_policy() {
        let mut control = Control::new();
        let epoch = control.begin(100);
        assert_eq!(
            arm_before_launch(None, epoch, 1550, |armed| {
                assert!(!armed.0);
                7
            }),
            Ok(7)
        );
        assert!(!disarm_completion(None, epoch));
        control.signal(epoch, StopOutcome::Normal);
        assert_eq!(control.evaluate(101), TxOutcome::PhysicalComplete);
        assert_eq!(control.evaluate(1550), TxOutcome::Fault(TxFault::TimedOut));
        assert_eq!(disarm_before_restore(None, || 8), 8);
    }

    #[test]
    fn completion_and_all_restoration_models_disable_the_route_first() {
        let guard = FakeGuard::default();
        let epoch = OperationEpoch(1);
        guard.arm(epoch, 1550).unwrap();
        assert!(!disarm_completion(Some(&guard), epoch));
        assert!(!guard.0.lock().unwrap().routed);
        for reason in [
            "timeout",
            "launch_error",
            "power_cycle",
            "idle_recovery",
            "dormant_config",
        ] {
            guard.arm(epoch, 1550).unwrap();
            disarm_before_restore(Some(&guard), || {
                let mut state = guard.0.lock().unwrap();
                assert!(!state.routed, "{reason}");
                state.calls.push(reason);
            });
        }
        // Names above model branches; production call sites are source-checked.
    }

    #[test]
    fn source_static_provider_and_primask_ordering_are_explicit() {
        // Source checks only, not target compilation or interrupt timing proof.
        let source = production_source();
        assert!(source.contains("pub unsafe trait TxDeadlineGuard: Sync"));
        assert!(source.contains("deadline: Option<&'static dyn TxDeadlineGuard>"));
        assert!(source.contains("deadline: &'static dyn TxDeadlineGuard"));
        assert!(source.contains("pub unsafe fn on_deadline_interrupt(&'static self)"));
        assert!(
            source.contains("Self::new_inner(timer, radio, pool, config, addresses, None, None)")
        );
        assert!(
            source.contains(
                "Self::new_inner(timer, radio, pool, config, addresses, Some(clock), None)"
            )
        );
        let begin = source
            .split("pub fn begin_send_no_ack_to_until")
            .nth(1)
            .unwrap()
            .split("pub fn service")
            .next()
            .unwrap();
        let mask = begin.find("let _guard = IrqMask::<T>::new()").unwrap();
        assert!(
            begin
                .find("if until_us <= now_us || until_us > maximum")
                .unwrap()
                < mask
        );
        let borrow = begin.find("&mut *self.inner.get()").unwrap();
        let prepare = begin
            .find("inner.control.begin_until(now_us, until_us)")
            .unwrap();
        let arm = begin.find("arm_before_launch(").unwrap();
        let launch = begin
            .find("driver.begin_exclusive_no_ack_with_launch(")
            .unwrap();
        assert!(mask < borrow && borrow < prepare && prepare < arm && arm < launch);
        let mask_source = include_str!("isr.rs");
        let mask_source = mask_source
            .split("impl<T: TimerInstance> IrqMask<T>")
            .nth(1)
            .unwrap();
        assert!(
            mask_source.find("cortex_m::interrupt::disable()").unwrap()
                < mask_source.find("NVIC::mask").unwrap()
        );
        let state_source = include_str!("state_machine.rs");
        let complete = state_source.split("StatePtx::TxNoAck => {").nth(1).unwrap();
        assert!(
            complete.find("before_completion()").unwrap()
                < complete.find("self.radio.stop()").unwrap()
        );
        assert!(
            complete.find("self.radio.stop()").unwrap()
                < complete.find("self.release_tx(pool)").unwrap()
        );
        assert!(source.contains("self.esb.cancel(self.epoch);"));
        let cancel = source
            .split("pub fn cancel(&self, epoch:")
            .nth(1)
            .unwrap()
            .split("pub fn configure_idle")
            .next()
            .unwrap();
        assert!(
            cancel.find("inner.control.matches(epoch)").unwrap()
                < cancel
                    .find("restore_current(current, inner.deadline")
                    .unwrap()
        );
        let configure = source.split("pub fn configure_idle").nth(1).unwrap();
        assert!(configure.contains("disarm_before_restore(inner.deadline, || inner.quiesce())"));
        let start = source.split("fn recv_start(&mut self)").nth(1).unwrap();
        assert!(
            start.find("disarm_before_restore(deadline").unwrap()
                < start.find(".start_exclusive_rx()").unwrap()
        );
        let irq = source
            .split("pub unsafe fn on_deadline_interrupt")
            .nth(1)
            .unwrap();
        assert!(irq.contains("provider.on_interrupt();"));
        assert!(!irq.contains("NVIC::unmask"));
        let restore = source.split("fn restore_prx_disarmed").nth(1).unwrap();
        assert!(
            restore.find("self.quiesce()").unwrap() < restore.find("prepare_session(").unwrap()
        );
        assert!(
            restore.find("prepare_session(").unwrap() < restore.find("self.take_parts()").unwrap()
        );
    }

    #[test]
    fn receive_api_source_uses_shared_policy_and_task_gate() {
        // Source assertions supplement helper tests; no MMIO facade is fabricated.
        let source = production_source();
        for signature in [
            "pub fn recv_start(&self)",
            "pub fn recv_stop(&self)",
            "pub fn receiver_epoch(&self)",
        ] {
            let method = source
                .split(signature)
                .nth(1)
                .unwrap()
                .split("\n    }")
                .next()
                .unwrap();
            assert!(
                method.find("task_access(||").unwrap()
                    < method.find("IrqMask::<T>::new()").unwrap()
            );
            assert!(
                method.find("IrqMask::<T>::new()").unwrap()
                    < method.find("self.inner.get()").unwrap()
            );
        }
        assert!(source.contains("pub unsafe fn activate_stopped(&'static self)"));
        assert!(source.contains("self.activate_with_receive(false)"));
        assert!(source.contains("self.activate_with_receive(true)"));
        let activation = source
            .split("unsafe fn activate_with_receive")
            .nth(1)
            .unwrap()
            .split("pub fn recv_start")
            .next()
            .unwrap();
        assert!(
            activation.find("inner.activated = true").unwrap()
                < activation.find("if request_rx").unwrap()
        );
        assert!(activation.contains("if inner.receiver.overflowed()"));
        assert!(activation.contains("guard.enable_on_drop()"));
        assert!(!activation.contains("start_exclusive_rx()"));
        assert!(source.contains("self.receive_control.start("));
        let stop = source
            .split("fn recv_stop(&mut self)")
            .nth(1)
            .unwrap()
            .split("fn quiesce")
            .next()
            .unwrap();
        assert!(stop.contains(".receive_control"));
        assert!(stop.contains(".stop(self.control.active.is_some(),"));
        assert!(
            stop.find("disarm_before_restore(deadline").unwrap()
                < stop.find("driver.quiesce()").unwrap()
        );
        let policy = include_str!("rx_metadata.rs")
            .split("#[cfg(test)]")
            .next()
            .unwrap();
        let stop_policy = policy.split("pub(crate) fn stop(").nth(1).unwrap();
        assert!(
            stop_policy.find("if tx_active").unwrap()
                < stop_policy.find("self.requested = false").unwrap()
        );
        assert!(
            stop_policy.find("self.requested = false").unwrap()
                < stop_policy.find("cleanup()").unwrap()
        );
        assert!(source.contains("inner.receive_control.epoch(&inner.receiver)"));
        let restore = source
            .split("fn restore_prx_disarmed")
            .nth(1)
            .unwrap()
            .split("fn poll_tx_event")
            .next()
            .unwrap();
        assert!(restore.contains("self.receive_control.prepare_session("));
        assert!(restore.contains("&mut self.receiver,"));
        assert!(restore.contains("self.config.channel,"));
        assert!(restore.contains("self.activated,"));
        assert!(restore.contains("if self.receive_control.wanted(self.activated)"));
        assert!(
            restore.find("prepare_session(").unwrap() < restore.find("self.take_parts()").unwrap()
        );
        let service = source
            .split("fn service(&mut self")
            .nth(1)
            .unwrap()
            .split("/// A single static")
            .next()
            .unwrap();
        assert!(
            service
                .contains("self.receive_control.wanted(self.activated) && driver.receiver_idle()")
        );
        assert!(service.contains("self.recv_start()"));
        assert!(source.contains("disarm_before_restore(deadline, ||"));
    }

    #[test]
    fn receive_controls_reject_exception_before_helper_or_guard() {
        use crate::config::Channel;
        use crate::rx_metadata::{ReceiveControl, SharedReceiverContext};

        let control = RefCell::new(ReceiveControl::new());
        let receiver = RefCell::new(SharedReceiverContext::new(None));
        let _control_loan = control.borrow_mut();
        let _receiver_loan = receiver.borrow_mut();
        for vector in 1..=u8::MAX {
            let Some(active) = VectActive::from(vector) else {
                continue;
            };
            assert_eq!(
                with_thread_context(active, || {
                    control.borrow_mut().start(
                        &mut receiver.borrow_mut(),
                        Channel(7),
                        true,
                        false,
                        |_| panic!("exception created guard/start"),
                    )
                }),
                Err(TxFault::WrongContext)
            );
            assert_eq!(
                with_thread_context(active, || {
                    control
                        .borrow_mut()
                        .stop(false, || panic!("exception disarmed"))
                }),
                Err(TxFault::WrongContext)
            );
            assert_eq!(
                with_thread_context(active, || { control.borrow().epoch(&receiver.borrow()) }),
                Err(TxFault::WrongContext)
            );
        }
    }

    #[test]
    fn every_exception_is_rejected_before_guard_state_or_pool_access() {
        // Model an interrupted task holding the exclusive state/channel borrow.
        // The production safe entrypoints all use this same closure gate; only
        // task_access performs MMIO, and that wrapper is not executed on host.
        let state = RefCell::new(Control::new());
        let borrowed = state.borrow_mut();
        let guard_creations = Cell::new(0);
        for vector in 1..=u8::MAX {
            let Some(active) = VectActive::from(vector) else {
                continue;
            };
            assert_eq!(
                with_thread_context(active, || {
                    guard_creations.set(guard_creations.get() + 1);
                    state.borrow_mut().begin(100)
                }),
                Err(TxFault::WrongContext),
                "vector {vector}"
            );
        }
        assert_eq!(guard_creations.get(), 0);
        assert!(borrowed.active.is_none());
        assert!(borrowed.last.is_none());
        drop(borrowed);
        assert_eq!(
            with_thread_context(VectActive::ThreadMode, || state.borrow_mut().begin(100)),
            Ok(OperationEpoch(1))
        );
    }

    #[test]
    fn receive_gate_is_checked_again_on_each_poll() {
        let queue_polls = Cell::new(0);
        let poll_queue = || {
            queue_polls.set(queue_polls.get() + 1);
            Poll::<usize>::Pending
        };
        assert_eq!(
            with_thread_context(VectActive::ThreadMode, poll_queue),
            Ok(Poll::Pending)
        );
        for vector in 1..=u8::MAX {
            if let Some(active) = VectActive::from(vector) {
                assert_eq!(
                    with_thread_context(active, poll_queue),
                    Err(TxFault::WrongContext)
                );
            }
        }
        assert_eq!(queue_polls.get(), 1);
        assert_eq!(
            with_thread_context(VectActive::ThreadMode, poll_queue),
            Ok(Poll::Pending)
        );
        assert_eq!(queue_polls.get(), 2);
    }

    #[test]
    fn wrong_context_diagnostic_does_not_read_or_change_sticky_fault() {
        let sticky = RefCell::new(Some(TxFault::PowerCycled));
        let borrowed = sticky.borrow_mut();
        let reads = Cell::new(0);
        let reported = with_thread_context(VectActive::from(2).expect("NMI"), || {
            reads.set(reads.get() + 1);
            *sticky.borrow()
        })
        .unwrap_or(Some(TxFault::WrongContext));
        assert_eq!(reported, Some(TxFault::WrongContext));
        assert_eq!(reads.get(), 0);
        assert_eq!(*borrowed, Some(TxFault::PowerCycled));
        drop(borrowed);
        assert_eq!(
            with_thread_context(VectActive::ThreadMode, || *sticky.borrow()),
            Ok(Some(TxFault::PowerCycled))
        );
    }

    #[test]
    fn wrong_context_is_nonterminal_and_leaves_pending_tx_cancellable() {
        let mut control = Control::new();
        let epoch = control.begin(100);
        let active = VectActive::from(2).expect("NMI");
        let outcome = with_thread_context(active, || {
            control.finish(TxOutcome::Fault(TxFault::Cancelled)).outcome
        })
        .unwrap_or_else(TxOutcome::Fault);
        assert_eq!(outcome, TxOutcome::Fault(TxFault::WrongContext));
        assert!(!ticket_finished(outcome));
        assert!(!ticket_finished(TxOutcome::Pending));
        assert!(control.matches(epoch));
        assert!(control.last.is_none());
        let cancelled = with_thread_context(VectActive::ThreadMode, || {
            control.finish(TxOutcome::Fault(TxFault::Cancelled))
        })
        .expect("task cancellation");
        assert!(ticket_finished(cancelled.outcome));
        assert!(ticket_finished(TxOutcome::PhysicalComplete));
        assert_eq!(control.outcome(epoch), cancelled.outcome);
    }

    #[test]
    fn idle_send_signal_and_service_publication_are_distinct() {
        let mut control = Control::new();
        let epoch = control.begin(100);
        assert_eq!(control.can_begin(101), Err(Error::Busy));
        assert_eq!(control.evaluate(101), TxOutcome::Pending);
        control.signal(epoch, StopOutcome::Normal);
        assert_eq!(control.outcome(epoch), TxOutcome::Pending);
        assert_eq!(control.evaluate(102), TxOutcome::PhysicalComplete);
        let report = control.finish(TxOutcome::PhysicalComplete);
        assert_eq!(report.epoch, epoch);
        assert_eq!(control.outcome(epoch), TxOutcome::PhysicalComplete);
        assert_eq!(control.can_begin(103), Ok(()));
    }

    #[test]
    fn deadline_is_exclusive_and_late_physical_signal_is_not_success() {
        for signal in [None, Some(StopOutcome::Normal)] {
            let mut control = Control::new();
            let epoch = control.begin(0);
            assert_eq!(control.evaluate(SEND_DEADLINE_US - 1), TxOutcome::Pending);
            if let Some(stop) = signal {
                control.signal(epoch, stop);
            }
            assert_eq!(
                control.evaluate(SEND_DEADLINE_US),
                TxOutcome::Fault(TxFault::TimedOut)
            );
        }
    }

    #[test]
    fn cancelled_or_completed_epochs_cannot_complete_or_cancel_new_send() {
        for terminal in [
            TxOutcome::PhysicalComplete,
            TxOutcome::Fault(TxFault::Cancelled),
            TxOutcome::Fault(TxFault::TimedOut),
        ] {
            let mut control = Control::new();
            let old = control.begin(10);
            control.finish(terminal);
            let current = control.begin(20);
            assert_ne!(old, current);
            assert!(!control.matches(old));
            control.signal(old, StopOutcome::Normal);
            assert_eq!(control.evaluate(21), TxOutcome::Pending);
            assert_eq!(control.outcome(current), TxOutcome::Pending);
            control.signal(current, StopOutcome::Normal);
            control.finish(TxOutcome::PhysicalComplete);
            assert_eq!(control.outcome(old), TxOutcome::Fault(TxFault::StaleEpoch));
        }
    }

    #[test]
    fn reset_and_clock_faults_never_become_physical_success() {
        let mut control = Control::new();
        let epoch = control.begin(100);
        control.signal(epoch, StopOutcome::PowerCycled);
        control.signal(epoch, StopOutcome::Normal);
        assert_eq!(
            control.evaluate(101),
            TxOutcome::Fault(TxFault::PowerCycled)
        );
        control.finish(TxOutcome::Fault(TxFault::PowerCycled));
        let next = control.begin(200);
        control.signal(next, StopOutcome::Normal);
        assert_eq!(
            control.evaluate(199),
            TxOutcome::Fault(TxFault::ClockRegressed)
        );
    }

    #[test]
    fn epochs_and_deadline_clock_never_wrap_or_accept_unknown_generation() {
        let mut control = Control::new();
        assert_eq!(
            control.outcome(OperationEpoch(0)),
            TxOutcome::Fault(TxFault::StaleEpoch)
        );
        assert_eq!(control.can_begin(u64::MAX), Err(Error::InvalidParam));
        control.sequence = u64::MAX;
        assert_eq!(control.can_begin(0), Err(Error::NotReady));
        control.sequence = 0;
        control.clock_us = 10;
        assert_eq!(control.can_begin(9), Err(Error::InvalidParam));
    }

    #[test]
    fn shared_pool_preserves_received_handle_during_software_tx_claim() {
        // Software pool transitions only: no RADIO/TIMER registers, consuming
        // driver conversions, actual DMA, or ticket Drop are executed here.
        let pool = PacketPool::<2, 16>::new();
        assert!(pool.rx_to_dma(0));
        unsafe {
            pool.header_mut(0).length = 3;
            pool.buf_mut(0)[4..7].copy_from_slice(&[10, 20, 30]);
        }
        pool.rx_complete(0, None);
        let rx = ReceivedPacket::from_pool(&pool, pool.try_receive_rx().expect("received packet"));
        let tx = pool.alloc_tx().expect("other free buffer");
        assert_ne!(tx, rx.index());
        unsafe {
            pool.buf_mut(tx)[4..7].copy_from_slice(&[40, 50, 60]);
        }
        pool.claim_allocated_tx(tx);
        assert_eq!(rx.payload(), &[10, 20, 30]);
        pool.release_tx(tx);
        assert!(pool.rx_to_dma(tx));
        assert_eq!(rx.payload(), &[10, 20, 30]);
        pool.release_tx(tx);
        drop(rx);
        assert!(pool.rx_to_dma(0));
        pool.release_tx(0);
    }
}
