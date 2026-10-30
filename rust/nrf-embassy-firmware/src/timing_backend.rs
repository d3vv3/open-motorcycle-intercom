//! Private TIMER0 owner. IRQ code touches only PAC compare flags and atomic
//! sidecars; engine, FIFO and WS sampler stay in thread mode. Clock extension
//! is independently shared with thread/maskable-IRQ callers under one CS.

use crate::shared_clock::{PrivateClockLease, SharedClock};
use core::cell::{Cell, RefCell};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use critical_section::CriticalSection;
use embassy_nrf::gpio::Pull;
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::interrupt::typelevel::{Binding, Handler, Interrupt, TIMER0};
use embassy_nrf::ppi::Ppi;
use embassy_nrf::timer::Timer;
use embassy_nrf::{Peri, pac, peripherals};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_sync::signal::Signal;
use nrf_embassy::tdma::{
    Callbacks, Clock, Tdma, TimerKind, Timers, WorkKind, WorkQueue, WsCapture,
};
use nrf_embassy::ws::WsSampler;
pub(crate) use nrf_embassy_firmware::actor_policy::ActorCallback;
use nrf_embassy_firmware::actor_policy::CallbackBuffer;
use nrf_embassy_firmware::clock::ClockFault;
use nrf_embassy_firmware::diagnostics::TimingSnapshot;
#[cfg(feature = "slot-entry-trace")]
use nrf_embassy_firmware::slot_entry_trace::{PHASES, RPC_BUCKETS, phase_bucket, rpc_bucket};
use nrf_embassy_firmware::timing::{
    AbsoluteArmRegisters, AlarmRecord, ArmResult, CompareRegisters, Expiry, Programmed,
    TIMER_KINDS, WorkFifo, arm_absolute_with, classify_expiry, deadline, drain_pending_alarms,
    mapped_absolute_deadline, next_epoch, program_compare, timer_index,
};
use nrf_embassy_firmware::timing_clock::{
    ClockMapping, ClockReading, PhysicalTime, TdmaClock, TimingTime, boot_mapping,
};

// No AtomicU64 on ARMv7-M. Every split-word access requires a CS token, including
// IRQ reads. Relaxed atomics are coherent because CS serializes all accesses.
struct AtomicWords {
    lo: AtomicU32,
    hi: AtomicU32,
}
impl AtomicWords {
    const fn new() -> Self {
        Self {
            lo: AtomicU32::new(0),
            hi: AtomicU32::new(0),
        }
    }
    fn read(&self, _cs: CriticalSection<'_>) -> u64 {
        u64::from(self.lo.load(Ordering::Relaxed))
            | (u64::from(self.hi.load(Ordering::Relaxed)) << 32)
    }
    fn write(&self, _cs: CriticalSection<'_>, value: u64) {
        self.lo.store(value as u32, Ordering::Relaxed);
        self.hi.store((value >> 32) as u32, Ordering::Relaxed);
    }
}

struct AtomicRecord {
    deadline: AtomicWords,
    epoch: AtomicWords,
}
impl AtomicRecord {
    const fn new() -> Self {
        Self {
            deadline: AtomicWords::new(),
            epoch: AtomicWords::new(),
        }
    }
    fn read(&self, cs: CriticalSection<'_>) -> AlarmRecord {
        AlarmRecord {
            deadline: self.deadline.read(cs) as i64,
            epoch: self.epoch.read(cs),
        }
    }
    fn write(&self, cs: CriticalSection<'_>, record: AlarmRecord) {
        self.deadline.write(cs, record.deadline as u64);
        self.epoch.write(cs, record.epoch);
    }
}

struct Sidecar {
    #[cfg(feature = "slot-entry-trace")]
    irq_stamp: AtomicU32,
    #[cfg(feature = "slot-entry-trace")]
    irq_epoch: AtomicWords,
    /// Actor phase / RPC active at IRQ time (packed phase | rpc << 8).
    #[cfg(feature = "slot-entry-trace")]
    irq_context: AtomicU32,
    armed: AtomicBool,
    pending: AtomicBool,
    active_record: AtomicRecord,
    pending_record: AtomicRecord,
}
impl Sidecar {
    const fn new() -> Self {
        Self {
            #[cfg(feature = "slot-entry-trace")]
            irq_stamp: AtomicU32::new(0),
            #[cfg(feature = "slot-entry-trace")]
            irq_epoch: AtomicWords::new(),
            #[cfg(feature = "slot-entry-trace")]
            irq_context: AtomicU32::new(0),
            armed: AtomicBool::new(false),
            pending: AtomicBool::new(false),
            active_record: AtomicRecord::new(),
            pending_record: AtomicRecord::new(),
        }
    }
    fn invalidate(&self, _cs: CriticalSection<'_>) {
        self.armed.store(false, Ordering::Relaxed);
        self.pending.store(false, Ordering::Relaxed);
    }
    fn publish(&self, cs: CriticalSection<'_>) {
        if self.armed.load(Ordering::Relaxed) {
            self.pending_record.write(cs, self.active_record.read(cs));
            self.pending.store(true, Ordering::Relaxed);
            WAKE.signal(());
        }
    }
}

static ALIVE: AtomicBool = AtomicBool::new(false);
static EPOCH: AtomicWords = AtomicWords::new(); // Never reset or reused, even across Drop.
static ALARMS: [Sidecar; 3] = [const { Sidecar::new() }; 3];
static WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
#[cfg(feature = "slot-entry-trace")]
static SLOT_ENTRY: Mutex<
    CriticalSectionRawMutex,
    RefCell<nrf_embassy_firmware::slot_entry_trace::Stats>,
> = Mutex::new(RefCell::new(
    nrf_embassy_firmware::slot_entry_trace::Stats::ZERO,
));
// Independent wrapping counters: slot at index 0, frame at index 1.
#[cfg(feature = "slot-entry-trace")]
static ALARM_PHASE: [[AtomicU32; PHASES]; 2] = [const { [const { AtomicU32::new(0) }; PHASES] }; 2];
#[cfg(feature = "slot-entry-trace")]
static ALARM_RPC: [[AtomicU32; RPC_BUCKETS]; 2] =
    [const { [const { AtomicU32::new(0) }; RPC_BUCKETS] }; 2];

#[cfg(feature = "slot-entry-trace")]
pub(crate) fn record_budget_reject(shortfall_us: u32, packet_len: usize) {
    SLOT_ENTRY.lock(|s| s.borrow_mut().budget_reject(shortfall_us, packet_len));
}
#[cfg(feature = "slot-entry-trace")]
pub(crate) fn slot_entry_snapshot() -> nrf_embassy_firmware::slot_entry_trace::Stats {
    let mut snapshot = SLOT_ENTRY.lock(|s| *s.borrow());
    // Counters are independent observations, not a simultaneous IRQ snapshot.
    snapshot.slot_alarm_phase = core::array::from_fn(|i| ALARM_PHASE[0][i].load(Ordering::Relaxed));
    snapshot.frame_alarm_phase =
        core::array::from_fn(|i| ALARM_PHASE[1][i].load(Ordering::Relaxed));
    snapshot.slot_alarm_rpc = core::array::from_fn(|i| ALARM_RPC[0][i].load(Ordering::Relaxed));
    snapshot.frame_alarm_rpc = core::array::from_fn(|i| ALARM_RPC[1][i].load(Ordering::Relaxed));
    snapshot
}

#[cfg(feature = "slot-entry-trace")]
pub(crate) fn record_slot_entry(slot: u8, now: u32, start: u32) {
    SLOT_ENTRY.lock(|s| {
        let mut s = s.borrow_mut();
        let h = if slot == 0 {
            &mut s.entry0
        } else {
            &mut s.entryn
        };
        h.elapsed(now, start);
    });
}
static LATEST: Mutex<CriticalSectionRawMutex, RefCell<Option<TimingSnapshot>>> =
    Mutex::new(RefCell::new(None));

pub fn latest() -> TimingSnapshot {
    LATEST.lock(|s| s.borrow().unwrap_or_default())
}

fn mask_clear(index: usize) {
    pac::TIMER0.intenclr().write(|w| w.set_compare(index, true));
    pac::TIMER0.events_compare(index).write_value(0);
    let _ = pac::TIMER0.events_compare(index).read();
}

pub struct AlarmHandler;
impl Handler<TIMER0> for AlarmHandler {
    // Safety: bind_interrupts invokes this only for TIMER0. No engine pointer,
    // borrowed peripheral owner or mutable task state is accessible here.
    unsafe fn on_interrupt() {
        #[cfg(feature = "slot-entry-trace")]
        let mut samples = [None; 3];
        critical_section::with(|cs| {
            for (index, sidecar) in ALARMS.iter().enumerate() {
                if pac::TIMER0.events_compare(index).read() != 0 {
                    mask_clear(index);
                    if ALIVE.load(Ordering::Relaxed) {
                        #[cfg(feature = "slot-entry-trace")]
                        if (index == timer_index(TimerKind::Frame)
                            || index == timer_index(TimerKind::Slot))
                            && sidecar.armed.load(Ordering::Relaxed)
                        {
                            use nrf_embassy_firmware::slot_entry_trace::{
                                ACTOR_PHASE, RPC_IN_FLIGHT,
                            };
                            let phase = ACTOR_PHASE.load(Ordering::Relaxed);
                            let rpc = RPC_IN_FLIGHT.load(Ordering::Relaxed);
                            // CC3 is the existing shared physical clock capture register.
                            pac::TIMER0.tasks_capture(3).write_value(1);
                            sidecar
                                .irq_stamp
                                .store(pac::TIMER0.cc(3).read(), Ordering::Relaxed);
                            sidecar
                                .irq_epoch
                                .write(cs, sidecar.active_record.read(cs).epoch);
                            sidecar
                                .irq_context
                                .store(u32::from(phase) | (u32::from(rpc) << 8), Ordering::Relaxed);
                            samples[index] = Some((phase, rpc));
                        }
                        sidecar.publish(cs);
                    }
                }
            }
        });
        // Trace CC3 capture and split-word epoch pairing need the IRQ CS.
        // Independent counters use atomic wrapping increments after unmasking.
        #[cfg(feature = "slot-entry-trace")]
        for (index, sample) in samples.into_iter().enumerate() {
            if let Some((phase, rpc)) = sample {
                let frame = usize::from(index == timer_index(TimerKind::Frame));
                ALARM_PHASE[frame][phase_bucket(phase)].fetch_add(1, Ordering::Relaxed);
                ALARM_RPC[frame][rpc_bucket(rpc)].fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

struct WsHardware {
    timer: Timer<'static>,
    _input: InputChannel<'static>,
    _count: Ppi<'static, peripherals::PPI_CH2, 1, 1>,
    sampler: WsSampler,
    frame: u32,
    count: u32,
}

impl WsHardware {
    fn new(
        timer: Peri<'static, peripherals::TIMER1>,
        channel: Peri<'static, peripherals::GPIOTE_CH1>,
        ppi: Peri<'static, peripherals::PPI_CH2>,
        pin: Peri<'static, peripherals::P0_02>,
    ) -> Self {
        let timer = Timer::new_counter(timer);
        let input = InputChannel::new(channel, pin, Pull::None, InputChannelPolarity::LoToHi);
        let mut count = Ppi::new_one_to_one(ppi, input.event_in(), timer.task_count());
        // Timer::new_counter clears once. No per-frame resets or sampler restart.
        timer.start();
        count.enable();
        let mut sampler = WsSampler::new();
        sampler.start();
        Self {
            timer,
            _input: input,
            _count: count,
            sampler,
            frame: 0,
            count: 0,
        }
    }
}

pub(crate) struct TimingPlatform {
    _timer: Peri<'static, peripherals::TIMER0>,
    lease: PrivateClockLease,
    last_now: i64,
    tdma_clock: TdmaClock,
    pending_clock_error: Option<ClockFault>,
    fault: u32,
    work: WorkFifo,
    ws: WsHardware,
    callbacks: CallbackBuffer,
    _not_sync: PhantomData<Cell<()>>,
}

// This view never escapes the owner's critical section. It exposes only the
// selected compare and sidecar, not counter reset/start or an engine reference.
struct CompareIo<'a, 'cs> {
    platform: &'a mut TimingPlatform,
    index: usize,
    cs: CriticalSection<'cs>,
}

impl CompareRegisters for CompareIo<'_, '_> {
    fn mask_interrupt(&mut self) {
        pac::TIMER0
            .intenclr()
            .write(|w| w.set_compare(self.index, true));
    }
    fn clear_pending(&mut self) {
        ALARMS[self.index].pending.store(false, Ordering::Relaxed);
    }
    fn clear_event(&mut self) {
        pac::TIMER0.events_compare(self.index).write_value(0);
    }
    fn read_event(&mut self) -> u32 {
        pac::TIMER0.events_compare(self.index).read()
    }
    fn write_compare(&mut self, low: u32) {
        pac::TIMER0.cc(self.index).write_value(low);
    }
    fn read_compare(&mut self) -> u32 {
        pac::TIMER0.cc(self.index).read()
    }
    fn read_clock(&mut self) -> Option<i64> {
        let now = self.platform.read_clock(self.cs);
        (self.platform.fault == 0).then_some(now)
    }
    fn publish_pending(&mut self, record: AlarmRecord) {
        debug_assert_eq!(ALARMS[self.index].active_record.read(self.cs), record);
        ALARMS[self.index].publish(self.cs);
    }
    fn enable_interrupt(&mut self) {
        pac::TIMER0
            .intenset()
            .write(|w| w.set_compare(self.index, true));
    }
}

impl AbsoluteArmRegisters for CompareIo<'_, '_> {
    fn reset_arm(&mut self) {
        mask_clear(self.index);
        ALARMS[self.index].invalidate(self.cs);
    }
    fn allocate_epoch(&mut self) -> Option<u64> {
        let epoch = next_epoch(EPOCH.read(self.cs))?;
        EPOCH.write(self.cs, epoch);
        Some(epoch)
    }
    fn activate_arm(&mut self, record: AlarmRecord) {
        ALARMS[self.index].active_record.write(self.cs, record);
        ALARMS[self.index].armed.store(true, Ordering::Relaxed);
    }
}

impl TimingPlatform {
    fn new(
        timer: Peri<'static, peripherals::TIMER0>,
        ws: WsHardware,
        _binding: impl Binding<TIMER0, AlarmHandler>,
        mapping: ClockMapping,
    ) -> Self {
        let lease = critical_section::with(|cs| {
            TIMER0::disable();
            ALIVE.store(false, Ordering::Relaxed);
            for index in 0..4 {
                mask_clear(index);
            }
            TIMER0::unpend();
            let lease = PrivateClockLease::initialize(cs, &timer);
            for sidecar in &ALARMS {
                sidecar.invalidate(cs);
            }
            WAKE.reset();
            lease
        });
        let platform = Self {
            _timer: timer,
            lease,
            last_now: 0,
            tdma_clock: TdmaClock::new(mapping),
            pending_clock_error: None,
            fault: 0,
            work: WorkFifo::default(),
            ws,
            callbacks: CallbackBuffer::default(),
            _not_sync: PhantomData,
        };
        critical_section::with(|_| {
            TIMER0::set_priority(Priority::P3); // Below RADIO/TIMER2/TIMER4 P2.
            TIMER0::unpend();
            ALIVE.store(true, Ordering::Relaxed);
            // Safety: unique TIMER0 token consumed, handler binding checked,
            // static sidecars initialized and published, all compares masked.
            unsafe {
                TIMER0::enable();
            }
        });
        platform
    }

    fn read_clock(&mut self, cs: CriticalSection<'_>) -> i64 {
        match self.lease.clock().read(cs) {
            Ok(now) => {
                self.last_now = now;
                now
            }
            Err(error) => {
                self.pending_clock_error.get_or_insert(error);
                self.fault = self.fault.max(1);
                // Core's infallible Clock trait retains the diagnostic last
                // value only; pending error suppresses arms and shuts it down.
                self.last_now
            }
        }
    }

    pub(crate) fn clock_mapping(&self) -> ClockMapping {
        self.tdma_clock.mapping()
    }

    #[cfg_attr(not(feature = "sync-trace"), allow(dead_code))] // Trace-only observation.
    pub(crate) fn core_clock_reading(&self) -> Option<ClockReading> {
        self.tdma_clock.last_reading()
    }

    pub(crate) fn timing_at_physical(&self, physical_us: i64) -> Option<i64> {
        let physical = PhysicalTime::new(u64::try_from(physical_us).ok()?);
        self.clock_mapping()
            .timing_at(physical)
            .ok()
            .map(TimingTime::value)
    }

    fn take_pending(&mut self) -> [Option<AlarmRecord>; 3] {
        critical_section::with(|cs| {
            core::array::from_fn(|index| {
                let sidecar = &ALARMS[index];
                sidecar
                    .pending
                    .swap(false, Ordering::Relaxed)
                    .then(|| sidecar.pending_record.read(cs))
            })
        })
    }

    fn accept(&mut self, index: usize, record: AlarmRecord) -> bool {
        critical_section::with(|cs| {
            let now = self.read_clock(cs);
            let sidecar = &ALARMS[index];
            let active = sidecar
                .armed
                .load(Ordering::Relaxed)
                .then(|| sidecar.active_record.read(cs));
            if self.fault != 0 {
                return false;
            }
            match classify_expiry(record, active, now) {
                Expiry::Stale => false, // Never touch a different/current arm.
                Expiry::Early => {
                    // IRQ masked this compare. Restore the SAME arm, including
                    // post-clear lateness recovery, without consuming its epoch.
                    self.program_compare(index, record, cs);
                    false
                }
                Expiry::Due => {
                    #[cfg(feature = "slot-entry-trace")]
                    if (index == timer_index(TimerKind::Frame)
                        || index == timer_index(TimerKind::Slot))
                        && sidecar.irq_epoch.read(cs) == record.epoch
                    {
                        let elapsed =
                            (now as u32).wrapping_sub(sidecar.irq_stamp.load(Ordering::Relaxed));
                        let context = sidecar.irq_context.load(Ordering::Relaxed);
                        SLOT_ENTRY.lock(|s| {
                            let mut s = s.borrow_mut();
                            s.irq_service.record(elapsed);
                            s.late_service(elapsed, context as u8, (context >> 8) as u8);
                        });
                        sidecar.irq_epoch.write(cs, 0);
                    }
                    sidecar.armed.store(false, Ordering::Relaxed);
                    true
                }
            }
        })
    }

    fn program_compare(&mut self, index: usize, record: AlarmRecord, cs: CriticalSection<'_>) {
        let mut io = CompareIo {
            platform: self,
            index,
            cs,
        };
        if program_compare(&mut io, record) == Programmed::ClockFault {
            ALARMS[index].invalidate(cs);
        }
    }

    fn publish_snapshot(&mut self, engine: &Tdma) {
        let now = critical_section::with(|cs| self.read_clock(cs));
        let stats = engine.stats();
        let snapshot = TimingSnapshot {
            clock_us: now as u64,
            frame: engine.frame_counter(),
            jitter_max_us: stats.callback_jitter_max_us,
            skipped: stats.skipped_frame_count,
            correction_pending_us: stats.correction_pending_us,
            fault: self.fault,
            ws_frame: self.ws.frame,
            ws_count: self.ws.count,
            ws: self.ws.sampler.stats(),
        };
        LATEST.lock(|s| *s.borrow_mut() = Some(snapshot));
    }
}

impl Drop for TimingPlatform {
    fn drop(&mut self) {
        critical_section::with(|cs| {
            self.lease.deactivate(cs); // Revoke all copies BEFORE raw shutdown.
            TIMER0::disable();
            for index in 0..4 {
                mask_clear(index);
            }
            for sidecar in &ALARMS {
                sidecar.invalidate(cs);
            }
            ALIVE.store(false, Ordering::Relaxed);
            TIMER0::unpend();
            pac::TIMER0.tasks_stop().write_value(1);
            WAKE.reset();
        });
    }
}

impl Clock for TimingPlatform {
    fn now_us(&mut self) -> i64 {
        critical_section::with(|cs| {
            let physical = self.read_clock(cs);
            self.tdma_clock.clear_reading();
            if self.fault != 0 {
                return self.tdma_clock.last_timing().value();
            }
            let timing = u64::try_from(physical)
                .ok()
                .and_then(|physical| self.tdma_clock.sample(PhysicalTime::new(physical)).ok());
            match timing {
                Some(timing) => timing.value(),
                None => {
                    self.fault = 4;
                    self.tdma_clock.last_timing().value()
                }
            }
        })
    }
    fn uptime_ms(&mut self) -> u32 {
        (critical_section::with(|cs| self.read_clock(cs)) as u64 / 1_000) as u32
    }
}
impl Timers for TimingPlatform {
    fn init_timer(&mut self, kind: TimerKind) {
        self.cancel_timer(kind);
    }
    fn arm_timer(&mut self, kind: TimerKind, delay_us: i64) {
        critical_section::with(|cs| {
            let index = timer_index(kind);
            let sidecar = &ALARMS[index];
            mask_clear(index);
            sidecar.invalidate(cs);
            let now = self.read_clock(cs);
            if self.fault != 0 {
                return;
            }
            let Some(deadline) = deadline(now, delay_us) else {
                self.fault = 2;
                return;
            };
            let Some(epoch) = next_epoch(EPOCH.read(cs)) else {
                self.fault = 3;
                return;
            };
            EPOCH.write(cs, epoch);
            let record = AlarmRecord { deadline, epoch };
            sidecar.active_record.write(cs, record);
            sidecar.armed.store(true, Ordering::Relaxed);
            self.program_compare(index, record, cs);
        });
    }
    fn arm_timer_at(&mut self, kind: TimerKind, target_us: i64, _delay_us: i64) {
        let mapping = self.clock_mapping();
        critical_section::with(|cs| {
            let result = {
                let mut io = CompareIo {
                    platform: self,
                    index: timer_index(kind),
                    cs,
                };
                arm_absolute_with(&mut io, target_us, |now, target| {
                    mapped_absolute_deadline(now, target, mapping)
                })
            };
            match result {
                ArmResult::InvalidDeadline => self.fault = 2,
                ArmResult::MappingFault => self.fault = 4,
                ArmResult::EpochExhausted => self.fault = 3,
                ArmResult::ClockFault | ArmResult::Programmed { .. } => {}
            }
        });
    }
    fn cancel_timer(&mut self, kind: TimerKind) {
        critical_section::with(|cs| {
            let index = timer_index(kind);
            mask_clear(index);
            ALARMS[index].invalidate(cs);
        });
    }
}
impl WorkQueue for TimingPlatform {
    fn init_work(&mut self, kind: WorkKind) {
        self.work.cancel(kind);
    }
    fn submit_work(&mut self, kind: WorkKind) -> i32 {
        self.work.submit(kind)
    }
    fn cancel_work(&mut self, kind: WorkKind) {
        self.work.cancel(kind);
    }
}
impl WsCapture for TimingPlatform {
    fn capture_ws(&mut self) -> Option<u32> {
        // Core calls this after timer arms, and owns the paired frame metadata.
        Some(critical_section::with(|_| self.ws.timer.cc(1).capture()))
    }
    fn sample_ws(&mut self, frame: u32, absolute_edges: u32) -> Option<i32> {
        self.ws.frame = frame;
        self.ws.count = absolute_edges;
        let result = self.ws.sampler.sample(absolute_edges, frame);
        result.valid.then_some(result.correction_us)
    }
}
impl Callbacks for TimingPlatform {
    fn slot_callback(&mut self, slot: u8, frame: u32) {
        self.push_callback(ActorCallback::Slot { slot, frame });
    }
    fn control_callback(&mut self, frame: u32) {
        self.push_callback(ActorCallback::Control { frame });
    }
}

impl TimingPlatform {
    fn push_callback(&mut self, callback: ActorCallback) {
        self.callbacks.push(callback);
    }

    pub(crate) fn assert_healthy(&mut self) {
        critical_section::with(|cs| self.read_clock(cs));
        if self.pending_clock_error.is_some() || self.fault != 0 {
            cortex_m::interrupt::disable();
            panic!("TDMA clock/arm fault");
        }
    }

    /// Execute checked windows synchronously, then move callbacks out so the
    /// actor releases every timing borrow before awaiting RF or Bridge IO.
    pub(crate) fn drain(&mut self, engine: &mut Tdma) -> [Option<ActorCallback>; 3] {
        self.assert_healthy();
        // Probe: physical clock at Frame accept, and after its timer_expired.
        #[cfg(feature = "slot-entry-trace")]
        let mut frame_probe: Option<(u32, u32)> = None;
        drain_pending_alarms(
            &mut (&mut *self, &mut *engine),
            |(platform, _)| platform.take_pending(),
            |(platform, engine), index, record| {
                if platform.accept(index, record) {
                    // Timer arms overwrite last_now; preserve the acceptance sample.
                    #[cfg(feature = "slot-entry-trace")]
                    let accepted = platform.last_now as u32;
                    engine
                        .timer_expired(TIMER_KINDS[index], *platform)
                        .expect("TDMA alarm arithmetic");
                    #[cfg(feature = "slot-entry-trace")]
                    if index == timer_index(TimerKind::Frame) {
                        frame_probe = Some((accepted, platform.physical_now()));
                    }
                }
                platform.assert_healthy();
            },
        );
        while let Some(kind) = self.work.pop() {
            engine
                .work_executed(kind, self)
                .expect("TDMA work arithmetic");
            self.assert_healthy();
        }
        self.publish_snapshot(engine);
        self.assert_healthy();
        #[cfg(feature = "slot-entry-trace")]
        if let Some((accepted, dispatched)) = frame_probe {
            let end = self.physical_now();
            SLOT_ENTRY.lock(|s| {
                let mut s = s.borrow_mut();
                s.frame_dispatch.elapsed(dispatched, accepted);
                s.frame_drain.elapsed(end, accepted);
            });
        }
        self.callbacks.take()
    }

    #[cfg(feature = "slot-entry-trace")]
    fn physical_now(&mut self) -> u32 {
        critical_section::with(|cs| self.read_clock(cs)) as u32
    }
}

/// Initialize in main before any Runtime/backend clock use or task first poll.
pub(crate) fn initialize(
    timer: Peri<'static, peripherals::TIMER0>,
    ws_timer: Peri<'static, peripherals::TIMER1>,
    ws_channel: Peri<'static, peripherals::GPIOTE_CH1>,
    ws_ppi: Peri<'static, peripherals::PPI_CH2>,
    ws_pin: Peri<'static, peripherals::P0_02>,
) -> (TimingPlatform, SharedClock) {
    initialize_with_mapping(timer, ws_timer, ws_channel, ws_ppi, ws_pin, boot_mapping())
}

/// The caller selects a fixed mapping when constructing a quiesced owner.
pub(crate) fn initialize_with_mapping(
    timer: Peri<'static, peripherals::TIMER0>,
    ws_timer: Peri<'static, peripherals::TIMER1>,
    ws_channel: Peri<'static, peripherals::GPIOTE_CH1>,
    ws_ppi: Peri<'static, peripherals::PPI_CH2>,
    ws_pin: Peri<'static, peripherals::P0_02>,
    mapping: ClockMapping,
) -> (TimingPlatform, SharedClock) {
    let ws = WsHardware::new(ws_timer, ws_channel, ws_ppi, ws_pin);
    let platform = TimingPlatform::new(timer, ws, crate::Irqs, mapping);
    let clock = platform.lease.clock();
    (platform, clock)
}

pub(crate) async fn wait() {
    WAKE.wait().await;
}

/// True when an alarm IRQ has published a callback the actor has not drained yet.
/// Cheap enough to poll at every actor phase boundary.
pub(crate) fn alarm_pending() -> bool {
    ALARMS.iter().any(|s| s.pending.load(Ordering::Relaxed))
}

/// Microseconds until the earliest armed alarm; None when nothing is armed or the
/// clock is unavailable (callers then treat the window as open).
pub(crate) fn next_alarm_in_us() -> Option<i64> {
    let now = crate::shared_clock::SharedClock::read_static().ok()?;
    critical_section::with(|cs| {
        ALARMS
            .iter()
            .filter(|s| s.armed.load(Ordering::Relaxed))
            .map(|s| s.active_record.read(cs).deadline - now)
            .min()
    })
}
