//! ESB ISR glue and async driver API.
//!
//! Provides the user-facing API for ESB PTX and PRX modes:
//! - `EsbPtx`: async PTX driver with `send().await`
//! - `EsbPrx`: async PRX driver with `receive().await`
//! - ISR handler methods called from `#[interrupt]` handlers
//!
//! The RADIO ISR re-entry guard and TIMER → RADIO pending pattern are adapted
//! from [esb-ng](https://github.com/jamesmunns/esb) (MIT OR Apache-2.0).
//! See `NOTICE.md` for attribution.
//!
//! # Single-ISR-context architecture (R9)
//!
//! All ESB state machine logic runs in the RADIO ISR. The TIMER ISR is
//! minimal: clear events, set flag, pend RADIO ISR.
//!
//! # Usage
//!
//! ```ignore
//! static ESB: StaticCell<EsbPtx<TIMER1>> = StaticCell::new();
//! let ptx = ESB.init(EsbPtx::new(timer, radio, &pool, &config, &addresses, 0)?);
//! // Bind vectors and set non-nesting IRQ priorities before activation.
//! unsafe { ptx.activate(); }
//!
//! #[embassy_nrf::pac::interrupt]
//! fn RADIO() { unsafe { ptx.on_radio_interrupt(); } }
//!
//! #[embassy_nrf::pac::interrupt]
//! fn TIMER1() { unsafe { ptx.on_timer_interrupt(); } }
//! ```

use core::cell::UnsafeCell;
#[allow(unused_imports)]
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::addresses::EsbAddresses;
use crate::config::EsbConfig;
use crate::error::Error;
use crate::esb::{DeadlineGuardError, TxLaunch};
use crate::header::EsbHeader;
use crate::payload::PacketPool;
use crate::radio::{EsbRadio, StopOutcome};
use crate::state_machine::{ExclusiveSendError, PrxStateMachine, PtxStateMachine};
use crate::suspend::EsbSavedState;
use crate::timer::{EsbTimer, TimerInstance};

/// Default pool size (number of packet buffers).
pub const DEFAULT_POOL_N: usize = 4;

/// Default buffer size per packet (4-byte header + 252-byte payload).
pub const DEFAULT_POOL_SIZE: usize = 256;

/// Unique owners moved only between quiescent, consuming role conversions.
pub(crate) struct DriverParts<T: TimerInstance, const N: usize, const SIZE: usize> {
    pub(crate) radio: EsbRadio,
    timer: EsbTimer<T>,
    radio_token: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
    pool: &'static PacketPool<N, SIZE>,
}

impl<T: TimerInstance, const N: usize, const SIZE: usize> DriverParts<T, N, SIZE> {
    pub(crate) fn prepare(
        &mut self,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        reset_duplicates: bool,
    ) {
        assert!(self.radio.is_quiescent());
        self.radio.init(config, addresses);
        if reset_duplicates {
            self.radio.reset_detection_state();
        }
        self.radio.clear_all_events();
        self.timer.clear_all_events();
        unpend_irqs::<T>();
    }
}

// ---- NVIC helpers ----

#[cfg(any(feature = "nrf52840", feature = "nrf52833", feature = "nrf52832"))]
fn unpend_radio_irq() {
    cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::RADIO);
}

pub(crate) struct IrqMask<T: TimerInstance> {
    radio_enabled: bool,
    timer_enabled: bool,
    interrupts_enabled: bool,
    _timer: core::marker::PhantomData<T>,
}

impl<T: TimerInstance> IrqMask<T> {
    pub(crate) fn new() -> Self {
        // PRIMASK also serializes safe task calls made from other IRQ contexts.
        let interrupts_enabled = cortex_m::register::primask::read().is_active();
        cortex_m::interrupt::disable();
        let radio_enabled = cortex_m::peripheral::NVIC::is_enabled(crate::pac::Interrupt::RADIO);
        let timer_enabled =
            cortex_m::peripheral::NVIC::is_enabled(crate::timer::timer_interrupt::<T>());
        cortex_m::peripheral::NVIC::mask(crate::pac::Interrupt::RADIO);
        cortex_m::peripheral::NVIC::mask(crate::timer::timer_interrupt::<T>());
        Self {
            radio_enabled,
            timer_enabled,
            interrupts_enabled,
            _timer: core::marker::PhantomData,
        }
    }

    fn keep_masked(mut self) {
        self.radio_enabled = false;
        self.timer_enabled = false;
    }

    pub(crate) fn enable_on_drop(&mut self) {
        self.radio_enabled = true;
        self.timer_enabled = true;
    }
}

impl<T: TimerInstance> Drop for IrqMask<T> {
    fn drop(&mut self) {
        // SAFETY: restore only the states saved before this non-awaiting guard.
        unsafe {
            if self.radio_enabled {
                cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::RADIO);
            }
            if self.timer_enabled {
                cortex_m::peripheral::NVIC::unmask(crate::timer::timer_interrupt::<T>());
            }
            if self.interrupts_enabled {
                cortex_m::interrupt::enable();
            }
        }
    }
}

fn with_irqs_masked<T: TimerInstance, R>(f: impl FnOnce() -> R) -> R {
    let _guard = IrqMask::<T>::new();
    f()
}

pub(crate) fn unpend_irqs<T: TimerInstance>() {
    unpend_radio_irq();
    cortex_m::peripheral::NVIC::unpend(crate::timer::timer_interrupt::<T>());
}

fn validate_pool<const N: usize, const SIZE: usize>(payload_length: u8) -> Result<(), Error> {
    if N < 2 || SIZE < EsbHeader::PAYLOAD_OFFSET + usize::from(payload_length) {
        return Err(Error::InvalidParam);
    }
    Ok(())
}

fn update_default_pipe(default_pipe: &mut u8, pipe: u8) {
    *default_pipe = pipe;
}

struct SuspendRequestGuard<'a> {
    requested: &'a AtomicBool,
}

impl<'a> SuspendRequestGuard<'a> {
    fn new(requested: &'a AtomicBool) -> Self {
        requested.store(true, Ordering::Release);
        Self { requested }
    }
}

impl Drop for SuspendRequestGuard<'_> {
    fn drop(&mut self) {
        self.requested.store(false, Ordering::Release);
    }
}

// ---- PTX Driver ----

/// Embassy async PTX (Primary Transmitter) driver.
///
/// Combines radio, timer, packet pool, and PTX state machine.
/// Place in a `static` via `static_cell`.
pub struct EsbPtx<
    T: TimerInstance,
    const N: usize = DEFAULT_POOL_N,
    const SIZE: usize = DEFAULT_POOL_SIZE,
> {
    sm: UnsafeCell<PtxStateMachine<T>>,
    _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
    pool: &'static PacketPool<N, SIZE>,
    /// Shared flag set by TIMER ISR, read/cleared by RADIO ISR.
    timer_flag: AtomicBool,
    /// Set by RADIO ISR when max retransmit attempts reached, cleared by app.
    max_attempts_flag: AtomicBool,
    /// Set by app to request ISR to stop after current transaction.
    suspend_requested: AtomicBool,
    /// Woken by ISR when it goes idle with suspend_requested set.
    suspend_signal: AtomicWaker,
    max_payload_len: usize,
}

// SAFETY: `EsbPtx` is intended to live in a `static`. The inner state machine
// is mutated from RADIO ISR context and from a small set of task-context
// methods. Task-context direct state access masks RADIO/TIMER and PRIMASK before touching
// `sm`; ISR entrypoints are documented as ISR-only and are not re-entrant on
// Cortex-M. The packet pool is `'static` and enforces buffer ownership with
// atomic state transitions.
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Send for EsbPtx<T, N, SIZE> {}
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Sync for EsbPtx<T, N, SIZE> {}

#[allow(dead_code)]
impl<T: TimerInstance, const N: usize, const SIZE: usize> EsbPtx<T, N, SIZE> {
    pub(crate) fn from_parts(
        parts: DriverParts<T, N, SIZE>,
        config: &EsbConfig,
        next_pid: u8,
    ) -> Self {
        Self {
            sm: UnsafeCell::new(PtxStateMachine::from_parts(
                parts.radio,
                parts.timer,
                config,
                next_pid,
            )),
            _radio: parts.radio_token,
            pool: parts.pool,
            timer_flag: AtomicBool::new(false),
            max_attempts_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: usize::from(config.payload_length),
        }
    }

    pub(crate) fn into_parts(self) -> (DriverParts<T, N, SIZE>, u8) {
        let (radio, timer, next_pid) = self.sm.into_inner().into_parts();
        (
            DriverParts {
                radio,
                timer,
                radio_token: self._radio,
                pool: self.pool,
            },
            next_pid,
        )
    }

    pub(crate) fn quiesce(&mut self) -> StopOutcome {
        let sm = self.sm.get_mut();
        let stop = sm.quiesce(self.pool);
        self.timer_flag.store(false, Ordering::Release);
        unpend_irqs::<T>();
        if sm.radio.take_stop_fault() {
            StopOutcome::PowerCycled
        } else {
            stop
        }
    }

    pub(crate) fn begin_exclusive_no_ack(&mut self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        self.begin_exclusive_no_ack_with_launch(pipe, payload, || Ok(TxLaunch::Cpu))
            .map_err(|error| match error {
                ExclusiveSendError::Driver(error) => error,
                ExclusiveSendError::Guard(_) => unreachable!("CPU launch cannot fail"),
            })
    }

    pub(crate) fn begin_exclusive_no_ack_with_launch(
        &mut self,
        pipe: u8,
        payload: &[u8],
        launch: impl FnOnce() -> Result<TxLaunch, DeadlineGuardError>,
    ) -> Result<(), ExclusiveSendError> {
        self.sm
            .get_mut()
            .start_exclusive_no_ack_with_launch(self.pool, pipe, payload, launch)
    }

    pub(crate) fn exclusive_radio_event_with_completion(
        &mut self,
        before_completion: impl FnOnce(),
    ) -> Option<StopOutcome> {
        let timer = self.timer_flag.swap(false, Ordering::AcqRel);
        let sm = self.sm.get_mut();
        sm.handle_radio_event_with_completion(self.pool, timer, true, before_completion);
        let completion = sm.take_no_ack_completion();
        if sm.radio.take_stop_fault() {
            Some(StopOutcome::PowerCycled)
        } else {
            completion
        }
    }

    pub(crate) fn exclusive_timer_event(&mut self) {
        self.timer_flag.store(true, Ordering::Release);
        self.sm.get_mut().handle_timer_event();
    }

    /// Create a new PTX driver.
    ///
    /// - `timer`: Embassy TIMER peripheral (consumed, prevents other access)
    /// - `_radio`: Embassy RADIO peripheral (consumed for ownership proof)
    /// - `pool`: Static packet pool
    /// - `config`: ESB configuration
    /// - `addresses`: ESB address configuration
    /// - `tx_pipe`: Pipe number for TX (typically 0)
    ///
    /// Validates configuration/pool before MMIO and returns with both IRQs
    /// masked and unpended. Place statically, bind IRQs, then activate.
    pub fn new(
        timer: embassy_nrf::Peri<'static, T>,
        _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        tx_pipe: u8,
    ) -> Result<Self, Error> {
        config.validate()?;
        validate_pool::<N, SIZE>(config.payload_length)?;
        if tx_pipe >= 8 {
            return Err(Error::InvalidParam);
        }
        let guard = IrqMask::<T>::new();
        unpend_irqs::<T>();
        let mut radio = EsbRadio::new(crate::pac::RADIO);
        radio.stop();
        radio.init(config, addresses);

        let esb_timer = EsbTimer::new(timer);
        let sm = PtxStateMachine::new(radio, esb_timer, config, tx_pipe);

        unpend_irqs::<T>();
        guard.keep_masked();

        Ok(Self {
            sm: UnsafeCell::new(sm),
            _radio,
            pool,
            timer_flag: AtomicBool::new(false),
            max_attempts_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: config.payload_length as usize,
        })
    }

    /// Create a new PTX driver for MPSL timeslot-managed mode (S6).
    ///
    /// Like [`new`](Self::new) but does **not** unmask the RADIO or TIMER ISRs —
    /// in timeslot mode the RADIO ISR vector belongs to MPSL (the app receives
    /// events via `SIGNAL_RADIO`), and the protocol timing comes from TIMER0
    /// owned by MPSL, not from `EsbTimer<T>`. RADIO is also left un-initialised
    /// here because each timeslot power-cycles it; `ts_start_tx` does the init.
    ///
    /// **NVIC note:** unlike `new()`, this does NOT mask/unmask RADIO or TIMER.
    /// MPSL owns the RADIO vector; the mpsl layer's TIMER0 callback drives
    /// retransmit/ACK-timeout timing via `timer_flag` passed to `ts_on_radio`.
    #[cfg(feature = "mpsl")]
    pub fn new_timeslot(
        timer: embassy_nrf::Peri<'static, T>,
        _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        tx_pipe: u8,
    ) -> Result<Self, Error> {
        config.validate()?;
        validate_pool::<N, SIZE>(config.payload_length)?;
        let guard = IrqMask::<T>::new();
        unpend_irqs::<T>();
        // SM keeps the config + a fresh radio instance; RADIO registers are
        // programmed on each slot by `ts_start_tx`.
        let radio = EsbRadio::new(crate::pac::RADIO);
        let esb_timer = EsbTimer::new(timer);
        let mut sm = PtxStateMachine::new(radio, esb_timer, config, tx_pipe);
        sm.set_addresses(addresses);
        sm.set_timeslot_managed(true);

        guard.keep_masked();
        Ok(Self {
            sm: UnsafeCell::new(sm),
            _radio,
            pool,
            timer_flag: AtomicBool::new(false),
            max_attempts_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: config.payload_length as usize,
        })
    }

    /// Called from the RADIO interrupt handler.
    ///
    /// Processes radio events and advances the PTX state machine.
    /// Unpends RADIO ISR to prevent spurious re-entry (esb-ng line 149).
    ///
    /// # Safety
    /// Call only from this driver's bound RADIO IRQ after activation. RADIO and
    /// TIMER handlers must not nest or run concurrently, and no task or handler
    /// may hold a state-machine, RADIO, or TIMER borrow across this call.
    pub unsafe fn on_radio_interrupt(&self) {
        let timer_flag = self.timer_flag.load(Ordering::Acquire);
        if timer_flag {
            self.timer_flag.store(false, Ordering::Release);
        }
        let suppress = self.suspend_requested.load(Ordering::Acquire);
        // SAFETY: ISR-only access — no concurrent ISR or app mutation of sm.
        let sm = unsafe { &mut *self.sm.get() };
        let event = sm.handle_radio_event(self.pool, timer_flag, suppress);
        if event == crate::state_machine::PtxEvent::MaxAttempts {
            self.max_attempts_flag.store(true, Ordering::Release);
        }

        if suppress && sm.state() == crate::state_machine::StatePtx::Idle {
            self.suspend_signal.wake();
        }

        // Clear any latched RADIO pending bit to prevent spurious re-entry
        // (esb-ng irq.rs line 149).
        #[cfg(any(feature = "nrf52840", feature = "nrf52833", feature = "nrf52832"))]
        cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::RADIO);
    }

    /// Called from the TIMER interrupt handler.
    ///
    /// Minimal: sets flag, then clears timer events and pends RADIO ISR (R9).
    /// Flag must be set BEFORE pending RADIO ISR to prevent race (esb-ng line 63).
    /// # Safety
    /// Call only from this driver's bound TIMER IRQ after activation. It must
    /// not nest with RADIO, reenter, or overlap any RADIO/TIMER/state borrow.
    pub unsafe fn on_timer_interrupt(&self) {
        self.timer_flag.store(true, Ordering::Release);
        // SAFETY: ISR-only access.
        let sm = unsafe { &*self.sm.get() };
        sm.handle_timer_event();
    }

    /// Unsupported ACK-requesting TX API; always returns InvalidParam.
    ///
    /// Use [`send_no_ack`](Self::send_no_ack) for the supported profile.
    pub async fn send(&self, payload: &[u8]) -> Result<(), Error> {
        self.send_to(0, payload).await
    }

    /// Unsupported ACK-requesting TX API; always returns InvalidParam.
    ///
    /// Use [`send_no_ack_to`](Self::send_no_ack_to); nothing is allocated/queued.
    pub async fn send_to(&self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        let _ = (pipe, payload);
        Err(Error::InvalidParam)
    }

    /// Queue a NoAck packet for transmission on the current default pipe.
    ///
    /// NoAck packets do not wait for acknowledgment. Use
    /// [`send_no_ack_to`](Self::send_no_ack_to) for multi-pipe firmware or
    /// when multiple tasks can enqueue packets.
    pub async fn send_no_ack(&self, payload: &[u8]) -> Result<(), Error> {
        let pipe = self.default_pipe();
        self.send_no_ack_to(pipe, payload).await
    }

    /// Queue a NoAck packet for transmission on a specific pipe.
    ///
    /// The pipe is stored in the packet metadata. Returns [`Error::TxFull`]
    /// when the packet pool is full and [`Error::InvalidParam`] for an invalid
    /// pipe, empty payload, or oversized payload.
    pub async fn send_no_ack_to(&self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        if pipe >= 8 {
            return Err(Error::InvalidParam);
        }
        if payload.is_empty() || payload.len() > self.max_payload_len {
            return Err(Error::InvalidParam);
        }
        let payload_offset = EsbHeader::PAYLOAD_OFFSET;
        if payload_offset + payload.len() > SIZE {
            return Err(Error::InvalidParam);
        }
        let idx = self.pool.alloc_tx().ok_or(Error::TxFull)?;

        let header = unsafe { self.pool.header_mut(idx) };
        header.pipe = pipe;
        header.length = payload.len() as u8;
        header.set_no_ack(true);

        let buf = unsafe { self.pool.buf_mut(idx) };
        buf[payload_offset..payload_offset + payload.len()].copy_from_slice(payload);

        self.pool.enqueue_tx(idx).await;
        // SAFETY: trigger_send only writes NVIC, no data race with ISR.
        with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.trigger_send());
        Ok(())
    }

    /// Try to receive an ACK payload (non-blocking).
    pub fn try_receive(&self) -> Option<ReceivedPacket<'_, N, SIZE>> {
        let idx = self.pool.try_receive_rx()?;
        Some(ReceivedPacket {
            pool: self.pool,
            idx,
        })
    }

    /// Receive an ACK payload (async).
    pub async fn receive(&self) -> ReceivedPacket<'_, N, SIZE> {
        let idx = self.pool.receive_rx().await;
        ReceivedPacket {
            pool: self.pool,
            idx,
        }
    }

    /// Set the default TX pipe for subsequent [`send_no_ack`](Self::send_no_ack) calls.
    ///
    /// This is only a convenience default for simple single-task firmware.
    /// Already queued packets keep their own pipe metadata. Prefer
    /// [`send_no_ack_to`](Self::send_no_ack_to)
    /// when multiple tasks or queued multi-pipe traffic are used.
    pub fn set_pipe(&self, pipe: u8) {
        debug_assert!(pipe < 8);
        if pipe >= 8 {
            return;
        }
        with_irqs_masked::<T, _>(|| {
            let sm = unsafe { &mut *self.sm.get() };
            // A queued send may already have pended RADIO. Preserve that work.
            update_default_pipe(&mut sm.tx_pipe, pipe);
        });
    }

    /// Get current PTX state.
    pub fn state(&self) -> crate::state_machine::StatePtx {
        with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.state())
    }

    /// Check if max retransmit attempts was reached since last check.
    ///
    /// Returns `true` if a packet was dropped due to max retransmit.
    /// Clears the flag on read.
    pub fn max_attempts_reached(&self) -> bool {
        self.max_attempts_flag.swap(false, Ordering::AcqRel)
    }

    /// Check if the PTX transmitter is idle (no packet in flight).
    pub fn is_tx_idle(&self) -> bool {
        self.state() == crate::state_machine::StatePtx::Idle
    }

    fn default_pipe(&self) -> u8 {
        with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.tx_pipe)
    }

    // ---- Suspend / Resume ----

    /// Non-blocking suspend. Returns `Err(Busy)` if mid-transaction.
    ///
    /// After success, RADIO IRQ is disabled and the radio is stopped.
    /// Call `restore()` to re-initialize and resume operation.
    pub fn try_suspend(&self) -> Result<EsbSavedState, Error> {
        let guard = IrqMask::<T>::new();

        let sm = unsafe { &mut *self.sm.get() };
        if sm.state() != crate::state_machine::StatePtx::Idle {
            return Err(Error::Busy);
        }

        let saved = sm.do_suspend(self.pool);
        unpend_irqs::<T>();
        guard.keep_masked();
        Ok(saved)
    }

    /// Async suspend. Waits for current transaction to complete, then suspends.
    ///
    /// If already idle, returns immediately. Otherwise, signals the ISR to
    /// stop after the current transaction and waits for it to go idle.
    ///
    /// After return, RADIO IRQ is disabled and the radio is stopped.
    /// Call `restore()` to re-initialize and resume operation.
    pub async fn suspend(&self) -> EsbSavedState {
        // Fast path: already idle
        if let Ok(saved) = self.try_suspend() {
            return saved;
        }

        // Set the flag so ISR will suppress new TX after current transaction.
        // If this future is cancelled, the guard clears the request.
        let _request_guard = SuspendRequestGuard::new(&self.suspend_requested);

        // Re-check under IRQ-disabled to close the race window where
        // the ISR completed between try_suspend() fail and the store above.
        {
            let guard = IrqMask::<T>::new();
            let sm = unsafe { &mut *self.sm.get() };
            if sm.state() == crate::state_machine::StatePtx::Idle {
                let saved = sm.do_suspend(self.pool);
                unpend_irqs::<T>();
                guard.keep_masked();
                return saved;
            }
        }

        // ISR is running and will see suspend_requested — wait for wake.
        core::future::poll_fn(|cx| {
            self.suspend_signal.register(cx.waker());
            let state = with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.state());
            if state == crate::state_machine::StatePtx::Idle {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // ISR is done and state is idle — finalize
        let guard = IrqMask::<T>::new();
        let sm = unsafe { &mut *self.sm.get() };
        let saved = sm.do_suspend(self.pool);
        unpend_irqs::<T>();
        guard.keep_masked();
        saved
    }

    /// Restore ESB from saved state. Full RADIO re-init + PID/CRC restore.
    ///
    /// Reinitializes under both IRQ masks. Call unsafe `activate()` to resume.
    /// Does NOT automatically start sending — queued packets will be sent on
    /// the next NoAck send after activation.
    pub fn restore(&self, state: &EsbSavedState, addresses: &EsbAddresses) {
        let guard = IrqMask::<T>::new();
        let sm = unsafe { &mut *self.sm.get() };
        sm.do_suspend(self.pool);
        sm.do_restore(state, addresses);
        self.suspend_requested.store(false, Ordering::Release);
        unpend_irqs::<T>();
        guard.keep_masked();
    }

    /// Enable the bound RADIO and TIMER interrupts after static placement/setup.
    ///
    /// # Safety
    /// Both IRQ vectors must dispatch exclusively to this exact static driver.
    /// Set identical preemption priorities so RADIO and TIMER cannot nest;
    /// neither handler may reenter or retain borrows. Complete clocks, addresses,
    /// vector dispatch, and priority setup before calling. No outstanding raw
    /// RADIO/TIMER or state-machine borrow is allowed, and no other owner may
    /// access either peripheral. Do not use this from a handler or during an
    /// in-flight transaction. The driver must remain live for every IRQ call.
    /// Waker callbacks must defer task polling and never synchronously reenter
    /// driver state methods from an ISR. NMI/fault handlers must not access it.
    /// Activate before sending (activation clears pending IRQs).
    pub unsafe fn activate(&'static self) {
        cortex_m::interrupt::free(|_| {
            unpend_irqs::<T>();
            unsafe {
                cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::RADIO);
                cortex_m::peripheral::NVIC::unmask(crate::timer::timer_interrupt::<T>());
            }
        });
    }

    // ---- Timeslot entrypoints (S6) ----
    //
    // Thin public wrappers over `PtxStateMachine`'s `ts_*` methods. Called by
    // the MPSL timeslot callback via the `TimeslotPtxDriver` trait object. Each
    // method accesses the SM under the same MPSL signal serialization contract
    // as PRX (single slot at a time, no locking needed inside).

    /// Record addresses + TX pipe so each slot can re-init RADIO.
    #[cfg(feature = "mpsl")]
    pub fn ts_configure(&self, addresses: &EsbAddresses, tx_pipe: u8) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.set_addresses(addresses);
        sm.set_tx_pipe(tx_pipe);
    }

    /// Slot start — power-cycle + re-init RADIO, restore dup-detection, arm TX.
    #[cfg(feature = "mpsl")]
    pub fn ts_start_tx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error> {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_start_tx(self.pool, saved_pid, saved_crc, saved_valid)
    }

    /// SIGNAL_RADIO — drive one PtxStateMachine event. `timer_flag` is true
    /// when the mpsl layer's TIMER0 CC[1] (ACK timeout) fired.
    #[cfg(feature = "mpsl")]
    pub fn ts_on_radio(&self, timer_flag: bool) -> crate::state_machine::PtxEvent {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_on_radio(self.pool, timer_flag)
    }

    /// Slot end — stop radio, release buffers, clear timeslot_managed.
    #[cfg(feature = "mpsl")]
    pub fn ts_force_stop(&self) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_force_stop(self.pool);
    }

    /// Snapshot the current duplicate-detection state (for cross-slot
    /// preservation at slot boundaries).
    #[cfg(feature = "mpsl")]
    pub fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]) {
        let sm = unsafe { &*self.sm.get() };
        (sm.saved_pid(), sm.saved_crc(), sm.saved_valid())
    }

    /// Returns the last pipe a TX was sent on (0xFF if none). Diagnostic only.
    #[cfg(feature = "mpsl")]
    pub fn ts_last_pipe(&self) -> u8 {
        let sm = unsafe { &*self.sm.get() };
        sm.last_pipe()
    }

    /// Returns the current PID. Diagnostic only.
    #[cfg(feature = "mpsl")]
    pub fn ts_pid(&self) -> u8 {
        let sm = unsafe { &*self.sm.get() };
        sm.pid()
    }

    /// Returns the current retransmit attempt count. Diagnostic only.
    #[cfg(feature = "mpsl")]
    pub fn ts_attempts(&self) -> u8 {
        let sm = unsafe { &*self.sm.get() };
        sm.attempts()
    }

    /// Returns the current PTX state.
    #[cfg(feature = "mpsl")]
    pub fn ts_state(&self) -> crate::state_machine::StatePtx {
        let sm = unsafe { &*self.sm.get() };
        sm.state()
    }
}

// ---- PTX Timeslot driver trait (S6 / D6) ----

/// Object-safe bridge between the generic `EsbPtx<T,N,SIZE>` and the non-generic
/// MPSL timeslot callback. Symmetric to [`TimeslotPrxDriver`].
///
/// The PTX timeslot callback is a single global `extern "C" fn` that must work
/// for any `T/N/SIZE`. It stores `Option<&'static dyn TimeslotPtxDriver>` and
/// dispatches SIGNAL_START / SIGNAL_RADIO / SIGNAL_TIMER0 / slot-end through
/// this trait.
///
/// SAFETY contract: all methods are called from the high-priority MPSL signal
/// context, serialised by the MPSL session (one slot at a time). No locking is
/// needed inside the methods.
#[cfg(feature = "mpsl")]
pub trait TimeslotPtxDriver: Sync {
    /// Record addresses + TX pipe before a timeslot session drives this PTX.
    fn ts_configure(&self, addresses: &crate::addresses::EsbAddresses, tx_pipe: u8);

    /// Slot start: power-cycle + re-init RADIO, restore dup-detection, arm TX.
    fn ts_start_tx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error>;

    /// SIGNAL_RADIO: drive one PtxStateMachine event. `timer_flag` is true when
    /// the mpsl layer's TIMER0 CC[1] (ACK timeout) fired.
    fn ts_on_radio(&self, timer_flag: bool) -> crate::state_machine::PtxEvent;

    /// Slot end / EXTEND_FAILED / OVERSTAYED: stop radio, release buffers.
    fn ts_force_stop(&self);

    /// Snapshot the current duplicate-detection state.
    fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]);

    /// Last pipe a TX was sent on (0xFF if none). Diagnostic only.
    fn ts_last_pipe(&self) -> u8;

    /// Current PID. Diagnostic only.
    fn ts_pid(&self) -> u8;

    /// Current retransmit attempt count. Diagnostic only.
    fn ts_attempts(&self) -> u8;

    /// Current PTX state. Diagnostic only.
    fn ts_state(&self) -> crate::state_machine::StatePtx;
}

#[cfg(feature = "mpsl")]
impl<T: TimerInstance, const N: usize, const SIZE: usize> TimeslotPtxDriver for EsbPtx<T, N, SIZE> {
    fn ts_configure(&self, addresses: &crate::addresses::EsbAddresses, tx_pipe: u8) {
        EsbPtx::ts_configure(self, addresses, tx_pipe)
    }

    fn ts_start_tx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error> {
        EsbPtx::ts_start_tx(self, saved_pid, saved_crc, saved_valid)
    }

    fn ts_on_radio(&self, timer_flag: bool) -> crate::state_machine::PtxEvent {
        EsbPtx::ts_on_radio(self, timer_flag)
    }

    fn ts_force_stop(&self) {
        EsbPtx::ts_force_stop(self)
    }

    fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]) {
        EsbPtx::ts_snapshot_dup_state(self)
    }

    fn ts_last_pipe(&self) -> u8 {
        EsbPtx::ts_last_pipe(self)
    }

    fn ts_pid(&self) -> u8 {
        EsbPtx::ts_pid(self)
    }

    fn ts_attempts(&self) -> u8 {
        EsbPtx::ts_attempts(self)
    }

    fn ts_state(&self) -> crate::state_machine::StatePtx {
        EsbPtx::ts_state(self)
    }
}

/// Embassy async PRX (Primary Receiver) driver.
pub struct EsbPrx<
    T: TimerInstance,
    const N: usize = DEFAULT_POOL_N,
    const SIZE: usize = DEFAULT_POOL_SIZE,
> {
    sm: UnsafeCell<PrxStateMachine<T>>,
    _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
    receive_auto_ack: bool,
    pool: &'static PacketPool<N, SIZE>,
    timer_flag: AtomicBool,
    /// Set by app to request ISR to stop after current ACK TX.
    suspend_requested: AtomicBool,
    /// Woken by ISR when it returns to Receiver/Idle with suspend_requested set.
    suspend_signal: AtomicWaker,
    max_payload_len: usize,
}

// SAFETY: Same ownership model as `EsbPtx`: the driver is static, RADIO ISR
// owns normal state-machine progress, and task-context state access masks
// RADIO/TIMER and PRIMASK. RX/TX buffers follow the pool ownership state machine.
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Send for EsbPrx<T, N, SIZE> {}
unsafe impl<T: TimerInstance, const N: usize, const SIZE: usize> Sync for EsbPrx<T, N, SIZE> {}

#[allow(dead_code)]
impl<T: TimerInstance, const N: usize, const SIZE: usize> EsbPrx<T, N, SIZE> {
    pub(crate) fn from_parts(
        parts: DriverParts<T, N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
    ) -> Self {
        Self {
            sm: UnsafeCell::new(PrxStateMachine::from_parts(
                parts.radio,
                parts.timer,
                config,
                addresses.enabled_mask(),
            )),
            _radio: parts.radio_token,
            receive_auto_ack: false,
            pool: parts.pool,
            timer_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: usize::from(config.payload_length),
        }
    }

    pub(crate) fn into_parts(self) -> DriverParts<T, N, SIZE> {
        let (radio, timer) = self.sm.into_inner().into_parts();
        DriverParts {
            radio,
            timer,
            radio_token: self._radio,
            pool: self.pool,
        }
    }

    pub(crate) fn quiesce(&mut self) -> StopOutcome {
        let sm = self.sm.get_mut();
        let stop = sm.quiesce(self.pool);
        self.timer_flag.store(false, Ordering::Release);
        unpend_irqs::<T>();
        if sm.radio.take_stop_fault() {
            StopOutcome::PowerCycled
        } else {
            stop
        }
    }

    pub(crate) fn start_exclusive_rx(&mut self) -> Result<(), Error> {
        self.sm.get_mut().start_receiving(self.pool)
    }

    pub(crate) fn set_rx_metadata_context(
        &mut self,
        context: Option<crate::rx_metadata::RxMetadataContext>,
    ) {
        self.sm.get_mut().set_rx_metadata_context(context);
    }

    pub(crate) fn exclusive_radio_event(&mut self) {
        let timer = self.timer_flag.swap(false, Ordering::AcqRel);
        self.sm.get_mut().handle_radio_event(self.pool, timer);
    }

    pub(crate) fn exclusive_timer_event(&mut self) {
        self.timer_flag.store(true, Ordering::Release);
        self.sm.get_mut().handle_timer_event();
    }

    pub(crate) fn take_stop_fault(&mut self) -> bool {
        self.sm.get_mut().radio.take_stop_fault()
    }

    pub(crate) fn receiver_idle(&mut self) -> bool {
        self.sm.get_mut().state() == crate::state_machine::StatePrx::Idle
    }

    /// Create a new PRX driver.
    /// Validates configuration/pool before MMIO and returns with both IRQs
    /// masked and unpended. Place statically, bind IRQs, then activate.
    pub fn new(
        timer: embassy_nrf::Peri<'static, T>,
        _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
    ) -> Result<Self, Error> {
        config.validate()?;
        validate_pool::<N, SIZE>(config.payload_length)?;
        let guard = IrqMask::<T>::new();
        unpend_irqs::<T>();
        let mut radio = EsbRadio::new(crate::pac::RADIO);
        radio.stop();
        radio.init(config, addresses);

        let esb_timer = EsbTimer::new(timer);
        let enabled_pipes = addresses.enabled_mask();
        let sm = PrxStateMachine::new(radio, esb_timer, config, enabled_pipes);

        unpend_irqs::<T>();
        guard.keep_masked();

        Ok(Self {
            sm: UnsafeCell::new(sm),
            _radio,
            receive_auto_ack: config.receive_auto_ack,
            pool,
            timer_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: config.payload_length as usize,
        })
    }

    /// Create a new PRX driver for MPSL timeslot-managed mode (S3).
    ///
    /// Like [`new`](Self::new) but does **not** unmask the RADIO or TIMER ISRs —
    /// in timeslot mode the RADIO ISR vector belongs to MPSL (the app receives
    /// events via `SIGNAL_RADIO`), and the protocol timing comes from TIMER0
    /// owned by MPSL, not from `EsbTimer<T>`. RADIO is also left un-initialised
    /// here because each timeslot power-cycles it; `ts_start_rx` does the init.
    ///
    /// **NVIC note:** unlike `new()`, this does NOT mask/unmask RADIO. MPSL
    /// owns the RADIO vector; masking it here would break MPSL signal delivery.
    #[cfg(feature = "mpsl")]
    pub fn new_timeslot(
        timer: embassy_nrf::Peri<'static, T>,
        _radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
        pool: &'static PacketPool<N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
    ) -> Result<Self, Error> {
        config.validate()?;
        validate_pool::<N, SIZE>(config.payload_length)?;
        let guard = IrqMask::<T>::new();
        unpend_irqs::<T>();
        // SM keeps the config + a fresh radio instance; RADIO registers are
        // programmed on each slot by `ts_start_rx`.
        let radio = EsbRadio::new(crate::pac::RADIO);
        let esb_timer = EsbTimer::new(timer);
        let enabled_pipes = addresses.enabled_mask();
        let mut sm = PrxStateMachine::new(radio, esb_timer, config, enabled_pipes);
        sm.set_addresses(addresses);
        sm.set_timeslot_managed(true);

        // Deliberately do NOT touch NVIC RADIO — MPSL owns it in timeslot mode.
        // TIMER interrupt is also left alone; PRX has no ACK-timeout in either
        // mode, and the timer token is only consumed to prevent aliasing.
        guard.keep_masked();
        Ok(Self {
            sm: UnsafeCell::new(sm),
            _radio,
            receive_auto_ack: config.receive_auto_ack,
            pool,
            timer_flag: AtomicBool::new(false),
            suspend_requested: AtomicBool::new(false),
            suspend_signal: AtomicWaker::new(),
            max_payload_len: config.payload_length as usize,
        })
    }

    /// Called from the RADIO interrupt handler.
    ///
    /// # Safety
    /// Call only from this driver's bound RADIO IRQ after activation. RADIO and
    /// TIMER handlers must not nest or run concurrently, and no task or handler
    /// may hold a state-machine, RADIO, or TIMER borrow across this call.
    pub unsafe fn on_radio_interrupt(&self) {
        let timer_flag = self.timer_flag.load(Ordering::Acquire);
        if timer_flag {
            self.timer_flag.store(false, Ordering::Release);
        }
        // SAFETY: ISR-only access.
        let sm = unsafe { &mut *self.sm.get() };
        sm.handle_radio_event(self.pool, timer_flag);

        // If suspend was requested and we're in a suspendable state
        // (Idle or Receiver), wake the waiting async task.
        if self.suspend_requested.load(Ordering::Acquire) {
            let state = sm.state();
            if state == crate::state_machine::StatePrx::Idle
                || state == crate::state_machine::StatePrx::Receiver
            {
                self.suspend_signal.wake();
            }
        }

        #[cfg(any(feature = "nrf52840", feature = "nrf52833", feature = "nrf52832"))]
        cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::RADIO);
    }

    /// Called from the TIMER interrupt handler.
    /// # Safety
    /// Call only from this driver's bound TIMER IRQ after activation. It must
    /// not nest with RADIO, reenter, or overlap any RADIO/TIMER/state borrow.
    pub unsafe fn on_timer_interrupt(&self) {
        self.timer_flag.store(true, Ordering::Release);
        // SAFETY: ISR-only access.
        let sm = unsafe { &*self.sm.get() };
        sm.handle_timer_event();
    }

    /// Start listening for incoming packets.
    pub fn start_listening(&self) -> Result<(), Error> {
        with_irqs_masked::<T, _>(|| {
            let sm = unsafe { &mut *self.sm.get() };
            let result = sm.start_receiving(self.pool);
            unpend_irqs::<T>();
            result
        })
    }

    /// Receive the next packet (async).
    pub async fn receive(&self) -> ReceivedPacket<'_, N, SIZE> {
        let idx = self.pool.receive_rx().await;
        ReceivedPacket {
            pool: self.pool,
            idx,
        }
    }

    /// Queue an ACK payload for a specific RX pipe.
    ///
    /// ACK payloads are pipe-filtered by the PRX state machine, so a payload
    /// queued for pipe 1 cannot be consumed by a packet received on pipe 0.
    /// Returns [`Error::TxFull`] when the packet pool is full and
    /// [`Error::InvalidParam`] for an invalid pipe, empty payload, or oversized
    /// payload, or when `receive_auto_ack` is false.
    pub async fn send_ack_payload(&self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        if !self.receive_auto_ack {
            return Err(Error::InvalidParam);
        }
        if payload.is_empty() || payload.len() > self.max_payload_len {
            return Err(Error::InvalidParam);
        }
        if pipe >= 8 {
            return Err(Error::InvalidParam);
        }
        let payload_offset = EsbHeader::PAYLOAD_OFFSET;
        if payload_offset + payload.len() > SIZE {
            return Err(Error::InvalidParam);
        }
        let idx = self.pool.alloc_tx().ok_or(Error::TxFull)?;

        let header = unsafe { self.pool.header_mut(idx) };
        header.length = payload.len() as u8;
        header.pipe = pipe;
        header.set_no_ack(false);

        let buf = unsafe { self.pool.buf_mut(idx) };
        buf[payload_offset..payload_offset + payload.len()].copy_from_slice(payload);

        self.pool.enqueue_tx(idx).await;
        Ok(())
    }

    /// Stop listening and go idle.
    pub fn stop(&self) {
        with_irqs_masked::<T, _>(|| {
            let sm = unsafe { &mut *self.sm.get() };
            sm.stop_receiving(self.pool);
            unpend_irqs::<T>();
        });
    }

    /// Get current PRX state.
    pub fn state(&self) -> crate::state_machine::StatePrx {
        with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.state())
    }

    // ---- Suspend / Resume ----

    /// Non-blocking suspend. Returns `Err(Busy)` if mid-ACK-TX.
    ///
    /// PRX in `Idle` or `Receiver` state can be suspended immediately.
    /// `TxAck`/`TxRepeatedAck` states return `Err(Busy)`.
    pub fn try_suspend(&self) -> Result<EsbSavedState, Error> {
        let guard = IrqMask::<T>::new();

        let sm = unsafe { &mut *self.sm.get() };
        let state = sm.state();
        if state != crate::state_machine::StatePrx::Idle
            && state != crate::state_machine::StatePrx::Receiver
        {
            return Err(Error::Busy);
        }

        let saved = sm.do_suspend(self.pool);
        unpend_irqs::<T>();
        guard.keep_masked();
        Ok(saved)
    }

    /// Async suspend. Waits for any in-progress ACK TX to complete.
    ///
    /// If in `Idle` or `Receiver` state, returns immediately.
    /// Otherwise waits for the ISR to finish the ACK TX.
    pub async fn suspend(&self) -> EsbSavedState {
        if let Ok(saved) = self.try_suspend() {
            return saved;
        }

        // If this future is cancelled before suspension completes, clear the
        // request so the ISR does not keep suppressing progress.
        let _request_guard = SuspendRequestGuard::new(&self.suspend_requested);

        // Re-check under IRQ-disabled to close the race window where
        // the ISR completed between try_suspend() fail and the store above.
        {
            let guard = IrqMask::<T>::new();
            let sm = unsafe { &mut *self.sm.get() };
            let state = sm.state();
            if state == crate::state_machine::StatePrx::Idle
                || state == crate::state_machine::StatePrx::Receiver
            {
                let saved = sm.do_suspend(self.pool);
                unpend_irqs::<T>();
                guard.keep_masked();
                return saved;
            }
        }

        core::future::poll_fn(|cx| {
            self.suspend_signal.register(cx.waker());
            let state = with_irqs_masked::<T, _>(|| unsafe { &*self.sm.get() }.state());
            if state == crate::state_machine::StatePrx::Idle
                || state == crate::state_machine::StatePrx::Receiver
            {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        let guard = IrqMask::<T>::new();
        let sm = unsafe { &mut *self.sm.get() };
        let saved = sm.do_suspend(self.pool);
        unpend_irqs::<T>();
        guard.keep_masked();
        saved
    }

    /// Restore ESB from saved state. Full RADIO re-init + PID/CRC restore.
    ///
    /// Reinitializes under both IRQ masks. Call unsafe `activate()` to resume.
    /// Does NOT automatically start listening — call `start_listening()` after.
    pub fn restore(&self, state: &EsbSavedState, addresses: &EsbAddresses) {
        let guard = IrqMask::<T>::new();
        let sm = unsafe { &mut *self.sm.get() };
        sm.do_suspend(self.pool);
        sm.do_restore(state, addresses);
        self.suspend_requested.store(false, Ordering::Release);
        unpend_irqs::<T>();
        guard.keep_masked();
    }

    /// Enable the bound RADIO and TIMER interrupts after static placement/setup.
    ///
    /// # Safety
    /// Both IRQ vectors must dispatch exclusively to this exact static driver.
    /// Set identical preemption priorities so RADIO and TIMER cannot nest;
    /// neither handler may reenter or retain borrows. Complete clocks, addresses,
    /// vector dispatch, and priority setup before calling. No outstanding raw
    /// RADIO/TIMER or state-machine borrow is allowed, and no other owner may
    /// access either peripheral. Do not use this from a handler or during an
    /// in-flight transaction. The driver must remain live for every IRQ call.
    /// Waker callbacks must defer task polling and never synchronously reenter
    /// driver state methods from an ISR. NMI/fault handlers must not access it.
    /// Activate before starting RX (activation clears pending IRQs).
    pub unsafe fn activate(&'static self) {
        cortex_m::interrupt::free(|_| {
            unpend_irqs::<T>();
            unsafe {
                cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::RADIO);
                cortex_m::peripheral::NVIC::unmask(crate::timer::timer_interrupt::<T>());
            }
        });
    }

    // ---- Timeslot entrypoints (S3) ----
    //
    // Thin public wrappers over `PrxStateMachine`'s `ts_*` methods. Called by
    // the MPSL timeslot callback via the `TimeslotPrxDriver` trait object. Each
    // method takes the RADIO-TIMER IRQ mask guard pattern as read by the caller
    // (MPSL callback runs with TIMER0 already owned, and the RADIO NVIC line is
    // *not* used in timeslot mode — MPSL delivers events via SIGNAL_RADIO).

    /// Record addresses + enabled pipes so each slot can re-init RADIO.
    #[cfg(feature = "mpsl")]
    pub fn ts_configure(&self, addresses: &EsbAddresses, enabled_pipes: u8) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.set_addresses(addresses);
        sm.set_enabled_pipes(enabled_pipes);
    }

    /// Slot start — power-cycle + re-init RADIO, restore dup-detection, arm RX.
    #[cfg(feature = "mpsl")]
    pub fn ts_start_rx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error> {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_start_rx(self.pool, saved_pid, saved_crc, saved_valid)
    }

    /// SIGNAL_RADIO — drive one PrxStateMachine event.
    #[cfg(feature = "mpsl")]
    pub fn ts_on_radio(&self) -> (crate::state_machine::PrxEvent, Option<usize>) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_on_radio(self.pool)
    }

    /// Provide diagnostic ACK context for the next in-slot PRX event.
    #[cfg(feature = "mpsl")]
    pub fn ts_set_diag_ack_context(&self, context: crate::state_machine::TimeslotDiagAckContext) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.set_timeslot_diag_ack_context(context);
    }

    /// Discard RX packets queued by the timeslot diagnostic driver.
    #[cfg(feature = "mpsl")]
    pub fn ts_discard_received(&self) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_discard_received(self.pool);
    }

    /// Slot end — stop radio, release buffers, clear timeslot_managed.
    #[cfg(feature = "mpsl")]
    pub fn ts_force_stop(&self) {
        let sm = unsafe { &mut *self.sm.get() };
        sm.ts_force_stop(self.pool);
    }

    /// Snapshot the current duplicate-detection state (for cross-slot
    /// preservation at slot boundaries).
    #[cfg(feature = "mpsl")]
    pub fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]) {
        let sm = unsafe { &*self.sm.get() };
        (sm.saved_pid(), sm.saved_crc(), sm.saved_valid())
    }

    /// Returns the last pipe processed by the state machine (0xFF if none).
    /// Diagnostic only — used by the timeslot callback to attribute per-pipe
    /// counters in converged mode.
    #[cfg(feature = "mpsl")]
    pub fn ts_last_pipe(&self) -> u8 {
        let sm = unsafe { &*self.sm.get() };
        sm.last_pipe()
    }
}

// ---- Timeslot driver trait (S3 / D6) ----

/// Object-safe bridge between the generic `EsbPrx<T,N,SIZE>` and the non-generic
/// MPSL timeslot callback.
///
/// The PRX timeslot callback is a single global `extern "C" fn` that must work
/// for any `T/N/SIZE`. It stores `Option<&'static dyn TimeslotPrxDriver>` and
/// dispatches SIGNAL_START / SIGNAL_RADIO / slot-end through this trait.
///
/// SAFETY contract: all methods are called from the high-priority MPSL signal
/// context, serialised by the MPSL session (one slot at a time). No locking is
/// needed inside the methods.
#[cfg(feature = "mpsl")]
pub trait TimeslotPrxDriver: Sync {
    /// Record addresses + enabled pipes before a timeslot session drives this
    /// PRX. This keeps the shared state machine aligned with the session config.
    fn ts_configure(&self, addresses: &crate::addresses::EsbAddresses, enabled_pipes: u8);

    /// Slot start: configure addresses (first slot only) + arm RX. The driver
    /// internally handles RADIO power-cycle / re-init and dup-state restore.
    fn ts_start_rx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error>;

    /// SIGNAL_RADIO: drive one PrxStateMachine event. Returns the protocol
    /// event so the timeslot layer can update diagnostic counters.
    fn ts_on_radio(&self) -> (crate::state_machine::PrxEvent, Option<usize>);

    /// Provide per-slot diagnostic ACK metadata before handling a RADIO event.
    fn ts_set_diag_ack_context(&self, context: crate::state_machine::TimeslotDiagAckContext);

    /// Discard packets queued for app delivery by diagnostic timeslot sessions.
    fn ts_discard_received(&self);

    /// Slot end / EXTEND_FAILED / OVERSTAYED: stop radio, release buffers,
    /// clear timeslot_managed. After return, the driver is quiescent.
    fn ts_force_stop(&self);

    /// Snapshot the current duplicate-detection state (called at slot end to
    /// carry into the next slot).
    fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]);

    /// Last pipe processed (0xFF if none). Diagnostic only.
    fn ts_last_pipe(&self) -> u8;
}

#[cfg(feature = "mpsl")]
impl<T: TimerInstance, const N: usize, const SIZE: usize> TimeslotPrxDriver for EsbPrx<T, N, SIZE> {
    fn ts_configure(&self, addresses: &crate::addresses::EsbAddresses, enabled_pipes: u8) {
        EsbPrx::ts_configure(self, addresses, enabled_pipes)
    }

    fn ts_start_rx(
        &self,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), crate::error::Error> {
        EsbPrx::ts_start_rx(self, saved_pid, saved_crc, saved_valid)
    }

    fn ts_on_radio(&self) -> (crate::state_machine::PrxEvent, Option<usize>) {
        EsbPrx::ts_on_radio(self)
    }

    fn ts_set_diag_ack_context(&self, context: crate::state_machine::TimeslotDiagAckContext) {
        EsbPrx::ts_set_diag_ack_context(self, context)
    }

    fn ts_discard_received(&self) {
        EsbPrx::ts_discard_received(self)
    }

    fn ts_force_stop(&self) {
        EsbPrx::ts_force_stop(self)
    }

    fn ts_snapshot_dup_state(&self) -> ([u8; 8], [u16; 8], [bool; 8]) {
        EsbPrx::ts_snapshot_dup_state(self)
    }

    fn ts_last_pipe(&self) -> u8 {
        EsbPrx::ts_last_pipe(self)
    }
}

// ---- Received Packet Handle ----

/// A received packet from the ESB pool.
///
/// Provides read access to the packet metadata and payload.
/// Drop to release the buffer back to the pool.
pub struct ReceivedPacket<'a, const N: usize, const SIZE: usize> {
    pool: &'a PacketPool<N, SIZE>,
    idx: usize,
}

#[allow(dead_code)]
impl<const N: usize, const SIZE: usize> ReceivedPacket<'_, N, SIZE> {
    pub(crate) fn from_pool(pool: &PacketPool<N, SIZE>, idx: usize) -> ReceivedPacket<'_, N, SIZE> {
        ReceivedPacket { pool, idx }
    }
    /// Get the pipe number.
    pub fn pipe(&self) -> u8 {
        let header = unsafe { self.pool.header(self.idx) };
        header.pipe
    }

    /// Copy the receive snapshot. Legacy drivers and constructors return None.
    /// A failed clock retains channel/epoch metadata with timestamp_us = None.
    pub fn metadata(&self) -> Option<crate::RxMetadata> {
        self.pool.rx_metadata(self.idx)
    }

    /// Get the RSSI value.
    pub fn rssi(&self) -> u8 {
        let header = unsafe { self.pool.header(self.idx) };
        header.rssi
    }

    /// Get the payload length.
    pub fn len(&self) -> usize {
        let header = unsafe { self.pool.header(self.idx) };
        header.length as usize
    }

    /// Check if the payload is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get the payload as a byte slice.
    pub fn payload(&self) -> &[u8] {
        let buf = unsafe { self.pool.buf(self.idx) };
        let header = unsafe { self.pool.header(self.idx) };
        let len = header.length as usize;
        let payload_offset = EsbHeader::PAYLOAD_OFFSET;
        let end = (payload_offset + len).min(buf.len());
        &buf[payload_offset..end]
    }

    /// Get the pool index (for advanced use).
    pub(crate) fn index(&self) -> usize {
        self.idx
    }
}

impl<const N: usize, const SIZE: usize> Drop for ReceivedPacket<'_, N, SIZE> {
    fn drop(&mut self) {
        self.pool.release_rx(self.idx);
    }
}

#[cfg(test)]
mod tests {
    use super::{update_default_pipe, validate_pool};
    use crate::error::Error;
    use crate::payload::PacketPool;

    #[test]
    fn changing_default_pipe_preserves_queued_no_ack_packet() {
        let pool = PacketPool::<2, 16>::new();
        let mut default_pipe = 1;
        let idx = pool.alloc_tx().expect("free packet");
        unsafe {
            let header = pool.header_mut(idx);
            header.pipe = default_pipe;
            header.length = 1;
            header.set_no_ack(true);
        }
        let mut enqueue = core::pin::pin!(pool.enqueue_tx(idx));
        let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
        assert_eq!(
            core::future::Future::poll(enqueue.as_mut(), &mut cx),
            core::task::Poll::Ready(())
        );

        update_default_pipe(&mut default_pipe, 3);
        assert_eq!(default_pipe, 3);
        // Models consuming previously queued software work; no NVIC execution.
        assert_eq!(pool.try_dequeue_tx(), Some(idx));
        let header = unsafe { pool.header(idx) };
        assert_eq!(header.pipe, 1);
        assert!(header.no_ack());
        pool.release_tx(idx);
    }

    #[test]
    fn pool_validation_checks_count_and_complete_dma_capacity() {
        assert_eq!(validate_pool::<2, 256>(252), Ok(()));
        assert_eq!(validate_pool::<2, 255>(252), Err(Error::InvalidParam));
        assert_eq!(validate_pool::<0, 256>(252), Err(Error::InvalidParam));
        assert_eq!(validate_pool::<1, 256>(252), Err(Error::InvalidParam));
        assert_eq!(validate_pool::<2, 0>(32), Err(Error::InvalidParam));
        assert_eq!(validate_pool::<2, 35>(32), Err(Error::InvalidParam));
        assert_eq!(validate_pool::<2, 36>(32), Ok(()));
        assert_eq!(validate_pool::<2, 3>(0), Err(Error::InvalidParam));
    }
}
