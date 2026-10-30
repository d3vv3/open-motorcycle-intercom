//! MPSL timeslot adapter for ESB.
//!
//! Manages timeslot sessions via nrf-mpsl, providing async APIs for
//! requesting individual timeslots, chained timeslot sequences, and
//! PTX transmissions within timeslots.
//!
//! # Stability
//!
//! This module is experimental and diagnostic. The current public functions are
//! useful for bring-up and hardware regression runs, but they are not a
//! stable application-transport API.
//!
//! Current limitations:
//!
//! - sessions are free functions backed by static global state;
//! - PTX/PRX buffers are fixed static buffers rather than caller-owned pools;
//! - the in-slot PTX/PRX protocol logic is duplicated from the exclusive ESB
//!   state machines;
//! - the diagnostic PTX path does not yet implement the full ACK timeout and
//!   retry behavior used by the exclusive ESB core;
//! - active BLE connection coexistence still needs scheduler tuning.
//!
//! Use the exclusive `EsbPtx`/`EsbPrx` path as the stable baseline for
//! production ESB applications.

use core::cell::RefCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, compiler_fence};
use core::task::Poll;

use cortex_m::peripheral::NVIC;
use embassy_nrf::interrupt::Interrupt;
use embassy_nrf::pac;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::waitqueue::WakerRegistration;

use nrf_mpsl::{MultiprotocolServiceLayer, RetVal, raw};

use crate::error::Error;
use crate::mpsl_common::{
    NUM_PIPES, advance_pid, delta_per_pipe, next_pipe_in_mask, read_counter_payload,
    write_counter_packet, write_counter_payload_packet, write_counter_schedule_extension_packet,
};
pub use crate::mpsl_profile::{
    BleCoexistenceHint, CoexistenceProfile, CoexistenceProfileConfig, PrxScheduleConfig,
    PrxSlotConfig, PtxEventConfig, PtxPollConfig, PtxScheduleGateConfig, PtxScheduleMode,
    TimeslotRequestConfig,
};
pub use crate::mpsl_radio::{RadioDisableResult, RadioQuiesceResult, RadioRecoveryPolicy};
use crate::mpsl_radio::{disable_radio_bounded, quiesce_radio_before_timeslot_end};
use crate::mpsl_schedule::{
    ScheduleHint, ScheduleTracker, ScheduleTrackerSnapshot, decode_counter_payload_schedule_hint,
};

const TIMESLOT_TIMER_INTERRUPT: Interrupt = Interrupt::TIMER0;

/// Maximum caller-defined ACK extension length after the scheduler's counter
/// and timing hint.
pub const PRX_ACK_EXTENSION_MAX_LEN: usize = crate::header::EsbHeader::MAX_PAYLOAD as usize
    - crate::mpsl_schedule::SCHEDULE_COUNTER_HINT_PAYLOAD_LEN;

/// Opaque caller-defined bytes appended to a PRX ACK payload.
///
/// The MPSL adapter neither interprets nor validates these bytes. A higher
/// transport may use them for an application acknowledgement, while a simple
/// ESB user can leave them empty.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PrxAckExtension {
    len: u8,
    bytes: [u8; PRX_ACK_EXTENSION_MAX_LEN],
}

impl PrxAckExtension {
    pub const EMPTY: Self = Self {
        len: 0,
        bytes: [0; PRX_ACK_EXTENSION_MAX_LEN],
    };

    /// Create an extension if `bytes` fits in the available ACK payload space.
    pub fn from_slice(bytes: &[u8]) -> Option<Self> {
        if bytes.len() > PRX_ACK_EXTENSION_MAX_LEN {
            return None;
        }

        let mut extension = Self::EMPTY;
        extension.bytes[..bytes.len()].copy_from_slice(bytes);
        extension.len = bytes.len() as u8;
        Some(extension)
    }

    /// Return the caller-defined bytes carried after the scheduler hint.
    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

impl Default for PrxAckExtension {
    fn default() -> Self {
        Self::EMPTY
    }
}

/// Called from the PRX timeslot callback for a newly received application
/// payload. The callback must be bounded and non-blocking.
///
/// `queued` reports whether the adapter's raw event queue accepted `payload`.
/// A transport can withhold its acknowledgement when `queued` is false.
pub type PrxAckExtensionProvider = fn(u8, &[u8], bool) -> PrxAckExtension;

const TIMESLOT_HFCLK: u8 = raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8;
const TIMESLOT_PRIORITY_NORMAL: u8 = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
const TIMESLOT_PRIORITY_HIGH: u8 = raw::MPSL_TIMESLOT_PRIORITY_HIGH as u8;

fn mpsl_ok(ret: i32) -> Result<(), Error> {
    RetVal::from(ret)
        .to_result()
        .map(|_| ())
        .map_err(|_| Error::Mpsl)
}

fn configure_earliest_request(
    request: &mut raw::mpsl_timeslot_request_t,
    priority: u8,
    length_us: u32,
    timeout_us: u32,
) {
    request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
    request.params.earliest = raw::mpsl_timeslot_request_earliest_t {
        hfclk: TIMESLOT_HFCLK,
        priority,
        length_us,
        timeout_us,
    };
}

fn configure_normal_request(
    request: &mut raw::mpsl_timeslot_request_t,
    priority: u8,
    distance_us: u32,
    length_us: u32,
) {
    request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_NORMAL as u8;
    request.params.normal = raw::mpsl_timeslot_request_normal_t {
        hfclk: TIMESLOT_HFCLK,
        priority,
        distance_us,
        length_us,
    };
}

struct Timer0RawMutex;
// SAFETY: this RawMutex serializes task context against the high-priority MPSL
// timeslot callback (which runs from the TIMER0/RADIO IRQ during a granted
// slot) by masking the timeslot timer interrupt for the duration of `f`. The
// callback is the only other code touching the guarded state, so masking its
// IRQ line is sufficient mutual exclusion on this single-core part. The
// compiler fences keep the critical-section body from being reordered outside
// the masked window.
unsafe impl RawMutex for Timer0RawMutex {
    const INIT: Self = Timer0RawMutex;
    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        // SAFETY: NVIC::PTR is a valid static register block; we set/clear only
        // the single timeslot-timer interrupt bit and restore it after `f`.
        unsafe {
            let nvic = &*NVIC::PTR;
            let irq = TIMESLOT_TIMER_INTERRUPT as usize;
            nvic.icer[irq / 32].write(1u32 << (irq % 32));
            compiler_fence(Ordering::SeqCst);
            let r = f();
            compiler_fence(Ordering::SeqCst);
            nvic.iser[irq / 32].write(1u32 << (irq % 32));
            r
        }
    }
}

/// Cumulative signal counts across a timeslot session.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SignalCounters {
    pub start: u32,
    pub timer0: u32,
    pub radio: u32,
    pub blocked: u32,
    pub cancelled: u32,
    pub session_idle: u32,
    pub session_closed: u32,
    pub overstayed: u32,
    pub invalid_return: u32,
    pub extend_failed: u32,
    pub extend_succeeded: u32,
    pub radio_disable_timeout: u32,
}

impl SignalCounters {
    pub const ZERO: Self = Self {
        start: 0,
        timer0: 0,
        radio: 0,
        blocked: 0,
        cancelled: 0,
        session_idle: 0,
        session_closed: 0,
        overstayed: 0,
        invalid_return: 0,
        extend_failed: 0,
        extend_succeeded: 0,
        radio_disable_timeout: 0,
    };

    pub fn saturating_add(self, other: Self) -> Self {
        Self {
            start: self.start.saturating_add(other.start),
            timer0: self.timer0.saturating_add(other.timer0),
            radio: self.radio.saturating_add(other.radio),
            blocked: self.blocked.saturating_add(other.blocked),
            cancelled: self.cancelled.saturating_add(other.cancelled),
            session_idle: self.session_idle.saturating_add(other.session_idle),
            session_closed: self.session_closed.saturating_add(other.session_closed),
            overstayed: self.overstayed.saturating_add(other.overstayed),
            invalid_return: self.invalid_return.saturating_add(other.invalid_return),
            extend_failed: self.extend_failed.saturating_add(other.extend_failed),
            extend_succeeded: self.extend_succeeded.saturating_add(other.extend_succeeded),
            radio_disable_timeout: self
                .radio_disable_timeout
                .saturating_add(other.radio_disable_timeout),
        }
    }

    fn saturating_sub(self, previous: Self) -> Self {
        Self {
            start: self.start.saturating_sub(previous.start),
            timer0: self.timer0.saturating_sub(previous.timer0),
            radio: self.radio.saturating_sub(previous.radio),
            blocked: self.blocked.saturating_sub(previous.blocked),
            cancelled: self.cancelled.saturating_sub(previous.cancelled),
            session_idle: self.session_idle.saturating_sub(previous.session_idle),
            session_closed: self.session_closed.saturating_sub(previous.session_closed),
            overstayed: self.overstayed.saturating_sub(previous.overstayed),
            invalid_return: self.invalid_return.saturating_sub(previous.invalid_return),
            extend_failed: self.extend_failed.saturating_sub(previous.extend_failed),
            extend_succeeded: self
                .extend_succeeded
                .saturating_sub(previous.extend_succeeded),
            radio_disable_timeout: self
                .radio_disable_timeout
                .saturating_sub(previous.radio_disable_timeout),
        }
    }
}

struct InnerState {
    counters: SignalCounters,
    done: bool,
    waker: WakerRegistration,
    request: raw::mpsl_timeslot_request_t,
    return_param: raw::mpsl_timeslot_signal_return_param_t,
    in_slot_match_us: u32,
    /// Chaining: target number of SIGNAL_START events. 0 = single-shot (no chaining).
    target_count: u32,
}

// SAFETY: InnerState is never accessed concurrently without synchronization.
// It lives inside `Mutex<Timer0RawMutex, RefCell<_>>`, so every read/write from
// task context masks the timeslot IRQ first, and the only other accessor (the
// timeslot callback) runs in that masked window. The raw MPSL request/return
// params it holds are plain POD with no thread-affinity, so moving the state
// between contexts is sound.
unsafe impl Send for InnerState {}
unsafe impl Sync for InnerState {}

struct State {
    busy: AtomicBool,
    inner: Mutex<Timer0RawMutex, RefCell<InnerState>>,
}

static STATE: State = State::new();

impl State {
    const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            inner: Mutex::new(RefCell::new(InnerState {
                counters: SignalCounters::ZERO,
                done: false,
                waker: WakerRegistration::new(),
                request: raw::mpsl_timeslot_request_t {
                    request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
                    params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                        earliest: raw::mpsl_timeslot_request_earliest_t {
                            hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                            priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                            length_us: 0,
                            timeout_us: 1_000_000,
                        },
                    },
                },
                return_param: raw::mpsl_timeslot_signal_return_param_t {
                    callback_action: 0,
                    params: raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1 {
                        request:
                            raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1__bindgen_ty_1 {
                                p_next: core::ptr::null_mut(),
                            },
                    },
                },
                in_slot_match_us: 4500,
                target_count: 0,
            })),
        }
    }

    fn with_inner<F: FnOnce(&mut InnerState) -> R, R>(&self, f: F) -> R {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            f(&mut inner)
        })
    }

    fn try_enter(&'static self) -> Result<BusyGuard, Error> {
        BusyGuard::new(&self.busy)
    }
}

struct BusyGuard {
    busy: &'static AtomicBool,
}

impl BusyGuard {
    fn new(busy: &'static AtomicBool) -> Result<Self, Error> {
        match busy.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => Ok(Self { busy }),
            Err(_) => Err(Error::Busy),
        }
    }
}

impl Drop for BusyGuard {
    fn drop(&mut self) {
        self.busy.store(false, Ordering::Release);
    }
}

// SAFETY: invoked by MPSL from the high-priority timeslot signal context with a
// valid `session_id` for the open session and a `signal` from the MPSL signal
// enum. It must stay deterministic and minimal (no allocation, no async, no
// unmasked locking) and either return a pointer to a `'static` return-param
// struct that outlives the call or null. We only ever return `&STATE`'s
// return_param (a static) or null, satisfying the lifetime contract.
unsafe extern "C" fn timeslot_callback(
    session_id: u8,
    signal: u32,
) -> *mut raw::mpsl_timeslot_signal_return_param_t {
    match signal {
        raw::MPSL_TIMESLOT_SIGNAL_START => STATE.with_inner(|state| {
            state.counters.start += 1;

            let t = pac::TIMER0;
            t.events_compare(0).write_value(0);
            t.cc(0).write_value(state.in_slot_match_us);
            t.intenset().write(|w| w.set_compare(0, true));

            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_TIMER0 => STATE.with_inner(|state| {
            state.counters.timer0 += 1;

            let t = pac::TIMER0;
            t.events_compare(0).write_value(0);
            t.intenclr().write(|w| w.set_compare(0, true));

            let chain = state.target_count > 0 && state.counters.start < state.target_count;
            if chain {
                // Chain next slot via ACTION_REQUEST.
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                state.return_param.params.request.p_next = core::ptr::from_mut(&mut state.request);
            } else {
                state.done = true;
                state.waker.wake();
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            }
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_RADIO => STATE.with_inner(|state| {
            state.counters.radio += 1;
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_IDLE => STATE.with_inner(|state| {
            state.counters.session_idle += 1;
            state.waker.wake();
            core::ptr::null_mut()
        }),

        raw::MPSL_TIMESLOT_SIGNAL_BLOCKED | raw::MPSL_TIMESLOT_SIGNAL_CANCELLED => {
            let request = STATE.with_inner(|state| {
                if signal == raw::MPSL_TIMESLOT_SIGNAL_BLOCKED {
                    state.counters.blocked += 1;
                } else {
                    state.counters.cancelled += 1;
                }
                state.request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_HIGH as u8;
                state.request.params.earliest.timeout_us =
                    raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US;
                core::ptr::from_ref(&state.request)
            });
            let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
            if ret < 0 {
                STATE.with_inner(|state| {
                    state.counters.invalid_return += 1;
                    state.done = true;
                    state.waker.wake();
                });
            }
            core::ptr::null_mut()
        }

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED => STATE.with_inner(|state| {
            state.counters.session_closed += 1;
            state.done = true;
            state.waker.wake();
            core::ptr::null_mut()
        }),

        raw::MPSL_TIMESLOT_SIGNAL_OVERSTAYED => STATE.with_inner(|state| {
            state.counters.overstayed += 1;
            state.done = true;
            state.waker.wake();
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_INVALID_RETURN => STATE.with_inner(|state| {
            state.counters.invalid_return += 1;
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED => STATE.with_inner(|state| {
            state.counters.extend_failed += 1;
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED => STATE.with_inner(|state| {
            state.counters.extend_succeeded += 1;
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        _ => STATE.with_inner(|state| {
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),
    }
}

struct OnDrop<F: FnOnce()> {
    f: core::mem::MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    fn new(f: F) -> Self {
        Self {
            f: core::mem::MaybeUninit::new(f),
        }
    }
    fn defuse(self) {
        core::mem::forget(self);
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() };
    }
}

/// Request a single timeslot and wait for completion.
///
/// Opens a session, requests one EARLIEST timeslot of `slot_length_us` µs,
/// arms TIMER0 CC[0] at `in_slot_match_us` µs from slot start. When the
/// TIMER0 compare fires the slot is ended. Returns signal counters for
/// diagnostics.
pub async fn run_single_slot(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    slot_length_us: u32,
    in_slot_match_us: u32,
) -> Result<SignalCounters, Error> {
    let _busy = STATE.try_enter()?;

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(timeslot_callback), (&mut session_id) as *mut _)
    };
    mpsl_ok(ret)?;

    let _drop = OnDrop::new(|| {
        let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
    });

    STATE.with_inner(|state| {
        state.counters = SignalCounters::ZERO;
        state.done = false;
        state.target_count = 0; // single-shot
        state.in_slot_match_us = in_slot_match_us;
        state.request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
        state.request.params.earliest.hfclk = raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8;
        state.request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
        state.request.params.earliest.length_us = slot_length_us;
        state.request.params.earliest.timeout_us = 1_000_000;
    });

    let request = STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    mpsl_ok(ret)?;

    poll_fn(|cx| {
        STATE.with_inner(|state| {
            state.waker.register(cx.waker());
            if state.done {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    })
    .await;

    _drop.defuse();
    unsafe {
        let ret = raw::mpsl_timeslot_session_close(session_id);
        mpsl_ok(ret)?;
    }

    Ok(STATE.with_inner(|state| state.counters))
}

/// Request `count` chained timeslots in a single session.
///
/// Opens a session, requests the first EARLIEST timeslot, then chains each
/// subsequent slot via `ACTION_REQUEST` from the `SIGNAL_TIMER0` callback.
/// After all `count` slots complete, returns cumulative signal counters.
///
/// BLOCKED/CANCELLED are handled by upgrading to HIGH priority and retrying.
pub async fn run_chained_slots(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    slot_length_us: u32,
    in_slot_match_us: u32,
    count: u32,
) -> Result<SignalCounters, Error> {
    let _busy = STATE.try_enter()?;

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(timeslot_callback), (&mut session_id) as *mut _)
    };
    mpsl_ok(ret)?;

    let _drop = OnDrop::new(|| {
        let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
    });

    STATE.with_inner(|state| {
        state.counters = SignalCounters::ZERO;
        state.done = false;
        state.target_count = count;
        state.in_slot_match_us = in_slot_match_us;
        state.request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
        state.request.params.earliest.hfclk = raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8;
        state.request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
        state.request.params.earliest.length_us = slot_length_us;
        state.request.params.earliest.timeout_us = 1_000_000;
    });

    let request = STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    mpsl_ok(ret)?;

    poll_fn(|cx| {
        STATE.with_inner(|state| {
            state.waker.register(cx.waker());
            if state.done {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    })
    .await;

    _drop.defuse();
    unsafe {
        let ret = raw::mpsl_timeslot_session_close(session_id);
        mpsl_ok(ret)?;
    }

    Ok(STATE.with_inner(|state| state.counters))
}

// ---- PTX-in-timeslot ----

use crate::addresses::EsbAddresses;
use crate::config::EsbConfig;
use crate::header::EsbHeader;
use crate::radio::EsbRadio;

use core::cell::UnsafeCell;

/// Result of a PTX timeslot session.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtxSlotResult {
    pub counters: SignalCounters,
    pub tx_count: u32,
    pub ack_ok_count: u32,
    pub ack_payload_count: u32,
    pub ack_inversions: u32,
    pub last_ack_counter: u32,
}

/// Phase within a single PTX timeslot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PtxPhase {
    Idle,
    Tx,
    WaitAck,
    Done,
}

struct PtxInnerState {
    counters: SignalCounters,
    done: bool,
    waker: WakerRegistration,
    request: raw::mpsl_timeslot_request_t,
    return_param: raw::mpsl_timeslot_signal_return_param_t,
    slot_length_us: u32,
    in_slot_match_us: u32,
    request_timeout_us: u32,
    target_count: u32,
    config: Option<crate::config::EsbConfig>,
    addresses: Option<crate::addresses::EsbAddresses>,
    /// When `Some`, the timeslot callback dispatches through the
    /// shared `EsbPtx` state machine via `TimeslotPtxDriver`; when `None`,
    /// the legacy inline PTX engine runs (fallback / A-B regression path).
    driver: Option<&'static dyn crate::isr::TimeslotPtxDriver>,
    /// Cross-slot duplicate-detection state for the converged driver.
    saved_pid: [u8; 8],
    saved_crc: [u16; 8],
    saved_valid: [bool; 8],
    pid: u8,
    phase: PtxPhase,
    tx_count: u32,
    ack_ok_count: u32,
    tx_pipe: u8,
    payload_byte: u8,
    packets_per_slot: u32,
    packets_sent_this_slot: u32,
    ack_payload_count: u32,
    ack_inversions: u32,
    last_ack_counter: u32,
    schedule_hint_count: u32,
    schedule_hint_bad_count: u32,
    last_schedule_window_id: u32,
    last_schedule_hint: Option<ScheduleHint>,
    schedule_tracker: ScheduleTracker,
    schedule_gate: PtxScheduleGateConfig,
    schedule_skip_slots_remaining: u8,
    schedule_skip_count: u32,
    schedule_lock_active: bool,
    schedule_lock_miss_streak: u8,
    schedule_lock_count: u32,
    schedule_reacquire_count: u32,
    schedule_period_us: u32,
    schedule_next_distance_us: u32,
    poll_pipes: u8,
    poll_pipe_mask: u8,
    poll_report_every: u32,
    poll_slots_since_report: u32,
    poll_report_ready: bool,
    ack_ok_per_pipe: [u32; NUM_PIPES],
    tx_per_pipe: [u32; NUM_PIPES],
    retry_count: u8,
    max_retries: u8,
    acked_this_slot: bool,
    ack_timeout_us: u32,
    ack_timeout_count: u32,
    ack_crc_fail_count: u32,
    ack_timeout_per_pipe: [u32; NUM_PIPES],
    ack_crc_fail_per_pipe: [u32; NUM_PIPES],
    event_mode: bool,
    event_pending: bool,
    event_result_ready: bool,
    event_ack_ok: bool,
    last_ack_extension: Option<PrxAckExtension>,
}

// SAFETY: same invariant as InnerState — PtxInnerState is only reached through
// `Mutex<Timer0RawMutex, RefCell<_>>`, which masks the timeslot IRQ around task
// access, and the timeslot callback (the only other accessor) runs inside that
// masked window. Its fields are POD counters/flags plus raw MPSL params with no
// thread-affinity.
unsafe impl Send for PtxInnerState {}
unsafe impl Sync for PtxInnerState {}

impl PtxInnerState {
    /// Clear all per-session runtime accumulators, counters, and flags back to
    /// their initial values. Config-derived fields (slot timing, pipe, mode,
    /// retry limits, and the MPSL request) are left for the session opener to
    /// set after this call.
    fn reset_runtime(&mut self) {
        self.counters = SignalCounters::ZERO;
        self.done = false;
        self.target_count = 0;
        self.phase = PtxPhase::Idle;
        self.tx_count = 0;
        self.ack_ok_count = 0;
        self.payload_byte = 0;
        self.packets_sent_this_slot = 0;
        self.ack_payload_count = 0;
        self.ack_inversions = 0;
        self.last_ack_counter = 0;
        self.schedule_hint_count = 0;
        self.schedule_hint_bad_count = 0;
        self.last_schedule_window_id = 0;
        self.last_schedule_hint = None;
        self.schedule_tracker.reset();
        self.schedule_skip_slots_remaining = 0;
        self.schedule_skip_count = 0;
        self.schedule_lock_active = false;
        self.schedule_lock_miss_streak = 0;
        self.schedule_lock_count = 0;
        self.schedule_reacquire_count = 0;
        self.schedule_period_us = 0;
        self.schedule_next_distance_us = 0;
        self.pid = 0;
        // Converged-driver cross-slot dup state starts cleared.
        self.saved_pid = [0; 8];
        self.saved_crc = [0; 8];
        self.saved_valid = [false; 8];
        self.poll_pipes = 0;
        self.poll_pipe_mask = 0;
        self.poll_report_every = 0;
        self.poll_slots_since_report = 0;
        self.poll_report_ready = false;
        self.ack_ok_per_pipe = [0; NUM_PIPES];
        self.tx_per_pipe = [0; NUM_PIPES];
        self.ack_timeout_count = 0;
        self.ack_crc_fail_count = 0;
        self.ack_timeout_per_pipe = [0; NUM_PIPES];
        self.ack_crc_fail_per_pipe = [0; NUM_PIPES];
        self.retry_count = 0;
        self.acked_this_slot = false;
        self.event_mode = false;
        self.event_pending = false;
        self.event_result_ready = false;
        self.event_ack_ok = false;
        self.last_ack_extension = None;
    }
}

struct PtxState {
    busy: AtomicBool,
    inner: Mutex<Timer0RawMutex, RefCell<PtxInnerState>>,
}

static PTX_STATE: PtxState = PtxState::new();

#[repr(C, align(4))]
struct PtxBuffers {
    tx: UnsafeCell<[u8; 256]>,
    rx: UnsafeCell<[u8; 256]>,
}
// SAFETY: the UnsafeCell DMA buffers are only ever dereferenced from inside the
// PTX timeslot callback, which MPSL runs in a single high-priority context with
// at most one timeslot active at a time. There is no concurrent aliasing: task
// code never touches PTX_BUFS, and the radio's EasyDMA reads/writes the buffer
// only while the callback owns the slot. `#[link_section = ".data"]` keeps it in
// RAM so EasyDMA can address it. The `align(4)` satisfies the RADIO PACKETPTR
// alignment requirement.
unsafe impl Sync for PtxBuffers {}

#[unsafe(link_section = ".data")]
static PTX_BUFS: PtxBuffers = PtxBuffers {
    tx: UnsafeCell::new([0u8; 256]),
    rx: UnsafeCell::new([0u8; 256]),
};

fn ptx_set_earliest_request(state: &mut PtxInnerState, priority: u8, timeout_us: u32) {
    configure_earliest_request(
        &mut state.request,
        priority,
        state.slot_length_us,
        timeout_us,
    );
}

fn ptx_set_normal_request(state: &mut PtxInnerState, distance_us: u32) {
    configure_normal_request(
        &mut state.request,
        TIMESLOT_PRIORITY_NORMAL,
        distance_us,
        state.slot_length_us,
    );
}

fn ptx_configure_next_poll_request(state: &mut PtxInnerState) {
    if state.schedule_gate.mode == PtxScheduleMode::PhaseLocked && state.schedule_lock_active {
        let distance_us = if state.schedule_next_distance_us > 0 {
            let distance = state.schedule_next_distance_us;
            state.schedule_next_distance_us = 0;
            distance
        } else {
            state.schedule_period_us
        };

        if distance_us > 0 && distance_us <= raw::MPSL_TIMESLOT_DISTANCE_MAX_US {
            ptx_set_normal_request(state, distance_us);
            return;
        }

        state.schedule_lock_active = false;
        state.schedule_lock_miss_streak = 0;
        state.schedule_reacquire_count += 1;
    }

    ptx_set_earliest_request(state, TIMESLOT_PRIORITY_NORMAL, state.request_timeout_us);
}

fn ptx_observe_schedule_hint(state: &mut PtxInnerState, hint: ScheduleHint, ack_offset_us: u32) {
    state.schedule_hint_count += 1;
    state.last_schedule_window_id = hint.window_id;
    let elapsed_after_ack_us = state.in_slot_match_us.saturating_sub(ack_offset_us);
    state.last_schedule_hint = Some(ScheduleHint::new(
        hint.flags,
        hint.window_id,
        hint.next_delay_us.saturating_sub(elapsed_after_ack_us),
        hint.period_us,
        hint.window_us,
    ));
    state.schedule_tracker.observe_hint(hint);

    match state.schedule_gate.mode {
        PtxScheduleMode::Disabled => {}
        PtxScheduleMode::FixedSkipAfterHint => {
            state.schedule_skip_slots_remaining = state.schedule_gate.skip_after_hint_slots;
        }
        PtxScheduleMode::PhaseLocked => {
            if !state.schedule_lock_active {
                state.schedule_lock_count += 1;
            }
            state.schedule_lock_active = true;
            state.schedule_lock_miss_streak = 0;
            state.schedule_period_us = hint.period_us;

            let min_distance = state.in_slot_match_us.saturating_add(100);
            let hinted_distance = ack_offset_us
                .saturating_add(hint.next_delay_us)
                .saturating_add(state.schedule_gate.lock_tx_offset_us);
            state.schedule_next_distance_us = hinted_distance.max(min_distance);
        }
    }
}

fn ptx_observe_schedule_miss(state: &mut PtxInnerState) {
    state.schedule_tracker.observe_miss();

    if state.schedule_gate.mode == PtxScheduleMode::PhaseLocked && state.schedule_lock_active {
        state.schedule_lock_miss_streak = state.schedule_lock_miss_streak.saturating_add(1);
        if state.schedule_lock_miss_streak >= state.schedule_gate.lock_miss_limit {
            state.schedule_lock_active = false;
            state.schedule_lock_miss_streak = 0;
            state.schedule_next_distance_us = 0;
            state.schedule_reacquire_count += 1;
        }
    }
}

impl PtxState {
    const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            inner: Mutex::new(RefCell::new(PtxInnerState {
                counters: SignalCounters::ZERO,
                done: false,
                waker: WakerRegistration::new(),
                request: raw::mpsl_timeslot_request_t {
                    request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
                    params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                        earliest: raw::mpsl_timeslot_request_earliest_t {
                            hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                            priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                            length_us: 0,
                            timeout_us: 1_000_000,
                        },
                    },
                },
                return_param: raw::mpsl_timeslot_signal_return_param_t {
                    callback_action: 0,
                    params: raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1 {
                        request:
                            raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1__bindgen_ty_1 {
                                p_next: core::ptr::null_mut(),
                            },
                    },
                },
                slot_length_us: 0,
                in_slot_match_us: 5500,
                request_timeout_us: 1_000_000,
                target_count: 0,
                config: None,
                addresses: None,
                driver: None,
                saved_pid: [0; 8],
                saved_crc: [0; 8],
                saved_valid: [false; 8],
                pid: 0,
                phase: PtxPhase::Idle,
                tx_count: 0,
                ack_ok_count: 0,
                tx_pipe: 0,
                payload_byte: 0,
                packets_per_slot: 1,
                packets_sent_this_slot: 0,
                ack_payload_count: 0,
                ack_inversions: 0,
                last_ack_counter: 0,
                schedule_hint_count: 0,
                schedule_hint_bad_count: 0,
                last_schedule_window_id: 0,
                last_schedule_hint: None,
                schedule_tracker: ScheduleTracker::new(),
                schedule_gate: PtxScheduleGateConfig::disabled(),
                schedule_skip_slots_remaining: 0,
                schedule_skip_count: 0,
                schedule_lock_active: false,
                schedule_lock_miss_streak: 0,
                schedule_lock_count: 0,
                schedule_reacquire_count: 0,
                schedule_period_us: 0,
                schedule_next_distance_us: 0,
                poll_pipes: 0,
                poll_pipe_mask: 0,
                poll_report_every: 0,
                poll_slots_since_report: 0,
                poll_report_ready: false,
                ack_ok_per_pipe: [0; NUM_PIPES],
                tx_per_pipe: [0; NUM_PIPES],
                retry_count: 0,
                max_retries: 3,
                acked_this_slot: false,
                ack_timeout_us: 600,
                ack_timeout_count: 0,
                ack_crc_fail_count: 0,
                ack_timeout_per_pipe: [0; NUM_PIPES],
                ack_crc_fail_per_pipe: [0; NUM_PIPES],
                event_mode: false,
                event_pending: false,
                event_result_ready: false,
                event_ack_ok: false,
                last_ack_extension: None,
            })),
        }
    }

    fn with_inner<F: FnOnce(&mut PtxInnerState) -> R, R>(&self, f: F) -> R {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            f(&mut inner)
        })
    }

    fn try_enter(&'static self) -> Result<BusyGuard, Error> {
        BusyGuard::new(&self.busy)
    }
}

// SAFETY: same MPSL contract as `timeslot_callback` — called from the
// high-priority timeslot signal context with a valid `session_id`/`signal`,
// must stay deterministic and minimal, and returns either a pointer into the
// static `PTX_STATE` return_param or null. The PTX_BUFS EasyDMA buffers it
// programs are only ever touched from this single-context callback (see the
// `unsafe impl Sync for PtxBuffers` note).
unsafe extern "C" fn ptx_timeslot_callback(
    session_id: u8,
    signal: u32,
) -> *mut raw::mpsl_timeslot_signal_return_param_t {
    match signal {
        raw::MPSL_TIMESLOT_SIGNAL_START => PTX_STATE.with_inner(|state| {
            state.counters.start += 1;

            if state.event_mode && !state.event_pending {
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                return &mut state.return_param as *mut _;
            }

            state.phase = PtxPhase::Tx;

            let (Some(config), Some(addresses)) = (state.config.as_ref(), state.addresses.as_ref())
            else {
                state.counters.invalid_return += 1;
                state.done = true;
                state.waker.wake();
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                return &mut state.return_param as *mut _;
            };

            // In poll mode, advance to next pipe in mask.
            if state.poll_pipes > 0
                && let Some(next) = next_pipe_in_mask(state.tx_pipe, state.poll_pipe_mask)
            {
                state.tx_pipe = next;
            }

            if state.schedule_gate.mode == PtxScheduleMode::FixedSkipAfterHint
                && state.schedule_skip_slots_remaining > 0
            {
                state.schedule_skip_slots_remaining -= 1;
                state.schedule_skip_count += 1;
                state.phase = PtxPhase::Done;

                let t = pac::TIMER0;
                t.events_compare(0).write_value(0);
                t.events_compare(1).write_value(0);
                t.cc(0).write_value(state.in_slot_match_us);
                t.cc(1).write_value(0xFFFFFFFF);
                t.intenset().write(|w| {
                    w.set_compare(0, true);
                    w.set_compare(1, false);
                });

                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                return &mut state.return_param as *mut _;
            }

            // ---- Converged-driver path (S6) ----
            // When a TimeslotPtxDriver is registered, the shared PtxStateMachine
            // owns RADIO register programming, PID, and retransmit logic. This
            // layer only arms TIMER0 (slot end + ACK timeout) and unmasks RADIO
            // NVIC so events arrive as SIGNAL_RADIO.
            if let Some(driver) = state.driver {
                let _ = config; // addresses already recorded in driver via set_ptx_driver
                let _ = addresses;

                // Drive the shared SM: power-cycle, re-init, restore dup state,
                // arm first TX. The SM dequeues from PacketPool.
                if driver
                    .ts_start_tx(state.saved_pid, state.saved_crc, state.saved_valid)
                    .is_err()
                {
                    state.counters.invalid_return += 1;
                    state.phase = PtxPhase::Done;
                    state.return_param.callback_action =
                        raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                    let t = pac::TIMER0;
                    t.cc(0).write_value(state.in_slot_match_us);
                    t.intenset().write(|w| w.set_compare(0, true));
                    return &mut state.return_param as *mut _;
                }

                state.tx_count = state.tx_count.wrapping_add(1);
                state.packets_sent_this_slot = 1;
                state.retry_count = 0;
                state.acked_this_slot = false;
                let pipe = driver.ts_last_pipe() as usize;
                if pipe < NUM_PIPES {
                    state.tx_per_pipe[pipe] += 1;
                }

                // Arm TIMER0 CC[0] for slot end, CC[1] for ACK timeout. The SM
                // is now in Tx (TX ramping); when RADIO END fires, SIGNAL_RADIO
                // will call ts_on_radio(false) and the SM will enter WaitAck.
                let t = pac::TIMER0;
                t.events_compare(0).write_value(0);
                t.events_compare(1).write_value(0);
                t.cc(0).write_value(state.in_slot_match_us);
                t.cc(1).write_value(0xFFFFFFFF);
                t.intenset().write(|w| {
                    w.set_compare(0, true);
                    w.set_compare(1, true);
                });

                // Unmask RADIO so events are delivered as SIGNAL_RADIO.
                unsafe {
                    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::RADIO);
                }

                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                return &mut state.return_param as *mut _;
            }

            // ---- Legacy inline-engine path (fallback) ----
            // Power cycle RADIO.
            let r = pac::RADIO;
            r.power().write(|w| w.set_power(false));
            r.power().write(|w| w.set_power(true));

            // Full ESB register init.
            let mut radio = crate::radio::EsbRadio::new(pac::RADIO);
            radio.init(config, addresses);
            radio.restore_pid_state([state.pid; 8]);

            // Prepare TX buffer. Event mode keeps the payload staged by send().
            let tx_buf = unsafe { &mut *PTX_BUFS.tx.get() };
            let dma_off = crate::header::EsbHeader::DMA_OFFSET;
            if !state.event_mode {
                let counter = state.tx_count;
                write_counter_packet(tx_buf, state.pid, counter);
            }

            // Arm TIMER0 CC[0] for slot end, CC[1] for ACK timeout.
            let t = pac::TIMER0;
            t.events_compare(0).write_value(0);
            t.events_compare(1).write_value(0);
            t.cc(0).write_value(state.in_slot_match_us);
            t.cc(1).write_value(0xFFFFFFFF);
            t.intenset().write(|w| {
                w.set_compare(0, true);
                w.set_compare(1, true);
            });

            // Trigger TX with ACK.
            let dma_ptr = unsafe { tx_buf.as_mut_ptr().add(dma_off) };
            radio.transmit(state.tx_pipe, dma_ptr, true);
            state.tx_count += 1;
            state.packets_sent_this_slot = 1;
            state.retry_count = 0;
            state.acked_this_slot = false;
            if state.poll_pipes > 0 {
                let pipe = state.tx_pipe as usize;
                if pipe < NUM_PIPES {
                    state.tx_per_pipe[pipe] += 1;
                }
            }

            // MPSL hands the app the RADIO during a timeslot but leaves the
            // NVIC line masked; the app must unmask it so RADIO events are
            // delivered back as SIGNAL_RADIO. Without this, rd stays 0 and no
            // RX/ACK is ever processed.
            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::RADIO);
            }

            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_RADIO => PTX_STATE.with_inner(|state| {
            state.counters.radio += 1;

            // ---- Converged-driver path (S6) ----
            if let Some(driver) = state.driver {
                let r = pac::RADIO;
                let disabled = r.events_disabled().read() == 1;
                if disabled {
                    r.events_disabled().write_value(0);
                }

                // timer_flag=false: this is a RADIO event, not a TIMER0 ACK-timeout.
                let event = driver.ts_on_radio(false);
                let sm_state = driver.ts_state();

                // Update mpsl-layer counters from the SM event.
                use crate::state_machine::{PtxEvent, StatePtx};
                match (event, sm_state) {
                    // Tx END → SM entered WaitAck. Arm TIMER0 CC[1] ACK timeout.
                    (PtxEvent::None, StatePtx::WaitAck) => {
                        let t = pac::TIMER0;
                        t.tasks_capture(1).write_value(1);
                        let now = t.cc(1).read();
                        t.cc(1).write_value(now.wrapping_add(state.ack_timeout_us));
                        t.events_compare(1).write_value(0);
                        t.intenset().write(|w| w.set_compare(1, true));
                    }
                    // ACK received OK → SM advanced PID, released TX, sent next or idle.
                    // Also covers Tx/TxNoAck (SM dequeued next packet) and Idle (queue empty).
                    (PtxEvent::None, StatePtx::Idle | StatePtx::Tx | StatePtx::TxNoAck) => {
                        state.acked_this_slot = true;
                        state.ack_ok_count += 1;
                        let pipe = driver.ts_last_pipe() as usize;
                        if pipe < NUM_PIPES {
                            state.ack_ok_per_pipe[pipe] += 1;
                        }
                        // Disarm TIMER0 ACK-timeout (slot-end CC[0] still armed).
                        let t = pac::TIMER0;
                        t.intenclr().write(|w| w.set_compare(1, true));
                        t.events_compare(1).write_value(0);
                        state.phase = PtxPhase::Done;
                    }
                    // Max retransmit attempts reached — packet dropped.
                    (PtxEvent::MaxAttempts, _) => {
                        state.phase = PtxPhase::Done;
                        let t = pac::TIMER0;
                        t.intenclr().write(|w| w.set_compare(1, true));
                    }
                    // WaitRetransmit cannot occur in timeslot mode (EsbTimer ISR
                    // is not active; retransmit timing is driven by TIMER0 CC[1]
                    // via the SIGNAL_TIMER0 path with timer_flag=true). No-op.
                    (PtxEvent::None, StatePtx::WaitRetransmit) => {}
                }

                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                return &mut state.return_param as *mut _;
            }

            // ---- Legacy inline-engine path (fallback) ----
            let r = pac::RADIO;
            let disabled = r.events_disabled().read() == 1;

            match state.phase {
                PtxPhase::Tx if disabled => {
                    r.events_disabled().write_value(0);

                    let rx_buf = unsafe { &mut *PTX_BUFS.rx.get() };
                    let dma_ptr = unsafe {
                        rx_buf
                            .as_mut_ptr()
                            .add(crate::header::EsbHeader::DMA_OFFSET)
                    };

                    r.events_ready().write_value(0);
                    compiler_fence(Ordering::Release);
                    r.packetptr().write_value(dma_ptr as u32);
                    r.shorts().modify(|w| w.set_disabled_rxen(false));

                    let t = pac::TIMER0;
                    t.tasks_capture(1).write_value(1);
                    let now = t.cc(1).read();
                    t.cc(1).write_value(now.wrapping_add(state.ack_timeout_us));
                    t.events_compare(1).write_value(0);
                    t.intenset().write(|w| w.set_compare(1, true));

                    state.phase = PtxPhase::WaitAck;
                }
                PtxPhase::WaitAck if disabled => {
                    r.events_disabled().write_value(0);

                    let crc_ok =
                        r.crcstatus().read().crcstatus() == pac::radio::vals::Crcstatus::CRCOK;
                    compiler_fence(Ordering::Acquire);

                    if crc_ok {
                        let t = pac::TIMER0;
                        t.intenclr().write(|w| w.set_compare(1, true));
                        t.events_compare(1).write_value(0);

                        state.ack_ok_count += 1;
                        state.acked_this_slot = true;
                        if state.poll_pipes > 0 {
                            let pipe = state.tx_pipe as usize;
                            if pipe < NUM_PIPES {
                                state.ack_ok_per_pipe[pipe] += 1;
                            }
                        }

                        let rx_buf = unsafe { &*PTX_BUFS.rx.get() };
                        if let Some(counter) = read_counter_payload(rx_buf) {
                            state.ack_payload_count += 1;
                            if counter < state.last_ack_counter {
                                state.ack_inversions += 1;
                            }
                            state.last_ack_counter = counter;

                            let dma = crate::header::EsbHeader::DMA_OFFSET;
                            let payload = crate::header::EsbHeader::PAYLOAD_OFFSET;
                            let len = rx_buf[dma] as usize;
                            let end = payload.saturating_add(len);
                            if end <= rx_buf.len() {
                                let t = pac::TIMER0;
                                t.tasks_capture(2).write_value(1);
                                let ack_offset_us = t.cc(2).read();
                                let ack_payload = &rx_buf[payload..end];
                                match decode_counter_payload_schedule_hint(&rx_buf[payload..end]) {
                                    Ok(hint) => {
                                        ptx_observe_schedule_hint(state, hint, ack_offset_us)
                                    }
                                    Err(_) => {
                                        state.schedule_hint_bad_count += 1;
                                        state.schedule_tracker.observe_bad_hint();
                                    }
                                }
                                let extension_start =
                                    crate::mpsl_schedule::SCHEDULE_COUNTER_HINT_PAYLOAD_LEN;
                                if ack_payload.len() > extension_start {
                                    state.last_ack_extension = PrxAckExtension::from_slice(
                                        &ack_payload[extension_start..],
                                    );
                                }
                            } else {
                                state.schedule_hint_bad_count += 1;
                                state.schedule_tracker.observe_bad_hint();
                            }
                        } else {
                            state.schedule_hint_bad_count += 1;
                            state.schedule_tracker.observe_bad_hint();
                        }
                    }

                    if !crc_ok {
                        state.ack_crc_fail_count += 1;
                        ptx_observe_schedule_miss(state);
                        let pipe = state.tx_pipe as usize;
                        if pipe < NUM_PIPES {
                            state.ack_crc_fail_per_pipe[pipe] += 1;
                        }
                    }

                    if state.acked_this_slot {
                        state.pid = advance_pid(state.pid);
                        state.payload_byte = state.payload_byte.wrapping_add(1);
                    }
                    if disable_radio_bounded(false, false).timed_out() {
                        state.counters.radio_disable_timeout += 1;
                    }

                    state.phase = PtxPhase::Done;
                }
                _ => {}
            }

            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_TIMER0 => PTX_STATE.with_inner(|state| {
            state.counters.timer0 += 1;

            let t = pac::TIMER0;

            // ---- Converged-driver path (S6) ----
            if let Some(driver) = state.driver {
                // CC[1]: ACK timeout. Feed timer_flag=true to the SM so it
                // enters its retransmit/MaxAttempts path. The SM itself calls
                // radio.stop() and radio.transmit() to re-arm the TX.
                if t.events_compare(1).read() == 1 {
                    t.events_compare(1).write_value(0);
                    t.intenclr().write(|w| w.set_compare(1, true));

                    state.ack_timeout_count += 1;
                    ptx_observe_schedule_miss(state);
                    let pipe = driver.ts_last_pipe() as usize;
                    if pipe < NUM_PIPES {
                        state.ack_timeout_per_pipe[pipe] += 1;
                    }

                    let event = driver.ts_on_radio(true);
                    let sm_state = driver.ts_state();
                    use crate::state_machine::{PtxEvent, StatePtx};

                    match (event, sm_state) {
                        // SM retransmitted — back in Tx. Re-arm CC[1] for the
                        // next ACK wait window.
                        (PtxEvent::None, StatePtx::Tx) => {
                            state.retry_count = driver.ts_attempts();
                            t.tasks_capture(1).write_value(1);
                            let now = t.cc(1).read();
                            t.cc(1).write_value(now.wrapping_add(state.ack_timeout_us));
                            t.events_compare(1).write_value(0);
                            t.intenset().write(|w| w.set_compare(1, true));
                        }
                        // Max retransmit reached — packet dropped.
                        (PtxEvent::MaxAttempts, _) => {
                            state.phase = PtxPhase::Done;
                        }
                        // Any other state (Idle, TxNoAck, WaitAck with late ACK
                        // arrival, etc.) — end this slot's attempt.
                        _ => {
                            state.phase = PtxPhase::Done;
                        }
                    }
                }

                // CC[0]: slot end. Force-stop the SM and reschedule.
                if t.events_compare(0).read() == 1 {
                    t.events_compare(0).write_value(0);
                    t.intenclr().write(|w| {
                        w.set_compare(0, true);
                        w.set_compare(1, true);
                    });

                    if state.phase != PtxPhase::Done {
                        driver.ts_force_stop();
                        // Save dup state for next slot.
                        let (p, c, v) = driver.ts_snapshot_dup_state();
                        state.saved_pid = p;
                        state.saved_crc = c;
                        state.saved_valid = v;
                        state.phase = PtxPhase::Done;
                    } else {
                        // Already Done — still capture dup state.
                        let (p, c, v) = driver.ts_snapshot_dup_state();
                        state.saved_pid = p;
                        state.saved_crc = c;
                        state.saved_valid = v;
                    }

                    let poll_active = state.poll_pipes > 0;
                    let event_active = state.event_mode;

                    if event_active {
                        state.event_result_ready = true;
                        state.event_pending = false;
                        state.event_ack_ok = state.acked_this_slot;
                        state.waker.wake();
                        state.return_param.callback_action =
                            raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                    } else if poll_active {
                        state.poll_slots_since_report += 1;
                        if state.poll_slots_since_report >= state.poll_report_every {
                            state.poll_slots_since_report = 0;
                            state.poll_report_ready = true;
                            state.waker.wake();
                        }
                        ptx_configure_next_poll_request(state);
                        state.return_param.callback_action =
                            raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                        state.return_param.params.request.p_next =
                            core::ptr::from_mut(&mut state.request);
                    } else {
                        let chain =
                            state.target_count > 0 && state.counters.start < state.target_count;
                        if chain {
                            state.return_param.callback_action =
                                raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                            state.return_param.params.request.p_next =
                                core::ptr::from_mut(&mut state.request);
                        } else {
                            state.done = true;
                            state.waker.wake();
                            state.return_param.callback_action =
                                raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                        }
                    }
                }

                return &mut state.return_param as *mut _;
            }

            // ---- Legacy inline-engine path (fallback) ----
            // Check CC[1] first: ACK timeout → retry.
            if t.events_compare(1).read() == 1 && state.phase == PtxPhase::WaitAck {
                t.events_compare(1).write_value(0);
                state.ack_timeout_count += 1;
                ptx_observe_schedule_miss(state);
                let pipe = state.tx_pipe as usize;
                if pipe < NUM_PIPES {
                    state.ack_timeout_per_pipe[pipe] += 1;
                }

                let r = pac::RADIO;
                if disable_radio_bounded(true, false).timed_out() {
                    state.counters.radio_disable_timeout += 1;
                }

                if state.retry_count < state.max_retries {
                    state.retry_count += 1;
                    state.phase = PtxPhase::Tx;

                    let tx_buf = unsafe { &mut *PTX_BUFS.tx.get() };
                    let dma_off = crate::header::EsbHeader::DMA_OFFSET;
                    if !state.event_mode {
                        write_counter_packet(tx_buf, state.pid, state.tx_count.wrapping_sub(1));
                    }

                    r.shorts().modify(|w| w.set_disabled_rxen(true));
                    r.intenset().write(|w| w.set_disabled(true));
                    r.txaddress().write(|w| w.set_txaddress(state.tx_pipe));
                    r.rxaddresses()
                        .write_value(pac::radio::regs::Rxaddresses(1 << state.tx_pipe));
                    let dma_ptr = unsafe { tx_buf.as_mut_ptr().add(dma_off) };
                    r.packetptr().write_value(dma_ptr as u32);
                    r.events_address().write_value(0);
                    r.events_disabled().write_value(0);
                    r.events_ready().write_value(0);
                    r.events_end().write_value(0);
                    r.events_payload().write_value(0);
                    compiler_fence(Ordering::Release);
                    r.tasks_txen().write_value(1);

                    t.tasks_capture(1).write_value(1);
                    let now = t.cc(1).read();
                    t.cc(1).write_value(now.wrapping_add(state.ack_timeout_us));
                    t.events_compare(1).write_value(0);
                    t.intenset().write(|w| w.set_compare(1, true));

                    state.return_param.callback_action =
                        raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                    return &mut state.return_param as *mut _;
                } else {
                    state.phase = PtxPhase::Done;
                    t.intenclr().write(|w| w.set_compare(1, true));
                }
            }

            // CC[0]: slot end.
            if t.events_compare(0).read() == 1 {
                t.events_compare(0).write_value(0);
                t.intenclr().write(|w| {
                    w.set_compare(0, true);
                    w.set_compare(1, true);
                });

                if state.phase != PtxPhase::Done && state.phase != PtxPhase::Idle {
                    if disable_radio_bounded(true, true).timed_out() {
                        state.counters.radio_disable_timeout += 1;
                    }
                    state.phase = PtxPhase::Done;
                }

                let poll_active = state.poll_pipes > 0;
                let event_active = state.event_mode;

                if event_active {
                    state.event_result_ready = true;
                    state.event_pending = false;
                    state.event_ack_ok = state.acked_this_slot;
                    state.waker.wake();
                    state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                } else if poll_active {
                    state.poll_slots_since_report += 1;
                    if state.poll_slots_since_report >= state.poll_report_every {
                        state.poll_slots_since_report = 0;
                        state.poll_report_ready = true;
                        state.waker.wake();
                    }

                    ptx_configure_next_poll_request(state);
                    state.return_param.callback_action =
                        raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                    state.return_param.params.request.p_next =
                        core::ptr::from_mut(&mut state.request);
                } else {
                    let chain = state.target_count > 0 && state.counters.start < state.target_count;
                    if chain {
                        state.return_param.callback_action =
                            raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                        state.return_param.params.request.p_next =
                            core::ptr::from_mut(&mut state.request);
                    } else {
                        state.done = true;
                        state.waker.wake();
                        state.return_param.callback_action =
                            raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                    }
                }
            }

            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_IDLE => {
            let request = PTX_STATE.with_inner(|state| {
                state.counters.session_idle += 1;
                if state.poll_pipes > 0 {
                    if state.schedule_gate.mode == PtxScheduleMode::PhaseLocked {
                        state.schedule_lock_active = false;
                        state.schedule_lock_miss_streak = 0;
                        state.schedule_next_distance_us = 0;
                        state.schedule_reacquire_count += 1;
                    }
                    ptx_set_earliest_request(
                        state,
                        TIMESLOT_PRIORITY_NORMAL,
                        state.request_timeout_us,
                    );
                    state.poll_report_ready = true;
                    state.waker.wake();
                    Some(core::ptr::from_ref(&state.request))
                } else {
                    state.waker.wake();
                    None
                }
            });

            if let Some(request) = request {
                let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
                if ret < 0 {
                    PTX_STATE.with_inner(|state| {
                        state.counters.invalid_return += 1;
                        state.done = true;
                        state.waker.wake();
                    });
                }
            }

            core::ptr::null_mut()
        }

        raw::MPSL_TIMESLOT_SIGNAL_BLOCKED | raw::MPSL_TIMESLOT_SIGNAL_CANCELLED => {
            let request = PTX_STATE.with_inner(|state| {
                if signal == raw::MPSL_TIMESLOT_SIGNAL_BLOCKED {
                    state.counters.blocked += 1;
                } else {
                    state.counters.cancelled += 1;
                }
                if state.schedule_gate.mode == PtxScheduleMode::PhaseLocked
                    && state.schedule_lock_active
                {
                    state.schedule_lock_active = false;
                    state.schedule_lock_miss_streak = 0;
                    state.schedule_next_distance_us = 0;
                    state.schedule_reacquire_count += 1;
                }
                ptx_set_earliest_request(
                    state,
                    TIMESLOT_PRIORITY_HIGH,
                    raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
                );
                core::ptr::from_ref(&state.request)
            });
            let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
            if ret < 0 {
                PTX_STATE.with_inner(|state| {
                    state.counters.invalid_return += 1;
                    #[cfg(feature = "defmt")]
                    defmt::warn!(
                        "ptx blocked/cancelled rerequest failed ret={} sig={} cnt s={} t0={} rd={} bk={} cn={} idle={} closed={} over={} inv={}",
                        ret,
                        signal,
                        state.counters.start,
                        state.counters.timer0,
                        state.counters.radio,
                        state.counters.blocked,
                        state.counters.cancelled,
                        state.counters.session_idle,
                        state.counters.session_closed,
                        state.counters.overstayed,
                        state.counters.invalid_return,
                    );
                    state.done = true;
                    state.waker.wake();
                });
            }
            core::ptr::null_mut()
        }

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED => PTX_STATE.with_inner(|state| {
            state.counters.session_closed += 1;
            state.done = true;
            state.waker.wake();
            core::ptr::null_mut()
        }),

        raw::MPSL_TIMESLOT_SIGNAL_OVERSTAYED => PTX_STATE.with_inner(|state| {
            state.counters.overstayed += 1;
            // Converged-driver: force SM to quiescent state before rescheduling.
            if let Some(driver) = state.driver {
                driver.ts_force_stop();
                let (p, c, v) = driver.ts_snapshot_dup_state();
                state.saved_pid = p;
                state.saved_crc = c;
                state.saved_valid = v;
            }
            state.phase = PtxPhase::Idle;
            if state.poll_pipes > 0 {
                state.schedule_lock_active = false;
                state.schedule_lock_miss_streak = 0;
                state.schedule_next_distance_us = 0;
                state.schedule_reacquire_count += 1;
                ptx_set_earliest_request(
                    state,
                    TIMESLOT_PRIORITY_HIGH,
                    raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
                );
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
                state.return_param.params.request.p_next = core::ptr::from_mut(&mut state.request);
            } else {
                if state.event_mode {
                    state.event_result_ready = true;
                    state.event_pending = false;
                    state.event_ack_ok = false;
                    state.waker.wake();
                    state.return_param.callback_action =
                        raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                } else {
                    state.done = true;
                    state.waker.wake();
                    state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                }
            }
            &mut state.return_param as *mut _
        }),

        _ => PTX_STATE.with_inner(|state| {
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),
    }
}

/// Run PTX transmissions inside chained timeslots.
///
/// Each timeslot sends exactly 1 ACK packet with an incrementing payload.
/// After `count` slots, returns cumulative results including ACK stats.
/// PID is saved/restored across slot boundaries.
#[allow(clippy::too_many_arguments)]
pub async fn run_ptx_slots(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &crate::config::EsbConfig,
    addresses: &crate::addresses::EsbAddresses,
    slot_length_us: u32,
    in_slot_match_us: u32,
    count: u32,
    tx_pipe: u8,
    packets_per_slot: u32,
) -> Result<PtxSlotResult, Error> {
    let _busy = PTX_STATE.try_enter()?;
    if config.payload_length < 4 {
        return Err(Error::InvalidParam);
    }

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(ptx_timeslot_callback), (&mut session_id) as *mut _)
    };
    mpsl_ok(ret)?;

    let _drop = OnDrop::new(|| {
        let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
    });

    PTX_STATE.with_inner(|state| {
        state.reset_runtime();

        state.target_count = count;
        state.slot_length_us = slot_length_us;
        state.in_slot_match_us = in_slot_match_us;
        state.request_timeout_us = 1_000_000;
        state.config = Some(config.clone());
        state.addresses = Some(addresses.clone());
        state.schedule_gate = PtxScheduleGateConfig::disabled();
        state.tx_pipe = tx_pipe;
        state.packets_per_slot = packets_per_slot;
        ptx_set_earliest_request(state, TIMESLOT_PRIORITY_NORMAL, 1_000_000);
    });

    let request = PTX_STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    mpsl_ok(ret)?;

    poll_fn(|cx| {
        PTX_STATE.with_inner(|state| {
            state.waker.register(cx.waker());
            if state.done {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    })
    .await;

    _drop.defuse();
    unsafe {
        let ret = raw::mpsl_timeslot_session_close(session_id);
        mpsl_ok(ret)?;
    }

    Ok(PTX_STATE.with_inner(|state| PtxSlotResult {
        counters: state.counters,
        tx_count: state.tx_count,
        ack_ok_count: state.ack_ok_count,
        ack_payload_count: state.ack_payload_count,
        ack_inversions: state.ack_inversions,
        last_ack_counter: state.last_ack_counter,
    }))
}

// ---- PRX-in-timeslot ----

/// Result of a PRX timeslot session.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PrxSlotResult {
    pub counters: SignalCounters,
    pub rx_count: u32,
    pub dup_count: u32,
    pub bad_crc_count: u32,
    pub rx_per_pipe: [u32; NUM_PIPES],
    pub dup_per_pipe: [u32; NUM_PIPES],
    pub bad_crc_per_pipe: [u32; NUM_PIPES],
    pub ack_tx_per_pipe: [u32; NUM_PIPES],
    pub slot_length_us: u32,
    pub in_slot_match_us: u32,
    pub report_every: u32,
    pub phase: u8,
    pub slot_active: bool,
    pub last_request_kind: u8,
    pub normal_blocked: u32,
    pub earliest_blocked: u32,
    pub normal_cancelled: u32,
    pub earliest_cancelled: u32,
}

/// One raw application payload received by a long-lived PRX timeslot session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PrxReceivedEvent {
    pub pipe: u8,
    pub len: u8,
    pub payload: [u8; 32],
}

const PRX_REQ_EARLIEST: u8 = 1;
const PRX_REQ_NORMAL: u8 = 2;
const PRX_EVENT_QUEUE_CAP: usize = 32;

/// Phase within a single PRX timeslot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PrxPhase {
    Idle,
    Receiving,
    TxAck,
    TxRepeatedAck,
}

impl PrxPhase {
    const fn as_u8(self) -> u8 {
        match self {
            Self::Idle => 0,
            Self::Receiving => 1,
            Self::TxAck => 2,
            Self::TxRepeatedAck => 3,
        }
    }
}

struct PrxInnerState {
    counters: SignalCounters,
    done: bool,
    waker: WakerRegistration,
    request: raw::mpsl_timeslot_request_t,
    return_param: raw::mpsl_timeslot_signal_return_param_t,
    slot_length_us: u32,
    in_slot_match_us: u32,
    request_timeout_us: u32,
    target_count: u32,
    config: Option<EsbConfig>,
    addresses: Option<EsbAddresses>,
    /// Converged-engine driver (S3/D6). When set, the PRX callback drives the
    /// shared `EsbPrx` state machine through `TimeslotPrxDriver`; when `None`,
    /// the legacy inline-engine path runs.
    driver: Option<&'static dyn crate::isr::TimeslotPrxDriver>,
    /// Duplicate-detection state preserved across slots. In converged mode the
    /// SM owns dup detection during a slot; we snapshot at slot end and restore
    /// at the next slot start.
    saved_pid: [u8; NUM_PIPES],
    saved_crc: [u16; NUM_PIPES],
    saved_valid: [bool; NUM_PIPES],
    /// Legacy inline-engine phase. Only used when `driver` is `None`.
    phase: PrxPhase,
    /// Legacy inline-engine counters / dup-detection. Removed in S9 once the
    /// converged path is the only one.
    rx_count: u32,
    dup_count: u32,
    bad_crc_count: u32,
    last_pid: [u8; NUM_PIPES],
    last_crc: [u16; NUM_PIPES],
    last_valid: [bool; NUM_PIPES],
    enabled_pipes: u8,
    recovery_policy: RadioRecoveryPolicy,
    slot_active: bool,
    extend_count: u32,
    max_extends: u32,
    extend_length_us: u32,
    prx_extend_mode: bool,
    ack_counter: [u32; NUM_PIPES],
    rx_per_pipe: [u32; NUM_PIPES],
    dup_per_pipe: [u32; NUM_PIPES],
    bad_crc_per_pipe: [u32; NUM_PIPES],
    ack_tx_per_pipe: [u32; NUM_PIPES],
    report_every: u32,
    last_report_start: u32,
    report_ready: bool,
    prx_schedule: PrxScheduleConfig,
    prx_windows_since_gap: u32,
    /// Whether a completed PRX window should immediately request another one.
    /// Parked sessions leave this false and are resumed explicitly by the
    /// application, for example after a BLE radio notification.
    auto_reschedule: bool,
    /// An earliest request is queued with MPSL or its granted slot is active.
    /// This prevents callers from queuing overlapping manual PRX windows.
    request_outstanding: bool,
    retry_blocked_at_high_priority: bool,
    last_request_kind: u8,
    normal_blocked: u32,
    earliest_blocked: u32,
    normal_cancelled: u32,
    earliest_cancelled: u32,
    event_queue: [PrxReceivedEvent; PRX_EVENT_QUEUE_CAP],
    event_head: usize,
    event_tail: usize,
    event_len: usize,
    event_dropped: u32,
    ack_extension_provider: Option<PrxAckExtensionProvider>,
    ack_extensions: [PrxAckExtension; NUM_PIPES],
}

// SAFETY: same invariant as InnerState/PtxInnerState — PrxInnerState is reached
// only through `Mutex<Timer0RawMutex, RefCell<_>>` (task access masks the
// timeslot IRQ) and the PRX timeslot callback runs inside that masked window.
// Fields are POD counters/flags plus raw MPSL params with no thread-affinity.
unsafe impl Send for PrxInnerState {}
unsafe impl Sync for PrxInnerState {}

impl PrxInnerState {
    /// Clear all per-session runtime accumulators, counters, and per-pipe state
    /// back to their initial values. Config-derived fields (slot timing, pipe
    /// enables, schedule, recovery policy, report cadence, and the MPSL request)
    /// are left for the session opener to set after this call.
    fn reset_runtime(&mut self) {
        self.counters = SignalCounters::ZERO;
        self.done = false;
        self.target_count = 0;
        self.driver = None;
        self.saved_pid = [0; NUM_PIPES];
        self.saved_crc = [0; NUM_PIPES];
        self.saved_valid = [false; NUM_PIPES];
        self.phase = PrxPhase::Idle;
        self.rx_count = 0;
        self.dup_count = 0;
        self.bad_crc_count = 0;
        self.last_pid = [0; NUM_PIPES];
        self.last_crc = [0; NUM_PIPES];
        self.last_valid = [false; NUM_PIPES];
        self.slot_active = false;
        self.extend_count = 0;
        self.prx_extend_mode = false;
        self.ack_counter = [0; NUM_PIPES];
        self.rx_per_pipe = [0; NUM_PIPES];
        self.dup_per_pipe = [0; NUM_PIPES];
        self.bad_crc_per_pipe = [0; NUM_PIPES];
        self.ack_tx_per_pipe = [0; NUM_PIPES];
        self.last_report_start = 0;
        self.report_ready = false;
        self.prx_windows_since_gap = 0;
        self.auto_reschedule = true;
        self.request_outstanding = false;
        self.last_request_kind = PRX_REQ_EARLIEST;
        self.normal_blocked = 0;
        self.earliest_blocked = 0;
        self.normal_cancelled = 0;
        self.earliest_cancelled = 0;
        self.event_head = 0;
        self.event_tail = 0;
        self.event_len = 0;
        self.event_dropped = 0;
        self.ack_extensions = [PrxAckExtension::EMPTY; NUM_PIPES];
    }

    fn push_event(&mut self, pipe: u8, payload: &[u8]) -> bool {
        if payload.len() > 32 {
            // Payload exceeds the fixed 32-byte event slot — drop it (counted)
            // rather than silently truncating, which would let the receiver act
            // on partial data. The 32-byte cap is a known PrxReceivedEvent limit.
            self.event_dropped = self.event_dropped.saturating_add(1);
            return false;
        }
        if self.event_len == PRX_EVENT_QUEUE_CAP {
            self.event_dropped = self.event_dropped.saturating_add(1);
            return false;
        }

        let len = payload.len();
        let mut event = PrxReceivedEvent {
            pipe,
            len: len as u8,
            payload: [0; 32],
        };
        event.payload[..len].copy_from_slice(&payload[..len]);

        self.event_queue[self.event_tail] = event;
        self.event_tail = (self.event_tail + 1) % PRX_EVENT_QUEUE_CAP;
        self.event_len += 1;
        self.waker.wake();
        true
    }

    fn pop_event(&mut self) -> Option<PrxReceivedEvent> {
        if self.event_len == 0 {
            return None;
        }

        let event = self.event_queue[self.event_head];
        self.event_head = (self.event_head + 1) % PRX_EVENT_QUEUE_CAP;
        self.event_len -= 1;
        Some(event)
    }

    fn observe_payload(&mut self, pipe: usize, payload: &[u8], queued: bool) {
        if pipe >= NUM_PIPES {
            return;
        }

        self.ack_extensions[pipe] = self
            .ack_extension_provider
            .map(|provider| provider(pipe as u8, payload, queued))
            .unwrap_or(PrxAckExtension::EMPTY);
    }

    fn ack_extension_for_pipe(&self, pipe: usize) -> &[u8] {
        self.ack_extensions
            .get(pipe)
            .map(PrxAckExtension::as_slice)
            .unwrap_or(&[])
    }
}

struct PrxState {
    busy: AtomicBool,
    inner: Mutex<Timer0RawMutex, RefCell<PrxInnerState>>,
}

static PRX_STATE: PrxState = PrxState::new();

/// Whether the shared PRX timeslot currently owns the radio.
///
/// This is intentionally an atomic snapshot for high-priority callbacks such
/// as MPSL radio notifications. It is not a replacement for session ownership
/// or a guarantee that a packet is being received at the exact instant read.
static PRX_TIMESLOT_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Return whether the active PRX timeslot currently owns the radio.
pub fn prx_timeslot_active() -> bool {
    PRX_TIMESLOT_ACTIVE.load(Ordering::Acquire)
}

/// BLE conn interval (μs) learned from the host, used so the parked-PRX
/// schedule hint tells the PTX when the next BLE-gap RX window will land
/// (one conn interval away). 0 = unset → hint falls back to slot_length.
static PRX_CONN_INTERVAL_US: AtomicU32 = AtomicU32::new(0);

/// Set the BLE connection interval (μs) for parked-PRX schedule hints.
/// Call from the host HCI Connection-Complete / Connection-Update handler.
pub fn set_prx_conn_interval_us(us: u32) {
    PRX_CONN_INTERVAL_US.store(us, Ordering::Relaxed);
}

/// Adaptive fast-cadence state for parked PRX. When a frame arrives, the
/// callback refills `PRX_FAST_BUDGET` to `PRX_FAST_BUDGET_REFILL` and each
/// slot-end chains one more NORMAL window at `PRX_FAST_DISTANCE_US` until the
/// budget is exhausted — i.e. while the right half is actively sending, RX
/// windows chain fast (low ESB latency); when it goes idle the budget runs out
/// and the session falls back to one BLE-gap-triggered window per conn event.
/// All access is from the single MPSL high-priority callback context.
static PRX_FAST_DISTANCE_US: AtomicU32 = AtomicU32::new(0);
static PRX_FAST_BUDGET_REFILL: AtomicU32 = AtomicU32::new(0);
static PRX_FAST_BUDGET: AtomicU32 = AtomicU32::new(0);

/// Configure adaptive fast-window chaining. `distance_us` is the NORMAL-request
/// distance between chained fast windows (must be >= slot_length_us); `refill`
/// is how many fast windows a single received frame funds (sets the active-hold
/// ≈ refill × distance). 0 distance disables (pure parked cadence).
pub fn configure_prx_fast_window(distance_us: u32, refill: u32) {
    PRX_FAST_DISTANCE_US.store(distance_us, Ordering::Relaxed);
    PRX_FAST_BUDGET_REFILL.store(refill, Ordering::Relaxed);
}

/// Refill the fast-window budget (called from the callback on a received frame).
fn prx_refill_fast_budget() {
    let refill = PRX_FAST_BUDGET_REFILL.load(Ordering::Relaxed);
    if refill > 0 {
        PRX_FAST_BUDGET.store(refill, Ordering::Relaxed);
    }
}

/// Consume one unit of fast-window budget. Returns true if chaining should
/// continue (budget was > 0). Called from the callback at slot-end.
fn prx_consume_fast_budget() -> bool {
    let b = PRX_FAST_BUDGET.load(Ordering::Relaxed);
    if b > 0 {
        PRX_FAST_BUDGET.store(b - 1, Ordering::Relaxed);
        true
    } else {
        false
    }
}

#[repr(C, align(4))]
struct PrxBuffers {
    rx: UnsafeCell<[u8; 256]>,
    ack_tx: UnsafeCell<[u8; 256]>,
}
// SAFETY: identical invariant to PtxBuffers. The rx/ack_tx EasyDMA buffers are
// dereferenced only from inside the PRX timeslot callback (single high-priority
// MPSL context, one slot at a time), never from task code, so there is no
// concurrent aliasing with the radio's DMA access. `.data` placement keeps them
// in RAM and `align(4)` meets the PACKETPTR alignment requirement.
unsafe impl Sync for PrxBuffers {}

#[unsafe(link_section = ".data")]
static PRX_BUFS: PrxBuffers = PrxBuffers {
    rx: UnsafeCell::new([0u8; 256]),
    ack_tx: UnsafeCell::new([0u8; 256]),
};

fn prx_set_earliest_request(state: &mut PrxInnerState, priority: u8, timeout_us: u32) {
    configure_earliest_request(
        &mut state.request,
        priority,
        state.slot_length_us,
        timeout_us,
    );
    state.last_request_kind = PRX_REQ_EARLIEST;
}

fn prx_set_normal_request(state: &mut PrxInnerState, distance_us: u32) {
    configure_normal_request(
        &mut state.request,
        TIMESLOT_PRIORITY_NORMAL,
        distance_us,
        state.slot_length_us,
    );
    state.last_request_kind = PRX_REQ_NORMAL;
}

fn prx_next_normal_distance_us(state: &mut PrxInnerState) -> u32 {
    state.prx_windows_since_gap = state.prx_windows_since_gap.saturating_add(1);

    if state.prx_schedule.gap_after_windows > 0
        && state.prx_windows_since_gap >= state.prx_schedule.gap_after_windows
    {
        state.prx_windows_since_gap = 0;
        return state.prx_schedule.gap_distance_us;
    }

    if state.prx_schedule.normal_distance_us > 0 {
        state.prx_schedule.normal_distance_us
    } else {
        state.slot_length_us
    }
}

/// Finalize a completed PRX slot: surface a batch report when `report_every`
/// slots have elapsed, then either reschedule the next slot or end the session.
/// Shared by the normal TIMER0 slot-end and the EXTEND end paths so the report
/// waker is driven identically regardless of how the slot terminated.
fn prx_finalize_and_reschedule(state: &mut PrxInnerState) {
    let long_session = state.report_every > 0;

    if long_session
        && state.counters.start.saturating_sub(state.last_report_start) >= state.report_every
    {
        state.last_report_start = state.counters.start;
        state.report_ready = true;
        state.waker.wake();
    }

    if state.auto_reschedule && (long_session || state.counters.start < state.target_count) {
        let distance_us = prx_next_normal_distance_us(state);
        prx_set_normal_request(state, distance_us);
        state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
        state.return_param.params.request.p_next = core::ptr::from_mut(&mut state.request);
    } else if prx_consume_fast_budget() {
        // Adaptive fast-cadence chain: a recent frame funded this window; chain
        // the next NORMAL window one fast-distance from this slot's start. When
        // the budget runs out (right half idle) we fall through to ACTION_END
        // and the parked loop's next BLE_INACTIVE re-opens a window.
        let distance_us = PRX_FAST_DISTANCE_US.load(Ordering::Relaxed);
        if distance_us >= state.slot_length_us && distance_us <= raw::MPSL_TIMESLOT_DISTANCE_MAX_US
        {
            prx_set_normal_request(state, distance_us);
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST as u8;
            state.return_param.params.request.p_next = core::ptr::from_mut(&mut state.request);
        } else {
            // Invalid fast distance (misconfiguration) — never submit a NORMAL
            // request MPSL would reject (it surfaces as INVALID_RETURN, which
            // otherwise leaves the session wedged). Drain the budget and fall
            // back to parked (ACTION_END).
            PRX_FAST_BUDGET.store(0, Ordering::Relaxed);
            state.request_outstanding = false;
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
        }
    } else {
        state.request_outstanding = false;
        if !state.auto_reschedule {
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            return;
        }

        state.done = true;
        state.waker.wake();
        state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
    }
}

fn prx_next_window_delay_us(state: &PrxInnerState) -> u32 {
    let t = pac::TIMER0;
    t.tasks_capture(2).write_value(1);
    let elapsed_us = t.cc(2).read();
    prx_schedule_period_us(state).saturating_sub(elapsed_us)
}

fn prx_schedule_period_us(state: &PrxInnerState) -> u32 {
    // Adaptive fast cadence: while the budget is non-zero (right half active),
    // the hint period is the fast-window distance, not the BLE conn interval.
    if PRX_FAST_BUDGET.load(Ordering::Relaxed) > 0 {
        let fast = PRX_FAST_DISTANCE_US.load(Ordering::Relaxed);
        if fast > 0 {
            return fast;
        }
    }
    // Parked PRX (BLE-gap triggered): the next RX window is one conn-interval
    // away, so the hint period must be the conn interval, not the slot length.
    let conn_interval = PRX_CONN_INTERVAL_US.load(Ordering::Relaxed);
    if conn_interval > 0 {
        conn_interval
    } else if state.prx_schedule.normal_distance_us > 0 {
        state.prx_schedule.normal_distance_us
    } else {
        state.slot_length_us
    }
}

fn write_ack_counter_packet(
    buf: &mut [u8; 256],
    counter: u32,
    window_id: u32,
    next_delay_us: u32,
    period_us: u32,
    window_us: u32,
    extension: &[u8],
) {
    let hint = ScheduleHint::new(0, window_id, next_delay_us, period_us, window_us);
    write_counter_schedule_extension_packet(buf, 0, counter, hint, extension);
}

impl PrxState {
    const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            inner: Mutex::new(RefCell::new(PrxInnerState {
                counters: SignalCounters::ZERO,
                done: false,
                waker: WakerRegistration::new(),
                request: raw::mpsl_timeslot_request_t {
                    request_type: raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8,
                    params: raw::mpsl_timeslot_request_t__bindgen_ty_1 {
                        earliest: raw::mpsl_timeslot_request_earliest_t {
                            hfclk: raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8,
                            priority: raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8,
                            length_us: 0,
                            timeout_us: 1_000_000,
                        },
                    },
                },
                return_param: raw::mpsl_timeslot_signal_return_param_t {
                    callback_action: 0,
                    params: raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1 {
                        request:
                            raw::mpsl_timeslot_signal_return_param_t__bindgen_ty_1__bindgen_ty_1 {
                                p_next: core::ptr::null_mut(),
                            },
                    },
                },
                slot_length_us: 0,
                in_slot_match_us: 5500,
                request_timeout_us: 1_000_000,
                target_count: 0,
                config: None,
                addresses: None,
                driver: None,
                saved_pid: [0; NUM_PIPES],
                saved_crc: [0; NUM_PIPES],
                saved_valid: [false; NUM_PIPES],
                phase: PrxPhase::Idle,
                rx_count: 0,
                dup_count: 0,
                bad_crc_count: 0,
                last_pid: [0; NUM_PIPES],
                last_crc: [0; NUM_PIPES],
                last_valid: [false; NUM_PIPES],
                enabled_pipes: 0x01,
                recovery_policy: RadioRecoveryPolicy::ForceResetAfterBoundedDisable,
                slot_active: false,
                extend_count: 0,
                max_extends: 0,
                extend_length_us: 0,
                prx_extend_mode: false,
                ack_counter: [0; NUM_PIPES],
                rx_per_pipe: [0; NUM_PIPES],
                dup_per_pipe: [0; NUM_PIPES],
                bad_crc_per_pipe: [0; NUM_PIPES],
                ack_tx_per_pipe: [0; NUM_PIPES],
                report_every: 0,
                last_report_start: 0,
                report_ready: false,
                prx_schedule: PrxScheduleConfig::continuous(),
                prx_windows_since_gap: 0,
                auto_reschedule: true,
                request_outstanding: false,
                retry_blocked_at_high_priority: true,
                last_request_kind: PRX_REQ_EARLIEST,
                normal_blocked: 0,
                earliest_blocked: 0,
                normal_cancelled: 0,
                earliest_cancelled: 0,
                event_queue: [PrxReceivedEvent {
                    pipe: 0,
                    len: 0,
                    payload: [0; 32],
                }; PRX_EVENT_QUEUE_CAP],
                event_head: 0,
                event_tail: 0,
                event_len: 0,
                event_dropped: 0,
                ack_extension_provider: None,
                ack_extensions: [PrxAckExtension::EMPTY; NUM_PIPES],
            })),
        }
    }

    fn with_inner<F: FnOnce(&mut PrxInnerState) -> R, R>(&self, f: F) -> R {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            f(&mut inner)
        })
    }

    fn try_enter(&'static self) -> Result<BusyGuard, Error> {
        BusyGuard::new(&self.busy)
    }
}

// SAFETY: same MPSL contract as `timeslot_callback` — called from the
// high-priority timeslot signal context with a valid `session_id`/`signal`,
// must stay deterministic and minimal, and returns either a pointer into the
// static `PRX_STATE` return_param or null. The PRX_BUFS EasyDMA buffers it
// programs are only ever touched from this single-context callback (see the
// `unsafe impl Sync for PrxBuffers` note).
unsafe extern "C" fn prx_timeslot_callback(
    session_id: u8,
    signal: u32,
) -> *mut raw::mpsl_timeslot_signal_return_param_t {
    match signal {
        raw::MPSL_TIMESLOT_SIGNAL_START => PRX_STATE.with_inner(|state| {
            state.counters.start += 1;
            state.slot_active = true;
            state.request_outstanding = true;
            PRX_TIMESLOT_ACTIVE.store(true, Ordering::Release);
            state.extend_count = 0;

            let (Some(config), Some(addresses)) = (state.config.as_ref(), state.addresses.as_ref())
            else {
                state.counters.invalid_return += 1;
                state.done = true;
                state.waker.wake();
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
                return &mut state.return_param as *mut _;
            };

            if let Some(driver) = state.driver {
                // ---- Converged path: drive the shared state machine ----
                if let Err(_e) =
                    driver.ts_start_rx(state.saved_pid, state.saved_crc, state.saved_valid)
                {
                    state.counters.invalid_return += 1;
                }
            } else {
                // ---- Legacy inline-engine path ----
                let r = pac::RADIO;
                r.power().write(|w| w.set_power(false));
                r.power().write(|w| w.set_power(true));

                let mut radio = EsbRadio::new(pac::RADIO);
                radio.init(config, addresses);
                radio.restore_pid_state(state.last_pid);
                radio.restore_crc_state(state.last_crc);
                radio.restore_detection_valid_state(state.last_valid);

                let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                let dma_ptr = unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                state.phase = PrxPhase::Receiving;
            }

            // Common: unmask NVIC RADIO so events reach SIGNAL_RADIO, and arm
            // the in-slot TIMER0 CC0 for slot end / extend gating.
            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::RADIO);
            }

            let t = pac::TIMER0;
            t.events_compare(0).write_value(0);
            t.cc(0).write_value(state.in_slot_match_us);
            t.intenset().write(|w| w.set_compare(0, true));
            if state.prx_extend_mode {
                t.shorts().write(|w| w.set_compare_clear(0, true));
            }

            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_RADIO => PRX_STATE.with_inner(|state| {
            state.counters.radio += 1;

            // ---- Converged path: drive the shared state machine ----
            if let Some(driver) = state.driver {
                driver.ts_set_diag_ack_context(crate::state_machine::TimeslotDiagAckContext {
                    window_id: state.counters.start,
                    next_delay_us: prx_next_window_delay_us(state),
                    period_us: prx_schedule_period_us(state),
                    window_us: state.in_slot_match_us,
                });
                let (event, _rx_idx) = driver.ts_on_radio();
                driver.ts_discard_received();
                let pipe = driver.ts_last_pipe() as usize;
                use crate::state_machine::PrxEvent;
                match event {
                    PrxEvent::Received => {
                        state.rx_count += 1;
                        if pipe < NUM_PIPES {
                            state.rx_per_pipe[pipe] += 1;
                            state.ack_tx_per_pipe[pipe] += 1;
                        }
                    }
                    PrxEvent::Duplicate => {
                        state.dup_count += 1;
                        if pipe < NUM_PIPES {
                            state.dup_per_pipe[pipe] += 1;
                            state.ack_tx_per_pipe[pipe] += 1;
                        }
                    }
                    PrxEvent::BadCrc => {
                        state.bad_crc_count += 1;
                        if pipe < NUM_PIPES {
                            state.bad_crc_per_pipe[pipe] += 1;
                        }
                    }
                    PrxEvent::ReceivedNoAck => {
                        state.rx_count += 1;
                        if pipe < NUM_PIPES {
                            state.rx_per_pipe[pipe] += 1;
                        }
                    }
                    PrxEvent::None => {}
                }
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
                return &mut state.return_param as *mut _;
            }

            // ---- Legacy inline-engine path below ----
            let r = pac::RADIO;
            let disabled = r.events_disabled().read() == 1;

            match state.phase {
                PrxPhase::Receiving if disabled => {
                    r.events_disabled().write_value(0);
                    compiler_fence(Ordering::Acquire);

                    // Check CRC.
                    if r.crcstatus().read().crcstatus() == pac::radio::vals::Crcstatus::CRCERROR {
                        state.bad_crc_count += 1;
                        let pipe = r.rxmatch().read().rxmatch() as usize;
                        if pipe < NUM_PIPES {
                            state.bad_crc_per_pipe[pipe] += 1;
                        }
                        // Restart RX: stop radio, re-enable shortcuts, start RX again.
                        let mut radio = EsbRadio::new(pac::RADIO);
                        radio.stop();
                        let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                        let dma_ptr = unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                        radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                    } else {
                        // CRC OK — read metadata from DMA buffer.
                        let rx_buf = unsafe { &*PRX_BUFS.rx.get() };
                        let pid_no_ack = rx_buf[EsbHeader::DMA_OFFSET + 1];
                        let pid = (pid_no_ack >> 1) & 0x03;
                        let no_ack = (pid_no_ack & 0x01) != 0;
                        let pipe = r.rxmatch().read().rxmatch() as usize;
                        let crc = r.rxcrc().read().rxcrc() as u16;

                        let is_dup = pipe < NUM_PIPES
                            && state.last_valid[pipe]
                            && state.last_crc[pipe] == crc
                            && state.last_pid[pipe] == pid;

                        if is_dup {
                            state.dup_count += 1;
                            if pipe < NUM_PIPES {
                                state.dup_per_pipe[pipe] += 1;
                            }
                            if no_ack {
                                // Dup NoAck — stop TX ramp, restart RX.
                                let mut radio = EsbRadio::new(pac::RADIO);
                                radio.stop();
                                let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                                let dma_ptr =
                                    unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                                radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                            } else {
                                // Dup with ACK — re-send same counter (retransmit).
                                let counter = if pipe < NUM_PIPES {
                                    state.ack_counter[pipe]
                                } else {
                                    0
                                };
                                let ack_buf = unsafe { &mut *PRX_BUFS.ack_tx.get() };
                                let next_delay_us = prx_next_window_delay_us(state);
                                let period_us = prx_schedule_period_us(state);
                                write_ack_counter_packet(
                                    ack_buf,
                                    counter,
                                    state.counters.start,
                                    next_delay_us,
                                    period_us,
                                    state.in_slot_match_us,
                                    state.ack_extension_for_pipe(pipe),
                                );
                                let dma_ptr =
                                    unsafe { ack_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                                let mut radio = EsbRadio::new(pac::RADIO);
                                radio.transmit_ack_manual(pipe as u8, dma_ptr);
                                if pipe < NUM_PIPES {
                                    state.ack_tx_per_pipe[pipe] += 1;
                                }
                                state.phase = PrxPhase::TxRepeatedAck;
                            }
                        } else {
                            // New packet.
                            if pipe < NUM_PIPES {
                                state.last_pid[pipe] = pid;
                                state.last_crc[pipe] = crc;
                                state.last_valid[pipe] = true;
                                state.rx_per_pipe[pipe] += 1;
                            }
                            state.rx_count += 1;
                            // A new frame arrived → the right half is active:
                            // fund the fast-cadence budget so this slot chains
                            // fast windows (and the ACK hint we're about to send
                            // carries the fast period, no one-ACK lag).
                            prx_refill_fast_budget();

                            let app_len = rx_buf[EsbHeader::DMA_OFFSET] as usize;
                            if app_len > 4 && pipe < NUM_PIPES {
                                let payload_start = EsbHeader::PAYLOAD_OFFSET + 4;
                                let payload_end =
                                    (EsbHeader::PAYLOAD_OFFSET + app_len).min(rx_buf.len());
                                if payload_start < payload_end {
                                    let payload = &rx_buf[payload_start..payload_end];
                                    let queued = state.push_event(pipe as u8, payload);
                                    state.observe_payload(pipe, payload, queued);
                                }
                            }

                            if no_ack {
                                // NoAck — stop TX ramp, restart RX.
                                let mut radio = EsbRadio::new(pac::RADIO);
                                radio.stop();
                                let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                                let dma_ptr =
                                    unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                                radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                            } else {
                                // Need ACK — send monotonic counter as payload.
                                let counter = if pipe < NUM_PIPES {
                                    state.ack_counter[pipe] += 1;
                                    state.ack_counter[pipe]
                                } else {
                                    0
                                };
                                let ack_buf = unsafe { &mut *PRX_BUFS.ack_tx.get() };
                                let next_delay_us = prx_next_window_delay_us(state);
                                let period_us = prx_schedule_period_us(state);
                                write_ack_counter_packet(
                                    ack_buf,
                                    counter,
                                    state.counters.start,
                                    next_delay_us,
                                    period_us,
                                    state.in_slot_match_us,
                                    state.ack_extension_for_pipe(pipe),
                                );
                                let dma_ptr =
                                    unsafe { ack_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                                let mut radio = EsbRadio::new(pac::RADIO);
                                radio.transmit_ack_manual(pipe as u8, dma_ptr);
                                if pipe < NUM_PIPES {
                                    state.ack_tx_per_pipe[pipe] += 1;
                                }
                                state.phase = PrxPhase::TxAck;
                            }
                        }
                    }
                }

                PrxPhase::TxAck if disabled => {
                    r.events_disabled().write_value(0);
                    compiler_fence(Ordering::Acquire);

                    // ACK sent — restart RX.
                    let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                    let dma_ptr = unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                    let mut radio = EsbRadio::new(pac::RADIO);
                    radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                    state.phase = PrxPhase::Receiving;
                }

                PrxPhase::TxRepeatedAck if disabled => {
                    r.events_disabled().write_value(0);
                    compiler_fence(Ordering::Acquire);

                    // Repeated ACK sent — restart RX.
                    let rx_buf = unsafe { &mut *PRX_BUFS.rx.get() };
                    let dma_ptr = unsafe { rx_buf.as_mut_ptr().add(EsbHeader::DMA_OFFSET) };
                    let mut radio = EsbRadio::new(pac::RADIO);
                    radio.start_receiving_manual_ack(state.enabled_pipes, dma_ptr);
                    state.phase = PrxPhase::Receiving;
                }

                _ => {}
            }

            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_TIMER0 => PRX_STATE.with_inner(|state| {
            state.counters.timer0 += 1;

            let t = pac::TIMER0;
            t.events_compare(0).write_value(0);

            if state.prx_extend_mode && state.extend_count < state.max_extends {
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_EXTEND as u8;
                state.return_param.params.extend.length_us = state.extend_length_us;
                return &mut state.return_param as *mut _;
            }

            state.slot_active = false;
            PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
            t.intenclr().write(|w| w.set_compare(0, true));
            t.shorts().write(|w| w.set_compare_clear(0, false));

            // Converged path: stop driver + snapshot dup state for next slot.
            if let Some(driver) = state.driver {
                driver.ts_force_stop();
                let (p, c, v) = driver.ts_snapshot_dup_state();
                state.saved_pid = p;
                state.saved_crc = c;
                state.saved_valid = v;
            } else {
                state.phase = PrxPhase::Idle;
            }

            if quiesce_radio_before_timeslot_end(state.recovery_policy).timed_out() {
                state.counters.radio_disable_timeout += 1;
            }

            prx_finalize_and_reschedule(state);
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED => PRX_STATE.with_inner(|state| {
            state.counters.extend_succeeded += 1;
            state.extend_count += 1;

            if state.extend_count >= state.max_extends {
                state.slot_active = false;
                PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
                let t = pac::TIMER0;
                t.intenclr().write(|w| w.set_compare(0, true));
                t.shorts().write(|w| w.set_compare_clear(0, false));

                if let Some(driver) = state.driver {
                    driver.ts_force_stop();
                    let (p, c, v) = driver.ts_snapshot_dup_state();
                    state.saved_pid = p;
                    state.saved_crc = c;
                    state.saved_valid = v;
                } else {
                    state.phase = PrxPhase::Idle;
                }

                if quiesce_radio_before_timeslot_end(state.recovery_policy).timed_out() {
                    state.counters.radio_disable_timeout += 1;
                }

                prx_finalize_and_reschedule(state);
            } else {
                state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_NONE as u8;
            }
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED => PRX_STATE.with_inner(|state| {
            state.counters.extend_failed += 1;
            state.slot_active = false;
            PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);

            let t = pac::TIMER0;
            t.intenclr().write(|w| w.set_compare(0, true));
            t.shorts().write(|w| w.set_compare_clear(0, false));

            if let Some(driver) = state.driver {
                driver.ts_force_stop();
                let (p, c, v) = driver.ts_snapshot_dup_state();
                state.saved_pid = p;
                state.saved_crc = c;
                state.saved_valid = v;
            } else {
                state.phase = PrxPhase::Idle;
            }

            if quiesce_radio_before_timeslot_end(state.recovery_policy).timed_out() {
                state.counters.radio_disable_timeout += 1;
            }

            prx_finalize_and_reschedule(state);
            &mut state.return_param as *mut _
        }),

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_IDLE => PRX_STATE.with_inner(|state| {
            state.counters.session_idle += 1;
            state.request_outstanding = false;
            state.waker.wake();
            core::ptr::null_mut()
        }),

        raw::MPSL_TIMESLOT_SIGNAL_BLOCKED | raw::MPSL_TIMESLOT_SIGNAL_CANCELLED => {
            let manual_session = PRX_STATE.with_inner(|state| {
                if signal == raw::MPSL_TIMESLOT_SIGNAL_BLOCKED {
                    state.counters.blocked += 1;
                } else {
                    state.counters.cancelled += 1;
                }

                if !state.auto_reschedule {
                    state.request_outstanding = false;
                    state.waker.wake();
                    true
                } else {
                    false
                }
            });
            if manual_session {
                return core::ptr::null_mut();
            }

            let request = PRX_STATE.with_inner(|state| {
                if signal == raw::MPSL_TIMESLOT_SIGNAL_BLOCKED {
                    match state.last_request_kind {
                        PRX_REQ_NORMAL => state.normal_blocked += 1,
                        PRX_REQ_EARLIEST => state.earliest_blocked += 1,
                        _ => {}
                    }
                } else {
                    match state.last_request_kind {
                        PRX_REQ_NORMAL => state.normal_cancelled += 1,
                        PRX_REQ_EARLIEST => state.earliest_cancelled += 1,
                        _ => {}
                    }
                }
                let priority = if state.retry_blocked_at_high_priority {
                    TIMESLOT_PRIORITY_HIGH
                } else {
                    TIMESLOT_PRIORITY_NORMAL
                };
                prx_set_earliest_request(
                    state,
                    priority,
                    raw::MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US,
                );
                core::ptr::from_ref(&state.request)
            });
            let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
            if ret < 0 {
                PRX_STATE.with_inner(|state| {
                    state.counters.invalid_return += 1;
                    state.done = true;
                    state.waker.wake();
                });
            }
            core::ptr::null_mut()
        }

        raw::MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED => PRX_STATE.with_inner(|state| {
            state.counters.session_closed += 1;
            state.request_outstanding = false;
            PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
            state.done = true;
            state.waker.wake();
            core::ptr::null_mut()
        }),

        raw::MPSL_TIMESLOT_SIGNAL_OVERSTAYED => PRX_STATE.with_inner(|state| {
            state.counters.overstayed += 1;
            state.slot_active = false;
            state.request_outstanding = false;
            PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
            if let Some(driver) = state.driver {
                driver.ts_force_stop();
            } else {
                state.phase = PrxPhase::Idle;
            }
            state.done = true;
            state.waker.wake();
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),

        _ => PRX_STATE.with_inner(|state| {
            // Unhandled signal (e.g. INVALID_RETURN from a malformed request).
            // Clear request_outstanding/slot_active so the parked loop can
            // submit the next window instead of wedging, and end the slot.
            // (INVALID_RETURN shouldn't occur now that the fast-distance is
            // validated, but stay self-healing rather than hang the session.)
            state.counters.invalid_return += 1;
            state.slot_active = false;
            state.request_outstanding = false;
            PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
            if let Some(driver) = state.driver {
                driver.ts_force_stop();
            } else {
                state.phase = PrxPhase::Idle;
            }
            state.return_param.callback_action = raw::MPSL_TIMESLOT_SIGNAL_ACTION_END as u8;
            &mut state.return_param as *mut _
        }),
    }
}

/// Run PRX reception inside chained timeslots.
///
/// Each timeslot power-cycles the RADIO, inits ESB, and listens for
/// incoming packets. ACK is sent automatically for each received packet.
/// After `count` slots, returns cumulative results.
/// Duplicate detection state is preserved across slot boundaries.
pub async fn run_prx_slots(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    slot_length_us: u32,
    in_slot_match_us: u32,
    count: u32,
    enabled_pipes: u8,
) -> Result<PrxSlotResult, Error> {
    let _busy = PRX_STATE.try_enter()?;
    if config.payload_length < 4 {
        return Err(Error::InvalidParam);
    }

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(prx_timeslot_callback), (&mut session_id) as *mut _)
    };
    mpsl_ok(ret)?;

    let _drop = OnDrop::new(|| {
        let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
    });

    PRX_STATE.with_inner(|state| {
        state.reset_runtime();

        state.target_count = count;
        state.slot_length_us = slot_length_us;
        state.in_slot_match_us = in_slot_match_us;
        state.request_timeout_us = 1_000_000;
        state.config = Some(config.clone());
        state.addresses = Some(addresses.clone());
        state.enabled_pipes = enabled_pipes;
        state.report_every = 0;
        state.prx_schedule = PrxScheduleConfig::continuous();
        state.retry_blocked_at_high_priority = true;
        prx_set_earliest_request(state, TIMESLOT_PRIORITY_NORMAL, 1_000_000);
    });

    let request = PRX_STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    mpsl_ok(ret)?;

    poll_fn(|cx| {
        PRX_STATE.with_inner(|state| {
            state.waker.register(cx.waker());
            if state.done {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    })
    .await;

    _drop.defuse();
    unsafe {
        let ret = raw::mpsl_timeslot_session_close(session_id);
        mpsl_ok(ret)?;
    }

    Ok(PRX_STATE.with_inner(|state| PrxSlotResult {
        counters: state.counters,
        rx_count: state.rx_count,
        dup_count: state.dup_count,
        bad_crc_count: state.bad_crc_count,
        rx_per_pipe: state.rx_per_pipe,
        dup_per_pipe: state.dup_per_pipe,
        bad_crc_per_pipe: state.bad_crc_per_pipe,
        ack_tx_per_pipe: state.ack_tx_per_pipe,
        slot_length_us: state.slot_length_us,
        in_slot_match_us: state.in_slot_match_us,
        report_every: state.report_every,
        phase: state.phase.as_u8(),
        slot_active: state.slot_active,
        last_request_kind: state.last_request_kind,
        normal_blocked: state.normal_blocked,
        earliest_blocked: state.earliest_blocked,
        normal_cancelled: state.normal_cancelled,
        earliest_cancelled: state.earliest_cancelled,
    }))
}

/// Long-lived PRX timeslot session.
///
/// This keeps the MPSL session open and chains timeslots continuously. It is
/// intended for BLE coexistence tests where repeated session open/close can
/// contend with the SoftDevice Controller scheduler.
pub struct PrxSlotSession {
    _busy: BusyGuard,
    session_id: u8,
    last_counters: SignalCounters,
    last_rx_count: u32,
    last_dup_count: u32,
    last_bad_crc_count: u32,
    last_rx_per_pipe: [u32; NUM_PIPES],
    last_dup_per_pipe: [u32; NUM_PIPES],
    last_bad_crc_per_pipe: [u32; NUM_PIPES],
    last_ack_tx_per_pipe: [u32; NUM_PIPES],
    last_normal_blocked: u32,
    last_earliest_blocked: u32,
    last_normal_cancelled: u32,
    last_earliest_cancelled: u32,
}

impl PrxSlotSession {
    /// Request one earliest PRX receive window for a parked session.
    ///
    /// Returns `Ok(true)` when a request was submitted. `Ok(false)` means a
    /// previous request or its granted slot is still active, so callers can
    /// safely coalesce repeated wakeups such as radio notifications.
    pub fn request_window(&mut self) -> Result<bool, Error> {
        let request = PRX_STATE.with_inner(|state| {
            if state.done || state.slot_active || state.request_outstanding {
                return None;
            }

            prx_set_earliest_request(state, TIMESLOT_PRIORITY_NORMAL, state.request_timeout_us);
            state.request_outstanding = true;
            Some(core::ptr::from_ref(&state.request))
        });

        let Some(request) = request else {
            return Ok(false);
        };

        let ret = unsafe { raw::mpsl_timeslot_request(self.session_id, request) };
        if let Err(e) = mpsl_ok(ret) {
            PRX_STATE.with_inner(|state| {
                state.request_outstanding = false;
            });
            #[cfg(feature = "defmt")]
            defmt::warn!("mpsl_timeslot_request ret={}", ret);
            return Err(e);
        }

        Ok(true)
    }

    /// Remove and return one received application payload without waiting.
    ///
    /// Report-only consumers should call this often enough to avoid dropping
    /// payloads when the bounded event queue fills.
    pub fn try_next_event(&mut self) -> Option<PrxReceivedEvent> {
        PRX_STATE.with_inner(|state| state.pop_event())
    }

    pub async fn next_event(&mut self) -> Result<PrxReceivedEvent, Error> {
        poll_fn(|cx| {
            PRX_STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                if state.done {
                    Poll::Ready(None)
                } else if let Some(event) = state.pop_event() {
                    Poll::Ready(Some(event))
                } else {
                    Poll::Pending
                }
            })
        })
        .await
        .ok_or(Error::Mpsl)
    }

    pub async fn next_report(&mut self) -> Result<PrxSlotResult, Error> {
        poll_fn(|cx| {
            PRX_STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                if state.done {
                    Poll::Ready(())
                } else if state.report_ready {
                    state.report_ready = false;
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await;

        let result = PRX_STATE.with_inner(|state| {
            if state.done {
                return Err(Error::Mpsl);
            }

            let counters = state.counters.saturating_sub(self.last_counters);
            let rx_per_pipe = delta_per_pipe(&state.rx_per_pipe, &self.last_rx_per_pipe);
            let dup_per_pipe = delta_per_pipe(&state.dup_per_pipe, &self.last_dup_per_pipe);
            let bad_crc_per_pipe =
                delta_per_pipe(&state.bad_crc_per_pipe, &self.last_bad_crc_per_pipe);
            let ack_tx_per_pipe =
                delta_per_pipe(&state.ack_tx_per_pipe, &self.last_ack_tx_per_pipe);

            let result = PrxSlotResult {
                counters,
                rx_count: state.rx_count.saturating_sub(self.last_rx_count),
                dup_count: state.dup_count.saturating_sub(self.last_dup_count),
                bad_crc_count: state.bad_crc_count.saturating_sub(self.last_bad_crc_count),
                rx_per_pipe,
                dup_per_pipe,
                bad_crc_per_pipe,
                ack_tx_per_pipe,
                slot_length_us: state.slot_length_us,
                in_slot_match_us: state.in_slot_match_us,
                report_every: state.report_every,
                phase: state.phase.as_u8(),
                slot_active: state.slot_active,
                last_request_kind: state.last_request_kind,
                normal_blocked: state
                    .normal_blocked
                    .saturating_sub(self.last_normal_blocked),
                earliest_blocked: state
                    .earliest_blocked
                    .saturating_sub(self.last_earliest_blocked),
                normal_cancelled: state
                    .normal_cancelled
                    .saturating_sub(self.last_normal_cancelled),
                earliest_cancelled: state
                    .earliest_cancelled
                    .saturating_sub(self.last_earliest_cancelled),
            };

            self.last_counters = state.counters;
            self.last_rx_count = state.rx_count;
            self.last_dup_count = state.dup_count;
            self.last_bad_crc_count = state.bad_crc_count;
            self.last_rx_per_pipe = state.rx_per_pipe;
            self.last_dup_per_pipe = state.dup_per_pipe;
            self.last_bad_crc_per_pipe = state.bad_crc_per_pipe;
            self.last_ack_tx_per_pipe = state.ack_tx_per_pipe;
            self.last_normal_blocked = state.normal_blocked;
            self.last_earliest_blocked = state.earliest_blocked;
            self.last_normal_cancelled = state.normal_cancelled;
            self.last_earliest_cancelled = state.earliest_cancelled;

            Ok(result)
        })?;

        Ok(result)
    }
}

impl Drop for PrxSlotSession {
    fn drop(&mut self) {
        PRX_STATE.with_inner(|state| {
            state.driver = None;
            state.request_outstanding = false;
        });
        PRX_TIMESLOT_ACTIVE.store(false, Ordering::Release);
        let _ = unsafe { raw::mpsl_timeslot_session_close(self.session_id) };
    }
}

/// Convenience wrapper for driving an [`EsbPrx`](crate::isr::EsbPrx) from MPSL
/// timeslots.
///
/// This is the S4-d lifecycle surface: it opens a PRX timeslot session,
/// registers the shared PRX state machine as the callback driver, and clears the
/// registration when dropped. The wrapper is intentionally thin while the
/// diagnostic `PrxSlotSession` API remains available for bring-up.
pub struct EsbTimeslotPrx<T, const N: usize, const SIZE: usize>
where
    T: crate::timer::TimerInstance,
{
    prx: &'static crate::isr::EsbPrx<T, N, SIZE>,
    session: PrxSlotSession,
}

impl<T, const N: usize, const SIZE: usize> EsbTimeslotPrx<T, N, SIZE>
where
    T: crate::timer::TimerInstance,
{
    pub fn start(
        mpsl: &MultiprotocolServiceLayer<'_>,
        prx: &'static crate::isr::EsbPrx<T, N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        slot_config: PrxSlotConfig,
    ) -> Result<Self, Error> {
        // Open without submitting the first timeslot request yet, but keep
        // auto-reschedule so the session still runs continuously. Registering
        // the driver *before* the request goes out closes the race where MPSL
        // could grant the first slot and run SIGNAL_START with `driver == None`
        // (the legacy inline engine) between open and set_prx_driver.
        let mut session = open_prx_session_with_mode(
            mpsl,
            config,
            addresses,
            slot_config,
            None,
            true,  // auto_reschedule: continuous PRX
            false, // submit_now: defer until driver is registered
        )?;
        set_prx_driver(prx, addresses, slot_config.enabled_pipes);
        session.request_window()?;
        Ok(Self { prx, session })
    }

    pub async fn receive(&self) -> crate::isr::ReceivedPacket<'_, N, SIZE> {
        self.prx.receive().await
    }

    pub async fn send_ack_payload(&self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        self.prx.send_ack_payload(pipe, payload).await
    }

    pub async fn next_report(&mut self) -> Result<PrxSlotResult, Error> {
        self.session.next_report().await
    }

    pub fn prx(&self) -> &'static crate::isr::EsbPrx<T, N, SIZE> {
        self.prx
    }

    pub fn stop(self) -> &'static crate::isr::EsbPrx<T, N, SIZE> {
        let prx = self.prx;
        drop(self);
        prx
    }
}

/// Convenience wrapper for driving an [`EsbPtx`](crate::isr::EsbPtx) from MPSL
/// timeslots.
///
/// This is the S6-d lifecycle surface: it opens a PTX timeslot session,
/// registers the shared PTX state machine as the callback driver, and clears
/// the registration when dropped. The wrapper is intentionally thin while the
/// diagnostic `PtxPollSession` / `PtxEventSession` APIs remain available for
/// bring-up.
pub struct EsbTimeslotPtx<T, const N: usize, const SIZE: usize>
where
    T: crate::timer::TimerInstance,
{
    ptx: &'static crate::isr::EsbPtx<T, N, SIZE>,
    session: PtxPollSession,
}

impl<T, const N: usize, const SIZE: usize> EsbTimeslotPtx<T, N, SIZE>
where
    T: crate::timer::TimerInstance,
{
    /// Open a converged PTX timeslot session.
    ///
    /// Creates the poll session, registers `ptx` as the timeslot driver so the
    /// callback dispatches through the shared `PtxStateMachine` instead of the
    /// inline engine.
    pub fn start(
        mpsl: &MultiprotocolServiceLayer<'_>,
        ptx: &'static crate::isr::EsbPtx<T, N, SIZE>,
        config: &EsbConfig,
        addresses: &EsbAddresses,
        poll_config: PtxPollConfig,
    ) -> Result<Self, Error> {
        let session = open_ptx_poll_session(mpsl, config, addresses, poll_config)?;
        let first_pipe = poll_config.pipe_mask.trailing_zeros() as u8;
        set_ptx_driver(ptx, addresses, first_pipe);
        Ok(Self { ptx, session })
    }

    /// Wait for the next periodic report from the poll session.
    pub async fn next_report(&mut self) -> Result<PtxPollResult, Error> {
        self.session.next_report().await
    }

    /// Queue a payload for transmission on the given pipe.
    pub async fn send_to(&self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        self.ptx.send_to(pipe, payload).await
    }

    /// Queue a payload on the default pipe.
    pub async fn send(&self, payload: &[u8]) -> Result<(), Error> {
        self.ptx.send(payload).await
    }

    /// Returns the underlying PTX driver.
    pub fn ptx(&self) -> &'static crate::isr::EsbPtx<T, N, SIZE> {
        self.ptx
    }

    /// Close the session, unregister the driver, and return the PTX driver.
    pub fn stop(self) -> &'static crate::isr::EsbPtx<T, N, SIZE> {
        let ptx = self.ptx;
        drop(self);
        ptx
    }
}

/// Open a long-lived PRX session with scheduler counter ACK payloads only.
///
/// For a higher-level protocol that needs to append caller-defined bytes to
/// each ACK, use [`open_prx_session_with_ack_extension`].
pub fn open_prx_session(
    mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    slot_config: PrxSlotConfig,
) -> Result<PrxSlotSession, Error> {
    open_prx_session_with_ack_extension(mpsl, config, addresses, slot_config, None)
}

/// Open a long-lived PRX session with an optional caller-defined ACK extension.
///
/// `ack_extension_provider` runs in the MPSL RADIO callback after the raw
/// application payload has been queued. It must be bounded and non-blocking.
/// The ESB/MPSL adapter only appends the returned bytes; it does not interpret
/// their format or impose a transport protocol.
pub fn open_prx_session_with_ack_extension(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    slot_config: PrxSlotConfig,
    ack_extension_provider: Option<PrxAckExtensionProvider>,
) -> Result<PrxSlotSession, Error> {
    open_prx_session_with_mode(
        _mpsl,
        config,
        addresses,
        slot_config,
        ack_extension_provider,
        true,
        true,
    )
}

/// Open a PRX session that waits for [`PrxSlotSession::request_window`] before
/// requesting its first receive window.
///
/// This is useful for application-owned scheduling policies, such as opening a
/// bounded ESB receive window immediately after another protocol's radio
/// activity. Each request runs at most one PRX window; the session stays open
/// for a later request instead of chaining a NORMAL timeslot automatically.
pub fn open_parked_prx_session_with_ack_extension(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    slot_config: PrxSlotConfig,
    ack_extension_provider: Option<PrxAckExtensionProvider>,
) -> Result<PrxSlotSession, Error> {
    open_prx_session_with_mode(
        _mpsl,
        config,
        addresses,
        slot_config,
        ack_extension_provider,
        false,
        false,
    )
}

fn open_prx_session_with_mode(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    slot_config: PrxSlotConfig,
    ack_extension_provider: Option<PrxAckExtensionProvider>,
    auto_reschedule: bool,
    submit_now: bool,
) -> Result<PrxSlotSession, Error> {
    let busy = PRX_STATE.try_enter()?;
    if config.payload_length < 4 {
        drop(busy);
        return Err(Error::InvalidParam);
    }
    if let Err(e) = slot_config.validate() {
        drop(busy);
        return Err(e);
    }

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(prx_timeslot_callback), (&mut session_id) as *mut _)
    };
    if let Err(e) = mpsl_ok(ret) {
        drop(busy);
        return Err(e);
    }

    PRX_STATE.with_inner(|state| {
        state.reset_runtime();

        state.slot_length_us = slot_config.slot_length_us;
        state.in_slot_match_us = slot_config.in_slot_match_us;
        state.request_timeout_us = slot_config.request.timeout_us;
        state.config = Some(config.clone());
        state.addresses = Some(addresses.clone());
        state.enabled_pipes = slot_config.enabled_pipes;
        state.recovery_policy = slot_config.recovery;
        state.report_every = slot_config.report_every;
        state.prx_schedule = slot_config.schedule;
        state.prx_extend_mode = slot_config.extend_mode;
        state.max_extends = slot_config.max_extends;
        state.extend_length_us = slot_config.extend_length_us;
        state.retry_blocked_at_high_priority = slot_config.request.retry_blocked_at_high_priority;
        state.ack_extension_provider = ack_extension_provider;
        state.auto_reschedule = auto_reschedule;
        state.request_outstanding = submit_now;
        prx_set_earliest_request(
            state,
            TIMESLOT_PRIORITY_NORMAL,
            slot_config.request.timeout_us,
        );
    });

    let session = PrxSlotSession {
        _busy: busy,
        session_id,
        last_counters: SignalCounters::ZERO,
        last_rx_count: 0,
        last_dup_count: 0,
        last_bad_crc_count: 0,
        last_rx_per_pipe: [0; NUM_PIPES],
        last_dup_per_pipe: [0; NUM_PIPES],
        last_bad_crc_per_pipe: [0; NUM_PIPES],
        last_ack_tx_per_pipe: [0; NUM_PIPES],
        last_normal_blocked: 0,
        last_earliest_blocked: 0,
        last_normal_cancelled: 0,
        last_earliest_cancelled: 0,
    };

    if !submit_now {
        return Ok(session);
    }

    let request = PRX_STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    if let Err(e) = mpsl_ok(ret) {
        drop(session);
        return Err(e);
    }

    Ok(session)
}

/// Register the converged-engine driver on the current PRX session (S3/D6).
///
/// After this call, the PRX timeslot callback drives the shared
/// `EsbPrx`/`PrxStateMachine` through `TimeslotPrxDriver` instead of the legacy
/// inline engine. Must be called between `open_prx_session` and the first
/// `next_report`. The driver must live in a `&'static` (StaticCell) and already
/// be configured with the same `config`/`addresses`/`enabled_pipes` as the
/// session. Pass `addresses` and `enabled_pipes` so the driver can re-init
/// RADIO each slot.
///
/// To revert to the inline engine (e.g. for A/B regression), call
/// [`clear_prx_driver`] before the next slot starts.
pub fn set_prx_driver(
    driver: &'static dyn crate::isr::TimeslotPrxDriver,
    addresses: &EsbAddresses,
    enabled_pipes: u8,
) {
    driver.ts_configure(addresses, enabled_pipes);
    PRX_STATE.with_inner(|state| {
        state.driver = Some(driver);
        // Ensure the legacy dup-detection fields are seeded from the driver's
        // initial snapshot so the first slot doesn't falsely mark the first
        // packet as a dup of stale state.
        let (p, c, v) = driver.ts_snapshot_dup_state();
        state.saved_pid = p;
        state.saved_crc = c;
        state.saved_valid = v;
        // Mirror into legacy fields in case fallback is used later.
        state.last_pid = p;
        state.last_crc = c;
        state.last_valid = v;
        state.addresses = Some(addresses.clone());
        state.enabled_pipes = enabled_pipes;
    });
}

/// Unregister the converged-engine driver, reverting to the legacy inline path.
pub fn clear_prx_driver() {
    PRX_STATE.with_inner(|state| {
        state.driver = None;
    });
}

/// Register a converged-engine PTX driver so the PTX timeslot callback dispatches
/// through the shared [`EsbPtx`](crate::isr::EsbPtx) state machine instead of the
/// legacy inline PTX engine. Must be called between `open_ptx_poll_session`
/// (or `open_event_session`) and the first slot. The driver must live in a
/// `&'static` (StaticCell) and already be configured with the same
/// `config`/`addresses`/`tx_pipe` as the session. Pass `addresses` and `tx_pipe`
/// so the driver can re-init RADIO each slot.
///
/// To revert to the inline engine (e.g. for A/B regression), call
/// [`clear_ptx_driver`] before the next slot starts.
pub fn set_ptx_driver(
    driver: &'static dyn crate::isr::TimeslotPtxDriver,
    addresses: &EsbAddresses,
    tx_pipe: u8,
) {
    driver.ts_configure(addresses, tx_pipe);
    PTX_STATE.with_inner(|state| {
        state.driver = Some(driver);
        let (p, c, v) = driver.ts_snapshot_dup_state();
        state.saved_pid = p;
        state.saved_crc = c;
        state.saved_valid = v;
        state.pid = driver.ts_pid();
        state.tx_pipe = tx_pipe;
        state.addresses = Some(addresses.clone());
    });
}

/// Unregister the converged-engine PTX driver, reverting to the legacy inline path.
pub fn clear_ptx_driver() {
    PTX_STATE.with_inner(|state| {
        state.driver = None;
    });
}

/// Per-round result for the PTX poll session.
pub struct PtxPollResult {
    /// Signal counters for this reporting period.
    pub counters: SignalCounters,
    /// Total TX packets in this period.
    pub tx_count: u32,
    /// Total ACK OK in this period.
    pub ack_ok_count: u32,
    /// Per-pipe TX count.
    pub tx_per_pipe: [u32; NUM_PIPES],
    /// Per-pipe ACK OK count.
    pub ack_per_pipe: [u32; NUM_PIPES],
    /// ACK wait windows that reached the diagnostic timeout.
    pub ack_timeout_count: u32,
    /// ACK packets received with a bad CRC.
    pub ack_crc_fail_count: u32,
    /// Per-pipe ACK wait windows that reached the diagnostic timeout.
    pub ack_timeout_per_pipe: [u32; NUM_PIPES],
    /// Per-pipe ACK packets received with a bad CRC.
    pub ack_crc_fail_per_pipe: [u32; NUM_PIPES],
    /// ACK payloads that carried a valid passive schedule hint.
    pub schedule_hint_count: u32,
    /// ACK payloads whose schedule hint was missing or invalid.
    pub schedule_hint_bad_count: u32,
    /// Last valid PRX window id observed in a schedule hint.
    pub last_schedule_window_id: u32,
    /// Passive schedule relationship diagnostics for this report.
    pub schedule: ScheduleTrackerSnapshot,
    /// PTX timeslots intentionally skipped by the schedule gate.
    pub schedule_skip_count: u32,
    /// Whether phase-locked scheduling is currently active.
    pub schedule_lock_active: bool,
    /// New phase-lock acquisitions in this period.
    pub schedule_lock_count: u32,
    /// Phase-lock fallbacks to scanning in this period.
    pub schedule_reacquire_count: u32,
    /// Consecutive misses while phase-locked.
    pub schedule_lock_miss_streak: u8,
    /// Current phase-locked request distance.
    pub schedule_period_us: u32,
}

/// Long-lived PTX poll session that round-robins across pipes.
///
/// Each timeslot polls exactly one pipe (1 TX + wait ACK). The pipe
/// auto-advances through `pipe_mask` in round-robin order. After
/// `report_every` slots, `next_report()` returns incremental stats.
pub struct PtxPollSession {
    _busy: BusyGuard,
    session_id: u8,
    last_counters: SignalCounters,
    last_tx_count: u32,
    last_ack_ok_count: u32,
    last_tx_per_pipe: [u32; NUM_PIPES],
    last_ack_per_pipe: [u32; NUM_PIPES],
    last_ack_timeout_count: u32,
    last_ack_crc_fail_count: u32,
    last_schedule_hint_count: u32,
    last_schedule_hint_bad_count: u32,
    last_schedule: ScheduleTrackerSnapshot,
    last_schedule_skip_count: u32,
    last_schedule_lock_count: u32,
    last_schedule_reacquire_count: u32,
    last_ack_timeout_per_pipe: [u32; NUM_PIPES],
    last_ack_crc_fail_per_pipe: [u32; NUM_PIPES],
}

impl PtxPollSession {
    pub async fn next_report(&mut self) -> Result<PtxPollResult, Error> {
        poll_fn(|cx| {
            PTX_STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                if state.done {
                    Poll::Ready(())
                } else if state.poll_report_ready {
                    state.poll_report_ready = false;
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await;

        let result = PTX_STATE.with_inner(|state| {
            if state.done {
                return Err(Error::Mpsl);
            }

            let counters = state.counters.saturating_sub(self.last_counters);
            let tx_per_pipe = delta_per_pipe(&state.tx_per_pipe, &self.last_tx_per_pipe);
            let ack_per_pipe = delta_per_pipe(&state.ack_ok_per_pipe, &self.last_ack_per_pipe);
            let ack_timeout_per_pipe =
                delta_per_pipe(&state.ack_timeout_per_pipe, &self.last_ack_timeout_per_pipe);
            let ack_crc_fail_per_pipe = delta_per_pipe(
                &state.ack_crc_fail_per_pipe,
                &self.last_ack_crc_fail_per_pipe,
            );
            let schedule = state
                .schedule_tracker
                .snapshot()
                .saturating_sub(self.last_schedule);

            let result = PtxPollResult {
                counters,
                tx_count: state.tx_count.saturating_sub(self.last_tx_count),
                ack_ok_count: state.ack_ok_count.saturating_sub(self.last_ack_ok_count),
                tx_per_pipe,
                ack_per_pipe,
                ack_timeout_count: state
                    .ack_timeout_count
                    .saturating_sub(self.last_ack_timeout_count),
                ack_crc_fail_count: state
                    .ack_crc_fail_count
                    .saturating_sub(self.last_ack_crc_fail_count),
                ack_timeout_per_pipe,
                ack_crc_fail_per_pipe,
                schedule_hint_count: state
                    .schedule_hint_count
                    .saturating_sub(self.last_schedule_hint_count),
                schedule_hint_bad_count: state
                    .schedule_hint_bad_count
                    .saturating_sub(self.last_schedule_hint_bad_count),
                last_schedule_window_id: state.last_schedule_window_id,
                schedule,
                schedule_skip_count: state
                    .schedule_skip_count
                    .saturating_sub(self.last_schedule_skip_count),
                schedule_lock_active: state.schedule_lock_active,
                schedule_lock_count: state
                    .schedule_lock_count
                    .saturating_sub(self.last_schedule_lock_count),
                schedule_reacquire_count: state
                    .schedule_reacquire_count
                    .saturating_sub(self.last_schedule_reacquire_count),
                schedule_lock_miss_streak: state.schedule_lock_miss_streak,
                schedule_period_us: state.schedule_period_us,
            };

            self.last_counters = state.counters;
            self.last_tx_count = state.tx_count;
            self.last_ack_ok_count = state.ack_ok_count;
            self.last_tx_per_pipe = state.tx_per_pipe;
            self.last_ack_per_pipe = state.ack_ok_per_pipe;
            self.last_ack_timeout_count = state.ack_timeout_count;
            self.last_ack_crc_fail_count = state.ack_crc_fail_count;
            self.last_schedule_hint_count = state.schedule_hint_count;
            self.last_schedule_hint_bad_count = state.schedule_hint_bad_count;
            self.last_schedule = state.schedule_tracker.snapshot();
            self.last_schedule_skip_count = state.schedule_skip_count;
            self.last_schedule_lock_count = state.schedule_lock_count;
            self.last_schedule_reacquire_count = state.schedule_reacquire_count;
            self.last_ack_timeout_per_pipe = state.ack_timeout_per_pipe;
            self.last_ack_crc_fail_per_pipe = state.ack_crc_fail_per_pipe;

            Ok(result)
        })?;

        Ok(result)
    }
}

impl Drop for PtxPollSession {
    fn drop(&mut self) {
        PTX_STATE.with_inner(|state| {
            state.driver = None;
        });
        let _ = unsafe { raw::mpsl_timeslot_session_close(self.session_id) };
    }
}

/// Open a long-lived PTX poll session that round-robins across the given pipes.
///
/// `poll_config` controls the slot length, pipe mask, report cadence, ACK
/// timeout, and optional in-slot retries.
pub fn open_ptx_poll_session(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    poll_config: PtxPollConfig,
) -> Result<PtxPollSession, Error> {
    let busy = PTX_STATE.try_enter()?;
    if config.payload_length < 4 {
        drop(busy);
        return Err(Error::InvalidParam);
    }
    if let Err(e) = poll_config.validate() {
        drop(busy);
        return Err(e);
    }

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(ptx_timeslot_callback), (&mut session_id) as *mut _)
    };
    if let Err(e) = mpsl_ok(ret) {
        drop(busy);
        return Err(e);
    }

    let first_pipe = poll_config.pipe_mask.trailing_zeros() as u8;

    PTX_STATE.with_inner(|state| {
        state.reset_runtime();

        state.slot_length_us = poll_config.slot_length_us;
        state.in_slot_match_us = poll_config.in_slot_match_us;
        state.request_timeout_us = poll_config.request.timeout_us;
        state.config = Some(config.clone());
        state.addresses = Some(addresses.clone());
        state.tx_pipe = first_pipe;
        state.packets_per_slot = 1;
        state.schedule_gate = poll_config.schedule_gate;
        state.poll_pipes = poll_config.pipe_mask.count_ones() as u8;
        state.poll_pipe_mask = poll_config.pipe_mask;
        state.poll_report_every = poll_config.report_every;
        state.max_retries = poll_config.max_retries;
        state.ack_timeout_us = poll_config.ack_timeout_us;
        ptx_set_earliest_request(
            state,
            TIMESLOT_PRIORITY_NORMAL,
            poll_config.request.timeout_us,
        );
    });

    let request = PTX_STATE.with_inner(|state| core::ptr::from_ref(&state.request));
    let ret = unsafe { raw::mpsl_timeslot_request(session_id, request) };
    if let Err(e) = mpsl_ok(ret) {
        let _ = unsafe { raw::mpsl_timeslot_session_close(session_id) };
        drop(busy);
        return Err(e);
    }

    Ok(PtxPollSession {
        _busy: busy,
        session_id,
        last_counters: SignalCounters::ZERO,
        last_tx_count: 0,
        last_ack_ok_count: 0,
        last_tx_per_pipe: [0; NUM_PIPES],
        last_ack_per_pipe: [0; NUM_PIPES],
        last_ack_timeout_count: 0,
        last_ack_crc_fail_count: 0,
        last_schedule_hint_count: 0,
        last_schedule_hint_bad_count: 0,
        last_schedule: ScheduleTrackerSnapshot::ZERO,
        last_schedule_skip_count: 0,
        last_schedule_lock_count: 0,
        last_schedule_reacquire_count: 0,
        last_ack_timeout_per_pipe: [0; NUM_PIPES],
        last_ack_crc_fail_per_pipe: [0; NUM_PIPES],
    })
}

// ---- Event-driven PTX ----

/// Result of a single event-driven PTX send attempt.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtxSendResult {
    /// Whether the packet was ACKed by the PRX.
    pub ack_ok: bool,
    /// Signal counters observed during this send attempt.
    pub counters: SignalCounters,
    /// Valid PRX schedule hint carried in the ACK payload, if present.
    pub schedule_hint: Option<ScheduleHint>,
    /// Caller-defined bytes carried after the PRX scheduler hint in its ACK
    /// payload. The ESB/MPSL adapter does not interpret this extension.
    pub ack_extension: Option<PrxAckExtension>,
}

/// A long-lived event-driven PTX session.
///
/// The session keeps the MPSL timeslot session open but does **not**
/// continuously chain slots. Instead, each `send()` call requests one
/// timeslot, transmits the pending payload, and returns the ACK result.
///
/// If ACK is not received, the caller should retry by calling `send()`
/// again after a short delay to align with the next PRX receive window.
pub struct PtxEventSession {
    _busy: BusyGuard,
    session_id: u8,
    pid: u8,
    tx_count: u32,
}

/// Open an event-driven PTX session.
///
/// Unlike `open_ptx_poll_session`, this does **not** start sending
/// immediately. The caller calls `send()` when a key event (or other
/// application event) triggers a transmission.
pub fn open_event_session(
    _mpsl: &MultiprotocolServiceLayer<'_>,
    config: &EsbConfig,
    addresses: &EsbAddresses,
    event_config: PtxEventConfig,
) -> Result<PtxEventSession, Error> {
    let busy = PTX_STATE.try_enter()?;
    if config.payload_length < 4 {
        drop(busy);
        return Err(Error::InvalidParam);
    }
    if let Err(e) = event_config.validate() {
        drop(busy);
        return Err(e);
    }

    let mut session_id: u8 = 0;
    let ret = unsafe {
        raw::mpsl_timeslot_session_open(Some(ptx_timeslot_callback), (&mut session_id) as *mut _)
    };
    if let Err(e) = mpsl_ok(ret) {
        drop(busy);
        return Err(e);
    }

    PTX_STATE.with_inner(|state| {
        // Clear all runtime accumulators/flags from any prior session.
        state.reset_runtime();

        // Apply this event session's configuration.
        state.slot_length_us = event_config.slot_length_us;
        state.in_slot_match_us = event_config.in_slot_match_us;
        state.config = Some(config.clone());
        state.addresses = Some(addresses.clone());
        state.tx_pipe = event_config.pipe;
        state.packets_per_slot = 1;
        state.schedule_gate = PtxScheduleGateConfig::disabled();
        state.max_retries = event_config.max_retries;
        state.ack_timeout_us = event_config.ack_timeout_us;
        state.event_mode = true;
        state.request.request_type = raw::MPSL_TIMESLOT_REQ_TYPE_EARLIEST as u8;
        state.request.params.earliest.hfclk = raw::MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED as u8;
        state.request.params.earliest.priority = raw::MPSL_TIMESLOT_PRIORITY_NORMAL as u8;
        state.request.params.earliest.length_us = event_config.slot_length_us;
        state.request.params.earliest.timeout_us = event_config.request.timeout_us;
    });

    Ok(PtxEventSession {
        _busy: busy,
        session_id,
        pid: 0,
        tx_count: 0,
    })
}

impl PtxEventSession {
    /// Send a key report payload and wait for ACK.
    ///
    /// `payload` is written into the ESB TX buffer starting at the
    /// payload offset. The first byte after the ESB header will be
    /// the payload length seen by the PRX.
    ///
    /// If ACK is not received within the timeslot (including in-slot
    /// retries), returns `ack_ok: false`. The caller should retry.
    pub async fn send(&mut self, payload: &[u8]) -> Result<PtxSendResult, Error> {
        let max_app_payload = PTX_STATE.with_inner(|state| {
            state
                .config
                .as_ref()
                .map(|c| c.payload_length as usize)
                .unwrap_or(0)
                .saturating_sub(4)
        });
        if payload.len() > max_app_payload {
            return Err(Error::InvalidParam);
        }

        let staged = PTX_STATE.with_inner(|state| {
            let tx_buf = unsafe { &mut *PTX_BUFS.tx.get() };
            if !write_counter_payload_packet(tx_buf, self.pid, self.tx_count, payload) {
                return false;
            }

            state.event_pending = true;
            state.event_result_ready = false;
            state.event_ack_ok = false;
            state.last_schedule_hint = None;
            state.last_ack_extension = None;
            state.done = false;
            state.counters = SignalCounters::ZERO;
            true
        });
        if !staged {
            return Err(Error::InvalidParam);
        }

        let request = PTX_STATE.with_inner(|state| core::ptr::from_ref(&state.request));
        let ret = unsafe { raw::mpsl_timeslot_request(self.session_id, request) };
        if let Err(e) = mpsl_ok(ret) {
            #[cfg(feature = "defmt")]
            defmt::warn!("ptx event request failed ret={} err={:?}", ret, e);
            PTX_STATE.with_inner(|state| {
                state.event_pending = false;
                state.event_result_ready = false;
                state.event_ack_ok = false;
                state.last_ack_extension = None;
            });
            return Err(e);
        }

        poll_fn(|cx| {
            PTX_STATE.with_inner(|state| {
                state.waker.register(cx.waker());
                if state.event_result_ready || state.done {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await;

        let result = PTX_STATE.with_inner(|state| {
            if state.done && !state.event_result_ready {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "ptx event ended without result cnt s={} t0={} rd={} bk={} cn={} idle={} closed={} over={} inv={} extf={} exts={} rdt={}",
                    state.counters.start,
                    state.counters.timer0,
                    state.counters.radio,
                    state.counters.blocked,
                    state.counters.cancelled,
                    state.counters.session_idle,
                    state.counters.session_closed,
                    state.counters.overstayed,
                    state.counters.invalid_return,
                    state.counters.extend_failed,
                    state.counters.extend_succeeded,
                    state.counters.radio_disable_timeout,
                );
                return Err(Error::Mpsl);
            }

            let ack_ok = state.event_ack_ok;
            if ack_ok {
                self.pid = advance_pid(self.pid);
            }
            self.tx_count += 1;

            let counters = state.counters;
            let schedule_hint = state.last_schedule_hint;
            let ack_extension = state.last_ack_extension;

            state.event_result_ready = false;
            state.event_pending = false;

            Ok(PtxSendResult {
                ack_ok,
                counters,
                schedule_hint,
                ack_extension,
            })
        })?;

        Ok(result)
    }
}

impl Drop for PtxEventSession {
    fn drop(&mut self) {
        PTX_STATE.with_inner(|state| {
            state.driver = None;
            state.event_mode = false;
            state.event_pending = false;
        });
        let _ = unsafe { raw::mpsl_timeslot_session_close(self.session_id) };
    }
}
