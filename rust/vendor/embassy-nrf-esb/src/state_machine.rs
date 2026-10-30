//! ESB PTX and PRX state machines.
//!
//! Implements the ESB protocol state machines for Primary Transmitter (PTX)
//! and Primary Receiver (PRX) roles. All state machine logic runs in a single
//! ISR context (RADIO ISR) - the TIMER ISR only sets a flag and pends the
//! RADIO ISR (R9). The exclusive owning facade also processes TX events and
//! quiescent role transitions in task context with both IRQs/PRIMASK masked.
//!
//! The state machines are pure event processors: they receive event flags
//! rather than owning shared state. This avoids self-referential struct
//! issues and makes the code testable.
//!
//! Ref: esb-ng `src/irq.rs` lines 155–425.

use crate::addresses::EsbAddresses;
use crate::config::EsbConfig;
use crate::error::Error;
use crate::header::EsbHeader;
use crate::payload::PacketPool;
use crate::radio::{EsbRadio, RxResult, StopOutcome};
use crate::suspend::{EsbSavedState, SavedProtocolState};
use crate::timer::EsbTimer;
use crate::timer::TimerInstance;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ExclusiveSendError {
    Driver(Error),
    Guard(crate::esb::DeadlineGuardError),
}

/// Exclusive production launch ordering. The caller already owns the DMA claim
/// and has published TxNoAck. Neither failure nor Scheduled releases that claim.
pub(crate) fn prepare_and_launch<C>(
    context: &mut C,
    prepare: impl FnOnce(&mut C),
    launch: impl FnOnce() -> Result<crate::esb::TxLaunch, crate::esb::DeadlineGuardError>,
    launch_cpu: impl FnOnce(&mut C),
) -> Result<(), crate::esb::DeadlineGuardError> {
    prepare(context);
    match launch()? {
        crate::esb::TxLaunch::Cpu => launch_cpu(context),
        crate::esb::TxLaunch::Scheduled => {}
    }
    Ok(())
}

// ---- PTX States ----

/// PTX state machine states.
///
/// Ref: esb-ng `src/irq.rs` lines 17–28.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StatePtx {
    /// Radio idle — waiting for TX queue entry.
    #[default]
    Idle,
    /// Transmitting a packet (ACK requested).
    Tx,
    /// Transmitting a NoAck packet (no ACK expected).
    TxNoAck,
    /// Waiting for ACK response after TX.
    WaitAck,
    /// Waiting for retransmit timeout.
    WaitRetransmit,
}

// ---- PRX States ----

/// PRX state machine states.
///
/// Ref: esb-ng `src/irq.rs` lines 31–41.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StatePrx {
    /// Radio idle — not listening.
    #[default]
    Idle,
    /// Listening for incoming packets.
    Receiver,
    /// Transmitting ACK for a new packet.
    TxAck,
    /// Transmitting ACK for a duplicate (repeated) packet.
    TxRepeatedAck,
}

// ---- ISR Event Flags ----

/// Events checked at the start of each ISR invocation.
///
/// Passed to the state machine from the ISR wrapper.
/// Ref: esb-ng `src/irq.rs` lines 105–108.
#[derive(Debug, Clone, Copy)]
pub(crate) struct IsrEvents {
    /// RADIO EVENTS_DISABLED fired.
    pub disabled: bool,
    /// TIMER ISR set the flag and pended RADIO ISR.
    pub timer: bool,
}

impl IsrEvents {
    /// Neither disabled nor timer — user-triggered event.
    pub(crate) fn user_event(self) -> bool {
        !self.disabled && !self.timer
    }
}

/// Result of PTX event handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PtxEvent {
    /// No special event — state machine progressed normally.
    None,
    /// Max retransmit attempts reached — packet dropped.
    MaxAttempts,
}

/// Result of PRX event handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PrxEvent {
    /// No special event.
    None,
    /// New valid packet received and queued.
    Received,
    /// Duplicate packet detected — ACK sent but no new data.
    Duplicate,
    /// NoAck packet received — queued but no ACK sent.
    ReceivedNoAck,
    /// CRC check failed — radio auto-restarted.
    BadCrc,
}

#[cfg(any(test, feature = "mpsl"))]
#[derive(Clone, Copy)]
pub struct TimeslotDiagAckContext {
    pub window_id: u32,
    pub next_delay_us: u32,
    pub period_us: u32,
    pub window_us: u32,
}

/// Sentinel value meaning "no active DMA buffer".
const NO_IDX: usize = usize::MAX;

// ---- PTX State Machine ----

/// PTX (Primary Transmitter) state machine.
///
/// Holds radio, timer, and protocol state. All methods are called from
/// ISR context only (R9).
///
/// Ref: esb-ng `src/irq.rs` lines 191–310.
#[allow(dead_code)]
pub(crate) struct PtxStateMachine<T: TimerInstance> {
    pub(crate) radio: EsbRadio,
    pub(crate) timer: EsbTimer<T>,
    config: EsbConfig,
    /// ESB addresses. Kept in the SM so the timeslot entrypoints can re-init
    /// the RADIO at the start of every slot (the MPSL path power-cycles RADIO
    /// between slots). Filled in by `set_addresses` at wrap time.
    addresses: Option<EsbAddresses>,
    state: StatePtx,
    /// Retransmit attempt counter for the current packet.
    attempts: u8,
    /// TX pipe (typically 0 for PTX).
    pub(crate) tx_pipe: u8,
    /// Pipe used by the packet currently in flight.
    active_tx_pipe: u8,
    /// Retransmit delay in µs (from config, minus ramp-up).
    retransmit_delay_us: u16,
    /// ACK timeout in µs (from config, plus ramp-up).
    ack_timeout_us: u16,
    /// Maximum retransmit attempts.
    max_attempts: u8,
    /// 2-bit packet identifier for duplicate detection (incremented per new packet).
    pid: u8,
    /// Current TX packet pool index (IN_DMA while transmitting).
    tx_idx: usize,
    /// Current ACK RX buffer pool index (IN_DMA while waiting for ACK).
    ack_rx_idx: usize,
    /// Last pipe a TX was sent on (0xFF = none yet). Exposed for timeslot-mode
    /// per-pipe diagnostics.
    last_pipe: u8,
    /// MPSL timeslot-managed mode. When `true`, this state machine is driven
    /// from the MPSL timeslot callback (`SIGNAL_RADIO`) rather than the RADIO
    /// ISR, and timing comes from the timeslot layer (TIMER0 / in-slot poll)
    /// instead of the `EsbTimer` PPI path. Defaults to `false` (exclusive mode).
    /// See `docs/single-engine-convergence-plan.md` (S2/S6).
    timeslot_managed: bool,
    no_ack_completion: Option<StopOutcome>,
}

#[allow(dead_code)]
impl<T: TimerInstance> PtxStateMachine<T> {
    /// Create a new PTX state machine.
    pub(crate) fn new(
        radio: EsbRadio,
        timer: EsbTimer<T>,
        config: &EsbConfig,
        tx_pipe: u8,
    ) -> Self {
        Self {
            radio,
            timer,
            config: config.clone(),
            addresses: None,
            state: StatePtx::Idle,
            attempts: 0,
            tx_pipe,
            active_tx_pipe: tx_pipe,
            // Timer calculations (R8):
            // Retransmit: subtract ramp-up (radio re-enables from DISABLED)
            retransmit_delay_us: config.retransmit_timer_delay_us(),
            // ACK timeout: add ramp-up (radio ramps to RX)
            ack_timeout_us: config.ack_timer_timeout_us(),
            max_attempts: config.retransmit.count,
            pid: 0,
            tx_idx: NO_IDX,
            ack_rx_idx: NO_IDX,
            last_pipe: 0xFF,
            timeslot_managed: false,
            no_ack_completion: None,
        }
    }

    /// Get current PTX state.
    pub(crate) fn state(&self) -> StatePtx {
        self.state
    }

    /// Enable or disable MPSL timeslot-managed operation.
    ///
    /// When `true`, the state machine expects to be driven from the MPSL
    /// timeslot callback (manual radio turnaround, timeslot-layer timing) rather
    /// than the RADIO/TIMER ISR path. The manual turnaround branches are wired in
    /// S4/S6; today this only records the mode for the timeslot wrapper (S3).
    pub(crate) fn set_timeslot_managed(&mut self, enabled: bool) {
        self.timeslot_managed = enabled;
    }

    /// Returns whether MPSL timeslot-managed operation is enabled.
    pub(crate) fn is_timeslot_managed(&self) -> bool {
        self.timeslot_managed
    }

    /// Store ESB addresses for timeslot-mode RADIO re-init.
    ///
    /// In exclusive mode the RADIO is initialized once at construction; in
    /// timeslot mode each slot power-cycles RADIO, so the entrypoints need the
    /// addresses to re-run `init`. Set once at wrap time.
    pub(crate) fn set_addresses(&mut self, addresses: &EsbAddresses) {
        self.addresses = Some(addresses.clone());
    }

    /// Update the TX pipe (used by the timeslot wrapper when arming a slot).
    pub(crate) fn set_tx_pipe(&mut self, pipe: u8) {
        self.tx_pipe = pipe;
    }

    /// Returns the saved duplicate-detection PID array (for cross-slot
    /// preservation by the timeslot wrapper).
    pub(crate) fn saved_pid(&self) -> [u8; 8] {
        self.radio.save_pid_state()
    }

    /// Returns the saved duplicate-detection CRC array.
    pub(crate) fn saved_crc(&self) -> [u16; 8] {
        self.radio.save_crc_state()
    }

    /// Returns the saved duplicate-detection valid-flags array.
    pub(crate) fn saved_valid(&self) -> [bool; 8] {
        self.radio.save_detection_valid_state()
    }

    /// Returns the last pipe a TX was sent on (0xFF if none yet). Diagnostic only.
    pub(crate) fn last_pipe(&self) -> u8 {
        self.last_pipe
    }

    /// Returns the current PID (2-bit packet identifier).
    pub(crate) fn pid(&self) -> u8 {
        self.pid
    }

    /// Returns the current retransmit attempt count for this packet.
    pub(crate) fn attempts(&self) -> u8 {
        self.attempts
    }

    /// Timeslot entrypoint: slot start.
    ///
    /// Power-cycles RADIO, re-inits ESB registers, restores duplicate-detection
    /// state, sets `timeslot_managed = true`, and arms the first TX. Must be
    /// called from the MPSL signal context (not the RADIO ISR). The caller
    /// passes the saved PID/CRC/valid arrays (last slot's final state).
    ///
    /// After this returns, the caller (mpsl layer) is responsible for:
    /// - Programming TIMER0 CC[0] for slot end and CC[1] for ACK timeout
    /// - Unmasking the RADIO NVIC line so events arrive as SIGNAL_RADIO
    /// - Driving retransmit timing via TIMER0 (D1=B2: protocol logic in the SM,
    ///   timing source split by mode)
    pub(crate) fn ts_start_tx<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), Error> {
        let addresses = self
            .addresses
            .clone()
            .expect("ts_start_tx: addresses must be set via set_addresses first");

        self.radio.power_cycle();
        self.radio.init(&self.config, &addresses);
        self.radio.restore_pid_state(saved_pid);
        self.radio.restore_crc_state(saved_crc);
        self.radio.restore_detection_valid_state(saved_valid);

        self.timeslot_managed = true;
        self.state = StatePtx::Idle;
        self.attempts = 0;
        self.tx_idx = NO_IDX;
        self.ack_rx_idx = NO_IDX;

        // Arm first TX (dequeues from pool if available).
        self.send_next(pool);
        if self.state == StatePtx::Tx || self.state == StatePtx::TxNoAck {
            self.last_pipe = self.active_tx_pipe;
        }
        Ok(())
    }

    /// Timeslot entrypoint: SIGNAL_RADIO.
    ///
    /// Drives one state-machine event. `timer_flag` is `true` when the mpsl
    /// layer's TIMER0 CC[1] (ACK timeout) fired before this RADIO event; the
    /// caller reads and clears TIMER0 events before calling. Returns the PTX
    /// event so the mpsl layer can update diagnostic counters and decide
    /// whether to re-arm TIMER0 for a retransmit.
    pub(crate) fn ts_on_radio<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        timer_flag: bool,
    ) -> PtxEvent {
        if self.state == StatePtx::Idle && !timer_flag {
            return PtxEvent::None;
        }

        let event = self.handle_radio_event(pool, timer_flag, false);

        if self.state == StatePtx::Tx || self.state == StatePtx::TxNoAck {
            self.last_pipe = self.active_tx_pipe;
        }

        event
    }

    /// Timeslot entrypoint: slot end / EXTEND_FAILED / OVERSTAYED / handoff.
    ///
    /// Stops the radio, releases any in-flight TX/ACK-RX buffer, and clears
    /// `timeslot_managed`. After this, the SM is in `Idle` and ready for the
    /// next slot.
    pub(crate) fn ts_force_stop<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) {
        self.radio.stop();
        self.release_tx(pool);
        self.release_ack_rx(pool);
        self.state = StatePtx::Idle;
        self.attempts = 0;
        self.timeslot_managed = false;
    }

    /// Check and clear ISR event flags.
    ///
    /// Returns events with disabled flag from radio, timer flag from caller.
    /// Leave END latched across timer IRQs until quiescence or the next TX handoff.
    /// Ref: esb-ng `src/irq.rs` lines 135–152.
    pub(crate) fn check_events(&mut self, timer_flag: bool) -> IsrEvents {
        let disabled = self.radio.check_disabled_event();
        if disabled {
            self.radio.clear_disabled_event();
        }
        IsrEvents {
            disabled,
            timer: timer_flag,
        }
    }

    /// Handle a RADIO ISR event.
    ///
    /// Must be called from the RADIO ISR only. Reads events, transitions
    /// state, and operates radio/timer registers inline (A3).
    ///
    /// `timer_flag` is the value of the shared timer AtomicBool, already
    /// loaded and cleared by the ISR wrapper.
    ///
    /// Ref: esb-ng `src/irq.rs` lines 196–294.
    pub(crate) fn handle_radio_event<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        timer_flag: bool,
        suppress_next_tx: bool,
    ) -> PtxEvent {
        self.handle_radio_event_with_completion(pool, timer_flag, suppress_next_tx, || {})
    }

    /// Exclusive facade disarms its autonomous route before stop/claim release.
    pub(crate) fn handle_radio_event_with_completion<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        timer_flag: bool,
        suppress_next_tx: bool,
        before_completion: impl FnOnce(),
    ) -> PtxEvent {
        let evts = self.check_events(timer_flag);

        // If neither disabled nor timer, it's a user-triggered event
        // (e.g., new packet enqueued). Only valid in Idle state.
        if evts.user_event() && self.state != StatePtx::Idle {
            return PtxEvent::None;
        }

        match self.state {
            StatePtx::Idle => {
                if !suppress_next_tx {
                    self.send_next(pool);
                }
                PtxEvent::None
            }

            StatePtx::TxNoAck => {
                // DISABLED alone may be an abort, not a full packet transmission.
                if !no_ack_tx_completed(self.state, evts, self.radio.check_end_event()) {
                    return PtxEvent::None;
                }
                before_completion();
                let stop = self.radio.stop();
                // TX END for NoAck packet — release and send next
                // (esb-ng lines 215–219).
                self.radio.finish_tx_no_ack();
                self.release_tx(pool);
                self.advance_pid();
                self.no_ack_completion = Some(stop);
                if suppress_next_tx {
                    self.go_idle();
                } else {
                    self.send_next(pool);
                }
                PtxEvent::None
            }

            StatePtx::Tx => {
                // TX END — prepare for ACK reception (esb-ng lines 220–244).
                debug_assert!(evts.disabled, "Tx: expected disabled event");

                let ack_idx = alloc_dma_buffer(pool);
                if let Some(idx) = ack_idx {
                    let dma_ptr = unsafe { pool.dma_ptr(idx) };
                    self.radio.prepare_for_ack(dma_ptr);
                    self.ack_rx_idx = idx;
                    self.state = StatePtx::WaitAck;
                } else {
                    self.radio.stop();
                    // Keep tx_idx set — send_next() will resend it
                    self.state = StatePtx::Idle;
                    return PtxEvent::None;
                }

                // Arm both timers (R8).
                self.timer.arm_retransmit(self.retransmit_delay_us);
                self.timer.arm_ack_timeout(self.ack_timeout_us);

                PtxEvent::None
            }

            StatePtx::WaitAck => {
                if evts.disabled {
                    // Got something — check CRC (esb-ng lines 249–259).
                    self.timer.disarm_ack_timeout();

                    if self.radio.check_ack() {
                        self.timer.disarm_retransmit();
                        self.radio.stop();

                        // Check if ACK contains payload data
                        if self.ack_rx_idx != NO_IDX {
                            let header = unsafe { pool.header(self.ack_rx_idx) };
                            if header.length > 0 {
                                // ACK has payload — deliver to app via rx_queue
                                pool.rx_complete(self.ack_rx_idx, None);
                                self.ack_rx_idx = NO_IDX;
                            } else {
                                self.release_ack_rx(pool);
                            }
                        }

                        self.release_tx(pool);
                        self.attempts = 0;
                        self.advance_pid();
                        if suppress_next_tx {
                            self.go_idle();
                        } else {
                            self.send_next(pool);
                        }
                        return PtxEvent::None;
                    }
                    // CRC mismatch — stop radio, release ACK buffer, fall through
                    self.radio.stop();
                    self.release_ack_rx(pool);
                } else if evts.timer {
                    // ACK timeout (esb-ng line 263).
                    // Don't disarm retransmit timer — it's still counting down
                    // and will trigger the WaitRetransmit state.
                    self.timer.disarm_ack_timeout();
                    self.radio.stop();
                    self.release_ack_rx(pool);
                } else {
                    return PtxEvent::None;
                }

                // Retransmit needed — check max attempts FIRST (esb-ng line 270).
                // R6: use >= not > — esb-ng uses > giving one extra attempt.
                self.attempts += 1;
                if self.attempts >= self.max_attempts {
                    // Max reached — drop packet, try next
                    self.release_tx(pool);
                    self.attempts = 0;
                    self.advance_pid();
                    if suppress_next_tx {
                        self.go_idle();
                    } else {
                        self.send_next(pool);
                    }
                    return PtxEvent::MaxAttempts;
                }

                // Retransmit — wait for timer (radio already stopped above)
                self.state = StatePtx::WaitRetransmit;

                PtxEvent::None
            }

            StatePtx::WaitRetransmit => {
                debug_assert!(evts.timer, "WaitRetransmit: expected timer event");
                self.retransmit(pool);
                PtxEvent::None
            }
        }
    }

    /// Transition to idle without dequeuing the next packet.
    fn go_idle(&mut self) {
        self.radio.disable_disabled_interrupt();
        self.state = StatePtx::Idle;
    }

    /// Send the next queued packet, or go idle if queue is empty.
    /// If tx_idx is already set (from a previous failed ACK buffer alloc),
    /// resend that packet instead of dequeuing a new one.
    /// Ref: esb-ng `src/irq.rs` lines 296–309.
    fn send_next<const N: usize, const SIZE: usize>(&mut self, pool: &PacketPool<N, SIZE>) {
        // If a TX packet is pending from a failed ACK buffer allocation,
        // resend it (tx_idx is still in IN_DMA state).
        if self.tx_idx != NO_IDX {
            let dma_ptr = unsafe { pool.dma_ptr(self.tx_idx) };
            self.radio.transmit(self.active_tx_pipe, dma_ptr, true);
            self.state = StatePtx::Tx;
            return;
        }

        if let Some(idx) = pool.try_dequeue_tx() {
            let header = unsafe { pool.header_mut(idx) };
            let no_ack = header.no_ack();
            self.active_tx_pipe = header.pipe;
            // Set PID in header before TX (S1 field bits 2:1).
            header.set_pid(self.pid);
            let dma_ptr = unsafe { pool.dma_ptr(idx) };
            self.tx_idx = idx;
            self.radio.transmit(self.active_tx_pipe, dma_ptr, !no_ack);
            self.state = if no_ack {
                StatePtx::TxNoAck
            } else {
                StatePtx::Tx
            };
        } else {
            self.radio.disable_disabled_interrupt();
            self.state = StatePtx::Idle;
        }
    }

    /// Advance PID after a packet is fully processed (ACK received,
    /// max retransmit reached, or NoAck TX completed).
    fn advance_pid(&mut self) {
        self.pid = (self.pid + 1) & 0x03;
    }

    /// Retransmit the current TX packet.
    fn retransmit<const N: usize, const SIZE: usize>(&mut self, pool: &PacketPool<N, SIZE>) {
        if self.tx_idx != NO_IDX {
            // SAFETY: tx_idx is in IN_DMA state; buffer data is still valid.
            let dma_ptr = unsafe { pool.dma_ptr(self.tx_idx) };
            self.radio.transmit(self.active_tx_pipe, dma_ptr, true);
            self.state = StatePtx::Tx;
        } else {
            self.radio.disable_disabled_interrupt();
            self.state = StatePtx::Idle;
        }
    }

    /// Release the current TX buffer back to the pool.
    fn release_tx<const N: usize, const SIZE: usize>(&mut self, pool: &PacketPool<N, SIZE>) {
        if self.tx_idx != NO_IDX {
            pool.release_tx(self.tx_idx);
            self.tx_idx = NO_IDX;
        }
    }

    /// Release the current ACK RX buffer back to the pool.
    fn release_ack_rx<const N: usize, const SIZE: usize>(&mut self, pool: &PacketPool<N, SIZE>) {
        if self.ack_rx_idx != NO_IDX {
            pool.release_rx(self.ack_rx_idx);
            self.ack_rx_idx = NO_IDX;
        }
    }

    /// Handle TIMER ISR — clear events, pend RADIO ISR.
    /// Ref: esb-ng `src/irq.rs` lines 49–66.
    pub(crate) fn handle_timer_event(&self) {
        if self.timer.is_retransmit_fired() {
            self.timer.disarm_retransmit();
        }
        if self.timer.is_ack_timeout_fired() {
            self.timer.disarm_ack_timeout();
        }
        pend_radio_isr();
    }

    /// Trigger a send from application context (pend RADIO ISR).
    pub(crate) fn trigger_send(&self) {
        pend_radio_isr();
    }

    // ---- Suspend / Resume ----

    pub(crate) fn start_exclusive_no_ack_with_launch<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        pipe: u8,
        payload: &[u8],
        launch: impl FnOnce() -> Result<crate::esb::TxLaunch, crate::esb::DeadlineGuardError>,
    ) -> Result<(), ExclusiveSendError> {
        if self.state != StatePtx::Idle || self.tx_idx != NO_IDX || self.ack_rx_idx != NO_IDX {
            return Err(ExclusiveSendError::Driver(Error::Busy));
        }
        let idx = pool
            .alloc_tx()
            .ok_or(ExclusiveSendError::Driver(Error::TxFull))?;
        let header = unsafe { pool.header_mut(idx) };
        header.pipe = pipe;
        header.length = payload.len() as u8;
        header.set_no_ack(true);
        header.set_pid(self.pid);
        let buffer = unsafe { pool.buf_mut(idx) };
        buffer[EsbHeader::PAYLOAD_OFFSET..EsbHeader::PAYLOAD_OFFSET + payload.len()]
            .copy_from_slice(payload);
        pool.claim_allocated_tx(idx);
        self.no_ack_completion = None;
        self.tx_idx = idx;
        self.active_tx_pipe = pipe;
        self.state = StatePtx::TxNoAck;
        prepare_and_launch(
            &mut self.radio,
            |radio| radio.prepare_transmit(pipe, unsafe { pool.dma_ptr(idx) }, false),
            launch,
            |radio| radio.launch_transmit_cpu(),
        )
        .map_err(ExclusiveSendError::Guard)
    }

    pub(crate) fn take_no_ack_completion(&mut self) -> Option<StopOutcome> {
        self.no_ack_completion.take()
    }

    /// Caller masks both IRQs and PRIMASK. Claims remain owned until stop returns.
    pub(crate) fn quiesce<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> StopOutcome {
        let stop = self.radio.stop();
        if self.tx_idx != NO_IDX {
            // An aborted packet may already have reached a peer; don't reuse PID.
            self.advance_pid();
        }
        self.release_tx(pool);
        self.release_ack_rx(pool);
        self.timer.clear_all_events();
        self.radio.clear_all_events();
        self.state = StatePtx::Idle;
        self.attempts = 0;
        self.no_ack_completion = None;
        stop
    }

    pub(crate) fn into_parts(self) -> (EsbRadio, EsbTimer<T>, u8) {
        assert!(self.state == StatePtx::Idle && self.tx_idx == NO_IDX && self.ack_rx_idx == NO_IDX);
        assert!(self.radio.is_quiescent());
        (self.radio, self.timer, self.pid)
    }

    pub(crate) fn from_parts(
        radio: EsbRadio,
        timer: EsbTimer<T>,
        config: &EsbConfig,
        next_pid: u8,
    ) -> Self {
        let mut sm = Self::new(radio, timer, config, 0);
        sm.pid = next_pid & 3;
        sm
    }

    /// Save ESB state and stop hardware. Called with RADIO IRQ already disabled.
    pub(crate) fn do_suspend<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> EsbSavedState {
        self.radio.stop();
        let protocol_state = if self.state == StatePtx::Idle {
            SavedProtocolState::Idle
        } else {
            let attempt = self.attempts;
            self.release_tx(pool);
            self.release_ack_rx(pool);
            SavedProtocolState::ForcedIdle {
                dropped_attempt: attempt,
            }
        };

        self.timer.stop();
        self.timer.disarm_retransmit();
        self.timer.disarm_ack_timeout();

        let saved = EsbSavedState {
            pid: self.radio.save_pid_state(),
            last_crc: self.radio.save_crc_state(),
            last_valid: self.radio.save_detection_valid_state(),
            tx_pipe: self.tx_pipe,
            attempts: self.attempts,
            protocol_state,
        };

        self.state = StatePtx::Idle;
        self.attempts = 0;
        self.tx_idx = NO_IDX;
        self.ack_rx_idx = NO_IDX;

        saved
    }

    /// Full re-init and restore from saved state.
    /// Called with RADIO IRQ disabled; caller re-enables after return.
    pub(crate) fn do_restore(&mut self, state: &EsbSavedState, addresses: &EsbAddresses) {
        self.radio.power_cycle();
        self.radio.init(&self.config, addresses);
        self.radio.restore_pid_state(state.pid);
        self.radio.restore_crc_state(state.last_crc);
        self.radio.restore_detection_valid_state(state.last_valid);
        self.tx_pipe = state.tx_pipe;
        self.active_tx_pipe = state.tx_pipe;
        self.state = StatePtx::Idle;
    }
}

// ---- PRX State Machine ----

/// PRX (Primary Receiver) state machine.
/// Ref: esb-ng `src/irq.rs` lines 312–425.
#[allow(dead_code)]
pub(crate) struct PrxStateMachine<T: TimerInstance> {
    pub(crate) radio: EsbRadio,
    pub(crate) timer: EsbTimer<T>,
    config: EsbConfig,
    /// ESB addresses. Kept in the SM so the timeslot entrypoints can re-init
    /// the RADIO at the start of every slot (the MPSL path power-cycles RADIO
    /// between slots). Filled in by `set_addresses` at wrap time.
    addresses: Option<EsbAddresses>,
    state: StatePrx,
    /// Enabled pipe bitmask.
    enabled_pipes: u8,
    /// Current RX DMA buffer pool index.
    rx_idx: usize,
    /// Current received packet pool index (queued for app after RX).
    pending_rx_idx: usize,
    /// Current ACK TX buffer pool index (IN_DMA while sending ACK payload).
    ack_tx_idx: usize,
    /// Last pipe processed in `handle_radio_event` (0xFF = none yet). Exposed
    /// for timeslot-mode diagnostic counters.
    last_pipe: u8,
    #[cfg(feature = "mpsl")]
    diag_ack_context: Option<TimeslotDiagAckContext>,
    #[cfg(feature = "mpsl")]
    diag_ack_counter: [u32; 8],
    /// MPSL timeslot-managed mode. When `true`, this state machine is driven
    /// from the MPSL timeslot callback (`SIGNAL_RADIO`) and uses manual ACK
    /// turnaround (`start_receiving_manual_ack` / `transmit_ack_manual`) instead
    /// of the auto `disabled_txen`/`disabled_rxen` shortcuts. Defaults to `false`
    /// (exclusive mode). See `docs/single-engine-convergence-plan.md` (S2/S4).
    timeslot_managed: bool,
    rx_metadata_context: Option<crate::rx_metadata::RxMetadataContext>,
}

#[allow(dead_code)]
impl<T: TimerInstance> PrxStateMachine<T> {
    /// Create a new PRX state machine.
    pub(crate) fn new(
        radio: EsbRadio,
        timer: EsbTimer<T>,
        config: &EsbConfig,
        enabled_pipes: u8,
    ) -> Self {
        Self {
            radio,
            timer,
            config: config.clone(),
            addresses: None,
            state: StatePrx::Idle,
            enabled_pipes,
            rx_idx: NO_IDX,
            pending_rx_idx: NO_IDX,
            ack_tx_idx: NO_IDX,
            last_pipe: 0xFF,
            #[cfg(feature = "mpsl")]
            diag_ack_context: None,
            #[cfg(feature = "mpsl")]
            diag_ack_counter: [0; 8],
            timeslot_managed: false,
            rx_metadata_context: None,
        }
    }

    /// Get current PRX state.
    pub(crate) fn state(&self) -> StatePrx {
        self.state
    }

    /// Only install a new session while no RX DMA claim is active.
    pub(crate) fn set_rx_metadata_context(
        &mut self,
        context: Option<crate::rx_metadata::RxMetadataContext>,
    ) {
        assert!(
            self.state == StatePrx::Idle
                && self.rx_idx == NO_IDX
                && self.pending_rx_idx == NO_IDX
                && self.ack_tx_idx == NO_IDX
        );
        self.rx_metadata_context = context;
    }

    /// Enable or disable MPSL timeslot-managed operation.
    ///
    /// When `true`, the state machine expects to be driven from the MPSL
    /// timeslot callback and to use manual ACK turnaround instead of the auto
    /// `disabled_txen`/`disabled_rxen` shortcuts. The manual turnaround branches
    /// are wired in S4; today this only records the mode for the wrapper (S3).
    pub(crate) fn set_timeslot_managed(&mut self, enabled: bool) {
        self.timeslot_managed = enabled;
    }

    /// Returns whether MPSL timeslot-managed operation is enabled.
    pub(crate) fn is_timeslot_managed(&self) -> bool {
        self.timeslot_managed
    }

    /// Check and clear ISR event flags.
    pub(crate) fn check_events(&mut self, timer_flag: bool) -> IsrEvents {
        let disabled = self.radio.check_disabled_event();
        if disabled {
            self.radio.clear_disabled_event();
        }
        IsrEvents {
            disabled,
            timer: timer_flag,
        }
    }

    /// Start receiving on enabled pipes.
    /// Ref: esb-ng `src/irq.rs` lines 386–395.
    pub(crate) fn start_receiving<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> Result<(), Error> {
        if self.state != StatePrx::Idle {
            return Ok(());
        }

        let Some(idx) = alloc_dma_buffer(pool) else {
            #[cfg(feature = "rx-blocking-trace")]
            pool.record_rx(|c| c.start_alloc_failed = c.start_alloc_failed.wrapping_add(1));
            return Err(Error::OutOfMemory);
        };
        self.rx_idx = idx;
        let dma_ptr = unsafe { pool.dma_ptr(idx) };
        self.arm_rx(dma_ptr);
        self.state = StatePrx::Receiver;
        Ok(())
    }

    /// Store ESB addresses for timeslot-mode RADIO re-init.
    ///
    /// In exclusive mode the RADIO is initialized once at construction; in
    /// timeslot mode each slot power-cycles RADIO, so the entrypoints need the
    /// addresses to re-run `init`. Set once at wrap time.
    pub(crate) fn set_addresses(&mut self, addresses: &EsbAddresses) {
        self.addresses = Some(addresses.clone());
    }

    /// Update the enabled-pipe mask (used by the timeslot wrapper when opening
    /// a session; exclusive mode sets this once at construction).
    pub(crate) fn set_enabled_pipes(&mut self, mask: u8) {
        self.enabled_pipes = mask;
    }

    /// Returns the saved duplicate-detection PID array (for cross-slot
    /// preservation by the timeslot wrapper).
    pub(crate) fn saved_pid(&self) -> [u8; 8] {
        self.radio.save_pid_state()
    }

    /// Returns the saved duplicate-detection CRC array.
    pub(crate) fn saved_crc(&self) -> [u16; 8] {
        self.radio.save_crc_state()
    }

    /// Returns the saved duplicate-detection valid-flags array.
    pub(crate) fn saved_valid(&self) -> [bool; 8] {
        self.radio.save_detection_valid_state()
    }

    /// Returns the last pipe processed (0xFF if none yet). Diagnostic only.
    pub(crate) fn last_pipe(&self) -> u8 {
        self.last_pipe
    }

    #[cfg(feature = "mpsl")]
    pub(crate) fn set_timeslot_diag_ack_context(&mut self, context: TimeslotDiagAckContext) {
        self.diag_ack_context = Some(context);
    }

    /// Timeslot entrypoint: slot start.
    ///
    /// Power-cycles RADIO, re-inits ESB registers, restores duplicate-detection
    /// state, sets `timeslot_managed = true`, and arms the first RX buffer.
    /// Must be called from the MPSL signal context (not the RADIO ISR). The
    /// caller passes the saved PID/CRC/valid arrays (last slot's final state).
    pub(crate) fn ts_start_rx<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        saved_pid: [u8; 8],
        saved_crc: [u16; 8],
        saved_valid: [bool; 8],
    ) -> Result<(), Error> {
        let addresses = self
            .addresses
            .clone()
            .expect("ts_start_rx: addresses must be set via set_addresses first");

        self.radio.power_cycle();
        self.radio.init(&self.config, &addresses);
        self.radio.restore_pid_state(saved_pid);
        self.radio.restore_crc_state(saved_crc);
        self.radio.restore_detection_valid_state(saved_valid);

        self.timeslot_managed = true;
        self.state = StatePrx::Idle;

        // Arm first RX. If buffer alloc fails the slot is wasted but not fatal.
        self.start_receiving(pool)
    }

    /// Timeslot entrypoint: SIGNAL_RADIO.
    ///
    /// Drives one state-machine event. Must be called from the MPSL signal
    /// context (which has taken the RADIO interrupt). Returns the PRX event
    /// for diagnostic counter updates in the timeslot layer.
    pub(crate) fn ts_on_radio<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> (PrxEvent, Option<usize>) {
        if self.state == StatePrx::Idle {
            return (PrxEvent::None, None);
        }
        // timer_flag is irrelevant for PRX (PRX has no ACK-timeout timer).
        self.handle_radio_event(pool, false)
    }

    /// Timeslot entrypoint: slot end / EXTEND_FAILED / OVERSTAYED / handoff.
    ///
    /// Stops the radio, releases any in-flight DMA buffer, and saves
    /// duplicate-detection state into the arrays returned by the caller.
    /// After this, the SM is in `Idle` and ready for the next slot.
    pub(crate) fn ts_force_stop<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) {
        self.stop_receiving(pool);
        self.timeslot_managed = false;
    }

    /// Timeslot diagnostic entrypoint: discard packets queued by the shared
    /// state machine so a headless PRX session cannot exhaust its RX pool.
    pub(crate) fn ts_discard_received<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) {
        pool.discard_received();
    }

    /// Handle a RADIO ISR event.
    /// Ref: esb-ng `src/irq.rs` lines 316–383.
    pub(crate) fn handle_radio_event<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        timer_flag: bool,
    ) -> (PrxEvent, Option<usize>) {
        let evts = self.check_events(timer_flag);

        if evts.user_event() && self.state != StatePrx::Idle {
            return (PrxEvent::None, None);
        }

        match self.state {
            StatePrx::Receiver => {
                debug_assert!(evts.disabled, "Receiver: expected disabled event");

                match self.radio.check_packet() {
                    RxResult::RadioFault => (PrxEvent::None, None),
                    RxResult::BadCrc => {
                        #[cfg(feature = "rx-blocking-trace")]
                        pool.record_rx(|c| c.bad_crc = c.bad_crc.wrapping_add(1));
                        // Bad CRC — radio already restarted with same PACKETPTR.
                        // Keep using the same DMA buffer. Record pipe for
                        // timeslot-mode per-pipe diagnostics.
                        self.last_pipe = self.radio.rx_match();
                        (PrxEvent::BadCrc, None)
                    }
                    RxResult::NewPacket => {
                        // No DMA ownership transfer or CPU buffer borrow before
                        // RADIO is stopped, even on duplicate/unsupported ACK paths.
                        let radio = stop_before_access(&mut self.radio, |radio| radio.stop());
                        // Power-cycling invalidated configuration/metadata. Keep
                        // the DMA claim until task service quiesces and recovers.
                        if radio.has_stop_fault() {
                            return (PrxEvent::None, None);
                        }
                        let pipe = radio.rx_match() as usize;
                        let crc = radio.rx_crc();
                        let rssi = radio.rssi_sample();
                        self.last_pipe = pipe as u8;

                        let rx_idx = self.rx_idx;
                        let header = unsafe { pool.header_mut(rx_idx) };
                        let pid = header.pid();
                        // Local no-ACK profile also delivers ACK-request packets,
                        // without pending RX, ACK payload dequeue, or fallback TX.
                        let no_ack = !self.config.should_ack(header.no_ack());

                        let is_dup = self.radio.check_duplicate(pipe, pid, crc);

                        if is_dup {
                            #[cfg(feature = "rx-blocking-trace")]
                            pool.record_rx(|c| c.duplicate = c.duplicate.wrapping_add(1));
                            if no_ack {
                                // Duplicate NoAck — restart RX with new buffer
                                match alloc_dma_buffer(pool) {
                                    Some(new_idx) => {
                                        // rx_idx is IN_DMA, use release_tx for IN_DMA→FREE
                                        pool.release_tx(rx_idx);
                                        self.rx_idx = new_idx;
                                        let dma_ptr = unsafe { pool.dma_ptr(new_idx) };
                                        self.restart_rx_no_ack(dma_ptr);
                                    }
                                    None => {
                                        // Re-use current buffer
                                        let dma_ptr = unsafe { pool.dma_ptr(rx_idx) };
                                        self.restart_rx_no_ack(dma_ptr);
                                    }
                                }
                                return (PrxEvent::Duplicate, None);
                            } else {
                                // Duplicate with ACK — send repeated fallback ACK.
                                // The original rx_idx buffer is no longer used by DMA
                                // (PACKETPTR was changed by setup_ack_tx_fallback).
                                // Release it before entering TxRepeatedAck.
                                pool.release_tx(rx_idx);
                                self.rx_idx = NO_IDX;
                                self.start_repeated_ack(pipe as u8);
                                self.state = StatePrx::TxRepeatedAck;
                                return (PrxEvent::Duplicate, None);
                            }
                        }

                        // New packet — update detection, write metadata
                        self.radio.update_detection(pipe, pid, crc);
                        let header = unsafe { pool.header_mut(rx_idx) };
                        header.rssi = rssi;
                        header.pipe = pipe as u8;

                        if no_ack {
                            // NoAck: deliver to app, no ACK
                            crate::rx_metadata::publish_new_rx(
                                pool,
                                rx_idx,
                                self.rx_metadata_context,
                            );
                            self.rx_idx = NO_IDX;

                            match alloc_dma_buffer(pool) {
                                Some(new_idx) => {
                                    self.rx_idx = new_idx;
                                    let dma_ptr = unsafe { pool.dma_ptr(new_idx) };
                                    self.restart_rx_no_ack(dma_ptr);
                                }
                                None => {
                                    // No buffer available - stop radio and go idle
                                    #[cfg(feature = "rx-blocking-trace")]
                                    pool.record_rx(|c| {
                                        c.post_delivery_starved =
                                            c.post_delivery_starved.wrapping_add(1)
                                    });
                                    self.radio.stop();
                                    self.state = StatePrx::Idle;
                                }
                            }
                            return (PrxEvent::ReceivedNoAck, Some(rx_idx));
                        }

                        // Need ACK
                        self.start_ack_tx(pool, pipe as u8);
                        self.pending_rx_idx = rx_idx;
                        self.rx_idx = NO_IDX;
                        self.state = StatePrx::TxAck;
                        (PrxEvent::Received, Some(rx_idx))
                    }
                }
            }

            StatePrx::TxAck => {
                debug_assert!(evts.disabled, "TxAck: expected disabled event");
                self.radio.stop();

                if self.pending_rx_idx != NO_IDX {
                    pool.rx_complete(self.pending_rx_idx, None);
                    self.pending_rx_idx = NO_IDX;
                }
                self.release_ack_tx(pool);

                match alloc_dma_buffer(pool) {
                    Some(idx) => {
                        self.rx_idx = idx;
                        let dma_ptr = unsafe { pool.dma_ptr(idx) };
                        self.restart_rx_after_ack(dma_ptr);
                        self.state = StatePrx::Receiver;
                    }
                    None => {
                        // No buffer — stop radio and go idle
                        self.radio.stop();
                        self.state = StatePrx::Idle;
                        return (PrxEvent::None, None);
                    }
                }
                (PrxEvent::None, None)
            }

            StatePrx::TxRepeatedAck => {
                debug_assert!(evts.disabled, "TxRepeatedAck: expected disabled event");
                self.radio.stop();

                self.release_ack_tx(pool);

                match alloc_dma_buffer(pool) {
                    Some(idx) => {
                        self.rx_idx = idx;
                        let dma_ptr = unsafe { pool.dma_ptr(idx) };
                        self.restart_rx_after_ack(dma_ptr);
                        self.state = StatePrx::Receiver;
                    }
                    None => {
                        // No buffer — stop radio and go idle
                        self.radio.stop();
                        self.state = StatePrx::Idle;
                        return (PrxEvent::None, None);
                    }
                }
                (PrxEvent::None, None)
            }

            StatePrx::Idle => {
                debug_assert!(evts.user_event(), "Idle: expected user event");
                let _ = self.start_receiving(pool);
                (PrxEvent::None, None)
            }
        }
    }

    // ---- Mode-aware radio turnaround (S4) ----
    //
    // Each helper routes between the exclusive auto-shortcut path and the MPSL
    // manual-ACK path based on `timeslot_managed`. Under `not(feature = "mpsl")`
    // the cfg block is removed entirely, so exclusive builds compile to the exact
    // same radio calls as before (no behavioral change, no regression).

    /// Arm RX to start listening.
    fn arm_rx(&mut self, dma_ptr: *mut u8) {
        #[cfg(feature = "mpsl")]
        if self.timeslot_managed {
            self.radio
                .start_receiving_manual_ack(self.enabled_pipes, dma_ptr);
            return;
        }
        self.radio.start_receiving(self.enabled_pipes, dma_ptr);
    }

    /// Restart RX after a NoAck packet (stop any TX ramp, then re-arm RX).
    fn restart_rx_no_ack(&mut self, dma_ptr: *mut u8) {
        #[cfg(feature = "mpsl")]
        if self.timeslot_managed {
            self.radio
                .start_receiving_manual_ack(self.enabled_pipes, dma_ptr);
            return;
        }
        // Receiver processing already stopped RADIO before touching pool claims.
        self.radio.complete_rx_no_ack(dma_ptr);
    }

    /// Restart RX after an ACK TX completes.
    fn restart_rx_after_ack(&mut self, dma_ptr: *mut u8) {
        #[cfg(feature = "mpsl")]
        if self.timeslot_managed {
            self.radio
                .start_receiving_manual_ack(self.enabled_pipes, dma_ptr);
            return;
        }
        self.radio.start_receiving(self.enabled_pipes, dma_ptr);
    }

    /// Start an ACK TX for a new packet (dequeue ACK payload or empty fallback).
    fn start_ack_tx<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        pipe: u8,
    ) {
        #[cfg(feature = "mpsl")]
        if self.timeslot_managed {
            self.setup_ack_tx_manual(pool, pipe);
            return;
        }
        self.setup_ack_tx(pool, pipe);
    }

    /// Start a repeated (duplicate) ACK — empty fallback ACK.
    fn start_repeated_ack(&mut self, pipe: u8) {
        #[cfg(feature = "mpsl")]
        if self.timeslot_managed {
            self.transmit_timeslot_fallback_ack(pipe, false);
            return;
        }
        self.radio.setup_ack_tx_fallback(pipe);
    }

    /// Manual-turnaround variant of `setup_ack_tx` for MPSL timeslot mode.
    /// Dequeues a pipe-matched ACK payload from the pool (empty fallback if
    /// none) and starts the ACK TX explicitly (no `disabled_txen` shortcut).
    #[cfg(feature = "mpsl")]
    fn setup_ack_tx_manual<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        pipe: u8,
    ) {
        if let Some(idx) = pool.try_dequeue_tx_for_pipe(pipe) {
            let dma_ptr = unsafe { pool.dma_ptr(idx) };
            self.ack_tx_idx = idx;
            self.radio.transmit_ack_manual(pipe, dma_ptr);
        } else {
            self.ack_tx_idx = NO_IDX;
            self.transmit_timeslot_fallback_ack(pipe, true);
        }
    }

    #[cfg(feature = "mpsl")]
    fn transmit_timeslot_fallback_ack(&mut self, pipe: u8, advance_counter: bool) {
        let Some(context) = self.diag_ack_context else {
            self.radio.transmit_ack_manual_fallback(pipe);
            return;
        };

        let pipe_idx = pipe as usize;
        let counter = if pipe_idx < self.diag_ack_counter.len() {
            if advance_counter {
                self.diag_ack_counter[pipe_idx] = self.diag_ack_counter[pipe_idx].saturating_add(1);
            }
            self.diag_ack_counter[pipe_idx]
        } else {
            0
        };

        let ack = timeslot_diag_ack_buf();
        write_timeslot_diag_ack(ack, context, counter);
        let dma_ptr = unsafe { ack.as_mut_ptr().add(crate::header::EsbHeader::DMA_OFFSET) };
        self.radio.transmit_ack_manual(pipe, dma_ptr);
    }

    /// Set up ACK TX for a new (non-duplicate) packet.
    /// Saves the ACK TX buffer index for release after transmission.
    fn setup_ack_tx<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
        pipe: u8,
    ) {
        if let Some(idx) = pool.try_dequeue_tx_for_pipe(pipe) {
            let dma_ptr = unsafe { pool.dma_ptr(idx) };
            self.ack_tx_idx = idx;
            self.radio.setup_ack_tx(pipe, dma_ptr);
        } else {
            self.ack_tx_idx = NO_IDX;
            self.radio.setup_ack_tx_fallback(pipe);
        }
    }

    /// Release the current ACK TX buffer back to the pool.
    fn release_ack_tx<const N: usize, const SIZE: usize>(&mut self, pool: &PacketPool<N, SIZE>) {
        if self.ack_tx_idx != NO_IDX {
            pool.release_tx(self.ack_tx_idx);
            self.ack_tx_idx = NO_IDX;
        }
    }

    /// Handle TIMER ISR — defensively clear any stale timer events.
    /// PRX doesn't use timers in normal operation, so we don't pend
    /// the RADIO ISR (unlike PTX). A stale timer event should not
    /// trigger state machine processing.
    pub(crate) fn handle_timer_event(&self) {
        if self.timer.is_retransmit_fired() {
            self.timer.disarm_retransmit();
        }
        if self.timer.is_ack_timeout_fired() {
            self.timer.disarm_ack_timeout();
        }
    }

    /// Stop receiving and go idle.
    /// Ref: esb-ng `src/irq.rs` lines 398–406.
    pub(crate) fn stop_receiving<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) {
        self.radio.stop();
        self.timer.disarm_retransmit();
        self.timer.disarm_ack_timeout();
        let _ = self.check_events(false);

        if self.rx_idx != NO_IDX {
            // rx_idx is IN_DMA, use release_tx (issue 3).
            pool.release_tx(self.rx_idx);
            self.rx_idx = NO_IDX;
        }
        if self.pending_rx_idx != NO_IDX {
            // pending_rx_idx is IN_DMA when stopped from TxAck state.
            pool.release_tx(self.pending_rx_idx);
            self.pending_rx_idx = NO_IDX;
        }
        if self.ack_tx_idx != NO_IDX {
            pool.release_tx(self.ack_tx_idx);
            self.ack_tx_idx = NO_IDX;
        }
        self.state = StatePrx::Idle;
    }

    // ---- Suspend / Resume ----

    pub(crate) fn quiesce<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> StopOutcome {
        let stop = self.radio.stop();
        // stop_receiving's second stop is normal on the now-disabled hardware.
        self.stop_receiving(pool);
        self.timer.clear_all_events();
        self.radio.clear_all_events();
        stop
    }

    pub(crate) fn into_parts(self) -> (EsbRadio, EsbTimer<T>) {
        assert!(
            self.state == StatePrx::Idle
                && self.rx_idx == NO_IDX
                && self.pending_rx_idx == NO_IDX
                && self.ack_tx_idx == NO_IDX
        );
        assert!(self.radio.is_quiescent());
        (self.radio, self.timer)
    }

    pub(crate) fn from_parts(
        radio: EsbRadio,
        timer: EsbTimer<T>,
        config: &EsbConfig,
        enabled_pipes: u8,
    ) -> Self {
        Self::new(radio, timer, config, enabled_pipes)
    }

    /// Save ESB state and stop hardware. Called with RADIO IRQ already disabled.
    pub(crate) fn do_suspend<const N: usize, const SIZE: usize>(
        &mut self,
        pool: &PacketPool<N, SIZE>,
    ) -> EsbSavedState {
        self.stop_receiving(pool);
        self.timer.stop();

        EsbSavedState {
            pid: self.radio.save_pid_state(),
            last_crc: self.radio.save_crc_state(),
            last_valid: self.radio.save_detection_valid_state(),
            tx_pipe: 0,
            attempts: 0,
            protocol_state: SavedProtocolState::Idle,
        }
    }

    /// Full re-init and restore from saved state.
    /// Called with RADIO IRQ disabled; caller re-enables after return.
    pub(crate) fn do_restore(&mut self, state: &EsbSavedState, addresses: &EsbAddresses) {
        self.radio.power_cycle();
        self.radio.init(&self.config, addresses);
        self.radio.restore_pid_state(state.pid);
        self.radio.restore_crc_state(state.last_crc);
        self.radio.restore_detection_valid_state(state.last_valid);
        self.enabled_pipes = addresses.enabled_mask();
        self.state = StatePrx::Idle;
    }
}

// ---- Helpers ----

fn no_ack_tx_completed(state: StatePtx, events: IsrEvents, end: bool) -> bool {
    state == StatePtx::TxNoAck && end && events.disabled
}

/// Return CPU access only after the supplied quiescence operation completes.
fn stop_before_access<T, S>(owner: &mut T, stop: impl FnOnce(&mut T) -> S) -> &mut T {
    stop(owner);
    owner
}

/// Allocate a free buffer and transition it to DMA state.
fn alloc_dma_buffer<const N: usize, const SIZE: usize>(
    pool: &PacketPool<N, SIZE>,
) -> Option<usize> {
    (0..N).find(|&i| pool.rx_to_dma(i))
}

#[cfg(any(test, feature = "mpsl"))]
fn write_timeslot_diag_ack(buf: &mut [u8; 256], context: TimeslotDiagAckContext, counter: u32) {
    crate::mpsl_common::write_counter_schedule_packet(
        buf,
        0,
        counter,
        crate::mpsl_schedule::ScheduleHint::new(
            0,
            context.window_id,
            context.next_delay_us,
            context.period_us,
            context.window_us,
        ),
    );
}

#[cfg(feature = "mpsl")]
fn timeslot_diag_ack_buf() -> &'static mut [u8; 256] {
    use core::cell::UnsafeCell;

    #[repr(C, align(4))]
    struct TimeslotDiagAckBuf(UnsafeCell<[u8; 256]>);
    unsafe impl Sync for TimeslotDiagAckBuf {}

    #[unsafe(link_section = ".data")]
    static ACK: TimeslotDiagAckBuf = TimeslotDiagAckBuf(UnsafeCell::new([0u8; 256]));

    unsafe { &mut *ACK.0.get() }
}

/// Pend the RADIO ISR.
#[cfg(any(feature = "nrf52840", feature = "nrf52833", feature = "nrf52832"))]
fn pend_radio_isr() {
    cortex_m::peripheral::NVIC::pend(crate::pac::Interrupt::RADIO);
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::{TimeslotDiagAckContext, write_timeslot_diag_ack};
    use crate::header::EsbHeader;
    use crate::mpsl_common::read_counter_payload;
    use crate::mpsl_schedule::{
        SCHEDULE_COUNTER_HINT_PAYLOAD_LEN, ScheduleHint, decode_counter_payload_schedule_hint,
    };

    #[test]
    fn mocked_helper_returns_access_only_after_stop_callback() {
        let mut trace = std::vec::Vec::new();
        let stopped = super::stop_before_access(&mut trace, |trace| trace.push("stop callback"));
        stopped.push("returned access");
        assert_eq!(trace, ["stop callback", "returned access"]);
    }

    #[test]
    fn no_ack_completion_requires_end_and_disabled_independent_of_timer() {
        let mut cases = 0;
        for state in [
            super::StatePtx::Idle,
            super::StatePtx::TxNoAck,
            super::StatePtx::Tx,
            super::StatePtx::WaitAck,
            super::StatePtx::WaitRetransmit,
        ] {
            for disabled in [false, true] {
                for timer in [false, true] {
                    for end in [false, true] {
                        let events = super::IsrEvents { disabled, timer };
                        assert_eq!(
                            super::no_ack_tx_completed(state, events, end),
                            state == super::StatePtx::TxNoAck && end && disabled,
                            "state={state:?}, end={end}, disabled={disabled}, timer={timer}"
                        );
                        cases += 1;
                    }
                }
            }
        }
        assert_eq!(cases, 40);
    }

    #[test]
    fn no_ack_disabled_without_end_is_an_abort_not_completion() {
        for timer in [false, true] {
            let events = super::IsrEvents {
                disabled: true,
                timer,
            };
            assert!(!super::no_ack_tx_completed(
                super::StatePtx::TxNoAck,
                events,
                false
            ));
        }
    }

    #[test]
    fn no_ack_normal_end_and_disabled_complete() {
        let events = super::IsrEvents {
            disabled: true,
            timer: false,
        };
        assert!(super::no_ack_tx_completed(
            super::StatePtx::TxNoAck,
            events,
            true
        ));
    }

    #[test]
    fn timeslot_diag_ack_preserves_legacy_counter_and_schedule_hint() {
        let mut buf = [0u8; 256];
        let context = TimeslotDiagAckContext {
            window_id: 42,
            next_delay_us: 1500,
            period_us: 5000,
            window_us: 4500,
        };

        write_timeslot_diag_ack(&mut buf, context, 0x4433_2211);

        let header = unsafe { &*(buf.as_ptr().cast::<EsbHeader>()) };
        assert_eq!(header.length, SCHEDULE_COUNTER_HINT_PAYLOAD_LEN as u8);
        assert_eq!(header.pid(), 0);
        assert!(!header.no_ack());
        assert_eq!(read_counter_payload(&buf), Some(0x4433_2211));

        let payload = &buf[EsbHeader::PAYLOAD_OFFSET
            ..EsbHeader::PAYLOAD_OFFSET + SCHEDULE_COUNTER_HINT_PAYLOAD_LEN];
        assert_eq!(
            decode_counter_payload_schedule_hint(payload),
            Ok(ScheduleHint::new(0, 42, 1500, 5000, 4500))
        );
    }
}
