//! Policies shared by the live composition and host tests, not protocol algorithms.

use crate::bridge_rpc::Request;
use crate::timing_clock::{ClockError, ClockMapping, ClockProfile, PhysicalTime, TimingTime};
use nrf_embassy::platform::Effect;
use nrf_embassy::protocol_audio::IngressAdmissionGate;
use nrf_embassy::protocol_tx::{RadioSendResult, SubmissionDisposition};
use nrf_embassy::tdma::{Snapshot, WorkMetadata};

/// Experimental early service requires opt-in. Command events only enqueue work;
/// finish that work before servicing due TX, even when the experiment is enabled.
pub fn early_timing_allowed(enabled: bool, command_present: bool) -> bool {
    enabled && !command_present
}

/// A due alarm is serviced at the next phase boundary instead of waiting for the
/// rest of the pass; the same command-ordering rule as early service applies.
pub fn due_timing_allowed(command_present: bool, alarm_pending: bool) -> bool {
    alarm_pending && !command_present
}

/// Formatting one diagnostic line costs up to about a millisecond of thread-mode
/// CPU that the cooperative executor cannot preempt, so the actor's alarm service
/// waits behind it (capture 010: every slot entry >400 \u00b5s sat in Wait, ~1/s = the
/// publication rate). Start a line only when no alarm is pending and none is due
/// within the formatting budget.
pub const DIAGNOSTIC_FORMAT_BUDGET_US: i64 = 1500;
pub fn diagnostic_format_allowed(alarm_pending: bool, next_alarm_in_us: Option<i64>) -> bool {
    !alarm_pending && next_alarm_in_us.is_none_or(|us| us > DIAGNOSTIC_FORMAT_BUDGET_US)
}

pub fn ingress_boundary_gate(gate: IngressAdmissionGate) -> IngressAdmissionGate {
    IngressAdmissionGate {
        enabled: false,
        ..gate
    }
}

/// Reserve only in the firmware-provided fit value; admission budgets and the
/// portable C-oracle fitter remain unchanged. With 64 us, passive captures 010-012
/// still saw radio budget rejects whose shortfall was 1..=60 us (fit-to-submit
/// software cost exceeded the reserve); 128 us covers that with 2x margin.
pub const AUDIO_FIT_HEADROOM_US: u32 = if cfg!(feature = "admission-headroom") {
    128
} else {
    0
};

pub fn audio_fit_remaining_us(remaining: u32) -> u32 {
    remaining.saturating_sub(AUDIO_FIT_HEADROOM_US)
}

/// Audio fitting uses physical airtime, so measure the current slot against one
/// fresh physical sample and the same ceil-start/floor-END mapping as admission.
/// Invalid/unrepresentable windows fail closed; retain the core's u32 clamp.
pub fn slot_remaining_physical_us(
    snapshot: Snapshot,
    physical_now: i64,
    mapping: ClockMapping,
) -> u32 {
    if !snapshot.running || snapshot.slot_index < 0 || physical_now < 0 {
        return 0;
    }
    let slot = snapshot.slot;
    let default_mapping = ClockMapping::new(
        ClockProfile::Normalized,
        PhysicalTime::new(0),
        TimingTime::new(0),
    );
    if mapping == default_mapping {
        // Like evaluate_for_clock, preserve normalized numeric history even
        // when the slot starts before zero.
        return if physical_now < slot.start_us {
            0
        } else {
            slot.deadline_us
                .saturating_sub(physical_now)
                .clamp(0, i64::from(u32::MAX)) as u32
        };
    }
    let Ok(window) = mapping.convert(
        TimingTime::new(slot.start_us),
        TimingTime::new(slot.deadline_us),
    ) else {
        return 0;
    };
    let now = physical_now as u64;
    let deadline = window.deadline().value();
    // Admission rejects deadlines it cannot narrow to i64; never fit more optimistically.
    if now < window.start().value() || i64::try_from(deadline).is_err() {
        return 0;
    }
    deadline.saturating_sub(now).min(u64::from(u32::MAX)) as u32
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxWindow {
    Slot(WorkMetadata),
    Control(WorkMetadata),
}

impl TxWindow {
    /// `now` uses the supplied snapshot's timing units; capture preserves the
    /// source metadata for later physical deadline admission.
    pub fn capture(snapshot: Snapshot, callback: ActorCallback, now: i64) -> Option<Self> {
        Self::capture_result(snapshot, callback, now).ok()
    }

    pub fn capture_result(
        snapshot: Snapshot,
        callback: ActorCallback,
        now: i64,
    ) -> Result<Self, AdmissionRejection> {
        let (window, frame) = match callback {
            ActorCallback::Slot { slot, frame } if snapshot.slot_index == slot as i8 => {
                (Self::Slot(snapshot.slot), frame)
            }
            ActorCallback::Control { frame } => (Self::Control(snapshot.control), frame),
            _ => return Err(AdmissionRejection::CallbackSlotMismatch),
        };
        let metadata = window.metadata();
        if !snapshot.running {
            return Err(AdmissionRejection::Stopped);
        }
        if snapshot.generation != metadata.generation {
            return Err(AdmissionRejection::GenerationMismatch);
        }
        if snapshot.frame_counter != frame {
            return Err(AdmissionRejection::FrameMismatch);
        }
        if metadata.frame != frame {
            return Err(AdmissionRejection::WorkFrameMismatch);
        }
        if now < metadata.start_us {
            return Err(AdmissionRejection::BeforeTimingStart);
        }
        if now >= metadata.deadline_us {
            return Err(AdmissionRejection::TimingExpired);
        }
        Ok(window)
    }

    pub(crate) fn metadata(self) -> WorkMetadata {
        match self {
            Self::Slot(m) | Self::Control(m) => m,
        }
    }

    /// Original numeric admission: `now` and metadata are normalized physical us.
    /// Preserve its numeric behavior, including windows starting before zero.
    pub fn deadline(self, snapshot: Snapshot, now: i64, packet_len: usize) -> Option<u64> {
        self.evaluate(snapshot, now, packet_len).result.ok()
    }

    fn recapture(self, snapshot: Snapshot, now: i64) -> Result<(), AdmissionRejection> {
        let callback = match self {
            Self::Slot(m) => ActorCallback::Slot {
                slot: snapshot.slot_index as u8,
                frame: m.frame,
            },
            Self::Control(m) => ActorCallback::Control { frame: m.frame },
        };
        if Self::capture_result(snapshot, callback, now)? != self {
            return Err(AdmissionRejection::WindowMetadataChanged);
        }
        Ok(())
    }

    pub fn evaluate(self, snapshot: Snapshot, now: i64, packet_len: usize) -> AdmissionEvaluation {
        let mut evaluation = AdmissionEvaluation::new();
        evaluation.timing_now = Some(now);
        evaluation.result = (|| {
            self.recapture(snapshot, now)?;
            // Numeric history can have a negative start; no unsigned witness is invented.
            evaluation.physical_window_start = u64::try_from(self.metadata().start_us)
                .ok()
                .map(PhysicalTime::new);
            evaluation.physical_window_deadline = u64::try_from(self.metadata().deadline_us)
                .ok()
                .map(PhysicalTime::new);
            evaluate_budget(
                now,
                self.metadata().deadline_us,
                packet_len,
                &mut evaluation,
            )
        })();
        evaluation
    }

    /// Live admission preserves original numeric behavior only for the exact
    /// default Normalized mapping anchored at physical zero and timing zero.
    /// All other mappings retain checked conversion and before-anchor rejection.
    pub fn deadline_for_clock(
        self,
        snapshot: Snapshot,
        physical_now: i64,
        packet_len: usize,
        mapping: ClockMapping,
    ) -> Option<u64> {
        self.evaluate_for_clock(snapshot, physical_now, packet_len, mapping)
            .result
            .ok()
    }

    pub fn evaluate_for_clock(
        self,
        snapshot: Snapshot,
        physical_now: i64,
        packet_len: usize,
        mapping: ClockMapping,
    ) -> AdmissionEvaluation {
        const DEFAULT_MAPPING: ClockMapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(0),
        );
        if mapping == DEFAULT_MAPPING {
            self.evaluate(snapshot, physical_now, packet_len)
        } else {
            match u64::try_from(physical_now) {
                Ok(now) => self.evaluate_with_mapping(
                    snapshot,
                    PhysicalTime::new(now),
                    packet_len,
                    mapping,
                ),
                Err(_) => AdmissionEvaluation {
                    result: Err(AdmissionRejection::NegativePhysicalClock),
                    ..AdmissionEvaluation::new()
                },
            }
        }
    }

    /// Mapped admission. Metadata stays in timing units; the
    /// airtime budget and returned radio deadline use physical microseconds.
    /// Windows before the mapping anchor are rejected, not reinterpreted as
    /// belonging to the current mapping. This API manages no clock lifecycle.
    pub fn deadline_with_mapping(
        self,
        snapshot: Snapshot,
        physical_now: PhysicalTime,
        packet_len: usize,
        mapping: ClockMapping,
    ) -> Option<u64> {
        self.evaluate_with_mapping(snapshot, physical_now, packet_len, mapping)
            .result
            .ok()
    }

    pub fn evaluate_with_mapping(
        self,
        snapshot: Snapshot,
        physical_now: PhysicalTime,
        packet_len: usize,
        mapping: ClockMapping,
    ) -> AdmissionEvaluation {
        let mut evaluation = AdmissionEvaluation::new();
        evaluation.result = (|| {
            let timing_now = mapping
                .timing_at(physical_now)
                .map_err(|error| AdmissionRejection::Conversion {
                    stage: ConversionStage::TimingNow,
                    error,
                })?
                .value();
            evaluation.timing_now = Some(timing_now);
            self.recapture(snapshot, timing_now)?;
            let metadata = self.metadata();
            let physical_window = mapping
                .convert(
                    TimingTime::new(metadata.start_us),
                    TimingTime::new(metadata.deadline_us),
                )
                .map_err(|error| AdmissionRejection::Conversion {
                    stage: ConversionStage::Window,
                    error,
                })?;
            evaluation.physical_window_start = Some(physical_window.start());
            evaluation.physical_window_deadline = Some(physical_window.deadline());
            if physical_now.value() < physical_window.start().value() {
                return Err(AdmissionRejection::BeforePhysicalStart);
            }
            if physical_now.value() >= physical_window.deadline().value() {
                return Err(AdmissionRejection::PhysicalExpired);
            }
            let now = i64::try_from(physical_now.value())
                .map_err(|_| AdmissionRejection::PhysicalNowNarrowing)?;
            let until = i64::try_from(physical_window.deadline().value())
                .map_err(|_| AdmissionRejection::PhysicalDeadlineNarrowing)?;
            evaluate_budget(now, until, packet_len, &mut evaluation)
        })();
        evaluation
    }
}

/// Source 2 Mbps budget includes ramp and the 100 us margin (also covers the
/// guard's 20 us latest-CPU-launch lead). Not physical timing qualification.
pub fn bounded_air_deadline(now: i64, until: i64, packet_len: usize) -> Option<u64> {
    evaluate_budget(now, until, packet_len, &mut AdmissionEvaluation::new()).ok()
}

/// Only the pre-driver airtime check can establish that no send was submitted.
pub fn bounded_air_deadline_classified(
    now: i64,
    until: i64,
    packet_len: usize,
) -> Result<u64, RadioSendResult> {
    bounded_air_deadline(now, until, packet_len).ok_or(RadioSendResult {
        code: -11,
        disposition: SubmissionDisposition::NotSubmitted,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConversionStage {
    TimingNow,
    Window,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdmissionRejection {
    CallbackSlotMismatch,
    Stopped,
    GenerationMismatch,
    FrameMismatch,
    WorkFrameMismatch,
    BeforeTimingStart,
    TimingExpired,
    WindowMetadataChanged,
    NegativePhysicalClock,
    Conversion {
        stage: ConversionStage,
        error: ClockError,
    },
    BeforePhysicalStart,
    PhysicalExpired,
    PhysicalNowNarrowing,
    PhysicalDeadlineNarrowing,
    PacketLengthConversion,
    BudgetArithmetic,
    NegativeBudgetClock,
    BudgetEndOverflow,
    BudgetTooShort,
    DeadlineCapOverflow,
    FinalDeadlineConversion,
}

impl AdmissionRejection {
    /// Stable bounded counter indices; conversion reserves seven errors per stage.
    pub const COUNT: usize = 34;

    pub const fn index(self) -> usize {
        match self {
            Self::CallbackSlotMismatch => 0,
            Self::Stopped => 1,
            Self::GenerationMismatch => 2,
            Self::FrameMismatch => 3,
            Self::WorkFrameMismatch => 4,
            Self::BeforeTimingStart => 5,
            Self::TimingExpired => 6,
            Self::WindowMetadataChanged => 7,
            Self::NegativePhysicalClock => 8,
            Self::Conversion { stage, error } => {
                let stage = match stage {
                    ConversionStage::TimingNow => 0,
                    ConversionStage::Window => 1,
                };
                let error = match error {
                    ClockError::PhysicalBeforeAnchor => 0,
                    ClockError::TimingBeforeAnchor => 1,
                    ClockError::TimingOverflow => 2,
                    ClockError::PhysicalOverflow => 3,
                    ClockError::CheckedArithmeticOverflow => 4,
                    ClockError::InvalidWindow => 5,
                    ClockError::RoundedEmptyWindow => 6,
                };
                9 + stage * 7 + error
            }
            Self::BeforePhysicalStart => 23,
            Self::PhysicalExpired => 24,
            Self::PhysicalNowNarrowing => 25,
            Self::PhysicalDeadlineNarrowing => 26,
            Self::PacketLengthConversion => 27,
            Self::BudgetArithmetic => 28,
            Self::NegativeBudgetClock => 29,
            Self::BudgetEndOverflow => 30,
            Self::BudgetTooShort => 31,
            Self::DeadlineCapOverflow => 32,
            Self::FinalDeadlineConversion => 33,
        }
    }
}

impl core::fmt::Display for AdmissionRejection {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmissionEvaluation {
    pub result: Result<u64, AdmissionRejection>,
    pub timing_now: Option<i64>,
    pub physical_window_start: Option<PhysicalTime>,
    pub physical_window_deadline: Option<PhysicalTime>,
    pub budget_us: Option<i64>,
}

impl AdmissionEvaluation {
    fn new() -> Self {
        Self {
            result: Err(AdmissionRejection::Stopped),
            timing_now: None,
            physical_window_start: None,
            physical_window_deadline: None,
            budget_us: None,
        }
    }
}

fn evaluate_budget(
    now: i64,
    until: i64,
    packet_len: usize,
    evaluation: &mut AdmissionEvaluation,
) -> Result<u64, AdmissionRejection> {
    use AdmissionRejection::*;
    let len = i64::try_from(packet_len).map_err(|_| PacketLengthConversion)?;
    let budget = 129i64
        .checked_add(
            4i64.checked_mul(len.checked_add(11).ok_or(BudgetArithmetic)?)
                .ok_or(BudgetArithmetic)?,
        )
        .ok_or(BudgetArithmetic)?
        .checked_add(100)
        .ok_or(BudgetArithmetic)?;
    evaluation.budget_us = Some(budget);
    if now < 0 {
        return Err(NegativeBudgetClock);
    }
    if now.checked_add(budget).ok_or(BudgetEndOverflow)? > until {
        return Err(BudgetTooShort);
    }
    u64::try_from(until.min(now.checked_add(1450).ok_or(DeadlineCapOverflow)?))
        .map_err(|_| FinalDeadlineConversion)
}

pub fn runtime_output(effect: Effect) -> Option<Result<Request, i32>> {
    Some(match effect {
        Effect::Status(status) => Ok(Request::Status(status)),
        Effect::Ack { command, result } => Ok(Request::Ack {
            command: command.kind as u8,
            generation: command.generation,
            result,
        }),
        Effect::DiscardBridgeAudio => Ok(Request::DiscardAudio),
        Effect::BridgeDelivery(enabled) => Ok(Request::Delivery(enabled)),
        Effect::MeshStopped => Request::event(4, &[]),
        Effect::SyncLost => Request::event(5, &[]),
        _ => return None,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActorCallback {
    Slot { slot: u8, frame: u32 },
    Control { frame: u32 },
}

#[derive(Default)]
pub struct CallbackBuffer {
    entries: [Option<ActorCallback>; 3],
}

impl CallbackBuffer {
    pub fn push(&mut self, callback: ActorCallback) {
        let entry = self
            .entries
            .iter_mut()
            .find(|entry| entry.is_none())
            .expect("checked TDMA callback buffer overflow");
        *entry = Some(callback);
    }
    pub fn take(&mut self) -> [Option<ActorCallback>; 3] {
        core::mem::take(&mut self.entries)
    }
}

pub fn status_tx(
    input: &[u8; nrf_embassy::bridge::TRANSFER_SIZE],
    capable: bool,
) -> [u8; nrf_embassy::bridge::TRANSFER_SIZE] {
    if capable {
        *input
    } else {
        crate::unsupported_status_tx(input)
    }
}

#[derive(Default)]
pub struct Capability {
    strict_initialized: bool,
    external_ingress: bool,
    gate_published: bool,
}

impl Capability {
    pub fn snapshot(&self) -> (bool, bool, bool) {
        (
            self.strict_initialized,
            self.external_ingress,
            self.gate_published,
        )
    }
    pub fn strict_initialized(&mut self) {
        self.strict_initialized = true;
    }
    pub fn external_ingress(&mut self) {
        self.external_ingress = true;
    }
    pub fn gate_published(&mut self) {
        self.gate_published = true;
    }
    pub fn ready(&self) -> bool {
        self.strict_initialized && self.external_ingress && self.gate_published
    }
}

#[derive(Default)]
pub struct RfLifecycle {
    floor: u64,
    channel: u8,
    receiving: bool,
}

impl RfLifecycle {
    pub fn snapshot(&self) -> (bool, u64, u8) {
        (self.receiving, self.floor, self.channel)
    }
    /// Explicit stop/config invalidates all sessions observed so far. TX role
    /// restoration never calls this: queued pre-TX packets remain admissible.
    pub fn stop_or_configure(&mut self, last_epoch: u64, channel: u8) {
        self.floor = last_epoch.checked_add(1).expect("RF epoch exhausted");
        self.channel = channel;
        self.receiving = false;
    }
    pub fn started(&mut self, epoch: u64) {
        self.floor = self.floor.max(epoch);
        self.receiving = true;
    }
    pub fn accepts(&self, len: usize, channel: u8, epoch: u64, stamp: Option<i64>) -> bool {
        self.receiving
            && len <= 209
            && channel == self.channel
            && epoch >= self.floor
            && stamp.is_some()
    }
}

/// Counts terminal observations only; reservation is not physical completion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TxCounters {
    pub attempted: u32,
    pub physical_complete: u32,
    pub deadline: u32,
    pub fault: u32,
}

#[derive(Clone, Copy)]
pub enum TxTerminal {
    PhysicalComplete,
    Deadline,
    Pending,
    Fault,
}

impl TxCounters {
    /// Called only after awaiting an accepted ticket, including fault reports.
    pub fn record_send_result(&mut self, terminal: TxTerminal, code: i32) -> RadioSendResult {
        self.record(terminal);
        RadioSendResult {
            code,
            disposition: SubmissionDisposition::Accepted,
        }
    }

    pub fn record(&mut self, terminal: TxTerminal) {
        self.attempted = self.attempted.wrapping_add(1);
        let counter = match terminal {
            TxTerminal::PhysicalComplete => &mut self.physical_complete,
            TxTerminal::Deadline => &mut self.deadline,
            TxTerminal::Pending | TxTerminal::Fault => &mut self.fault,
        };
        *counter = counter.wrapping_add(1);
    }
}

#[cfg(test)]
#[path = "fixtures/tx_window_pre_diagnostic.rs"]
mod pre_diagnostic;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classified_preflight_and_accepted_fault_keep_same_code_distinct() {
        let rejected = bounded_air_deadline_classified(1_000, 1_000, 29).unwrap_err();
        assert_eq!(rejected.code, -11);
        assert_eq!(rejected.disposition, SubmissionDisposition::NotSubmitted);
        assert_eq!(
            bounded_air_deadline_classified(1_000, 17_500, 29),
            Ok(2_450)
        );

        // Pure terminal conversion used after wait; this supplies the existing
        // ReceiverUnavailable errno and does not simulate a hardware report.
        let mut counters = TxCounters::default();
        let fault = counters.record_send_result(TxTerminal::Fault, -11);
        assert_eq!(fault.code, rejected.code);
        assert_eq!(fault.disposition, SubmissionDisposition::Accepted);
        assert_eq!(counters.attempted, 1);
        assert_eq!(counters.fault, 1);
        assert_eq!(counters.physical_complete, 0);
    }

    #[test]
    fn classified_terminal_conversion_preserves_codes_and_counters() {
        let mut counters = TxCounters::default();
        for (terminal, code) in [
            (TxTerminal::PhysicalComplete, 0),
            (TxTerminal::Pending, -5),
            (TxTerminal::Deadline, -110),
            (TxTerminal::Fault, -125),
        ] {
            assert_eq!(
                counters.record_send_result(terminal, code),
                RadioSendResult {
                    code,
                    disposition: SubmissionDisposition::Accepted,
                }
            );
        }
        assert_eq!(
            counters,
            TxCounters {
                attempted: 4,
                physical_complete: 1,
                deadline: 1,
                fault: 2,
            }
        );
    }

    fn audio_slot(start: i64, deadline: i64) -> Snapshot {
        let mut snapshot = nrf_embassy::tdma::Tdma::new().snapshot();
        snapshot.running = true;
        snapshot.slot_index = 0;
        snapshot.slot.start_us = start;
        snapshot.slot.deadline_us = deadline;
        snapshot
    }

    #[test]
    fn fit_headroom_edges_and_later_admission() {
        let candidates = [161, 113, 65];
        let fit = |remaining| {
            openhelmet_mesh::core::fit_airtime(remaining, 100, 129, 4, 11, Some(&candidates))
        };
        let margin = AUDIO_FIT_HEADROOM_US;
        assert_eq!(
            margin,
            if cfg!(feature = "admission-headroom") {
                128
            } else {
                0
            }
        );
        for (budget, index) in [(533, 2), (725, 1), (917, 0)] {
            assert_eq!(fit(budget), Some(index));
            assert_eq!(
                fit(budget - 1),
                if index == 2 { None } else { Some(index + 1) }
            );
            assert_eq!(fit(audio_fit_remaining_us(budget + margin)), Some(index));
            assert_eq!(
                fit(audio_fit_remaining_us(budget + margin - 1)),
                if index == 2 { None } else { Some(index + 1) }
            );
            let selected = fit(audio_fit_remaining_us(budget));
            assert_eq!(
                selected,
                if margin == 0 {
                    Some(index)
                } else if index == 2 {
                    None
                } else {
                    Some(index + 1)
                }
            );
            if let Some(selected) = selected {
                let snapshot = audio_slot(0, i64::from(budget) + 100);
                let window = TxWindow::Slot(snapshot.slot);
                for elapsed in 0..=margin {
                    assert!(
                        window
                            .evaluate(
                                snapshot,
                                100 + i64::from(elapsed),
                                candidates[selected] as usize
                            )
                            .result
                            .is_ok()
                    );
                }
            }
        }
        assert_eq!(audio_fit_remaining_us(0), 0);
        assert_eq!(audio_fit_remaining_us(margin.saturating_sub(1)), 0);
    }

    #[test]
    fn audio_remaining_normalized_matches_old_numeric_value() {
        let mapping = clock_mapping(ClockProfile::Normalized);
        for start in [-100, 0, 100] {
            let snapshot = audio_slot(start, 1500);
            for now in [0, 99, 100, 673, 1499, 1500, 1501, i64::MAX] {
                let old = if now < start {
                    0
                } else {
                    (1500_i64 - now).clamp(0, i64::from(u32::MAX)) as u32
                };
                assert_eq!(slot_remaining_physical_us(snapshot, now, mapping), old);
            }
        }
    }

    #[test]
    fn audio_remaining_slot_boundaries_and_stopped_states() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = clock_mapping(profile);
            let snapshot = audio_slot(100, 1500);
            let window = mapping
                .convert(TimingTime::new(100), TimingTime::new(1500))
                .unwrap();
            let start = window.start().value() as i64;
            let end = window.deadline().value() as i64;
            assert_eq!(slot_remaining_physical_us(snapshot, start - 1, mapping), 0);
            assert_eq!(
                slot_remaining_physical_us(snapshot, start, mapping),
                (end - start) as u32
            );
            assert_eq!(slot_remaining_physical_us(snapshot, end - 1, mapping), 1);
            assert_eq!(slot_remaining_physical_us(snapshot, end, mapping), 0);
            assert_eq!(slot_remaining_physical_us(snapshot, end + 1, mapping), 0);
            assert_eq!(
                slot_remaining_physical_us(
                    Snapshot {
                        running: false,
                        ..snapshot
                    },
                    start,
                    mapping
                ),
                0
            );
            assert_eq!(
                slot_remaining_physical_us(
                    Snapshot {
                        slot_index: -1,
                        ..snapshot
                    },
                    start,
                    mapping
                ),
                0
            );
        }
        let legacy = clock_mapping(ClockProfile::LegacyNrfRtc10000);
        // 1500 timing units end at 1373.291... us. The floored end has no
        // remaining airtime even though timing_at(1373) is still only 1499.
        assert_eq!(
            legacy
                .physical_not_after(TimingTime::new(1500))
                .unwrap()
                .value(),
            1373
        );
        assert_eq!(
            legacy.timing_at(PhysicalTime::new(1373)).unwrap().value(),
            1499
        );
        assert_eq!(
            slot_remaining_physical_us(audio_slot(0, 1500), 1373, legacy),
            0
        );
    }

    #[test]
    fn audio_remaining_exact_budget_matches_admission_for_both_profiles() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = clock_mapping(profile);
            let snapshot = audio_slot(0, 1500);
            let end = mapping
                .physical_not_after(TimingTime::new(1500))
                .unwrap()
                .value() as i64;
            let now = end - 725;
            assert_eq!(slot_remaining_physical_us(snapshot, now, mapping), 725);
            let window = TxWindow::Slot(snapshot.slot);
            let admitted = window.evaluate_for_clock(snapshot, now, 113, mapping);
            assert_eq!(admitted.budget_us, Some(725));
            assert!(admitted.result.is_ok());
            assert_eq!(slot_remaining_physical_us(snapshot, now + 1, mapping), 724);
            assert_eq!(
                window
                    .evaluate_for_clock(snapshot, now + 1, 113, mapping)
                    .result,
                Err(AdmissionRejection::BudgetTooShort)
            );
        }
    }

    #[test]
    fn audio_remaining_invalid_negative_and_overflow_fail_closed() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = clock_mapping(profile);
            for (start, end) in [(0, 0), (100, 99), (-2, -1)] {
                assert_eq!(
                    slot_remaining_physical_us(audio_slot(start, end), 0, mapping),
                    0
                );
            }
            assert_eq!(
                slot_remaining_physical_us(audio_slot(0, 1500), -1, mapping),
                0
            );
            let overflow =
                ClockMapping::new(profile, PhysicalTime::new(u64::MAX - 1), TimingTime::new(0));
            assert_eq!(
                slot_remaining_physical_us(audio_slot(0, i64::MAX), 0, overflow),
                0
            );
            let anchored = ClockMapping::new(profile, PhysicalTime::new(100), TimingTime::new(-50));
            assert_eq!(
                slot_remaining_physical_us(audio_slot(-51, 1500), 100, anchored),
                0
            );
            assert_eq!(
                slot_remaining_physical_us(audio_slot(-50, 1500), 99, anchored),
                0
            );
        }
        let legacy = clock_mapping(ClockProfile::LegacyNrfRtc10000);
        assert_eq!(slot_remaining_physical_us(audio_slot(0, 1), 0, legacy), 0);
    }

    #[test]
    fn audio_remaining_signed_anchors_and_large_spans_are_checked_and_clamped() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = ClockMapping::new(profile, PhysicalTime::new(100), TimingTime::new(-50));
            let snapshot = audio_slot(-50, 1500);
            let end = mapping
                .physical_not_after(TimingTime::new(1500))
                .unwrap()
                .value();
            assert_eq!(
                slot_remaining_physical_us(snapshot, 100, mapping),
                (end - 100) as u32
            );
            let narrowable = |m: ClockMapping| {
                m.physical_not_after(TimingTime::new(i64::MAX))
                    .is_ok_and(|end| i64::try_from(end.value()).is_ok())
            };
            let expected = |m| if narrowable(m) { u32::MAX } else { 0 };
            assert_eq!(
                slot_remaining_physical_us(audio_slot(-50, i64::MAX), 100, mapping),
                expected(mapping)
            );
            let wide = ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(i64::MIN));
            assert_eq!(
                slot_remaining_physical_us(audio_slot(i64::MIN, i64::MAX), 0, wide),
                expected(wide)
            );
        }
        assert_eq!(
            slot_remaining_physical_us(
                audio_slot(0, i64::MAX),
                0,
                clock_mapping(ClockProfile::Normalized)
            ),
            u32::MAX
        );
    }

    #[test]
    fn audio_remaining_legacy_core_fit_strips_prev1_and_passes_physical_admission() {
        let mapping = clock_mapping(ClockProfile::LegacyNrfRtc10000);
        let snapshot = audio_slot(0, 1500);
        let now = 673;
        let remaining = slot_remaining_physical_us(snapshot, now, mapping);
        assert_eq!(remaining, 700);
        // These are fit_candidates' lengths for a 113-byte bundle with prev1.
        let candidates = [113, 65];
        let old_remaining = (snapshot.slot.deadline_us
            - mapping
                .timing_at(PhysicalTime::new(now as u64))
                .unwrap()
                .value()) as u32;
        assert_eq!(
            openhelmet_mesh::core::fit_airtime(old_remaining, 100, 129, 4, 11, Some(&candidates)),
            Some(0)
        );
        let selected =
            openhelmet_mesh::core::fit_airtime(remaining, 100, 129, 4, 11, Some(&candidates))
                .unwrap();
        assert_eq!(selected, 1);
        assert_eq!(candidates[selected], 65);
        let window = TxWindow::Slot(snapshot.slot);
        assert_eq!(
            window
                .evaluate_for_clock(snapshot, now, 113, mapping)
                .result,
            Err(AdmissionRejection::BudgetTooShort)
        );
        let admitted =
            window.evaluate_for_clock(snapshot, now, candidates[selected] as usize, mapping);
        assert_eq!(admitted.budget_us, Some(533));
        assert!(admitted.result.is_ok());
    }

    fn parity_case(
        saved: TxWindow,
        current: Snapshot,
        now: i64,
        physical: u64,
        len: usize,
        mapping: ClockMapping,
    ) {
        let old = match saved {
            TxWindow::Slot(m) => pre_diagnostic::TxWindow::Slot(m),
            TxWindow::Control(m) => pre_diagnostic::TxWindow::Control(m),
        };
        assert_eq!(
            saved.deadline(current, now, len),
            old.deadline(current, now, len)
        );
        assert_eq!(
            saved.deadline_for_clock(current, now, len, mapping),
            old.deadline_for_clock(current, now, len, mapping),
        );
        assert_eq!(
            saved.deadline_with_mapping(current, PhysicalTime::new(physical), len, mapping),
            old.deadline_with_mapping(current, PhysicalTime::new(physical), len, mapping),
        );
        for callback in [
            ActorCallback::Slot {
                slot: current.slot_index as u8,
                frame: saved.metadata().frame,
            },
            ActorCallback::Slot {
                slot: (current.slot_index as u8).wrapping_add(1),
                frame: saved.metadata().frame,
            },
            ActorCallback::Control {
                frame: saved.metadata().frame,
            },
        ] {
            let new = TxWindow::capture(current, callback, now).map(|w| match w {
                TxWindow::Slot(m) => pre_diagnostic::TxWindow::Slot(m),
                TxWindow::Control(m) => pre_diagnostic::TxWindow::Control(m),
            });
            assert_eq!(
                new,
                pre_diagnostic::TxWindow::capture(current, callback, now)
            );
        }
    }

    #[test]
    fn frozen_baseline_parity_varied_windows_mappings_and_limits() {
        let base = control_snapshot();
        let mut state = 0x9cb666_u64;
        for case in 0..384 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            let start = match case % 12 {
                0 => i64::MIN,
                1 => i64::MAX - 2000,
                2 => -1000,
                _ => (state % 100_000) as i64 - 20_000,
            };
            let duration = [0, 1, 2, 273, 389, 1073, 1450, 1500, 2000, 8000][case % 10];
            let end = start.saturating_add(duration);
            let metadata = WorkMetadata {
                start_us: start,
                deadline_us: end,
                ..base.control
            };
            let mut current = Snapshot {
                slot: metadata,
                control: metadata,
                slot_index: [-128, -1, 0, 7, 127][case % 5],
                ..base
            };
            let saved = if case % 2 == 0 {
                TxWindow::Slot(metadata)
            } else {
                TxWindow::Control(metadata)
            };
            match case % 9 {
                0 => current.running = false,
                1 => current.generation = current.generation.wrapping_add(1),
                2 => current.frame_counter = current.frame_counter.wrapping_add(1),
                3 => {
                    current.slot.frame = metadata.frame.wrapping_add(1);
                    current.control.frame = current.slot.frame;
                }
                4 => {
                    current.slot.generation = metadata.generation.wrapping_add(1);
                    current.control.generation = current.slot.generation;
                }
                5 => {
                    current.slot.start_us = start.saturating_add(1);
                    current.control.start_us = current.slot.start_us;
                }
                6 => {
                    current.slot.deadline_us = end.saturating_sub(1);
                    current.control.deadline_us = current.slot.deadline_us;
                }
                7 => current.synchronized = false,
                _ => {}
            }
            let len = [
                0,
                29,
                200,
                209,
                307,
                usize::MAX,
                i64::MAX as usize,
                (i64::MAX / 4) as usize,
            ][case % 8];
            for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
                for (physical_anchor, timing_anchor) in [
                    (0, 0),
                    (1000, -2000),
                    (1000, 1000),
                    (i64::MAX as u64 - 1000, start),
                    (i64::MAX as u64 + 1, start),
                    (u64::MAX - 1000, start),
                    (0, i64::MIN),
                    (0, i64::MAX),
                ] {
                    let mapping = ClockMapping::new(
                        profile,
                        PhysicalTime::new(physical_anchor),
                        TimingTime::new(timing_anchor),
                    );
                    for now in [
                        -1,
                        0,
                        start.saturating_sub(1),
                        start,
                        start.saturating_add(1),
                        end.saturating_sub(389),
                        end.saturating_sub(388),
                        end.saturating_sub(1),
                        end,
                        i64::MAX,
                    ] {
                        parity_case(saved, current, now, now as u64, len, mapping);
                    }
                    for physical in [
                        physical_anchor.saturating_sub(1),
                        physical_anchor,
                        physical_anchor.saturating_add(1),
                        u64::MAX,
                    ] {
                        parity_case(saved, current, physical as i64, physical, len, mapping);
                    }
                    if let Ok(bounds) =
                        mapping.convert(TimingTime::new(start), TimingTime::new(end))
                    {
                        for physical in [
                            bounds.start().value().saturating_sub(1),
                            bounds.start().value(),
                            bounds.deadline().value().saturating_sub(389),
                            bounds.deadline().value().saturating_sub(388),
                            bounds.deadline().value().saturating_sub(1),
                            bounds.deadline().value(),
                        ] {
                            parity_case(saved, current, physical as i64, physical, len, mapping);
                        }
                    }
                }
            }
        }
        for now in [-1, 0, 1, i64::MAX - 1450, i64::MAX - 273, i64::MAX] {
            for until in [-1, 0, 273, 1450, i64::MAX] {
                for len in [0, 29, 209, usize::MAX, (i64::MAX / 4) as usize] {
                    assert_eq!(
                        bounded_air_deadline(now, until, len),
                        pre_diagnostic::bounded_air_deadline(now, until, len),
                    );
                }
            }
        }
    }

    #[test]
    fn evaluation_reports_first_failure_and_only_reached_observations() {
        use AdmissionRejection::*;
        let base = snapshot_at_frame_start(-17_000);
        let window = TxWindow::Control(base.control);
        let default = clock_mapping(ClockProfile::Normalized);
        let stopped = Snapshot {
            running: false,
            generation: base.generation.wrapping_add(1),
            ..base
        };
        let evaluation = window.evaluate_for_clock(stopped, -1, usize::MAX, default);
        assert_eq!(evaluation.result, Err(Stopped));
        assert_eq!(evaluation.timing_now, Some(-1));
        assert_eq!(evaluation.physical_window_deadline, None);
        assert_eq!(evaluation.budget_us, None);
        let anchored = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(1),
            TimingTime::new(0),
        );
        let evaluation = window.evaluate_for_clock(stopped, -1, 0, anchored);
        assert_eq!(evaluation.result, Err(NegativePhysicalClock));
        assert_eq!(evaluation.timing_now, None);
        assert_eq!(
            window.evaluate(base, -1, 0).result,
            Err(NegativeBudgetClock)
        );
        let accepted = window.evaluate_for_clock(base, 0, 29, default);
        assert_eq!(accepted.result, Ok(500));
        assert_eq!(accepted.physical_window_start, None);
        assert_eq!(
            accepted.physical_window_deadline,
            Some(PhysicalTime::new(500))
        );
        assert_eq!(accepted.budget_us, Some(389));
        let mapped = window.evaluate_with_mapping(base, PhysicalTime::new(0), 29, default);
        assert_eq!(
            mapped.result,
            Err(Conversion {
                stage: ConversionStage::Window,
                error: ClockError::TimingBeforeAnchor,
            })
        );
        assert_eq!(mapped.timing_now, Some(0));
        assert_eq!(mapped.physical_window_start, None);
        assert_eq!(mapped.budget_us, None);
        let changed = Snapshot {
            control: WorkMetadata {
                start_us: 1,
                ..base.control
            },
            ..base
        };
        assert_eq!(
            window.evaluate(changed, 0, 0).result,
            Err(BeforeTimingStart)
        );
        assert_eq!(
            window.evaluate(changed, 2, 0).result,
            Err(WindowMetadataChanged)
        );
    }

    #[test]
    fn conversion_failure_stage_and_reachability_are_preserved() {
        use AdmissionRejection::*;
        let base = control_snapshot();
        let mapping = clock_mapping(ClockProfile::LegacyNrfRtc10000);
        let current = Snapshot {
            control: WorkMetadata {
                start_us: 1,
                deadline_us: 2,
                ..base.control
            },
            ..base
        };
        let evaluation = TxWindow::Control(current.control).evaluate_with_mapping(
            current,
            PhysicalTime::new(1),
            0,
            mapping,
        );
        assert_eq!(
            evaluation.result,
            Err(Conversion {
                stage: ConversionStage::Window,
                error: ClockError::RoundedEmptyWindow,
            })
        );
        assert_eq!(evaluation.timing_now, Some(1));
        assert_eq!(evaluation.physical_window_deadline, None);
        // InvalidWindow is a conversion error, but invalid metadata fails recapture first.
        let invalid = Snapshot {
            control: WorkMetadata {
                deadline_us: 1,
                ..current.control
            },
            ..current
        };
        assert_eq!(
            mapping.convert(TimingTime::new(1), TimingTime::new(1)),
            Err(ClockError::InvalidWindow)
        );
        assert_eq!(
            TxWindow::Control(invalid.control)
                .evaluate_with_mapping(invalid, PhysicalTime::new(1), 0, mapping)
                .result,
            Err(TimingExpired)
        );
        let overflow = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(i64::MAX),
        );
        let evaluation = TxWindow::Control(base.control).evaluate_with_mapping(
            base,
            PhysicalTime::new(1),
            0,
            overflow,
        );
        assert_eq!(
            evaluation.result,
            Err(Conversion {
                stage: ConversionStage::TimingNow,
                error: ClockError::TimingOverflow,
            })
        );
        assert_eq!(evaluation.timing_now, None);
        for stage in [ConversionStage::TimingNow, ConversionStage::Window] {
            for error in [
                ClockError::PhysicalBeforeAnchor,
                ClockError::TimingBeforeAnchor,
                ClockError::TimingOverflow,
                ClockError::PhysicalOverflow,
                ClockError::CheckedArithmeticOverflow,
                ClockError::InvalidWindow,
                ClockError::RoundedEmptyWindow,
            ] {
                assert!(Conversion { stage, error }.index() < AdmissionRejection::COUNT);
            }
        }
    }

    #[test]
    fn rejection_indices_and_display_strings_are_stable_and_complete() {
        extern crate std;

        use AdmissionRejection::*;
        assert_eq!(AdmissionRejection::COUNT, 34);
        let mut seen = [false; AdmissionRejection::COUNT];
        let mut check = |reason: AdmissionRejection, index: usize, display: &str| {
            assert_eq!(reason.index(), index);
            assert!(!seen[index]);
            seen[index] = true;
            assert_eq!(std::format!("{reason}"), display);
        };
        for (reason, index, display) in [
            (CallbackSlotMismatch, 0, "CallbackSlotMismatch"),
            (Stopped, 1, "Stopped"),
            (GenerationMismatch, 2, "GenerationMismatch"),
            (FrameMismatch, 3, "FrameMismatch"),
            (WorkFrameMismatch, 4, "WorkFrameMismatch"),
            (BeforeTimingStart, 5, "BeforeTimingStart"),
            (TimingExpired, 6, "TimingExpired"),
            (WindowMetadataChanged, 7, "WindowMetadataChanged"),
            (NegativePhysicalClock, 8, "NegativePhysicalClock"),
            (BeforePhysicalStart, 23, "BeforePhysicalStart"),
            (PhysicalExpired, 24, "PhysicalExpired"),
            (PhysicalNowNarrowing, 25, "PhysicalNowNarrowing"),
            (PhysicalDeadlineNarrowing, 26, "PhysicalDeadlineNarrowing"),
            (PacketLengthConversion, 27, "PacketLengthConversion"),
            (BudgetArithmetic, 28, "BudgetArithmetic"),
            (NegativeBudgetClock, 29, "NegativeBudgetClock"),
            (BudgetEndOverflow, 30, "BudgetEndOverflow"),
            (BudgetTooShort, 31, "BudgetTooShort"),
            (DeadlineCapOverflow, 32, "DeadlineCapOverflow"),
            (FinalDeadlineConversion, 33, "FinalDeadlineConversion"),
        ] {
            check(reason, index, display);
        }
        for (stage, first_index) in [
            (ConversionStage::TimingNow, 9),
            (ConversionStage::Window, 16),
        ] {
            for (offset, (error, timing_display, window_display)) in [
                (
                    ClockError::PhysicalBeforeAnchor,
                    "Conversion { stage: TimingNow, error: PhysicalBeforeAnchor }",
                    "Conversion { stage: Window, error: PhysicalBeforeAnchor }",
                ),
                (
                    ClockError::TimingBeforeAnchor,
                    "Conversion { stage: TimingNow, error: TimingBeforeAnchor }",
                    "Conversion { stage: Window, error: TimingBeforeAnchor }",
                ),
                (
                    ClockError::TimingOverflow,
                    "Conversion { stage: TimingNow, error: TimingOverflow }",
                    "Conversion { stage: Window, error: TimingOverflow }",
                ),
                (
                    ClockError::PhysicalOverflow,
                    "Conversion { stage: TimingNow, error: PhysicalOverflow }",
                    "Conversion { stage: Window, error: PhysicalOverflow }",
                ),
                (
                    ClockError::CheckedArithmeticOverflow,
                    "Conversion { stage: TimingNow, error: CheckedArithmeticOverflow }",
                    "Conversion { stage: Window, error: CheckedArithmeticOverflow }",
                ),
                (
                    ClockError::InvalidWindow,
                    "Conversion { stage: TimingNow, error: InvalidWindow }",
                    "Conversion { stage: Window, error: InvalidWindow }",
                ),
                (
                    ClockError::RoundedEmptyWindow,
                    "Conversion { stage: TimingNow, error: RoundedEmptyWindow }",
                    "Conversion { stage: Window, error: RoundedEmptyWindow }",
                ),
            ]
            .into_iter()
            .enumerate()
            {
                let display = match stage {
                    ConversionStage::TimingNow => timing_display,
                    ConversionStage::Window => window_display,
                };
                check(Conversion { stage, error }, first_index + offset, display);
            }
        }
        assert!(seen.into_iter().all(|present| present));
    }

    #[test]
    fn recapture_order_budget_boundaries_and_narrowing_witnesses() {
        use AdmissionRejection::*;
        let base = snapshot_at_frame_start(0);
        let window = TxWindow::Control(base.control);
        let mut changed = base;
        changed.running = false;
        changed.generation = base.generation.wrapping_add(1);
        changed.frame_counter = base.frame_counter.wrapping_add(1);
        changed.control.frame = base.control.frame.wrapping_add(1);
        assert_eq!(window.evaluate(changed, 0, 0).result, Err(Stopped));
        changed.running = true;
        assert_eq!(
            window.evaluate(changed, 0, 0).result,
            Err(GenerationMismatch)
        );
        changed.generation = base.generation;
        assert_eq!(window.evaluate(changed, 0, 0).result, Err(FrameMismatch));
        changed.frame_counter = base.frame_counter;
        assert_eq!(
            window.evaluate(changed, 0, 0).result,
            Err(WorkFrameMismatch)
        );
        changed.control.frame = base.control.frame;
        assert_eq!(
            window.evaluate(changed, 0, 0).result,
            Err(BeforeTimingStart)
        );
        assert_eq!(window.evaluate(base, 17_500, 0).result, Err(TimingExpired));

        for (now, until, len, expected, budget) in [
            (0, 389, 29, Ok(389), Some(389)),
            (1, 389, 29, Err(BudgetTooShort), Some(389)),
            (-1, 389, 29, Err(NegativeBudgetClock), Some(389)),
            (
                i64::MAX - 100,
                i64::MAX,
                0,
                Err(BudgetEndOverflow),
                Some(273),
            ),
            (
                i64::MAX - 1000,
                i64::MAX,
                0,
                Err(DeadlineCapOverflow),
                Some(273),
            ),
        ] {
            let mut evaluation = AdmissionEvaluation::new();
            assert_eq!(evaluate_budget(now, until, len, &mut evaluation), expected);
            assert_eq!(evaluation.budget_us, budget);
        }
        if let Ok(len) = usize::try_from(i64::MAX) {
            let mut evaluation = AdmissionEvaluation::new();
            assert_eq!(
                evaluate_budget(-1, 0, len, &mut evaluation),
                Err(BudgetArithmetic)
            );
            assert_eq!(evaluation.budget_us, None);
            if let Some(len) = len.checked_add(1) {
                assert_eq!(
                    evaluate_budget(-1, 0, len, &mut evaluation),
                    Err(PacketLengthConversion)
                );
            }
        }
        for (anchor, expected) in [
            (i64::MAX as u64 - 1000, PhysicalDeadlineNarrowing),
            (i64::MAX as u64 + 1, PhysicalNowNarrowing),
        ] {
            let mapping = ClockMapping::new(
                ClockProfile::Normalized,
                PhysicalTime::new(anchor),
                TimingTime::new(16_000),
            );
            let evaluation =
                window.evaluate_with_mapping(base, PhysicalTime::new(anchor), 0, mapping);
            assert_eq!(evaluation.result, Err(expected));
            assert_eq!(evaluation.timing_now, Some(16_000));
            assert_eq!(
                evaluation.physical_window_start,
                Some(PhysicalTime::new(anchor))
            );
            assert_eq!(
                evaluation.physical_window_deadline,
                Some(PhysicalTime::new(anchor + 1500))
            );
            assert_eq!(evaluation.budget_us, None);
        }
    }

    #[test]
    fn terminal_pending_and_timeout_never_count_physical_completion() {
        let mut counters = TxCounters::default();
        assert_eq!(counters.attempted, 0); // No submission observation.
        counters.record(TxTerminal::Pending);
        counters.record(TxTerminal::Deadline); // TimedOut/Deadline share this bucket.
        counters.record(TxTerminal::Fault);
        assert_eq!(counters.physical_complete, 0);
        assert_eq!(
            (counters.attempted, counters.deadline, counters.fault),
            (3, 1, 2)
        );
        counters.record(TxTerminal::PhysicalComplete);
        assert_eq!(counters.physical_complete, 1);
    }

    #[test]
    fn diagnostic_policy_copies_track_actual_capability_and_rf_lifecycle() {
        let mut capability = Capability::default();
        assert_eq!(capability.snapshot(), (false, false, false));
        capability.strict_initialized();
        capability.external_ingress();
        assert_eq!(capability.snapshot(), (true, true, false));
        assert!(!capability.ready());
        capability.gate_published();
        assert!(capability.ready());
        let mut rf = RfLifecycle::default();
        rf.stop_or_configure(7, 20);
        let stopped = rf.snapshot();
        rf.started(8);
        assert_eq!(stopped, (false, 8, 20));
        assert_eq!(rf.snapshot(), (true, 8, 20));
    }

    struct Timing(i64);
    impl nrf_embassy::tdma::Clock for Timing {
        fn now_us(&mut self) -> i64 {
            self.0
        }
        fn uptime_ms(&mut self) -> u32 {
            (self.0 / 1000) as u32
        }
    }
    impl nrf_embassy::tdma::Timers for Timing {
        fn init_timer(&mut self, _: nrf_embassy::tdma::TimerKind) {}
        fn arm_timer(&mut self, _: nrf_embassy::tdma::TimerKind, _: i64) {}
        fn cancel_timer(&mut self, _: nrf_embassy::tdma::TimerKind) {}
    }
    impl nrf_embassy::tdma::WorkQueue for Timing {
        fn init_work(&mut self, _: nrf_embassy::tdma::WorkKind) {}
        fn submit_work(&mut self, _: nrf_embassy::tdma::WorkKind) -> i32 {
            1
        }
        fn cancel_work(&mut self, _: nrf_embassy::tdma::WorkKind) {}
    }
    impl nrf_embassy::tdma::WsCapture for Timing {
        fn capture_ws(&mut self) -> Option<u32> {
            None
        }
        fn sample_ws(&mut self, _: u32, _: u32) -> Option<i32> {
            None
        }
    }
    impl nrf_embassy::tdma::Callbacks for Timing {
        fn slot_callback(&mut self, _: u8, _: u32) {}
        fn control_callback(&mut self, _: u32) {}
    }
    fn control_snapshot() -> Snapshot {
        let mut tdma = nrf_embassy::tdma::Tdma::new();
        tdma.start(0, true, &mut Timing(0)).unwrap();
        tdma.timer_expired(nrf_embassy::tdma::TimerKind::Frame, &mut Timing(20_000))
            .unwrap();
        tdma.snapshot()
    }

    fn snapshot_at_frame_start(frame_start: i64) -> Snapshot {
        let mut tdma = nrf_embassy::tdma::Tdma::new();
        tdma.start(0, true, &mut Timing(frame_start - 20_000))
            .unwrap();
        tdma.timer_expired(
            nrf_embassy::tdma::TimerKind::Frame,
            &mut Timing(frame_start),
        )
        .unwrap();
        tdma.snapshot()
    }

    fn clock_mapping(profile: ClockProfile) -> ClockMapping {
        ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(0))
    }

    #[test]
    fn live_dispatcher_maps_normalized_negative_and_positive_offsets() {
        let snapshot = snapshot_at_frame_start(-17_000);
        assert_eq!(snapshot.frame_start_us, -17_000);
        assert_eq!(snapshot.frame_counter, 1);
        assert_eq!(
            (snapshot.control.start_us, snapshot.control.deadline_us),
            (-1000, 500)
        );
        let window = TxWindow::capture(
            snapshot,
            ActorCallback::Control {
                frame: snapshot.frame_counter,
            },
            0,
        )
        .unwrap();
        let mapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(-2000),
        );
        // 29 bytes need 389 physical us; the remaining 500 physical us fit.
        assert_eq!(
            window.deadline_for_clock(snapshot, 2000, 29, mapping),
            Some(2500),
        );
        assert_eq!(window.deadline(snapshot, 2000, 29), None);

        let snapshot = snapshot_at_frame_start(0);
        let window = TxWindow::Control(snapshot.control);
        let mapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(10),
        );
        assert_eq!(
            window.deadline_for_clock(snapshot, 17_000, 29, mapping),
            Some(17_490),
        );
        assert_eq!(window.deadline(snapshot, 17_000, 29), Some(17_500));
        assert_eq!(
            window.deadline_for_clock(snapshot, 17_102, 29, mapping),
            None,
        );
    }

    #[test]
    fn live_dispatcher_reserves_numeric_history_for_exact_default_mapping() {
        let initial = snapshot_at_frame_start(-17_000);
        let window = TxWindow::Control(initial.control);
        let default = clock_mapping(ClockProfile::Normalized);
        assert_eq!(
            window.deadline_for_clock(initial, 0, 29, default),
            Some(500),
        );
        assert_eq!(window.deadline_for_clock(initial, -1, 0, default), None);
        let anchored = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(1000),
            TimingTime::new(1000),
        );
        assert_eq!(window.deadline_for_clock(initial, 0, 29, anchored), None);
        // Even equal nonzero anchors reject a window beginning before the anchor.
        let before_anchor = snapshot_at_frame_start(-15_500);
        let window = TxWindow::Control(before_anchor.control);
        assert_eq!(window.deadline(before_anchor, 1000, 29), Some(2000));
        assert_eq!(
            window.deadline_for_clock(before_anchor, 1000, 29, anchored),
            None,
        );
        let after_anchor = snapshot_at_frame_start(0);
        let window = TxWindow::Control(after_anchor.control);
        assert_eq!(
            window.deadline_for_clock(after_anchor, 17_000, 29, anchored),
            Some(17_500),
        );
        assert_eq!(
            window.deadline_for_clock(after_anchor, -1, 0, anchored),
            None,
        );
        // Compare the production dispatcher against all original numeric cases
        // across initial, zero-based and later frames, including late windows.
        for snapshot in [initial, after_anchor, control_snapshot()] {
            for window in [
                TxWindow::Slot(snapshot.slot),
                TxWindow::Control(snapshot.control),
            ] {
                let metadata = window.metadata();
                for now in metadata.start_us - 1..=metadata.deadline_us + 1 {
                    for packet_len in [0, 29, 200, usize::MAX] {
                        assert_eq!(
                            window.deadline_for_clock(snapshot, now, packet_len, default),
                            window.deadline(snapshot, now, packet_len),
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn live_dispatcher_revalidates_metadata_generation_frame_and_time_for_both_profiles() {
        let snapshot = control_snapshot();
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = ClockMapping::new(profile, PhysicalTime::new(1000), TimingTime::new(10));
            for window in [
                TxWindow::Slot(snapshot.slot),
                TxWindow::Control(snapshot.control),
            ] {
                let metadata = window.metadata();
                let now = i64::try_from(
                    mapping
                        .physical_not_before(TimingTime::new(metadata.start_us + 200))
                        .unwrap()
                        .value(),
                )
                .unwrap();
                assert!(
                    window
                        .deadline_for_clock(snapshot, now, 29, mapping)
                        .is_some()
                );
                for changed in [
                    Snapshot {
                        generation: snapshot.generation.wrapping_add(1),
                        ..snapshot
                    },
                    Snapshot {
                        frame_counter: snapshot.frame_counter.wrapping_add(1),
                        ..snapshot
                    },
                    Snapshot {
                        running: false,
                        ..snapshot
                    },
                ] {
                    assert_eq!(window.deadline_for_clock(changed, now, 29, mapping), None);
                }
                for changed_metadata in [
                    WorkMetadata {
                        generation: metadata.generation.wrapping_add(1),
                        ..metadata
                    },
                    WorkMetadata {
                        frame: metadata.frame.wrapping_add(1),
                        ..metadata
                    },
                    WorkMetadata {
                        start_us: metadata.start_us + 1,
                        ..metadata
                    },
                    WorkMetadata {
                        deadline_us: metadata.deadline_us - 1,
                        ..metadata
                    },
                ] {
                    let mut changed = snapshot;
                    match window {
                        TxWindow::Slot(_) => changed.slot = changed_metadata,
                        TxWindow::Control(_) => changed.control = changed_metadata,
                    }
                    assert_eq!(window.deadline_for_clock(changed, now, 29, mapping), None);
                }
                let before = mapping
                    .physical_not_before(TimingTime::new(metadata.start_us))
                    .unwrap()
                    .value() as i64
                    - 1;
                let late = mapping
                    .physical_not_before(TimingTime::new(metadata.deadline_us))
                    .unwrap()
                    .value() as i64;
                for invalid_now in [-1, 999, before, late] {
                    assert_eq!(
                        window.deadline_for_clock(snapshot, invalid_now, 29, mapping),
                        None,
                    );
                }
                assert_eq!(
                    window.deadline_for_clock(snapshot, now, usize::MAX, mapping),
                    None,
                );
            }
        }
    }

    #[test]
    fn normalized_mapped_deadlines_match_existing_slot_and_control_admission() {
        let mapping = clock_mapping(ClockProfile::Normalized);
        for snapshot in [snapshot_at_frame_start(0), control_snapshot()] {
            for window in [
                TxWindow::Slot(snapshot.slot),
                TxWindow::Control(snapshot.control),
            ] {
                let metadata = window.metadata();
                for now in (metadata.start_us - 1).max(0)..=metadata.deadline_us + 1 {
                    for packet_len in [0, 29, 200] {
                        assert_eq!(
                            window.deadline_with_mapping(
                                snapshot,
                                PhysicalTime::new(now as u64),
                                packet_len,
                                mapping,
                            ),
                            window.deadline(snapshot, now, packet_len),
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn legacy_control_window_rounds_inward_and_uses_physical_airtime() {
        let snapshot = snapshot_at_frame_start(0);
        let window = TxWindow::capture(
            snapshot,
            ActorCallback::Control {
                frame: snapshot.frame_counter,
            },
            16_000,
        )
        .unwrap();
        assert_eq!(window, TxWindow::Control(snapshot.control));
        assert_eq!(
            (snapshot.control.start_us, snapshot.control.deadline_us),
            (16_000, 17_500)
        );
        let mapping = clock_mapping(ClockProfile::LegacyNrfRtc10000);
        let physical_window = mapping
            .convert(TimingTime::new(16_000), TimingTime::new(17_500))
            .unwrap();
        assert_eq!(physical_window.start().value(), 14_649);
        assert_eq!(physical_window.deadline().value(), 16_021);
        for packet_len in [0, 29, 200] {
            assert_eq!(
                window.deadline_with_mapping(
                    snapshot,
                    PhysicalTime::new(14_649),
                    packet_len,
                    mapping,
                ),
                Some(16_021),
            );
            for now in [14_648, 16_020, 16_021] {
                assert_eq!(
                    window.deadline_with_mapping(
                        snapshot,
                        PhysicalTime::new(now),
                        packet_len,
                        mapping,
                    ),
                    None,
                );
            }
        }
        // 29 bytes require 389 physical us. The remaining timing units would
        // incorrectly admit this packet if compared directly with that budget.
        let timing_now = mapping
            .timing_at(PhysicalTime::new(15_640))
            .unwrap()
            .value();
        assert!(17_500 - timing_now >= 389);
        assert_eq!(bounded_air_deadline(timing_now, 17_500, 29), Some(17_500));
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(15_640), 29, mapping),
            None,
        );
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(15_632), 29, mapping),
            Some(16_021),
        );
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(15_633), 29, mapping),
            None,
        );
    }

    #[test]
    fn both_profiles_keep_slot_and_control_airtime_costs_in_physical_us() {
        let snapshot = control_snapshot();
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = clock_mapping(profile);
            for window in [
                TxWindow::Slot(snapshot.slot),
                TxWindow::Control(snapshot.control),
            ] {
                let metadata = window.metadata();
                let physical_window = mapping
                    .convert(
                        TimingTime::new(metadata.start_us),
                        TimingTime::new(metadata.deadline_us),
                    )
                    .unwrap();
                let start = physical_window.start().value();
                let end = physical_window.deadline().value();
                for (packet_len, budget) in [(0, 273), (29, 389), (200, 1073)] {
                    for now in start - 1..=end + 1 {
                        let deadline = window.deadline_with_mapping(
                            snapshot,
                            PhysicalTime::new(now),
                            packet_len,
                            mapping,
                        );
                        let expected = (now >= start && now < end && now + budget <= end)
                            .then_some(end.min(now + 1450));
                        assert_eq!(deadline, expected);
                        if let Some(deadline) = deadline {
                            assert!(deadline > now);
                            assert!(deadline <= end);
                        }
                    }
                    assert_eq!(
                        window.deadline_with_mapping(
                            snapshot,
                            PhysicalTime::new(end - budget),
                            packet_len,
                            mapping,
                        ),
                        Some(end),
                    );
                    assert_eq!(
                        window.deadline_with_mapping(
                            snapshot,
                            PhysicalTime::new(end - budget + 1),
                            packet_len,
                            mapping,
                        ),
                        None,
                    );
                }
                assert_eq!(
                    window.deadline_with_mapping(
                        snapshot,
                        PhysicalTime::new(start),
                        usize::MAX,
                        mapping,
                    ),
                    None,
                );
            }
        }
    }

    #[test]
    fn mapped_admission_revalidates_authoritative_metadata_and_existing_state_rules() {
        let snapshot = control_snapshot();
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = clock_mapping(profile);
            for window in [
                TxWindow::Slot(snapshot.slot),
                TxWindow::Control(snapshot.control),
            ] {
                let metadata = window.metadata();
                let now = mapping
                    .physical_not_before(TimingTime::new(metadata.start_us + 200))
                    .unwrap();
                assert!(
                    window
                        .deadline_with_mapping(snapshot, now, 29, mapping)
                        .is_some()
                );
                for changed in [
                    Snapshot {
                        generation: snapshot.generation.wrapping_add(1),
                        ..snapshot
                    },
                    Snapshot {
                        frame_counter: snapshot.frame_counter.wrapping_add(1),
                        ..snapshot
                    },
                    Snapshot {
                        running: false,
                        ..snapshot
                    },
                ] {
                    assert_eq!(
                        window.deadline_with_mapping(changed, now, 29, mapping),
                        None
                    );
                }
                for changed_metadata in [
                    WorkMetadata {
                        generation: metadata.generation.wrapping_add(1),
                        ..metadata
                    },
                    WorkMetadata {
                        frame: metadata.frame.wrapping_add(1),
                        ..metadata
                    },
                    WorkMetadata {
                        start_us: metadata.start_us + 1,
                        ..metadata
                    },
                    WorkMetadata {
                        deadline_us: metadata.deadline_us - 1,
                        ..metadata
                    },
                ] {
                    let mut changed = snapshot;
                    match window {
                        TxWindow::Slot(_) => changed.slot = changed_metadata,
                        TxWindow::Control(_) => changed.control = changed_metadata,
                    }
                    assert_eq!(
                        window.deadline_with_mapping(changed, now, 29, mapping),
                        None
                    );
                }
                // capture/deadline have no synchronization predicate. Preserve
                // that existing policy rather than adding one in the mapped API.
                let unsynchronized = Snapshot {
                    synchronized: false,
                    ..snapshot
                };
                assert_eq!(
                    window.deadline_with_mapping(unsynchronized, now, 29, mapping),
                    window.deadline_with_mapping(snapshot, now, 29, mapping),
                );
            }
        }
    }

    #[test]
    fn mapped_api_rejects_before_anchor_history_without_changing_live_numeric_policy() {
        let snapshot = snapshot_at_frame_start(-17_000);
        assert_eq!(
            (snapshot.control.start_us, snapshot.control.deadline_us),
            (-1000, 500)
        );
        let window = TxWindow::capture(
            snapshot,
            ActorCallback::Control {
                frame: snapshot.frame_counter,
            },
            0,
        )
        .unwrap();
        assert_eq!(window.deadline(snapshot, 0, 29), Some(500));
        assert_eq!(window.deadline(snapshot, -1, 0), None);
        assert_eq!(
            window.deadline_with_mapping(
                snapshot,
                PhysicalTime::new(0),
                29,
                clock_mapping(ClockProfile::Normalized),
            ),
            None,
        );
        let mapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(-2000),
        );
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(2000), 29, mapping),
            Some(2500),
        );
        let later_anchor = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(2000),
            TimingTime::new(-500),
        );
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(2000), 0, later_anchor),
            None,
        );
        assert_eq!(
            window.deadline_with_mapping(snapshot, PhysicalTime::new(1999), 0, later_anchor),
            None,
        );
    }

    #[test]
    fn mapped_empty_windows_and_physical_i64_overflow_fail_closed() {
        let snapshot = snapshot_at_frame_start(0);
        let empty_snapshot = Snapshot {
            control: WorkMetadata {
                start_us: 1,
                deadline_us: 2,
                ..snapshot.control
            },
            ..snapshot
        };
        assert_eq!(
            TxWindow::Control(empty_snapshot.control).deadline_with_mapping(
                empty_snapshot,
                PhysicalTime::new(1),
                0,
                clock_mapping(ClockProfile::LegacyNrfRtc10000),
            ),
            None,
        );
        for anchor in [
            i64::MAX as u64 - 1000,
            i64::MAX as u64 + 1,
            u64::MAX - 20_000,
            u64::MAX - 1000,
            u64::MAX,
        ] {
            let mapping = ClockMapping::new(
                ClockProfile::Normalized,
                PhysicalTime::new(anchor),
                TimingTime::new(16_000),
            );
            assert_eq!(
                TxWindow::Control(snapshot.control).deadline_with_mapping(
                    snapshot,
                    PhysicalTime::new(anchor),
                    0,
                    mapping,
                ),
                None,
            );
        }
    }

    #[test]
    fn production_control_window_uses_source_metadata_and_exact_airtime_fit() {
        let snapshot = control_snapshot();
        assert_eq!(snapshot.control.start_us - snapshot.frame_start_us, 16_000);
        assert_eq!(
            snapshot.control.deadline_us - snapshot.frame_start_us,
            17_500
        );
        let callback = ActorCallback::Control {
            frame: snapshot.frame_counter,
        };
        assert!(TxWindow::capture(snapshot, callback, 35_999).is_none());
        let window = TxWindow::capture(snapshot, callback, 36_000).unwrap();
        assert_eq!(window.deadline(snapshot, 36_000, 209), Some(37_450));
        // 129 + 4 * (209 + 11) + 100 = 1109 us, inclusive exact fit.
        assert_eq!(window.deadline(snapshot, 36_391, 209), Some(37_500));
        assert_eq!(window.deadline(snapshot, 36_392, 209), None);
        assert_eq!(window.deadline(snapshot, 37_500, 0), None);
    }

    #[test]
    fn production_control_window_rejects_drift_frame_generation_and_stale_callback() {
        let snapshot = control_snapshot();
        let callback = ActorCallback::Control {
            frame: snapshot.frame_counter,
        };
        let window = TxWindow::capture(snapshot, callback, 36_000).unwrap();
        for changed in [
            Snapshot {
                generation: snapshot.generation.wrapping_add(1),
                ..snapshot
            },
            Snapshot {
                frame_counter: snapshot.frame_counter.wrapping_add(1),
                ..snapshot
            },
            Snapshot {
                running: false,
                ..snapshot
            },
            Snapshot {
                control: WorkMetadata {
                    deadline_us: 37_400,
                    ..snapshot.control
                },
                ..snapshot
            },
            Snapshot {
                control: WorkMetadata {
                    start_us: 36_100,
                    ..snapshot.control
                },
                ..snapshot
            },
        ] {
            assert_eq!(window.deadline(changed, 36_200, 9), None);
        }
        assert!(TxWindow::capture(snapshot, callback, 37_500).is_none());
        assert!(
            TxWindow::capture(
                snapshot,
                ActorCallback::Control {
                    frame: snapshot.frame_counter.wrapping_sub(1)
                },
                36_000
            )
            .is_none()
        );
    }

    #[test]
    fn pending_status_rpc_clock_advance_rejects_physical_launch_with_same_frame_snapshot() {
        use crate::bridge_rpc::{Request, State};
        use core::{
            future::poll_fn,
            pin::pin,
            task::{Context, Poll, Waker},
        };
        let snapshot = control_snapshot();
        let window = TxWindow::capture(
            snapshot,
            ActorCallback::Control {
                frame: snapshot.frame_counter,
            },
            36_000,
        )
        .unwrap();
        let mut rpc = State::inactive();
        rpc.initialize().unwrap();
        let ticket = rpc
            .reserve(Request::Status(nrf_embassy::runtime::Membership::default()))
            .unwrap();
        let mut now = 36_000;
        let mut physical_attempts = 0;
        {
            let mut pending = true;
            let mut callback = pin!(async {
                poll_fn(|_| {
                    if core::mem::take(&mut pending) {
                        assert_eq!(rpc.take_result(ticket).unwrap(), None);
                        now = 37_500;
                        Poll::Pending
                    } else {
                        let (claimed, request) = rpc.claim().unwrap();
                        let result = request.apply(&mut nrf_embassy::bridge::Bridge::new());
                        rpc.complete(claimed, result).unwrap();
                        assert_eq!(rpc.take_result(ticket).unwrap(), Some(result));
                        Poll::Ready(())
                    }
                })
                .await;
                if window.deadline(snapshot, now, 9).is_some() {
                    physical_attempts += 1;
                }
            });
            let mut cx = Context::from_waker(Waker::noop());
            assert!(core::future::Future::poll(callback.as_mut(), &mut cx).is_pending());
            assert!(core::future::Future::poll(callback.as_mut(), &mut cx).is_ready());
        }
        assert_eq!(physical_attempts, 0);
    }

    #[test]
    fn capability_requires_all_three_software_boundaries() {
        let mut capability = Capability::default();
        let mut bridge = nrf_embassy::bridge::Bridge::new();
        let input = *bridge.prepare().bytes();
        assert!(!capability.ready());
        capability.gate_published();
        capability.external_ingress();
        assert!(!capability.ready());
        let masked = status_tx(&input, capability.ready());
        assert_eq!(
            &openhelmet_wire::bridge::decode(&masked, nrf_embassy::bridge::MAX_PAYLOAD)
                .unwrap()
                .payload[7..10],
            &[0, 0, 0]
        );
        capability.strict_initialized();
        assert!(capability.ready());
        let ready = status_tx(&input, capability.ready());
        assert_eq!(
            &openhelmet_wire::bridge::decode(&ready, nrf_embassy::bridge::MAX_PAYLOAD)
                .unwrap()
                .payload[7..10],
            &[0xa5, 2, 20]
        );
    }

    #[test]
    fn rf_floor_survives_normal_tx_restore_and_rejects_prior_lifecycle() {
        let mut lifecycle = RfLifecycle::default();
        lifecycle.stop_or_configure(0, 40);
        lifecycle.started(1);
        // Facade advances its RX epoch during TX restore; the actor does not.
        for epoch in [1, 2, 3] {
            assert!(lifecycle.accepts(209, 40, epoch, Some(42)));
        }
        assert!(!lifecycle.accepts(210, 40, 3, Some(42)));
        assert!(!lifecycle.accepts(9, 20, 3, Some(42)));
        assert!(!lifecycle.accepts(9, 40, 3, None));
        lifecycle.stop_or_configure(3, 40);
        assert!(!lifecycle.accepts(9, 40, 4, Some(42)));
        lifecycle.started(4);
        assert!(!lifecycle.accepts(9, 40, 3, Some(42)));
        assert!(lifecycle.accepts(9, 40, 4, Some(42)));
    }

    #[test]
    fn actual_output_requests_preserve_results_and_bridge_mutations() {
        use crate::bridge_rpc::State;
        use nrf_embassy::{
            bridge::Bridge,
            runtime::{Command, CommandKind, Membership},
        };
        let mut state = State::inactive();
        state.initialize().unwrap();
        let mut bridge = Bridge::new();
        for effect in [
            Effect::Status(Membership::default()),
            Effect::Ack {
                command: Command {
                    kind: CommandKind::Start,
                    generation: 254,
                    talk_channel: 2,
                },
                result: -5,
            },
            Effect::MeshStopped,
            Effect::SyncLost,
            Effect::DiscardBridgeAudio,
            Effect::BridgeDelivery(false),
        ] {
            let request = runtime_output(effect).unwrap().unwrap();
            let ticket = state.reserve(request).unwrap();
            assert_eq!(state.take_result(ticket).unwrap(), None);
            let (claimed, request) = state.claim().unwrap();
            let result = request.apply(&mut bridge);
            assert_eq!(state.take_result(ticket).unwrap(), None);
            state.complete(claimed, result).unwrap();
            assert_eq!(state.take_result(ticket).unwrap(), Some(result));
        }
        assert_eq!(Request::audio_bundle(1, &[0; 4]), Err(-22));
        let mut bundle = [0; 56];
        assert_eq!(
            openhelmet_wire::audio_bundle::audio_bundle_encode(
                &openhelmet_wire::audio_bundle::BundleView {
                    current_data: &[42; 48],
                    previous1_data: &[],
                    previous2_data: &[],
                    current_seq: 1,
                    stream_id: 1,
                    flags: 1,
                    codec: 2,
                },
                &mut bundle,
            ),
            56
        );
        let audio = Request::audio_bundle(1, &bundle).unwrap();
        assert_eq!(audio.apply(&mut bridge), -11); // Actual execution gate, not reservation.
        assert_eq!(Request::Delivery(true).apply(&mut bridge), 0);
        assert_eq!(audio.apply(&mut bridge), 0);
        assert_eq!(Request::log(&[]), Err(-22));
        assert_eq!(Request::event(1, &[0; 208]), Err(-90));
        assert!(runtime_output(Effect::StartRx).is_none());
    }
}
