//! Pure, bounded TX accounting, enabled only by `sync-trace`.
//!
//! Classification runs once at the hardware async backend boundary. Invalid
//! means invalid Mesh envelope (including the NRF payload bound), not invalid
//! type-specific payload; unknown types with valid envelopes are Other.
//! Requests reconcile with four pre-ticket rejects plus accepted tickets;
//! accepted tickets reconcile with raw terminal outcomes once await returns.
//! Actor handlers run to completion. Dropping an outer send future before its
//! terminal return leaves an accepted ticket outstanding here: drop cancellation
//! is not observed or counted. These software counters do not qualify RF timing.

use crate::actor_policy::{AdmissionEvaluation, AdmissionRejection, TxWindow};
use crate::timing_clock::ClockMapping;
use nrf_embassy::tdma::{Snapshot, WorkMetadata};
use openhelmet_wire::mesh;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindowKind {
    Slot,
    Control,
}

/// Only the current snapshot fields needed to interpret the first predicate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmissionSnapshot {
    pub generation: u32,
    pub frame: u32,
    pub frame_start: i64,
    pub slot_index: i8,
    pub running: bool,
    pub synchronized: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LastAdmissionRejection {
    pub reason: AdmissionRejection,
    pub kind: WindowKind,
    // usize preserves the exact host overflow input as well as ARM packet lengths.
    pub packet_len: usize,
    pub mapping: ClockMapping,
    pub physical_now: i64,
    pub timing_now: Option<i64>,
    pub physical_window_start: Option<u64>,
    pub physical_window_deadline: Option<u64>,
    pub budget_us: Option<i64>,
    pub saved: WorkMetadata,
    pub current: WorkMetadata,
    pub snapshot: AdmissionSnapshot,
    /// Per-class backend rejection count at capture, not the emitter's actor_seq.
    pub ordinal: u32,
}

impl LastAdmissionRejection {
    pub fn from_evaluation(
        window: TxWindow,
        snapshot: Snapshot,
        physical_now: i64,
        packet_len: usize,
        mapping: ClockMapping,
        evaluation: AdmissionEvaluation,
    ) -> Option<Self> {
        let reason = evaluation.result.err()?;
        let (kind, current) = match window {
            TxWindow::Slot(_) => (WindowKind::Slot, snapshot.slot),
            TxWindow::Control(_) => (WindowKind::Control, snapshot.control),
        };
        Some(Self {
            reason,
            kind,
            packet_len,
            mapping,
            physical_now,
            timing_now: evaluation.timing_now,
            physical_window_start: evaluation.physical_window_start.map(|time| time.value()),
            physical_window_deadline: evaluation.physical_window_deadline.map(|time| time.value()),
            budget_us: evaluation.budget_us,
            saved: window.metadata(),
            current,
            snapshot: AdmissionSnapshot {
                generation: snapshot.generation,
                frame: snapshot.frame_counter,
                frame_start: snapshot.frame_start_us,
                slot_index: snapshot.slot_index,
                running: snapshot.running,
                synchronized: snapshot.synchronized,
            },
            ordinal: 0,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireClass {
    Sync,
    Audio,
    Join,
    JoinAck,
    Other,
    Invalid,
}

impl WireClass {
    pub const ALL: [Self; 6] = [
        Self::Sync,
        Self::Audio,
        Self::Join,
        Self::JoinAck,
        Self::Other,
        Self::Invalid,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Sync => "Sync",
            Self::Audio => "Audio",
            Self::Join => "Join",
            Self::JoinAck => "JoinAck",
            Self::Other => "Other",
            Self::Invalid => "Invalid",
        }
    }
}

pub fn classify(bytes: &[u8]) -> WireClass {
    match mesh::decode(bytes, mesh::NRF_RECEIVE_PAYLOAD_CAPACITY) {
        Ok(envelope) => match envelope.header.packet_type {
            5 | 17 => WireClass::Sync,
            1 | 13 => WireClass::Audio,
            2 | 11 | 15 => WireClass::Join,
            3 | 12 | 16 => WireClass::JoinAck,
            _ => WireClass::Other,
        },
        Err(_) => WireClass::Invalid,
    }
}

/// Stages remain separate from errno and the existing aggregate TX semantics.
#[derive(Clone, Copy, Debug)]
pub enum Stage {
    Requests,
    BackendWindowRejected,
    ValidationRejected,
    BudgetRejected,
    SubmissionRejected,
    AcceptedTickets,
    Complete,
    GuardDeadline,
    SoftwareTimedOut,
    Cancelled,
    Other,
}

/// Lossless projection of the raw ticket outcome causes relevant to accounting.
#[derive(Clone, Copy, Debug)]
pub enum Terminal {
    PhysicalComplete,
    Deadline,
    TimedOut,
    Cancelled,
    Pending,
    OtherFault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Row {
    pub requests: u32,
    pub backend_window_rejected: u32,
    pub validation_rejected: u32,
    pub budget_rejected: u32,
    pub submission_rejected: u32,
    pub accepted_tickets: u32,
    pub complete: u32,
    pub guard_deadline: u32,
    pub software_timed_out: u32,
    pub cancelled: u32,
    pub other: u32,
}

/// Bounded actor-owned storage, also copied through existing snapshot publication.
/// These static bounds do not qualify CPU copy time, alarm latency or RF timing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stats {
    pub rows: [Row; 6],
    pub counter_saturated: bool,
    pub admission_reasons: [[u32; AdmissionRejection::COUNT]; 6],
    pub last_admission_rejection: [Option<LastAdmissionRejection>; 6],
}

impl Default for Stats {
    fn default() -> Self {
        Self {
            rows: [Row::default(); 6],
            counter_saturated: false,
            admission_reasons: [[0; AdmissionRejection::COUNT]; 6],
            last_admission_rejection: [None; 6],
        }
    }
}

const _: () = assert!(core::mem::size_of::<LastAdmissionRejection>() <= 256);
const _: () = assert!(core::mem::size_of::<Stats>() <= 4096);

impl Stats {
    /// One actor mutation updates the legacy stage and its reason together.
    pub fn record_backend_window_rejection(
        &mut self,
        class: WireClass,
        mut witness: LastAdmissionRejection,
    ) {
        self.record(class, Stage::BackendWindowRejected);
        let index = class as usize;
        let counter = &mut self.admission_reasons[index][witness.reason.index()];
        *counter = counter.saturating_add(1);
        self.counter_saturated |= *counter == u32::MAX;
        witness.ordinal = self.rows[index].backend_window_rejected;
        self.last_admission_rejection[index] = Some(witness);
    }

    pub fn record_terminal(&mut self, class: WireClass, terminal: Terminal) {
        self.record(
            class,
            match terminal {
                Terminal::PhysicalComplete => Stage::Complete,
                Terminal::Deadline => Stage::GuardDeadline,
                Terminal::TimedOut => Stage::SoftwareTimedOut,
                Terminal::Cancelled => Stage::Cancelled,
                Terminal::Pending | Terminal::OtherFault => Stage::Other,
            },
        );
    }

    pub fn record(&mut self, class: WireClass, stage: Stage) {
        let row = &mut self.rows[class as usize];
        let counter = match stage {
            Stage::Requests => &mut row.requests,
            Stage::BackendWindowRejected => &mut row.backend_window_rejected,
            Stage::ValidationRejected => &mut row.validation_rejected,
            Stage::BudgetRejected => &mut row.budget_rejected,
            Stage::SubmissionRejected => &mut row.submission_rejected,
            Stage::AcceptedTickets => &mut row.accepted_tickets,
            Stage::Complete => &mut row.complete,
            Stage::GuardDeadline => &mut row.guard_deadline,
            Stage::SoftwareTimedOut => &mut row.software_timed_out,
            Stage::Cancelled => &mut row.cancelled,
            Stage::Other => &mut row.other,
        };
        *counter = counter.saturating_add(1);
        self.counter_saturated |= *counter == u32::MAX;
    }
}

#[cfg(test)]
pub(crate) fn all_reasons() -> [AdmissionRejection; AdmissionRejection::COUNT] {
    use crate::actor_policy::ConversionStage;
    use crate::timing_clock::ClockError;
    use AdmissionRejection::*;
    let mut reasons = [Stopped; AdmissionRejection::COUNT];
    reasons[..9].copy_from_slice(&[
        CallbackSlotMismatch,
        Stopped,
        GenerationMismatch,
        FrameMismatch,
        WorkFrameMismatch,
        BeforeTimingStart,
        TimingExpired,
        WindowMetadataChanged,
        NegativePhysicalClock,
    ]);
    let mut index = 9;
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
            reasons[index] = Conversion { stage, error };
            index += 1;
        }
    }
    reasons[23..].copy_from_slice(&[
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
    ]);
    reasons
}

#[cfg(test)]
pub(crate) fn test_witness(reason: AdmissionRejection) -> LastAdmissionRejection {
    use crate::timing_clock::{ClockProfile, PhysicalTime, TimingTime};
    let mut snapshot = nrf_embassy::tdma::Tdma::new().snapshot();
    snapshot.control = WorkMetadata {
        generation: u32::MAX,
        frame: u32::MAX,
        start_us: i64::MIN,
        deadline_us: i64::MAX,
    };
    snapshot.generation = u32::MAX;
    snapshot.frame_counter = u32::MAX;
    snapshot.frame_start_us = i64::MIN;
    snapshot.slot_index = i8::MIN;
    LastAdmissionRejection::from_evaluation(
        TxWindow::Control(snapshot.control),
        snapshot,
        i64::MIN,
        usize::MAX,
        ClockMapping::new(
            ClockProfile::LegacyNrfRtc10000,
            PhysicalTime::new(u64::MAX),
            TimingTime::new(i64::MIN),
        ),
        AdmissionEvaluation {
            result: Err(reason),
            timing_now: Some(i64::MIN),
            physical_window_start: Some(PhysicalTime::new(u64::MAX)),
            physical_window_deadline: Some(PhysicalTime::new(u64::MAX)),
            budget_us: Some(i64::MIN),
        },
    )
    .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_reason_records_once_and_other_stages_retain_the_witness() {
        let mut stats = Stats::default();
        for class in WireClass::ALL {
            for (index, reason) in all_reasons().into_iter().enumerate() {
                assert_eq!(reason.index(), index);
                let before = stats;
                stats.record(class, Stage::Requests);
                stats.record_backend_window_rejection(class, test_witness(reason));
                let last = stats.last_admission_rejection[class as usize].unwrap();
                assert_eq!(last.reason, reason);
                assert_eq!(last.ordinal, index as u32 + 1);
                for (counter, (current, previous)) in stats.admission_reasons[class as usize]
                    .iter()
                    .zip(before.admission_reasons[class as usize].iter())
                    .enumerate()
                {
                    assert_eq!(*current, *previous + u32::from(counter == index),);
                }
            }
            let index = class as usize;
            let last = stats.last_admission_rejection[index];
            for stage in [
                Stage::ValidationRejected,
                Stage::BudgetRejected,
                Stage::SubmissionRejected,
                Stage::AcceptedTickets,
            ] {
                stats.record(class, Stage::Requests);
                stats.record(class, stage);
            }
            stats.record_terminal(class, Terminal::PhysicalComplete);
            assert_eq!(stats.last_admission_rejection[index], last);
            let row = stats.rows[index];
            assert_eq!(
                stats.admission_reasons[index].iter().sum::<u32>(),
                row.backend_window_rejected,
            );
            assert_eq!(
                row.requests,
                row.backend_window_rejected
                    + row.validation_rejected
                    + row.budget_rejected
                    + row.submission_rejected
                    + row.accepted_tickets,
            );
        }
        assert!(!stats.counter_saturated);
        assert!(core::mem::size_of::<LastAdmissionRejection>() <= 256);
        assert!(core::mem::size_of::<Stats>() <= 4096);
    }

    #[test]
    fn admission_saturation_and_ordinal_are_sticky_without_wrapping() {
        let mut stats = Stats::default();
        let reason = AdmissionRejection::BudgetTooShort;
        stats.rows[1].backend_window_rejected = u32::MAX - 1;
        stats.admission_reasons[1][reason.index()] = u32::MAX - 1;
        for _ in 0..3 {
            stats.record_backend_window_rejection(WireClass::Audio, test_witness(reason));
            assert_eq!(stats.rows[1].backend_window_rejected, u32::MAX);
            assert_eq!(stats.admission_reasons[1][reason.index()], u32::MAX);
            assert_eq!(stats.last_admission_rejection[1].unwrap().ordinal, u32::MAX);
            assert!(stats.counter_saturated);
        }
        stats.record(WireClass::Other, Stage::Complete);
        assert!(stats.counter_saturated);
    }

    #[test]
    fn witness_uses_reached_values_and_relevant_work_without_reevaluation() {
        use crate::timing_clock::{ClockProfile, PhysicalTime, TimingTime};
        let mut snapshot = nrf_embassy::tdma::Tdma::new().snapshot();
        snapshot.running = true;
        snapshot.frame_counter = 7;
        snapshot.generation = 3;
        snapshot.control = WorkMetadata {
            generation: 3,
            frame: 7,
            start_us: 0,
            deadline_us: 1000,
        };
        let window = TxWindow::Control(snapshot.control);
        let mapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(0),
        );
        let success = window.evaluate_for_clock(snapshot, 0, 0, mapping);
        assert!(success.result.is_ok());
        assert!(
            LastAdmissionRejection::from_evaluation(window, snapshot, 0, 0, mapping, success)
                .is_none()
        );
        let overflow = window.evaluate_for_clock(snapshot, 0, usize::MAX, mapping);
        let witness = LastAdmissionRejection::from_evaluation(
            window,
            snapshot,
            0,
            usize::MAX,
            mapping,
            overflow,
        )
        .unwrap();
        assert_eq!(witness.packet_len, usize::MAX);
        assert_eq!(witness.timing_now, Some(0));
        assert_eq!(witness.budget_us, overflow.budget_us);
        assert!(overflow.result.is_err());
        if usize::BITS == 64 {
            assert_eq!(witness.reason, AdmissionRejection::PacketLengthConversion);
            assert_eq!(witness.budget_us, None);
        }
        snapshot.control.frame = 8;
        let evaluation = window.evaluate_for_clock(snapshot, 0, 0, mapping);
        let witness =
            LastAdmissionRejection::from_evaluation(window, snapshot, 0, 0, mapping, evaluation)
                .unwrap();
        assert_eq!(witness.reason, AdmissionRejection::WorkFrameMismatch);
        assert_eq!(
            (
                witness.saved.frame,
                witness.current.frame,
                witness.snapshot.frame
            ),
            (7, 8, 7)
        );
        assert_eq!(witness.physical_window_start, None);
        assert_eq!(witness.physical_window_deadline, None);
        assert_eq!(witness.budget_us, None);
        let legacy = ClockMapping::new(
            ClockProfile::LegacyNrfRtc10000,
            PhysicalTime::new(2),
            TimingTime::new(0),
        );
        for window in [
            TxWindow::Control(snapshot.control),
            TxWindow::Slot(snapshot.slot),
        ] {
            let evaluation = window.evaluate_for_clock(snapshot, 1, 0, legacy);
            let witness =
                LastAdmissionRejection::from_evaluation(window, snapshot, 1, 0, legacy, evaluation)
                    .unwrap();
            assert_eq!(witness.mapping, legacy);
            assert_eq!(witness.physical_now, 1);
            assert_eq!(witness.timing_now, None);
            assert_eq!(witness.physical_window_start, None);
            assert_eq!(witness.physical_window_deadline, None);
            assert_eq!(witness.budget_us, None);
            assert_eq!(
                witness.reason,
                AdmissionRejection::Conversion {
                    stage: crate::actor_policy::ConversionStage::TimingNow,
                    error: crate::timing_clock::ClockError::PhysicalBeforeAnchor,
                },
            );
            match window {
                TxWindow::Slot(_) => {
                    assert_eq!(witness.kind, WindowKind::Slot);
                    assert_eq!(witness.current, snapshot.slot);
                }
                TxWindow::Control(_) => {
                    assert_eq!(witness.kind, WindowKind::Control);
                    assert_eq!(witness.current, snapshot.control);
                }
            }
        }
    }

    #[test]
    fn all_types_classify_by_envelope_even_with_bad_typed_payloads() {
        for id in 0..=u8::MAX {
            let bytes = [mesh::VERSION, id, 0, 0, 0, 0xff, 0, 0, 0];
            let expected = match id {
                5 | 17 => WireClass::Sync,
                1 | 13 => WireClass::Audio,
                2 | 11 | 15 => WireClass::Join,
                3 | 12 | 16 => WireClass::JoinAck,
                _ => WireClass::Other,
            };
            assert_eq!(classify(&bytes), expected);
        }
        let mut bytes = [0; mesh::HEADER_SIZE + 201];
        bytes[0] = mesh::VERSION;
        bytes[1] = 255;
        bytes[7] = 200;
        assert_eq!(
            classify(&bytes[..mesh::HEADER_SIZE + 200]),
            WireClass::Other
        );
        assert_eq!(classify(&bytes), WireClass::Invalid); // length disagreement
        bytes[7] = 201;
        assert_eq!(classify(&bytes), WireClass::Invalid); // NRF capacity
        bytes[0] = mesh::VERSION - 1;
        assert_eq!(classify(&bytes), WireClass::Invalid);
        assert_eq!(classify(&bytes[..8]), WireClass::Invalid);
        assert_eq!(classify(&[]), WireClass::Invalid);
    }

    #[test]
    fn rejects_and_raw_terminal_causes_reconcile_independently() {
        let mut stats = Stats::default();
        for class in WireClass::ALL {
            for (index, rejection) in [
                Stage::BackendWindowRejected,
                Stage::ValidationRejected,
                Stage::BudgetRejected,
                Stage::SubmissionRejected,
            ]
            .into_iter()
            .enumerate()
            {
                for _ in 0..=index {
                    stats.record(class, Stage::Requests);
                    stats.record(class, rejection);
                }
            }
            for (index, terminal) in [
                Terminal::PhysicalComplete,
                Terminal::Deadline,
                Terminal::TimedOut,
                Terminal::Cancelled,
                Terminal::Pending,
                Terminal::OtherFault,
            ]
            .into_iter()
            .enumerate()
            {
                for _ in 0..=index {
                    stats.record(class, Stage::Requests);
                    stats.record(class, Stage::AcceptedTickets);
                    stats.record_terminal(class, terminal);
                }
            }
            let row = stats.rows[class as usize];
            assert_eq!(row.requests, 31);
            assert_eq!(row.accepted_tickets, 21);
            assert_eq!(
                (
                    row.backend_window_rejected,
                    row.validation_rejected,
                    row.budget_rejected,
                    row.submission_rejected
                ),
                (1, 2, 3, 4)
            );
            assert_eq!(
                row.requests,
                row.backend_window_rejected
                    + row.validation_rejected
                    + row.budget_rejected
                    + row.submission_rejected
                    + row.accepted_tickets
            );
            assert_eq!(
                (
                    row.complete,
                    row.guard_deadline,
                    row.software_timed_out,
                    row.cancelled,
                    row.other
                ),
                (1, 2, 3, 4, 11)
            );
            assert_eq!(
                row.accepted_tickets,
                row.complete
                    + row.guard_deadline
                    + row.software_timed_out
                    + row.cancelled
                    + row.other
            );
        }
        assert!(!stats.counter_saturated);
        let mut outstanding = Stats::default();
        outstanding.record(WireClass::Audio, Stage::Requests);
        outstanding.record(WireClass::Audio, Stage::AcceptedTickets);
        assert_eq!(outstanding.rows[1].accepted_tickets, 1);
        assert_eq!(outstanding.rows[1].cancelled, 0);
    }

    #[test]
    fn saturation_is_sticky_nonwrapping_and_copies_are_independent() {
        let mut stats = Stats::default();
        stats.rows[0].requests = u32::MAX - 1;
        let before = stats;
        stats.record(WireClass::Sync, Stage::Requests);
        assert!(stats.counter_saturated);
        let saturated = stats;
        stats.record(WireClass::Sync, Stage::Requests);
        stats.record(WireClass::Audio, Stage::Complete);
        assert_eq!(stats.rows[0].requests, u32::MAX);
        assert!(stats.counter_saturated);
        assert_eq!(before.rows[0].requests, u32::MAX - 1);
        assert!(!before.counter_saturated);
        assert_eq!(saturated.rows[1].complete, 0);
        assert_eq!(stats.rows[1].complete, 1);
        assert_eq!(core::mem::size_of::<[Row; 6]>(), 264);
    }
}
