//! Accepted-ticket failures only. The last waiter evaluation is a software
//! sample in the actor's run-to-completion path, never RF END or IRQ entry.
//! That attribution cannot universally be applied to other vendor callers.
//! END fields are observational TIMER4 raw candidates, never RF END physical
//! time or latency calibration, including reason Candidate. Compact parser keys:
//! end_d=domain, end_r=reason, end_e=capture epoch, end_m=match to ticket epoch,
//! end_a/end_c=raw anchor/cutoff, end_s0/end_s1=raw CC2 samples,
//! end_o=raw observation, end_t=elapsed us, end_h=horizon us, end_dt=cutoff delta us.

use core::fmt::{self, Write};
use openhelmet_wire::mesh;

pub const RECORD_CAPACITY: usize = 8;
pub const LINE_CAPACITY: usize = crate::sync_trace::LINE_CAPACITY;

/// Exact vendor terminal names, without errno/class folding. PhysicalComplete
/// is included only so the pure capture boundary can explicitly filter it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Terminal {
    PhysicalComplete,
    Pending,
    WrongContext,
    Busy,
    NotReady,
    TimedOut,
    Deadline,
    DeadlineGuardFault,
    Cancelled,
    PowerCycled,
    ClockRegressed,
    ReceiverUnavailable,
    StaleEpoch,
    RxEpochOverflow,
}

impl fmt::Display for Terminal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PhysicalComplete | Self::Pending => write!(f, "{self:?}"),
            _ => write!(f, "Fault({self:?})"),
        }
    }
}

/// First observation of the existing current-TX END + DISABLED predicate.
/// This is provenance only, never a timestamp or proof of an RF deadline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompletionOrigin {
    RadioIrq,
    TaskService,
}

impl fmt::Display for CompletionOrigin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EndCaptureReason {
    Missing,
    Unstable,
    NoCompletionEvidence,
    OutsideEpochRange,
    ObservationOutsideHorizon,
    NotArmed,
    Candidate,
}

impl fmt::Display for EndCaptureReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

/// Pure numeric mirror, with no vendor, clock or peripheral dependency.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EndCapture {
    pub epoch: u64,
    /// Direct comparison of original vendor epochs against the accepted ticket.
    pub epoch_match: bool,
    pub anchor_raw: u32,
    pub cutoff_raw: u32,
    pub sampled_cc: [u32; 2],
    pub observation_raw: u32,
    pub elapsed_us: u64,
    pub horizon_us: u32,
    pub reason: EndCaptureReason,
    pub cutoff_delta_us: Option<i32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Observations {
    pub completion: Option<CompletionOrigin>,
    /// At least one existing matching provider expiry query returned true.
    /// False includes an absent provider and does not prove TX was unexpired.
    pub provider_expiry_observed: bool,
    pub end_capture: Option<EndCapture>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Envelope {
    pub packet_type: u8,
    pub source: u8,
    pub seq: u8,
    pub payload_len: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    /// Vendor ticket epoch is the record identity; report identity is independent.
    pub ticket_epoch: u64,
    pub report_epoch: u64,
    /// Supplied by comparing the original vendor epochs, not reconstructed ones.
    pub epoch_match: bool,
    pub envelope: Result<Envelope, mesh::Error>,
    pub wire_len: usize,
    pub admission_us: u64,
    pub effective_deadline_us: Option<u64>,
    pub last_poll_sample_us: Option<u64>,
    pub terminal: Terminal,
    /// None means the vendor report carried no matching flight observations.
    pub observations: Option<Observations>,
}

impl Record {
    /// Parse immutable bytes only after vendor wait has returned. Unknown packet
    /// types/source IDs remain raw; envelope validity does not validate payloads.
    pub fn failure(
        bytes: &[u8],
        epochs: (u64, u64, bool),
        admission_us: u64,
        deadline: Option<u64>,
        last_poll_sample_us: Option<u64>,
        (terminal, observations): (Terminal, Option<Observations>),
    ) -> Option<Self> {
        if terminal == Terminal::PhysicalComplete {
            return None;
        }
        Some(Self {
            ticket_epoch: epochs.0,
            report_epoch: epochs.1,
            epoch_match: epochs.2,
            envelope: mesh::decode(bytes, mesh::NRF_RECEIVE_PAYLOAD_CAPACITY).map(|e| Envelope {
                packet_type: e.header.packet_type,
                source: e.header.src_id,
                seq: e.header.seq,
                payload_len: e.header.payload_len,
            }),
            wire_len: bytes.len(),
            admission_us,
            effective_deadline_us: deadline.or_else(|| admission_us.checked_add(1450)),
            last_poll_sample_us,
            terminal,
            observations,
        })
    }
}

fn add(value: &mut u64, amount: u64, saturated: &mut bool) {
    match value.checked_add(amount) {
        Some(next) => *value = next,
        None => {
            *value = u64::MAX;
            *saturated = true;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub captured: u64,
    pub overflow: u64,
    pub batches: u64,
    pub id_exhausted: u64,
    pub saturated: bool,
    pub pending: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Batch {
    pub id: u64,
    records: [Option<Record>; RECORD_CAPACITY],
    len: usize,
}

impl Batch {
    pub fn records(&self) -> impl Iterator<Item = &Record> {
        self.records[..self.len].iter().flatten()
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Publication {
    pub batch: Option<Batch>,
    pub counters: Counters,
}

pub struct Buffer {
    records: [Option<Record>; RECORD_CAPACITY],
    next_batch_id: Option<u64>,
    counters: Counters,
}

impl Default for Buffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Buffer {
    pub const fn new() -> Self {
        Self {
            records: [None; RECORD_CAPACITY],
            next_batch_id: Some(0),
            counters: Counters {
                captured: 0,
                overflow: 0,
                batches: 0,
                id_exhausted: 0,
                saturated: false,
                pending: 0,
            },
        }
    }

    pub fn capture(&mut self, record: Record) {
        if record.terminal == Terminal::PhysicalComplete {
            return;
        }
        let c = &mut self.counters;
        if c.pending == RECORD_CAPACITY {
            add(&mut c.overflow, 1, &mut c.saturated);
            return;
        }
        self.records[c.pending] = Some(record);
        c.pending += 1;
        add(&mut c.captured, 1, &mut c.saturated);
    }

    /// Empty ticks publish health only. Exhaustion retains all pending records,
    /// never wraps an ID, and continues publishing health on every tick.
    pub fn drain(&mut self) -> Publication {
        let c = &mut self.counters;
        let batch = if c.pending == 0 {
            None
        } else if let Some(id) = self.next_batch_id {
            self.next_batch_id = id.checked_add(1);
            add(&mut c.batches, 1, &mut c.saturated);
            let len = core::mem::replace(&mut c.pending, 0);
            Some(Batch {
                id,
                records: core::mem::replace(&mut self.records, [None; RECORD_CAPACITY]),
                len,
            })
        } else {
            add(&mut c.id_exhausted, 1, &mut c.saturated);
            None
        };
        Publication {
            batch,
            counters: *c,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Loss {
    Queue,
    Format,
    Usb,
}

/// Separate from SYNC transport totals. Records count the unsubmitted suffix,
/// including a possibly partial current row. Health counts lost health rows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransportCounters {
    pub queue_records: u64,
    pub queue_health: u64,
    pub format_records: u64,
    pub format_health: u64,
    pub usb_records: u64,
    pub usb_health: u64,
    pub saturated: bool,
}

impl TransportCounters {
    pub const fn new() -> Self {
        Self {
            queue_records: 0,
            queue_health: 0,
            format_records: 0,
            format_health: 0,
            usb_records: 0,
            usb_health: 0,
            saturated: false,
        }
    }

    pub fn lose(&mut self, cause: Loss, remaining_records: usize, health_pending: bool) {
        let (records, health) = match cause {
            Loss::Queue => (&mut self.queue_records, &mut self.queue_health),
            Loss::Format => (&mut self.format_records, &mut self.format_health),
            Loss::Usb => (&mut self.usb_records, &mut self.usb_health),
        };
        add(records, remaining_records as u64, &mut self.saturated);
        add(health, u64::from(health_pending), &mut self.saturated);
    }
}

struct Optional<T>(Option<T>);
impl<T: fmt::Display> fmt::Display for Optional<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(value) => value.fmt(f),
            None => f.write_str("none"),
        }
    }
}

pub struct Line {
    bytes: [u8; LINE_CAPACITY],
    len: usize,
}

impl Line {
    fn new() -> Self {
        Self {
            bytes: [0; LINE_CAPACITY],
            len: 0,
        }
    }

    pub fn record(batch: u64, r: Record) -> Result<Self, fmt::Error> {
        let mut line = Self::new();
        let e = r.envelope.ok();
        let end = r.observations.and_then(|o| o.end_capture);
        write!(
            line,
            "TXFAIL batch={} ticket_epoch={} report_epoch={} epoch_match={} parse={} type={} source={} seq={} payload_len={} wire_len={} admission_us={} effective_deadline_us={} deadline_absent={} last_poll_sample_us={} poll_absent={} terminal={} sample=waiter_eval observations_present={} completion_origin={} provider_expiry_observed={} end_present={} end_d={} end_r={} end_e={} end_m={} end_a={} end_c={} end_s0={} end_s1={} end_o={} end_t={} end_h={} end_dt={}\r\n",
            batch,
            r.ticket_epoch,
            r.report_epoch,
            r.epoch_match,
            match r.envelope {
                Ok(_) => "valid",
                Err(mesh::Error::Length) => "length",
                Err(mesh::Error::Version) => "version",
                Err(mesh::Error::Capacity) => "capacity",
                Err(mesh::Error::UnknownType) => "unknown_type",
            },
            Optional(e.map(|e| e.packet_type)),
            Optional(e.map(|e| e.source)),
            Optional(e.map(|e| e.seq)),
            Optional(e.map(|e| e.payload_len)),
            r.wire_len,
            r.admission_us,
            Optional(r.effective_deadline_us),
            r.effective_deadline_us.is_none(),
            Optional(r.last_poll_sample_us),
            r.last_poll_sample_us.is_none(),
            r.terminal,
            r.observations.is_some(),
            Optional(r.observations.and_then(|o| o.completion)),
            Optional(r.observations.map(|o| o.provider_expiry_observed)),
            end.is_some(),
            Optional(end.map(|_| "TIMER4raw")),
            Optional(end.map(|c| c.reason)),
            Optional(end.map(|c| c.epoch)),
            Optional(end.map(|c| c.epoch_match)),
            Optional(end.map(|c| c.anchor_raw)),
            Optional(end.map(|c| c.cutoff_raw)),
            Optional(end.map(|c| c.sampled_cc[0])),
            Optional(end.map(|c| c.sampled_cc[1])),
            Optional(end.map(|c| c.observation_raw)),
            Optional(end.map(|c| c.elapsed_us)),
            Optional(end.map(|c| c.horizon_us)),
            Optional(end.and_then(|c| c.cutoff_delta_us)),
        )?;
        Ok(line)
    }

    pub fn health(p: Publication, t: TransportCounters) -> Result<Self, fmt::Error> {
        let mut line = Self::new();
        let c = p.counters;
        write!(
            line,
            "TXFAIL_HEALTH batch={} records={} captured={} overflow={} batches={} id_exhausted={} capture_saturated={} pending={} queue_records={} queue_health={} format_records={} format_health={} usb_records={} usb_health={} transport_saturated={}\r\n",
            Optional(p.batch.map(|b| b.id)),
            p.batch.map_or(0, |b| b.len()),
            c.captured,
            c.overflow,
            c.batches,
            c.id_exhausted,
            c.saturated,
            c.pending,
            t.queue_records,
            t.queue_health,
            t.format_records,
            t.format_health,
            t.usb_records,
            t.usb_health,
            t.saturated,
        )?;
        Ok(line)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Write for Line {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len.checked_add(text.len()).ok_or(fmt::Error)?;
        if end > LINE_CAPACITY || !text.is_ascii() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    const OBSERVATION_CASES: [(Option<Observations>, &str); 7] = [
        (
            None,
            "observations_present=false completion_origin=none provider_expiry_observed=none",
        ),
        (
            Some(Observations {
                completion: None,
                provider_expiry_observed: false,
                end_capture: None,
            }),
            "observations_present=true completion_origin=none provider_expiry_observed=false",
        ),
        (
            Some(Observations {
                completion: None,
                provider_expiry_observed: true,
                end_capture: None,
            }),
            "observations_present=true completion_origin=none provider_expiry_observed=true",
        ),
        (
            Some(Observations {
                completion: Some(CompletionOrigin::RadioIrq),
                provider_expiry_observed: false,
                end_capture: None,
            }),
            "observations_present=true completion_origin=RadioIrq provider_expiry_observed=false",
        ),
        (
            Some(Observations {
                completion: Some(CompletionOrigin::RadioIrq),
                provider_expiry_observed: true,
                end_capture: None,
            }),
            "observations_present=true completion_origin=RadioIrq provider_expiry_observed=true",
        ),
        (
            Some(Observations {
                completion: Some(CompletionOrigin::TaskService),
                provider_expiry_observed: false,
                end_capture: None,
            }),
            "observations_present=true completion_origin=TaskService provider_expiry_observed=false",
        ),
        (
            Some(Observations {
                completion: Some(CompletionOrigin::TaskService),
                provider_expiry_observed: true,
                end_capture: None,
            }),
            "observations_present=true completion_origin=TaskService provider_expiry_observed=true",
        ),
    ];

    fn epochs(ticket: u64, report: u64) -> (u64, u64, bool) {
        (ticket, report, ticket == report)
    }

    const END_REASONS: [(EndCaptureReason, &str); 7] = [
        (EndCaptureReason::Missing, "Missing"),
        (EndCaptureReason::Unstable, "Unstable"),
        (
            EndCaptureReason::NoCompletionEvidence,
            "NoCompletionEvidence",
        ),
        (EndCaptureReason::OutsideEpochRange, "OutsideEpochRange"),
        (
            EndCaptureReason::ObservationOutsideHorizon,
            "ObservationOutsideHorizon",
        ),
        (EndCaptureReason::NotArmed, "NotArmed"),
        (EndCaptureReason::Candidate, "Candidate"),
    ];

    fn end_capture(reason: EndCaptureReason) -> EndCapture {
        EndCapture {
            epoch: 2,
            epoch_match: false,
            anchor_raw: 10,
            cutoff_raw: 20,
            sampled_cc: [30, 31],
            observation_raw: 40,
            elapsed_us: 50,
            horizon_us: 60,
            reason,
            cutoff_delta_us: Some(-7),
        }
    }

    fn record(epoch: u64) -> Record {
        Record::failure(
            &[],
            epochs(epoch, epoch),
            100,
            None,
            Some(200),
            (Terminal::TimedOut, None),
        )
        .unwrap()
    }

    #[test]
    fn filtering_envelope_and_deadline_are_independent() {
        assert!(
            Record::failure(
                &[],
                epochs(1, 1),
                0,
                None,
                None,
                (Terminal::PhysicalComplete, None),
            )
            .is_none()
        );
        let bytes = [5, 250, 255, 254, 0, 0, 0, 1, 0, 99];
        let r = Record::failure(
            &bytes,
            epochs(1, 2),
            u64::MAX,
            None,
            None,
            (Terminal::Pending, None),
        )
        .unwrap();
        assert_eq!(
            r.envelope.unwrap(),
            Envelope {
                packet_type: 250,
                source: 255,
                seq: 254,
                payload_len: 1,
            }
        );
        assert!(!r.epoch_match);
        assert_eq!(r.effective_deadline_us, None);
        assert_eq!(r.last_poll_sample_us, None);
        assert_eq!(record(0).effective_deadline_us, Some(1550));
        let r = Record::failure(
            &bytes[..9],
            epochs(1, 1),
            u64::MAX,
            Some(42),
            None,
            (Terminal::Deadline, None),
        )
        .unwrap();
        assert_eq!(r.envelope, Err(mesh::Error::Length));
        assert_eq!(r.effective_deadline_us, Some(42));
        let mut bad = bytes;
        bad[0] = 4;
        assert_eq!(
            Record::failure(&bad, epochs(0, 0), 0, None, None, (Terminal::Busy, None))
                .unwrap()
                .envelope,
            Err(mesh::Error::Version)
        );
        let mut b = Buffer::new();
        let mut success = record(1);
        success.terminal = Terminal::PhysicalComplete;
        b.capture(success);
        assert_eq!(b.drain().counters, Counters::default());
        let r = Record::failure(
            &[],
            epochs(1, 1),
            u64::MAX - 1450,
            None,
            None,
            (Terminal::TimedOut, None),
        )
        .unwrap();
        assert_eq!(r.effective_deadline_us, Some(u64::MAX));
    }

    #[test]
    fn report_observations_survive_capture_and_render_explicitly() {
        let mut buffer = Buffer::new();
        for (observations, _) in OBSERVATION_CASES {
            let r = Record::failure(
                &[],
                epochs(1, 1),
                100,
                None,
                Some(200),
                (Terminal::TimedOut, observations),
            )
            .unwrap();
            assert_eq!(r.observations, observations);
            buffer.capture(r);
            assert!(
                Record::failure(
                    &[],
                    epochs(1, 1),
                    100,
                    None,
                    Some(200),
                    (Terminal::PhysicalComplete, observations),
                )
                .is_none()
            );
        }
        let publication = buffer.drain();
        assert_eq!(publication.counters.captured, 7);
        assert_eq!(publication.counters.overflow, 0);
        let batch = publication.batch.unwrap();
        assert_eq!(batch.len(), OBSERVATION_CASES.len());
        for (r, (observations, expected)) in batch.records().zip(OBSERVATION_CASES) {
            assert_eq!(r.observations, observations);
            let line = Line::record(batch.id, *r).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains(expected));
            assert!(text.contains(
                "end_present=false end_d=none end_r=none end_e=none end_m=none end_a=none end_c=none end_s0=none end_s1=none end_o=none end_t=none end_h=none end_dt=none"
            ));
            assert!(text.ends_with("\r\n"));
            assert!(line.as_bytes().is_ascii());
            assert!(line.as_bytes().len() <= 896);
        }
        let health = Line::health(publication, TransportCounters::new()).unwrap();
        let text = core::str::from_utf8(health.as_bytes()).unwrap();
        assert!(text.contains("batch=0 records=7 captured=7 overflow=0 batches=1"));
        assert!(!text.contains("observations_present="));
    }

    #[test]
    fn all_end_reasons_preserve_pure_data_and_render_observational_fields() {
        for (reason, name) in END_REASONS {
            for delta in [None, Some(i32::MIN), Some(-7), Some(0), Some(i32::MAX)] {
                let mut capture = end_capture(reason);
                capture.cutoff_delta_us = delta;
                let observations = Observations {
                    completion: Some(CompletionOrigin::RadioIrq),
                    provider_expiry_observed: false,
                    end_capture: Some(capture),
                };
                let r = Record::failure(
                    &[],
                    epochs(1, 1),
                    100,
                    None,
                    Some(200),
                    (Terminal::TimedOut, Some(observations)),
                )
                .unwrap();
                let mut buffer = Buffer::new();
                buffer.capture(r);
                let publication = buffer.drain();
                let retained = *publication.batch.unwrap().records().next().unwrap();
                assert_eq!(retained, r);
                assert_eq!(retained.observations, Some(observations));
                let line = Line::record(0, retained).unwrap();
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert!(text.contains("ticket_epoch=1 report_epoch=1 epoch_match=true"));
                assert!(text.contains("end_present=true end_d=TIMER4raw end_r="));
                assert!(text.contains(&std::format!("end_r={name} end_e=2 end_m=false")));
                assert!(
                    text.contains(
                        "end_a=10 end_c=20 end_s0=30 end_s1=31 end_o=40 end_t=50 end_h=60"
                    )
                );
                assert!(text.ends_with(&std::format!("end_dt={}\r\n", Optional(delta))));
                assert!(text.contains("terminal=Fault(TimedOut) sample=waiter_eval"));
                assert!(!text.contains("RF"));
                assert!(!text.contains("latency"));
                assert!(line.as_bytes().is_ascii());
                assert!(line.as_bytes().len() <= 896);
            }
        }
    }

    #[test]
    fn capture_epoch_match_is_supplied_independently_of_record_epoch_match() {
        let mut r = record(1);
        for matched in [false, true] {
            // Keep identical numeric mirrors: formatting must preserve the direct
            // vendor comparison flag, rather than derive it from those mirrors.
            let mut capture = end_capture(EndCaptureReason::Candidate);
            capture.epoch = 1;
            capture.epoch_match = matched;
            r.observations = Some(Observations {
                completion: None,
                provider_expiry_observed: false,
                end_capture: Some(capture),
            });
            let line = Line::record(0, r).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("report_epoch=1 epoch_match=true"));
            assert!(text.contains(&std::format!("end_r=Candidate end_e=1 end_m={matched}")));
        }
    }

    #[test]
    fn maximum_end_capture_ascii_rows_fit_unchanged_line_policy() {
        assert_eq!(LINE_CAPACITY, 896);
        let mut r = record(u64::MAX);
        r.report_epoch = u64::MAX - 1;
        r.epoch_match = false;
        r.wire_len = usize::MAX;
        r.admission_us = u64::MAX;
        r.effective_deadline_us = Some(u64::MAX);
        r.last_poll_sample_us = Some(u64::MAX);
        r.envelope = Ok(Envelope {
            packet_type: u8::MAX,
            source: u8::MAX,
            seq: u8::MAX,
            payload_len: u16::MAX,
        });
        r.terminal = Terminal::ReceiverUnavailable;
        for (reason, _) in END_REASONS {
            for matched in [false, true] {
                for delta in [None, Some(i32::MIN), Some(i32::MAX)] {
                    let capture = EndCapture {
                        epoch: u64::MAX,
                        epoch_match: matched,
                        anchor_raw: u32::MAX,
                        cutoff_raw: u32::MAX,
                        sampled_cc: [u32::MAX; 2],
                        observation_raw: u32::MAX,
                        elapsed_us: u64::MAX,
                        horizon_us: u32::MAX,
                        reason,
                        cutoff_delta_us: delta,
                    };
                    r.observations = Some(Observations {
                        completion: Some(CompletionOrigin::TaskService),
                        provider_expiry_observed: false,
                        end_capture: Some(capture),
                    });
                    let line = Line::record(u64::MAX, r).unwrap();
                    assert!(line.as_bytes().is_ascii());
                    assert!(line.as_bytes().len() <= 896);
                    let text = core::str::from_utf8(line.as_bytes()).unwrap();
                    assert!(text.contains("deadline_absent=false"));
                    assert!(text.contains("poll_absent=false"));
                    assert!(text.contains("observations_present=true"));
                    assert!(text.contains("end_present=true end_d=TIMER4raw"));
                    assert!(text.contains("end_e=18446744073709551615"));
                    assert!(text.contains("end_s0=4294967295 end_s1=4294967295"));
                    assert!(text.ends_with(&std::format!("end_dt={}\r\n", Optional(delta))));
                }
            }
        }
    }

    #[test]
    fn fifo_drop_new_exhaustion_retention_and_saturation() {
        let mut b = Buffer::new();
        b.next_batch_id = Some(u64::MAX);
        for epoch in 0..10 {
            b.capture(record(epoch));
        }
        let p = b.drain();
        assert_eq!(p.counters.overflow, 2);
        let batch = p.batch.unwrap();
        assert_eq!(batch.id, u64::MAX);
        assert!(
            batch
                .records()
                .enumerate()
                .all(|(i, r)| r.ticket_epoch == i as u64)
        );
        b.capture(record(10));
        b.counters.id_exhausted = u64::MAX;
        let p = b.drain();
        assert!(p.batch.is_none());
        assert_eq!(p.counters.pending, 1);
        assert!(p.counters.saturated);
        assert_eq!(b.records[0].unwrap().ticket_epoch, 10);
        assert!(b.drain().batch.is_none());
        b.counters.captured = u64::MAX;
        b.capture(record(11));
        b.counters.overflow = u64::MAX;
        for _ in 0..8 {
            b.capture(record(12));
        }
        assert_eq!(b.counters.captured, u64::MAX);
        assert_eq!(b.counters.overflow, u64::MAX);
        assert!(b.counters.saturated);
    }

    #[test]
    fn failure_only_partial_batch_loss_counts_suffix_and_health() {
        // Pure accounting model only: this does not execute the firmware writer's
        // queue/format/USB await paths (the ARM binary has test = false).
        let mut b = Buffer::new();
        for epoch in 0..8 {
            b.capture(record(epoch));
        }
        let publication = b.drain();
        let batch = publication.batch.unwrap();
        let mut t = TransportCounters::new();
        t.lose(Loss::Queue, batch.len(), true);
        t.lose(Loss::Usb, batch.len() - 5, false); // five complete rows, current row possibly partial
        t.lose(Loss::Format, 0, true); // empty health-only publication
        assert_eq!((t.queue_records, t.queue_health), (8, 1));
        assert_eq!((t.usb_records, t.usb_health), (3, 0));
        assert_eq!((t.format_records, t.format_health), (0, 1));
        // A later empty tick still publishes the loss of a fully dropped batch.
        let health = Line::health(b.drain(), t).unwrap();
        let text = core::str::from_utf8(health.as_bytes()).unwrap();
        assert!(text.contains("batch=none records=0 captured=8"));
        assert!(text.contains("queue_records=8 queue_health=1"));
        assert!(text.contains("usb_records=3 usb_health=0"));
        t.usb_records = u64::MAX;
        t.lose(Loss::Usb, 1, false);
        assert_eq!(t.usb_records, u64::MAX);
        assert!(t.saturated);
    }

    #[test]
    fn maximum_ascii_rows_and_absent_flags_fit_line_policy() {
        let mut r = record(u64::MAX);
        let matched = Line::record(u64::MAX, r).unwrap();
        let text = core::str::from_utf8(matched.as_bytes()).unwrap();
        assert!(text.contains("ticket_epoch=18446744073709551615"));
        assert!(text.contains("report_epoch=18446744073709551615 epoch_match=true"));
        r.ticket_epoch = u64::MAX;
        r.report_epoch = u64::MAX - 1;
        r.epoch_match = false;
        r.wire_len = usize::MAX;
        r.admission_us = u64::MAX;
        r.effective_deadline_us = Some(u64::MAX);
        r.last_poll_sample_us = Some(u64::MAX);
        r.envelope = Ok(Envelope {
            packet_type: 255,
            source: 255,
            seq: 255,
            payload_len: u16::MAX,
        });
        r.terminal = Terminal::ReceiverUnavailable;
        for absent in [false, true] {
            if absent {
                r.effective_deadline_us = None;
                r.last_poll_sample_us = None;
                r.envelope = Err(mesh::Error::Capacity);
            }
            for (observations, expected) in OBSERVATION_CASES {
                r.observations = observations;
                let line = Line::record(u64::MAX, r).unwrap();
                assert!(line.as_bytes().is_ascii());
                assert!(line.as_bytes().len() <= 896);
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert!(text.contains("ticket_epoch=18446744073709551615"));
                assert!(text.contains("report_epoch=18446744073709551614 epoch_match=false"));
                assert!(text.contains("terminal=Fault(ReceiverUnavailable)"));
                assert!(text.contains(expected));
                assert!(text.contains(if absent {
                    "deadline_absent=true"
                } else {
                    "deadline_absent=false"
                }));
                assert!(text.contains(if absent {
                    "poll_absent=true"
                } else {
                    "poll_absent=false"
                }));
            }
        }
        let p = Publication {
            batch: Some(Batch {
                id: u64::MAX,
                records: [Some(r); RECORD_CAPACITY],
                len: RECORD_CAPACITY,
            }),
            counters: Counters {
                captured: u64::MAX,
                overflow: u64::MAX,
                batches: u64::MAX,
                id_exhausted: u64::MAX,
                saturated: true,
                pending: 8,
            },
        };
        let t = TransportCounters {
            queue_records: u64::MAX,
            queue_health: u64::MAX,
            format_records: u64::MAX,
            format_health: u64::MAX,
            usb_records: u64::MAX,
            usb_health: u64::MAX,
            saturated: true,
        };
        let health = Line::health(p, t).unwrap();
        assert!(health.as_bytes().is_ascii());
        assert!(health.as_bytes().len() <= 896);
    }
}
