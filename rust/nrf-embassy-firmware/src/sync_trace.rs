//! Passive software observations only. Times are software bounds, never radio
//! START/END timestamps. No tracing result may change a radio decision.

use core::fmt::{self, Write};
use openhelmet_wire::mesh::{self, Identity};

pub const RECORD_CAPACITY: usize = 8;
pub const LINE_CAPACITY: usize = 896;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockSample {
    pub frame: u32,
    /// TDMA timing-domain units; their scale is determined by timing_profile.
    pub phase_us: u16,
    pub generation: u32,
    /// Signed TDMA timing-domain anchor, in the same units as phase_us.
    pub frame_start_us: i64,
    /// Exact physical Clock capture paired with the actual core clock read.
    /// This is never reconstructed by inverting the timing-domain mapping.
    pub physical_sample_us: i64,
    pub timing_profile: crate::timing_clock::ClockProfile,
}

impl ClockSample {
    /// Derived TDMA timing-domain timestamp, with scale set by timing_profile.
    /// Signed anchors are valid; overflow never substitutes/saturates time.
    /// In particular, Legacy values are not physical microseconds. Subtract
    /// physical_sample_us (not this value) from physical submit/terminal times.
    pub fn exact_sample_us(self) -> Option<i64> {
        self.frame_start_us.checked_add(i64::from(self.phase_us))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TraceOrigin {
    Local,
    Forwarded,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WireSync {
    pub src_id: u8,
    pub seq: u8,
    pub term: u32,
    pub frame: u32,
    pub phase_us: u16,
    pub relay_depth: u8,
    pub flags: u8,
    pub ttl: u8,
    pub origin: TraceOrigin,
}

/// Unknown/inconsistent origin metadata survives as Unknown for observation.
/// Non-SYNC envelopes return None; malformed envelopes/payloads return an error.
pub fn parse_sync(bytes: &[u8], local_identity: Identity) -> Result<Option<WireSync>, mesh::Error> {
    let envelope = mesh::decode(bytes, 200)?;
    if envelope.header.packet_type != 17 {
        return Ok(None);
    }
    let sync = mesh::SyncV3::decode(envelope.payload)?;
    let header = envelope.header;
    let identity_len = usize::from(sync.leader.address_len);
    let valid_identity = (1..=6).contains(&identity_len)
        && sync.leader.address[identity_len..]
            .iter()
            .all(|&byte| byte == 0)
        && !sync.leader.address[..identity_len]
            .iter()
            .all(|&byte| byte == 0)
        && !sync.leader.address[..identity_len]
            .iter()
            .all(|&byte| byte == 255);
    let consistent_identity = valid_identity && sync.leader_id == header.src_id;
    let origin = match (header.flags, header.ttl, sync.relay_depth) {
        (2, 1, 1) if consistent_identity => TraceOrigin::Forwarded,
        (0, 2, 0) if consistent_identity && sync.leader == local_identity => TraceOrigin::Local,
        _ => TraceOrigin::Unknown,
    };
    Ok(Some(WireSync {
        src_id: header.src_id,
        seq: header.seq,
        term: sync.term,
        frame: sync.frame_counter,
        phase_us: sync.phase_us,
        relay_depth: sync.relay_depth,
        flags: header.flags,
        ttl: header.ttl,
        origin,
    }))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SendResult {
    BackendWindowRejected,
    ValidationRejected,
    DeadlineRejected,
    SubmissionRejected,
    PhysicalComplete,
    TimedOut,
    OtherFault,
    UnexpectedPending,
}

/// Populate before begin, then finish after await returns and capture the record.
/// Result and ticket acceptance are independent: a DeadlineRejected observation
/// can occur before a ticket or as an accepted ticket's terminal outcome.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SendRecord {
    pub id: u64,
    pub wire: WireSync,
    pub clock_sample: Option<ClockSample>,
    pub submit_us: Option<i64>,
    pub terminal_return_us: Option<i64>,
    pub result: SendResult,
    pub accepted_ticket: bool,
    pub epoch_match: Option<bool>,
}

impl SendRecord {
    /// A failed observational clock read does not invalidate the ticket outcome.
    pub fn finish_terminal(
        &mut self,
        result: SendResult,
        epoch_match: bool,
        terminal_return_us: Option<i64>,
    ) {
        self.result = result;
        self.epoch_match = Some(epoch_match);
        self.terminal_return_us = terminal_return_us;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriterSource {
    Diagnostic,
    Trace,
}

/// Alternates after each consumed input; unavailable inputs never block a ready
/// peer. The callback must consume only when returning Some, and may register a
/// waiter when returning None. Stop at the first ready input to retain its peer.
pub struct WriterPreference {
    next: WriterSource,
}

impl Default for WriterPreference {
    fn default() -> Self {
        Self::new()
    }
}

impl WriterPreference {
    pub const fn new() -> Self {
        Self {
            next: WriterSource::Diagnostic,
        }
    }

    pub fn choose<T>(&mut self, mut ready: impl FnMut(WriterSource) -> Option<T>) -> Option<T> {
        let order = match self.next {
            WriterSource::Diagnostic => [WriterSource::Diagnostic, WriterSource::Trace],
            WriterSource::Trace => [WriterSource::Trace, WriterSource::Diagnostic],
        };
        for source in order {
            if let Some(input) = ready(source) {
                self.next = match source {
                    WriterSource::Diagnostic => WriterSource::Trace,
                    WriterSource::Trace => WriterSource::Diagnostic,
                };
                return Some(input);
            }
        }
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CounterFailure {
    SendIdExhausted,
    BatchIdExhausted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObserveError {
    Wire(mesh::Error),
    Counter(CounterFailure),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TrackerCounters {
    pub id_exhausted: u64,
    /// Parse failures across ALL send envelopes, not only malformed SYNC packets.
    pub send_boundary_parse_errors: u64,
    /// Tracker accounting lost precision; independent of buffer accounting.
    pub counter_saturated: bool,
}

pub struct Tracker {
    sample: Option<ClockSample>,
    next_send_id: Option<u64>,
    counters: TrackerCounters,
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

impl Tracker {
    pub const fn new() -> Self {
        Self {
            sample: None,
            next_send_id: Some(0),
            counters: TrackerCounters {
                id_exhausted: 0,
                send_boundary_parse_errors: 0,
                counter_saturated: false,
            },
        }
    }

    pub fn counters(&self) -> TrackerCounters {
        self.counters
    }

    /// Call on every snapshot attempt, including None on failure.
    pub fn clock_snapshot(&mut self, sample: Option<ClockSample>) {
        self.sample = sample
            .filter(|sample| sample.physical_sample_us >= 0 && sample.exact_sample_us().is_some());
    }

    /// Call at EVERY send boundary, before any backend rejection. The optional
    /// explicit generation is an additional check, not a replacement for current.
    /// None means this was a non-SYNC send; errors still consume the snapshot.
    pub fn send_boundary(
        &mut self,
        bytes: &[u8],
        local_identity: Identity,
        current_generation: u32,
        explicit_generation: Option<u32>,
    ) -> Result<Option<SendRecord>, ObserveError> {
        let sample = self.sample.take();
        let parsed = parse_sync(bytes, local_identity).map_err(|error| {
            increment(
                &mut self.counters.send_boundary_parse_errors,
                &mut self.counters.counter_saturated,
            );
            ObserveError::Wire(error)
        })?;
        let Some(wire) = parsed else {
            return Ok(None);
        };
        let Some(id) = self.next_send_id else {
            increment(
                &mut self.counters.id_exhausted,
                &mut self.counters.counter_saturated,
            );
            return Err(ObserveError::Counter(CounterFailure::SendIdExhausted));
        };
        self.next_send_id = id.checked_add(1);
        let clock_sample = sample.filter(|sample| {
            wire.origin == TraceOrigin::Local
                && sample.frame == wire.frame
                && sample.phase_us == wire.phase_us
                && sample.generation == current_generation
                && explicit_generation.is_none_or(|generation| generation == sample.generation)
        });
        Ok(Some(SendRecord {
            id,
            wire,
            clock_sample,
            submit_us: None,
            terminal_return_us: None,
            result: SendResult::UnexpectedPending,
            accepted_ticket: false,
            epoch_match: None,
        }))
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TraceCounters {
    /// Successfully stored records, cumulative across drains.
    pub captured: u64,
    /// New records discarded because all eight slots were occupied.
    pub overflow: u64,
    pub high_water: usize,
    pub batches: u64,
    pub batch_id_exhausted: u64,
    /// Accounting saturates rather than wrapping; this marks lost precision.
    pub counter_saturated: bool,
}

fn increment(value: &mut u64, saturated: &mut bool) {
    if let Some(next) = value.checked_add(1) {
        *value = next;
    } else {
        *saturated = true;
    }
}

/// Transport loss is separate from capture loss. Integration owns these totals.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransportCounters {
    pub queue_rejected: u64,
    pub coalesced: u64,
    pub format_failed: u64,
    pub usb_interrupted: u64,
    /// Transport totals lost precision; independent of tracker/capture counters.
    pub counter_saturated: bool,
}

impl TransportCounters {
    /// Each event counts an undelivered or possibly partial publication (batch
    /// or counter-only health report).
    pub fn reject_queue(&mut self) {
        increment(&mut self.queue_rejected, &mut self.counter_saturated);
    }

    pub fn fail_format(&mut self) {
        increment(&mut self.format_failed, &mut self.counter_saturated);
    }

    pub fn interrupt_usb(&mut self) {
        increment(&mut self.usb_interrupted, &mut self.counter_saturated);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TraceBatch {
    pub id: u64,
    pub counters: TraceCounters,
    records: [Option<SendRecord>; RECORD_CAPACITY],
    len: usize,
}

impl TraceBatch {
    pub fn records(&self) -> impl Iterator<Item = &SendRecord> {
        self.records[..self.len].iter().flatten()
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

pub struct TraceBuffer {
    records: [Option<SendRecord>; RECORD_CAPACITY],
    len: usize,
    next_batch_id: Option<u64>,
    counters: TraceCounters,
}

impl Default for TraceBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TraceBuffer {
    pub const fn new() -> Self {
        Self {
            records: [None; RECORD_CAPACITY],
            len: 0,
            next_batch_id: Some(0),
            counters: TraceCounters {
                captured: 0,
                overflow: 0,
                high_water: 0,
                batches: 0,
                batch_id_exhausted: 0,
                counter_saturated: false,
            },
        }
    }

    /// Drop-new policy. False reports an explicit overflow, without mutation of
    /// any retained record or radio state.
    pub fn capture(&mut self, record: SendRecord) -> bool {
        if self.len == RECORD_CAPACITY {
            increment(
                &mut self.counters.overflow,
                &mut self.counters.counter_saturated,
            );
            return false;
        }
        self.records[self.len] = Some(record);
        self.len += 1;
        self.counters.high_water = self.counters.high_water.max(self.len);
        increment(
            &mut self.counters.captured,
            &mut self.counters.counter_saturated,
        );
        true
    }

    pub fn counters(&self) -> TraceCounters {
        self.counters
    }

    /// An exhausted batch ID leaves pending records intact.
    pub fn drain(&mut self) -> Result<TraceBatch, CounterFailure> {
        let Some(id) = self.next_batch_id else {
            increment(
                &mut self.counters.batch_id_exhausted,
                &mut self.counters.counter_saturated,
            );
            return Err(CounterFailure::BatchIdExhausted);
        };
        self.next_batch_id = id.checked_add(1);
        increment(
            &mut self.counters.batches,
            &mut self.counters.counter_saturated,
        );
        Ok(TraceBatch {
            id,
            counters: self.counters,
            records: core::mem::replace(&mut self.records, [None; RECORD_CAPACITY]),
            len: core::mem::replace(&mut self.len, 0),
        })
    }
}

struct Optional<T>(Option<T>);

fn profile_name(profile: crate::timing_clock::ClockProfile) -> &'static str {
    match profile {
        crate::timing_clock::ClockProfile::Normalized => "normalized",
        crate::timing_clock::ClockProfile::LegacyNrfRtc10000 => "legacy_rtc10000",
    }
}

impl<T: fmt::Display> fmt::Display for Optional<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(value) => value.fmt(f),
            None => f.write_str("none"),
        }
    }
}

/// One owned ASCII row, bounded identically to DiagnosticLine. Use as_bytes()
/// with the existing USB queue; no formatter allocation or USB side effect.
pub struct TraceLine {
    bytes: [u8; LINE_CAPACITY],
    len: usize,
}

impl TraceLine {
    fn new() -> Self {
        Self {
            bytes: [0; LINE_CAPACITY],
            len: 0,
        }
    }

    pub fn header(
        batch: &TraceBatch,
        tracker: TrackerCounters,
        transport: TransportCounters,
    ) -> Result<Self, fmt::Error> {
        let mut line = Self::new();
        let c = batch.counters;
        writeln!(
            line,
            "sync_trace batch_id={} records={} captured={} overflow={} high_water={} batches={} send_id_exhausted={} send_boundary_parse_errors={} tracker_counter_saturated={} batch_id_exhausted={} counter_saturated={} queue_rejected={} coalesced={} format_failed={} usb_interrupted={} transport_counter_saturated={} timing=software_bounds RFqualified=false",
            batch.id,
            batch.len,
            c.captured,
            c.overflow,
            c.high_water,
            c.batches,
            tracker.id_exhausted,
            tracker.send_boundary_parse_errors,
            tracker.counter_saturated,
            c.batch_id_exhausted,
            c.counter_saturated,
            transport.queue_rejected,
            transport.coalesced,
            transport.format_failed,
            transport.usb_interrupted,
            transport.counter_saturated,
        )?;
        Ok(line)
    }

    /// Counter-only publication after batch IDs exhaust. No batch ID or row is
    /// fabricated; the buffer still owns its pending records.
    pub fn health(
        trace: TraceCounters,
        tracker: TrackerCounters,
        transport: TransportCounters,
    ) -> Result<Self, fmt::Error> {
        let mut line = Self::new();
        writeln!(
            line,
            "sync_trace health=true captured={} overflow={} high_water={} batches={} send_id_exhausted={} send_boundary_parse_errors={} tracker_counter_saturated={} batch_id_exhausted={} counter_saturated={} queue_rejected={} coalesced={} format_failed={} usb_interrupted={} transport_counter_saturated={} timing=software_bounds RFqualified=false",
            trace.captured,
            trace.overflow,
            trace.high_water,
            trace.batches,
            tracker.id_exhausted,
            tracker.send_boundary_parse_errors,
            tracker.counter_saturated,
            trace.batch_id_exhausted,
            trace.counter_saturated,
            transport.queue_rejected,
            transport.coalesced,
            transport.format_failed,
            transport.usb_interrupted,
            transport.counter_saturated,
        )?;
        Ok(line)
    }

    pub fn record(batch_id: u64, record: SendRecord) -> Result<Self, fmt::Error> {
        let mut line = Self::new();
        let w = record.wire;
        let s = record.clock_sample;
        writeln!(
            line,
            "sync_trace batch_id={} send_id={} src_id={} seq={} term={} wire_frame={} wire_phase_us={} relay_depth={} flags={} ttl={} origin={:?} sample_frame={} sample_phase_us={} sample_generation={} sample_frame_start_us={} exact_sample_us={} submit_us={} terminal_return_us={} result={:?} accepted_ticket={} epoch_match={} timing=software_bounds RFqualified=false sample_domain=tdma timing_profile={} physical_sample_us={}",
            batch_id,
            record.id,
            w.src_id,
            w.seq,
            w.term,
            w.frame,
            w.phase_us,
            w.relay_depth,
            w.flags,
            w.ttl,
            w.origin,
            Optional(s.map(|s| s.frame)),
            Optional(s.map(|s| s.phase_us)),
            Optional(s.map(|s| s.generation)),
            Optional(s.map(|s| s.frame_start_us)),
            Optional(s.and_then(ClockSample::exact_sample_us)),
            Optional(record.submit_us),
            Optional(record.terminal_return_us),
            record.result,
            record.accepted_ticket,
            Optional(record.epoch_match),
            Optional(s.map(|s| profile_name(s.timing_profile))),
            Optional(s.map(|s| s.physical_sample_us)),
        )?;
        Ok(line)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Write for TraceLine {
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
    use super::*;

    const LOCAL: Identity = Identity {
        address_len: 5,
        address: [1, 2, 3, 4, 5, 0],
    };
    const SAMPLE: ClockSample = ClockSample {
        frame: u32::MAX,
        phase_us: u16::MAX,
        generation: u32::MAX,
        frame_start_us: 123,
        physical_sample_us: 65_658,
        timing_profile: crate::timing_clock::ClockProfile::Normalized,
    };

    fn packet(flags: u8, ttl: u8, depth: u8, leader: Identity) -> [u8; 29] {
        let sync = mesh::SyncV3 {
            leader,
            leader_id: 255,
            member_count: 8,
            term: u32::MAX,
            frame_counter: SAMPLE.frame,
            phase_us: SAMPLE.phase_us,
            relay_depth: depth,
        };
        let mut payload = [0; mesh::SyncV3::SIZE];
        sync.encode(&mut payload).unwrap();
        let mut bytes = [0; 29];
        mesh::encode(
            &mesh::Header {
                version: mesh::VERSION,
                packet_type: 17,
                src_id: 255,
                seq: 255,
                ttl,
                flags,
                talk_channel: 0,
                payload_len: mesh::SyncV3::SIZE as u16,
            },
            &payload,
            &mut bytes,
            200,
        )
        .unwrap();
        bytes
    }

    fn record() -> SendRecord {
        let mut tracker = Tracker::new();
        tracker.clock_snapshot(Some(SAMPLE));
        tracker
            .send_boundary(&packet(0, 2, 0, LOCAL), LOCAL, u32::MAX, None)
            .unwrap()
            .unwrap()
    }

    #[test]
    fn exact_sample_bounds() {
        assert_eq!(SAMPLE.exact_sample_us(), Some(65_658));
        assert_eq!(
            ClockSample {
                frame_start_us: i64::MAX,
                ..SAMPLE
            }
            .exact_sample_us(),
            None
        );
        assert_eq!(
            ClockSample {
                frame_start_us: -65_536,
                ..SAMPLE
            }
            .exact_sample_us(),
            Some(-1)
        );
        let sample = ClockSample {
            frame_start_us: i64::MAX - 65_535,
            ..SAMPLE
        };
        assert_eq!(sample.exact_sample_us(), Some(i64::MAX));
    }

    #[test]
    fn legacy_samples_keep_captured_physical_time_and_signed_timing_anchors() {
        let sync = packet(0, 2, 0, LOCAL);
        for (timing_profile, frame_start_us, exact) in [
            (
                crate::timing_clock::ClockProfile::LegacyNrfRtc10000,
                134_465,
                200_000,
            ),
            (
                crate::timing_clock::ClockProfile::LegacyNrfRtc10000,
                -65_536,
                -1,
            ),
            (crate::timing_clock::ClockProfile::Normalized, -65_536, -1),
        ] {
            let mut sample = ClockSample {
                timing_profile,
                frame_start_us,
                physical_sample_us: 183_106,
                ..SAMPLE
            };
            let mut tracker = Tracker::new();
            tracker.clock_snapshot(Some(sample));
            let mut observed = observe(&mut tracker, &sync);
            let captured = sample;
            sample.physical_sample_us = 183_110;
            tracker.clock_snapshot(Some(sample));
            observed.submit_us = Some(183_116);
            observed.finish_terminal(SendResult::PhysicalComplete, true, Some(183_126));
            assert_eq!(observed.clock_sample, Some(captured));
            assert_eq!(captured.exact_sample_us(), Some(exact));
            assert_eq!(
                observed.submit_us.unwrap() - captured.physical_sample_us,
                10
            );
            let line = TraceLine::record(0, observed).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("sample_domain=tdma "));
            assert!(text.contains(profile_name(timing_profile)));
            assert!(text.contains("physical_sample_us=183106\n"));
            assert!(text.contains("submit_us=183116 terminal_return_us=183126 "));
            if exact == 200_000 {
                assert!(text.contains("sample_frame_start_us=134465 exact_sample_us=200000 "));
            } else {
                assert!(text.contains("sample_frame_start_us=-65536 exact_sample_us=-1 "));
            }
            assert_eq!(observe(&mut tracker, &sync).clock_sample, Some(sample));
            assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        }
    }

    #[test]
    fn negative_physical_capture_clears_pending_sample_without_allocating_ids() {
        let mut tracker = Tracker::new();
        tracker.clock_snapshot(Some(SAMPLE));
        tracker.clock_snapshot(Some(ClockSample {
            physical_sample_us: -1,
            ..SAMPLE
        }));
        let sync = packet(0, 2, 0, LOCAL);
        let first = observe(&mut tracker, &sync);
        assert_eq!(first.id, 0);
        assert_eq!(first.clock_sample, None);
        assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        tracker.clock_snapshot(Some(SAMPLE));
        let fresh = observe(&mut tracker, &sync);
        assert_eq!(fresh.id, 2);
        assert_eq!(fresh.clock_sample, Some(SAMPLE));
        assert_eq!(tracker.counters(), TrackerCounters::default());
    }

    #[test]
    fn overflowing_timing_sum_clears_pending_even_with_valid_physical_capture() {
        let mut tracker = Tracker::new();
        tracker.clock_snapshot(Some(SAMPLE));
        let overflow = ClockSample {
            frame_start_us: i64::MAX,
            physical_sample_us: i64::MAX,
            timing_profile: crate::timing_clock::ClockProfile::LegacyNrfRtc10000,
            ..SAMPLE
        };
        assert_eq!(overflow.exact_sample_us(), None);
        tracker.clock_snapshot(Some(overflow));
        let sync = packet(0, 2, 0, LOCAL);
        assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        let valid = ClockSample {
            frame_start_us: i64::MAX - i64::from(SAMPLE.phase_us),
            ..overflow
        };
        tracker.clock_snapshot(Some(valid));
        assert_eq!(observe(&mut tracker, &sync).clock_sample, Some(valid));
    }

    #[test]
    fn maximum_profile_and_physical_fields_fit_ascii_rows_and_missing_is_explicit() {
        for timing_profile in [
            crate::timing_clock::ClockProfile::Normalized,
            crate::timing_clock::ClockProfile::LegacyNrfRtc10000,
        ] {
            let sample = ClockSample {
                frame_start_us: i64::MAX - i64::from(SAMPLE.phase_us),
                physical_sample_us: i64::MAX,
                timing_profile,
                ..SAMPLE
            };
            for clock_sample in [None, Some(sample)] {
                let line = TraceLine::record(
                    u64::MAX,
                    SendRecord {
                        id: u64::MAX,
                        clock_sample,
                        submit_us: Some(i64::MIN),
                        terminal_return_us: Some(i64::MAX),
                        result: SendResult::BackendWindowRejected,
                        accepted_ticket: false,
                        epoch_match: Some(false),
                        wire: WireSync {
                            flags: u8::MAX,
                            ttl: u8::MAX,
                            relay_depth: u8::MAX,
                            origin: TraceOrigin::Forwarded,
                            ..record().wire
                        },
                    },
                )
                .unwrap();
                assert!(line.as_bytes().is_ascii());
                assert!(line.as_bytes().len() <= LINE_CAPACITY);
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert!(text.contains("sample_domain=tdma "));
                if clock_sample.is_some() {
                    assert!(text.contains(profile_name(timing_profile)));
                    assert!(text.contains("physical_sample_us=9223372036854775807\n"));
                } else {
                    assert!(text.contains("timing_profile=none physical_sample_us=none\n"));
                }
            }
        }
    }

    #[test]
    fn identity_and_relay_metadata_classify_without_src_shortcut() {
        assert_eq!(record().wire.origin, TraceOrigin::Local);
        let other = Identity {
            address: [9, 9, 9, 9, 9, 0],
            ..LOCAL
        };
        let invalid = Identity {
            address_len: 0,
            ..LOCAL
        };
        for (flags, ttl, depth, leader, expected) in [
            (2, 1, 1, LOCAL, TraceOrigin::Forwarded),
            (2, 1, 1, other, TraceOrigin::Forwarded),
            (0, 2, 0, other, TraceOrigin::Unknown),
            (0, 1, 0, LOCAL, TraceOrigin::Unknown),
            (2, 2, 1, LOCAL, TraceOrigin::Unknown),
            (0, 2, 1, LOCAL, TraceOrigin::Unknown),
            (3, 1, 1, LOCAL, TraceOrigin::Unknown),
            (0, 2, 0, invalid, TraceOrigin::Unknown),
        ] {
            let bytes = packet(flags, ttl, depth, leader);
            let wire = parse_sync(&bytes, LOCAL).unwrap().unwrap();
            assert_eq!(wire.origin, expected);
            let mut tracker = Tracker::new();
            tracker.clock_snapshot(Some(SAMPLE));
            let record = observe(&mut tracker, &bytes);
            assert_eq!(record.clock_sample, None);
        }
        let mut inconsistent = packet(0, 2, 0, LOCAL);
        inconsistent[2] = 1;
        assert_eq!(
            parse_sync(&inconsistent, LOCAL).unwrap().unwrap().origin,
            TraceOrigin::Unknown
        );
    }

    #[test]
    fn all_boundaries_consume_and_failed_snapshots_invalidate() {
        let sync = packet(0, 2, 0, LOCAL);
        let mut non_sync = sync;
        non_sync[1] = 1;
        let mut malformed = sync;
        malformed[8] = 1;
        let mut short_sync = sync;
        short_sync[7] -= 1;
        for bytes in [&non_sync[..], &malformed[..], &short_sync[..28], &[][..]] {
            let mut tracker = Tracker::new();
            tracker.clock_snapshot(Some(SAMPLE));
            let outcome = tracker.send_boundary(bytes, LOCAL, u32::MAX, None);
            if bytes == non_sync {
                assert_eq!(outcome, Ok(None));
            } else {
                assert!(matches!(outcome, Err(ObserveError::Wire(_))));
            }
            assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        }
        let mut tracker = Tracker::new();
        tracker.clock_snapshot(Some(SAMPLE));
        tracker.clock_snapshot(None);
        assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        tracker.clock_snapshot(Some(SAMPLE));
        let rejected = observe(&mut tracker, &sync);
        assert_eq!(rejected.clock_sample, Some(SAMPLE));
        assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
    }

    #[test]
    fn malformed_send_envelopes_are_counted_without_ids_or_reused_samples() {
        let sync = packet(0, 2, 0, LOCAL);
        for packet_type in [1, 17] {
            let mut malformed = sync;
            malformed[1] = packet_type;
            malformed[8] = 1; // Declared payload length exceeds the envelope.
            let error = parse_sync(&malformed, LOCAL).unwrap_err();
            let mut tracker = Tracker::new();
            tracker.clock_snapshot(Some(SAMPLE));
            assert_eq!(
                tracker.send_boundary(&malformed, LOCAL, u32::MAX, None),
                Err(ObserveError::Wire(error))
            );
            assert_eq!(tracker.counters().send_boundary_parse_errors, 1);
            assert_eq!(tracker.counters().id_exhausted, 0);
            assert!(!tracker.counters().counter_saturated);
            let valid = observe(&mut tracker, &sync);
            assert_eq!(valid.id, 0, "parse failure must not allocate a send ID");
            assert_eq!(valid.wire.origin, TraceOrigin::Local);
            assert_eq!(
                valid.clock_sample, None,
                "failed boundary consumed the sample"
            );
            assert_eq!(observe(&mut tracker, &sync).id, 1);
            let mut non_sync = sync;
            non_sync[1] = 1;
            assert_eq!(
                tracker.send_boundary(&non_sync, LOCAL, u32::MAX, None),
                Ok(None)
            );
            assert_eq!(tracker.counters().send_boundary_parse_errors, 1);
        }
    }

    #[test]
    fn send_boundary_parse_error_precision_loss_is_visible_without_wrap() {
        let mut tracker = Tracker::new();
        tracker.counters.send_boundary_parse_errors = u64::MAX - 1;
        assert!(matches!(
            tracker.send_boundary(&[], LOCAL, u32::MAX, None),
            Err(ObserveError::Wire(_))
        ));
        assert_eq!(tracker.counters().send_boundary_parse_errors, u64::MAX);
        assert!(!tracker.counters().counter_saturated);
        for _ in 0..2 {
            assert!(matches!(
                tracker.send_boundary(&[], LOCAL, u32::MAX, None),
                Err(ObserveError::Wire(_))
            ));
            assert_eq!(tracker.counters().send_boundary_parse_errors, u64::MAX);
            assert!(tracker.counters().counter_saturated);
        }
        let batch = TraceBuffer::new().drain().unwrap();
        let transport = TransportCounters::default();
        for line in [
            TraceLine::header(&batch, tracker.counters(), transport).unwrap(),
            TraceLine::health(batch.counters, tracker.counters(), transport).unwrap(),
        ] {
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("send_boundary_parse_errors=18446744073709551615 "));
            assert!(text.contains("tracker_counter_saturated=true "));
            assert!(text.contains(" counter_saturated=false "));
        }
        assert_eq!(observe(&mut tracker, &packet(0, 2, 0, LOCAL)).id, 0);
    }

    fn observe(tracker: &mut Tracker, bytes: &[u8]) -> SendRecord {
        tracker
            .send_boundary(bytes, LOCAL, u32::MAX, None)
            .unwrap()
            .unwrap()
    }

    #[test]
    fn frame_phase_and_both_generations_must_match_exactly() {
        let sync = packet(0, 2, 0, LOCAL);
        let wrong_frame = ClockSample { frame: 0, ..SAMPLE };
        let wrong_phase = ClockSample {
            phase_us: 0,
            ..SAMPLE
        };
        let wrong_generation = ClockSample {
            generation: 0,
            ..SAMPLE
        };
        let negative = ClockSample {
            physical_sample_us: -1,
            ..SAMPLE
        };
        for (sample, current, explicit) in [
            (wrong_frame, u32::MAX, None),
            (wrong_phase, u32::MAX, None),
            (wrong_generation, u32::MAX, None),
            (SAMPLE, 0, None),
            (SAMPLE, u32::MAX, Some(0)),
            (negative, u32::MAX, None),
        ] {
            let mut tracker = Tracker::new();
            tracker.clock_snapshot(Some(sample));
            let record = tracker
                .send_boundary(&sync, LOCAL, current, explicit)
                .unwrap()
                .unwrap();
            assert_eq!(record.clock_sample, None);
            assert_eq!(observe(&mut tracker, &sync).clock_sample, None);
        }
        let mut tracker = Tracker::new();
        tracker.clock_snapshot(Some(SAMPLE));
        let record = tracker
            .send_boundary(&sync, LOCAL, u32::MAX, Some(u32::MAX))
            .unwrap()
            .unwrap();
        assert_eq!(record.clock_sample, Some(SAMPLE));
    }

    #[test]
    fn zero_after_wrap_matches_only_a_fresh_zero_sample() {
        let mut bytes = packet(0, 2, 0, LOCAL);
        let mut sync = mesh::SyncV3::decode(&bytes[mesh::HEADER_SIZE..]).unwrap();
        sync.frame_counter = 0;
        sync.phase_us = 0;
        sync.encode(&mut bytes[mesh::HEADER_SIZE..]).unwrap();
        let mut tracker = Tracker::default();
        tracker.clock_snapshot(Some(SAMPLE));
        let old = tracker
            .send_boundary(&bytes, LOCAL, 0, None)
            .unwrap()
            .unwrap();
        assert_eq!(old.clock_sample, None);
        let fresh = ClockSample {
            frame: 0,
            phase_us: 0,
            generation: 0,
            frame_start_us: 65_658,
            physical_sample_us: 65_658,
            timing_profile: crate::timing_clock::ClockProfile::Normalized,
        };
        tracker.clock_snapshot(Some(fresh));
        let record = tracker
            .send_boundary(&bytes, LOCAL, 0, Some(0))
            .unwrap()
            .unwrap();
        assert_eq!(record.clock_sample, Some(fresh));
        assert_eq!(record.id, 1);
    }

    #[test]
    fn drop_new_drains_preserve_order_and_cumulative_totals() {
        let mut buffer = TraceBuffer::new();
        for id in 0..10 {
            assert_eq!(buffer.capture(SendRecord { id, ..record() }), id < 8);
        }
        let batch = buffer.drain().unwrap();
        assert_eq!(batch.id, 0);
        assert_eq!(batch.len(), 8);
        for (expected, record) in (0..8).zip(batch.records()) {
            assert_eq!(record.id, expected);
        }
        assert_eq!(batch.counters.captured, 8);
        assert_eq!(batch.counters.overflow, 2);
        assert_eq!(batch.counters.high_water, 8);
        let empty = buffer.drain().unwrap();
        assert_eq!(empty.id, 1);
        assert!(empty.is_empty());
        assert_eq!(empty.counters.captured, 8);
        assert!(buffer.capture(record()));
        assert_eq!(buffer.drain().unwrap().counters.captured, 9);
    }

    #[test]
    fn ids_exhaust_explicitly_without_wrapping_or_losing_pending_records() {
        let mut tracker = Tracker {
            next_send_id: Some(u64::MAX),
            ..Tracker::new()
        };
        let bytes = packet(0, 2, 0, LOCAL);
        assert_eq!(observe(&mut tracker, &bytes).id, u64::MAX);
        assert_eq!(
            tracker.send_boundary(&bytes, LOCAL, 0, None),
            Err(ObserveError::Counter(CounterFailure::SendIdExhausted))
        );
        assert_eq!(tracker.counters().id_exhausted, 1);
        let mut buffer = TraceBuffer {
            next_batch_id: Some(u64::MAX),
            ..TraceBuffer::new()
        };
        assert_eq!(buffer.drain().unwrap().id, u64::MAX);
        buffer.capture(record());
        assert_eq!(buffer.drain(), Err(CounterFailure::BatchIdExhausted));
        assert_eq!(buffer.len, 1);
        assert_eq!(buffer.counters().batch_id_exhausted, 1);
        buffer.counters.captured = u64::MAX;
        buffer.capture(record());
        assert!(buffer.counters().counter_saturated);
    }

    #[test]
    fn tracker_exhaustion_precision_loss_is_reported_independently() {
        let mut tracker = Tracker {
            next_send_id: None,
            counters: TrackerCounters {
                id_exhausted: u64::MAX,
                send_boundary_parse_errors: 0,
                counter_saturated: false,
            },
            ..Tracker::new()
        };
        let bytes = packet(0, 2, 0, LOCAL);
        let mut buffer = TraceBuffer::new();
        let batch = buffer.drain().unwrap();
        let transport = TransportCounters::default();
        let before = TraceLine::header(&batch, tracker.counters(), transport).unwrap();
        let before = core::str::from_utf8(before.as_bytes()).unwrap();
        assert!(before.contains("tracker_counter_saturated=false"));
        for _ in 0..2 {
            assert_eq!(
                tracker.send_boundary(&bytes, LOCAL, 0, None),
                Err(ObserveError::Counter(CounterFailure::SendIdExhausted))
            );
            assert_eq!(tracker.counters().id_exhausted, u64::MAX);
            assert!(tracker.counters().counter_saturated);
        }
        assert!(!batch.counters.counter_saturated);
        let header = TraceLine::header(&batch, tracker.counters(), transport).unwrap();
        let text = core::str::from_utf8(header.as_bytes()).unwrap();
        assert!(text.contains("send_id_exhausted=18446744073709551615 "));
        assert!(text.contains("tracker_counter_saturated=true "));
        assert!(text.contains(" counter_saturated=false "));

        for _ in 0..RECORD_CAPACITY {
            assert!(buffer.capture(record()));
        }
        buffer.counters.overflow = u64::MAX;
        for _ in 0..2 {
            assert!(!buffer.capture(record()));
            assert_eq!(buffer.counters().overflow, u64::MAX);
            assert!(buffer.counters().counter_saturated);
        }
    }

    #[test]
    fn maximum_ascii_rows_and_ticket_outcomes_remain_distinct() {
        let mut buffer = TraceBuffer::new();
        buffer.capture(record());
        let mut batch = buffer.drain().unwrap();
        batch.id = u64::MAX;
        batch.counters = TraceCounters {
            captured: u64::MAX,
            overflow: u64::MAX,
            high_water: usize::MAX,
            batches: u64::MAX,
            batch_id_exhausted: u64::MAX,
            counter_saturated: true,
        };
        let transport = TransportCounters {
            queue_rejected: u64::MAX,
            coalesced: u64::MAX,
            format_failed: u64::MAX,
            usb_interrupted: u64::MAX,
            counter_saturated: true,
        };
        let tracker = TrackerCounters {
            id_exhausted: u64::MAX,
            send_boundary_parse_errors: u64::MAX,
            counter_saturated: true,
        };
        let header = TraceLine::header(&batch, tracker, transport).unwrap();
        assert!(header.as_bytes().is_ascii());
        assert!(header.as_bytes().len() <= LINE_CAPACITY);
        assert!(
            core::str::from_utf8(header.as_bytes())
                .unwrap()
                .contains("send_boundary_parse_errors=18446744073709551615 ")
        );
        let health = TraceLine::health(batch.counters, tracker, transport).unwrap();
        assert!(health.as_bytes().is_ascii());
        assert!(health.as_bytes().len() <= LINE_CAPACITY);
        let health = core::str::from_utf8(health.as_bytes()).unwrap();
        assert!(health.contains("health=true "));
        assert!(health.contains("send_boundary_parse_errors=18446744073709551615 "));
        assert!(!health.contains("batch_id="));
        assert!(health.contains("batch_id_exhausted=18446744073709551615 "));
        assert!(health.contains("tracker_counter_saturated=true "));
        assert!(health.contains(" counter_saturated=true "));
        assert!(health.contains("transport_counter_saturated=true "));
        for result in [
            SendResult::BackendWindowRejected,
            SendResult::ValidationRejected,
            SendResult::DeadlineRejected,
            SendResult::SubmissionRejected,
            SendResult::PhysicalComplete,
            SendResult::TimedOut,
            SendResult::OtherFault,
            SendResult::UnexpectedPending,
        ] {
            let maximum_sample = ClockSample {
                frame_start_us: i64::MAX - 65_535,
                ..SAMPLE
            };
            for sample in [None, Some(maximum_sample)] {
                let before = SendRecord {
                    id: u64::MAX,
                    clock_sample: sample,
                    submit_us: Some(i64::MIN),
                    terminal_return_us: Some(i64::MAX),
                    result,
                    wire: WireSync {
                        flags: u8::MAX,
                        ttl: u8::MAX,
                        relay_depth: u8::MAX,
                        ..record().wire
                    },
                    ..record()
                };
                let accepted = SendRecord {
                    accepted_ticket: true,
                    epoch_match: Some(false),
                    ..before
                };
                let a = TraceLine::record(u64::MAX, before).unwrap();
                let b = TraceLine::record(u64::MAX, accepted).unwrap();
                assert_ne!(a.as_bytes(), b.as_bytes());
                for line in [a, b] {
                    assert!(line.as_bytes().len() <= LINE_CAPACITY);
                    assert!(line.as_bytes().is_ascii());
                }
            }
        }
        let none = TraceLine::record(
            0,
            SendRecord {
                submit_us: None,
                terminal_return_us: None,
                ..record()
            },
        )
        .unwrap();
        assert!(none.as_bytes().is_ascii());
        assert!(TraceLine::new().write_str("\u{e9}").is_err());
    }

    #[test]
    fn transport_loss_preserves_totals_and_reports_precision_loss() {
        let mut transport = TransportCounters {
            queue_rejected: u64::MAX - 1,
            format_failed: u64::MAX - 1,
            usb_interrupted: u64::MAX - 1,
            ..TransportCounters::default()
        };
        transport.reject_queue();
        transport.fail_format();
        transport.interrupt_usb();
        assert!(!transport.counter_saturated);
        for _ in 0..2 {
            transport.reject_queue();
            transport.fail_format();
            transport.interrupt_usb();
        }
        assert_eq!(transport.queue_rejected, u64::MAX);
        assert_eq!(transport.format_failed, u64::MAX);
        assert_eq!(transport.usb_interrupted, u64::MAX);
        assert_eq!(transport.coalesced, 0);
        assert!(transport.counter_saturated);
        let batch = TraceBuffer::new().drain().unwrap();
        let line = TraceLine::header(&batch, TrackerCounters::default(), transport).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("transport_counter_saturated=true "));
        assert!(text.contains("coalesced=0 "));
        assert!(text.contains("tracker_counter_saturated=false "));
        assert!(text.contains(" counter_saturated=false "));
    }

    #[test]
    fn missing_terminal_clock_preserves_accepted_outcome_and_renders_none() {
        for result in [
            SendResult::PhysicalComplete,
            SendResult::DeadlineRejected,
            SendResult::TimedOut,
            SendResult::OtherFault,
            SendResult::UnexpectedPending,
        ] {
            let mut observed = SendRecord {
                submit_us: Some(123),
                accepted_ticket: true,
                ..record()
            };
            let before = observed;
            let clock: Result<i64, crate::clock::ClockFault> =
                Err(crate::clock::ClockFault::SignedRange);
            observed.finish_terminal(result, false, clock.ok());
            assert_eq!(
                observed,
                SendRecord {
                    result,
                    epoch_match: Some(false),
                    terminal_return_us: None,
                    ..before
                }
            );
            let line = TraceLine::record(0, observed).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("terminal_return_us=none "));
            assert!(text.contains("accepted_ticket=true "));
            assert!(text.contains("epoch_match=false "));
            let mut buffer = TraceBuffer::new();
            assert!(buffer.capture(observed));
            assert_eq!(buffer.drain().unwrap().records().next(), Some(&observed));
        }
    }

    #[test]
    fn exhausted_batch_ids_keep_records_and_publish_changing_health() {
        let mut buffer = TraceBuffer {
            next_batch_id: Some(u64::MAX),
            ..TraceBuffer::new()
        };
        assert_eq!(buffer.drain().unwrap().id, u64::MAX);
        for id in 0..RECORD_CAPACITY as u64 {
            assert!(buffer.capture(SendRecord { id, ..record() }));
        }
        let retained = buffer.records;
        for expected in 1..=3 {
            assert!(!buffer.capture(record()));
            assert_eq!(buffer.drain(), Err(CounterFailure::BatchIdExhausted));
            let counters = buffer.counters();
            assert_eq!(counters.overflow, expected);
            assert_eq!(counters.batch_id_exhausted, expected);
            assert_eq!(buffer.len, RECORD_CAPACITY);
            assert_eq!(buffer.records, retained);
            let line = TraceLine::health(
                counters,
                TrackerCounters::default(),
                TransportCounters::default(),
            )
            .unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(!text.contains("batch_id="));
            match expected {
                1 => assert!(text.contains("batch_id_exhausted=1 ")),
                2 => assert!(text.contains("batch_id_exhausted=2 ")),
                3 => assert!(text.contains("batch_id_exhausted=3 ")),
                _ => unreachable!(),
            }
        }
        buffer.counters.batch_id_exhausted = u64::MAX;
        assert_eq!(buffer.drain(), Err(CounterFailure::BatchIdExhausted));
        let line = TraceLine::health(
            buffer.counters(),
            TrackerCounters::default(),
            TransportCounters::default(),
        )
        .unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("batch_id_exhausted=18446744073709551615 "));
        assert!(text.contains(" counter_saturated=true "));
        assert_eq!(buffer.records, retained);
    }

    #[test]
    fn writer_selection_progresses_both_ready_sources_without_consuming_peer() {
        let mut preference = WriterPreference::new();
        let mut diagnostics = 0i32;
        let mut traces = 0i32;
        let mut previous = None;
        for _ in 0..4096 {
            let mut consumed = 0;
            let selected = preference.choose(|source| {
                consumed += 1;
                Some(source)
            });
            assert_eq!(consumed, 1, "ready peer must remain queued");
            assert_ne!(selected, previous);
            match selected.unwrap() {
                WriterSource::Diagnostic => diagnostics += 1,
                WriterSource::Trace => traces += 1,
            }
            assert!((diagnostics - traces).abs() <= 1);
            previous = selected;
        }
        assert_eq!(diagnostics, 2048);
        assert_eq!(traces, 2048);
    }

    #[test]
    fn writer_selection_handles_absent_sources_and_registers_both_pending() {
        let mut preference = WriterPreference::new();
        for only_ready in [WriterSource::Diagnostic, WriterSource::Trace] {
            for _ in 0..1024 {
                let selected = preference.choose(|source| (source == only_ready).then_some(source));
                assert_eq!(selected, Some(only_ready));
            }
            let mut visited = [false; 2];
            let pending: Option<()> = preference.choose(|source| {
                visited[match source {
                    WriterSource::Diagnostic => 0,
                    WriterSource::Trace => 1,
                }] = true;
                None
            });
            assert_eq!(pending, None);
            assert_eq!(visited, [true; 2], "both pending sources must be polled");
            // Pending polls do not reset preference when the peer becomes ready.
            let selected = preference.choose(Some);
            assert_ne!(selected, Some(only_ready));
        }
    }
}
