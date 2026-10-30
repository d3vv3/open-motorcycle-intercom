//! Passive receiver diagnostics only; no protocol, actor, or send capability.

pub const DRAIN_LIMIT: usize = 8;

/// Old C DEVICEID byte order. The fifth byte is wire identity, not pipe-1 prefix.
pub fn device_identity(deviceid0: u32, deviceid1: u32) -> [u8; 5] {
    let bytes = deviceid0.to_le_bytes();
    [bytes[0], bytes[1], bytes[2], bytes[3], deviceid1 as u8]
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReceiverMetadata {
    pub channel: u8,
    pub epoch: u64,
    pub timestamp_us: Option<i64>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReceiverRecord {
    pub bytes: u32,
    pub metadata: Option<ReceiverMetadata>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReceiverSnapshot {
    pub irq_count: u32,
    pub received_packets: u32,
    pub bytes: u32,
    pub missing_metadata: u32,
    pub missing_clock: u32,
    pub clock_errors: u32,
    pub last: Option<ReceiverMetadata>,
}

impl ReceiverSnapshot {
    pub fn record(&mut self, record: ReceiverRecord) {
        self.received_packets = self.received_packets.wrapping_add(1);
        self.bytes = self.bytes.wrapping_add(record.bytes);
        self.last = record.metadata;
        match record.metadata {
            None => self.missing_metadata = self.missing_metadata.wrapping_add(1),
            Some(metadata) if metadata.timestamp_us.is_none() => {
                self.missing_clock = self.missing_clock.wrapping_add(1);
            }
            Some(_) => {}
        }
    }

    pub fn age_us(self, now_us: u64) -> Option<u64> {
        let timestamp = u64::try_from(self.last?.timestamp_us?).ok()?;
        now_us.checked_sub(timestamp)
    }
}

/// The source copies data and releases its packet claim before returning a record.
/// At most eight dequeues per wake, even when the producer keeps filling the FIFO.
pub fn drain(snapshot: &mut ReceiverSnapshot, mut next: impl FnMut() -> Option<ReceiverRecord>) {
    for _ in 0..DRAIN_LIMIT {
        let Some(record) = next() else { break };
        snapshot.record(record);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_matches_c_but_fifth_byte_is_not_a_prefix() {
        assert_eq!(
            device_identity(0x67452301, 0xefcdab89),
            [1, 0x23, 0x45, 0x67, 0x89]
        );
    }

    #[test]
    fn bounded_fifo_drain_copies_records_without_mutating_source() {
        let records = core::array::from_fn::<_, 10, _>(|index| ReceiverRecord {
            bytes: index as u32 + 1,
            metadata: Some(ReceiverMetadata {
                channel: 40,
                epoch: 1,
                timestamp_us: Some(index as i64),
            }),
        });
        let original = records;
        let mut fifo = records.iter().copied();
        let mut snapshot = ReceiverSnapshot::default();
        drain(&mut snapshot, || fifo.next());
        assert_eq!(snapshot.received_packets, 8);
        assert_eq!(snapshot.bytes, 36);
        assert_eq!(snapshot.last, records[7].metadata);
        assert_eq!(snapshot.age_us(10), Some(3));
        drain(&mut snapshot, || fifo.next());
        assert_eq!(snapshot.received_packets, 10);
        assert_eq!(snapshot.bytes, 55);
        assert_eq!(snapshot.last, records[9].metadata);
        assert_eq!(records, original);
    }

    #[test]
    fn missing_metadata_and_clock_never_become_zero_timestamps() {
        let mut snapshot = ReceiverSnapshot::default();
        snapshot.record(ReceiverRecord {
            bytes: 252,
            metadata: None,
        });
        assert_eq!(snapshot.missing_metadata, 1);
        assert_eq!(snapshot.age_us(100), None);
        snapshot.record(ReceiverRecord {
            bytes: 252,
            metadata: Some(ReceiverMetadata {
                channel: 40,
                epoch: u64::MAX,
                timestamp_us: None,
            }),
        });
        assert_eq!(snapshot.missing_clock, 1);
        assert_eq!(snapshot.last.unwrap().epoch, u64::MAX);
        assert_eq!(snapshot.age_us(100), None);
        snapshot.last.as_mut().unwrap().timestamp_us = Some(0);
        assert_eq!(snapshot.age_us(100), Some(100));
        snapshot.last.as_mut().unwrap().timestamp_us = Some(101);
        assert_eq!(snapshot.age_us(100), None);
    }
}
