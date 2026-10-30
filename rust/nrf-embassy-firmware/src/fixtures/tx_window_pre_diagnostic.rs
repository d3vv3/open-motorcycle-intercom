// Frozen phase9cb / retained B666 pre-diagnostic policy, original lines 16..144.
// Full actor_policy.rs SHA-256: 3917bf2af33528316d05645a390598f5417c8e4e972fbc9c24624842067163ee
// Git blob: 6bf90e9b355b9e9746ee4e2d91037850c53c7075. Only imports adapted.
// TxWindow enum + impl (lines 16..132, trailing LF) SHA-256:
// 141f70e37f6e591503496e87498422170eeecb040bd30da9c747a7d124a85d0a
// bounded_air_deadline function (lines 136..144, trailing LF) SHA-256:
// 8319725733784f50b073ecec4008895887ef92b50bb4c52c7ca4cf7f78394c92
use crate::actor_policy::ActorCallback;
use crate::timing_clock::{ClockMapping, ClockProfile, PhysicalTime, TimingTime};
use nrf_embassy::tdma::{Snapshot, WorkMetadata};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxWindow {
    Slot(WorkMetadata),
    Control(WorkMetadata),
}

impl TxWindow {
    /// `now` uses the supplied snapshot's timing units; capture preserves the
    /// source metadata for later physical deadline admission.
    pub fn capture(snapshot: Snapshot, callback: ActorCallback, now: i64) -> Option<Self> {
        let (window, frame) = match callback {
            ActorCallback::Slot { slot, frame } if snapshot.slot_index == slot as i8 => {
                (Self::Slot(snapshot.slot), frame)
            }
            ActorCallback::Control { frame } => (Self::Control(snapshot.control), frame),
            _ => return None,
        };
        let metadata = window.metadata();
        (snapshot.running
            && snapshot.generation == metadata.generation
            && snapshot.frame_counter == frame
            && metadata.frame == frame
            && now >= metadata.start_us
            && now < metadata.deadline_us)
            .then_some(window)
    }

    fn metadata(self) -> WorkMetadata {
        match self {
            Self::Slot(m) | Self::Control(m) => m,
        }
    }

    /// Original numeric admission: `now` and metadata are normalized physical us.
    /// Preserve its numeric behavior, including windows starting before zero.
    pub fn deadline(self, snapshot: Snapshot, now: i64, packet_len: usize) -> Option<u64> {
        let callback = match self {
            Self::Slot(m) => ActorCallback::Slot {
                slot: snapshot.slot_index as u8,
                frame: m.frame,
            },
            Self::Control(m) => ActorCallback::Control { frame: m.frame },
        };
        if Self::capture(snapshot, callback, now) != Some(self) {
            return None;
        }
        bounded_air_deadline(now, self.metadata().deadline_us, packet_len)
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
        const DEFAULT_MAPPING: ClockMapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(0),
        );
        if mapping == DEFAULT_MAPPING {
            self.deadline(snapshot, physical_now, packet_len)
        } else {
            self.deadline_with_mapping(
                snapshot,
                PhysicalTime::new(u64::try_from(physical_now).ok()?),
                packet_len,
                mapping,
            )
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
        let timing_now = mapping.timing_at(physical_now).ok()?.value();
        let callback = match self {
            Self::Slot(m) => ActorCallback::Slot {
                slot: snapshot.slot_index as u8,
                frame: m.frame,
            },
            Self::Control(m) => ActorCallback::Control { frame: m.frame },
        };
        if Self::capture(snapshot, callback, timing_now) != Some(self) {
            return None;
        }
        let metadata = self.metadata();
        let physical_window = mapping
            .convert(
                TimingTime::new(metadata.start_us),
                TimingTime::new(metadata.deadline_us),
            )
            .ok()?;
        if physical_now.value() < physical_window.start().value()
            || physical_now.value() >= physical_window.deadline().value()
        {
            return None;
        }
        bounded_air_deadline(
            i64::try_from(physical_now.value()).ok()?,
            i64::try_from(physical_window.deadline().value()).ok()?,
            packet_len,
        )
    }
}

/// Source 2 Mbps budget includes ramp and the 100 us margin (also covers the
/// guard's 20 us latest-CPU-launch lead). Not physical timing qualification.
pub fn bounded_air_deadline(now: i64, until: i64, packet_len: usize) -> Option<u64> {
    let budget = 129i64
        .checked_add(4i64.checked_mul(i64::try_from(packet_len).ok()?.checked_add(11)?)?)?
        .checked_add(100)?;
    if now < 0 || now.checked_add(budget)? > until {
        return None;
    }
    u64::try_from(until.min(now.checked_add(1450)?)).ok()
}
