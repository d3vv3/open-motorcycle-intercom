//! Call-local SYNC production and last-pre-submit stamp projection.
//! Clock deltas use the signed modular half-range contract of frame/TX slot.
use crate::adaptive::{self, Adaptive, Identity, Sync};

pub fn local(owner: &Adaptive, identity: Identity, frame: u32) -> Sync {
    let (local_id, _, members, term) = owner.session();
    Sync {
        leader: identity,
        leader_id: local_id,
        member_count: (members.count_ones() as u8).max(1),
        term,
        frame_counter: frame,
        ..Sync::default()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub now_us: i64,
    pub frame_start_us: i64,
    pub local_frame: u32,
    pub upstream_us: i64,
}

pub fn stamp(input: Sync, relay: bool, sample: Sample) -> Option<Sync> {
    let elapsed = sample.now_us.wrapping_sub(sample.frame_start_us).max(0);
    let frame = sample.local_frame.wrapping_add((elapsed / 20_000) as u32);
    let phase = (elapsed % 20_000) as u16;
    let upstream_elapsed = sample.now_us.wrapping_sub(sample.upstream_us);
    let age = if sample.upstream_us > 0 && upstream_elapsed >= 0 {
        (upstream_elapsed / 1000) as u32
    } else {
        u32::MAX
    };
    let mut output = input;
    if relay && output.relay_depth == 1 {
        output.relay_depth = 0;
    }
    adaptive::stamp_sync(Some(&mut output), relay, frame, phase, age).then_some(output)
}
