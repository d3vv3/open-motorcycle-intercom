//! ESP audio admission and canonical relay storage. Platform clocks, locks,
//! codecs, membership, speaker grants, dedupe and transport remain their owners.
pub const CAPACITY: usize = 16;
pub const PACKET_BYTES: usize = 79;
#[derive(Clone, Copy)]
pub struct Entry {
    pub data: [u8; PACKET_BYTES],
    pub len: u16,
    pub enqueued_us: i64,
}
impl Default for Entry {
    fn default() -> Self {
        Self {
            data: [0; PACKET_BYTES],
            len: 0,
            enqueued_us: 0,
        }
    }
}
#[derive(Default)]
pub struct Ring {
    pub entries: [Entry; CAPACITY],
    pub head: u8,
    pub tail: u8,
}
impl Ring {
    pub const fn new() -> Self {
        Self {
            entries: [Entry {
                data: [0; PACKET_BYTES],
                len: 0,
                enqueued_us: 0,
            }; CAPACITY],
            head: 0,
            tail: 0,
        }
    }
}
/// Detached packet only; never an owner-memory view or durable in-flight bank.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Packet {
    pub data: [u8; PACKET_BYTES],
    pub len: u16,
}
const _: () = {
    assert!(core::mem::size_of::<Packet>() == 82);
    assert!(core::mem::align_of::<Packet>() == 2);
    assert!(core::mem::offset_of!(Packet, data) == 0);
    assert!(core::mem::offset_of!(Packet, len) == 80);
};
pub fn empty(q: &Ring) -> bool {
    q.head == q.tail
}
pub fn front(q: &Ring) -> Option<&Entry> {
    if !valid(q) || empty(q) {
        return None;
    }
    let entry = &q.entries[q.tail as usize];
    (9..=PACKET_BYTES as u16)
        .contains(&entry.len)
        .then_some(entry)
}
pub fn peek(q: &Ring, out: &mut Packet) -> bool {
    let Some(entry) = front(q) else {
        return false;
    };
    out.data.copy_from_slice(&entry.data);
    out.len = entry.len;
    true
}
fn next(i: u8) -> u8 {
    (i + 1) % CAPACITY as u8
}
fn valid(q: &Ring) -> bool {
    q.head < 16 && q.tail < 16
}
/// 0 invalid, 1 accepted, 2 accepted after dropping oldest. No packet-by-value.
pub fn enqueue(q: &mut Ring, packet: &[u8], ttl: u8, flags: u8, now: i64) -> u8 {
    if !valid(q) || !(9..=PACKET_BYTES).contains(&packet.len()) || ttl == 0 {
        return 0;
    }
    let head = next(q.head);
    let overwritten = head == q.tail;
    if overwritten {
        q.tail = next(q.tail);
    }
    let e = &mut q.entries[q.head as usize];
    e.data[..packet.len()].copy_from_slice(packet);
    e.data[4] = ttl;
    e.data[5] = flags;
    e.len = packet.len() as u16;
    e.enqueued_us = now;
    q.head = head;
    if overwritten { 2 } else { 1 }
}
pub fn expire(q: &mut Ring, now: i64) -> u8 {
    if !valid(q) {
        return 0;
    }
    let mut count = 0;
    while q.tail != q.head && now.wrapping_sub(q.entries[q.tail as usize].enqueued_us) > 20_000 {
        q.tail = next(q.tail);
        count += 1;
    }
    count
}
pub fn complete(q: &mut Ring, success: bool) {
    if valid(q) && success && q.tail != q.head {
        q.tail = next(q.tail);
    }
}
pub fn remove(q: &mut Ring, id: u8) {
    if !valid(q) {
        return;
    }
    let mut write = q.tail;
    let mut read = q.tail;
    while read != q.head {
        if q.entries[read as usize].data[2] != id {
            if write != read {
                q.entries[write as usize] = q.entries[read as usize];
            }
            write = next(write);
        }
        read = next(read);
    }
    q.head = write;
}
pub fn reset(q: &mut Ring) {
    // Clear in place; do not materialize a 1.5KB queue snapshot on the stack.
    for e in &mut q.entries {
        e.data.fill(0);
        e.len = 0;
        e.enqueued_us = 0;
    }
    q.head = 0;
    q.tail = 0;
}
/// Authentication precedes the existing dedupe owner. `direct` is copied
/// evidence of RF MAC equality; `via` comes from current membership/freshness.
pub fn authenticate(ttl: u8, direct_packet: u8, origin_bound: bool, direct: bool, via: bool) -> u8 {
    if !origin_bound {
        return 0;
    }
    if direct_packet != 0 {
        if ttl == 2 && direct { 1 } else { 0 }
    } else if ttl == 1 && via {
        2
    } else {
        0
    }
}
pub fn relay_allowed(
    ttl: u8,
    flags: u8,
    source: u8,
    local_bit: u8,
    ids: [u8; 2],
    masks: [u8; 2],
) -> bool {
    ttl > 1
        && flags & 5 == 5
        && ids
            .iter()
            .zip(masks)
            .any(|(&id, mask)| id == source && mask & local_bit != 0)
}
/// 0 empty, 1 local, 2 relay. Caller checks busy before expiry/selection.
pub fn select(local: bool, relay: bool, turn: bool) -> u8 {
    if relay && (!local || turn) {
        2
    } else if local {
        1
    } else {
        0
    }
}
/// An attempted submission changes turn even on synchronous error.
pub fn attempt_turn(selected: u8, relay_pending: bool) -> bool {
    selected == 1 && relay_pending
}
pub fn payload_valid(codec: u8, frame_ms: u8, len: u16, local_codec: u8) -> bool {
    frame_ms == 20
        && codec == local_codec
        && match codec {
            1 => len > 0 && len <= 64,
            2 => len == 48,
            _ => false,
        }
}
pub fn producer_size_valid(len: u16, lc3: bool) -> bool {
    len > 0 && len <= 64 && (!lc3 || len == 48)
}
pub fn pair_eligible(
    active: bool,
    role: u8,
    local: u8,
    members: u8,
    remotes: u8,
    remote_valid: bool,
) -> bool {
    active
        && (role == 1 || role == 2)
        && (1..=8).contains(&local)
        && members.count_ones() == 2
        && remotes == 1
        && remote_valid
}

/// Synchronous primitive executor. Packet is call-local and reused for both paths.
/// Only LOCAL and SUBMIT may access it; no owner reference crosses an effect.
pub mod effect {
    pub const IDLE: u8 = 0;
    pub const BUSY: u8 = 1;
    pub const CLOCK: u8 = 2;
    pub const DROPPED: u8 = 3;
    pub const ENABLED: u8 = 4;
    pub const LOCAL: u8 = 5;
    pub const META: u8 = 6;
    pub const WARN: u8 = 7;
    pub const PREPARE: u8 = 8;
    pub const SUBMIT: u8 = 9;
    pub const TURN: u8 = 10;
    pub const RELAY_ACTIVITY: u8 = 11;
    pub const LOCAL_CONSUME: u8 = 12;
    pub const LOCAL_ACTIVITY: u8 = 13;
    pub const COORDINATOR: u8 = 14;
    pub const GRANTS: u8 = 15;
    pub const SPEAKERS: u8 = 16;
}
pub trait Executor {
    fn effect(&mut self, op: u8, value: i64, packet: Option<&mut Packet>) -> i64;
    fn expire(&mut self, now: i64) -> u8;
    fn pending(&self) -> bool;
    fn peek(&self, packet: &mut Packet) -> bool;
    fn complete(&mut self);
}
/// Owns the complete send_audio_in_slot transaction, not transport stamping.
pub fn service(e: &mut impl Executor, deadline: i64) -> crate::tx_slot::SendResult {
    use crate::tx_slot::SendResult::*;
    use effect::*;
    if e.effect(IDLE, 0, None) != 1 {
        e.effect(BUSY, 0, None);
        return Busy;
    }
    let now = e.effect(CLOCK, 0, None);
    let dropped = e.expire(now);
    e.effect(DROPPED, dropped as i64, None);
    // Exactly one 82-byte packet backing, never retained by an executor.
    let mut packet = Packet {
        data: [0; PACKET_BYTES],
        len: 0,
    };
    let enabled = e.effect(ENABLED, 0, None);
    if !(0..=1).contains(&enabled) {
        return Error;
    }
    let local_result = if enabled == 1 {
        e.effect(LOCAL, 0, Some(&mut packet))
    } else {
        0
    };
    if !(0..=1).contains(&local_result) {
        return Error;
    }
    let local = local_result == 1;
    if local && packet.len > 64 {
        return Error;
    }
    let meta = e.effect(META, 0, None);
    if !(0..=0x7ffffff).contains(&meta) {
        return Error;
    }
    let turn = meta & (1 << 24) != 0;
    let logged = meta & (1 << 25) != 0;
    if local && e.pending() && !logged {
        e.effect(WARN, 0, None);
    }
    let selected = select(local, e.pending(), turn);
    if selected == 2 {
        if !e.peek(&mut packet) {
            return Error;
        }
        let source = packet.data[2];
        let ok = e.effect(SUBMIT, deadline, Some(&mut packet)) == 1;
        let turn = attempt_turn(selected, e.pending());
        e.effect(TURN, turn as i64, None);
        if ok {
            e.effect(RELAY_ACTIVITY, source as i64, None);
            e.complete();
        }
        return if ok { Submitted } else { Error };
    }
    if selected != 1 || e.effect(ENABLED, 0, None) != 1 {
        return Empty;
    }
    let node = meta as u8;
    let channel = (meta >> 8) as u8;
    let codec = (meta >> 16) as u8;
    if !(1..=8).contains(&node) || !(1..=2).contains(&codec) {
        return Error;
    }
    let ids = e.effect(SPEAKERS, 0, None);
    if !(0..=65535).contains(&ids) {
        return Error;
    }
    let granted = ids as u8 == node || (ids >> 8) as u8 == node;
    let len = packet.len;
    packet.data[..12].copy_from_slice(&[
        5,
        1,
        node,
        0,
        2,
        1 | if granted { 4 } else { 0 },
        channel,
        (4 + len) as u8,
        0,
        codec,
        20,
        node,
    ]);
    packet.len = 13 + len;
    e.effect(PREPARE, 0, None);
    if e.effect(ENABLED, 0, None) != 1 {
        return Empty;
    }
    let ok = e.effect(SUBMIT, deadline, Some(&mut packet)) == 1;
    let turn = attempt_turn(selected, e.pending());
    e.effect(TURN, turn as i64, None);
    if ok {
        e.effect(LOCAL_CONSUME, 0, None);
        e.effect(LOCAL_ACTIVITY, node as i64, None);
        if e.effect(COORDINATOR, 0, None) == 1 {
            e.effect(GRANTS, 0, None);
        }
    }
    if ok { Submitted } else { Error }
}
