//! Semantic port of all 29 `mesh_adaptive_*` entry points.
//! Nullable C arguments use `Option`; storage and internal indices cannot be forged.
use crate::core::SlotMap;

pub const REPORT_MS: u32 = 1000;
pub const EXPIRE_MS: u32 = 3000;
pub const HOLD_MS: u32 = 10000;
pub const BETTER_MS: u32 = 3000;
pub const SWITCH_LEAD_FRAMES: u32 = 160;
pub const PREPARE_MIN_FRAMES: u32 = 20;
pub const ACK_DEADLINE_MS: u32 = 2400;
pub const RECOVERY_GRACE_MS: u32 = 2000;
pub const SYNC_MAX_AGE_MS: u32 = 600;
pub const FORWARD_CACHE: usize = 32;
pub const FORWARD_QUEUE: usize = 8;
pub const REQUEST_MAX_AGE_FRAMES: u32 = 50;
pub const REQUEST_ACTIVE_MS: u32 = 1500;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Identity {
    pub address_len: u8,
    pub address: [u8; 6],
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Topology {
    pub report_seq: u32,
    pub origin: Identity,
    pub direct_mask: u8,
    pub rssi_dbm: [i8; 8],
    pub quality: [u8; 8],
    pub leader_id: u8,
    pub term: u32,
    pub slot_map: SlotMap,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Membership {
    pub term: u32,
    pub revision: u32,
    pub leader_id: u8,
    pub slot_map: SlotMap,
    pub identities: [Identity; 8],
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sync {
    pub leader: Identity,
    pub leader_id: u8,
    pub member_count: u8,
    pub term: u32,
    pub frame_counter: u32,
    pub phase_us: u16,
    pub relay_depth: u8,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Handover {
    pub old_leader_id: u8,
    pub candidate_id: u8,
    pub next_term: u32,
    pub switch_frame: u32,
    pub members: u8,
    pub slot_map: SlotMap,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HandoverAck {
    pub old_leader_id: u8,
    pub candidate_id: u8,
    pub next_term: u32,
    pub switch_frame: u32,
    pub acknowledger_id: u8,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpeakerRequest {
    pub term: u32,
    pub request_seq: u32,
    pub frame_counter: u32,
    pub active: u8,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Header {
    pub version: u8,
    pub packet_type: u8,
    pub src_id: u8,
    pub seq: u8,
    pub ttl: u8,
    pub flags: u8,
    pub talk_channel: u8,
    pub payload_len: u16,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Member {
    pub present: bool,
    pub report_seen: bool,
    pub seq_seen: bool,
    pub identity: Identity,
    pub last_seen_ms: u32,
    pub report_ms: u32,
    pub report_seq: u32,
    pub loss_quality: u8,
    pub direct_mask: u8,
    pub rssi: [i8; 8],
    pub quality: [u8; 8],
    pub leader_id: u8,
    pub term: u32,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ForwardKey {
    pub packet_type: u8,
    pub src_id: u8,
    pub seq: u8,
    pub term: u32,
    pub seen_ms: u32,
    pub origin: Identity,
    pub used: bool,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ForwardPacket {
    pub header: Header,
    pub payload: [u8; 200],
    pub payload_len: u16,
    pub enqueued_ms: u32,
}
impl Default for ForwardPacket {
    fn default() -> Self {
        Self {
            header: Header::default(),
            payload: [0; 200],
            payload_len: 0,
            enqueued_ms: 0,
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub local_id: u8,
    pub leader_id: u8,
    pub members: u8,
    pub term: u32,
    pub leader_since_ms: u32,
    pub better_since_ms: u32,
    pub better_id: u8,
    pub nodes: [Member; 8],
    pub edge_live: [u8; 8],
    pub pending: Handover,
    pub ack_mask: u8,
    pub deadline_ms: u32,
    pub recovery_deadline_ms: u32,
    pub membership_revision: u32,
    pub publish_revision: u32,
    pub membership_revision_seen: bool,
    pub request_seq: [u32; 8],
    pub request_deadline_ms: [u32; 8],
    pub request_seq_seen: [bool; 8],
    pub request_active: [bool; 8],
    pub prepared: bool,
    pub committed: bool,
    pub transition_done: bool,
    pub cache: [ForwardKey; 32],
    pub queue: [ForwardPacket; 8],
    pub queue_head: u8,
    pub queue_count: u8,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Adaptive {
    s: Snapshot,
}
impl Adaptive {
    /// Reset in place: avoid constructing a full queue/snapshot on the mesh
    /// task's bounded stack. Storage and dedupe keys are reset, not mirrored.
    pub fn reset(&mut self) {
        let s = &mut self.s;
        s.local_id = 0;
        s.leader_id = 0;
        s.members = 0;
        s.term = 0;
        s.leader_since_ms = 0;
        s.better_since_ms = 0;
        s.better_id = 0;
        s.nodes.fill(Member::default());
        s.edge_live.fill(0);
        s.pending = Handover::default();
        s.ack_mask = 0;
        s.deadline_ms = 0;
        s.recovery_deadline_ms = 0;
        s.membership_revision = 0;
        s.publish_revision = 0;
        s.membership_revision_seen = false;
        s.request_seq.fill(0);
        s.request_deadline_ms.fill(0);
        s.request_seq_seen.fill(false);
        s.request_active.fill(false);
        s.prepared = false;
        s.committed = false;
        s.transition_done = false;
        s.cache.fill(ForwardKey::default());
        s.queue.fill(ForwardPacket::default());
        s.queue_head = 0;
        s.queue_count = 0;
    }

    pub fn snapshot(&self) -> Snapshot {
        self.s.clone()
    }

    pub fn session(&self) -> (u8, u8, u8, u32) {
        (
            self.s.local_id,
            self.s.leader_id,
            self.s.members,
            self.s.term,
        )
    }

    pub fn transition(&self) -> (bool, bool, bool, u32, u32, Handover) {
        (
            self.s.prepared,
            self.s.committed,
            self.s.transition_done,
            self.s.deadline_ms,
            self.s.recovery_deadline_ms,
            self.s.pending,
        )
    }

    pub fn member_identity(&self, id: u8) -> Identity {
        if bit(id) == 0 || self.s.members & bit(id) == 0 {
            Identity::default()
        } else {
            self.s.nodes[usize::from(id - 1)].identity
        }
    }

    pub fn request(&self, id: u8) -> (bool, u32) {
        if bit(id) == 0 || self.s.members & bit(id) == 0 {
            (false, 0)
        } else {
            let i = usize::from(id - 1);
            (self.s.request_active[i], self.s.request_deadline_ms[i])
        }
    }

    /// Replace only an already queued matching SYNC, without adding a queue or
    /// dedupe key. The adapter calls this after sender/clock validation.
    pub fn coalesce_sync(&mut self, header: Header, sync: Sync, now: u32) -> bool {
        if header.packet_type != 17
            || header.payload_len != 20
            || header.version != 5
            || header.ttl != 2
            || header.flags & 2 != 0
            || !identity_valid(Some(&sync.leader))
            || sync.phase_us >= 20000
            || sync.relay_depth != 0
            || sync.term == 0
            || bit(sync.leader_id) == 0
            || !(1..=8).contains(&sync.member_count)
        {
            return false;
        }
        for offset in 0..self.forward_queue_len() {
            let index = (usize::from(self.s.queue_head) + offset) % FORWARD_QUEUE;
            let packet = &mut self.s.queue[index];
            if packet.header.packet_type != 17 || packet.header.src_id != header.src_id {
                continue;
            }
            let older = read_sync(&packet.payload);
            if older.term != sync.term || older.leader_id != sync.leader_id {
                continue;
            }
            packet.header = header;
            packet.header.flags |= 2;
            packet.header.ttl = 1;
            packet.payload[..7].copy_from_slice(&[
                sync.leader.address_len,
                sync.leader.address[0],
                sync.leader.address[1],
                sync.leader.address[2],
                sync.leader.address[3],
                sync.leader.address[4],
                sync.leader.address[5],
            ]);
            packet.payload[7] = sync.leader_id;
            packet.payload[8] = sync.member_count;
            packet.payload[9..13].copy_from_slice(&sync.term.to_le_bytes());
            packet.payload[13..17].copy_from_slice(&sync.frame_counter.to_le_bytes());
            packet.payload[17..19].copy_from_slice(&sync.phase_us.to_le_bytes());
            packet.payload[19] = sync.relay_depth;
            packet.enqueued_ms = now;
            return true;
        }
        false
    }

    /// Existing S31 strict priority: lifecycle > SYNC/request > other traffic.
    /// Equal priority remains FIFO; promotion never swaps unrelated entries.
    pub fn select_forward(&mut self) {
        let mut best = 0;
        let mut priority = 0;
        for offset in 0..self.forward_queue_len() {
            let t = self.forward_queue_entry(offset).unwrap().header.packet_type;
            let p = match t {
                15 | 16 | 22 | 6 | 18..=21 => 2,
                17 | 23 => 1,
                _ => 0,
            };
            if p > priority {
                best = offset;
                priority = p;
            }
        }
        self.promote_forward_queue(best);
    }

    pub fn forward_queue_len(&self) -> usize {
        usize::from(self.s.queue_count)
    }

    pub fn forward_queue_contains_type(&self, packet_type: u8) -> bool {
        (0..self.forward_queue_len()).any(|offset| {
            self.forward_queue_entry(offset)
                .is_some_and(|entry| entry.header.packet_type == packet_type)
        })
    }

    /// Borrows an active entry in logical FIFO order, excluding stale storage.
    pub fn forward_queue_entry(&self, logical_offset: usize) -> Option<&ForwardPacket> {
        if logical_offset >= self.forward_queue_len() {
            return None;
        }
        Some(&self.s.queue[(usize::from(self.s.queue_head) + logical_offset) % FORWARD_QUEUE])
    }

    /// Clears only the cursors, retaining queue storage and the deduplication cache.
    pub fn clear_forward_queue(&mut self) {
        self.s.queue_head = 0;
        self.s.queue_count = 0;
    }

    /// Moves the selected entry to logical head, preserving all other FIFO order.
    /// Selection policy belongs to the caller; cursors and packet metadata stay intact.
    pub fn promote_forward_queue(&mut self, logical_offset: usize) -> bool {
        let Some(selected) = self.forward_queue_entry(logical_offset).copied() else {
            return false;
        };
        let head = usize::from(self.s.queue_head);
        for offset in (1..=logical_offset).rev() {
            self.s.queue[(head + offset) % FORWARD_QUEUE] =
                self.s.queue[(head + offset - 1) % FORWARD_QUEUE];
        }
        self.s.queue[head] = selected;
        true
    }
}
fn bit(id: u8) -> u8 {
    if (1..=8).contains(&id) {
        1 << (id - 1)
    } else {
        0
    }
}
fn after(a: u32, b: u32) -> bool {
    a.wrapping_sub(b) < 0x8000_0000
}
fn newer(a: u32, b: u32) -> bool {
    a != b && after(a, b)
}
fn fresh(now: u32, then: u32, limit: u32) -> bool {
    now.wrapping_sub(then) <= limit
}
pub fn identity_valid(id: Option<&Identity>) -> bool {
    let Some(id) = id else { return false };
    let n = usize::from(id.address_len);
    (1..=6).contains(&n)
        && id.address[n..].iter().all(|&v| v == 0)
        && !id.address[..n].iter().all(|&v| v == 0)
        && !id.address[..n].iter().all(|&v| v == 255)
}
pub fn cold_join(
    local: Option<&Identity>,
    local_count: u8,
    remote: Option<&Identity>,
    remote_count: u8,
) -> bool {
    if !identity_valid(local) || !identity_valid(remote) {
        return false;
    }
    let (a, b) = (local.unwrap(), remote.unwrap());
    a.address_len == b.address_len
        && local_count == 1
        && (1..=8).contains(&remote_count)
        && a.address != b.address
        && (remote_count > 1 || a.address > b.address)
}
pub fn init(s: Option<&mut Adaptive>, local: u8, leader: u8, term: u32, members: u8, now: u32) {
    let Some(s) = s else { return };
    s.reset();
    if bit(local) == 0
        || bit(leader) == 0
        || members & bit(local) == 0
        || members & bit(leader) == 0
    {
        return;
    }
    s.s.local_id = local;
    s.s.leader_id = leader;
    s.s.term = term;
    s.s.members = members;
    s.s.leader_since_ms = now;
    s.s.nodes[usize::from(local - 1)].present = true;
    s.s.nodes[usize::from(local - 1)].last_seen_ms = now;
}
fn forward_references_member(q: &ForwardPacket, id: u8) -> bool {
    let p = &q.payload;
    if q.header.src_id == id || matches!(q.header.packet_type, 18..=21) {
        return true;
    }
    let map = match q.header.packet_type {
        6 => Some(read_slots(p)),
        14 => Some(read_slots(&p[33..])),
        22 => Some(read_slots(&p[9..])),
        16 => return p[7] == id,
        9 => return p[1..3].contains(&id) || p[3..5].iter().any(|v| v & bit(id) != 0),
        10 => return p[1..3].contains(&id),
        _ => None,
    };
    map.is_some_and(|m| {
        m.slot_ids.contains(&id)
            || m.active_speaker_ids.contains(&id)
            || m.relay_masks.iter().any(|v| v & bit(id) != 0)
    })
}
fn reset_member(s: &mut Snapshot, id: u8) {
    let i = usize::from(id - 1);
    s.nodes[i] = Member::default();
    s.request_seq[i] = 0;
    s.request_deadline_ms[i] = 0;
    s.request_seq_seen[i] = false;
    s.request_active[i] = false;
    s.edge_live[i] = 0;
    for j in 0..8 {
        s.edge_live[j] &= !bit(id);
        s.nodes[j].direct_mask &= !bit(id);
        s.nodes[j].quality[i] = 0;
    }
    // Identity replacement invalidates only this source's dedupe binding.
    // Cancelling a handover invalidates proposal traffic, not survivor traffic.
    for key in &mut s.cache {
        if key.src_id == id || matches!(key.packet_type, 18..=21) {
            *key = ForwardKey::default();
        }
    }
    let mut kept = 0;
    for offset in 0..usize::from(s.queue_count) {
        let index = (usize::from(s.queue_head) + offset) % FORWARD_QUEUE;
        if forward_references_member(&s.queue[index], id) {
            continue;
        }
        let dest = (usize::from(s.queue_head) + kept) % FORWARD_QUEUE;
        s.queue[dest] = s.queue[index];
        kept += 1;
    }
    s.queue_count = kept as u8;
    s.prepared = false;
    s.committed = false;
    s.transition_done = false;
    s.better_id = 0;
}
pub fn set_member(s: Option<&mut Adaptive>, id: u8, identity: Option<&Identity>) -> bool {
    let Some(s) = s else { return false };
    let s = &mut s.s;
    if bit(id) == 0
        || identity.is_some_and(|v| !identity_valid(Some(v)))
        || (identity.is_none() && (id == s.local_id || id == s.leader_id))
    {
        return false;
    }
    if let Some(v) = identity {
        for other in 1..=8 {
            let n = &s.nodes[usize::from(other - 1)];
            if other != id
                && s.members & bit(other) != 0
                && n.identity.address_len != 0
                && n.identity == *v
            {
                return false;
            }
        }
        let n = &s.nodes[usize::from(id - 1)];
        if s.members & bit(id) != 0 && n.identity.address_len != 0 && n.identity == *v {
            return true;
        }
    }
    reset_member(s, id);
    if let Some(v) = identity {
        s.members |= bit(id);
        s.nodes[usize::from(id - 1)].present = true;
        s.nodes[usize::from(id - 1)].identity = *v;
    } else {
        s.members &= !bit(id);
    }
    true
}
pub fn update_members(s: Option<&mut Adaptive>, members: u8) -> bool {
    let Some(s) = s else { return false };
    let s = &mut s.s;
    if members & bit(s.local_id) == 0 || members & bit(s.leader_id) == 0 {
        return false;
    }
    for id in 1..=8 {
        if (s.members ^ members) & bit(id) != 0 {
            reset_member(s, id);
        }
    }
    s.members = members;
    true
}
pub fn control_authorized(s: Option<&Adaptive>, t: u8, source: u8) -> bool {
    let Some(s) = s else { return false };
    let s = &s.s;
    if t == 15 {
        return source == 0 || s.members & bit(source) != 0;
    }
    if s.members & bit(source) == 0 {
        return false;
    }
    match t {
        14 | 19 | 23 => true,
        6 | 9 | 10 | 16 | 17 | 18 | 20 | 21 | 22 => source == s.leader_id,
        _ => false,
    }
}
/// Participant-only SLOT_MAP authority. No map is retained, and the wire has no
/// revision/term: same-binding, same-membership replay cannot be distinguished.
pub fn participant_map(s: &Adaptive, active: bool, source: u8, map: &SlotMap) -> Option<i8> {
    let (local, leader, members, _) = s.session();
    if !active
        || bit(local) == 0
        || local == leader
        || source != leader
        || members & bit(leader) == 0
        || !slots_valid(map, members)
    {
        return None;
    }
    map.slot_ids
        .iter()
        .position(|id| *id == local)
        .map(|slot| slot as i8)
}
pub(crate) fn slots_valid(m: &SlotMap, members: u8) -> bool {
    if members == 0
        || m.slot_count == 0
        || m.slot_count > 8
        || m.slot_ids[usize::from(m.slot_count - 1)] == 0
        || m.active_speaker_count > 2
    {
        return false;
    }
    let (mut found, mut speakers) = (0, 0);
    for i in 0..8 {
        let v = m.slot_ids[i];
        let b = bit(v);
        if v != 0 && (i >= usize::from(m.slot_count) || members & b == 0 || found & b != 0) {
            return false;
        }
        found |= b;
    }
    for i in 0..2 {
        let v = m.active_speaker_ids[i];
        let b = bit(v);
        if i >= usize::from(m.active_speaker_count) {
            if v != 0 || m.relay_masks[i] != 0 {
                return false;
            }
        } else if members & b == 0 || speakers & b != 0 || m.relay_masks[i] & !members != 0 {
            return false;
        }
        speakers |= b;
    }
    found == members
}
pub fn membership_valid(m: Option<&Membership>, members: Option<&mut u8>) -> bool {
    let Some(m) = m else { return false };
    if m.term == 0 || bit(m.leader_id) == 0 {
        return false;
    }
    let mut mask = 0;
    for &id in &m.slot_map.slot_ids {
        if id != 0 && bit(id) == 0 {
            return false;
        }
        mask |= bit(id);
    }
    if mask & bit(m.leader_id) == 0 || !slots_valid(&m.slot_map, mask) {
        return false;
    }
    for i in 0..8 {
        let id = &m.identities[i];
        if mask & bit(i as u8 + 1) == 0 {
            if *id != Identity::default() {
                return false;
            }
        } else {
            if !identity_valid(Some(id)) {
                return false;
            }
            for j in 0..i {
                if mask & bit(j as u8 + 1) != 0 && *id == m.identities[j] {
                    return false;
                }
            }
        }
    }
    if let Some(out) = members {
        *out = mask;
    }
    true
}
pub fn apply_membership(s: Option<&mut Adaptive>, source: u8, m: Option<&Membership>) -> bool {
    let (Some(s), Some(m)) = (s, m) else {
        return false;
    };
    let mut mask = 0;
    if !control_authorized(Some(s), 22, source) || !membership_valid(Some(m), Some(&mut mask)) {
        return false;
    }
    let s = &mut s.s;
    if m.term != s.term
        || m.leader_id != s.leader_id
        || (s.membership_revision_seen && !newer(m.revision, s.membership_revision))
        || mask & bit(s.local_id) == 0
    {
        return false;
    }
    for id in [s.local_id, s.leader_id] {
        let bound = s.nodes[usize::from(id - 1)].identity;
        if bound.address_len != 0 && bound != m.identities[usize::from(id - 1)] {
            return false;
        }
    }
    for id in 1..=8 {
        let i = usize::from(id - 1);
        if s.members & bit(id) == mask & bit(id)
            && (mask & bit(id) == 0 || s.nodes[i].identity == m.identities[i])
        {
            continue;
        }
        reset_member(s, id);
        if mask & bit(id) != 0 {
            s.nodes[i].identity = m.identities[i];
            s.nodes[i].present = true;
        }
    }
    s.members = mask;
    s.membership_revision = m.revision;
    s.membership_revision_seen = true;
    true
}
pub fn membership_removes_local(s: Option<&Adaptive>, source: u8, m: Option<&Membership>) -> bool {
    let (Some(s), Some(m)) = (s, m) else {
        return false;
    };
    let mut mask = 0;
    if bit(s.s.local_id) == 0
        || s.s.members & bit(s.s.local_id) == 0
        || !control_authorized(Some(s), 22, source)
        || !membership_valid(Some(m), Some(&mut mask))
    {
        return false;
    }
    let s = &s.s;
    if m.term != s.term
        || m.leader_id != s.leader_id
        || (s.membership_revision_seen && !newer(m.revision, s.membership_revision))
        || mask & bit(s.local_id) != 0
    {
        return false;
    }
    let leader = &s.nodes[usize::from(s.leader_id - 1)].identity;
    identity_valid(Some(leader)) && *leader == m.identities[usize::from(s.leader_id - 1)]
}
pub fn next_membership_revision(s: Option<&mut Adaptive>) -> u32 {
    let Some(s) = s else { return 0 };
    let s = &mut s.s;
    if s.term == 0 || s.local_id != s.leader_id {
        return 0;
    }
    s.publish_revision = s.publish_revision.wrapping_add(1);
    s.publish_revision
}
pub fn report_valid(s: Option<&Adaptive>, source: u8, r: Option<&Topology>) -> bool {
    let (Some(s), Some(r)) = (s, r) else {
        return false;
    };
    let s = &s.s;
    if s.members & bit(source) == 0
        || !identity_valid(Some(&r.origin))
        || bit(r.leader_id) == 0
        || r.term != s.term
        || r.leader_id != s.leader_id
        || r.direct_mask & bit(source) != 0
        || (r.slot_map.slot_count != 0 && !slots_valid(&r.slot_map, s.members))
        || (r.slot_map.slot_count == 0 && r.slot_map.active_speaker_count != 0)
    {
        return false;
    }
    for i in 0..8 {
        if r.direct_mask & bit(i as u8 + 1) != 0
            && (s.members & bit(i as u8 + 1) == 0
                || r.quality[i] > 100
                || r.rssi_dbm[i] > 0
                || r.rssi_dbm[i] < -127)
        {
            return false;
        }
    }
    let n = &s.nodes[usize::from(source - 1)];
    n.present && identity_valid(Some(&n.identity)) && n.identity == r.origin
}
pub fn report(
    s: Option<&mut Adaptive>,
    source: u8,
    r: Option<&Topology>,
    relayed: bool,
    now: u32,
) -> bool {
    let (Some(s), Some(r)) = (s, r) else {
        return false;
    };
    if !report_valid(Some(s), source, Some(r)) {
        return false;
    }
    let n = &mut s.s.nodes[usize::from(source - 1)];
    if n.seq_seen {
        let delta = r.report_seq.wrapping_sub(n.report_seq);
        if delta == 0 || delta >= 0x8000_0000 {
            return false;
        }
        n.loss_quality = ((3 * u32::from(n.loss_quality) + 100 / delta + 2) / 4) as u8;
    } else {
        n.loss_quality = 100;
    }
    let first = !n.report_seen;
    n.seq_seen = true;
    n.report_seen = true;
    n.report_seq = r.report_seq;
    n.report_ms = now;
    n.direct_mask = r.direct_mask;
    n.leader_id = r.leader_id;
    n.term = r.term;
    if !relayed {
        n.last_seen_ms = now;
    }
    for i in 0..8 {
        if r.direct_mask & bit(i as u8 + 1) != 0 {
            let initial = first || n.quality[i] == 0;
            n.rssi[i] = if initial {
                r.rssi_dbm[i]
            } else {
                ((3 * i32::from(n.rssi[i]) + i32::from(r.rssi_dbm[i])) / 4) as i8
            };
            n.quality[i] = if initial {
                r.quality[i]
            } else {
                ((3 * u32::from(n.quality[i]) + u32::from(r.quality[i]) + 2) / 4) as u8
            };
        } else {
            n.quality[i] = 0;
        }
    }
    true
}
pub fn speaker_request(
    s: Option<&mut Adaptive>,
    source: u8,
    r: Option<&SpeakerRequest>,
    frame: u32,
    now: u32,
) -> bool {
    let (Some(s), Some(r)) = (s, r) else {
        return false;
    };
    let s = &mut s.s;
    if bit(source) == 0
        || s.members & bit(source) == 0
        || !identity_valid(Some(&s.nodes[usize::from(source - 1)].identity))
        || r.term != s.term
        || r.active > 1
        || frame.wrapping_sub(r.frame_counter) > 50
    {
        return false;
    }
    let i = usize::from(source - 1);
    if s.request_seq_seen[i] && !newer(r.request_seq, s.request_seq[i]) {
        return false;
    }
    s.request_seq_seen[i] = true;
    s.request_seq[i] = r.request_seq;
    s.request_active[i] = r.active != 0;
    s.request_deadline_ms[i] = now.wrapping_add(1500);
    true
}
pub fn speaker_request_active(s: Option<&Adaptive>, source: u8, now: u32) -> bool {
    let Some(s) = s else { return false };
    let s = &s.s;
    bit(source) != 0
        && s.members & bit(source) != 0
        && s.request_active[usize::from(source - 1)]
        && after(s.request_deadline_ms[usize::from(source - 1)], now)
}
pub fn expire(s: Option<&mut Adaptive>, now: u32) {
    let Some(s) = s else { return };
    expire_state(&mut s.s, now);
}
fn expire_state(s: &mut Snapshot, now: u32) {
    for i in 0..8 {
        if s.nodes[i].report_seen && !fresh(now, s.nodes[i].report_ms, 3000) {
            s.nodes[i].report_seen = false;
            s.nodes[i].direct_mask = 0;
            s.edge_live[i] = 0;
        }
    }
}
pub fn direct_graph(s: Option<&mut Adaptive>, id: u8, now: u32) -> u8 {
    let Some(s) = s else { return 0 };
    graph(&mut s.s, id, now)
}
fn graph(s: &mut Snapshot, id: u8, now: u32) -> u8 {
    if s.members & bit(id) == 0 {
        return 0;
    }
    expire_state(s, now);
    let i = usize::from(id - 1);
    let a = s.nodes[i];
    if !a.report_seen {
        return 0;
    }
    let mut edges = 0;
    for j in 0..8 {
        let edge = bit(j as u8 + 1);
        let b = s.nodes[j];
        if s.members & edge == 0
            || a.direct_mask & edge == 0
            || !b.report_seen
            || b.direct_mask & bit(id) == 0
        {
            s.edge_live[i] &= !edge;
            continue;
        }
        let q = a.quality[j].min(b.quality[i]);
        let threshold = if s.edge_live[i] & edge != 0 { 45 } else { 65 };
        if q >= threshold {
            edges |= edge;
            s.edge_live[i] |= edge;
        } else {
            s.edge_live[i] &= !edge;
        }
    }
    edges
}
pub fn route(s: Option<&mut Adaptive>, speaker: u8, now: u32, relay_mask: Option<&mut u8>) -> i32 {
    let Some(out) = relay_mask else { return -1 };
    *out = 0;
    let Some(s) = s else { return -1 };
    route_state(&mut s.s, speaker, now, out)
}
fn route_state(s: &mut Snapshot, speaker: u8, now: u32, out: &mut u8) -> i32 {
    *out = 0;
    if s.members & bit(speaker) == 0 {
        return -1;
    }
    let first = graph(s, speaker, now);
    let missing = s.members & !(first | bit(speaker));
    if missing == 0 {
        return 0;
    }
    let (mut best, mut best_count) = (0, 9);
    for mask in 1u16..=255 {
        let mask = mask as u8;
        let n = mask.count_ones() as i32;
        if mask & !first != 0 || n >= best_count {
            continue;
        }
        let mut covered = 0;
        for id in 1..=8 {
            if mask & bit(id) != 0 {
                covered |= graph(s, id, now);
            }
        }
        if covered & missing == missing {
            best = mask;
            best_count = n;
        }
    }
    if best_count == 9 {
        return -1;
    }
    *out = best;
    best_count
}
fn universal(s: &mut Snapshot, id: u8, now: u32) -> bool {
    graph(s, id, now) | bit(id) == s.members
}
fn weakest(s: &mut Snapshot, id: u8, now: u32) -> i32 {
    if route_state(s, id, now, &mut 0) < 0 {
        return -1000;
    }
    let mut min = 1000;
    for peer in 1..=8 {
        if s.members & bit(peer) == 0 || peer == id || graph(s, id, now) & bit(peer) == 0 {
            continue;
        }
        let a = s.nodes[usize::from(id - 1)];
        let b = s.nodes[usize::from(peer - 1)];
        let r = a.rssi[usize::from(peer - 1)].min(b.rssi[usize::from(id - 1)]);
        let q = a.quality[usize::from(peer - 1)].min(b.quality[usize::from(id - 1)]);
        min = min.min(i32::from(r) + i32::from(q) / 4);
    }
    min - if universal(s, id, now) { 0 } else { 12 }
}
pub fn candidate(s: Option<&mut Adaptive>, now: u32) -> u8 {
    let Some(s) = s else { return 0 };
    candidate_state(&mut s.s, now)
}
fn candidate_state(s: &mut Snapshot, now: u32) -> u8 {
    if s.local_id != s.leader_id || s.prepared || !after(now, s.leader_since_ms.wrapping_add(10000))
    {
        return 0;
    }
    let leader_score = weakest(s, s.leader_id, now);
    if leader_score == -1000 {
        s.better_id = 0;
        return 0;
    }
    let (mut best, mut score) = (0, leader_score + 7);
    for id in 1..=8 {
        if s.members & bit(id) == 0 || id == s.leader_id || !universal(s, id, now) {
            continue;
        }
        let value = weakest(s, id, now);
        if value > score {
            best = id;
            score = value;
        }
    }
    if best != s.better_id {
        s.better_id = best;
        s.better_since_ms = now;
    }
    if best != 0 && after(now, s.better_since_ms.wrapping_add(3000)) {
        best
    } else {
        0
    }
}
fn reset_term_reports(s: &mut Snapshot) {
    s.membership_revision_seen = false;
    s.membership_revision = 0;
    s.publish_revision = 0;
    s.request_seq = [0; 8];
    s.request_seq_seen = [false; 8];
    s.request_active = [false; 8];
    s.request_deadline_ms = [0; 8];
    s.edge_live = [0; 8];
    for n in &mut s.nodes {
        n.report_seen = false;
        n.seq_seen = false;
        n.direct_mask = 0;
        n.quality = [0; 8];
    }
}
pub fn prepare(
    s: Option<&mut Adaptive>,
    proposed: u8,
    frame: u32,
    slots: Option<&SlotMap>,
    now: u32,
    out: Option<&mut Handover>,
) -> bool {
    let (Some(s), Some(out)) = (s, out) else {
        return false;
    };
    let s = &mut s.s;
    if s.prepared
        || s.local_id != s.leader_id
        || s.term == 0
        || s.term == u32::MAX
        || proposed != candidate_state(s, now)
    {
        return false;
    }
    let Some(slots) = slots else { return false };
    if !slots_valid(slots, s.members) {
        return false;
    }
    // C accepts proposed == 0 when election returns 0. Production callers must check nonzero.
    s.pending = Handover {
        old_leader_id: s.leader_id,
        candidate_id: proposed,
        next_term: s.term + 1,
        switch_frame: frame.wrapping_add(160),
        members: s.members,
        slot_map: *slots,
    };
    s.ack_mask = bit(s.local_id);
    s.prepared = true;
    s.deadline_ms = now.wrapping_add(2400);
    s.recovery_deadline_ms = now.wrapping_add(5200);
    s.committed = false;
    s.transition_done = false;
    *out = s.pending;
    true
}
pub fn receive_prepare(
    s: Option<&mut Adaptive>,
    sender: u8,
    p: Option<&Handover>,
    frame: u32,
    now: u32,
) -> bool {
    let (Some(s), Some(p)) = (s, p) else {
        return false;
    };
    let s = &mut s.s;
    if sender != s.leader_id
        || sender == s.local_id
        || p.old_leader_id != sender
        || s.members & bit(p.candidate_id) == 0
        || p.candidate_id == sender
        || p.members != s.members
        || s.term == 0
        || s.term == u32::MAX
        || p.next_term != s.term + 1
    {
        return false;
    }
    if s.prepared {
        return s.pending == *p;
    }
    let lead = p.switch_frame.wrapping_sub(frame);
    if !(20..=160).contains(&lead) || !slots_valid(&p.slot_map, s.members) {
        return false;
    }
    let reachable = if s.local_id == p.candidate_id {
        universal(s, p.candidate_id, now)
    } else {
        identity_valid(Some(&s.nodes[usize::from(s.local_id - 1)].identity))
            && identity_valid(Some(&s.nodes[usize::from(p.candidate_id - 1)].identity))
            && graph(s, s.local_id, now) & bit(p.candidate_id) != 0
    };
    if !reachable {
        return false;
    }
    s.pending = *p;
    s.prepared = true;
    s.committed = false;
    s.deadline_ms = now.wrapping_add(2400);
    s.recovery_deadline_ms = now.wrapping_add(lead * 20 + 2000);
    s.transition_done = false;
    true
}
pub fn ack(s: Option<&mut Adaptive>, sender: u8, a: Option<&HandoverAck>, now: u32) -> bool {
    let (Some(s), Some(a)) = (s, a) else {
        return false;
    };
    let s = &mut s.s;
    if !s.prepared
        || s.committed
        || bit(s.local_id) == 0
        || s.members & bit(s.local_id) == 0
        || !after(s.deadline_ms, now)
        || s.members & bit(sender) == 0
        || sender != a.acknowledger_id
        || s.pending.old_leader_id != s.leader_id
        || s.term == u32::MAX
        || a.old_leader_id != s.leader_id
        || a.candidate_id != s.pending.candidate_id
        || a.next_term != s.term + 1
        || a.next_term != s.pending.next_term
        || a.switch_frame != s.pending.switch_frame
    {
        return false;
    }
    if s.local_id == s.leader_id {
        s.ack_mask |= bit(sender);
    }
    true
}
pub fn commit(s: Option<&mut Adaptive>, out: Option<&mut Handover>, now: u32) -> bool {
    let (Some(s), Some(out)) = (s, out) else {
        return false;
    };
    let s = &mut s.s;
    if !s.prepared
        || s.local_id != s.leader_id
        || s.ack_mask != s.members
        || !after(s.recovery_deadline_ms, now)
        || (!s.committed && !after(s.deadline_ms, now))
    {
        return false;
    }
    s.committed = true;
    *out = s.pending;
    true
}
pub fn receive_commit(
    s: Option<&mut Adaptive>,
    sender: u8,
    p: Option<&Handover>,
    now: u32,
) -> bool {
    let (Some(s), Some(p)) = (s, p) else {
        return false;
    };
    let s = &mut s.s;
    if sender != s.leader_id
        || !s.prepared
        || s.pending != *p
        || !after(s.recovery_deadline_ms, now)
    {
        return false;
    }
    s.committed = true;
    true
}
pub fn cancel(s: Option<&mut Adaptive>, sender: u8, p: Option<&Handover>) -> bool {
    let (Some(s), Some(p)) = (s, p) else {
        return false;
    };
    let s = &mut s.s;
    if !s.prepared || s.committed || sender != s.leader_id || s.pending != *p {
        return false;
    }
    s.prepared = false;
    s.better_id = 0;
    true
}
pub fn tick(s: Option<&mut Adaptive>, frame: u32, now: u32) -> bool {
    let Some(s) = s else { return false };
    let s = &mut s.s;
    if !s.prepared {
        return false;
    }
    if (!s.committed && s.local_id == s.leader_id && after(now, s.deadline_ms))
        || (!s.transition_done && after(now, s.recovery_deadline_ms))
    {
        s.prepared = false;
        s.better_id = 0;
        return false;
    }
    if !s.committed
        || s.transition_done
        || s.local_id != s.pending.candidate_id
        || !after(frame, s.pending.switch_frame)
    {
        return false;
    }
    s.leader_id = s.pending.candidate_id;
    s.term = s.pending.next_term;
    s.leader_since_ms = now;
    s.transition_done = true;
    s.better_id = 0;
    s.prepared = false;
    reset_term_reports(s);
    true
}
pub fn reconcile_sync(s: Option<&mut Adaptive>, sender: u8, sync: Option<&Sync>, now: u32) -> bool {
    let (Some(s), Some(sync)) = (s, sync) else {
        return false;
    };
    let s = &mut s.s;
    if !s.prepared
        || s.transition_done
        || s.local_id == s.pending.candidate_id
        || !after(s.recovery_deadline_ms, now)
        || sender != s.pending.candidate_id
        || sync.leader_id != sender
        || !(1..=8).contains(&sync.member_count)
        || sync.term != s.pending.next_term
        || sync.phase_us >= 20000
        || !identity_valid(Some(&sync.leader))
    {
        return false;
    }
    // Only candidate-zero proposals can reach here with an invalid index; excluded C UB.
    if bit(sender) == 0
        || sync.leader != s.nodes[usize::from(sender - 1)].identity
        || !after(sync.frame_counter, s.pending.switch_frame)
    {
        return false;
    }
    s.committed = true;
    s.leader_id = sender;
    s.term = sync.term;
    s.leader_since_ms = now;
    s.transition_done = true;
    s.better_id = 0;
    s.prepared = false;
    reset_term_reports(s);
    true
}
pub fn stamp_sync(sync: Option<&mut Sync>, relay: bool, frame: u32, phase: u16, age: u32) -> bool {
    let Some(sync) = sync else { return false };
    if !identity_valid(Some(&sync.leader))
        || bit(sync.leader_id) == 0
        || !(1..=8).contains(&sync.member_count)
        || sync.term == 0
        || phase >= 20000
        || (relay && (sync.relay_depth != 0 || age > 600))
    {
        return false;
    }
    sync.frame_counter = frame;
    sync.phase_us = phase;
    sync.relay_depth = u8::from(relay);
    true
}
fn length(t: u8) -> Option<usize> {
    match t {
        15 | 16 => Some(15),
        14 => Some(47),
        23 => Some(13),
        22 => Some(79),
        6 => Some(14),
        9 => Some(5),
        10 => Some(3),
        17 => Some(20),
        19 => Some(11),
        18 | 20 | 21 => Some(25),
        _ => None,
    }
}
fn priority(t: u8) -> u8 {
    match t {
        15 | 16 | 22 | 6 | 18..=21 => 2,
        23 => 1,
        _ => 0,
    }
}
fn read32(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}
fn read_identity(b: &[u8]) -> Identity {
    Identity {
        address_len: b[0],
        address: b[1..7].try_into().unwrap(),
    }
}
fn read_slots(b: &[u8]) -> SlotMap {
    SlotMap {
        slot_count: b[0],
        slot_ids: b[1..9].try_into().unwrap(),
        active_speaker_count: b[9],
        active_speaker_ids: b[10..12].try_into().unwrap(),
        relay_masks: b[12..14].try_into().unwrap(),
    }
}
fn read_sync(b: &[u8]) -> Sync {
    Sync {
        leader: read_identity(b),
        leader_id: b[7],
        member_count: b[8],
        term: read32(b, 9),
        frame_counter: read32(b, 13),
        phase_us: u16::from_le_bytes([b[17], b[18]]),
        relay_depth: b[19],
    }
}
#[allow(clippy::too_many_arguments)]
pub fn forward_enqueue(
    s: Option<&mut Adaptive>,
    h: Option<&Header>,
    payload: Option<&[u8]>,
    origin: Option<&Identity>,
    term: u32,
    joined_synced: bool,
    now: u32,
) -> bool {
    let (Some(s), Some(h), Some(payload)) = (s, h, payload) else {
        return false;
    };
    let t = h.packet_type;
    if !joined_synced
        || bit(s.s.local_id) == 0
        || h.version != 5
        || length(t) != Some(payload.len())
        || h.flags & 2 != 0
        || h.ttl != 2
        || usize::from(h.payload_len) != payload.len()
        || payload.len() > 200
        || (h.src_id == 0 && (t != 15 || !identity_valid(origin)))
        || (h.src_id != 0 && s.s.members & bit(h.src_id) == 0 && t != 15)
    {
        return false;
    }
    if t == 15 && origin.copied() != Some(read_identity(payload)) {
        return false;
    }
    if (s.s.local_id == s.s.leader_id && matches!(t, 15 | 23 | 19 | 14))
        || (h.src_id == s.s.leader_id && matches!(t, 23 | 14))
    {
        return false;
    }
    if t == 16 {
        let local = &s.s.nodes[usize::from(s.s.local_id - 1)].identity;
        if identity_valid(Some(local)) && read_identity(payload) == *local {
            return false;
        }
    }
    if t == 22 {
        let mut m = Membership {
            term: read32(payload, 0),
            revision: read32(payload, 4),
            leader_id: payload[8],
            slot_map: read_slots(&payload[9..]),
            ..Membership::default()
        };
        for i in 0..8 {
            m.identities[i] = read_identity(&payload[23 + i * 7..]);
        }
        if !control_authorized(Some(s), t, h.src_id)
            || m.term != s.s.term
            || m.leader_id != s.s.leader_id
            || term != m.term
            || !membership_valid(Some(&m), None)
        {
            return false;
        }
    }
    if t == 23
        && (!control_authorized(Some(s), t, h.src_id)
            || !identity_valid(Some(&s.s.nodes[usize::from(h.src_id - 1)].identity))
            || read32(payload, 0) != s.s.term
            || term != read32(payload, 0)
            || payload[12] > 1)
    {
        return false;
    }
    if t == 17 {
        let sync = read_sync(payload);
        if !identity_valid(Some(&sync.leader))
            || bit(sync.leader_id) == 0
            || !(1..=8).contains(&sync.member_count)
            || sync.term == 0
            || sync.phase_us >= 20000
            || sync.relay_depth != 0
        {
            return false;
        }
    }
    let s = &mut s.s;
    for k in &s.cache {
        if k.used
            && fresh(now, k.seen_ms, 1000)
            && k.packet_type == t
            && k.src_id == h.src_id
            && k.seq == h.seq
            && k.term == term
            && (h.src_id != 0 || origin == Some(&k.origin))
        {
            return false;
        }
    }
    if s.queue_count == 8 {
        let mut replaced = false;
        if priority(t) != 0 {
            for i in 0..s.queue_count {
                let q = s.queue[usize::from((s.queue_head + i) % 8)];
                if priority(q.header.packet_type) < priority(t) && !fresh(now, q.enqueued_ms, 1000)
                {
                    for j in i..s.queue_count - 1 {
                        s.queue[usize::from((s.queue_head + j) % 8)] =
                            s.queue[usize::from((s.queue_head + j + 1) % 8)];
                    }
                    s.queue_count -= 1;
                    replaced = true;
                    break;
                }
            }
        }
        if !replaced {
            return false;
        }
    }
    let index = s
        .cache
        .iter()
        .position(|k| !k.used || !fresh(now, k.seen_ms, 1000))
        .unwrap_or_else(|| {
            let (mut index, mut oldest) = (0, now.wrapping_sub(s.cache[0].seen_ms));
            for i in 1..32 {
                let age = now.wrapping_sub(s.cache[i].seen_ms);
                if age > oldest {
                    index = i;
                    oldest = age;
                }
            }
            index
        });
    s.cache[index] = ForwardKey {
        packet_type: t,
        src_id: h.src_id,
        seq: h.seq,
        term,
        seen_ms: now,
        origin: origin.copied().unwrap_or_default(),
        used: true,
    };
    let q = &mut s.queue[usize::from((s.queue_head + s.queue_count) % 8)];
    q.header = *h;
    q.header.ttl = 1;
    q.header.flags |= 2;
    q.payload_len = h.payload_len;
    q.enqueued_ms = now;
    q.payload[..payload.len()].copy_from_slice(payload);
    s.queue_count += 1;
    true
}
pub fn forward_dequeue(
    s: Option<&mut Adaptive>,
    out: Option<&mut ForwardPacket>,
    frame: u32,
    phase: u16,
    age: u32,
) -> bool {
    let (Some(s), Some(out)) = (s, out) else {
        return false;
    };
    let s = &mut s.s;
    if s.queue_count == 0 {
        return false;
    }
    *out = s.queue[usize::from(s.queue_head)];
    s.queue_head = (s.queue_head + 1) % 8;
    s.queue_count -= 1;
    if out.header.packet_type == 17 {
        let mut sync = read_sync(&out.payload);
        if !stamp_sync(Some(&mut sync), true, frame, phase, age) {
            return false;
        }
        out.payload[13..17].copy_from_slice(&sync.frame_counter.to_le_bytes());
        out.payload[17..19].copy_from_slice(&sync.phase_us.to_le_bytes());
        out.payload[19] = sync.relay_depth;
    }
    true
}

#[cfg(test)]
mod publication_revision_tests {
    use super::*;

    #[test]
    fn existing_revision_allocator_wraps_only_for_initialized_leader() {
        let mut adaptive = Adaptive::default();
        init(Some(&mut adaptive), 1, 1, 1, 1, 0);
        adaptive.s.publish_revision = u32::MAX;
        assert_eq!(next_membership_revision(Some(&mut adaptive)), 0);
        assert_eq!(next_membership_revision(Some(&mut adaptive)), 1);
        adaptive.s.local_id = 2;
        assert_eq!(next_membership_revision(Some(&mut adaptive)), 0);
        assert_eq!(adaptive.s.publish_revision, 1);
        adaptive.s.local_id = 1;
        adaptive.s.term = 0;
        assert_eq!(next_membership_revision(Some(&mut adaptive)), 0);
        assert_eq!(adaptive.s.publish_revision, 1);
        assert_eq!(next_membership_revision(None), 0);
    }
}
