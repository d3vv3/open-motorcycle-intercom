//! Bounded coordinator ordinary-member removal. Empty direct LEAVE has no wire
//! incarnation: same-MAC/same-ID delayed replay after rejoin is indistinguishable.
use crate::{
    adaptive::{self as a, Adaptive, Identity},
    core::{SlotMap, node_bit},
};
/// Complete already-committed projection cleanup even if stop is announced.
/// Publication admission (op zero) follows blocking cleanup and precedes every
/// external send. Callback is last; no owner/projection access follows it.
pub fn committed_effects(mut effect: impl FnMut(u32) -> bool) {
    for op in 1..=3 {
        effect(op);
    }
    for op in 4..=8 {
        if !effect(0) {
            return;
        }
        effect(op);
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    pub generation: u32,
    pub active: bool,
    /// 1=bound direct LEAVE, 2=projection timeout.
    pub reason: u8,
    pub id: u8,
    pub identity: Identity,
    pub map: SlotMap,
    pub last_seen_ms: i64,
    pub now_ms: i64,
    pub peers: crate::peer_evidence::Rows,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Effect {
    pub id: u8,
    pub row: u8,
    pub map: SlotMap,
}
pub fn plan(s: &Adaptive, i: Input) -> Option<Effect> {
    let (local, leader, members, term) = s.session();
    let bit = node_bit(i.id);
    if !i.active
        || term == 0
        || local != leader
        || members & node_bit(local) == 0
        || bit == 0
        || i.id == local
        || i.id == leader
        || members & bit == 0
        || i.identity.address_len != 6
        || !a::identity_valid(Some(&i.identity))
        || s.member_identity(i.id) != i.identity
    {
        return None;
    }
    let mut row = crate::peer_evidence::ABSENT;
    for (n, r) in i.peers.iter().enumerate() {
        if r.active == 0 {
            continue;
        }
        let same_id = r.id == i.id;
        let same_mac = r.mac == i.identity.address;
        if same_id || same_mac {
            if r.active != 1 || !same_id || !same_mac || row != crate::peer_evidence::ABSENT {
                return None;
            }
            row = n as u8;
        }
    }
    match i.reason {
        1 => (),
        2 if row != crate::peer_evidence::ABSENT
            && i.peers[usize::from(row)].last_seen() == i.last_seen_ms
            && i.last_seen_ms >= 0
            && i.now_ms >= i.last_seen_ms
            && i.now_ms - i.last_seen_ms > 3000 => {}
        _ => return None,
    }
    if !a::slots_valid(&i.map, members) {
        return None;
    }
    let mut seen = 0;
    for (slot, id) in i.map.slot_ids.iter().copied().enumerate() {
        if id == 0 {
            continue;
        }
        let b = node_bit(id);
        if slot >= usize::from(i.map.slot_count) || b == 0 || seen & b != 0 {
            return None;
        }
        seen |= b;
    }
    if seen != members {
        return None;
    }
    let mut map = i.map;
    for id in &mut map.slot_ids {
        if *id == i.id {
            *id = 0;
        }
    }
    while map.slot_count > 0 && map.slot_ids[usize::from(map.slot_count - 1)] == 0 {
        map.slot_count -= 1;
    }
    map.active_speaker_count = 0;
    map.active_speaker_ids = [0; 2];
    map.relay_masks = [0; 2];
    for n in 0..usize::from(i.map.active_speaker_count) {
        if i.map.active_speaker_ids[n] == i.id {
            continue;
        }
        let out = usize::from(map.active_speaker_count);
        map.active_speaker_ids[out] = i.map.active_speaker_ids[n];
        map.relay_masks[out] = i.map.relay_masks[n] & !bit;
        map.active_speaker_count += 1;
    }
    Some(Effect { id: i.id, row, map })
}
#[derive(Clone, Debug, Default)]
pub struct Transaction {
    token: u32,
    generation: u32,
    reason: u8,
    id: u8,
    identity: Identity,
    map: SlotMap,
    last_seen: [u8; 8],
    now: [u8; 8],
    term: u32,
    local: u8,
    members: u8,
}
impl Transaction {
    pub fn cancel(&mut self) {
        self.token = 0;
    }
    pub fn prepare(&mut self, s: &Adaptive, i: Input, token: u32) -> Option<Effect> {
        self.cancel();
        if token == 0 {
            return None;
        }
        let p = plan(s, i)?;
        let (local, _, members, term) = s.session();
        *self = Self {
            token,
            generation: i.generation,
            reason: i.reason,
            id: i.id,
            identity: i.identity,
            map: i.map,
            last_seen: i.last_seen_ms.to_le_bytes(),
            now: i.now_ms.to_le_bytes(),
            term,
            local,
            members,
        };
        Some(p)
    }
    pub fn commit(&mut self, s: &mut Adaptive, i: Input, token: u32) -> Option<Effect> {
        if token == 0 || token != self.token {
            return None;
        }
        self.cancel();
        let (local, leader, members, term) = s.session();
        // now may advance, but the observed liveness evidence must not refresh.
        if i.generation != self.generation
            || i.reason != self.reason
            || i.id != self.id
            || i.identity != self.identity
            || i.map != self.map
            || i.last_seen_ms.to_le_bytes() != self.last_seen
            || term != self.term
            || local != self.local
            || leader != local
            || members != self.members
            || i.now_ms < i64::from_le_bytes(self.now)
        {
            return None;
        }
        let p = plan(s, i)?;
        if !a::set_member(Some(s), i.id, None) {
            return None;
        }
        Some(p)
    }
}
