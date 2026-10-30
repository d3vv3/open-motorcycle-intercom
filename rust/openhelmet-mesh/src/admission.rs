//! Coordinator admission against live Adaptive bindings and a copied operational map.
//! Preparation never mutates Adaptive. Only bounded transaction metadata is retained.
use crate::{
    adaptive::{self as a, Adaptive, Identity},
    core::{SlotMap, first_free_node_id, node_bit},
};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    pub generation: u32,
    pub active: bool,
    pub source: u8,
    pub origin: Identity,
    pub target: Identity,
    pub map: SlotMap,
    pub local_slot: i8,
    pub peers: crate::peer_evidence::Rows,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Assignment {
    /// 1: existing binding; 2: new binding.
    pub kind: u8,
    pub id: u8,
    pub slot: u8,
    pub row: u8,
    pub map: SlotMap,
}
pub fn plan(s: &Adaptive, input: Input) -> Option<Assignment> {
    let mut p = allocation(s, input)?;
    let (local, _, members, _) = s.session();
    let local_identity = s.member_identity(local);
    let local_slot = usize::try_from(input.local_slot).ok()?;
    if input.map.slot_ids.get(local_slot).copied() != Some(local) {
        return None;
    }
    let mut projected = node_bit(local);
    let mut self_seen = false;
    let mut row = None;
    let mut free = None;
    for (n, r) in input.peers.iter().enumerate() {
        if r.active == 0 {
            free = free.or(Some(n));
            continue;
        }
        if r.active != 1 {
            return None;
        }
        let bit = node_bit(r.id);
        let identity = s.member_identity(r.id);
        let local_mac = r.mac == local_identity.address;
        if r.id == local || local_mac {
            if self_seen || bit == 0 || r.id != local || !local_mac || r.slot != input.local_slot {
                return None;
            }
            self_seen = true;
            continue;
        }
        let slot = usize::try_from(r.slot).ok()?;
        if bit == 0
            || projected & bit != 0
            || input.map.slot_ids.get(slot).copied() != Some(r.id)
            || identity.address_len != 6
            || identity.address != r.mac
        {
            return None;
        }
        projected |= bit;
        if r.mac == input.origin.address || r.id == p.id {
            if r.mac != input.origin.address
                || r.id != p.id
                || slot != usize::from(p.slot)
                || row.is_some()
            {
                return None;
            }
            row = Some(n);
        }
    }
    let expected = if p.kind == 1 && row.is_none() {
        members & !node_bit(p.id)
    } else {
        members
    };
    if projected != expected {
        return None;
    }
    p.row = row.or(free)? as u8;
    Some(p)
}
fn allocation(s: &Adaptive, input: Input) -> Option<Assignment> {
    let (local, leader, members, term) = s.session();
    if !input.active
        || term == 0
        || local != leader
        || node_bit(local) & members == 0
        || input.source != 0
        || input.origin.address_len != 6
        || !a::identity_valid(Some(&input.origin))
        || input.origin == input.target
        || input.target.address_len != 6
        || !a::identity_valid(Some(&input.target))
        || input.target != s.member_identity(leader)
    {
        return None;
    }
    let mut existing = 0;
    for id in 1..=8 {
        if members & node_bit(id) == 0 {
            continue;
        }
        let identity = s.member_identity(id);
        if identity.address_len != 6 || !a::identity_valid(Some(&identity)) {
            return None;
        }
        for other in 1..id {
            if members & node_bit(other) != 0 && identity == s.member_identity(other) {
                return None;
            }
        }
        if identity == input.origin {
            existing = id;
        }
    }
    let mut map = input.map;
    if !a::slots_valid(&map, members) {
        return None;
    }
    let mut seen = 0;
    let mut existing_slot = 0;
    for (slot, id) in map.slot_ids.iter().copied().enumerate() {
        if id == 0 {
            continue;
        }
        let bit = node_bit(id);
        if slot >= usize::from(map.slot_count) || bit == 0 || seen & bit != 0 {
            return None;
        }
        seen |= bit;
        if id == existing {
            existing_slot = slot as u8;
        }
    }
    if seen != members {
        return None;
    }
    if existing != 0 {
        return Some(Assignment {
            kind: 1,
            id: existing,
            slot: existing_slot,
            row: 0,
            map,
        });
    }
    let id = first_free_node_id(members);
    if id == 0 {
        return None;
    }
    let slot = map.slot_ids.iter().position(|id| *id == 0)?;
    map.slot_ids[slot] = id;
    map.slot_count = map.slot_count.max(slot as u8 + 1);
    Some(Assignment {
        kind: 2,
        id,
        slot: slot as u8,
        row: 0,
        map,
    })
}
#[derive(Clone, Debug, Default)]
pub struct Transaction {
    token: u32,
    generation: u32,
    term: u32,
    local: u8,
    members: u8,
    kind: u8,
    id: u8,
    slot: u8,
    origin: Identity,
}
impl Transaction {
    pub fn cancel(&mut self) {
        self.token = 0;
    }
    pub fn prepare(&mut self, s: &Adaptive, input: Input, token: u32) -> Option<Assignment> {
        self.cancel();
        if token == 0 {
            return None;
        }
        let p = plan(s, input)?;
        let (local, _, members, term) = s.session();
        *self = Self {
            token,
            generation: input.generation,
            term,
            local,
            members,
            kind: p.kind,
            id: p.id,
            slot: p.slot,
            origin: input.origin,
        };
        Some(p)
    }
    pub fn commit(&mut self, s: &mut Adaptive, input: Input, token: u32) -> Option<Assignment> {
        if token == 0 || token != self.token {
            return None;
        }
        self.token = 0;
        let (local, leader, members, term) = s.session();
        if input.generation != self.generation
            || term != self.term
            || local != self.local
            || leader != local
            || members != self.members
            || input.origin != self.origin
        {
            return None;
        }
        let p = plan(s, input)?;
        if (p.kind, p.id, p.slot) != (self.kind, self.id, self.slot) {
            return None;
        }
        if p.kind == 2 && !a::set_member(Some(s), p.id, Some(&input.origin)) {
            return None;
        }
        Some(p)
    }
}
