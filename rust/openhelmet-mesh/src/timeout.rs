//! Participant projection timeout batch and one-shot leader-loss authorization.
use crate::{
    adaptive::{Adaptive, Identity},
    core::node_bit,
};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Row {
    pub last_seen: [u8; 8],
    pub identity: Identity,
    pub id: u8,
    pub active: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    pub now_ms: i64,
    pub generation: u32,
    pub active: bool,
    pub participant: bool,
    pub local: u8,
    pub leader: u8,
    pub rows: [Row; 8],
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Effect {
    pub rows: u8,
    pub recover: bool,
}
pub fn plan(s: &Adaptive, i: Input) -> Option<Effect> {
    let (local, leader, members, term) = s.session();
    if !i.active
        || !i.participant
        || local == leader
        || term == 0
        || local != i.local
        || leader != i.leader
        || members & node_bit(local) == 0
        || members & node_bit(leader) == 0
    {
        return None;
    }
    let mut seen = 0;
    let mut p = Effect::default();
    for (n, r) in i.rows.iter().enumerate() {
        if !r.active {
            continue;
        }
        let bit = node_bit(r.id);
        if bit == 0
            || seen & bit != 0
            || members & bit == 0
            || r.identity.address_len != 6
            || s.member_identity(r.id) != r.identity
        {
            return None;
        }
        seen |= bit;
        let last = i64::from_le_bytes(r.last_seen);
        if r.id != local && last >= 0 && i.now_ms >= last && i.now_ms - last > 3000 {
            p.rows |= 1 << n;
            p.recover |= r.id == leader;
        }
    }
    Some(p)
}
// Store timestamps as bytes so the task owner retains its 4-byte alignment.
#[derive(Clone, Debug, Default)]
pub struct Transaction {
    token: u32,
    generation: u32,
    term: u32,
    local: u8,
    leader: u8,
    members: u8,
    now: [u8; 8],
    rows: [Row; 8],
}
impl Transaction {
    pub fn cancel(&mut self) {
        self.token = 0;
    }
    pub fn prepare(&mut self, s: &Adaptive, i: Input, token: u32) -> Option<Effect> {
        self.cancel();
        let p = plan(s, i)?;
        if token != 0 {
            let (local, leader, members, term) = s.session();
            *self = Self {
                token,
                generation: i.generation,
                term,
                local,
                leader,
                members,
                now: i.now_ms.to_le_bytes(),
                rows: i.rows,
            };
        }
        Some(p)
    }
    pub fn valid(&self, s: &Adaptive, i: Input, token: u32) -> bool {
        token != 0
            && token == self.token
            && i.generation == self.generation
            && s.session() == (self.local, self.leader, self.members, self.term)
            && i.rows == self.rows
            && i.now_ms >= i64::from_le_bytes(self.now)
            && plan(s, i).is_some()
    }
    pub fn consume(&mut self, s: &Adaptive, i: Input, token: u32) -> bool {
        let valid = self.valid(s, i, token) && plan(s, i).is_some_and(|p| p.recover);
        if token == self.token {
            self.cancel();
        }
        valid
    }
}
