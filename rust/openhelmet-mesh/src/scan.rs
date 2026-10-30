//! Bounded task-owned scan episode. No peers, packets or session mirrors.
use crate::core::address_compare;
use crate::{adaptive as a, control_admission as c};

/// Cold discovery deliberately does not require an existing membership binding.
/// Validate the complete copied envelope before consuming any episode state.
pub fn discovery_valid(header: &a::Header, sync: &a::Sync, sender: &[u8; 6]) -> bool {
    c::classify(
        header.packet_type,
        header.payload_len,
        header.flags,
        header.ttl,
    ) == c::Admission::Envelope(c::Control::Sync)
        && (1..=8).contains(&header.src_id)
        && sync.relay_depth == u8::from(header.flags & 2 != 0)
        && sync.leader.address_len == 6
        && a::identity_valid(Some(&sync.leader))
        && (1..=8).contains(&sync.member_count)
        && sync.leader_id == header.src_id
        && sync.term != 0
        && sync.phase_us < 20_000
        && (header.flags & 2 != 0 || sync.leader.address == *sender)
}

pub fn lower_valid(header: &a::Header, origin: &a::Identity, sender: &[u8; 6]) -> bool {
    c::classify(
        header.packet_type,
        header.payload_len,
        header.flags,
        header.ttl,
    ) == c::Admission::Envelope(c::Control::Join)
        && header.src_id == 0
        && header.flags == 0
        && origin.address_len == 6
        && a::identity_valid(Some(origin))
        && origin.address == *sender
}

// Two words deliberately keep the existing owner alignment at four bytes.
#[derive(Default)]
struct Micros([u32; 2]);
impl Micros {
    fn set(&mut self, v: i64) {
        self.0 = [v as u32, (v as u64 >> 32) as u32];
    }
    fn get(&self) -> i64 {
        (u64::from(self.0[0]) | (u64::from(self.0[1]) << 32)) as i64
    }
}
#[derive(Debug, PartialEq, Eq)]
pub enum Effect {
    None,
    Beacon(u32),
    Activate,
}
// All fields are zero-valid and Default is the inactive/reset construction.
#[derive(Default)]
pub struct Transaction {
    pending: bool,
    lower: bool,
    awaiting: bool,
    generation: u32,
    token: u32,
    start: Micros,
    beacon: Micros,
    offered: Micros,
}
impl Transaction {
    pub fn begin(&mut self, now: i64, generation: u32) {
        self.pending = true;
        self.lower = false;
        self.awaiting = false;
        self.generation = generation;
        self.token = self.token.wrapping_add(1);
        self.start.set(now);
        self.beacon.set(0);
    }
    fn current(&self, generation: u32) -> bool {
        self.pending && self.generation == generation
    }
    pub fn cancel(&mut self, generation: u32) {
        if self.generation == generation {
            self.pending = false;
            self.awaiting = false;
        }
    }
    pub fn discover(&mut self, generation: u32) -> bool {
        if !self.current(generation) {
            return false;
        }
        self.cancel(generation);
        true
    }
    pub fn lower(&mut self, now: i64, generation: u32, remote: &[u8; 6], local: &[u8; 6]) -> bool {
        if !self.current(generation) || address_compare(Some(remote), Some(local), 6) >= 0 {
            return false;
        }
        self.lower = true;
        self.start.set(now);
        true
    }
    pub fn beacon(&mut self, now: i64, generation: u32) -> Effect {
        if self.current(generation)
            && !self.awaiting
            && now.saturating_sub(self.beacon.get()) > 300_000
        {
            self.token = self.token.wrapping_add(1);
            self.awaiting = true;
            self.offered.set(now);
            Effect::Beacon(self.token)
        } else {
            Effect::None
        }
    }
    pub fn result(&mut self, generation: u32, token: u32, success: bool) -> bool {
        if !self.current(generation) || !self.awaiting || self.token != token {
            return false;
        }
        self.awaiting = false;
        if success {
            self.beacon.set(self.offered.get());
        }
        true
    }
    pub fn elect(&mut self, now: i64, generation: u32) -> Effect {
        let timeout = if self.lower { 3_000_000 } else { 2_000_000 };
        if self.current(generation) && now.saturating_sub(self.start.get()) > timeout {
            self.cancel(generation);
            Effect::Activate
        } else {
            Effect::None
        }
    }
}
