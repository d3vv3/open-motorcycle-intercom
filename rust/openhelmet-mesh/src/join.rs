//! Task-owned JOINING transaction; wider session context is call-local only.
use crate::membership as m;

// Explicit word storage preserves alignment-4 opaque owner ABI on all targets.
#[derive(Default)]
struct Micros([u32; 2]);
impl Micros {
    fn set(&mut self, value: i64) {
        self.0 = [value as u32, (value as u64 >> 32) as u32];
    }
    fn get(&self) -> i64 {
        (u64::from(self.0[0]) | (u64::from(self.0[1]) << 32)) as i64
    }
}
#[derive(Debug, PartialEq, Eq)]
pub enum Effect {
    None,
    Send(u32),
    Expire,
}
#[derive(Default)]
pub struct Transaction {
    pending: bool,
    awaiting: bool,
    generation: u32,
    token: u32,
    deadline: Micros,
    previous: Micros,
    successes: u32,
}
impl Transaction {
    pub fn begin(&mut self, now: i64, generation: u32) {
        self.pending = true;
        self.awaiting = false;
        self.generation = generation;
        self.token = self.token.wrapping_add(1);
        self.deadline.set(now.saturating_add(5_000_000));
        self.previous.set(0);
        self.successes = 0;
    }
    pub fn cancel(&mut self, generation: u32) {
        if self.generation == generation {
            self.pending = false;
            self.awaiting = false;
        }
    }
    pub fn poll(&mut self, now: i64, generation: u32) -> Effect {
        if !self.pending || generation != self.generation {
            return Effect::None;
        }
        if now >= self.deadline.get() {
            self.cancel(generation);
            return Effect::Expire;
        }
        if !self.awaiting && now.saturating_sub(self.previous.get()) > 500_000 {
            self.previous.set(now);
            self.token = self.token.wrapping_add(1);
            self.awaiting = true;
            return Effect::Send(self.token);
        }
        Effect::None
    }
    pub fn result(&mut self, generation: u32, token: u32, success: bool) -> Option<u32> {
        if !self.pending || !self.awaiting || self.generation != generation || self.token != token {
            return None;
        }
        self.awaiting = false;
        if success {
            self.successes += 1;
        }
        Some(self.successes)
    }
    pub fn ack(
        &mut self,
        generation: u32,
        context: &m::Snapshot,
        sender: u8,
        ack: m::JoinAck,
    ) -> bool {
        if !self.pending || generation != self.generation {
            return false;
        }
        let event = m::Event {
            sender_id: sender,
            payload_valid: true,
            data: m::EventData::JoinAck(ack),
        };
        if m::reduce(Some(context), Some(&event)).action != m::Action::ActivateParticipant {
            return false;
        }
        self.cancel(generation);
        true
    }
}
