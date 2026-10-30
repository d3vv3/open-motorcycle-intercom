//! Generic ACTIVE cadence. Poll one ordered stage at a time; never batch effects.
#[derive(Clone, Copy)]
pub enum Stage {
    Keepalive,
    Status,
    Membership,
}
#[derive(Clone, Copy, Default)]
pub struct Cadence {
    // Byte representation preserves the enclosing opaque owner's align-4 ABI.
    keepalive: [u8; 8],
    status: [u8; 8],
    membership: u32,
}
impl Cadence {
    pub fn poll(&mut self, stage: Stage, now: i64) -> bool {
        let (clock, interval) = match stage {
            Stage::Keepalive => (&mut self.keepalive, 500_000),
            Stage::Status => (&mut self.status, 1_000_000),
            Stage::Membership => {
                return ((now / 1000) as u32).wrapping_sub(self.membership) >= 1000;
            }
        };
        if now.wrapping_sub(i64::from_le_bytes(*clock)) <= interval {
            return false;
        }
        *clock = now.to_le_bytes();
        true
    }
    pub fn membership_success(&mut self, fresh_ms: u32) {
        self.membership = fresh_ms;
    }
    pub fn reset_membership(&mut self) {
        self.membership = 0;
    }
}
