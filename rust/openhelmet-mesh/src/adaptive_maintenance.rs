//! Bounded maintenance stages; retry clocks belong to the mesh owner, not task cadence.
use crate::{adaptive as a, core::SlotMap};

#[derive(Default)]
pub struct RetryClocks {
    prepare: u32,
    commit: u32,
}
impl RetryClocks {
    /// One copied proposal per ordered stage. Clock advances on attempt, not success.
    pub fn stage(
        &mut self,
        adaptive: &mut a::Adaptive,
        stage: u8,
        now: u32,
        frame: u32,
        slots: &SlotMap,
    ) -> Option<(u8, a::Handover)> {
        let (local, leader, _, _) = adaptive.session();
        if local == 0 || local != leader {
            return None;
        }
        let (prepared, committed, done, deadline, recovery, pending) = adaptive.transition();
        let mut proposal = a::Handover::default();
        match stage {
            1 if !prepared => {
                let candidate = a::candidate(Some(adaptive), now);
                if candidate != 0
                    && a::prepare(
                        Some(adaptive),
                        candidate,
                        frame,
                        Some(slots),
                        now,
                        Some(&mut proposal),
                    )
                {
                    self.prepare = now;
                    return Some((1, proposal));
                }
            }
            2 if prepared && !committed => {
                if a::commit(Some(adaptive), Some(&mut proposal), now) {
                    self.commit = now;
                    return Some((2, proposal));
                }
                if now.wrapping_sub(self.prepare) >= 180 && (deadline.wrapping_sub(now) as i32) > 0
                {
                    self.prepare = now;
                    return Some((1, pending));
                }
            }
            3 if committed
                && !done
                && (recovery.wrapping_sub(now) as i32) >= 0
                && now.wrapping_sub(self.commit) >= 180 =>
            {
                self.commit = now;
                return Some((2, pending));
            }
            _ => {}
        }
        None
    }
}
