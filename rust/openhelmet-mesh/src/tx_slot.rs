//! Authoritative installed TX-slot record and reusable compatibility policy.
//! Live times use modulo-2^64 half-range ordering. Exactly half-range is
//! ambiguous and fails closed; zero only disables the deadline admission gate.
pub const RETRY_US: i64 = 200;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct State {
    pub completed: bool,
    pub retried: bool,
    pub assigned_index: i8,
    pub retry_due_us: i64,
}
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Check {
    Ignored,
    Invalid,
    Early,
    Expired,
    Attempt,
}
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SendResult {
    Submitted,
    Empty,
    Busy,
    Error,
}
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Finished,
    Recovered,
    Retry,
    Missed,
    Exhausted,
}

pub fn deadline_passed(deadline: i64, now: i64) -> bool {
    let elapsed = now.wrapping_sub(deadline);
    deadline != 0 && (elapsed > 0 || elapsed == i64::MIN)
}
impl State {
    pub fn init() -> Self {
        Self {
            completed: true,
            retried: false,
            assigned_index: -1,
            retry_due_us: 0,
        }
    }
    pub fn install(index: i8) -> Self {
        Self {
            completed: false,
            retried: false,
            assigned_index: index,
            retry_due_us: 0,
        }
    }
    pub fn complete(&mut self) {
        self.completed = true;
    }
    #[allow(clippy::too_many_arguments)]
    pub fn check(
        &mut self,
        active: bool,
        stopping: bool,
        index: i8,
        generation: u32,
        current_generation: u32,
        start: i64,
        deadline: i64,
        now: i64,
    ) -> Check {
        if self.completed {
            return Check::Ignored;
        }
        if !active
            || stopping
            || index < 0
            || index != self.assigned_index
            || generation != current_generation
        {
            return Check::Invalid;
        }
        let start_delta = now.wrapping_sub(start);
        let retry_delta = now.wrapping_sub(self.retry_due_us);
        if start_delta == i64::MIN || (self.retried && retry_delta == i64::MIN) {
            self.complete();
            return Check::Expired;
        }
        if start_delta < 0 || (self.retried && retry_delta < 0) {
            return Check::Early;
        }
        if deadline_passed(deadline, now) {
            self.complete();
            return Check::Expired;
        }
        Check::Attempt
    }
    pub fn rollover(
        &mut self,
        generation: u32,
        current_generation: u32,
        deadline: i64,
        now: i64,
    ) -> bool {
        if self.completed || generation != current_generation || !deadline_passed(deadline, now) {
            return false;
        }
        self.complete();
        true
    }
    pub fn finish(&mut self, result: SendResult, now: i64, deadline: i64, guard: i64) -> Outcome {
        let remaining = deadline.wrapping_sub(now);
        // Zero is not a retry budget. Reject invalid/ambiguous margins without
        // overflowing either guard addition or clock arithmetic.
        if result == SendResult::Busy
            && deadline != 0
            && remaining >= 0
            && guard >= 0
            && guard
                .checked_add(RETRY_US)
                .is_some_and(|margin| remaining >= margin)
        {
            self.retried = true;
            self.retry_due_us = now.wrapping_add(RETRY_US);
            return Outcome::Retry;
        }
        self.complete();
        match result {
            SendResult::Submitted => {
                if self.retried {
                    Outcome::Recovered
                } else {
                    Outcome::Finished
                }
            }
            SendResult::Empty => Outcome::Finished,
            _ if self.retried && (result == SendResult::Busy || deadline_passed(deadline, now)) => {
                Outcome::Exhausted
            }
            _ => Outcome::Missed,
        }
    }
}

/// Private shipping record. Callers serialize the entire install/service transaction;
/// no reference to this object may survive platform execution.
pub struct Slot {
    state: State,
    start: i64,
    deadline: i64,
    generation: u32,
}
impl Slot {
    pub const fn new() -> Self {
        Self {
            state: State {
                completed: true,
                retried: false,
                assigned_index: -1,
                retry_due_us: 0,
            },
            start: 0,
            deadline: 0,
            generation: 0,
        }
    }
    pub fn install(&mut self, index: i8, generation: u32, start: i64, deadline: i64) {
        *self = Self {
            state: State::install(index),
            start,
            deadline,
            generation,
        };
    }
    pub fn observe(&self) -> Observation {
        Observation {
            start: self.start,
            deadline: self.deadline,
            retry_due: self.state.retry_due_us,
            generation: self.generation,
            check: Check::Ignored as i32,
            completed: self.state.completed as u8,
            retried: self.state.retried as u8,
        }
    }
    pub fn check(
        &mut self,
        active: bool,
        stopping: bool,
        index: i8,
        current: u32,
        now: i64,
    ) -> Observation {
        let check = self.state.check(
            active,
            stopping,
            index,
            self.generation,
            current,
            self.start,
            self.deadline,
            now,
        );
        Observation {
            check: check as i32,
            ..self.observe()
        }
    }
    /// 0 no miss, 1 missed, 2 missed after retry.
    pub fn rollover(&mut self, current: u32, now: i64) -> u8 {
        if self
            .state
            .rollover(self.generation, current, self.deadline, now)
        {
            1 + self.state.retried as u8
        } else {
            0
        }
    }
    pub fn complete(&mut self, generation: u32) {
        if generation == self.generation {
            self.state.complete();
        }
    }
    pub fn finish(&mut self, generation: u32, result: SendResult, now: i64, guard: i64) -> Outcome {
        if generation != self.generation || self.state.completed {
            return Outcome::Finished;
        }
        self.state.finish(result, now, self.deadline, guard)
    }
}
impl Default for Slot {
    fn default() -> Self {
        Self::new()
    }
}
/// Detached diagnostics; never accepted as replacement storage.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Observation {
    pub start: i64,
    pub deadline: i64,
    pub retry_due: i64,
    pub generation: u32,
    pub check: i32,
    pub completed: u8,
    pub retried: u8,
}

#[cfg(test)]
mod tests {
    #[test]
    fn installed_record_reset_generation_and_detached_observation() {
        let mut slot = super::Slot::new();
        slot.install(1, 7, 100, 1000);
        let mut detached = slot.observe();
        detached.deadline = 0;
        assert_eq!(detached.deadline, 0);
        assert_eq!(
            slot.check(true, false, 1, 8, 100).check,
            super::Check::Invalid as i32
        );
        assert_eq!(
            slot.finish(6, super::SendResult::Busy, 100, 100),
            super::Outcome::Finished
        );
        assert_eq!(slot.observe().retried, 0);
        assert_eq!(
            slot.finish(7, super::SendResult::Busy, 100, 100),
            super::Outcome::Retry
        );
        assert_eq!(
            slot.check(true, false, 1, 7, 299).check,
            super::Check::Early as i32
        );
        assert_eq!(slot.rollover(8, 1001), 0);
        assert_eq!(slot.rollover(7, 1001), 2);
        slot.install(2, 8, 2000, 3000);
        slot.complete(7);
        assert_eq!(slot.observe().completed, 0);
        assert_eq!(slot.observe().retry_due, 0);
        slot = super::Slot::new();
        assert_eq!(slot.observe().completed, 1);
    }
    use super::*;
    fn check(s: &mut State, now: i64) -> Check {
        s.check(true, false, 1, 7, 7, 100, 1000, now)
    }
    #[test]
    fn precedence_and_boundaries() {
        let mut s = State::init();
        assert_eq!(s.check(false, true, -1, 0, 1, 0, 1, 2), Check::Ignored);
        s = State::install(1);
        for (active, stopping, index, generation) in [
            (false, false, 1, 7),
            (true, true, 1, 7),
            (true, false, -1, 7),
            (true, false, 2, 7),
            (true, false, 1, 8),
        ] {
            let before = s;
            assert_eq!(
                s.check(active, stopping, index, generation, 7, 100, 1000, 1001),
                Check::Invalid
            );
            assert_eq!(s, before);
        }
        assert_eq!(check(&mut s, 99), Check::Early);
        assert_eq!(check(&mut s, 100), Check::Attempt);
        assert_eq!(check(&mut s, 1000), Check::Attempt);
        assert_eq!(check(&mut s, 1001), Check::Expired);
        assert_eq!(check(&mut s, 1002), Check::Ignored);
    }
    #[test]
    fn repeated_busy_exact_margin_and_results() {
        let mut s = State::install(1);
        for now in [100, 300, 500, 700] {
            assert_eq!(s.finish(SendResult::Busy, now, 1000, 100), Outcome::Retry);
            assert_eq!(s.retry_due_us, now + 200);
            assert_eq!(check(&mut s, now + 199), Check::Early);
            assert_eq!(check(&mut s, now + 200), Check::Attempt);
        }
        let retried = s;
        assert_eq!(
            s.finish(SendResult::Busy, 701, 1000, 100),
            Outcome::Exhausted
        );
        for (result, now, want) in [
            (SendResult::Submitted, 1001, Outcome::Recovered),
            (SendResult::Empty, 1001, Outcome::Finished),
            (SendResult::Error, 1000, Outcome::Missed),
            (SendResult::Error, 1001, Outcome::Exhausted),
        ] {
            s = retried;
            assert_eq!(s.finish(result, now, 1000, 100), want);
            assert!(s.completed);
        }
        for result in [SendResult::Busy, SendResult::Error] {
            s = State::install(1);
            assert_eq!(s.finish(result, 701, 1000, 100), Outcome::Missed);
        }
        s = State::install(1);
        assert_eq!(
            s.finish(SendResult::Submitted, 1001, 1000, 100),
            Outcome::Finished
        );
    }
    #[test]
    fn sentinel_rollover_and_wrap() {
        assert!(!deadline_passed(0, i64::MAX));
        let mut ungated = State::install(1);
        assert_eq!(
            ungated.check(true, false, 1, 7, 7, 100, 0, 1001),
            Check::Attempt
        );
        assert!(!ungated.rollover(7, 7, 0, 1001));
        let mut ambiguous_retry = State::install(1);
        ambiguous_retry.retried = true;
        ambiguous_retry.retry_due_us = 1;
        let ambiguous_now = 1i64.wrapping_add(i64::MIN);
        assert_eq!(
            ambiguous_retry.check(true, false, 1, 7, 7, ambiguous_now, 0, ambiguous_now),
            Check::Expired
        );
        let mut s = State::install(1);
        assert_eq!(s.finish(SendResult::Busy, -1000, 0, 0), Outcome::Missed);
        s = State::install(1);
        s.retried = true;
        assert!(!s.rollover(6, 7, 100, 101));
        assert!(!s.rollover(7, 7, 100, 100));
        assert!(s.rollover(7, 7, 100, 101));
        assert!(!s.rollover(7, 7, 100, 102));
        let start = i64::MAX - 100;
        let now = start.wrapping_add(50);
        let deadline = start.wrapping_add(600);
        s = State::install(1);
        assert_eq!(
            s.check(true, false, 1, 7, 7, start, deadline, now),
            Check::Attempt
        );
        assert_eq!(
            s.finish(SendResult::Busy, now, deadline, 100),
            Outcome::Retry
        );
        let due = now.wrapping_add(200);
        assert_eq!(
            s.check(true, false, 1, 7, 7, start, deadline, due.wrapping_sub(1)),
            Check::Early
        );
        assert_eq!(
            s.check(true, false, 1, 7, 7, start, deadline, due),
            Check::Attempt
        );
        assert!(!deadline_passed(deadline, deadline));
        assert!(deadline_passed(deadline, deadline.wrapping_add(1)));
        assert!(deadline_passed(1, 1i64.wrapping_add(i64::MIN)));
        s = State::install(1);
        assert_eq!(
            s.check(true, false, 1, 7, 7, 1, 0, 1i64.wrapping_add(i64::MIN)),
            Check::Expired
        );
        s = State::install(1);
        assert_eq!(
            s.finish(SendResult::Busy, 1, 1i64.wrapping_add(i64::MIN), 0),
            Outcome::Missed
        );
        s = State::install(1);
        assert_eq!(
            s.finish(SendResult::Busy, 1, 1000, i64::MAX),
            Outcome::Missed
        );
    }
}
