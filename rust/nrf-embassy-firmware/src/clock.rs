//! TIMER0 microsecond extension. Read at least once per strictly less than
//! 2^32 us: a raw counter cannot reveal a whole unobserved wrap.

#[derive(Default)]
pub struct MicroClock {
    last: u32,
    high: u64,
}

impl MicroClock {
    pub const fn new() -> Self {
        Self { last: 0, high: 0 }
    }

    pub fn read(&mut self, raw: u32) -> Option<i64> {
        if raw < self.last {
            self.high = self.high.checked_add(1u64 << 32)?;
        }
        self.last = raw;
        i64::try_from(self.high.checked_add(u64::from(raw))?).ok()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockFault {
    WrongContext,
    Inactive,
    SignedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockContext {
    Thread,
    MaskableException,
    Nmi,
    HardFault,
}

/// The closure includes guard acquisition and state borrowing, not just MMIO.
pub fn context_gate<R>(
    context: ClockContext,
    access: impl FnOnce() -> Result<R, ClockFault>,
) -> Result<R, ClockFault> {
    match context {
        ClockContext::Nmi | ClockContext::HardFault => Err(ClockFault::WrongContext),
        ClockContext::Thread | ClockContext::MaskableException => access(),
    }
}

/// Only extension and lifetime/fault state. The caller serializes this entire
/// capture-plus-extension operation; no peripheral or actor reference is stored.
pub struct SharedClockState {
    clock: MicroClock,
    alive: bool,
    fault: Option<ClockFault>,
}

impl SharedClockState {
    pub const fn inactive() -> Self {
        Self {
            clock: MicroClock::new(),
            alive: false,
            fault: None,
        }
    }

    pub fn activate(&mut self) {
        self.clock = MicroClock::new();
        self.fault = None;
        self.alive = true;
    }

    pub fn deactivate(&mut self) {
        self.alive = false;
    }

    pub fn capture_and_extend(&mut self, capture: impl FnOnce() -> u32) -> Result<i64, ClockFault> {
        if !self.alive {
            return Err(ClockFault::Inactive);
        }
        if let Some(fault) = self.fault {
            return Err(fault);
        }
        match self.clock.read(capture()) {
            Some(now) => Ok(now),
            None => {
                self.fault = Some(ClockFault::SignedRange);
                Err(ClockFault::SignedRange)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_wrap_is_extended_not_signed_or_saturated() {
        let mut clock = MicroClock::default();
        assert_eq!(clock.read(u32::MAX - 1), Some(4_294_967_294));
        assert_eq!(clock.read(7), Some(4_294_967_303));
        assert_eq!(clock.read(8), Some(4_294_967_304));
    }

    #[test]
    fn signed_range_and_high_overflow_fail() {
        let mut clock = MicroClock {
            last: u32::MAX,
            high: (i64::MAX as u64) & !0xffff_ffff,
        };
        assert_eq!(clock.read(u32::MAX), Some(i64::MAX));
        assert_eq!(clock.read(0), None);
        let mut clock = MicroClock {
            last: 1,
            high: !0xffff_ffff_u64,
        };
        assert_eq!(clock.read(0), None);
    }

    #[test]
    fn thread_irq_captures_extend_in_serial_capture_order() {
        struct MockCounter {
            raw: u32,
            captures: usize,
        }
        impl MockCounter {
            fn capture(&mut self) -> u32 {
                self.captures += 1;
                self.raw
            }
        }
        let mut state = SharedClockState::inactive();
        state.activate();
        let state = core::cell::RefCell::new(state);
        let mut counter = MockCounter {
            raw: 0,
            captures: 0,
        };
        for (context, raw, expected) in [
            (ClockContext::Thread, u32::MAX - 1, 4_294_967_294),
            (ClockContext::MaskableException, u32::MAX, 4_294_967_295),
            (ClockContext::Thread, 0, 4_294_967_296),
            (ClockContext::MaskableException, 1, 4_294_967_297),
        ] {
            counter.raw = raw;
            assert_eq!(
                context_gate(context, || {
                    state.borrow_mut().capture_and_extend(|| counter.capture())
                }),
                Ok(expected)
            );
        }
        assert_eq!(counter.captures, 4);
    }

    #[test]
    fn unmaskable_contexts_reject_before_guard_or_borrow() {
        let state = core::cell::RefCell::new(SharedClockState::inactive());
        let _live_borrow = state.borrow_mut();
        for context in [ClockContext::Nmi, ClockContext::HardFault] {
            assert_eq!(
                context_gate(context, || {
                    state
                        .borrow_mut()
                        .capture_and_extend(|| panic!("guard/borrow must not be entered"))
                }),
                Err(ClockFault::WrongContext)
            );
        }
    }

    #[test]
    fn inactive_and_sticky_range_fault_never_capture() {
        let mut state = SharedClockState::inactive();
        assert_eq!(
            state.capture_and_extend(|| panic!("inactive capture")),
            Err(ClockFault::Inactive)
        );
        state.activate();
        state.clock = MicroClock {
            last: u32::MAX,
            high: (i64::MAX as u64) & !0xffff_ffff,
        };
        assert_eq!(state.capture_and_extend(|| 0), Err(ClockFault::SignedRange));
        assert_eq!(
            state.capture_and_extend(|| panic!("faulted capture")),
            Err(ClockFault::SignedRange)
        );
        state.deactivate();
        assert_eq!(
            state.capture_and_extend(|| panic!("capture after owner drop")),
            Err(ClockFault::Inactive)
        );
    }

    #[test]
    fn whole_unobserved_wrap_is_not_recoverable() {
        // Identical raw values after 2^32 us cannot distinguish elapsed wraps.
        let mut clock = MicroClock::new();
        assert_eq!(clock.read(10), Some(10));
        assert_eq!(clock.read(10), Some(10));
    }
}
