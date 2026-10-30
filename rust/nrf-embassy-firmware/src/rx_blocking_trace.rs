//! Sampled CS body spans, not IRQ latency. Nested spans overlap.
use core::cell::RefCell;
use critical_section::Mutex;

#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub enum Operation {
    ApplyAudio,
    ApplyStatus,
    ApplyEvent,
    ApplyAck,
    ApplyLog,
    Discard,
    Delivery,
    Begin,
    Finish,
    Cancel,
    Snapshot,
}
impl Operation {
    pub const ALL: [Self; 11] = [
        Self::ApplyAudio,
        Self::ApplyStatus,
        Self::ApplyEvent,
        Self::ApplyAck,
        Self::ApplyLog,
        Self::Discard,
        Self::Delivery,
        Self::Begin,
        Self::Finish,
        Self::Cancel,
        Self::Snapshot,
    ];
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Histogram {
    pub bins: [u32; 8],
    /// All sampled operations, including clock errors.
    pub count: u32,
    pub max_us: u64,
    pub errors: u32,
}
impl Histogram {
    pub fn record(&mut self, start: Option<i64>, end: Option<i64>) {
        self.count = self.count.wrapping_add(1);
        let elapsed = start.zip(end).and_then(|(s, e)| {
            if s < 0 {
                return None;
            }
            e.checked_sub(s).and_then(|d| u64::try_from(d).ok())
        });
        if let Some(us) = elapsed {
            let bin = [10, 25, 50, 100, 200, 400, 800]
                .iter()
                .position(|limit| us < *limit)
                .unwrap_or(7);
            self.bins[bin] = self.bins[bin].wrapping_add(1);
            self.max_us = self.max_us.max(us);
        } else {
            self.errors = self.errors.wrapping_add(1);
        }
    }
}

type Clock = fn() -> Option<i64>;
static CLOCK: Mutex<RefCell<Option<Clock>>> = Mutex::new(RefCell::new(None));
static STATS: Mutex<RefCell<[Histogram; 11]>> = Mutex::new(RefCell::new(
    [Histogram {
        bins: [0; 8],
        count: 0,
        max_us: 0,
        errors: 0,
    }; 11],
));

/// Install an independent bounded clock. Must never reenter Bridge or poll tasks.
/// No registry or statistics loan is held while calling it.
pub fn install_clock(clock: Clock) {
    critical_section::with(|cs| *CLOCK.borrow(cs).borrow_mut() = Some(clock));
}
pub fn now() -> Option<i64> {
    let clock = critical_section::with(|cs| *CLOCK.borrow(cs).borrow());
    clock.and_then(|clock| clock())
}
pub fn record(op: Operation, start: Option<i64>, end: Option<i64>) {
    critical_section::with(|cs| STATS.borrow(cs).borrow_mut()[op as usize].record(start, end));
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Publication {
    pub sequence: u32,
    pub clock_us: Option<i64>,
    pub bridge_available: bool,
    pub bridge: [Histogram; 11],
    pub radio_clock_us: Option<i64>,
    /// bad CRC, duplicate, start allocation failure, post-delivery starvation,
    /// RX queue rejection. None means observation unavailable.
    pub radio: Option<[u32; 5]>,
}
pub fn capture(sequence: u32) -> Publication {
    let clock_us = now();
    let bridge = critical_section::with(|cs| *STATS.borrow(cs).borrow());
    Publication {
        sequence,
        clock_us,
        bridge_available: cfg!(feature = "nonblocking-bridge-delivery"),
        bridge,
        ..Publication::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn boundaries_and_clock_failures_are_not_zero_samples() {
        let mut h = Histogram::default();
        for us in [
            0,
            9,
            10,
            24,
            25,
            49,
            50,
            99,
            100,
            199,
            200,
            399,
            400,
            799,
            800,
            i64::MAX,
        ] {
            h.record(Some(0), Some(us));
        }
        assert_eq!(h.bins, [2; 8]);
        for (s, e) in [
            (None, Some(0)),
            (Some(0), None),
            (Some(2), Some(1)),
            (Some(-1), Some(0)),
        ] {
            h.record(s, e);
        }
        assert_eq!(h.count, 20);
        assert_eq!(h.errors, 4);
        assert_eq!(h.max_us, i64::MAX as u64);
    }
}
