//! Portable WS math and serial sampler, after successful hardware initialization.

pub const EXPECTED_EDGES_PER_FRAME: u32 = 960;
pub const MAX_ELAPSED_FRAMES: u32 = 50;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleDecision {
    Valid { error_us: i32, correction_us: i32 },
    NoSignal,
    Rejected,
}

impl SampleDecision {
    /// `None` means the original C helper writes neither output pointer.
    pub const fn outputs(self) -> Option<(i32, i32)> {
        match self {
            Self::Valid {
                error_us,
                correction_us,
            } => Some((error_us, correction_us)),
            Self::NoSignal | Self::Rejected => None,
        }
    }
}

pub fn evaluate(delta: u32, elapsed: u32) -> SampleDecision {
    if elapsed == 0 || elapsed > MAX_ELAPSED_FRAMES {
        return SampleDecision::Rejected;
    }
    if delta == 0 {
        return SampleDecision::NoSignal;
    }
    let expected = elapsed * EXPECTED_EDGES_PER_FRAME;
    if delta < expected * 3 / 4 || delta > expected * 5 / 4 {
        return SampleDecision::Rejected;
    }
    let numerator = (i64::from(expected) - i64::from(delta)) * 125;
    let error_us = if numerator >= 0 {
        (numerator + 3) / 6
    } else {
        (numerator - 3) / 6
    } as i32;
    SampleDecision::Valid {
        error_us,
        correction_us: (error_us / 4).clamp(-500, 500),
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SampleResult {
    pub valid: bool,
    /// Always zero for invalid samples, including stopped samples.
    pub correction_us: i32,
}

/// Semantic fields and widths of `ws_sync_diag_t`, not a C ABI layout.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WsStats {
    pub total_edges: u32,
    pub sample_count: u32,
    pub valid_count: u32,
    pub no_signal_count: u32,
    pub rejected_count: u32,
    pub last_delta_edges: u32,
    pub last_correction_us: i32,
    pub cumulative_drift_us: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WsSnapshot {
    pub initialized: bool,
    pub running: bool,
    pub first_sample: bool,
    pub last_count: u32,
    pub last_frame_counter: u32,
}

pub struct WsSampler {
    state: WsSnapshot,
    stats: WsStats,
}

impl Default for WsSampler {
    fn default() -> Self {
        Self::new()
    }
}

impl WsSampler {
    /// Construct in the logical domain where `ws_sync_init` succeeded.
    /// The hardware owner must establish resources before calling `start`.
    pub fn new() -> Self {
        Self {
            state: WsSnapshot {
                initialized: true,
                running: false,
                first_sample: true,
                last_count: 0,
                last_frame_counter: 0,
            },
            stats: WsStats::default(),
        }
    }

    pub fn start(&mut self) {
        if !self.state.initialized || self.state.running {
            return;
        }
        self.state.running = true;
        self.state.first_sample = true;
        self.state.last_count = 0;
        self.state.last_frame_counter = 0;
        self.stats = WsStats::default();
    }

    pub fn stop(&mut self) {
        self.state.running = false;
    }

    pub fn sample(&mut self, absolute_count: u32, frame: u32) -> SampleResult {
        if !self.state.running {
            return SampleResult::default();
        }
        let delta = absolute_count.wrapping_sub(self.state.last_count);
        self.state.last_count = absolute_count;
        self.stats.total_edges = self.stats.total_edges.wrapping_add(delta);
        self.stats.sample_count = self.stats.sample_count.wrapping_add(1);
        self.stats.last_delta_edges = delta;
        if self.state.first_sample {
            self.state.first_sample = false;
            self.state.last_frame_counter = frame;
            return SampleResult::default();
        }
        let elapsed = frame.wrapping_sub(self.state.last_frame_counter);
        self.state.last_frame_counter = frame;
        match evaluate(delta, elapsed) {
            SampleDecision::Valid {
                error_us,
                correction_us,
            } => {
                self.stats.last_correction_us = correction_us;
                self.stats.cumulative_drift_us =
                    self.stats.cumulative_drift_us.saturating_add(error_us);
                self.stats.valid_count = self.stats.valid_count.wrapping_add(1);
                SampleResult {
                    valid: true,
                    correction_us,
                }
            }
            SampleDecision::NoSignal => {
                self.stats.no_signal_count = self.stats.no_signal_count.wrapping_add(1);
                SampleResult::default()
            }
            SampleDecision::Rejected => {
                self.stats.rejected_count = self.stats.rejected_count.wrapping_add(1);
                SampleResult::default()
            }
        }
    }

    pub fn stats(&self) -> WsStats {
        self.stats
    }

    /// Read-only logical baseline projection for diagnostics and oracle checks.
    pub fn snapshot(&self) -> WsSnapshot {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagnostic_counters_wrap() {
        let mut sampler = WsSampler::new();
        sampler.start();
        sampler.sample(u32::MAX, 0);
        sampler.stats.sample_count = u32::MAX;
        sampler.stats.valid_count = u32::MAX;
        assert!(sampler.sample(959, 1).valid);
        assert_eq!(sampler.stats.total_edges, 959);
        assert_eq!(sampler.stats.sample_count, 0);
        assert_eq!(sampler.stats.valid_count, 0);
        sampler.stats.no_signal_count = u32::MAX;
        sampler.sample(959, 2);
        assert_eq!(sampler.stats.no_signal_count, 0);
        sampler.stats.rejected_count = u32::MAX;
        sampler.sample(960, 2);
        assert_eq!(sampler.stats.rejected_count, 0);
    }
}
