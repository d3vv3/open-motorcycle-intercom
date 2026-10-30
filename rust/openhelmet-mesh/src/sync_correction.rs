//! Stateless participant correction arithmetic, in microseconds.
//! Frame products are widened; timestamp operations explicitly wrap. Only the
//! reported drift is narrowed/clamped: the full RX-derived start is installed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Correction {
    pub actual_start_us: i64,
    pub drift_us: i32,
    pub error: bool,
}
pub fn correction(
    local_start: i64,
    local_frame: u32,
    frame: u32,
    rx: i64,
    phase: u32,
    latency: i64,
    frame_us: i64,
) -> Correction {
    let delta = frame.wrapping_sub(local_frame) as i32;
    let expected = local_start.wrapping_add(frame_us.wrapping_mul(i64::from(delta)));
    let actual_start_us = rx.wrapping_sub(i64::from(phase)).wrapping_sub(latency);
    let drift_us = (actual_start_us.wrapping_sub(expected) as i32).clamp(-500, 500);
    Correction {
        actual_start_us,
        drift_us,
        error: !(-100..=100).contains(&drift_us),
    }
}
/// Caller supplies the fresh sample taken after installing the correction.
/// The production frame duration is strictly positive.
pub fn next_delay(now: i64, actual_start: i64, frame_us: i64) -> i64 {
    let elapsed = now.wrapping_sub(actual_start) % frame_us;
    let normalized = if elapsed < 0 {
        elapsed + frame_us
    } else {
        elapsed
    };
    frame_us - normalized
}
