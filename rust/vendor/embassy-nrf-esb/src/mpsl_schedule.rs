//! Pure schedule-hint helpers for MPSL ESB diagnostics.
//!
//! The first scheduling increment is intentionally passive: PRX advertises a
//! hint in ACK payloads and PTX validates/parses it without changing transmit
//! timing. Later increments can use the same wire format to gate PTX slots.

pub const SCHEDULE_HINT_VERSION: u8 = 1;
pub const SCHEDULE_HINT_MAGIC: u16 = 0x4853; // "SH", little-endian on wire.
pub const SCHEDULE_HINT_ENCODED_LEN: usize = 20;
pub const SCHEDULE_HINT_PAYLOAD_OFFSET: usize = 4;
pub const SCHEDULE_COUNTER_HINT_PAYLOAD_LEN: usize =
    SCHEDULE_HINT_PAYLOAD_OFFSET + SCHEDULE_HINT_ENCODED_LEN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScheduleHintError {
    BufferTooShort,
    BadMagic,
    UnsupportedVersion,
    InvalidTiming,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScheduleHint {
    pub version: u8,
    pub flags: u8,
    pub window_id: u32,
    pub next_delay_us: u32,
    pub period_us: u32,
    pub window_us: u32,
}

impl ScheduleHint {
    pub const fn new(
        flags: u8,
        window_id: u32,
        next_delay_us: u32,
        period_us: u32,
        window_us: u32,
    ) -> Self {
        Self {
            version: SCHEDULE_HINT_VERSION,
            flags,
            window_id,
            next_delay_us,
            period_us,
            window_us,
        }
    }

    pub fn validate(self) -> Result<(), ScheduleHintError> {
        if self.version != SCHEDULE_HINT_VERSION {
            return Err(ScheduleHintError::UnsupportedVersion);
        }
        if self.period_us == 0 || self.window_us == 0 || self.window_us > self.period_us {
            return Err(ScheduleHintError::InvalidTiming);
        }
        Ok(())
    }
}

pub fn encode_schedule_hint(buf: &mut [u8], hint: ScheduleHint) -> Result<(), ScheduleHintError> {
    if buf.len() < SCHEDULE_HINT_ENCODED_LEN {
        return Err(ScheduleHintError::BufferTooShort);
    }
    hint.validate()?;

    buf[0..2].copy_from_slice(&SCHEDULE_HINT_MAGIC.to_le_bytes());
    buf[2] = hint.version;
    buf[3] = hint.flags;
    buf[4..8].copy_from_slice(&hint.window_id.to_le_bytes());
    buf[8..12].copy_from_slice(&hint.next_delay_us.to_le_bytes());
    buf[12..16].copy_from_slice(&hint.period_us.to_le_bytes());
    buf[16..20].copy_from_slice(&hint.window_us.to_le_bytes());
    Ok(())
}

pub fn decode_schedule_hint(buf: &[u8]) -> Result<ScheduleHint, ScheduleHintError> {
    if buf.len() < SCHEDULE_HINT_ENCODED_LEN {
        return Err(ScheduleHintError::BufferTooShort);
    }

    let magic = u16::from_le_bytes([buf[0], buf[1]]);
    if magic != SCHEDULE_HINT_MAGIC {
        return Err(ScheduleHintError::BadMagic);
    }

    let hint = ScheduleHint {
        version: buf[2],
        flags: buf[3],
        window_id: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        next_delay_us: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        period_us: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        window_us: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
    };
    hint.validate()?;
    Ok(hint)
}

pub fn decode_counter_payload_schedule_hint(
    payload: &[u8],
) -> Result<ScheduleHint, ScheduleHintError> {
    if payload.len() < SCHEDULE_COUNTER_HINT_PAYLOAD_LEN {
        return Err(ScheduleHintError::BufferTooShort);
    }
    decode_schedule_hint(
        &payload[SCHEDULE_HINT_PAYLOAD_OFFSET
            ..SCHEDULE_HINT_PAYLOAD_OFFSET + SCHEDULE_HINT_ENCODED_LEN],
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScheduleTrackerSnapshot {
    pub valid_hint_count: u32,
    pub bad_hint_count: u32,
    pub repeat_count: u32,
    pub jump_count: u32,
    pub missed_window_count: u32,
    pub regress_count: u32,
    pub current_miss_streak: u32,
    pub max_miss_streak: u32,
    pub last_window_id: u32,
}

impl ScheduleTrackerSnapshot {
    pub const ZERO: Self = Self {
        valid_hint_count: 0,
        bad_hint_count: 0,
        repeat_count: 0,
        jump_count: 0,
        missed_window_count: 0,
        regress_count: 0,
        current_miss_streak: 0,
        max_miss_streak: 0,
        last_window_id: 0,
    };

    pub fn saturating_sub(self, previous: Self) -> Self {
        Self {
            valid_hint_count: self
                .valid_hint_count
                .saturating_sub(previous.valid_hint_count),
            bad_hint_count: self.bad_hint_count.saturating_sub(previous.bad_hint_count),
            repeat_count: self.repeat_count.saturating_sub(previous.repeat_count),
            jump_count: self.jump_count.saturating_sub(previous.jump_count),
            missed_window_count: self
                .missed_window_count
                .saturating_sub(previous.missed_window_count),
            regress_count: self.regress_count.saturating_sub(previous.regress_count),
            current_miss_streak: self.current_miss_streak,
            max_miss_streak: self.max_miss_streak,
            last_window_id: self.last_window_id,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScheduleTracker {
    seen_hint: bool,
    snapshot: ScheduleTrackerSnapshot,
}

impl Default for ScheduleTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl ScheduleTracker {
    pub const fn new() -> Self {
        Self {
            seen_hint: false,
            snapshot: ScheduleTrackerSnapshot::ZERO,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn observe_hint(&mut self, hint: ScheduleHint) {
        if self.seen_hint {
            let last = self.snapshot.last_window_id;
            if hint.window_id == last {
                self.snapshot.repeat_count += 1;
            } else if hint.window_id > last {
                let delta = hint.window_id - last;
                if delta > 1 {
                    self.snapshot.jump_count += 1;
                    self.snapshot.missed_window_count += delta - 1;
                }
            } else {
                self.snapshot.regress_count += 1;
            }
        } else {
            self.seen_hint = true;
        }

        self.snapshot.valid_hint_count += 1;
        self.snapshot.current_miss_streak = 0;
        self.snapshot.last_window_id = hint.window_id;
    }

    pub fn observe_bad_hint(&mut self) {
        self.snapshot.bad_hint_count += 1;
        self.observe_miss();
    }

    pub fn observe_miss(&mut self) {
        self.snapshot.current_miss_streak += 1;
        if self.snapshot.current_miss_streak > self.snapshot.max_miss_streak {
            self.snapshot.max_miss_streak = self.snapshot.current_miss_streak;
        }
    }

    pub const fn snapshot(self) -> ScheduleTrackerSnapshot {
        self.snapshot
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkTimingMode {
    Unsynced,
    Synced,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkTimingFallback {
    NoHint,
    HintExpired,
    WaitTooLong,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkTimingSnapshot {
    pub mode: LinkTimingMode,
    pub lock_count: u32,
    pub fallback_count: u32,
    pub miss_streak: u8,
    pub last_window_id: u32,
    pub hint_age_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoundedWindowWait {
    pub wait_us: u32,
    pub fallback: Option<LinkTimingFallback>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkTimingConfig {
    pub hint_valid_us: u32,
    pub max_wait_us: u32,
    pub window_guard_us: u32,
    pub miss_limit: u8,
}

impl LinkTimingConfig {
    pub const fn keyboard_default() -> Self {
        Self {
            hint_valid_us: 200_000,
            max_wait_us: 4_000,
            window_guard_us: 400,
            miss_limit: 4,
        }
    }

    pub fn validate(self) -> bool {
        self.hint_valid_us > 0 && self.max_wait_us > 0 && self.miss_limit > 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkTiming {
    mode: LinkTimingMode,
    last_hint: Option<ScheduleHint>,
    last_hint_at_us: u64,
    next_window_at_us: u64,
    lock_count: u32,
    fallback_count: u32,
    miss_streak: u8,
}

impl Default for LinkTiming {
    fn default() -> Self {
        Self::new()
    }
}

impl LinkTiming {
    pub const fn new() -> Self {
        Self {
            mode: LinkTimingMode::Unsynced,
            last_hint: None,
            last_hint_at_us: 0,
            next_window_at_us: 0,
            lock_count: 0,
            fallback_count: 0,
            miss_streak: 0,
        }
    }

    pub fn observe_hint(&mut self, now_us: u64, hint: ScheduleHint) {
        let was_synced = matches!(self.mode, LinkTimingMode::Synced);
        self.mode = LinkTimingMode::Synced;
        self.last_hint = Some(hint);
        self.last_hint_at_us = now_us;
        self.next_window_at_us = now_us.saturating_add(hint.next_delay_us as u64);
        self.miss_streak = 0;
        if !was_synced {
            self.lock_count = self.lock_count.saturating_add(1);
        }
    }

    pub fn observe_miss(&mut self, config: LinkTimingConfig) {
        self.miss_streak = self.miss_streak.saturating_add(1);
        if self.miss_streak >= config.miss_limit {
            self.mode = LinkTimingMode::Degraded;
        }
    }

    pub fn reset_to_scan(&mut self) {
        self.mode = LinkTimingMode::Unsynced;
        self.last_hint = None;
        self.next_window_at_us = 0;
        self.miss_streak = 0;
    }

    pub fn bounded_wait(&mut self, now_us: u64, config: LinkTimingConfig) -> BoundedWindowWait {
        if !config.validate() {
            self.fallback_count = self.fallback_count.saturating_add(1);
            return BoundedWindowWait {
                wait_us: 0,
                fallback: Some(LinkTimingFallback::NoHint),
            };
        }

        if matches!(self.mode, LinkTimingMode::Degraded) {
            self.fallback_count = self.fallback_count.saturating_add(1);
            return BoundedWindowWait {
                wait_us: 0,
                fallback: Some(LinkTimingFallback::Degraded),
            };
        }

        let Some(hint) = self.last_hint else {
            self.fallback_count = self.fallback_count.saturating_add(1);
            return BoundedWindowWait {
                wait_us: 0,
                fallback: Some(LinkTimingFallback::NoHint),
            };
        };

        let age_us = now_us.saturating_sub(self.last_hint_at_us);
        if age_us > config.hint_valid_us as u64 {
            self.mode = LinkTimingMode::Unsynced;
            self.last_hint = None;
            self.fallback_count = self.fallback_count.saturating_add(1);
            return BoundedWindowWait {
                wait_us: 0,
                fallback: Some(LinkTimingFallback::HintExpired),
            };
        }

        while self.next_window_at_us.saturating_add(hint.window_us as u64) <= now_us {
            self.next_window_at_us = self.next_window_at_us.saturating_add(hint.period_us as u64);
        }

        let target_us = self
            .next_window_at_us
            .saturating_sub(config.window_guard_us as u64);
        let wait_us = target_us.saturating_sub(now_us);
        if wait_us > config.max_wait_us as u64 {
            self.fallback_count = self.fallback_count.saturating_add(1);
            return BoundedWindowWait {
                wait_us: 0,
                fallback: Some(LinkTimingFallback::WaitTooLong),
            };
        }

        BoundedWindowWait {
            wait_us: wait_us as u32,
            fallback: None,
        }
    }

    pub fn snapshot(&self, now_us: u64) -> LinkTimingSnapshot {
        LinkTimingSnapshot {
            mode: self.mode,
            lock_count: self.lock_count,
            fallback_count: self.fallback_count,
            miss_streak: self.miss_streak,
            last_window_id: self.last_hint.map(|h| h.window_id).unwrap_or(0),
            hint_age_us: now_us
                .saturating_sub(self.last_hint_at_us)
                .min(u32::MAX as u64) as u32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        LinkTiming, LinkTimingConfig, LinkTimingFallback, LinkTimingMode,
        SCHEDULE_COUNTER_HINT_PAYLOAD_LEN, SCHEDULE_HINT_PAYLOAD_OFFSET, ScheduleHint,
        ScheduleHintError, ScheduleTracker, ScheduleTrackerSnapshot,
        decode_counter_payload_schedule_hint, decode_schedule_hint, encode_schedule_hint,
    };

    #[test]
    fn schedule_hint_round_trips() {
        let hint = ScheduleHint::new(0x5a, 42, 1500, 5000, 4500);
        let mut buf = [0u8; 20];

        encode_schedule_hint(&mut buf, hint).unwrap();

        assert_eq!(decode_schedule_hint(&buf), Ok(hint));
    }

    #[test]
    fn schedule_hint_rejects_bad_magic_version_and_timing() {
        let hint = ScheduleHint::new(0, 1, 1500, 5000, 4500);
        let mut buf = [0u8; 20];
        encode_schedule_hint(&mut buf, hint).unwrap();

        buf[0] ^= 0xff;
        assert_eq!(decode_schedule_hint(&buf), Err(ScheduleHintError::BadMagic));

        encode_schedule_hint(&mut buf, hint).unwrap();
        buf[2] = 2;
        assert_eq!(
            decode_schedule_hint(&buf),
            Err(ScheduleHintError::UnsupportedVersion)
        );

        let invalid = ScheduleHint::new(0, 1, 1500, 4000, 4500);
        assert_eq!(
            encode_schedule_hint(&mut buf, invalid),
            Err(ScheduleHintError::InvalidTiming)
        );
    }

    #[test]
    fn counter_payload_hint_starts_after_legacy_counter() {
        let hint = ScheduleHint::new(0, 7, 1500, 5000, 4500);
        let mut payload = [0u8; SCHEDULE_COUNTER_HINT_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&123u32.to_le_bytes());
        encode_schedule_hint(&mut payload[SCHEDULE_HINT_PAYLOAD_OFFSET..], hint).unwrap();

        assert_eq!(decode_counter_payload_schedule_hint(&payload), Ok(hint));
    }

    #[test]
    fn schedule_tracker_classifies_repeats_jumps_regressions_and_misses() {
        let mut tracker = ScheduleTracker::new();

        tracker.observe_miss();
        tracker.observe_miss();
        tracker.observe_hint(ScheduleHint::new(0, 10, 1500, 5000, 4500));
        tracker.observe_hint(ScheduleHint::new(0, 10, 1500, 5000, 4500));
        tracker.observe_hint(ScheduleHint::new(0, 13, 1500, 5000, 4500));
        tracker.observe_hint(ScheduleHint::new(0, 12, 1500, 5000, 4500));
        tracker.observe_bad_hint();

        assert_eq!(
            tracker.snapshot(),
            ScheduleTrackerSnapshot {
                valid_hint_count: 4,
                bad_hint_count: 1,
                repeat_count: 1,
                jump_count: 1,
                missed_window_count: 2,
                regress_count: 1,
                current_miss_streak: 1,
                max_miss_streak: 2,
                last_window_id: 12,
            }
        );
    }

    #[test]
    fn schedule_tracker_snapshot_delta_keeps_current_state_fields() {
        let previous = ScheduleTrackerSnapshot {
            valid_hint_count: 2,
            bad_hint_count: 1,
            repeat_count: 1,
            jump_count: 0,
            missed_window_count: 0,
            regress_count: 0,
            current_miss_streak: 3,
            max_miss_streak: 3,
            last_window_id: 10,
        };
        let current = ScheduleTrackerSnapshot {
            valid_hint_count: 5,
            bad_hint_count: 2,
            repeat_count: 1,
            jump_count: 1,
            missed_window_count: 4,
            regress_count: 1,
            current_miss_streak: 1,
            max_miss_streak: 5,
            last_window_id: 20,
        };

        assert_eq!(
            current.saturating_sub(previous),
            ScheduleTrackerSnapshot {
                valid_hint_count: 3,
                bad_hint_count: 1,
                repeat_count: 0,
                jump_count: 1,
                missed_window_count: 4,
                regress_count: 1,
                current_miss_streak: 1,
                max_miss_streak: 5,
                last_window_id: 20,
            }
        );
    }

    #[test]
    fn link_timing_locks_on_hint_and_waits_inside_budget() {
        let mut timing = LinkTiming::new();
        let config = LinkTimingConfig::keyboard_default();

        timing.observe_hint(1_000, ScheduleHint::new(0, 7, 2_000, 11_000, 10_500));

        let wait = timing.bounded_wait(2_000, config);
        assert_eq!(wait.fallback, None);
        assert_eq!(wait.wait_us, 600);

        let snapshot = timing.snapshot(2_000);
        assert_eq!(snapshot.mode, LinkTimingMode::Synced);
        assert_eq!(snapshot.lock_count, 1);
        assert_eq!(snapshot.last_window_id, 7);
    }

    #[test]
    fn link_timing_falls_back_when_wait_exceeds_latency_budget() {
        let mut timing = LinkTiming::new();
        let config = LinkTimingConfig {
            max_wait_us: 500,
            ..LinkTimingConfig::keyboard_default()
        };

        timing.observe_hint(1_000, ScheduleHint::new(0, 1, 4_000, 11_000, 10_500));

        let wait = timing.bounded_wait(1_100, config);
        assert_eq!(wait.wait_us, 0);
        assert_eq!(wait.fallback, Some(LinkTimingFallback::WaitTooLong));
        assert_eq!(timing.snapshot(1_100).fallback_count, 1);
    }

    #[test]
    fn link_timing_expires_old_hints_and_degrades_after_misses() {
        let mut timing = LinkTiming::new();
        let config = LinkTimingConfig {
            hint_valid_us: 1_000,
            miss_limit: 2,
            ..LinkTimingConfig::keyboard_default()
        };

        assert_eq!(
            timing.bounded_wait(100, config).fallback,
            Some(LinkTimingFallback::NoHint)
        );

        timing.observe_hint(1_000, ScheduleHint::new(0, 3, 1_000, 11_000, 10_500));
        assert_eq!(
            timing.bounded_wait(3_001, config).fallback,
            Some(LinkTimingFallback::HintExpired)
        );
        assert_eq!(timing.snapshot(3_001).mode, LinkTimingMode::Unsynced);

        timing.observe_hint(4_000, ScheduleHint::new(0, 4, 1_000, 11_000, 10_500));
        timing.observe_miss(config);
        timing.observe_miss(config);
        assert_eq!(
            timing.bounded_wait(4_100, config).fallback,
            Some(LinkTimingFallback::Degraded)
        );
        assert_eq!(timing.snapshot(4_100).mode, LinkTimingMode::Degraded);
    }

    #[test]
    fn link_timing_advances_past_stale_windows() {
        let mut timing = LinkTiming::new();
        let config = LinkTimingConfig {
            max_wait_us: 20_000,
            ..LinkTimingConfig::keyboard_default()
        };

        timing.observe_hint(1_000, ScheduleHint::new(0, 9, 1_000, 11_000, 10_500));

        let wait = timing.bounded_wait(12_500, config);
        assert_eq!(wait.fallback, None);
        assert_eq!(wait.wait_us, 100);
    }
}
