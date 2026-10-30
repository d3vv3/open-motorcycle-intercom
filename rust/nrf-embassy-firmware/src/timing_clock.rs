//! Pure conversion between physical microseconds and a separate timing domain.
//! Profiles describe fixed source ratios, not physical calibration, dynamic
//! frequency, or guarantees for unknown clocks. The legacy ratio is continuous;
//! it does not emulate RTC hardware or its 100 us quantization.
//! This module selects no packet/wire mode and does not rescale SharedClock,
//! protocol wall timeouts, radio timestamps, or DMA clocks.
//! The default-off `legacy-rtc-timing` experiment selects only the fixed boot TDMA mapping.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicalTime(u64);

impl PhysicalTime {
    pub const fn new(microseconds: u64) -> Self {
        Self(microseconds)
    }

    pub const fn value(self) -> u64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimingTime(i64);

impl TimingTime {
    pub const fn new(microseconds: i64) -> Self {
        Self(microseconds)
    }

    pub const fn value(self) -> i64 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockProfile {
    Normalized,
    LegacyNrfRtc10000,
}

impl ClockProfile {
    /// Timing microseconds per physical microsecond, as numerator/denominator.
    pub const fn ratio(self) -> (u64, u64) {
        match self {
            Self::Normalized => (1, 1),
            Self::LegacyNrfRtc10000 => (32_768, 30_000),
        }
    }
}

/// Select the fixed boot mapping with zero physical and timing anchors.
pub const fn boot_mapping() -> ClockMapping {
    let profile = if cfg!(feature = "legacy-rtc-timing") {
        ClockProfile::LegacyNrfRtc10000
    } else {
        ClockProfile::Normalized
    };
    ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(0))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockError {
    PhysicalBeforeAnchor,
    TimingBeforeAnchor,
    TimingOverflow,
    PhysicalOverflow,
    CheckedArithmeticOverflow,
    InvalidWindow,
    RoundedEmptyWindow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicalWindow {
    start: PhysicalTime,
    deadline: PhysicalTime,
}

impl PhysicalWindow {
    pub const fn start(self) -> PhysicalTime {
        self.start
    }

    pub const fn deadline(self) -> PhysicalTime {
        self.deadline
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockMapping {
    profile: ClockProfile,
    physical_anchor: PhysicalTime,
    timing_anchor: TimingTime,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockReading {
    pub physical: PhysicalTime,
    pub timing: TimingTime,
}

/// Production TDMA clock owner. Its mapping is fixed for the owner's lifetime.
pub struct TdmaClock {
    mapping: ClockMapping,
    last_reading: Option<ClockReading>,
    last_timing: TimingTime,
}

impl TdmaClock {
    pub const fn new(mapping: ClockMapping) -> Self {
        Self {
            mapping,
            last_reading: None,
            last_timing: TimingTime::new(0),
        }
    }

    pub const fn mapping(&self) -> ClockMapping {
        self.mapping
    }

    pub const fn last_reading(&self) -> Option<ClockReading> {
        self.last_reading
    }

    pub const fn last_timing(&self) -> TimingTime {
        self.last_timing
    }

    pub fn clear_reading(&mut self) {
        self.last_reading = None;
    }

    pub fn sample(&mut self, physical: PhysicalTime) -> Result<TimingTime, ClockError> {
        self.clear_reading();
        let timing = self.mapping.timing_at(physical)?;
        self.last_timing = timing;
        self.last_reading = Some(ClockReading { physical, timing });
        Ok(timing)
    }
}

impl ClockMapping {
    pub const fn new(
        profile: ClockProfile,
        physical_anchor: PhysicalTime,
        timing_anchor: TimingTime,
    ) -> Self {
        Self {
            profile,
            physical_anchor,
            timing_anchor,
        }
    }

    pub const fn profile(self) -> ClockProfile {
        self.profile
    }

    pub const fn physical_anchor(self) -> PhysicalTime {
        self.physical_anchor
    }

    pub const fn timing_anchor(self) -> TimingTime {
        self.timing_anchor
    }

    pub fn timing_at(self, physical: PhysicalTime) -> Result<TimingTime, ClockError> {
        let delta = physical
            .value()
            .checked_sub(self.physical_anchor.value())
            .ok_or(ClockError::PhysicalBeforeAnchor)?;
        let (numerator, denominator) = self.profile.ratio();
        // Exact common path. Overflow falls back to the full-domain reference
        // calculation below, preserving even signed-anchor edge cases.
        if let Some(scaled) = delta.checked_mul(numerator) {
            return self
                .timing_anchor
                .value()
                .checked_add_unsigned(scaled / denominator)
                .map(TimingTime::new)
                .ok_or(ClockError::TimingOverflow);
        }
        let scaled = u128::from(delta)
            .checked_mul(u128::from(numerator))
            .ok_or(ClockError::CheckedArithmeticOverflow)?
            / u128::from(denominator);
        let delta = i128::try_from(scaled).map_err(|_| ClockError::CheckedArithmeticOverflow)?;
        let timing = i128::from(self.timing_anchor.value())
            .checked_add(delta)
            .ok_or(ClockError::CheckedArithmeticOverflow)?;
        i64::try_from(timing)
            .map(TimingTime::new)
            .map_err(|_| ClockError::TimingOverflow)
    }

    /// Ceil the inverse ratio so an alarm start cannot precede its timing target.
    pub fn physical_not_before(self, timing: TimingTime) -> Result<PhysicalTime, ClockError> {
        self.physical_at(timing, true)
    }

    /// Floor the inverse ratio so a physical END deadline cannot exceed its target.
    pub fn physical_not_after(self, timing: TimingTime) -> Result<PhysicalTime, ClockError> {
        self.physical_at(timing, false)
    }

    fn physical_at(self, timing: TimingTime, round_up: bool) -> Result<PhysicalTime, ClockError> {
        if timing.value() < self.timing_anchor.value() {
            return Err(ClockError::TimingBeforeAnchor);
        }
        // Ordered signed endpoints span at most u64::MAX. Wrapping subtraction
        // computes that nonnegative distance exactly, including MIN..MAX.
        let distance = timing.value().wrapping_sub(self.timing_anchor.value()) as u64;
        let (numerator, denominator) = self.profile.ratio();
        if let Some(scaled) = distance.checked_mul(denominator) {
            let delta =
                scaled / numerator + u64::from(round_up && !scaled.is_multiple_of(numerator));
            return self
                .physical_anchor
                .value()
                .checked_add(delta)
                .map(PhysicalTime::new)
                .ok_or(ClockError::PhysicalOverflow);
        }
        let delta = i128::from(timing.value())
            .checked_sub(i128::from(self.timing_anchor.value()))
            .ok_or(ClockError::CheckedArithmeticOverflow)?;
        let delta = u128::try_from(delta).map_err(|_| ClockError::CheckedArithmeticOverflow)?;
        let (numerator, denominator) = self.profile.ratio();
        let scaled = delta
            .checked_mul(u128::from(denominator))
            .ok_or(ClockError::CheckedArithmeticOverflow)?;
        let divisor = u128::from(numerator);
        let mut physical_delta = scaled / divisor;
        if round_up && !scaled.is_multiple_of(divisor) {
            physical_delta = physical_delta
                .checked_add(1)
                .ok_or(ClockError::CheckedArithmeticOverflow)?;
        }
        let physical = u128::from(self.physical_anchor.value())
            .checked_add(physical_delta)
            .ok_or(ClockError::CheckedArithmeticOverflow)?;
        u64::try_from(physical)
            .map(PhysicalTime::new)
            .map_err(|_| ClockError::PhysicalOverflow)
    }

    pub fn convert(self, start: TimingTime, end: TimingTime) -> Result<PhysicalWindow, ClockError> {
        if start.value() >= end.value() {
            return Err(ClockError::InvalidWindow);
        }
        let start = self.physical_not_before(start)?;
        let deadline = self.physical_not_after(end)?;
        if start.value() >= deadline.value() {
            return Err(ClockError::RoundedEmptyWindow);
        }
        Ok(PhysicalWindow { start, deadline })
    }

    /// Callers must quiesce TDMA generation timers and retire stale work before
    /// installing a different mapping. This pure method manages no active clock,
    /// ownership, or epoch invalidation. A different profile preserves the current
    /// integer timing value, dropping less than one timing unit of fractional carry.
    pub fn reprofile(
        self,
        profile: ClockProfile,
        physical_now: PhysicalTime,
    ) -> Result<Self, ClockError> {
        if physical_now.value() < self.physical_anchor.value() {
            return Err(ClockError::PhysicalBeforeAnchor);
        }
        if profile == self.profile {
            // Keep the original anchor and fractional carry to avoid rounding drift.
            return Ok(self);
        }
        let timing_now = self.timing_at(physical_now)?;
        Ok(Self::new(profile, physical_now, timing_now))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mapping(profile: ClockProfile) -> ClockMapping {
        ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(0))
    }

    // Original wide arithmetic oracle. Products for the two fixed ratios fit
    // u128 and signed sums fit i128 over the entire public input domain.
    fn wide_timing(clock: ClockMapping, physical: u64) -> Result<TimingTime, ClockError> {
        let delta = physical
            .checked_sub(clock.physical_anchor.value())
            .ok_or(ClockError::PhysicalBeforeAnchor)?;
        let (n, d) = clock.profile.ratio();
        let scaled = u128::from(delta) * u128::from(n) / u128::from(d);
        i64::try_from(i128::from(clock.timing_anchor.value()) + scaled as i128)
            .map(TimingTime::new)
            .map_err(|_| ClockError::TimingOverflow)
    }

    fn wide_physical(
        clock: ClockMapping,
        timing: i64,
        ceil: bool,
    ) -> Result<PhysicalTime, ClockError> {
        if timing < clock.timing_anchor.value() {
            return Err(ClockError::TimingBeforeAnchor);
        }
        let delta = (i128::from(timing) - i128::from(clock.timing_anchor.value())) as u128;
        let (n, d) = clock.profile.ratio();
        let scaled = delta * u128::from(d);
        let n = u128::from(n);
        let delta = scaled / n + u128::from(ceil && !scaled.is_multiple_of(n));
        u64::try_from(u128::from(clock.physical_anchor.value()) + delta)
            .map(PhysicalTime::new)
            .map_err(|_| ClockError::PhysicalOverflow)
    }

    #[test]
    fn fast_conversions_match_wide_oracle_residues_and_full_domain_edges() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let (n, d) = profile.ratio();
            // Exhaust every remainder of both divisors, at small and large
            // quotients, including the checked-multiply/fallback transition.
            for (factor, divisor) in [(n, d), (d, n)] {
                for base in [0, (u64::MAX / factor / divisor) * divisor] {
                    for remainder in 0..divisor {
                        let value = base + remainder;
                        let clock = ClockMapping::new(
                            profile,
                            PhysicalTime::new(0),
                            TimingTime::new(i64::MIN),
                        );
                        assert_eq!(
                            clock.timing_at(PhysicalTime::new(value)),
                            wide_timing(clock, value)
                        );
                        let timing = i64::MIN.wrapping_add(value as i64);
                        for ceil in [false, true] {
                            assert_eq!(
                                clock.physical_at(TimingTime::new(timing), ceil),
                                wide_physical(clock, timing, ceil)
                            );
                        }
                    }
                }
            }
            let mut state = 0x5eed_u64;
            for _ in 0..10_000 {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1);
                for anchor in [0, 1, u64::MAX - 1, u64::MAX, state] {
                    for timing_anchor in [i64::MIN, -1, 0, i64::MAX, state as i64] {
                        let clock = ClockMapping::new(
                            profile,
                            PhysicalTime::new(anchor),
                            TimingTime::new(timing_anchor),
                        );
                        for value in [0, u64::MAX, state] {
                            assert_eq!(
                                clock.timing_at(PhysicalTime::new(value)),
                                wide_timing(clock, value)
                            );
                            for ceil in [false, true] {
                                assert_eq!(
                                    clock.physical_at(TimingTime::new(value as i64), ceil),
                                    wide_physical(clock, value as i64, ceil)
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn boot_mapping_selects_fixed_profile_and_zero_anchors() {
        let (profile, timing_value, physical_ceiling) = if cfg!(feature = "legacy-rtc-timing") {
            (ClockProfile::LegacyNrfRtc10000, 32_768, 18_311)
        } else {
            (ClockProfile::Normalized, 30_000, 20_000)
        };
        let mapping = boot_mapping();
        assert_eq!(mapping.profile(), profile);
        assert_eq!(mapping.physical_anchor(), PhysicalTime::new(0));
        assert_eq!(mapping.timing_anchor(), TimingTime::new(0));
        let physical = PhysicalTime::new(30_000);
        let timing = TimingTime::new(timing_value);
        assert_eq!(mapping.timing_at(physical), Ok(timing));
        assert_eq!(
            mapping.physical_not_before(TimingTime::new(20_000)),
            Ok(PhysicalTime::new(physical_ceiling))
        );
        let mut clock = TdmaClock::new(mapping);
        assert_eq!(clock.sample(physical), Ok(timing));
        assert_eq!(
            clock.last_reading(),
            Some(ClockReading { physical, timing })
        );
    }

    #[test]
    fn value_types_are_copy_and_eq() {
        fn assert_traits<T: Copy + Eq>() {}

        assert_traits::<PhysicalTime>();
        assert_traits::<TimingTime>();
        assert_traits::<PhysicalWindow>();
        assert_traits::<ClockProfile>();
        assert_traits::<ClockMapping>();
        assert_traits::<ClockReading>();
        assert_traits::<ClockError>();
    }

    #[test]
    fn production_owner_normalized_pair_is_the_exact_sample() {
        let mut clock = TdmaClock::new(mapping(ClockProfile::Normalized));
        assert_eq!(clock.last_timing(), TimingTime::new(0));
        assert_eq!(clock.last_reading(), None);
        assert_eq!(
            clock.sample(PhysicalTime::new(1_000)),
            Ok(TimingTime::new(1_000))
        );
        assert_eq!(
            clock.last_reading(),
            Some(ClockReading {
                physical: PhysicalTime::new(1_000),
                timing: TimingTime::new(1_000),
            })
        );
    }

    #[test]
    fn production_owner_legacy_pair_survives_other_physical_observations() {
        let mut clock = TdmaClock::new(mapping(ClockProfile::LegacyNrfRtc10000));
        assert_eq!(
            clock.sample(PhysicalTime::new(1_000)),
            Ok(TimingTime::new(1_092))
        );
        let pair = Some(ClockReading {
            physical: PhysicalTime::new(1_000),
            timing: TimingTime::new(1_092),
        });
        assert_eq!(clock.last_reading(), pair);
        // Read-only conversions and later physical reads do not sample the owner.
        let later_physical = PhysicalTime::new(2_000);
        assert_eq!(
            clock.mapping().timing_at(later_physical),
            Ok(TimingTime::new(2_184))
        );
        assert_eq!(clock.last_reading(), pair);
        assert_eq!(clock.last_timing(), TimingTime::new(1_092));
    }

    #[test]
    fn production_owner_before_anchor_failure_clears_pair_and_retains_time() {
        let mapping = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(100),
            TimingTime::new(-50),
        );
        let mut clock = TdmaClock::new(mapping);
        assert_eq!(
            clock.sample(PhysicalTime::new(110)),
            Ok(TimingTime::new(-40))
        );
        assert_eq!(
            clock.sample(PhysicalTime::new(99)),
            Err(ClockError::PhysicalBeforeAnchor)
        );
        assert_eq!(clock.last_reading(), None);
        assert_eq!(clock.last_timing(), TimingTime::new(-40));
        assert_eq!(clock.mapping(), mapping);
    }

    #[test]
    fn production_owner_overflow_clears_pair_and_retains_time() {
        let mut clock = TdmaClock::new(ClockMapping::new(
            ClockProfile::LegacyNrfRtc10000,
            PhysicalTime::new(0),
            TimingTime::new(i64::MAX - 1),
        ));
        assert_eq!(
            clock.sample(PhysicalTime::new(1)),
            Ok(TimingTime::new(i64::MAX))
        );
        assert_eq!(
            clock.sample(PhysicalTime::new(2)),
            Err(ClockError::TimingOverflow)
        );
        assert_eq!(clock.last_reading(), None);
        assert_eq!(clock.last_timing(), TimingTime::new(i64::MAX));
    }

    #[test]
    fn production_owner_initial_failure_keeps_zero_diagnostic_fallback() {
        let mut clock = TdmaClock::new(ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(10),
            TimingTime::new(-100),
        ));
        assert_eq!(
            clock.sample(PhysicalTime::new(0)),
            Err(ClockError::PhysicalBeforeAnchor)
        );
        assert_eq!(clock.last_reading(), None);
        assert_eq!(clock.last_timing(), TimingTime::new(0));
    }

    #[test]
    fn production_owner_clear_preserves_mapping_and_last_timing() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = ClockMapping::new(profile, PhysicalTime::new(100), TimingTime::new(-10));
            let mut clock = TdmaClock::new(mapping);
            let timing = clock.sample(PhysicalTime::new(1_000)).unwrap();
            clock.clear_reading();
            assert_eq!(clock.last_reading(), None);
            assert_eq!(clock.last_timing(), timing);
            assert_eq!(clock.mapping(), mapping);
            assert_eq!(clock.mapping().profile(), profile);
            // A copied mapping can change without changing the owner's mapping.
            let other = match profile {
                ClockProfile::Normalized => ClockProfile::LegacyNrfRtc10000,
                ClockProfile::LegacyNrfRtc10000 => ClockProfile::Normalized,
            };
            let changed = clock
                .mapping()
                .reprofile(other, PhysicalTime::new(1_000))
                .unwrap();
            assert_eq!(changed.profile(), other);
            assert_eq!(clock.mapping(), mapping);
        }
    }

    #[test]
    fn normalized_identity_and_signed_anchored_offsets() {
        let identity = mapping(ClockProfile::Normalized);
        for value in [0, 1, 20_000, i64::MAX as u64] {
            let physical = PhysicalTime::new(value);
            let timing = TimingTime::new(value as i64);
            assert_eq!(identity.timing_at(physical), Ok(timing));
            assert_eq!(identity.physical_not_before(timing), Ok(physical));
            assert_eq!(identity.physical_not_after(timing), Ok(physical));
        }
        for timing_anchor in [i64::MIN, -20_000, 0, i64::MAX - 100] {
            let clock = ClockMapping::new(
                ClockProfile::Normalized,
                PhysicalTime::new(u64::MAX - 100),
                TimingTime::new(timing_anchor),
            );
            assert_eq!(clock.profile(), ClockProfile::Normalized);
            assert_eq!(clock.physical_anchor().value(), u64::MAX - 100);
            assert_eq!(clock.timing_anchor().value(), timing_anchor);
            for delta in [0, 1, 100] {
                let physical = PhysicalTime::new(u64::MAX - 100 + delta);
                let timing = TimingTime::new(timing_anchor + delta as i64);
                assert_eq!(clock.timing_at(physical), Ok(timing));
                assert_eq!(clock.physical_not_before(timing), Ok(physical));
                assert_eq!(clock.physical_not_after(timing), Ok(physical));
            }
        }
    }

    #[test]
    fn legacy_source_ratio_and_twenty_thousand_timing_units() {
        let clock = mapping(ClockProfile::LegacyNrfRtc10000);
        assert_eq!(clock.profile().ratio(), (32_768, 30_000));
        assert_eq!(
            clock.timing_at(PhysicalTime::new(30_000)).unwrap().value(),
            32_768
        );
        let target = TimingTime::new(20_000);
        assert_eq!(clock.physical_not_before(target).unwrap().value(), 18_311);
        assert_eq!(clock.physical_not_after(target).unwrap().value(), 18_310);
        assert_eq!(
            clock.timing_at(PhysicalTime::new(18_310)).unwrap().value(),
            19_999
        );
        assert_eq!(
            clock.timing_at(PhysicalTime::new(18_311)).unwrap().value(),
            20_000
        );
        let aligned = ClockMapping::new(
            ClockProfile::LegacyNrfRtc10000,
            PhysicalTime::new(u64::MAX - 30_000),
            TimingTime::new(-32_768),
        );
        assert_eq!(
            aligned.timing_at(PhysicalTime::new(u64::MAX)),
            Ok(TimingTime::new(0))
        );
        assert_eq!(
            aligned.physical_not_before(TimingTime::new(0)),
            Ok(PhysicalTime::new(u64::MAX))
        );
        assert_eq!(
            aligned.physical_not_after(TimingTime::new(0)),
            Ok(PhysicalTime::new(u64::MAX))
        );
    }

    #[test]
    fn before_anchor_and_range_errors_are_explicit() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let clock = ClockMapping::new(profile, PhysicalTime::new(10), TimingTime::new(-5));
            assert_eq!(
                clock.timing_at(PhysicalTime::new(9)),
                Err(ClockError::PhysicalBeforeAnchor)
            );
            for target in [i64::MIN, -6] {
                assert_eq!(
                    clock.physical_not_before(TimingTime::new(target)),
                    Err(ClockError::TimingBeforeAnchor)
                );
                assert_eq!(
                    clock.physical_not_after(TimingTime::new(target)),
                    Err(ClockError::TimingBeforeAnchor)
                );
            }
            for anchor in [0, i64::MAX] {
                let clock =
                    ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(anchor));
                assert_eq!(
                    clock.timing_at(PhysicalTime::new(u64::MAX)),
                    Err(ClockError::TimingOverflow)
                );
            }
            let clock = ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(i64::MAX));
            assert_eq!(
                clock.timing_at(PhysicalTime::new(1)),
                Err(ClockError::TimingOverflow)
            );
            for anchor in [u64::MAX - 1, u64::MAX] {
                let clock =
                    ClockMapping::new(profile, PhysicalTime::new(anchor), TimingTime::new(0));
                assert_eq!(
                    clock.physical_not_before(TimingTime::new(i64::MAX)),
                    Err(ClockError::PhysicalOverflow)
                );
                assert_eq!(
                    clock.physical_not_after(TimingTime::new(i64::MAX)),
                    Err(ClockError::PhysicalOverflow)
                );
            }
        }
    }

    #[test]
    fn full_signed_timing_span_uses_wide_arithmetic() {
        let clock = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(i64::MIN),
        );
        let target = TimingTime::new(i64::MAX);
        assert_eq!(
            clock.physical_not_before(target),
            Ok(PhysicalTime::new(u64::MAX))
        );
        assert_eq!(
            clock.physical_not_after(target),
            Ok(PhysicalTime::new(u64::MAX))
        );
        assert_eq!(clock.timing_at(PhysicalTime::new(u64::MAX)), Ok(target));
        let shifted =
            ClockMapping::new(clock.profile(), PhysicalTime::new(1), clock.timing_anchor());
        assert_eq!(
            shifted.physical_not_before(target),
            Err(ClockError::PhysicalOverflow)
        );
        assert_eq!(
            shifted.physical_not_after(target),
            Err(ClockError::PhysicalOverflow)
        );
        let legacy = ClockMapping::new(
            ClockProfile::LegacyNrfRtc10000,
            PhysicalTime::new(0),
            TimingTime::new(i64::MIN),
        );
        let open = legacy.physical_not_before(target).unwrap();
        let close = legacy.physical_not_after(target).unwrap();
        let span = u128::from(u64::MAX) * 30_000;
        assert!(u128::from(open.value()) * 32_768 >= span);
        assert!(u128::from(close.value()) * 32_768 <= span);
    }

    #[test]
    fn invalid_and_rounded_empty_windows_are_rejected() {
        let clock = mapping(ClockProfile::LegacyNrfRtc10000);
        for (start, end) in [(1, 1), (2, 1)] {
            assert_eq!(
                clock.convert(TimingTime::new(start), TimingTime::new(end)),
                Err(ClockError::InvalidWindow)
            );
        }
        for (start, end) in [(0, 1), (1, 2)] {
            assert_eq!(
                clock.convert(TimingTime::new(start), TimingTime::new(end)),
                Err(ClockError::RoundedEmptyWindow)
            );
        }
        assert_eq!(
            clock.convert(TimingTime::new(-1), TimingTime::new(2)),
            Err(ClockError::TimingBeforeAnchor)
        );
        let window = mapping(ClockProfile::Normalized)
            .convert(TimingTime::new(1), TimingTime::new(2))
            .unwrap();
        assert_eq!(window.start().value(), 1);
        assert_eq!(window.deadline().value(), 2);
    }

    #[test]
    fn deterministic_intervals_obey_continuous_rational_limits() {
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let clock = ClockMapping::new(
                profile,
                PhysicalTime::new(1_000_000),
                TimingTime::new(-50_000),
            );
            let (numerator, denominator) = profile.ratio();
            let mut state = 0x5eed_u64;
            for _ in 0..10_000 {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1);
                let start_delta = state % 1_000_000;
                let end_delta = start_delta + 3 + (state >> 32) % 10_000;
                let start = TimingTime::new(-50_000 + start_delta as i64);
                let end = TimingTime::new(-50_000 + end_delta as i64);
                let window = clock.convert(start, end).unwrap();
                let open_delta = window.start().value() - clock.physical_anchor().value();
                let close_delta = window.deadline().value() - clock.physical_anchor().value();
                let start_limit = u128::from(start_delta) * u128::from(denominator);
                let end_limit = u128::from(end_delta) * u128::from(denominator);
                assert!(u128::from(open_delta) * u128::from(numerator) >= start_limit);
                assert!(u128::from(close_delta) * u128::from(numerator) <= end_limit);
                assert!(u128::from(close_delta + 1) * u128::from(numerator) > end_limit);
                assert!(clock.timing_at(window.start()).unwrap().value() >= start.value());
                if open_delta > 0 {
                    let prior = PhysicalTime::new(window.start().value() - 1);
                    assert!(clock.timing_at(prior).unwrap().value() < start.value());
                    assert!(u128::from(open_delta - 1) * u128::from(numerator) < start_limit);
                }
                let end_ceil = clock.physical_not_before(end).unwrap().value();
                assert!(window.deadline().value() <= end_ceil);
                assert!(end_ceil - window.deadline().value() <= 1);
            }
        }
    }

    #[test]
    fn different_profiles_preserve_current_integer_time_and_future_monotonicity() {
        for (old, new) in [
            (ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000),
            (ClockProfile::LegacyNrfRtc10000, ClockProfile::Normalized),
        ] {
            let clock = ClockMapping::new(old, PhysicalTime::new(100), TimingTime::new(-500));
            let now = PhysicalTime::new(117);
            let current = clock.timing_at(now).unwrap();
            let changed = clock.reprofile(new, now).unwrap();
            assert_eq!(changed.profile(), new);
            assert_eq!(changed.physical_anchor(), now);
            assert_eq!(changed.timing_anchor(), current);
            assert_eq!(changed.timing_at(now), Ok(current));
            assert!(changed.timing_at(PhysicalTime::new(118)).unwrap().value() > current.value());
            assert_eq!(
                changed.timing_at(PhysicalTime::new(116)),
                Err(ClockError::PhysicalBeforeAnchor)
            );
            assert_eq!(clock.physical_anchor(), PhysicalTime::new(100));
            assert_eq!(
                clock.reprofile(new, PhysicalTime::new(99)),
                Err(ClockError::PhysicalBeforeAnchor)
            );
        }
    }

    #[test]
    fn same_profile_preserves_fractional_carry_and_exact_mapping() {
        let clock = mapping(ClockProfile::LegacyNrfRtc10000);
        let mut unchanged = clock;
        for now in 1..100 {
            unchanged = unchanged
                .reprofile(clock.profile(), PhysicalTime::new(now))
                .unwrap();
            assert_eq!(unchanged, clock);
        }
        assert_eq!(
            unchanged.timing_at(PhysicalTime::new(11)),
            Ok(TimingTime::new(12))
        );
        let reset = ClockMapping::new(clock.profile(), PhysicalTime::new(1), TimingTime::new(1));
        assert_eq!(
            reset.timing_at(PhysicalTime::new(11)),
            Ok(TimingTime::new(11))
        );
        let anchored =
            ClockMapping::new(clock.profile(), PhysicalTime::new(10), TimingTime::new(0));
        assert_eq!(
            anchored.reprofile(clock.profile(), PhysicalTime::new(9)),
            Err(ClockError::PhysicalBeforeAnchor)
        );
    }

    #[test]
    fn reprofile_propagates_timing_overflow_for_a_different_profile() {
        let clock = ClockMapping::new(
            ClockProfile::Normalized,
            PhysicalTime::new(0),
            TimingTime::new(i64::MAX),
        );
        assert_eq!(
            clock.reprofile(ClockProfile::LegacyNrfRtc10000, PhysicalTime::new(1)),
            Err(ClockError::TimingOverflow)
        );
        assert_eq!(
            clock.reprofile(clock.profile(), PhysicalTime::new(1)),
            Ok(clock)
        );
    }

    #[test]
    fn legacy_control_metadata_converts_before_physical_ticket_admission() {
        fn admits(window: PhysicalWindow, now_us: u64, end_us: u64) -> bool {
            now_us >= window.start().value()
                && now_us < window.deadline().value()
                && end_us >= now_us
                && end_us <= window.deadline().value()
        }

        let clock = mapping(ClockProfile::LegacyNrfRtc10000);
        let frame_end = clock.physical_not_after(TimingTime::new(20_000)).unwrap();
        let control = clock
            .convert(TimingTime::new(16_000), TimingTime::new(17_500))
            .unwrap();
        assert_eq!(control.start().value(), 14_649);
        assert_eq!(control.deadline().value(), 16_021);
        assert_eq!(frame_end.value(), 18_310);
        assert!(control.deadline().value() < frame_end.value());
        let deadline = u128::from(control.deadline().value());
        assert!(deadline * 32_768 < 17_500_u128 * 30_000);
        assert!((deadline + 1) * 32_768 > 17_500_u128 * 30_000);
        assert!(admits(control, 15_000, 16_021));
        assert!(!admits(control, 14_648, 15_000));
        assert!(!admits(control, 15_000, 16_022));
        assert!(!admits(control, 16_021, 16_021));
        assert!(!admits(control, 16_500, 17_000));
        let physical_frame = PhysicalWindow {
            start: PhysicalTime::new(0),
            deadline: frame_end,
        };
        assert!(admits(physical_frame, 16_500, 17_000));
    }
}
