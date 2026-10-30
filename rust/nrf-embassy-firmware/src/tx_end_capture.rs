//! Pure observational TX END capture reducer; no hardware has been validated.
//!
//! Assume a continuous 32-bit, 1 MHz TIMER4 domain for anchor, cutoff, capture
//! and actual observation. No absolute SharedClock conversion is involved.
//! Preparation supplies the cutoff reference and a horizon, not a sampling timestamp.
//! Production callers must use `freeze_at` with actual observation and an elapsed
//! witness from an independent monotonic clock sharing the preparation anchor.
//! The caller must establish clock origins, units, sampling order and continuous
//! counter operation: modular arithmetic alone cannot detect a full counter wrap.
//! Proposed CC2/PPI5 wiring, event clearing, PPI drain/replay and compare timing
//! remain integration work. Even two equal reads never prove a drained pipeline
//! or qualify an RF timestamp. No clocks, registers, callbacks or allocation.

const HALF_RANGE: u32 = 1 << 31;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateValidity {
    /// Both reads equal the excluded seed, including any seed collision.
    Missing,
    /// The two reads differ; even equal reads would not prove pipeline drain.
    Unstable,
    /// The caller did not observe the current-TX completion predicate.
    NoCompletionEvidence,
    /// Actual raw observation or independent elapsed witness exceeds the horizon.
    ObservationOutsideHorizon,
    /// Stable raw value is outside the inclusive anchor..actual-observation bound.
    OutsideEpochRange,
    /// Stable observation only; signed microseconds relative to the raw cutoff.
    Candidate { raw: u32, cutoff_delta_us: i32 },
    /// Preparation was aborted before the explicit arm boundary.
    NotArmed,
    /// Operation rejected without altering or replacing retained diagnostics.
    WrongEpoch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Busy,
    InvalidEpoch,
    EpochExhausted,
    InvalidRange,
    Rejected(CandidateValidity),
    InvalidTransition,
    RouteActive,
    AlreadyFrozen,
}

/// Instructions to the caller, never register writes performed by this model.
/// These copyable actions are not capabilities and cannot be safely replayed.
/// The hardware caller must execute them immediately and serially with the
/// matching epoch check, before any peripheral mutation or next transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Seed { epoch: u64, raw: u32 },
    EnableRoute { epoch: u64 },
    DisableRoute { epoch: u64 },
    Retired { epoch: u64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Prepared,
    Armed,
    Disarmed,
}

/// Epoch-qualified range and software contract history. Fields are copies;
/// changing them cannot change the owner's state or validate hardware events.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub epoch: u64,
    pub anchor_raw: u32,
    pub cutoff_raw: u32,
    pub horizon_us: u32,
    pub seed_raw: u32,
    pub stage: Stage,
    /// Explicit arm boundary declared by caller, not proof END was cleared.
    pub arm_boundary_recorded: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frozen {
    pub snapshot: Snapshot,
    pub sampled_cc: [u32; 2],
    /// Actual observation supplied to freeze_at.
    pub actual_observation_raw: u32,
    /// Independent monotonic elapsed witness sharing the preparation anchor.
    pub elapsed_us: u64,
    /// Both raw observation distance and elapsed witness are within the horizon.
    /// This records numerical checks, not hardware or clock qualification.
    pub observation_within_horizon: bool,
    pub completion_predicate_observed: bool,
    pub validity: CandidateValidity,
}

impl Frozen {
    /// Reclassify the same retained observation with caller-supplied completion
    /// evidence. The caller must establish the actual current-TX predicate.
    /// This changes only a copy: no sampling, owner transition, hardware/clock
    /// validation or RF qualification follows from changing this flag.
    pub fn with_completion_evidence(mut self) -> Self {
        self.completion_predicate_observed = true;
        self.validity = self.classify(true);
        self
    }

    fn classify(&self, completion_predicate_observed: bool) -> CandidateValidity {
        let snapshot = self.snapshot;
        let raw = self.sampled_cc[0];
        let offset = raw.wrapping_sub(snapshot.anchor_raw);
        let end = self
            .actual_observation_raw
            .wrapping_sub(snapshot.anchor_raw);
        let cutoff = snapshot.cutoff_raw.wrapping_sub(snapshot.anchor_raw);
        if !snapshot.arm_boundary_recorded {
            CandidateValidity::NotArmed
        } else if !self.observation_within_horizon {
            CandidateValidity::ObservationOutsideHorizon
        } else if self.sampled_cc[0] != self.sampled_cc[1] {
            CandidateValidity::Unstable
        } else if raw == snapshot.seed_raw {
            CandidateValidity::Missing
        } else if !completion_predicate_observed {
            CandidateValidity::NoCompletionEvidence
        } else if offset >= HALF_RANGE || offset > end {
            CandidateValidity::OutsideEpochRange
        } else {
            // Both offsets from owner preparation are below half range. Use a
            // checked signed difference even for externally edited copies.
            match i32::try_from(i64::from(offset) - i64::from(cutoff)) {
                Ok(cutoff_delta_us) => CandidateValidity::Candidate {
                    raw,
                    cutoff_delta_us,
                },
                Err(_) => CandidateValidity::OutsideEpochRange,
            }
        }
    }
}

/// One exclusive reducer for the entire hardware-owner lifetime (not Clone).
/// Epochs are opaque read-only numeric identities supplied by the owner. They
/// must increase strictly, never reuse or wrap; u64::MAX is a valid final epoch.
/// Do not recreate this reducer within an owner lifetime to reset that history.
/// Exactly one frozen result is retained, including while the next epoch runs.
#[derive(Debug, Default)]
pub struct ExclusiveOwner {
    last_epoch: u64,
    current: Option<Snapshot>,
    frozen: Option<Frozen>,
}

impl ExclusiveOwner {
    pub const fn new() -> Self {
        Self {
            last_epoch: 0,
            current: None,
            frozen: None,
        }
    }

    pub fn current(&self) -> Option<Snapshot> {
        self.current
    }

    pub fn frozen(&self) -> Option<Frozen> {
        self.frozen
    }

    /// Stage preparation BEFORE capture enable or launch. Returns an excluded
    /// seed for the caller to write later. Horizon is caller policy, not an RF
    /// delay allowance. The cutoff's forward offset must be positive and within
    /// the nonzero horizon, which is strictly below half range. Cutoff is a delta
    /// reference, not an actual observation or a capture upper bound.
    pub fn prepare(
        &mut self,
        epoch: u64,
        anchor_raw: u32,
        cutoff_raw: u32,
        horizon_us: u32,
    ) -> Result<Action, Error> {
        if self.current.is_some() {
            return Err(Error::Busy);
        }
        if self.last_epoch == u64::MAX {
            return Err(Error::EpochExhausted);
        }
        if epoch == 0 || epoch <= self.last_epoch {
            return Err(Error::InvalidEpoch);
        }
        let cutoff = cutoff_raw.wrapping_sub(anchor_raw);
        if horizon_us == 0 || horizon_us >= HALF_RANGE || cutoff == 0 || cutoff > horizon_us {
            return Err(Error::InvalidRange);
        }
        let seed_raw = anchor_raw.wrapping_sub(1);
        self.current = Some(Snapshot {
            epoch,
            anchor_raw,
            cutoff_raw,
            horizon_us,
            seed_raw,
            stage: Stage::Prepared,
            arm_boundary_recorded: false,
        });
        self.last_epoch = epoch;
        Ok(Action::Seed {
            epoch,
            raw: seed_raw,
        })
    }

    /// Caller explicitly declares the boundary AFTER RADIO prepare clears END
    /// and BEFORE launch. This records the contract, not actual event clearing.
    pub fn arm(&mut self, epoch: u64) -> Result<Action, Error> {
        let mut snapshot = self.matching(epoch)?;
        if snapshot.stage != Stage::Prepared {
            return Err(Error::InvalidTransition);
        }
        snapshot.stage = Stage::Armed;
        snapshot.arm_boundary_recorded = true;
        self.current = Some(snapshot);
        Ok(Action::EnableRoute { epoch })
    }

    /// Abort or normal disarm makes the modeled route inactive while retaining
    /// diagnostics. Caller must execute DisableRoute before sampling for freeze.
    pub fn disarm(&mut self, epoch: u64) -> Result<Action, Error> {
        let mut snapshot = self.matching(epoch)?;
        if snapshot.stage == Stage::Disarmed {
            return Err(Error::InvalidTransition);
        }
        snapshot.stage = Stage::Disarmed;
        self.current = Some(snapshot);
        Ok(Action::DisableRoute { epoch })
    }

    /// Sample twice only AFTER explicitly disabling the route. The caller must
    /// bound both CC reads by actual_observation_raw in the continuous 32-bit,
    /// 1 MHz capture domain and supply elapsed_us from an independent monotonic
    /// clock with the same preparation anchor. Clock origin, continuity, rate,
    /// units and sampling order remain caller proof obligations, not validation
    /// performed by this reducer. Elapsed and raw distances need not be numerically
    /// equal: separate clocks and sampling order may differ. Both must be within
    /// the configured horizon (< half range). Zero is allowed, including a real
    /// capture and observation at the anchor; no positive minimum is imposed.
    ///
    /// Actual observation may precede cutoff. Captures in the inclusive
    /// anchor..actual-observation interval can be candidates even after cutoff;
    /// cutoff is used only for the signed delta, not as a capture upper bound.
    /// Both actual observation witnesses are checked against the horizon.
    ///
    /// Freeze retires the matching disarmed epoch and retains invalid bounds as
    /// diagnostics, never vendor/send faults. Stable reads do not assert PPI drain.
    /// Reasons take precedence: not armed, observation outside horizon, unstable,
    /// missing, no completion, outside range, candidate. Rejected operations do
    /// not replace the result or mutate a newer epoch.
    pub fn freeze_at(
        &mut self,
        epoch: u64,
        sampled_cc: [u32; 2],
        actual_observation_raw: u32,
        elapsed_us: u64,
        completion_predicate_observed: bool,
    ) -> Result<Frozen, Error> {
        let snapshot = self.matching(epoch)?;
        if snapshot.stage != Stage::Disarmed {
            return Err(Error::RouteActive);
        }
        let end = actual_observation_raw.wrapping_sub(snapshot.anchor_raw);
        let observation_within_horizon =
            end <= snapshot.horizon_us && elapsed_us <= u64::from(snapshot.horizon_us);
        let mut frozen = Frozen {
            snapshot,
            sampled_cc,
            actual_observation_raw,
            elapsed_us,
            observation_within_horizon,
            completion_predicate_observed,
            validity: CandidateValidity::NoCompletionEvidence,
        };
        frozen.validity = frozen.classify(completion_predicate_observed);
        self.frozen = Some(frozen);
        self.current = None;
        Ok(frozen)
    }

    /// Retire without a sampled result only after disarm. Keeps the last frozen
    /// result; a stale cancellation cannot retire a newer current epoch.
    pub fn retire(&mut self, epoch: u64) -> Result<Action, Error> {
        if self.matching(epoch)?.stage != Stage::Disarmed {
            return Err(Error::RouteActive);
        }
        self.current = None;
        Ok(Action::Retired { epoch })
    }

    fn matching(&self, epoch: u64) -> Result<Snapshot, Error> {
        match self.current {
            Some(snapshot) if snapshot.epoch == epoch => Ok(snapshot),
            None if self
                .frozen
                .is_some_and(|result| result.snapshot.epoch == epoch) =>
            {
                Err(Error::AlreadyFrozen)
            }
            _ => Err(Error::Rejected(CandidateValidity::WrongEpoch)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed(anchor: u32, cutoff: u32) -> ExclusiveOwner {
        let mut owner = ExclusiveOwner::new();
        owner.prepare(1, anchor, cutoff, 100).unwrap();
        assert_eq!(owner.arm(1), Ok(Action::EnableRoute { epoch: 1 }));
        owner
    }

    fn classify(anchor: u32, cutoff: u32, end: u32, reads: [u32; 2], completion: bool) -> Frozen {
        // Explicit test observation at end, with elapsed measured from anchor
        // in this fixture's single-rate clock model.
        let mut owner = armed(anchor, cutoff);
        owner.disarm(1).unwrap();
        owner
            .freeze_at(
                1,
                reads,
                end,
                u64::from(end.wrapping_sub(anchor)),
                completion,
            )
            .unwrap()
    }

    #[test]
    fn completion_copy_reclassifies_only_valid_bounds_without_mutating_owner() {
        let anchor = u32::MAX - 9;
        for (raw, expected) in [
            (
                anchor,
                CandidateValidity::Candidate {
                    raw: anchor,
                    cutoff_delta_us: -15,
                },
            ),
            (
                5,
                CandidateValidity::Candidate {
                    raw: 5,
                    cutoff_delta_us: 0,
                },
            ),
            (
                10,
                CandidateValidity::Candidate {
                    raw: 10,
                    cutoff_delta_us: 5,
                },
            ),
            (11, CandidateValidity::OutsideEpochRange),
            (
                anchor.wrapping_add(HALF_RANGE),
                CandidateValidity::OutsideEpochRange,
            ),
        ] {
            let mut owner = armed(anchor, 5);
            owner.disarm(1).unwrap();
            let original = owner.freeze_at(1, [raw; 2], 10, 20, false).unwrap();
            assert_eq!(original.validity, CandidateValidity::NoCompletionEvidence);
            // A retained copy can be queried while a newer route is armed.
            owner.prepare(2, 200, 210, 100).unwrap();
            owner.arm(2).unwrap();
            let before = (owner.current(), owner.frozen());
            let report = original.with_completion_evidence();
            assert_eq!(report.validity, expected);
            assert_eq!(
                report,
                Frozen {
                    completion_predicate_observed: true,
                    validity: expected,
                    ..original
                }
            );
            assert_eq!((owner.current(), owner.frozen()), before);
            assert_eq!(owner.frozen(), Some(original));
            assert!(!original.completion_predicate_observed);
            assert_eq!(report.with_completion_evidence(), report);
        }
    }

    #[test]
    fn completion_copy_preserves_prior_reasons_and_observation_context() {
        for (armed, reads, actual, elapsed, expected) in [
            (true, [99; 2], 120, 20, CandidateValidity::Missing),
            (true, [99, 115], 120, 20, CandidateValidity::Unstable),
            (
                true,
                [115; 2],
                201,
                20,
                CandidateValidity::ObservationOutsideHorizon,
            ),
            (
                true,
                [115; 2],
                120,
                101,
                CandidateValidity::ObservationOutsideHorizon,
            ),
            (false, [99, 115], 201, 101, CandidateValidity::NotArmed),
        ] {
            let mut owner = ExclusiveOwner::new();
            owner.prepare(1, 100, 110, 100).unwrap();
            if armed {
                owner.arm(1).unwrap();
            }
            owner.disarm(1).unwrap();
            let original = owner.freeze_at(1, reads, actual, elapsed, false).unwrap();
            assert_eq!(original.validity, expected);
            let report = original.with_completion_evidence();
            assert_eq!(
                report,
                Frozen {
                    completion_predicate_observed: true,
                    ..original
                }
            );
            assert_eq!(owner.frozen(), Some(original));
            assert_eq!(owner.current(), None);
        }
    }

    #[test]
    fn lifecycle_requires_boundary_disable_and_retirement() {
        let mut owner = ExclusiveOwner::new();
        assert_eq!(
            owner.prepare(7, 100, 110, 20),
            Ok(Action::Seed { epoch: 7, raw: 99 })
        );
        assert_eq!(owner.current().unwrap().stage, Stage::Prepared);
        assert_eq!(owner.prepare(8, 100, 110, 20), Err(Error::Busy));
        owner.arm(7).unwrap();
        assert_eq!(owner.arm(7), Err(Error::InvalidTransition));
        assert_eq!(
            owner.freeze_at(7, [110; 2], 120, 20, true),
            Err(Error::RouteActive)
        );
        assert_eq!(owner.retire(7), Err(Error::RouteActive));
        owner.disarm(7).unwrap();
        assert_eq!(owner.prepare(8, 100, 110, 20), Err(Error::Busy));
        let result = owner.freeze_at(7, [110; 2], 120, 20, true).unwrap();
        assert!(result.snapshot.arm_boundary_recorded);
        assert_eq!(
            result.validity,
            CandidateValidity::Candidate {
                raw: 110,
                cutoff_delta_us: 0
            }
        );
        assert_eq!(
            owner.freeze_at(7, [111; 2], 120, 20, true),
            Err(Error::AlreadyFrozen)
        );
        assert_eq!(owner.frozen(), Some(result));
        assert_eq!(owner.prepare(7, 100, 110, 20), Err(Error::InvalidEpoch));
        owner.prepare(8, 100, 110, 20).unwrap();
        assert_eq!(owner.frozen(), Some(result));
        owner.disarm(8).unwrap();
        let unarmed = owner.freeze_at(8, [110; 2], 120, 20, true).unwrap();
        assert_eq!(unarmed.validity, CandidateValidity::NotArmed);
        assert_eq!(owner.frozen(), Some(unarmed));
    }

    #[test]
    fn stale_cancel_freeze_and_arm_cannot_change_new_route() {
        let mut owner = armed(100, 110);
        owner.disarm(1).unwrap();
        owner.retire(1).unwrap();
        owner.prepare(2, 200, 210, 20).unwrap();
        owner.arm(2).unwrap();
        let before = owner.current();
        let wrong = Err(Error::Rejected(CandidateValidity::WrongEpoch));
        assert_eq!(owner.disarm(1), wrong);
        assert_eq!(owner.retire(1), wrong);
        assert_eq!(owner.arm(1), wrong);
        assert_eq!(
            owner.freeze_at(1, [210; 2], 220, 20, true),
            Err(Error::Rejected(CandidateValidity::WrongEpoch))
        );
        assert_eq!(owner.current(), before);
        owner.disarm(2).unwrap();
        assert_eq!(owner.disarm(2), Err(Error::InvalidTransition));
        owner.freeze_at(2, [215; 2], 220, 20, true).unwrap();
    }

    #[test]
    fn stale_operations_preserve_new_current_and_previous_frozen() {
        let mut owner = armed(100, 110);
        owner.disarm(1).unwrap();
        let previous = owner.freeze_at(1, [115; 2], 120, 20, true).unwrap();
        owner.prepare(2, 200, 210, 20).unwrap();

        // Reject the frozen epoch at every live stage, including stages where
        // arm or retire would otherwise be valid for the matching current epoch.
        for stage in [Stage::Prepared, Stage::Armed, Stage::Disarmed] {
            match stage {
                Stage::Prepared => {}
                Stage::Armed => {
                    owner.arm(2).unwrap();
                }
                Stage::Disarmed => {
                    owner.disarm(2).unwrap();
                }
            }
            let before = (owner.current(), owner.frozen());
            assert_eq!(before.0.unwrap().stage, stage);
            assert_eq!(before.1, Some(previous));
            let wrong = Err(Error::Rejected(CandidateValidity::WrongEpoch));
            assert_eq!(owner.arm(1), wrong);
            assert_eq!((owner.current(), owner.frozen()), before);
            assert_eq!(owner.disarm(1), wrong);
            assert_eq!((owner.current(), owner.frozen()), before);
            assert_eq!(
                owner.freeze_at(1, [210; 2], 220, u64::MAX, true),
                Err(Error::Rejected(CandidateValidity::WrongEpoch))
            );
            assert_eq!((owner.current(), owner.frozen()), before);
            assert_eq!(owner.retire(1), wrong);
            assert_eq!((owner.current(), owner.frozen()), before);
        }

        assert_eq!(owner.retire(2), Ok(Action::Retired { epoch: 2 }));
        assert_eq!(owner.current(), None);
        assert_eq!(owner.frozen(), Some(previous));
    }

    #[test]
    fn candidate_bounds_and_cutoff_deltas_across_raw_wrap() {
        let anchor = u32::MAX - 9;
        for (raw, delta) in [(anchor, -15), (u32::MAX, -6), (0, -5), (5, 0), (10, 5)] {
            assert_eq!(
                classify(anchor, 5, 10, [raw; 2], true).validity,
                CandidateValidity::Candidate {
                    raw,
                    cutoff_delta_us: delta
                }
            );
        }
        for raw in [11, anchor.wrapping_add(HALF_RANGE), anchor.wrapping_sub(2)] {
            assert_eq!(
                classify(anchor, 5, 10, [raw; 2], true).validity,
                CandidateValidity::OutsideEpochRange
            );
        }
    }

    #[test]
    fn early_completion_uses_actual_observation_before_cutoff() {
        let mut owner = armed(100, 150);
        owner.disarm(1).unwrap();
        // Independent clocks/order need not give identical numeric distances.
        let result = owner.freeze_at(1, [120; 2], 125, 26, true).unwrap();
        assert_eq!(result.actual_observation_raw, 125);
        assert_eq!(result.elapsed_us, 26);
        assert!(result.observation_within_horizon);
        assert_eq!(
            result.validity,
            CandidateValidity::Candidate {
                raw: 120,
                cutoff_delta_us: -30
            }
        );
    }

    #[test]
    fn exact_cutoff_and_late_end_are_candidates_within_actual_bound() {
        for (raw, delta) in [(150, 0), (170, 20), (180, 30)] {
            // Cutoff is only a delta reference; the actual observation bounds
            // the capture, and both witnesses remain within the horizon.
            let mut owner = armed(100, 150);
            owner.disarm(1).unwrap();
            let result = owner.freeze_at(1, [raw; 2], 180, 81, true).unwrap();
            assert_eq!(
                result.validity,
                CandidateValidity::Candidate {
                    raw,
                    cutoff_delta_us: delta
                }
            );
        }
    }

    #[test]
    fn observation_outside_1450_horizon_is_retained_as_diagnostics() {
        // Check each witness independently, including raw before anchor and
        // half-range ambiguity. Neither witness can rescue the other.
        for (actual, elapsed) in [(1551, 1450), (1550, 1451), (99, 0), (100 + HALF_RANGE, 0)] {
            let mut owner = ExclusiveOwner::new();
            owner.prepare(1, 100, 750, 1450).unwrap();
            owner.arm(1).unwrap();
            owner.disarm(1).unwrap();
            let result = owner.freeze_at(1, [750; 2], actual, elapsed, true).unwrap();
            assert_eq!(
                result.validity,
                CandidateValidity::ObservationOutsideHorizon
            );
            assert_eq!(result.actual_observation_raw, actual);
            assert_eq!(result.elapsed_us, elapsed);
            assert_eq!(result.sampled_cc, [750; 2]);
            assert!(result.completion_predicate_observed);
            assert!(!result.observation_within_horizon);
            assert_eq!(owner.current(), None);
            assert_eq!(owner.frozen(), Some(result));
        }
    }

    #[test]
    fn independent_elapsed_rejects_full_wrap_with_plausible_raw_observation() {
        let anchor = u32::MAX - 9;
        let mut owner = armed(anchor, 5);
        owner.disarm(1).unwrap();
        let elapsed = (1_u64 << 32) + 20;
        let result = owner.freeze_at(1, [5; 2], 10, elapsed, true).unwrap();
        assert_eq!(10_u32.wrapping_sub(anchor), 20);
        assert_eq!(
            result.validity,
            CandidateValidity::ObservationOutsideHorizon
        );
        assert_eq!(result.actual_observation_raw, 10);
        assert_eq!(result.elapsed_us, elapsed);
        assert!(!result.observation_within_horizon);
        assert_eq!(owner.frozen(), Some(result));
    }

    #[test]
    fn capture_after_actual_observation_is_outside_epoch_even_before_cutoff() {
        let mut owner = armed(100, 150);
        owner.disarm(1).unwrap();
        let result = owner.freeze_at(1, [130; 2], 125, 25, true).unwrap();
        assert!(result.observation_within_horizon);
        assert_eq!(result.validity, CandidateValidity::OutsideEpochRange);
    }

    #[test]
    fn anchor_and_horizon_endpoints_are_inclusive_for_both_witnesses() {
        for (actual, elapsed, delta) in [(100, 0, -50), (200, 100, 50)] {
            let mut owner = armed(100, 150);
            owner.disarm(1).unwrap();
            let result = owner
                .freeze_at(1, [actual; 2], actual, elapsed, true)
                .unwrap();
            assert!(result.observation_within_horizon);
            assert_eq!(
                result.validity,
                CandidateValidity::Candidate {
                    raw: actual,
                    cutoff_delta_us: delta
                }
            );
        }
    }

    #[test]
    fn invalid_observation_precedes_sample_reasons_and_retains_flags() {
        let mut owner = armed(100, 110);
        owner.disarm(1).unwrap();
        let result = owner.freeze_at(1, [99, 110], 120, 101, false).unwrap();
        assert_eq!(
            result.validity,
            CandidateValidity::ObservationOutsideHorizon
        );
        assert_eq!(result.sampled_cc, [99, 110]);
        assert!(!result.completion_predicate_observed);
        assert!(!result.observation_within_horizon);
    }

    #[test]
    fn seed_collision_unstable_and_completion_gate() {
        // Anchor zero gives an excluded seed at MAX, outside the epoch bound.
        assert_eq!(
            classify(0, 10, 20, [u32::MAX; 2], true).validity,
            CandidateValidity::Missing
        );
        assert_eq!(
            classify(100, 110, 120, [99, 110], true).validity,
            CandidateValidity::Unstable
        );
        assert_eq!(
            classify(100, 110, 120, [115, 116], true).validity,
            CandidateValidity::Unstable
        );
        assert_eq!(
            classify(100, 110, 120, [115; 2], false).validity,
            CandidateValidity::NoCompletionEvidence
        );
        assert_eq!(
            classify(100, 110, 120, [115; 2], true).validity,
            CandidateValidity::Candidate {
                raw: 115,
                cutoff_delta_us: 5
            }
        );
    }

    #[test]
    fn competing_reasons_follow_documented_precedence() {
        for (reads, expected) in [
            ([99; 2], CandidateValidity::Missing),
            ([115, 116], CandidateValidity::Unstable),
            ([121; 2], CandidateValidity::NoCompletionEvidence),
        ] {
            let result = classify(100, 110, 120, reads, false);
            assert_eq!(result.validity, expected);
            assert_eq!(result.sampled_cc, reads);
            assert!(!result.completion_predicate_observed);
        }

        let mut owner = ExclusiveOwner::new();
        owner.prepare(1, 100, 110, 20).unwrap();
        owner.disarm(1).unwrap();
        let unarmed_unstable = owner.freeze_at(1, [115, 116], 120, 20, true).unwrap();
        assert_eq!(unarmed_unstable.validity, CandidateValidity::NotArmed);
        assert!(!unarmed_unstable.snapshot.arm_boundary_recorded);
        assert_eq!(unarmed_unstable.sampled_cc, [115, 116]);
    }

    #[test]
    fn ranges_reject_ambiguity_and_accept_largest_unique_signed_delta() {
        let mut owner = ExclusiveOwner::new();
        for (cutoff, horizon) in [
            (0, 10),
            (1, 0),
            (21, 20),
            (HALF_RANGE, HALF_RANGE - 1),
            (HALF_RANGE, HALF_RANGE),
            (1, HALF_RANGE),
            (u32::MAX, 20),
        ] {
            assert_eq!(
                owner.prepare(1, 0, cutoff, horizon),
                Err(Error::InvalidRange)
            );
            assert_eq!(owner.current(), None);
        }
        owner.prepare(1, 0, HALF_RANGE - 1, HALF_RANGE - 1).unwrap();
        owner.arm(1).unwrap();
        owner.disarm(1).unwrap();
        assert_eq!(
            owner.freeze_at(1, [0; 2], 0, 0, true).unwrap().validity,
            CandidateValidity::Candidate {
                raw: 0,
                cutoff_delta_us: -i32::MAX
            }
        );
        owner.prepare(2, 0, 1, HALF_RANGE - 1).unwrap();
        owner.arm(2).unwrap();
        owner.disarm(2).unwrap();
        assert_eq!(
            owner
                .freeze_at(
                    2,
                    [HALF_RANGE - 1; 2],
                    HALF_RANGE - 1,
                    u64::from(HALF_RANGE - 1),
                    true,
                )
                .unwrap()
                .validity,
            CandidateValidity::Candidate {
                raw: HALF_RANGE - 1,
                cutoff_delta_us: i32::MAX - 1
            }
        );
    }

    #[test]
    fn epochs_are_nonzero_monotonic_and_max_is_final() {
        let mut owner = ExclusiveOwner::new();
        assert_eq!(owner.prepare(0, 0, 1, 1), Err(Error::InvalidEpoch));
        owner.prepare(u64::MAX - 1, 0, 1, 1).unwrap();
        owner.disarm(u64::MAX - 1).unwrap();
        owner.retire(u64::MAX - 1).unwrap();
        assert_eq!(owner.prepare(1, 0, 1, 1), Err(Error::InvalidEpoch));
        owner.prepare(u64::MAX, 0, 1, 1).unwrap();
        owner.disarm(u64::MAX).unwrap();
        owner.freeze_at(u64::MAX, [1; 2], 1, 1, true).unwrap();
        for epoch in [0, 1, u64::MAX] {
            assert_eq!(owner.prepare(epoch, 0, 1, 1), Err(Error::EpochExhausted));
        }
    }
}
