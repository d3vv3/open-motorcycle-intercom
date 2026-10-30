//! Pure policies shared by the TIMER0 adapter and host models.

use nrf_embassy::tdma::{TimerKind, WorkKind};

use crate::timing_clock::{ClockError, ClockMapping, PhysicalTime, TimingTime};

pub const TIMER_KINDS: [TimerKind; 3] = [TimerKind::Frame, TimerKind::Slot, TimerKind::Control];
pub const HALF_RANGE_US: i64 = 1i64 << 31;

pub fn timer_index(kind: TimerKind) -> usize {
    match kind {
        TimerKind::Frame => 0,
        TimerKind::Slot => 1,
        TimerKind::Control => 2,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AlarmRecord {
    pub deadline: i64,
    pub epoch: u64,
}

/// Hardware delays are actual microseconds, not rounded to the core's legacy
/// 100 us correction quantum. A low-word compare must be within half a wrap.
pub fn deadline(now: i64, delay: i64) -> Option<i64> {
    if now < 0 || !(0..HALF_RANGE_US).contains(&delay) {
        return None;
    }
    now.checked_add(delay)
}

/// Signed normalized timing targets share the current physical clock domain.
/// A negative target predates normalized anchor zero and is already due; it is
/// deferred at `now`, never installed as a negative physical deadline. Future
/// legacy-domain targets require a separate conversion before using this API.
pub fn absolute_deadline(now: i64, target: i64) -> Option<i64> {
    if now < 0 {
        return None;
    }
    if target < 0 {
        return Some(now);
    }
    if target <= now || target.checked_sub(now)? < HALF_RANGE_US {
        Some(target)
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmPlanError {
    InvalidDeadline,
    MappingFault,
}

/// Convert a timing-domain target to a physical alarm deadline. Only a proven
/// due target before the timing anchor may fall back to the fresh physical now.
pub fn mapped_absolute_deadline(
    now: i64,
    target: i64,
    mapping: ClockMapping,
) -> Result<i64, ArmPlanError> {
    let physical_now = u64::try_from(now).map_err(|_| ArmPlanError::InvalidDeadline)?;
    let timing_now = mapping
        .timing_at(PhysicalTime::new(physical_now))
        .map_err(|_| ArmPlanError::MappingFault)?;
    let due = target <= timing_now.value();
    let physical_target = match mapping.physical_not_before(TimingTime::new(target)) {
        Ok(physical) => i64::try_from(physical.value()).map_err(|_| ArmPlanError::MappingFault)?,
        Err(ClockError::TimingBeforeAnchor) if due => now,
        Err(_) => return Err(ArmPlanError::MappingFault),
    };
    if due && physical_target > now {
        return Err(ArmPlanError::InvalidDeadline);
    }
    absolute_deadline(now, physical_target).ok_or(ArmPlanError::InvalidDeadline)
}

pub fn next_epoch(previous: u64) -> Option<u64> {
    previous.checked_add(1)
}

/// Recheck the extended clock after writing the low CC word. Includes delay=0.
pub fn programmed_overdue(record: AlarmRecord, after_programming: i64) -> bool {
    after_programming >= record.deadline
}

pub fn is_current_due(pending: AlarmRecord, armed: Option<AlarmRecord>, now: i64) -> bool {
    classify_expiry(pending, armed, now) == Expiry::Due
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Expiry {
    Stale,
    Early,
    Due,
}

pub fn classify_expiry(pending: AlarmRecord, armed: Option<AlarmRecord>, now: i64) -> Expiry {
    if armed != Some(pending) {
        Expiry::Stale
    } else if now < pending.deadline {
        Expiry::Early
    } else {
        Expiry::Due
    }
}

/// Register/sidecar operations for the shared programming protocol. The hardware
/// adapter holds a critical section throughout; tests supply racing registers.
pub trait CompareRegisters {
    fn mask_interrupt(&mut self);
    fn clear_pending(&mut self);
    fn clear_event(&mut self);
    fn read_event(&mut self) -> u32;
    fn write_compare(&mut self, low: u32);
    fn read_compare(&mut self) -> u32;
    fn read_clock(&mut self) -> Option<i64>;
    fn publish_pending(&mut self, record: AlarmRecord);
    fn enable_interrupt(&mut self);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Programmed {
    Enabled,
    Pending,
    ClockFault,
}

/// Arm operations in the current normalized physical domain, under one CS.
pub trait AbsoluteArmRegisters: CompareRegisters {
    /// Mask/clear the selected compare, then invalidate its arm and pending record.
    fn reset_arm(&mut self);
    /// Commit one checked global epoch increment; never wrap or reuse an epoch.
    fn allocate_epoch(&mut self) -> Option<u64>;
    /// Install the active record and mark it armed before any publication.
    fn activate_arm(&mut self, record: AlarmRecord);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmResult {
    InvalidDeadline,
    MappingFault,
    EpochExhausted,
    ClockFault,
    Programmed {
        record: AlarmRecord,
        result: Programmed,
    },
}

/// Consume the authoritative absolute target, independently of relative hints.
/// Due alarms are published for thread-mode drain, never invoked synchronously.
pub fn arm_absolute(io: &mut impl AbsoluteArmRegisters, target: i64) -> ArmResult {
    arm_absolute_with(io, target, |now, target| {
        absolute_deadline(now, target).ok_or(ArmPlanError::InvalidDeadline)
    })
}

/// Reset and sample the physical clock before planning, then commit the same
/// physical record and compare protocol used by normalized absolute arms.
pub fn arm_absolute_with(
    io: &mut impl AbsoluteArmRegisters,
    target: i64,
    planner: impl FnOnce(i64, i64) -> Result<i64, ArmPlanError>,
) -> ArmResult {
    io.reset_arm();
    let Some(now) = io.read_clock() else {
        return ArmResult::ClockFault;
    };
    let deadline = match planner(now, target) {
        Ok(deadline) => deadline,
        Err(ArmPlanError::InvalidDeadline) => return ArmResult::InvalidDeadline,
        Err(ArmPlanError::MappingFault) => return ArmResult::MappingFault,
    };
    let Some(epoch) = io.allocate_epoch() else {
        return ArmResult::EpochExhausted;
    };
    let record = AlarmRecord { deadline, epoch };
    io.activate_arm(record);
    // Reset already masked and cleared the old arm; a due target needs no compare.
    let result = if deadline <= now {
        io.publish_pending(record);
        Programmed::Pending
    } else {
        program_compare(io, record)
    };
    if result == Programmed::ClockFault {
        io.reset_arm();
    }
    ArmResult::Programmed { record, result }
}

/// Used for fresh arms AND repair, without allocating/changing an epoch. The
/// second clear removes old-CC latches during replacement. Its following clock
/// read recovers any new-deadline event erased by that clear. Pipeline-delayed
/// old notifications beyond this sequence are handled by Expiry::Early repair.
pub fn program_compare(io: &mut impl CompareRegisters, record: AlarmRecord) -> Programmed {
    io.mask_interrupt();
    io.clear_pending();
    io.clear_event();
    let _ = io.read_event();
    io.write_compare(record.deadline as u32);
    let _ = io.read_compare();
    io.clear_event();
    let _ = io.read_event();
    let Some(after) = io.read_clock() else {
        return Programmed::ClockFault;
    };
    if programmed_overdue(record, after) {
        io.publish_pending(record);
        Programmed::Pending
    } else {
        io.enable_interrupt();
        Programmed::Enabled
    }
}

/// Choose by stored absolute deadline, never by TimerKind priority.
pub fn earliest(records: &[Option<AlarmRecord>; 3]) -> Option<usize> {
    records
        .iter()
        .enumerate()
        .filter_map(|(i, r)| r.map(|r| (i, r.deadline)))
        .min_by_key(|&(_, deadline)| deadline)
        .map(|(i, _)| i)
}

/// Drain at most two snapshots; dispatch-created due alarms can join the second batch.
pub fn drain_pending_alarms<C>(
    context: &mut C,
    mut collect: impl FnMut(&mut C) -> [Option<AlarmRecord>; 3],
    mut dispatch: impl FnMut(&mut C, usize, AlarmRecord),
) {
    for _ in 0..2 {
        let mut records = collect(context);
        if records.iter().all(Option::is_none) {
            break;
        }
        while let Some(index) = earliest(&records) {
            let record = records[index].take().expect("selected alarm");
            dispatch(context, index, record);
        }
    }
}

#[derive(Default)]
pub struct WorkFifo {
    entries: [Option<WorkKind>; 3],
    len: usize,
}

impl WorkFifo {
    pub fn submit(&mut self, kind: WorkKind) -> i32 {
        if self.entries[..self.len].contains(&Some(kind)) {
            return 0;
        }
        if self.len == self.entries.len() {
            return 0;
        }
        self.entries[self.len] = Some(kind);
        self.len += 1;
        1
    }

    /// Removes the queued/running marker before invoking the handler, allowing
    /// that handler to resubmit itself. Metadata remains owned by core Tdma.
    pub fn pop(&mut self) -> Option<WorkKind> {
        let kind = self.entries[0]?;
        self.cancel(kind);
        Some(kind)
    }

    pub fn cancel(&mut self, kind: WorkKind) {
        if let Some(index) = self.entries[..self.len]
            .iter()
            .position(|&k| k == Some(kind))
        {
            for i in index + 1..self.len {
                self.entries[i - 1] = self.entries[i];
            }
            self.len -= 1;
            self.entries[self.len] = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn microsecond_deadlines_wrap_low_word_and_reject_far_or_overflow() {
        let d = deadline(i64::from(u32::MAX) - 2, 7).unwrap();
        assert_eq!(d as u32, 4);
        assert_eq!(deadline(10, 0), Some(10));
        assert_eq!(deadline(10, 1), Some(11));
        assert_eq!(deadline(10, -1), None);
        assert_eq!(deadline(10, HALF_RANGE_US), None);
        assert_eq!(deadline(i64::MAX, 1), None);
    }

    #[test]
    fn rearm_cancel_late_and_stale_epoch_validation() {
        let old = AlarmRecord {
            deadline: 20,
            epoch: 1,
        };
        let new = AlarmRecord {
            deadline: 20,
            epoch: next_epoch(old.epoch).unwrap(),
        };
        assert!(!is_current_due(old, Some(new), 30));
        assert!(!is_current_due(old, None, 30));
        assert!(!is_current_due(new, Some(new), 19));
        assert!(is_current_due(new, Some(new), 20));
        assert!(is_current_due(new, Some(new), 30));
        assert_eq!(next_epoch(u64::MAX), None);
        assert_eq!(next_epoch(u64::from(u32::MAX)), Some(1u64 << 32));
        // Equal low words must not make an old IRQ record current.
        let high_changed = AlarmRecord {
            epoch: old.epoch + (1u64 << 32),
            ..old
        };
        assert!(!is_current_due(old, Some(high_changed), 30));
        assert!(!is_current_due(
            AlarmRecord {
                deadline: 19,
                ..new
            },
            Some(new),
            30
        ));
    }

    #[test]
    fn compare_programming_race_and_zero_delay_are_deferred_now() {
        let record = AlarmRecord {
            deadline: deadline(i64::from(u32::MAX) - 1, 3).unwrap(),
            epoch: 1,
        };
        assert!(!programmed_overdue(record, record.deadline - 1));
        assert!(programmed_overdue(record, record.deadline));
        assert!(programmed_overdue(record, record.deadline + 12));
        assert!(programmed_overdue(
            AlarmRecord {
                deadline: deadline(10, 0).unwrap(),
                epoch: 2
            },
            10
        ));
    }

    // Actual shared protocol against mutable fake registers, with injected
    // counter/event races. Stage checks enforce reads and writes, not log labels.
    struct Registers {
        now: i64,
        cc: u32,
        event: bool,
        enabled: bool,
        active: Option<AlarmRecord>,
        pending: Option<AlarmRecord>,
        stage: u8,
        writes: u32,
        masks: u32,
        old_latches: u32,
        inject_old_before_write: bool,
        cross_during_post_clear: bool,
        clock_fault: bool,
        repairs: u32,
        callbacks: u32,
        armed: bool,
        epoch: u64,
        allocations: u32,
        activations: u32,
        resets: u32,
        post_clock_fault: bool,
    }

    impl Registers {
        fn new(record: AlarmRecord) -> Self {
            Self {
                now: 9,
                cc: 10,
                event: false,
                enabled: true,
                active: Some(record),
                pending: None,
                stage: 0,
                writes: 0,
                masks: 0,
                old_latches: 0,
                inject_old_before_write: false,
                cross_during_post_clear: false,
                clock_fault: false,
                repairs: 0,
                callbacks: 0,
                armed: true,
                epoch: record.epoch,
                allocations: 0,
                activations: 0,
                resets: 0,
                post_clock_fault: false,
            }
        }

        fn advance(&mut self, now: i64) {
            assert!(now >= self.now);
            if self.now < i64::from(self.cc) && now >= i64::from(self.cc) {
                self.event = true;
            }
            self.now = now;
        }

        fn irq(&mut self) -> Option<AlarmRecord> {
            if !self.enabled || !self.event {
                return None;
            }
            self.enabled = false;
            self.event = false;
            self.pending = self.active;
            self.pending
        }

        fn accept(&mut self, record: AlarmRecord) -> bool {
            if self.clock_fault {
                return false;
            }
            match classify_expiry(record, self.active, self.now) {
                Expiry::Stale => false,
                Expiry::Early => {
                    self.repairs += 1;
                    let _ = program_compare(self, record);
                    false
                }
                Expiry::Due => {
                    self.active = None;
                    self.callbacks += 1;
                    true
                }
            }
        }
    }

    impl CompareRegisters for Registers {
        fn mask_interrupt(&mut self) {
            self.enabled = false;
            self.masks += 1;
            self.stage = 1;
        }
        fn clear_pending(&mut self) {
            assert_eq!(self.stage, 1);
            self.pending = None;
            self.stage = 2;
        }
        fn clear_event(&mut self) {
            match self.stage {
                2 => self.stage = 3,
                6 => {
                    if self.cross_during_post_clear {
                        self.cross_during_post_clear = false;
                        self.advance(self.active.unwrap().deadline);
                        assert!(self.event);
                    }
                    self.stage = 7;
                }
                _ => panic!("clear outside programming sequence"),
            }
            self.event = false;
        }
        fn read_event(&mut self) -> u32 {
            self.stage = match self.stage {
                3 => 4,
                7 => 8,
                _ => panic!("event readback out of order"),
            };
            u32::from(self.event)
        }
        fn write_compare(&mut self, low: u32) {
            assert_eq!(self.stage, 4);
            if self.inject_old_before_write {
                self.inject_old_before_write = false;
                self.advance(i64::from(self.cc));
                assert!(self.event); // Old CC matched after the initial clear.
                self.old_latches += 1;
            }
            self.cc = low;
            self.writes += 1;
            self.stage = 5;
        }
        fn read_compare(&mut self) -> u32 {
            assert_eq!(self.stage, 5);
            self.stage = 6;
            self.cc
        }
        fn read_clock(&mut self) -> Option<i64> {
            if self.stage == 11 {
                assert!(!self.enabled && !self.armed && !self.event);
                assert!(self.active.is_none() && self.pending.is_none());
                self.stage = 12;
                return (!self.clock_fault).then_some(self.now);
            }
            assert_eq!(self.stage, 8);
            self.stage = 9;
            if self.post_clock_fault {
                self.clock_fault = true;
            }
            (!self.clock_fault).then_some(self.now)
        }
        fn publish_pending(&mut self, record: AlarmRecord) {
            // Either a post-program clock check or a freshly activated due arm.
            assert!(self.stage == 9 || (self.stage == 14 && record.deadline <= self.now));
            assert_eq!(self.active, Some(record));
            assert!(self.armed);
            assert!(!self.enabled);
            self.pending = Some(record);
            self.stage = 10;
        }
        fn enable_interrupt(&mut self) {
            assert_eq!(self.stage, 9);
            assert!(self.armed && self.active.is_some());
            assert!(self.pending.is_none());
            self.enabled = true;
            self.stage = 10;
        }
    }

    impl AbsoluteArmRegisters for Registers {
        fn reset_arm(&mut self) {
            self.enabled = false;
            self.event = false;
            self.armed = false;
            self.active = None;
            self.pending = None;
            self.resets += 1;
            self.stage = 11;
        }
        fn allocate_epoch(&mut self) -> Option<u64> {
            assert_eq!(self.stage, 12);
            let epoch = next_epoch(self.epoch)?;
            self.epoch = epoch;
            self.allocations += 1;
            self.stage = 13;
            Some(epoch)
        }
        fn activate_arm(&mut self, record: AlarmRecord) {
            assert_eq!(self.stage, 13);
            assert_eq!(record.epoch, self.epoch);
            self.active = Some(record);
            self.armed = true;
            self.activations += 1;
            self.stage = 14;
        }
    }

    fn absolute_registers(now: i64) -> Registers {
        let mut registers = Registers::new(AlarmRecord {
            deadline: 10,
            epoch: 0,
        });
        registers.now = now;
        registers.pending = registers.active;
        registers.event = true;
        registers
    }

    fn armed_record(registers: &mut Registers, target: i64, result: Programmed) -> AlarmRecord {
        let outcome = arm_absolute(registers, target);
        let record = registers.active.unwrap();
        assert_eq!(outcome, ArmResult::Programmed { record, result });
        assert_eq!(registers.allocations, registers.activations);
        record
    }

    #[test]
    fn absolute_target_is_authoritative_with_fresh_clock_and_raw_wrap() {
        for (now, target) in [(1_000, 20_000), (i64::from(u32::MAX) - 2, 1i64 << 32)] {
            let mut registers = absolute_registers(now);
            let record = armed_record(&mut registers, target, Programmed::Enabled);
            assert_eq!(
                record,
                AlarmRecord {
                    deadline: target,
                    epoch: 1
                }
            );
            assert_eq!(registers.cc, target as u32);
            assert_eq!(
                (
                    registers.allocations,
                    registers.activations,
                    registers.resets
                ),
                (1, 1, 1)
            );
            assert!(registers.pending.is_none());
        }
    }

    #[test]
    fn absolute_past_and_before_anchor_targets_are_deferred_once() {
        for (now, target, expected) in [
            (0, 0, 0),
            (50, 20, 20),
            (3i64 << 32, 20, 20),
            (50, i64::MIN, 50),
            (i64::MAX, -1, i64::MAX),
            (i64::MAX, i64::MAX, i64::MAX),
        ] {
            let mut registers = absolute_registers(now);
            let record = armed_record(&mut registers, target, Programmed::Pending);
            assert_eq!(record.deadline, expected);
            assert_eq!(registers.pending, Some(record));
            assert!(!registers.enabled);
            assert_eq!(registers.callbacks, 0);
            assert_eq!(
                registers.writes, 0,
                "already-due arms need no compare write"
            );
            assert!(registers.accept(record));
            assert!(!registers.accept(record));
            assert_eq!(registers.callbacks, 1);
        }
    }

    #[test]
    fn absolute_invalid_deadlines_clear_old_arm_without_allocating() {
        for (now, target) in [(-1, -2), (0, HALF_RANGE_US), (0, i64::MAX)] {
            let mut registers = absolute_registers(now);
            assert_eq!(
                arm_absolute(&mut registers, target),
                ArmResult::InvalidDeadline
            );
            assert_eq!(
                (
                    registers.allocations,
                    registers.activations,
                    registers.writes
                ),
                (0, 0, 0)
            );
            assert!(!registers.enabled && !registers.armed);
            assert!(registers.active.is_none() && registers.pending.is_none());
        }
        assert_eq!(
            absolute_deadline(0, HALF_RANGE_US - 1),
            Some(HALF_RANGE_US - 1)
        );
        assert_eq!(absolute_deadline(i64::MAX - 1, i64::MAX), Some(i64::MAX));
    }

    #[test]
    fn absolute_epoch_exhaustion_never_wraps_or_activates() {
        let mut registers = absolute_registers(9);
        registers.epoch = u64::MAX;
        assert_eq!(arm_absolute(&mut registers, 30), ArmResult::EpochExhausted);
        assert_eq!(registers.epoch, u64::MAX);
        assert_eq!(
            (
                registers.allocations,
                registers.activations,
                registers.writes
            ),
            (0, 0, 0)
        );
        assert!(!registers.enabled && !registers.armed);
        assert!(registers.active.is_none() && registers.pending.is_none());
        registers.epoch = u64::MAX - 1;
        let last = armed_record(&mut registers, 9, Programmed::Pending);
        assert_eq!(last.epoch, u64::MAX);
        assert_eq!(arm_absolute(&mut registers, 9), ArmResult::EpochExhausted);
        assert_eq!(registers.epoch, u64::MAX);
        assert_eq!((registers.allocations, registers.activations), (1, 1));
        assert!(!registers.enabled && !registers.armed);
        assert!(registers.active.is_none() && registers.pending.is_none());
    }

    #[test]
    fn absolute_clock_faults_suppress_allocation_or_cleanup_committed_arm() {
        let mut registers = absolute_registers(9);
        registers.clock_fault = true;
        assert_eq!(arm_absolute(&mut registers, 30), ArmResult::ClockFault);
        assert_eq!((registers.allocations, registers.activations), (0, 0));
        registers.clock_fault = false;
        registers.post_clock_fault = true;
        let record = AlarmRecord {
            deadline: 30,
            epoch: 1,
        };
        assert_eq!(
            arm_absolute(&mut registers, 30),
            ArmResult::Programmed {
                record,
                result: Programmed::ClockFault,
            }
        );
        assert_eq!(
            (
                registers.epoch,
                registers.allocations,
                registers.activations
            ),
            (1, 1, 1)
        );
        assert_eq!(registers.resets, 3);
        assert!(!registers.enabled && !registers.armed);
        assert!(registers.active.is_none() && registers.pending.is_none());
        assert_eq!(arm_absolute(&mut registers, 30), ArmResult::ClockFault);
        assert_eq!(registers.allocations, 1);
    }

    #[test]
    fn absolute_rearm_cancel_and_duplicate_do_not_touch_current_epoch() {
        let mut registers = absolute_registers(50);
        let old = armed_record(&mut registers, 20, Programmed::Pending);
        let new = armed_record(&mut registers, 30, Programmed::Pending);
        assert_eq!(new.epoch, old.epoch + 1);
        assert!(!registers.accept(old));
        assert_eq!(registers.active, Some(new));
        assert_eq!(registers.pending, Some(new));
        assert!(registers.accept(new));
        let current = armed_record(&mut registers, 80, Programmed::Enabled);
        assert!(!registers.accept(new));
        assert_eq!(registers.active, Some(current));
        registers.reset_arm();
        assert!(!registers.accept(current));
        assert_eq!(registers.allocations, 3);
        assert_eq!(registers.callbacks, 1);
    }

    #[test]
    fn absolute_arm_recovers_compare_races_and_repairs_without_new_epoch() {
        let mut registers = absolute_registers(9);
        registers.inject_old_before_write = true;
        registers.cross_during_post_clear = true;
        let due = armed_record(&mut registers, 30, Programmed::Pending);
        assert_eq!(registers.old_latches, 1);
        assert!(registers.accept(due));
        let future = armed_record(&mut registers, 60, Programmed::Enabled);
        registers.event = true;
        let early = registers.irq().unwrap();
        assert!(!registers.accept(early));
        assert_eq!(registers.active, Some(future));
        assert_eq!(registers.allocations, 2);
        assert_eq!(registers.repairs, 1);
        registers.advance(60);
        let pending = registers.irq().unwrap();
        assert!(registers.accept(pending));
        assert!(!registers.accept(pending));
        assert_eq!(registers.callbacks, 2);
    }

    // Only the normalized start path is modeled; register programming uses the
    // same arm_absolute implementation as the live adapter.
    struct AbsoluteModel {
        registers: Registers,
        clock_samples: u32,
        hint: Option<i64>,
    }
    impl nrf_embassy::tdma::Clock for AbsoluteModel {
        fn now_us(&mut self) -> i64 {
            let now = i64::from(self.clock_samples) * 100;
            self.clock_samples += 1;
            now
        }
        fn uptime_ms(&mut self) -> u32 {
            0
        }
    }
    impl nrf_embassy::tdma::Timers for AbsoluteModel {
        fn init_timer(&mut self, kind: TimerKind) {
            self.cancel_timer(kind);
        }
        fn arm_timer(&mut self, _kind: TimerKind, _delay: i64) {
            panic!("normalized TDMA must use the absolute override");
        }
        fn arm_timer_at(&mut self, kind: TimerKind, target: i64, hint: i64) {
            assert_eq!(kind, TimerKind::Frame);
            self.hint = Some(hint);
            self.registers.now = 200;
            armed_record(&mut self.registers, target, Programmed::Enabled);
        }
        fn cancel_timer(&mut self, _kind: TimerKind) {
            self.registers.reset_arm();
        }
    }
    impl nrf_embassy::tdma::WorkQueue for AbsoluteModel {
        fn init_work(&mut self, _kind: WorkKind) {}
        fn submit_work(&mut self, _kind: WorkKind) -> i32 {
            1
        }
        fn cancel_work(&mut self, _kind: WorkKind) {}
    }
    impl nrf_embassy::tdma::WsCapture for AbsoluteModel {
        fn capture_ws(&mut self) -> Option<u32> {
            None
        }
        fn sample_ws(&mut self, _frame: u32, _edges: u32) -> Option<i32> {
            None
        }
    }
    impl nrf_embassy::tdma::Callbacks for AbsoluteModel {
        fn slot_callback(&mut self, _slot: u8, _frame: u32) {}
        fn control_callback(&mut self, _frame: u32) {}
    }

    #[test]
    fn tdma_start_keeps_core_target_despite_elapsed_arm_time_and_hint() {
        let mut model = AbsoluteModel {
            registers: absolute_registers(200),
            clock_samples: 0,
            hint: None,
        };
        let mut tdma = nrf_embassy::tdma::Tdma::new();
        tdma.init(&mut model);
        tdma.start(0, false, &mut model).unwrap();
        assert_eq!(model.clock_samples, 2);
        assert_eq!(model.hint, Some(19_900));
        assert_eq!(model.registers.active.unwrap().deadline, 20_000);
        assert_eq!(model.registers.cc, 20_000);
        assert_eq!(model.registers.allocations, 1);
        for hint in [100, 0] {
            nrf_embassy::tdma::Timers::arm_timer_at(&mut model, TimerKind::Frame, 20_000, hint);
            assert_eq!(model.registers.active.unwrap().deadline, 20_000);
            assert_eq!(model.registers.cc, 20_000);
        }
        assert_eq!(model.registers.allocations, 3);
    }

    fn source_mapping(profile: crate::timing_clock::ClockProfile) -> ClockMapping {
        ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(0))
    }

    fn mapped_record(
        registers: &mut Registers,
        target: i64,
        mapping: ClockMapping,
        result: Programmed,
    ) -> AlarmRecord {
        let outcome = arm_absolute_with(registers, target, |now, target| {
            mapped_absolute_deadline(now, target, mapping)
        });
        let record = registers.active.unwrap();
        assert_eq!(outcome, ArmResult::Programmed { record, result });
        if result == Programmed::Pending {
            assert_eq!(registers.pending, Some(record));
            assert_eq!(registers.writes, 0);
        } else {
            assert_eq!(registers.cc, record.deadline as u32);
        }
        assert_eq!((registers.allocations, registers.activations), (1, 1));
        record
    }

    // Exercise the actual core start path with a continuous source mapping;
    // lifecycle records and registers still carry physical microseconds.
    struct DefaultSourceClockModel {
        inner: AbsoluteModel,
        mapping: ClockMapping,
    }

    impl nrf_embassy::tdma::Clock for DefaultSourceClockModel {
        fn now_us(&mut self) -> i64 {
            let physical = nrf_embassy::tdma::Clock::now_us(&mut self.inner);
            self.mapping
                .timing_at(PhysicalTime::new(physical as u64))
                .unwrap()
                .value()
        }
        fn uptime_ms(&mut self) -> u32 {
            nrf_embassy::tdma::Clock::uptime_ms(&mut self.inner)
        }
    }
    impl nrf_embassy::tdma::Timers for DefaultSourceClockModel {
        fn init_timer(&mut self, kind: TimerKind) {
            nrf_embassy::tdma::Timers::init_timer(&mut self.inner, kind);
        }
        fn arm_timer(&mut self, _kind: TimerKind, _delay: i64) {
            panic!("source model must use absolute targets");
        }
        fn arm_timer_at(&mut self, kind: TimerKind, target: i64, hint: i64) {
            assert_eq!(kind, TimerKind::Frame);
            self.inner.hint = Some(hint);
            self.inner.registers.now = 200;
            let mapping = self.mapping;
            let mut calls = 0;
            let result = arm_absolute_with(&mut self.inner.registers, target, |now, target| {
                calls += 1;
                assert_eq!(now, 200);
                mapped_absolute_deadline(now, target, mapping)
            });
            assert_eq!(calls, 1);
            assert_eq!(
                result,
                ArmResult::Programmed {
                    record: self.inner.registers.active.unwrap(),
                    result: Programmed::Enabled,
                }
            );
        }
        fn cancel_timer(&mut self, kind: TimerKind) {
            nrf_embassy::tdma::Timers::cancel_timer(&mut self.inner, kind);
        }
    }
    impl nrf_embassy::tdma::WorkQueue for DefaultSourceClockModel {
        fn init_work(&mut self, kind: WorkKind) {
            nrf_embassy::tdma::WorkQueue::init_work(&mut self.inner, kind);
        }
        fn submit_work(&mut self, kind: WorkKind) -> i32 {
            nrf_embassy::tdma::WorkQueue::submit_work(&mut self.inner, kind)
        }
        fn cancel_work(&mut self, kind: WorkKind) {
            nrf_embassy::tdma::WorkQueue::cancel_work(&mut self.inner, kind);
        }
    }
    impl nrf_embassy::tdma::WsCapture for DefaultSourceClockModel {
        fn capture_ws(&mut self) -> Option<u32> {
            nrf_embassy::tdma::WsCapture::capture_ws(&mut self.inner)
        }
        fn sample_ws(&mut self, frame: u32, edges: u32) -> Option<i32> {
            nrf_embassy::tdma::WsCapture::sample_ws(&mut self.inner, frame, edges)
        }
    }
    impl nrf_embassy::tdma::Callbacks for DefaultSourceClockModel {
        fn slot_callback(&mut self, slot: u8, frame: u32) {
            nrf_embassy::tdma::Callbacks::slot_callback(&mut self.inner, slot, frame);
        }
        fn control_callback(&mut self, frame: u32) {
            nrf_embassy::tdma::Callbacks::control_callback(&mut self.inner, frame);
        }
    }

    #[test]
    fn mapped_core_start_uses_absolute_target_and_fresh_physical_sample() {
        use crate::timing_clock::ClockProfile;
        for (profile, expected, hint) in [
            (ClockProfile::Normalized, 20_000, 19_900),
            (ClockProfile::LegacyNrfRtc10000, 18_311, 19_891),
        ] {
            let mut model = DefaultSourceClockModel {
                inner: AbsoluteModel {
                    registers: absolute_registers(200),
                    clock_samples: 0,
                    hint: None,
                },
                mapping: source_mapping(profile),
            };
            let mut tdma = nrf_embassy::tdma::Tdma::new();
            tdma.init(&mut model);
            tdma.start(0, false, &mut model).unwrap();
            assert_eq!(model.inner.clock_samples, 2);
            assert_eq!(model.inner.hint, Some(hint));
            assert_eq!(model.inner.registers.active.unwrap().deadline, expected);
            assert_eq!(model.inner.registers.cc, expected as u32);
            assert_eq!(model.inner.registers.allocations, 1);
            // A physical RF end at 20 ms remains distinct from the legacy alarm.
            assert_eq!(
                20_000 - expected,
                if expected == 20_000 { 0 } else { 1_689 }
            );
        }
    }

    #[test]
    fn mapped_slot_and_control_keep_actual_due_physical_targets() {
        use crate::timing_clock::ClockProfile;
        for (profile, control) in [
            (ClockProfile::Normalized, 16_000),
            (ClockProfile::LegacyNrfRtc10000, 14_649),
        ] {
            let mapping = source_mapping(profile);
            for (now, target, expected, result) in [
                (0, 0, 0, Programmed::Pending),
                (100, 16_000, control, Programmed::Enabled),
                (20_000, 16_000, control, Programmed::Pending),
            ] {
                let mut registers = absolute_registers(now);
                let record = mapped_record(&mut registers, target, mapping, result);
                assert_eq!(record.deadline, expected);
                assert_eq!(registers.callbacks, 0);
            }
        }
        let mapping = source_mapping(ClockProfile::LegacyNrfRtc10000);
        assert_eq!(
            mapping
                .timing_at(PhysicalTime::new(15_000))
                .unwrap()
                .value(),
            16_384
        );
        let mut registers = absolute_registers(15_000);
        let record = mapped_record(&mut registers, 16_000, mapping, Programmed::Pending);
        assert_eq!(record.deadline, 14_649);
        assert_eq!(registers.pending, Some(record));
    }

    #[test]
    fn mapped_signed_anchors_only_fall_back_for_proven_before_anchor_due() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            for anchor in [5_000, -5_000] {
                let mapping =
                    ClockMapping::new(profile, PhysicalTime::new(1_000), TimingTime::new(anchor));
                for (target, expected) in [(anchor - 1_000, 1_100), (anchor, 1_000)] {
                    let mut registers = absolute_registers(1_100);
                    let record =
                        mapped_record(&mut registers, target, mapping, Programmed::Pending);
                    assert_eq!(record.deadline, expected);
                }
                let mut registers = absolute_registers(1_000);
                let record =
                    mapped_record(&mut registers, anchor + 100, mapping, Programmed::Enabled);
                assert!(record.deadline > 1_000);
            }
        }
    }

    fn assert_failed_mapped_arm(now: i64, target: i64, mapping: ClockMapping, expected: ArmResult) {
        let mut registers = absolute_registers(now);
        let old_cc = registers.cc;
        assert_eq!(
            arm_absolute_with(&mut registers, target, |now, target| {
                mapped_absolute_deadline(now, target, mapping)
            }),
            expected
        );
        assert_eq!(registers.stage, 12); // Reset, then fresh read, no allocation.
        assert_eq!(registers.resets, 1);
        assert_eq!(registers.epoch, 0);
        assert_eq!(
            (
                registers.allocations,
                registers.activations,
                registers.writes
            ),
            (0, 0, 0)
        );
        assert_eq!(registers.cc, old_cc);
        assert!(!registers.enabled && !registers.armed && !registers.event);
        assert!(registers.active.is_none() && registers.pending.is_none());
    }

    #[test]
    fn mapped_current_faults_and_negative_physical_now_precede_epoch_allocation() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let anchored =
                ClockMapping::new(profile, PhysicalTime::new(1_000), TimingTime::new(5_000));
            assert_failed_mapped_arm(999, 4_000, anchored, ArmResult::MappingFault);
            let overflow =
                ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(i64::MAX));
            assert_failed_mapped_arm(1, i64::MIN, overflow, ArmResult::MappingFault);
            assert_failed_mapped_arm(-1, 0, source_mapping(profile), ArmResult::InvalidDeadline);
        }
        assert_failed_mapped_arm(
            i64::MAX,
            0,
            source_mapping(ClockProfile::LegacyNrfRtc10000),
            ArmResult::MappingFault,
        );
    }

    #[test]
    fn mapped_half_range_guard_uses_physical_gap_after_ceiling() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mapping = source_mapping(profile);
            let now = 123;
            for gap in [HALF_RANGE_US - 1, HALF_RANGE_US] {
                let physical_target = now + gap;
                // First timing unit strictly above the preceding physical tick
                // inverts to this exact physical tick under either profile.
                let target = mapping
                    .timing_at(PhysicalTime::new((physical_target - 1) as u64))
                    .unwrap()
                    .value()
                    + 1;
                assert_eq!(
                    mapping
                        .physical_not_before(TimingTime::new(target))
                        .unwrap()
                        .value(),
                    physical_target as u64
                );
                if gap < HALF_RANGE_US {
                    let mut registers = absolute_registers(now);
                    let record =
                        mapped_record(&mut registers, target, mapping, Programmed::Enabled);
                    assert_eq!(record.deadline, physical_target);
                    if profile == ClockProfile::LegacyNrfRtc10000 {
                        assert!(
                            target
                                - mapping
                                    .timing_at(PhysicalTime::new(now as u64))
                                    .unwrap()
                                    .value()
                                > HALF_RANGE_US
                        );
                    }
                } else {
                    assert_failed_mapped_arm(now, target, mapping, ArmResult::InvalidDeadline);
                }
            }
        }
        let normalized = source_mapping(ClockProfile::Normalized);
        assert_failed_mapped_arm(1, i64::MAX, normalized, ArmResult::InvalidDeadline);
        assert_eq!(
            mapped_absolute_deadline(i64::MAX - 1, i64::MAX, normalized),
            Ok(i64::MAX)
        );
    }

    #[test]
    fn mapped_inverse_overflow_and_signed_narrowing_are_mapping_faults() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let narrowing = ClockMapping::new(
                profile,
                PhysicalTime::new(i64::MAX as u64 - 10),
                TimingTime::new(0),
            );
            assert_failed_mapped_arm(i64::MAX - 10, 100, narrowing, ArmResult::MappingFault);
            let overflow = ClockMapping::new(
                profile,
                PhysicalTime::new(i64::MAX as u64),
                TimingTime::new(i64::MIN),
            );
            assert_eq!(
                overflow.physical_not_before(TimingTime::new(i64::MAX)),
                Err(ClockError::PhysicalOverflow)
            );
            assert_failed_mapped_arm(i64::MAX, i64::MAX, overflow, ArmResult::MappingFault);
        }
    }

    #[test]
    fn mapped_planner_is_fn_once_after_reset_and_read_with_distinct_errors() {
        struct PlannerToken(i64);
        impl Drop for PlannerToken {
            fn drop(&mut self) {}
        }

        for (error, expected) in [
            (ArmPlanError::MappingFault, ArmResult::MappingFault),
            (ArmPlanError::InvalidDeadline, ArmResult::InvalidDeadline),
        ] {
            let mut registers = absolute_registers(200);
            let mut calls = 0;
            // Moving and dropping this value makes the closure FnOnce.
            let token = PlannerToken(20_000);
            assert_eq!(
                arm_absolute_with(&mut registers, 20_000, |now, target| {
                    calls += 1;
                    assert_eq!((now, target), (200, token.0));
                    drop(token);
                    Err(error)
                }),
                expected
            );
            assert_eq!(calls, 1);
            assert_eq!(registers.stage, 12);
            assert_eq!(registers.resets, 1);
            assert_eq!(
                (
                    registers.allocations,
                    registers.activations,
                    registers.writes
                ),
                (0, 0, 0)
            );
            assert!(!registers.armed && !registers.enabled);
            assert!(registers.active.is_none() && registers.pending.is_none());
        }
        let mut registers = absolute_registers(200);
        registers.clock_fault = true;
        assert_eq!(
            arm_absolute_with(&mut registers, 20_000, |_, _| {
                panic!("planner must not run without a physical sample")
            }),
            ArmResult::ClockFault
        );
        assert_eq!(registers.allocations, 0);
    }

    #[test]
    fn mapped_deadlines_are_first_physical_tick_not_before_timing_target() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            for timing_anchor in [-100, 0, 5_000] {
                let mapping = ClockMapping::new(
                    profile,
                    PhysicalTime::new(1_000),
                    TimingTime::new(timing_anchor),
                );
                for offset in 1..=1_024 {
                    let target = timing_anchor + offset;
                    let physical = mapped_absolute_deadline(1_000, target, mapping).unwrap();
                    assert!(physical > 1_000);
                    assert!(
                        mapping
                            .timing_at(PhysicalTime::new(physical as u64))
                            .unwrap()
                            .value()
                            >= target
                    );
                    assert!(
                        mapping
                            .timing_at(PhysicalTime::new((physical - 1) as u64))
                            .unwrap()
                            .value()
                            < target
                    );
                }
            }
        }
    }

    fn future_record() -> AlarmRecord {
        AlarmRecord {
            deadline: 30,
            epoch: (1u64 << 32) + 1,
        }
    }

    #[test]
    fn shared_programming_removes_old_latch_between_clear_and_cc_write() {
        let record = future_record();
        let mut registers = Registers::new(record);
        registers.pending = Some(AlarmRecord { epoch: 1, ..record });
        registers.inject_old_before_write = true;
        assert_eq!(program_compare(&mut registers, record), Programmed::Enabled);
        assert_eq!(registers.old_latches, 1);
        assert_eq!(registers.cc, record.deadline as u32);
        assert!(!registers.event);
        assert!(registers.pending.is_none());
        assert!(registers.enabled);
        assert_eq!(registers.irq(), None);
        assert_eq!(registers.active, Some(record));
    }

    #[test]
    fn shared_programming_recovers_new_deadline_erased_by_post_clear() {
        let record = future_record();
        let mut registers = Registers::new(record);
        registers.cross_during_post_clear = true;
        assert_eq!(program_compare(&mut registers, record), Programmed::Pending);
        assert!(!registers.event);
        assert!(!registers.enabled);
        assert_eq!(registers.pending, Some(record));
        assert_eq!(registers.now, record.deadline);
        assert!(registers.accept(record));
        assert!(!registers.accept(record));
        assert_eq!(registers.callbacks, 1);
    }

    #[test]
    fn delayed_old_pipeline_irq_is_repaired_without_new_epoch_and_fires_once() {
        let record = future_record();
        let mut registers = Registers::new(record);
        assert_eq!(program_compare(&mut registers, record), Programmed::Enabled);
        // Old notification arrives even beyond the post-program event readback.
        registers.event = true;
        let early = registers.irq().unwrap();
        assert!(!registers.enabled);
        assert_eq!(
            classify_expiry(early, registers.active, registers.now),
            Expiry::Early
        );
        assert!(!registers.accept(early));
        assert_eq!(registers.callbacks, 0);
        assert_eq!(registers.repairs, 1);
        assert_eq!(registers.active, Some(record));
        assert!(registers.enabled);
        assert_eq!(registers.writes, 2);
        registers.advance(record.deadline);
        let due = registers.irq().unwrap();
        assert!(registers.accept(due));
        assert!(!registers.accept(due));
        assert_eq!(registers.callbacks, 1);
        assert_eq!(registers.irq(), None);
    }

    #[test]
    fn early_repair_crossing_deadline_republishes_same_record() {
        let record = future_record();
        let mut registers = Registers::new(record);
        let _ = program_compare(&mut registers, record);
        registers.event = true;
        let early = registers.irq().unwrap();
        registers.cross_during_post_clear = true;
        assert!(!registers.accept(early));
        assert_eq!(registers.active, Some(record));
        assert_eq!(registers.pending, Some(record));
        assert!(!registers.enabled);
        assert!(registers.accept(registers.pending.unwrap()));
        assert_eq!(registers.callbacks, 1);
    }

    #[test]
    fn cancelled_and_rearmed_stale_epochs_never_reprogram_and_fault_suppresses_repair() {
        let old = future_record();
        let mut registers = Registers::new(old);
        let _ = program_compare(&mut registers, old);
        registers.active = None; // Cancellation.
        registers.enabled = false;
        let writes = registers.writes;
        let masks = registers.masks;
        assert!(!registers.accept(old));
        assert_eq!((registers.writes, registers.masks), (writes, masks));
        let new = AlarmRecord {
            epoch: next_epoch(old.epoch).unwrap(),
            ..old
        };
        registers.active = Some(new);
        let _ = program_compare(&mut registers, new);
        let before = (
            registers.writes,
            registers.masks,
            registers.cc,
            registers.enabled,
            registers.event,
            registers.pending,
            registers.active,
            registers.stage,
        );
        assert!(!registers.accept(old));
        assert_eq!(
            (
                registers.writes,
                registers.masks,
                registers.cc,
                registers.enabled,
                registers.event,
                registers.pending,
                registers.active,
                registers.stage
            ),
            before
        );
        registers.event = true;
        let early = registers.irq().unwrap();
        registers.clock_fault = true;
        let writes = registers.writes;
        assert!(!registers.accept(early));
        assert_eq!(registers.writes, writes);
        assert_eq!(registers.repairs, 0);
        assert!(!registers.enabled); // Owner stop/cancel handles the fault.
    }

    #[test]
    fn old_control_precedes_later_frame_even_across_raw_wrap() {
        let base = i64::from(u32::MAX);
        let mut records = [
            Some(AlarmRecord {
                deadline: base + 20,
                epoch: 3,
            }),
            None,
            Some(AlarmRecord {
                deadline: base - 1,
                epoch: 2,
            }),
        ];
        assert_eq!(earliest(&records), Some(2));
        records[2] = None;
        assert_eq!(earliest(&records), Some(0));
    }

    #[test]
    fn fifo_coalesces_cancels_and_allows_handler_resubmit() {
        let mut queue = WorkFifo::default();
        assert_eq!(queue.submit(WorkKind::Control), 1);
        assert_eq!(queue.submit(WorkKind::Slot), 1);
        assert_eq!(queue.submit(WorkKind::Control), 0);
        assert_eq!(queue.submit(WorkKind::Discipline), 1);
        assert_eq!(queue.pop(), Some(WorkKind::Control));
        assert_eq!(queue.submit(WorkKind::Control), 1);
        queue.cancel(WorkKind::Slot);
        assert_eq!(queue.pop(), Some(WorkKind::Discipline));
        assert_eq!(queue.pop(), Some(WorkKind::Control));
        assert_eq!(queue.pop(), None);
    }

    // Physical sidecars use the production arm, compare, expiry, and drain policies.
    struct DrainModel {
        now: i64,
        epoch: u64,
        mapping: ClockMapping,
        registers: [Registers; 3],
        work: WorkFifo,
        callbacks: crate::actor_policy::CallbackBuffer,
        wake: bool,
        collections: usize,
        dispatched: [Option<(usize, AlarmRecord)>; 12],
        dispatches: usize,
    }

    impl DrainModel {
        fn new(profile: crate::timing_clock::ClockProfile) -> Self {
            Self {
                now: 0,
                epoch: 0,
                mapping: source_mapping(profile),
                registers: core::array::from_fn(|_| absolute_registers(0)),
                work: WorkFifo::default(),
                callbacks: crate::actor_policy::CallbackBuffer::default(),
                wake: false,
                collections: 0,
                dispatched: [None; 12],
                dispatches: 0,
            }
        }

        fn advance(&mut self, now: i64) {
            assert!(now >= self.now);
            self.now = now;
            for registers in &mut self.registers {
                registers.advance(now);
                if registers.irq().is_some() {
                    self.wake = true;
                }
            }
        }

        fn arm(&mut self, index: usize, target: i64) -> AlarmRecord {
            let mapping = self.mapping;
            let registers = &mut self.registers[index];
            registers.epoch = self.epoch;
            let result = arm_absolute_with(registers, target, |now, target| {
                mapped_absolute_deadline(now, target, mapping)
            });
            let ArmResult::Programmed { record, result } = result else {
                panic!("model arm failed");
            };
            assert_ne!(result, Programmed::ClockFault);
            self.epoch = record.epoch;
            self.wake |= result == Programmed::Pending;
            record
        }

        fn collect(&mut self) -> [Option<AlarmRecord>; 3] {
            self.collections += 1;
            core::array::from_fn(|index| self.registers[index].pending.take())
        }

        fn accept(&mut self, index: usize, record: AlarmRecord) -> bool {
            let registers = &mut self.registers[index];
            let accepted = registers.accept(record);
            self.wake |= registers.pending.is_some();
            let now = registers.now;
            if now > self.now {
                self.advance(now);
            }
            if accepted {
                self.dispatched[self.dispatches] = Some((index, record));
                self.dispatches += 1;
            }
            accepted
        }

        fn alarms(&mut self, engine: &mut nrf_embassy::tdma::Tdma) {
            self.wake = false;
            drain_pending_alarms(
                &mut (&mut *self, engine),
                |(platform, _)| platform.collect(),
                |(platform, engine), index, record| {
                    if platform.accept(index, record) {
                        engine.timer_expired(TIMER_KINDS[index], *platform).unwrap();
                    }
                },
            );
        }

        fn execute_work(&mut self, engine: &mut nrf_embassy::tdma::Tdma) {
            while let Some(kind) = self.work.pop() {
                engine.work_executed(kind, self).unwrap();
            }
        }

        fn drain(
            &mut self,
            engine: &mut nrf_embassy::tdma::Tdma,
        ) -> [Option<crate::actor_policy::ActorCallback>; 3] {
            self.alarms(engine);
            self.execute_work(engine);
            self.callbacks.take()
        }

        fn start(&mut self, slot: i8) -> nrf_embassy::tdma::Tdma {
            let mut engine = nrf_embassy::tdma::Tdma::new();
            engine.init(self);
            engine.set_callbacks(true, true);
            engine.start(slot, true, self).unwrap();
            engine
        }
    }

    impl nrf_embassy::tdma::Clock for DrainModel {
        fn now_us(&mut self) -> i64 {
            self.mapping
                .timing_at(PhysicalTime::new(self.now as u64))
                .unwrap()
                .value()
        }
        fn uptime_ms(&mut self) -> u32 {
            (self.now / 1_000) as u32
        }
    }
    impl nrf_embassy::tdma::Timers for DrainModel {
        fn init_timer(&mut self, kind: TimerKind) {
            self.cancel_timer(kind);
        }
        fn arm_timer(&mut self, _kind: TimerKind, _delay: i64) {
            panic!("drain model requires absolute targets");
        }
        fn arm_timer_at(&mut self, kind: TimerKind, target: i64, _hint: i64) {
            self.arm(timer_index(kind), target);
        }
        fn cancel_timer(&mut self, kind: TimerKind) {
            self.registers[timer_index(kind)].reset_arm();
        }
    }
    impl nrf_embassy::tdma::WorkQueue for DrainModel {
        fn init_work(&mut self, kind: WorkKind) {
            self.work.cancel(kind);
        }
        fn submit_work(&mut self, kind: WorkKind) -> i32 {
            self.work.submit(kind)
        }
        fn cancel_work(&mut self, kind: WorkKind) {
            self.work.cancel(kind);
        }
    }
    impl nrf_embassy::tdma::WsCapture for DrainModel {
        fn capture_ws(&mut self) -> Option<u32> {
            Some(960)
        }
        fn sample_ws(&mut self, _frame: u32, _edges: u32) -> Option<i32> {
            None
        }
    }
    impl nrf_embassy::tdma::Callbacks for DrainModel {
        fn slot_callback(&mut self, slot: u8, frame: u32) {
            self.callbacks
                .push(crate::actor_policy::ActorCallback::Slot { slot, frame });
        }
        fn control_callback(&mut self, frame: u32) {
            self.callbacks
                .push(crate::actor_policy::ActorCallback::Control { frame });
        }
    }

    #[test]
    fn drain_frame_publishes_slot_zero_in_same_call_for_both_clock_profiles() {
        use crate::actor_policy::{ActorCallback, AdmissionRejection, TxWindow};
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(0);
            let frame = model.registers[0].active.unwrap();
            model.advance(frame.deadline + 20);
            let callback = ActorCallback::Slot { slot: 0, frame: 1 };
            assert_eq!(model.drain(&mut engine), [Some(callback), None, None]);
            assert_eq!(model.collections, 2);
            assert_eq!(model.dispatches, 2);
            assert_eq!(model.dispatched[0], Some((0, frame)));
            assert_eq!(model.dispatched[1].unwrap().0, 1);
            assert!(model.registers[1].active.is_none());
            let snapshot = engine.snapshot();
            let timing_now = nrf_embassy::tdma::Clock::now_us(&mut model);
            let window = TxWindow::capture(snapshot, callback, timing_now).unwrap();
            let physical = model
                .mapping
                .convert(
                    TimingTime::new(snapshot.slot.start_us),
                    TimingTime::new(snapshot.slot.deadline_us),
                )
                .unwrap();
            let expected_window = match profile {
                ClockProfile::Normalized => (20_000, 21_500),
                ClockProfile::LegacyNrfRtc10000 => (18_311, 19_683),
            };
            assert_eq!(
                (physical.start().value(), physical.deadline().value()),
                expected_window
            );
            let admission = window.evaluate_for_clock(snapshot, model.now, 113, model.mapping);
            assert_eq!(admission.budget_us, Some(725));
            assert_eq!(admission.physical_window_start, Some(physical.start()));
            assert_eq!(
                admission.physical_window_deadline,
                Some(physical.deadline())
            );
            assert!(admission.result.is_ok());
            // Model extra actor-pass delay without assigning measured hardware latency.
            let delayed = physical.deadline().value() as i64 - 700;
            assert_eq!(
                window
                    .evaluate_for_clock(snapshot, delayed, 113, model.mapping)
                    .result,
                Err(AdmissionRejection::BudgetTooShort)
            );
            assert!(TxWindow::capture(snapshot, callback, snapshot.slot.deadline_us).is_none());
            let collections = model.collections;
            assert_eq!(model.drain(&mut engine), [None; 3]);
            assert_eq!(model.collections - collections, 1);
            assert_eq!(model.dispatches, 2);
            let control = model.registers[2].active.unwrap();
            assert_eq!(snapshot.control.start_us, snapshot.frame_start_us + 16_000);
            model.advance(control.deadline);
            assert_eq!(
                model.drain(&mut engine),
                [Some(ActorCallback::Control { frame: 1 }), None, None]
            );
        }
    }

    // Model the experimental actor's two service points using production drain
    // policies, real TDMA work execution, and consuming callback buffer. These
    // models do not approve the experiment for default use or qualify RF timing.
    fn service_actor_model(model: &mut DrainModel, engine: &mut nrf_embassy::tdma::Tdma) -> usize {
        use crate::actor_policy::TxWindow;
        let callbacks = model.drain(engine);
        let generation = engine.snapshot().generation;
        callbacks
            .into_iter()
            .flatten()
            .filter(|callback| {
                let current = engine.snapshot();
                let now = model
                    .mapping
                    .timing_at(PhysicalTime::new(model.now as u64))
                    .unwrap()
                    .value();
                current.running
                    && current.generation == generation
                    && TxWindow::capture(current, *callback, now).is_some()
            })
            .count()
    }

    #[test]
    fn experimental_actor_due_callback_precedes_pending_background_and_is_not_dispatched_twice() {
        use crate::timing_clock::ClockProfile;
        use core::{
            future::Future,
            future::poll_fn,
            pin::pin,
            task::{Context, Poll, Waker},
        };
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(0);
            model.advance(model.registers[0].active.unwrap().deadline);
            let mut emitted = 0;
            let mut background_pending = true;
            let mut pass = pin!(async {
                emitted += service_actor_model(&mut model, &mut engine);
                poll_fn(|_| {
                    // This assertion runs on the first background poll, before
                    // it yields. Keep the window live across both services so
                    // expiry cannot hide a duplicate callback.
                    assert_eq!(emitted, 1);
                    if core::mem::take(&mut background_pending) {
                        model.advance(model.now + 100);
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                })
                .await;
                assert_eq!(service_actor_model(&mut model, &mut engine), 0);
                assert_eq!(emitted, 1);
                assert_eq!(model.callbacks.take(), [None; 3]);
                assert_eq!(model.work.len, 0);
                assert_eq!(model.registers[1].callbacks, 1);
            });
            let mut cx = Context::from_waker(Waker::noop());
            assert!(pass.as_mut().poll(&mut cx).is_pending());
            assert!(pass.as_mut().poll(&mut cx).is_ready());
        }
    }

    #[test]
    fn experimental_actor_alarm_arriving_during_background_is_serviced_late_once() {
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(1);
            model.advance(model.registers[0].active.unwrap().deadline);
            assert_eq!(service_actor_model(&mut model, &mut engine), 0);
            let slot = model.registers[1].active.unwrap();
            // Background work advances physical time through the slot compare.
            model.advance(slot.deadline);
            assert_eq!(service_actor_model(&mut model, &mut engine), 1);
            assert_eq!(service_actor_model(&mut model, &mut engine), 0);
            assert_eq!(model.registers[1].callbacks, 1);
        }
    }

    #[test]
    fn experimental_actor_stop_before_early_service_and_stale_generation_emit_nothing() {
        use crate::actor_policy::ActorCallback;
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(0);
            model.advance(model.registers[0].active.unwrap().deadline);
            engine.stop(&mut model);
            assert_eq!(service_actor_model(&mut model, &mut engine), 0);
            assert_eq!(service_actor_model(&mut model, &mut engine), 0);

            engine.start(0, true, &mut model).unwrap();
            model.advance(model.registers[0].active.unwrap().deadline);
            // Retain a callback from a prior generation while its authoritative
            // window metadata is stale; the live capture policy must reject it.
            model.alarms(&mut engine);
            model.execute_work(&mut engine);
            let old = engine.snapshot();
            let mut stale = old;
            stale.generation = stale.generation.wrapping_add(1);
            let callback = ActorCallback::Slot {
                slot: 0,
                frame: old.frame_counter,
            };
            assert!(
                crate::actor_policy::TxWindow::capture(stale, callback, old.slot.start_us)
                    .is_none()
            );
            engine.stop(&mut model);
            assert_eq!(service_actor_model(&mut model, &mut engine), 0);
        }
    }

    #[test]
    fn experimental_actor_early_and_late_drains_remain_bounded_with_pending_next_pass() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        for index in 0..3 {
            model.arm(index, 0);
        }
        for _service_point in 0..2 {
            let collections = model.collections;
            let dispatches = model.dispatches;
            drain_pending_alarms(
                &mut model,
                DrainModel::collect,
                |platform, index, record| {
                    assert!(platform.accept(index, record));
                    platform.arm(index, 0);
                },
            );
            assert_eq!(model.collections - collections, 2);
            assert_eq!(model.dispatches - dispatches, 6);
        }
        assert!(model.wake);
        for registers in &model.registers {
            assert!(registers.pending.is_some());
            assert_eq!(registers.pending, registers.active);
        }
    }

    #[test]
    fn actor_queued_runtime_stop_skips_early_tx_and_invalidates_before_late_service() {
        use crate::actor_policy::early_timing_allowed;
        use crate::timing_clock::ClockProfile;
        use nrf_embassy::platform::{Effect, Integration, Platform, Reply};
        use nrf_embassy::runtime::{Command, CommandKind, Event, Runtime, State};

        struct StopIo<'a> {
            model: &'a mut DrainModel,
            engine: &'a mut nrf_embassy::tdma::Tdma,
        }
        impl Platform for StopIo<'_> {
            fn integration(&self) -> Integration {
                Integration::ActorOracle
            }
            fn now_us(&self) -> i64 {
                self.model.now
            }
            fn apply(&mut self, effect: Effect) -> Reply {
                match effect {
                    Effect::GetAddress => Reply::Address([2; 5]),
                    Effect::TdmaStop => {
                        self.engine.stop(self.model);
                        Reply::Value(0)
                    }
                    _ => Reply::Value(0),
                }
            }
        }

        for (profile, enabled) in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000]
            .into_iter()
            .flat_map(|profile| [false, true].map(|enabled| (profile, enabled)))
        {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(0);
            model.advance(model.registers[0].active.unwrap().deadline);
            let generation = engine.snapshot().generation;
            let mut runtime = Runtime::default();
            let mut io = StopIo {
                model: &mut model,
                engine: &mut engine,
            };
            assert_eq!(runtime.init(&mut io), 0);
            assert_eq!(runtime.start(&mut io), 0);
            let command = Some(Command {
                kind: CommandKind::Stop,
                generation: 1,
                talk_channel: 0,
            });
            let command_present = command.is_some();
            if let Some(command) = command {
                runtime.handle(Event::Command(command), &mut io);
            }
            // handle only enqueues STOP: the runtime and due TDMA are still live.
            assert_ne!(runtime.snapshot().membership.state, State::Idle);
            assert!(io.engine.snapshot().running);
            assert_eq!(io.engine.snapshot().generation, generation);
            assert!(io.model.registers[0].pending.is_some());
            let mut emitted = 0;
            if early_timing_allowed(enabled, command_present) {
                emitted += service_actor_model(io.model, io.engine);
            }
            assert_eq!(emitted, 0);
            assert_eq!(io.model.dispatches, 0);
            assert!(io.model.registers[0].pending.is_some());
            assert_eq!(early_timing_allowed(enabled, false), enabled);
            {
                use core::{
                    future::Future,
                    pin::pin,
                    task::{Context, Poll, Waker},
                };
                let mut work = pin!(runtime.command_work_handler_async(&mut io));
                let mut cx = Context::from_waker(Waker::noop());
                assert_eq!(work.as_mut().poll(&mut cx), Poll::Ready(()));
            }
            assert_eq!(runtime.snapshot().membership.state, State::Idle);
            assert!(!io.engine.snapshot().running);
            assert_ne!(io.engine.snapshot().generation, generation);
            emitted += service_actor_model(io.model, io.engine);
            assert_eq!(emitted, 0);
        }
    }

    #[test]
    fn actor_feature_disabled_keeps_due_callback_for_late_service() {
        for profile in [
            crate::timing_clock::ClockProfile::Normalized,
            crate::timing_clock::ClockProfile::LegacyNrfRtc10000,
        ] {
            for enabled in [false, true] {
                let mut model = DrainModel::new(profile);
                let mut engine = model.start(0);
                model.advance(model.registers[0].active.unwrap().deadline);
                let early = if crate::actor_policy::early_timing_allowed(enabled, false) {
                    service_actor_model(&mut model, &mut engine)
                } else {
                    0
                };
                assert_eq!(early, usize::from(enabled));
                assert_eq!(model.collections, if enabled { 2 } else { 0 });
                let late = service_actor_model(&mut model, &mut engine);
                assert_eq!(late, usize::from(!enabled));
                assert_eq!(early + late, 1);
            }
        }
    }

    #[test]
    fn early_timing_policy_requires_opt_in_and_no_pending_command() {
        for (enabled, command_present, expected) in [
            (false, false, false),
            (false, true, false),
            (true, false, true),
            (true, true, false),
        ] {
            assert_eq!(
                crate::actor_policy::early_timing_allowed(enabled, command_present),
                expected
            );
        }
    }

    #[test]
    fn due_timing_services_only_pending_alarms_after_command_work() {
        use crate::actor_policy::due_timing_allowed;
        for (command_present, pending, expected) in [
            (false, false, false),
            (false, true, true),
            (true, false, false),
            (true, true, false),
        ] {
            assert_eq!(due_timing_allowed(command_present, pending), expected);
        }
        // Source structure: every mid-pass due check sits after command work and
        // before the unconditional late service; none sits before command work.
        let source = include_str!("protocol_actor.rs");
        let actor = source.split("    loop {").nth(1).unwrap();
        let due = "service_due_timing(&mut runtime, &mut backend, &mut inbox).await;";
        let late = actor
            .rfind("service_timing(&mut runtime, &mut backend, &mut inbox).await;")
            .unwrap();
        let command_work = actor.find("command_work_handler_async").unwrap();
        let mut positions = actor.match_indices(due).map(|(i, _)| i);
        assert!(
            positions
                .by_ref()
                .take(3)
                .all(|p| command_work < p && p < late)
        );
        assert_eq!(actor.matches(due).count(), 3);
        assert!(source.contains("if due_timing_allowed(false, timing_backend::alarm_pending())"));
    }

    #[test]
    fn diagnostic_formatting_waits_for_a_quiet_alarm_window() {
        use crate::actor_policy::{DIAGNOSTIC_FORMAT_BUDGET_US, diagnostic_format_allowed};
        for (pending, next, expected) in [
            (false, None, true),
            (false, Some(DIAGNOSTIC_FORMAT_BUDGET_US + 1), true),
            (false, Some(DIAGNOSTIC_FORMAT_BUDGET_US), false),
            (false, Some(0), false),
            (false, Some(-50), false),
            (true, None, false),
            (true, Some(10_000), false),
        ] {
            assert_eq!(diagnostic_format_allowed(pending, next), expected);
        }
        // The gate sits in the shared USB send path so every formatted line is covered.
        let usb = include_str!("usb_diagnostics.rs");
        assert_eq!(usb.matches("diagnostic_format_allowed(").count(), 1);
    }

    #[test]
    fn actor_source_gates_only_early_service_and_keeps_unconditional_late_service() {
        let source = include_str!("protocol_actor.rs");
        let actor = source.split("    loop {").nth(1).unwrap();
        let service = "service_timing(&mut runtime, &mut backend, &mut inbox).await;";
        let early = actor.find(service).unwrap();
        let late = actor.rfind(service).unwrap();
        assert_eq!(actor.matches(service).count(), 2);
        let captured = actor
            .find("let command_present = requests.command.is_some();")
            .unwrap();
        let moved = actor
            .find("if let Some(command) = requests.command")
            .unwrap();
        assert!(captured < moved);
        let conditional = actor
            .find("if early_timing_allowed(cfg!(feature = \"early-timing-service\"), command_present) {")
            .unwrap();
        assert!(moved < conditional && conditional < early);
        let after_early = &actor[early + service.len()..];
        assert!(after_early.trim_start().starts_with('}'));
        assert!(actor.find("Event::Command(command)").unwrap() < early);
        assert!(actor.find("if requests.status").unwrap() < early);
        assert!(early < actor.find("command_work_handler_async").unwrap());
        assert!(actor.find("audio_ingress_work_handler").unwrap() < late);
        assert!(actor[..late].ends_with("}\n\n        "));
        assert_eq!(
            actor
                .matches("cfg!(feature = \"early-timing-service\")")
                .count(),
            1
        );
        assert!(late < actor.find("let now = backend.now_us()").unwrap());
        assert!(late < actor.find("wait(&mut inbox).await").unwrap());
        assert_eq!(source.matches("backend.timing.drain(").count(), 1);
    }

    #[test]
    fn drain_participant_slot_one_waits_for_physical_compare() {
        use crate::actor_policy::ActorCallback;
        use crate::timing_clock::ClockProfile;
        for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
            let mut model = DrainModel::new(profile);
            let mut engine = model.start(1);
            model.advance(model.registers[0].active.unwrap().deadline);
            assert_eq!(model.drain(&mut engine), [None; 3]);
            let slot = model.registers[1].active.unwrap();
            assert!(model.registers[1].enabled);
            assert!(slot.deadline > model.now);
            model.advance(slot.deadline - 1);
            assert_eq!(model.drain(&mut engine), [None; 3]);
            assert_eq!(model.registers[1].active, Some(slot));
            model.advance(slot.deadline);
            assert_eq!(
                model.drain(&mut engine),
                [Some(ActorCallback::Slot { slot: 1, frame: 1 }), None, None]
            );
        }
    }

    #[test]
    fn drain_rejects_canceled_and_stale_records_without_consuming_current_arm() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        let mut engine = model.start(1);
        let old = model.arm(1, 100);
        let current = model.arm(1, 200);
        model.registers[1].pending = Some(old);
        let canceled = model.arm(2, 100);
        model.registers[2].reset_arm();
        model.registers[2].pending = Some(canceled);
        model.advance(100);
        assert_eq!(model.drain(&mut engine), [None; 3]);
        assert_eq!(model.registers[1].active, Some(current));
        assert!(model.registers[1].enabled);
        assert_eq!(model.registers[1].repairs, 0);
        assert_eq!(model.dispatches, 0);
        assert_eq!(model.registers[2].active, None);
    }

    #[test]
    fn drain_early_repair_crossing_compare_is_accepted_in_second_batch_once() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        let mut engine = model.start(1);
        model.advance(model.registers[0].active.unwrap().deadline);
        assert_eq!(model.drain(&mut engine), [None; 3]);
        let slot = model.registers[1].active.unwrap();
        let allocations = model.registers[1].allocations;
        model.registers[1].event = true;
        assert_eq!(model.registers[1].irq(), Some(slot));
        model.registers[1].cross_during_post_clear = true;
        let collections = model.collections;
        assert_eq!(
            model.drain(&mut engine),
            [
                Some(crate::actor_policy::ActorCallback::Slot { slot: 1, frame: 1 }),
                None,
                None,
            ]
        );
        assert_eq!(model.collections - collections, 2);
        assert_eq!(model.registers[1].allocations, allocations);
        assert_eq!(model.registers[1].repairs, 1);
        assert_eq!(model.registers[1].callbacks, 1);
        assert_eq!(model.dispatched[1], Some((1, slot)));
        assert_eq!(model.drain(&mut engine), [None; 3]);
        assert_eq!(model.registers[1].callbacks, 1);
    }

    #[test]
    fn drain_two_batch_cap_preserves_new_pending_epochs_and_wake() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        for index in 0..3 {
            model.arm(index, 0);
        }
        for expected in [6, 12] {
            model.wake = false;
            let collections = model.collections;
            drain_pending_alarms(
                &mut model,
                DrainModel::collect,
                |platform, index, record| {
                    assert!(platform.accept(index, record));
                    let next = platform.arm(index, 0);
                    assert!(next.epoch > record.epoch);
                },
            );
            assert_eq!(model.collections - collections, 2);
            assert_eq!(model.dispatches, expected);
            for offset in 0..6 {
                assert_eq!(
                    model.dispatched[expected - 6 + offset].unwrap().0,
                    offset % 3
                );
            }
            assert!(model.wake);
            for registers in &model.registers {
                assert!(registers.pending.is_some());
                assert_eq!(registers.pending, registers.active);
            }
        }
    }

    #[test]
    fn drain_orders_full_deadlines_and_rejects_same_kind_rearm_collision() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        let frame = model.arm(0, 20);
        let slot = model.arm(1, 30);
        let control = model.arm(2, 10);
        model.advance(30);
        let mut replacement = None;
        drain_pending_alarms(
            &mut model,
            DrainModel::collect,
            |platform, index, record| {
                if platform.accept(index, record) && index == 0 {
                    replacement = Some(platform.arm(1, 30));
                }
            },
        );
        assert_eq!(model.collections, 2);
        assert_eq!(model.dispatches, 3);
        assert_eq!(model.dispatched[0], Some((2, control)));
        assert_eq!(model.dispatched[1], Some((0, frame)));
        assert_eq!(model.dispatched[2], Some((1, replacement.unwrap())));
        assert_ne!(replacement, Some(slot));
        assert_eq!(model.registers[1].callbacks, 1);
    }

    #[test]
    fn drain_batches_coalesce_all_work_before_one_callback_buffer_take() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        let mut engine = model.start(0);
        engine.set_clock_source(true);
        model.advance(36_000);
        for index in [1, 2] {
            model.arm(index, 0);
        }
        model.alarms(&mut engine);
        assert_eq!(model.collections, 2);
        assert_eq!(model.dispatches, 4);
        assert_eq!(model.work.len, 3);
        assert_eq!(engine.stats().control_submit_drop, 1);
        model.execute_work(&mut engine);
        assert_eq!(model.work.len, 0);
        assert_eq!(
            model.callbacks.take(),
            [
                Some(crate::actor_policy::ActorCallback::Control { frame: 1 }),
                None,
                None,
            ]
        );
    }

    #[test]
    fn drain_expired_work_and_skipped_frames_do_not_create_slot_catchup() {
        use crate::timing_clock::ClockProfile;
        let mut model = DrainModel::new(ClockProfile::Normalized);
        let mut engine = model.start(0);
        model.advance(20_000);
        model.alarms(&mut engine);
        assert_eq!(model.work.len, 1);
        model.advance(21_501);
        model.execute_work(&mut engine);
        assert_eq!(model.callbacks.take(), [None; 3]);
        assert_eq!(engine.stats().slot_late_drop, 1);
        model.advance(63_000);
        assert_eq!(model.drain(&mut engine), [None; 3]);
        assert_eq!(engine.stats().skipped_frame_count, 1);
        assert_eq!(model.registers[1].active, None);
        assert_eq!(engine.stats().slot_late_drop, 2);
        assert!(model.registers[0].active.unwrap().deadline > model.now);
    }

    // A thread-mode model, not a simulated Ready runtime or hardware IRQ proof.
    struct Model {
        now: i64,
        epoch: u64,
        arms: [Option<AlarmRecord>; 3],
        work: WorkFifo,
        sampler: nrf_embassy::ws::WsSampler,
        edges: u32,
        sampled_frame: u32,
        captures: u32,
        callbacks: crate::actor_policy::CallbackBuffer,
    }
    impl nrf_embassy::tdma::Clock for Model {
        fn now_us(&mut self) -> i64 {
            self.now
        }
        fn uptime_ms(&mut self) -> u32 {
            (self.now / 1_000) as u32
        }
    }
    impl nrf_embassy::tdma::Timers for Model {
        fn init_timer(&mut self, kind: TimerKind) {
            self.cancel_timer(kind);
        }
        fn arm_timer(&mut self, kind: TimerKind, delay: i64) {
            self.epoch = next_epoch(self.epoch).unwrap();
            self.arms[timer_index(kind)] = Some(AlarmRecord {
                deadline: deadline(self.now, delay).unwrap(),
                epoch: self.epoch,
            });
        }
        fn cancel_timer(&mut self, kind: TimerKind) {
            self.arms[timer_index(kind)] = None;
        }
    }
    impl nrf_embassy::tdma::WorkQueue for Model {
        fn init_work(&mut self, kind: WorkKind) {
            self.work.cancel(kind);
        }
        fn submit_work(&mut self, kind: WorkKind) -> i32 {
            self.work.submit(kind)
        }
        fn cancel_work(&mut self, kind: WorkKind) {
            self.work.cancel(kind);
        }
    }
    impl nrf_embassy::tdma::WsCapture for Model {
        fn capture_ws(&mut self) -> Option<u32> {
            // The next Frame arm must already be installed at capture phase.
            assert!(self.arms[0].unwrap().deadline > self.now);
            self.captures += 1;
            Some(self.edges)
        }
        fn sample_ws(&mut self, frame: u32, edges: u32) -> Option<i32> {
            self.sampled_frame = frame;
            let result = self.sampler.sample(edges, frame);
            result.valid.then_some(result.correction_us)
        }
    }
    impl nrf_embassy::tdma::Callbacks for Model {
        fn slot_callback(&mut self, slot: u8, frame: u32) {
            self.callbacks
                .push(crate::actor_policy::ActorCallback::Slot { slot, frame });
        }
        fn control_callback(&mut self, frame: u32) {
            self.callbacks
                .push(crate::actor_policy::ActorCallback::Control { frame });
        }
    }

    #[test]
    fn unsynchronized_slot_zero_frames_capture_and_discipline_without_rf() {
        let mut model = Model {
            now: 0,
            epoch: 0,
            arms: [None; 3],
            work: WorkFifo::default(),
            sampler: nrf_embassy::ws::WsSampler::new(),
            edges: 0,
            sampled_frame: 0,
            captures: 0,
            callbacks: crate::actor_policy::CallbackBuffer::default(),
        };
        model.sampler.start();
        let mut engine = nrf_embassy::tdma::Tdma::new();
        engine.init(&mut model);
        engine.set_callbacks(false, false);
        engine.start(0, false, &mut model).unwrap();
        engine.set_clock_source(true);
        for edges in [960, 1880, 2840] {
            let record = model.arms[0].take().unwrap();
            model.now = record.deadline;
            model.edges = edges;
            engine.timer_expired(TimerKind::Frame, &mut model).unwrap();
            assert_eq!(model.arms[1..], [None, None]);
            while let Some(kind) = model.work.pop() {
                engine.work_executed(kind, &mut model).unwrap();
            }
            assert_eq!(model.callbacks.take(), [None; 3]);
            assert_eq!(model.sampled_frame, engine.frame_counter());
        }
        let state = engine.snapshot();
        assert_eq!(state.slot_index, 0);
        assert!(!state.synchronized);
        assert!(!state.slot_callback_enabled && !state.control_callback_enabled);
        assert_eq!(model.captures, 3);
        assert_eq!(state.stats.tune_request_count, 2);
        assert_eq!(state.stats.correction_apply_count, 1);
        assert_eq!(state.pending_correction_us, 108);
        engine.stop(&mut model);
        assert_eq!(model.arms, [None; 3]);
        assert_eq!(model.work.pop(), None);
    }

    #[test]
    fn checked_tdma_windows_buffer_callbacks_in_work_fifo_order() {
        use crate::actor_policy::{ActorCallback, CallbackBuffer};
        use nrf_embassy::tdma::{FRAME_US, Tdma};
        let mut model = Model {
            now: 0,
            epoch: 0,
            arms: [None; 3],
            work: WorkFifo::default(),
            sampler: nrf_embassy::ws::WsSampler::new(),
            edges: 0,
            sampled_frame: 0,
            captures: 0,
            callbacks: CallbackBuffer::default(),
        };
        let mut tdma = Tdma::new();
        tdma.init(&mut model);
        tdma.set_callbacks(true, true);
        tdma.start(0, true, &mut model).unwrap();
        model.now = FRAME_US;
        tdma.timer_expired(TimerKind::Frame, &mut model).unwrap();
        let slot = model.arms[1].unwrap();
        model.now = slot.deadline;
        tdma.timer_expired(TimerKind::Slot, &mut model).unwrap();
        while let Some(work) = model.work.pop() {
            tdma.work_executed(work, &mut model).unwrap();
        }
        assert_eq!(
            model.callbacks.take(),
            [Some(ActorCallback::Slot { slot: 0, frame: 1 }), None, None]
        );
        let control = model.arms[2].unwrap();
        model.now = control.deadline;
        tdma.timer_expired(TimerKind::Control, &mut model).unwrap();
        while let Some(work) = model.work.pop() {
            tdma.work_executed(work, &mut model).unwrap();
        }
        let callbacks = model.callbacks.take();
        assert_eq!(
            callbacks,
            [Some(ActorCallback::Control { frame: 1 }), None, None]
        );
        // The buffer loan has ended before an actor handler can await; returned
        // values remain stable while the platform is used again.
        tdma.stop(&mut model);
        assert_eq!(model.callbacks.take(), [None; 3]);
        assert_eq!(callbacks[0], Some(ActorCallback::Control { frame: 1 }));
        tdma.work_executed(WorkKind::Control, &mut model).unwrap();
        assert_eq!(model.callbacks.take(), [None; 3]);
    }
}
