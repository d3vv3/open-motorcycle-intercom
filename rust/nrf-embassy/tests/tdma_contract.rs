use nrf_embassy::tdma::{
    Callbacks, Clock, Error, Tdma, TimerKind, Timers, WorkKind, WorkQueue, WsCapture, FRAME_US,
    LEGACY_CORRECTION_QUANTUM_US,
};
use nrf_embassy::ws::evaluate;
use std::fmt::Write as _;

fn timer_id(k: TimerKind) -> usize {
    match k {
        TimerKind::Frame => 0,
        TimerKind::Slot => 1,
        TimerKind::Control => 2,
    }
}
fn work_id(k: WorkKind) -> usize {
    match k {
        WorkKind::Slot => 0,
        WorkKind::Control => 1,
        WorkKind::Discipline => 2,
    }
}

// Legacy-oracle conversions only; production TimingPlatform remains in us.
fn legacy_ticks(raw_us: i64) -> i64 {
    assert!(raw_us >= 0);
    raw_us / 100
}
fn ticks_to_us(ticks: i64) -> i64 {
    ticks.checked_mul(100).unwrap()
}
fn timeout_ticks(delay_us: i64) -> i64 {
    let delay_us = delay_us.max(0);
    delay_us / 100 + i64::from(delay_us % 100 != 0)
}
fn pending_deadline(raw_now_us: i64, ticks: i64) -> i64 {
    assert!(ticks >= 0);
    ticks_to_us(legacy_ticks(raw_now_us))
        .checked_add(ticks_to_us(ticks.max(1)))
        .unwrap()
}

struct Mock {
    output: String,
    now: i64,
    second: i64,
    reads: u32,
    timers: [(bool, i64); 3],
    // Rust-side request arguments remain separate from normalized oracle effects.
    requested_arms: Vec<(TimerKind, i64)>,
    absolute_arms: Option<Vec<(TimerKind, i64, i64)>>,
    // Extreme-domain tests bypass only the legacy oracle's clock quantization.
    clock_samples: Option<[i64; 2]>,
    work: [bool; 3],
    submit: i32,
    capture: Option<u32>,
    first: bool,
    last_frame: u32,
    last_edges: u32,
}
impl Default for Mock {
    fn default() -> Self {
        Self {
            output: String::new(),
            now: 0,
            second: 0,
            reads: 0,
            timers: [(false, 0); 3],
            requested_arms: Vec::new(),
            absolute_arms: None,
            clock_samples: None,
            work: [false; 3],
            submit: 1,
            capture: None,
            first: true,
            last_frame: 0,
            last_edges: 0,
        }
    }
}
impl Mock {
    fn arm_timeout(&mut self, k: TimerKind, ticks: i64, period: i64) {
        if ticks == -1 {
            return; // K_FOREVER duration is no effect, not cancellation.
        }
        assert!(period == 0 || period == -1);
        let i = timer_id(k);
        let deadline = pending_deadline(self.now, ticks);
        self.timers[i] = (true, deadline);
        writeln!(self.output, "A {i} {ticks} {deadline}").unwrap();
    }
}
impl Clock for Mock {
    fn now_us(&mut self) -> i64 {
        if let Some(samples) = self.clock_samples {
            let sample = samples[usize::from(self.reads != 0)];
            self.reads += 1;
            return sample;
        }
        if self.reads != 0 {
            self.now = self.second;
        }
        self.reads += 1;
        ticks_to_us(legacy_ticks(self.now))
    }
    fn uptime_ms(&mut self) -> u32 {
        (self.now / 1000) as u32
    }
}
impl Timers for Mock {
    fn init_timer(&mut self, k: TimerKind) {
        let i = timer_id(k);
        self.timers[i] = (false, 0);
        writeln!(self.output, "I T {i}").unwrap();
    }
    fn arm_timer(&mut self, k: TimerKind, delay: i64) {
        assert!(delay >= 0);
        self.requested_arms.push((k, delay));
        let ticks = timeout_ticks(delay);
        // Normalize at the same post-K_USEC boundary visible to the C mock.
        self.arm_timeout(k, ticks, 0);
    }
    fn arm_timer_at(&mut self, k: TimerKind, target: i64, delay: i64) {
        if let Some(arms) = &mut self.absolute_arms {
            arms.push((k, target, delay));
        }
        self.arm_timer(k, delay);
    }
    fn cancel_timer(&mut self, k: TimerKind) {
        let i = timer_id(k);
        self.timers[i].0 = false;
        writeln!(self.output, "C T {i}").unwrap();
    }
}
impl WorkQueue for Mock {
    fn init_work(&mut self, k: WorkKind) {
        let i = work_id(k);
        self.work[i] = false;
        writeln!(self.output, "I W {i}").unwrap();
    }
    fn submit_work(&mut self, k: WorkKind) -> i32 {
        let i = work_id(k);
        if self.submit > 0 {
            self.work[i] = true;
        }
        writeln!(self.output, "S {i} {}", self.submit).unwrap();
        self.submit
    }
    fn cancel_work(&mut self, k: WorkKind) {
        let i = work_id(k);
        self.work[i] = false;
        writeln!(self.output, "C W {i}").unwrap();
    }
}
impl Callbacks for Mock {
    fn slot_callback(&mut self, slot: u8, frame: u32) {
        writeln!(self.output, "B S {slot} {frame}").unwrap();
    }
    fn control_callback(&mut self, frame: u32) {
        writeln!(self.output, "B C {frame}").unwrap();
    }
}
impl WsCapture for Mock {
    fn capture_ws(&mut self) -> Option<u32> {
        writeln!(
            self.output,
            "X {} {}",
            u8::from(self.capture.is_some()),
            self.capture.unwrap_or(0)
        )
        .unwrap();
        self.capture
    }
    fn sample_ws(&mut self, frame: u32, edges: u32) -> Option<i32> {
        let result = if self.first {
            None
        } else {
            evaluate(
                edges.wrapping_sub(self.last_edges),
                frame.wrapping_sub(self.last_frame),
            )
            .outputs()
            .map(|(_, correction)| correction)
        };
        self.first = false;
        self.last_frame = frame;
        self.last_edges = edges;
        writeln!(
            self.output,
            "Y {frame} {edges} {} {}",
            u8::from(result.is_some()),
            result.unwrap_or(0)
        )
        .unwrap();
        result
    }
}

#[test]
fn detached_snapshot_and_checked_unsupported_domain() {
    let mut t = Tdma::new();
    let mut p = Mock::default();
    t.init(&mut p);
    let mut detached = t.snapshot();
    detached.slot_index = -128;
    detached.history[0].valid = true;
    assert_eq!(t.snapshot().slot_index, -1);
    assert!(!t.snapshot().history[0].valid);
    assert_eq!(detached.slot_index, -128);
    assert!(detached.history[0].valid);
    t.start(0, true, &mut p).unwrap();
    p.now = 10;
    p.second = 10;
    p.reads = 0;
    assert_eq!(t.frame_phase(&mut p).unwrap(), Some(0));
    p.now = 0;
    p.second = 0;
    p.reads = 0;
    let arms_before = p.requested_arms.len();
    // MIN + FRAME_US and 0 - (MIN + FRAME_US) are defined i64 arithmetic.
    // This fails because the catch-up increment is not representable in u32.
    assert_eq!(t.sync(100, 0, i64::MIN, &mut p), Err(Error::TimingOverflow));
    assert_eq!(p.requested_arms.len(), arms_before);
    let mut t = Tdma::new();
    p.now = i64::MAX;
    p.second = i64::MAX;
    p.reads = 0;
    assert_eq!(t.start(0, true, &mut p), Err(Error::TimingOverflow));
}

#[test]
fn sync_catchup_bound_and_small_negative_epoch_never_arm_past_deadline() {
    let max = i64::from(u32::MAX);
    let mut t = Tdma::new();
    let mut p = Mock::default();
    t.start(0, false, &mut p).unwrap();
    t.sync(1, 0, -max * FRAME_US, &mut p).unwrap();
    assert_eq!(t.frame_counter(), 0); // Valid MAX increment retains u32 wrap.
    assert_eq!(t.snapshot().frame_start_us, 0);
    assert_eq!(t.snapshot().next_frame_deadline_us, FRAME_US);
    assert_eq!(p.requested_arms.last(), Some(&(TimerKind::Frame, FRAME_US)));
    assert!(p.requested_arms.iter().all(|(_, delay)| *delay >= 0));

    let mut t = Tdma::new();
    let mut p = Mock::default();
    t.start(0, false, &mut p).unwrap();
    let arms_before = p.requested_arms.len();
    assert_eq!(
        t.sync(1, 0, -(max + 1) * FRAME_US, &mut p),
        Err(Error::TimingOverflow)
    );
    assert_eq!(p.requested_arms.len(), arms_before);
    assert!(p.requested_arms.iter().all(|(_, delay)| *delay >= 0));

    let mut t = Tdma::new();
    let mut p = Mock::default();
    t.start(0, false, &mut p).unwrap();
    t.sync(7, 0, -500, &mut p).unwrap();
    assert_eq!(t.frame_counter(), 7);
    assert_eq!(t.snapshot().frame_start_us, -500);
    assert_eq!(t.snapshot().next_frame_deadline_us, FRAME_US - 500);
    assert_eq!(
        p.requested_arms.last(),
        Some(&(TimerKind::Frame, FRAME_US - 500))
    );
    assert!(p.requested_arms.iter().all(|(_, delay)| *delay >= 0));
}

#[test]
fn legacy_conversion_goldens_and_pending_expiry() {
    for (delay, ticks, rounded, deadline) in [
        (0, 0, 0, 200),
        (1, 1, 100, 200),
        (99, 1, 100, 200),
        (100, 1, 100, 200),
        (101, 2, 200, 300),
        (19_999, 200, 20_000, 20_100),
        (20_000, 200, 20_000, 20_100),
    ] {
        assert_eq!(timeout_ticks(delay), ticks);
        assert_eq!(ticks_to_us(ticks), rounded);
        let mut p = Mock {
            now: 199,
            second: 199,
            ..Mock::default()
        };
        p.arm_timer(TimerKind::Frame, delay);
        assert_eq!(p.timers[0], (true, deadline));
        assert_eq!(p.output, format!("A 0 {ticks} {deadline}\n"));
        assert_eq!(p.requested_arms, [(TimerKind::Frame, delay)]);
    }
    assert_eq!(timeout_ticks(0), 0);
    for (now, ticks, us) in [(19, 0, 0), (99, 0, 0), (100, 1, 100), (199, 1, 100)] {
        assert_eq!(legacy_ticks(now), ticks);
        let mut p = Mock {
            now,
            second: now,
            ..Mock::default()
        };
        assert_eq!(p.now_us(), us);
        p.arm_timer(TimerKind::Frame, 0);
        assert_eq!(p.timers[0], (true, us + 100));
    }
    let mut p = Mock {
        now: 199,
        second: 199,
        ..Mock::default()
    };
    p.arm_timeout(TimerKind::Frame, -1, 0);
    assert_eq!(p.timers[0], (false, 0));
    assert!(p.output.is_empty());
    p.arm_timeout(TimerKind::Frame, 0, -1);
    assert_eq!(p.timers[0], (true, 200));
    assert_eq!(p.output, "A 0 0 200\n");
    p.arm_timeout(TimerKind::Frame, 1, -1);
    assert_eq!(p.timers[0], (true, 200));
    let output = p.output.clone();
    p.arm_timeout(TimerKind::Frame, -1, -1);
    assert_eq!(p.timers[0], (true, 200));
    assert_eq!(p.output, output);
    let mut t = Tdma::new();
    let mut p = Mock {
        now: 199,
        second: 199,
        ..Mock::default()
    };
    t.start(2, false, &mut p).unwrap();
    t.sync(0, 0, 101, &mut p).unwrap();
    assert_eq!(t.snapshot().frame_start_us, 101);
    assert_eq!(t.snapshot().next_frame_deadline_us, 20_101);
    assert_eq!(p.requested_arms.last(), Some(&(TimerKind::Frame, 20_001)));
    assert_eq!(p.timers[0], (true, 20_200));
    assert!(p.output.ends_with("A 0 201 20200\n"));
    for (first, second, frame, deadline, request, backend_deadline) in [
        (20_101, 20_199, 0, 20_101, 1, 20_200),
        (20_199, 20_299, 1, 40_101, 19_901, 40_200),
    ] {
        let mut t = Tdma::new();
        let mut p = Mock::default();
        t.start(2, false, &mut p).unwrap();
        p.now = first;
        p.second = second;
        p.reads = 0;
        t.sync(0, 0, 101, &mut p).unwrap();
        assert_eq!(t.frame_counter(), frame);
        assert_eq!(t.snapshot().next_frame_deadline_us, deadline);
        assert_eq!(p.requested_arms.last(), Some(&(TimerKind::Frame, request)));
        assert_eq!(p.timers[0], (true, backend_deadline));
    }
}

#[test]
fn default_absolute_thunk_forwards_each_hint_once_without_clock_reads() {
    #[derive(Default)]
    struct Recorder {
        arms: Vec<(TimerKind, i64)>,
    }
    impl Timers for Recorder {
        fn init_timer(&mut self, _: TimerKind) {}
        fn arm_timer(&mut self, kind: TimerKind, delay_us: i64) {
            self.arms.push((kind, delay_us));
        }
        fn cancel_timer(&mut self, _: TimerKind) {}
    }
    impl Clock for Recorder {
        fn now_us(&mut self) -> i64 {
            panic!("the default timer thunk must not sample the clock")
        }
        fn uptime_ms(&mut self) -> u32 {
            panic!("the default timer thunk must not sample uptime")
        }
    }

    let mut p = Recorder::default();
    let expected = [
        (TimerKind::Slot, 0),
        (TimerKind::Frame, 100),
        (TimerKind::Control, 20_001),
        (TimerKind::Frame, -1),
    ];
    for (target, (kind, delay)) in [i64::MIN, i64::MAX, i64::MIN, i64::MAX]
        .into_iter()
        .zip(expected)
    {
        p.arm_timer_at(kind, target, delay);
    }
    // The negative hint exercises only this primitive, not production TDMA.
    assert_eq!(p.arms, expected);
}

#[test]
fn start_absolute_target_survives_second_clock_sample_and_minimum_hint() {
    let mut t = Tdma::new();
    let mut p = Mock {
        now: 199,
        second: 21_199,
        absolute_arms: Some(Vec::new()),
        ..Mock::default()
    };
    t.start(0, true, &mut p).unwrap();
    assert_eq!(p.reads, 2);
    assert_eq!(t.snapshot().frame_start_us, 100);
    assert_eq!(t.snapshot().next_frame_deadline_us, 20_100);
    assert_eq!(
        p.absolute_arms.as_deref().unwrap(),
        [(TimerKind::Frame, 20_100, 100)]
    );
    assert_eq!(p.requested_arms, [(TimerKind::Frame, 100)]);
    assert_eq!(p.timers[0], (true, 21_200));
    assert_eq!(p.output, "A 0 1 21200\n");
}

#[test]
fn late_frame_preserves_past_slot_and_control_targets_with_zero_hints() {
    // Slot and Control have distinct windows: each overdue start remains
    // available independently of its clamped hint and the future Frame target.
    for (slot, now, expected, trace) in [
        (
            0,
            20_500,
            vec![
                (TimerKind::Slot, 20_000, 0),
                (TimerKind::Control, 36_000, 15_500),
                (TimerKind::Frame, 40_000, 19_500),
            ],
            "A 1 0 20600\nA 2 155 36000\nA 0 195 40000\n",
        ),
        (
            7,
            36_500,
            vec![
                (TimerKind::Control, 36_000, 0),
                (TimerKind::Frame, 40_000, 3500),
            ],
            "A 2 0 36600\nA 0 35 40000\n",
        ),
    ] {
        let mut t = Tdma::new();
        let mut p = Mock::default();
        t.start(slot, true, &mut p).unwrap();
        p.now = now;
        p.second = 900_000;
        p.reads = 0;
        p.output.clear();
        p.requested_arms.clear();
        p.absolute_arms = Some(Vec::new());
        t.timer_expired(TimerKind::Frame, &mut p).unwrap();
        assert_eq!(p.reads, 1);
        assert_eq!(p.absolute_arms.as_ref().unwrap(), &expected);
        assert_eq!(p.output, trace);
        let snapshot = t.snapshot();
        for &(kind, target, _) in &expected {
            let metadata_target = match kind {
                TimerKind::Slot => snapshot.slot.start_us,
                TimerKind::Control => snapshot.control.start_us,
                TimerKind::Frame => snapshot.next_frame_deadline_us,
            };
            assert_eq!(target, metadata_target);
        }
        assert_eq!(snapshot.stats.slot_late_drop, u32::from(slot == 7));
    }
}

#[test]
fn frame_absolute_sequence_matches_metadata_and_precedes_ws_capture() {
    for synchronized in [false, true] {
        let mut t = Tdma::new();
        let mut p = Mock::default();
        t.start(0, synchronized, &mut p).unwrap();
        // Callback enable flags do not gate timer arms; synchronization does.
        t.set_callbacks(false, false);
        t.set_clock_source(true);
        t.tune_timing(100);
        p.capture = Some(123);
        for (now, next, delay) in [(20_000, 40_100, 20_100), (40_100, 60_100, 20_000)] {
            p.now = now;
            p.second = 900_000;
            p.reads = 0;
            p.output.clear();
            p.requested_arms.clear();
            p.absolute_arms = Some(Vec::new());
            t.timer_expired(TimerKind::Frame, &mut p).unwrap();
            let mut detached = t.snapshot();
            let mut expected = Vec::new();
            let mut trace = String::new();
            if synchronized {
                expected.push((TimerKind::Slot, detached.slot.start_us, 0));
                expected.push((TimerKind::Control, detached.control.start_us, 16_000));
                writeln!(trace, "A 1 0 {}", now + 100).unwrap();
                writeln!(trace, "A 2 160 {}", now + 16_000).unwrap();
            }
            expected.push((TimerKind::Frame, next, delay));
            writeln!(trace, "A 0 {} {next}", delay / 100).unwrap();
            trace.push_str("X 1 123\nS 2 1\n");
            assert_eq!(p.absolute_arms.as_ref().unwrap(), &expected);
            assert_eq!(p.output, trace);
            assert_eq!(p.reads, 1);
            assert_eq!(detached.next_frame_deadline_us, next);
            assert_eq!(detached.frame_start_us, now);
            detached.next_frame_deadline_us = -1;
            detached.slot.start_us = -1;
            assert_eq!(detached.next_frame_deadline_us, -1);
            assert_eq!(detached.slot.start_us, -1);
            assert_eq!(t.snapshot().next_frame_deadline_us, next);
            assert_eq!(p.absolute_arms.as_ref().unwrap(), &expected);
        }
    }
}

#[test]
fn sync_absolute_catchup_uses_post_cancel_clock_and_invalidates_stale_work() {
    let mut t = Tdma::new();
    let mut p = Mock::default();
    t.start(2, true, &mut p).unwrap();
    t.set_callbacks(true, true);
    p.now = 20_000;
    p.second = 20_000;
    p.reads = 0;
    t.timer_expired(TimerKind::Frame, &mut p).unwrap();
    t.timer_expired(TimerKind::Slot, &mut p).unwrap();
    t.timer_expired(TimerKind::Control, &mut p).unwrap();
    let stale = t.snapshot();
    p.now = 25_000;
    p.second = 65_000;
    p.reads = 0;
    p.output.clear();
    p.requested_arms.clear();
    p.absolute_arms = Some(Vec::new());
    // A distant frame triggers the same acquisition path as the golden corpus.
    t.sync(100, 17, 0, &mut p).unwrap();
    assert_eq!(p.reads, 2);
    assert_eq!(
        p.absolute_arms.as_deref().unwrap(),
        [(TimerKind::Frame, 80_000, 15_000)]
    );
    assert_eq!(
        p.output,
        "C T 0\nC T 1\nC T 2\nC W 0\nC W 1\nC W 2\nA 0 150 80000\n"
    );
    let current = t.snapshot();
    assert_eq!(current.frame_counter, 103);
    assert_eq!(current.frame_start_us, 60_000);
    assert_eq!(current.next_frame_deadline_us, 80_000);
    assert_eq!(current.generation, stale.generation.wrapping_add(1));
    assert_eq!(current.slot, stale.slot);
    assert_eq!(current.control, stale.control);
    assert_eq!(p.timers, [(true, 80_000), (false, 24_000), (false, 36_000)]);
    assert_eq!(p.work, [false; 3]);
    p.output.clear();
    // Deliver stale work in its old valid window: generation still rejects it.
    p.now = 24_000;
    p.second = 24_000;
    p.reads = 0;
    t.work_executed(WorkKind::Slot, &mut p).unwrap();
    p.now = 36_000;
    p.second = 36_000;
    p.reads = 0;
    t.work_executed(WorkKind::Control, &mut p).unwrap();
    assert!(p.output.is_empty());
}

#[test]
fn absolute_hook_preserves_checked_overflow_and_prior_partial_effects() {
    let mut t = Tdma::new();
    let mut p = Mock {
        absolute_arms: Some(Vec::new()),
        clock_samples: Some([0, i64::MIN]),
        ..Mock::default()
    };
    assert_eq!(t.start(0, true, &mut p), Err(Error::TimingOverflow));
    assert_eq!(p.reads, 2);
    assert!(t.snapshot().running);
    assert_eq!(t.snapshot().generation, 1);
    assert_eq!(t.snapshot().next_frame_deadline_us, FRAME_US);
    assert!(p.absolute_arms.as_ref().unwrap().is_empty());
    assert!(p.requested_arms.is_empty());
    assert!(p.output.is_empty());

    // Public start constructs the near-MAX state; no snapshot mutation or
    // private-state injection is needed. Control's subtraction then overflows
    // after Slot has armed, so neither Frame nor WS capture may run.
    let initial = i64::MAX - 60_007;
    let mut t = Tdma::new();
    let mut p = Mock {
        now: initial,
        second: initial,
        ..Mock::default()
    };
    t.start(0, true, &mut p).unwrap();
    t.set_clock_source(true);
    p.now = 0;
    p.reads = 0;
    p.clock_samples = Some([-30_000, 900_000]);
    p.absolute_arms = Some(Vec::new());
    p.requested_arms.clear();
    p.output.clear();
    assert_eq!(
        t.timer_expired(TimerKind::Frame, &mut p),
        Err(Error::TimingOverflow)
    );
    let snapshot = t.snapshot();
    assert_eq!(p.reads, 1);
    assert_eq!(snapshot.frame_counter, 1);
    assert_eq!(snapshot.frame_start_us, i64::MAX - 40_007);
    assert_eq!(snapshot.next_frame_deadline_us, i64::MAX - 20_007);
    assert_eq!(
        p.absolute_arms.as_deref().unwrap(),
        [(TimerKind::Slot, snapshot.slot.start_us, i64::MAX - 10_007)]
    );
    assert_eq!(p.requested_arms.len(), 1);
    assert!(p.timers[1].0);
    assert_eq!(p.output.lines().count(), 1);
    assert!(p.output.starts_with("A 1 "));
    assert_eq!(snapshot.stats.discipline_due, 0);

    // With the same public near-MAX setup, -23_000 leaves both Slot and
    // Control differences representable. Frame's later subtraction is
    // MAX + 2993, so both earlier arms survive and WS is still unreached.
    let mut t = Tdma::new();
    let mut p = Mock {
        now: initial,
        second: initial,
        capture: Some(123),
        ..Mock::default()
    };
    t.start(0, true, &mut p).unwrap();
    t.set_clock_source(true);
    p.now = 0;
    p.reads = 0;
    p.clock_samples = Some([-23_000, 900_000]);
    p.absolute_arms = Some(Vec::new());
    p.requested_arms.clear();
    p.output.clear();
    assert_eq!(
        t.timer_expired(TimerKind::Frame, &mut p),
        Err(Error::TimingOverflow)
    );
    let snapshot = t.snapshot();
    assert_eq!(p.reads, 1);
    assert_eq!(snapshot.frame_counter, 1);
    assert_eq!(snapshot.frame_start_us, i64::MAX - 40_007);
    assert_eq!(snapshot.next_frame_deadline_us, i64::MAX - 20_007);
    assert_eq!(snapshot.slot.start_us, snapshot.frame_start_us);
    assert_eq!(snapshot.control.start_us, i64::MAX - 24_007);
    let slot_delay = snapshot.slot.start_us.checked_sub(-23_000).unwrap();
    let control_delay = snapshot.control.start_us.checked_sub(-23_000).unwrap();
    assert_eq!(slot_delay, i64::MAX - 17_007);
    assert_eq!(control_delay, i64::MAX - 1007);
    assert_eq!(snapshot.next_frame_deadline_us.checked_sub(-23_000), None);
    assert_eq!(
        p.absolute_arms.as_deref().unwrap(),
        [
            (TimerKind::Slot, snapshot.slot.start_us, slot_delay),
            (TimerKind::Control, snapshot.control.start_us, control_delay),
        ]
    );
    assert_eq!(
        p.requested_arms,
        [
            (TimerKind::Slot, slot_delay),
            (TimerKind::Control, control_delay),
        ]
    );
    assert_eq!(p.timers[1], (true, slot_delay));
    assert_eq!(p.timers[2], (true, control_delay));
    assert_eq!(
        p.output,
        format!(
            "A 1 {} {slot_delay}\nA 2 {} {control_delay}\n",
            slot_delay / 100,
            control_delay / 100
        )
    );
    assert_eq!(snapshot.stats.discipline_due, 0);
    assert_eq!(snapshot.discipline_frame, 0);
    assert_eq!(snapshot.discipline_edge_count, 0);
    assert_eq!(p.work, [false; 3]);
}

struct Harness {
    t: Tdma,
    p: Mock,
}
impl Harness {
    fn start(slot: i8, synchronized: bool) -> Self {
        let mut h = Self {
            t: Tdma::new(),
            p: Mock::default(),
        };
        h.t.init(&mut h.p);
        h.t.start(slot, synchronized, &mut h.p).unwrap();
        h
    }
    fn at(&mut self, now: i64) -> &mut Self {
        self.p.now = now;
        self.p.second = now;
        self.p.reads = 0;
        self
    }
    fn frame(&mut self) -> &mut Self {
        self.p.timers[0].0 = false;
        self.t.timer_expired(TimerKind::Frame, &mut self.p).unwrap();
        self
    }
    fn sync(&mut self, frame: u32, ppm: i16, start: i64) -> &mut Self {
        self.t.sync(frame, ppm, start, &mut self.p).unwrap();
        self
    }
    fn rate(&self) -> (bool, i16) {
        let s = self.t.stats();
        (s.remote_rate_valid, s.remote_rate_ppm)
    }
    fn out(&mut self) -> String {
        std::mem::take(&mut self.p.output)
    }
}

/// Independent statement of `tests/c/tdma_remote_rate_test.c`: the estimator
/// learns the remote frame period from SYNC observations once it holds >= 10
/// frames of span, clamps to +-5000 ppm, ignores reordered/duplicate/outlier
/// observations and rebaselines after a gap of more than 250 frames.
#[test]
fn remote_rate_estimator_requirements() {
    // Phase reacquisition never resets the learned period.
    let mut h = Harness::start(0, true);
    assert_eq!(h.sync(0, 0, 0).rate(), (false, 0));
    for frame in (10..=120).step_by(10) {
        h.sync(frame, 0, i64::from(frame) * 20_100);
        assert_eq!(h.rate(), (true, 5000));
    }
    // Sign and clamp, from a fresh estimator each time.
    for (time, ppm) in [
        (199_000, -5000),
        (200_000, 0),
        (201_200, 5000),
        (198_800, -5000),
    ] {
        let mut h = Harness::start(0, true);
        h.sync(0, 0, 0);
        assert_eq!(h.sync(10, 0, time).rate(), (true, ppm), "{time}");
    }
    // Jitter within tolerance is averaged across the window; an outlier is ignored,
    // then a corrected timestamp for the same frame is accepted.
    let mut h = Harness::start(0, true);
    h.sync(0, 0, 0);
    for frame in (10..=110).step_by(10) {
        let jitter = if (frame / 10) % 2 == 1 { 100 } else { -100 };
        h.sync(frame, 0, i64::from(frame) * 20_100 + jitter);
        let (valid, ppm) = h.rate();
        assert!(valid && (4500..=5000).contains(&ppm), "{frame}: {ppm}");
    }
    let learned = h.rate();
    assert_eq!(h.sync(120, 0, 120 * 20_100 + 3000).rate(), learned);
    let (valid, ppm) = h.sync(120, 0, 120 * 20_100).rate();
    assert!(valid && (4500..=5000).contains(&ppm));
    // Duplicate, reordered, zero-elapsed and bad long-gap observations are ignored;
    // a plausible long gap rebaselines and must relearn.
    let mut h = Harness::start(0, true);
    h.sync(0, 0, 0);
    assert_eq!(h.sync(30, 0, 603_000).rate(), (true, 5000));
    for (frame, time) in [(30, 1), (20, 402_000), (40, 603_000), (300, 1)] {
        assert_eq!(h.sync(frame, 0, time).rate(), (true, 5000), "{frame}");
    }
    assert_eq!(h.sync(300, 0, 6_030_000).rate(), (false, 0));
    assert_eq!(h.sync(330, 0, 6_633_000).rate(), (true, 5000));
    // Serial frame wrap.
    let mut h = Harness::start(0, true);
    h.sync(u32::MAX - 9, 0, 1_000_000);
    assert_eq!(h.sync(0, 0, 1_201_000).rate(), (true, 5000));
    // New leader epoch after a long gap is learned from the new anchor only.
    let mut h = Harness::start(0, true);
    h.sync(0, 0, 1_000_000);
    assert_eq!(h.sync(10, 0, 1_201_000).rate(), (true, 5000));
    assert_eq!(h.sync(300, 0, 8_000_000).rate(), (false, 0));
    assert_eq!(h.sync(330, 0, 8_597_000).rate(), (true, -5000));
    assert_eq!(h.sync(340, 0, 9_000_000).rate(), (true, -5000));
    // Frequent syncs keep a long enough window to converge.
    let mut h = Harness::start(0, true);
    for frame in 0..=100 {
        h.sync(frame, 0, i64::from(frame) * 20_100);
    }
    assert_eq!(h.rate(), (true, 5000));
    // Local clock source suppresses the estimator and uses the advertised drift.
    let mut h = Harness::start(0, false);
    h.t.set_clock_source(true);
    assert_eq!(h.sync(10, 1234, 201_000).rate(), (false, 0));
    let scaled = 1234 * FRAME_US as i32;
    assert_eq!(h.t.snapshot().rate_whole_us, scaled / 1_000_000);
    assert_eq!(h.t.snapshot().rate_fraction, scaled % 1_000_000);
    assert_eq!(h.t.snapshot().slot_index, 0);
}

#[test]
fn slot_and_control_windows_fire_once_and_drop_late_work() {
    for slot in 0..8_i64 {
        let mut h = Harness::start(slot as i8, true);
        h.t.set_callbacks(true, true);
        h.at(20_000).frame();
        let start = 20_000 + slot * 2000;
        let arms = h.p.requested_arms.clone();
        assert!(
            arms.iter()
                .any(|&(k, d)| k == TimerKind::Slot && d == slot * 2000),
            "{arms:?}"
        );
        assert!(arms
            .iter()
            .any(|&(k, d)| k == TimerKind::Control && d == 16_000));
        // On time: callback uses the current slot index with the current frame.
        h.at(start);
        h.p.timers[1].0 = false;
        h.t.timer_expired(TimerKind::Slot, &mut h.p).unwrap();
        h.p.work[0] = false;
        h.t.work_executed(WorkKind::Slot, &mut h.p).unwrap();
        assert!(h.out().ends_with(&format!("B S {slot} 1\n")));
        // Beyond 1500 us late the deferred work is dropped, not executed.
        // (The mock clock has 100 us resolution, as the legacy kernel tick.)
        h.at(40_000).frame();
        h.at(start + 20_000 + 1600);
        h.p.timers[1].0 = false;
        h.t.timer_expired(TimerKind::Slot, &mut h.p).unwrap();
        h.p.work[0] = false;
        h.t.work_executed(WorkKind::Slot, &mut h.p).unwrap();
        let o = h.out();
        assert!(
            !o.contains("B S"),
            "slot {slot}: {o:?} {:?}",
            h.t.snapshot().slot
        );
        let s = h.t.stats();
        assert_eq!((s.slot_due, s.slot_late_drop), (2, 1));
        // Control window is at +16 ms of every frame.
        h.at(56_000);
        h.p.timers[2].0 = false;
        h.t.timer_expired(TimerKind::Control, &mut h.p).unwrap();
        h.p.work[1] = false;
        h.t.work_executed(WorkKind::Control, &mut h.p).unwrap();
        assert!(h.out().ends_with("B C 2\n"));
        h.at(57_600);
        h.p.timers[2].0 = false;
        h.t.timer_expired(TimerKind::Control, &mut h.p).unwrap();
        h.p.work[1] = false;
        h.t.work_executed(WorkKind::Control, &mut h.p).unwrap();
        assert!(!h.out().contains("B C"));
        let s = h.t.stats();
        assert_eq!((s.control_due, s.control_late_drop), (2, 1));
        // A failed submission counts as a drop, and disabled callbacks stay silent.
        h.p.submit = -1;
        h.at(60_000).frame();
        h.at(60_000 + slot * 2000);
        h.p.timers[1].0 = false;
        h.t.timer_expired(TimerKind::Slot, &mut h.p).unwrap();
        assert_eq!(h.t.stats().slot_submit_drop, 1);
        h.p.submit = 1;
        h.t.set_callbacks(false, false);
        h.at(80_000).frame();
        h.at(80_000 + slot * 2000);
        h.p.timers[1].0 = false;
        h.t.timer_expired(TimerKind::Slot, &mut h.p).unwrap();
        h.p.work[0] = false;
        h.t.work_executed(WorkKind::Slot, &mut h.p).unwrap();
        assert!(!h.out().contains("B S"));
        assert_eq!(h.t.frame_counter(), 4);
    }
}

#[test]
fn sync_reacquires_outside_frame_and_phase_tolerances_and_defers_history_misses() {
    // Frame-count difference beyond +-4 reacquires; within it uses history phase.
    for diff in [-5_i32, -4, -1, 0, 1, 4, 5] {
        let mut h = Harness::start(1, false);
        h.sync(100, 0, 0);
        assert_eq!(h.t.stats().sync_acquire_count, 1);
        for frame in 1..=6 {
            h.at(i64::from(frame) * 20_000).frame();
        }
        let incoming = (106 + diff) as u32;
        let start = i64::from(incoming - 100) * 20_000;
        h.sync(incoming, 23, start);
        let s = h.t.stats();
        assert_eq!(s.sync_frame_diff, diff);
        assert_eq!(s.sync_reacquire_count, u32::from(diff.abs() > 4), "{diff}");
        if diff.abs() <= 4 {
            assert_eq!(s.sync_phase_correction_us, 0);
            assert_eq!(s.correction_pending_us, 0);
        }
        if diff.abs() <= 4 {
            assert_eq!(h.t.frame_counter(), 106);
        } else {
            // Reacquired from the remote epoch, advanced to the present boundary.
            assert!(h.t.frame_counter() >= incoming);
            assert_eq!(
                h.t.snapshot().frame_start_us + FRAME_US,
                h.t.snapshot().next_frame_deadline_us
            );
        }
    }
    // Phase error beyond +-500 us reacquires; within it becomes a pending correction.
    for phase in [-2001_i32, -501, -500, -499, 0, 499, 500, 501, 2001] {
        let mut h = Harness::start(0, true);
        h.t.set_clock_source(true);
        h.at(20_000).frame();
        h.sync(1, -23, 20_000 + i64::from(phase));
        let s = h.t.stats();
        // Reacquisition restarts from the remote epoch and clears the diagnostics.
        let reacquired = phase.abs() > 500;
        assert_eq!(s.sync_reacquire_count, u32::from(reacquired), "{phase}");
        assert_eq!(
            s.sync_phase_correction_us,
            if reacquired { 0 } else { phase }
        );
        assert_eq!(s.correction_pending_us, if reacquired { 0 } else { phase });
    }
    // Synchronized with no matching history entry: deferred, no correction.
    let mut h = Harness::start(0, true);
    h.at(20_000).frame();
    h.sync(3, 0, 60_000);
    let s = h.t.stats();
    assert_eq!((s.sync_history_miss_count, s.sync_reacquire_count), (1, 0));
    assert_eq!(s.sync_phase_correction_us, 0);
    // Not running: SYNC is ignored entirely.
    let mut h = Harness::start(0, true);
    h.t.stop(&mut h.p);
    h.sync(50, 0, 1_000_000);
    assert_eq!(h.t.stats().sync_acquire_count, 0);
    assert_eq!(h.t.frame_counter(), 0);
}

#[test]
fn pending_corrections_apply_at_most_the_quantum_per_frame_and_tune_clamps() {
    let mut h = Harness::start(7, true);
    h.t.tune_timing(i32::MAX);
    assert_eq!(h.t.stats().correction_pending_us, 2000);
    h.t.tune_timing(i32::MIN);
    assert_eq!(h.t.stats().correction_pending_us, -2000);
    h.t.tune_timing(2000);
    h.t.tune_timing(350);
    let s = h.t.stats();
    assert_eq!((s.tune_request_count, s.tune_clamp_count), (4, 2));
    assert_eq!(s.correction_pending_us, 350);
    let mut applied = vec![];
    for _ in 0..5 {
        let due = h.t.snapshot().next_frame_deadline_us;
        h.at(due).frame();
        applied.push(h.t.stats().last_correction_us);
    }
    // Each frame applies one 100 us quantum while at least a quantum is pending.
    assert_eq!(applied, [100, 100, 100, 0, 0]);
    let s = h.t.stats();
    assert_eq!(s.correction_applied_us, 300);
    assert_eq!(s.correction_apply_count, 3);
    assert_eq!(s.correction_pending_us, 50);
    // Frame timer requests moved by exactly the applied corrections.
    let arms: Vec<_> =
        h.p.requested_arms
            .iter()
            .filter(|(k, _)| *k == TimerKind::Frame)
            .map(|(_, d)| *d)
            .collect();
    assert_eq!(arms, [20_000, 20_100, 20_100, 20_100, 20_000, 20_000]);
    // Negative corrections pull the deadline in; a callback already past the
    // pulled-in deadline keeps the nominal boundary and returns the quantum.
    h.t.tune_timing(-2000);
    assert_eq!(h.t.snapshot().next_frame_deadline_us, 120_300);
    h.at(120_300).frame();
    assert_eq!(h.t.stats().last_correction_us, -100);
    assert_eq!(h.t.snapshot().next_frame_deadline_us, 140_200);
    h.at(160_150).frame();
    assert_eq!(h.t.stats().last_correction_us, 0);
    assert_eq!(h.t.stats().correction_pending_us, -1850);
    assert_eq!(h.t.snapshot().next_frame_deadline_us, 160_200);
}

#[test]
fn rate_ppm_accumulates_fractional_microseconds_without_drift() {
    for ppm in [-5000_i16, -51, -1, 0, 1, 51, 5000] {
        let mut h = Harness::start(0, false);
        h.t.set_clock_source(true);
        h.sync(0, ppm, 0);
        for frame in 1..=120 {
            h.at(i64::from(frame) * 20_000).frame();
        }
        let s = h.t.stats();
        let expected = i64::from(ppm) * FRAME_US * 120 / 1_000_000;
        let total = s.correction_applied_us + i64::from(s.correction_pending_us);
        assert!(
            (total - expected).abs() <= 1,
            "{ppm}: {total} vs {expected}"
        );
        assert_eq!(
            s.correction_applied_us % i64::from(LEGACY_CORRECTION_QUANTUM_US),
            0
        );
        assert_eq!(h.t.frame_counter(), 120);
        assert_eq!(s.skipped_frame_count, 0);
    }
}

#[test]
fn skipped_frames_advance_the_counter_to_the_present_boundary() {
    let mut h = Harness::start(0, true);
    h.at(20_000).frame();
    h.at(40_000).frame();
    // Late by many frames: counter jumps and the skip is accounted.
    h.at(120_000).frame();
    let s = h.t.stats();
    assert!(s.skipped_frame_count >= 3, "{}", s.skipped_frame_count);
    assert_eq!(h.t.frame_counter(), 6);
    assert!(h.t.snapshot().next_frame_deadline_us > 120_000);
    assert_eq!(
        h.t.clock_snapshot(&mut h.p).unwrap().map(|(f, _)| f),
        Some(6)
    );
}
