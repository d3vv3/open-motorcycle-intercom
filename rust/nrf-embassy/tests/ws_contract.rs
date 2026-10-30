use nrf_embassy::ws::{
    evaluate, SampleDecision, WsSampler, WsStats, EXPECTED_EDGES_PER_FRAME, MAX_ELAPSED_FRAMES,
};

/// Independent statement of the WS math: a sample is valid only inside the ±25% edge window
/// of 1..=50 frames; error is the edge deficit in microseconds (48 kHz edges, 960 per 20 ms
/// frame, rounded half away from zero); correction is a quarter of the error clamped to ±500.
fn expected(delta: u32, elapsed: u32) -> SampleDecision {
    if elapsed == 0 || elapsed > MAX_ELAPSED_FRAMES {
        return SampleDecision::Rejected;
    }
    if delta == 0 {
        return SampleDecision::NoSignal;
    }
    let nominal = elapsed * EXPECTED_EDGES_PER_FRAME;
    if delta < nominal * 3 / 4 || delta > nominal * 5 / 4 {
        return SampleDecision::Rejected;
    }
    let edges = i64::from(nominal) - i64::from(delta);
    // 1 edge = 1/48000 s = 20.8333 us
    let scaled = edges * 1_000_000;
    let error_us = (scaled.abs() + 24_000) / 48_000 * scaled.signum();
    SampleDecision::Valid {
        error_us: error_us as i32,
        correction_us: (error_us / 4).clamp(-500, 500) as i32,
    }
}

#[test]
fn math_matches_independent_window_rounding_and_clamp_model() {
    for elapsed in 0..=MAX_ELAPSED_FRAMES + 1 {
        let nominal = elapsed * EXPECTED_EDGES_PER_FRAME;
        let (low, high) = (nominal * 3 / 4, nominal * 5 / 4);
        let mut probes = vec![0, 1, low.saturating_sub(1), low, high, high + 1, u32::MAX];
        if (1..=MAX_ELAPSED_FRAMES).contains(&elapsed) {
            probes.extend(low..=high);
        }
        for delta in probes {
            assert_eq!(
                evaluate(delta, elapsed),
                expected(delta, elapsed),
                "{delta} {elapsed}"
            );
        }
    }
    for elapsed in [MAX_ELAPSED_FRAMES + 1, 256, u32::MAX] {
        for delta in [0, 960, u32::MAX] {
            assert_eq!(evaluate(delta, elapsed), SampleDecision::Rejected);
        }
    }
    assert_eq!(evaluate(960 * 3 / 4, 1).outputs(), Some((5000, 500)));
    assert_eq!(evaluate(960 * 5 / 4, 1).outputs(), Some((-5000, -500)));
    assert_eq!(evaluate(960 * 50, 50).outputs(), Some((0, 0)));
}

#[test]
fn sampler_counts_every_sample_and_folds_only_valid_corrections() {
    let mut sampler = WsSampler::new();
    assert!(!sampler.snapshot().running);
    // Stopped: no counters, no state.
    let idle = (sampler.snapshot(), sampler.stats());
    assert_eq!(sampler.sample(100, 10), Default::default());
    assert_eq!((sampler.snapshot(), sampler.stats()), idle);
    sampler.start();
    // First sample only anchors; counts edges and samples, not validity.
    assert!(!sampler.sample(u32::MAX, u32::MAX).valid);
    let s = sampler.stats();
    assert_eq!(
        (s.sample_count, s.total_edges, s.last_delta_edges),
        (1, u32::MAX, u32::MAX)
    );
    assert_eq!(s.valid_count + s.no_signal_count + s.rejected_count, 0);
    assert!(!sampler.snapshot().first_sample);
    // Wrapping edge and frame counters: 0 - MAX = 1 edge over 1 frame -> rejected (too few).
    assert!(!sampler.sample(0, 0).valid);
    assert_eq!(sampler.stats().rejected_count, 1);
    // No edges over one frame -> no signal.
    assert!(!sampler.sample(0, 1).valid);
    assert_eq!(sampler.stats().no_signal_count, 1);
    // Exactly nominal edges -> valid with zero error.
    let r = sampler.sample(960, 2);
    assert_eq!(
        r,
        nrf_embassy::ws::SampleResult {
            valid: true,
            correction_us: 0
        }
    );
    // Four edges short over one frame -> +83 us error, +20 us correction.
    let r = sampler.sample(960 + 956, 3);
    assert_eq!(r.correction_us, 20);
    let s = sampler.stats();
    assert_eq!(
        s,
        WsStats {
            total_edges: u32::MAX.wrapping_add(1).wrapping_add(960).wrapping_add(956),
            sample_count: 5,
            valid_count: 2,
            no_signal_count: 1,
            rejected_count: 1,
            last_delta_edges: 956,
            last_correction_us: 20,
            cumulative_drift_us: 83,
        }
    );
    // Rejected sample keeps the last correction and drift, still counts edges.
    assert!(!sampler.sample(960 + 956 + 5000, 4).valid);
    let s2 = sampler.stats();
    assert_eq!((s2.last_correction_us, s2.cumulative_drift_us), (20, 83));
    assert_eq!(s2.last_delta_edges, 5000);
    // Stop freezes everything; restart resets stats and re-anchors.
    sampler.stop();
    let frozen = (sampler.snapshot(), sampler.stats());
    assert_eq!(sampler.sample(1, 1), Default::default());
    assert_eq!((sampler.snapshot(), sampler.stats()), frozen);
    sampler.start();
    assert_eq!(sampler.stats(), WsStats::default());
    assert!(sampler.snapshot().first_sample && sampler.snapshot().running);
    assert_eq!(sampler.snapshot().last_count, 0);
    // start while running is a no-op (does not re-anchor or clear stats).
    sampler.sample(10, 10);
    sampler.start();
    assert!(!sampler.snapshot().first_sample);
    assert_eq!(sampler.stats().sample_count, 1);
}

#[test]
fn api_semantics_and_saturation() {
    assert_eq!(evaluate(0, 0), SampleDecision::Rejected);
    assert_eq!(evaluate(0, 1), SampleDecision::NoSignal);
    assert_eq!(evaluate(u32::MAX, 50).outputs(), None);
    assert_eq!(evaluate(956, 1).outputs(), Some((83, 20)));
    assert_eq!(evaluate(964, 1).outputs(), Some((-83, -20)));
    let mut sampler = WsSampler::new();
    let before = sampler.snapshot();
    assert!(!sampler.sample(u32::MAX, u32::MAX).valid);
    assert_eq!(sampler.snapshot(), before);
    sampler.start();
    assert!(!sampler.sample(0, 0).valid);
    let mut count = 0_u32;
    let mut frame = 0_u32;
    for (delta, repetitions, limit) in [
        (36000_u32, 9000, i32::MAX),
        (60000, 18000, i32::MIN),
        (36000, 18000, i32::MAX),
    ] {
        for _ in 0..repetitions {
            count = count.wrapping_add(delta);
            frame = frame.wrapping_add(50);
            assert!(sampler.sample(count, frame).valid);
        }
        assert_eq!(sampler.stats().cumulative_drift_us, limit);
        let retained = sampler.stats();
        frame = frame.wrapping_add(1);
        assert!(!sampler.sample(count, frame).valid);
        assert_eq!(
            sampler.stats().last_correction_us,
            retained.last_correction_us
        );
        assert_eq!(sampler.stats().cumulative_drift_us, limit);
    }
    sampler.stop();
    let retained = sampler.stats();
    let baseline = sampler.snapshot();
    assert_eq!(sampler.sample(42, 42).correction_us, 0);
    assert_eq!(sampler.stats(), retained);
    assert_eq!(sampler.snapshot(), baseline);
    sampler.start();
    assert_eq!(sampler.stats(), Default::default());
    assert!(sampler.snapshot().first_sample);
}
