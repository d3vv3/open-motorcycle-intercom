use openhelmet_mesh::scan::{Effect, Transaction};

#[test]
fn timing_success_only_pacing_and_single_consumption() {
    let mut s = Transaction::default();
    s.begin(1_000_000, 7);
    assert_eq!(s.beacon(300_000, 7), Effect::None);
    let Effect::Beacon(t) = s.beacon(1_000_000, 7) else {
        panic!()
    };
    assert_eq!(s.beacon(1_000_001, 7), Effect::None);
    assert!(!s.result(8, t, true));
    assert!(s.result(7, t, false));
    let Effect::Beacon(t) = s.beacon(1_000_001, 7) else {
        panic!()
    };
    assert!(s.result(7, t, true));
    assert!(!s.result(7, t, true));
    assert_eq!(s.beacon(1_300_001, 7), Effect::None);
    assert!(matches!(s.beacon(1_300_002, 7), Effect::Beacon(_)));
    assert_eq!(s.elect(3_000_000, 7), Effect::None);
    assert_eq!(s.elect(3_000_001, 8), Effect::None);
    assert_eq!(s.elect(3_000_001, 7), Effect::Activate);
    assert_eq!(s.elect(3_000_002, 7), Effect::None);
}

#[test]
fn lower_address_restarts_episode_full_width_and_stale_completion() {
    let mut s = Transaction::default();
    let now = (1_i64 << 40) + 1;
    s.begin(now, 2);
    let Effect::Beacon(old) = s.beacon(now, 2) else {
        panic!()
    };
    s.begin(now, 2);
    assert!(!s.result(2, old, true));
    assert!(!s.lower(now + 1_000_000, 2, &[9; 6], &[8; 6]));
    assert!(!s.lower(now + 1_000_000, 2, &[8; 6], &[8; 6]));
    assert!(s.lower(now + 1_000_000, 2, &[7; 6], &[8; 6]));
    assert!(s.lower(now + 3_000_000, 2, &[7; 6], &[8; 6]));
    assert_eq!(s.elect(now + 6_000_000, 2), Effect::None);
    assert_eq!(s.elect(now + 6_000_001, 2), Effect::Activate);
    s.begin(now, 2);
    assert!(s.discover(2));
    assert!(!s.discover(2));
    assert_eq!(s.elect(now + 9_000_000, 2), Effect::None);
    s.begin(now, 3);
    s.cancel(2);
    assert!(s.discover(3));
    s.begin(now, 3);
    s.cancel(3);
    assert!(!s.discover(3));
}
