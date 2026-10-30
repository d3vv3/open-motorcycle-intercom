use openhelmet_mesh::{adaptive as a, adaptive_maintenance::RetryClocks, core::SlotMap};
fn setup(now: u32) -> (a::Adaptive, RetryClocks, SlotMap, a::Handover) {
    let mut s = a::Adaptive::default();
    a::init(Some(&mut s), 1, 1, 7, 3, now);
    let map = SlotMap {
        slot_count: 2,
        slot_ids: [1, 2, 0, 0, 0, 0, 0, 0],
        ..Default::default()
    };
    let mut p = a::Handover::default();
    // API-produced pending proposal; candidate zero is accepted by the scalar API.
    assert!(a::prepare(
        Some(&mut s),
        0,
        u32::MAX,
        Some(&map),
        now,
        Some(&mut p)
    ));
    (s, RetryClocks::default(), map, p)
}
#[test]
fn retry_boundaries_attempt_clock_and_immutable_proposal() {
    let (mut s, mut retry, map, p) = setup(0);
    assert_eq!(retry.stage(&mut s, 2, 179, 1000, &map), None);
    assert_eq!(retry.stage(&mut s, 2, 180, 1000, &map), Some((1, p)));
    // Vendor failure needs no completion/rollback: next attempt remains 180ms away.
    assert_eq!(retry.stage(&mut s, 2, 359, 1001, &map), None);
    assert_eq!(retry.stage(&mut s, 2, 360, 1001, &map), Some((1, p)));
    assert_eq!(retry.stage(&mut s, 2, 2400, 1002, &map), None);
    assert_eq!(retry.stage(&mut s, 2, 2401, 1003, &map), None);
}
#[test]
fn first_commit_at_ack_equality_and_retry_at_recovery_equality() {
    let (mut s, mut retry, map, p) = setup(0);
    let ack = a::HandoverAck {
        old_leader_id: 1,
        candidate_id: p.candidate_id,
        next_term: 8,
        switch_frame: p.switch_frame,
        acknowledger_id: 2,
    };
    assert!(a::ack(Some(&mut s), 2, Some(&ack), 2400));
    assert_eq!(retry.stage(&mut s, 2, 2400, 999, &map), Some((2, p)));
    assert_eq!(retry.stage(&mut s, 3, 2579, 999, &map), None);
    assert_eq!(retry.stage(&mut s, 3, 2580, 999, &map), Some((2, p)));
    assert_eq!(retry.stage(&mut s, 3, 5200, 999, &map), Some((2, p)));
    assert_eq!(retry.stage(&mut s, 3, 5380, 999, &map), None);
}
#[test]
fn wrapped_deadline_and_clock_domain() {
    let now = u32::MAX - 100;
    let (mut s, mut retry, map, p) = setup(now);
    assert_eq!(retry.stage(&mut s, 2, now, 0, &map), Some((1, p)));
    assert_eq!(retry.stage(&mut s, 2, now.wrapping_add(179), 0, &map), None);
    assert_eq!(
        retry.stage(&mut s, 2, now.wrapping_add(180), 0, &map),
        Some((1, p))
    );
    assert_eq!(
        retry.stage(&mut s, 2, now.wrapping_add(2400), 0, &map),
        None
    );
}
