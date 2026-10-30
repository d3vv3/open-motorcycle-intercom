use openhelmet_mesh::{adaptive as a, core::SlotMap, topology::Topology};
fn setup() -> a::Adaptive {
    let mut s = a::Adaptive::default();
    a::init(Some(&mut s), 1, 1, 7, 3, 0);
    for id in 1..=2 {
        assert!(a::set_member(
            Some(&mut s),
            id,
            Some(&a::Identity {
                address_len: 6,
                address: [id; 6]
            })
        ));
    }
    s
}
fn map() -> SlotMap {
    SlotMap {
        slot_count: 2,
        slot_ids: [1, 2, 0, 0, 0, 0, 0, 0],
        ..Default::default()
    }
}
#[test]
fn observation_zero_rounding_pdr_wrap_and_expiry() {
    let mut s = setup();
    let mut t = Topology::default();
    assert!(!t.observe(&s, 2, [9; 6], false, -40, 1));
    assert!(!t.observe(&s, 2, [2; 6], true, -40, 1));
    assert!(t.observe(&s, 2, [2; 6], false, -200, 0));
    t.observe(&s, 2, [2; 6], false, -41, 1);
    t.observe(&s, 2, [2; 6], false, -40, 2);
    t.direct_report(2, u32::MAX, 2);
    t.direct_report(2, 1, 2);
    t.direct_report(2, 1, 9999); // duplicate does not refresh report freshness
    let r = t.produce(&mut s, true, map(), 3002).unwrap();
    assert_eq!(r.rssi_dbm[1], -40);
    assert_eq!(r.quality[1], 88);
    assert_eq!(r.direct_mask, 2);
    assert_eq!(t.produce(&mut s, true, map(), 3003).unwrap().direct_mask, 0);
    t.observe(&s, 2, [2; 6], false, 10, u32::MAX - 1);
    t.direct_report(2, 2, u32::MAX - 1);
    assert_eq!(t.produce(&mut s, true, map(), 1).unwrap().direct_mask, 2);
}
#[test]
fn relayed_first_direct_second_and_term_transition_persist_estimator() {
    let mut s = setup();
    let mut t = Topology::default();
    let remote = a::Topology {
        origin: s.member_identity(2),
        report_seq: 10,
        leader_id: 1,
        term: 7,
        direct_mask: 1,
        rssi_dbm: [-40, 0, 0, 0, 0, 0, 0, 0],
        quality: [100, 0, 0, 0, 0, 0, 0, 0],
        slot_map: map(),
    };
    assert!(a::report(Some(&mut s), 2, Some(&remote), true, 100));
    assert!(a::report_valid(Some(&s), 2, Some(&remote)));
    assert!(t.observe(&s, 2, [2; 6], false, -40, 100));
    t.direct_report(2, 10, 100);
    assert!(!a::report(Some(&mut s), 2, Some(&remote), false, 100));
    let local = t.produce(&mut s, true, map(), 100).unwrap();
    assert_eq!(local.direct_mask, 2);
    assert_eq!(local.quality[1], 100);
    // A real graph term transition must not reset the independent estimator/counters.
    let mut follower = a::Adaptive::default();
    a::init(Some(&mut follower), 2, 1, 7, 3, 0);
    for id in 1..=2 {
        assert!(a::set_member(
            Some(&mut follower),
            id,
            Some(&s.member_identity(id))
        ));
    }
    let mut peer = local;
    peer.direct_mask = 2;
    peer.rssi_dbm = [0, -40, 0, 0, 0, 0, 0, 0];
    peer.quality = [0, 100, 0, 0, 0, 0, 0, 0];
    assert!(a::report(Some(&mut follower), 1, Some(&peer), false, 100));
    let mut estimator = Topology::default();
    assert!(estimator.observe(&follower, 1, [1; 6], false, -41, 100));
    estimator.direct_report(1, 10, 100);
    let before = estimator.produce(&mut follower, true, map(), 100).unwrap();
    let (wire, token) = estimator.stamp(&follower, before, 55).unwrap();
    assert_eq!(wire.report_seq, 0);
    let p = a::Handover {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 20,
        members: 3,
        slot_map: map(),
    };
    assert!(a::receive_prepare(Some(&mut follower), 1, Some(&p), 0, 100));
    assert!(a::receive_commit(Some(&mut follower), 1, Some(&p), 100));
    assert!(a::tick(Some(&mut follower), 20, 101));
    assert_eq!(follower.session().3, 8);
    assert!(!estimator.result_current(&follower, token, false));
    let next = estimator.produce(&mut follower, true, map(), 101).unwrap();
    assert_eq!(next.report_seq, 1);
    assert_eq!(next.direct_mask, 1);
    let (wire, _) = estimator.stamp(&follower, next, 56).unwrap();
    assert_eq!(wire.report_seq, 1);
}
#[test]
fn cadence_reinit_local_acceptance_and_wire_result() {
    let mut s = setup();
    let mut t = Topology::default();
    assert!(!t.due(999_999));
    assert!(t.due(1_000_000));
    t.reinit();
    assert!(!t.due(1_999_999));
    assert!(t.due(2_000_000));
    assert!(
        t.produce(
            &mut s,
            true,
            SlotMap {
                slot_count: 9,
                ..Default::default()
            },
            2000
        )
        .is_none()
    );
    let r = t.produce(&mut s, true, map(), 2000).unwrap();
    assert_eq!(r.report_seq, 1);
    assert!(!a::report(Some(&mut s), 1, Some(&r), false, 2000));
    let (r0, token) = t.stamp(&s, r, 123).unwrap();
    assert_eq!(r0.report_seq, 0);
    assert!(!t.result(token, 124, false));
    assert!(t.result(token, 123, false));
    assert!(!t.result(token, 123, false));
    let (r0, token) = t.stamp(&s, r, 123).unwrap();
    assert_eq!(r0.report_seq, 0);
    assert!(t.result(token, 123, true));
    let (r1, token) = t.stamp(&s, r, 123).unwrap();
    assert_eq!(r1.report_seq, 1);
    t.reinit();
    assert!(!t.result(token, 123, false));
}
