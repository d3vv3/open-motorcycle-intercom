use openhelmet_mesh::{
    adaptive::{self, Adaptive, Identity, Membership},
    participant_session as p,
};
fn identity(n: u8) -> Identity {
    Identity {
        address_len: 6,
        address: [n; 6],
    }
}
fn owner() -> Adaptive {
    let mut s = Adaptive::default();
    adaptive::init(Some(&mut s), 2, 1, 7, 3, 0);
    for id in 1..=2 {
        adaptive::set_member(Some(&mut s), id, Some(&identity(id)));
    }
    s
}
#[test]
fn departure_is_bound_projection_only_including_leader() {
    let s = owner();
    assert_eq!(
        p::departure(&s, true, 2, 1, 1, identity(1), identity(2), true),
        Some(1)
    );
    for (active, local, leader, source, mac, projection) in [
        (false, 2, 1, 1, 1, true),
        (true, 3, 1, 1, 1, true),
        (true, 2, 3, 1, 1, true),
        (true, 2, 1, 2, 2, true),
        (true, 2, 1, 1, 9, true),
        (true, 2, 1, 1, 1, false),
        (true, 2, 1, 0, 1, true),
    ] {
        assert_eq!(
            p::departure(
                &s,
                active,
                local,
                leader,
                source,
                identity(mac),
                identity(2),
                projection
            ),
            None
        );
    }
    assert_eq!(s.session(), (2, 1, 3, 7));
    // Empty LEAVE has no incarnation: the same binding is intentionally accepted again.
    assert_eq!(
        p::departure(&s, true, 2, 1, 1, identity(1), identity(2), true),
        Some(1)
    );
}
#[test]
fn cold_destination_preserves_slot_zero_and_requires_accepted_yield() {
    let d = p::cold_destination(true, identity(1), 1, 1).unwrap();
    assert_eq!((d.slot, d.term, d.leader_id), (0, 1, 1));
    assert!(p::cold_destination(false, identity(1), 1, 1).is_none());
    assert!(p::cold_destination(true, identity(1), 0, 1).is_none());
    assert!(p::cold_destination(true, identity(1), 1, 2).is_none());
}
#[test]
fn eviction_reuses_authority_and_constructs_exact_destination() {
    let s = owner();
    let mut m = Membership {
        term: 7,
        leader_id: 1,
        revision: 1,
        ..Default::default()
    };
    m.slot_map.slot_count = 2;
    m.slot_map.slot_ids[1] = 1;
    m.identities[0] = identity(1);
    let d = p::eviction(&s, 1, &m).unwrap();
    assert_eq!((d.leader, d.slot, d.term), (identity(1), 1, 7));
    assert!(p::eviction(&s, 2, &m).is_none());
    m.term = 8;
    assert!(p::eviction(&s, 1, &m).is_none());
    m.term = 7;
    m.identities[0] = identity(9);
    assert!(p::eviction(&s, 1, &m).is_none());
}
