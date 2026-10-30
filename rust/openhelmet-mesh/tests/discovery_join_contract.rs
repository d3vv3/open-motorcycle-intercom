use openhelmet_mesh::{
    adaptive::{EXPIRE_MS, Identity},
    control_admission::{JoinAdmission as A, JoinContext, admit_ack, admit_join},
};
fn id(n: u8) -> Identity {
    Identity {
        address_len: 6,
        address: [n; 6],
    }
}
fn context() -> JoinContext {
    JoinContext {
        discovered_term: 7,
        relay_ms: 100,
        state: 1,
        coordinator: 0,
        coordinator_id: 1,
        local: [2; 6],
        coordinator_mac: [1; 6],
        relay: [3; 6],
    }
}
#[test]
fn join_origin_target_and_role_matrix() {
    let c = context();
    assert_eq!(admit_join(c, 0, id(4), id(1), id(1), true), A::Join);
    assert_eq!(admit_join(c, 1, id(4), id(1), id(1), true), A::Rejected);
    assert_eq!(admit_join(c, 0, id(4), id(1), id(1), false), A::Rejected);
    assert_eq!(admit_join(c, 0, id(0), id(1), id(1), true), A::Rejected);
    let short = Identity {
        address_len: 3,
        ..id(4)
    };
    assert_eq!(admit_join(c, 0, short, id(1), id(1), true), A::Rejected);
    for target in [id(3), Identity::default(), short] {
        assert_eq!(admit_join(c, 0, id(4), target, id(1), true), A::Rejected);
    }
    let scanning = JoinContext { state: 0, ..c };
    assert_eq!(
        admit_join(scanning, 0, id(4), Identity::default(), id(1), true),
        A::Join
    );
    let leader = JoinContext {
        coordinator: 1,
        state: 0,
        ..c
    };
    assert_eq!(admit_join(leader, 0, id(4), id(2), id(1), true), A::Join);
    assert_eq!(
        admit_join(leader, 0, id(4), id(1), id(1), true),
        A::Rejected
    );
    for value in 3..=255 {
        assert_eq!(
            admit_join(
                JoinContext { state: value, ..c },
                0,
                id(4),
                id(1),
                id(1),
                true
            ),
            A::Rejected
        );
    }
    for value in 2..=255 {
        assert_eq!(
            admit_join(
                JoinContext {
                    coordinator: value,
                    ..c
                },
                0,
                id(4),
                id(1),
                id(1),
                true
            ),
            A::Rejected
        );
    }
}
#[test]
fn ack_discovered_term_target_binding_and_wrapping_expiry() {
    let c = JoinContext {
        state: 2,
        ..context()
    };
    let accept = |c, source, mac, target, coordinator, term, now, relayed| {
        admit_ack(c, source, mac, target, coordinator, term, now, relayed)
    };
    assert_eq!(accept(c, 1, [1; 6], id(2), 1, 7, 9999, false), A::Ack);
    for term in [0, 6, 8, u32::MAX] {
        assert_eq!(
            accept(c, 1, [1; 6], id(2), 1, term, 100, false),
            A::Rejected
        );
    }
    assert_eq!(accept(c, 2, [1; 6], id(2), 1, 7, 100, false), A::Rejected);
    assert_eq!(accept(c, 1, [1; 6], id(2), 2, 7, 100, false), A::Rejected);
    assert_eq!(accept(c, 1, [3; 6], id(2), 1, 7, 100, false), A::Rejected);
    assert_eq!(accept(c, 1, [1; 6], id(3), 1, 7, 100, false), A::Rejected);
    assert_eq!(
        accept(c, 1, [3; 6], id(2), 1, 7, 100 + EXPIRE_MS, true),
        A::Ack
    );
    assert_eq!(
        accept(c, 1, [3; 6], id(2), 1, 7, 101 + EXPIRE_MS, true),
        A::Rejected
    );
    assert_eq!(
        accept(
            JoinContext { relay_ms: 0, ..c },
            1,
            [3; 6],
            id(2),
            1,
            7,
            0,
            true
        ),
        A::Rejected
    );
    let wrap = JoinContext {
        relay_ms: u32::MAX - 10,
        ..c
    };
    assert_eq!(accept(wrap, 1, [3; 6], id(2), 1, 7, 2989, true), A::Ack);
    assert_eq!(
        accept(wrap, 1, [3; 6], id(2), 1, 7, 2990, true),
        A::Rejected
    );
}
