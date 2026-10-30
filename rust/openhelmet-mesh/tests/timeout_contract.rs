use openhelmet_mesh::{
    adaptive::{self as a, Adaptive, Identity},
    timeout::{Input, Row, Transaction, plan},
};
fn setup() -> (Adaptive, Input) {
    let mut s = Adaptive::default();
    a::init(Some(&mut s), 2, 1, 7, 3, 0);
    for id in [1, 2, 3] {
        a::set_member(
            Some(&mut s),
            id,
            Some(&Identity {
                address_len: 6,
                address: [id; 6],
            }),
        );
    }
    let mut i = Input {
        generation: 4,
        active: true,
        participant: true,
        local: 2,
        leader: 1,
        now_ms: 3101,
        ..Input::default()
    };
    i.rows[0] = Row {
        id: 3,
        active: true,
        identity: s.member_identity(3),
        last_seen: 100i64.to_le_bytes(),
    };
    i.rows[3] = Row {
        id: 1,
        active: true,
        identity: s.member_identity(1),
        last_seen: 100i64.to_le_bytes(),
    };
    (s, i)
}
#[test]
fn strict_timeout_order_binding_and_role() {
    let (s, i) = setup();
    let p = plan(&s, i).unwrap();
    assert_eq!(p.rows, 9);
    assert!(p.recover);
    for now in [-1, 0, 100, 3100] {
        let mut b = i;
        b.now_ms = now;
        assert_eq!(plan(&s, b).unwrap().rows, 0);
    }
    let mut b = i;
    b.rows[3].identity.address = [9; 6];
    assert!(plan(&s, b).is_none());
    let mut b = i;
    b.rows[3].active = false;
    assert!(!plan(&s, b).unwrap().recover);
    let mut b = i;
    b.participant = false;
    assert!(plan(&s, b).is_none());
    let mut b = i;
    b.active = false;
    assert!(plan(&s, b).is_none());
}
#[test]
fn token_consumption_cancellation_replacement_and_evidence() {
    let (s, i) = setup();
    for mode in 0..6 {
        let mut t = Transaction::default();
        assert!(t.prepare(&s, i, 11).unwrap().recover);
        let mut b = i;
        match mode {
            0 => b.generation += 1,
            1 => b.rows[3].last_seen = 101i64.to_le_bytes(),
            2 => b.now_ms -= 1,
            3 => t.cancel(),
            4 => {
                t.prepare(&s, i, 12);
            }
            _ => b.leader = 3,
        }
        assert!(!t.consume(&s, b, 11));
    }
    let mut t = Transaction::default();
    t.prepare(&s, i, 11);
    assert!(t.consume(&s, i, 11));
    assert!(!t.consume(&s, i, 11));
}
