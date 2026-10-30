use openhelmet_mesh::{
    join::{Effect, Transaction},
    membership as m,
};

#[test]
fn precise_retry_failure_deadline_and_once_only_results() {
    let mut t = Transaction::default();
    t.begin(1_000_001, 7);
    let Effect::Send(token) = t.poll(1_000_001, 7) else {
        panic!()
    };
    assert_eq!(t.result(7, token, false), Some(0));
    assert_eq!(t.result(7, token, true), None);
    assert_eq!(t.poll(1_500_001, 7), Effect::None);
    let Effect::Send(next) = t.poll(1_500_002, 7) else {
        panic!()
    };
    assert_ne!(next, token);
    assert_eq!(t.result(6, next, true), None);
    assert_eq!(t.result(7, next, true), Some(1));
    assert_eq!(t.poll(6_000_000, 6), Effect::None);
    let _ = t.poll(6_000_000, 7); // deadline minus one
    assert_eq!(t.poll(6_000_001, 7), Effect::Expire);
    assert_eq!(t.poll(6_000_002, 7), Effect::None);
    assert_eq!(t.result(7, next, true), None);
    // Above the low-word boundary, preserving full microsecond precision.
    let entry = (1_i64 << 40) + 123;
    t.begin(entry, 9);
    assert_ne!(t.poll(entry + 4_999_999, 9), Effect::Expire);
    assert_eq!(t.poll(entry + 5_000_000, 9), Effect::Expire);
}

#[test]
fn ack_before_deadline_poll_slot_zero_collision_cancel_and_stale_generation() {
    let mut t = Transaction::default();
    t.begin(100, 1);
    let context = m::Snapshot {
        state: m::State::Joining,
        coordinator_id: 3,
        address_len: 6,
        local_address: [9; 6],
        ..Default::default()
    };
    let mut ack = m::JoinAck {
        assigned_id: 3,
        coordinator_id: 3,
        slot_index: 0,
        target_address: [9; 6],
        address_len: 6,
    };
    assert!(!t.ack(1, &context, 3, ack)); // collision, not legacy slot-zero restriction
    ack.assigned_id = 2;
    assert!(!t.ack(2, &context, 3, ack));
    assert!(t.ack(1, &context, 3, ack)); // no RX timestamp rejection, even at deadline
    assert!(!t.ack(1, &context, 3, ack));
    assert_eq!(t.poll(5_000_100, 1), Effect::None);
    t.begin(200, 1);
    let Effect::Send(old) = t.poll(500_001, 1) else {
        panic!()
    };
    t.cancel(2); // wrong lifecycle cannot cancel current attempt
    t.begin(300, 1);
    assert_eq!(t.result(1, old, true), None);
    t.cancel(1);
    assert!(!t.ack(1, &context, 3, ack));
}
