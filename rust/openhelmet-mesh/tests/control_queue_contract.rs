use openhelmet_mesh::control_queue::*;
fn item(t: u8, src: u8, age: i64) -> Item {
    let mut i = Item {
        kind: t,
        ..Item::default()
    };
    i.data[1] = t;
    i.data[2] = src;
    i.len = 17;
    i.enqueued_us = age;
    i.priority = priority(t);
    i
}
#[test]
fn priorities_replacement_and_status_union() {
    assert_eq!(
        (
            priority(0x11),
            priority(0x0f),
            priority(6),
            priority(0x17),
            priority(9),
            priority(7)
        ),
        (6, 5, 4, 3, 2, 1)
    );
    let mut q = Queue::default();
    let mut a = item(7, 3, 10);
    a.data[14] = 1;
    a.data[15] = 2;
    assert_eq!(enqueue(&mut q, a, false, 3).accepted, 1);
    let mut b = item(7, 3, 90);
    b.data[14] = 32;
    b.data[15] = 64;
    assert_eq!(enqueue(&mut q, b, false, 3).replaced, 1);
    assert_eq!(q.count, 1);
    assert_eq!(q.items[0].enqueued_us, 10);
    assert_eq!((q.items[0].data[14], q.items[0].data[15]), (33, 66));
    b.dest_mac[0] = 1;
    enqueue(&mut q, b, false, 3);
    assert_eq!(q.count, 2);
}
#[test]
fn victims_equal_rules_wrap_reset_and_forward_age() {
    let mut q = Queue {
        next_order: u32::MAX,
        ..Queue::default()
    };
    for n in 0..12 {
        let mut i = item(8, 3, n);
        i.dest_mac[0] = n as u8;
        enqueue(&mut q, i, false, 3);
    }
    assert_eq!((q.items[0].order, q.items[1].order), (u32::MAX, 0));
    assert_eq!(enqueue(&mut q, item(1, 3, 0), false, 3).accepted, 0);
    let r = enqueue(&mut q, item(9, 3, 0), false, 3);
    assert_eq!(r.have_evicted, 1);
    assert!(!q.items.iter().any(|i| i.kind == 8 && i.dest_mac[0] == 1));
    assert_eq!(q.items[0].order, u32::MAX);
    assert_eq!(enqueue(&mut q, item(9, 4, 0), true, 3).accepted, 1);
    let order = q.next_order;
    reset(&mut q);
    assert_eq!(q.next_order, order);
    assert_eq!(q.count, 0);
    let mut a = item(0x17, 4, 10);
    a.data[5] = 2;
    enqueue(&mut q, a, true, 3);
    a.enqueued_us = 90;
    assert_eq!(enqueue(&mut q, a, true, 3).replaced, 1);
    assert_eq!(q.items[0].enqueued_us, 90);
}
#[test]
fn eviction_credit_matches_exact_restore_predicate_and_zero_credit() {
    for forward in [false, true] {
        for (kind, wire_kind, src, flags, len, heard, expected) in [
            (7, 7, 3, 0, 17, 33, 33),
            (7, 99, 3, 0, 17, 33, 33), // C restore does not inspect wire kind.
            (7, 7, 6, 0, 17, 33, 0),
            (7, 7, 3, 2, 17, 33, 0),
            (7, 7, 3, 0, 16, 33, 0),
            (8, 7, 3, 0, 17, 33, 0),
            (7, 7, 3, 0, 17, 0, 0),
        ] {
            let mut q = Queue::default();
            // Distinct destinations avoid replacement; entry zero is the victim.
            for n in 0..12 {
                let mut a = item(8, 3, n);
                a.dest_mac[0] = n as u8;
                enqueue(&mut q, a, false, 3);
            }
            let victim = &mut q.items[0];
            victim.kind = kind;
            victim.data[1] = wire_kind;
            victim.data[2] = src;
            victim.data[5] = flags;
            victim.len = len;
            victim.data[14] = heard;
            victim.data[15] = heard;
            let r = enqueue(&mut q, item(0x0f, 3, 99), forward, 3);
            assert_eq!((r.accepted, r.have_evicted, r.drops), (1, 1, 1));
            assert_eq!((r.local_heard, r.local_relay), (expected, expected));
            assert!(!q.items.iter().any(|i| i.kind != 0x0f && i.dest_mac[0] == 0));
        }
    }
}

#[test]
fn sync_override_and_priority_not_fairness() {
    for (now, critical, expected) in [
        (549000, false, 0x17),
        (550000, false, 0x11),
        (599000, true, 0x0f),
        (600000, true, 0x11),
    ] {
        let mut q = Queue::default();
        let mut sync = item(0x11, 4, 0);
        sync.data[5] = 2;
        enqueue(&mut q, sync, true, 3);
        enqueue(
            &mut q,
            item(if critical { 0x0f } else { 0x17 }, 3, now),
            false,
            3,
        );
        assert_eq!(dequeue(&mut q, now + 1000, 1).unwrap().kind, expected);
    }
    let mut q = Queue::default();
    enqueue(&mut q, item(7, 3, 0), false, 3);
    enqueue(&mut q, item(9, 3, 500000), false, 3);
    assert_eq!(dequeue(&mut q, 500000, 500).unwrap().kind, 9);
    assert!(dequeue(&mut Queue::default(), 0, 0).is_none());
}
#[test]
fn scrub_and_reuse_keep_survivor_and_order() {
    let mut q = Queue::default();
    let mut a = item(7, 3, 0);
    a.data[14] = 33;
    a.data[15] = 33;
    enqueue(&mut q, a, false, 3);
    let release = item(10, 3, 0);
    enqueue(&mut q, release, false, 3);
    q.items[1].data[10] = 1;
    remove_member(&mut q, 1, [1; 6], 3, 1, 2, 1);
    assert_eq!(q.count, 1);
    assert_eq!((q.items[0].data[14], q.items[0].data[15]), (32, 32));
    assert_eq!((q.items[0].data[11], q.items[0].data[16]), (2, 1));
    let mut r = item(10, 3, 0);
    r.data[10] = 1;
    enqueue(&mut q, r, false, 3);
    before_reuse(&mut q, 1);
    assert_eq!(q.count, 1);
}
