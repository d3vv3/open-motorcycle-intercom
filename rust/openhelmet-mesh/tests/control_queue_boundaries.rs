use openhelmet_mesh::control_queue::*;
fn packet(kind: u8, src: u8) -> Item {
    let mut data = [0; 209];
    data[1] = kind;
    data[2] = src;
    Item {
        data,
        kind,
        len: 209,
        ..Item::default()
    }
}
#[test]
fn producer_priority_ignores_supplied_class_for_every_type() {
    for kind in 0..=255 {
        let expected = match kind {
            17 => 6,
            15 | 16 | 18..=21 => 5,
            4 | 6 | 22 => 4,
            23 => 3,
            9 | 10 | 14 => 2,
            _ => 1,
        };
        for forward in [false, true] {
            let mut q = Queue::new();
            let mut item = packet(kind, 2);
            item.priority = 255;
            let result = enqueue(&mut q, item, forward, 1);
            if kind == 17 && !forward {
                assert_eq!(result.accepted, 0);
            } else {
                assert_eq!(result.accepted, 1);
                assert_eq!(q.items[0].priority, if kind == 17 { 2 } else { expected });
            }
        }
    }
}
#[test]
fn cleanup_every_reference_class_and_survivor() {
    for (kind, offset, value) in [
        (6, 10, 1),
        (6, 19, 1),
        (6, 21, 1),
        (0x16, 19, 1),
        (0x16, 28, 1),
        (0x16, 30, 1),
        (14, 43, 1),
        (14, 52, 1),
        (14, 54, 1),
        (0x12, 19, 1),
        (0x14, 19, 1),
        (0x15, 19, 1),
        (0x10, 17, 1),
        (0x10, 10, 1),
        (10, 10, 1),
        (10, 11, 1),
        (9, 10, 1),
        (9, 12, 1),
    ] {
        let mut q = Queue::default();
        let mut p = packet(kind, 6);
        p.data[offset] = value;
        if kind == 0x10 && offset == 10 {
            p.data[10..16].fill(1);
        }
        enqueue(&mut q, p, false, 3);
        enqueue(&mut q, packet(1, 6), false, 3);
        remove_member(&mut q, 1, [1; 6], 3, 1, 2, 0);
        assert_eq!(q.count, 1, "{kind:x} offset {offset}");
        assert_eq!(q.items[0].kind, 1);
    }
    for mut p in [packet(0x13, 6), packet(0x11, 3), packet(1, 1), packet(1, 6)] {
        let forward = p.kind == 0x11;
        if p.kind == 1 && p.data[2] == 6 {
            p.dest_mac = [1; 6];
        }
        let mut q = Queue::default();
        enqueue(&mut q, p, forward, 3);
        remove_member(&mut q, 1, [1; 6], 3, 1, 2, 0);
        assert_eq!(q.count, 0);
    }
}
#[test]
fn sync_coalescing_exact_identity_age_and_wire() {
    let mut q = Queue::default();
    let mut p = packet(0x11, 6);
    p.len = 29;
    p.data[5] = 2;
    p.data[18] = 7;
    p.data[16] = 4;
    enqueue(&mut q, p, true, 3);
    let order = q.items[0].order;
    let mut incoming = p;
    incoming.enqueued_us = 123;
    incoming.data[3] = 55;
    incoming.data[5] = 0;
    incoming.data[18] = 8;
    assert!(!coalesce_sync(&mut q, &incoming));
    incoming.data[18] = 7;
    incoming.data[16] = 5;
    assert!(!coalesce_sync(&mut q, &incoming));
    incoming.data[16] = 4;
    incoming.len = 28;
    assert!(!coalesce_sync(&mut q, &incoming));
    incoming.len = 29;
    assert!(coalesce_sync(&mut q, &incoming));
    assert_eq!(q.items[0].order, order);
    assert_eq!(q.items[0].enqueued_us, 123);
    assert_eq!(
        (q.items[0].data[3], q.items[0].data[4], q.items[0].data[5]),
        (55, 1, 2)
    );
}
#[test]
fn local_forward_request_separation_and_both_victim_ties() {
    let mut q = Queue::default();
    let mut p = packet(0x17, 6);
    p.data[5] = 2;
    enqueue(&mut q, p, true, 3);
    let local = packet(0x17, 3);
    enqueue(&mut q, local, false, 3);
    assert_eq!(q.count, 2);
    p.dest_mac = [9; 6];
    p.enqueued_us = 99;
    enqueue(&mut q, p, true, 3);
    assert_eq!(q.count, 2);
    assert_eq!(q.items[0].enqueued_us, 99);
    for forward in [false, true] {
        reset(&mut q);
        for n in 0..12 {
            let mut a = packet(9, 6);
            a.dest_mac[0] = n;
            enqueue(&mut q, a, false, 3);
        }
        // GRANT replaces by destination, so these are twelve distinct entries.
        q.items[0].order = 99;
        q.items[1].order = 0;
        let mut incoming = packet(0x17, 3);
        incoming.dest_mac = [99; 6];
        let r = enqueue(&mut q, incoming, forward, 3);
        assert_eq!(r.accepted, 1);
        assert_eq!(r.have_evicted, 1);
        assert!(!q.items.iter().any(|i| i.dest_mac[0] == u8::from(!forward)));
        assert_eq!(
            q.items[if forward { 1 } else { 0 }].order,
            if forward { 0 } else { 99 }
        );
        assert_eq!(
            q.items[if forward { 0 } else { 1 }].dest_mac[0],
            11,
            "swap-last"
        );
        let mut equal = packet(9, 4);
        equal.dest_mac = [98; 6];
        assert_eq!(
            enqueue(&mut q, equal, forward, 3).accepted,
            u8::from(!forward)
        );
    }
}
#[test]
fn raw_order_overdue_boundary_and_wrapped_sync_clock() {
    let mut q = Queue::default();
    let mut a = packet(1, 6);
    a.enqueued_us = 1;
    a.order = 0;
    enqueue(&mut q, a, false, 3);
    enqueue(&mut q, a, false, 3);
    q.items[0].enqueued_us = 1;
    q.items[0].order = 0;
    q.items[1].enqueued_us = 0;
    q.items[1].order = u32::MAX;
    assert_eq!(dequeue(&mut q, 500000, 500).unwrap().order, u32::MAX);
    for last in [0, u32::MAX - 549] {
        reset(&mut q);
        let mut sync = packet(0x11, 6);
        sync.data[5] = 2;
        enqueue(&mut q, sync, true, 3);
        enqueue(&mut q, packet(0x17, 3), false, 3);
        assert_eq!(dequeue(&mut q, 0, last).unwrap().kind, 0x11);
    }
    let mut broken = Queue {
        count: 13,
        ..Queue::default()
    };
    let before = broken.next_order;
    assert!(dequeue(&mut broken, 0, 0).is_none());
    assert_eq!(enqueue(&mut broken, packet(1, 6), false, 3).accepted, 0);
    assert_eq!(broken.next_order, before);
}
