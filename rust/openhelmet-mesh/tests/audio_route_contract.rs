use openhelmet_mesh::audio_route::{self as audio, Ring};

#[test]
fn physical_ring_fifo_wrap_drop_oldest_and_age() {
    let mut ring = Ring::default();
    let mut packet = [0u8; 79];
    packet[2] = 3;
    for seq in 0..35 {
        packet[3] = seq;
        assert_eq!(
            audio::enqueue(&mut ring, &packet, 1, 7, 100),
            if seq < 15 { 1 } else { 2 }
        );
    }
    assert_eq!((ring.head + 16 - ring.tail) % 16, 15);
    assert_eq!(ring.entries[ring.tail as usize].data[3], 20);
    assert_eq!(audio::expire(&mut ring, 20100), 0);
    assert_eq!(audio::expire(&mut ring, 20101), 15);
    assert_eq!(ring.head, ring.tail);
    packet[3] = 99;
    assert_eq!(audio::enqueue(&mut ring, &packet, 1, 5, i64::MAX - 99), 1);
    assert_eq!(audio::expire(&mut ring, i64::MIN + 100), 0);
    assert_eq!(audio::expire(&mut ring, i64::MIN + 20000), 1);
    assert_eq!(audio::enqueue(&mut ring, &packet, 0, 0, 0), 0);
    assert_eq!(audio::enqueue(&mut ring, &[0; 8], 1, 0, 0), 0);
    assert_eq!(audio::enqueue(&mut ring, &[0; 80], 1, 0, 0), 0);
}

#[test]
fn removal_stable_and_failure_does_not_consume() {
    let mut ring = Ring::default();
    for seq in 0..30 {
        let mut p = [0; 9];
        p[2] = if seq % 2 == 0 { 2 } else { 3 };
        p[3] = seq;
        audio::enqueue(&mut ring, &p, 1, 0, seq as i64);
    }
    audio::remove(&mut ring, 2);
    let mut turn = false;
    assert_eq!(audio::select(true, true, turn), 1);
    turn = audio::attempt_turn(1, true);
    assert!(turn);
    assert_eq!(audio::select(true, true, turn), 2);
    assert!(!audio::attempt_turn(2, true));
    assert_eq!(audio::select(false, true, false), 2);
    assert_eq!(audio::select(false, false, true), 0);
    let tail = ring.tail;
    audio::complete(&mut ring, false);
    assert_eq!(ring.tail, tail);
    for expected in (15..30).step_by(2) {
        let e = &ring.entries[ring.tail as usize];
        assert_eq!(
            (e.data[2], e.data[3], e.enqueued_us),
            (3, expected, expected as i64)
        );
        audio::complete(&mut ring, true);
    }
    assert_eq!(ring.head, ring.tail);
}

#[test]
fn reset_clears_all_slots_in_place_and_detached_peek_is_checked() {
    use audio::Packet;
    let mut ring = Ring::new();
    let address = &ring as *const Ring;
    let mut out = Packet {
        data: [0xa5; 79],
        len: 0xa5a5,
    };
    assert!(!audio::peek(&ring, &mut out));
    assert_eq!((out.data, out.len), ([0xa5; 79], 0xa5a5));
    for seq in 0..35 {
        let mut packet = [0; 79];
        packet[3] = seq;
        audio::enqueue(&mut ring, &packet, 1, 7, seq as i64);
    }
    assert!(audio::peek(&ring, &mut out));
    assert_eq!(out.data[3], 20);
    out.data.fill(0xff);
    assert_eq!(ring.entries[ring.tail as usize].data[3], 20);
    ring.entries[ring.tail as usize].len = 80;
    assert!(!audio::peek(&ring, &mut out));
    audio::reset(&mut ring);
    assert_eq!(&ring as *const Ring, address);
    assert!(audio::empty(&ring));
    assert_eq!((ring.head, ring.tail), (0, 0));
    assert!(
        ring.entries
            .iter()
            .all(|e| e.data == [0; 79] && e.len == 0 && e.enqueued_us == 0)
    );
}

#[test]
fn authenticated_route_grant_and_privacy_are_separate() {
    for len in 0..=80 {
        assert_eq!(audio::payload_valid(1, 20, len, 1), (1..=64).contains(&len));
        assert_eq!(audio::payload_valid(2, 20, len, 2), len == 48);
        assert_eq!(
            audio::producer_size_valid(len, false),
            (1..=64).contains(&len)
        );
        assert_eq!(audio::producer_size_valid(len, true), len == 48);
    }
    assert!(!audio::payload_valid(1, 10, 3, 1));
    assert!(!audio::payload_valid(1, 20, 3, 2));
    assert!(audio::pair_eligible(true, 1, 1, 3, 1, true));
    assert!(audio::pair_eligible(true, 2, 1, 3, 1, true));
    for (active, role, id, members, remotes, valid) in [
        (false, 1, 1, 3, 1, true),
        (true, 0, 1, 3, 1, true),
        (true, 1, 0, 3, 1, true),
        (true, 1, 1, 7, 1, true),
        (true, 1, 1, 3, 2, true),
        (true, 1, 1, 3, 1, false),
    ] {
        assert!(!audio::pair_eligible(
            active, role, id, members, remotes, valid
        ));
    }
    // 1 = authenticated direct, 2 = authenticated relay, 0 = reject.
    assert_eq!(audio::authenticate(2, 1, true, true, false), 1);
    assert_eq!(audio::authenticate(1, 0, true, false, true), 2);
    for ttl in [0, 1, 3, 255] {
        assert_eq!(audio::authenticate(ttl, 1, true, true, true), 0);
    }
    assert_eq!(audio::authenticate(1, 0, true, true, false), 0);
    assert_eq!(audio::authenticate(2, 1, false, true, true), 0);
    // Flags: RELAY_REQUEST=1, RELAYED=2, SPEAKER_GRANTED=4 (wire constants).
    assert!(audio::relay_allowed(2, 5, 3, 2, [3, 0], [2, 0]));
    assert!(!audio::relay_allowed(2, 1, 3, 2, [3, 0], [2, 0]));
    assert!(!audio::relay_allowed(2, 5, 3, 2, [3, 0], [4, 0]));
    assert!(!audio::relay_allowed(1, 5, 3, 2, [3, 0], [2, 0]));
}
