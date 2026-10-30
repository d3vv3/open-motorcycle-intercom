//! Pure Rust queue contracts, not independent C-adapter differential evidence.
use openhelmet_mesh::adaptive::*;

fn enqueue(s: &mut Adaptive, seq: u8, packet_type: u8, now: u32) -> bool {
    let len = match packet_type {
        14 => 47,
        9 => 5,
        10 => 3,
        _ => unreachable!(),
    };
    let header = Header {
        version: 5,
        packet_type,
        src_id: 3,
        seq,
        ttl: 2,
        talk_channel: seq,
        payload_len: len,
        ..Header::default()
    };
    forward_enqueue(
        Some(s),
        Some(&header),
        Some(&[seq; 47][..usize::from(len)]),
        None,
        7,
        true,
        now,
    )
}

fn pop(s: &mut Adaptive) -> ForwardPacket {
    let mut out = ForwardPacket::default();
    assert!(forward_dequeue(Some(s), Some(&mut out), 0, 0, 0));
    out
}

fn fixture(head: usize, count: usize) -> Adaptive {
    let mut s = Adaptive::default();
    init(Some(&mut s), 2, 1, 7, 7, u32::MAX - 20);
    let identity = Identity {
        address_len: 1,
        address: [3, 0, 0, 0, 0, 0],
    };
    assert!(set_member(Some(&mut s), 3, Some(&identity)));
    assert!(speaker_request(
        Some(&mut s),
        3,
        Some(&SpeakerRequest {
            term: 7,
            request_seq: u32::MAX,
            frame_counter: u32::MAX,
            active: 1,
        }),
        0,
        u32::MAX - 10,
    ));
    // Wrap both cursors and overwrite cache rows; leave long stale payload tails.
    for seq in 0..40 + head {
        assert!(enqueue(&mut s, seq as u8, 14, seq as u32));
        pop(&mut s);
    }
    for offset in 0..count {
        assert!(enqueue(
            &mut s,
            100 + offset as u8,
            [9, 10, 14][offset % 3],
            (u32::MAX - 3).wrapping_add(offset as u32),
        ));
    }
    assert_eq!(usize::from(s.snapshot().queue_head), head);
    s
}

#[test]
fn inspection_and_clear_preserve_complete_state_except_cursors() {
    for head in 0..FORWARD_QUEUE {
        for count in 0..=FORWARD_QUEUE {
            let mut s = fixture(head, count);
            let before = s.snapshot();
            assert_eq!(s.forward_queue_len(), count);
            for offset in 0..count {
                assert_eq!(
                    s.forward_queue_entry(offset),
                    Some(&before.queue[(head + offset) % FORWARD_QUEUE]),
                );
            }
            for offset in [count, usize::MAX] {
                assert_eq!(s.forward_queue_entry(offset), None);
            }
            for packet_type in 0..=u8::MAX {
                let expected = (0..count).any(|offset| {
                    before.queue[(head + offset) % FORWARD_QUEUE]
                        .header
                        .packet_type
                        == packet_type
                });
                assert_eq!(s.forward_queue_contains_type(packet_type), expected);
            }
            assert_eq!(s.snapshot(), before);
            let mut expected = before;
            expected.queue_head = 0;
            expected.queue_count = 0;
            s.clear_forward_queue();
            assert_eq!(s.snapshot(), expected);
            assert_eq!(s.forward_queue_len(), 0);
            assert_eq!(s.forward_queue_entry(0), None);
            assert!(!s.forward_queue_contains_type(14));
            s.clear_forward_queue();
            assert_eq!(s.snapshot(), expected);
            // A cleared/lost packet remains deduplicated while its cache key is fresh.
            if count != 0 {
                assert!(!enqueue(&mut s, 100, 9, 4));
                assert_eq!(s.snapshot(), expected);
            }
        }
    }
}

#[test]
fn promotion_is_stable_for_every_cursor_count_and_offset() {
    for head in 0..FORWARD_QUEUE {
        for count in 0..=FORWARD_QUEUE {
            let s = fixture(head, count);
            let before = s.snapshot();
            for offset in [count, usize::MAX] {
                let mut actual = s.clone();
                assert!(!actual.promote_forward_queue(offset));
                assert_eq!(actual.snapshot(), before);
            }
            for selected in 0..count {
                let mut actual = s.clone();
                let mut expected = before.clone();
                let mut logical: Vec<_> = (0..count)
                    .map(|offset| before.queue[(head + offset) % FORWARD_QUEUE])
                    .collect();
                logical[..=selected].rotate_right(1);
                for (offset, entry) in logical.iter().enumerate() {
                    expected.queue[(head + offset) % FORWARD_QUEUE] = *entry;
                }
                assert!(actual.promote_forward_queue(selected));
                assert_eq!(actual.snapshot(), expected);
                // Promoting logical head is a successful complete-state no-op.
                assert!(actual.promote_forward_queue(0));
                assert_eq!(actual.snapshot(), expected);
                for entry in logical {
                    assert_eq!(pop(&mut actual), entry);
                }
            }
        }
    }
}

#[test]
fn promotion_and_clear_do_not_require_a_resident_cache_key() {
    let mut s = fixture(7, FORWARD_QUEUE);
    // Keep the oldest packet queued while later packets displace its cache key.
    for seq in 150..190 {
        assert!(s.promote_forward_queue(FORWARD_QUEUE - 1));
        pop(&mut s);
        assert!(enqueue(&mut s, seq, 10, u32::from(seq)));
    }
    let before = s.snapshot();
    let selected = *s.forward_queue_entry(1).unwrap();
    assert_eq!(selected.header.seq, 101);
    assert!(!before.cache.iter().any(|key| {
        key.used && key.seq == selected.header.seq && key.packet_type == selected.header.packet_type
    }));
    let mut expected = before;
    expected.queue.swap(7, 0);
    assert!(s.promote_forward_queue(1));
    assert_eq!(s.snapshot(), expected);
    expected.queue_head = 0;
    expected.queue_count = 0;
    s.clear_forward_queue();
    assert_eq!(s.snapshot(), expected);
}
