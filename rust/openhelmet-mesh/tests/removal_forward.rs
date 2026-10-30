use openhelmet_mesh::adaptive::{self as a, Adaptive, ForwardPacket, Header, Identity};
#[test]
fn removal_preserves_live_forward_fifo_and_survivor_dedupe() {
    let mut s = Adaptive::default();
    a::init(Some(&mut s), 2, 3, 7, 39, 0);
    for id in [1, 2, 3, 6] {
        assert!(a::set_member(
            Some(&mut s),
            id,
            Some(&Identity {
                address_len: 6,
                address: [id; 6]
            })
        ));
    }
    let enqueue = |s: &mut Adaptive, src, seq| {
        a::forward_enqueue(
            Some(s),
            Some(&Header {
                version: 5,
                packet_type: 14,
                src_id: src,
                seq,
                ttl: 2,
                payload_len: 47,
                ..Header::default()
            }),
            Some(&[0; 47]),
            None,
            7,
            true,
            100,
        )
    };
    // Advance the physical head across the ring boundary without growing it.
    let mut discarded = ForwardPacket::default();
    for seq in 10..17 {
        assert!(enqueue(&mut s, 6, seq));
        assert!(a::forward_dequeue(
            Some(&mut s),
            Some(&mut discarded),
            0,
            0,
            0
        ));
    }
    assert!(enqueue(&mut s, 6, 1));
    assert!(enqueue(&mut s, 1, 2));
    assert!(enqueue(&mut s, 6, 3));
    assert!(a::set_member(Some(&mut s), 1, None));
    assert_eq!(s.forward_queue_len(), 2);
    assert!(!enqueue(&mut s, 6, 1), "survivor cache retained");
    let mut out = ForwardPacket::default();
    for seq in [1, 3] {
        assert!(a::forward_dequeue(Some(&mut s), Some(&mut out), 0, 0, 0));
        assert_eq!((out.header.src_id, out.header.seq), (6, seq));
    }
    assert!(a::set_member(
        Some(&mut s),
        1,
        Some(&Identity {
            address_len: 6,
            address: [8; 6]
        })
    ));
    assert!(enqueue(&mut s, 1, 2), "removed binding cache invalidated");
}
