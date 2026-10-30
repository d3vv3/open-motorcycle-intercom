use openhelmet_mesh::status::*;

#[test]
fn evidence_is_lossless_and_id_bounded() {
    for id in 0..=255 {
        let bit = if (1..=8).contains(&id) {
            1u8 << (id - 1)
        } else {
            0
        };
        assert_eq!(heard(0, id, true), u16::from(bit));
        assert_eq!(heard(0, id, false), 0);
        assert_eq!(relay(0, id), u16::from(bit) << 8);
        assert_eq!(scrub(0xffff, id), !((u16::from(bit) << 8) | u16::from(bit)));
    }
    let snapshot = 0x1281;
    // Snapshot clears authoritative storage; activity and failed-send restore
    // commute in either speaker-lock winner order, never overwrite new bits.
    assert_eq!(restore(heard(0, 3, true), snapshot), 0x1285);
    assert_eq!(heard(restore(0, snapshot), 3, true), 0x1285);
}

#[test]
fn literal_wire_and_exact_decode() {
    assert_eq!(keepalive(), [100, 0]);
    assert_eq!(
        build(11, 3, 0x81, 0xfe, 2),
        [100, 127, 11, 3, 127, 0x81, 0xfe, 2]
    );
    let bytes = [255, 23, 19, 71, 0x80, 0xfe, 0xab, 99];
    let p = decode_status(&bytes).unwrap();
    assert_eq!(
        p,
        Projection {
            battery: 255,
            peers: 19,
            firmware: 71,
            temperature: -128,
            heard: 0xfe,
            relayed: 0xab
        }
    );
    assert_eq!(decode_status(&bytes[..7]), None);
    assert_eq!(decode_status(&[0; 9]), None);
    assert_eq!(decode_keepalive(&[222, 199]), Some(222));
    assert_eq!(decode_keepalive(&[222]), None);
    assert_eq!(decode_keepalive(&[222, 0, 0]), None);
}

#[test]
fn detached_credit_uses_item_kind_not_embedded_wire_type() {
    for kind in 0..=255 {
        for len in [0, 16, 17, 18, 255] {
            for (source, local, flags) in [(3, 3, 0), (2, 3, 0), (3, 3, 2), (0, 0, 0)] {
                let expected = if kind == 7 && len == 17 && source == local && flags & 2 == 0 {
                    0x7281
                } else {
                    0
                };
                assert_eq!(
                    local_credit(kind, len, source, local, flags, 0x81, 0x72),
                    expected
                );
            }
        }
    }
}
