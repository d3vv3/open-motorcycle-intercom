use openhelmet_wire::bridge::{self, DecodeError};

const LIMITS: [usize; 5] = [0, 208, 251, 253, 254];

fn next(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

#[test]
fn explicit_contracts_and_precedence() {
    assert_eq!(bridge::MAX_PAYLOAD, 253);
    assert_eq!(bridge::OVERHEAD, 5);
    assert_eq!(bridge::MAX_SIZE, 258);
    assert_eq!(bridge::NRF_SPI_MAX_PAYLOAD, 208);
    assert_eq!(bridge::ESP_SPI_MAX_PAYLOAD, 251);
    assert_eq!(bridge::crc8(b"123456789"), 0xf4);
    assert_eq!(bridge::crc8(&[]), 0);
    let cases: &[(&[u8], usize, DecodeError)] = &[
        (&[], 254, DecodeError::BadLength),
        (&[0], 254, DecodeError::BadLength),
        (&[], 253, DecodeError::Truncated),
        (&[0], 0, DecodeError::Idle),
        (&[0, 0, 0, 0, 0], 0, DecodeError::Idle),
        (&[0x55, 0, 0, 0], 0, DecodeError::Truncated),
        (&[0x55, 0, 0, 0, 0], 0, DecodeError::BadSync),
        (&[0xaa, 0, 0, 0, 0], 253, DecodeError::BadLength),
        (&[0xaa, 1, 0, 0, 0], 253, DecodeError::BadLength),
        (&[0xaa, 255, 0, 0, 0], 208, DecodeError::BadLength),
        (&[0xaa, 255, 0, 0, 0], 253, DecodeError::Truncated),
        (&[0xaa, 2, 0, 0, 0], 0, DecodeError::BadCrc),
    ];
    for &(frame, max, error) in cases {
        assert_eq!(bridge::decode(frame, max), Err(error));
    }
    let mut dst = [0x5a; 512];
    assert_eq!(bridge::encode(&mut dst, usize::MAX, 0, 0, &[]), 0);
    assert_eq!(bridge::encode(&mut dst, 253, 0, 0, &[0; 254]), 0);
    assert_eq!(dst, [0x5a; 512]);
    assert_eq!(bridge::decode(&[], usize::MAX), Err(DecodeError::BadLength));
}

#[test]
fn golden_frames_and_crc_vectors() {
    // Independently computed CRC-8/ATM (poly 0x07, init 0) vectors.
    for (data, crc) in [
        (&b""[..], 0x00),
        (b"\x00", 0x00),
        (b"\x01", 0x07),
        (b"\xff", 0xf3),
        (b"123456789", 0xf4),
        (b"\x02\x05\x06", 0x85),
    ] {
        assert_eq!(bridge::crc8(data), crc, "{data:?}");
    }
    let mut dst = [0x5a; 16];
    assert_eq!(bridge::encode(&mut dst, 253, 0x05, 0x06, &[]), 5);
    assert_eq!(&dst[..6], &[0xaa, 2, 5, 6, 0x85, 0x5a]);
    assert_eq!(bridge::encode(&mut dst, 253, 0x11, 0x22, b"ab"), 7);
    assert_eq!(
        &dst[..7],
        &[
            0xaa,
            4,
            0x11,
            0x22,
            b'a',
            b'b',
            bridge::crc8(&[4, 0x11, 0x22, b'a', b'b'])
        ]
    );
    let view = bridge::decode(&dst, 253).unwrap();
    assert_eq!(
        (view.seq, view.frame_type, view.payload),
        (0x11, 0x22, &b"ab"[..])
    );
    assert_eq!(view.payload.as_ptr(), dst[4..].as_ptr());
}

#[test]
fn every_payload_length_round_trips_and_truncations_corruptions_fail() {
    for len in 0..=254usize {
        let payload: Vec<u8> = (0..len).map(|i| i as u8).collect();
        for max in LIMITS {
            for capacity in [0, 4, len + 4, len + 5, 272] {
                let mut dst = vec![0x5a; capacity];
                let encoded = bridge::encode(&mut dst, max, len as u8, !(len as u8), &payload);
                let valid = max <= 253 && len <= max && capacity >= len + 5;
                assert_eq!(
                    encoded,
                    if valid { len + 5 } else { 0 },
                    "{len} {max} {capacity}"
                );
                assert!(dst[encoded..].iter().all(|&b| b == 0x5a));
            }
        }
        if len > 253 {
            continue;
        }
        let mut frame = vec![0xce; len + 5 + 16];
        let encoded = bridge::encode(&mut frame, 253, len as u8, !(len as u8), &payload);
        assert_eq!(encoded, len + 5);
        for max in LIMITS {
            let want = if max > 253 || len > max {
                Err(DecodeError::BadLength)
            } else {
                Ok(bridge::FrameView {
                    payload: &payload,
                    seq: len as u8,
                    frame_type: !(len as u8),
                })
            };
            // Trailing padding (nonzero) is ignored.
            assert_eq!(bridge::decode(&frame[..encoded], max), want);
            assert_eq!(bridge::decode(&frame, max), want);
        }
        for truncated in 1..encoded {
            assert_eq!(
                bridge::decode(&frame[..truncated], 253),
                Err(DecodeError::Truncated)
            );
        }
        assert_eq!(
            bridge::decode(&frame[..0], 253),
            Err(DecodeError::Truncated)
        );
        for byte in 1..encoded {
            let mut corrupted = frame.clone();
            corrupted[byte] ^= 1;
            let result = bridge::decode(&corrupted, 253);
            // A flipped length byte may still describe a shorter/longer frame;
            // any other single-bit flip is caught by the CRC.
            if byte == 1 {
                assert!(result != Ok(bridge::decode(&frame, 253).unwrap()), "{len}");
            } else {
                assert_eq!(result, Err(DecodeError::BadCrc), "{len} {byte}");
            }
        }
    }
}

#[test]
fn sequence_type_sync_and_length_domains_are_exhaustive() {
    for seq in 0..=255u8 {
        for kind in 0..=255u8 {
            let mut frame = [0xce; 8];
            assert_eq!(bridge::encode(&mut frame, 0, seq, kind, &[]), 5);
            let view = bridge::decode(&frame, 0).unwrap();
            assert_eq!(
                (view.seq, view.frame_type, view.payload.len()),
                (seq, kind, 0)
            );
        }
    }
    for sync in 0..=255u8 {
        for wire_len in 0..=255u8 {
            let frame = [sync, wire_len, 255, 0, 0];
            for max in LIMITS {
                let want = if max > 253 {
                    DecodeError::BadLength
                } else if sync == 0 {
                    DecodeError::Idle
                } else if sync != 0xaa {
                    DecodeError::BadSync
                } else if wire_len < 2 || usize::from(wire_len) - 2 > max {
                    DecodeError::BadLength
                } else if wire_len > 2 {
                    DecodeError::Truncated
                } else {
                    DecodeError::BadCrc
                };
                assert_eq!(
                    bridge::decode(&frame, max),
                    Err(want),
                    "{sync} {wire_len} {max}"
                );
            }
        }
        for len in 0..5 {
            let frame = [sync; 5];
            for max in LIMITS {
                let want = if max > 253 {
                    DecodeError::BadLength
                } else if len == 0 {
                    DecodeError::Truncated
                } else if sync == 0 {
                    DecodeError::Idle
                } else {
                    DecodeError::Truncated
                };
                assert_eq!(bridge::decode(&frame[..len], max), Err(want));
            }
        }
    }
}

#[test]
fn random_inputs_never_panic_and_random_frames_round_trip() {
    let mut state = 0x6d2b79f5;
    for iteration in 0..20_000 {
        let len = (next(&mut state) % 274) as usize;
        let max = (next(&mut state) % 256) as usize;
        let mut bytes: Vec<u8> = (0..len).map(|_| next(&mut state) as u8).collect();
        if !bytes.is_empty() {
            match iteration % 4 {
                0 => bytes[0] = 0,
                1 => bytes[0] = bridge::SYNC_BYTE,
                _ => {}
            }
        }
        if let Ok(view) = bridge::decode(&bytes, max) {
            assert!(view.payload.len() <= max && max <= 253);
            assert_eq!(view.payload.as_ptr(), bytes[4..].as_ptr());
        }
        let _ = bridge::crc8(&bytes);
        let capacity = (next(&mut state) % 274) as usize;
        let mut dst = vec![0x5a; capacity];
        let encoded = bridge::encode(
            &mut dst,
            max,
            next(&mut state) as u8,
            next(&mut state) as u8,
            &bytes,
        );
        assert_eq!(
            encoded != 0,
            max <= 253 && len <= max && capacity >= len + 5
        );

        let payload_len = (next(&mut state) % 254) as usize;
        let payload: Vec<u8> = (0..payload_len).map(|_| next(&mut state) as u8).collect();
        let mut frame = vec![0; payload_len + 5 + (next(&mut state) % 17) as usize];
        for byte in &mut frame {
            *byte = next(&mut state) as u8;
        }
        let seq = next(&mut state) as u8;
        let kind = next(&mut state) as u8;
        let encoded = bridge::encode(&mut frame, 253, seq, kind, &payload);
        assert_eq!(encoded, payload_len + 5);
        let view = bridge::decode(&frame, payload_len).unwrap();
        assert_eq!(
            (view.seq, view.frame_type, view.payload),
            (seq, kind, &payload[..])
        );
        let cut = next(&mut state) as usize % encoded;
        assert!(bridge::decode(&frame[..cut], 253).is_err());
        frame[encoded - 1] ^= 1;
        assert_eq!(bridge::decode(&frame, 253), Err(DecodeError::BadCrc));
    }
}
