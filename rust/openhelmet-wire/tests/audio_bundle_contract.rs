use openhelmet_wire::audio_bundle::*;
use openhelmet_wire::mesh::{Message, Transport};

// Protocol fixture: LC3/20 ms, stream 0xdb, sequence 0x1234, newest active,
// two active predecessors, relayed; payload order is oldest to newest.
fn golden() -> Vec<u8> {
    let mut bytes = vec![2, 20, 0xdb, 0x3f, 0x12, 0x34, 48, 48];
    bytes.extend([0x22; 48]);
    bytes.extend([0x11; 48]);
    bytes.extend([0xcc; 48]);
    bytes
}

#[test]
fn golden_wire_bytes_and_borrowed_frame_order() {
    let bytes = golden();
    let view = audio_bundle_parse(&bytes).unwrap();
    assert_eq!(
        (view.codec, view.stream_id, view.current_seq, view.flags),
        (2, 0xdb, 0x1234, 0x3f)
    );
    assert_eq!(view.previous2_data, [0x22; 48]);
    assert_eq!(view.previous1_data, [0x11; 48]);
    assert_eq!(view.current_data, [0xcc; 48]);
    for (frame, offset) in [
        (view.previous2_data, 8),
        (view.previous1_data, 56),
        (view.current_data, 104),
    ] {
        assert_eq!(frame.as_ptr(), bytes[offset..].as_ptr());
    }
    let input = BundleView {
        previous2_data: &[0x22; 48],
        previous1_data: &[0x11; 48],
        current_data: &[0xcc; 48],
        current_seq: 0x1234,
        stream_id: 0xdb,
        flags: 0x3f,
        codec: 2,
    };
    let mut output = [0x5a; 200];
    assert_eq!(audio_bundle_encode(&input, &mut output), 152);
    assert_eq!(&output[..152], bytes);
    assert_eq!(&output[152..], &[0x5a; 48]);
    for transport in [Transport::Nrf, Transport::Esp] {
        assert_eq!(
            Message::decode(13, &bytes, transport),
            Ok(Message::AudioV2(view))
        );
    }
    let mut wrapped = [0x5a; 200];
    assert_eq!(Message::AudioV2(input).encode(&mut wrapped), Ok(152));
    assert_eq!(wrapped, output);
}

#[test]
fn every_flag_and_predecessor_presence_combination_has_explicit_validity() {
    for flags in 0..=255u8 {
        for history in 0..=2 {
            // Activity is meaningful only for present history; reserved bits reject.
            let valid = flags < 64
                && match history {
                    0 => flags & 0x1e == 0,
                    1 => flags & 2 != 0 && flags & 0x18 == 0,
                    _ => flags & 0x0a == 0x0a,
                };
            let mut bytes = vec![
                2,
                20,
                7,
                flags,
                0xff,
                0xff,
                48,
                if history == 0 { 0 } else { 48 },
            ];
            bytes.resize(56 + history * 48, 0x81);
            assert_eq!(
                audio_bundle_parse(&bytes).is_some(),
                valid,
                "flags={flags} history={history}"
            );
            for transport in [Transport::Nrf, Transport::Esp] {
                assert_eq!(Message::decode(13, &bytes, transport).is_ok(), valid);
            }
            let frame = [0x81; 48];
            let view = BundleView {
                current_data: &frame,
                previous1_data: if history >= 1 { &frame } else { &[] },
                previous2_data: if history == 2 { &frame } else { &[] },
                current_seq: 65535,
                stream_id: 7,
                flags,
                codec: 2,
            };
            for capacity in [0, 55, 56, 103, 104, 151, 152, 200] {
                let mut output = vec![0x5a; capacity];
                let accepted = valid && capacity >= bytes.len();
                let len = audio_bundle_encode(&view, &mut output);
                assert_eq!(len, if accepted { bytes.len() } else { 0 });
                if accepted {
                    assert_eq!(&output[..len], bytes);
                }
                assert!(output[len..].iter().all(|b| *b == 0x5a));
            }
        }
    }
}

#[test]
fn malformed_headers_lengths_and_payloads_reject_without_strip_mutation() {
    let original = golden();
    let mut cases = Vec::new();
    for length in 0..original.len() {
        cases.push(original[..length].to_vec());
    }
    for length in [153, 199, 200, 201, 250] {
        let mut bytes = original.clone();
        bytes.resize(length, 0);
        cases.push(bytes);
    }
    for (offset, good) in [(0, 2), (1, 20), (6, 48), (7, 48)] {
        for value in 0..=255u8 {
            if value == good {
                continue;
            }
            let mut bytes = original.clone();
            bytes[offset] = value;
            cases.push(bytes);
        }
    }
    for bytes in cases {
        assert!(audio_bundle_parse(&bytes).is_none());
        let mut output = bytes.clone();
        let mut len = output.len();
        assert!(!audio_bundle_strip_oldest(&mut output, &mut len));
        assert_eq!(len, bytes.len());
        assert_eq!(output, bytes);
    }
    for codec in [0, 1, 3, 255] {
        let view = BundleView {
            codec,
            current_data: &[0; 48],
            previous1_data: &[],
            previous2_data: &[],
            current_seq: 0,
            stream_id: 0,
            flags: 0,
        };
        let mut output = [0x5a; 200];
        assert_eq!(audio_bundle_encode(&view, &mut output), 0);
        assert_eq!(output, [0x5a; 200]);
    }
}

#[test]
fn invalid_frame_sizes_reject_encoding_without_writes() {
    let frame = [0x81; 200];
    for bad in [0, 1, 47, 49, 64, 65, 200] {
        for field in 0..3 {
            let view = BundleView {
                current_data: &frame[..if field == 0 { bad } else { 48 }],
                previous1_data: &frame[..if field == 1 { bad } else { 48 }],
                previous2_data: &frame[..if field == 2 { bad } else { 48 }],
                current_seq: 0,
                stream_id: 255,
                flags: 0x3f,
                codec: 2,
            };
            let mut output = [0x5a; 200];
            assert_eq!(audio_bundle_encode(&view, &mut output), 0);
            assert_eq!(output, [0x5a; 200]);
        }
    }
    for sequence in [0u16, 1, 65535] {
        let view = BundleView {
            current_data: &[0xcc; 48],
            previous1_data: &[],
            previous2_data: &[],
            current_seq: sequence,
            stream_id: 255,
            flags: 1,
            codec: 2,
        };
        let mut output = [0; 56];
        assert_eq!(audio_bundle_encode(&view, &mut output), 56);
        assert_eq!(
            &output[4..6],
            &[(sequence / 256) as u8, (sequence % 256) as u8]
        );
        assert_eq!(audio_bundle_parse(&output).unwrap().current_seq, sequence);
    }
}

#[test]
fn stripping_preserves_newer_frames_flags_and_unused_storage() {
    let mut bytes = golden();
    bytes.resize(200, 0x5a);
    let mut len = 152;
    let before = bytes.clone();
    assert!(audio_bundle_strip_oldest(&mut bytes, &mut len));
    assert_eq!(len, 104);
    assert_eq!(&bytes[..8], &[2, 20, 0xdb, 0x27, 0x12, 0x34, 48, 48]);
    assert_eq!(&bytes[8..56], &[0x11; 48]);
    assert_eq!(&bytes[56..104], &[0xcc; 48]);
    assert_eq!(&bytes[104..], &before[104..]);
    let before = bytes.clone();
    assert!(audio_bundle_strip_oldest(&mut bytes, &mut len));
    assert_eq!(len, 56);
    assert_eq!(&bytes[..8], &[2, 20, 0xdb, 0x21, 0x12, 0x34, 48, 0]);
    assert_eq!(&bytes[8..56], &[0xcc; 48]);
    assert_eq!(&bytes[56..], &before[56..]);
    let before = bytes.clone();
    assert!(!audio_bundle_strip_oldest(&mut bytes, &mut len));
    assert_eq!(len, 56);
    assert_eq!(bytes, before);
    len = 201;
    assert!(!audio_bundle_strip_oldest(&mut bytes, &mut len));
    assert_eq!(len, 201);
    assert_eq!(bytes, before);
}
