use openhelmet_wire::mesh::*;
use std::path::PathBuf;
use std::process::Command;

fn identity_fields(value: Identity, bytes: &[u8]) {
    assert_eq!(value.address_len, bytes[0]);
    assert_eq!(value.address, bytes[1..7]);
}

fn slot_fields(value: SlotMap, bytes: &[u8]) {
    assert_eq!(value.slot_count, bytes[0]);
    assert_eq!(value.slot_ids, bytes[1..9]);
    assert_eq!(value.active_speaker_count, bytes[9]);
    assert_eq!(value.active_speaker_ids, bytes[10..12]);
    assert_eq!(value.relay_masks, bytes[12..14]);
}

/// nRF-transport layouts that differ from (or do not exist on) the live ESP
/// mesh.h: fixed wire records with distinct bytes for each address position.
fn nrf_goldens() -> Vec<(u8, Vec<u8>)> {
    vec![
        (4, vec![0x81, 0x82, 0x83, 0x84, 0x85]),
        (
            5,
            vec![
                0x12, 0x34, 0x56, 0x78, 0xc7, 0xcf, 0x87, 0x88, 0x89, 0x8a, 0x8b,
            ],
        ),
        (11, (0x81..=0x87).collect()),
        (12, (0x81..=0x88).collect()),
    ]
}

#[test]
fn packed_layouts_match_live_esp_mesh_h_and_nrf_goldens() {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let root = manifest.parent().unwrap().parent().unwrap();
    let executable = std::env::temp_dir().join(format!("mesh-oracle-{}", std::process::id()));
    let compiled = Command::new(std::env::var_os("CC").unwrap_or_else(|| "cc".into()))
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-I"])
        .arg(root)
        .arg("-I")
        .arg(root.join("shared"))
        .arg("-I")
        .arg(manifest.join("tests/esp_stub_include"))
        .arg(manifest.join("tests/mesh_layout_fixture.c"))
        .arg("-o")
        .arg(&executable)
        .output()
        .unwrap();
    assert!(
        compiled.status.success(),
        "{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    let output = Command::new(&executable).output().unwrap();
    std::fs::remove_file(&executable).unwrap();
    assert!(output.status.success());
    let mut esp = Vec::new();
    let mut bytes = output.stdout.as_slice();
    while bytes[0] != 255 {
        let size = usize::from(bytes[1]);
        esp.push((bytes[0], bytes[2..2 + size].to_vec()));
        bytes = &bytes[2 + size..];
    }
    let tail = &bytes[1..];
    let mut nrf: Vec<_> = esp.iter().filter(|(id, _)| *id != 5).cloned().collect();
    nrf.extend(nrf_goldens());
    for (transport, records) in [(Transport::Esp, &esp), (Transport::Nrf, &nrf)] {
        for (id, data) in records {
            let id = *id;
            let data = data.as_slice();
            let size = data.len();
            {
                if id == 0 {
                    assert_eq!(size, HEADER_SIZE);
                    let header = Header {
                        version: VERSION,
                        packet_type: 0xd7,
                        src_id: 0x29,
                        seq: 0x4b,
                        ttl: 0x6d,
                        flags: 0x8f,
                        talk_channel: 0xb1,
                        payload_len: 0x1234,
                    };
                    let payload = vec![0xa5; usize::from(header.payload_len)];
                    let packet_len = HEADER_SIZE + payload.len();
                    let mut encoded = vec![0x5a; packet_len + 1];
                    assert_eq!(
                        encode(&header, &payload, &mut encoded, payload.len()),
                        Ok(packet_len)
                    );
                    // Compare every byte with the original C struct, independently
                    // of Rust decode, so symmetric field-offset swaps cannot pass.
                    assert_eq!(&encoded[..HEADER_SIZE], data);
                    assert_eq!(&encoded[HEADER_SIZE..packet_len], payload);
                    assert_eq!(encoded[packet_len], 0x5a);
                    let mut c_packet = data.to_vec();
                    c_packet.extend_from_slice(&payload);
                    let decoded = decode(&c_packet, payload.len()).unwrap();
                    assert_eq!(decoded.header, header);
                    assert_eq!(decoded.payload, payload);
                } else if id == 24 {
                    assert_eq!(size, Identity::SIZE);
                    let identity = Identity::decode(data).unwrap();
                    let mut dst = [0; 7];
                    identity.encode(&mut dst).unwrap();
                    assert_eq!(dst, data);
                    assert_eq!(identity.address_len, 0x81); // Not a policy validator.
                } else if id == 25 {
                    assert_eq!(data, [6, 7, 23, 12, 29, 17, 11]);
                } else {
                    let message = Message::decode(id, data, transport).unwrap();
                    assert_eq!(message.packet_type(), id);
                    let mut encoded = [0x5a; 209];
                    assert_eq!(message.encode(&mut encoded).unwrap(), size);
                    assert_eq!(&encoded[..size], data);
                    assert!(encoded[size..].iter().all(|&x| x == 0x5a));
                    for len in 0..size {
                        // Legacy AUDIO is variable; nRF LEAVE also has an empty variant.
                        if id != 1 && !(id == 4 && len == 0) {
                            assert!(Message::decode(id, &data[..len], transport).is_err());
                        }
                    }
                    if id != 1 {
                        let mut padded = data.to_vec();
                        padded.push(0);
                        assert!(Message::decode(id, &padded, transport).is_err());
                    }
                    if size != 0 {
                        let mut short = vec![0x5a; size - 1];
                        assert!(message.encode(&mut short).is_err());
                        assert!(short.iter().all(|&x| x == 0x5a));
                    }
                    match message {
                        Message::Audio(v) => {
                            assert_eq!(
                                [v.codec, v.frame_ms, v.stream_id, v.audio_flags],
                                data[..4]
                            );
                            assert_eq!(v.data, &data[4..]);
                        }
                        Message::Join(v) => assert_eq!([v.capabilities, v.reserved], data),
                        Message::JoinAck(v) => {
                            assert_eq!([v.assigned_id, v.slot_index, v.coordinator_id], data)
                        }
                        Message::LeaveNrf(v) => assert_eq!(v.sender_addr, data),
                        Message::SlotMap(v) => slot_fields(v, data),
                        Message::Keepalive(v) => assert_eq!([v.battery_pct, v.reserved], data),
                        Message::SpeakerGrant(v) => {
                            assert_eq!(v.speaker_count, data[0]);
                            assert_eq!(v.speaker_ids, data[1..3]);
                            assert_eq!(v.relay_masks, data[3..5]);
                        }
                        Message::SpeakerRelease(v) => {
                            assert_eq!(v.speaker_count, data[0]);
                            assert_eq!(v.speaker_ids, data[1..3]);
                        }
                        Message::JoinV2(v) => {
                            assert_eq!([v.capabilities, v.reserved], data[..2]);
                            assert_eq!(v.requester_addr, data[2..]);
                        }
                        Message::JoinAckV2(v) => {
                            assert_eq!([v.assigned_id, v.slot_index, v.coordinator_id], data[..3]);
                            assert_eq!(v.target_addr, data[3..]);
                        }
                        Message::JoinV3(v) => {
                            identity_fields(v.origin, &data[..7]);
                            assert_eq!(v.capabilities, data[7]);
                            identity_fields(v.target, &data[8..]);
                        }
                        Message::SyncNrf(v) => {
                            assert_eq!(v.frame_counter, 0x78563412);
                            assert_eq!(v.drift_ppm, -12345);
                            assert_eq!(v.coordinator_addr, [0x87, 0x88, 0x89, 0x8a, 0x8b]);
                            assert_eq!(size, 11);
                        }
                        Message::SyncEsp(v) => {
                            assert_eq!(v.frame_counter, 0x78563412);
                            assert_eq!(v.drift_ppm, -12345);
                            assert_eq!(v.coordinator_mac, data[6..12]);
                            assert_eq!(size, 12);
                        }
                        Message::Status(v) => {
                            assert_eq!(v.rssi_dbm, -101);
                            assert_eq!(v.temperature_c, -37);
                        }
                        Message::Topology(v) => {
                            assert_eq!(v.report_seq, 0x78563412);
                            assert_eq!(v.term, 0xefcdab89);
                            assert_eq!(v.rssi_dbm[0], -128);
                            assert_eq!(v.rssi_dbm[7], -1);
                            identity_fields(v.origin, &data[4..11]);
                            assert_eq!(v.direct_mask, data[11]);
                            assert_eq!(v.quality, data[20..28]);
                            assert_eq!(v.leader_id, data[28]);
                            slot_fields(v.slot_map, &data[33..]);
                        }
                        Message::JoinAckV3(v) => {
                            assert_eq!(v.term, 0x78563412);
                            identity_fields(v.target, &data[..7]);
                            assert_eq!(
                                [
                                    v.capabilities,
                                    v.assigned_id,
                                    v.slot_index,
                                    v.coordinator_id
                                ],
                                data[7..11]
                            );
                        }
                        Message::SyncV3(v) => {
                            assert_eq!(v.term, 0x78563412);
                            assert_eq!(v.frame_counter, 0xefcdab89);
                            assert_eq!(v.phase_us, 0xfedc);
                            identity_fields(v.leader, &data[..7]);
                            assert_eq!([v.leader_id, v.member_count], data[7..9]);
                            assert_eq!(v.relay_depth, data[19]);
                        }
                        Message::HandoverPrepare(v)
                        | Message::HandoverCommit(v)
                        | Message::HandoverCancel(v) => {
                            assert_eq!(v.next_term, 0x78563412);
                            assert_eq!(v.switch_frame, 0xefcdab89);
                            assert_eq!([v.old_leader_id, v.candidate_id], data[..2]);
                            assert_eq!(v.members, data[10]);
                            slot_fields(v.slot_map, &data[11..]);
                        }
                        Message::HandoverAck(v) => {
                            assert_eq!(v.next_term, 0x78563412);
                            assert_eq!(v.switch_frame, 0xefcdab89);
                            assert_eq!([v.old_leader_id, v.candidate_id], data[..2]);
                            assert_eq!(v.acknowledger_id, data[10]);
                        }
                        Message::MembershipV3(v) => {
                            assert_eq!(v.term, 0x78563412);
                            assert_eq!(v.revision, 0xefcdab89);
                            assert_eq!(size, 79);
                            assert_eq!(v.leader_id, data[8]);
                            slot_fields(v.slot_map, &data[9..23]);
                            for (identity, bytes) in
                                v.identities.iter().zip(data[23..].chunks_exact(7))
                            {
                                identity_fields(*identity, bytes);
                            }
                        }
                        Message::SpeakerRequest(v) => {
                            assert_eq!(v.term, 0x78563412);
                            assert_eq!(v.request_seq, 0xefcdab89);
                            assert_eq!(v.frame_counter, 0x12345678);
                            assert_eq!(v.active, data[12]);
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    let mut expected = Vec::new();
    for codec in 0..=255 {
        for local in 0..4 {
            for ms in 19..=21 {
                for len in [0, 1, 47, 48, 49, 64, 65, 196, 65535] {
                    expected.push(u8::from(audio_wire_validate(codec, ms, len, local)));
                }
            }
        }
    }
    assert_eq!(tail, expected);
}

#[test]
fn envelope_is_not_dispatch_policy() {
    for packet_type in 0..=255 {
        for len in [0, 1, 70, 119, 120, 128, 129, 191, 192, 200, 201, 209] {
            let payload = vec![0xa5; len];
            let header = Header {
                version: VERSION,
                packet_type,
                src_id: 255,
                seq: 255,
                ttl: 255,
                flags: 255,
                talk_channel: 255,
                payload_len: len as u16,
            };
            let mut output = [0x5a; 219];
            let size = encode(&header, &payload, &mut output, 209).unwrap();
            let mut short = vec![0x5a; size - 1];
            assert_eq!(
                encode(&header, &payload, &mut short, usize::MAX),
                Err(Error::Capacity)
            );
            assert!(short.iter().all(|&byte| byte == 0x5a));
            for capacity in [128, 200, 209] {
                let result = decode(&output[..size], capacity);
                if len > capacity {
                    assert_eq!(result, Err(Error::Capacity));
                } else {
                    let view = result.unwrap();
                    assert_eq!(view.header, header);
                    assert_eq!(view.payload, payload);
                }
                let mut dst = [0x5a; 219];
                let result = encode(&header, &payload, &mut dst, capacity);
                if len > capacity {
                    assert_eq!(result, Err(Error::Capacity));
                    assert_eq!(dst, [0x5a; 219]);
                } else {
                    assert_eq!(result, Ok(size));
                }
            }
            for truncated in 0..size {
                assert!(decode(&output[..truncated], 209).is_err());
            }
            assert!(decode(&output[..size + 1], 210).is_err());
            output[0] = 4;
            assert_eq!(decode(&output[..size], 209), Err(Error::Version));
        }
    }
    for transport in [Transport::Nrf, Transport::Esp] {
        assert_eq!(Message::decode(4, &[], transport), Ok(Message::Leave));
        assert_eq!(
            Message::decode(255, &[], transport),
            Err(Error::UnknownType)
        );
    }
    let leave = Message::Leave;
    assert_eq!(leave.packet_type(), 4);
    let mut leave_output = [0x5a; 8];
    assert_eq!(leave.encode(&mut leave_output), Ok(0));
    assert_eq!(leave_output, [0x5a; 8]);
    assert_eq!(leave.encode(&mut []), Ok(0));
    assert!(Message::decode(4, &[0; 5], Transport::Nrf).is_ok());
    assert!(Message::decode(4, &[0; 5], Transport::Esp).is_err());
    for len in 0..=201 {
        assert_eq!(
            Audio::decode(&vec![0; len]).is_ok(),
            (4..=200).contains(&len)
        );
    }
    let header = Header {
        version: 5,
        packet_type: 255,
        src_id: 0,
        seq: 0,
        ttl: 0,
        flags: 0,
        talk_channel: 0,
        payload_len: 1,
    };
    let mut dst = [0x5a; 10];
    assert_eq!(encode(&header, &[], &mut dst, 209), Err(Error::Length));
    assert_eq!(dst, [0x5a; 10]);
    let header = Header {
        payload_len: 200,
        ..header
    };
    let mut packet = [0x5a; CONTROL_MAX_PACKET_SIZE];
    assert_eq!(
        encode(
            &header,
            &[0; 200],
            &mut packet,
            NRF_RECEIVE_PAYLOAD_CAPACITY
        ),
        Ok(209)
    );
    assert!(decode(&packet, NRF_RECEIVE_PAYLOAD_CAPACITY).is_ok());
    assert_eq!(
        decode(&packet, ESP_RECEIVE_PAYLOAD_CAPACITY),
        Err(Error::Capacity)
    );
}
