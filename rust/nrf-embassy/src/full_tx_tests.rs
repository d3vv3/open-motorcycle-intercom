//! Full TX corpus: membership + adaptive + TX with typed packet IO review.
//! These tests are authored pending execution; no hardware qualification.
use super::*;
use crate::protocol_tx::CombinedFullTxPlatform;

#[path = "audio_differential.rs"]
mod audio_differential;

#[derive(Clone, Copy)]
enum TxCheck {
    None,
    Return(i32),
    PriorityRetained,
    ForwardUnavailable,
    ForwardConsumed { rf: bool },
    OwnRetained,
    Coalesced,
    NewMembership,
    CapacityRejected,
    Prepared,
    ConvergenceReady,
    Radio { packet_type: u8, seq: Option<u8> },
    Topology { success: bool },
    RequestFrame,
    SyncDeferred,
    SyncBeforePriority,
}
#[derive(Clone, Copy)]
struct TxOp(JointOp, TxCheck);
impl From<JointOp> for TxOp {
    fn from(op: JointOp) -> Self {
        Self(op, TxCheck::None)
    }
}
fn checked(op: JointOp, check: TxCheck) -> TxOp {
    TxOp(op, check)
}
fn tx(code: u8, t: u8, payload: &[u8], now: i64) -> JointOp {
    assert!(payload.len() <= 201);
    let mut op = JointOp::new(code, now).arg(1, t);
    op.0[34..36].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    op.0[36..36 + payload.len()].copy_from_slice(payload);
    op.0[254] = 1;
    op
}
fn message(code: u8, msg: Message<'_>, now: i64) -> JointOp {
    let mut payload = [0; 200];
    let len = msg.encode(&mut payload).unwrap();
    tx(code, msg.packet_type(), &payload[..len], now)
}
fn forward(msg: Message<'_>, src: u8, seq: u8, now: i64) -> JointOp {
    let mut op = JointOp::packet(msg, src, seq, now).arg(0, 52).scalar(16, 7);
    if msg.packet_type() == 15 {
        op = op.arg(6, 1).identity(remote(4));
    }
    op.0[254] = 1;
    op
}
fn attempt(frame: u32, now: i64, result: i32) -> JointOp {
    JointOp::new(51, now)
        .frame(frame)
        .tx_failure(false, result)
        .arg(254, 1)
}
fn setup_tx(ops: &mut Vec<TxOp>, local: u8, now: i64) {
    let mut base = Vec::new();
    setup(&mut base, local, now);
    ops.extend(base.into_iter().map(TxOp::from));
    // Context-only cursor/sequence seed; stale buffers intentionally survive.
    ops.push(JointOp::new(50, now).into());
}

fn stale_entry(index: u8, now: i64) -> JointOp {
    let mut op = tx(53, index, &[0xa5; 200], now);
    op.0[27..36].copy_from_slice(&[5, 7, 1, 77, 2, 0, 1, 200, 0]);
    op
}

fn run_full(r: &mut Runtime, p: &mut Joint, op: JointOp) -> Option<i32> {
    assert!(p.full_tx);
    let x = op.0;
    if x[0] < 40 {
        run_joint(r, p, op);
        return None;
    }
    p.inner.now = g64(&x[8..]);
    p.inner.logs = x[255] != 0;
    p.frame = g32(&x[240..]);
    p.queue_result = g32(&x[244..]) as i32;
    p.priority_result = g32(&x[248..]) as i32;
    p.clock_available = x[254] != 0;
    p.clock_phase = u16::from_le_bytes([x[252], x[253]]);
    p.grant_count = x[220];
    p.grant_ids.copy_from_slice(&x[221..223]);
    p.grant_masks.copy_from_slice(&x[223..225]);
    let mut platform = CombinedFullTxPlatform(p);
    let h = header(&x);
    let payload = &x[36..36 + usize::from(h.payload_len)];
    let mut handled = false;
    let ret = match x[0] {
        40 => Some(r.tx_send_packet_ex(x[1], payload, x[2], x[3], x[4], x[5], &mut platform)),
        41 => Some(r.tx_send_packet(x[1], payload, &mut platform)),
        42 => Some(r.tx_queue_control(x[1], payload)),
        43 => Some(r.tx_queue_priority(x[1], payload)),
        44 => Some(r.tx_send_join_request(&mut platform)),
        45 => Some(r.tx_send_keepalive()),
        46 => {
            let map = r.tx_slot_snapshot();
            platform.0.inner.e(53);
            slots(&mut platform.0.inner.bytes, map);
            None
        }
        47 => Some(r.tx_send_slot_map(&mut platform)),
        48 => Some(r.tx_send_status_packet()),
        49 => Some(r.tx_send_join_ack(x[1], x[2], x[36..41].try_into().unwrap())),
        50 => {
            r.protocol.tx_seq = x[1];
            r.protocol.control_turn = x[2];
            r.protocol.air_report_seq = g32(&x[16..]);
            r.session.control_head = x[3] % 32;
            r.session.control_tail = x[4] % 32;
            r.protocol.priority_pending = x[5] != 0;
            None
        }
        51 => {
            r.tx_control_handler(platform.0.frame, &mut platform);
            None
        }
        52 => {
            let id = wire::Identity::decode(&x[20..27]).unwrap();
            let id = IdentityValue {
                address_len: id.address_len,
                address: id.address,
            };
            let h = a::Header {
                version: h.version,
                packet_type: h.packet_type,
                src_id: h.src_id,
                seq: h.seq,
                ttl: h.ttl,
                flags: h.flags,
                talk_channel: h.talk_channel,
                payload_len: h.payload_len,
            };
            handled = a::forward_enqueue(
                Some(&mut r.protocol.adaptive),
                Some(&h),
                Some(payload),
                (x[6] != 0).then_some(&id),
                g32(&x[16..]),
                true,
                platform.0.inner.now as u32,
            );
            None
        }
        53 => {
            let entry = &mut r.session.control_ring[usize::from(x[1] % 32)];
            entry.len = wire::encode(&h, payload, &mut entry.data, 200).unwrap() as u8;
            None
        }
        _ => panic!("unknown full TX opcode {}", x[0]),
    };
    if let Some(ret) = ret {
        platform.0.inner.e(52);
        p32(&mut platform.0.inner.bytes, ret as u32);
    }
    snapshot_joint(r, platform.0, handled);
    ret
}

/// Each type is decoded/re-encoded by the actual typed wire layout, not a
/// handwritten packed-layout substitute. Source TX still decides acceptance.
fn typed_payload(t: u8) -> Vec<u8> {
    if t == 13 {
        let data = [0x81; 48];
        let bundle = openhelmet_wire::audio_bundle::BundleView {
            previous1_data: &[],
            previous2_data: &[],
            current_data: &data,
            current_seq: 0xffff,
            stream_id: 7,
            flags: 1,
            codec: 2,
        };
        let mut payload = [0; 200];
        let len = Message::AudioV2(bundle).encode(&mut payload).unwrap();
        return payload[..len].to_vec();
    }
    let len = match t {
        1 => 52,
        2 => wire::Join::SIZE,
        3 => wire::JoinAck::SIZE,
        4 => wire::LeaveNrf::SIZE,
        5 => wire::SyncNrf::SIZE,
        6 => wire::SlotMap::SIZE,
        7 => wire::Status::SIZE,
        8 => wire::Keepalive::SIZE,
        9 => wire::SpeakerGrant::SIZE,
        10 => wire::SpeakerRelease::SIZE,
        11 => wire::JoinV2::SIZE,
        12 => wire::JoinAckV2::SIZE,
        14 => wire::Topology::SIZE,
        15 => wire::JoinV3::SIZE,
        16 => wire::JoinAckV3::SIZE,
        17 => wire::SyncV3::SIZE,
        18 | 20 | 21 => wire::HandoverProposal::SIZE,
        19 => wire::HandoverAck::SIZE,
        22 => wire::MembershipV3::SIZE,
        23 => wire::SpeakerRequest::SIZE,
        _ => unreachable!(),
    };
    let payload: Vec<_> = (0..len)
        .map(|i| (i as u8).wrapping_mul(17).wrapping_add(t))
        .collect();
    let msg = Message::decode(t, &payload, wire::Transport::Nrf).unwrap();
    let mut encoded = [0; 200];
    let written = msg.encode(&mut encoded).unwrap();
    assert_eq!(&encoded[..written], payload);
    payload
}

fn fixtures() -> Vec<TxOp> {
    // This is a separate replay, not a change to the old boundary corpus.
    let mut ops: Vec<_> = joint_fixtures()
        .into_iter()
        .map(|mut op| {
            op.0[254] = 1;
            op.0[252..254].copy_from_slice(&123u16.to_le_bytes());
            TxOp::from(op)
        })
        .collect();
    setup_tx(&mut ops, 2, 100);
    for t in 1..=23 {
        let op = tx(40, t, &typed_payload(t), 101)
            .arg(2, 251)
            .arg(3, 250)
            .arg(4, 249)
            .arg(5, 248);
        ops.push(checked(op, TxCheck::Return(if t == 1 { -22 } else { 0 })));
    }
    ops.push(checked(
        tx(40, 255, &[0x99; 200], 102)
            .arg(2, 254)
            .arg(3, 253)
            .arg(4, 252)
            .arg(5, 251),
        TxCheck::Radio {
            packet_type: 255,
            seq: Some(251),
        },
    ));
    for (t, len, result) in [(1, 201, -22), (13, 201, -22), (13, 0, -22), (255, 201, -90)] {
        ops.push(checked(
            tx(41, t, &vec![0; len], 103),
            TxCheck::Return(result),
        ));
    }
    ops.push(JointOp::new(50, 104).arg(1, 255).into());
    ops.push(checked(
        tx(41, 255, &[7], 105).tx_failure(false, -5),
        TxCheck::Radio {
            packet_type: 255,
            seq: Some(255),
        },
    ));
    ops.push(checked(
        tx(41, 254, &[8], 106),
        TxCheck::Radio {
            packet_type: 254,
            seq: Some(0),
        },
    ));
    ops.push(checked(
        JointOp::new(44, 107),
        TxCheck::Radio {
            packet_type: 15,
            seq: Some(1),
        },
    ));
    ops.push(checked(tx(43, 1, &[99; 200], 108), TxCheck::Return(0)));
    ops.push(checked(tx(43, 16, &[1], 109), TxCheck::Return(-105)));
    ops.push(checked(tx(43, 16, &[1; 201], 109), TxCheck::Return(-105)));
    ops.push(checked(
        attempt(1, 110, -5).arg(254, 0),
        TxCheck::PriorityRetained,
    ));
    ops.push(checked(
        attempt(9, 111, 0).arg(254, 0),
        TxCheck::Radio {
            packet_type: 1,
            seq: Some(2),
        },
    ));
    ops.push(JointOp::new(49, 112).arg(1, 3).arg(2, 7).into());
    ops.push(checked(
        attempt(17, 113, 0),
        TxCheck::Radio {
            packet_type: 16,
            seq: Some(3),
        },
    ));
    ops.push(JointOp::new(45, 114).into());
    ops.push(JointOp::new(48, 114).into());
    ops.push(JointOp::new(31, 114).arg(1, 0).arg(2, 3).arg(4, 7).into());
    ops.push(JointOp::new(46, 114).into());
    ops.push(checked(JointOp::new(47, 114), TxCheck::Return(-11)));

    // Capacity is 31, and coalescing precedes its rejection. Failed enqueues
    // cannot allocate an RF seq or publication revision; stale suffixes survive.
    setup_tx(&mut ops, 1, 200);
    ops.push(message(42, Message::MembershipV3(map(99)), 201).into());
    for i in 0..30 {
        ops.push(tx(42, 7, &[i; 8], 202).into());
    }
    ops.push(checked(tx(42, 8, &[255, 0], 203), TxCheck::Return(-105)));
    ops.push(checked(tx(42, 22, &[88; 8], 204), TxCheck::Coalesced));
    ops.push(checked(tx(42, 14, &[0; 200], 204), TxCheck::Return(-105)));
    ops.push(checked(tx(42, 1, &[0; 201], 204), TxCheck::Return(-22)));
    ops.push(checked(
        tx(42, 13, &typed_payload(13), 204),
        TxCheck::Return(-22),
    ));
    ops.push(checked(tx(42, 7, &[0; 201], 204), TxCheck::Return(-90)));

    // First matching coalesced entry, duplicate type and full-length stale tail.
    setup_tx(&mut ops, 2, 300);
    ops.push(JointOp::new(50, 300).arg(3, 2).into());
    for (i, seq) in [(0, 71), (1, 72)] {
        let mut op = tx(53, i, &[0x55; 200], 300);
        op.0[27..36].copy_from_slice(&[5, 14, 2, seq, 2, 0, 1, 200, 0]);
        ops.push(op.into());
    }
    ops.push(checked(tx(42, 14, &[0x66; 4], 301), TxCheck::Coalesced));
    ops.push(checked(
        attempt(1, 302, -5),
        TxCheck::Topology { success: false },
    ));
    ops.push(checked(
        attempt(9, 303, 0),
        TxCheck::Topology { success: true },
    ));

    // Wrapped own ring, stable equal-rank ordering and retained RF failure.
    setup_tx(&mut ops, 2, 400);
    ops.push(
        JointOp::new(50, 400)
            .arg(1, 255)
            .arg(3, 30)
            .arg(4, 30)
            .scalar(16, u32::MAX)
            .into(),
    );
    ops.push(JointOp::new(45, 401).into());
    ops.push(tx(42, 9, &[1; wire::SpeakerGrant::SIZE], 401).into());
    ops.push(tx(42, 10, &[2; wire::SpeakerRelease::SIZE], 401).into());
    ops.push(tx(42, 6, &[3; wire::SlotMap::SIZE], 401).into());
    ops.push(checked(attempt(1, 402, -5), TxCheck::OwnRetained));
    for (frame, t, seq) in [(9, 6, 3), (17, 9, 0), (25, 10, 1), (33, 8, 255)] {
        ops.push(checked(
            attempt(frame, 403, 0),
            TxCheck::Radio {
                packet_type: t,
                seq: Some(seq),
            },
        ));
    }
    ops.push(message(42, Message::Topology(topo(2, 44)), 404).into());
    ops.push(checked(
        attempt(41, 405, -5),
        TxCheck::Topology { success: false },
    ));
    ops.push(checked(
        attempt(49, 406, 0),
        TxCheck::Topology { success: true },
    ));
    ops.push(
        message(
            42,
            Message::SpeakerRequest(wire::SpeakerRequest {
                term: 7,
                request_seq: u32::MAX,
                frame_counter: 0,
                active: 1,
            }),
            407,
        )
        .into(),
    );
    ops.push(checked(attempt(57, 408, -5), TxCheck::RequestFrame));
    ops.push(checked(attempt(65, 409, 0), TxCheck::RequestFrame));

    // Coordinator sync cadence is exclusive even when its control slot matches;
    // unavailable/invalid clock does not consume seq, RF failure consumes it.
    setup_tx(&mut ops, 1, 500);
    ops.push(JointOp::new(49, 500).arg(1, 2).arg(2, 1).into());
    ops.push(checked(
        attempt(10, 501, 0).arg(254, 0),
        TxCheck::SyncDeferred,
    ));
    ops.push(checked(
        attempt(20, 502, 0).arg(252, 0x20).arg(253, 0x4e),
        TxCheck::SyncDeferred,
    ));
    ops.push(checked(
        attempt(30, 503, -5).arg(252, 99),
        TxCheck::Radio {
            packet_type: 17,
            seq: Some(1),
        },
    ));
    ops.push(checked(
        attempt(40, 504, 0).arg(252, 101),
        TxCheck::Radio {
            packet_type: 17,
            seq: Some(2),
        },
    ));
    ops.push(attempt(1, 505, 0).into()); // wrong slot
    ops.push(checked(
        attempt(8, 506, 0),
        TxCheck::Radio {
            packet_type: 16,
            seq: Some(0),
        },
    ));
    ops.push(JointOp::new(47, 507).into());
    ops.push(attempt(16, 508, 0).into());

    // Actual shared forward selection, head wrap, retained cache and fairness.
    setup_tx(&mut ops, 2, 600);
    let ack = Message::HandoverAck(wire::HandoverAck {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 260,
        acknowledger_id: 3,
    });
    for seq in 0..7 {
        ops.push(forward(ack, 3, seq, 600).into());
        ops.push(checked(
            attempt(1 + u32::from(seq) * 8, 601, 0),
            TxCheck::ForwardConsumed { rf: true },
        ));
    }
    ops.push(forward(Message::Topology(topo(3, 1)), 3, 10, 602).into());
    ops.push(forward(ack, 3, 11, 602).into());
    ops.push(forward(ack, 3, 12, 602).into());
    ops.push(JointOp::new(45, 602).into());
    ops.push(
        JointOp::new(50, 602)
            .arg(1, 255)
            .arg(2, 255)
            .arg(3, 1)
            .into(),
    );
    ops.push(checked(
        attempt(1, 603, 0),
        TxCheck::Radio {
            packet_type: 8,
            seq: Some(0),
        },
    ));
    ops.push(checked(
        attempt(9, 604, 0).arg(254, 0),
        TxCheck::ForwardUnavailable,
    ));
    ops.push(checked(
        attempt(17, 605, -5),
        TxCheck::ForwardConsumed { rf: true },
    ));
    ops.push(checked(
        attempt(25, 606, 0),
        TxCheck::ForwardConsumed { rf: true },
    ));
    ops.push(checked(
        attempt(33, 607, 0),
        TxCheck::ForwardConsumed { rf: true },
    ));
    let sync = Message::SyncV3(wire::SyncV3 {
        leader: local(),
        leader_id: 1,
        member_count: 3,
        term: 7,
        frame_counter: 1,
        phase_us: 2,
        relay_depth: 0,
    });
    ops.push(forward(sync, 1, 13, 608).into());
    ops.push(checked(
        attempt(41, 5000, 0),
        TxCheck::ForwardConsumed { rf: false },
    ));
    ops.push(forward(sync, 1, 14, 5001).into());
    ops.push(
        JointOp::old(
            seed(3, 2, 2, 5001)
                .arg(3, 2)
                .arg(4, 1)
                .addr([2, 2, 3, 4, 5]),
        )
        .scalar(45, 5001)
        .into(),
    );
    ops.push(checked(
        attempt(49, 5002, 0),
        TxCheck::ForwardConsumed { rf: true },
    ));
    // Source forward ranks 4/3/2/1, equal-rank FIFO, low-head promotion.
    setup_tx(&mut ops, 2, 6000);
    let proposal = wire::HandoverProposal {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 260,
        members: 7,
        slot_map: slot_map(),
    };
    for (msg, src, seq) in [
        (Message::Topology(topo(3, 9)), 3, 20),
        (
            Message::SpeakerRequest(wire::SpeakerRequest {
                term: 7,
                request_seq: 1,
                frame_counter: 0,
                active: 1,
            }),
            3,
            21,
        ),
        (Message::HandoverPrepare(proposal), 1, 22),
        (
            Message::JoinV3(wire::JoinV3 {
                origin: remote(4),
                capabilities: 2,
                target: local(),
            }),
            0,
            23,
        ),
        (Message::HandoverCancel(proposal), 1, 24),
        (ack, 3, 25),
        (Message::HandoverCommit(proposal), 1, 26),
        (Message::MembershipV3(map(1)), 1, 27),
    ] {
        ops.push(forward(msg, src, seq, 6000).into());
    }
    ops.push(
        forward(
            Message::JoinAckV3(wire::JoinAckV3 {
                target: remote(4),
                capabilities: 2,
                assigned_id: 4,
                slot_index: 3,
                coordinator_id: 1,
                term: 7,
            }),
            1,
            28,
            8001,
        )
        .into(),
    );
    for (frame, t, seq) in [
        (1, 19, 25),
        (9, 20, 26),
        (17, 22, 27),
        (25, 16, 28),
        (33, 18, 22),
        (41, 15, 23),
        (49, 21, 24),
        (57, 23, 21),
    ] {
        ops.push(checked(
            attempt(frame, 8002, 0),
            TxCheck::Radio {
                packet_type: t,
                seq: Some(seq),
            },
        ));
    }
    // Actual receive/transition wrappers create convergence, not a forged
    // Adaptive snapshot. Convergence SYNC precedes the priority packet.
    setup_tx(&mut ops, 2, 0);
    // receive_prepare requires a fresh universal graph for the candidate.
    // Populate it with the same real report API used by the source fixtures.
    for (now, seq) in [(10000, 1), (13000, 2)] {
        for id in 1..=3 {
            ops.push(
                JointOp::packet(Message::Topology(topo(id, seq)), id, seq as u8, now)
                    .arg(0, 30)
                    .arg(1, id)
                    .into(),
            );
        }
    }
    ops.push(checked(
        JointOp::packet(Message::HandoverPrepare(proposal), 1, 2, 13001).frame(100),
        TxCheck::Prepared,
    ));
    ops.push(JointOp::packet(Message::HandoverCommit(proposal), 1, 3, 13002).into());
    ops.push(checked(
        JointOp::new(25, 13003).frame(260),
        TxCheck::ConvergenceReady,
    ));
    ops.push(JointOp::new(50, 13004).into());
    ops.push(JointOp::new(49, 13004).arg(1, 3).arg(2, 2).into());
    ops.push(checked(attempt(273, 13005, 0), TxCheck::SyncBeforePriority));
    ops.push(checked(
        attempt(281, 13006, 0),
        TxCheck::Radio {
            packet_type: 16,
            seq: Some(0),
        },
    ));
    // Participant frame ten is exclusive even at its matching local slot.
    setup_tx(&mut ops, 3, 14000);
    ops.push(JointOp::new(49, 14001).arg(1, 2).arg(2, 1).into());
    ops.push(checked(attempt(10, 14002, 0), TxCheck::SyncDeferred));
    ops.push(checked(
        attempt(18, 14003, 0),
        TxCheck::Radio {
            packet_type: 16,
            seq: Some(0),
        },
    ));
    // Revision is an entry-storage scalar, not a minimum payload length.
    // New entries and coalescing both retain exact stale suffix bytes while
    // keeping header/payload lengths at 9 + len, including an empty payload.
    for len in 0..=8 {
        setup_tx(&mut ops, 1, 15000);
        ops.push(stale_entry(0, 15000).into());
        ops.push(checked(
            tx(42, 22, &vec![0x3c; len], 15001),
            TxCheck::NewMembership,
        ));
        ops.push(checked(
            tx(42, 22, &vec![0x69; len], 15002),
            TxCheck::Coalesced,
        ));
        ops.push(checked(
            attempt(8, 15003, 0),
            TxCheck::Radio {
                packet_type: 22,
                seq: Some(0),
            },
        ));
    }
    setup_tx(&mut ops, 1, 15100);
    ops.push(stale_entry(0, 15100).into());
    ops.push(checked(
        message(42, Message::MembershipV3(map(99)), 15101),
        TxCheck::NewMembership,
    ));
    for i in 0..30 {
        ops.push(tx(42, 7, &[i; 8], 15102).into());
    }
    for len in 0..=8 {
        ops.push(checked(
            tx(42, 22, &vec![0x70 + len as u8; len], 15103),
            TxCheck::Coalesced,
        ));
    }
    setup_tx(&mut ops, 1, 15200);
    for i in 0..31 {
        ops.push(tx(42, 7, &[i; 8], 15201).into());
    }
    for len in 0..=8 {
        ops.push(checked(
            tx(42, 22, &vec![0x45; len], 15202),
            TxCheck::CapacityRejected,
        ));
    }
    ops
}

#[test]
fn join_radio_packet_matches_literal_wire_record_even_on_send_failure() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    p.full_tx = true;
    let mut ops = Vec::new();
    setup_tx(&mut ops, 2, 0);
    for op in ops {
        run_full(&mut r, &mut p, op.0);
    }
    r.talk_channel = 3;
    r.protocol.tx_seq = 0x42;
    let head = r.session.control_head;
    let tail = r.session.control_tail;
    assert_eq!(
        run_full(&mut r, &mut p, JointOp::new(44, 1).tx_failure(false, -5)),
        Some(-5)
    );
    assert_eq!(
        p.radio_frames.last().unwrap().as_slice(),
        &[5, 15, 0, 0x42, 2, 0, 3, 15, 0, 5, 2, 2, 3, 4, 5, 0, 2, 5, 1, 2, 3, 4, 5, 0,]
    );
    assert_eq!(
        (r.session.control_head, r.session.control_tail),
        (head, tail)
    );
    assert_eq!(r.protocol.tx_seq, 0x43);
}

#[test]
fn full_tx_corpus_state_effects_and_physical_packet_bytes() {
    let ops = fixtures();
    let mut p = Joint::new();
    p.full_tx = true;
    let mut r = Runtime::default();
    for (i, op) in ops.iter().enumerate() {
        let before = r.protocol.adaptive.snapshot();
        let ring = r.session.control_ring;
        let head = r.session.control_head;
        let tail = r.session.control_tail;
        let priority = r.protocol.priority_control;
        let seq = r.protocol.tx_seq;
        let air = r.protocol.air_report_seq;
        let turn = r.protocol.control_turn;
        let frames = p.radio_frames.len();
        let effects = p.effects.len();
        let drops = r.protocol.stat_control_ring_drop;
        if matches!(op.1, TxCheck::SyncBeforePriority) {
            let now = g64(&op.0 .0[8..]) as u32;
            let frame = g32(&op.0 .0[240..]);
            assert_eq!(r.membership.role, Role::Coordinator);
            assert_eq!(r.membership.state, State::Active);
            assert_eq!(r.membership.slot_index, 1);
            assert_eq!(r.protocol.term, before.term);
            assert_eq!(before.leader_id, r.membership.node_id);
            assert!(before.transition_done);
            assert!((r.protocol.convergence_until_ms.wrapping_sub(now) as i32) > 0);
            assert!(r.adaptive_converging(now));
            assert_eq!(frame % 8, r.membership.slot_index as u32);
            assert!((frame / 8).is_multiple_of(2));
            assert!(!frame.is_multiple_of(10));
            assert_ne!(op.0 .0[254], 0);
            assert!(r.protocol.priority_pending);
        }
        let ret = run_full(&mut r, &mut p, op.0);
        match op.1 {
            TxCheck::None => {}
            TxCheck::Return(expected) => assert_eq!(ret, Some(expected), "operation {i}"),
            TxCheck::PriorityRetained => {
                assert!(r.protocol.priority_pending);
                assert_eq!(r.protocol.priority_control, priority);
                assert_eq!(r.protocol.tx_seq, seq);
                assert_eq!(p.radio_frames.len(), frames + 1);
            }
            TxCheck::ForwardUnavailable => {
                assert_eq!(
                    r.protocol.adaptive.forward_queue_len(),
                    usize::from(before.queue_count)
                );
                assert_eq!(r.protocol.adaptive.snapshot().cache, before.cache);
                assert_eq!(
                    r.protocol
                        .adaptive
                        .forward_queue_entry(0)
                        .unwrap()
                        .header
                        .packet_type,
                    19
                );
                assert_eq!(r.protocol.control_turn, turn.wrapping_add(1));
                assert_eq!(p.radio_frames.len(), frames);
            }
            TxCheck::ForwardConsumed { rf } => {
                assert_eq!(
                    r.protocol.adaptive.forward_queue_len(),
                    usize::from(before.queue_count) - 1
                );
                assert_eq!(r.protocol.adaptive.snapshot().cache, before.cache);
                assert_eq!(p.radio_frames.len(), frames + usize::from(rf));
            }
            TxCheck::OwnRetained => {
                assert_eq!(r.session.control_tail, tail);
                assert_eq!(r.session.control_head, head);
                assert_eq!(r.session.control_ring[usize::from(tail)].data[1], 6);
                assert_eq!(r.session.control_ring[usize::from(tail)].data[3], seq);
                assert_eq!(r.protocol.tx_seq, seq.wrapping_add(1));
            }
            TxCheck::Coalesced => {
                assert_eq!(ret, Some(0));
                let t = op.0 .0[1];
                let mut first = tail;
                while ring[usize::from(first)].data[1] != t {
                    first = (first + 1) % 32;
                }
                let entry = &r.session.control_ring[usize::from(first)];
                assert_eq!(entry.data[3], ring[usize::from(first)].data[3]);
                assert_eq!(r.protocol.tx_seq, seq);
                assert_eq!(r.session.control_head, head);
                assert_eq!(r.session.control_tail, tail);
                assert_eq!(r.protocol.stat_control_ring_drop, drops);
                assert_eq!(
                    r.protocol.adaptive.snapshot().publish_revision,
                    before.publish_revision
                );
                if t == 22 {
                    assert_eq!(&entry.data[13..17], &ring[usize::from(first)].data[13..17]);
                }
                let len = u16::from_le_bytes([op.0 .0[34], op.0 .0[35]]) as usize;
                assert_eq!(entry.len as usize, 9 + len);
                let mut expected = ring[usize::from(first)];
                expected.data[9..9 + len].copy_from_slice(&op.0 .0[36..36 + len]);
                if t == 22 {
                    expected.data[13..17].copy_from_slice(&ring[usize::from(first)].data[13..17]);
                }
                expected.data[7..9].copy_from_slice(&(len as u16).to_le_bytes());
                expected.len = (9 + len) as u8;
                assert_eq!(*entry, expected);
                let mut next = (first + 1) % 32;
                while next != head {
                    assert_eq!(
                        r.session.control_ring[usize::from(next)],
                        ring[usize::from(next)]
                    );
                    next = (next + 1) % 32;
                }
            }
            TxCheck::NewMembership => {
                assert_eq!(ret, Some(0));
                assert_eq!(before.local_id, before.leader_id);
                assert_ne!(before.term, 0);
                let len = u16::from_le_bytes([op.0 .0[34], op.0 .0[35]]) as usize;
                let revision = before.publish_revision.wrapping_add(1);
                let mut expected = ring[usize::from(head)];
                let h = Header {
                    version: 5,
                    packet_type: 22,
                    src_id: r.membership.node_id,
                    seq,
                    ttl: 2,
                    flags: 0,
                    talk_channel: r.talk_channel,
                    payload_len: len as u16,
                };
                expected.len = wire::encode(&h, &op.0 .0[36..36 + len], &mut expected.data, 200)
                    .unwrap() as u8;
                expected.data[13..17].copy_from_slice(&revision.to_le_bytes());
                assert_eq!(r.session.control_ring[usize::from(head)], expected);
                assert_eq!(expected.len as usize, 9 + len);
                assert_eq!(
                    wire::decode(&expected.data[..usize::from(expected.len)], 200)
                        .unwrap()
                        .payload
                        .len(),
                    len
                );
                for (i, old) in ring.iter().enumerate() {
                    if i != usize::from(head) {
                        assert_eq!(r.session.control_ring[i], *old);
                    }
                }
                assert_eq!(r.session.control_head, (head + 1) % 32);
                assert_eq!(r.session.control_tail, tail);
                assert_eq!(r.protocol.tx_seq, seq.wrapping_add(1));
                assert_eq!(r.protocol.adaptive.snapshot().publish_revision, revision);
            }
            TxCheck::CapacityRejected => {
                assert_eq!(ret, Some(-105));
                assert_eq!(r.session.control_ring, ring);
                assert_eq!(r.session.control_head, head);
                assert_eq!(r.session.control_tail, tail);
                assert_eq!(r.protocol.tx_seq, seq);
                assert_eq!(
                    r.protocol.adaptive.snapshot().publish_revision,
                    before.publish_revision
                );
                assert_eq!(r.protocol.stat_control_ring_drop, drops.wrapping_add(1));
            }
            TxCheck::Prepared => {
                let shared = r.protocol.adaptive.snapshot();
                assert!(shared.prepared);
                assert!(!shared.committed);
                assert_eq!(shared.pending.candidate_id, 2);
                assert_eq!(r.membership.role, Role::Participant);
            }
            TxCheck::ConvergenceReady => {
                let shared = r.protocol.adaptive.snapshot();
                assert!(shared.transition_done);
                assert_eq!((shared.local_id, shared.leader_id, shared.term), (2, 2, 8));
                assert_eq!(r.membership.role, Role::Coordinator);
                assert_eq!(
                    (
                        r.membership.coordinator_id,
                        r.membership.slot_index,
                        r.protocol.term
                    ),
                    (2, 1, 8)
                );
                assert!(r.adaptive_converging(p.inner.now as u32));
                assert!(p.effects[effects..].contains(&A::ClockSource(true)));
            }
            TxCheck::Radio { packet_type, seq } => {
                assert_eq!(p.radio_frames.len(), frames + 1);
                let packet = wire::decode(p.radio_frames.last().unwrap(), 200).unwrap();
                assert_eq!(packet.header.packet_type, packet_type);
                if let Some(seq) = seq {
                    assert_eq!(packet.header.seq, seq);
                }
                if packet_type == 15 {
                    assert_eq!(packet.header.src_id, 0);
                    assert_eq!(
                        packet.header.ttl,
                        if packet.header.flags & 2 != 0 { 1 } else { 2 }
                    );
                }
            }
            TxCheck::Topology { success } => {
                let frame = p.radio_frames.last().unwrap();
                assert_eq!(frame[1], 14);
                assert_eq!(g32(&frame[9..]), air.wrapping_add(1));
                assert_eq!(
                    r.protocol.air_report_seq,
                    if success { air.wrapping_add(1) } else { air }
                );
                assert_eq!(
                    r.session.control_tail,
                    if success { (tail + 1) % 32 } else { tail }
                );
            }
            TxCheck::RequestFrame => {
                let frame = p.radio_frames.last().unwrap();
                assert_eq!(frame[1], 23);
                assert_eq!(g32(&frame[17..]), p.frame);
            }
            TxCheck::SyncDeferred => {
                assert_eq!(p.radio_frames.len(), frames);
                assert_eq!(r.protocol.tx_seq, seq);
                assert_eq!(r.protocol.priority_control, priority);
                assert!(r.protocol.priority_pending);
                assert_eq!(r.protocol.control_turn, turn);
            }
            TxCheck::SyncBeforePriority => {
                assert_eq!(p.radio_frames.len(), frames + 1);
                assert!(r.protocol.priority_pending);
                assert_eq!(r.protocol.priority_control, priority);
                let sync = wire::SyncV3::decode(&p.radio_frames.last().unwrap()[9..]).unwrap();
                assert_eq!(sync.frame_counter, p.frame);
                assert_eq!(sync.phase_us, p.clock_phase);
                assert_eq!(r.protocol.last_sync_time, p.inner.now as u32);
            }
        }
    }
    assert!(p.radio_frames.iter().any(|frame| frame.len() == 209));
    std::eprintln!(
        "full TX inputs={} radio attempts={}",
        ops.len(),
        p.radio_frames.len()
    );
}
