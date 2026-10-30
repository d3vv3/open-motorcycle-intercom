//! Separate 98-function replay; retains the complete 93-function audio corpus.
use super::*;
use crate::protocol_audio::{AudioEffect, AudioPlatform};
use crate::protocol_rx::{FullRxPlatform, RxEffect, RxPlatform};

#[path = "protocol_differential.rs"]
mod protocol_differential;

impl AudioPlatform for Joint {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_effect(effect)
    }
}
impl RxPlatform for Joint {
    fn rx_io(&mut self, effect: RxEffect) -> i32 {
        assert!(self.full_rx);
        match effect {
            RxEffect::InitWork => {
                self.inner.e(62);
                self.rx_pending = false;
            }
            RxEffect::AttachCallback => self.inner.e(63),
            RxEffect::CancelWork => {
                self.inner.e(64);
                self.rx_pending = false;
            }
            RxEffect::SubmitWork => {
                let result = if self.rx_pending && self.rx_submit_result > 0 {
                    0
                } else {
                    self.rx_submit_result
                };
                self.inner.e(65);
                p32(&mut self.inner.bytes, result as u32);
                if result > 0 {
                    self.rx_pending = true;
                }
                return result;
            }
        }
        0
    }
}
fn rx_snapshot(v: &mut Vec<u8>, r: &Runtime) {
    let s = &r.rx_state;
    v.extend([s.head, s.tail, s.work_pending as u8]);
    p32(v, s.count);
    p32(v, s.drop as u32);
    for entry in s.slots {
        v.extend(entry.data);
        v.extend([entry.len, entry.rssi as u8]);
        p64(v, entry.received_us as u64);
    }
}
fn run_rx(r: &mut Runtime, p: &mut Joint, token: &mut Option<IngressToken>, op: JointOp) {
    let x = op.0;
    if x[0] < 90 {
        run_audio(r, p, token, op);
        if x[0] == 0 {
            p.rx_pending = false;
        }
    } else {
        configure(p, &x);
        p.rx_submit_result = g32(&x[4..]) as i32;
        let data = &x[27..27 + usize::from(x[1])];
        let receipt = g64(&x[16..]);
        let mut io = FullRxPlatform(p);
        match x[0] {
            90 | 91 => {
                let slots = r.rx_state.slots;
                let cursors = (r.rx_state.head, r.rx_state.tail);
                let counters = (r.rx_state.count, r.rx_state.drop);
                if x[0] == 90 {
                    r.rx_init(&mut io);
                    assert_eq!((r.rx_state.head, r.rx_state.tail), cursors);
                } else {
                    r.rx_stop(&mut io);
                    assert_eq!((r.rx_state.head, r.rx_state.tail), (0, 0));
                }
                assert!(!r.rx_state.work_pending);
                assert_eq!(r.rx_state.slots, slots);
                assert_eq!((r.rx_state.count, r.rx_state.drop), counters);
            }
            92 | 95 => {
                let before = crate::protocol_rx::RxState {
                    slots: r.rx_state.slots,
                    head: r.rx_state.head,
                    tail: r.rx_state.tail,
                    work_pending: r.rx_state.work_pending,
                    count: r.rx_state.count,
                    drop: r.rx_state.drop,
                };
                let accepted = r.rx_publish(data, x[2] as i8, receipt, &mut io).unwrap();
                let full = (before.head + 1) % 8 == before.tail;
                assert_eq!(accepted, !full, "RX capacity is seven entries");
                assert_eq!(r.rx_state.tail, before.tail);
                assert_eq!(r.rx_state.count, before.count);
                assert_eq!(r.rx_state.drop, before.drop.wrapping_add(i32::from(full)));
                assert_eq!(
                    r.rx_state.head,
                    if full {
                        before.head
                    } else {
                        (before.head + 1) % 8
                    }
                );
                for (index, entry) in r.rx_state.slots.iter().enumerate() {
                    let previous = &before.slots[index];
                    if !full && index == usize::from(before.head) {
                        assert_eq!(&entry.data[..data.len()], data);
                        assert_eq!(&entry.data[data.len()..], &previous.data[data.len()..]);
                        assert_eq!(
                            (entry.len, entry.rssi, entry.received_us),
                            (data.len() as u8, x[2] as i8, receipt)
                        );
                    } else {
                        assert_eq!(entry.data, previous.data);
                        assert_eq!(
                            (entry.len, entry.rssi, entry.received_us),
                            (previous.len, previous.rssi, previous.received_us)
                        );
                    }
                }
            }
            96 => {
                let mut scratch = [0; 209];
                scratch[..data.len()].copy_from_slice(data);
                r.rx_publish(&scratch[..data.len()], x[2] as i8, receipt, &mut io)
                    .unwrap();
                scratch.fill(0xa5);
                assert!(scratch.iter().all(|byte| *byte == 0xa5));
            }
            93 => {
                let counters = r.audio_state.counters;
                let count = r.rx_state.count;
                let malformed = data.len() < 9
                    || data.len() > 209
                    || usize::from(u16::from_le_bytes([data[7], data[8]])) != data.len() - 9;
                let wrong_channel_or_version = !malformed
                    && (data[0] != 5 || !(1..=3).contains(&data[6]) || data[6] != r.talk_channel);
                let effect_bytes = io.0.inner.bytes.len();
                r.rx_process_packet(data, x[2] as i8, receipt, &mut io)
                    .unwrap();
                // Valid envelopes can still fail downstream audio payload validation.
                if malformed || wrong_channel_or_version {
                    assert_eq!(
                        r.audio_state.counters.rf_rx_malformed,
                        counters.rf_rx_malformed.wrapping_add(u32::from(malformed))
                    );
                }
                assert_eq!(
                    r.audio_state.counters.rf_rx_version_drop,
                    counters
                        .rf_rx_version_drop
                        .wrapping_add(u32::from(wrong_channel_or_version))
                );
                assert_eq!(
                    r.rx_state.count,
                    count.wrapping_add(u32::from(!malformed && !wrong_channel_or_version))
                );
                if malformed || wrong_channel_or_version {
                    assert_eq!(
                        io.0.inner.bytes.len(),
                        effect_bytes,
                        "invalid RF envelope must not emit platform effects"
                    );
                }
            }
            94 => {
                io.0.rx_pending = false;
                r.rx_work_handler(&mut io).unwrap();
                assert_eq!(
                    r.rx_state.head, r.rx_state.tail,
                    "worker drains the pending ring"
                );
                assert!(!r.rx_state.work_pending);
            }
            _ => panic!("unknown RX operation {}", x[0]),
        }
        snapshot_joint(r, io.0, false);
        audio_snapshot(&mut io.0.inner.bytes, r);
    }
    assert_eq!(r.rx_state.work_pending, p.rx_pending);
    rx_snapshot(&mut p.inner.bytes, r);
}
fn rf_op(code: u8, data: &[u8], receipt: i64, now: i64) -> JointOp {
    assert!(data.len() <= 229);
    let mut op = JointOp::new(code, now)
        .arg(1, data.len() as u8)
        .arg(2, (-60i8) as u8)
        .scalar(4, 1);
    op.0[16..24].copy_from_slice(&receipt.to_le_bytes());
    op.0[27..27 + data.len()].copy_from_slice(data);
    op
}
fn outer(t: u8, src: u8, seq: u8, payload: &[u8]) -> Vec<u8> {
    let h = Header {
        version: 5,
        packet_type: t,
        src_id: src,
        seq,
        ttl: 2,
        flags: 0,
        talk_channel: 1,
        payload_len: payload.len() as u16,
    };
    let mut data = [0; 209];
    let len = wire::encode(&h, payload, &mut data, 200).unwrap();
    data[..len].to_vec()
}
fn rx_fixtures() -> Vec<JointOp> {
    let mut ops = audio_fixtures();
    // Independent new RX fixtures after the unchanged 2,428-operation baseline.
    audio_setup(&mut ops, 2, 6000);
    ops.push(rf_op(90, &[], 0, 6000));
    let unknown = outer(255, 3, 1, &[0xa5; 200]);
    // Fill all seven usable slots, overflow, then drain; init does not clear RX.
    for i in 0..8 {
        ops.push(rf_op(92, &unknown, 6_000_001 + i, 6000).arg(3, i as u8));
    }
    ops.push(rf_op(90, &[], 0, 6000));
    ops.push(rf_op(94, &[], 0, 6001));
    // A shorter overwrite preserves old slot suffix bytes, signed RSSI and time.
    ops.push(rf_op(95, &outer(255, 3, 2, &[]), 6_001_099, 6001).arg(2, 128));
    ops.push(rf_op(96, &outer(255, 3, 3, &[0x11, 0x22]), 6_001_199, 6001).arg(2, 127));
    ops.push(rf_op(94, &[], 0, 6001));
    for bytes in [&[][..], &[5][..], &[5; 8][..]] {
        ops.push(rf_op(93, bytes, 0, 6002));
    }
    let valid = outer(255, 3, 3, &[1, 2]);
    ops.push(rf_op(93, &valid[..10], 0, 6002));
    let mut trailing = valid.clone();
    trailing.push(0);
    ops.push(rf_op(93, &trailing, 0, 6002));
    let mut bad = valid.clone();
    bad[7] = 201;
    ops.push(rf_op(93, &bad, 0, 6002));
    let mut over_capacity = vec![0; 210];
    over_capacity[..9].copy_from_slice(&[5, 255, 3, 3, 2, 0, 1, 201, 0]);
    ops.push(rf_op(93, &over_capacity, 0, 6002));
    for version in [0, 4, 6] {
        let mut packet = valid.clone();
        packet[0] = version;
        ops.push(rf_op(93, &packet, 0, 6002));
    }
    for channel in [0, 1, 2, 3, 4, 255] {
        let mut packet = valid.clone();
        packet[6] = channel;
        ops.push(rf_op(93, &packet, 0, 6002));
    }
    // Source fallback grant/release authority and malformed-size fallthrough.
    for src in [3, 1] {
        ops.push(rf_op(
            93,
            &outer(9, src, 4, &[1, 3, 0, c::node_bit(2), 0]),
            0,
            6003,
        ));
        ops.push(rf_op(93, &outer(10, src, 5, &[1, 3, 0]), 0, 6003));
        ops.push(rf_op(93, &outer(9, src, 6, &[1, 3]), 0, 6003));
    }
    // Actual adaptive control presence/auth/sequence ordering through outer RX.
    let mut control = JointOp::packet(Message::Topology(topo(3, 1)), 3, 40, 6004);
    control.0[32] = 0;
    let len = 9 + usize::from(header(&control.0).payload_len);
    let packet = &control.0[27..27 + len];
    ops.push(rf_op(92, packet, 6_004_000, 6004));
    ops.push(rf_op(92, packet, 6_004_000, 6004));
    ops.push(rf_op(94, &[], 0, 6004));
    let mut invalid = packet.to_vec();
    invalid[5] = 128;
    ops.push(rf_op(93, &invalid, 6_004_000, 6004));
    // Receipt cutoff equality is admitted; one microsecond earlier is gated.
    ops.push(audio_op(63, &[], 6010).arg(1, 0));
    ops.push(audio_op(63, &[], 6011).arg(1, 1));
    ops.push(audio_op(75, &[1, 3, 0, c::node_bit(2), 0], 6011));
    for (src, seq, receipt) in [(3, 50, 6_011_000), (3, 51, 6_010_999), (2, 52, 6_011_000)] {
        let mut packet = outer(13, src, seq, &bundle(u16::from(seq), 2, true));
        packet[5] = 1 | 4;
        ops.push(rf_op(92, &packet, receipt, 6011));
    }
    ops.push(rf_op(94, &[], 0, 6011));
    let bad_bundle = outer(13, 3, 53, &[0; 8]);
    ops.push(rf_op(93, &bad_bundle, 6_011_000, 6011));
    // Queued work cancellation preserves storage; stopped callback remains valid.
    ops.push(rf_op(92, &valid, 6_012_000, 6012));
    ops.push(rf_op(91, &[], 0, 6012));
    ops.push(rf_op(94, &[], 0, 6012));
    ops.push(rf_op(92, &valid, 6_012_001, 6012));
    ops.push(rf_op(90, &[], 0, 6012));
    ops.push(rf_op(94, &[], 0, 6012));
    for result in [0, -5, 1, 0, -5] {
        ops.push(rf_op(92, &valid, 6_013_000, 6013).scalar(4, result as u32));
    }
    ops.push(rf_op(94, &[], 0, 6013));
    // Serialized producer/owner alternation, never a reentrant mutable owner.
    let mut seed = 0x1327_acedu32;
    for i in 0..512 {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let code = match seed % 13 {
            0 => 91,
            1 => 90,
            2..=5 => 94,
            _ => 92,
        };
        ops.push(
            rf_op(
                code,
                if code == 92 { &valid } else { &[] },
                6_020_000 + i,
                6020,
            )
            .arg(2, seed as u8)
            .arg(3, (seed >> 8) as u8),
        );
    }
    ops.push(rf_op(94, &[], 0, 6020));
    ops
}

#[test]
fn runtime_lifecycle_dispatches_owned_rx_through_full_rx_platform() {
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    p.full_rx = true;
    p.inner.now = 7000;
    let mut r = Runtime::default();
    let mut io = FullRxPlatform(&mut p);
    assert_eq!(io.integration(), Integration::ActorOracle);
    assert_eq!(r.init(&mut io), 0);
    // Audio work binding precedes RX work binding and callback registration.
    assert!(io
        .0
        .inner
        .bytes
        .starts_with(&[b'E', 54, b'E', 55, b'E', 62, b'E', 63]));
    assert_eq!(r.rx_state().cursors(), (0, 0));
    assert!(!r.rx_state().work_pending());
    assert!(!io.0.rx_pending);
    assert_eq!(r.snapshot().membership.state, State::Idle);

    let packet = outer(255, 3, 1, &[0xa5, 0x5a]);
    let captured_us = io.now_us() + 99;
    io.0.inner.now += 10;
    io.0.inner.bytes.clear();
    for i in 0..8 {
        assert_eq!(
            r.rx_publish(&packet, -60, captured_us + i, &mut io),
            Ok(i < 7)
        );
    }
    assert_eq!(r.rx_state().cursors(), (7, 0));
    assert!(r.rx_state().work_pending());
    assert!(io.0.rx_pending);
    assert_eq!(r.rx_state().drop_count(), 1);
    assert_eq!(r.rx_state().storage()[0].received_us, captured_us);
    assert_eq!(r.rx_state().storage()[0].rssi, -60);
    assert_eq!(r.rx_state().storage()[0].len as usize, packet.len());
    assert_eq!(&r.rx_state().storage()[0].data[..packet.len()], packet);
    let storage = *r.rx_state().storage();
    let count = r.rx_state().count();
    let drops = r.rx_state().drop_count();

    // Source init binds work again, but does not clear cursors or stale storage.
    io.0.inner.bytes.clear();
    assert_eq!(r.init(&mut io), 0);
    assert_eq!(
        io.0.inner.bytes,
        [b'E', 54, b'E', 55, b'E', 62, b'E', 63, b'E', 20, 9, 8, 7, 6, 5]
    );
    assert_eq!(r.rx_state().cursors(), (7, 0));
    assert_eq!(*r.rx_state().storage(), storage);
    assert_eq!(r.rx_state().count(), count);
    assert_eq!(r.rx_state().drop_count(), drops);
    assert!(!r.rx_state().work_pending());
    assert!(!io.0.rx_pending);

    // Make room through lifecycle STOP, then queue pending work for another STOP.
    r.stop(&mut io);
    assert_eq!(r.rx_state().cursors(), (0, 0));
    assert_eq!(*r.rx_state().storage(), storage);
    assert_eq!(r.rx_publish(&packet, -59, captured_us, &mut io), Ok(true));
    assert!(r.rx_state().work_pending());
    assert!(io.0.rx_pending);
    let storage = *r.rx_state().storage();
    io.0.inner.bytes.clear();
    r.stop(&mut io);
    assert_eq!(
        io.0.inner.bytes,
        [b'E', 31, 0, b'E', 31, 1, b'E', 31, 2, b'E', 64, b'E', 56]
    );
    assert_eq!(r.rx_state().cursors(), (0, 0));
    assert!(!r.rx_state().work_pending());
    assert!(!io.0.rx_pending);
    assert_eq!(*r.rx_state().storage(), storage);
    assert_eq!(r.rx_state().count(), count);
    assert_eq!(r.rx_state().drop_count(), drops);
    assert_eq!(r.snapshot().membership.state, State::Idle);
    assert_eq!(io.integration(), Integration::ActorOracle);
    assert!(!io
        .0
        .runtime_effects
        .iter()
        .any(|e| matches!(e, Effect::RxInit | Effect::RxStop)));
}

#[test]
fn outer_rx_rejects_foreign_grants_and_stale_audio_receipts() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    let mut token = None;
    let mut ops = Vec::new();
    audio_setup(&mut ops, 2, 6000);
    for op in ops {
        run_rx(&mut r, &mut p, &mut token, op);
    }
    r.audio_apply_speaker_grant(1, [2, 0], [0, 0]);
    run_rx(
        &mut r,
        &mut p,
        &mut token,
        rf_op(93, &outer(9, 3, 4, &[1, 3, 0, 2, 0]), 0, 6003),
    );
    assert_eq!(r.session.active_speaker_ids, [2, 0]);
    run_rx(
        &mut r,
        &mut p,
        &mut token,
        rf_op(93, &outer(9, 1, 5, &[1, 3, 0, 2, 0]), 0, 6003),
    );
    assert_eq!(r.session.active_speaker_ids, [3, 0]);
    r.audio_state.boundary.enabled = true;
    r.audio_state.boundary.receipt_cutoff_us = 6_011_000;
    let delivered = r.audio_state.counters.spi_out_ok;
    let mut packet = outer(13, 3, 50, &bundle(50, 2, true));
    packet[5] = 1;
    p.inner.now = 6011;
    p.bridge_result = 0;
    r.rx_process_packet(&packet, -60, 6_010_999, &mut FullRxPlatform(&mut p))
        .unwrap();
    assert_eq!(r.audio_state.counters.spi_out_ok, delivered);
    let mut packet = outer(13, 3, 51, &bundle(51, 2, true));
    packet[5] = 1;
    r.rx_process_packet(&packet, -60, 6_011_000, &mut FullRxPlatform(&mut p))
        .unwrap();
    assert_eq!(r.audio_state.counters.spi_out_ok, delivered + 1);
}

#[test]
fn full_rx_corpus_runs_owned_rx_audio_adaptive_membership_tx() {
    let ops = rx_fixtures();
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    p.full_rx = true;
    let mut r = Runtime::default();
    let mut token = None;
    for op in &ops {
        run_rx(&mut r, &mut p, &mut token, *op);
    }
    std::eprintln!("authored full-RX corpus: {} operations", ops.len());
}
