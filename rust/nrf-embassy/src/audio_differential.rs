//! Separate full-audio replay. Historical mock corpora remain separate tests.
use super::*;
use crate::protocol_audio::{AudioEntry, IngressToken};
use openhelmet_wire::audio_bundle as b;

#[path = "rx_differential.rs"]
mod rx_differential;

fn audio_snapshot(v: &mut Vec<u8>, r: &Runtime) {
    let s = &r.audio_state;
    v.push(s.boundary.enabled as u8);
    p32(v, s.boundary.epoch);
    p64(v, s.boundary.receipt_cutoff_us as u64);
    v.extend([
        s.ingress_enabled as u8,
        s.ingress_read,
        s.ingress_write,
        s.ingress_used,
        s.work_pending as u8,
    ]);
    for entry in s.ingress.iter().chain(s.tx.iter()) {
        v.extend(entry.data);
        v.extend([entry.len, entry.audio_flags, entry.packet_type]);
    }
    v.extend([
        s.tx_head,
        s.tx_tail,
        s.relay_contention_turn as u8,
        s.deferred_pending as u8,
    ]);
    v.extend(s.deferred_seq.to_le_bytes());
    for seq in s.spi_seq {
        v.extend(seq.last.to_le_bytes());
        v.push(seq.initialized as u8);
    }
    for seq in s.rf_seq {
        v.extend(seq.last.to_le_bytes());
        v.push(seq.initialized as u8);
    }
    let c = s.counters;
    for n in [
        c.ingress_purge_drop,
        c.tx_purge_drop,
        c.tx_count,
        c.tx_starvation,
        c.tx_queue_drain,
        c.audio_fwd,
        c.tx_overwrite,
        c.spi_audio_in,
        c.ingress_inactive_drop,
        c.ingress_msgq_drop,
        c.tx_ring_drop,
        c.relay_ring_drop,
        c.rf_audio_try,
        c.rf_audio_ok,
        c.rf_audio_fail,
        c.rf_rx_audio_ok,
        c.rf_rx_malformed,
        c.rf_rx_version_drop,
        c.rf_rx_self_drop,
        c.rf_rx_duplicate_drop,
        c.rf_rx_inactive_drop,
        c.spi_out_ok,
        c.spi_out_drop,
        c.bundle_tx,
        c.bundle_rx,
        c.bundle_bad,
        c.prev1_forwarded,
        c.prev2_forwarded,
        c.prev1_stripped,
        c.prev2_stripped,
        c.bundle_late_drop,
        c.bundle_max_bytes,
        c.local_deferred_recovery,
        c.spi_in_frames,
        c.spi_in_gap_evt,
        c.spi_in_gap_fr,
        c.spi_in_reset_evt,
        c.rf_tx_frames,
        c.rf_rx_frames,
        c.rf_rx_gap_evt,
        c.rf_rx_gap_fr,
        c.rf_rx_reset_evt,
        c.spi_out_frames,
    ] {
        p32(v, n);
    }
    p32(v, s.last_audio_in_time);
    for n in s.phase_hist {
        p32(v, n);
    }
    v.push(s.tx_depth_dbg);
}
fn configure(p: &mut Joint, x: &[u8; 256]) {
    p.inner.now = g64(&x[8..]);
    p.inner.logs = x[255] != 0;
    p.frame = g32(&x[240..]);
    p.queue_result = g32(&x[244..]) as i32;
    p.priority_result = g32(&x[248..]) as i32;
    p.clock_available = x[254] != 0;
    p.clock_phase = u16::from_le_bytes([x[252], x[253]]);
    p.slot_remaining = g32(&x[16..]);
    p.time_to_slot = g32(&x[20..]) as i32;
    p.bridge_result = g32(&x[24..]) as i32;
}
fn run_audio(
    r: &mut Runtime,
    p: &mut Joint,
    token: &mut Option<IngressToken>,
    op: JointOp,
) -> Option<i32> {
    let x = op.0;
    configure(p, &x);
    if x[0] < 60 {
        let ret = run_full(r, p, op);
        audio_snapshot(&mut p.inner.bytes, r);
        return ret;
    }
    let data = if x[0] == 86 {
        &[][..]
    } else {
        &x[36..36 + usize::from(x[7])]
    };
    let mut handled = false;
    let mut io = CombinedFullTxPlatform(p);
    let ret = match x[0] {
        60 => {
            r.audio_init(&mut io);
            None
        }
        61 => {
            r.audio_cancel_work(&mut io);
            None
        }
        62 => {
            r.audio_set_ingress_enabled(x[1] != 0, x[2] != 0);
            None
        }
        63 => {
            r.audio_set_local_enabled(x[1] != 0, &mut io);
            None
        }
        64 => Some(r.audio_send_v2(data, &mut io)),
        65 => {
            r.audio_ingress_work_handler(&mut io);
            None
        }
        66 => {
            *token = Some(r.audio_capture_ingress());
            None
        }
        67 => Some(r.audio_admit_ingress(
            token.expect("captured producer").prepare(data).unwrap(),
            &mut io,
        )),
        68 => {
            r.audio_slot_tx_handler(x[1], io.0.frame, &mut io);
            None
        }
        69 => {
            let h = header(&x);
            let len = usize::from(h.payload_len);
            let mut packet = [0; 209];
            // Raw readable backing, not outer RX validation. Header bytes remain
            // untouched, including legacy/unknown types and declared lengths.
            packet[..9].copy_from_slice(&x[27..36]);
            packet[9..9 + len].copy_from_slice(&x[36..36 + len]);
            handled = r
                .audio_process_rx_packet(&packet[..9 + len], x[2] as i8, g64(&x[232..]), &mut io)
                .unwrap();
            None
        }
        70 => {
            r.audio_purge_tx_ring();
            None
        }
        71 => {
            r.audio_clear_relay_ring();
            None
        }
        72 => {
            r.audio_clear_speaker_activity();
            None
        }
        73 => {
            r.audio_clear_speaker_grants();
            None
        }
        74 => {
            r.audio_clear_bitmaps();
            None
        }
        75 => {
            r.audio_apply_speaker_grant(x[36], [x[37], x[38]], [x[39], x[40]]);
            None
        }
        76 => {
            r.audio_apply_speaker_release(x[36], [x[37], x[38]]);
            None
        }
        77 => {
            let map = wire::SlotMap::decode(&x[36..36 + wire::SlotMap::SIZE]).unwrap();
            r.audio_apply_slot_map_speakers(&c::SlotMap {
                slot_count: map.slot_count,
                slot_ids: map.slot_ids,
                active_speaker_count: map.active_speaker_count,
                active_speaker_ids: map.active_speaker_ids,
                relay_masks: map.relay_masks,
            });
            None
        }
        78 => {
            r.audio_update_speaker_grants(&mut io);
            None
        }
        79 => {
            r.audio_expire_speakers(io.0.inner.now as u32, &mut io);
            None
        }
        80 => {
            r.audio_reset_rf_tracker(x[1]);
            None
        }
        81 => {
            r.audio_reset_all_rf_trackers();
            None
        }
        82 => Some(r.audio_send_legacy(data, x[1])),
        83 => {
            let mut entry = AudioEntry {
                len: x[7],
                audio_flags: x[1],
                packet_type: x[2],
                ..AudioEntry::default()
            };
            entry.data[..data.len()].copy_from_slice(data);
            Some(r.audio_test_process(&entry, &mut io))
        }
        84 => {
            r.audio_drain_ingress(&mut io);
            None
        }
        85 => {
            r.audio_test_helpers(data, x[1], g32(&x[4..]) as u16, &mut io);
            None
        }
        86 => {
            r.audio_state.boundary.epoch = g32(&x[4..]);
            None
        }
        87 => Some(r.audio_test_transmit_probe(data, x[1], x[2], g32(&x[226..]), &mut io)),
        88 => {
            let c = &mut r.audio_state.counters;
            c.bundle_bad = u32::MAX;
            c.ingress_purge_drop = u32::MAX;
            c.tx_purge_drop = u32::MAX;
            c.tx_ring_drop = u32::MAX;
            c.ingress_msgq_drop = u32::MAX;
            c.tx_overwrite = u32::MAX;
            None
        }
        _ => panic!("unknown audio operation {}", x[0]),
    };
    if let Some(ret) = ret {
        io.0.inner.e(52);
        p32(&mut io.0.inner.bytes, ret as u32);
    }
    snapshot_joint(r, io.0, handled);
    audio_snapshot(&mut io.0.inner.bytes, r);
    ret
}
fn bundle(seq: u16, history: usize, active: bool) -> Vec<u8> {
    let current = [seq as u8; 48];
    let prev1 = [seq.wrapping_sub(1) as u8; 48];
    let prev2 = [seq.wrapping_sub(2) as u8; 48];
    let flags =
        u8::from(active) | if history >= 1 { 6 } else { 0 } | if history >= 2 { 24 } else { 0 };
    let view = b::BundleView {
        current_seq: seq,
        stream_id: 7,
        flags,
        codec: 2,
        current_data: &current,
        previous1_data: if history >= 1 { &prev1 } else { &[] },
        previous2_data: if history >= 2 { &prev2 } else { &[] },
    };
    let mut bytes = [0; 200];
    let len = b::audio_bundle_encode(&view, &mut bytes);
    bytes[..len].to_vec()
}
fn audio_op(code: u8, data: &[u8], now: i64) -> JointOp {
    assert!(data.len() <= 200);
    let mut op = JointOp::new(code, now)
        .arg(7, data.len() as u8)
        .scalar(16, 2000)
        .scalar(20, 19000);
    op.0[36..36 + data.len()].copy_from_slice(data);
    op
}
fn receive(src: u8, outer_seq: u8, seq: u16, history: usize, now: i64) -> JointOp {
    let data = bundle(seq, history, true);
    let mut op = audio_op(69, &data, now).arg(2, (-60i8) as u8);
    op.0[27..36].copy_from_slice(&[5, 13, src, outer_seq, 2, 7, 1, data.len() as u8, 0]);
    op.0[232..240].copy_from_slice(&(now * 1000).to_le_bytes());
    op
}
fn audio_setup(ops: &mut Vec<JointOp>, local: u8, now: i64) {
    setup(ops, local, now);
    ops.push(audio_op(60, &[], now));
    ops.push(audio_op(62, &[], now).arg(1, 1));
    ops.push(audio_op(63, &[], now).arg(1, 1));
}
const RELAY_QUEUED: u8 = 1;
const RELAY_SENT: u8 = 2;
const RELAY_RF_FAILED: u8 = 3;
const RELAY_NO_RETRY: u8 = 4;
const RELAY_EXPIRED: u8 = 5;

fn relay_slot_fixtures(ops: &mut Vec<JointOp>) {
    audio_setup(ops, 2, 2300);
    let mut sync = JointOp::packet(
        Message::SyncV3(wire::SyncV3 {
            leader: remote(1),
            leader_id: 1,
            member_count: 3,
            term: 7,
            frame_counter: 115,
            phase_us: 0,
            relay_depth: 0,
        }),
        1,
        49,
        2300,
    );
    // RX latency subtraction establishes the frame start at 2,300,000 us.
    sync.0[232..240].copy_from_slice(&2_300_300i64.to_le_bytes());
    ops.push(sync);
    ops.push(audio_op(75, &[1, 3, 0, c::node_bit(2), 0], 2300));

    for (outer_seq, now) in [(50, 2301), (51, 2321), (52, 2362), (53, 2401)] {
        // Direct TTL=2 needs REQUEST|GRANTED, not RELAYED (the old flags=7
        // fixtures reject before enqueue). The real RX path writes TTL=1/flags=7.
        let mut rx = receive(3, outer_seq, u16::from(outer_seq), 2, now);
        rx.0[32] = 1 | 4;
        ops.push(rx.arg(225, RELAY_QUEUED));

        let (slot_now, result, check) = match outer_seq {
            50 => (2302, 0, RELAY_SENT),
            51 => (2322, -5, RELAY_RF_FAILED),
            52 => (2382, 0, RELAY_SENT), // Exactly 20 ms old is still eligible.
            53 => (2422, 0, RELAY_EXPIRED), // 21 ms old expires before transmission.
            _ => unreachable!(),
        };
        // Node 2 owns slot 1: phase 2,000 us is its start, with 1,500 us
        // remaining before the 500-us guard. Supply a coherent scripted clock;
        // opcode 68 dispatches the callback registered by actual audio_init.
        let mut slot = audio_op(68, &[], slot_now)
            .arg(1, 1)
            .arg(225, check)
            .frame((slot_now / 20) as u32)
            .scalar(16, 1500)
            .scalar(20, 20000)
            .arg(254, 1)
            .tx_failure(false, result);
        slot.0[252..254].copy_from_slice(&2000u16.to_le_bytes());
        ops.push(slot);
        if outer_seq == 51 {
            let mut empty_slot = audio_op(68, &[], 2342)
                .arg(1, 1)
                .arg(225, RELAY_NO_RETRY)
                .frame(117)
                .scalar(16, 1500);
            empty_slot.0[252..254].copy_from_slice(&2000u16.to_le_bytes());
            empty_slot.0[254] = 1;
            ops.push(empty_slot);
        }
    }
}

fn check_relay_slot(r: &Runtime, p: &Joint, op: JointOp, before: &RuntimeRelayCheck) {
    assert_eq!(r.membership.state, State::Active);
    assert_eq!((r.membership.node_id, r.membership.slot_index), (2, 1));
    assert_eq!(r.protocol.last_sync_time, 2300);
    assert_eq!(r.audio_state.tx_depth(), 0);
    assert_eq!(r.audio_state.ingress_used, 0);
    assert_eq!(r.audio_state.counters.spi_audio_in, 0);
    if op.0[225] == RELAY_QUEUED {
        assert_eq!(r.session.active_speaker_ids, [3, 0]);
        assert_eq!(r.session.relay_masks, [c::node_bit(2), 0]);
        assert_eq!((r.session.relay_head + 16 - r.session.relay_tail) % 16, 1);
        let queued = &r.session.relay_ring[usize::from(r.session.relay_tail)];
        assert_eq!(&queued.data[..7], &[5, 13, 3, op.0[30], 1, 7, 1]);
        assert_eq!(queued.queued_ms, g64(&op.0[8..]) as u32);
        assert_eq!(p.radio_frames.len(), before.frames);
        return;
    }
    assert_eq!(op.0[1], r.membership.slot_index as u8);
    assert!(p.clock_available);
    assert_eq!(p.clock_phase, 2000);
    assert_eq!(r.session.relay_head, r.session.relay_tail);
    let counters = r.audio_state.counters;
    if matches!(op.0[225], RELAY_SENT | RELAY_RF_FAILED) {
        assert_eq!(r.session.relay_tail, (before.tail + 1) % 16);
        assert_eq!(p.radio_frames.len(), before.frames + 1);
        assert_eq!(
            p.radio_frames.last().unwrap().as_slice(),
            &before.entry.data[..usize::from(before.entry.len)]
        );
        // Both success and failure consume the relay but retain its stale row.
        assert_eq!(r.session.relay_ring[usize::from(before.tail)], before.entry);
        let success = op.0[225] == RELAY_SENT;
        let mut expected = before.counters;
        expected.tx_count += u32::from(success);
        expected.bundle_tx += u32::from(success);
        expected.prev1_forwarded += u32::from(success);
        expected.prev2_forwarded += u32::from(success);
        assert_eq!(counters, expected);
        assert_eq!(
            r.protocol.stat_tx_fail,
            before.tx_fail + u32::from(!success)
        );
        assert_eq!(
            r.session.relay_bitmap,
            before.bitmap | if success { c::node_bit(3) } else { 0 }
        );
        assert_eq!(counters.relay_ring_drop, before.counters.relay_ring_drop);
    } else {
        assert_eq!(p.radio_frames.len(), before.frames);
        assert_eq!(r.protocol.stat_tx_fail, before.tx_fail);
        if op.0[225] == RELAY_EXPIRED {
            assert_eq!(r.session.relay_tail, (before.tail + 1) % 16);
            assert_eq!(r.session.relay_ring[usize::from(before.tail)], before.entry);
            assert_eq!(
                counters.relay_ring_drop,
                before.counters.relay_ring_drop + 1
            );
            let mut expected = before.counters;
            expected.relay_ring_drop += 1;
            assert_eq!(counters, expected);
        } else {
            assert_eq!(op.0[225], RELAY_NO_RETRY);
            assert_eq!(r.session.relay_tail, before.tail);
            assert_eq!(counters, before.counters);
        }
    }
}

struct RuntimeRelayCheck {
    frames: usize,
    tail: u8,
    entry: RelayEntry,
    counters: crate::protocol_audio::AudioCounters,
    tx_fail: u32,
    bitmap: u8,
}

fn audio_fixtures() -> Vec<JointOp> {
    // Retain all 52 earlier original function entry targets in this separate
    // real-audio replay. Legacy checks of scripted grant replies are not reused.
    let mut ops: Vec<_> = fixtures().into_iter().map(|x| x.0).collect();
    audio_setup(&mut ops, 2, 100);
    ops.push(audio_op(82, &[1, 2, 3], 100));
    for invalid in [&[][..], &[0; 8][..], &[0; 200][..]] {
        ops.push(audio_op(64, invalid, 100));
    }
    // Capture before validation, then cross pause/resume and exact source wrap.
    ops.push(audio_op(86, &[], 101).scalar(4, u32::MAX));
    ops.push(audio_op(66, &[], 101));
    ops.push(audio_op(63, &[], 102));
    ops.push(audio_op(63, &[], 103).arg(1, 1));
    ops.push(audio_op(67, &bundle(1, 0, true), 104).arg(1, 1).arg(2, 13));
    ops.push(audio_op(66, &[], 104));
    ops.push(audio_op(67, &bundle(2, 0, true), 104).arg(1, 1).arg(2, 13));
    ops.push(audio_op(62, &[], 105).arg(2, 1));
    ops.push(audio_op(62, &[], 106).arg(1, 1));
    for seq in 0..34 {
        ops.push(audio_op(64, &bundle(seq, 2, true), 107));
    }
    ops.push(audio_op(65, &[], 108));
    ops.push(audio_op(70, &[], 109));
    // Process helper has valid backing for legacy/malformed rejection, even
    // though the public producer intentionally cannot construct these requests.
    ops.push(audio_op(83, &bundle(42, 0, true), 110).arg(2, 1));
    ops.push(audio_op(83, &[0; 8], 110).arg(2, 13));
    ops.push(audio_op(63, &[], 111));
    ops.push(audio_op(83, &bundle(42, 0, true), 111).arg(2, 13));
    ops.push(audio_op(63, &[], 112).arg(1, 1));
    for seq in [65535, 0, 3, 2, 2] {
        ops.push(audio_op(64, &bundle(seq, 0, true), 113));
    }
    ops.push(audio_op(84, &[], 114));
    for budget in [2000, 917, 725, 533, 532, 0] {
        ops.push(audio_op(68, &[], 115).scalar(16, budget));
    }
    ops.push(audio_op(68, &[], 232));
    ops.push(audio_op(68, &[], 233));
    // Validation order, direct RSSI vs relayed presence, gated receipt delivery.
    for (src, flags, ttl, t) in [
        (0, 0, 2, 13),
        (2, 0, 2, 13),
        (3, 0, 1, 13),
        (3, 2, 2, 13),
        (3, 0, 2, 1),
        (3, 0, 2, 255),
    ] {
        let mut op = receive(src, 1, 100, 0, 240);
        op.0[28] = t;
        op.0[31] = ttl;
        op.0[32] = flags;
        ops.push(op);
    }
    ops.push(audio_op(75, &[1, 3, 0, 2, 0], 241));
    ops.push(audio_op(63, &[], 242));
    ops.push(receive(3, 10, 100, 2, 243));
    ops.push(audio_op(63, &[], 244).arg(1, 1));
    let mut stale = receive(3, 11, 101, 2, 245);
    stale.0[232..240].copy_from_slice(&243999i64.to_le_bytes());
    ops.push(stale);
    ops.push(receive(3, 12, 102, 2, 245));
    ops.push(receive(3, 12, 103, 2, 245));
    ops.push(receive(3, 13, 99, 1, 245).scalar(24, (-5i32) as u32));
    let mut malformed = receive(3, 14, 103, 0, 245);
    malformed.0[36] = 0;
    ops.push(malformed);
    let mut relayed = receive(3, 15, 104, 0, 245);
    relayed.0[31] = 1;
    relayed.0[32] = 2;
    ops.push(relayed);
    for budget in [917, 725, 533, 532] {
        ops.push(audio_op(68, &[], 246).scalar(16, budget));
    }
    ops.push(audio_op(68, &[], 267));
    // Reserve-two path and serialized physical clock consumption.
    for first in 0..2 {
        audio_setup(&mut ops, 2, 300 + first * 10);
        ops.push(audio_op(75, &[1, 3, 0, 2, 0], 300 + first * 10));
        for seq in 20..23 {
            ops.push(audio_op(64, &bundle(seq, 2, true), 300 + first * 10));
            ops.push(receive(3, seq as u8, seq, 2, 300 + first * 10));
            ops.push(audio_op(68, &[], 300 + first * 10).scalar(16, 1600));
        }
    }
    // Deferred tail plus exact immediate-successor proof; radio failure retains
    // both entries; insufficient prev1 budget falls back without strip counters.
    for (budget, radio_result) in [(917, 0), (533, 0), (917, -5), (532, 0)] {
        audio_setup(&mut ops, 2, 400);
        ops.push(audio_op(75, &[1, 3, 0, 2, 0], 400));
        ops.push(audio_op(64, &bundle(99, 0, true), 400));
        ops.push(receive(3, 1, 50, 0, 400));
        ops.push(audio_op(68, &[], 400).scalar(16, 600));
        ops.push(audio_op(64, &bundle(100, 0, true), 400));
        ops.push(receive(3, 2, 51, 0, 400));
        ops.push(audio_op(68, &[], 400).scalar(16, 600));
        ops.push(audio_op(64, &bundle(101, 2, true), 400));
        ops.push(
            audio_op(68, &[], 400)
                .scalar(16, budget)
                .tx_failure(false, radio_result),
        );
        ops.push(audio_op(68, &[], 400).scalar(16, 2000));
    }
    // Overflow/stale relay storage and strict frame-age edge.
    audio_setup(&mut ops, 2, 500);
    ops.push(audio_op(75, &[1, 3, 0, 2, 0], 500));
    for seq in 0..20 {
        ops.push(receive(3, seq, u16::from(seq), 2, 500));
    }
    ops.push(audio_op(68, &[], 520));
    ops.push(audio_op(68, &[], 521));
    for code in [71, 72, 73, 74, 78, 79, 80, 81, 61] {
        ops.push(audio_op(code, &[], 522).arg(1, 255));
    }
    ops.push(audio_op(75, &[255, 1, 3, 0, 2], 523));
    ops.push(audio_op(76, &[255, 3, 1], 523));
    let mut map = slot_map();
    map.active_speaker_count = 2;
    map.active_speaker_ids = [1, 3];
    map.relay_masks = [0, 2];
    let mut raw = [0; wire::SlotMap::SIZE];
    map.encode(&mut raw).unwrap();
    ops.push(audio_op(77, &raw, 523));
    ops.push(
        audio_op(85, &bundle(0, 0, true), 524)
            .arg(1, 3)
            .scalar(4, 65535),
    );
    ops.push(audio_op(85, &[], 524).arg(1, 255));
    // Actual coordinator selection/release/grace routing is not a grant mock.
    audio_setup(&mut ops, 1, 600);
    ops.push(receive(3, 1, 1, 0, 600));
    ops.push(audio_op(78, &[], 600));
    ops.push(audio_op(79, &[], 2100));
    ops.push(audio_op(63, &[], 2101));
    audio_setup(&mut ops, 2, 2200);
    for tts in [
        -1i32, 0, 1999, 2000, 4000, 6000, 8000, 10000, 12000, 14000, 16000, 18000, 20000,
    ] {
        ops.push(audio_op(64, &bundle(tts as u16, 0, true), 2200));
        ops.push(audio_op(65, &[], 2200).scalar(20, tts as u32));
    }
    ops.push(audio_op(88, &[], 2201));
    ops.push(audio_op(64, &[0; 8], 2201));
    for seq in 0..17 {
        ops.push(audio_op(64, &bundle(seq, 0, true), 2201));
    }
    ops.push(audio_op(65, &[], 2201));
    ops.push(audio_op(64, &bundle(42, 0, true), 2201));
    ops.push(audio_op(63, &[], 2201));
    for (data, t) in [
        (bundle(1, 0, true), 1),
        (vec![0; 8], 13),
        (bundle(1, 2, true), 13),
    ] {
        ops.push(audio_op(87, &data, 2202).arg(1, t));
    }
    let mut bad_relay = [0; 65];
    bad_relay[..9].copy_from_slice(&[5, 13, 3, 0, 1, 2, 1, 55, 0]);
    ops.push(audio_op(87, &bad_relay, 2202).arg(2, 1));
    bad_relay[7] = 56;
    ops.push(audio_op(87, &bad_relay, 2202).arg(2, 1));
    relay_slot_fixtures(&mut ops);
    ops
}

#[test]
fn coordinator_expires_grant_at_deadline_but_participant_cannot_revoke_it() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    let mut io = CombinedFullTxPlatform(&mut p);
    r.membership.role = Role::Coordinator;
    r.session.active_speaker_ids = [2, 0];
    r.session.speaker_deadlines_ms[2] = 500;
    r.session.speaker_active_since_ms[2] = 100;
    io.0.inner.now = 499;
    r.audio_expire_speakers(499, &mut io);
    assert_eq!(r.session.active_speaker_ids, [2, 0]);
    assert_eq!(r.session.control_head, r.session.control_tail);
    r.membership.role = Role::Participant;
    io.0.inner.now = 500;
    r.audio_expire_speakers(500, &mut io);
    assert_eq!(r.session.active_speaker_ids, [2, 0]);
    assert_eq!(r.session.control_head, r.session.control_tail);
    r.membership.role = Role::Coordinator;
    r.audio_expire_speakers(500, &mut io);
    assert_eq!(r.session.active_speaker_ids, [0; 2]);
    assert_eq!(r.session.relay_masks, [0; 2]);
    assert_eq!(queued_payload(&r, 10), &[1, 2, 0]);
    assert_eq!(queued_payload(&r, 9), &[0, 0, 0, 0, 0]);
}

#[test]
fn cleanup_and_grant_updates_preserve_unrelated_audio_ownership() {
    let mut r = Runtime::default();
    r.audio_state.boundary = AudioBoundary {
        enabled: true,
        epoch: 42,
        receipt_cutoff_us: 99,
    };
    let boundary = r.audio_state.boundary;
    r.session.relay_ring[0].len = 10;
    r.session.relay_ring[0].data.fill(0xa5);
    r.session.relay_head = 3;
    r.session.relay_tail = 1;
    r.session.speaker_deadlines_ms.fill(500);
    r.session.speaker_active_since_ms.fill(100);
    r.session.heard_bitmap = 7;
    r.session.relay_bitmap = 6;
    r.audio_apply_speaker_grant(2, [2, 3], [4, 2]);
    assert_eq!(r.session.active_speaker_ids, [2, 3]);
    assert_eq!(r.session.relay_masks, [4, 2]);
    r.audio_apply_speaker_release(1, [2, 3]);
    assert_eq!(r.session.active_speaker_ids, [0, 3]);
    assert_eq!(r.session.relay_masks, [0, 2]);
    r.audio_apply_slot_map_speakers(&c::SlotMap {
        active_speaker_count: 1,
        active_speaker_ids: [4, 5],
        relay_masks: [2, 7],
        ..c::SlotMap::default()
    });
    assert_eq!(r.session.active_speaker_ids, [4, 0]);
    assert_eq!(r.session.relay_masks, [2, 0]);
    r.audio_clear_relay_ring();
    assert_eq!((r.session.relay_head, r.session.relay_tail), (0, 0));
    assert!(r
        .session
        .relay_ring
        .iter()
        .all(|entry| *entry == RelayEntry::default()));
    assert_eq!(r.session.active_speaker_ids, [4, 0]);
    assert_eq!(r.session.speaker_deadlines_ms, [500; 9]);
    r.audio_clear_speaker_activity();
    assert_eq!(r.session.speaker_deadlines_ms, [0; 9]);
    assert_eq!(r.session.speaker_active_since_ms, [0; 9]);
    r.audio_clear_speaker_grants();
    assert_eq!(r.session.active_speaker_ids, [0; 2]);
    assert_eq!(r.session.relay_masks, [0; 2]);
    r.audio_clear_bitmaps();
    assert_eq!((r.session.heard_bitmap, r.session.relay_bitmap), (0, 0));
    assert_eq!(r.audio_state.boundary, boundary);
}

#[test]
fn full_audio_corpus_relay_slots_and_disabled_admission() {
    let ops = audio_fixtures();
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    let mut r = Runtime::default();
    let mut token = None;
    for op in &ops {
        let relay_before = RuntimeRelayCheck {
            frames: p.radio_frames.len(),
            tail: r.session.relay_tail,
            entry: r.session.relay_ring[usize::from(r.session.relay_tail)],
            counters: r.audio_state.counters,
            tx_fail: r.protocol.stat_tx_fail,
            bitmap: r.session.relay_bitmap,
        };
        run_audio(&mut r, &mut p, &mut token, *op);
        if matches!(op.0[0], 68 | 69) && op.0[225] != 0 {
            check_relay_slot(&r, &p, *op, &relay_before);
        }
    }
    std::eprintln!("authored full-audio corpus: {} operations", ops.len());

    // Separate fresh replay, not a change to the historical audio/protocol corpus.
    // Existing opcodes call actual source admission, gate closure and work drain.
    let mut ops = Vec::new();
    audio_setup(&mut ops, 2, 100);
    ops.push(audio_op(64, &bundle(7, 0, true), 101));
    ops.push(audio_op(62, &[], 102)); // disabled, no purge
    ops.push(audio_op(64, &bundle(8, 0, true), 103)); // future admission rejected
    ops.push(audio_op(65, &[], 104));
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    let mut r = Runtime::default();
    let mut token = None;
    let mut accepted_epoch = None;
    for op in &ops {
        let ret = run_audio(&mut r, &mut p, &mut token, *op);
        match op.0[0] {
            64 if accepted_epoch.is_none() => {
                assert_eq!(ret, Some(0));
                accepted_epoch = Some(r.audio_state.boundary.epoch);
            }
            62 if accepted_epoch.is_some() => {
                assert!(!r.ingress_admission_gate().enabled);
                assert!(r.audio_state.boundary.enabled);
                assert_eq!(Some(r.audio_state.boundary.epoch), accepted_epoch);
                assert_eq!(r.audio_state.ingress_used(), 1);
            }
            64 => assert_eq!(ret, Some(crate::protocol_tx::EAGAIN)),
            _ => {}
        }
    }
    assert_eq!(r.audio_state.counters().spi_in_frames, 1);
    assert_eq!(r.audio_state.counters().ingress_inactive_drop, 0);
    assert_eq!(r.audio_state.tx_depth(), 1);
    assert_eq!(r.audio_state.ingress_used(), 0);
}
