use super::*;
use std::vec::Vec;

#[derive(Default)]
struct Io {
    now: i64,
    remaining: u32,
    remaining_reads: Vec<u32>,
    packets: Vec<Vec<u8>>,
    replies: Vec<Effect>,
}
impl Platform for Io {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.replies.push(e);
        Reply::None
    }
}
impl MembershipPlatform for Io {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        panic!("audio escaped to membership mock")
    }
}
impl TypedAudioPlatform for Io {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        <Self as AdaptivePlatform>::audio_effect(self, e)
    }
}
impl AdaptivePlatform for Io {
    fn adaptive_effect(&mut self, e: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        match e {
            AdaptiveEffect::FrameCounter => AdaptiveReply::Frame(0),
            _ => panic!("audio escaped to adaptive mock: {e:?}"),
        }
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        match e {
            AudioEffect::NowUs => self.now,
            AudioEffect::SlotRemainingUs => i64::from(if self.remaining_reads.is_empty() {
                self.remaining
            } else {
                self.remaining_reads.remove(0)
            }),
            AudioEffect::TimeToSlotUs => -1,
            _ => 0,
        }
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.packets.push(packet.bytes.to_vec());
        0
    }
}
fn bundle(seq: u16, history: usize) -> Vec<u8> {
    let current = [seq as u8; 48];
    let prev1 = [seq.wrapping_sub(1) as u8; 48];
    let prev2 = [seq.wrapping_sub(2) as u8; 48];
    let view = b::BundleView {
        current_data: &current,
        previous1_data: if history >= 1 { &prev1 } else { &[] },
        previous2_data: if history >= 2 { &prev2 } else { &[] },
        current_seq: seq,
        stream_id: 7,
        flags: 1 | if history >= 1 { 6 } else { 0 } | if history >= 2 { 24 } else { 0 },
        codec: 2,
    };
    let mut data = [0; 200];
    let len = b::audio_bundle_encode(&view, &mut data);
    data[..len].to_vec()
}
fn active() -> Runtime {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Coordinator;
    r.membership.node_id = 1;
    r.audio_state.boundary.enabled = true;
    r.audio_set_ingress_enabled(true, false);
    r
}
#[test]
fn delayed_producer_cannot_recapture_pause_resume_epoch_and_wraps_at_u32() {
    let mut r = active();
    let mut backend = Io::default();
    let mut io = FullAudioPlatform(&mut backend);
    r.audio_state.boundary.epoch = u32::MAX;
    let token = r.audio_capture_ingress();
    r.audio_set_local_enabled(false, &mut io);
    assert_eq!(r.audio_state.boundary.epoch, 0);
    let invalid = token.validate(&[0; 8]);
    assert_eq!(r.audio_admit_prepared(invalid, &mut io), EINVAL);
    assert_eq!(r.audio_state.counters.bundle_bad, 1);
    r.audio_set_local_enabled(true, &mut io);
    let old = token.prepare(&bundle(1, 0)).unwrap();
    assert_eq!(old.captured_epoch(), u32::MAX);
    assert_eq!(r.audio_admit_ingress(old, &mut io), EAGAIN);
    assert_eq!(r.audio_state.ingress_used, 0);
    let request = r.audio_capture_ingress().prepare(&bundle(2, 0)).unwrap();
    assert_eq!(r.audio_admit_ingress(request, &mut io), 0);
    let saved = *r.audio_state.ingress_storage();
    r.audio_set_ingress_enabled(false, true);
    assert_eq!(r.audio_state.ingress_used, 0);
    assert_eq!(*r.audio_state.ingress_storage(), saved);
    assert_eq!(r.audio_state.counters.ingress_purge_drop, 1);
}
#[test]
fn resume_while_enabled_keeps_epoch_and_queued_audio() {
    let mut r = active();
    let mut backend = Io::default();
    let mut io = FullAudioPlatform(&mut backend);
    for seq in 0..4 {
        assert_eq!(r.audio_send_v2(&bundle(seq, 0), &mut io), 0);
    }
    r.audio_ingress_work_handler(&mut io);
    assert_eq!(r.audio_state.tx_depth(), 4);
    let epoch = r.audio_state.boundary.epoch;
    r.audio_set_local_enabled(true, &mut io);
    assert_eq!(r.audio_state.boundary.epoch, epoch);
    assert_eq!(r.audio_state.tx_depth(), 4);
    assert_eq!(r.audio_state.counters.tx_purge_drop, 0);
    r.audio_set_local_enabled(false, &mut io);
    assert_eq!(r.audio_state.boundary.epoch, epoch.wrapping_add(1));
    assert_eq!(r.audio_state.tx_depth(), 0);
    r.audio_set_local_enabled(false, &mut io);
    assert_eq!(r.audio_state.boundary.epoch, epoch.wrapping_add(1));
}
#[test]
fn msgq_uses_all_sixteen_tx_uses_fifteen_and_keeps_stale_storage() {
    let mut r = active();
    let mut backend = Io::default();
    let mut io = FullAudioPlatform(&mut backend);
    for seq in 0..17 {
        assert_eq!(r.audio_send_v2(&bundle(seq, 2), &mut io), 0);
    }
    assert_eq!(r.audio_state.ingress_used, 16);
    assert_eq!(r.audio_state.counters.ingress_msgq_drop, 1);
    r.audio_ingress_work_handler(&mut io);
    assert_eq!(r.audio_state.tx_depth(), 15);
    assert_eq!(r.audio_state.counters.tx_ring_drop, 1);
    let stale = *r.audio_state.tx_storage();
    r.audio_purge_tx_ring();
    assert_eq!(r.audio_state.tx_depth(), 0);
    assert_eq!(*r.audio_state.tx_storage(), stale);
    assert_eq!(r.audio_state.counters.tx_purge_drop, 15);
}
#[test]
fn kernel_pause_resume_uses_actual_owned_boundary_not_reply_snapshot() {
    let mut r = active();
    let mut backend = Io {
        now: 12300,
        ..Io::default()
    };
    let mut io = FullAudioPlatform(&mut backend);
    r.request_audio_pause(255, &mut io);
    r.command_work_handler(&mut io);
    assert_eq!(
        r.snapshot().audio,
        AudioBoundary {
            enabled: false,
            epoch: 1,
            receipt_cutoff_us: 12300
        }
    );
    r.request_audio_resume(0, &mut io);
    r.command_work_handler(&mut io);
    assert_eq!(
        r.snapshot().audio,
        AudioBoundary {
            enabled: true,
            epoch: 2,
            receipt_cutoff_us: 12300
        }
    );
    assert!(!io
        .0
        .replies
        .iter()
        .any(|e| matches!(e, Effect::LocalAudio(_))));
}
#[test]
fn rf_and_shared_sequences_do_not_rewind() {
    let mut r = active();
    for seq in [100, 99, 100, 103] {
        r.track_rf_e2e_sequence(2, seq);
    }
    assert_eq!(r.audio_state.rf_seq[2].last, 103);
    assert_eq!(
        (
            r.audio_state.counters.rf_rx_reset_evt,
            r.audio_state.counters.rf_rx_gap_fr
        ),
        (2, 2)
    );
    let mut tracker = c::Seq16::default();
    for seq in [100, 99] {
        c::seq16_accept(Some(&mut tracker), seq);
    }
    assert_eq!(tracker.last, 100);
    assert_eq!(bundle(0x1234, 0)[4..6], [0x12, 0x34]);
    assert_eq!(CURRENT_ONLY_AIRTIME_US, 533);
}

fn queue(r: &mut Runtime, io: &mut Io, seq: u16, history: usize) {
    assert_eq!(
        r.audio_send_v2(&bundle(seq, history), &mut FullAudioPlatform(&mut *io)),
        0
    );
    r.audio_drain_ingress(&mut FullAudioPlatform(&mut *io));
}

fn accounting(r: &Runtime, expected: (u32, u32, u32)) {
    #[cfg(feature = "audio-accounting-trace")]
    {
        let c = r.audio_state.counters;
        assert_eq!(
            (
                c.local_prefit_no_time,
                c.local_prefit_consumed,
                c.local_entries_consumed
            ),
            expected
        );
    }
    #[cfg(not(feature = "audio-accounting-trace"))]
    let _ = (r, expected);
}

#[test]
fn accounting_layout_is_opt_in_and_refit_still_adds_two_words() {
    assert_eq!(
        core::mem::size_of::<AudioCounters>(),
        172 + if cfg!(feature = "audio-accounting-trace") {
            12
        } else {
            0
        } + if cfg!(feature = "same-slot-audio-refit") {
            8
        } else {
            0
        }
    );
}

// The same packet, queue and original-counter expectations run with trace OFF/ON.
// The C differential corpus independently retains its original projection.
#[test]
fn ordinary_prefit_boundary_preserves_original_packet_queue_and_counters() {
    for remaining in [532, 533] {
        let mut r = active();
        let mut io = Io {
            remaining,
            ..Io::default()
        };
        queue(&mut r, &mut io, 7, 0);
        let storage = r.audio_state.tx;
        let mut expected = r.audio_state.counters;
        let seq = r.protocol.tx_seq;
        r.audio_slot_tx_handler(0, 0, &mut io);
        assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (1, 0));
        assert_eq!(r.audio_state.tx, storage);
        let mut actual = r.audio_state.counters;
        if remaining == 532 {
            assert!(io.packets.is_empty());
            assert_eq!(r.protocol.tx_seq, seq);
            accounting(&r, (1, 1, 1));
        } else {
            assert_eq!(io.packets.len(), 1);
            assert_eq!(io.packets[0].len(), 65);
            assert_eq!(&io.packets[0][9..], bundle(7, 0));
            assert_eq!(r.protocol.tx_seq, seq.wrapping_add(1));
            expected.tx_count += 1;
            expected.tx_queue_drain += 1;
            expected.rf_audio_try += 1;
            expected.rf_audio_ok += 1;
            expected.bundle_tx += 1;
            expected.bundle_max_bytes = 56;
            expected.rf_tx_frames += 1;
            accounting(&r, (0, 0, 1));
        }
        #[cfg(feature = "audio-accounting-trace")]
        {
            actual.local_prefit_no_time = 0;
            actual.local_prefit_consumed = 0;
            actual.local_entries_consumed = 0;
        }
        // Keep the default build warning-free when the diagnostic fields vanish.
        let _ = &mut actual;
        assert_eq!(actual, expected);
        assert_eq!(r.protocol.stat_tx_fail, 0);
    }
}

#[test]
fn ordinary_prefit_loss_recovery_depends_on_successor_history_and_airtime() {
    for seq in [7u16, u16::MAX] {
        for history in [0, 1] {
            for remaining in [CURRENT_ONLY_AIRTIME_US, 10000] {
                let mut r = active();
                let mut io = Io {
                    remaining: 532,
                    ..Io::default()
                };
                queue(&mut r, &mut io, seq, 0);
                r.audio_slot_tx_handler(0, 0, &mut io);
                assert!(io.packets.is_empty());
                assert_eq!(r.audio_state.tx_depth(), 0);

                queue(&mut r, &mut io, seq.wrapping_add(1), history);
                io.remaining = remaining;
                r.audio_slot_tx_handler(0, 1, &mut io);
                assert_eq!(io.packets.len(), 1);
                let sent = b::audio_bundle_parse(&io.packets[0][9..]).unwrap();
                assert_eq!(sent.current_seq, seq.wrapping_add(1));
                let recovered = !sent.previous1_data.is_empty();
                assert_eq!(recovered, history == 1 && remaining == 10000);
                if recovered {
                    assert_eq!(sent.previous1_data, &[seq as u8; 48]);
                }
                assert_eq!(r.audio_state.tx_depth(), 0);
            }
        }
    }
}

fn mandatory(io: &mut Io) -> Runtime {
    let mut r = active();
    queue(&mut r, io, 6, 0);
    queue(&mut r, io, 7, 1);
    r.audio_state.deferred_pending = true;
    r.audio_state.deferred_seq = 6;
    assert!(r.deferred_tail_has_proven_successor());
    r
}

#[test]
fn mandatory_prefit_preserves_both_entries_then_uses_existing_next_slot_policy() {
    let mut io = Io {
        remaining: 532,
        ..Io::default()
    };
    let mut r = mandatory(&mut io);
    let storage = r.audio_state.tx;
    r.audio_slot_tx_handler(0, 0, &mut io);
    assert!(io.packets.is_empty());
    assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (0, 2));
    assert_eq!(r.audio_state.tx, storage);
    assert!(!r.audio_state.deferred_pending);
    assert!(!r.audio_state.relay_contention_turn);
    accounting(&r, (1, 0, 0));
    io.remaining = 10000;
    r.audio_slot_tx_handler(0, 1, &mut io);
    assert_eq!(io.packets.len(), 1);
    assert_eq!(
        b::audio_bundle_parse(&io.packets[0][9..])
            .unwrap()
            .current_seq,
        6
    );
    assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (1, 1));
    assert_eq!(r.audio_state.counters.local_deferred_recovery, 0);
    accounting(&r, (1, 0, 1));
}

#[test]
fn successful_proven_successor_accounts_for_two_actual_entries() {
    let mut io = Io {
        remaining: 10000,
        ..Io::default()
    };
    let mut r = mandatory(&mut io);
    r.audio_slot_tx_handler(0, 0, &mut io);
    assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (2, 0));
    assert_eq!(r.audio_state.counters.local_deferred_recovery, 1);
    assert_eq!(io.packets.len(), 1);
    assert_eq!(
        b::audio_bundle_parse(&io.packets[0][9..])
            .unwrap()
            .current_seq,
        7
    );
    accounting(&r, (0, 0, 2));
}

#[test]
fn dual_send_prefit_reason_keeps_relay_opportunity_in_both_orders() {
    for relay_first in [false, true] {
        let mut r = active();
        let mut io = Io {
            remaining: 10000,
            ..Io::default()
        };
        queue(&mut r, &mut io, 7, 0);
        let payload = bundle(20, 0);
        let mut packet = [0; 209];
        packet[1] = 13;
        packet[2] = 2;
        packet[7..9].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        packet[9..9 + payload.len()].copy_from_slice(&payload);
        assert!(r.enqueue_relay_packet(&packet[..9 + payload.len()], 1, 0, 0));
        r.audio_state.relay_contention_turn = relay_first;
        // First read admits dual service. Local-first then lacks its reserve;
        // relay-first admits local service before its leaf read expires.
        io.remaining_reads = if relay_first {
            std::vec![1066, 1066, 533, 532]
        } else {
            std::vec![1066, 1065, 533, 533]
        };
        r.audio_slot_tx_handler(0, 0, &mut io);
        assert_eq!(io.packets.len(), 1);
        assert_eq!(&io.packets[0][9..], payload);
        assert_eq!(r.audio_state.tx_depth(), 0);
        assert_eq!(r.session.relay_tail, r.session.relay_head);
        assert_eq!(r.audio_state.counters.rf_audio_try, 0);
        assert_eq!(r.audio_state.counters.tx_count, 1);
        accounting(&r, (1, 1, 1));
    }
}

#[test]
fn consume_accounting_bounds_actual_entries_without_changing_tail_policy() {
    for (tail, head, requested, actual) in
        [(15, 1, 2, 2), (15, 0, 2, 1), (0, 0, 1, 0), (4, 4, 0, 0)]
    {
        let mut r = active();
        r.audio_state.tx_tail = tail;
        r.audio_state.tx_head = head;
        r.consume_local_audio_entries(requested, false);
        assert_eq!(r.audio_state.tx_tail, (tail + requested) % 16);
        accounting(&r, (0, 0, actual));
        assert_eq!(
            (
                r.audio_state.counters.tx_purge_drop,
                r.audio_state.counters.tx_ring_drop
            ),
            (0, 0)
        );
    }
}

#[test]
fn pause_purge_and_overwrite_remain_separate_from_consumption() {
    let mut r = active();
    let mut io = Io::default();
    for seq in 0..17 {
        assert_eq!(
            r.audio_send_v2(&bundle(seq, 0), &mut FullAudioPlatform(&mut io)),
            0
        );
    }
    r.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
    assert_eq!(r.audio_state.counters.ingress_msgq_drop, 1);
    assert_eq!(r.audio_state.counters.tx_ring_drop, 1);
    r.audio_set_local_enabled(false, &mut FullAudioPlatform(&mut io));
    assert_eq!(r.audio_state.counters.tx_purge_drop, 15);
    assert_eq!(r.audio_state.tx_depth(), 0);
    r.audio_purge_tx_ring();
    r.use_external_ingress().unwrap();
    r.account_external_ingress(IngressDeltaAccounting {
        overwritten: 3,
        purged: 2,
        failed_enqueue: 1,
    })
    .unwrap();
    assert_eq!(r.audio_state.counters.tx_overwrite, 4);
    assert_eq!(r.audio_state.counters.ingress_msgq_drop, 5);
    assert_eq!(r.audio_state.counters.ingress_purge_drop, 2);
    accounting(&r, (0, 0, 0));
}

#[cfg(feature = "audio-accounting-trace")]
#[test]
fn accounting_counters_wrap_without_changing_the_prefit_result() {
    let mut r = active();
    let mut io = Io {
        remaining: 532,
        ..Io::default()
    };
    queue(&mut r, &mut io, 7, 0);
    r.audio_state.counters.local_prefit_no_time = u32::MAX;
    r.audio_state.counters.local_prefit_consumed = u32::MAX;
    r.audio_state.counters.local_entries_consumed = u32::MAX;
    r.audio_slot_tx_handler(0, 0, &mut io);
    assert!(io.packets.is_empty());
    assert_eq!(r.audio_state.tx_depth(), 0);
    accounting(&r, (0, 0, 0));
}
