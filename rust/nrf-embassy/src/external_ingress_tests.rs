use std::vec::Vec;

use crate::platform::{Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use crate::protocol_audio::{AudioEffect, FullAudioPlatform, IngressDeltaAccounting};
use crate::protocol_membership::{MembershipEffect, MembershipPlatform, ProtocolState};
use crate::protocol_tx::EAGAIN;
use crate::runtime::{Role, Runtime, State, EBUSY, EINVAL, ENOTSUP};
use openhelmet_wire::audio_bundle as b;

struct Io {
    integrated: bool,
    owned: bool,
    strict: bool,
    work_submissions: u32,
    audio_calls: u32,
    adaptive: Vec<AdaptiveEffect>,
}
impl Default for Io {
    fn default() -> Self {
        Self {
            integrated: true,
            owned: true,
            strict: false,
            work_submissions: 0,
            audio_calls: 0,
            adaptive: Vec::new(),
        }
    }
}
impl Platform for Io {
    fn integration(&self) -> Integration {
        if self.integrated {
            Integration::ActorOracle
        } else {
            Integration::NotIntegrated
        }
    }
    fn owned_protocol_algorithms(&self) -> bool {
        self.strict
    }
    fn now_us(&self) -> i64 {
        1_000_000
    }
    fn apply(&mut self, _: Effect) -> Reply {
        Reply::Value(0)
    }
}
impl MembershipPlatform for Io {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
}
impl AdaptivePlatform for Io {
    fn adaptive_effect(&mut self, effect: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        self.adaptive.push(effect);
        AdaptiveReply::Applied
    }
    fn owned_audio_wrappers(&self) -> bool {
        self.owned
    }
    fn audio_effect(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_calls += 1;
        match effect {
            AudioEffect::SubmitWork => {
                self.work_submissions += 1;
                0
            }
            AudioEffect::NowUs => self.now_us(),
            AudioEffect::TimeToSlotUs => 2000,
            _ => 0,
        }
    }
}
impl crate::protocol_audio::AudioPlatform for Io {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_effect(effect)
    }
}

fn active() -> Runtime {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    r.membership.node_id = 1;
    r.membership.slot_index = 1;
    r.audio_state.boundary.enabled = true;
    r.audio_set_ingress_enabled(true, false);
    r
}
fn bundle(seq: u16, flags: u8) -> Vec<u8> {
    let data = [42; 48];
    let view = b::BundleView {
        current_data: &data,
        previous1_data: &[],
        previous2_data: &[],
        current_seq: seq,
        stream_id: 1,
        flags,
        codec: 2,
    };
    let mut raw = [0; 200];
    let len = b::audio_bundle_encode(&view, &mut raw);
    assert!(len != 0);
    raw[..len].to_vec()
}

fn assert_processing_matches(direct: &Runtime, local: &Runtime, direct_io: &Io, local_io: &Io) {
    assert_eq!(direct.audio_state.counters(), local.audio_state.counters());
    assert_eq!(direct.audio_state.tx, local.audio_state.tx);
    assert_eq!(direct.audio_state.tx_head, local.audio_state.tx_head);
    assert_eq!(direct.audio_state.tx_tail, local.audio_state.tx_tail);
    assert_eq!(
        direct.audio_state.tx_depth_dbg,
        local.audio_state.tx_depth_dbg
    );
    assert_eq!(direct.audio_state.spi_seq, local.audio_state.spi_seq);
    assert_eq!(direct.audio_state.phase_hist, local.audio_state.phase_hist);
    assert_eq!(
        direct.audio_state.last_audio_in_time,
        local.audio_state.last_audio_in_time
    );
    assert_eq!(
        direct.audio_state.deferred_pending,
        local.audio_state.deferred_pending
    );
    assert_eq!(
        direct.audio_state.deferred_seq,
        local.audio_state.deferred_seq
    );
    assert!(direct.session == local.session);
    assert_eq!(
        direct.protocol.adaptive.snapshot(),
        local.protocol.adaptive.snapshot()
    );
    assert_eq!(
        direct.protocol.last_local_active_ms,
        local.protocol.last_local_active_ms
    );
    assert_eq!(direct_io.adaptive, local_io.adaptive);
}

#[test]
fn external_mode_refuses_busy_local_queue_without_mutation() {
    let mut r = active();
    let mut io = Io::default();
    let token = r.audio_capture_ingress();
    let request = token.prepare(&bundle(7, 1)).unwrap();
    assert_eq!(r.audio_admit_ingress(request, &mut io), 0);
    let storage = *r.audio_state.ingress_storage();
    let counters = r.audio_state.counters();
    let gate = r.ingress_admission_gate();
    let cursors = (r.audio_state.ingress_read, r.audio_state.ingress_write);
    assert_eq!(r.use_external_ingress(), Err(EBUSY));
    assert_eq!(r.audio_state.ingress_used(), 1);
    assert_eq!(*r.audio_state.ingress_storage(), storage);
    assert_eq!(r.audio_state.counters(), counters);
    assert_eq!(r.ingress_admission_gate(), gate);
    assert_eq!(token.epoch(), gate.epoch);
    assert_eq!(
        (r.audio_state.ingress_read, r.audio_state.ingress_write),
        cursors
    );
    assert_eq!(
        r.account_external_ingress(IngressDeltaAccounting::default()),
        Err(ENOTSUP)
    );
    r.audio_drain_ingress(&mut io);
    r.use_external_ingress().unwrap();
    assert_eq!(r.audio_admit_ingress(request, &mut io), ENOTSUP);
    assert_eq!(r.audio_send_v2(&bundle(8, 1), &mut io), ENOTSUP);
    assert_eq!(io.work_submissions, 1);
}

#[test]
fn direct_external_consumption_matches_local_worker_without_second_queue() {
    let mut local = active();
    let mut direct = active();
    let mut local_io = Io::default();
    let mut direct_io = Io::default();
    direct.use_external_ingress().unwrap();
    let ingress_storage = *direct.audio_state.ingress_storage();
    for i in 0..100u16 {
        // Cross u16 sequence wrap, exercise a gap and an old/reset sequence,
        // and keep the TX ring full long enough to advance its tail repeatedly.
        let seq = 65530u16.wrapping_add(match i {
            30 => 32,
            31 => 29,
            _ => i,
        });
        let raw = bundle(seq, if i % 2 == 0 { 1 } else { 0 });
        let prepared = direct.ingress_admission_gate().capture().validate(&raw);
        let request = prepared.request().unwrap();
        assert_eq!(
            request.captured_epoch(),
            direct.ingress_admission_gate().epoch
        );
        assert_eq!(
            &request.entry().data[..usize::from(request.entry().len)],
            &raw
        );
        assert_eq!(local.audio_send_v2(&raw, &mut local_io), 0);
        local.audio_ingress_work_handler(&mut local_io);
        assert_eq!(
            direct.consume_prepared_ingress(&prepared, &mut direct_io),
            0
        );
        assert_eq!(direct.audio_state.ingress_used(), 0);
        assert_eq!(
            (
                direct.audio_state.ingress_read,
                direct.audio_state.ingress_write
            ),
            (0, 0)
        );
        assert_eq!(*direct.audio_state.ingress_storage(), ingress_storage);
        assert!(!direct.audio_state.work_pending);
        assert_processing_matches(&direct, &local, &direct_io, &local_io);
    }
    assert_eq!(direct_io.work_submissions, 0);
    assert_eq!(local_io.work_submissions, 100);
    assert_eq!(direct.audio_state.counters().spi_in_frames, 100);
    assert_eq!(direct.audio_state.counters().tx_ring_drop, 85);
}

#[test]
fn published_external_message_survives_admission_disable_without_purge() {
    let mut local = active();
    let mut direct = active();
    let mut local_io = Io::default();
    let mut direct_io = Io::default();
    for (r, io) in [(&mut local, &mut local_io), (&mut direct, &mut direct_io)] {
        r.membership.coordinator_id = 2;
        r.adaptive_reset();
        r.adaptive_members_owned(&mut FullAudioPlatform(io));
        assert_eq!(r.protocol.adaptive.snapshot().local_id, 1);
    }
    direct.use_external_ingress().unwrap();
    let storage = *direct.audio_state.ingress_storage();
    let gate = direct.ingress_admission_gate();
    assert!(gate.enabled);
    let published = gate.capture().validate(&bundle(7, 1));
    // Model the trusted external mailbox copy after successful publication.
    // PreparedIngress itself proves validation, not publication.
    assert_eq!(
        local.audio_admit_prepared(published, &mut FullAudioPlatform(&mut local_io)),
        0
    );
    for r in [&mut local, &mut direct] {
        r.audio_set_ingress_enabled(false, false);
        assert!(!r.ingress_admission_gate().enabled);
        assert_eq!(r.ingress_admission_gate().epoch, gate.epoch);
        assert!(r.audio_state.boundary.enabled);
        assert_eq!(r.membership.state, State::Active);
    }
    assert_eq!(local.audio_state.ingress_used(), 1);
    // Admission is closed to future producers; do not consume unpublished input.
    assert_eq!(
        local.audio_admit_prepared(published, &mut FullAudioPlatform(&mut local_io)),
        EAGAIN
    );
    local.audio_ingress_work_handler(&mut FullAudioPlatform(&mut local_io));
    assert_eq!(
        direct.consume_prepared_ingress(&published, &mut FullAudioPlatform(&mut direct_io)),
        0
    );
    assert_processing_matches(&direct, &local, &direct_io, &local_io);
    assert_eq!(direct.audio_state.counters().spi_in_frames, 1);
    assert_eq!(direct.audio_state.counters().ingress_inactive_drop, 0);
    assert_eq!(direct.audio_state.tx_depth(), 1);
    assert_eq!(direct.audio_state.ingress_used(), 0);
    assert_eq!(
        (
            direct.audio_state.ingress_read,
            direct.audio_state.ingress_write
        ),
        (0, 0)
    );
    assert_eq!(*direct.audio_state.ingress_storage(), storage);
    assert!(!direct.audio_state.work_pending);
    assert_eq!(direct_io.work_submissions, 0);
    assert_eq!(local_io.work_submissions, 1);
}

#[test]
fn external_gate_admits_scanning_but_worker_drops_and_rechecks_epoch() {
    let mut r = active();
    let mut io = Io::default();
    let mut local = active();
    let mut local_io = Io::default();
    r.use_external_ingress().unwrap();
    r.membership.state = State::Scanning;
    local.membership.state = State::Scanning;
    let gate = r.ingress_admission_gate();
    assert!(gate.enabled); // Producer can publish successfully and ACK here.
    let prepared = gate.capture().validate(&bundle(7, 1));
    assert_eq!(local.audio_send_v2(&bundle(7, 1), &mut local_io), 0);
    local.audio_ingress_work_handler(&mut local_io);
    assert_eq!(r.consume_prepared_ingress(&prepared, &mut io), EAGAIN);
    assert_eq!(r.audio_state.counters(), local.audio_state.counters());
    assert_eq!(r.audio_state.counters().ingress_inactive_drop, 1);
    assert_eq!(r.audio_state.tx_depth(), 0);
    r.membership.state = State::Active;
    r.audio_set_local_enabled(false, &mut io);
    assert!(!r.ingress_admission_gate().enabled);
    assert_eq!(r.consume_prepared_ingress(&prepared, &mut io), EAGAIN);
    r.audio_set_local_enabled(true, &mut io);
    assert!(r.ingress_admission_gate().enabled);
    assert_eq!(r.consume_prepared_ingress(&prepared, &mut io), EAGAIN);
    assert_eq!(prepared.request().unwrap().captured_epoch(), gate.epoch);
    assert_eq!(r.audio_state.counters().ingress_inactive_drop, 1);
    assert_eq!(r.audio_state.counters().spi_audio_in, 0);
    assert_eq!(r.audio_state.tx_depth(), 0);
}

#[test]
fn malformed_external_validation_counts_once_before_gate_but_requires_owned_mode() {
    let mut r = active();
    let mut io = Io::default();
    let token = r.ingress_admission_gate().capture();
    let invalid = token.validate(&[]);
    assert_eq!(invalid.request(), Err(token));
    assert_eq!(r.consume_prepared_ingress(&invalid, &mut io), ENOTSUP);
    r.use_external_ingress().unwrap();
    io.owned = false;
    assert_eq!(r.consume_prepared_ingress(&invalid, &mut io), ENOTSUP);
    io.owned = true;
    io.integrated = false;
    assert_eq!(r.consume_prepared_ingress(&invalid, &mut io), ENOTSUP);
    io.integrated = true;
    assert_eq!(r.audio_state.counters().bundle_bad, 0);
    r.audio_set_local_enabled(false, &mut io);
    r.audio_set_ingress_enabled(false, false);
    assert_eq!(r.consume_prepared_ingress(&invalid, &mut io), EINVAL);
    assert_eq!(r.audio_state.counters().bundle_bad, 1);
    assert_eq!(r.audio_state.counters().ingress_inactive_drop, 0);
    assert_eq!(r.audio_state.tx_depth(), 0);
}

#[test]
fn external_completed_deltas_wrap_and_purge_does_not_duplicate_accounting() {
    let mut r = active();
    let mut io = Io::default();
    let delta = IngressDeltaAccounting {
        overwritten: 3,
        purged: 16,
        failed_enqueue: 2,
    };
    assert_eq!(r.account_external_ingress(delta), Err(ENOTSUP));
    assert_eq!(r.audio_state.counters(), Default::default());
    r.use_external_ingress().unwrap();
    r.audio_state.boundary.epoch = u32::MAX;
    let old = r.ingress_admission_gate().capture().validate(&bundle(7, 1));
    r.audio_state.counters.ingress_msgq_drop = u32::MAX;
    r.audio_state.counters.tx_overwrite = u32::MAX;
    r.audio_state.counters.ingress_purge_drop = u32::MAX;
    r.audio_set_local_enabled(false, &mut io);
    assert_eq!(r.ingress_admission_gate().epoch, 0);
    // The future mailbox owner must purge before publishing this gate/awaiting.
    let calls = io.audio_calls;
    r.account_external_ingress(delta).unwrap();
    assert_eq!(io.audio_calls, calls);
    let expected = crate::protocol_audio::AudioCounters {
        ingress_msgq_drop: 4,
        tx_overwrite: 2,
        ingress_purge_drop: 15,
        ..Default::default()
    };
    assert_eq!(r.audio_state.counters(), expected);
    r.audio_set_local_enabled(true, &mut io);
    assert_eq!(r.ingress_admission_gate().epoch, 1);
    assert_eq!(r.audio_state.counters(), expected);
    assert_eq!(r.consume_prepared_ingress(&old, &mut io), EAGAIN);
    let new = r.ingress_admission_gate().capture().validate(&bundle(8, 1));
    assert_eq!(r.consume_prepared_ingress(&new, &mut io), 0);
    assert_eq!(r.audio_state.counters().ingress_purge_drop, 15);
}

#[test]
fn external_validation_scalar_accounting_mode_guard_and_wrapping() {
    let mut r = active();
    assert_eq!(
        r.account_external_ingress_validation(u32::MAX),
        Err(ENOTSUP)
    );
    assert_eq!(r.audio_state.counters.bundle_bad, 0);
    r.use_external_ingress().unwrap();
    r.account_external_ingress_validation(u32::MAX).unwrap();
    assert_eq!(r.audio_state.counters.bundle_bad, u32::MAX);
    r.account_external_ingress_validation(2).unwrap();
    assert_eq!(r.audio_state.counters.bundle_bad, 1);
    r.account_external_ingress_validation(0).unwrap();
    assert_eq!(r.audio_state.counters.bundle_bad, 1);
    r.audio_init(&mut Io::default());
    assert_eq!(r.account_external_ingress_validation(9), Err(ENOTSUP));
    assert_eq!(r.audio_state.counters.bundle_bad, 1);
}

#[test]
fn audio_and_runtime_reinit_reset_external_mode_even_before_failed_init_hook() {
    let mut r = active();
    let mut io = Io::default();
    r.use_external_ingress().unwrap();
    r.audio_init(&mut io);
    assert_eq!(r.account_external_ingress(Default::default()), Err(ENOTSUP));
    assert_eq!(r.audio_send_v2(&bundle(7, 1), &mut io), 0);
    r.audio_drain_ingress(&mut io);
    r.use_external_ingress().unwrap();
    assert_eq!(r.init(&mut FullAudioPlatform(&mut io)), 0);
    assert_eq!(r.account_external_ingress(Default::default()), Err(ENOTSUP));
    r.audio_set_ingress_enabled(true, false);
    r.audio_set_local_enabled(true, &mut io);
    let old = r.ingress_admission_gate().capture().validate(&bundle(8, 1));
    r.use_external_ingress().unwrap();
    io.strict = true; // Missing owned BindMembershipWork fails before AudioInit.
    assert_eq!(r.init(&mut io), ENOTSUP);
    assert_eq!(r.account_external_ingress(Default::default()), Err(ENOTSUP));
    assert_eq!(r.consume_prepared_ingress(&old, &mut io), ENOTSUP);
    r.audio_set_local_enabled(false, &mut io);
    r.audio_set_local_enabled(true, &mut io);
    assert_eq!(
        r.audio_admit_ingress(*old.request().unwrap(), &mut io),
        EAGAIN
    );
    assert_eq!(r.audio_send_v2(&bundle(9, 1), &mut io), 0);
}
