use core::future::Future;
use core::pin::{pin, Pin};
use core::task::{Context, Poll, Waker};
use std::vec::Vec;

use crate::platform::{Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{
    AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply, CombinedMembershipPlatform,
};
use crate::protocol_audio::{AudioEffect, AudioPlatform, FullAudioPlatform};
use crate::protocol_membership::{Input, MembershipEffect, MembershipPlatform, ProtocolState};
use crate::protocol_runtime::FullProtocolPlatform;
use crate::protocol_rx::{FullRxPlatform, RxEffect, RxPlatform};
use crate::protocol_tx::{
    ClockSnapshot, CombinedFullTxPlatform, RadioSend, RadioSendResult, SubmissionDisposition,
    EAGAIN,
};
use crate::runtime::{Event, Role, Runtime, State, Timer, TimerToken, ENOTSUP};
use openhelmet_mesh::adaptive as a;
use openhelmet_wire::{audio_bundle as b, mesh as w};

struct PendingIo {
    result: i32,
    remaining: u32,
    clock: Option<ClockSnapshot>,
    owned_tx: bool,
    sync_calls: usize,
    async_calls: usize,
    completed: usize,
    cancelled: usize,
    frames: Vec<Vec<u8>>,
    effects: Vec<Effect>,
    membership: Vec<MembershipEffect>,
}
impl Default for PendingIo {
    fn default() -> Self {
        Self {
            result: 0,
            remaining: 10000,
            clock: Some(ClockSnapshot {
                frame: 123,
                phase_us: 456,
            }),
            owned_tx: true,
            sync_calls: 0,
            async_calls: 0,
            completed: 0,
            cancelled: 0,
            frames: Vec::new(),
            effects: Vec::new(),
            membership: Vec::new(),
        }
    }
}
struct Send<'a, 'b> {
    io: &'a mut PendingIo,
    packet: RadioSend<'b>,
    polls: u8,
    finished: bool,
}
impl Future for Send<'_, '_> {
    type Output = i32;
    fn poll(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<i32> {
        let this = self.get_mut();
        assert!(!this.finished, "completed send polled again");
        this.polls += 1;
        if this.polls <= 2 {
            return Poll::Pending;
        }
        assert_eq!(this.packet.pipe, 0);
        assert_eq!(this.packet.address, [0xe7; 5]);
        this.io.frames.push(this.packet.bytes.to_vec());
        this.io.completed += 1;
        this.finished = true;
        Poll::Ready(this.io.result)
    }
}
impl Drop for Send<'_, '_> {
    fn drop(&mut self) {
        if !self.finished {
            self.io.cancelled += 1;
        }
    }
}
impl Platform for PendingIo {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        1_000_000
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.effects.push(e);
        match e {
            Effect::GetAddress => Reply::Address([1, 2, 3, 4, 5]),
            _ => Reply::Value(0),
        }
    }
}
impl MembershipPlatform for PendingIo {
    async fn dispatch_membership_async(&mut self, _: MembershipEffect, _: &mut Runtime) -> bool {
        panic!("sync mode or owned JOIN bypassed the intended dispatch")
    }
    fn membership_effect(&mut self, e: MembershipEffect, _: &mut ProtocolState) -> bool {
        self.membership.push(e);
        true
    }
}
impl AdaptivePlatform for PendingIo {
    fn adaptive_effect(&mut self, e: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        match e {
            AdaptiveEffect::FrameCounter => AdaptiveReply::Frame(123),
            AdaptiveEffect::JoinRequest => AdaptiveReply::Accepted(self.result),
            _ => AdaptiveReply::Applied,
        }
    }
    fn owned_tx_wrappers(&self) -> bool {
        self.owned_tx
    }
    fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
        self.clock
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        match e {
            AudioEffect::SlotRemainingUs => i64::from(self.remaining),
            AudioEffect::NowUs => self.now_us(),
            _ => 0,
        }
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.sync_calls += 1;
        self.frames.push(packet.bytes.to_vec());
        self.result
    }
    async fn radio_send_async(&mut self, packet: RadioSend<'_>) -> i32 {
        self.async_calls += 1;
        Send {
            io: self,
            packet,
            polls: 0,
            finished: false,
        }
        .await
    }
}
impl AudioPlatform for PendingIo {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        self.audio_effect(e)
    }
}
impl RxPlatform for PendingIo {
    fn rx_io(&mut self, _: RxEffect) -> i32 {
        0
    }
}
fn poll<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}
fn finish<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    assert!(poll(future.as_mut()).is_pending());
    assert!(poll(future.as_mut()).is_pending());
    match poll(future.as_mut()) {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("send did not finish on third poll"),
    }
}
fn active() -> Runtime {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    r.membership.node_id = 1;
    r.membership.slot_index = 1;
    r.address = [1, 2, 3, 4, 5];
    r.audio_state.boundary.enabled = true;
    r.audio_set_ingress_enabled(true, false);
    r
}
fn valid_sync() -> Runtime {
    let mut r = active();
    r.membership.role = Role::Coordinator;
    r.membership.coordinator_id = r.membership.node_id;
    r.protocol.term = 1;
    let own = r.local_identity();
    r.protocol.leader_identity = own;
    a::init(Some(&mut r.protocol.adaptive), 1, 1, 1, 1, 0);
    assert!(a::set_member(Some(&mut r.protocol.adaptive), 1, Some(&own)));

    let shared = r.protocol.adaptive.snapshot();
    assert_ne!(r.protocol.term, 0);
    assert_ne!(shared.members, 0);
    assert_eq!(shared.term, r.protocol.term);
    assert_eq!(shared.local_id, r.membership.node_id);
    assert_eq!(shared.leader_id, r.membership.coordinator_id);
    assert_eq!(r.membership.coordinator_id, r.membership.node_id);
    assert_eq!(r.protocol.leader_identity, r.local_identity());
    assert_eq!(shared.nodes[0].identity, r.local_identity());
    r
}
fn bundle(history: bool) -> Vec<u8> {
    let data = [42; 48];
    let view = b::BundleView {
        current_data: &data,
        previous1_data: if history { &data } else { &[] },
        previous2_data: if history { &data } else { &[] },
        current_seq: 7,
        stream_id: 1,
        flags: if history { 31 } else { 1 },
        codec: 2,
    };
    let mut raw = [0; 200];
    let len = b::audio_bundle_encode(&view, &mut raw);
    raw[..len].to_vec()
}

#[test]
fn all_five_wrappers_forward_pending_radio_and_join_dispatch() {
    macro_rules! check {
        ($wrapper:ident) => {
            for result in [0, -5] {
                let mut r = active();
                let mut io = PendingIo {
                    result,
                    ..PendingIo::default()
                };
                {
                    let mut p = $wrapper(&mut io);
                    assert_eq!(
                        finish(r.tx_send_packet_async(4, &[1, 2, 3, 4, 5], &mut p)),
                        result
                    );
                    assert_eq!(
                        finish(p.dispatch_membership_async(MembershipEffect::JoinRequest, &mut r)),
                        result == 0
                    );
                    assert_eq!(r.tx_send_packet(4, &[1, 2, 3, 4, 5], &mut p), result);
                }
                assert_eq!(
                    (io.async_calls, io.completed, io.cancelled, io.sync_calls),
                    (2, 2, 0, 1)
                );
                assert_eq!(io.frames.len(), 3);
                assert_eq!(io.frames[1][1], 15);
                assert_eq!(r.protocol.tx_seq, 3);
            }
        };
    }
    check!(CombinedMembershipPlatform);
    check!(CombinedFullTxPlatform);
    check!(FullAudioPlatform);
    check!(FullRxPlatform);
    check!(FullProtocolPlatform);
}

#[test]
fn packet_validation_and_missing_sync_clock_do_not_await() {
    let mut r = active();
    let mut io = PendingIo {
        clock: None,
        ..PendingIo::default()
    };
    assert_eq!(
        crate::corefuture::ready(r.tx_send_sync_async(&mut io)),
        EAGAIN
    );
    assert_eq!(
        crate::corefuture::ready(r.tx_send_packet_ex_async(1, &[], 0, 0, 1, 2, &mut io)),
        crate::runtime::EINVAL
    );
    assert_eq!(
        crate::corefuture::ready(r.tx_send_packet_ex_async(4, &[0; 201], 0, 0, 1, 2, &mut io)),
        crate::protocol_tx::EMSGSIZE
    );
    assert_eq!(
        (io.async_calls, io.sync_calls, r.protocol.tx_seq),
        (0, 0, 0)
    );
}

/// Legacy hooks reject local sends: typed overrides must survive every wrapper.
/// Only the dual-send fixture opts into the existing code-only relay path.
struct ClassifiedIo {
    io: PendingIo,
    disposition: SubmissionDisposition,
    integration: Integration,
    next_result: Option<RadioSendResult>,
    fresh_remaining: Option<u32>,
    allow_legacy_relay: bool,
    legacy_relay_calls: usize,
    classified_results: Vec<RadioSendResult>,
}
impl ClassifiedIo {
    fn validate_legacy_relay(&self, packet: RadioSend<'_>) {
        assert!(
            self.allow_legacy_relay,
            "typed send fell back to legacy hook"
        );
        assert_eq!(packet.pipe, 0);
        assert_eq!(packet.address, [0xe7; 5]);
        let decoded = w::decode(packet.bytes, 200).unwrap();
        // Match the packet injected by relay(false), including its stored flags.
        assert_eq!(decoded.header.version, w::VERSION);
        assert_eq!(decoded.header.packet_type, 13);
        assert_eq!(decoded.header.src_id, 3, "local audio used legacy hook");
        assert_eq!(decoded.header.seq, 7);
        assert_eq!(decoded.header.ttl, 2);
        assert_eq!(decoded.header.flags, 5);
        assert_eq!(decoded.header.talk_channel, 1);
        assert_eq!(decoded.payload, bundle(false));
    }

    fn completed_result(&mut self, code: i32) -> RadioSendResult {
        let result = RadioSendResult {
            code,
            disposition: self.disposition,
        };
        self.classified_results.push(result);
        if let Some(next) = self.next_result.take() {
            self.io.result = next.code;
            self.disposition = next.disposition;
        }
        if let Some(remaining) = self.fresh_remaining.take() {
            self.io.remaining = remaining;
        }
        result
    }
}
impl Platform for ClassifiedIo {
    fn integration(&self) -> Integration {
        self.integration
    }
    fn now_us(&self) -> i64 {
        self.io.now_us()
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.io.apply(e)
    }
}
impl MembershipPlatform for ClassifiedIo {
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.io.membership_effect(e, s)
    }
}
impl AdaptivePlatform for ClassifiedIo {
    fn adaptive_effect(&mut self, e: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply {
        self.io.adaptive_effect(e, io)
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.io.audio_effect(e)
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.validate_legacy_relay(packet);
        self.legacy_relay_calls += 1;
        self.io.radio_send(packet)
    }
    async fn radio_send_async(&mut self, packet: RadioSend<'_>) -> i32 {
        self.validate_legacy_relay(packet);
        self.legacy_relay_calls += 1;
        self.io.radio_send_async(packet).await
    }
    fn radio_send_classified(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        let code = self.io.radio_send(packet);
        self.completed_result(code)
    }
    async fn radio_send_classified_async(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        let code = self.io.radio_send_async(packet).await;
        self.completed_result(code)
    }
}
impl AudioPlatform for ClassifiedIo {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        self.audio_effect(e)
    }
}
impl RxPlatform for ClassifiedIo {
    fn rx_io(&mut self, _: RxEffect) -> i32 {
        0
    }
}
fn classified_io(disposition: SubmissionDisposition) -> ClassifiedIo {
    ClassifiedIo {
        io: PendingIo {
            result: EAGAIN,
            ..PendingIo::default()
        },
        disposition,
        integration: Integration::ActorOracle,
        next_result: None,
        fresh_remaining: None,
        allow_legacy_relay: false,
        legacy_relay_calls: 0,
        classified_results: Vec::new(),
    }
}

#[test]
fn identical_codes_preserve_all_dispositions_through_five_wrappers() {
    macro_rules! check {
        ($wrapper:ident) => {
            for disposition in [SubmissionDisposition::Accepted, SubmissionDisposition::NotSubmitted, SubmissionDisposition::Unknown] {
                let mut r = active();
                let mut io = classified_io(disposition);
                let expected = RadioSendResult { code: EAGAIN, disposition };
                {
                    let mut p = $wrapper(&mut io);
                    assert_eq!(crate::corefuture::ready(r.tx_send_packet_ex_classified_impl::<false>(4, &[1, 2], 2, 3, 1, 7, &mut p)), expected);
                    assert_eq!(finish(r.tx_send_packet_ex_classified_impl::<true>(4, &[1, 2], 2, 3, 1, 7, &mut p)), expected);
                    assert_eq!(r.tx_send_packet_ex(4, &[1, 2], 2, 3, 1, 7, &mut p), EAGAIN);
                    assert_eq!(finish(r.tx_send_packet_ex_async(4, &[1, 2], 2, 3, 1, 7, &mut p)), EAGAIN);
                }
                assert_eq!((io.io.sync_calls, io.io.async_calls, io.io.completed), (2, 2, 2));
                assert_eq!(io.io.frames, std::vec![std::vec![w::VERSION, 4, 1, 7, 2, 3, 1, 2, 0, 1, 2]; 4]);
            }
        };
    }
    check!(CombinedMembershipPlatform);
    check!(CombinedFullTxPlatform);
    check!(FullAudioPlatform);
    check!(FullRxPlatform);
    check!(FullProtocolPlatform);
}

#[test]
fn classified_defaults_preserve_legacy_async_override_and_sync_selection() {
    let mut io = PendingIo {
        result: EAGAIN,
        ..PendingIo::default()
    };
    let packet = RadioSend {
        bytes: &[1, 2],
        pipe: 0,
        address: [0xe7; 5],
    };
    let expected = RadioSendResult {
        code: EAGAIN,
        disposition: SubmissionDisposition::Unknown,
    };
    assert_eq!(
        crate::corefuture::ready(crate::corefuture::radio_send_classified::<false>(
            &mut io, packet
        )),
        expected
    );
    assert_eq!((io.sync_calls, io.async_calls), (1, 0));
    assert_eq!(
        finish(crate::corefuture::radio_send_classified::<true>(
            &mut io, packet
        )),
        expected
    );
    assert_eq!((io.sync_calls, io.async_calls, io.completed), (1, 1, 1));

    let mut immediate = ImmediateIo(PendingIo {
        result: EAGAIN,
        ..PendingIo::default()
    });
    assert_eq!(
        crate::corefuture::ready(immediate.radio_send_classified_async(packet)),
        expected
    );
    assert_eq!((immediate.0.sync_calls, immediate.0.async_calls), (1, 0));
}

#[test]
fn classified_validation_is_not_submitted_without_awaiting_radio() {
    let mut r = active();
    let mut io = classified_io(SubmissionDisposition::Accepted);
    for (t, payload, code) in [
        (1, &[][..], crate::runtime::EINVAL),
        (13, &[][..], crate::runtime::EINVAL),
        (4, &[0; 201][..], crate::protocol_tx::EMSGSIZE),
    ] {
        assert_eq!(
            crate::corefuture::ready(
                r.tx_send_packet_ex_classified_impl::<true>(t, payload, 0, 0, 1, 2, &mut io)
            ),
            RadioSendResult {
                code,
                disposition: SubmissionDisposition::NotSubmitted
            }
        );
    }
    io.integration = Integration::NotIntegrated;
    assert_eq!(
        crate::corefuture::ready(r.tx_send_packet_ex_classified_impl::<true>(
            4,
            &[],
            0,
            0,
            1,
            2,
            &mut io
        )),
        RadioSendResult {
            code: ENOTSUP,
            disposition: SubmissionDisposition::NotSubmitted
        }
    );
    assert_eq!(
        (io.io.sync_calls, io.io.async_calls, r.protocol.tx_seq),
        (0, 0, 0)
    );
}

#[test]
fn local_audio_projects_code_even_when_fault_was_accepted() {
    for disposition in [
        SubmissionDisposition::Accepted,
        SubmissionDisposition::NotSubmitted,
        SubmissionDisposition::Unknown,
    ] {
        for code in [EAGAIN, -5] {
            let mut r = active();
            let mut io = classified_io(disposition);
            io.io.result = code;
            assert_eq!(
                r.audio_send_v2(&bundle(false), &mut FullAudioPlatform(&mut io)),
                0
            );
            finish(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
            assert_eq!(r.audio_state.tx_tail, 1);
            assert_eq!(r.audio_state.counters.rf_audio_try, 1);
            assert_eq!(r.audio_state.counters.rf_audio_ok, 0);
            assert_eq!(r.audio_state.counters.rf_audio_fail, 1);
            assert_eq!(r.audio_state.counters.tx_count, 0);
            assert_eq!(r.audio_state.counters.bundle_tx, 0);
            assert_eq!(r.protocol.stat_tx_fail, 1);
            assert_eq!((io.io.sync_calls, io.io.async_calls), (0, 1));
        }
    }
}

#[test]
fn cancelled_packet_keeps_sequence_without_emit_or_replay() {
    let mut r = active();
    let mut io = PendingIo::default();
    {
        let mut future = pin!(r.tx_send_packet_async(4, &[1, 2, 3, 4, 5], &mut io));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.protocol.tx_seq, 1);
    assert_eq!(
        (io.async_calls, io.cancelled, io.completed, io.sync_calls),
        (1, 1, 0, 0)
    );
    assert!(io.frames.is_empty());
}

#[cfg(feature = "same-slot-audio-refit")]
mod same_slot_refit {
    use super::*;
    use crate::protocol_audio::CURRENT_ONLY_AIRTIME_US;

    fn run<F: Future>(future: F) -> F::Output {
        let mut future = pin!(future);
        for _ in 0..16 {
            if let Poll::Ready(value) = poll(future.as_mut()) {
                return value;
            }
        }
        panic!("bounded sends did not finish");
    }

    fn setup(history: bool) -> (Runtime, ClassifiedIo) {
        let mut r = active();
        let mut io = classified_io(SubmissionDisposition::NotSubmitted);
        io.next_result = Some(RadioSendResult {
            code: 0,
            disposition: SubmissionDisposition::Accepted,
        });
        io.fresh_remaining = Some(CURRENT_ONLY_AIRTIME_US);
        assert_eq!(
            r.audio_send_v2(&bundle(history), &mut FullAudioPlatform(&mut io)),
            0
        );
        r.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
        (r, io)
    }

    fn assert_one_local_sequence(r: &Runtime, io: &ClassifiedIo, initial_seq: u8) {
        assert_eq!(r.protocol.tx_seq, initial_seq.wrapping_add(1));
        for frame in &io.io.frames {
            let packet = w::decode(frame, 200).unwrap();
            if packet.header.packet_type == 13 && packet.header.src_id == r.membership.node_id {
                assert_eq!(packet.header.seq, initial_seq);
            }
        }
    }

    #[test]
    fn current_bytes_header_and_sequence_survive_one_same_slot_refit() {
        for asynchronous in [false, true] {
            let (mut r, mut io) = setup(true);
            let initial_seq = r.protocol.tx_seq;
            // Distinct history/current bytes catch copying a predecessor as current.
            let entry = &mut r.audio_state.tx[0];
            entry.data[8..56].fill(11);
            entry.data[56..104].fill(22);
            for (i, byte) in entry.data[104..152].iter_mut().enumerate() {
                *byte = i as u8;
            }
            let storage = r.audio_state.tx;
            if asynchronous {
                run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
            } else {
                r.audio_slot_tx_handler(1, 1, &mut FullAudioPlatform(&mut io));
            }
            // Mock frames record backend requests, including the pre-submit rejection.
            assert_eq!(
                io.io.frames.iter().map(Vec::len).collect::<Vec<_>>(),
                [161, 65]
            );
            let first = w::decode(&io.io.frames[0], 200).unwrap();
            let second = w::decode(&io.io.frames[1], 200).unwrap();
            assert_eq!(&io.io.frames[0][..7], &io.io.frames[1][..7]);
            assert_eq!(first.header.seq, initial_seq);
            assert_eq!(second.header.seq, initial_seq);
            let first = b::audio_bundle_parse(first.payload).unwrap();
            let second = b::audio_bundle_parse(second.payload).unwrap();
            assert_eq!(second.current_data, first.current_data);
            assert_eq!(second.current_seq, first.current_seq);
            assert_eq!(second.stream_id, first.stream_id);
            assert_eq!(second.codec, first.codec);
            assert_eq!(second.flags, b::CURRENT_ACTIVE);
            assert!(second.previous1_data.is_empty() && second.previous2_data.is_empty());
            assert_one_local_sequence(&r, &io, initial_seq);
            assert_eq!(r.audio_state.tx, storage);
            assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (1, 0));
            assert!(!r.audio_state.deferred_pending);
            let c = r.audio_state.counters;
            assert_eq!((c.rf_audio_try, c.rf_audio_ok, c.rf_audio_fail), (1, 1, 0));
            assert_eq!(
                (c.same_slot_refit_attempt, c.same_slot_refit_complete),
                (1, 1)
            );
            assert_eq!((c.prev1_stripped, c.prev2_stripped), (1, 1));
            assert_eq!((c.prev1_forwarded, c.prev2_forwarded), (0, 0));
            assert_eq!((c.tx_count, c.bundle_tx, c.rf_tx_frames), (1, 1, 1));
            assert_eq!(
                (c.rf_rx_audio_ok, c.spi_out_frames, r.protocol.stat_tx_fail),
                (0, 0, 0)
            );
        }
    }

    #[test]
    fn only_proven_not_submitted_eagain_with_optional_history_can_refit() {
        for (history, code, disposition, fresh) in [
            (
                false,
                EAGAIN,
                SubmissionDisposition::NotSubmitted,
                CURRENT_ONLY_AIRTIME_US,
            ),
            (
                true,
                EAGAIN,
                SubmissionDisposition::Unknown,
                CURRENT_ONLY_AIRTIME_US,
            ),
            (
                true,
                EAGAIN,
                SubmissionDisposition::Accepted,
                CURRENT_ONLY_AIRTIME_US,
            ),
            (
                true,
                -5,
                SubmissionDisposition::NotSubmitted,
                CURRENT_ONLY_AIRTIME_US,
            ),
            (
                true,
                EAGAIN,
                SubmissionDisposition::NotSubmitted,
                CURRENT_ONLY_AIRTIME_US - 1,
            ),
        ] {
            let (mut r, mut io) = setup(history);
            let initial_seq = r.protocol.tx_seq;
            io.io.result = code;
            io.disposition = disposition;
            io.fresh_remaining = Some(fresh);
            run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
            assert_eq!(io.io.frames.len(), 1);
            assert_eq!(io.io.frames[0].len(), if history { 161 } else { 65 });
            assert_one_local_sequence(&r, &io, initial_seq);
            assert_eq!(r.audio_state.tx_tail, 1);
            let c = r.audio_state.counters;
            assert_eq!(
                (c.same_slot_refit_attempt, c.same_slot_refit_complete),
                (0, 0)
            );
            assert_eq!((c.rf_audio_try, c.rf_audio_ok, c.rf_audio_fail), (1, 0, 1));
            assert_eq!(r.protocol.stat_tx_fail, 1);
        }
    }

    #[test]
    fn candidate_already_stripped_to_65_bytes_does_not_refit() {
        let (mut r, mut io) = setup(true);
        let initial_seq = r.protocol.tx_seq;
        io.io.remaining = CURRENT_ONLY_AIRTIME_US;
        io.fresh_remaining = Some(10_000);
        run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!((io.io.frames.len(), io.io.frames[0].len()), (1, 65));
        assert_one_local_sequence(&r, &io, initial_seq);
        assert_eq!(r.audio_state.counters.same_slot_refit_attempt, 0);
        assert_eq!(r.audio_state.counters.rf_audio_fail, 1);
    }

    #[test]
    fn optional_prev1_only_candidate_can_refit_after_initial_prev2_strip() {
        let (mut r, mut io) = setup(true);
        let initial_seq = r.protocol.tx_seq;
        io.io.remaining = 725; // Exactly the 113-byte candidate airtime.
        run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!(
            io.io.frames.iter().map(Vec::len).collect::<Vec<_>>(),
            [113, 65]
        );
        assert_one_local_sequence(&r, &io, initial_seq);
        let c = r.audio_state.counters;
        assert_eq!((c.prev1_stripped, c.prev2_stripped), (1, 1));
        assert_eq!(
            (c.same_slot_refit_attempt, c.same_slot_refit_complete),
            (1, 1)
        );
    }

    #[test]
    fn second_rejection_is_final_without_third_request() {
        let (mut r, mut io) = setup(true);
        let initial_seq = r.protocol.tx_seq;
        io.next_result = None; // Both responses prove pre-submit EAGAIN.
        run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!((io.io.async_calls, io.io.frames.len()), (2, 2));
        assert_one_local_sequence(&r, &io, initial_seq);
        let c = r.audio_state.counters;
        assert_eq!(
            (c.same_slot_refit_attempt, c.same_slot_refit_complete),
            (1, 0)
        );
        assert_eq!((c.rf_audio_try, c.rf_audio_ok, c.rf_audio_fail), (1, 0, 1));
        assert_eq!(r.protocol.stat_tx_fail, 1);
        assert_eq!(r.audio_state.tx_tail, 1);
        run(r.audio_slot_tx_handler_async(1, 2, &mut FullAudioPlatform(&mut io)));
        assert_eq!(io.io.async_calls, 2);
        assert_one_local_sequence(&r, &io, initial_seq);
    }

    #[test]
    fn cancellation_of_first_future_neither_emits_nor_refits() {
        let (mut r, mut io) = setup(true);
        let initial_seq = r.protocol.tx_seq;
        let storage = r.audio_state.tx;
        {
            let mut p = FullAudioPlatform(&mut io);
            let mut future = pin!(r.audio_slot_tx_handler_async(1, 1, &mut p));
            assert!(poll(future.as_mut()).is_pending());
            assert!(poll(future.as_mut()).is_pending());
        }
        assert_eq!(
            (io.io.async_calls, io.io.cancelled, io.io.completed),
            (1, 1, 0)
        );
        assert!(io.io.frames.is_empty());
        assert_eq!(r.protocol.tx_seq, initial_seq.wrapping_add(1));
        assert_eq!(r.audio_state.tx_tail, 0);
        assert_eq!(r.audio_state.tx, storage);
        let c = r.audio_state.counters;
        assert_eq!((c.rf_audio_try, c.rf_audio_ok, c.rf_audio_fail), (1, 0, 0));
        assert_eq!(
            (c.same_slot_refit_attempt, c.same_slot_refit_complete),
            (0, 0)
        );
    }

    fn mandatory_setup() -> (Runtime, ClassifiedIo) {
        let (mut r, mut io) = setup(false);
        // Tail seq6 is exactly the active prev1 of successor seq7.
        r.audio_state.tx[0].data[4..6].copy_from_slice(&6u16.to_be_bytes());
        assert_eq!(
            r.audio_send_v2(&bundle(true), &mut FullAudioPlatform(&mut io)),
            0
        );
        r.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
        r.audio_state.deferred_pending = true;
        r.audio_state.deferred_seq = 6;
        (r, io)
    }

    #[test]
    fn mandatory_prev1_failure_retains_both_entries_without_refit() {
        let (mut r, mut io) = mandatory_setup();
        let initial_seq = r.protocol.tx_seq;
        let storage = r.audio_state.tx;
        run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!((io.io.frames.len(), io.io.frames[0].len()), (1, 161));
        assert_one_local_sequence(&r, &io, initial_seq);
        assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (0, 2));
        assert_eq!(r.audio_state.tx, storage);
        assert_eq!(r.audio_state.counters.same_slot_refit_attempt, 0);
        assert_eq!(r.audio_state.counters.local_deferred_recovery, 0);
    }

    #[test]
    fn mandatory_prev1_fit_failure_falls_back_to_original_current() {
        let (mut r, mut io) = mandatory_setup();
        let initial_seq = r.protocol.tx_seq;
        io.io.remaining = CURRENT_ONLY_AIRTIME_US;
        io.io.result = 0;
        run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!((io.io.frames.len(), io.io.frames[0].len()), (1, 65));
        assert_one_local_sequence(&r, &io, initial_seq);
        assert_eq!(
            b::audio_bundle_parse(&io.io.frames[0][9..])
                .unwrap()
                .current_seq,
            6
        );
        assert_eq!((r.audio_state.tx_tail, r.audio_state.tx_depth()), (1, 1));
        assert_eq!(r.audio_state.counters.same_slot_refit_attempt, 0);
        assert_eq!(r.audio_state.counters.local_deferred_recovery, 0);
    }

    #[test]
    fn dual_send_refit_preserves_relay_reserve_and_ring_consumption() {
        for fresh in [2 * CURRENT_ONLY_AIRTIME_US - 1, 2 * CURRENT_ONLY_AIRTIME_US] {
            let (mut r, mut io) = setup(true);
            relay(&mut r, false);
            io.allow_legacy_relay = true;
            let initial_seq = r.protocol.tx_seq;
            io.fresh_remaining = Some(fresh);
            let local_storage = r.audio_state.tx;
            let relay_storage = r.session.relay_ring;
            run(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
            let refits = u32::from(fresh == 2 * CURRENT_ONLY_AIRTIME_US);
            assert_eq!(io.io.frames.len(), 2 + refits as usize);
            assert_eq!(io.legacy_relay_calls, 1);
            assert_eq!((io.io.sync_calls, io.io.cancelled), (0, 0));
            assert_eq!(io.io.async_calls, 2 + refits as usize);
            assert_eq!(io.io.completed, 2 + refits as usize);
            assert_eq!(io.classified_results.len(), 1 + refits as usize);
            assert_eq!(
                io.classified_results[0],
                RadioSendResult {
                    code: EAGAIN,
                    disposition: SubmissionDisposition::NotSubmitted,
                }
            );
            assert_eq!(
                io.classified_results
                    .iter()
                    .filter(|result| result.disposition == SubmissionDisposition::NotSubmitted)
                    .count(),
                1
            );
            if refits != 0 {
                assert_eq!(
                    io.classified_results[1],
                    RadioSendResult {
                        code: 0,
                        disposition: SubmissionDisposition::Accepted,
                    }
                );
            }
            assert_one_local_sequence(&r, &io, initial_seq);
            assert_eq!(io.io.frames.last().unwrap()[2], 3); // Relay still gets its turn.
            assert_eq!((r.audio_state.tx_tail, r.session.relay_tail), (1, 1));
            assert_eq!(r.audio_state.tx, local_storage);
            assert_eq!(r.session.relay_ring, relay_storage);
            assert_eq!(r.audio_state.counters.same_slot_refit_attempt, refits);
            assert_eq!(r.audio_state.counters.rf_audio_try, 1);
            assert_eq!(r.audio_state.counters.rf_audio_fail, 1 - refits);
            assert_eq!(r.audio_state.counters.rf_audio_ok, refits);
            assert!(!r.audio_state.deferred_pending);
            assert!(r.audio_state.relay_contention_turn);
        }
    }
}

#[cfg(not(feature = "same-slot-audio-refit"))]
#[test]
fn default_optional_audio_preserves_single_request_after_proven_rejection() {
    let mut r = active();
    let mut io = classified_io(SubmissionDisposition::NotSubmitted);
    r.audio_send_v2(&bundle(true), &mut FullAudioPlatform(&mut io));
    // Active ingress may allocate a queued SpeakerRequest sequence before audio TX.
    r.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
    let initial_seq = r.protocol.tx_seq;
    finish(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
    assert_eq!((io.io.async_calls, io.io.frames.len()), (1, 1));
    assert_eq!(r.protocol.tx_seq, initial_seq.wrapping_add(1));
    assert_eq!(
        w::decode(&io.io.frames[0], 200).unwrap().header.seq,
        initial_seq
    );
    assert_eq!(io.io.frames[0].len(), 161);
    assert_eq!(r.audio_state.counters.rf_audio_fail, 1);
}

#[test]
fn control_and_priority_commit_only_after_physical_success() {
    for priority in [false, true] {
        for result in [0, -5] {
            let mut r = active();
            let mut io = PendingIo {
                result,
                ..PendingIo::default()
            };
            if priority {
                r.tx_queue_priority(16, &[42]);
            } else {
                r.tx_queue_control(14, &[0; w::Topology::SIZE]);
            }
            finish(r.tx_control_handler_async(1, &mut io));
            assert_eq!(io.frames.len(), 1);
            assert_eq!(io.sync_calls, 0);
            if priority {
                assert_eq!(r.protocol.priority_pending, result != 0);
            } else {
                assert_eq!(r.session.control_tail, u8::from(result == 0));
                assert_eq!(r.protocol.stat_topology_tx_ok, u32::from(result == 0));
                assert_eq!(r.protocol.stat_tx_fail, u32::from(result != 0));
                assert_eq!(r.protocol.tx_seq, if result == 0 { 1 } else { 2 });
            }
        }
        let mut r = active();
        let mut io = PendingIo::default();
        if priority {
            r.tx_queue_priority(16, &[42]);
        } else {
            r.tx_queue_control(14, &[0; w::Topology::SIZE]);
        }
        {
            let mut future = pin!(r.tx_control_handler_async(1, &mut io));
            assert!(poll(future.as_mut()).is_pending());
        }
        assert_eq!(r.protocol.priority_pending, priority);
        assert_eq!(r.session.control_tail, 0);
        assert_eq!(r.protocol.stat_topology_tx_ok, 0);
        assert_eq!(r.protocol.stat_tx_fail, 0);
        assert_eq!(io.cancelled, 1);
    }
}

#[test]
fn sync_timestamp_is_post_success_and_cancellation_keeps_allocated_sequence() {
    for result in [0, -5] {
        let mut r = valid_sync();
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        assert_eq!(finish(r.tx_send_sync_async(&mut io)), result);
        assert_eq!(
            r.protocol.last_sync_time,
            if result == 0 { 1000 } else { 0 }
        );
        assert_eq!(r.protocol.tx_seq, 1);
        assert_eq!((io.async_calls, io.completed, io.sync_calls), (1, 1, 0));
        assert_eq!(io.frames.len(), 1);
        let packet = w::decode(&io.frames[0], 200).unwrap();
        assert_eq!(packet.header.packet_type, 17);
        let sync = w::SyncV3::decode(packet.payload).unwrap();
        assert_eq!((sync.term, sync.member_count, sync.leader_id), (1, 1, 1));
        assert_eq!(sync.leader.address_len, r.local_identity().address_len);
        assert_eq!(sync.leader.address, r.local_identity().address);
    }
    let mut r = valid_sync();
    let mut io = PendingIo::default();
    {
        let mut future = pin!(r.tx_control_handler_async(10, &mut io));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.protocol.last_sync_time, 0);
    assert_eq!(r.protocol.tx_seq, 1);
    assert_eq!(io.cancelled, 1);
    assert_eq!((io.async_calls, io.completed, io.sync_calls), (1, 0, 0));
    assert!(io.frames.is_empty());
}

fn forward(r: &mut Runtime) {
    a::init(Some(&mut r.protocol.adaptive), 1, 2, 1, 7, 0);
    let h = a::Header {
        version: 5,
        packet_type: 19,
        src_id: 3,
        seq: 7,
        ttl: 2,
        flags: 0,
        talk_channel: 1,
        payload_len: w::HandoverAck::SIZE as u16,
    };
    assert!(a::forward_enqueue(
        Some(&mut r.protocol.adaptive),
        Some(&h),
        Some(&[0; w::HandoverAck::SIZE]),
        None,
        1,
        true,
        0
    ));
}
#[test]
fn cancelled_forward_retains_source_pre_send_dequeue() {
    let mut r = active();
    forward(&mut r);
    let mut io = PendingIo::default();
    {
        let mut future = pin!(r.tx_control_handler_async(1, &mut io));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.protocol.adaptive.forward_queue_len(), 0);
    assert_eq!(r.protocol.control_turn, 1);
    assert_eq!(r.protocol.stat_tx_fail, 0);
    assert_eq!(io.cancelled, 1);
    crate::corefuture::ready(r.tx_control_handler_async(9, &mut io));
    assert_eq!(io.async_calls, 1);
}

#[test]
fn join_timer_guards_consumption_and_post_send_retry_effects() {
    for result in [0, -5] {
        let mut r = active();
        r.membership.state = State::Joining;
        let token = TimerToken {
            timer: Timer::Join,
            generation: 1,
            deadline_us: 1_000_000,
        };
        r.timers[Timer::Join as usize] = Some(token);
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        let mut p = FullProtocolPlatform(&mut io);
        let stale = TimerToken {
            generation: 2,
            ..token
        };
        assert_eq!(
            crate::corefuture::ready(r.handle_async(Event::Timer(stale), &mut p)),
            0
        );
        let early = TimerToken {
            deadline_us: 1_000_001,
            ..token
        };
        r.timers[Timer::Join as usize] = Some(early);
        assert_eq!(
            crate::corefuture::ready(r.handle_async(Event::Timer(early), &mut p)),
            0
        );
        assert_eq!(r.timer(Timer::Join), Some(early));
        r.timers[Timer::Join as usize] = Some(token);
        assert_eq!(finish(r.handle_async(Event::Timer(token), &mut p)), 0);
        assert_eq!(r.protocol.join_attempts, 1);
        assert_ne!(r.timer(Timer::Join), Some(token));
        assert_eq!(io.async_calls, 1);
        assert_eq!(io.frames[0][1], 15);
    }
    let mut r = active();
    r.membership.state = State::Joining;
    let token = TimerToken {
        timer: Timer::Join,
        generation: 1,
        deadline_us: 1_000_000,
    };
    r.timers[Timer::Join as usize] = Some(token);
    let mut io = PendingIo::default();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.timer(Timer::Join), None);
    assert_eq!(r.protocol.join_attempts, 0);
    assert_eq!(r.protocol.tx_seq, 1);
    assert!(!io.effects.iter().any(|e| matches!(e, Effect::Schedule(_))));
    assert_eq!(io.cancelled, 1);
}

#[test]
fn unowned_join_uses_scripted_sync_effect_and_sync_join_never_polls_async() {
    let mut r = active();
    r.membership.state = State::Joining;
    let mut io = PendingIo {
        owned_tx: false,
        ..PendingIo::default()
    };
    crate::corefuture::ready(
        r.membership_handle_async(Input::JoinWork, &mut CombinedMembershipPlatform(&mut io)),
    )
    .unwrap();
    assert_eq!(r.protocol.join_attempts, 1);
    assert_eq!((io.async_calls, io.sync_calls), (0, 0));
    r.membership_handle(Input::JoinWork, &mut CombinedFullTxPlatform(&mut io))
        .unwrap();
    assert_eq!((io.async_calls, io.sync_calls), (0, 1));
    assert_eq!(r.protocol.join_attempts, 2);
}

#[test]
fn local_audio_pending_success_failure_and_drop_preserve_source_order() {
    for result in [0, -5] {
        let mut r = active();
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        assert_eq!(
            r.audio_send_v2(&bundle(false), &mut FullAudioPlatform(&mut io)),
            0
        );
        finish(r.audio_slot_tx_handler_async(1, 1, &mut FullAudioPlatform(&mut io)));
        assert_eq!(r.audio_state.tx_tail, 1);
        assert_eq!(r.audio_state.counters.rf_audio_ok, u32::from(result == 0));
        assert_eq!(r.audio_state.counters.rf_audio_fail, u32::from(result != 0));
        assert_eq!(io.frames.len(), 1);
        assert_eq!(io.sync_calls, 0);
    }
    let mut r = active();
    let mut io = PendingIo::default();
    r.audio_send_v2(&bundle(false), &mut FullAudioPlatform(&mut io));
    r.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
    let seq_before = r.protocol.tx_seq;
    {
        let mut p = FullAudioPlatform(&mut io);
        let mut future = pin!(r.audio_slot_tx_handler_async(1, 1, &mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.protocol.tx_seq, seq_before.wrapping_add(1));
    assert_eq!(r.audio_state.tx_tail, 0);
    assert_eq!(r.audio_state.counters.rf_audio_try, 1);
    assert_eq!(r.audio_state.counters.rf_audio_ok, 0);
    assert_eq!(r.audio_state.counters.rf_audio_fail, 0);
    assert_eq!(io.cancelled, 1);
    assert!(io.frames.is_empty());
}

fn relay(r: &mut Runtime, history: bool) {
    let payload = bundle(history);
    let entry = &mut r.session.relay_ring[0];
    let h = w::Header {
        version: w::VERSION,
        packet_type: 13,
        src_id: 3,
        seq: 7,
        ttl: 2,
        flags: 5,
        talk_channel: 1,
        payload_len: payload.len() as u16,
    };
    entry.len = w::encode(&h, &payload, &mut entry.data, 200).unwrap() as u8;
    entry.queued_ms = 1000;
    r.session.relay_head = 1;
}

#[test]
fn relay_completion_consumes_even_failure_but_pending_drop_does_not() {
    for result in [0, -5] {
        let mut r = active();
        relay(&mut r, false);
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        finish(r.audio_slot_tx_handler_async(1, 1, &mut io));
        assert_eq!(r.session.relay_tail, 1);
        assert_eq!(r.audio_state.counters.tx_count, u32::from(result == 0));
        assert_eq!(r.protocol.stat_tx_fail, u32::from(result != 0));
        assert_eq!(io.frames.len(), 1);
    }
    let mut r = active();
    relay(&mut r, true);
    let before = r.session.relay_ring[0];
    let mut io = PendingIo {
        remaining: 600,
        ..PendingIo::default()
    };
    {
        let mut future = pin!(r.audio_slot_tx_handler_async(1, 1, &mut io));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.session.relay_tail, 0);
    assert_eq!(r.session.relay_ring[0], before);
    assert_eq!(r.audio_state.counters.prev1_stripped, 1);
    assert_eq!(r.audio_state.counters.prev2_stripped, 1);
    assert_eq!(r.audio_state.counters.tx_count, 0);
    assert_eq!(r.protocol.stat_tx_fail, 0);
    assert_eq!(io.cancelled, 1);
}

#[test]
fn async_audio_probe_awaits_both_local_and_relay_without_duplicate_emit() {
    for kind in [0, 1] {
        for result in [0, -5] {
            let mut r = active();
            let mut io = PendingIo {
                result,
                ..PendingIo::default()
            };
            let raw = if kind == 0 {
                bundle(false)
            } else {
                relay(&mut r, false);
                let entry = r.session.relay_ring[0];
                entry.data[..usize::from(entry.len)].to_vec()
            };
            let ret = finish(r.audio_test_transmit_probe_async(&raw, 13, kind, 0, &mut io));
            assert_eq!(
                ret,
                if kind == 0 {
                    i32::from(result != 0)
                } else {
                    result
                }
            );
            assert_eq!((io.async_calls, io.completed, io.sync_calls), (1, 1, 0));
            assert_eq!(io.frames.len(), 1);
        }
    }
}

#[test]
fn contention_drop_keeps_turn_and_sequence_but_not_local_consumption() {
    let mut r = active();
    let mut io = PendingIo::default();
    r.audio_send_v2(&bundle(false), &mut FullAudioPlatform(&mut io));
    relay(&mut r, false);
    let old = r.audio_state.relay_contention_turn;
    {
        let mut future = pin!(r.audio_slot_tx_handler_async(1, 1, &mut io));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.audio_state.relay_contention_turn, !old);
    assert_eq!(r.session.relay_tail, 0);
    assert_eq!(r.audio_state.tx_tail, 0);
    assert_eq!(r.audio_state.counters.tx_count, 0);
    assert_eq!(io.cancelled, 1);
}

#[test]
fn queued_stop_waits_for_leave_before_reset_discard_and_ack() {
    for result in [0, -5] {
        let mut r = active();
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        {
            let mut p = FullProtocolPlatform(&mut io);
            r.request_stop(7, &mut p);
            assert_eq!(finish(r.handle_async(Event::DispatchCommands, &mut p)), 0);
        }
        assert_eq!(r.membership.state, State::Idle);
        assert_eq!((io.async_calls, io.completed, io.sync_calls), (1, 1, 0));
        let leave = w::decode(&io.frames[0], 200).unwrap();
        assert_eq!(leave.header.packet_type, 4);
        assert_eq!(leave.payload, &[1, 2, 3, 4, 5]);
        assert_eq!(leave.header.src_id, 1);
        let stopped = io
            .effects
            .iter()
            .position(|e| *e == Effect::MeshStopped)
            .unwrap();
        let discard = io
            .effects
            .iter()
            .position(|e| *e == Effect::DiscardBridgeAudio)
            .unwrap();
        let ack = io
            .effects
            .iter()
            .position(|e| matches!(e, Effect::Ack { .. }))
            .unwrap();
        assert!(stopped < discard && discard < ack);
    }
    let mut r = active();
    let mut io = PendingIo::default();
    {
        let mut p = FullProtocolPlatform(&mut io);
        r.request_stop(7, &mut p);
        let mut future = pin!(r.handle_async(Event::DispatchCommands, &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.membership.state, State::Active);
    assert!(!r.audio_state.ingress_enabled);
    assert_eq!(r.pending_request(), 0);
    assert_eq!(r.protocol.tx_seq, 1);
    assert_eq!(io.cancelled, 1);
    assert!(io.frames.is_empty());
    assert!(!io.effects.iter().any(|e| matches!(
        e,
        Effect::Ack { .. }
            | Effect::MeshStopped
            | Effect::DiscardBridgeAudio
            | Effect::StopRx
            | Effect::TdmaStop
    )));
    assert_eq!(
        io.effects
            .iter()
            .filter(|e| matches!(e, Effect::Cancel(_)))
            .count(),
        3
    );
}

#[test]
fn direct_async_stop_and_join_request_return_only_after_physical_result() {
    for result in [0, -5] {
        let mut r = active();
        let mut io = PendingIo {
            result,
            ..PendingIo::default()
        };
        assert_eq!(finish(r.tx_send_join_request_async(&mut io)), result);
        assert_eq!(
            finish(
                FullProtocolPlatform(&mut io)
                    .dispatch_protocol_async(&mut r, Effect::Leave([1, 2, 3, 4, 5]))
            ),
            Some(Reply::Value(result))
        );
        finish(r.stop_async(&mut FullProtocolPlatform(&mut io)));
        assert_eq!(r.membership.state, State::Idle);
        assert_eq!(io.frames.len(), 3);
        assert_eq!(io.sync_calls, 0);
    }
}

struct MissingHook;
impl Platform for MissingHook {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        1_000_000
    }
    fn owned_protocol_algorithms(&self) -> bool {
        true
    }
    fn apply(&mut self, e: Effect) -> Reply {
        assert!(!matches!(e, Effect::SetChannel(_) | Effect::StartRx));
        Reply::Value(0)
    }
}
#[test]
fn async_entrypoints_preserve_missing_hook_injection_and_initialization_guards() {
    let mut r = Runtime::default();
    let mut p = MissingHook;
    assert_eq!(r.init(&mut p), ENOTSUP);
    r.request_start(7, 1, &mut p);
    crate::corefuture::ready(r.command_work_handler_async(&mut p));
    assert_eq!(r.membership.state, State::Idle);
    let membership = active().membership;
    assert_eq!(
        crate::corefuture::ready(r.handle_async(Event::InjectMembership(membership), &mut p)),
        ENOTSUP
    );
    assert_eq!(
        crate::corefuture::ready(r.handle_async(Event::Membership(membership), &mut p)),
        ENOTSUP
    );
    let token = TimerToken {
        timer: Timer::Join,
        generation: 1,
        deadline_us: 1_000_000,
    };
    r.timers[Timer::Join as usize] = Some(token);
    assert_eq!(
        crate::corefuture::ready(r.handle_async(Event::Timer(token), &mut p)),
        ENOTSUP
    );
    assert_eq!(r.timer(Timer::Join), Some(token));
    let mut p = crate::platform::NotIntegrated;
    assert_eq!(
        crate::corefuture::ready(r.membership_handle_async(Input::JoinWork, &mut p)),
        Err(crate::protocol_membership::Error::NotIntegrated)
    );
}

struct ImmediateIo(PendingIo);
impl Platform for ImmediateIo {
    fn integration(&self) -> Integration {
        self.0.integration()
    }
    fn now_us(&self) -> i64 {
        self.0.now_us()
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.0.apply(e)
    }
}
impl MembershipPlatform for ImmediateIo {
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.0.membership_effect(e, s)
    }
}
impl AdaptivePlatform for ImmediateIo {
    fn adaptive_effect(&mut self, e: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply {
        self.0.adaptive_effect(e, io)
    }
    fn owned_tx_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_effect(e)
    }
    fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
        self.0.clock_snapshot()
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.0.radio_send(packet)
    }
}
#[test]
fn default_async_hook_is_immediate_and_matches_sync_packet_bytes_and_effects() {
    for result in [0, -5] {
        let mut sync = active();
        let mut awaited = active();
        let mut io = ImmediateIo(PendingIo {
            result,
            ..PendingIo::default()
        });
        let mut reference = ImmediateIo(PendingIo {
            result,
            ..PendingIo::default()
        });
        assert_eq!(
            sync.tx_send_packet_ex(4, &[1, 2, 3, 4, 5], 2, 3, 4, 5, &mut reference),
            crate::corefuture::ready(awaited.tx_send_packet_ex_async(
                4,
                &[1, 2, 3, 4, 5],
                2,
                3,
                4,
                5,
                &mut io
            ))
        );
        assert_eq!(
            sync.tx_send_packet(4, &[1, 2, 3, 4, 5], &mut reference),
            crate::corefuture::ready(awaited.tx_send_packet_async(4, &[1, 2, 3, 4, 5], &mut io))
        );
        assert_eq!(
            sync.tx_send_join_request(&mut reference),
            crate::corefuture::ready(awaited.tx_send_join_request_async(&mut io))
        );
        assert_eq!(
            sync.tx_send_sync(&mut reference),
            crate::corefuture::ready(awaited.tx_send_sync_async(&mut io))
        );
        sync.tx_queue_control(14, &[0; w::Topology::SIZE]);
        awaited.tx_queue_control(14, &[0; w::Topology::SIZE]);
        sync.tx_control_handler(1, &mut reference);
        crate::corefuture::ready(awaited.tx_control_handler_async(1, &mut io));
        assert_eq!(io.0.frames, reference.0.frames);
        assert_eq!(awaited.protocol.tx_seq, sync.protocol.tx_seq);
        assert_eq!(
            awaited.protocol.last_sync_time,
            sync.protocol.last_sync_time
        );
        assert_eq!(awaited.protocol.stat_tx_fail, sync.protocol.stat_tx_fail);
        assert_eq!(awaited.session.control_ring, sync.session.control_ring);
        assert_eq!(awaited.session.control_tail, sync.session.control_tail);
        assert_eq!(io.0.async_calls, 0);
    }
}

#[test]
fn synchronous_physical_apis_never_poll_backend_async_override() {
    let mut r = valid_sync();
    let mut io = PendingIo::default();
    r.tx_send_packet_ex(4, &[1, 2, 3, 4, 5], 0, 0, 1, 2, &mut io);
    r.tx_send_packet(4, &[1, 2, 3, 4, 5], &mut io);
    r.tx_send_join_request(&mut io);
    assert_eq!(r.tx_send_sync(&mut io), 0);
    assert_eq!(io.frames.len(), 4);
    let packet = w::decode(&io.frames[3], 200).unwrap();
    assert_eq!(packet.header.packet_type, 17);
    let sync = w::SyncV3::decode(packet.payload).unwrap();
    assert_eq!((sync.term, sync.member_count, sync.leader_id), (1, 1, 1));
    r.tx_queue_priority(16, &[42]);
    r.tx_control_handler(1, &mut io);
    r.audio_send_v2(&bundle(false), &mut FullAudioPlatform(&mut io));
    r.audio_slot_tx_handler(1, 1, &mut io);
    r.audio_test_transmit_probe(&bundle(false), 13, 0, 0, &mut io);
    r.membership.state = State::Joining;
    r.membership_handle(Input::JoinWork, &mut FullProtocolPlatform(&mut io))
        .unwrap();
    r.membership.state = State::Active;
    r.stop(&mut FullProtocolPlatform(&mut io));
    assert_eq!(io.async_calls, 0);
    assert_eq!(io.cancelled, 0);
    assert_eq!(io.sync_calls, 9);
}
