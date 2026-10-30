use core::future::{poll_fn, Future};
use core::pin::{pin, Pin};
use core::task::{Context, Poll, Waker};
use std::{cell::RefCell, rc::Rc, vec::Vec};

use crate::platform::{Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{
    AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply, CombinedMembershipPlatform,
};
use crate::protocol_audio::{AudioEffect, AudioPlatform, FullAudioPlatform};
use crate::protocol_membership::{MembershipEffect, MembershipPlatform, ProtocolState};
use crate::protocol_runtime::FullProtocolPlatform;
use crate::protocol_rx::{FullRxPlatform, RxEffect, RxPlatform};
use crate::protocol_tx::CombinedFullTxPlatform;
use crate::runtime::{Role, Runtime, State};
use openhelmet_mesh::{adaptive as a, core as c};
use openhelmet_wire::{audio_bundle as b, mesh as w};

#[derive(Debug, PartialEq, Eq)]
enum Call {
    Time(i64),
    Sync(u8, Vec<u8>),
    Start(u8, Vec<u8>),
    Result(i64),
}
struct Io {
    calls: Rc<RefCell<Vec<Call>>>,
    now_ms: i64,
    completed_ms: i64,
    result: i64,
}
impl Io {
    fn new(result: i64, completed_ms: i64) -> Self {
        Self {
            calls: Rc::default(),
            now_ms: 100,
            completed_ms,
            result,
        }
    }
    async fn deliver(&mut self, e: AudioEffect<'_>) -> i64 {
        let AudioEffect::BridgeDeliver { source, bundle } = e else {
            panic!("only BridgeDeliver may be awaited");
        };
        self.calls
            .borrow_mut()
            .push(Call::Start(source, bundle.to_vec()));
        let mut polls = 0;
        poll_fn(|_| {
            polls += 1;
            if polls <= 2 {
                Poll::Pending
            } else {
                self.now_ms = self.completed_ms;
                self.calls.borrow_mut().push(Call::Result(self.result));
                Poll::Ready(self.result)
            }
        })
        .await
    }
}
impl Platform for Io {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now_ms * 1000
    }
    fn apply(&mut self, _: Effect) -> Reply {
        Reply::None
    }
}
impl MembershipPlatform for Io {
    fn uptime_ms(&mut self) -> i64 {
        self.calls.borrow_mut().push(Call::Time(self.now_ms));
        self.now_ms
    }
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
}
impl AdaptivePlatform for Io {
    fn adaptive_effect(&mut self, _: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        panic!("unexpected adaptive IO");
    }
    fn owned_audio_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.audio_io(e)
    }
    async fn audio_effect_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.deliver(e).await
    }
}
impl AudioPlatform for Io {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        if let AudioEffect::BridgeDeliver { source, bundle } = e {
            self.calls
                .borrow_mut()
                .push(Call::Sync(source, bundle.to_vec()));
            self.result
        } else {
            0
        }
    }
    async fn audio_io_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.deliver(e).await
    }
}
impl RxPlatform for Io {
    fn rx_io(&mut self, _: RxEffect) -> i32 {
        1
    }
}
fn fixture() -> (Runtime, Vec<u8>) {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    r.membership.node_id = 2;
    r.audio_state.boundary.enabled = true;
    r.audio_state.boundary.receipt_cutoff_us = 100_000;
    r.session.peers[0].active = true;
    r.session.peers[0].node_id = 3;
    a::init(Some(&mut r.protocol.adaptive), 2, 1, 1, 1, 100);
    let identity = a::Identity {
        address_len: 5,
        address: [3, 3, 3, 3, 3, 0],
    };
    assert!(a::set_member(
        Some(&mut r.protocol.adaptive),
        3,
        Some(&identity)
    ));
    r.audio_apply_speaker_grant(1, [3, 0], [c::node_bit(2), 0]);
    let mut packet = [0; 209];
    let len = b::audio_bundle_encode(
        &b::BundleView {
            current_data: &[42; 48],
            previous1_data: &[],
            previous2_data: &[],
            current_seq: 7,
            stream_id: 1,
            flags: b::CURRENT_ACTIVE | b::RELAYED,
            codec: 2,
        },
        &mut packet[9..],
    );
    packet[..9].copy_from_slice(&[w::VERSION, 13, 3, 9, 2, 5, r.talk_channel, len as u8, 0]);
    (r, packet[..9 + len].to_vec())
}
#[test]
fn replay_admission_blocks_evicted_audio_and_accepts_relay_first_and_restart() {
    let (mut r, mut packet) = fixture();
    let mut io = Io::new(0, 100);
    // Three hops: origin -> relay receiver -> destination. The destination
    // first hears the relayed envelope, then a direct duplicate.
    r.audio_process_rx_packet(&packet, -40, 100_000, &mut io)
        .unwrap();
    assert_eq!(r.session.relay_head, 1);
    let (mut destination, _) = fixture();
    packet[4] = 1;
    packet[5] |= 2;
    destination
        .audio_process_rx_packet(&packet, -40, 100_000, &mut io)
        .unwrap();
    assert_eq!(destination.audio_state.counters().audio_fwd, 1);
    packet[4] = 2;
    packet[5] &= !2;
    destination
        .audio_process_rx_packet(&packet, -40, 100_000, &mut io)
        .unwrap();
    assert_eq!(destination.audio_state.counters().audio_fwd, 1);
    for seq in 8u16..=1000 {
        packet[3] = seq as u8;
        packet[13..15].copy_from_slice(&seq.to_be_bytes());
        r.audio_process_rx_packet(&packet, -40, 100_000, &mut io)
            .unwrap();
    }
    let before = r.audio_state.counters().audio_fwd;
    packet[3] = 9;
    packet[13..15].copy_from_slice(&7u16.to_be_bytes());
    r.audio_process_rx_packet(&packet, -40, 100_000, &mut io)
        .unwrap();
    assert_eq!(r.audio_state.counters().audio_fwd, before);
    // Same binding quick reboot/rejoin is not necessarily visible remotely.
    // Rejected restarted packets cannot postpone recovery indefinitely.
    packet[13..15].copy_from_slice(&0u16.to_be_bytes());
    io.now_ms = 100 + i64::from(a::EXPIRE_MS);
    r.audio_process_rx_packet(&packet, -40, io.now_ms * 1000, &mut io)
        .unwrap();
    assert_eq!(r.audio_state.counters().audio_fwd, before);
    io.now_ms += 1;
    r.audio_process_rx_packet(&packet, -40, io.now_ms * 1000, &mut io)
        .unwrap();
    assert_eq!(r.audio_state.counters().audio_fwd, before + 1);
    c::dedupe_purge_node(Some(&mut r.dedupe), 3);
    r.audio_process_rx_packet(&packet, -40, io.now_ms * 1000, &mut io)
        .unwrap();
    assert_eq!(r.audio_state.counters().audio_fwd, before + 2);
}

fn poll<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}
fn finish<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    assert!(poll(future.as_mut()).is_pending());
    assert!(poll(future.as_mut()).is_pending());
    match poll(future.as_mut()) {
        Poll::Ready(result) => result,
        Poll::Pending => panic!("delivery did not complete"),
    }
}
fn assert_result(r: &Runtime, result: i64, relay: bool) {
    let counters = r.audio_state.counters();
    assert_eq!(counters.bundle_rx, 1);
    assert_eq!(counters.rf_rx_audio_ok, 1);
    assert_eq!(counters.rf_rx_frames, 1);
    assert_eq!(counters.audio_fwd, u32::from(result == 0));
    assert_eq!(counters.spi_out_ok, u32::from(result == 0));
    assert_eq!(counters.spi_out_frames, u32::from(result == 0));
    assert_eq!(counters.spi_out_drop, u32::from(result != 0));
    assert_eq!(r.session.relay_head, u8::from(relay));
}

#[test]
fn pending_success_and_error_forward_through_all_five_wrappers() {
    macro_rules! check {
        ($wrapper:ident) => {
            for result in [0, -5] {
                let (mut r, packet) = fixture();
                let mut io = Io::new(result, 120);
                assert_eq!(finish(r.audio_process_rx_packet_async(
                    &packet, -40, 100_000, &mut $wrapper(&mut io)
                )), Ok(true));
                assert_result(&r, result, true);
                let calls = io.calls.borrow();
                assert_eq!(calls.len(), 6);
                assert!(matches!(&calls[3], Call::Start(3, bytes) if bytes[3] & b::RELAYED == 0));
                assert_eq!(calls[4], Call::Result(result));
                assert_eq!(calls[5], Call::Time(120));
                drop(calls);
                let (mut sync_r, packet) = fixture();
                let mut sync_io = Io::new(result, 121);
                assert_eq!(sync_r.audio_process_rx_packet(
                    &packet, -40, 100_000, &mut $wrapper(&mut sync_io)
                ), Ok(true));
                assert_result(&sync_r, result, true);
                assert!(sync_io.calls.borrow().iter().all(|call|
                    !matches!(call, Call::Start(..) | Call::Result(..))));
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
fn relay_age_is_queried_after_delivery_not_before_pending() {
    for result in [0, -5] {
        let (mut r, packet) = fixture();
        let mut io = Io::new(result, 121);
        finish(r.audio_process_rx_packet_async(
            &packet,
            -40,
            100_000,
            &mut FullAudioPlatform(&mut io),
        ))
        .unwrap();
        assert_result(&r, result, false);
        assert_eq!(io.calls.borrow().last(), Some(&Call::Time(121)));
    }
}

#[test]
fn cancellation_keeps_preawait_rx_state_without_delivery_result_or_replay() {
    let (mut r, packet) = fixture();
    let mut io = Io::new(0, 120);
    {
        let mut p = FullAudioPlatform(&mut io);
        let mut future = pin!(r.audio_process_rx_packet_async(&packet, -40, 100_000, &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
    }
    let counters = r.audio_state.counters();
    assert_eq!(
        (
            counters.bundle_rx,
            counters.rf_rx_audio_ok,
            counters.rf_rx_frames
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (
            counters.audio_fwd,
            counters.spi_out_ok,
            counters.spi_out_frames,
            counters.spi_out_drop
        ),
        (0, 0, 0, 0)
    );
    assert_eq!(r.session.relay_head, 0);
    assert_eq!(r.session.peers[0].last_seen_ms, 100);
    assert_eq!(r.session.peers[0].rssi_dbm, -40);
    assert_eq!(r.session.heard_bitmap, c::node_bit(3));
    assert_eq!(r.session.speaker_deadlines_ms[3], 1600);
    assert!(r.audio_state.rf_seq[3].initialized);
    assert_eq!(r.audio_state.rf_seq[3].last, 7);
    assert_eq!(io.calls.borrow().len(), 4);
    assert_eq!(
        crate::corefuture::ready(r.audio_process_rx_packet_async(
            &packet,
            -40,
            100_000,
            &mut FullAudioPlatform(&mut io)
        )),
        Ok(true)
    );
    assert_eq!(r.audio_state.counters().rf_rx_duplicate_drop, 1);
    assert_eq!(io.calls.borrow().len(), 5);
}

#[test]
fn synchronous_audio_and_rx_routes_never_poll_async_override() {
    for route in 0..3 {
        let (mut r, packet) = fixture();
        let mut io = Io::new(-5, 121);
        let mut p = FullRxPlatform(&mut io);
        match route {
            0 => assert_eq!(
                r.audio_process_rx_packet(&packet, -40, 100_000, &mut p),
                Ok(true)
            ),
            1 => r.rx_process_packet(&packet, -40, 100_000, &mut p).unwrap(),
            _ => {
                r.rx_publish(&packet, -40, 100_000, &mut p).unwrap();
                r.rx_work_handler(&mut p).unwrap();
            }
        }
        assert_result(&r, -5, true);
        assert!(io
            .calls
            .borrow()
            .iter()
            .all(|call| !matches!(call, Call::Start(..) | Call::Result(..))));
    }
}

#[test]
fn async_rx_packet_and_work_routes_await_the_actual_leaf() {
    for work in [false, true] {
        for result in [0, -5] {
            let (mut r, packet) = fixture();
            let mut io = Io::new(result, 120);
            let mut p = FullRxPlatform(&mut io);
            if work {
                r.rx_publish(&packet, -40, 100_000, &mut p).unwrap();
                finish(r.rx_work_handler_async(&mut p)).unwrap();
                assert_eq!(r.rx_state.cursors(), (1, 1));
            } else {
                finish(r.rx_process_packet_async(&packet, -40, 100_000, &mut p)).unwrap();
            }
            assert_eq!(r.rx_state.count(), 1);
            assert_result(&r, result, true);
        }
    }
}

#[test]
fn cancelled_rx_work_keeps_popped_entry_and_leaves_later_record_queued() {
    let (mut r, packet) = fixture();
    let mut io = Io::new(0, 120);
    let mut second = packet.clone();
    second[3] += 1;
    second[13..15].copy_from_slice(&8u16.to_be_bytes());
    {
        let mut p = FullRxPlatform(&mut io);
        r.rx_publish(&packet, -40, 100_000, &mut p).unwrap();
        r.rx_publish(&second, -40, 100_000, &mut p).unwrap();
        let mut future = pin!(r.rx_work_handler_async(&mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.rx_state.cursors(), (2, 1));
    assert!(!r.rx_state.work_pending());
    assert_eq!(r.rx_state.count(), 1);
    assert_eq!(r.rx_state.storage()[0].data[..packet.len()], packet);
    assert_eq!(r.audio_state.counters().spi_out_ok, 0);
    assert_eq!(r.audio_state.counters().spi_out_drop, 0);
    finish(r.rx_work_handler_async(&mut FullRxPlatform(&mut io))).unwrap();
    assert_eq!(r.rx_state.cursors(), (2, 2));
    assert_eq!(r.rx_state.count(), 2);
    assert_eq!(
        io.calls
            .borrow()
            .iter()
            .filter(|call| matches!(call, Call::Start(..)))
            .count(),
        2
    );
    assert_eq!(r.audio_state.counters().spi_out_ok, 1);
}

#[test]
fn closed_or_stale_delivery_gate_never_awaits() {
    for (enabled, received_us) in [(false, 100_000), (true, 99_999)] {
        let (mut r, packet) = fixture();
        r.audio_state.boundary.enabled = enabled;
        let mut io = Io::new(0, 120);
        assert_eq!(
            crate::corefuture::ready(r.audio_process_rx_packet_async(
                &packet,
                -40,
                received_us,
                &mut FullAudioPlatform(&mut io)
            )),
            Ok(true)
        );
        assert_eq!(r.audio_state.counters().spi_out_ok, 0);
        let counters = r.audio_state.counters();
        assert_eq!(
            (
                counters.bundle_rx,
                counters.rf_rx_audio_ok,
                counters.rf_rx_frames
            ),
            (1, 1, 1)
        );
        assert_eq!(
            (
                counters.audio_fwd,
                counters.spi_out_frames,
                counters.spi_out_drop
            ),
            (0, 0, 0)
        );
        assert_eq!(r.session.peers[0].last_seen_ms, 100);
        assert_eq!(r.session.peers[0].rssi_dbm, -40);
        assert_eq!(r.session.heard_bitmap, c::node_bit(3));
        assert_eq!(r.session.speaker_deadlines_ms[3], 1600);
        assert!(r.audio_state.rf_seq[3].initialized);
        assert_eq!(r.audio_state.rf_seq[3].last, 7);
        assert_eq!(r.session.relay_head, 1);
        assert!(io
            .calls
            .borrow()
            .iter()
            .all(|call| matches!(call, Call::Time(_))));
    }
}

struct SyncOnly(Io);
impl Platform for SyncOnly {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.0.now_us()
    }
    fn apply(&mut self, _: Effect) -> Reply {
        Reply::None
    }
}
impl MembershipPlatform for SyncOnly {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
}
impl AdaptivePlatform for SyncOnly {
    fn adaptive_effect(&mut self, _: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        panic!("unexpected adaptive IO");
    }
    fn owned_audio_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
}
impl AudioPlatform for SyncOnly {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
}

#[test]
fn both_async_trait_hooks_default_to_synchronous_delivery() {
    for typed in [false, true] {
        for result in [0, -5] {
            let (mut r, packet) = fixture();
            let mut io = SyncOnly(Io::new(result, 120));
            let outcome = if typed {
                crate::corefuture::ready(r.audio_process_rx_packet_async(
                    &packet,
                    -40,
                    100_000,
                    &mut FullAudioPlatform(&mut io),
                ))
            } else {
                crate::corefuture::ready(
                    r.audio_process_rx_packet_async(&packet, -40, 100_000, &mut io),
                )
            };
            assert_eq!(outcome, Ok(true));
            assert_result(&r, result, true);
            assert!(matches!(&io.0.calls.borrow()[0], Call::Sync(3, _)));
        }
    }
}

#[test]
fn typed_async_io_forwards_through_nested_full_wrappers() {
    let mut io = Io::new(-5, 120);
    let mut protocol = FullProtocolPlatform(&mut io);
    let mut rx = FullRxPlatform(&mut protocol);
    assert_eq!(
        finish(rx.audio_io_async(AudioEffect::BridgeDeliver {
            source: 3,
            bundle: &[1, 2, 3],
        })),
        -5
    );
    assert_eq!(
        *io.calls.borrow(),
        [Call::Start(3, std::vec![1, 2, 3]), Call::Result(-5)]
    );
}
