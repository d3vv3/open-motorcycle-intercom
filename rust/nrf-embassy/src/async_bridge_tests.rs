use core::future::{poll_fn, Future};
use core::pin::{pin, Pin};
use core::task::{Context, Poll, Waker};
use std::{
    cell::{Cell, RefCell},
    format,
    rc::Rc,
    vec,
    vec::Vec,
};

use crate::platform::{AudioBoundary, Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{
    AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply, CombinedMembershipPlatform,
};
use crate::protocol_audio::{AudioEffect, AudioPlatform, FullAudioPlatform};
use crate::protocol_membership::{
    Input, MembershipEffect, MembershipOutput, MembershipPlatform, ProtocolState,
};
use crate::protocol_runtime::FullProtocolPlatform;
use crate::protocol_rx::{FullRxPlatform, RxEffect, RxPlatform};
use crate::protocol_tx::{CombinedFullTxPlatform, RadioSend};
use crate::runtime::{Event, Membership, MembershipTransition, Role, Runtime, State, Timer};
use openhelmet_mesh::adaptive as a;
use openhelmet_wire::{audio_bundle as b, mesh as w};

#[derive(Clone, Debug, PartialEq, Eq)]
enum Output {
    Runtime(Effect),
    Membership(MembershipOutput),
    Adaptive(Membership),
    Audio(u8),
}
#[derive(Clone, Debug, PartialEq, Eq)]
enum Call {
    Sync(Effect),
    Member(MembershipEffect),
    Adaptive(AdaptiveEffect),
    Start(Output),
    Done(Output),
    Radio,
    AudioNow,
}
struct Io {
    integration: Integration,
    tdma_timestamp: Option<Option<i64>>,
    physical_stamps: Vec<i64>,
    clock: Option<crate::protocol_tx::ClockSnapshot>,
    boundaries: Rc<RefCell<Vec<crate::protocol_audio::IngressAdmissionGate>>>,
    window_valid: Rc<Cell<bool>>,
    gates: Rc<RefCell<Vec<(crate::protocol_audio::IngressAdmissionGate, bool)>>>,
    trace: Rc<RefCell<Vec<Call>>>,
    now_us: i64,
    result: i32,
    epoch: u32,
    lost: bool,
    async_only: bool,
    sync_outputs: usize,
    bridge_enabled: bool,
}
impl Default for Io {
    fn default() -> Self {
        Self {
            integration: Integration::ActorOracle,
            tdma_timestamp: None,
            physical_stamps: Vec::new(),
            clock: None,
            boundaries: Rc::default(),
            window_valid: Rc::new(Cell::new(true)),
            gates: Rc::default(),
            trace: Rc::default(),
            now_us: 100_000,
            result: 0,
            epoch: 0,
            lost: false,
            async_only: false,
            sync_outputs: 0,
            bridge_enabled: true,
        }
    }
}
impl Io {
    fn async_only() -> Self {
        Self {
            async_only: true,
            ..Self::default()
        }
    }
    fn sync_output(&mut self) {
        self.sync_outputs += 1;
        assert!(!self.async_only, "synchronous outgoing emission");
    }
    async fn output(&mut self, output: Output) {
        self.trace.borrow_mut().push(Call::Start(output.clone()));
        let mut polls = 0;
        poll_fn(|_| {
            polls += 1;
            if polls <= 2 {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        if let Output::Runtime(Effect::BridgeDelivery(enabled)) = output {
            self.bridge_enabled = enabled;
        }
        self.trace.borrow_mut().push(Call::Done(output));
    }
    fn completed(&self) -> Vec<Output> {
        self.trace
            .borrow()
            .iter()
            .filter_map(|call| match call {
                Call::Done(output) => Some(output.clone()),
                _ => None,
            })
            .collect()
    }
}
impl Platform for Io {
    fn prepare_ingress_boundary(&mut self, gate: crate::protocol_audio::IngressAdmissionGate) {
        self.boundaries.borrow_mut().push(gate);
    }
    fn tx_window_valid(&self) -> bool {
        self.window_valid.get()
    }
    fn ingress_gate_changed(
        &mut self,
        gate: crate::protocol_audio::IngressAdmissionGate,
        purge: bool,
    ) {
        self.gates.borrow_mut().push((gate, purge));
    }
    fn integration(&self) -> Integration {
        self.integration
    }
    fn now_us(&self) -> i64 {
        self.now_us
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        if matches!(
            effect,
            Effect::Status(_)
                | Effect::MeshStopped
                | Effect::SyncLost
                | Effect::DiscardBridgeAudio
                | Effect::BridgeDelivery(_)
                | Effect::Ack { .. }
        ) {
            self.sync_output();
        }
        self.trace.borrow_mut().push(Call::Sync(effect));
        match effect {
            Effect::GetAddress => Reply::Address([1, 2, 3, 4, 5]),
            Effect::LocalAudio(enabled) => {
                self.epoch = self.epoch.wrapping_add(1);
                Reply::Audio(AudioBoundary {
                    enabled,
                    epoch: self.epoch,
                    receipt_cutoff_us: self.now_us,
                })
            }
            Effect::CoordinatorTimeout => Reply::Membership(if self.lost {
                MembershipTransition::CoordinatorLost
            } else {
                MembershipTransition::Unchanged
            }),
            _ => Reply::Value(0),
        }
    }
    async fn apply_async(&mut self, effect: Effect) -> Reply {
        self.output(Output::Runtime(effect)).await;
        Reply::Value(self.result)
    }
}
impl MembershipPlatform for Io {
    fn membership_effect(&mut self, effect: MembershipEffect, _: &mut ProtocolState) -> bool {
        if matches!(
            effect,
            MembershipEffect::Status(_)
                | MembershipEffect::Event { .. }
                | MembershipEffect::Log { .. }
        ) {
            self.sync_output();
        }
        self.trace.borrow_mut().push(Call::Member(effect));
        true
    }
    async fn membership_output_async(&mut self, output: MembershipOutput) -> bool {
        self.output(Output::Membership(output)).await;
        self.result == 0
    }
}
impl AdaptivePlatform for Io {
    fn tdma_rx_timestamp(&mut self, physical_stamp: i64) -> Option<i64> {
        self.physical_stamps.push(physical_stamp);
        self.tdma_timestamp.unwrap_or(Some(physical_stamp))
    }
    fn clock_snapshot(&mut self) -> Option<crate::protocol_tx::ClockSnapshot> {
        self.clock
    }
    fn adaptive_effect(&mut self, effect: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        if matches!(effect, AdaptiveEffect::Status(_)) {
            self.sync_output();
        }
        self.trace.borrow_mut().push(Call::Adaptive(effect));
        match effect {
            AdaptiveEffect::FrameCounter => AdaptiveReply::Frame(10),
            AdaptiveEffect::JoinRequest => AdaptiveReply::Accepted(0),
            _ => AdaptiveReply::Applied,
        }
    }
    async fn adaptive_status_async(&mut self, status: Membership) -> AdaptiveReply {
        self.output(Output::Adaptive(status)).await;
        AdaptiveReply::Accepted(self.result)
    }
    fn radio_send(&mut self, _: RadioSend<'_>) -> i32 {
        self.trace.borrow_mut().push(Call::Radio);
        self.result
    }
    fn audio_effect(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_io(effect)
    }
    async fn audio_effect_async(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_io_async(effect).await
    }
}
impl AudioPlatform for Io {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        match effect {
            AudioEffect::NowUs => {
                self.trace.borrow_mut().push(Call::AudioNow);
                self.now_us
            }
            _ => 0,
        }
    }
    async fn audio_io_async(&mut self, effect: AudioEffect<'_>) -> i64 {
        let AudioEffect::BridgeDeliver { source, .. } = effect else {
            panic!("only BridgeDeliver may await");
        };
        self.output(Output::Audio(source)).await;
        i64::from(self.result)
    }
}
impl RxPlatform for Io {
    fn rx_io(&mut self, _: RxEffect) -> i32 {
        0
    }
}
fn poll<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}
fn finish<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    for _ in 0..128 {
        if let Poll::Ready(result) = poll(future.as_mut()) {
            return result;
        }
    }
    panic!("bounded outputs did not finish");
}
fn outputs_sync(trace: &[Call]) -> bool {
    trace
        .iter()
        .all(|call| !matches!(call, Call::Start(_) | Call::Done(_)))
}

#[test]
fn all_five_wrappers_forward_owned_output_hooks_with_pending_and_errors() {
    macro_rules! check {
        ($wrapper:ident) => {
            for result in [0, -5] {
                let mut io = Io {
                    result,
                    ..Io::async_only()
                };
                let m = Membership::default();
                let mut p = $wrapper(&mut io);
                assert_eq!(
                    finish(p.apply_async(Effect::MeshStopped)),
                    Reply::Value(result)
                );
                assert_eq!(
                    finish(
                        p.membership_output_async(MembershipOutput::Event { id: 5, node: None })
                    ),
                    result == 0
                );
                assert_eq!(
                    finish(p.adaptive_status_async(m)),
                    AdaptiveReply::Accepted(result)
                );
                assert_eq!(
                    io.completed(),
                    vec![
                        Output::Runtime(Effect::MeshStopped),
                        Output::Membership(MembershipOutput::Event { id: 5, node: None }),
                        Output::Adaptive(m),
                    ]
                );
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
fn stop_status_stopped_discard_ack_status_order_ignores_output_errors() {
    for result in [0, -5] {
        let mut r = Runtime::default();
        r.membership.state = State::Active;
        r.membership.node_id = 1;
        r.membership.role = Role::Coordinator;
        let mut io = Io {
            result,
            ..Io::async_only()
        };
        r.request_stop(7, &mut io);
        finish(r.command_work_handler_async(&mut FullProtocolPlatform(&mut io)));
        let completed = io.completed();
        assert!(matches!(&completed[..], [
            Output::Runtime(Effect::Status(_)), Output::Runtime(Effect::MeshStopped),
            Output::Runtime(Effect::DiscardBridgeAudio), Output::Runtime(Effect::Ack { command, result: 0 }),
            Output::Runtime(Effect::Status(_)),
        ] if command.generation == 7));
        assert_eq!(r.membership.state, State::Idle);
        assert!(io.trace.borrow().contains(&Call::Radio));
    }
}

#[test]
fn first_resume_flushes_before_local_mutation_then_gate_ack_status() {
    let mut r = Runtime::default();
    let mut io = Io::async_only();
    r.request_audio_resume(1, &mut io);
    let trace = io.trace.clone();
    {
        let mut future = pin!(r.command_work_handler_async(&mut io));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
        assert!(!trace
            .borrow()
            .contains(&Call::Sync(Effect::LocalAudio(true))));
        finish(future.as_mut());
    }
    assert_eq!(r.snapshot().audio.epoch, 1);
    let completed = io.completed();
    assert!(matches!(
        &completed[..],
        [
            Output::Runtime(Effect::BridgeDelivery(false)),
            Output::Runtime(Effect::BridgeDelivery(true)),
            Output::Runtime(Effect::Ack { result: 0, .. }),
            Output::Runtime(Effect::Status(_))
        ]
    ));
    io.trace.borrow_mut().clear();
    // An owned repeated Resume is acknowledged without any bridge gate or
    // local boundary traffic: nothing queued is flushed and the epoch is kept.
    r.request_audio_resume(2, &mut io);
    finish(r.command_work_handler_async(&mut FullProtocolPlatform(&mut io)));
    assert_eq!(r.snapshot().audio.epoch, 1);
    assert!(!trace
        .borrow()
        .iter()
        .any(|c| matches!(c, Call::Sync(Effect::LocalAudio(_)))));
    let completed = io.completed();
    assert!(matches!(
        &completed[..],
        [
            Output::Runtime(Effect::Ack { result: 0, .. }),
            Output::Runtime(Effect::Status(_))
        ]
    ));
}

#[test]
fn cancelled_command_consumes_request_without_ack_or_replay() {
    let mut r = Runtime::default();
    let mut io = Io::async_only();
    r.request_stop(9, &mut io);
    {
        let mut future = pin!(r.command_work_handler_async(&mut io));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.pending_request(), 0);
    assert!(io.completed().is_empty());
    finish(r.command_work_handler_async(&mut io));
    assert_eq!(
        io.trace
            .borrow()
            .iter()
            .filter(|call| matches!(call, Call::Start(_)))
            .count(),
        1
    );
}

fn presence_fixture() -> (Runtime, Vec<u8>) {
    let mut r = Runtime::default();
    r.membership = Membership {
        state: State::Active,
        role: Role::Coordinator,
        node_id: 1,
        coordinator_id: 1,
        slot_index: 0,
        ..Membership::default()
    };
    r.address = [1, 2, 3, 4, 5];
    r.audio_state.boundary.enabled = true;
    r.protocol.term = 1;
    r.protocol.leader_identity = r.local_identity();
    a::init(Some(&mut r.protocol.adaptive), 1, 1, 1, 1, 100);
    let local = r.local_identity();
    assert!(a::set_member(
        Some(&mut r.protocol.adaptive),
        1,
        Some(&local)
    ));
    let peer = a::Identity {
        address_len: 5,
        address: [3, 3, 3, 3, 3, 0],
    };
    assert!(a::set_member(
        Some(&mut r.protocol.adaptive),
        3,
        Some(&peer)
    ));
    r.session.peers[0].active = true;
    r.session.peers[0].node_id = 3;
    r.session.peers[0].slot_index = 1;
    r.session.peers[0].address = [3; 5];
    let mut packet = [0; 209];
    let len = b::audio_bundle_encode(
        &b::BundleView {
            current_data: &[42; 48],
            previous1_data: &[],
            previous2_data: &[],
            current_seq: 7,
            stream_id: 1,
            flags: b::CURRENT_ACTIVE,
            codec: 2,
        },
        &mut packet[9..],
    );
    packet[..9].copy_from_slice(&[w::VERSION, 13, 3, 9, 2, 1, r.talk_channel, len as u8, 0]);
    (r, packet[..9 + len].to_vec())
}

#[test]
fn first_presence_status_and_event_complete_before_audio_delivery() {
    for result in [0, -5] {
        let (mut r, packet) = presence_fixture();
        let mut io = Io {
            result,
            ..Io::async_only()
        };
        assert_eq!(
            finish(r.rx_process_packet_async(&packet, -40, 100_000, &mut FullRxPlatform(&mut io))),
            Ok(())
        );
        assert!(
            matches!(&io.completed()[..], [Output::Membership(MembershipOutput::Status(m)),
            Output::Membership(MembershipOutput::Event { id: 2, node: Some(3) }), Output::Audio(3)] if m.peer_count == 1)
        );
        assert_eq!(r.audio_state.counters().spi_out_ok, u32::from(result == 0));
        assert_eq!(
            r.audio_state.counters().spi_out_drop,
            u32::from(result != 0)
        );
    }
}

#[test]
fn cancellation_at_presence_retains_announcement_without_delivered_audio() {
    let (mut r, packet) = presence_fixture();
    let mut io = Io::async_only();
    {
        let mut p = FullAudioPlatform(&mut io);
        let mut future = pin!(r.audio_process_rx_packet_async(&packet, -40, 100_000, &mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert!(r.session.peers[0].announced);
    assert_eq!(r.membership.peer_count, 1);
    assert_eq!(r.audio_state.counters().rf_rx_audio_ok, 0);
    assert!(io.completed().is_empty());
    finish(r.audio_process_rx_packet_async(&packet, -40, 100_000, &mut FullAudioPlatform(&mut io)))
        .unwrap();
    assert!(io.completed().is_empty());
    assert_eq!(r.audio_state.counters().rf_rx_duplicate_drop, 1);
}

#[test]
fn join_retry_log_finishes_before_source_timer_and_uses_owned_bytes() {
    let mut r = Runtime::default();
    r.membership.state = State::Joining;
    let mut io = Io::async_only();
    let trace = io.trace.clone();
    {
        let mut future = pin!(r.membership_handle_async(Input::JoinWork, &mut io));
        assert!(poll(future.as_mut()).is_pending());
        assert!(trace
            .borrow()
            .contains(&Call::Member(MembershipEffect::LogInitialized)));
        assert!(!trace
            .borrow()
            .iter()
            .any(|call| matches!(call, Call::Sync(Effect::Schedule(_)))));
        finish(future.as_mut()).unwrap();
    }
    assert_eq!(r.protocol.join_attempts, 1);
    assert_eq!(r.timer(Timer::Join).unwrap().deadline_us, 600_000);
    let completed = io.completed();
    let Output::Membership(MembershipOutput::Log { bytes, len }) = &completed[0] else {
        panic!("missing log");
    };
    assert_eq!(&bytes[..usize::from(*len)], b"MESH: JOIN attempt 1/10");
    assert_eq!(bytes[usize::from(*len)], 0);
}

#[test]
fn join_retry_cancellation_consumes_timer_and_keeps_attempt_without_reschedule() {
    let mut r = Runtime::default();
    r.membership.state = State::Joining;
    let mut io = Io::async_only();
    r.schedule(Timer::Join, 0, &mut io);
    let token = r.timer(Timer::Join).unwrap();
    {
        let mut future = pin!(r.membership_handle_async(Input::Timer(token), &mut io));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.protocol.join_attempts, 1);
    assert_eq!(r.timer(Timer::Join), None);
    assert!(io.completed().is_empty());
    finish(r.membership_handle_async(Input::Timer(token), &mut io)).unwrap();
    assert_eq!(r.protocol.join_attempts, 1);
}

#[test]
fn owned_status_timer_awaits_peer_event_before_reports_and_next_timer() {
    let (mut r, _) = presence_fixture();
    r.session.peers[0].announced = true;
    r.membership.peer_count = 1;
    let mut io = Io {
        now_us: 4_000_000,
        ..Io::async_only()
    };
    r.schedule(Timer::Status, 0, &mut io);
    let token = r.timer(Timer::Status).unwrap();
    let trace = io.trace.clone();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(!trace.borrow().contains(&Call::Sync(Effect::RadioStats)));
        assert_eq!(finish(future.as_mut()), 0);
    }
    assert_eq!(
        io.completed(),
        vec![Output::Membership(MembershipOutput::Event {
            id: 3,
            node: Some(3)
        })]
    );
    assert_eq!(r.timer(Timer::Status).unwrap().deadline_us, 5_000_000);
}

#[test]
fn legacy_loss_awaits_sync_lost_then_status_before_scan_schedule() {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    let mut io = Io {
        lost: true,
        ..Io::async_only()
    };
    finish(r.status_work_handler_async(&mut io));
    assert!(
        matches!(&io.completed()[..], [Output::Runtime(Effect::SyncLost), Output::Runtime(Effect::Status(m))] if m.state == State::Scanning)
    );
    assert_eq!(r.timer(Timer::Status), None);
    assert!(r.timer(Timer::Scan).is_some());
}

#[test]
fn sync_false_bypasses_all_owned_output_overrides() {
    macro_rules! check {
        ($wrapper:ident) => {
            let mut r = Runtime::default();
            let mut io = Io::default();
            r.membership.state = State::Scanning;
            r.membership_handle(Input::ScanWork, &mut $wrapper(&mut io))
                .unwrap();
            r.request_audio_resume(1, &mut io);
            r.command_work_handler(&mut $wrapper(&mut io));
            assert!(outputs_sync(&io.trace.borrow()));
            assert!(io
                .trace
                .borrow()
                .iter()
                .any(|call| matches!(call, Call::Member(MembershipEffect::Status(_)))));
        };
    }
    check!(CombinedMembershipPlatform);
    check!(CombinedFullTxPlatform);
    check!(FullAudioPlatform);
    check!(FullRxPlatform);
    check!(FullProtocolPlatform);
}

#[test]
fn log_format_and_initialized_query_precede_owned_127_byte_await() {
    let mut r = Runtime::default();
    let mut io = Io::async_only();
    let text = "x".repeat(200);
    finish(r.mesh_log_impl::<true>(format_args!("{text}"), &mut io));
    let trace = io.trace.borrow();
    assert_eq!(trace[0], Call::Member(MembershipEffect::LogInitialized));
    let Call::Start(Output::Membership(MembershipOutput::Log { bytes, len })) = &trace[1] else {
        panic!("missing owned log");
    };
    assert_eq!(*len, 127);
    assert_eq!(&bytes[..127], &[b'x'; 127]);
    assert_eq!(bytes[127], 0);
}

#[test]
fn owned_coordinator_loss_awaits_log_event_status_in_source_order() {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    let mut io = Io {
        now_us: 6_000_000,
        ..Io::async_only()
    };
    finish(r.status_work_handler_async(&mut FullProtocolPlatform(&mut io)));
    assert!(matches!(&io.completed()[..], [
        Output::Membership(MembershipOutput::Log { .. }),
        Output::Membership(MembershipOutput::Event { id: 5, node: None }),
        Output::Membership(MembershipOutput::Status(m)),
    ] if m.state == State::Scanning));
    assert_eq!(r.timer(Timer::Status), None);
    assert!(r.timer(Timer::Scan).is_some());
}

fn handover_fixture(local: u8) -> Runtime {
    handover_fixture_at(local, 100)
}

fn handover_fixture_at(local: u8, now_ms: u32) -> Runtime {
    let mut r = Runtime::default();
    r.membership = Membership {
        state: State::Active,
        role: if local == 1 {
            Role::Coordinator
        } else {
            Role::Participant
        },
        node_id: local,
        coordinator_id: 1,
        slot_index: local as i8 - 1,
        ..Membership::default()
    };
    r.protocol.term = 1;
    r.address = [local; 5];
    // Leader election needs 10 s tenure and 3 s of a better candidate.
    let initialized_ms = if local == 1 { 0 } else { now_ms };
    a::init(
        Some(&mut r.protocol.adaptive),
        local,
        1,
        1,
        7,
        initialized_ms,
    );
    let map = openhelmet_mesh::core::SlotMap {
        slot_count: 3,
        slot_ids: [1, 2, 3, 0, 0, 0, 0, 0],
        active_speaker_count: 0,
        active_speaker_ids: [0; 2],
        relay_masks: [0; 2],
    };
    let report_ms = if local == 1 {
        assert!(now_ms >= 13_000);
        now_ms - 3000
    } else {
        now_ms
    };
    for id in 1..=3 {
        let identity = a::Identity {
            address_len: 5,
            address: [id, id, id, id, id, 0],
        };
        assert!(a::set_member(
            Some(&mut r.protocol.adaptive),
            id,
            Some(&identity)
        ));
        r.session.peers[usize::from(id - 1)].active = true;
        r.session.peers[usize::from(id - 1)].announced = true;
        r.session.peers[usize::from(id - 1)].node_id = id;
        r.session.peers[usize::from(id - 1)].slot_index = id as i8 - 1;
        r.session.peers[usize::from(id - 1)].address = [id; 5];
    }
    for id in 1..=3 {
        let identity = a::Identity {
            address_len: 5,
            address: [id, id, id, id, id, 0],
        };
        let topology = a::Topology {
            report_seq: 1,
            origin: identity,
            direct_mask: 7 & !(1 << (id - 1)),
            rssi_dbm: if local == 1 && id <= 2 {
                let mut rssi = [-40; 8];
                rssi[usize::from(2 - id)] = -80;
                rssi
            } else {
                [-40; 8]
            },
            quality: [100; 8],
            leader_id: 1,
            term: 1,
            slot_map: map,
        };
        assert!(a::report(
            Some(&mut r.protocol.adaptive),
            id,
            Some(&topology),
            false,
            report_ms
        ));
    }
    r.protocol.leader_identity = a::Identity {
        address_len: 5,
        address: [1, 1, 1, 1, 1, 0],
    };
    let mut proposal = a::Handover {
        old_leader_id: 1,
        candidate_id: 3,
        next_term: 2,
        switch_frame: 30,
        members: 7,
        slot_map: map,
    };
    if local == 1 {
        assert_eq!(a::candidate(Some(&mut r.protocol.adaptive), report_ms), 0);
        assert_eq!(r.protocol.adaptive.snapshot().better_id, 3);
        // Refresh reports at preparation time so graph edges have not expired.
        for id in 1..=3 {
            let node = r.protocol.adaptive.snapshot().nodes[usize::from(id - 1)];
            let topology = a::Topology {
                report_seq: 2,
                origin: node.identity,
                direct_mask: 7 & !(1 << (id - 1)),
                rssi_dbm: node.rssi,
                quality: [100; 8],
                leader_id: 1,
                term: 1,
                slot_map: map,
            };
            assert!(a::report(
                Some(&mut r.protocol.adaptive),
                id,
                Some(&topology),
                false,
                now_ms
            ));
        }
        assert!(a::prepare(
            Some(&mut r.protocol.adaptive),
            3,
            10,
            Some(&map),
            now_ms,
            Some(&mut proposal)
        ));
        for id in [2, 3] {
            assert!(a::ack(
                Some(&mut r.protocol.adaptive),
                id,
                Some(&a::HandoverAck {
                    old_leader_id: 1,
                    candidate_id: 3,
                    next_term: 2,
                    switch_frame: 170,
                    acknowledger_id: id,
                }),
                now_ms
            ));
        }
        assert!(a::commit(
            Some(&mut r.protocol.adaptive),
            Some(&mut proposal),
            now_ms
        ));
        return r;
    }
    assert!(a::receive_prepare(
        Some(&mut r.protocol.adaptive),
        1,
        Some(&proposal),
        10,
        now_ms
    ));
    assert!(a::receive_commit(
        Some(&mut r.protocol.adaptive),
        1,
        Some(&proposal),
        now_ms
    ));
    r
}

#[test]
fn async_tx_frame_tick_awaits_actual_handover_status_before_tx() {
    let mut r = handover_fixture(3);
    let mut io = Io::async_only();
    let trace = io.trace.clone();
    {
        let mut p = CombinedFullTxPlatform(&mut io);
        let mut future = pin!(r.tx_control_handler_async(31, &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(!trace.borrow().contains(&Call::Radio));
        finish(future.as_mut());
    }
    assert_eq!(r.membership.coordinator_id, 3);
    assert_eq!(r.membership.role, Role::Coordinator);
    assert!(matches!(&io.completed()[..], [Output::Adaptive(m)] if m.coordinator_id == 3));
}

#[test]
fn strict_control_revalidates_after_pending_handover_status_before_sync_tx() {
    for valid in [true, false] {
        let mut r = handover_fixture(3);
        let mut io = Io {
            clock: Some(crate::protocol_tx::ClockSnapshot {
                frame: 40,
                phase_us: 16_000,
            }),
            ..Io::async_only()
        };
        let window = io.window_valid.clone();
        let trace = io.trace.clone();
        {
            let mut p = FullProtocolPlatform(&mut io);
            let mut future = pin!(r.tx_control_handler_async(40, &mut p));
            assert!(poll(future.as_mut()).is_pending());
            assert!(!trace.borrow().contains(&Call::Radio));
            window.set(valid);
            finish(future.as_mut());
        }
        assert_eq!(trace.borrow().contains(&Call::Radio), valid);
        assert_eq!(r.membership.role, Role::Coordinator);
    }
}

#[test]
fn all_five_wrappers_forward_preflight_and_window_revalidation() {
    macro_rules! check {
        ($wrapper:ident) => {{
            let mut io = Io::default();
            let gate = crate::protocol_audio::IngressAdmissionGate {
                enabled: true,
                epoch: 42,
            };
            let mut p = $wrapper(&mut io);
            p.prepare_ingress_boundary(gate);
            assert!(p.tx_window_valid());
            io.window_valid.set(false);
            assert!(!$wrapper(&mut io).tx_window_valid());
            assert_eq!(&*io.boundaries.borrow(), &[gate]);
        }};
    }
    check!(CombinedMembershipPlatform);
    check!(CombinedFullTxPlatform);
    check!(FullAudioPlatform);
    check!(FullRxPlatform);
    check!(FullProtocolPlatform);
}

#[test]
fn all_five_wrappers_forward_tdma_rx_timestamp_without_masking_backend() {
    macro_rules! check {
        ($wrapper:ident) => {
            for converted in [Some(1_092_266), None] {
                let mut io = Io {
                    tdma_timestamp: Some(converted),
                    ..Io::default()
                };
                assert_eq!($wrapper(&mut io).tdma_rx_timestamp(1_000_000), converted);
                assert_eq!(io.physical_stamps, vec![1_000_000]);
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
fn strict_resume_preflight_precedes_initial_disable_without_changing_core_epoch() {
    let (mut r, _) = presence_fixture();
    let mut io = Io::async_only();
    r.audio_set_ingress_enabled(true, false);
    r.audio_state.boundary.epoch = 42;
    r.audio_state.boundary.enabled = true;
    // Bridge delivery still disabled (e.g. Pause while idle, then Start), so
    // this Resume is not redundant and must run the full preflight.
    r.bridge_delivery_enabled = false;
    r.request_audio_resume(1, &mut io);
    let boundary = io.boundaries.clone();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.command_work_handler_async(&mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert_eq!(
            &*boundary.borrow(),
            &[crate::protocol_audio::IngressAdmissionGate {
                enabled: true,
                epoch: 42,
            }]
        );
    }
    assert_eq!(r.ingress_admission_gate().epoch, 42);
    assert!(r.ingress_admission_gate().enabled);
}

#[test]
fn strict_loss_preflight_precedes_log_without_changing_core_epoch() {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    r.audio_set_ingress_enabled(true, false);
    r.audio_state.boundary.epoch = 42;
    r.audio_state.boundary.enabled = true;
    let mut io = Io {
        now_us: 6_000_000,
        ..Io::async_only()
    };
    let boundary = io.boundaries.clone();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.status_work_handler_async(&mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert_eq!(boundary.borrow().len(), 1);
        assert_eq!(boundary.borrow()[0].epoch, 42);
    }
    assert_eq!(r.ingress_admission_gate().epoch, 42);
    assert!(r.ingress_admission_gate().enabled);
    assert!(matches!(
        io.trace.borrow().last(),
        Some(Call::Start(Output::Membership(
            MembershipOutput::Log { .. }
        )))
    ));
}

#[test]
fn async_rx_sync_reconciliation_awaits_adaptive_status_before_clock_update() {
    let mut r = handover_fixture(2);
    let sync = w::SyncV3 {
        leader: w::Identity {
            address_len: 5,
            address: [3, 3, 3, 3, 3, 0],
        },
        leader_id: 3,
        member_count: 3,
        term: 2,
        frame_counter: 31,
        phase_us: 100,
        relay_depth: 0,
    };
    let mut payload = [0; w::SyncV3::SIZE];
    sync.encode(&mut payload).unwrap();
    let header = w::Header {
        version: w::VERSION,
        packet_type: 17,
        src_id: 3,
        seq: 1,
        ttl: 2,
        flags: 0,
        talk_channel: r.talk_channel,
        payload_len: payload.len() as u16,
    };
    let mut packet = [0; 209];
    let len = w::encode(&header, &payload, &mut packet, 200).unwrap();
    let mut io = Io::async_only();
    let trace = io.trace.clone();
    {
        let mut p = FullRxPlatform(&mut io);
        let mut future = pin!(r.rx_process_packet_async(&packet[..len], -40, 100_000, &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(!trace
            .borrow()
            .iter()
            .any(|call| matches!(call, Call::Adaptive(AdaptiveEffect::Sync { .. }))));
        finish(future.as_mut()).unwrap();
    }
    assert!(matches!(&io.completed()[..], [Output::Adaptive(m)] if m.coordinator_id == 3));
    assert_eq!(r.membership.coordinator_id, 3);
}

fn sync_packet(leader: u8, term: u32, frame: u32, channel: u8) -> Vec<u8> {
    let sync = w::SyncV3 {
        leader: w::Identity {
            address_len: 5,
            address: [leader, leader, leader, leader, leader, 0],
        },
        leader_id: leader,
        member_count: 3,
        term,
        frame_counter: frame,
        phase_us: 100,
        relay_depth: 0,
    };
    let mut payload = [0; w::SyncV3::SIZE];
    sync.encode(&mut payload).unwrap();
    let header = member_packet(17, leader, payload.len(), channel);
    let mut packet = [0; 209];
    let len = w::encode(&header, &payload, &mut packet, 200).unwrap();
    packet[..len].to_vec()
}

#[test]
fn strict_rx_work_converts_once_before_handover_status_and_keeps_physical_time() {
    for (local, converted) in [(2, 1_092_266), (1, 1_092_266), (2, 9_000_000_000)] {
        let physical_stamp = if local == 1 { 16_000_000 } else { 1_000_000 };
        let now_us = physical_stamp + 100_000;
        let mut r = handover_fixture_at(local, (now_us / 1000) as u32);
        let frame = if local == 1 { 171 } else { 31 };
        let packet = sync_packet(3, 2, frame, r.talk_channel);
        let mut io = Io {
            integration: Integration::HardwareUnqualified,
            now_us,
            tdma_timestamp: Some(Some(converted)),
            ..Io::async_only()
        };
        r.schedule(Timer::Status, 1_000_000, &mut io);
        let timers = r.timers;
        let trace = io.trace.clone();
        {
            let mut p = FullProtocolPlatform(&mut io);
            assert_eq!(r.rx_publish(&packet, -40, physical_stamp, &mut p), Ok(true));
            let mut future = pin!(r.rx_work_handler_async(&mut p));
            assert!(poll(future.as_mut()).is_pending());
            assert!(!trace
                .borrow()
                .iter()
                .any(|call| matches!(call, Call::Adaptive(AdaptiveEffect::Sync { .. }))));
            finish(future.as_mut()).unwrap();
        }
        assert_eq!(io.physical_stamps, vec![physical_stamp]);
        assert!(trace
            .borrow()
            .contains(&Call::Adaptive(AdaptiveEffect::Sync {
                frame,
                drift: 0,
                base_us: converted - 100 - 300,
            })));
        assert_eq!(r.membership.coordinator_id, 3);
        assert_eq!(r.membership.role, Role::Participant);
        assert_eq!(r.protocol.last_sync_time, (physical_stamp / 1000) as u32);
        assert_eq!(r.session.peers[2].last_seen_ms, now_us / 1000);
        assert_eq!(r.protocol.last_forward_sync, (now_us / 1000) as u32);
        assert_eq!(r.timers, timers);
        assert_eq!(io.now_us(), now_us);
        let forwarded = r.protocol.adaptive.forward_queue_entry(0).unwrap();
        assert_eq!(forwarded.enqueued_ms, (now_us / 1000) as u32);
        assert_eq!(
            &forwarded.payload[..usize::from(forwarded.payload_len)],
            &packet[9..]
        );
    }
}

#[test]
fn strict_rx_none_consumes_before_handover_and_preserves_owned_state_and_queues() {
    let mut r = handover_fixture(2);
    r.protocol.last_sync_time = 77;
    r.protocol.last_forward_sync = 88;
    r.protocol.last_sync_log = 99;
    r.session.peers[2].announced = false;
    r.session.peers[2].last_seen_ms = 42;
    r.session.control_head = 1;
    r.session.control_ring[0].len = 1;
    r.session.control_ring[0].data[0] = 42;
    r.protocol.priority_pending = true;
    r.protocol.priority_control = r.session.control_ring[0];
    let old_packet = sync_packet(1, 1, 10, r.talk_channel);
    let origin = r.protocol.leader_identity;
    assert!(a::forward_enqueue(
        Some(&mut r.protocol.adaptive),
        Some(&a::Header {
            version: w::VERSION,
            packet_type: 17,
            src_id: 1,
            seq: 9,
            ttl: 2,
            flags: 0,
            talk_channel: r.talk_channel,
            payload_len: (old_packet.len() - 9) as u16,
        }),
        Some(&old_packet[9..]),
        Some(&origin),
        1,
        true,
        100,
    ));
    let membership = r.membership;
    let adaptive = r.protocol.adaptive.clone();
    let peers = r.session.peers;
    let control_ring = r.session.control_ring;
    let presence = r.protocol.control_presence_dedupe.clone();
    let packet = sync_packet(3, 2, 31, r.talk_channel);
    let mut io = Io {
        integration: Integration::HardwareUnqualified,
        now_us: 1_000_000,
        tdma_timestamp: Some(None),
        ..Io::async_only()
    };
    {
        let mut p = FullProtocolPlatform(&mut io);
        assert_eq!(r.rx_publish(&packet, -40, 1_000_000, &mut p), Ok(true));
        let mut future = pin!(r.rx_work_handler_async(&mut p));
        assert_eq!(poll(future.as_mut()), Poll::Ready(Ok(())));
    }
    assert_eq!(io.physical_stamps, vec![1_000_000]);
    assert!(io.trace.borrow().is_empty());
    assert_eq!(r.membership, membership);
    assert_eq!(r.protocol.term, 1);
    assert_eq!(r.protocol.leader_identity, origin);
    assert_eq!(r.protocol.adaptive, adaptive);
    assert_eq!(r.session.peers, peers);
    assert_eq!(r.session.control_ring, control_ring);
    assert_eq!((r.session.control_head, r.session.control_tail), (1, 0));
    assert!(r.protocol.priority_pending);
    assert_eq!(r.protocol.priority_control, control_ring[0]);
    assert_eq!(r.protocol.control_presence_dedupe, presence);
    assert_eq!(r.protocol.last_sync_time, 77);
    assert_eq!(r.protocol.last_forward_sync, 88);
    assert_eq!(r.protocol.last_sync_log, 99);
    assert_eq!(r.rx_state.count, 1);
    assert_eq!(r.rx_state.tail, r.rx_state.head);
}

#[test]
fn strict_rx_none_prevents_coordinator_cold_join_demotion() {
    for converted in [None, Some(1_092_266)] {
        let mut r = Runtime::default();
        r.membership = Membership {
            state: State::Active,
            role: Role::Coordinator,
            node_id: 2,
            coordinator_id: 2,
            slot_index: 0,
            ..Membership::default()
        };
        r.address = [2; 5];
        r.protocol.term = 1;
        r.protocol.leader_identity = r.local_identity();
        a::init(Some(&mut r.protocol.adaptive), 2, 2, 1, 2, 100);
        let local = r.local_identity();
        assert!(a::set_member(
            Some(&mut r.protocol.adaptive),
            2,
            Some(&local)
        ));
        r.protocol.last_sync_time = 77;
        let before = r.membership;
        let adaptive = r.protocol.adaptive.clone();
        let packet = sync_packet(1, 1, 31, r.talk_channel);
        let mut io = Io {
            integration: Integration::HardwareUnqualified,
            now_us: 1_000_000,
            tdma_timestamp: Some(converted),
            ..Io::async_only()
        };
        {
            let mut p = FullProtocolPlatform(&mut io);
            let mut future = pin!(r.rx_process_packet_async(&packet, -40, 1_000_000, &mut p));
            if converted.is_none() {
                assert_eq!(poll(future.as_mut()), Poll::Ready(Ok(())));
            } else {
                finish(future.as_mut()).unwrap();
            }
        }
        assert_eq!(io.physical_stamps, vec![1_000_000]);
        if converted.is_none() {
            assert_eq!(r.membership, before);
            assert_eq!(r.protocol.adaptive, adaptive);
            assert_eq!(r.protocol.last_sync_time, 77);
            assert!(io.trace.borrow().is_empty());
        } else {
            assert_eq!(r.membership.state, State::Joining);
            assert_eq!(r.membership.role, Role::None);
            assert_eq!(r.membership.coordinator_id, 1);
        }
    }
}

#[test]
fn strict_rx_none_prevents_member_owned_initialization() {
    let mut r = handover_fixture(2);
    r.protocol.adaptive = a::Adaptive::default();
    r.protocol.last_sync_time = 77;
    let adaptive = r.protocol.adaptive.clone();
    let membership = r.membership;
    let peers = r.session.peers;
    let packet = sync_packet(1, 1, 10, r.talk_channel);
    let mut io = Io {
        integration: Integration::HardwareUnqualified,
        now_us: 1_000_000,
        tdma_timestamp: Some(None),
        ..Io::async_only()
    };
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.rx_process_packet_async(&packet, -40, 1_000_000, &mut p));
        assert_eq!(poll(future.as_mut()), Poll::Ready(Ok(())));
    }
    assert_eq!(io.physical_stamps, vec![1_000_000]);
    assert_eq!(r.protocol.adaptive, adaptive);
    assert_eq!(r.membership, membership);
    assert_eq!(r.session.peers, peers);
    assert_eq!(r.protocol.last_sync_time, 77);
    assert!(io.trace.borrow().is_empty());
}

#[test]
fn strict_rx_stale_physical_sync_is_rejected_before_conversion() {
    let mut r = handover_fixture(2);
    let before = r.protocol.adaptive.clone();
    let packet = sync_packet(3, 2, 31, r.talk_channel);
    let mut io = Io {
        integration: Integration::HardwareUnqualified,
        now_us: 2_000_000,
        tdma_timestamp: Some(Some(2_000_000)),
        ..Io::async_only()
    };
    finish(r.rx_process_packet_async(&packet, -40, 1_000_000, &mut FullProtocolPlatform(&mut io)))
        .unwrap();
    assert!(io.physical_stamps.is_empty());
    assert_eq!(r.protocol.adaptive, before);
    assert!(io.trace.borrow().is_empty());
}

#[test]
fn strict_rx_converted_arithmetic_error_remains_after_reconciliation_status() {
    let mut r = handover_fixture(2);
    let packet = sync_packet(3, 2, 31, r.talk_channel);
    let mut io = Io {
        integration: Integration::HardwareUnqualified,
        now_us: 1_000_000,
        tdma_timestamp: Some(Some(i64::MIN)),
        ..Io::async_only()
    };
    assert_eq!(
        finish(r.rx_process_packet_async(
            &packet,
            -40,
            1_000_000,
            &mut FullProtocolPlatform(&mut io)
        )),
        Err(crate::protocol_membership::Error::UndefinedArithmetic)
    );
    assert_eq!(io.physical_stamps, vec![1_000_000]);
    assert_eq!(r.membership.coordinator_id, 3);
    assert!(matches!(&io.completed()[..], [Output::Adaptive(m)] if m.coordinator_id == 3));
    assert!(!io
        .trace
        .borrow()
        .iter()
        .any(|call| matches!(call, Call::Adaptive(AdaptiveEffect::Sync { .. }))));
}

#[test]
fn nonactive_sync_discovery_and_join_paths_do_not_convert() {
    for state in [State::Scanning, State::Joining] {
        for leader in [1, 3] {
            let mut r = handover_fixture(2);
            r.membership.state = state;
            let packet = sync_packet(leader, 1, 10, r.talk_channel);
            let mut io = Io {
                tdma_timestamp: Some(None),
                ..Io::async_only()
            };
            finish(r.rx_process_packet_async(&packet, -40, 100_000, &mut FullRxPlatform(&mut io)))
                .unwrap();
            assert!(io.physical_stamps.is_empty());
        }
    }
}

#[test]
fn synchronous_handover_bypasses_adaptive_status_override() {
    let mut r = handover_fixture(3);
    let mut io = Io::default();
    r.adaptive_frame_tick(31, &mut CombinedFullTxPlatform(&mut io));
    assert!(outputs_sync(&io.trace.borrow()));
    assert!(io.trace.borrow().iter().any(
        |call| matches!(call, Call::Adaptive(AdaptiveEffect::Status(m)) if m.coordinator_id == 3)
    ));
}

fn member_packet(t: u8, src: u8, len: usize, channel: u8) -> w::Header {
    w::Header {
        version: w::VERSION,
        packet_type: t,
        src_id: src,
        seq: 1,
        ttl: 2,
        flags: 0,
        talk_channel: channel,
        payload_len: len as u16,
    }
}

#[test]
fn async_join_and_leave_await_status_event_before_following_queue_effects() {
    let mut r = Runtime::default();
    r.membership = Membership {
        state: State::Active,
        role: Role::Coordinator,
        node_id: 1,
        coordinator_id: 1,
        slot_index: 0,
        ..Membership::default()
    };
    r.address = [1; 5];
    r.protocol.leader_identity = a::Identity {
        address_len: 5,
        address: [1, 1, 1, 1, 1, 0],
    };
    let join = w::JoinV3 {
        origin: w::Identity {
            address_len: 5,
            address: [2, 2, 2, 2, 2, 0],
        },
        target: w::Identity {
            address_len: 5,
            address: [1, 1, 1, 1, 1, 0],
        },
        capabilities: 2,
    };
    let mut payload = [0; w::JoinV3::SIZE];
    join.encode(&mut payload).unwrap();
    let mut io = Io::async_only();
    finish(r.membership_handle_async(
        Input::Packet {
            header: member_packet(15, 0, payload.len(), r.talk_channel),
            payload: &payload,
        },
        &mut io,
    ))
    .unwrap();
    assert!(matches!(
        &io.completed()[..],
        [
            Output::Membership(MembershipOutput::Status(_)),
            Output::Membership(MembershipOutput::Event {
                id: 2,
                node: Some(2)
            })
        ]
    ));
    {
        let trace = io.trace.borrow();
        let event = trace
            .iter()
            .position(|call| {
                matches!(
                    call,
                    Call::Done(Output::Membership(MembershipOutput::Event { id: 2, .. }))
                )
            })
            .unwrap();
        let ack = trace
            .iter()
            .position(|call| matches!(call, Call::Member(MembershipEffect::JoinAck { .. })))
            .unwrap();
        assert!(event < ack);
    }
    io.trace.borrow_mut().clear();
    let leave = w::LeaveNrf {
        sender_addr: [2; 5],
    };
    let mut payload = [0; w::LeaveNrf::SIZE];
    leave.encode(&mut payload).unwrap();
    finish(r.membership_handle_async(
        Input::Packet {
            header: member_packet(4, 2, payload.len(), r.talk_channel),
            payload: &payload,
        },
        &mut io,
    ))
    .unwrap();
    assert!(matches!(
        &io.completed()[..],
        [
            Output::Membership(MembershipOutput::Status(_)),
            Output::Membership(MembershipOutput::Event {
                id: 3,
                node: Some(2)
            })
        ]
    ));
}

#[test]
fn async_join_ack_and_rejoin_preserve_status_event_source_order() {
    let mut r = Runtime::default();
    r.membership.state = State::Joining;
    r.membership.coordinator_id = 1;
    r.protocol.term = 7;
    r.address = [2; 5];
    r.protocol.leader_identity = a::Identity {
        address_len: 5,
        address: [1, 1, 1, 1, 1, 0],
    };
    let ack = w::JoinAckV3 {
        capabilities: 2,
        assigned_id: 2,
        slot_index: 1,
        coordinator_id: 1,
        term: 7,
        target: w::Identity {
            address_len: 5,
            address: [2, 2, 2, 2, 2, 0],
        },
    };
    let mut payload = [0; w::JoinAckV3::SIZE];
    ack.encode(&mut payload).unwrap();
    let mut io = Io::async_only();
    finish(r.membership_handle_async(
        Input::Packet {
            header: member_packet(16, 1, payload.len(), r.talk_channel),
            payload: &payload,
        },
        &mut io,
    ))
    .unwrap();
    assert!(
        matches!(&io.completed()[..], [Output::Membership(MembershipOutput::Status(m)),
        Output::Membership(MembershipOutput::Event { id: 1, node: None })] if m.peer_count == 255)
    );
    assert_eq!(r.membership.state, State::Active);
    io.trace.borrow_mut().clear();
    finish(r.membership_handle_async(
        Input::Rejoin {
            leader: 3,
            term: 8,
            identity: a::Identity {
                address_len: 5,
                address: [3, 3, 3, 3, 3, 0],
            },
        },
        &mut io,
    ))
    .unwrap();
    assert!(
        matches!(&io.completed()[..], [Output::Membership(MembershipOutput::Event { id: 5, node: None }),
        Output::Membership(MembershipOutput::Status(m))] if m.state == State::Joining && m.peer_count == 255)
    );
    assert_eq!(r.timer(Timer::Join).unwrap().deadline_us, io.now_us);
}

#[test]
fn default_owned_output_hooks_forward_legacy_sync_callbacks() {
    let stored = Membership {
        state: State::Active,
        role: Role::Participant,
        peer_count: 3,
        membership_known: false,
        node_id: 7,
        slot_index: 4,
        coordinator_id: 2,
    };
    let status = Membership {
        peer_count: stored.bridge_peer_count(),
        ..stored
    };
    assert_eq!(stored.peer_count, 3);
    assert_eq!(status.peer_count, 255);
    struct Legacy(Io, Membership);
    impl Platform for Legacy {
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
    impl MembershipPlatform for Legacy {
        fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
            assert_eq!(e, MembershipEffect::Status(self.1));
            self.0.membership_effect(e, s)
        }
    }
    impl AdaptivePlatform for Legacy {
        fn adaptive_effect(&mut self, e: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply {
            assert_eq!(io.membership, self.1);
            assert_eq!(e, AdaptiveEffect::Status(self.1));
            self.0.adaptive_effect(e, io)
        }
    }
    let mut p = Legacy(Io::default(), status);
    assert_eq!(
        crate::corefuture::ready(p.apply_async(Effect::MeshStopped)),
        Reply::Value(0)
    );
    assert!(crate::corefuture::ready(
        p.membership_output_async(MembershipOutput::Status(status))
    ));
    assert_eq!(
        crate::corefuture::ready(p.adaptive_status_async(status)),
        AdaptiveReply::Applied
    );
    assert_eq!(
        &p.0.trace.borrow()[..],
        &[
            Call::Sync(Effect::MeshStopped),
            Call::Member(MembershipEffect::Status(status)),
            Call::Adaptive(AdaptiveEffect::Status(status)),
        ]
    );
    p.0.trace.borrow_mut().clear();
    let mut r = Runtime::default();
    r.membership = stored;
    r.request_status(&mut p);
    crate::corefuture::ready(r.command_work_handler_async(&mut p));
    assert_eq!(r.membership, stored);
    assert_eq!(
        &p.0.trace.borrow()[..],
        &[
            Call::Sync(Effect::SubmitCommand),
            Call::Sync(Effect::PeerCount(stored)),
            Call::Sync(Effect::Status(status)),
        ]
    );
}

#[test]
fn strict_fake_rejects_and_counts_every_synchronous_outgoing_family() {
    use std::panic::{catch_unwind, AssertUnwindSafe};
    let mut io = Io::async_only();
    for effect in [
        Effect::Status(Membership::default()),
        Effect::MeshStopped,
        Effect::SyncLost,
        Effect::Ack {
            command: crate::runtime::Command {
                kind: crate::runtime::CommandKind::Pause,
                generation: 7,
                talk_channel: 0,
            },
            result: 0,
        },
        Effect::BridgeDelivery(false),
        Effect::DiscardBridgeAudio,
    ] {
        assert!(catch_unwind(AssertUnwindSafe(|| io.apply(effect))).is_err());
    }
    for effect in [
        MembershipEffect::Status(Membership::default()),
        MembershipEffect::Event { id: 1, node: None },
        MembershipEffect::Log {
            bytes: [0; 128],
            len: 1,
        },
    ] {
        assert!(catch_unwind(AssertUnwindSafe(|| {
            io.membership_effect(effect, &mut ProtocolState::default())
        }))
        .is_err());
    }
    assert!(catch_unwind(AssertUnwindSafe(|| {
        Runtime::legacy_adaptive_status(Membership::default(), &mut io)
    }))
    .is_err());
    assert_eq!(io.sync_outputs, 10);
    assert_eq!(
        io.apply(Effect::GetAddress),
        Reply::Address([1, 2, 3, 4, 5])
    );
    assert!(io.membership_effect(
        MembershipEffect::LogInitialized,
        &mut ProtocolState::default()
    ));
    assert_eq!(io.sync_outputs, 10);
}

#[test]
fn owned_pause_and_repeated_resume_order_local_epoch_between_bridge_gates() {
    let (mut r, _) = presence_fixture();
    r.audio_set_ingress_enabled(true, false);
    r.audio_state.boundary.epoch = 41;
    let mut io = Io::async_only();
    // The repeated resume (generation 3) is acknowledged without gate traffic.
    for (generation, enabled, epoch) in [(1, false, 42), (2, true, 43), (3, true, 43)] {
        let redundant = generation == 3;
        if enabled {
            r.request_audio_resume(generation, &mut io);
        } else {
            r.request_audio_pause(generation, &mut io);
        }
        finish(r.command_work_handler_async(&mut FullProtocolPlatform(&mut io)));
        assert_eq!(
            r.snapshot().audio,
            AudioBoundary {
                enabled,
                epoch,
                receipt_cutoff_us: io.now_us,
            }
        );
        assert_eq!(r.ingress_admission_gate().enabled, enabled);
        assert_eq!(r.ingress_admission_gate().epoch, epoch);
        assert_eq!(io.bridge_enabled, enabled);
        let trace = io.trace.borrow();
        let gate = trace
            .iter()
            .position(|c| *c == Call::Start(Output::Runtime(Effect::BridgeDelivery(enabled))));
        let local = trace.iter().position(|c| *c == Call::AudioNow);
        if redundant {
            assert_eq!((local, gate), (None, None));
        } else {
            assert!(local.unwrap() < gate.unwrap());
        }
        if enabled && !redundant {
            let disable = trace
                .iter()
                .position(|c| *c == Call::Done(Output::Runtime(Effect::BridgeDelivery(false))))
                .unwrap();
            assert!(disable < gate.unwrap());
        }
        assert!(!trace
            .iter()
            .any(|c| matches!(c, Call::Sync(Effect::LocalAudio(_)))));
        let mut expected = vec![];
        if enabled && !redundant {
            expected.push(Output::Runtime(Effect::BridgeDelivery(false)));
        }
        if !redundant {
            expected.push(Output::Runtime(Effect::BridgeDelivery(enabled)));
        }
        expected.push(Output::Runtime(Effect::Ack {
            command: crate::runtime::Command {
                kind: if enabled {
                    crate::runtime::CommandKind::Resume
                } else {
                    crate::runtime::CommandKind::Pause
                },
                generation,
                talk_channel: 0,
            },
            result: 0,
        }));
        expected.push(Output::Runtime(Effect::Status(r.membership)));
        drop(trace);
        assert_eq!(io.completed(), expected);
        assert_eq!(io.sync_outputs, 0);
        io.trace.borrow_mut().clear();
    }
}

#[test]
fn owned_pause_publishes_purge_before_output_first_pending_poll() {
    let (mut runtime, _) = presence_fixture();
    runtime.audio_set_ingress_enabled(true, false);
    runtime.audio_state.boundary.epoch = u32::MAX;
    let mut io = Io::async_only();
    let gates = io.gates.clone();
    runtime.request_audio_pause(7, &mut io);
    {
        let mut strict = FullProtocolPlatform(&mut io);
        let mut future = pin!(runtime.command_work_handler_async(&mut strict));
        assert!(poll(future.as_mut()).is_pending());
        assert_eq!(
            &*gates.borrow(),
            &[(
                crate::protocol_audio::IngressAdmissionGate {
                    enabled: false,
                    epoch: 0
                },
                true
            )]
        );
        // Ordinary actor scheduling must not cancel this handler. Complete it.
        let mut complete = false;
        for _ in 0..32 {
            if poll(future.as_mut()).is_ready() {
                complete = true;
                break;
            }
        }
        assert!(complete, "ordinary handler completed without cancellation");
    }
    assert_eq!(runtime.snapshot().audio.epoch, 0);
    assert_eq!(io.sync_outputs, 0);
}

#[test]
fn owned_ingress_same_value_purge_and_membership_reset_publish_synchronously() {
    let (mut runtime, _) = presence_fixture();
    runtime.audio_set_ingress_enabled(true, false);
    let mut io = Io::async_only();
    let epoch = runtime.snapshot().audio.epoch;
    for _ in 0..2 {
        let mut strict = FullProtocolPlatform(&mut io);
        assert_eq!(
            strict.dispatch_protocol(
                &mut runtime,
                Effect::Ingress {
                    enabled: true,
                    purge: true
                }
            ),
            Some(Reply::None)
        );
    }
    assert_eq!(
        &*io.gates.borrow(),
        &[(
            crate::protocol_audio::IngressAdmissionGate {
                enabled: true,
                epoch
            },
            true
        ); 2]
    );
    finish(
        runtime.membership_handle_async(Input::ResetSession, &mut FullProtocolPlatform(&mut io)),
    )
    .unwrap();
    assert_eq!(
        io.gates.borrow().last(),
        Some(&(runtime.ingress_admission_gate(), true))
    );
}

#[test]
fn owned_resume_cancellation_retains_pre_disable_or_pending_enable_state_without_replay() {
    for polls in [1, 3] {
        let (mut r, _) = presence_fixture();
        r.audio_state.boundary.enabled = false;
        r.audio_state.boundary.epoch = 41;
        r.audio_set_ingress_enabled(true, false);
        let before = r.snapshot().audio;
        let mut io = Io::async_only();
        r.request_audio_resume(7, &mut io);
        {
            let mut p = FullProtocolPlatform(&mut io);
            let mut future = pin!(r.command_work_handler_async(&mut p));
            for _ in 0..polls {
                assert!(poll(future.as_mut()).is_pending());
            }
        }
        assert_eq!(r.pending_request(), 0);
        if polls == 1 {
            assert_eq!(r.snapshot().audio, before);
            assert!(!r.ingress_admission_gate().enabled);
            assert_eq!(r.ingress_admission_gate().epoch, 41);
            assert!(io.bridge_enabled);
            assert!(io.completed().is_empty());
            assert!(!io.trace.borrow().contains(&Call::AudioNow));
        } else {
            assert_eq!(
                r.snapshot().audio,
                AudioBoundary {
                    enabled: true,
                    epoch: 42,
                    receipt_cutoff_us: io.now_us,
                }
            );
            assert!(r.ingress_admission_gate().enabled);
            assert!(!io.bridge_enabled);
            assert_eq!(
                io.completed(),
                vec![Output::Runtime(Effect::BridgeDelivery(false))]
            );
            assert_eq!(
                io.trace.borrow().last(),
                Some(&Call::Start(Output::Runtime(Effect::BridgeDelivery(true))))
            );
        }
        let trace = io.trace.borrow().clone();
        finish(r.command_work_handler_async(&mut FullProtocolPlatform(&mut io)));
        assert_eq!(*io.trace.borrow(), trace);
        assert_eq!(io.sync_outputs, 0);
    }
}

#[test]
fn owned_pause_cancellation_keeps_local_disable_without_completed_gate_or_ack() {
    let (mut r, _) = presence_fixture();
    r.audio_set_ingress_enabled(true, false);
    let mut io = Io::async_only();
    r.request_audio_pause(8, &mut io);
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.command_work_handler_async(&mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.pending_request(), 0);
    assert_eq!(
        r.snapshot().audio,
        AudioBoundary {
            enabled: false,
            epoch: 1,
            receipt_cutoff_us: io.now_us,
        }
    );
    assert!(!r.ingress_admission_gate().enabled);
    assert!(io.bridge_enabled);
    assert!(io.completed().is_empty());
    assert_eq!(
        io.trace.borrow().last(),
        Some(&Call::Start(Output::Runtime(Effect::BridgeDelivery(false))))
    );
    let trace = io.trace.borrow().clone();
    finish(r.command_work_handler_async(&mut FullProtocolPlatform(&mut io)));
    assert_eq!(*io.trace.borrow(), trace);
}

#[test]
fn owned_status_timer_peer_left_cancellation_consumes_token_and_retains_removal() {
    let (mut r, _) = presence_fixture();
    r.session.peers[0].announced = true;
    r.membership.peer_count = 1;
    let mut io = Io {
        now_us: 4_000_000,
        ..Io::async_only()
    };
    r.schedule(Timer::Status, 0, &mut io);
    let token = r.timer(Timer::Status).unwrap();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(r.timer(Timer::Status), None);
    assert_eq!(r.timer(Timer::Scan), None);
    assert_eq!(r.timer(Timer::Join), None);
    assert!(!r.session.peers[0].active);
    assert!(r.session.peers[0].announced);
    assert_eq!(r.session.peers[0].address, [3; 5]);
    assert_eq!(r.membership.peer_count, 0);
    assert_eq!(r.snapshot().auto_ticks, 0);
    assert!(io.completed().is_empty());
    assert_eq!(
        io.trace.borrow().last(),
        Some(&Call::Start(Output::Membership(MembershipOutput::Event {
            id: 3,
            node: Some(3)
        })))
    );
    assert!(!io.trace.borrow().contains(&Call::Sync(Effect::RadioStats)));
    let trace = io.trace.borrow().clone();
    assert_eq!(
        finish(r.handle_async(Event::Timer(token), &mut FullProtocolPlatform(&mut io))),
        0
    );
    assert_eq!(*io.trace.borrow(), trace);
    assert_eq!(r.timer(Timer::Status), None);
    // An explicit new callback restores periodic work; the consumed token does not.
    r.schedule(Timer::Status, 0, &mut io);
    let fresh = r.timer(Timer::Status).unwrap();
    assert_ne!(fresh, token);
    assert_eq!(
        finish(r.handle_async(Event::Timer(fresh), &mut FullProtocolPlatform(&mut io))),
        0
    );
    assert_eq!(r.timer(Timer::Status).unwrap().deadline_us, 5_000_000);
    assert!(io.completed().is_empty());
    assert_eq!(r.snapshot().auto_ticks, 1);
}

#[test]
fn owned_status_timer_coordinator_loss_cancellation_retains_each_partial_stage_without_timer() {
    for polls in [1, 3, 5] {
        let mut r = handover_fixture(2);
        r.membership.peer_count = 2;
        r.audio_set_ingress_enabled(true, false);
        let before = r.membership;
        let mut io = Io {
            now_us: 6_000_000,
            ..Io::async_only()
        };
        r.schedule(Timer::Status, 0, &mut io);
        let token = r.timer(Timer::Status).unwrap();
        {
            let mut p = FullProtocolPlatform(&mut io);
            let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
            for _ in 0..polls {
                assert!(poll(future.as_mut()).is_pending());
            }
        }
        assert_eq!(r.timer(Timer::Status), None);
        assert_eq!(r.timer(Timer::Scan), None);
        assert_eq!(r.timer(Timer::Join), None);
        assert_eq!(r.snapshot().auto_ticks, 1);
        assert_eq!(r.snapshot().status_log_decim, 1);
        assert_eq!(r.audio_state.ingress_enabled, polls == 1);
        let completed = io.completed();
        if polls == 1 {
            assert!(completed.is_empty());
            assert!(matches!(
                io.trace.borrow().last(),
                Some(Call::Start(Output::Membership(
                    MembershipOutput::Log { .. }
                )))
            ));
        } else {
            let Output::Membership(MembershipOutput::Log { bytes, len }) = completed[0] else {
                panic!("missing loss log")
            };
            assert_eq!(
                &bytes[..usize::from(len)],
                b"MESH: Coordinator lost, rescanning"
            );
        }
        if polls < 5 {
            assert_eq!(r.membership, before);
            assert!(r.session.peers[0].active);
            assert!(!io
                .trace
                .borrow()
                .contains(&Call::Member(MembershipEffect::TdmaStop)));
            assert_eq!(completed.len(), usize::from(polls == 3));
            if polls == 3 {
                assert_eq!(
                    io.trace.borrow().last(),
                    Some(&Call::Start(Output::Membership(MembershipOutput::Event {
                        id: 5,
                        node: None,
                    })))
                );
            }
        } else {
            assert_eq!(completed.len(), 2);
            assert_eq!(
                completed[1],
                Output::Membership(MembershipOutput::Event { id: 5, node: None })
            );
            assert_eq!(
                r.membership,
                Membership {
                    state: State::Scanning,
                    ..Membership::default()
                }
            );
            assert!(r.session.peers.iter().all(|peer| !peer.active));
            assert_eq!(
                io.trace.borrow().last(),
                Some(&Call::Start(Output::Membership(MembershipOutput::Status(
                    r.membership
                ))))
            );
        }
        let trace = io.trace.borrow().clone();
        assert_eq!(
            finish(r.handle_async(Event::Timer(token), &mut FullProtocolPlatform(&mut io))),
            0
        );
        assert_eq!(*io.trace.borrow(), trace);
        assert_eq!(r.timer(Timer::Status), None);
        assert_eq!(r.timer(Timer::Scan), None);
        assert_eq!(io.sync_outputs, 0);
    }
}

#[test]
fn owned_scan_timer_pending_status_then_became_coordinator_then_ready() {
    let mut r = Runtime::default();
    r.membership.state = State::Scanning;
    r.address = [1, 2, 3, 4, 5];
    let mut io = Io::async_only();
    r.schedule(Timer::Scan, 0, &mut io);
    let token = r.timer(Timer::Scan).unwrap();
    let trace = io.trace.clone();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
        assert!(poll(future.as_mut()).is_pending());
        assert!(poll(future.as_mut()).is_pending());
        assert_eq!(
            trace
                .borrow()
                .iter()
                .filter(|c| matches!(c, Call::Start(_)))
                .count(),
            1
        );
        assert!(
            matches!(trace.borrow().last(), Some(Call::Start(Output::Membership(MembershipOutput::Status(m)))) if m.state == State::Active && m.role == Role::Coordinator)
        );
        assert_eq!(finish(future.as_mut()), 0);
    }
    assert_eq!(
        io.completed(),
        vec![
            Output::Membership(MembershipOutput::Status(r.membership)),
            Output::Membership(MembershipOutput::Event { id: 4, node: None }),
            Output::Membership(MembershipOutput::Event { id: 1, node: None }),
        ]
    );
    assert_eq!(r.membership.node_id, 1);
    assert_eq!(r.membership.coordinator_id, 1);
    assert_eq!(r.protocol.term, 1);
    assert_eq!(r.timer(Timer::Scan), None);
    assert_eq!(r.timer(Timer::Status).unwrap().deadline_us, 1_100_000);
}

#[test]
fn owned_exhausted_join_timer_awaits_log_then_status_before_scan_schedule() {
    let mut r = Runtime::default();
    r.membership.state = State::Joining;
    r.membership.coordinator_id = 2;
    r.protocol.join_attempts = 10;
    r.address = [1, 2, 3, 4, 5];
    let delay = r.membership_scan_timeout_ms();
    let mut io = Io::async_only();
    r.schedule(Timer::Join, 0, &mut io);
    let token = r.timer(Timer::Join).unwrap();
    io.trace.borrow_mut().clear();
    let trace = io.trace.clone();
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = pin!(r.handle_async(Event::Timer(token), &mut p));
        for _ in 0..2 {
            assert!(poll(future.as_mut()).is_pending());
            assert!(!trace.borrow().iter().any(|c| matches!(
                c,
                Call::Start(Output::Membership(MembershipOutput::Status(_)))
            )));
        }
        for _ in 0..2 {
            assert!(poll(future.as_mut()).is_pending());
            assert!(
                matches!(trace.borrow().last(), Some(Call::Start(Output::Membership(MembershipOutput::Status(m)))) if m.state == State::Scanning)
            );
            assert!(!trace
                .borrow()
                .iter()
                .any(|c| matches!(c, Call::Sync(Effect::Schedule(_)))));
        }
        assert_eq!(poll(future.as_mut()), Poll::Ready(0));
    }
    let completed = io.completed();
    assert_eq!(completed.len(), 2);
    let Output::Membership(MembershipOutput::Log { bytes, len }) = completed[0] else {
        panic!("missing timeout log")
    };
    assert_eq!(
        &bytes[..usize::from(len)],
        format!("MESH: JOIN timeout, rescanning for {delay} ms").as_bytes()
    );
    assert_eq!(bytes[usize::from(len)], 0);
    assert_eq!(
        completed[1],
        Output::Membership(MembershipOutput::Status(r.membership))
    );
    assert_eq!(
        r.membership,
        Membership {
            state: State::Scanning,
            ..Membership::default()
        }
    );
    assert_eq!(r.protocol.join_attempts, 10);
    assert_eq!(r.timer(Timer::Join), None);
    let scan = r.timer(Timer::Scan).unwrap();
    assert_eq!(scan.deadline_us, io.now_us + i64::from(delay) * 1000);
    assert_eq!(
        io.trace.borrow().last(),
        Some(&Call::Sync(Effect::Schedule(scan)))
    );
}
