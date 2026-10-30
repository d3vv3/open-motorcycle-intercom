use super::*;
use core::cell::{Cell, RefCell};
use nrf_embassy::platform::{Effect, Integration, Platform, Reply};
use nrf_embassy::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use nrf_embassy::protocol_audio::AudioEffect;
use nrf_embassy::protocol_membership::{MembershipEffect, MembershipPlatform, ProtocolState};
use nrf_embassy::runtime::{Event, Membership, Role, Runtime, State};
use openhelmet_wire::audio_bundle as b;

fn gate(enabled: bool, epoch: u32) -> IngressAdmissionGate {
    IngressAdmissionGate { enabled, epoch }
}
fn inbox() -> TransportState {
    let mut state = TransportState::inactive();
    state.initialize_owned_gate(gate(true, 7)).unwrap();
    state
}
fn prepared(token: IngressToken, seq: u16) -> PreparedIngress {
    let data = [42; 48];
    let view = b::BundleView {
        current_data: &data,
        previous1_data: &[],
        previous2_data: &[],
        current_seq: seq,
        stream_id: 1,
        flags: 1,
        codec: 2,
    };
    let mut raw = [0; 200];
    let len = b::audio_bundle_encode(&view, &mut raw);
    assert_ne!(len, 0);
    token.validate(&raw[..len])
}
fn command(kind: CommandKind, generation: u8) -> Command {
    Command {
        kind,
        generation,
        talk_channel: 2,
    }
}

#[test]
fn bridge_ack_only_follows_zero_publication_or_previously_admitted_duplicate() {
    use nrf_embassy::bridge::{Bridge, Consumer, MAX_PAYLOAD, TRANSFER_SIZE};
    use openhelmet_wire::bridge as wire;
    struct Adapter<'a> {
        state: &'a mut TransportState,
        token: IngressToken,
        pulses: u32,
    }
    impl Consumer for Adapter<'_> {
        fn accept_audio_v2(&mut self, payload: &[u8]) -> i32 {
            self.state.publish(self.token.validate(payload))
        }
        fn request_start(&mut self, _: u8, _: u8) {}
        fn request_stop(&mut self, _: u8) {}
        fn request_pause(&mut self, _: u8) {}
        fn request_resume(&mut self, _: u8) {}
        fn request_status(&mut self) {}
        fn pulse_ack(&mut self) {
            self.pulses += 1;
        }
    }
    let mut raw = [0; 56];
    assert_eq!(
        b::audio_bundle_encode(
            &b::BundleView {
                current_data: &[42; 48],
                previous1_data: &[],
                previous2_data: &[],
                current_seq: 1,
                stream_id: 1,
                flags: 1,
                codec: 2,
            },
            &mut raw
        ),
        raw.len()
    );
    let mut state = inbox();
    let token = state.capture().unwrap(); // Before Bridge prepare/transfer.
    let mut bridge = Bridge::new();
    let mut rx = [0; TRANSFER_SIZE];
    wire::encode(&mut rx, MAX_PAYLOAD, 1, 6, &raw);
    let mut adapter = Adapter {
        state: &mut state,
        token,
        pulses: 0,
    };
    bridge.prepare().finish(true, &rx, &mut adapter);
    assert_eq!(adapter.pulses, 1);
    assert_eq!(adapter.state.snapshot().queued, 1);
    bridge.prepare().finish(true, &rx, &mut adapter);
    assert_eq!(adapter.pulses, 2);
    assert_eq!(adapter.state.snapshot().queued, 1); // Duplicate admitted, no second publication.
    adapter.state.publish_gate(gate(true, 8), true);
    wire::encode(&mut rx, MAX_PAYLOAD, 2, 6, &raw);
    bridge.prepare().finish(true, &rx, &mut adapter);
    assert_eq!(adapter.pulses, 2); // Original token now stale.
    assert_eq!(adapter.state.snapshot().queued, 0);
    wire::encode(&mut rx, MAX_PAYLOAD, 3, 6, &[0; 4]);
    bridge.prepare().finish(true, &rx, &mut adapter);
    assert_eq!(adapter.pulses, 2);
    assert_eq!(adapter.state.take_accounting().bundle_bad, 1);
}

#[test]
fn sixteen_usable_seventeenth_succeeds_drops_oldest_and_preserves_fifo() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    for seq in 0..17 {
        assert_eq!(state.publish(prepared(token, seq)), 0);
    }
    assert_eq!(MAILBOX_RING, 16);
    assert_eq!(state.snapshot().queued, 16);
    assert_eq!(
        state.snapshot().accounting.ingress,
        IngressDeltaAccounting {
            overwritten: 1,
            purged: 0,
            failed_enqueue: 0
        }
    );
    for seq in 1..17 {
        assert_eq!(state.pop_owned(), Some(prepared(token, seq)));
    }
    assert_eq!(state.pop_owned(), None);
}

#[test]
fn repeated_ring_wrap_retains_latest_sixteen() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    for seq in 0..73 {
        assert_eq!(state.publish(prepared(token, seq)), 0);
    }
    for seq in 57..73 {
        assert_eq!(state.pop_owned(), Some(prepared(token, seq)));
    }
    assert_eq!(state.take_accounting().ingress.overwritten, 57);
}

#[test]
fn purge_counts_occupancy_resets_cursors_retains_backing_packets() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    for seq in 0..16 {
        state.publish(prepared(token, seq));
    }
    let _ = state.pop_owned();
    let storage = state.data;
    state.publish_gate(gate(false, 8), true);
    assert_eq!(state.data, storage);
    assert_eq!((state.read, state.write, state.count), (0, 0, 0));
    assert_eq!(state.pop_owned(), None);
    assert_eq!(state.take_accounting().ingress.purged, 15);
    state.publish_gate(gate(true, 9), true);
    assert_eq!(state.take_accounting(), PendingAccounting::default());
    let new = prepared(state.capture().unwrap(), 50);
    assert_eq!(state.publish(new), 0);
    assert_eq!(state.pop_owned(), Some(new));
}

#[test]
fn token_captured_before_validation_is_stale_at_publication() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    let record = prepared(token, 1);
    state.publish_gate(gate(true, 8), true);
    assert_eq!(state.publish(record), EAGAIN);
    assert_eq!(state.snapshot().queued, 0);
    assert_eq!(state.take_accounting(), PendingAccounting::default());
}

#[test]
fn closed_gate_captures_epoch_but_rejects_valid_without_counters() {
    let mut state = inbox();
    state.publish_gate(gate(false, 7), false);
    let token = state.capture().unwrap();
    assert_eq!(token.epoch(), 7);
    assert_eq!(state.publish(prepared(token, 1)), EAGAIN);
    assert_eq!(state.take_accounting(), PendingAccounting::default());
}

#[test]
fn malformed_precedes_closed_and_stale_gate_without_queue_or_ack_success() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    state.publish_gate(gate(false, 8), true);
    assert_eq!(state.publish(token.validate(&[])), EINVAL);
    assert_eq!(state.snapshot().queued, 0);
    assert_eq!(
        state.take_accounting(),
        PendingAccounting {
            bundle_bad: 1,
            ..Default::default()
        }
    );
}

#[test]
fn pending_scalar_deltas_wrap_and_are_taken_exactly_once() {
    let mut state = inbox();
    let token = state.capture().unwrap();
    // Private unit seeding, no public counter/storage forge or restore API.
    state.pending.bundle_bad = u32::MAX;
    assert_eq!(state.publish(token.validate(&[])), EINVAL);
    assert_eq!(state.pending.bundle_bad, 0);
    for seq in 0..16 {
        state.publish(prepared(token, seq));
    }
    state.pending.ingress.overwritten = u32::MAX;
    state.pending.ingress.purged = u32::MAX;
    state.publish(prepared(token, 16));
    state.publish_gate(gate(false, 8), true);
    state.pending.bundle_bad = u32::MAX;
    let delta = state.take_accounting();
    assert_eq!(delta.bundle_bad, u32::MAX);
    assert_eq!(
        delta.ingress,
        IngressDeltaAccounting {
            overwritten: 0,
            purged: 15,
            failed_enqueue: 0
        }
    );
    assert_eq!(state.take_accounting(), PendingAccounting::default());
}

#[test]
fn pause_resume_epoch_wrap_explicitly_purges_before_new_publications() {
    let mut state = inbox();
    state.publish_gate(gate(true, u32::MAX), true);
    let token = state.capture().unwrap();
    state.publish(prepared(token, 1));
    state.publish_gate(gate(false, 0), true);
    assert_eq!(state.snapshot().queued, 0);
    state.publish_gate(gate(true, 1), true);
    assert_eq!(state.publish(prepared(token, 2)), EAGAIN);
    assert_eq!(state.take_accounting().ingress.purged, 1);
}

#[test]
fn command_last_writer_status_independent_and_generation_wrap_not_filtered() {
    let mut state = inbox();
    for kind in [
        CommandKind::Start,
        CommandKind::Pause,
        CommandKind::Resume,
        CommandKind::Stop,
    ] {
        assert_eq!(state.publish_command(command(kind, 255)), 0);
    }
    assert_eq!(state.request_status(), 0);
    assert_eq!(state.request_status(), 0);
    assert_eq!(
        state.take_requests(),
        PendingRequests {
            command: Some(command(CommandKind::Stop, 255)),
            status: true
        }
    );
    assert_eq!(state.publish_command(command(CommandKind::Start, 0)), 0);
    assert_eq!(
        state.take_requests().command,
        Some(command(CommandKind::Start, 0))
    );
    assert_eq!(state.take_requests(), PendingRequests::default());
}

#[test]
fn publications_after_take_survive_for_later_actor_pass() {
    let mut state = inbox();
    state.publish_command(command(CommandKind::Start, 2));
    let taken = state.take_requests();
    state.request_status();
    state.publish_command(command(CommandKind::Stop, 3));
    assert_eq!(taken.command, Some(command(CommandKind::Start, 2)));
    assert_eq!(
        state.take_requests(),
        PendingRequests {
            command: Some(command(CommandKind::Stop, 3)),
            status: true
        }
    );
}

#[test]
fn invalid_start_groups_do_not_replace_existing_command() {
    let mut state = inbox();
    state.publish_command(command(CommandKind::Stop, 2));
    for talk_channel in [0, 4] {
        assert_eq!(
            state.publish_command(Command {
                talk_channel,
                ..command(CommandKind::Start, 3)
            }),
            EINVAL
        );
    }
    assert_eq!(
        state.take_requests().command,
        Some(command(CommandKind::Stop, 2))
    );
}

#[test]
fn initialization_once_and_snapshot_copy_cannot_mutate_live_state() {
    let mut state = TransportState::inactive();
    assert_eq!(state.capture(), Err(TransportError::NotInitialized));
    assert_eq!(state.request_status(), EAGAIN);
    state.initialize_owned_gate(gate(false, 0)).unwrap();
    assert_eq!(state.initialize_owned_gate(gate(true, 1)), Err(EALREADY));
    let mut copy = state.snapshot();
    copy.gate.enabled = true;
    copy.queued = 16;
    assert_ne!(copy, state.snapshot());
    assert!(!state.snapshot().gate.enabled);
}

#[test]
fn every_exception_rejects_before_access_and_notification() {
    let called = Cell::new(false);
    for vector in 1..=511 {
        assert_eq!(
            thread_access(
                vector,
                || {
                    called.set(true);
                    ((), true)
                },
                || called.set(true)
            ),
            Err(TransportError::WrongContext)
        );
        assert!(!called.get());
    }
    assert_eq!(thread_mode(0), Ok(()));
}

#[test]
fn production_access_notifies_only_after_borrow_ends_and_allows_reentry() {
    let state = RefCell::new(inbox());
    let result = thread_access(
        0,
        || {
            let mut loan = state.borrow_mut();
            loan.request_status();
            (loan.take_requests(), true)
        },
        || {
            thread_access(
                0,
                || {
                    let mut loan = state.borrow_mut();
                    loan.publish_command(command(CommandKind::Stop, 4));
                    ((), false)
                },
                || panic!("unexpected nested wake"),
            )
            .unwrap();
        },
    )
    .unwrap();
    assert!(result.status);
    assert_eq!(
        state.borrow_mut().take_requests().command,
        Some(command(CommandKind::Stop, 4))
    );
}

// ActorOracle harness only: non-strict membership injection is not hardware
// eligibility. Owned audio remains required for direct ingress consumption.
struct Io;
impl Platform for Io {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
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
    fn adaptive_effect(&mut self, _: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        AdaptiveReply::Applied
    }
    fn owned_audio_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, _: AudioEffect<'_>) -> i64 {
        0
    }
}
fn active() -> Runtime {
    let mut runtime = Runtime::default();
    inject_membership(&mut runtime, State::Active);
    runtime.audio_set_local_enabled(true, &mut Io);
    runtime.audio_set_ingress_enabled(true, false);
    runtime.use_external_ingress().unwrap();
    runtime
}

fn inject_membership(runtime: &mut Runtime, state: State) {
    assert_eq!(
        runtime.handle(
            Event::InjectMembership(Membership {
                state,
                role: Role::Participant,
                node_id: 1,
                slot_index: 1,
                ..Membership::default()
            }),
            &mut Io,
        ),
        0
    );
}

#[test]
fn source_mailbox_direct_consume_has_no_second_queue_and_closed_gate_no_purge_processes() {
    let mut runtime = active();
    let mut state = TransportState::inactive();
    state
        .initialize_owned_gate(runtime.ingress_admission_gate())
        .unwrap();
    let token = state.capture().unwrap();
    assert_eq!(state.publish(prepared(token, 1)), 0);
    let storage = *runtime.audio_state().ingress_storage();
    runtime.audio_set_ingress_enabled(false, false);
    state.publish_gate(runtime.ingress_admission_gate(), false);
    assert_eq!(state.publish(prepared(token, 2)), EAGAIN);
    let owned = state.pop_owned().unwrap();
    assert_eq!(runtime.consume_prepared_ingress(&owned, &mut Io), 0);
    assert_eq!(runtime.audio_state().ingress_used(), 0);
    assert_eq!(*runtime.audio_state().ingress_storage(), storage);
    assert_eq!(runtime.audio_state().counters().spi_in_frames, 1);
    assert_eq!(runtime.audio_state().tx_depth(), 1);
}

#[test]
fn scanning_is_actor_policy_not_producer_admission_and_deltas_apply_once() {
    let mut runtime = active();
    inject_membership(&mut runtime, State::Scanning);
    let mut state = TransportState::inactive();
    state
        .initialize_owned_gate(runtime.ingress_admission_gate())
        .unwrap();
    let token = state.capture().unwrap();
    assert_eq!(state.publish(prepared(token, 1)), 0);
    let owned = state.pop_owned().unwrap();
    assert_eq!(runtime.consume_prepared_ingress(&owned, &mut Io), EAGAIN);
    assert_eq!(runtime.audio_state().counters().ingress_inactive_drop, 1);
    for seq in 0..17 {
        assert_eq!(state.publish(prepared(token, seq)), 0);
    }
    state.publish(token.validate(&[]));
    state.publish_gate(gate(false, token.epoch()), true);
    for _ in 0..2 {
        let pending = state.take_accounting();
        runtime.account_external_ingress(pending.ingress).unwrap();
        runtime
            .account_external_ingress_validation(pending.bundle_bad)
            .unwrap();
    }
    let counters = runtime.audio_state().counters();
    assert_eq!(counters.ingress_msgq_drop, 1);
    assert_eq!(counters.tx_overwrite, 1);
    assert_eq!(counters.ingress_purge_drop, 16);
    assert_eq!(counters.bundle_bad, 1);
}

struct StrictBoundaryIo<'a> {
    transport: &'a RefCell<TransportState>,
    now: &'a Cell<i64>,
    output: &'a Cell<u8>,
}
impl Platform for StrictBoundaryIo<'_> {
    fn integration(&self) -> Integration {
        Integration::HardwareUnqualified
    }
    fn now_us(&self) -> i64 {
        self.now.get()
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        match effect {
            Effect::GetAddress => Reply::Address([2; 5]),
            _ => Reply::Value(0),
        }
    }
    fn ingress_gate_changed(&mut self, gate: IngressAdmissionGate, purge: bool) {
        self.transport.borrow_mut().publish_gate(gate, purge);
    }
    fn prepare_ingress_boundary(&mut self, gate: IngressAdmissionGate) {
        self.ingress_gate_changed(crate::actor_policy::ingress_boundary_gate(gate), true);
    }
    async fn apply_async(&mut self, effect: Effect) -> Reply {
        self.output.set(if effect == Effect::BridgeDelivery(false) {
            1
        } else {
            2
        });
        pending_once().await;
        Reply::Value(0)
    }
}
impl MembershipPlatform for StrictBoundaryIo<'_> {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
    async fn membership_output_async(
        &mut self,
        output: nrf_embassy::protocol_membership::MembershipOutput,
    ) -> bool {
        use nrf_embassy::protocol_membership::MembershipOutput;
        self.output.set(match output {
            MembershipOutput::Log { .. } => 3,
            MembershipOutput::Event { id: 5, .. } => 4,
            _ => 5,
        });
        pending_once().await;
        true
    }
}
impl AdaptivePlatform for StrictBoundaryIo<'_> {
    fn adaptive_effect(&mut self, _: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        AdaptiveReply::Applied
    }
}
impl nrf_embassy::protocol_audio::AudioPlatform for StrictBoundaryIo<'_> {
    fn audio_io(&mut self, _: AudioEffect<'_>) -> i64 {
        0
    }
}
impl nrf_embassy::protocol_rx::RxPlatform for StrictBoundaryIo<'_> {
    fn rx_io(&mut self, _: nrf_embassy::protocol_rx::RxEffect) -> i32 {
        0
    }
}
async fn pending_once() {
    let mut pending = true;
    core::future::poll_fn(|_| {
        if core::mem::take(&mut pending) {
            core::task::Poll::Pending
        } else {
            core::task::Poll::Ready(())
        }
    })
    .await
}
fn poll_boundary<F: core::future::Future>(
    future: core::pin::Pin<&mut F>,
) -> core::task::Poll<F::Output> {
    future.poll(&mut core::task::Context::from_waker(
        core::task::Waker::noop(),
    ))
}
fn finish_boundary<F: core::future::Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    loop {
        if let core::task::Poll::Ready(output) = poll_boundary(future.as_mut()) {
            return output;
        }
    }
}

#[test]
fn strict_first_resume_closes_real_transport_before_initial_disable_and_repeat_keeps_queue() {
    use nrf_embassy::protocol_runtime::FullProtocolPlatform;
    let transport = RefCell::new(inbox());
    let now = Cell::new(100_000);
    let output = Cell::new(0);
    let mut io = StrictBoundaryIo {
        transport: &transport,
        now: &now,
        output: &output,
    };
    let mut runtime = Runtime::default();
    assert_eq!(runtime.init(&mut FullProtocolPlatform(&mut io)), 0);
    runtime.use_external_ingress().unwrap();
    runtime.audio_set_ingress_enabled(true, false);
    // init() enables the boundary; a prior Pause makes the first Resume real.
    runtime.audio_set_local_enabled(false, &mut FullProtocolPlatform(&mut io));
    for generation in [1, 2] {
        let gate = runtime.ingress_admission_gate();
        // Queue sixteen packets under an open transport gate of this epoch.
        let open = nrf_embassy::protocol_audio::IngressAdmissionGate {
            enabled: true,
            epoch: gate.epoch,
        };
        transport.borrow_mut().publish_gate(open, false);
        let token = transport.borrow().capture().unwrap();
        for seq in 0..16 {
            assert_eq!(transport.borrow_mut().publish(prepared(token, seq)), 0);
        }
        runtime.request_audio_resume(generation, &mut FullProtocolPlatform(&mut io));
        output.set(0);
        {
            let mut p = FullProtocolPlatform(&mut io);
            let mut future = core::pin::pin!(runtime.command_work_handler_async(&mut p));
            assert!(poll_boundary(future.as_mut()).is_pending());
            if generation == 1 {
                // First resume: the real transport is closed and purged before
                // the initial bridge disable is even started.
                assert_eq!(output.get(), 1);
                assert_eq!(
                    transport.borrow().snapshot().gate,
                    crate::actor_policy::ingress_boundary_gate(gate)
                );
                assert_eq!(transport.borrow_mut().publish(prepared(token, 99)), EAGAIN);
                assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 16);
                assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 0);
            } else {
                // Repeated resume: the Ack is the first output; no gate traffic.
                assert_eq!(output.get(), 2);
            }
            finish_boundary(future.as_mut());
        }
        if generation == 1 {
            assert!(runtime.ingress_admission_gate().enabled);
            assert_ne!(runtime.ingress_admission_gate().epoch, gate.epoch);
            assert_eq!(
                transport.borrow().snapshot().gate,
                runtime.ingress_admission_gate()
            );
        } else {
            // The transport gate and its sixteen queued packets are untouched.
            assert_eq!(runtime.ingress_admission_gate(), gate);
            assert_eq!(transport.borrow().snapshot().gate, open);
            assert_eq!(transport.borrow().snapshot().queued, 16);
        }
        assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 0);
    }
}

#[test]
fn strict_participant_loss_closes_real_transport_during_log_and_event_preserving_source_epoch() {
    use nrf_embassy::protocol_membership::Input;
    use nrf_embassy::protocol_runtime::FullProtocolPlatform;
    use openhelmet_wire::mesh as w;
    let transport = RefCell::new(inbox());
    let now = Cell::new(100_000);
    let output = Cell::new(0);
    let mut io = StrictBoundaryIo {
        transport: &transport,
        now: &now,
        output: &output,
    };
    let mut runtime = Runtime::default();
    assert_eq!(runtime.init(&mut FullProtocolPlatform(&mut io)), 0);
    runtime.use_external_ingress().unwrap();
    assert_eq!(runtime.start(&mut FullProtocolPlatform(&mut io)), 0);
    runtime
        .membership_handle(Input::Discover(1), &mut FullProtocolPlatform(&mut io))
        .unwrap();
    let ack = w::JoinAckV3 {
        target: w::Identity {
            address_len: 5,
            address: [2, 2, 2, 2, 2, 0],
        },
        capabilities: 2,
        assigned_id: 2,
        slot_index: 1,
        coordinator_id: 1,
        term: 0,
    };
    let mut payload = [0; w::JoinAckV3::SIZE];
    ack.encode(&mut payload).unwrap();
    runtime
        .membership_handle(
            Input::Packet {
                header: w::Header {
                    version: 5,
                    packet_type: 16,
                    src_id: 1,
                    seq: 1,
                    ttl: 1,
                    flags: 0,
                    talk_channel: 0,
                    payload_len: payload.len() as u16,
                },
                payload: &payload,
            },
            &mut FullProtocolPlatform(&mut io),
        )
        .unwrap();
    assert_eq!(runtime.snapshot().membership.role, Role::Participant);
    runtime.audio_set_local_enabled(true, &mut FullProtocolPlatform(&mut io));
    let gate = runtime.ingress_admission_gate();
    assert!(gate.enabled);
    let token = transport.borrow().capture().unwrap();
    assert_eq!(transport.borrow_mut().publish(prepared(token, 1)), 0);
    now.set(6_000_000);
    {
        let mut p = FullProtocolPlatform(&mut io);
        let mut future = core::pin::pin!(runtime.status_work_handler_async(&mut p));
        assert!(poll_boundary(future.as_mut()).is_pending());
        assert_eq!(output.get(), 3);
        assert_eq!(
            transport.borrow().snapshot().gate,
            crate::actor_policy::ingress_boundary_gate(gate)
        );
        assert_eq!(transport.borrow_mut().publish(prepared(token, 2)), EAGAIN);
        assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 1);
        assert!(poll_boundary(future.as_mut()).is_pending());
        assert_eq!(output.get(), 4);
        assert_eq!(transport.borrow_mut().publish(prepared(token, 3)), EAGAIN);
        assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 0);
        finish_boundary(future.as_mut());
    }
    assert_eq!(runtime.snapshot().membership.state, State::Scanning);
    assert_eq!(runtime.ingress_admission_gate().epoch, gate.epoch);
    assert_eq!(
        transport.borrow().snapshot().gate,
        runtime.ingress_admission_gate()
    );
    assert_eq!(transport.borrow_mut().take_accounting().ingress.purged, 0);
}
