use super::*;
use nrf_embassy::bridge::{Consumer, EAGAIN, TRANSFER_SIZE};
use nrf_embassy::runtime::{Role, State as RuntimeState};
use openhelmet_wire::{
    audio_bundle::{BundleView, audio_bundle_encode},
    bridge as frame,
};

fn ready() -> State {
    let mut state = State::inactive();
    state.initialize().unwrap();
    state
}

fn audio() -> [u8; 56] {
    let mut bytes = [0; 56];
    assert_eq!(
        audio_bundle_encode(
            &BundleView {
                previous1_data: &[],
                previous2_data: &[],
                current_data: &[42; 48],
                current_seq: 65535,
                stream_id: 7,
                flags: 1,
                codec: 2,
            },
            &mut bytes
        ),
        56
    );
    bytes
}

fn equal_bridge(actual: &Bridge, expected: &Bridge) {
    assert_eq!(actual.snapshot(), expected.snapshot());
    assert_eq!(actual.control_storage(), expected.control_storage());
    assert_eq!(actual.audio_storage(), expected.audio_storage());
}

fn execute(state: &mut State, bridge: &mut Bridge) {
    let (ticket, request) = state.claim().unwrap();
    let result = request.apply(bridge);
    state.complete(ticket, result).unwrap();
}

#[cfg(not(feature = "nonblocking-bridge-delivery"))]
#[test]
fn default_delivery_waits_for_server_after_borrowed_transfer() {
    let mut state = ready();
    let mut bridge = Bridge::new();
    let before = bridge.snapshot();
    let ticket = {
        let _prepared = bridge.prepare();
        let ticket = state
            .reserve(Request::audio_bundle(3, &audio()).unwrap())
            .unwrap();
        assert_eq!(state.take_result(ticket), Ok(None));
        ticket
    };
    assert_eq!(bridge.snapshot(), before);
    execute(&mut state, &mut bridge);
    assert_eq!(state.take_result(ticket), Ok(Some(0)));
    assert_eq!(bridge.snapshot().audio_head, 1);
}

#[test]
fn initialization_is_unique_and_snapshots_are_readonly_copies() {
    let mut state = State::inactive();
    let before = state.snapshot();
    assert_eq!(
        state.reserve(Request::DiscardAudio),
        Err(Error::NotInitialized)
    );
    state.initialize().unwrap();
    assert_eq!(state.initialize(), Err(Error::AlreadyInitialized));
    assert!(!before.initialized);
    assert!(state.snapshot().initialized);
}

#[test]
fn one_capacity_busy_in_every_occupied_phase() {
    let mut state = ready();
    let ticket = state.reserve(Request::Delivery(false)).unwrap();
    assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Busy));
    assert_eq!(state.take_result(ticket), Ok(None));
    let (_, request) = state.claim().unwrap();
    assert_eq!(request, Request::Delivery(false));
    assert!(state.claim().is_none());
    assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Busy));
    assert_eq!(state.take_result(ticket), Ok(None));
    assert_eq!(state.complete(ticket, -11), Ok(true));
    assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Busy));
    assert_eq!(state.take_result(ticket), Ok(Some(-11)));
    assert_eq!(state.take_result(ticket), Err(Error::Stale));
    assert!(state.reserve(Request::DiscardAudio).is_ok());
}

#[test]
fn requested_cancellation_removes_pending_before_execution() {
    let mut state = ready();
    let ticket = state.reserve(Request::Delivery(false)).unwrap();
    state.cancel(ticket).unwrap();
    assert!(state.claim().is_none());
    assert_eq!(state.snapshot().phase, Phase::Idle);
    assert_eq!(state.complete(ticket, 0), Err(Error::Stale));
}

#[test]
fn claimed_cancellation_holds_slot_until_actual_mutation_finishes() {
    let mut state = ready();
    let mut bridge = Bridge::new();
    let ticket = state.reserve(Request::Delivery(false)).unwrap();
    let (claimed, request) = state.claim().unwrap();
    state.cancel(ticket).unwrap();
    assert!(state.snapshot().abandoned);
    assert_eq!(state.snapshot().phase, Phase::Claimed);
    assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Busy));
    assert_eq!(state.take_result(ticket), Err(Error::Stale));
    assert_eq!(
        state.complete(claimed, request.apply(&mut bridge)),
        Ok(false)
    );
    assert!(!bridge.snapshot().delivery_enabled);
    assert_eq!(state.snapshot().phase, Phase::Idle);
    assert_eq!(state.take_result(ticket), Err(Error::Stale));
}

#[test]
fn completed_cancellation_releases_without_rollback() {
    let mut state = ready();
    let mut bridge = Bridge::new();
    let ticket = state.reserve(Request::Delivery(false)).unwrap();
    execute(&mut state, &mut bridge);
    state.cancel(ticket).unwrap();
    assert!(!bridge.snapshot().delivery_enabled);
    assert_eq!(state.snapshot().phase, Phase::Idle);
    assert_eq!(state.take_result(ticket), Err(Error::Stale));
}

#[test]
fn stale_completion_and_cancel_never_touch_new_ticket() {
    let mut state = ready();
    let old = state.reserve(Request::DiscardAudio).unwrap();
    state.cancel(old).unwrap();
    let new = state.reserve(Request::Delivery(true)).unwrap();
    assert_ne!(old, new);
    for phase in [Phase::Requested, Phase::Claimed, Phase::Completed] {
        assert_eq!(state.snapshot().phase, phase);
        let before = state.snapshot();
        assert_eq!(state.cancel(old), Err(Error::Stale));
        assert_eq!(state.complete(old, 0), Err(Error::Stale));
        assert_eq!(state.take_result(old), Err(Error::Stale));
        assert_eq!(state.snapshot(), before);
        match phase {
            Phase::Requested => {
                state.claim().unwrap();
            }
            Phase::Claimed => {
                state.complete(new, -90).unwrap();
            }
            _ => {}
        }
    }
    assert_eq!(state.take_result(new), Ok(Some(-90)));
}

#[test]
fn tickets_do_not_wrap_and_overflow_is_sticky() {
    let mut state = ready();
    state.last = u64::MAX - 1;
    let last = state.reserve(Request::DiscardAudio).unwrap();
    assert_eq!(last, Ticket(u64::MAX));
    state.cancel(last).unwrap();
    assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Exhausted));
    assert!(state.snapshot().exhausted);
    assert_eq!(state.snapshot().phase, Phase::Idle);
    assert_eq!(
        state.reserve(Request::Delivery(true)),
        Err(Error::Exhausted)
    );
    assert_eq!(state.initialize(), Err(Error::AlreadyInitialized));
    assert_eq!(state.cancel(last), Err(Error::Stale));
    assert!(state.snapshot().exhausted);
}

#[test]
fn owned_request_is_immutable_after_source_edit_and_reservation() {
    let mut bytes = [9; 207];
    let request = Request::event(6, &bytes).unwrap();
    let mut state = ready();
    let ticket = state.reserve(request).unwrap();
    bytes.fill(1);
    let mut bridge = Bridge::new();
    let before = bridge.snapshot();
    assert_eq!(state.take_result(ticket), Ok(None));
    assert_eq!(bridge.snapshot(), before);
    execute(&mut state, &mut bridge);
    let packet = frame::decode(&bridge.control_storage()[1].bytes, MAX_PAYLOAD).unwrap();
    assert_eq!(packet.payload[0], 6);
    assert_eq!(&packet.payload[1..], &[9; 207]);
    assert_eq!(state.take_result(ticket), Ok(Some(0)));
    assert_eq!(bridge.snapshot().attempts, 0);
    assert_eq!(bridge.snapshot().tx_control, 0);
}

#[test]
fn status_membership_projection_is_exact_legacy_bridge() {
    let membership = Membership {
        state: RuntimeState::Active,
        role: Role::Participant,
        peer_count: 3,
        membership_known: false,
        node_id: 4,
        slot_index: -1,
        coordinator_id: 8,
    };
    let mut actual = Bridge::new();
    let mut expected = Bridge::new();
    assert_eq!(Request::Status(membership).apply(&mut actual), 0);
    expected
        .send_status(
            membership.state as u8,
            membership.role as u8,
            membership.peer_count,
            membership.node_id,
            membership.slot_index,
            membership.coordinator_id,
        )
        .unwrap();
    equal_bridge(&actual, &expected);
    assert_eq!(
        frame::decode(&actual.control_storage()[1].bytes, MAX_PAYLOAD)
            .unwrap()
            .payload,
        &membership.status_v6()
    );
}

#[test]
fn command_ack_preserves_generation_and_signed_result_mapping() {
    for result in [0, -1, -22, i32::MIN, i32::MAX] {
        let mut actual = Bridge::new();
        let mut expected = Bridge::new();
        assert_eq!(
            Request::Ack {
                command: 5,
                generation: 255,
                result
            }
            .apply(&mut actual),
            0
        );
        expected.send_command_ack(5, 255, result).unwrap();
        equal_bridge(&actual, &expected);
    }
}

#[test]
fn event_log_boundaries_and_legacy_error_ordering() {
    assert_eq!(Request::event(0, &[0; 208]), Err(EMSGSIZE));
    assert_eq!(Request::log(&[]), Err(EINVAL));
    for data in [&[][..], &[1][..], &[2; 207][..]] {
        let mut actual = Bridge::new();
        let mut expected = Bridge::new();
        assert_eq!(Request::event(255, data).unwrap().apply(&mut actual), 0);
        expected.send_event(255, data).unwrap();
        equal_bridge(&actual, &expected);
    }
    for len in [1, 208, 209, 1024] {
        let bytes = [7; 1024];
        let mut actual = Bridge::new();
        let mut expected = Bridge::new();
        assert_eq!(Request::log(&bytes[..len]).unwrap().apply(&mut actual), 0);
        expected.send_log(&bytes[..len]).unwrap();
        equal_bridge(&actual, &expected);
    }
}

#[test]
fn audio_actual_eagain_result_is_not_successful_submission() {
    let mut state = ready();
    let mut bridge = Bridge::new();
    bridge.set_audio_delivery_enabled(false);
    let before = bridge.snapshot();
    let ticket = state
        .reserve(Request::audio_bundle(17, &audio()).unwrap())
        .unwrap();
    assert_eq!(state.take_result(ticket), Ok(None));
    execute(&mut state, &mut bridge);
    assert_eq!(state.take_result(ticket), Ok(Some(EAGAIN)));
    assert_eq!(bridge.snapshot(), before);
}

#[test]
fn malformed_audio_rejects_before_gate_and_never_reserves() {
    let mut bridge = Bridge::new();
    bridge.set_audio_delivery_enabled(false);
    for bytes in [&[][..], &[0; 200][..], &[0; 208][..], &[0; 209][..]] {
        assert_eq!(Request::audio_bundle(4, bytes), Err(EINVAL));
        assert_eq!(bridge.send_audio_v2(4, bytes), Err(EINVAL));
    }
    assert_eq!(ready().snapshot().phase, Phase::Idle);
}

#[test]
fn successful_audio_and_control_overwrites_match_complete_storage_and_sequence() {
    let mut actual = Bridge::new();
    let mut expected = Bridge::new();
    // Deliberately crosses sequence wrap; no mailbox burst size/drop policy.
    for source in 0..=255 {
        assert_eq!(
            Request::audio_bundle(source, &audio())
                .unwrap()
                .apply(&mut actual),
            0
        );
        expected.send_audio_v2(source, &audio()).unwrap();
        assert_eq!(
            Request::event(source, &[source; 2])
                .unwrap()
                .apply(&mut actual),
            0
        );
        expected.send_event(source, &[source; 2]).unwrap();
        equal_bridge(&actual, &expected);
    }
    assert_eq!(actual.snapshot().audio_overwrite, 241);
    assert!(actual.snapshot().control_overwrite > 0);
    assert_eq!(actual.snapshot().attempts, 0);
    assert_eq!(actual.snapshot().tx_seq, 1);
}

struct Sink;
impl Consumer for Sink {
    fn accept_audio_v2(&mut self, _: &[u8]) -> i32 {
        0
    }
    fn request_start(&mut self, _: u8, _: u8) {}
    fn request_stop(&mut self, _: u8) {}
    fn request_pause(&mut self, _: u8) {}
    fn request_resume(&mut self, _: u8) {}
    fn request_status(&mut self) {}
    fn pulse_ack(&mut self) {}
}

#[test]
fn repeated_delivery_and_discard_flush_and_invalidate_real_ingress_dedupe() {
    let mut bridge = Bridge::new();
    let mut rx = [0; TRANSFER_SIZE];
    frame::encode(&mut rx, MAX_PAYLOAD, 9, 6, &audio());
    for request in [
        Request::Delivery(true),
        Request::Delivery(true),
        Request::DiscardAudio,
        Request::Delivery(false),
        Request::Delivery(false),
    ] {
        bridge.prepare().finish(true, &rx, &mut Sink);
        assert!(bridge.snapshot().last_audio_seq_valid);
        let enabled = bridge.snapshot().delivery_enabled;
        if enabled {
            bridge.send_audio_v2(1, &audio()).unwrap();
        }
        let stale = *bridge.audio_storage();
        assert_eq!(request.apply(&mut bridge), 0);
        assert!(!bridge.snapshot().last_audio_seq_valid);
        assert_eq!(bridge.snapshot().audio_tail, bridge.snapshot().audio_head);
        assert_eq!(*bridge.audio_storage(), stale);
    }
    assert_eq!(bridge.snapshot().ingress_duplicate, 0);
    assert_eq!(bridge.snapshot().ingress_ok, 5);
}

#[test]
fn drop_model_cancels_each_phase_once() {
    struct Call<'a> {
        state: &'a mut State,
        ticket: Ticket,
    }
    impl Drop for Call<'_> {
        fn drop(&mut self) {
            self.state.cancel(self.ticket).unwrap();
        }
    }
    for phase in [Phase::Requested, Phase::Claimed, Phase::Completed] {
        let mut state = ready();
        let ticket = state.reserve(Request::Delivery(false)).unwrap();
        if phase != Phase::Requested {
            state.claim().unwrap();
        }
        if phase == Phase::Completed {
            state.complete(ticket, 0).unwrap();
        }
        drop(Call {
            state: &mut state,
            ticket,
        });
        if phase == Phase::Claimed {
            assert_eq!(state.snapshot().phase, Phase::Claimed);
            assert_eq!(state.complete(ticket, 0), Ok(false));
        }
        assert_eq!(state.snapshot().phase, Phase::Idle);
    }
}

#[test]
fn thread_gate_precedes_access_and_notification_reentry_has_no_loan() {
    use crate::transport::{TransportError, thread_access};
    use core::cell::RefCell;
    let state = RefCell::new(ready());
    for vector in [1, 2, 3, 16, 255] {
        assert_eq!(
            thread_access(
                vector,
                || -> ((), bool) { panic!("must not access") },
                || panic!("must not notify")
            ),
            Err(TransportError::WrongContext)
        );
    }
    thread_access(
        0,
        || {
            let ticket = state.borrow_mut().reserve(Request::DiscardAudio).unwrap();
            (ticket, true)
        },
        || {
            assert_eq!(state.borrow().snapshot().phase, Phase::Requested);
            assert!(state.borrow_mut().claim().is_some());
        },
    )
    .unwrap();
}
