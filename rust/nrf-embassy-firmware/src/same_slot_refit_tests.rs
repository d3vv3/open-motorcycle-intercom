//! Host composition regression: real firmware admission and portable async audio.
//! Driver starts/emissions below are model observations, not RF qualification.

extern crate std;

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::vec::Vec;

use crate::actor_policy::{
    ActorCallback, AdmissionEvaluation, AdmissionRejection, TxWindow, slot_remaining_physical_us,
};
use crate::timing_clock::{ClockMapping, ClockProfile, PhysicalTime, TimingTime};
use nrf_embassy::platform::{Effect, Integration, Platform, Reply};
use nrf_embassy::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use nrf_embassy::protocol_audio::{
    AudioEffect, AudioPlatform, CURRENT_ONLY_AIRTIME_US, FullAudioPlatform,
};
use nrf_embassy::protocol_membership::{
    Input, MembershipEffect, MembershipPlatform, ProtocolState,
};
use nrf_embassy::protocol_tx::{EAGAIN, RadioSend, RadioSendResult, SubmissionDisposition};
use nrf_embassy::runtime::{Role, Runtime, State, Timer};
use nrf_embassy::tdma::{Snapshot, Tdma, WorkMetadata};
use openhelmet_wire::{audio_bundle as b, mesh as w};

struct Request {
    bytes: Vec<u8>,
    window: TxWindow,
    now: i64,
    admission: AdmissionEvaluation,
    result: RadioSendResult,
}

struct WindowIo {
    snapshot: Snapshot,
    captured: TxWindow,
    mapping: ClockMapping,
    now: i64,
    second_now: i64,
    remaining: Vec<(i64, u32)>,
    requests: Vec<Request>,
    sequence_probes: Vec<Vec<u8>>,
    driver_starts: usize,
    physical_emissions: usize,
}

impl Platform for WindowIo {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        match effect {
            Effect::GetAddress => Reply::Address([1, 2, 3, 4, 5]),
            // The host model schedules its scan timer immediately.
            Effect::ScanTimeout => Reply::Value(0),
            _ => Reply::Value(0),
        }
    }
}

impl MembershipPlatform for WindowIo {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
}

impl AdaptivePlatform for WindowIo {
    fn adaptive_effect(&mut self, effect: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        match effect {
            AdaptiveEffect::FrameCounter => AdaptiveReply::Frame(1),
            _ => AdaptiveReply::Applied,
        }
    }
    fn owned_tx_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, effect: AudioEffect<'_>) -> i64 {
        match effect {
            AudioEffect::NowUs => self.now_us(),
            AudioEffect::SlotRemainingUs => {
                let remaining =
                    slot_remaining_physical_us(self.snapshot, self.now_us(), self.mapping);
                self.remaining.push((self.now_us(), remaining));
                i64::from(remaining)
            }
            _ => 0,
        }
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        // Synchronous probes observe the public packet helper's sequence cursor
        // outside the async window experiment; they model no driver operation.
        assert_eq!(w::decode(packet.bytes, 200).unwrap().header.packet_type, 4);
        self.sequence_probes.push(packet.bytes.to_vec());
        0
    }
    async fn radio_send_classified_async(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        assert!(self.requests.len() < 2, "unexpected third/later request");
        assert_eq!(packet.pipe, 0);
        assert_eq!(packet.address, [0xe7; 5]);
        // Time passes after initial selection, and again after the fresh refit
        // check. Suspend while holding the real packet-helper's borrowed bytes.
        let mut yielded = false;
        poll_fn(|cx| {
            if core::mem::replace(&mut yielded, true) {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
        self.now = if self.requests.is_empty() {
            400
        } else {
            self.second_now
        };
        let admission = self.captured.evaluate_for_clock(
            self.snapshot,
            self.now_us(),
            packet.bytes.len(),
            self.mapping,
        );
        let result = match admission.result {
            Ok(deadline) => {
                assert_eq!(deadline, 1000);
                self.driver_starts += 1;
                self.physical_emissions += 1;
                RadioSendResult {
                    code: 0,
                    disposition: SubmissionDisposition::Accepted,
                }
            }
            Err(_) => RadioSendResult {
                code: EAGAIN,
                disposition: SubmissionDisposition::NotSubmitted,
            },
        };
        self.requests.push(Request {
            bytes: packet.bytes.to_vec(),
            window: self.captured,
            now: self.now_us(),
            admission,
            result,
        });
        result
    }
}

impl AudioPlatform for WindowIo {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_effect(effect)
    }
}

fn run(future: impl Future<Output = ()>) {
    let mut future = pin!(future);
    for _ in 0..8 {
        if future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_ready()
        {
            return;
        }
    }
    panic!("bounded model sends did not finish");
}

fn setup(profile: ClockProfile, second_now: i64) -> (Runtime, WindowIo) {
    let mut runtime = Runtime::default();

    let mapping = ClockMapping::new(profile, PhysicalTime::new(0), TimingTime::new(0));
    // Legacy 1093 timing units floor to physical 1000 us; do not treat
    // legacy timing units as physical airtime.
    let deadline_us = match profile {
        ClockProfile::Normalized => 1000,
        ClockProfile::LegacyNrfRtc10000 => 1093,
    };
    let snapshot = Snapshot {
        running: true,
        slot_index: 0,
        frame_counter: 1,
        generation: 7,
        slot: WorkMetadata {
            generation: 7,
            frame: 1,
            start_us: 0,
            deadline_us,
        },
        ..Tdma::new().snapshot()
    };
    let captured = TxWindow::capture(snapshot, ActorCallback::Slot { slot: 0, frame: 1 }, 0)
        .expect("initial slot window");
    let mut io = WindowIo {
        snapshot,
        captured,
        mapping,
        now: 0,
        second_now,
        remaining: Vec::new(),
        requests: Vec::new(),
        sequence_probes: Vec::new(),
        driver_starts: 0,
        physical_emissions: 0,
    };
    // Activate through the real lifecycle and membership timer handler, without
    // injecting membership or mutating any Runtime-owned state across crates.
    assert_eq!(runtime.init(&mut FullAudioPlatform(&mut io)), 0);
    assert_eq!(runtime.start(&mut FullAudioPlatform(&mut io)), 0);
    let scan = runtime.timer(Timer::Scan).expect("scheduled scan timer");
    runtime
        .membership_handle(Input::Timer(scan), &mut FullAudioPlatform(&mut io))
        .unwrap();
    let membership = runtime.snapshot().membership;
    assert_eq!(membership.state, State::Active);
    assert_eq!(membership.role, Role::Coordinator);
    assert_eq!((membership.node_id, membership.slot_index), (1, 0));
    assert!(runtime.timer(Timer::Scan).is_none());
    assert!(runtime.timer(Timer::Status).is_some());
    assert!(runtime.ingress_admission_gate().enabled);
    let mut raw = [0; 104];
    let len = b::audio_bundle_encode(
        &b::BundleView {
            current_data: &[42; 48],
            previous1_data: &[11; 48],
            previous2_data: &[],
            current_seq: 7,
            stream_id: 1,
            flags: b::CURRENT_ACTIVE | b::PREVIOUS1_PRESENT | b::PREVIOUS1_ACTIVE,
            codec: 2,
        },
        &mut raw,
    );
    assert_eq!(len, raw.len());
    assert_eq!(
        runtime.audio_send_v2(&raw, &mut FullAudioPlatform(&mut io)),
        0
    );
    runtime.audio_drain_ingress(&mut FullAudioPlatform(&mut io));
    assert_eq!(runtime.audio_state().tx_depth(), 1);
    (runtime, io)
}

#[test]
fn async_optional_refit_rechecks_original_physical_window_before_driver_start() {
    for profile in [ClockProfile::Normalized, ClockProfile::LegacyNrfRtc10000] {
        for (second_now, rejection) in [
            (1001, Some(AdmissionRejection::TimingExpired)),
            (468, Some(AdmissionRejection::BudgetTooShort)),
            (460, None),
        ] {
            let (mut runtime, mut io) = setup(profile, second_now);
            let original = io.captured;
            assert_eq!(runtime.tx_send_packet(4, &[], &mut io), 0);
            let prefix_seq = w::decode(&io.sequence_probes[0], 200).unwrap().header.seq;
            let initial_seq = prefix_seq.wrapping_add(1);
            let initial = original.evaluate_for_clock(io.snapshot, 0, 113, io.mapping);
            assert_eq!(initial.result, Ok(1000));
            assert_eq!(
                initial.physical_window_deadline,
                Some(PhysicalTime::new(1000))
            );
            assert_eq!(initial.budget_us, Some(725));
            assert_eq!(CURRENT_ONLY_AIRTIME_US, 533);

            run(runtime.audio_slot_tx_handler_async(0, 1, &mut FullAudioPlatform(&mut io)));
            assert_eq!(io.remaining, [(0, 1000), (400, 600)]);
            assert_eq!(io.requests.len(), 2);
            let first = &io.requests[0];
            let second = &io.requests[1];
            assert_eq!((first.bytes.len(), second.bytes.len()), (113, 65));
            assert_eq!(first.now, 400);
            assert_eq!(first.admission.budget_us, Some(725));
            assert_eq!(first.result.code, EAGAIN);
            assert_eq!(
                first.admission.result,
                Err(AdmissionRejection::BudgetTooShort)
            );
            assert_eq!(
                first.result.disposition,
                SubmissionDisposition::NotSubmitted
            );
            assert_eq!(second.now, second_now);
            assert_eq!(second.admission.result, rejection.map_or(Ok(1000), Err));
            assert_eq!(
                second.result.code,
                if rejection.is_some() { EAGAIN } else { 0 }
            );
            assert_eq!(
                second.result.disposition,
                if rejection.is_some() {
                    SubmissionDisposition::NotSubmitted
                } else {
                    SubmissionDisposition::Accepted
                }
            );
            if second_now < 1000 {
                assert_eq!(second.admission.budget_us, Some(533));
                assert_eq!(
                    second.admission.physical_window_deadline,
                    Some(PhysicalTime::new(1000))
                );
            }
            assert_eq!(io.captured, original);
            assert_eq!(io.snapshot.slot, original.metadata());
            for request in &io.requests {
                assert_eq!(request.window, original);
                assert_eq!(
                    request.window.metadata().deadline_us,
                    original.metadata().deadline_us
                );
            }
            let first = w::decode(&first.bytes, 200).unwrap();
            let second = w::decode(&second.bytes, 200).unwrap();
            assert_eq!(
                first.header,
                w::Header {
                    payload_len: first.header.payload_len,
                    ..second.header
                }
            );
            assert_eq!(second.header.seq, initial_seq);
            let first = b::audio_bundle_parse(first.payload).unwrap();
            let second = b::audio_bundle_parse(second.payload).unwrap();
            assert_eq!(first.current_data, &[42; 48]);
            assert_eq!(second.current_data, first.current_data);
            assert_eq!(second.current_seq, first.current_seq);
            assert_eq!(
                (second.stream_id, second.codec),
                (first.stream_id, first.codec)
            );
            assert_eq!(second.flags, b::CURRENT_ACTIVE);
            assert!(second.previous1_data.is_empty() && second.previous2_data.is_empty());
            let success = u32::from(rejection.is_none());
            assert_eq!(
                (io.driver_starts, io.physical_emissions),
                (success as usize, success as usize)
            );
            assert_eq!(runtime.audio_state().tx_depth(), 0);
            let counters = runtime.audio_state().counters();
            assert_eq!(
                (
                    counters.same_slot_refit_attempt,
                    counters.same_slot_refit_complete
                ),
                (1, success)
            );
            assert_eq!(
                (
                    counters.rf_audio_try,
                    counters.rf_audio_ok,
                    counters.rf_audio_fail
                ),
                (1, success, 1 - success)
            );
            run(runtime.audio_slot_tx_handler_async(0, 2, &mut FullAudioPlatform(&mut io)));
            assert_eq!(io.requests.len(), 2, "consumed entry must not retry later");
            assert_eq!(runtime.audio_state().tx_depth(), 0);
            assert_eq!(runtime.tx_send_packet(4, &[], &mut io), 0);
            assert_eq!(io.sequence_probes.len(), 2);
            let suffix_seq = w::decode(&io.sequence_probes[1], 200).unwrap().header.seq;
            assert_eq!(suffix_seq, initial_seq.wrapping_add(1));
            assert_eq!(
                (io.driver_starts, io.physical_emissions),
                (success as usize, success as usize)
            );
        }
    }
}
