//! IO only. All protocol state and algorithms remain in strict FullProtocolPlatform.

use nrf_embassy::platform::{Effect, Integration, Platform, Reply};
use nrf_embassy::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use nrf_embassy::protocol_audio::{AudioEffect, AudioPlatform, IngressAdmissionGate};
use nrf_embassy::protocol_membership::{
    MembershipEffect, MembershipOutput, MembershipPlatform, ProtocolState,
};
use nrf_embassy::protocol_rx::{RxEffect, RxPlatform};
use nrf_embassy::protocol_tx::{ClockSnapshot, RadioSend, RadioSendResult, SubmissionDisposition};
use nrf_embassy::runtime::{ENOTSUP, Membership};
use nrf_embassy::tdma::Tdma;
use nrf_embassy_firmware::actor_policy::{
    TxWindow, audio_fit_remaining_us, ingress_boundary_gate, runtime_output,
    slot_remaining_physical_us,
};
use nrf_embassy_firmware::bridge_rpc::{Error, Request};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::sync_trace::{ClockSample, SendResult, Tracker};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::tx_diagnostics::{LastAdmissionRejection, Stage, Stats, classify};

use crate::{
    bridge_rpc_mailbox, radio_receiver::RadioOwner, shared_clock::SharedClock,
    timing_backend::TimingPlatform, transport_mailbox,
};

pub(crate) struct Backend {
    pub(crate) tdma: Tdma,
    pub(crate) timing: TimingPlatform,
    pub(crate) radio: RadioOwner,
    pub(crate) rpc: bridge_rpc_mailbox::Actor,
    pub(crate) producer: transport_mailbox::Producer,
    pub(crate) ingress_work: bool,
    pub(crate) rf_work: bool,
    #[cfg(feature = "nonblocking-bridge-delivery")]
    pub(crate) rx_outputs: bool,
    pub(crate) tx_window: Option<TxWindow>,
    #[cfg(feature = "sync-trace")]
    pub(crate) sync_trace: Tracker,
    #[cfg(feature = "sync-trace")]
    pub(crate) tx_diagnostics: Stats,
}

fn rpc_errno(error: Error) -> i32 {
    match error {
        Error::Busy => -16,
        Error::Stale => -116,
        Error::Exhausted => -75,
        Error::NotInitialized => ENOTSUP,
        Error::AlreadyInitialized => -114,
        Error::WrongContext => -1,
    }
}

impl Backend {
    async fn call(&mut self, request: Result<Request, i32>) -> i32 {
        #[cfg(feature = "nonblocking-bridge-delivery")]
        if nrf_embassy_firmware::bridge_delivery::direct_rx(self.rx_outputs, &request) {
            return self.rpc.apply_rx(request);
        }
        match request {
            Ok(request) => {
                #[cfg(feature = "slot-entry-trace")]
                let _trace = nrf_embassy_firmware::slot_entry_trace::RpcGuard::new(request.kind());
                self.rpc.call(request).await.unwrap_or_else(rpc_errno)
            }
            Err(error) => error,
        }
    }

    pub(crate) fn healthy(&mut self) {
        self.timing.assert_healthy();
    }

    fn tx_deadline(&self, window: TxWindow, packet_len: usize) -> Option<u64> {
        let snapshot = self.tdma.snapshot();
        let physical_now = self.now_us();
        let mapping = self.timing.clock_mapping();
        window
            .evaluate_for_clock(snapshot, physical_now, packet_len, mapping)
            .result
            .ok()
    }
}

impl Platform for Backend {
    fn integration(&self) -> Integration {
        Integration::HardwareUnqualified
    }
    fn now_us(&self) -> i64 {
        crate::clock_or_halt(SharedClock::read_static())
    }
    fn ingress_gate_changed(&mut self, gate: IngressAdmissionGate, purge: bool) {
        self.producer
            .publish_gate(gate, purge)
            .expect("synchronous gate publication");
    }
    fn prepare_ingress_boundary(&mut self, gate: IngressAdmissionGate) {
        self.ingress_gate_changed(ingress_boundary_gate(gate), true);
    }
    fn tx_window_valid(&self) -> bool {
        self.tx_window
            .is_none_or(|window| self.tx_deadline(window, 0).is_some())
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        assert!(
            runtime_output(effect).is_none(),
            "unexpected synchronous Bridge output"
        );
        match effect {
            Effect::InitTimer(_)
            | Effect::InitCommand
            | Effect::SubmitCommand
            | Effect::Schedule(_)
            | Effect::SchedulePending(_, _)
            | Effect::Cancel(_) => Reply::None,
            // Runtime owns all Scan/Join/Status tokens; the actor observes their
            // real deadlines each turn, so these notifications need no queue.
            Effect::SetControlCallback => {
                self.tdma.set_control_callback(true);
                Reply::None
            }
            Effect::GetAddress => Reply::Address(self.radio.identity()),
            Effect::SetChannel(channel) => Reply::Value(self.radio.configure(channel)),
            Effect::StartRx => Reply::Value(self.radio.start()),
            Effect::StopRx => Reply::Value(self.radio.stop()),
            Effect::TdmaStop => {
                self.tdma.stop(&mut self.timing);
                self.healthy();
                Reply::None
            }
            Effect::RadioStats | Effect::TdmaStats | Effect::WsDiag => {
                self.healthy();
                Reply::None
            }
            _ => Reply::Value(ENOTSUP),
        }
    }
    async fn apply_async(&mut self, effect: Effect) -> Reply {
        if let Some(request) = runtime_output(effect) {
            Reply::Value(self.call(request).await)
        } else {
            self.apply(effect)
        }
    }
}

impl MembershipPlatform for Backend {
    fn membership_effect(&mut self, effect: MembershipEffect, _: &mut ProtocolState) -> bool {
        match effect {
            MembershipEffect::BindWork => true, // sole actor dispatch, no external worker
            MembershipEffect::TdmaStop => {
                self.tdma.stop(&mut self.timing);
                self.healthy();
                true
            }
            MembershipEffect::TdmaStart { slot, coordinator } => {
                let result = self.tdma.start(slot, coordinator, &mut self.timing).is_ok();
                self.healthy();
                result
            }
            MembershipEffect::ClockSource(local) => {
                self.tdma.set_clock_source(local);
                true
            }
            MembershipEffect::LogInitialized => bridge_rpc_mailbox::bridge_initialized(),
            MembershipEffect::Status(_)
            | MembershipEffect::Event { .. }
            | MembershipEffect::Log { .. } => {
                panic!("unexpected synchronous membership output")
            }
            _ => false,
        }
    }
    async fn membership_output_async(&mut self, output: MembershipOutput) -> bool {
        let request = match output {
            MembershipOutput::Status(status) => Ok(Request::Status(status)),
            MembershipOutput::Event {
                id,
                node: Some(node),
            } => Request::event(id, &[node]),
            MembershipOutput::Event { id, node: None } => Request::event(id, &[]),
            MembershipOutput::Log { bytes, len } => Request::log(&bytes[..usize::from(len)]),
        };
        self.call(request).await == 0
    }
}

impl AdaptivePlatform for Backend {
    fn tdma_rx_timestamp(&mut self, physical_stamp: i64) -> Option<i64> {
        self.timing.timing_at_physical(physical_stamp)
    }

    fn adaptive_effect(&mut self, effect: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        match effect {
            AdaptiveEffect::FrameCounter => {
                self.healthy();
                AdaptiveReply::Frame(self.tdma.frame_counter())
            }
            AdaptiveEffect::ClockSource(local) => {
                self.tdma.set_clock_source(local);
                AdaptiveReply::Applied
            }
            AdaptiveEffect::SetSlot(slot) => {
                self.tdma.set_slot_index(slot);
                AdaptiveReply::Applied
            }
            AdaptiveEffect::Sync {
                frame,
                drift,
                base_us,
            } => {
                let result = self.tdma.sync(frame, drift, base_us, &mut self.timing);
                self.healthy();
                AdaptiveReply::Accepted(if result.is_ok() { 0 } else { -5 })
            }
            AdaptiveEffect::Status(_) => panic!("unexpected synchronous adaptive output"),
            _ => AdaptiveReply::Accepted(ENOTSUP),
        }
    }
    async fn adaptive_status_async(&mut self, status: Membership) -> AdaptiveReply {
        AdaptiveReply::Accepted(self.call(Ok(Request::Status(status))).await)
    }
    fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
        let snapshot = self.tdma.clock_snapshot(&mut self.timing);
        #[cfg(feature = "sync-trace")]
        {
            let tdma = self.tdma.snapshot();
            let reading = self.timing.core_clock_reading();
            let timing_profile = self.timing.clock_mapping().profile();
            self.sync_trace
                .clock_snapshot(snapshot.as_ref().ok().and_then(|sample| {
                    let (frame, phase_us) = (*sample)?;
                    let reading = reading?;
                    if tdma.frame_start_us.checked_add(i64::from(phase_us))?
                        != reading.timing.value()
                    {
                        return None;
                    }
                    Some(ClockSample {
                        frame,
                        phase_us,
                        generation: tdma.generation,
                        frame_start_us: tdma.frame_start_us,
                        physical_sample_us: i64::try_from(reading.physical.value()).ok()?,
                        timing_profile,
                    })
                }));
        }
        let snapshot = snapshot.expect("TDMA clock snapshot");
        self.healthy();
        snapshot.map(|(frame, phase_us)| ClockSnapshot { frame, phase_us })
    }
    fn radio_send(&mut self, _: RadioSend<'_>) -> i32 {
        ENOTSUP
    }
    async fn radio_send_async(&mut self, packet: RadioSend<'_>) -> i32 {
        self.radio_send_classified_async(packet).await.code
    }
    async fn radio_send_classified_async(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        // Decide admission before trace parsing/counter publication in the experiment.
        // Preserve the captured snapshot and sample for rejection witnesses.
        #[cfg(feature = "admission-headroom")]
        let admission = self.tx_window.map(|window| {
            let snapshot = self.tdma.snapshot();
            let physical_now = self.now_us();
            let mapping = self.timing.clock_mapping();
            let evaluation =
                window.evaluate_for_clock(snapshot, physical_now, packet.bytes.len(), mapping);
            (snapshot, physical_now, mapping, evaluation)
        });
        #[cfg(feature = "sync-trace")]
        let class = classify(packet.bytes);
        #[cfg(feature = "sync-trace")]
        self.tx_diagnostics.record(class, Stage::Requests);
        #[cfg(feature = "sync-trace")]
        let mut trace = {
            // Same five-byte, zero-padded wire identity as the protocol adapter.
            let mut identity = openhelmet_wire::mesh::Identity {
                address_len: 5,
                address: [0; 6],
            };
            identity.address[..5].copy_from_slice(&self.radio.identity());
            let trace = self
                .sync_trace
                .send_boundary(
                    packet.bytes,
                    identity,
                    self.tdma.snapshot().generation,
                    None,
                )
                .ok()
                .flatten();
            crate::sync_trace_runtime::observe(None, self.sync_trace.counters());
            trace
        };
        let deadline = if let Some(window) = self.tx_window {
            #[cfg(not(feature = "admission-headroom"))]
            let snapshot = self.tdma.snapshot();
            #[cfg(not(feature = "admission-headroom"))]
            let physical_now = self.now_us();
            #[cfg(not(feature = "admission-headroom"))]
            let mapping = self.timing.clock_mapping();
            #[cfg(not(feature = "admission-headroom"))]
            let evaluation =
                window.evaluate_for_clock(snapshot, physical_now, packet.bytes.len(), mapping);
            #[cfg(feature = "admission-headroom")]
            let (snapshot, physical_now, mapping, evaluation) = admission.expect("captured window");
            #[cfg(all(feature = "admission-headroom", not(feature = "sync-trace")))]
            let _ = (snapshot, physical_now, mapping, window);
            let Ok(deadline) = evaluation.result else {
                #[cfg(feature = "sync-trace")]
                {
                    let witness = LastAdmissionRejection::from_evaluation(
                        window,
                        snapshot,
                        physical_now,
                        packet.bytes.len(),
                        mapping,
                        evaluation,
                    )
                    .expect("rejected admission evaluation");
                    self.tx_diagnostics
                        .record_backend_window_rejection(class, witness);
                    if let Some(record) = trace.as_mut() {
                        record.result = SendResult::BackendWindowRejected;
                    }
                    crate::sync_trace_runtime::observe(trace, self.sync_trace.counters());
                }
                return RadioSendResult {
                    code: -11,
                    disposition: SubmissionDisposition::NotSubmitted,
                };
            };
            Some(deadline)
        } else {
            None // Source JOIN/LEAVE outside TDMA retain the default deadline.
        };
        #[cfg(feature = "sync-trace")]
        {
            let rc = self
                .radio
                .send_classified(
                    packet,
                    deadline,
                    trace.as_mut(),
                    &mut self.tx_diagnostics,
                    class,
                )
                .await;
            crate::sync_trace_runtime::observe(trace, self.sync_trace.counters());
            rc
        }
        #[cfg(not(feature = "sync-trace"))]
        self.radio.send_classified(packet, deadline).await
    }
}

impl AudioPlatform for Backend {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        match effect {
            AudioEffect::InitWork | AudioEffect::CancelWork => {
                self.ingress_work = false;
                0
            }
            AudioEffect::SetSlotCallback => {
                self.tdma.set_slot_callback(true);
                0
            }
            AudioEffect::SubmitWork => {
                self.ingress_work = true;
                1
            }
            AudioEffect::NowUs => self.now_us(),
            AudioEffect::SlotRemainingUs => {
                let snapshot = self.tdma.snapshot();
                let physical_now = self.now_us();
                let remaining =
                    slot_remaining_physical_us(snapshot, physical_now, self.timing.clock_mapping());
                self.healthy();
                i64::from(audio_fit_remaining_us(remaining))
            }
            AudioEffect::TimeToSlotUs => {
                let remaining = self
                    .tdma
                    .time_to_slot_us(&mut self.timing)
                    .expect("TDMA time to slot");
                self.healthy();
                i64::from(remaining)
            }
            AudioEffect::BridgeDeliver { .. } => panic!("unexpected synchronous audio delivery"),
            AudioEffect::Starvation { .. } => 0, // diagnostic only, no delivery promise
        }
    }
    async fn audio_io_async(&mut self, effect: AudioEffect<'_>) -> i64 {
        if let AudioEffect::BridgeDeliver { source, bundle } = effect {
            i64::from(self.call(Request::audio_bundle(source, bundle)).await)
        } else {
            self.audio_io(effect)
        }
    }
}

impl RxPlatform for Backend {
    fn rx_io(&mut self, effect: RxEffect) -> i32 {
        match effect {
            RxEffect::InitWork | RxEffect::CancelWork => {
                self.rf_work = false;
                0
            }
            RxEffect::AttachCallback => 0, // actor dequeues facade, copies, then publishes
            RxEffect::SubmitWork => {
                self.rf_work = true;
                1
            }
        }
    }
}
