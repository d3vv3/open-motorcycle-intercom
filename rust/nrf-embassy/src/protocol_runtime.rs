//! Default-off composition of the ten lifecycle functions with the 98 adapters.
//! ActorOracle is serial simulation only. Physical radio/clock/TDMA/bridge IO
//! remains external, and callbacks may not publish reentrant commands.
use crate::platform::{accepts_protocol_algorithms, Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use crate::protocol_audio::{AudioEffect, AudioPlatform};
use crate::protocol_membership::{Input, MembershipEffect, MembershipPlatform, ProtocolState};
use crate::protocol_rx::{FullRxPlatform, RxEffect, RxPlatform};
use crate::protocol_tx::{ClockSnapshot, RadioSend};
use crate::runtime::{Runtime, ENOTSUP};

pub(crate) fn owned_effect(e: Effect) -> bool {
    matches!(
        e,
        Effect::BindMembershipWork
            | Effect::ScanTimeout
            | Effect::PeerCount(_)
            | Effect::AdaptiveReset
            | Effect::AdaptiveReport
            | Effect::CheckPeerTimeouts
            | Effect::CoordinatorTimeout
            | Effect::Leave(_)
            | Effect::ResetMembershipSession
            | Effect::OwnedMembershipTimer(_)
            | Effect::MembershipTimer(_)
            | Effect::AudioInit
            | Effect::AudioCancel
            | Effect::ResetRfTrackers
            | Effect::Ingress { .. }
            | Effect::PurgeTx
            | Effect::ClearGrants
            | Effect::ClearBitmaps
            | Effect::LocalAudio(_)
            | Effect::UpdateGrants
            | Effect::RxInit
            | Effect::RxStop
    )
}

pub struct FullProtocolPlatform<'a, P: RxPlatform>(pub &'a mut P);
impl<P: RxPlatform> Platform for FullProtocolPlatform<'_, P> {
    fn ingress_gate_changed(
        &mut self,
        gate: crate::protocol_audio::IngressAdmissionGate,
        purge: bool,
    ) {
        self.0.ingress_gate_changed(gate, purge);
    }
    fn prepare_ingress_boundary(&mut self, gate: crate::protocol_audio::IngressAdmissionGate) {
        self.0.prepare_ingress_boundary(gate);
    }
    fn tx_window_valid(&self) -> bool {
        self.0.tx_window_valid()
    }
    async fn dispatch_protocol_async(&mut self, r: &mut Runtime, e: Effect) -> Option<Reply> {
        if owned_effect(e)
            && !accepts_protocol_algorithms(self.integration(), self.owned_protocol_algorithms())
        {
            return Some(Reply::Value(ENOTSUP));
        }
        match e {
            Effect::Leave(address) => Some(Reply::Value(
                r.tx_send_packet_async(4, &address, self).await,
            )),
            Effect::OwnedMembershipTimer(_)
            | Effect::CheckPeerTimeouts
            | Effect::CoordinatorTimeout => {
                let input = match e {
                    Effect::OwnedMembershipTimer(token) => Input::Timer(token),
                    Effect::CheckPeerTimeouts => Input::CheckPeerTimeouts,
                    _ => Input::CoordinatorTimeout,
                };
                Some(match r.membership_handle_async(input, self).await {
                    Ok(lost) if e == Effect::CoordinatorTimeout => Reply::Value(i32::from(lost)),
                    Ok(_) => Reply::Value(0),
                    Err(_) => Reply::Value(ENOTSUP),
                })
            }
            _ => self.dispatch_protocol(r, e),
        }
    }
    fn integration(&self) -> Integration {
        self.0.integration()
    }
    fn now_us(&self) -> i64 {
        self.0.now_us()
    }
    async fn apply_async(&mut self, e: Effect) -> Reply {
        if owned_effect(e) {
            Reply::Value(ENOTSUP)
        } else {
            self.0.apply_async(e).await
        }
    }
    fn apply(&mut self, e: Effect) -> Reply {
        if owned_effect(e) {
            Reply::Value(ENOTSUP)
        } else {
            self.0.apply(e)
        }
    }
    fn owned_protocol_algorithms(&self) -> bool {
        true
    }
    fn dispatch_protocol(&mut self, r: &mut Runtime, e: Effect) -> Option<Reply> {
        if owned_effect(e)
            && !accepts_protocol_algorithms(self.integration(), self.owned_protocol_algorithms())
        {
            return Some(Reply::Value(ENOTSUP));
        }
        let input = match e {
            Effect::BindMembershipWork => Some(Input::BindWork),
            Effect::CheckPeerTimeouts => Some(Input::CheckPeerTimeouts),
            Effect::CoordinatorTimeout => Some(Input::CoordinatorTimeout),
            Effect::ResetMembershipSession => Some(Input::ResetSession),
            Effect::OwnedMembershipTimer(token) => Some(Input::Timer(token)),
            _ => None,
        };
        if let Some(input) = input {
            let result = r.membership_handle(input, self);
            return Some(match result {
                Ok(lost) if e == Effect::CoordinatorTimeout => Reply::Value(i32::from(lost)),
                Ok(_) => Reply::Value(0),
                Err(_) => Reply::Value(ENOTSUP),
            });
        }
        match e {
            Effect::ScanTimeout => Some(Reply::Value(r.membership_scan_timeout_ms() as i32)),
            Effect::PeerCount(_) => Some(Reply::Value(i32::from(r.membership_bridge_peer_count()))),
            Effect::AdaptiveReset => {
                r.adaptive_reset();
                Some(Reply::None)
            }
            Effect::AdaptiveReport => {
                r.adaptive_report_tick(self);
                Some(Reply::None)
            }
            Effect::Leave(address) => Some(Reply::Value(r.tx_send_packet(4, &address, self))),
            Effect::MembershipTimer(_) => Some(Reply::Value(ENOTSUP)),
            _ => {
                let mut rx = FullRxPlatform(self);
                rx.dispatch_rx(e, r).or_else(|| rx.dispatch_audio(e, r))
            }
        }
    }
}
impl<P: RxPlatform> MembershipPlatform for FullProtocolPlatform<'_, P> {
    async fn membership_output_async(
        &mut self,
        output: crate::protocol_membership::MembershipOutput,
    ) -> bool {
        self.0.membership_output_async(output).await
    }
    async fn dispatch_membership_async(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        FullRxPlatform(self).dispatch_membership_async(e, r).await
    }
    fn uptime_ms(&mut self) -> i64 {
        self.0.uptime_ms()
    }
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.0.membership_effect(e, s)
    }
    fn dispatch_membership(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        FullRxPlatform(self).dispatch_membership(e, r)
    }
}
impl<P: RxPlatform> AdaptivePlatform for FullProtocolPlatform<'_, P> {
    fn tdma_rx_timestamp(&mut self, physical_stamp: i64) -> Option<i64> {
        self.0.tdma_rx_timestamp(physical_stamp)
    }
    async fn adaptive_status_async(&mut self, status: crate::runtime::Membership) -> AdaptiveReply {
        self.0.adaptive_status_async(status).await
    }
    fn adaptive_effect(&mut self, e: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply {
        self.0.adaptive_effect(e, io)
    }
    fn owned_adaptive_wrappers(&self) -> bool {
        true
    }
    fn owned_tx_wrappers(&self) -> bool {
        true
    }
    fn owned_audio_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
    async fn audio_effect_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io_async(e).await
    }
    fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
        self.0.clock_snapshot()
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.0.radio_send(packet)
    }
    async fn radio_send_async(&mut self, packet: RadioSend<'_>) -> i32 {
        self.0.radio_send_async(packet).await
    }
    fn radio_send_classified(
        &mut self,
        packet: RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified(packet)
    }
    async fn radio_send_classified_async(
        &mut self,
        packet: RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified_async(packet).await
    }
}
impl<P: RxPlatform> AudioPlatform for FullProtocolPlatform<'_, P> {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
    async fn audio_io_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io_async(e).await
    }
}
impl<P: RxPlatform> RxPlatform for FullProtocolPlatform<'_, P> {
    fn rx_io(&mut self, e: RxEffect) -> i32 {
        self.0.rx_io(e)
    }
}
