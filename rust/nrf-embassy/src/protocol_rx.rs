//! All five unchanged RX functions, serialized on the existing Runtime owner.
//! Receipt time is supplied by the producer before publication, in the audio
//! cutoff clock domain. No IRQ may borrow Runtime mutably; hardware binding,
//! concurrency and latency remain unqualified.
use crate::platform::{accepts_protocol_algorithms, Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use crate::protocol_audio::{AudioEffect, AudioPlatform, FullAudioPlatform};
use crate::protocol_membership::{
    Error, Input, MembershipEffect, MembershipPlatform, ProtocolState,
};
use crate::protocol_tx::{ClockSnapshot, RadioSend};
use crate::runtime::Runtime;
use openhelmet_wire::mesh::{self as w, Header};

pub const RX_RING_SIZE: usize = 8;

/// Immutable owned RF message for a future bounded hardware task queue.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfRecord {
    pub data: [u8; 209],
    pub len: u8,
    pub rssi: i8,
    pub received_us: i64,
}
impl Default for RfRecord {
    fn default() -> Self {
        Self {
            data: [0; 209],
            len: 0,
            rssi: 0,
            received_us: 0,
        }
    }
}
#[derive(Default, PartialEq, Eq)]
pub struct RxState {
    pub(crate) slots: [RfRecord; RX_RING_SIZE],
    pub(crate) head: u8,
    pub(crate) tail: u8,
    pub(crate) work_pending: bool,
    pub(crate) count: u32,
    // C atomic_t: signed 32-bit storage, wrapping increment.
    pub(crate) drop: i32,
}
impl RxState {
    pub fn storage(&self) -> &[RfRecord; RX_RING_SIZE] {
        &self.slots
    }
    pub fn cursors(&self) -> (u8, u8) {
        (self.head, self.tail)
    }
    pub fn work_pending(&self) -> bool {
        self.work_pending
    }
    pub fn count(&self) -> u32 {
        self.count
    }
    pub fn drop_count(&self) -> i32 {
        self.drop
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxEffect {
    InitWork,
    AttachCallback,
    CancelWork,
    SubmitWork,
}
pub trait RxPlatform: AudioPlatform {
    /// Serialized kernel shim result. Positive means newly queued; zero or
    /// negative never removes an already pending item. C ignores this result.
    fn rx_io(&mut self, effect: RxEffect) -> i32;
}

/// Explicit opt-in: actual audio/TX/adaptive/membership on the same owner.
pub struct FullRxPlatform<'a, P: RxPlatform>(pub &'a mut P);
impl<P: RxPlatform> Platform for FullRxPlatform<'_, P> {
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
    fn owned_protocol_algorithms(&self) -> bool {
        self.0.owned_protocol_algorithms()
    }
    fn integration(&self) -> Integration {
        self.0.integration()
    }
    fn now_us(&self) -> i64 {
        self.0.now_us()
    }
    async fn apply_async(&mut self, e: Effect) -> Reply {
        self.0.apply_async(e).await
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.0.apply(e)
    }
    fn dispatch_audio(&mut self, e: Effect, r: &mut Runtime) -> Option<Reply> {
        FullAudioPlatform(self.0).dispatch_audio(e, r)
    }
    fn dispatch_rx(&mut self, e: Effect, r: &mut Runtime) -> Option<Reply> {
        match e {
            Effect::RxInit => r.rx_init(self),
            Effect::RxStop => r.rx_stop(self),
            _ => return None,
        }
        Some(Reply::None)
    }
}
impl<P: RxPlatform> MembershipPlatform for FullRxPlatform<'_, P> {
    async fn membership_output_async(
        &mut self,
        output: crate::protocol_membership::MembershipOutput,
    ) -> bool {
        self.0.membership_output_async(output).await
    }
    async fn dispatch_membership_async(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        FullAudioPlatform(self.0)
            .dispatch_membership_async(e, r)
            .await
    }
    fn uptime_ms(&mut self) -> i64 {
        self.0.uptime_ms()
    }
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.0.membership_effect(e, s)
    }
    fn dispatch_membership(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        FullAudioPlatform(self.0).dispatch_membership(e, r)
    }
}
impl<P: RxPlatform> AdaptivePlatform for FullRxPlatform<'_, P> {
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
impl<P: RxPlatform> AudioPlatform for FullRxPlatform<'_, P> {
    fn audio_io(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
    async fn audio_io_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io_async(e).await
    }
}
impl<P: RxPlatform> RxPlatform for FullRxPlatform<'_, P> {
    fn rx_io(&mut self, e: RxEffect) -> i32 {
        self.0.rx_io(e)
    }
}

impl Runtime {
    pub fn rx_state(&self) -> &RxState {
        &self.rx_state
    }
    pub fn rx_init(&mut self, p: &mut impl RxPlatform) {
        self.rx_state.work_pending = false;
        p.rx_io(RxEffect::InitWork);
        p.rx_io(RxEffect::AttachCallback);
    }
    pub fn rx_stop(&mut self, p: &mut impl RxPlatform) {
        p.rx_io(RxEffect::CancelWork);
        self.rx_state.work_pending = false;
        self.rx_state.head = 0;
        self.rx_state.tail = 0;
    }
    /// Serialized source callback equivalent; source address is unused in C.
    /// Oversized buffers are outside the original callback's valid backing domain.
    pub fn rx_publish(
        &mut self,
        payload: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl RxPlatform,
    ) -> Result<bool, Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return Err(Error::NotIntegrated);
        }
        if payload.len() > 209 {
            return Err(Error::ShortPayload);
        }
        let next = (self.rx_state.head + 1) % 8;
        if next == self.rx_state.tail {
            self.rx_state.drop = self.rx_state.drop.wrapping_add(1);
            return Ok(false);
        }
        let entry = &mut self.rx_state.slots[usize::from(self.rx_state.head)];
        entry.data[..payload.len()].copy_from_slice(payload);
        entry.len = payload.len() as u8;
        entry.rssi = rssi;
        entry.received_us = received_us;
        self.rx_state.head = next;
        if p.rx_io(RxEffect::SubmitWork) > 0 {
            self.rx_state.work_pending = true;
        }
        Ok(true)
    }
    pub fn rx_work_handler(&mut self, p: &mut impl RxPlatform) -> Result<(), Error> {
        crate::corefuture::ready(self.rx_work_handler_impl::<false>(p))
    }
    /// Pops each record before processing; cancellation does not requeue it.
    pub async fn rx_work_handler_async(&mut self, p: &mut impl RxPlatform) -> Result<(), Error> {
        self.rx_work_handler_impl::<true>(p).await
    }
    async fn rx_work_handler_impl<const ASYNC_DELIVER: bool>(
        &mut self,
        p: &mut impl RxPlatform,
    ) -> Result<(), Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return Err(Error::NotIntegrated);
        }
        self.rx_state.work_pending = false;
        while self.rx_state.tail != self.rx_state.head {
            let entry = self.rx_state.slots[usize::from(self.rx_state.tail)];
            self.rx_state.tail = (self.rx_state.tail + 1) % 8;
            self.rx_process_packet_impl::<ASYNC_DELIVER>(
                &entry.data[..usize::from(entry.len)],
                entry.rssi,
                entry.received_us,
                p,
            )
            .await?;
        }
        Ok(())
    }
    pub fn rx_process_packet(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl RxPlatform,
    ) -> Result<(), Error> {
        crate::corefuture::ready(self.rx_process_packet_impl::<false>(data, rssi, received_us, p))
    }
    /// Preserves adaptive/audio/membership/grant dispatch order, awaiting owned
    /// Bridge outputs in the actual adaptive, audio and membership algorithms.
    pub async fn rx_process_packet_async(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl RxPlatform,
    ) -> Result<(), Error> {
        self.rx_process_packet_impl::<true>(data, rssi, received_us, p)
            .await
    }
    async fn rx_process_packet_impl<const ASYNC_DELIVER: bool>(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl RxPlatform,
    ) -> Result<(), Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return Err(Error::NotIntegrated);
        }
        let counters = &mut self.audio_state.counters;
        if data.len() < 9
            || usize::from(u16::from_le_bytes([data[7], data[8]])) != data.len() - 9
            || data.len() - 9 > 200
        {
            counters.rf_rx_malformed = counters.rf_rx_malformed.wrapping_add(1);
            return Ok(());
        }
        if data[0] != w::VERSION || !(1..=3).contains(&data[6]) || data[6] != self.talk_channel {
            counters.rf_rx_version_drop = counters.rf_rx_version_drop.wrapping_add(1);
            return Ok(());
        }
        let h = Header {
            version: data[0],
            packet_type: data[1],
            src_id: data[2],
            seq: data[3],
            ttl: data[4],
            flags: data[5],
            talk_channel: data[6],
            payload_len: u16::from_le_bytes([data[7], data[8]]),
        };
        let payload = &data[9..];
        self.rx_state.count = self.rx_state.count.wrapping_add(1);
        if self
            .adaptive_control_rx_impl::<ASYNC_DELIVER>(h, payload, rssi, received_us, p)
            .await?
        {
            return Ok(());
        }
        if matches!(h.packet_type, 1 | 13)
            && self
                .audio_process_rx_packet_impl::<ASYNC_DELIVER>(data, rssi, received_us, p)
                .await?
        {
            return Ok(());
        }
        if self
            .membership_handle_impl::<ASYNC_DELIVER>(Input::Packet { header: h, payload }, p)
            .await?
        {
            return Ok(());
        }
        if h.src_id == self.membership.coordinator_id {
            match h.packet_type {
                9 if payload.len() == 5 => self.audio_apply_speaker_grant(
                    payload[0],
                    [payload[1], payload[2]],
                    [payload[3], payload[4]],
                ),
                10 if payload.len() == 3 => {
                    self.audio_apply_speaker_release(payload[0], [payload[1], payload[2]])
                }
                _ => {}
            }
        }
        Ok(())
    }
}
