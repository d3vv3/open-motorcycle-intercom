//! The twelve original mesh_protocol_tx.c functions on the existing owner.
//! Sync and awaited physical IO share one canonical source-ordered TX algorithm.
use openhelmet_mesh::{adaptive as a, core as c};
use openhelmet_wire::{audio_bundle, mesh as w};

use crate::platform::{accepts_protocol_algorithms, Effect, Integration, Platform, Reply};
use crate::protocol_adaptive::{
    AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply, CombinedMembershipPlatform,
};
use crate::protocol_membership::{MembershipEffect, MembershipPlatform, ProtocolState};
use crate::runtime::{Role, Runtime, State, EINVAL, ENOTSUP};

pub const EMSGSIZE: i32 = -90;
pub const ENOBUFS: i32 = -105;
pub const EAGAIN: i32 = -11;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockSnapshot {
    pub frame: u32,
    pub phase_us: u16,
}

/// The original esb_radio_send boundary: broadcast on physical pipe zero.
/// The JOIN_ACK target belongs to its payload, not to this radio address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioSend<'a> {
    pub bytes: &'a [u8],
    pub pipe: u8,
    pub address: [u8; 5],
}

/// Whether the radio operation was admitted, independently of its result code.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmissionDisposition {
    /// Rejected before the radio could start. Absence of a ticket is insufficient.
    NotSubmitted,
    /// Admitted by the radio; this does not imply successful completion.
    Accepted,
    /// The backend cannot establish whether the radio could have started.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioSendResult {
    pub code: i32,
    pub disposition: SubmissionDisposition,
}

/// Explicitly closes membership -> adaptive -> TX on one Runtime. Kernel
/// Effect::Leave and Effect::AdaptiveReport still require caller integration.
pub struct CombinedFullTxPlatform<'a, P: AdaptivePlatform>(pub &'a mut P);
impl<P: AdaptivePlatform> Platform for CombinedFullTxPlatform<'_, P> {
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
        self.0.dispatch_audio(e, r)
    }
}
impl<P: AdaptivePlatform> MembershipPlatform for CombinedFullTxPlatform<'_, P> {
    async fn membership_output_async(
        &mut self,
        output: crate::protocol_membership::MembershipOutput,
    ) -> bool {
        self.0.membership_output_async(output).await
    }
    async fn dispatch_membership_async(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        CombinedMembershipPlatform(self)
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
        CombinedMembershipPlatform(self).dispatch_membership(e, r)
    }
}
impl<P: AdaptivePlatform> AdaptivePlatform for CombinedFullTxPlatform<'_, P> {
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
        self.0.owned_audio_wrappers()
    }
    fn audio_effect(&mut self, e: crate::protocol_audio::AudioEffect<'_>) -> i64 {
        self.0.audio_effect(e)
    }
    async fn audio_effect_async(&mut self, e: crate::protocol_audio::AudioEffect<'_>) -> i64 {
        self.0.audio_effect_async(e).await
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
    fn radio_send_classified(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        self.0.radio_send_classified(packet)
    }
    async fn radio_send_classified_async(&mut self, packet: RadioSend<'_>) -> RadioSendResult {
        self.0.radio_send_classified_async(packet).await
    }
}

async fn radio_impl<const ASYNC_SEND: bool>(p: &mut impl AdaptivePlatform, bytes: &[u8]) -> i32 {
    crate::corefuture::radio_send::<ASYNC_SEND>(
        p,
        RadioSend {
            bytes,
            pipe: 0,
            address: [0xe7; 5],
        },
    )
    .await
}
fn header(t: u8, src: u8, seq: u8, ttl: u8, flags: u8, channel: u8, len: usize) -> w::Header {
    w::Header {
        version: w::VERSION,
        packet_type: t,
        src_id: src,
        seq,
        ttl,
        flags,
        talk_channel: channel,
        payload_len: len as u16,
    }
}
fn forward_rank(t: u8) -> u8 {
    match t {
        19 | 20 | 22 | 16 => 4,
        18 | 15 | 21 => 3,
        23 => 2,
        14 => 1,
        _ => 0,
    }
}
fn own_rank(t: u8) -> i8 {
    match t {
        22 | 6 => 3,
        23 | 9 | 10 => 2,
        _ => 0,
    }
}
fn wire_identity(x: a::Identity) -> w::Identity {
    w::Identity {
        address_len: x.address_len,
        address: x.address,
    }
}

impl Runtime {
    fn tx_next_seq(&mut self) -> u8 {
        let seq = self.protocol.tx_seq;
        self.protocol.tx_seq = seq.wrapping_add(1);
        seq
    }

    #[allow(clippy::too_many_arguments)] // Preserve the source entry-point arguments.
    pub fn tx_send_packet_ex(
        &mut self,
        t: u8,
        payload: &[u8],
        ttl: u8,
        flags: u8,
        src: u8,
        seq: u8,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        crate::corefuture::ready(
            self.tx_send_packet_ex_impl::<false>(t, payload, ttl, flags, src, seq, p),
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub async fn tx_send_packet_ex_async(
        &mut self,
        t: u8,
        payload: &[u8],
        ttl: u8,
        flags: u8,
        src: u8,
        seq: u8,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        self.tx_send_packet_ex_impl::<true>(t, payload, ttl, flags, src, seq, p)
            .await
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn tx_send_packet_ex_impl<const ASYNC_SEND: bool>(
        &mut self,
        t: u8,
        payload: &[u8],
        ttl: u8,
        flags: u8,
        src: u8,
        seq: u8,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        self.tx_send_packet_ex_classified_impl::<ASYNC_SEND>(t, payload, ttl, flags, src, seq, p)
            .await
            .code
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn tx_send_packet_ex_classified_impl<const ASYNC_SEND: bool>(
        &mut self,
        t: u8,
        payload: &[u8],
        ttl: u8,
        flags: u8,
        src: u8,
        seq: u8,
        p: &mut impl AdaptivePlatform,
    ) -> RadioSendResult {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return RadioSendResult {
                code: ENOTSUP,
                disposition: SubmissionDisposition::NotSubmitted,
            };
        }
        if t == 1 || (t == 13 && audio_bundle::audio_bundle_parse(payload).is_none()) {
            return RadioSendResult {
                code: EINVAL,
                disposition: SubmissionDisposition::NotSubmitted,
            };
        }
        if payload.len() > 200 {
            return RadioSendResult {
                code: EMSGSIZE,
                disposition: SubmissionDisposition::NotSubmitted,
            };
        }
        let mut raw = [0; 209];
        let h = header(t, src, seq, ttl, flags, self.talk_channel, payload.len());
        let len = w::encode(&h, payload, &mut raw, 200).expect("bounded TX envelope");
        crate::corefuture::radio_send_classified::<ASYNC_SEND>(
            p,
            RadioSend {
                bytes: &raw[..len],
                pipe: 0,
                address: [0xe7; 5],
            },
        )
        .await
    }

    pub fn tx_send_packet(&mut self, t: u8, payload: &[u8], p: &mut impl AdaptivePlatform) -> i32 {
        crate::corefuture::ready(self.tx_send_packet_impl::<false>(t, payload, p))
    }
    pub async fn tx_send_packet_async(
        &mut self,
        t: u8,
        payload: &[u8],
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        self.tx_send_packet_impl::<true>(t, payload, p).await
    }
    async fn tx_send_packet_impl<const ASYNC_SEND: bool>(
        &mut self,
        t: u8,
        payload: &[u8],
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return ENOTSUP;
        }
        let seq = self.tx_next_seq();
        self.tx_send_packet_ex_impl::<ASYNC_SEND>(t, payload, 0, 0, self.membership.node_id, seq, p)
            .await
    }

    pub fn tx_queue_control(&mut self, t: u8, payload: &[u8]) -> i32 {
        if t == 1 || t == 13 {
            return EINVAL;
        }
        if payload.len() > 200 {
            return EMSGSIZE;
        }
        if matches!(t, 14 | 22 | 23 | 9) {
            let mut i = self.session.control_tail;
            while i != self.session.control_head {
                let entry = &mut self.session.control_ring[usize::from(i)];
                if entry.data[1] == t {
                    let revision: [u8; 4] = entry.data[13..17].try_into().expect("revision");
                    entry.data[9..9 + payload.len()].copy_from_slice(payload);
                    if t == 22 {
                        entry.data[13..17].copy_from_slice(&revision);
                    }
                    entry.data[7..9].copy_from_slice(&(payload.len() as u16).to_le_bytes());
                    entry.len = (9 + payload.len()) as u8;
                    return 0;
                }
                i = (i + 1) % 32;
            }
        }
        let next = (self.session.control_head + 1) % 32;
        if next == self.session.control_tail {
            self.protocol.stat_control_ring_drop =
                self.protocol.stat_control_ring_drop.wrapping_add(1);
            return ENOBUFS;
        }
        let seq = self.tx_next_seq();
        let h = header(
            t,
            self.membership.node_id,
            seq,
            2,
            0,
            self.talk_channel,
            payload.len(),
        );
        let entry = &mut self.session.control_ring[usize::from(self.session.control_head)];
        entry.len = w::encode(&h, payload, &mut entry.data, 200).expect("bounded control") as u8;
        if t == 22 {
            // The revision lives in the full entry even when the wire payload
            // ends before it. Stamping must not increase the declared length.
            let revision = a::next_membership_revision(Some(&mut self.protocol.adaptive));
            entry.data[13..17].copy_from_slice(&revision.to_le_bytes());
        }
        self.session.control_head = next;
        0
    }

    pub fn tx_queue_priority(&mut self, t: u8, payload: &[u8]) -> i32 {
        if payload.len() > 200 || self.protocol.priority_pending {
            return ENOBUFS;
        }
        let seq = self.tx_next_seq();
        let h = header(
            t,
            self.membership.node_id,
            seq,
            2,
            0,
            self.talk_channel,
            payload.len(),
        );
        let entry = &mut self.protocol.priority_control;
        entry.len = w::encode(&h, payload, &mut entry.data, 200).expect("bounded priority") as u8;
        self.protocol.priority_pending = true;
        0
    }

    pub fn tx_send_join_request(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        crate::corefuture::ready(self.tx_send_join_request_impl::<false>(p))
    }
    pub async fn tx_send_join_request_async(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        self.tx_send_join_request_impl::<true>(p).await
    }
    async fn tx_send_join_request_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return ENOTSUP;
        }
        let join = w::JoinV3 {
            origin: wire_identity(self.local_identity()),
            capabilities: 2,
            target: wire_identity(self.protocol.leader_identity),
        };
        let mut payload = [0; w::JoinV3::SIZE];
        join.encode(&mut payload).expect("JOIN layout");
        let seq = self.tx_next_seq();
        self.tx_send_packet_ex_impl::<ASYNC_SEND>(15, &payload, 2, 0, 0, seq, p)
            .await
    }

    pub fn tx_send_keepalive(&mut self) -> i32 {
        let mut payload = [0; w::Keepalive::SIZE];
        w::Keepalive {
            battery_pct: 255,
            reserved: 0,
        }
        .encode(&mut payload)
        .expect("keepalive layout");
        self.tx_queue_control(8, &payload)
    }

    pub fn tx_slot_snapshot(&self) -> c::SlotMap {
        let mut map = c::SlotMap::default();
        for peer in &self.session.peers {
            if peer.active && peer.announced && (0..8).contains(&peer.slot_index) {
                map.slot_ids[peer.slot_index as usize] = peer.node_id;
                map.slot_count = map.slot_count.max(peer.slot_index as u8 + 1);
            }
        }
        if (0..8).contains(&self.membership.slot_index) {
            map.slot_ids[self.membership.slot_index as usize] = self.membership.node_id;
            map.slot_count = map.slot_count.max(self.membership.slot_index as u8 + 1);
        }
        for i in 0..2 {
            if self.session.active_speaker_ids[i] != 0 {
                let n = usize::from(map.active_speaker_count);
                map.active_speaker_ids[n] = self.session.active_speaker_ids[i];
                map.relay_masks[n] = self.session.relay_masks[i];
                map.active_speaker_count += 1;
            }
        }
        map
    }

    pub fn tx_send_slot_map(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        self.adaptive_publish_membership(p)
    }

    pub fn tx_send_status_packet(&mut self) -> i32 {
        let status = w::Status {
            battery_pct: 255,
            rssi_dbm: 127,
            peer_count: self.membership.peer_count,
            fw_version: w::VERSION,
            temperature_c: 127,
            heard_bitmap: self.session.heard_bitmap,
            relay_bitmap: self.session.relay_bitmap,
            active_speakers: self
                .session
                .active_speaker_ids
                .iter()
                .filter(|&&id| id != 0)
                .count() as u8,
        };
        let mut payload = [0; w::Status::SIZE];
        status.encode(&mut payload).expect("status layout");
        self.tx_queue_control(7, &payload)
    }

    pub fn tx_send_join_ack(&mut self, id: u8, slot: u8, address: [u8; 5]) -> i32 {
        let mut target = w::Identity {
            address_len: 5,
            address: [0; 6],
        };
        target.address[..5].copy_from_slice(&address);
        let ack = w::JoinAckV3 {
            target,
            capabilities: 2,
            assigned_id: id,
            slot_index: slot,
            coordinator_id: self.membership.node_id,
            term: self.protocol.term,
        };
        let mut payload = [0; w::JoinAckV3::SIZE];
        ack.encode(&mut payload).expect("JOIN_ACK layout");
        self.tx_queue_priority(16, &payload)
    }

    pub fn tx_send_sync(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        crate::corefuture::ready(self.tx_send_sync_impl::<false>(p))
    }
    pub async fn tx_send_sync_async(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        self.tx_send_sync_impl::<true>(p).await
    }
    async fn tx_send_sync_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return ENOTSUP;
        }
        let Some(clock) = p.clock_snapshot() else {
            return EAGAIN;
        };
        let mut sync = a::Sync {
            leader: self.local_identity(),
            leader_id: self.membership.node_id,
            term: self.protocol.term,
            member_count: self.protocol.adaptive.snapshot().members.count_ones() as u8,
            ..a::Sync::default()
        };
        if !a::stamp_sync(Some(&mut sync), false, clock.frame, clock.phase_us, 0) {
            return EINVAL;
        }
        let sync = w::SyncV3 {
            leader: wire_identity(sync.leader),
            leader_id: sync.leader_id,
            term: sync.term,
            member_count: sync.member_count,
            frame_counter: sync.frame_counter,
            phase_us: sync.phase_us,
            relay_depth: sync.relay_depth,
        };
        let mut payload = [0; w::SyncV3::SIZE];
        sync.encode(&mut payload).expect("SYNC layout");
        let seq = self.tx_next_seq();
        let ret = self
            .tx_send_packet_ex_impl::<ASYNC_SEND>(
                17,
                &payload,
                2,
                0,
                self.membership.node_id,
                seq,
                p,
            )
            .await;
        if ret == 0 {
            self.protocol.last_sync_time = p.uptime_ms() as u32;
        }
        ret
    }

    pub fn tx_control_handler(&mut self, frame: u32, p: &mut impl AdaptivePlatform) {
        crate::corefuture::ready(self.tx_control_handler_impl::<false>(frame, p))
    }
    pub async fn tx_control_handler_async(&mut self, frame: u32, p: &mut impl AdaptivePlatform) {
        self.tx_control_handler_impl::<true>(frame, p).await
    }
    async fn tx_control_handler_impl<const ASYNC_SEND: bool>(
        &mut self,
        frame: u32,
        p: &mut impl AdaptivePlatform,
    ) {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return;
        }
        self.adaptive_frame_tick_impl::<ASYNC_SEND>(frame, p).await;
        if ASYNC_SEND && !p.tx_window_valid() {
            return;
        }
        if frame.is_multiple_of(10) {
            if self.membership.role == Role::Coordinator {
                self.tx_send_sync_impl::<ASYNC_SEND>(p).await;
            }
            return;
        }
        if self.membership.state != State::Active
            || self.membership.slot_index < 0
            || frame % 8 != self.membership.slot_index as u32
        {
            return;
        }
        if self.adaptive_converging(p.uptime_ms() as u32) && (frame / 8).is_multiple_of(2) {
            self.tx_send_sync_impl::<ASYNC_SEND>(p).await;
            return;
        }
        if self.protocol.priority_pending {
            let entry = &self.protocol.priority_control;
            if radio_impl::<ASYNC_SEND>(p, &entry.data[..usize::from(entry.len)]).await == 0 {
                self.protocol.priority_pending = false;
            }
            return;
        }
        let forward = if self.protocol.adaptive.forward_queue_len() != 0 {
            let old = self.protocol.control_turn;
            self.protocol.control_turn = old.wrapping_add(1);
            old.is_multiple_of(2) || self.session.control_tail == self.session.control_head
        } else {
            false
        };
        if forward {
            let mut best = 0;
            let mut rank = 0;
            for i in 0..self.protocol.adaptive.forward_queue_len() {
                let r = forward_rank(
                    self.protocol
                        .adaptive
                        .forward_queue_entry(i)
                        .expect("queue entry")
                        .header
                        .packet_type,
                );
                if r > rank {
                    rank = r;
                    best = i;
                }
            }
            if best != 0 {
                self.protocol.adaptive.promote_forward_queue(best);
            }
            let Some(clock) = p.clock_snapshot() else {
                return;
            };
            let mut packet = a::ForwardPacket::default();
            let age = (p.uptime_ms() as u32).wrapping_sub(self.protocol.last_sync_time);
            if a::forward_dequeue(
                Some(&mut self.protocol.adaptive),
                Some(&mut packet),
                clock.frame,
                clock.phase_us,
                age,
            ) {
                let h = packet.header;
                let mut raw = [0; 209];
                let h = header(
                    h.packet_type,
                    h.src_id,
                    h.seq,
                    h.ttl,
                    h.flags,
                    h.talk_channel,
                    usize::from(h.payload_len),
                );
                let len = w::encode(
                    &h,
                    &packet.payload[..usize::from(packet.payload_len)],
                    &mut raw,
                    200,
                )
                .expect("forward envelope");
                if radio_impl::<ASYNC_SEND>(p, &raw[..len]).await != 0 {
                    self.protocol.stat_tx_fail = self.protocol.stat_tx_fail.wrapping_add(1);
                }
            }
            return;
        }
        if self.session.control_tail == self.session.control_head {
            return;
        }
        let tail = self.session.control_tail;
        let mut best = tail;
        let mut rank = -1;
        let mut i = tail;
        while i != self.session.control_head {
            let r = own_rank(self.session.control_ring[usize::from(i)].data[1]);
            if r > rank {
                rank = r;
                best = i;
            }
            i = (i + 1) % 32;
        }
        if best != tail {
            let selected = self.session.control_ring[usize::from(best)];
            let mut i = best;
            while i != tail {
                let previous = (i + 31) % 32;
                self.session.control_ring[usize::from(i)] =
                    self.session.control_ring[usize::from(previous)];
                i = previous;
            }
            self.session.control_ring[usize::from(tail)] = selected;
        }
        let entry = &mut self.session.control_ring[usize::from(tail)];
        if entry.data[1] == 14 {
            entry.data[9..13]
                .copy_from_slice(&self.protocol.air_report_seq.wrapping_add(1).to_le_bytes());
        } else if entry.data[1] == 23 {
            // This query is distinct from SYNC's coherent frame/phase snapshot.
            let Some(frame) = self.frame_counter(p) else {
                return;
            };
            self.session.control_ring[usize::from(tail)].data[17..21]
                .copy_from_slice(&frame.to_le_bytes());
        }
        let entry = &self.session.control_ring[usize::from(tail)];
        let ret = radio_impl::<ASYNC_SEND>(p, &entry.data[..usize::from(entry.len)]).await;
        if ret == 0 {
            if entry.data[1] == 14 {
                self.protocol.air_report_seq = self.protocol.air_report_seq.wrapping_add(1);
                self.protocol.stat_topology_tx_ok =
                    self.protocol.stat_topology_tx_ok.wrapping_add(1);
            }
            self.session.control_tail = (tail + 1) % 32;
        } else {
            self.protocol.stat_tx_fail = self.protocol.stat_tx_fail.wrapping_add(1);
            let seq = self.tx_next_seq();
            self.session.control_ring[usize::from(tail)].data[3] = seq;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct Backend {
        frames: Vec<Vec<u8>>,
        result: i32,
        clock: Option<ClockSnapshot>,
        now_frame: u32,
        now_ms: i64,
    }
    impl Default for Backend {
        fn default() -> Self {
            Self {
                frames: Vec::new(),
                result: 0,
                clock: Some(ClockSnapshot {
                    frame: 123,
                    phase_us: 456,
                }),
                now_frame: 99,
                now_ms: 1000,
            }
        }
    }
    impl Platform for Backend {
        fn integration(&self) -> Integration {
            Integration::ActorOracle
        }
        fn now_us(&self) -> i64 {
            self.now_ms * 1000
        }
        fn apply(&mut self, _: Effect) -> Reply {
            panic!("unexpected kernel boundary")
        }
    }
    impl MembershipPlatform for Backend {
        fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
            panic!("unexpected membership boundary")
        }
    }
    impl AdaptivePlatform for Backend {
        fn adaptive_effect(&mut self, e: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
            match e {
                AdaptiveEffect::FrameCounter => AdaptiveReply::Frame(self.now_frame),
                AdaptiveEffect::ExpireSpeakers(_) => AdaptiveReply::Applied,
                _ => panic!("TX must not execute the mock algorithm: {e:?}"),
            }
        }
        fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
            self.clock
        }
        fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
            assert_eq!(packet.pipe, 0);
            assert_eq!(packet.address, [0xe7; 5]);
            self.frames.push(packet.bytes.to_vec());
            self.result
        }
    }
    fn active() -> Runtime {
        let mut r = Runtime::default();
        r.membership.state = State::Active;
        r.membership.role = Role::Participant;
        r.membership.node_id = 1;
        r.membership.slot_index = 1;
        r.membership.coordinator_id = 2;
        r.address = [1, 2, 3, 4, 5];
        r
    }

    #[test]
    fn raw_envelope_validation_order_and_sequence_wrap() {
        let mut r = active();
        let mut p = Backend::default();
        r.protocol.tx_seq = 255;
        assert_eq!(r.tx_send_packet(1, &[0; 201], &mut p), EINVAL);
        assert_eq!(r.protocol.tx_seq, 0);
        assert_eq!(r.tx_send_packet(13, &[0; 201], &mut p), EINVAL);
        assert_eq!(r.tx_send_packet(255, &[0; 201], &mut p), EMSGSIZE);
        assert!(p.frames.is_empty());
        assert_eq!(
            r.tx_send_packet_ex(255, &[42; 200], 254, 253, 252, 251, &mut p),
            0
        );
        assert_eq!(&p.frames[0][..9], &[5, 255, 252, 251, 254, 253, 1, 200, 0]);
        assert_eq!(p.frames[0].len(), 209);
        p.result = -5;
        assert_eq!(r.tx_send_packet(2, &[9], &mut p), -5);
        assert_eq!(r.protocol.tx_seq, 3);
    }

    #[test]
    fn capacity_coalescing_revision_and_stale_storage() {
        let mut r = active();
        a::init(Some(&mut r.protocol.adaptive), 1, 1, 1, 1, 0);
        let mut member = [17; w::MembershipV3::SIZE];
        assert_eq!(r.tx_queue_control(22, &member), 0);
        assert_eq!(&r.session.control_ring[0].data[13..17], &1u32.to_le_bytes());
        for _ in 0..30 {
            assert_eq!(r.tx_queue_control(7, &[77; 8]), 0);
        }
        let seq = r.protocol.tx_seq;
        assert_eq!(r.tx_queue_control(8, &[255, 0]), ENOBUFS);
        assert_eq!(r.protocol.stat_control_ring_drop, 1);
        assert_eq!(r.protocol.tx_seq, seq);
        member.fill(88);
        assert_eq!(r.tx_queue_control(22, &member[..8]), 0);
        let entry = r.session.control_ring[0];
        assert_eq!(entry.len, 17);
        assert_eq!(&entry.data[9..13], &[88; 4]);
        assert_eq!(&entry.data[13..17], &1u32.to_le_bytes());
        assert_eq!(entry.data[17], 17);
        assert_eq!(entry.data[3], 0);
        assert_eq!(r.protocol.tx_seq, seq);
        assert_eq!(r.protocol.adaptive.snapshot().publish_revision, 1);
    }

    #[test]
    fn short_membership_revision_uses_full_entry_without_growing_wire_length() {
        for len in 0..=8 {
            let mut r = active();
            r.membership.role = Role::Coordinator;
            r.membership.coordinator_id = 1;
            r.protocol.term = 1;
            a::init(Some(&mut r.protocol.adaptive), 1, 1, 1, 1, 0);
            r.session.control_ring[0].data.fill(0xa5);
            r.session.control_ring[0].queued_ms = 77;
            let payload = [0x3c; 8];
            assert_eq!(r.tx_queue_control(22, &payload[..len]), 0);
            let entry = r.session.control_ring[0];
            assert_eq!(entry.len as usize, 9 + len);
            assert_eq!(&entry.data[13..17], &1u32.to_le_bytes());
            assert_eq!(entry.queued_ms, 77);
            assert_eq!(entry.data[17], 0xa5);
            assert_eq!(r.protocol.tx_seq, 1);
            for _ in 0..30 {
                assert_eq!(r.tx_queue_control(7, &[1; 8]), 0);
            }
            assert_eq!(r.tx_queue_control(22, &payload[..len]), 0);
            assert_eq!(r.session.control_ring[0], entry);
            assert_eq!(r.protocol.tx_seq, 31);
            assert_eq!(r.protocol.adaptive.snapshot().publish_revision, 1);
            assert_eq!(r.protocol.stat_control_ring_drop, 0);
        }
    }

    #[test]
    fn priority_keeps_bytes_and_sequence_on_failure_and_needs_no_clock() {
        let mut r = active();
        let mut p = Backend {
            result: -5,
            clock: None,
            ..Backend::default()
        };
        assert_eq!(r.tx_queue_priority(1, &[12]), 0);
        let entry = r.protocol.priority_control;
        assert_eq!(r.tx_queue_priority(16, &[0]), ENOBUFS);
        assert_eq!(r.tx_queue_priority(16, &[0; 201]), ENOBUFS);
        assert_eq!(r.protocol.tx_seq, 1);
        r.tx_control_handler(1, &mut p);
        assert_eq!(r.protocol.priority_control, entry);
        assert!(r.protocol.priority_pending);
        assert_eq!(r.protocol.tx_seq, 1);
        assert_eq!(r.protocol.stat_tx_fail, 0);
        p.result = 0;
        r.tx_control_handler(9, &mut p);
        assert!(!r.protocol.priority_pending);
        assert_eq!(p.frames[0], p.frames[1]);
    }

    #[test]
    fn topology_attempt_sequence_and_request_frame_are_actual_io_time() {
        let mut r = active();
        let mut p = Backend {
            result: -5,
            ..Backend::default()
        };
        r.protocol.air_report_seq = u32::MAX;
        r.protocol.tx_seq = 255;
        assert_eq!(r.tx_queue_control(14, &[55; w::Topology::SIZE]), 0);
        r.tx_control_handler(1, &mut p);
        assert_eq!(&p.frames[0][9..13], &0u32.to_le_bytes());
        assert_eq!(p.frames[0][3], 255);
        assert_eq!(r.session.control_ring[0].data[3], 0);
        assert_eq!(r.protocol.air_report_seq, u32::MAX);
        p.result = 0;
        r.tx_control_handler(9, &mut p);
        assert_eq!(&p.frames[1][9..13], &0u32.to_le_bytes());
        assert_eq!(r.protocol.air_report_seq, 0);
        assert_eq!(r.protocol.stat_topology_tx_ok, 1);
        assert_eq!(r.protocol.stat_tx_fail, 1);
        assert_eq!(r.tx_queue_control(23, &[0; w::SpeakerRequest::SIZE]), 0);
        r.tx_control_handler(17, &mut p);
        assert_eq!(&p.frames[2][17..21], &99u32.to_le_bytes());
    }

    #[test]
    fn sync_clock_precedes_sequence_and_modulo_ten_is_exclusive() {
        let mut r = active();
        r.membership.role = Role::Coordinator;
        r.membership.coordinator_id = r.membership.node_id;
        r.protocol.term = 1;
        let own = r.local_identity();
        r.protocol.leader_identity = own;
        a::init(Some(&mut r.protocol.adaptive), 1, 1, 1, 1, 0);
        assert!(a::set_member(Some(&mut r.protocol.adaptive), 1, Some(&own)));
        let mut p = Backend {
            clock: None,
            ..Backend::default()
        };
        assert_eq!(r.tx_send_sync(&mut p), EAGAIN);
        assert_eq!(r.protocol.tx_seq, 0);
        p.clock = Some(ClockSnapshot {
            frame: 3,
            phase_us: 20000,
        });
        assert_eq!(r.tx_send_sync(&mut p), EINVAL);
        assert_eq!(r.protocol.tx_seq, 0);
        p.clock = Some(ClockSnapshot {
            frame: 123,
            phase_us: 456,
        });
        r.membership.slot_index = 2;
        assert_eq!(r.tx_queue_priority(16, &[42]), 0);
        r.tx_control_handler(10, &mut p);
        assert!(r.protocol.priority_pending);
        let sync = w::SyncV3::decode(&p.frames[0][9..]).unwrap();
        assert_eq!(sync.frame_counter, 123);
        assert_eq!(sync.phase_us, 456);
        assert_eq!(sync.leader, wire_identity(own));
        assert_eq!((sync.leader_id, sync.term, sync.member_count), (1, 1, 1));
        assert_eq!(r.protocol.tx_seq, 2);
        assert_eq!(r.protocol.last_sync_time, 1000);
        p.result = -5;
        p.now_ms = 2000;
        r.tx_control_handler(20, &mut p);
        assert_eq!(r.protocol.last_sync_time, 1000);
        assert_eq!(r.protocol.tx_seq, 3);
        assert!(r.protocol.priority_pending);
        p.clock = None;
        r.tx_control_handler(30, &mut p);
        assert_eq!(r.protocol.tx_seq, 3);
        assert_eq!(p.frames.len(), 2);
        assert_eq!(r.protocol.last_sync_time, 1000);
        assert!(r.protocol.priority_pending);
    }

    #[test]
    fn full_wrapper_closes_nested_join_without_mock_tx_and_maps_slots() {
        let mut r = active();
        let mut backend = Backend::default();
        assert!(!backend.owned_tx_wrappers());
        let mut p = CombinedFullTxPlatform(&mut backend);
        assert!(p.dispatch_membership(MembershipEffect::JoinRequest, &mut r));
        let h = w::decode(&p.0.frames[0], 200).unwrap().header;
        assert_eq!((h.packet_type, h.src_id, h.ttl), (15, 0, 2));
        assert!(p.dispatch_membership(
            MembershipEffect::JoinAck {
                id: 3,
                slot: 4,
                address: [8; 5]
            },
            &mut r
        ));
        let ack =
            w::JoinAckV3::decode(&r.protocol.priority_control.data[9..9 + w::JoinAckV3::SIZE])
                .unwrap();
        assert_eq!(ack.target.address, [8, 8, 8, 8, 8, 0]);
        assert_eq!(ack.capabilities, 2);
        r.session.peers[0].active = true;
        r.session.peers[0].announced = true;
        r.session.peers[0].node_id = 2;
        r.session.peers[0].slot_index = 7;
        r.session.active_speaker_ids = [0, 2];
        r.session.relay_masks = [9, 7];
        let map = r.tx_slot_snapshot();
        assert_eq!(map.slot_count, 8);
        assert_eq!(map.slot_ids[1], 1);
        assert_eq!(map.slot_ids[7], 2);
        assert_eq!(map.active_speaker_ids, [2, 0]);
        assert_eq!(map.relay_masks, [7, 0]);
    }

    fn enqueue(r: &mut Runtime, t: u8, seq: u8, payload: &[u8]) {
        let h = a::Header {
            version: 5,
            packet_type: t,
            src_id: 3,
            seq,
            ttl: 2,
            flags: 0,
            talk_channel: 1,
            payload_len: payload.len() as u16,
        };
        assert!(a::forward_enqueue(
            Some(&mut r.protocol.adaptive),
            Some(&h),
            Some(payload),
            None,
            1,
            true,
            0
        ));
    }

    #[test]
    fn forward_promotion_is_stable_across_head_wrap_before_clock_and_fairness_wrap() {
        let mut r = active();
        a::init(Some(&mut r.protocol.adaptive), 1, 2, 1, 7, 0);
        // Move the actual shared head to seven using its public operations.
        for seq in 0..7 {
            enqueue(&mut r, 14, seq, &[0; w::Topology::SIZE]);
            assert!(a::forward_dequeue(
                Some(&mut r.protocol.adaptive),
                Some(&mut a::ForwardPacket::default()),
                0,
                0,
                0
            ));
        }
        enqueue(&mut r, 14, 10, &[0; w::Topology::SIZE]);
        enqueue(&mut r, 19, 11, &[0; w::HandoverAck::SIZE]);
        enqueue(&mut r, 19, 12, &[0; w::HandoverAck::SIZE]);
        let cache = r.protocol.adaptive.snapshot().cache;
        r.tx_send_keepalive();
        r.protocol.control_turn = 255;
        let mut p = Backend {
            clock: None,
            ..Backend::default()
        };
        r.tx_control_handler(1, &mut p);
        assert_eq!(r.protocol.control_turn, 0);
        assert_eq!(p.frames[0][1], 8);
        r.tx_control_handler(9, &mut p);
        assert_eq!(r.protocol.control_turn, 1);
        let queue = &r.protocol.adaptive;
        assert_eq!(queue.forward_queue_len(), 3);
        assert_eq!(queue.forward_queue_entry(0).unwrap().header.seq, 11);
        assert_eq!(queue.forward_queue_entry(1).unwrap().header.seq, 10);
        assert_eq!(queue.forward_queue_entry(2).unwrap().header.seq, 12);
        assert_eq!(queue.snapshot().cache, cache);
        p.clock = Some(ClockSnapshot {
            frame: 8,
            phase_us: 10,
        });
        p.result = -5;
        r.tx_control_handler(17, &mut p);
        assert_eq!(r.protocol.adaptive.forward_queue_len(), 2);
        assert_eq!(r.protocol.stat_tx_fail, 1);
        assert_eq!(p.frames[1][3], 11);
        assert_eq!(p.frames[1][4], 1);
        assert_eq!(p.frames[1][5], 2);
    }

    #[test]
    fn stale_forward_sync_consumes_queue_without_rf_and_preserves_cache() {
        let mut r = active();
        r.membership.node_id = 2;
        r.membership.coordinator_id = 1;
        r.address = [2, 2, 3, 4, 5];
        r.protocol.term = 1;
        let leader = a::Identity {
            address_len: 5,
            address: [1, 2, 3, 4, 5, 0],
        };
        r.protocol.leader_identity = leader;
        let own = r.local_identity();
        let mut p = Backend {
            now_ms: 1000,
            ..Backend::default()
        };
        r.protocol.last_sync_time = p.now_ms as u32;
        a::init(Some(&mut r.protocol.adaptive), 2, 1, 1, 3, p.now_ms as u32);
        assert!(a::set_member(
            Some(&mut r.protocol.adaptive),
            1,
            Some(&leader)
        ));
        assert!(a::set_member(Some(&mut r.protocol.adaptive), 2, Some(&own)));
        let sync = w::SyncV3 {
            leader: wire_identity(leader),
            leader_id: 1,
            member_count: 2,
            term: 1,
            frame_counter: 1,
            phase_us: 0,
            relay_depth: 0,
        };
        let mut payload = [0; w::SyncV3::SIZE];
        sync.encode(&mut payload).unwrap();
        let h = a::Header {
            version: 5,
            packet_type: 17,
            src_id: 1,
            seq: 7,
            ttl: 2,
            flags: 0,
            talk_channel: 1,
            payload_len: payload.len() as u16,
        };
        assert!(a::forward_enqueue(
            Some(&mut r.protocol.adaptive),
            Some(&h),
            Some(&payload),
            Some(&leader),
            1,
            true,
            p.now_ms as u32
        ));
        let before = r.protocol.adaptive.snapshot();
        assert_eq!(before.queue_count, 1);
        p.now_ms += i64::from(a::SYNC_MAX_AGE_MS) + 1;
        r.tx_control_handler(1, &mut p);
        let after = r.protocol.adaptive.snapshot();
        assert_eq!(after.queue_count, 0);
        assert_eq!(after.queue_head, (before.queue_head + 1) % 8);
        assert_eq!(after.queue, before.queue);
        assert_eq!(after.cache, before.cache);
        assert_eq!(r.protocol.control_turn, 1);
        assert_eq!(r.protocol.stat_tx_fail, 0);
        assert!(p.frames.is_empty());
    }

    #[test]
    fn own_rank_promotion_keeps_equal_rank_fifo_across_ring_wrap() {
        let mut r = active();
        r.session.control_head = 30;
        r.session.control_tail = 30;
        assert_eq!(r.tx_send_keepalive(), 0);
        assert_eq!(r.tx_queue_control(9, &[1; w::SpeakerGrant::SIZE]), 0);
        assert_eq!(r.tx_queue_control(10, &[2; w::SpeakerRelease::SIZE]), 0);
        assert_eq!(r.tx_queue_control(6, &[3; w::SlotMap::SIZE]), 0);
        let mut p = Backend::default();
        for frame in [1, 9, 17, 25] {
            r.tx_control_handler(frame, &mut p);
        }
        assert_eq!(
            p.frames.iter().map(|x| x[1]).collect::<Vec<_>>(),
            [6, 9, 10, 8]
        );
        assert_eq!(
            p.frames.iter().map(|x| x[3]).collect::<Vec<_>>(),
            [3, 1, 2, 0]
        );
        assert_eq!(r.session.control_tail, 2);
        assert_eq!(r.protocol.control_turn, 0);
    }
}
