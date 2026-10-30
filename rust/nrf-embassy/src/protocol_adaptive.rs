//! Single-owner adapter of unchanged `mesh_protocol_adaptive.c` (19 functions).
//! TX/audio/TDMA/bridge are synchronous stateful IO boundaries, not algorithm ports.
//! Zephyr diagnostic strings are excluded; membership UART logs retain their
//! existing 127-byte content limit. Payload blobs are layouts, not RF frame proof.
use openhelmet_mesh::{adaptive as a, core as c, membership as m};
use openhelmet_wire::mesh::{self as w, Header, Message};

use crate::platform::{accepts_protocol_algorithms, Effect, Integration, Platform, Reply};
use crate::protocol_membership::{
    Error, Input, MembershipEffect, MembershipPlatform, ProtocolState,
};
use crate::runtime::{Membership, RelayEntry, Role, Runtime, State};

/// Exact owner storage available to external TX/audio models. No snapshot is
/// restored after a call: mutations affect the next statement immediately.
pub struct AdaptiveIo<'a> {
    pub membership: Membership,
    pub address: [u8; 5],
    pub talk_channel: u8,
    pub leader_identity: a::Identity,
    pub term: u32,
    pub peers: [SlotPeer; 8],
    pub adaptive: &'a mut a::Adaptive,
    pub speakers: &'a mut [u8; 2],
    pub masks: &'a mut [u8; 2],
    pub deadlines_ms: &'a mut [i64; 9],
    pub active_since_ms: &'a mut [i64; 9],
    pub control_ring: &'a mut [RelayEntry; 32],
    pub control_head: &'a mut u8,
    pub control_tail: &'a mut u8,
    pub priority_control: &'a mut RelayEntry,
    pub priority_pending: &'a mut bool,
    pub tx_seq: &'a mut u8,
    pub control_turn: &'a mut u8,
    pub control_drops: &'a mut u32,
    pub tx_fail: &'a mut u32,
    pub air_report_seq: &'a mut u32,
    pub topology_tx_ok: &'a mut u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotPeer {
    pub id: u8,
    pub slot: i8,
    pub active: bool,
    pub announced: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdaptiveEffect {
    Queue {
        priority: bool,
        packet_type: u8,
        payload: [u8; 79],
        len: u16,
    },
    SlotSnapshot,
    JoinRequest,
    JoinAck {
        id: u8,
        slot: u8,
        address: [u8; 5],
    },
    FrameCounter,
    ResetTracker(u8),
    ApplySlotMap(c::SlotMap),
    ApplyGrant {
        count: u8,
        ids: [u8; 2],
        masks: [u8; 2],
    },
    ApplyRelease {
        count: u8,
        ids: [u8; 2],
    },
    UpdateGrants,
    ExpireSpeakers(u32),
    ClockSource(bool),
    SetSlot(i8),
    Sync {
        frame: u32,
        drift: i16,
        /// TDMA base in supplied `Clock::now_us`/history units, shared with wire
        /// SYNC `phase_us` and the fixed 300 compensation; never raw RTC ticks.
        base_us: i64,
    },
    Status(Membership),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdaptiveReply {
    Applied,
    /// C enqueue result. Success means the backend applied its specified TX
    /// mutation (including coalescing/revision policy), not that RF sent a frame.
    Accepted(i32),
    SlotMap(c::SlotMap),
    Frame(u32),
}

#[allow(async_fn_in_trait)]
pub trait AdaptivePlatform: MembershipPlatform {
    /// Converts physical RX microseconds to the supplied TDMA `Clock::now_us`/
    /// history units shared by wire SYNC `phase_us` and fixed 300 compensation,
    /// not raw RTC ticks. Physical wall time and receipt freshness stay in us.
    /// `None` means the conversion is unavailable or the timestamp is rejected.
    fn tdma_rx_timestamp(&mut self, physical_stamp: i64) -> Option<i64> {
        Some(physical_stamp)
    }
    fn adaptive_effect(&mut self, effect: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply;
    /// Legacy status IO observes only the owned membership snapshot. Temporary
    /// compatibility storage is synchronous and is never lent to an await.
    async fn adaptive_status_async(&mut self, status: Membership) -> AdaptiveReply
    where
        Self: Sized,
    {
        Runtime::legacy_adaptive_status(status, self)
    }
    fn owned_adaptive_wrappers(&self) -> bool {
        false
    }
    fn owned_tx_wrappers(&self) -> bool {
        false
    }
    fn owned_audio_wrappers(&self) -> bool {
        false
    }
    fn audio_effect(&mut self, _effect: crate::protocol_audio::AudioEffect<'_>) -> i64 {
        i64::from(crate::runtime::ENOTSUP)
    }
    async fn audio_effect_async(&mut self, effect: crate::protocol_audio::AudioEffect<'_>) -> i64 {
        self.audio_effect(effect)
    }
    fn clock_snapshot(&mut self) -> Option<crate::protocol_tx::ClockSnapshot> {
        None
    }
    fn radio_send(&mut self, _packet: crate::protocol_tx::RadioSend<'_>) -> i32 {
        crate::runtime::ENOTSUP
    }
    async fn radio_send_async(&mut self, packet: crate::protocol_tx::RadioSend<'_>) -> i32 {
        self.radio_send(packet)
    }
    /// Legacy codes alone cannot establish whether the radio could start.
    fn radio_send_classified(
        &mut self,
        packet: crate::protocol_tx::RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        crate::protocol_tx::RadioSendResult {
            code: self.radio_send(packet),
            disposition: crate::protocol_tx::SubmissionDisposition::Unknown,
        }
    }
    /// Await the legacy async override, preserving its suspension and cancellation.
    async fn radio_send_classified_async(
        &mut self,
        packet: crate::protocol_tx::RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        crate::protocol_tx::RadioSendResult {
            code: self.radio_send_async(packet).await,
            disposition: crate::protocol_tx::SubmissionDisposition::Unknown,
        }
    }
}

/// Explicit opt-in. The old MembershipPlatform remains a boundary-fixture API;
/// this wrapper closes only the adaptive calls owned by this adapter.
pub struct CombinedMembershipPlatform<'a, P: AdaptivePlatform>(pub &'a mut P);
impl<P: AdaptivePlatform> Platform for CombinedMembershipPlatform<'_, P> {
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
impl<P: AdaptivePlatform> MembershipPlatform for CombinedMembershipPlatform<'_, P> {
    async fn membership_output_async(
        &mut self,
        output: crate::protocol_membership::MembershipOutput,
    ) -> bool {
        self.0.membership_output_async(output).await
    }
    async fn dispatch_membership_async(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        if e == MembershipEffect::JoinRequest && self.owned_tx_wrappers() {
            r.tx_send_join_request_async(self).await == 0
        } else {
            self.dispatch_membership(e, r)
        }
    }
    fn uptime_ms(&mut self) -> i64 {
        self.0.uptime_ms()
    }
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.0.membership_effect(e, s)
    }
    fn dispatch_membership(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        if self.owned_audio_wrappers() && r.audio_membership_effect(e, self) {
            return true;
        }
        match e {
            MembershipEffect::AdaptiveReset => {
                r.adaptive_reset();
                true
            }
            MembershipEffect::BindMember { id, identity } => {
                r.adaptive_bind_member(id, identity.as_ref(), self)
            }
            MembershipEffect::ControlSequence {
                id,
                packet_type,
                seq,
            } => r.adaptive_control_sequence_accept(id, packet_type, seq),
            MembershipEffect::AdaptiveMembers { .. } => {
                r.adaptive_members_owned(self);
                true
            }
            MembershipEffect::SlotMap => r.adaptive_publish_membership(self) == 0,
            MembershipEffect::JoinRequest => matches!(
                r.ae(self, AdaptiveEffect::JoinRequest),
                AdaptiveReply::Accepted(0)
            ),
            MembershipEffect::JoinAck { id, slot, address } => matches!(
                r.ae(self, AdaptiveEffect::JoinAck { id, slot, address }),
                AdaptiveReply::Accepted(0)
            ),
            MembershipEffect::UpdateGrants => {
                r.ae(self, AdaptiveEffect::UpdateGrants);
                true
            }
            _ => {
                // Known clearing effects must not leave stale grant aliases.
                if e == MembershipEffect::ClearGrants {
                    r.session.active_speaker_ids = [0; 2];
                    r.session.relay_masks = [0; 2];
                }
                self.0.membership_effect(e, &mut r.protocol)
            }
        }
    }
}
impl<P: AdaptivePlatform> AdaptivePlatform for CombinedMembershipPlatform<'_, P> {
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
        self.0.owned_tx_wrappers()
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
    fn clock_snapshot(&mut self) -> Option<crate::protocol_tx::ClockSnapshot> {
        self.0.clock_snapshot()
    }
    fn radio_send(&mut self, packet: crate::protocol_tx::RadioSend<'_>) -> i32 {
        self.0.radio_send(packet)
    }
    async fn radio_send_async(&mut self, packet: crate::protocol_tx::RadioSend<'_>) -> i32 {
        self.0.radio_send_async(packet).await
    }
    fn radio_send_classified(
        &mut self,
        packet: crate::protocol_tx::RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified(packet)
    }
    async fn radio_send_classified_async(
        &mut self,
        packet: crate::protocol_tx::RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified_async(packet).await
    }
}

fn bit(id: u8) -> u8 {
    c::node_bit(id)
}
/// A participant evicted by the coordinator (3 s without its audio) is told only
/// by MembershipV3; if every copy is lost it keeps hearing SYNC and never rejoins,
/// while the coordinator discards its audio as a non-member (one-sided silence
/// seen in the field). SYNC carries the coordinator's member count every frame:
/// a count that disagrees with the local view for longer than three report
/// periods means the local membership is stale, so rejoin.
const SYNC_MEMBER_MISMATCH_MS: u32 = 3 * a::REPORT_MS;
pub(crate) fn sync_member_mismatch(
    since: Option<u32>,
    now_ms: u32,
    mismatch: bool,
) -> (Option<u32>, bool) {
    match (mismatch, since) {
        (false, _) => (None, false),
        (true, None) => (Some(now_ms), false),
        (true, Some(since)) if now_ms.wrapping_sub(since) > SYNC_MEMBER_MISMATCH_MS => (None, true),
        (true, since) => (since, false),
    }
}
fn identity(x: w::Identity) -> a::Identity {
    a::Identity {
        address_len: x.address_len,
        address: x.address,
    }
}
fn wire_identity(x: a::Identity) -> w::Identity {
    w::Identity {
        address_len: x.address_len,
        address: x.address,
    }
}
fn slots(x: w::SlotMap) -> c::SlotMap {
    c::SlotMap {
        slot_count: x.slot_count,
        slot_ids: x.slot_ids,
        active_speaker_count: x.active_speaker_count,
        active_speaker_ids: x.active_speaker_ids,
        relay_masks: x.relay_masks,
    }
}
fn wire_slots(x: c::SlotMap) -> w::SlotMap {
    w::SlotMap {
        slot_count: x.slot_count,
        slot_ids: x.slot_ids,
        active_speaker_count: x.active_speaker_count,
        active_speaker_ids: x.active_speaker_ids,
        relay_masks: x.relay_masks,
    }
}
fn topology(x: w::Topology) -> a::Topology {
    a::Topology {
        report_seq: x.report_seq,
        origin: identity(x.origin),
        direct_mask: x.direct_mask,
        rssi_dbm: x.rssi_dbm,
        quality: x.quality,
        leader_id: x.leader_id,
        term: x.term,
        slot_map: slots(x.slot_map),
    }
}
/// Re-derives `apply_membership`'s first failing check so the reject can be
/// counted by reason; the pinned library decision itself is not changed.
pub(crate) fn membership_reject_counter<'c>(
    c: &'c mut crate::protocol_membership::MembershipRx,
    s: &a::Snapshot,
    source: u8,
    m: &a::Membership,
    removes_local: bool,
) -> &'c mut u32 {
    let mut mask = 0;
    if removes_local {
        &mut c.removed_local
    } else if !a::membership_valid(Some(m), Some(&mut mask)) {
        &mut c.other
    } else if s.members & bit(source) == 0 || source != s.leader_id {
        &mut c.source
    } else if m.term != s.term || m.leader_id != s.leader_id {
        &mut c.term
    } else if s.membership_revision_seen
        && (m.revision == s.membership_revision
            || m.revision.wrapping_sub(s.membership_revision) >= 0x8000_0000)
    {
        &mut c.revision
    } else if mask & bit(s.local_id) == 0 {
        &mut c.other
    } else {
        &mut c.identity
    }
}
fn membership(x: w::MembershipV3) -> a::Membership {
    a::Membership {
        term: x.term,
        revision: x.revision,
        leader_id: x.leader_id,
        slot_map: slots(x.slot_map),
        identities: x.identities.map(identity),
    }
}
fn sync(x: w::SyncV3) -> a::Sync {
    a::Sync {
        leader: identity(x.leader),
        leader_id: x.leader_id,
        member_count: x.member_count,
        term: x.term,
        frame_counter: x.frame_counter,
        phase_us: x.phase_us,
        relay_depth: x.relay_depth,
    }
}
fn handover(x: w::HandoverProposal) -> a::Handover {
    a::Handover {
        old_leader_id: x.old_leader_id,
        candidate_id: x.candidate_id,
        next_term: x.next_term,
        switch_frame: x.switch_frame,
        members: x.members,
        slot_map: slots(x.slot_map),
    }
}
fn proposal(x: a::Handover) -> w::HandoverProposal {
    w::HandoverProposal {
        old_leader_id: x.old_leader_id,
        candidate_id: x.candidate_id,
        next_term: x.next_term,
        switch_frame: x.switch_frame,
        members: x.members,
        slot_map: wire_slots(x.slot_map),
    }
}
fn forward_header(h: Header) -> a::Header {
    a::Header {
        version: h.version,
        packet_type: h.packet_type,
        src_id: h.src_id,
        seq: h.seq,
        ttl: h.ttl,
        flags: h.flags,
        talk_channel: h.talk_channel,
        payload_len: h.payload_len,
    }
}
fn control_size(t: u8, len: u16) -> bool {
    let expected = match t {
        17 => w::SyncV3::SIZE,
        15 => w::JoinV3::SIZE,
        16 => w::JoinAckV3::SIZE,
        14 => w::Topology::SIZE,
        22 => w::MembershipV3::SIZE,
        23 => w::SpeakerRequest::SIZE,
        6 => w::SlotMap::SIZE,
        9 => w::SpeakerGrant::SIZE,
        10 => w::SpeakerRelease::SIZE,
        18 | 20 | 21 => w::HandoverProposal::SIZE,
        19 => w::HandoverAck::SIZE,
        _ => return false,
    };
    usize::from(len) == expected
}

impl Runtime {
    pub(crate) fn ae(&mut self, p: &mut impl AdaptivePlatform, e: AdaptiveEffect) -> AdaptiveReply {
        if p.owned_audio_wrappers() && self.audio_adaptive_effect(e, p) {
            return AdaptiveReply::Applied;
        }
        if p.owned_tx_wrappers() {
            match e {
                AdaptiveEffect::Queue {
                    priority,
                    packet_type,
                    payload,
                    len,
                } => {
                    let Some(payload) = payload.get(..usize::from(len)) else {
                        return AdaptiveReply::Accepted(crate::runtime::EINVAL);
                    };
                    return AdaptiveReply::Accepted(if priority {
                        self.tx_queue_priority(packet_type, payload)
                    } else {
                        self.tx_queue_control(packet_type, payload)
                    });
                }
                AdaptiveEffect::SlotSnapshot => {
                    return AdaptiveReply::SlotMap(self.tx_slot_snapshot())
                }
                AdaptiveEffect::JoinRequest => {
                    return AdaptiveReply::Accepted(self.tx_send_join_request(p))
                }
                AdaptiveEffect::JoinAck { id, slot, address } => {
                    return AdaptiveReply::Accepted(self.tx_send_join_ack(id, slot, address));
                }
                _ => {}
            }
        }
        p.adaptive_effect(
            e,
            AdaptiveIo {
                membership: self.membership,
                address: self.address,
                talk_channel: self.talk_channel,
                term: self.protocol.term,
                leader_identity: self.protocol.leader_identity,
                peers: self.session.peers.map(|p| SlotPeer {
                    id: p.node_id,
                    slot: p.slot_index,
                    active: p.active,
                    announced: p.announced,
                }),
                adaptive: &mut self.protocol.adaptive,
                speakers: &mut self.session.active_speaker_ids,
                masks: &mut self.session.relay_masks,
                deadlines_ms: &mut self.session.speaker_deadlines_ms,
                active_since_ms: &mut self.session.speaker_active_since_ms,
                control_ring: &mut self.session.control_ring,
                control_head: &mut self.session.control_head,
                control_tail: &mut self.session.control_tail,
                priority_control: &mut self.protocol.priority_control,
                priority_pending: &mut self.protocol.priority_pending,
                tx_seq: &mut self.protocol.tx_seq,
                control_turn: &mut self.protocol.control_turn,
                control_drops: &mut self.protocol.stat_control_ring_drop,
                tx_fail: &mut self.protocol.stat_tx_fail,
                air_report_seq: &mut self.protocol.air_report_seq,
                topology_tx_ok: &mut self.protocol.stat_topology_tx_ok,
            },
        )
    }
    fn queue(&mut self, msg: Message<'_>, priority: bool, p: &mut impl AdaptivePlatform) -> i32 {
        let mut payload = [0; 79];
        let len = msg
            .encode(&mut payload)
            .expect("bounded adaptive control layout") as u16;
        match self.ae(
            p,
            AdaptiveEffect::Queue {
                priority,
                packet_type: msg.packet_type(),
                payload,
                len,
            },
        ) {
            AdaptiveReply::Accepted(result) => result,
            _ => crate::runtime::ENOTSUP,
        }
    }
    fn slot_snapshot(&mut self, p: &mut impl AdaptivePlatform) -> Option<c::SlotMap> {
        if let AdaptiveReply::SlotMap(map) = self.ae(p, AdaptiveEffect::SlotSnapshot) {
            Some(map)
        } else {
            None
        }
    }
    pub(crate) fn frame_counter(&mut self, p: &mut impl AdaptivePlatform) -> Option<u32> {
        if let AdaptiveReply::Frame(frame) = self.ae(p, AdaptiveEffect::FrameCounter) {
            Some(frame)
        } else {
            None
        }
    }
    pub fn local_identity(&self) -> a::Identity {
        let mut id = a::Identity {
            address_len: 5,
            ..a::Identity::default()
        };
        id.address[..5].copy_from_slice(&self.address);
        id
    }
    pub fn adaptive_control_sequence_accept(&mut self, id: u8, t: u8, seq: u8) -> bool {
        if bit(id) == 0 {
            return false;
        }
        let s = &mut self.protocol;
        let (seen, last) = if t == 15 {
            (&mut s.join_seq_seen, &mut s.join_last_seq)
        } else {
            (&mut s.ack_seq_seen, &mut s.ack_last_seq)
        };
        let i = usize::from(id - 1);
        let delta = seq.wrapping_sub(last[i]);
        if *seen & bit(id) != 0 && (delta == 0 || delta >= 128) {
            return false;
        }
        last[i] = seq;
        *seen |= bit(id);
        true
    }
    fn clear_direct(&mut self, id: u8) {
        if bit(id) == 0 {
            return;
        }
        let i = usize::from(id - 1);
        self.protocol.direct_seq_valid[i] = false;
        self.protocol.direct_quality[i] = 0;
        self.protocol.direct_seen[i] = 0;
    }
    fn reset_binding(&mut self, id: u8, p: &mut impl AdaptivePlatform) {
        self.clear_direct(id);
        c::dedupe_purge_node(Some(&mut self.dedupe), id);
        c::dedupe_purge_node(Some(&mut self.protocol.control_presence_dedupe), id);
        self.protocol.join_seq_seen &= !bit(id);
        self.protocol.ack_seq_seen &= !bit(id);
        self.ae(p, AdaptiveEffect::ResetTracker(id));
        self.session.speaker_deadlines_ms[usize::from(id)] = 0;
        self.session.speaker_active_since_ms[usize::from(id)] = 0;
    }
    pub fn adaptive_bind_member(
        &mut self,
        id: u8,
        identity: Option<&a::Identity>,
        p: &mut impl AdaptivePlatform,
    ) -> bool {
        if bit(id) == 0 {
            return false;
        }
        let s = self.protocol.adaptive.snapshot();
        let changed = s.members & bit(id) == 0
            || identity.is_none_or(|x| *x != s.nodes[usize::from(id - 1)].identity);
        if !a::set_member(Some(&mut self.protocol.adaptive), id, identity) {
            return false;
        }
        if changed {
            self.reset_binding(id, p);
            if self.session.active_speaker_ids.contains(&id) {
                self.ae(
                    p,
                    AdaptiveEffect::ApplyRelease {
                        count: 1,
                        ids: [id, 0],
                    },
                );
                if self.membership.state == State::Active
                    && self.membership.role == Role::Coordinator
                {
                    self.queue(
                        Message::SpeakerRelease(w::SpeakerRelease {
                            speaker_count: 1,
                            speaker_ids: [id, 0],
                        }),
                        false,
                        p,
                    );
                }
            }
        }
        true
    }
    pub fn adaptive_reset(&mut self) {
        let s = &mut self.protocol;
        s.adaptive = a::Adaptive::default();
        c::dedupe_reset(Some(&mut s.control_presence_dedupe));
        s.join_seq_seen = 0;
        s.ack_seq_seen = 0;
        s.direct_seq_valid = [false; 8];
        s.direct_quality = [0; 8];
        s.direct_seen = [0; 8];
        s.graph_report_seq = 0;
        s.air_report_seq = 0;
        s.local_request_seq = 0;
        s.last_local_active_ms = 0;
        s.last_request_ms = 0;
        s.local_request_active = false;
        s.term = 0;
        s.leader_identity = a::Identity::default();
        s.priority_pending = false;
        s.authoritative_members = 0;
        s.convergence_until_ms = 0;
        s.grant_hold_until_ms = 0;
        s.last_membership_tx = 0;
        s.last_sync_log = 0;
    }
    pub fn adaptive_members_owned(&mut self, p: &mut impl AdaptivePlatform) {
        let m = self.membership;
        if m.state != State::Active || bit(m.node_id) == 0 || bit(m.coordinator_id) == 0 {
            return;
        }
        let local = self.protocol.adaptive.snapshot().local_id;
        if local == 0 || local != m.node_id {
            a::init(
                Some(&mut self.protocol.adaptive),
                m.node_id,
                m.coordinator_id,
                self.protocol.term,
                bit(m.node_id) | bit(m.coordinator_id),
                p.uptime_ms() as u32,
            );
            let own = self.local_identity();
            self.adaptive_bind_member(m.node_id, Some(&own), p);
            let leader = self.protocol.leader_identity;
            if m.node_id != m.coordinator_id && a::identity_valid(Some(&leader)) {
                self.adaptive_bind_member(m.coordinator_id, Some(&leader), p);
            }
        }
    }
    fn identity_for(&self, src: u8, id: Option<&a::Identity>) -> bool {
        if bit(src) == 0 || !a::identity_valid(id) {
            return false;
        }
        let bound = self.protocol.adaptive.snapshot().nodes[usize::from(src - 1)].identity;
        a::identity_valid(Some(&bound)) && Some(&bound) == id
    }
    pub fn adaptive_observe(
        &mut self,
        h: Header,
        report: &a::Topology,
        rssi: i8,
        p: &mut impl AdaptivePlatform,
    ) {
        if h.packet_type != 14
            || h.flags & 2 != 0
            || bit(h.src_id) == 0
            || h.src_id == self.membership.node_id
            || self.protocol.adaptive.snapshot().members & bit(h.src_id) == 0
            || !(-127..0).contains(&rssi)
        {
            return;
        }
        let i = usize::from(h.src_id - 1);
        let now = p.uptime_ms() as u32;
        let s = &mut self.protocol;
        let delta = report.report_seq.wrapping_sub(s.direct_seq[i]);
        if s.direct_seq_valid[i] && (delta == 0 || delta >= 0x8000_0000) {
            return;
        }
        let sample = if s.direct_seq_valid[i] && now.wrapping_sub(s.direct_seen[i]) < 3000 {
            (100 / delta) as u8
        } else {
            100
        };
        s.direct_quality[i] = if s.direct_seq_valid[i] {
            ((3 * u32::from(s.direct_quality[i]) + u32::from(sample) + 2) / 4) as u8
        } else {
            sample
        };
        s.direct_rssi[i] = if s.direct_seq_valid[i] {
            ((3 * i32::from(s.direct_rssi[i]) + i32::from(rssi)) / 4) as i8
        } else {
            rssi
        };
        s.direct_seq[i] = report.report_seq;
        s.direct_seq_valid[i] = true;
        s.direct_seen[i] = now;
    }
    fn copy_slot_members(&mut self, map: &c::SlotMap, p: &mut impl AdaptivePlatform) {
        let s = self.protocol.adaptive.snapshot();
        for peer in &mut self.session.peers {
            if peer.active && s.members & bit(peer.node_id) == 0 {
                peer.active = false;
            }
        }
        for slot in 0..map.slot_count.min(8) {
            let id = map.slot_ids[usize::from(slot)];
            if bit(id) == 0 || id == self.membership.node_id {
                continue;
            }
            let found = self
                .session
                .peers
                .iter()
                .position(|x| x.active && x.node_id == id);
            let index = found.or_else(|| self.session.peers.iter().position(|x| !x.active));
            if let Some(i) = index {
                let peer = &mut self.session.peers[i];
                peer.slot_index = slot as i8;
                if found.is_none() {
                    peer.node_id = id;
                    peer.active = true;
                    peer.announced = true;
                    peer.last_seen_ms = p.uptime_ms();
                }
                let id = s.nodes[usize::from(id - 1)].identity;
                if id.address_len == 5 {
                    peer.address.copy_from_slice(&id.address[..5]);
                }
            }
        }
        self.membership.peer_count = s.members.count_ones().wrapping_sub(1) as u8;
        self.protocol.authoritative_members = s.members;
    }
    fn prune_grants_for_members(&mut self, members: u8, p: &mut impl AdaptivePlatform) {
        let mut release = w::SpeakerRelease {
            speaker_count: 0,
            speaker_ids: [0; 2],
        };
        let mut changed = false;
        for i in 0..2 {
            let speaker = self.session.active_speaker_ids[i];
            if speaker != 0 && members & bit(speaker) == 0 {
                release.speaker_ids[usize::from(release.speaker_count)] = speaker;
                release.speaker_count += 1;
                self.session.active_speaker_ids[i] = 0;
                self.session.relay_masks[i] = 0;
                changed = true;
            } else if speaker != 0 {
                let mask = self.session.relay_masks[i] & members & !bit(speaker);
                changed |= mask != self.session.relay_masks[i];
                self.session.relay_masks[i] = mask;
            } else if self.session.relay_masks[i] != 0 {
                self.session.relay_masks[i] = 0;
                changed = true;
            }
        }
        if release.speaker_count != 0 {
            self.queue(Message::SpeakerRelease(release), false, p);
        }
        if changed {
            let mut grant = w::SpeakerGrant {
                speaker_count: 0,
                speaker_ids: [0; 2],
                relay_masks: [0; 2],
            };
            for i in 0..2 {
                if self.session.active_speaker_ids[i] == 0 {
                    continue;
                }
                let n = usize::from(grant.speaker_count);
                grant.speaker_count += 1;
                grant.speaker_ids[n] = self.session.active_speaker_ids[i];
                grant.relay_masks[n] = self.session.relay_masks[i];
            }
            self.queue(Message::SpeakerGrant(grant), false, p);
        }
    }
    pub fn adaptive_publish_membership(&mut self, p: &mut impl AdaptivePlatform) -> i32 {
        if self.membership.role != Role::Coordinator || self.membership.state != State::Active {
            return -11;
        }
        self.adaptive_members_owned(p);
        self.prune_grants_for_members(self.protocol.adaptive.snapshot().members, p);
        let Some(map) = self.slot_snapshot(p) else {
            return crate::runtime::ENOTSUP;
        };
        let s = self.protocol.adaptive.snapshot();
        let mut snapshot = a::Membership {
            term: self.protocol.term,
            leader_id: self.membership.node_id,
            slot_map: map,
            ..a::Membership::default()
        };
        for id in 1..=8 {
            if s.members & bit(id) != 0 {
                snapshot.identities[usize::from(id - 1)] = s.nodes[usize::from(id - 1)].identity;
            }
        }
        let mut members = 0;
        if !a::membership_valid(Some(&snapshot), Some(&mut members)) || members != s.members {
            return -22;
        }
        let ret = self.queue(
            Message::MembershipV3(w::MembershipV3 {
                term: snapshot.term,
                revision: 0,
                leader_id: snapshot.leader_id,
                slot_map: wire_slots(map),
                identities: snapshot.identities.map(wire_identity),
            }),
            false,
            p,
        );
        if ret == 0 {
            self.protocol.last_membership_tx = p.uptime_ms() as u32;
        }
        ret
    }
    pub fn adaptive_converging(&self, now: u32) -> bool {
        self.membership.role == Role::Coordinator
            && self.protocol.adaptive.snapshot().transition_done
            && (self.protocol.convergence_until_ms.wrapping_sub(now) as i32) >= 0
    }
    pub fn adaptive_local_voice(&mut self, active: bool, p: &mut impl AdaptivePlatform) {
        if self.membership.state != State::Active || bit(self.membership.node_id) == 0 {
            return;
        }
        let now = p.uptime_ms() as u32;
        let refresh = if self
            .session
            .active_speaker_ids
            .contains(&self.membership.node_id)
        {
            1000
        } else {
            400
        };
        if self.membership.role != Role::Participant
            || (active == self.protocol.local_request_active
                && (!active || now.wrapping_sub(self.protocol.last_request_ms) < refresh))
        {
            return;
        }
        self.protocol.local_request_seq = self.protocol.local_request_seq.wrapping_add(1);
        let Some(frame) = self.frame_counter(p) else {
            return;
        };
        let request = w::SpeakerRequest {
            term: self.protocol.term,
            request_seq: self.protocol.local_request_seq,
            frame_counter: frame,
            active: u8::from(active),
        };
        if self.queue(Message::SpeakerRequest(request), false, p) == 0 {
            self.protocol.local_request_active = active;
            self.protocol.last_request_ms = now;
        }
    }
    async fn apply_transition_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl AdaptivePlatform,
    ) {
        let s = self.protocol.adaptive.snapshot();
        let old = self.membership.coordinator_id;
        let mut snapshot = self.membership_snapshot();
        if !m::apply_handover(Some(&mut snapshot), old, s.leader_id, s.term) {
            return;
        }
        self.membership.role = if snapshot.role == m::Role::Coordinator {
            Role::Coordinator
        } else {
            Role::Participant
        };
        self.protocol.term = snapshot.term;
        self.membership.coordinator_id = snapshot.coordinator_id;
        self.protocol.join_seq_seen = 0;
        self.protocol.ack_seq_seen = 0;
        self.protocol.leader_identity = s.nodes[usize::from(s.leader_id - 1)].identity;
        self.protocol.local_request_active = false;
        self.protocol.last_request_ms = 0;
        self.protocol.adaptive.clear_forward_queue();
        self.protocol.priority_pending = false;
        self.session.control_tail = self.session.control_head;
        self.copy_slot_members(&s.pending.slot_map, p);
        self.ae(
            p,
            AdaptiveEffect::ClockSource(self.membership.role == Role::Coordinator),
        );
        if self.membership.role == Role::Coordinator {
            self.protocol.convergence_until_ms =
                (p.uptime_ms() as u32).wrapping_add(a::RECOVERY_GRACE_MS);
            self.protocol.grant_hold_until_ms = (p.uptime_ms() as u32).wrapping_add(a::EXPIRE_MS);
            self.adaptive_publish_membership(p);
        }
        self.protocol.last_sync_time = p.uptime_ms() as u32;
        if ASYNC_SEND {
            p.adaptive_status_async(self.membership).await;
        } else {
            self.ae(p, AdaptiveEffect::Status(self.membership));
        }
    }
    pub fn adaptive_frame_tick(&mut self, frame: u32, p: &mut impl AdaptivePlatform) {
        crate::corefuture::ready(self.adaptive_frame_tick_impl::<false>(frame, p))
    }
    pub async fn adaptive_frame_tick_async(&mut self, frame: u32, p: &mut impl AdaptivePlatform) {
        self.adaptive_frame_tick_impl::<true>(frame, p).await
    }
    pub(crate) async fn adaptive_frame_tick_impl<const ASYNC_SEND: bool>(
        &mut self,
        frame: u32,
        p: &mut impl AdaptivePlatform,
    ) {
        if self.membership.state != State::Active || self.protocol.adaptive.snapshot().local_id == 0
        {
            return;
        }
        let now = p.uptime_ms() as u32;
        let before = self.protocol.adaptive.snapshot();
        let timed_out = before.prepared
            && !before.committed
            && self.membership.role == Role::Coordinator
            && (now.wrapping_sub(before.deadline_ms) as i32) > 0;
        if a::tick(Some(&mut self.protocol.adaptive), frame, now) {
            self.apply_transition_impl::<ASYNC_SEND>(p).await;
        }
        if self.membership.state == State::Active {
            if self.membership.role == Role::Participant && self.protocol.local_request_active {
                if now.wrapping_sub(self.protocol.last_local_active_ms) > 120 {
                    self.adaptive_local_voice(false, p);
                } else {
                    let refresh = if self
                        .session
                        .active_speaker_ids
                        .contains(&self.membership.node_id)
                    {
                        1000
                    } else {
                        400
                    };
                    if now.wrapping_sub(self.protocol.last_request_ms) >= refresh {
                        self.adaptive_local_voice(true, p);
                    }
                }
            } else if self.membership.role == Role::Coordinator {
                self.ae(p, AdaptiveEffect::ExpireSpeakers(now));
            }
        }
        let s = self.protocol.adaptive.snapshot();
        if before.prepared
            && !s.prepared
            && !s.transition_done
            && !timed_out
            && self.protocol.priority_pending
            && (18..=21).contains(&self.protocol.priority_control.data[1])
        {
            self.protocol.priority_pending = false;
        }
        if timed_out {
            if self.protocol.priority_pending && self.protocol.priority_control.data[1] == 18 {
                self.protocol.priority_pending = false;
            }
            let msg = Message::HandoverCancel(proposal(before.pending));
            if self.queue(msg, true, p) != 0 {
                self.queue(msg, false, p);
            }
        }
        let s = self.protocol.adaptive.snapshot();
        if self.membership.role != Role::Coordinator || !s.prepared || s.transition_done {
            return;
        }
        if !s.committed {
            a::commit(
                Some(&mut self.protocol.adaptive),
                Some(&mut a::Handover::default()),
                now,
            );
        }
        let s = self.protocol.adaptive.snapshot();
        let last = if s.committed {
            self.protocol.last_commit_tx
        } else {
            self.protocol.last_prepare_tx
        };
        if now.wrapping_sub(last) >= 120 {
            let msg = if s.committed {
                Message::HandoverCommit(proposal(s.pending))
            } else {
                Message::HandoverPrepare(proposal(s.pending))
            };
            if self.queue(msg, true, p) == 0 {
                if s.committed {
                    self.protocol.last_commit_tx = now;
                } else {
                    self.protocol.last_prepare_tx = now;
                }
            }
        }
    }
    pub fn adaptive_report_tick(&mut self, p: &mut impl AdaptivePlatform) {
        if self.membership.state != State::Active || self.protocol.adaptive.snapshot().local_id == 0
        {
            return;
        }
        let now = p.uptime_ms() as u32;
        self.adaptive_members_owned(p);
        if self.membership.role == Role::Coordinator
            && (self.protocol.last_membership_tx == 0
                || now.wrapping_sub(self.protocol.last_membership_tx) >= a::REPORT_MS)
        {
            self.adaptive_publish_membership(p);
        }
        self.protocol.graph_report_seq = self.protocol.graph_report_seq.wrapping_add(1);
        let Some(map) = self.slot_snapshot(p) else {
            return;
        };
        let mut report = a::Topology {
            report_seq: self.protocol.graph_report_seq,
            origin: self.local_identity(),
            leader_id: self.membership.coordinator_id,
            term: self.protocol.term,
            slot_map: map,
            ..a::Topology::default()
        };
        let members = self.protocol.adaptive.snapshot().members;
        for i in 0..8 {
            let id = i as u8 + 1;
            if id == self.membership.node_id
                || members & bit(id) == 0
                || !self.protocol.direct_seq_valid[i]
                || now.wrapping_sub(self.protocol.direct_seen[i]) > a::EXPIRE_MS
            {
                continue;
            }
            report.direct_mask |= bit(id);
            report.rssi_dbm[i] = self.protocol.direct_rssi[i];
            report.quality[i] = self.protocol.direct_quality[i];
        }
        if a::report(
            Some(&mut self.protocol.adaptive),
            self.membership.node_id,
            Some(&report),
            false,
            now,
        ) {
            self.queue(
                Message::Topology(w::Topology {
                    report_seq: report.report_seq,
                    origin: wire_identity(report.origin),
                    direct_mask: report.direct_mask,
                    rssi_dbm: report.rssi_dbm,
                    quality: report.quality,
                    leader_id: report.leader_id,
                    term: report.term,
                    slot_map: wire_slots(report.slot_map),
                }),
                false,
                p,
            );
        }
        if self.membership.role == Role::Coordinator && !self.protocol.adaptive.snapshot().prepared
        {
            let candidate = a::candidate(Some(&mut self.protocol.adaptive), now);
            // The shared helper intentionally permits candidate zero; C callers guard it.
            if candidate != 0 {
                let Some(map) = self.slot_snapshot(p) else {
                    return;
                };
                let Some(frame) = self.frame_counter(p) else {
                    return;
                };
                let mut out = a::Handover::default();
                if a::prepare(
                    Some(&mut self.protocol.adaptive),
                    candidate,
                    frame,
                    Some(&map),
                    now,
                    Some(&mut out),
                ) {
                    self.protocol.last_prepare_tx = now.wrapping_sub(120);
                }
            }
        }
    }

    /// Recognized invalid controls return true; unknown types/wrong sizes return
    /// false. Short backing slices and C signed overflow are explicit errors.
    /// Call with CombinedMembershipPlatform to close nested membership wrappers.
    pub fn adaptive_control_rx(
        &mut self,
        h: Header,
        payload: &[u8],
        rssi: i8,
        timestamp_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        crate::corefuture::ready(self.adaptive_control_rx_impl::<false>(
            h,
            payload,
            rssi,
            timestamp_us,
            p,
        ))
    }
    pub async fn adaptive_control_rx_async(
        &mut self,
        h: Header,
        payload: &[u8],
        rssi: i8,
        timestamp_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        self.adaptive_control_rx_impl::<true>(h, payload, rssi, timestamp_us, p)
            .await
    }
    pub(crate) async fn adaptive_control_rx_impl<const ASYNC_SEND: bool>(
        &mut self,
        h: Header,
        payload: &[u8],
        rssi: i8,
        timestamp_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_adaptive_wrappers()
        {
            return Err(Error::NotIntegrated);
        }
        if !control_size(h.packet_type, h.payload_len) {
            return Ok(false);
        }
        let now = p.uptime_ms() as u32;
        let received_ms = (timestamp_us / 1000) as u32;
        let relayed = h.flags & 2 != 0;
        if h.flags != if relayed { 2 } else { 0 } {
            return Ok(true);
        }
        if (relayed && h.ttl != 1) || (!relayed && h.ttl != 2) {
            return Ok(true);
        }
        if relayed && h.packet_type == 17 {
            let depth = *payload
                .get(w::SyncV3::SIZE - 1)
                .ok_or(Error::ShortPayload)?;
            if depth != 1 {
                return Ok(true);
            }
        }
        let payload = payload
            .get(..usize::from(h.payload_len))
            .ok_or(Error::ShortPayload)?;
        let msg = Message::decode(h.packet_type, payload, w::Transport::Nrf)
            .map_err(|_| Error::ShortPayload)?;
        let mut origin = None;
        let mut term = self.protocol.term;
        match msg {
            Message::JoinV3(x) => {
                origin = Some(identity(x.origin));
                if !a::identity_valid(origin.as_ref())
                    || !matches!(self.membership.state, State::Active | State::Joining)
                    || !a::identity_valid(Some(&identity(x.target)))
                    || identity(x.target) != self.protocol.leader_identity
                {
                    return Ok(true);
                }
            }
            Message::SyncV3(x) => {
                origin = Some(identity(x.leader));
                term = x.term;
                if !a::identity_valid(origin.as_ref())
                    || x.leader_id != h.src_id
                    || !(1..=8).contains(&x.member_count)
                    || x.phase_us >= 20000
                    || x.relay_depth != u8::from(relayed)
                    || now.wrapping_sub(received_ms) > a::SYNC_MAX_AGE_MS
                {
                    return Ok(true);
                }
            }
            Message::Topology(x) => {
                origin = Some(identity(x.origin));
                term = x.term;
            }
            Message::JoinAckV3(x) => {
                term = x.term;
                if !a::identity_valid(Some(&identity(x.target)))
                    || x.coordinator_id != h.src_id
                    || bit(x.assigned_id) == 0
                    || x.assigned_id == h.src_id
                    || x.slot_index >= 8
                {
                    return Ok(true);
                }
            }
            Message::MembershipV3(x) => {
                term = x.term;
                if !a::membership_valid(Some(&membership(x)), None) {
                    return Ok(true);
                }
            }
            Message::SpeakerRequest(x) => term = x.term,
            Message::HandoverPrepare(x)
            | Message::HandoverCommit(x)
            | Message::HandoverCancel(x) => term = x.next_term,
            Message::HandoverAck(x) => term = x.next_term,
            _ => {}
        }
        let mut tdma_timestamp = None;
        if let Message::SyncV3(x) = msg {
            if self.membership.state == State::Scanning {
                if (x.term != 0 && self.protocol.term != 0 && x.term < self.protocol.term)
                    || bit(x.leader_id) == 0
                    || x.leader.address[..5] == self.address
                {
                    return Ok(true);
                }
                self.protocol.leader_identity = identity(x.leader);
                self.protocol.term = x.term;
                self.membership_handle_impl::<ASYNC_SEND>(Input::Discover(x.leader_id), p)
                    .await?;
                return Ok(true);
            }
            if self.membership.state == State::Joining
                && (x.leader_id != self.membership.coordinator_id
                    || x.term != self.protocol.term
                    || identity(x.leader) != self.protocol.leader_identity)
            {
                return Ok(true);
            }
            if self.membership.state == State::Active {
                tdma_timestamp = p.tdma_rx_timestamp(timestamp_us);
                if tdma_timestamp.is_none() {
                    return Ok(true);
                }
            }
            if self.membership.state == State::Active
                && self.protocol.adaptive.snapshot().prepared
                && a::reconcile_sync(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&sync(x)),
                    now,
                )
            {
                self.apply_transition_impl::<ASYNC_SEND>(p).await;
            }
            if self.membership.state == State::Active
                && self.membership.role == Role::Coordinator
                && !self.identity_for(self.membership.node_id, origin.as_ref())
            {
                let s = self.protocol.adaptive.snapshot();
                if self.protocol.term == 1
                    && x.term == 1
                    && !s.prepared
                    && !s.transition_done
                    && a::cold_join(
                        Some(&self.local_identity()),
                        s.members.count_ones() as u8,
                        Some(&identity(x.leader)),
                        x.member_count,
                    )
                {
                    self.membership_handle_impl::<ASYNC_SEND>(
                        Input::Demote {
                            leader: x.leader_id,
                            term: x.term,
                            identity: identity(x.leader),
                        },
                        p,
                    )
                    .await?;
                }
                return Ok(true);
            }
        }
        if self.membership.state == State::Active {
            self.adaptive_members_owned(p);
            if !a::control_authorized(Some(&self.protocol.adaptive), h.packet_type, h.src_id) {
                return Ok(true);
            }
            if !matches!(h.packet_type, 15 | 14 | 17) {
                let id =
                    self.protocol.adaptive.snapshot().nodes[usize::from(h.src_id - 1)].identity;
                if !self.identity_for(h.src_id, Some(&id)) {
                    return Ok(true);
                }
            }
            if h.packet_type == 14
                && (!a::identity_valid(origin.as_ref())
                    || !self.identity_for(h.src_id, origin.as_ref()))
            {
                return Ok(true);
            }
            if h.packet_type == 17
                && (!self.identity_for(h.src_id, origin.as_ref()) || term != self.protocol.term)
            {
                return Ok(true);
            }
            if matches!(h.packet_type, 16 | 22) && term != self.protocol.term {
                return Ok(true);
            }
            if !matches!(h.packet_type, 15 | 16 | 22 | 17 | 14)
                && term != self.protocol.term
                && !(term == self.protocol.term.wrapping_add(1)
                    && (18..=21).contains(&h.packet_type))
            {
                return Ok(true);
            }
        } else if !matches!(h.packet_type, 15..=17) {
            return Ok(true);
        }
        match msg {
            Message::Topology(x) => {
                let report = topology(x);
                if !a::report_valid(Some(&self.protocol.adaptive), h.src_id, Some(&report)) {
                    return Ok(true);
                }
                if !relayed {
                    self.protocol.stat_topology_rx_direct =
                        self.protocol.stat_topology_rx_direct.wrapping_add(1);
                    self.adaptive_observe(h, &report, rssi, p);
                    if let Some(peer) = self
                        .session
                        .peers
                        .iter_mut()
                        .find(|x| x.active && x.node_id == h.src_id)
                    {
                        peer.rssi_dbm = rssi;
                    }
                }
                if !a::report(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&report),
                    relayed,
                    now,
                ) {
                    return Ok(true);
                }
                if relayed {
                    self.protocol.stat_topology_rx_relayed =
                        self.protocol.stat_topology_rx_relayed.wrapping_add(1);
                }
            }
            Message::SpeakerRequest(x) => {
                let Some(frame) = self.frame_counter(p) else {
                    return Err(Error::NotIntegrated);
                };
                let req = a::SpeakerRequest {
                    term: x.term,
                    request_seq: x.request_seq,
                    frame_counter: x.frame_counter,
                    active: x.active,
                };
                if !a::speaker_request(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&req),
                    frame,
                    now,
                ) {
                    return Ok(true);
                }
                if self.membership.role == Role::Coordinator {
                    let i = usize::from(h.src_id);
                    if x.active != 0 {
                        if self.session.speaker_deadlines_ms[i] <= i64::from(now) {
                            self.session.speaker_active_since_ms[i] = i64::from(now);
                        }
                        self.session.speaker_deadlines_ms[i] = i64::from(now.wrapping_add(1500));
                    } else {
                        self.session.speaker_deadlines_ms[i] = 0;
                    }
                    self.ae(p, AdaptiveEffect::UpdateGrants);
                }
            }
            Message::MembershipV3(x) => {
                let before = self.protocol.adaptive.snapshot();
                let authoritative = self.protocol.authoritative_members;
                let map = membership(x);
                if !a::apply_membership(Some(&mut self.protocol.adaptive), h.src_id, Some(&map)) {
                    let removes_local = a::membership_removes_local(
                        Some(&self.protocol.adaptive),
                        h.src_id,
                        Some(&map),
                    );
                    *membership_reject_counter(
                        &mut self.protocol.membership_rx,
                        &self.protocol.adaptive.snapshot(),
                        h.src_id,
                        &map,
                        removes_local,
                    ) += 1;
                    if removes_local {
                        let s = self.protocol.adaptive.snapshot();
                        let leader = s.nodes[usize::from(s.leader_id - 1)].identity;
                        self.membership_handle_impl::<ASYNC_SEND>(
                            Input::Rejoin {
                                leader: s.leader_id,
                                term: s.term,
                                identity: leader,
                            },
                            p,
                        )
                        .await?;
                    }
                    return Ok(true);
                }
                self.protocol.membership_rx.applied += 1;
                let s = self.protocol.adaptive.snapshot();
                for id in 1..=8 {
                    let i = usize::from(id - 1);
                    if before.members & bit(id) != s.members & bit(id)
                        || before.nodes[i].identity != s.nodes[i].identity
                    {
                        self.reset_binding(id, p);
                    }
                }
                self.copy_slot_members(&map.slot_map, p);
                self.membership.membership_known = true;
                for slot in 0..map.slot_map.slot_count {
                    if map.slot_map.slot_ids[usize::from(slot)] == self.membership.node_id
                        && self.membership.slot_index != slot as i8
                    {
                        self.membership.slot_index = slot as i8;
                        self.ae(p, AdaptiveEffect::SetSlot(slot as i8));
                        break;
                    }
                }
                self.ae(p, AdaptiveEffect::ApplySlotMap(map.slot_map));
                if authoritative != self.protocol.adaptive.snapshot().members {
                    if ASYNC_SEND {
                        p.adaptive_status_async(self.membership).await;
                    } else {
                        self.ae(p, AdaptiveEffect::Status(self.membership));
                    }
                }
            }
            Message::JoinAckV3(x)
                if self.membership.state == State::Active
                    && (!self.identity_for(h.src_id, Some(&self.protocol.leader_identity))
                        || x.term != self.protocol.adaptive.snapshot().term) =>
            {
                return Ok(true);
            }
            Message::JoinAckV3(_) if self.membership.state == State::Active => {}
            Message::HandoverPrepare(x) => {
                let Some(frame) = self.frame_counter(p) else {
                    return Err(Error::NotIntegrated);
                };
                if !a::receive_prepare(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&handover(x)),
                    frame,
                    now,
                ) {
                    return Ok(true);
                }
                self.queue(
                    Message::HandoverAck(w::HandoverAck {
                        old_leader_id: x.old_leader_id,
                        candidate_id: x.candidate_id,
                        next_term: x.next_term,
                        switch_frame: x.switch_frame,
                        acknowledger_id: self.membership.node_id,
                    }),
                    true,
                    p,
                );
            }
            Message::HandoverAck(x) => {
                let ack = a::HandoverAck {
                    old_leader_id: x.old_leader_id,
                    candidate_id: x.candidate_id,
                    next_term: x.next_term,
                    switch_frame: x.switch_frame,
                    acknowledger_id: x.acknowledger_id,
                };
                if !a::ack(Some(&mut self.protocol.adaptive), h.src_id, Some(&ack), now)
                    || !self.adaptive_control_sequence_accept(h.src_id, 19, h.seq)
                {
                    return Ok(true);
                }
            }
            Message::HandoverCommit(x)
                if !a::receive_commit(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&handover(x)),
                    now,
                ) =>
            {
                return Ok(true);
            }
            Message::HandoverCancel(x)
                if !a::cancel(
                    Some(&mut self.protocol.adaptive),
                    h.src_id,
                    Some(&handover(x)),
                ) =>
            {
                return Ok(true);
            }
            Message::HandoverCommit(_) | Message::HandoverCancel(_) => {}
            Message::SyncV3(x)
                if self.membership.state == State::Active
                    && self.membership.role == Role::Participant =>
            {
                let Some(tdma_timestamp) = tdma_timestamp else {
                    return Ok(true);
                };
                let base = tdma_timestamp
                    .checked_sub(i64::from(x.phase_us))
                    .and_then(|x| x.checked_sub(300))
                    .ok_or(Error::UndefinedArithmetic)?;
                self.ae(
                    p,
                    AdaptiveEffect::Sync {
                        frame: x.frame_counter,
                        drift: 0,
                        base_us: base,
                    },
                );
                self.protocol.last_sync_time = received_ms;
                let s = self.protocol.adaptive.snapshot();
                let (since, rejoin) = sync_member_mismatch(
                    self.protocol.sync_member_mismatch_since,
                    received_ms,
                    x.member_count != s.members.count_ones() as u8,
                );
                self.protocol.sync_member_mismatch_since = since;
                if rejoin && bit(s.leader_id) != 0 {
                    let leader = s.nodes[usize::from(s.leader_id - 1)].identity;
                    self.membership_handle_impl::<ASYNC_SEND>(
                        Input::Rejoin {
                            leader: s.leader_id,
                            term: s.term,
                            identity: leader,
                        },
                        p,
                    )
                    .await?;
                }
            }
            Message::JoinV3(_) | Message::JoinAckV3(_) => {
                self.membership_handle_impl::<ASYNC_SEND>(Input::Packet { header: h, payload }, p)
                    .await?;
            }
            Message::SlotMap(x) => {
                let s = self.protocol.adaptive.snapshot();
                let mut check = a::Membership {
                    term: s.term,
                    leader_id: s.leader_id,
                    slot_map: slots(x),
                    ..a::Membership::default()
                };
                for id in 1..=8 {
                    if s.members & bit(id) != 0 {
                        check.identities[usize::from(id - 1)] =
                            s.nodes[usize::from(id - 1)].identity;
                    }
                }
                let mut members = 0;
                if !a::membership_valid(Some(&check), Some(&mut members)) || members != s.members {
                    return Ok(true);
                }
                self.ae(p, AdaptiveEffect::ApplySlotMap(check.slot_map));
            }
            Message::SpeakerGrant(x) => {
                self.ae(
                    p,
                    AdaptiveEffect::ApplyGrant {
                        count: x.speaker_count,
                        ids: x.speaker_ids,
                        masks: x.relay_masks,
                    },
                );
            }
            Message::SpeakerRelease(x) => {
                self.ae(
                    p,
                    AdaptiveEffect::ApplyRelease {
                        count: x.speaker_count,
                        ids: x.speaker_ids,
                    },
                );
            }
            _ => {}
        }
        let pending_sync =
            h.packet_type == 17 && self.protocol.adaptive.forward_queue_contains_type(17);
        let forward_sync = h.packet_type != 17
            || (!pending_sync
                && now.wrapping_sub(self.protocol.last_forward_sync) >= 550
                && self.protocol.adaptive.forward_queue_len() < 2);
        if self.membership.state == State::Active
            && bit(h.src_id) != 0
            && h.src_id != self.membership.node_id
            && self.protocol.adaptive.snapshot().members & bit(h.src_id) != 0
            && c::dedupe_accept(
                Some(&mut self.protocol.control_presence_dedupe),
                h.packet_type,
                h.src_id,
                h.seq,
            )
        {
            self.membership_handle_impl::<ASYNC_SEND>(Input::NotePresence(h.src_id), p)
                .await?;
        }
        if self.membership.state == State::Active
            && h.src_id != self.membership.node_id
            && forward_sync
            && a::forward_enqueue(
                Some(&mut self.protocol.adaptive),
                Some(&forward_header(h)),
                Some(payload),
                origin.as_ref(),
                term,
                self.protocol.last_sync_time != 0
                    && now.wrapping_sub(self.protocol.last_sync_time) <= 600,
                now,
            )
            && h.packet_type == 17
        {
            self.protocol.last_forward_sync = now;
            if now.wrapping_sub(self.protocol.last_sync_log) >= 2000 {
                self.protocol.last_sync_log = now;
            }
        }
        Ok(true)
    }
}
