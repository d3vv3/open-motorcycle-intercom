//! Serial adapter of unchanged `nrf_mesh/src/mesh_protocol_membership.c`.
//! Standalone platforms preserve scripted wrapper boundaries. The explicit
//! CombinedMembershipPlatform dispatches adaptive wrappers on the same Runtime.
use core::fmt::{self, Write};
use openhelmet_mesh::{adaptive::Adaptive, core as mesh_core, membership as pure};
use openhelmet_wire::mesh::{Header, Identity, JoinAckV3, JoinV3, LeaveNrf};

use crate::platform::{accepts_protocol_algorithms, Effect, Platform};
use crate::runtime::{Membership, Peer, RelayEntry, Role, Runtime, State, Timer, TimerToken};

/// Why an Active participant refused a coordinator MembershipV3 (cumulative).
/// Diagnoses the field case where a participant stays Active while the
/// coordinator has already dropped it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MembershipRx {
    pub applied: u32,
    pub removed_local: u32,
    pub source: u32,
    pub term: u32,
    pub revision: u32,
    pub identity: u32,
    pub other: u32,
}

/// Only fields missing from Runtime/Session. There is no second membership table.
#[derive(Default)]
pub struct ProtocolState {
    pub membership_rx: MembershipRx,
    pub(crate) term: u32,
    pub(crate) leader_identity: IdentityValue,
    pub(crate) join_attempts: i32,
    pub(crate) last_sync_time: u32,
    pub(crate) sync_member_mismatch_since: Option<u32>,
    pub(crate) adaptive: Adaptive,
    pub(crate) control_presence_dedupe: mesh_core::Dedupe,
    pub(crate) direct_seq: [u32; 8],
    pub(crate) direct_seq_valid: [bool; 8],
    pub(crate) direct_quality: [u8; 8],
    pub(crate) direct_rssi: [i8; 8],
    pub(crate) direct_seen: [u32; 8],
    pub(crate) graph_report_seq: u32,
    pub(crate) air_report_seq: u32,
    pub(crate) local_request_seq: u32,
    pub(crate) last_local_active_ms: u32,
    pub(crate) last_request_ms: u32,
    pub(crate) local_request_active: bool,
    pub(crate) last_prepare_tx: u32,
    pub(crate) last_commit_tx: u32,
    pub(crate) last_forward_sync: u32,
    pub(crate) last_sync_log: u32,
    pub(crate) convergence_until_ms: u32,
    pub(crate) grant_hold_until_ms: u32,
    pub(crate) last_membership_tx: u32,
    pub(crate) authoritative_members: u8,
    pub(crate) join_last_seq: [u8; 8],
    pub(crate) ack_last_seq: [u8; 8],
    pub(crate) join_seq_seen: u8,
    pub(crate) ack_seq_seen: u8,
    pub(crate) priority_control: RelayEntry,
    pub(crate) priority_pending: bool,
    pub(crate) tx_seq: u8,
    pub(crate) control_turn: u8,
    pub(crate) stat_control_ring_drop: u32,
    pub(crate) stat_tx_fail: u32,
    pub(crate) stat_topology_tx_ok: u32,
    pub(crate) stat_topology_rx_direct: u32,
    pub(crate) stat_topology_rx_relayed: u32,
}

pub type IdentityValue = openhelmet_mesh::adaptive::Identity;

impl ProtocolState {
    pub fn adaptive(&self) -> &Adaptive {
        &self.adaptive
    }

    /// Borrow the existing shared value, not a second owner.
    pub fn adaptive_mut(&mut self) -> &mut Adaptive {
        &mut self.adaptive
    }

    /// Limited simulation reset model. This is NOT mesh_protocol_adaptive_reset:
    /// direct-link estimators, voice, priority TX and wrapper sequence resets
    /// are intentionally absent from the standalone boundary model.
    pub fn reset_boundary_model(&mut self) {
        self.adaptive = Adaptive::default();
        self.term = 0;
        self.leader_identity = IdentityValue::default();
    }
}

/// Algorithm responses are synchronous, ordered and non-reentrant. Implementations
/// may mutate the sole owned adaptive value through this borrow, never retain it.
/// Output futures instead receive owned MembershipOutput values.
/// The simulation's shared-helper model is NOT the production adaptive wrapper.
#[allow(async_fn_in_trait)]
pub trait MembershipPlatform: Platform {
    /// Output-only compatibility hook. Legacy output callbacks must not depend
    /// on or mutate protocol storage; no owner loan enters the output future.
    async fn membership_output_async(&mut self, output: MembershipOutput) -> bool {
        self.membership_effect(output.into(), &mut ProtocolState::default())
    }
    async fn dispatch_membership_async(
        &mut self,
        effect: MembershipEffect,
        runtime: &mut Runtime,
    ) -> bool {
        self.dispatch_membership(effect, runtime)
    }
    fn membership_effect(&mut self, effect: MembershipEffect, state: &mut ProtocolState) -> bool;
    /// Default preserves the standalone scripted boundary fixtures. Combined
    /// platforms override this to execute wrappers on the existing owner.
    fn dispatch_membership(&mut self, effect: MembershipEffect, owner: &mut Runtime) -> bool {
        self.membership_effect(effect, &mut owner.protocol)
    }
    fn uptime_ms(&mut self) -> i64 {
        self.now_us() / 1000
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MembershipOutput {
    Status(Membership),
    Event { id: u8, node: Option<u8> },
    Log { bytes: [u8; 128], len: u8 },
}

impl From<MembershipOutput> for MembershipEffect {
    fn from(output: MembershipOutput) -> Self {
        match output {
            MembershipOutput::Status(m) => Self::Status(m),
            MembershipOutput::Event { id, node } => Self::Event { id, node },
            MembershipOutput::Log { bytes, len } => Self::Log { bytes, len },
        }
    }
}

impl MembershipPlatform for crate::platform::NotIntegrated {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MembershipEffect {
    BindWork,
    AdaptiveReset,
    BindMember {
        id: u8,
        identity: Option<IdentityValue>,
    },
    ControlSequence {
        id: u8,
        packet_type: u8,
        seq: u8,
    },
    AdaptiveMembers {
        state: State,
        local: u8,
        leader: u8,
        term: u32,
    },
    ResetTracker(u8),
    ResetTrackers,
    ClearRelay,
    ClearActivity,
    ClearGrants,
    ClearBitmaps,
    PurgeTx,
    Ingress {
        enabled: bool,
        purge: bool,
    },
    UpdateGrants,
    SlotMap,
    JoinRequest,
    JoinAck {
        id: u8,
        slot: u8,
        address: [u8; 5],
    },
    TdmaStop,
    TdmaStart {
        slot: i8,
        coordinator: bool,
    },
    ClockSource(bool),
    Status(Membership),
    Event {
        id: u8,
        node: Option<u8>,
    },
    LogInitialized,
    Log {
        bytes: [u8; 128],
        len: u8,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    NotIntegrated,
    /// C signed subtraction/shift is undefined for this input.
    UndefinedArithmetic,
    /// Safe slices must contain the bytes the C header promises are readable.
    ShortPayload,
}

pub enum Input<'a> {
    BindWork,
    ResetSession,
    UpdateLastSeen {
        node: u8,
        rssi: i8,
    },
    NotePresence(u8),
    Discover(u8),
    Demote {
        leader: u8,
        term: u32,
        identity: IdentityValue,
    },
    Rejoin {
        leader: u8,
        term: u32,
        identity: IdentityValue,
    },
    Packet {
        header: Header,
        payload: &'a [u8],
    },
    ScanWork,
    JoinWork,
    CheckPeerTimeouts,
    CoordinatorTimeout,
    Timer(TimerToken),
}

fn padded(address: [u8; 5]) -> [u8; 6] {
    [
        address[0], address[1], address[2], address[3], address[4], 0,
    ]
}
fn identity(value: Identity) -> IdentityValue {
    IdentityValue {
        address_len: value.address_len,
        address: value.address,
    }
}
fn peer_snapshot(peer: Peer) -> pure::Peer {
    pure::Peer {
        present: true,
        active: peer.active,
        announced: peer.announced,
        node_id: peer.node_id,
        address: padded(peer.address),
        address_len: 5,
    }
}

impl Runtime {
    pub fn membership_scan_timeout_ms(&self) -> u32 {
        3000 + u32::from(u16::from_be_bytes([self.address[3], self.address[4]])) % 501
    }
    pub fn membership_bridge_peer_count(&self) -> u8 {
        self.membership.bridge_peer_count()
    }
    pub(crate) fn membership_snapshot(&self) -> pure::Snapshot {
        let m = self.membership;
        pure::Snapshot {
            state: match m.state {
                State::Idle => pure::State::Idle,
                State::Scanning => pure::State::Scanning,
                State::Joining => pure::State::Joining,
                State::Active => pure::State::Active,
            },
            role: match m.role {
                Role::None => pure::Role::None,
                Role::Coordinator => pure::Role::Coordinator,
                Role::Participant => pure::Role::Participant,
            },
            node_id: m.node_id,
            slot_index: m.slot_index,
            coordinator_id: m.coordinator_id,
            term: self.protocol.term,
            peer_count: m.peer_count,
            participant_membership_known: m.membership_known,
            address_len: 5,
            local_address: padded(self.address),
        }
    }
    pub(crate) fn apply_membership_snapshot(&mut self, s: pure::Snapshot) {
        self.membership = Membership {
            state: match s.state {
                pure::State::Idle => State::Idle,
                pure::State::Scanning => State::Scanning,
                pure::State::Joining => State::Joining,
                pure::State::Active => State::Active,
            },
            role: match s.role {
                pure::Role::None => Role::None,
                pure::Role::Coordinator => Role::Coordinator,
                pure::Role::Participant => Role::Participant,
            },
            node_id: s.node_id,
            slot_index: s.slot_index,
            coordinator_id: s.coordinator_id,
            peer_count: s.peer_count,
            membership_known: s.participant_membership_known,
        };
        self.protocol.term = s.term;
    }
    async fn membership_output_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl MembershipPlatform,
        output: MembershipOutput,
    ) -> bool {
        if ASYNC_SEND {
            p.membership_output_async(output).await
        } else {
            self.me(p, output.into())
        }
    }
    fn me(&mut self, p: &mut impl MembershipPlatform, e: MembershipEffect) -> bool {
        p.dispatch_membership(e, self)
    }
    async fn membership_status_impl<const ASYNC_SEND: bool>(
        &mut self,
        count: u8,
        p: &mut impl MembershipPlatform,
    ) {
        let mut m = self.membership;
        m.peer_count = count;
        self.membership_output_impl::<ASYNC_SEND>(p, MembershipOutput::Status(m))
            .await;
    }
    async fn membership_event_impl<const ASYNC_SEND: bool>(
        &mut self,
        id: u8,
        node: Option<u8>,
        p: &mut impl MembershipPlatform,
    ) {
        self.membership_output_impl::<ASYNC_SEND>(p, MembershipOutput::Event { id, node })
            .await;
    }
    fn membership_cancel(&mut self, timer: Timer, p: &mut impl MembershipPlatform) {
        self.timers[timer as usize] = None;
        p.apply(Effect::Cancel(timer));
    }
    fn adaptive_members(&mut self, p: &mut impl MembershipPlatform) {
        self.me(
            p,
            MembershipEffect::AdaptiveMembers {
                state: self.membership.state,
                local: self.membership.node_id,
                leader: self.membership.coordinator_id,
                term: self.protocol.term,
            },
        );
    }
    fn reset_session_data(&mut self, clear_bitmaps: bool, p: &mut impl MembershipPlatform) {
        self.me(p, MembershipEffect::AdaptiveReset);
        self.session.peers.fill(Peer::default());
        mesh_core::dedupe_reset(Some(&mut self.dedupe));
        mesh_core::dedupe_reset(Some(&mut self.protocol.control_presence_dedupe));
        self.me(p, MembershipEffect::ResetTrackers);
        self.me(p, MembershipEffect::ClearRelay);
        self.session.relay_ring.fill(RelayEntry::default());
        self.session.relay_head = 0;
        self.session.relay_tail = 0;
        self.session.control_ring.fill(RelayEntry::default());
        self.me(p, MembershipEffect::ClearActivity);
        self.session.speaker_deadlines_ms.fill(0);
        self.session.speaker_active_since_ms.fill(0);
        self.me(p, MembershipEffect::ClearGrants);
        if clear_bitmaps {
            self.me(p, MembershipEffect::ClearBitmaps);
            self.session.heard_bitmap = 0;
            self.session.relay_bitmap = 0;
        }
        self.membership.peer_count = 0;
        self.protocol.sync_member_mismatch_since = None;
        self.session.control_head = 0;
        self.session.control_tail = 0;
        self.me(p, MembershipEffect::PurgeTx);
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: false,
                purge: true,
            },
        );
    }
    /// Executes membership logic against this owner, only in explicit simulation.
    /// Existing Runtime::handle/Kernel interfaces remain unchanged until integration.
    /// Boolean is the C packet-handled / coordinator-timeout result (otherwise false).
    pub async fn membership_handle_async(
        &mut self,
        input: Input<'_>,
        p: &mut impl MembershipPlatform,
    ) -> Result<bool, Error> {
        self.membership_handle_impl::<true>(input, p).await
    }
    pub fn membership_handle(
        &mut self,
        input: Input<'_>,
        p: &mut impl MembershipPlatform,
    ) -> Result<bool, Error> {
        crate::corefuture::ready(self.membership_handle_impl::<false>(input, p))
    }
    pub(crate) async fn membership_handle_impl<const ASYNC_SEND: bool>(
        &mut self,
        input: Input<'_>,
        p: &mut impl MembershipPlatform,
    ) -> Result<bool, Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return Err(Error::NotIntegrated);
        }
        match input {
            Input::BindWork => {
                self.me(p, MembershipEffect::BindWork);
            }
            Input::ResetSession => self.reset_session_data(true, p),
            Input::UpdateLastSeen { node, rssi } => {
                self.update_peer_last_seen_impl::<ASYNC_SEND>(node, rssi, p)
                    .await
            }
            Input::NotePresence(node) => self.note_peer_presence(node, p),
            Input::Discover(leader) => self.membership_discover(leader, p),
            Input::Demote {
                leader,
                term,
                identity,
            } => {
                self.membership_change_leader_impl::<ASYNC_SEND>(false, leader, term, identity, p)
                    .await
            }
            Input::Rejoin {
                leader,
                term,
                identity,
            } => {
                self.membership_change_leader_impl::<ASYNC_SEND>(true, leader, term, identity, p)
                    .await
            }
            Input::Packet { header, payload } => {
                return self
                    .membership_process_rx_packet_impl::<ASYNC_SEND>(header, payload, p)
                    .await
            }
            Input::ScanWork => {
                self.membership_scan_work_handler_impl::<ASYNC_SEND>(p)
                    .await
            }
            Input::JoinWork => {
                self.membership_join_work_handler_impl::<ASYNC_SEND>(p)
                    .await
            }
            Input::CheckPeerTimeouts => {
                self.membership_check_peer_timeouts_impl::<ASYNC_SEND>(p)
                    .await?
            }
            Input::CoordinatorTimeout => {
                return Ok(self
                    .membership_handle_coordinator_timeout_impl::<ASYNC_SEND>(p)
                    .await)
            }
            Input::Timer(token) => {
                let i = token.timer as usize;
                if self.timers[i] != Some(token) || p.now_us() < token.deadline_us {
                    return Ok(false);
                }
                // Status remains owned by the existing kernel, not this adapter.
                if token.timer == Timer::Status {
                    return Ok(false);
                }
                self.timers[i] = None;
                match token.timer {
                    Timer::Scan => {
                        self.membership_scan_work_handler_impl::<ASYNC_SEND>(p)
                            .await
                    }
                    Timer::Join => {
                        self.membership_join_work_handler_impl::<ASYNC_SEND>(p)
                            .await
                    }
                    Timer::Status => {}
                }
            }
        }
        Ok(false)
    }
    pub(crate) async fn update_peer_last_seen_impl<const ASYNC_SEND: bool>(
        &mut self,
        node: u8,
        rssi: i8,
        p: &mut impl MembershipPlatform,
    ) {
        if let Some(i) = self
            .session
            .peers
            .iter()
            .position(|v| v.active && v.node_id == node)
        {
            self.session.peers[i].last_seen_ms = p.uptime_ms();
            self.session.peers[i].rssi_dbm = rssi;
            if !self.session.peers[i].announced && self.membership.role == Role::Coordinator {
                self.me(p, MembershipEffect::ResetTracker(node));
                self.session.peers[i].announced = true;
                self.membership.peer_count = self.membership.peer_count.wrapping_add(1);
                self.membership_status_impl::<ASYNC_SEND>(self.membership_bridge_peer_count(), p)
                    .await;
                self.membership_event_impl::<ASYNC_SEND>(2, Some(node), p)
                    .await;
                self.me(p, MembershipEffect::SlotMap);
            }
        }
    }
    pub(crate) fn note_peer_presence(&mut self, node: u8, p: &mut impl MembershipPlatform) {
        if let Some(peer) = self
            .session
            .peers
            .iter_mut()
            .find(|v| v.active && v.node_id == node)
        {
            peer.last_seen_ms = p.uptime_ms();
        }
    }
    fn membership_discover(&mut self, leader: u8, p: &mut impl MembershipPlatform) {
        if self.membership.state != State::Scanning || !mesh_core::node_id_valid(leader) {
            return;
        }
        self.membership.state = State::Joining;
        self.membership.coordinator_id = leader;
        self.protocol.join_attempts = 0;
        self.membership_cancel(Timer::Scan, p);
        self.schedule(Timer::Join, 0, p);
    }
    async fn membership_change_leader_impl<const ASYNC_SEND: bool>(
        &mut self,
        rejoin: bool,
        leader: u8,
        term: u32,
        identity: IdentityValue,
        p: &mut impl MembershipPlatform,
    ) {
        let expected = if rejoin {
            Role::Participant
        } else {
            Role::Coordinator
        };
        if self.membership.state != State::Active || self.membership.role != expected {
            return;
        }
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: false,
                purge: false,
            },
        );
        self.me(p, MembershipEffect::TdmaStop);
        self.reset_session_data(rejoin, p);
        self.membership_cancel(Timer::Status, p);
        self.membership.role = Role::None;
        self.membership.state = State::Joining;
        self.membership.node_id = 0;
        self.membership.slot_index = -1;
        self.membership.coordinator_id = leader;
        if rejoin {
            self.membership.membership_known = false;
        }
        self.protocol.term = term;
        self.protocol.leader_identity = identity;
        self.protocol.join_attempts = 0;
        if rejoin {
            self.membership_event_impl::<ASYNC_SEND>(5, None, p).await;
        }
        self.membership_status_impl::<ASYNC_SEND>(if rejoin { 255 } else { 0 }, p)
            .await;
        self.schedule(Timer::Join, 0, p);
    }
    async fn process_join_impl<const ASYNC_SEND: bool>(
        &mut self,
        h: Header,
        payload: &[u8],
        p: &mut impl MembershipPlatform,
    ) -> Result<(), Error> {
        if self.membership.role != Role::Coordinator
            || h.src_id != 0
            || usize::from(h.payload_len) != JoinV3::SIZE
        {
            return Ok(());
        }
        let join = JoinV3::decode(payload.get(..JoinV3::SIZE).ok_or(Error::ShortPayload)?)
            .map_err(|_| Error::ShortPayload)?;
        let origin = identity(join.origin);
        if join.capabilities & 2 == 0
            || origin.address_len != 5
            || origin == self.protocol.leader_identity
            || identity(join.target) != self.protocol.leader_identity
        {
            return Ok(());
        }
        let address: [u8; 5] = origin.address[..5].try_into().expect("five-byte prefix");
        let mut id = 0;
        let mut slot = -1;
        let mut new_member = false;
        if let Some(i) = self
            .session
            .peers
            .iter()
            .position(|v| v.active && v.address == address)
        {
            id = self.session.peers[i].node_id;
            slot = self.session.peers[i].slot_index;
            if !self.me(
                p,
                MembershipEffect::ControlSequence {
                    id,
                    packet_type: 15,
                    seq: h.seq,
                },
            ) {
                return Ok(());
            }
            self.session.peers[i].last_seen_ms = p.uptime_ms();
        }
        if id == 0 {
            let mut occupied = mesh_core::node_bit(self.membership.node_id);
            // Match the C promoted u32 shift then u8 narrowing, including slots 8..31.
            let mut slots = 1u32
                .checked_shl(self.membership.slot_index as u32)
                .ok_or(Error::UndefinedArithmetic)? as u8;
            for peer in &self.session.peers {
                if peer.active {
                    occupied |= mesh_core::node_bit(peer.node_id);
                    if (0..8).contains(&peer.slot_index) {
                        slots |= 1 << peer.slot_index;
                    }
                }
            }
            id = mesh_core::first_free_node_id(occupied);
            for s in 0..8 {
                if slots & (1 << s) == 0 {
                    slot = s;
                    break;
                }
            }
            if slot < 0 {
                return Ok(());
            }
            if id != 0
                && !self.me(
                    p,
                    MembershipEffect::BindMember {
                        id,
                        identity: Some(origin),
                    },
                )
            {
                return Ok(());
            }
            if id != 0 {
                if let Some(i) = self.session.peers.iter().position(|v| !v.active) {
                    // Reused inactive entries retain all fields not written by C.
                    let peer = &mut self.session.peers[i];
                    peer.node_id = id;
                    peer.slot_index = slot;
                    peer.address = address;
                    peer.last_seen_ms = p.uptime_ms();
                    peer.active = true;
                    peer.announced = true;
                    self.membership.peer_count = self.membership.peer_count.wrapping_add(1);
                    new_member = true;
                }
            }
        }
        if id == 0
            || !self.me(
                p,
                MembershipEffect::BindMember {
                    id,
                    identity: Some(origin),
                },
            )
        {
            return Ok(());
        }
        if new_member {
            self.me(
                p,
                MembershipEffect::ControlSequence {
                    id,
                    packet_type: 15,
                    seq: h.seq,
                },
            );
        }
        self.me(p, MembershipEffect::ResetTracker(id));
        if new_member {
            self.membership_status_impl::<ASYNC_SEND>(self.membership.peer_count, p)
                .await;
            self.membership_event_impl::<ASYNC_SEND>(2, Some(id), p)
                .await;
        }
        self.adaptive_members(p);
        self.me(
            p,
            MembershipEffect::JoinAck {
                id,
                slot: slot as u8,
                address,
            },
        );
        self.me(p, MembershipEffect::SlotMap);
        Ok(())
    }
    async fn process_join_ack_impl<const ASYNC_SEND: bool>(
        &mut self,
        h: Header,
        payload: &[u8],
        p: &mut impl MembershipPlatform,
    ) -> Result<(), Error> {
        let valid = usize::from(h.payload_len) == JoinAckV3::SIZE;
        let mut ack_event = pure::JoinAck::default();
        if valid {
            let ack = JoinAckV3::decode(payload.get(..JoinAckV3::SIZE).ok_or(Error::ShortPayload)?)
                .map_err(|_| Error::ShortPayload)?;
            if ack.term != self.protocol.term || ack.target.address_len != 5 {
                return Ok(());
            }
            ack_event = pure::JoinAck {
                assigned_id: ack.assigned_id,
                slot_index: ack.slot_index,
                coordinator_id: ack.coordinator_id,
                target_address: padded(ack.target.address[..5].try_into().expect("prefix")),
                address_len: 5,
            };
        }
        let result = pure::reduce(
            Some(&self.membership_snapshot()),
            Some(&pure::Event {
                sender_id: h.src_id,
                payload_valid: valid,
                data: pure::EventData::JoinAck(ack_event),
            }),
        );
        if result.action != pure::Action::ActivateParticipant {
            return Ok(());
        }
        self.apply_membership_snapshot(result.next);
        self.me(p, MembershipEffect::ResetTrackers);
        self.me(p, MembershipEffect::PurgeTx);
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: false,
                purge: true,
            },
        );
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: true,
                purge: false,
            },
        );
        self.membership_cancel(Timer::Join, p);
        self.me(
            p,
            MembershipEffect::TdmaStart {
                slot: self.membership.slot_index,
                coordinator: false,
            },
        );
        self.adaptive_members(p);
        if let Some(i) = self.session.peers.iter().position(|v| !v.active) {
            let peer = &mut self.session.peers[i];
            peer.node_id = self.membership.coordinator_id;
            peer.slot_index = 0;
            peer.address
                .copy_from_slice(&self.protocol.leader_identity.address[..5]);
            peer.active = true;
            peer.announced = true;
            peer.last_seen_ms = p.uptime_ms();
        }
        self.protocol.last_sync_time = p.uptime_ms() as u32;
        self.schedule(Timer::Status, 1000, p);
        self.membership_status_impl::<ASYNC_SEND>(255, p).await;
        self.membership_event_impl::<ASYNC_SEND>(1, None, p).await;
        Ok(())
    }
    async fn process_leave_impl<const ASYNC_SEND: bool>(
        &mut self,
        h: Header,
        payload: &[u8],
        p: &mut impl MembershipPlatform,
    ) -> Result<(), Error> {
        if self.membership.role != Role::Coordinator || usize::from(h.payload_len) != LeaveNrf::SIZE
        {
            return Ok(());
        }
        let leave = LeaveNrf::decode(payload.get(..LeaveNrf::SIZE).ok_or(Error::ShortPayload)?)
            .map_err(|_| Error::ShortPayload)?;
        let index = self
            .session
            .peers
            .iter()
            .position(|v| v.active && v.node_id == h.src_id);
        let result = pure::reduce(
            Some(&self.membership_snapshot()),
            Some(&pure::Event {
                sender_id: h.src_id,
                payload_valid: true,
                data: pure::EventData::Leave(pure::Leave {
                    identity: pure::LeaveIdentity::Address,
                    sender_address: padded(leave.sender_addr),
                    address_len: 5,
                    peer: index
                        .map(|i| peer_snapshot(self.session.peers[i]))
                        .unwrap_or_default(),
                }),
            }),
        );
        if h.src_id == self.membership.node_id {
            return Ok(());
        }
        if let Some(i) = index.filter(|_| result.action == pure::Action::RemovePeer) {
            self.session.peers[i].active = false;
            self.me(
                p,
                MembershipEffect::BindMember {
                    id: result.affected_node_id,
                    identity: None,
                },
            );
            self.apply_membership_snapshot(result.next);
            self.me(p, MembershipEffect::UpdateGrants);
            mesh_core::dedupe_purge_node(Some(&mut self.dedupe), result.affected_node_id);
            self.me(p, MembershipEffect::ResetTracker(result.affected_node_id));
            if result.effects & pure::EFFECT_REPORT_PEER_LEFT != 0 {
                self.membership_status_impl::<ASYNC_SEND>(self.membership_bridge_peer_count(), p)
                    .await;
                self.membership_event_impl::<ASYNC_SEND>(3, Some(result.affected_node_id), p)
                    .await;
            }
        }
        if result.effects & pure::EFFECT_PUBLISH_SLOT_MAP != 0 {
            self.me(p, MembershipEffect::SlotMap);
        }
        Ok(())
    }
    async fn membership_process_rx_packet_impl<const ASYNC_SEND: bool>(
        &mut self,
        h: Header,
        payload: &[u8],
        p: &mut impl MembershipPlatform,
    ) -> Result<bool, Error> {
        // Header version, RSSI and RF timestamp are intentionally not checked in C.
        match h.packet_type {
            15 => self.process_join_impl::<ASYNC_SEND>(h, payload, p).await?,
            16 => {
                self.process_join_ack_impl::<ASYNC_SEND>(h, payload, p)
                    .await?
            }
            4 => self.process_leave_impl::<ASYNC_SEND>(h, payload, p).await?,
            2 | 3 | 5 | 6 | 7 | 8 | 11 | 12 | 17 => {}
            _ => return Ok(false),
        }
        Ok(true)
    }
    async fn membership_scan_work_handler_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl MembershipPlatform,
    ) {
        if self.membership.state != State::Scanning {
            return;
        }
        self.membership.role = Role::Coordinator;
        self.membership.node_id = 1;
        self.membership.slot_index = 0;
        self.membership.coordinator_id = 1;
        self.membership.state = State::Active;
        self.me(p, MembershipEffect::ResetTrackers);
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: true,
                purge: false,
            },
        );
        self.session.peers[0].node_id = 1;
        // Address query remains an ordered boundary; Platform::GetAddress returns bytes.
        if let crate::platform::Reply::Address(address) = p.apply(Effect::GetAddress) {
            self.session.peers[0].address = address;
        }
        self.session.peers[0].slot_index = 0;
        self.session.peers[0].active = true;
        self.session.peers[0].announced = true;
        self.session.peers[0].last_seen_ms = p.uptime_ms();
        self.membership.peer_count = 0;
        self.protocol.term = 1;
        self.protocol.leader_identity = IdentityValue {
            address_len: 5,
            address: padded(self.address),
        };
        self.adaptive_members(p);
        self.me(
            p,
            MembershipEffect::TdmaStart {
                slot: 0,
                coordinator: true,
            },
        );
        self.me(p, MembershipEffect::ClockSource(true));
        self.schedule(Timer::Status, 1000, p);
        self.membership_status_impl::<ASYNC_SEND>(self.membership_bridge_peer_count(), p)
            .await;
        self.membership_event_impl::<ASYNC_SEND>(4, None, p).await;
        self.membership_event_impl::<ASYNC_SEND>(1, None, p).await;
    }
    #[cfg(test)]
    pub(crate) fn mesh_log(&mut self, args: fmt::Arguments<'_>, p: &mut impl MembershipPlatform) {
        crate::corefuture::ready(self.mesh_log_impl::<false>(args, p))
    }
    pub(crate) async fn mesh_log_impl<const ASYNC_SEND: bool>(
        &mut self,
        args: fmt::Arguments<'_>,
        p: &mut impl MembershipPlatform,
    ) {
        struct Buffer {
            bytes: [u8; 128],
            len: usize,
        }
        impl Write for Buffer {
            fn write_str(&mut self, text: &str) -> fmt::Result {
                let n = text.len().min(127 - self.len);
                self.bytes[self.len..self.len + n].copy_from_slice(&text.as_bytes()[..n]);
                self.len += n;
                Ok(())
            }
        }
        let mut buf = Buffer {
            bytes: [0; 128],
            len: 0,
        };
        let _ = buf.write_fmt(args);
        if buf.len > 0 && self.me(p, MembershipEffect::LogInitialized) {
            self.membership_output_impl::<ASYNC_SEND>(
                p,
                MembershipOutput::Log {
                    bytes: buf.bytes,
                    len: buf.len as u8,
                },
            )
            .await;
        }
    }
    async fn membership_join_work_handler_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl MembershipPlatform,
    ) {
        if self.membership.state != State::Joining {
            return;
        }
        if self.protocol.join_attempts < 10 {
            if ASYNC_SEND {
                p.dispatch_membership_async(MembershipEffect::JoinRequest, self)
                    .await;
            } else {
                self.me(p, MembershipEffect::JoinRequest);
            }
            self.protocol.join_attempts += 1;
            let attempts = self.protocol.join_attempts;
            self.mesh_log_impl::<ASYNC_SEND>(format_args!("MESH: JOIN attempt {attempts}/10"), p)
                .await;
            self.schedule(Timer::Join, 500, p);
        } else {
            let delay = self.membership_scan_timeout_ms();
            self.mesh_log_impl::<ASYNC_SEND>(
                format_args!("MESH: JOIN timeout, rescanning for {delay} ms"),
                p,
            )
            .await;
            self.membership.state = State::Scanning;
            self.membership.role = Role::None;
            self.membership.node_id = 0;
            self.membership.slot_index = -1;
            self.membership.coordinator_id = 0;
            self.me(p, MembershipEffect::ResetTrackers);
            self.membership_status_impl::<ASYNC_SEND>(self.membership_bridge_peer_count(), p)
                .await;
            self.schedule(Timer::Scan, delay, p);
        }
    }
    async fn membership_check_peer_timeouts_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl MembershipPlatform,
    ) -> Result<(), Error> {
        let now = p.uptime_ms();
        let mut changed = false;
        for i in 0..8 {
            let peer = self.session.peers[i];
            if !peer.active
                || self.membership.role != Role::Coordinator
                || peer.node_id == self.membership.node_id
                || peer.node_id == self.membership.coordinator_id
            {
                continue;
            }
            if now
                .checked_sub(peer.last_seen_ms)
                .ok_or(Error::UndefinedArithmetic)?
                > 3000
            {
                self.session.peers[i].active = false;
                self.me(
                    p,
                    MembershipEffect::BindMember {
                        id: peer.node_id,
                        identity: None,
                    },
                );
                mesh_core::dedupe_purge_node(Some(&mut self.dedupe), peer.node_id);
                self.me(p, MembershipEffect::ResetTracker(peer.node_id));
                if peer.announced && self.membership.peer_count > 0 {
                    self.membership.peer_count -= 1;
                }
                changed = true;
                if peer.announced {
                    self.membership_event_impl::<ASYNC_SEND>(3, Some(peer.node_id), p)
                        .await;
                }
            }
        }
        if changed && self.membership.role == Role::Coordinator {
            self.me(p, MembershipEffect::UpdateGrants);
            self.me(p, MembershipEffect::SlotMap);
        }
        Ok(())
    }
    async fn membership_handle_coordinator_timeout_impl<const ASYNC_SEND: bool>(
        &mut self,
        p: &mut impl MembershipPlatform,
    ) -> bool {
        if self.membership.role != Role::Participant
            || (p.uptime_ms() as u32).wrapping_sub(self.protocol.last_sync_time) <= 5000
        {
            return false;
        }
        if ASYNC_SEND {
            p.prepare_ingress_boundary(self.ingress_admission_gate());
        }
        self.mesh_log_impl::<ASYNC_SEND>(format_args!("MESH: Coordinator lost, rescanning"), p)
            .await;
        self.me(
            p,
            MembershipEffect::Ingress {
                enabled: false,
                purge: false,
            },
        );
        self.membership_event_impl::<ASYNC_SEND>(5, None, p).await;
        self.me(p, MembershipEffect::TdmaStop);
        self.reset_session_data(true, p);
        self.membership.state = State::Scanning;
        self.membership.role = Role::None;
        self.membership.node_id = 0;
        self.membership.slot_index = -1;
        self.membership.coordinator_id = 0;
        self.membership_status_impl::<ASYNC_SEND>(self.membership_bridge_peer_count(), p)
            .await;
        self.schedule(Timer::Scan, self.membership_scan_timeout_ms(), p);
        true
    }
}
