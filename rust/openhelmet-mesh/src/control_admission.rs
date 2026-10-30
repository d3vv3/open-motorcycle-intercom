//! Canonical adaptive-control discriminator. This is not semantic authorization:
//! discovery admission below is task-side only; stateful commits remain authoritative.

use crate::adaptive::{self as a, Identity};

/// Copied task-local discovery context, not a second discovery owner.
/// State: 0 other, 1 active, 2 joining. Coordinator: canonical byte boolean.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct JoinContext {
    pub discovered_term: u32,
    pub relay_ms: u32,
    pub state: u8,
    pub coordinator: u8,
    pub coordinator_id: u8,
    pub local: [u8; 6],
    pub coordinator_mac: [u8; 6],
    pub relay: [u8; 6],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum JoinAdmission {
    Rejected = 0,
    Join = 1,
    Ack = 2,
}

impl JoinContext {
    pub const fn valid(&self) -> bool {
        self.state <= 2 && self.coordinator <= 1
    }
}

/// Binding is obtained directly from the existing Owner, with nonzero-seen
/// freshness, before this call. No semantic JOIN transaction is committed here.
pub fn admit_join(
    context: JoinContext,
    source: u8,
    origin: Identity,
    target: Identity,
    coordinator_identity: Identity,
    relay_bound: bool,
) -> JoinAdmission {
    if !context.valid()
        || source != 0
        || origin.address_len != 6
        || !a::identity_valid(Some(&origin))
        || !relay_bound
        || (context.state == 1 && (target.address_len != 6 || target != coordinator_identity))
        || (target.address_len != 0
            && (!a::identity_valid(Some(&target))
                || (context.coordinator == 1 && target.address != context.local)))
    {
        return JoinAdmission::Rejected;
    }
    JoinAdmission::Join
}

#[allow(clippy::too_many_arguments)]
pub fn admit_ack(
    context: JoinContext,
    source: u8,
    mac: [u8; 6],
    target: Identity,
    coordinator: u8,
    term: u32,
    now: u32,
    relayed: bool,
) -> JoinAdmission {
    if !context.valid()
        || context.state != 2
        || source != context.coordinator_id
        || coordinator != source
        || term == 0
        || term != context.discovered_term
        || target.address_len != 6
        || target.address != context.local
        || if relayed {
            context.relay_ms == 0
                || now.wrapping_sub(context.relay_ms) > a::EXPIRE_MS
                || mac != context.relay
        } else {
            mac != context.coordinator_mac
        }
    {
        JoinAdmission::Rejected
    } else {
        JoinAdmission::Ack
    }
}

/// Admission effects only: 0 continue, 1 consumed/rejected, 2 invoke SYNC,
/// 3 invoke SYNC and write relay refresh, 4 membership admitted. Not commit authority.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(C)]
pub struct DiscoveryDecision {
    pub action: u8,
    pub members: u8,
    pub relay_id: u8,
    pub reserved: u8,
    pub refresh_ms: u32,
}

pub fn admit_sync(
    context: &JoinContext,
    source: u8,
    mac: &[u8; 6],
    sync: &a::Sync,
    relayed: bool,
    cold: bool,
    now: u32,
) -> DiscoveryDecision {
    let discovery = context.state == 2 || context.state == 3;
    if !discovery && !cold {
        return DiscoveryDecision::default();
    }
    let mut result = DiscoveryDecision {
        action: 1,
        ..Default::default()
    };
    if context.state > 3
        || context.coordinator > 1
        || sync.leader_id != source
        || sync.phase_us >= 20_000
        || !(1..=8).contains(&sync.member_count)
        || sync.leader.address_len != 6
        || if discovery {
            sync.term == 0
                || !a::identity_valid(Some(&sync.leader))
                || (relayed && sync.relay_depth != 1)
                || (!relayed && sync.leader.address != *mac)
        } else {
            sync.term != 1 || sync.leader.address == context.local || sync.leader.address != *mac
        }
    {
        return result;
    }
    result.action = 2;
    if context.state == 2
        && relayed
        && source == context.coordinator_id
        && sync.leader.address == context.coordinator_mac
        && context.relay_ms != 0
        && *mac == context.relay
    {
        // No expiry check here: a matching joining SYNC renews the discovery path.
        result.action = 3;
        result.refresh_ms = now;
    }
    result
}

/// Incoming map must contain a distinct relay, even when the old Owner binds it.
#[allow(clippy::too_many_arguments)]
pub fn admit_membership(
    context: &JoinContext,
    source: u8,
    local: u8,
    leader: u8,
    mac: &[u8; 6],
    membership: &a::Membership,
    members: u8,
    via_known: bool,
    relayed: bool,
    now: u32,
) -> DiscoveryDecision {
    let mut result = DiscoveryDecision {
        action: 1,
        ..Default::default()
    };
    if !context.valid() || context.state != 1 {
        return result;
    }
    let mapped = (1..=8)
        .find(|&id| {
            let identity = membership.identities[usize::from(id - 1)];
            id != source
                && id != local
                && members & crate::core::node_bit(id) != 0
                && identity.address_len == 6
                && identity.address == *mac
        })
        .unwrap_or(0);
    if (via_known && relayed && mapped == 0)
        || (!via_known
            && (!relayed
                || source != leader
                || context.relay_ms == 0
                || now.wrapping_sub(context.relay_ms) > a::EXPIRE_MS
                || *mac != context.relay
                || mapped == 0))
    {
        return result;
    }
    result.action = 4;
    result.members = members;
    if !via_known {
        result.relay_id = mapped;
    }
    result
}

/// Active task-side effects. Stateful transition commits remain authoritative.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ActiveDecision {
    Rejected = 0,
    Sync = 1,
    PreparedSync = 2,
    Ack = 3,
}

pub fn admit_active_sync(
    state: &a::Adaptive,
    source: u8,
    sync: &a::Sync,
    relayed: bool,
) -> ActiveDecision {
    let (prepared, _, _, _, _, pending) = state.transition();
    let candidate = prepared && source == pending.candidate_id;
    if sync.leader_id != source
        || sync.leader.address_len != 6
        || sync.leader != state.member_identity(source)
        || sync.relay_depth != u8::from(relayed)
    {
        return ActiveDecision::Rejected;
    }
    if candidate {
        // Term/member/phase and pending-target checks belong to transition_sync.
        return ActiveDecision::PreparedSync;
    }
    if !a::control_authorized(Some(state), Control::Sync as u8, source)
        || !(1..=8).contains(&sync.member_count)
        || sync.phase_us >= 20_000
        || sync.term != state.session().3
    {
        return ActiveDecision::Rejected;
    }
    ActiveDecision::Sync
}

/// Post-handler authority for an already committed SYNC. Unlike admission this
/// does not depend on prepared/pending state, which the transition has consumed.
/// Call with the current Owner and the original copied payload after callbacks.
pub fn sync_continuation_authorized(state: &a::Adaptive, source: u8, sync: &a::Sync) -> bool {
    a::control_authorized(Some(state), Control::Sync as u8, source)
        && sync.leader_id == source
        && sync.leader.address_len == 6
        && sync.leader == state.member_identity(source)
        && sync.term == state.session().3
}

pub struct ActiveAck {
    pub target: Identity,
    pub coordinator_id: u8,
    pub assigned_id: u8,
    pub slot_index: u8,
    pub term: u32,
}

pub fn admit_active_ack(
    state: &a::Adaptive,
    local: &[u8; 6],
    source: u8,
    ack: &ActiveAck,
) -> ActiveDecision {
    if !a::control_authorized(Some(state), Control::JoinAck as u8, source)
        || ack.coordinator_id != source
        || ack.term != state.session().3
        || !a::identity_valid(Some(&ack.target))
        || ack.target.address_len != 6
        || crate::core::node_bit(ack.assigned_id) == 0
        || ack.assigned_id == source
        || ack.slot_index >= 8
        || (ack.assigned_id == state.session().0 && ack.target.address != *local)
    {
        ActiveDecision::Rejected
    } else {
        ActiveDecision::Ack
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Control {
    SlotMap = 0x06,
    SpeakerGrant = 0x09,
    SpeakerRelease = 0x0a,
    Topology = 0x0e,
    Join = 0x0f,
    JoinAck = 0x10,
    Sync = 0x11,
    HandoverPrepare = 0x12,
    HandoverAck = 0x13,
    HandoverCommit = 0x14,
    HandoverCancel = 0x15,
    Membership = 0x16,
    SpeakerRequest = 0x17,
}

impl Control {
    pub const fn from_wire(kind: u8) -> Option<Self> {
        Some(match kind {
            0x06 => Self::SlotMap,
            0x09 => Self::SpeakerGrant,
            0x0a => Self::SpeakerRelease,
            0x0e => Self::Topology,
            0x0f => Self::Join,
            0x10 => Self::JoinAck,
            0x11 => Self::Sync,
            0x12 => Self::HandoverPrepare,
            0x13 => Self::HandoverAck,
            0x14 => Self::HandoverCommit,
            0x15 => Self::HandoverCancel,
            0x16 => Self::Membership,
            0x17 => Self::SpeakerRequest,
            _ => return None,
        })
    }

    pub const fn expected_size(self) -> u16 {
        match self {
            Self::Join | Self::JoinAck => 15,
            Self::Sync => 20,
            Self::Topology => 47,
            Self::Membership => 79,
            Self::SpeakerRequest => 13,
            Self::SlotMap => 14,
            Self::SpeakerGrant => 5,
            Self::SpeakerRelease => 3,
            Self::HandoverAck => 11,
            Self::HandoverPrepare | Self::HandoverCommit | Self::HandoverCancel => 25,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Admission {
    Unknown,
    Rejected,
    Envelope(Control),
}

/// Only adaptive controls share this flag mask/TTL policy. In particular this
/// must not harden STATUS, KEEPALIVE or audio as a side effect of extraction.
pub const fn classify(kind: u8, size: u16, flags: u8, ttl: u8) -> Admission {
    let Some(control) = Control::from_wire(kind) else {
        return Admission::Unknown;
    };
    if size != control.expected_size()
        || flags & !2 != 0
        || ttl != (if flags & 2 != 0 { 1 } else { 2 })
    {
        Admission::Rejected
    } else {
        Admission::Envelope(control)
    }
}
