//! Safe semantic port of `shared/mesh_membership.[ch]`.
//! Raw lengths are retained and validated before address prefixes are accessed.

use crate::core::{MAX_ACTIVE_SPEAKERS, MAX_NODES, SlotMap, node_bit, node_id_valid};

pub const MAX_ADDRESS_LEN: usize = 6;
pub const EFFECT_REPORT_PEER_LEFT: u32 = 1;
pub const EFFECT_PUBLISH_SLOT_MAP: u32 = 2;
pub const EFFECT_REPORT_LOCAL_WIN: u32 = 4;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle,
    Scanning,
    Joining,
    Active,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Role {
    #[default]
    None,
    Coordinator,
    Participant,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub state: State,
    pub role: Role,
    pub node_id: u8,
    pub slot_index: i8,
    pub coordinator_id: u8,
    pub term: u32,
    pub peer_count: u8,
    pub participant_membership_known: bool,
    pub local_address: [u8; MAX_ADDRESS_LEN],
    pub address_len: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Peer {
    pub present: bool,
    pub active: bool,
    pub announced: bool,
    pub node_id: u8,
    pub address: [u8; MAX_ADDRESS_LEN],
    pub address_len: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sync {
    pub coordinator_address: [u8; MAX_ADDRESS_LEN],
    pub address_len: u8,
    pub term: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JoinAck {
    pub assigned_id: u8,
    pub slot_index: u8,
    pub coordinator_id: u8,
    pub target_address: [u8; MAX_ADDRESS_LEN],
    pub address_len: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LeaveIdentity {
    #[default]
    Invalid,
    Legacy,
    Address,
    /// Unrecognized C identity values fail identity matching.
    Unknown(u32),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Leave {
    pub identity: LeaveIdentity,
    pub sender_address: [u8; MAX_ADDRESS_LEN],
    pub address_len: u8,
    pub peer: Peer,
}

/// Tagged semantic payload, not a C union or a wire representation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventData {
    Sync(Sync),
    JoinAck(JoinAck),
    SlotMap(SlotMap),
    Leave(Leave),
    /// Use for event tags outside the four recognized C values (0..=3).
    Unknown(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub sender_id: u8,
    pub payload_valid: bool,
    pub data: EventData,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Action {
    #[default]
    Ignore,
    DiscoverCoordinator,
    AcceptSync,
    DemoteCoordinator,
    KeepCoordinator,
    ActivateParticipant,
    ApplySlotMap,
    RemovePeer,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MembershipResult {
    pub next: Snapshot,
    pub action: Action,
    pub effects: u32,
    pub affected_node_id: u8,
}

fn address_length_valid(len: u8) -> bool {
    len > 0 && usize::from(len) <= MAX_ADDRESS_LEN
}

// Membership deliberately does not use core::slot_map_valid: its slot-zero,
// coordinator placement, and trailing-slot rules are different.
fn parse_slot_map(map: &SlotMap, local: u8, coordinator: u8) -> Option<(i8, u8)> {
    if !node_id_valid(local)
        || !node_id_valid(coordinator)
        || map.slot_count == 0
        || usize::from(map.slot_count) > MAX_NODES
        || usize::from(map.active_speaker_count) > MAX_ACTIVE_SPEAKERS
    {
        return None;
    }
    let count = usize::from(map.slot_count);
    let (mut members, mut member_count, mut local_slot) = (0u8, 0u8, -1i8);
    for (slot, &id) in map.slot_ids[..count].iter().enumerate() {
        if id == 0 {
            continue;
        }
        let bit = node_bit(id);
        if bit == 0 || members & bit != 0 {
            return None;
        }
        members |= bit;
        member_count += 1;
        if id == local {
            local_slot = slot as i8;
        }
    }
    if map.slot_ids[count - 1] == 0
        || local_slot < 0
        || member_count == 0
        || members & node_bit(coordinator) == 0
        || map.slot_ids[count..].iter().any(|&id| id != 0)
    {
        return None;
    }
    let mut speakers = 0u8;
    for &id in &map.active_speaker_ids[..usize::from(map.active_speaker_count)] {
        let bit = node_bit(id);
        if bit == 0 || members & bit == 0 || speakers & bit != 0 {
            return None;
        }
        speakers |= bit;
    }
    Some((local_slot, member_count))
}

/// Either missing argument returns an entirely zero-valued semantic result.
pub fn reduce(current: Option<&Snapshot>, event: Option<&Event>) -> MembershipResult {
    let (Some(current), Some(event)) = (current, event) else {
        return MembershipResult::default();
    };
    let mut result = MembershipResult {
        next: *current,
        ..MembershipResult::default()
    };
    match &event.data {
        EventData::Sync(sync) => {
            if !event.payload_valid
                || !node_id_valid(event.sender_id)
                || !address_length_valid(sync.address_len)
                || sync.address_len != current.address_len
            {
                return result;
            }
            let len = usize::from(sync.address_len);
            let remote = &sync.coordinator_address[..len];
            let local = &current.local_address[..len];
            if remote.iter().all(|&b| b == 0)
                || remote.iter().all(|&b| b == u8::MAX)
                || remote == local
            {
                return result;
            }
            if current.state == State::Scanning {
                result.next.state = State::Joining;
                result.next.coordinator_id = event.sender_id;
                result.action = Action::DiscoverCoordinator;
            } else if current.state == State::Active && current.role == Role::Participant {
                if event.sender_id == current.coordinator_id
                    && (current.term == 0 || sync.term == current.term)
                {
                    result.action = Action::AcceptSync;
                }
            } else if current.state == State::Active
                && current.role == Role::Coordinator
                && current.term == 0
                && sync.term == 0
            {
                if remote < local {
                    result.next.state = State::Joining;
                    result.next.role = Role::None;
                    result.next.node_id = 0;
                    result.next.slot_index = -1;
                    result.next.coordinator_id = event.sender_id;
                    result.next.peer_count = 0;
                    result.next.participant_membership_known = false;
                    result.action = Action::DemoteCoordinator;
                } else {
                    result.action = Action::KeepCoordinator;
                    result.effects = EFFECT_REPORT_LOCAL_WIN;
                }
            }
        }
        EventData::JoinAck(ack) => {
            if current.state == State::Joining
                && event.payload_valid
                && address_length_valid(ack.address_len)
                && ack.address_len == current.address_len
                && ack.target_address[..usize::from(ack.address_len)]
                    == current.local_address[..usize::from(ack.address_len)]
                && node_id_valid(ack.assigned_id)
                && usize::from(ack.slot_index) < MAX_NODES
                && node_id_valid(ack.coordinator_id)
                && ack.assigned_id != ack.coordinator_id
                && event.sender_id == ack.coordinator_id
                && (current.coordinator_id == 0 || current.coordinator_id == ack.coordinator_id)
            {
                result.next.state = State::Active;
                result.next.role = Role::Participant;
                result.next.node_id = ack.assigned_id;
                result.next.slot_index = ack.slot_index as i8;
                result.next.coordinator_id = ack.coordinator_id;
                result.next.peer_count = 1;
                result.next.participant_membership_known = false;
                result.action = Action::ActivateParticipant;
            }
        }
        EventData::SlotMap(map) => {
            if current.state == State::Active
                && current.role == Role::Participant
                && event.sender_id == current.coordinator_id
                && event.payload_valid
            {
                if let Some((slot, count)) =
                    parse_slot_map(map, current.node_id, current.coordinator_id)
                {
                    result.next.slot_index = slot;
                    result.next.peer_count = count - 1;
                    result.next.participant_membership_known = true;
                    result.action = Action::ApplySlotMap;
                }
            }
        }
        EventData::Leave(leave) => {
            let peer = &leave.peer;
            if !node_id_valid(event.sender_id)
                || event.sender_id == current.node_id
                || !event.payload_valid
                || !peer.present
                || !peer.active
                || peer.node_id != event.sender_id
            {
                return result;
            }
            let matches = leave.identity == LeaveIdentity::Legacy
                || (leave.identity == LeaveIdentity::Address
                    && address_length_valid(peer.address_len)
                    && leave.address_len == peer.address_len
                    && leave.sender_address[..usize::from(peer.address_len)]
                        == peer.address[..usize::from(peer.address_len)]);
            if !matches {
                return result;
            }
            result.action = Action::RemovePeer;
            result.affected_node_id = event.sender_id;
            if current.role == Role::Coordinator {
                result.effects |= EFFECT_PUBLISH_SLOT_MAP;
            }
            if peer.announced {
                if result.next.peer_count > 0 {
                    result.next.peer_count -= 1;
                }
                result.effects |= EFFECT_REPORT_PEER_LEFT;
            }
        }
        EventData::Unknown(_) => {}
    }
    result
}

/// Apply an already-validated handover. Failure leaves every field unchanged.
/// Terms use ordinary unsigned ordering, not serial/wrapping ordering.
pub fn apply_handover(
    snapshot: Option<&mut Snapshot>,
    old_leader: u8,
    new_leader: u8,
    term: u32,
) -> bool {
    let Some(snapshot) = snapshot else {
        return false;
    };
    if snapshot.state != State::Active
        || !node_id_valid(snapshot.node_id)
        || snapshot.coordinator_id != old_leader
        || !node_id_valid(new_leader)
        || new_leader == old_leader
        || snapshot.slot_index < 0
        || snapshot.slot_index as usize >= MAX_NODES
        || term <= snapshot.term
    {
        return false;
    }
    snapshot.coordinator_id = new_leader;
    snapshot.term = term;
    snapshot.role = if snapshot.node_id == new_leader {
        Role::Coordinator
    } else {
        Role::Participant
    };
    true
}
