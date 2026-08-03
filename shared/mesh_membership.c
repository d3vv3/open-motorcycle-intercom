#include "mesh_membership.h"

#include <string.h>

static bool node_id_valid(uint8_t node_id)
{
    return node_id > 0U && node_id <= MESH_MAX_NODES;
}

static bool address_length_valid(uint8_t address_len)
{
    return address_len > 0U && address_len <= MESH_MEMBERSHIP_MAX_ADDRESS_LEN;
}

static int address_compare(const uint8_t *a, const uint8_t *b, uint8_t address_len)
{
    for (uint8_t i = 0; i < address_len; i++) {
        if (a[i] != b[i]) {
            return a[i] < b[i] ? -1 : 1;
        }
    }
    return 0;
}

static bool address_all(const uint8_t *address, uint8_t address_len, uint8_t value)
{
    for (uint8_t i = 0; i < address_len; i++) {
        if (address[i] != value) {
            return false;
        }
    }
    return true;
}

static bool join_ack_valid(const mesh_membership_snapshot_t *current,
                           const mesh_membership_event_t *event)
{
    const uint8_t address_len = event->data.join_ack.address_len;
    return current->state == MESH_STATE_JOINING && event->payload_valid &&
           address_length_valid(address_len) && address_len == current->address_len &&
           memcmp(event->data.join_ack.target_address, current->local_address,
                  address_len) == 0 &&
           node_id_valid(event->data.join_ack.assigned_id) &&
           event->data.join_ack.slot_index > 0U &&
           event->data.join_ack.slot_index < MESH_MAX_NODES &&
           node_id_valid(event->data.join_ack.coordinator_id) &&
           event->data.join_ack.assigned_id != event->data.join_ack.coordinator_id &&
           event->sender_id == event->data.join_ack.coordinator_id &&
           (current->coordinator_id == 0U ||
            current->coordinator_id == event->data.join_ack.coordinator_id);
}

static bool slot_map_valid(const mesh_slot_map_payload_t *slot_map, uint8_t local_node_id,
                           uint8_t coordinator_id, int8_t *local_slot,
                           uint8_t *member_count)
{
    uint8_t member_bitmap = 0U;
    int8_t parsed_local_slot = -1;
    uint8_t parsed_member_count = 0U;

    if (!node_id_valid(local_node_id) || !node_id_valid(coordinator_id) ||
        slot_map->slot_count == 0U || slot_map->slot_count > MESH_MAX_NODES ||
        slot_map->slot_ids[0] != coordinator_id ||
        slot_map->active_speaker_count > MESH_MAX_ACTIVE_SPEAKERS) {
        return false;
    }

    for (uint8_t slot = 0; slot < slot_map->slot_count; slot++) {
        uint8_t node_id = slot_map->slot_ids[slot];
        if (node_id == 0U) {
            continue;
        }
        if (!node_id_valid(node_id)) {
            return false;
        }
        uint8_t bit = (uint8_t)(1U << (node_id - 1U));
        if ((member_bitmap & bit) != 0U) {
            return false;
        }
        member_bitmap |= bit;
        parsed_member_count++;
        if (node_id == local_node_id) {
            parsed_local_slot = (int8_t)slot;
        }
    }
    if (parsed_local_slot <= 0) {
        return false;
    }

    uint8_t speaker_bitmap = 0U;
    for (uint8_t i = 0; i < slot_map->active_speaker_count; i++) {
        uint8_t speaker_id = slot_map->active_speaker_ids[i];
        if (!node_id_valid(speaker_id)) {
            return false;
        }
        uint8_t bit = (uint8_t)(1U << (speaker_id - 1U));
        if ((member_bitmap & bit) == 0U || (speaker_bitmap & bit) != 0U) {
            return false;
        }
        speaker_bitmap |= bit;
    }

    *local_slot = parsed_local_slot;
    *member_count = parsed_member_count;
    return true;
}

static mesh_membership_result_t reduce_sync(const mesh_membership_snapshot_t *current,
                                              const mesh_membership_event_t *event)
{
    mesh_membership_result_t result = {.next = *current};
    if (!event->payload_valid || !node_id_valid(event->sender_id)) {
        return result;
    }

    uint8_t address_len = event->data.sync.address_len;
    if (!address_length_valid(address_len) || address_len != current->address_len ||
        address_all(event->data.sync.coordinator_address, address_len, 0U) ||
        address_all(event->data.sync.coordinator_address, address_len, UINT8_MAX) ||
        memcmp(event->data.sync.coordinator_address, current->local_address,
               address_len) == 0) {
        return result;
    }

    if (current->state == MESH_STATE_SCANNING) {
        result.next.state = MESH_STATE_JOINING;
        result.next.coordinator_id = event->sender_id;
        result.action = MESH_MEMBERSHIP_ACTION_DISCOVER_COORDINATOR;
        return result;
    }

    if (current->state == MESH_STATE_ACTIVE && current->role == MESH_ROLE_PARTICIPANT) {
        if (event->sender_id == current->coordinator_id) {
            result.action = MESH_MEMBERSHIP_ACTION_ACCEPT_SYNC;
        }
        return result;
    }

    if (current->state != MESH_STATE_ACTIVE || current->role != MESH_ROLE_COORDINATOR ||
        !address_length_valid(current->address_len) ||
        event->data.sync.address_len != current->address_len) {
        return result;
    }

    int comparison = address_compare(event->data.sync.coordinator_address,
                                     current->local_address, current->address_len);
    if (comparison < 0) {
        result.next.state = MESH_STATE_JOINING;
        result.next.role = MESH_ROLE_NONE;
        result.next.node_id = 0U;
        result.next.slot_index = -1;
        result.next.coordinator_id = event->sender_id;
        result.next.peer_count = 0U;
        result.next.participant_membership_known = false;
        result.action = MESH_MEMBERSHIP_ACTION_DEMOTE_COORDINATOR;
    } else {
        result.action = MESH_MEMBERSHIP_ACTION_KEEP_COORDINATOR;
        if (comparison > 0) {
            result.effects = MESH_MEMBERSHIP_EFFECT_REPORT_LOCAL_WIN;
        }
    }
    return result;
}

static mesh_membership_result_t reduce_join_ack(const mesh_membership_snapshot_t *current,
                                                 const mesh_membership_event_t *event)
{
    mesh_membership_result_t result = {.next = *current};
    if (!join_ack_valid(current, event)) {
        return result;
    }

    result.next.state = MESH_STATE_ACTIVE;
    result.next.role = MESH_ROLE_PARTICIPANT;
    result.next.node_id = event->data.join_ack.assigned_id;
    result.next.slot_index = (int8_t)event->data.join_ack.slot_index;
    result.next.coordinator_id = event->data.join_ack.coordinator_id;
    result.next.peer_count = 1U;
    result.next.participant_membership_known = false;
    result.action = MESH_MEMBERSHIP_ACTION_ACTIVATE_PARTICIPANT;
    return result;
}

static mesh_membership_result_t reduce_slot_map(const mesh_membership_snapshot_t *current,
                                                 const mesh_membership_event_t *event)
{
    mesh_membership_result_t result = {.next = *current};
    int8_t local_slot;
    uint8_t member_count;
    if (current->state != MESH_STATE_ACTIVE || current->role != MESH_ROLE_PARTICIPANT ||
        event->sender_id != current->coordinator_id || !event->payload_valid ||
        !slot_map_valid(&event->data.slot_map, current->node_id,
                        current->coordinator_id, &local_slot, &member_count)) {
        return result;
    }

    result.next.slot_index = local_slot;
    result.next.peer_count = (uint8_t)(member_count - 1U);
    result.next.participant_membership_known = true;
    result.action = MESH_MEMBERSHIP_ACTION_APPLY_SLOT_MAP;
    return result;
}

static mesh_membership_result_t reduce_leave(const mesh_membership_snapshot_t *current,
                                              const mesh_membership_event_t *event)
{
    mesh_membership_result_t result = {.next = *current};
    if (!node_id_valid(event->sender_id) || event->sender_id == current->node_id) {
        return result;
    }

    const mesh_membership_peer_t *peer = &event->data.leave.peer;
    if (!event->payload_valid || !peer->present || !peer->active ||
        peer->node_id != event->sender_id) {
        return result;
    }

    bool identity_matches = event->data.leave.identity == MESH_MEMBERSHIP_LEAVE_LEGACY;
    if (event->data.leave.identity == MESH_MEMBERSHIP_LEAVE_ADDRESS &&
        address_length_valid(peer->address_len) &&
        event->data.leave.address_len == peer->address_len) {
        identity_matches = memcmp(event->data.leave.sender_address, peer->address,
                                  peer->address_len) == 0;
    }
    if (!identity_matches) {
        return result;
    }

    result.action = MESH_MEMBERSHIP_ACTION_REMOVE_PEER;
    result.affected_node_id = event->sender_id;
    if (current->role == MESH_ROLE_COORDINATOR) {
        result.effects |= MESH_MEMBERSHIP_EFFECT_PUBLISH_SLOT_MAP;
    }
    if (peer->announced) {
        if (result.next.peer_count > 0U) {
            result.next.peer_count--;
        }
        result.effects |= MESH_MEMBERSHIP_EFFECT_REPORT_PEER_LEFT;
    }
    return result;
}

mesh_membership_result_t
mesh_membership_reduce(const mesh_membership_snapshot_t *current,
                       const mesh_membership_event_t *event)
{
    mesh_membership_result_t result = {0};
    if (current == NULL || event == NULL) {
        return result;
    }

    switch (event->type) {
    case MESH_MEMBERSHIP_EVENT_SYNC:
        return reduce_sync(current, event);
    case MESH_MEMBERSHIP_EVENT_JOIN_ACK:
        return reduce_join_ack(current, event);
    case MESH_MEMBERSHIP_EVENT_SLOT_MAP:
        return reduce_slot_map(current, event);
    case MESH_MEMBERSHIP_EVENT_LEAVE:
        return reduce_leave(current, event);
    default:
        result.next = *current;
        return result;
    }
}
