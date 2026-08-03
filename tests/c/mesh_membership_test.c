#include "shared/mesh_membership.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t local_address[5] = {0x10, 0x20, 0x30, 0x40, 0x50};

static mesh_membership_snapshot_t snapshot(mesh_state_t state, mesh_role_t role)
{
    mesh_membership_snapshot_t value = {
        .state = state,
        .role = role,
        .node_id = role == MESH_ROLE_COORDINATOR ? 1U : 3U,
        .slot_index = role == MESH_ROLE_COORDINATOR ? 0 : 2,
        .coordinator_id = 1U,
        .peer_count = 3U,
        .participant_membership_known = true,
        .address_len = sizeof(local_address),
    };
    memcpy(value.local_address, local_address, sizeof(local_address));
    return value;
}

static mesh_membership_event_t sync_event(uint8_t sender, const uint8_t address[5])
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_SYNC,
        .sender_id = sender,
        .payload_valid = true,
    };
    event.data.sync.address_len = 5U;
    memcpy(event.data.sync.coordinator_address, address, 5U);
    return event;
}

static void test_sync_discovery_and_filtering(void)
{
    mesh_membership_snapshot_t scanning = snapshot(MESH_STATE_SCANNING, MESH_ROLE_NONE);
    scanning.node_id = 0U;
    scanning.slot_index = -1;
    scanning.coordinator_id = 0U;
    const uint8_t remote_address[5] = {0x20, 0x20, 0x30, 0x40, 0x50};
    const uint8_t zero_address[5] = {0};
    const uint8_t broadcast_address[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    mesh_membership_event_t event = sync_event(2U, remote_address);
    mesh_membership_result_t result = mesh_membership_reduce(&scanning, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_DISCOVER_COORDINATOR);
    assert(result.next.state == MESH_STATE_JOINING && result.next.coordinator_id == 2U);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event.payload_valid = false;
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event.payload_valid = true;
    event.sender_id = 0U;
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event.sender_id = MESH_MAX_NODES + 1U;
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = sync_event(2U, zero_address);
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = sync_event(2U, broadcast_address);
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = sync_event(2U, local_address);
    assert(mesh_membership_reduce(&scanning, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);

    mesh_membership_snapshot_t participant = snapshot(MESH_STATE_ACTIVE,
                                                       MESH_ROLE_PARTICIPANT);
    event = sync_event(participant.coordinator_id, remote_address);
    result = mesh_membership_reduce(&participant, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_ACCEPT_SYNC);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);
    event.sender_id = 2U;
    assert(mesh_membership_reduce(&participant, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = sync_event(participant.coordinator_id, zero_address);
    assert(mesh_membership_reduce(&participant, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
}

static void test_dual_coordinator_resolution(void)
{
    mesh_membership_snapshot_t coordinator = snapshot(MESH_STATE_ACTIVE,
                                                       MESH_ROLE_COORDINATOR);
    const uint8_t lower[5] = {0x10, 0x20, 0x30, 0x40, 0x4f};
    const uint8_t higher[5] = {0x10, 0x20, 0x30, 0x40, 0x51};
    const uint8_t zero[5] = {0};
    const uint8_t broadcast[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    mesh_membership_event_t event = sync_event(2U, lower);
    mesh_membership_result_t result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_DEMOTE_COORDINATOR);
    assert(result.next.state == MESH_STATE_JOINING && result.next.role == MESH_ROLE_NONE);
    assert(result.next.node_id == 0U && result.next.slot_index == -1);
    assert(result.next.coordinator_id == 2U && result.next.peer_count == 0U);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event = sync_event(2U, higher);
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_KEEP_COORDINATOR);
    assert(result.next.role == MESH_ROLE_COORDINATOR && result.next.node_id == 1U);
    assert((result.effects & MESH_MEMBERSHIP_EFFECT_REPORT_LOCAL_WIN) != 0U);

    event = sync_event(2U, local_address);
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event = sync_event(2U, zero);
    assert(mesh_membership_reduce(&coordinator, &event).action ==
           MESH_MEMBERSHIP_ACTION_IGNORE);
    event = sync_event(2U, broadcast);
    assert(mesh_membership_reduce(&coordinator, &event).action ==
           MESH_MEMBERSHIP_ACTION_IGNORE);
}

static mesh_membership_event_t valid_ack(void)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_JOIN_ACK,
        .sender_id = 1U,
        .payload_valid = true,
        .data.join_ack = {
            .assigned_id = 3U,
            .slot_index = 2U,
            .coordinator_id = 1U,
            .address_len = sizeof(local_address),
        },
    };
    memcpy(event.data.join_ack.target_address, local_address, sizeof(local_address));
    return event;
}

static void test_join_ack_activation(void)
{
    mesh_membership_snapshot_t joining = snapshot(MESH_STATE_JOINING, MESH_ROLE_NONE);
    joining.node_id = 0U;
    joining.slot_index = -1;
    joining.participant_membership_known = false;
    mesh_membership_event_t event = valid_ack();
    mesh_membership_result_t result = mesh_membership_reduce(&joining, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_ACTIVATE_PARTICIPANT);
    assert(result.next.state == MESH_STATE_ACTIVE &&
           result.next.role == MESH_ROLE_PARTICIPANT);
    assert(result.next.node_id == 3U && result.next.slot_index == 2);
    assert(result.next.peer_count == 1U && !result.next.participant_membership_known);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event.data.join_ack.target_address[4] ^= 1U;
    assert(mesh_membership_reduce(&joining, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = valid_ack();
    mesh_membership_snapshot_t stale = joining;
    stale.state = MESH_STATE_SCANNING;
    assert(mesh_membership_reduce(&stale, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event.sender_id = 2U;
    assert(mesh_membership_reduce(&joining, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = valid_ack();
    event.data.join_ack.slot_index = 0U;
    assert(mesh_membership_reduce(&joining, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event.data.join_ack.slot_index = MESH_MAX_NODES;
    assert(mesh_membership_reduce(&joining, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
}

static mesh_membership_event_t valid_slot_map(void)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_SLOT_MAP,
        .sender_id = 1U,
        .payload_valid = true,
    };
    event.data.slot_map.slot_count = MESH_MAX_NODES;
    event.data.slot_map.slot_ids[0] = 1U;
    event.data.slot_map.slot_ids[3] = 3U;
    event.data.slot_map.slot_ids[5] = 5U;
    return event;
}

static void test_slot_map_update(void)
{
    mesh_membership_snapshot_t participant = snapshot(MESH_STATE_ACTIVE,
                                                       MESH_ROLE_PARTICIPANT);
    participant.participant_membership_known = false;
    mesh_membership_event_t event = valid_slot_map();
    mesh_membership_result_t result = mesh_membership_reduce(&participant, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_APPLY_SLOT_MAP);
    assert(result.next.slot_index == 3 && result.next.peer_count == 2U);
    assert(result.next.participant_membership_known);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event.data.slot_map.slot_ids[5] = 3U;
    assert(mesh_membership_reduce(&participant, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
    event = valid_slot_map();
    event.sender_id = 2U;
    assert(mesh_membership_reduce(&participant, &event).action == MESH_MEMBERSHIP_ACTION_IGNORE);
}

static mesh_membership_event_t leave_event(mesh_membership_leave_identity_t identity,
                                            bool announced)
{
    static const uint8_t peer_address[5] = {1U, 2U, 3U, 4U, 5U};
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_LEAVE,
        .sender_id = 4U,
        .payload_valid = true,
        .data.leave = {
            .identity = identity,
            .address_len = sizeof(peer_address),
            .peer = {
                .present = true,
                .active = true,
                .announced = announced,
                .node_id = 4U,
                .address_len = sizeof(peer_address),
            },
        },
    };
    memcpy(event.data.leave.sender_address, peer_address, sizeof(peer_address));
    memcpy(event.data.leave.peer.address, peer_address, sizeof(peer_address));
    return event;
}

static void test_leave_identity_and_count(void)
{
    mesh_membership_snapshot_t coordinator = snapshot(MESH_STATE_ACTIVE,
                                                       MESH_ROLE_COORDINATOR);
    mesh_membership_event_t event = leave_event(MESH_MEMBERSHIP_LEAVE_LEGACY, true);
    mesh_membership_result_t result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_REMOVE_PEER);
    assert(result.next.peer_count == coordinator.peer_count - 1U);
    assert((result.effects & MESH_MEMBERSHIP_EFFECT_REPORT_PEER_LEFT) != 0U);
    assert((result.effects & MESH_MEMBERSHIP_EFFECT_PUBLISH_SLOT_MAP) != 0U);

    mesh_membership_snapshot_t after_leave = result.next;
    event.data.leave.peer.active = false;
    result = mesh_membership_reduce(&after_leave, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.next.peer_count == after_leave.peer_count);

    event = leave_event(MESH_MEMBERSHIP_LEAVE_ADDRESS, true);
    assert(mesh_membership_reduce(&coordinator, &event).action ==
           MESH_MEMBERSHIP_ACTION_REMOVE_PEER);
    event.data.leave.sender_address[4] ^= 1U;
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.next.peer_count == coordinator.peer_count);

    event = leave_event(MESH_MEMBERSHIP_LEAVE_ADDRESS, false);
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_REMOVE_PEER);
    assert(result.next.peer_count == coordinator.peer_count);
    assert((result.effects & MESH_MEMBERSHIP_EFFECT_REPORT_PEER_LEFT) == 0U);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_PUBLISH_SLOT_MAP);

    event = leave_event(MESH_MEMBERSHIP_LEAVE_LEGACY, true);
    event.sender_id = coordinator.node_id;
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event = leave_event(MESH_MEMBERSHIP_LEAVE_ADDRESS, true);
    event.data.leave.sender_address[4] ^= 1U;
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);

    event = leave_event(MESH_MEMBERSHIP_LEAVE_LEGACY, true);
    event.sender_id = MESH_MAX_NODES + 1U;
    result = mesh_membership_reduce(&coordinator, &event);
    assert(result.action == MESH_MEMBERSHIP_ACTION_IGNORE);
    assert(result.effects == MESH_MEMBERSHIP_EFFECT_NONE);
}

int main(void)
{
    test_sync_discovery_and_filtering();
    test_dual_coordinator_resolution();
    test_join_ack_activation();
    test_slot_map_update();
    test_leave_identity_and_count();
    puts("mesh_membership tests passed");
    return 0;
}
