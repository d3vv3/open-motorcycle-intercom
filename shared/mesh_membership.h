#ifndef OMI_MESH_MEMBERSHIP_H
#define OMI_MESH_MEMBERSHIP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mesh_protocol_defs.h"

#define MESH_MEMBERSHIP_MAX_ADDRESS_LEN 6U

typedef struct {
    mesh_state_t state;
    mesh_role_t role;
    uint8_t node_id;
    int8_t slot_index;
    uint8_t coordinator_id;
    uint8_t peer_count;
    bool participant_membership_known;
    uint8_t local_address[MESH_MEMBERSHIP_MAX_ADDRESS_LEN];
    uint8_t address_len;
} mesh_membership_snapshot_t;

typedef struct {
    bool present;
    bool active;
    bool announced;
    uint8_t node_id;
    uint8_t address[MESH_MEMBERSHIP_MAX_ADDRESS_LEN];
    uint8_t address_len;
} mesh_membership_peer_t;

typedef enum {
    MESH_MEMBERSHIP_EVENT_SYNC,
    MESH_MEMBERSHIP_EVENT_JOIN_ACK,
    MESH_MEMBERSHIP_EVENT_SLOT_MAP,
    MESH_MEMBERSHIP_EVENT_LEAVE,
} mesh_membership_event_type_t;

typedef enum {
    MESH_MEMBERSHIP_LEAVE_INVALID,
    MESH_MEMBERSHIP_LEAVE_LEGACY,
    MESH_MEMBERSHIP_LEAVE_ADDRESS,
} mesh_membership_leave_identity_t;

typedef struct {
    mesh_membership_event_type_t type;
    uint8_t sender_id;
    bool payload_valid;
    union {
        struct {
            uint8_t coordinator_address[MESH_MEMBERSHIP_MAX_ADDRESS_LEN];
            uint8_t address_len;
        } sync;
        struct {
            uint8_t assigned_id;
            uint8_t slot_index;
            uint8_t coordinator_id;
            uint8_t target_address[MESH_MEMBERSHIP_MAX_ADDRESS_LEN];
            uint8_t address_len;
        } join_ack;
        mesh_slot_map_payload_t slot_map;
        struct {
            mesh_membership_leave_identity_t identity;
            uint8_t sender_address[MESH_MEMBERSHIP_MAX_ADDRESS_LEN];
            uint8_t address_len;
            mesh_membership_peer_t peer;
        } leave;
    } data;
} mesh_membership_event_t;

typedef enum {
    MESH_MEMBERSHIP_ACTION_IGNORE,
    MESH_MEMBERSHIP_ACTION_DISCOVER_COORDINATOR,
    MESH_MEMBERSHIP_ACTION_ACCEPT_SYNC,
    MESH_MEMBERSHIP_ACTION_DEMOTE_COORDINATOR,
    MESH_MEMBERSHIP_ACTION_KEEP_COORDINATOR,
    MESH_MEMBERSHIP_ACTION_ACTIVATE_PARTICIPANT,
    MESH_MEMBERSHIP_ACTION_APPLY_SLOT_MAP,
    MESH_MEMBERSHIP_ACTION_REMOVE_PEER,
} mesh_membership_action_t;

typedef enum {
    MESH_MEMBERSHIP_EFFECT_NONE = 0,
    MESH_MEMBERSHIP_EFFECT_REPORT_PEER_LEFT = 1U << 0,
    MESH_MEMBERSHIP_EFFECT_PUBLISH_SLOT_MAP = 1U << 1,
    MESH_MEMBERSHIP_EFFECT_REPORT_LOCAL_WIN = 1U << 2,
} mesh_membership_effect_t;

typedef struct {
    mesh_membership_snapshot_t next;
    mesh_membership_action_t action;
    uint32_t effects;
    uint8_t affected_node_id;
} mesh_membership_result_t;

mesh_membership_result_t
mesh_membership_reduce(const mesh_membership_snapshot_t *current,
                       const mesh_membership_event_t *event);

#endif /* OMI_MESH_MEMBERSHIP_H */
