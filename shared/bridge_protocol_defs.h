#ifndef OMI_BRIDGE_PROTOCOL_DEFS_H
#define OMI_BRIDGE_PROTOCOL_DEFS_H

#include <stddef.h>
#include <stdint.h>

#include "mesh_protocol_defs.h"

#define BRIDGE_PROTOCOL_VERSION    6
#define BRIDGE_PROTOCOL_VERSION_V3 3
#define BRIDGE_PROTOCOL_VERSION_V2 2
#define BRIDGE_STATUS_V2_MARKER    0xA5
#define BRIDGE_STATUS_V2_LENGTH    8
#define BRIDGE_PEER_COUNT_UNKNOWN  0xFF

#define BRIDGE_PKT_AUDIO      0x01
#define BRIDGE_PKT_STATUS     0x02
#define BRIDGE_PKT_MESH_EVENT 0x03
#define BRIDGE_PKT_CONTROL    0x04
#define BRIDGE_PKT_LOG        0x05
#define BRIDGE_PKT_AUDIO_V2   0x06

typedef enum {
    BRIDGE_COMMAND_MESH_START = 0x01,
    BRIDGE_COMMAND_MESH_STOP = 0x02,
    BRIDGE_COMMAND_STATUS = 0x03,
    BRIDGE_COMMAND_AUDIO_PAUSE = 0x04,
    BRIDGE_COMMAND_AUDIO_RESUME = 0x05,
} bridge_command_t;

typedef enum {
    BRIDGE_EVENT_MESH_READY = 0x01,
    BRIDGE_EVENT_PEER_JOINED = 0x02,
    BRIDGE_EVENT_PEER_LEFT = 0x03,
    BRIDGE_EVENT_BECAME_COORDINATOR = 0x04,
    BRIDGE_EVENT_SYNC_LOST = 0x05,
    BRIDGE_EVENT_MESH_STOPPED = 0x06,
    BRIDGE_EVENT_COMMAND_ACK = 0x07,
    BRIDGE_EVENT_AUDIO_V2_READY = 0x08,
} bridge_event_t;

typedef enum {
    BRIDGE_MESH_STATE_IDLE = 0,
    BRIDGE_MESH_STATE_SCANNING = 1,
    BRIDGE_MESH_STATE_JOINING = 2,
    BRIDGE_MESH_STATE_ACTIVE = 3,
} bridge_mesh_state_t;

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t generation;
    uint8_t talk_channel; /* START group; STOP/PAUSE/RESUME ignore this byte. */
} bridge_command_payload_t;

static inline int bridge_mesh_command_valid(const bridge_command_payload_t *payload, size_t len)
{
    return payload != NULL && len == sizeof(*payload) &&
           (payload->command == BRIDGE_COMMAND_MESH_STOP ||
            payload->command == BRIDGE_COMMAND_AUDIO_PAUSE ||
            payload->command == BRIDGE_COMMAND_AUDIO_RESUME ||
            (payload->command == BRIDGE_COMMAND_MESH_START &&
             mesh_channel_valid(payload->talk_channel)));
}

/* A producer that started before a pause/resume boundary cannot enqueue afterward. */
static inline int bridge_audio_epoch_accept(uint32_t arrival_epoch, uint32_t current_epoch,
                                            int local_audio_enabled)
{
    return local_audio_enabled && arrival_epoch == current_epoch;
}

/* One atomic word carries a coherent request; zero means no pending command. */
static inline uint32_t bridge_mesh_request_pack(uint8_t command, uint8_t generation,
                                                uint8_t talk_channel)
{
    return (uint32_t)command | ((uint32_t)generation << 8) | ((uint32_t)talk_channel << 16);
}

static inline bridge_command_payload_t bridge_mesh_request_unpack(uint32_t request)
{
    return (bridge_command_payload_t){
        .command = (uint8_t)request,
        .generation = (uint8_t)(request >> 8),
        .talk_channel = (uint8_t)(request >> 16),
    };
}

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t generation;
    int8_t result;
} bridge_command_ack_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t role;
    uint8_t peer_count;
    uint8_t node_id;
    uint8_t version;
    uint8_t mesh_state;
    int8_t slot_index;
    uint8_t coordinator_id;
    uint8_t marker;
    uint8_t audio_codec;
    uint8_t audio_frame_ms;
} bridge_status_payload_t;

static inline int bridge_audio_supports_lc3(uint8_t version, uint8_t codec, uint8_t frame_ms)
{
    return version == BRIDGE_PROTOCOL_VERSION && codec == MESH_AUDIO_V2_CODEC_LC3 &&
           frame_ms == MESH_FRAME_MS;
}

_Static_assert(sizeof(bridge_command_payload_t) == 3, "bridge command wire size changed");
_Static_assert(sizeof(bridge_command_ack_payload_t) == 3, "bridge ACK wire size changed");
_Static_assert(sizeof(bridge_status_payload_t) == 10, "bridge status wire size changed");
_Static_assert(offsetof(bridge_status_payload_t, marker) == 7, "bridge status marker offset changed");
_Static_assert(offsetof(bridge_status_payload_t, audio_codec) == BRIDGE_STATUS_V2_LENGTH,
               "bridge status codec offset changed");
_Static_assert(offsetof(bridge_status_payload_t, audio_frame_ms) == 9,
               "bridge status frame duration offset changed");
_Static_assert(offsetof(bridge_status_payload_t, role) == 0, "legacy status role prefix changed");
_Static_assert(offsetof(bridge_status_payload_t, peer_count) == 1,
               "legacy status peer prefix changed");
_Static_assert(offsetof(bridge_status_payload_t, node_id) == 2,
               "legacy status node prefix changed");
_Static_assert(BRIDGE_PKT_AUDIO_V2 == 0x06, "bridge audio v2 packet ID changed");
_Static_assert(BRIDGE_EVENT_AUDIO_V2_READY == 0x08, "bridge audio v2 event ID changed");

#endif
