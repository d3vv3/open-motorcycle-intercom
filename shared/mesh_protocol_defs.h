/**
 * @file mesh_protocol_defs.h
 * @brief Single source of truth for the OMI on-air mesh protocol.
 *
 * These definitions are shared verbatim by both transports:
 *   - ESP-NOW build  (components/mesh/include/mesh.h)
 *   - nRF52840/ESB   (nrf_mesh/include/mesh_protocol.h)
 *
 * Only definitions that MUST be byte-identical on the wire live here. Anything
 * that legitimately differs per transport (e.g. the coordinator/peer address
 * width: 6-byte WiFi MAC for ESP-NOW vs 5-byte ESB pipe address) stays in the
 * per-platform header. Do not add platform-specific fields to this file.
 *
 * Plain C only: no esp_err.h / Zephyr includes, and no extern "C" (the
 * including header provides linkage). Include it from both platform headers.
 */

#ifndef OMI_MESH_PROTOCOL_DEFS_H
#define OMI_MESH_PROTOCOL_DEFS_H

#include <stddef.h>
#include <stdint.h>
#include "mesh_channel.h"

/* ============================================================================
 * Protocol constants (wire-visible)
 * ============================================================================ */

#define MESH_MAX_NODES                  8
#define MESH_FRAME_MS                   20   /* TDMA frame duration */
#define MESH_SLOT_MS                    2    /* TDMA slot duration */
#define MESH_GUARD_US                   500  /* Guard time between slots */
#define MESH_SYNC_INTERVAL_FRAMES       10   /* SYNC broadcast cadence */
#define MESH_NODE_TIMEOUT_MS            3000 /* Drop peer after this silence */
#define MESH_KEEPALIVE_INTERVAL_MS      500  /* KEEPALIVE cadence */
#define MESH_PROTOCOL_VERSION           0x05
#define MESH_CONTROL_MAX_PACKET_SIZE    209U
#define MESH_MAX_OPUS_BYTES             64
#define MESH_LC3_FRAME_BYTES            48
#define MESH_E2E_SEQUENCE_BYTES         2
#define MESH_MAX_AUDIO_PAYLOAD          (MESH_MAX_OPUS_BYTES + MESH_E2E_SEQUENCE_BYTES)
#define MESH_AUDIO_V2_CODEC_OPUS        0x01
#define MESH_AUDIO_V2_CODEC_LC3         0x02
#define MESH_AUDIO_CODEC_OPUS            0x01
#define MESH_AUDIO_CODEC_LC3             0x02
#define MESH_CAP_OPUS                    0x01
#define MESH_CAP_LC3                     0x02
#define MESH_AUDIO_V2_FRAME_MS          20
#define MESH_AUDIO_V2_FIXED_HEADER_SIZE 8
#define MESH_AUDIO_V2_MAX_FRAME_BYTES   64
#define MESH_AUDIO_V2_MAX_FRAME_DATA    (3 * MESH_AUDIO_V2_MAX_FRAME_BYTES)
#define MESH_AUDIO_V2_MAX_BUNDLE_SIZE                                                              \
    (MESH_AUDIO_V2_FIXED_HEADER_SIZE + MESH_AUDIO_V2_MAX_FRAME_DATA)
#define MESH_AUDIO_V2_MAX_PACKET_SIZE (9 + MESH_AUDIO_V2_MAX_BUNDLE_SIZE)
#define MESH_MAX_ACTIVE_SPEAKERS      2 /* Concurrent relay-granted speakers */
#define MESH_AUDIO_TTL_DEFAULT        2 /* Default relay TTL */

/* Header control flags */
#define MESH_FLAG_RELAY_REQUEST   0x01
#define MESH_FLAG_RELAYED         0x02
#define MESH_FLAG_SPEAKER_GRANTED 0x04

/* Audio payload flags */
#define MESH_AUDIO_FLAG_ACTIVE               0x01
#define MESH_AUDIO_FLAG_RELAYED              0x02
#define MESH_AUDIO_V2_FLAG_CURRENT_ACTIVE    0x01
#define MESH_AUDIO_V2_FLAG_PREVIOUS1_PRESENT 0x02
#define MESH_AUDIO_V2_FLAG_PREVIOUS1_ACTIVE  0x04
#define MESH_AUDIO_V2_FLAG_PREVIOUS2_PRESENT 0x08
#define MESH_AUDIO_V2_FLAG_PREVIOUS2_ACTIVE  0x10
#define MESH_AUDIO_V2_FLAG_RELAYED           0x20
#define MESH_AUDIO_V2_FLAG_MASK              0x3F

/* ============================================================================
 * Enums
 * ============================================================================ */

typedef enum {
    MESH_ROLE_NONE = 0,    /* Not connected to mesh */
    MESH_ROLE_COORDINATOR, /* Time master / coordinator */
    MESH_ROLE_PARTICIPANT, /* Time slave / participant */
} mesh_role_t;

typedef enum {
    MESH_STATE_IDLE = 0, /* Not started */
    MESH_STATE_SCANNING, /* Listening for existing mesh */
    MESH_STATE_JOINING,  /* Sending JOIN requests */
    MESH_STATE_ACTIVE,   /* Connected and active */
} mesh_state_t;

typedef enum {
    MESH_PKT_AUDIO = 0x01,           /* Opus audio data */
    MESH_PKT_JOIN = 0x02,            /* Request to join mesh */
    MESH_PKT_JOIN_ACK = 0x03,        /* Join response with assigned ID */
    MESH_PKT_LEAVE = 0x04,           /* Graceful leave */
    MESH_PKT_SYNC = 0x05,            /* TDMA timing sync */
    MESH_PKT_SLOT_MAP = 0x06,        /* Slot assignment broadcast */
    MESH_PKT_STATUS = 0x07,          /* Battery / health */
    MESH_PKT_KEEPALIVE = 0x08,       /* Presence check */
    MESH_PKT_SPEAKER_GRANT = 0x09,   /* Relay grant broadcast */
    MESH_PKT_SPEAKER_RELEASE = 0x0A, /* Relay grant release */
    MESH_PKT_JOIN_V2 = 0x0B,         /* Identity-bearing JOIN (nRF/ESB) */
    MESH_PKT_JOIN_ACK_V2 = 0x0C,     /* Identity-targeted JOIN_ACK (nRF/ESB) */
    MESH_PKT_AUDIO_V2 = 0x0D,        /* Redundant LC3 audio bundle */
    MESH_PKT_TOPOLOGY = 0x0E,
    MESH_PKT_JOIN_V3 = 0x0F,
    MESH_PKT_JOIN_ACK_V3 = 0x10,
    MESH_PKT_SYNC_V3 = 0x11,
    MESH_PKT_HANDOVER_PREPARE = 0x12,
    MESH_PKT_HANDOVER_ACK = 0x13,
    MESH_PKT_HANDOVER_COMMIT = 0x14,
    MESH_PKT_HANDOVER_CANCEL = 0x15,
    MESH_PKT_MEMBERSHIP_V3 = 0x16, /* Authoritative complete ID/identity snapshot */
    MESH_PKT_SPEAKER_REQUEST = 0x17, /* Pre-grant activity, one relay allowed */
} mesh_pkt_type_t;

/* ============================================================================
 * Packet header (9 bytes). Version 5 peers reject older packets before parsing payloads.
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t version;      /* Protocol version */
    uint8_t type;         /* Packet type (mesh_pkt_type_t) */
    uint8_t src_id;       /* Source node ID (0 = unassigned) */
    uint8_t seq;          /* Sequence number */
    uint8_t ttl;          /* Relay time-to-live */
    uint8_t flags;        /* Control flags */
    uint8_t talk_channel; /* Logical talk group (1-3), not RF frequency */
    uint16_t payload_len; /* Payload length in bytes */
} mesh_header_t;

static inline int mesh_header_accepts_channel(const mesh_header_t *header, uint8_t local_channel)
{
    return header != NULL && header->version == MESH_PROTOCOL_VERSION &&
           mesh_channel_valid(local_channel) && header->talk_channel == local_channel;
}

/* ============================================================================
 * Payload structures (wire-identical across transports)
 *
 * Legacy mesh_sync_payload_t remains transport-specific. SYNC_V3 below uses
 * a padded identity so it is byte-identical on both transports.
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t codec;                        /* Legacy codec ID */
    uint8_t frame_ms;                     /* Frame duration (20) */
    uint8_t stream_id;                    /* Stream identifier */
    uint8_t audio_flags;                  /* Audio activity flags */
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD]; /* Opus encoded data */
} mesh_audio_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t capabilities; /* Node capabilities bitmap */
    uint8_t reserved;
} mesh_join_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t assigned_id;    /* Assigned node ID (1-8) */
    uint8_t slot_index;     /* Assigned TDMA slot */
    uint8_t coordinator_id; /* Current coordinator ID */
} mesh_join_ack_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t battery_pct; /* Battery percentage (0-100) */
    uint8_t reserved;
} mesh_keepalive_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t slot_count;                                   /* Last occupied slot + 1; holes valid */
    uint8_t slot_ids[MESH_MAX_NODES];                     /* Node ID per slot (0 = empty) */
    uint8_t active_speaker_count;                         /* Number of granted speakers */
    uint8_t active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS]; /* Granted speaker IDs */
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS];        /* Relay bitmap per speaker */
} mesh_slot_map_payload_t;

/* Version 5 control: multi-byte integers use little-endian wire order on both
 * radios. Address bytes beyond address_len must be zero. Source ID 0 requires
 * the JOIN origin identity; forwarded headers retain original src/seq. */
typedef struct __attribute__((packed)) {
    uint8_t address_len;
    uint8_t address[6];
} mesh_wire_identity_t;

typedef struct __attribute__((packed)) {
    uint32_t term;
    uint32_t revision; /* per leader/term, uint32 serial ordering */
    uint8_t leader_id;
    mesh_slot_map_payload_t slot_map;
    mesh_wire_identity_t identities[MESH_MAX_NODES]; /* indexed by node ID - 1 */
} mesh_membership_v3_payload_t;

typedef struct __attribute__((packed)) {
    mesh_wire_identity_t origin;
    uint8_t capabilities;
    mesh_wire_identity_t target;
} mesh_join_v3_payload_t;

typedef struct __attribute__((packed)) {
    mesh_wire_identity_t target;
    uint8_t capabilities;
    uint8_t assigned_id;
    uint8_t slot_index;
    uint8_t coordinator_id;
    uint32_t term;
} mesh_join_ack_v3_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t report_seq; /* independent monotonic reporting sequence */
    mesh_wire_identity_t origin;
    uint8_t direct_mask; /* original RF observations only */
    int8_t rssi_dbm[MESH_MAX_NODES];
    uint8_t quality[MESH_MAX_NODES]; /* 0..100 percent, 255 unknown */
    uint8_t leader_id;
    uint32_t term;
    mesh_slot_map_payload_t slot_map;
} mesh_topology_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t term;
    uint32_t request_seq;
    uint32_t frame_counter;
    uint8_t active; /* 0 release, 1 request */
} mesh_speaker_request_payload_t;

typedef struct __attribute__((packed)) {
    mesh_wire_identity_t leader;
    uint8_t leader_id;
    uint8_t member_count; /* Origin leader's joined count; relay preserves unchanged */
    uint32_t term;
    uint32_t frame_counter;
    uint16_t phase_us; /* 0..19999, sampled immediately before transmission */
    uint8_t relay_depth; /* 0 direct, 1 qualified relay */
} mesh_sync_v3_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t old_leader_id;
    uint8_t candidate_id;
    uint32_t next_term;
    uint32_t switch_frame;
    uint8_t members;
    mesh_slot_map_payload_t slot_map;
} mesh_handover_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t old_leader_id;
    uint8_t candidate_id;
    uint32_t next_term;
    uint32_t switch_frame;
    uint8_t acknowledger_id;
} mesh_handover_ack_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t battery_pct;     /* Battery percentage (0-100, 255=unknown) */
    int8_t rssi_dbm;         /* Last measured RSSI (dBm) */
    uint8_t peer_count;      /* Number of active peers */
    uint8_t fw_version;      /* Firmware version byte */
    int8_t temperature_c;    /* Temperature in Celsius (127=unknown) */
    uint8_t heard_bitmap;    /* Bitmap of source IDs heard recently */
    uint8_t relay_bitmap;    /* Bitmap of source IDs relayed recently */
    uint8_t active_speakers; /* Count of active/granted speakers */
} mesh_status_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t speaker_count;                         /* Number of granted speakers */
    uint8_t speaker_ids[MESH_MAX_ACTIVE_SPEAKERS]; /* Granted speaker IDs */
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS]; /* Relay bitmap per speaker */
} mesh_speaker_grant_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t speaker_count;                         /* Number of released speakers */
    uint8_t speaker_ids[MESH_MAX_ACTIVE_SPEAKERS]; /* Released speaker IDs */
} mesh_speaker_release_payload_t;

#if defined(__cplusplus)
#define MESH_STATIC_ASSERT(condition, message) static_assert(condition, message)
#else
#define MESH_STATIC_ASSERT(condition, message) _Static_assert(condition, message)
#endif

MESH_STATIC_ASSERT(sizeof(mesh_header_t) == 9, "mesh_header_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_wire_identity_t) == 7, "identity size");
MESH_STATIC_ASSERT(sizeof(mesh_membership_v3_payload_t) == 79, "membership v3 size");
MESH_STATIC_ASSERT(offsetof(mesh_membership_v3_payload_t, identities) == 23,
                   "membership v3 identity offset");
MESH_STATIC_ASSERT(sizeof(mesh_header_t) + sizeof(mesh_membership_v3_payload_t) <=
                       MESH_CONTROL_MAX_PACKET_SIZE, "membership v3 packet capacity");
MESH_STATIC_ASSERT(sizeof(mesh_join_v3_payload_t) == 15, "join size");
MESH_STATIC_ASSERT(sizeof(mesh_join_ack_v3_payload_t) == 15, "join ack size");
MESH_STATIC_ASSERT(sizeof(mesh_topology_payload_t) == 47, "topology size");
MESH_STATIC_ASSERT(sizeof(mesh_speaker_request_payload_t) == 13, "speaker request size");
MESH_STATIC_ASSERT(sizeof(mesh_sync_v3_payload_t) == 20, "sync size");
MESH_STATIC_ASSERT(sizeof(mesh_handover_payload_t) == 25, "handover size");
MESH_STATIC_ASSERT(sizeof(mesh_handover_ack_payload_t) == 11, "handover ack size");
MESH_STATIC_ASSERT(sizeof(mesh_header_t) + sizeof(mesh_topology_payload_t) <=
                       MESH_CONTROL_MAX_PACKET_SIZE, "control capacity");
MESH_STATIC_ASSERT(offsetof(mesh_header_t, talk_channel) == 6,
                   "mesh_header_t talk_channel offset changed");
MESH_STATIC_ASSERT(offsetof(mesh_header_t, payload_len) == 7,
                   "mesh_header_t payload_len offset changed");
MESH_STATIC_ASSERT(sizeof(mesh_audio_payload_t) == 70, "mesh_audio_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_audio_payload_t) == 4 + MESH_MAX_AUDIO_PAYLOAD,
                   "legacy audio payload wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_join_payload_t) == 2, "mesh_join_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_join_ack_payload_t) == 3,
                   "mesh_join_ack_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_keepalive_payload_t) == 2,
                   "mesh_keepalive_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_slot_map_payload_t) == 14,
                   "mesh_slot_map_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_status_payload_t) == 8, "mesh_status_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_speaker_grant_payload_t) == 5,
                   "mesh_speaker_grant_payload_t wire size changed");
MESH_STATIC_ASSERT(sizeof(mesh_speaker_release_payload_t) == 3,
                   "mesh_speaker_release_payload_t wire size changed");
MESH_STATIC_ASSERT(MESH_AUDIO_V2_FIXED_HEADER_SIZE == 8, "audio v2 fixed header size changed");
MESH_STATIC_ASSERT(MESH_AUDIO_V2_FRAME_MS == MESH_FRAME_MS, "audio v2 frame duration changed");
MESH_STATIC_ASSERT(MESH_AUDIO_V2_MAX_FRAME_BYTES == MESH_MAX_OPUS_BYTES,
                   "audio v2 frame limit changed");
MESH_STATIC_ASSERT(MESH_AUDIO_V2_MAX_BUNDLE_SIZE == 200, "audio v2 bundle limit changed");
MESH_STATIC_ASSERT(MESH_AUDIO_V2_MAX_PACKET_SIZE == 209, "audio v2 mesh packet limit changed");
MESH_STATIC_ASSERT(sizeof(mesh_header_t) + MESH_AUDIO_V2_MAX_BUNDLE_SIZE ==
                       MESH_AUDIO_V2_MAX_PACKET_SIZE,
                    "audio v2 packet size no longer matches mesh envelope");

static inline int mesh_audio_wire_payload_valid(uint8_t codec, uint8_t frame_ms,
                                                uint16_t data_len, uint8_t local_codec)
{
    if (frame_ms != MESH_FRAME_MS || codec != local_codec) {
        return 0;
    }
    if (codec == MESH_AUDIO_CODEC_LC3) {
        return data_len == MESH_LC3_FRAME_BYTES;
    }
    if (codec == MESH_AUDIO_CODEC_OPUS) {
        return data_len > 0 && data_len <= MESH_MAX_OPUS_BYTES;
    }
    return 0;
}

#undef MESH_STATIC_ASSERT

#endif /* OMI_MESH_PROTOCOL_DEFS_H */
