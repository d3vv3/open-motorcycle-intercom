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

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * Protocol constants (wire-visible)
 * ============================================================================ */

#define MESH_MAX_NODES             8
#define MESH_FRAME_MS              20   /* TDMA frame duration (aligned with Opus) */
#define MESH_SLOT_MS               2    /* TDMA slot duration */
#define MESH_GUARD_US              500  /* Guard time between slots */
#define MESH_SYNC_INTERVAL_FRAMES  10   /* SYNC broadcast cadence */
#define MESH_NODE_TIMEOUT_MS       3000 /* Drop peer after this silence */
#define MESH_KEEPALIVE_INTERVAL_MS 500  /* KEEPALIVE cadence */
#define MESH_PROTOCOL_VERSION      0x01
#define MESH_MAX_OPUS_BYTES        64
#define MESH_E2E_SEQUENCE_BYTES    2
#define MESH_MAX_AUDIO_PAYLOAD     (MESH_MAX_OPUS_BYTES + MESH_E2E_SEQUENCE_BYTES)
#define MESH_MAX_ACTIVE_SPEAKERS   2    /* Concurrent relay-granted speakers */
#define MESH_AUDIO_TTL_DEFAULT     2    /* Default relay TTL */

/* Header control flags */
#define MESH_FLAG_RELAY_REQUEST    0x01
#define MESH_FLAG_RELAYED          0x02
#define MESH_FLAG_SPEAKER_GRANTED  0x04

/* Audio payload flags */
#define MESH_AUDIO_FLAG_ACTIVE     0x01

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
} mesh_pkt_type_t;

/* ============================================================================
 * Packet header (8 bytes)
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t version;      /* Protocol version */
    uint8_t type;         /* Packet type (mesh_pkt_type_t) */
    uint8_t src_id;       /* Source node ID (0 = unassigned) */
    uint8_t seq;          /* Sequence number */
    uint8_t ttl;          /* Relay time-to-live */
    uint8_t flags;        /* Control flags */
    uint16_t payload_len; /* Payload length in bytes */
} mesh_header_t;

/* ============================================================================
 * Payload structures (wire-identical across transports)
 *
 * NOTE: mesh_sync_payload_t is intentionally NOT here - its coordinator
 * address width differs per transport (6-byte MAC vs 5-byte ESB address), so
 * each platform header defines its own.
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t codec;                        /* Codec ID (0x01 = Opus) */
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
    uint8_t slot_count;                                   /* Number of active slots */
    uint8_t slot_ids[MESH_MAX_NODES];                     /* Node ID per slot (0 = empty) */
    uint8_t active_speaker_count;                         /* Number of granted speakers */
    uint8_t active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS]; /* Granted speaker IDs */
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS];        /* Relay bitmap per speaker */
} mesh_slot_map_payload_t;

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

MESH_STATIC_ASSERT(sizeof(mesh_header_t) == 8, "mesh_header_t wire size changed");
MESH_STATIC_ASSERT(offsetof(mesh_header_t, payload_len) == 6,
                   "mesh_header_t payload_len offset changed");
MESH_STATIC_ASSERT(sizeof(mesh_audio_payload_t) == 70, "mesh_audio_payload_t wire size changed");
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

#undef MESH_STATIC_ASSERT

#endif /* OMI_MESH_PROTOCOL_DEFS_H */
