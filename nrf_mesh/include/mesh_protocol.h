/**
 * @file mesh_protocol.h
 * @brief Shared mesh protocol definitions (matching ESP32-S3 mesh.h)
 */

#ifndef OMI_MESH_PROTOCOL_H
#define OMI_MESH_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Constants (matching ESP32 mesh.h)
 * ============================================================================ */

#define MESH_MAX_NODES             8
#define MESH_FRAME_MS              20
#define MESH_SLOT_MS               2
#define MESH_GUARD_US              200
#define MESH_SYNC_INTERVAL_FRAMES  10
#define MESH_NODE_TIMEOUT_MS       3000
#define MESH_KEEPALIVE_INTERVAL_MS 500
#define MESH_PROTOCOL_VERSION      0x01
#define MESH_MAX_AUDIO_PAYLOAD     64

/* ============================================================================
 * Packet Types
 * ============================================================================ */

typedef enum {
    MESH_PKT_AUDIO = 0x01,
    MESH_PKT_JOIN = 0x02,
    MESH_PKT_JOIN_ACK = 0x03,
    MESH_PKT_LEAVE = 0x04,
    MESH_PKT_SYNC = 0x05,
    MESH_PKT_SLOT_MAP = 0x06,
    MESH_PKT_STATUS = 0x07,
    MESH_PKT_KEEPALIVE = 0x08,
} mesh_pkt_type_t;

/* ============================================================================
 * Packet Header (8 bytes, matching ESP32)
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t version;      /* Protocol version */
    uint8_t type;         /* Packet type */
    uint8_t src_id;       /* Source node ID */
    uint8_t seq;          /* Sequence number */
    uint8_t reserved0;    /* Reserved for future use */
    uint8_t flags;        /* Control flags */
    uint16_t payload_len; /* Payload length */
} mesh_header_t;

/* ============================================================================
 * Payload Structures
 * ============================================================================ */

typedef struct __attribute__((packed)) {
    uint8_t codec;
    uint8_t frame_ms;
    uint8_t stream_id;
    uint8_t reserved;
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD];
} mesh_audio_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t capabilities;
    uint8_t reserved;
} mesh_join_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t assigned_id;
    uint8_t slot_index;
    uint8_t coordinator_id;
} mesh_join_ack_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t frame_counter;
    int16_t drift_ppm;
} mesh_sync_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t battery_pct;
    uint8_t reserved;
} mesh_keepalive_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t slot_count;
    uint8_t slot_ids[MESH_MAX_NODES];
} mesh_slot_map_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t battery_pct;
    int8_t rssi_dbm;
    uint8_t peer_count;
    uint8_t fw_version;
    int8_t temperature_c;
    uint8_t reserved;
} mesh_status_payload_t;

/* ============================================================================
 * Node State
 * ============================================================================ */

typedef enum {
    MESH_ROLE_NONE = 0,
    MESH_ROLE_COORDINATOR,
    MESH_ROLE_PARTICIPANT,
} mesh_role_t;

typedef enum {
    MESH_STATE_IDLE = 0,
    MESH_STATE_SCANNING,
    MESH_STATE_JOINING,
    MESH_STATE_ACTIVE,
} mesh_state_t;

/* ============================================================================
 * Peer Info
 * ============================================================================ */

typedef struct {
    uint8_t node_id;
    uint8_t esb_addr[5]; /* ESB pipe address */
    int8_t slot_index;
    int64_t last_seen_ms;
    uint8_t battery_pct;
    int8_t rssi_dbm;
    uint8_t peer_count;
    uint8_t fw_version;
    int8_t temperature_c;
    bool active;
} mesh_peer_info_t;

/**
 * @brief Queue audio packet for transmission in next slot
 * @param data Audio payload
 * @param len Payload length
 * @return 0 on success
 */
int mesh_protocol_send_audio(const uint8_t *data, uint8_t len);

/**
 * @brief Initialize mesh protocol
 * @return 0 on success
 */
int mesh_protocol_init(void);

/**
 * @brief Start mesh protocol (scan/join/host)
 * @return 0 on success
 */
int mesh_protocol_start(void);

/**
 * @brief Stop mesh protocol
 */
void mesh_protocol_stop(void);

/**
 * @brief Get current mesh state
 * @return Current state (IDLE, SCANNING, JOINING, ACTIVE)
 */
mesh_state_t mesh_protocol_get_state(void);

/**
 * @brief Get current mesh role
 * @return Current role (NONE, COORDINATOR, PARTICIPANT)
 */
mesh_role_t mesh_protocol_get_role(void);

/**
 * @brief Get current node ID
 * @return Node ID (0 if not joined)
 */
uint8_t mesh_protocol_get_node_id(void);

/**
 * @brief Get current peer count
 * @return Number of active peers
 */
uint8_t mesh_protocol_get_peer_count(void);

#endif /* OMI_MESH_PROTOCOL_H */
