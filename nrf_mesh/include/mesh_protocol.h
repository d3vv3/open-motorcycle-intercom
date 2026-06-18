/**
 * @file mesh_protocol.h
 * @brief Shared mesh protocol definitions (matching ESP32-S3 mesh.h)
 */

#ifndef OMI_MESH_PROTOCOL_H
#define OMI_MESH_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/* Shared on-air protocol definitions (single source of truth, also used by the
 * ESP-NOW build). Only nRF/ESB-specific items are defined below. */
#include "mesh_protocol_defs.h"

/* ============================================================================
 * nRF/ESB-specific protocol types
 *
 * Constants, enums (mesh_role_t / mesh_state_t / mesh_pkt_type_t), the packet
 * header, and payload structs come from shared/mesh_protocol_defs.h (included
 * above). Only nRF/ESB-specific items are defined here.
 * ============================================================================ */

/**
 * @brief SYNC payload - nRF/ESB variant (5-byte ESB address for tiebreaking)
 *
 * Not in the shared header: the coordinator address width differs from the
 * ESP-NOW build (which uses a 6-byte WiFi MAC).
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_counter;
    int16_t drift_ppm;
    uint8_t coordinator_addr[5]; /* ESB address for coordinator tiebreaking */
} mesh_sync_payload_t;

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
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;
    bool active;
} mesh_peer_info_t;

/**
 * @brief Queue audio packet for transmission in next slot
 * @param data Audio payload
 * @param len Payload length
 * @return 0 on success
 */
int mesh_protocol_send_audio(const uint8_t *data, uint8_t len, uint8_t audio_flags);

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
