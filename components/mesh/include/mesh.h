/**
 * @file mesh.h
 * @brief OMI Mesh Networking Interface
 *
 * This component handles:
 * - ESP-NOW transport (Phase 1-2)
 * - TDMA scheduling
 * - Slot management
 * - Packet routing/relaying
 * - Time synchronization
 */

#ifndef OMI_MESH_H
#define OMI_MESH_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of nodes in mesh
 */
#define MESH_MAX_NODES      8

/**
 * @brief TDMA frame duration in milliseconds (aligned with Opus)
 */
#define MESH_FRAME_MS       20

/**
 * @brief TDMA slot duration in milliseconds
 */
#define MESH_SLOT_MS        2

/**
 * @brief Node roles in the mesh
 */
typedef enum {
    MESH_ROLE_NONE = 0,     /**< Not connected */
    MESH_ROLE_MASTER,       /**< Time master */
    MESH_ROLE_SLAVE,        /**< Time slave */
} mesh_role_t;

/**
 * @brief Mesh packet types (from protocol.md)
 */
typedef enum {
    MESH_PKT_AUDIO      = 0x01,
    MESH_PKT_JOIN       = 0x02,
    MESH_PKT_JOIN_ACK   = 0x03,
    MESH_PKT_LEAVE      = 0x04,
    MESH_PKT_SYNC       = 0x05,
    MESH_PKT_SLOT_MAP   = 0x06,
    MESH_PKT_STATUS     = 0x07,
    MESH_PKT_KEEPALIVE  = 0x08,
} mesh_pkt_type_t;

/**
 * @brief Mesh packet header (from protocol.md)
 */
typedef struct __attribute__((packed)) {
    uint8_t  version;       /**< Protocol version */
    uint8_t  type;          /**< Packet type */
    uint8_t  src_id;        /**< Source node ID */
    uint8_t  seq;           /**< Sequence number */
    uint8_t  hop;           /**< Remaining hop count */
    uint8_t  flags;         /**< Control flags */
    uint16_t payload_len;   /**< Payload length */
} mesh_header_t;

/**
 * @brief Audio packet payload
 */
typedef struct __attribute__((packed)) {
    uint8_t  codec;         /**< Codec ID (0x01 = Opus) */
    uint8_t  frame_ms;      /**< Frame duration */
    uint8_t  stream_id;     /**< Stream identifier */
    uint8_t  reserved;
    uint8_t  data[];        /**< Opus encoded data */
} mesh_audio_payload_t;

/**
 * @brief Mesh configuration
 */
typedef struct {
    uint8_t  node_id;       /**< Local node ID (0 = auto-assign) */
    uint8_t  max_hops;      /**< Maximum hop count (default: 2) */
    uint8_t  tx_power;      /**< TX power level */
    uint8_t  channel;       /**< WiFi channel (1-13) */
} mesh_config_t;

#define MESH_CONFIG_DEFAULT() { \
    .node_id = 0,               \
    .max_hops = 2,              \
    .tx_power = 20,             \
    .channel = 1,               \
}

/**
 * @brief Callback for received audio frames
 */
typedef void (*mesh_audio_cb_t)(const uint8_t *data, uint16_t len, uint8_t src_id);

/**
 * @brief Initialize the mesh subsystem
 * @return ESP_OK on success
 */
esp_err_t mesh_init(void);

/**
 * @brief Initialize with custom configuration
 * @param config Mesh configuration
 * @return ESP_OK on success
 */
esp_err_t mesh_init_with_config(const mesh_config_t *config);

/**
 * @brief Deinitialize the mesh subsystem
 * @return ESP_OK on success
 */
esp_err_t mesh_deinit(void);

/**
 * @brief Start mesh networking (create or join group)
 * @return ESP_OK on success
 */
esp_err_t mesh_start(void);

/**
 * @brief Stop mesh networking
 * @return ESP_OK on success
 */
esp_err_t mesh_stop(void);

/**
 * @brief Get current mesh role
 * @return Current role
 */
mesh_role_t mesh_get_role(void);

/**
 * @brief Get assigned slot index
 * @return Slot index (0-7) or -1 if not assigned
 */
int8_t mesh_get_slot(void);

/**
 * @brief Get number of connected nodes
 * @return Node count
 */
uint8_t mesh_get_node_count(void);

/**
 * @brief Queue audio frame for transmission in next slot
 * @param data Opus encoded audio data
 * @param len Data length
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t mesh_send_audio(const uint8_t *data, uint16_t len);

/**
 * @brief Register callback for received audio
 * @param cb Callback function
 * @return ESP_OK on success
 */
esp_err_t mesh_register_audio_callback(mesh_audio_cb_t cb);

/**
 * @brief Mesh statistics
 */
typedef struct {
    uint32_t packets_tx;
    uint32_t packets_rx;
    uint32_t packets_dropped;
    uint32_t packets_forwarded;
    uint32_t slot_misses;
    uint32_t sync_errors;
    int32_t  clock_drift_us;
} mesh_stats_t;

esp_err_t mesh_get_stats(mesh_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* OMI_MESH_H */
