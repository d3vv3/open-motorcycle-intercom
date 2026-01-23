/**
 * @file mesh.h
 * @brief OMI Mesh Networking Interface - Phase 2
 *
 * Phase 2 Implementation:
 * - ESP-NOW transport layer
 * - TDMA frame timing (20ms frames, 2ms slots)
 * - Dynamic join/leave protocol
 * - Time synchronization
 * - Simple jitter buffer
 *
 * This component handles:
 * - ESP-NOW transport
 * - TDMA scheduling
 * - Slot management
 * - Packet routing (single-hop only in Phase 2)
 * - Time synchronization
 */

#ifndef OMI_MESH_H
#define OMI_MESH_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ============================================================================ */

/**
 * @brief Maximum number of nodes in mesh
 */
#define MESH_MAX_NODES 8

/**
 * @brief TDMA frame duration in milliseconds (aligned with Opus)
 */
#define MESH_FRAME_MS 20

/**
 * @brief TDMA slot duration in milliseconds
 */
#define MESH_SLOT_MS 2

/**
 * @brief Guard time in microseconds (between slots)
 */
#define MESH_GUARD_US 200

/**
 * @brief Control window duration in milliseconds
 */
#define MESH_CONTROL_MS 2

/**
 * @brief Number of voice slots per frame
 */
#define MESH_VOICE_SLOTS 8

/**
 * @brief SYNC broadcast interval (every N frames)
 */
#define MESH_SYNC_INTERVAL_FRAMES 10

/**
 * @brief Node timeout in milliseconds (no KEEPALIVE/AUDIO)
 */
#define MESH_NODE_TIMEOUT_MS 3000

/**
 * @brief KEEPALIVE interval in milliseconds
 */
#define MESH_KEEPALIVE_INTERVAL_MS 500

/**
 * @brief JOIN request retry interval in milliseconds
 */
#define MESH_JOIN_RETRY_MS 500

/**
 * @brief Protocol version
 */
#define MESH_PROTOCOL_VERSION 0x01

/**
 * @brief Jitter buffer depth (number of frames)
 */
#define MESH_JITTER_BUFFER_DEPTH 4

/**
 * @brief Maximum audio payload size (Opus frame)
 */
#define MESH_MAX_AUDIO_PAYLOAD 64

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief Node roles in the mesh
 */
typedef enum {
    MESH_ROLE_NONE = 0,    /**< Not connected to mesh */
    MESH_ROLE_COORDINATOR, /**< Time master / coordinator */
    MESH_ROLE_PARTICIPANT, /**< Time slave / participant */
} mesh_role_t;

/**
 * @brief Mesh node state
 */
typedef enum {
    MESH_STATE_IDLE = 0, /**< Not started */
    MESH_STATE_SCANNING, /**< Listening for existing mesh */
    MESH_STATE_JOINING,  /**< Sending JOIN requests */
    MESH_STATE_ACTIVE,   /**< Connected and active */
} mesh_state_t;

/**
 * @brief Mesh packet types (from protocol.md)
 */
typedef enum {
    MESH_PKT_AUDIO = 0x01,     /**< Opus audio data */
    MESH_PKT_JOIN = 0x02,      /**< Request to join mesh */
    MESH_PKT_JOIN_ACK = 0x03,  /**< Join response with assigned ID */
    MESH_PKT_LEAVE = 0x04,     /**< Graceful leave (optional) */
    MESH_PKT_SYNC = 0x05,      /**< TDMA timing sync */
    MESH_PKT_SLOT_MAP = 0x06,  /**< Slot assignment broadcast */
    MESH_PKT_STATUS = 0x07,    /**< Battery / health */
    MESH_PKT_KEEPALIVE = 0x08, /**< Presence check */
} mesh_pkt_type_t;

/**
 * @brief Mesh packet header (8 bytes, from protocol.md)
 */
typedef struct __attribute__((packed)) {
    uint8_t version;      /**< Protocol version */
    uint8_t type;         /**< Packet type (mesh_pkt_type_t) */
    uint8_t src_id;       /**< Source node ID (0 = unassigned) */
    uint8_t seq;          /**< Sequence number */
    uint8_t hop;          /**< Remaining hop count */
    uint8_t flags;        /**< Control flags */
    uint16_t payload_len; /**< Payload length in bytes */
} mesh_header_t;

/**
 * @brief Audio packet payload (4 + N bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t codec;     /**< Codec ID (0x01 = Opus) */
    uint8_t frame_ms;  /**< Frame duration (20) */
    uint8_t stream_id; /**< Stream identifier */
    uint8_t reserved;
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD]; /**< Opus encoded data */
} mesh_audio_payload_t;

/**
 * @brief JOIN request payload (2 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t capabilities; /**< Node capabilities bitmap */
    uint8_t reserved;
} mesh_join_payload_t;

/**
 * @brief JOIN_ACK payload (3 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t assigned_id;    /**< Assigned node ID (1-8) */
    uint8_t slot_index;     /**< Assigned TDMA slot */
    uint8_t coordinator_id; /**< Current coordinator ID */
} mesh_join_ack_payload_t;

/**
 * @brief SYNC payload (6 bytes)
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_counter; /**< Current TDMA frame number */
    int16_t drift_ppm;      /**< Estimated clock drift */
} mesh_sync_payload_t;

/**
 * @brief KEEPALIVE payload (2 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t battery_pct; /**< Battery percentage (0-100) */
    uint8_t reserved;
} mesh_keepalive_payload_t;

/**
 * @brief SLOT_MAP payload (1 + N bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t slot_count;               /**< Number of active slots */
    uint8_t slot_ids[MESH_MAX_NODES]; /**< Node ID for each slot (0 = empty) */
} mesh_slot_map_payload_t;

/**
 * @brief Mesh configuration
 */
typedef struct {
    uint8_t node_id;  /**< Local node ID (0 = auto-assign) */
    uint8_t max_hops; /**< Maximum hop count (default: 2) */
    uint8_t tx_power; /**< TX power level (dBm) */
    uint8_t channel;  /**< WiFi channel (1-13) */
} mesh_config_t;

/**
 * @brief Default mesh configuration
 */
#define MESH_CONFIG_DEFAULT()                                                                      \
    {                                                                                              \
        .node_id = 0,                                                                              \
        .max_hops = 2,                                                                             \
        .tx_power = 20,                                                                            \
        .channel = 1,                                                                              \
    }

/**
 * @brief Information about a peer node
 */
typedef struct {
    uint8_t node_id;      /**< Assigned node ID */
    uint8_t mac_addr[6];  /**< MAC address */
    int8_t slot_index;    /**< Assigned slot (-1 if none) */
    int64_t last_seen_ms; /**< Last packet received timestamp */
    uint8_t battery_pct;  /**< Last reported battery level */
    bool active;          /**< Node is active in mesh */
} mesh_peer_info_t;

/**
 * @brief Mesh statistics
 */
typedef struct {
    /* Packet counters */
    uint32_t packets_tx;         /**< Total packets transmitted */
    uint32_t packets_rx;         /**< Total packets received */
    uint32_t packets_dropped;    /**< Packets dropped (TX failure) */
    uint32_t packets_forwarded;  /**< Packets forwarded (Phase 4+) */
    uint32_t rx_queue_overflows; /**< RX queue full events */

    /* Audio-specific */
    uint32_t audio_frames_tx;   /**< Audio frames transmitted */
    uint32_t audio_frames_rx;   /**< Audio frames received */
    uint32_t audio_frames_late; /**< Audio frames arrived too late */
    uint32_t audio_frames_lost; /**< Audio frames never received */

    /* TDMA timing */
    uint32_t slot_misses;   /**< Missed TX slot opportunities */
    uint32_t sync_received; /**< SYNC packets received */
    uint32_t sync_errors;   /**< SYNC timing errors */
    int32_t clock_drift_us; /**< Estimated clock drift */

    /* Jitter buffer */
    uint8_t jitter_depth;      /**< Current jitter buffer depth */
    uint32_t jitter_underruns; /**< Jitter buffer underruns */
    uint32_t jitter_overruns;  /**< Jitter buffer overruns */

    /* Latency */
    uint32_t latency_min_us; /**< Minimum observed latency */
    uint32_t latency_max_us; /**< Maximum observed latency */
    uint32_t latency_avg_us; /**< Average latency (EMA) */

    /* Protocol */
    uint32_t join_attempts;  /**< JOIN requests sent */
    uint32_t join_successes; /**< Successful joins */
    uint32_t node_timeouts;  /**< Peer timeout events */
} mesh_stats_t;

/**
 * @brief Callback for received audio frames
 *
 * @param data Opus-encoded audio data
 * @param len Length of audio data
 * @param src_id Source node ID
 * @param timestamp_us Receive timestamp (microseconds)
 */
typedef void (*mesh_audio_cb_t)(const uint8_t *data, uint16_t len, uint8_t src_id,
                                int64_t timestamp_us);

/**
 * @brief Callback for mesh state changes
 *
 * @param old_state Previous state
 * @param new_state New state
 */
typedef void (*mesh_state_cb_t)(mesh_state_t old_state, mesh_state_t new_state);

/**
 * @brief Callback for peer events (join/leave)
 *
 * @param peer Peer information
 * @param joined true if joined, false if left
 */
typedef void (*mesh_peer_cb_t)(const mesh_peer_info_t *peer, bool joined);

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/**
 * @brief Initialize the mesh subsystem with default configuration
 * @return ESP_OK on success
 */
esp_err_t mesh_init(void);

/**
 * @brief Initialize with custom configuration
 * @param config Mesh configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t mesh_init_with_config(const mesh_config_t *config);

/**
 * @brief Deinitialize the mesh subsystem
 * @return ESP_OK on success
 */
esp_err_t mesh_deinit(void);

/**
 * @brief Start mesh networking
 *
 * This will:
 * 1. Scan for existing mesh (listen for SYNC packets)
 * 2. If found, send JOIN request
 * 3. If not found after timeout, become coordinator
 *
 * @return ESP_OK on success
 */
esp_err_t mesh_start(void);

/**
 * @brief Stop mesh networking
 *
 * Sends LEAVE packet and stops TDMA scheduler.
 *
 * @return ESP_OK on success
 */
esp_err_t mesh_stop(void);

/**
 * @brief Get current mesh role
 * @return Current role (NONE, COORDINATOR, PARTICIPANT)
 */
mesh_role_t mesh_get_role(void);

/**
 * @brief Get current mesh state
 * @return Current state
 */
mesh_state_t mesh_get_state(void);

/**
 * @brief Get assigned slot index
 * @return Slot index (0-7) or -1 if not assigned
 */
int8_t mesh_get_slot(void);

/**
 * @brief Get local node ID
 * @return Node ID (1-8) or 0 if not assigned
 */
uint8_t mesh_get_node_id(void);

/**
 * @brief Get number of active nodes in mesh
 * @return Active node count (including self)
 */
uint8_t mesh_get_node_count(void);

/**
 * @brief Get information about a peer
 * @param node_id Node ID to query
 * @param info Output peer information
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t mesh_get_peer_info(uint8_t node_id, mesh_peer_info_t *info);

/**
 * @brief Queue audio frame for transmission in next slot
 *
 * Called by audio subsystem when a frame is ready to send.
 * Frame will be transmitted in the node's assigned TDMA slot.
 *
 * @param data Opus encoded audio data
 * @param len Data length (typically 20-40 bytes)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t mesh_send_audio(const uint8_t *data, uint16_t len);

/**
 * @brief Register callback for received audio frames
 * @param cb Callback function
 * @return ESP_OK on success
 */
esp_err_t mesh_register_audio_callback(mesh_audio_cb_t cb);

/**
 * @brief Register callback for state changes
 * @param cb Callback function
 * @return ESP_OK on success
 */
esp_err_t mesh_register_state_callback(mesh_state_cb_t cb);

/**
 * @brief Register callback for peer events
 * @param cb Callback function
 * @return ESP_OK on success
 */
esp_err_t mesh_register_peer_callback(mesh_peer_cb_t cb);

/**
 * @brief Get mesh statistics
 * @param stats Output statistics structure
 * @return ESP_OK on success
 */
esp_err_t mesh_get_stats(mesh_stats_t *stats);

/**
 * @brief Reset mesh statistics
 * @return ESP_OK on success
 */
esp_err_t mesh_reset_stats(void);

/**
 * @brief Get current TDMA frame counter
 * @return Frame counter value
 */
uint32_t mesh_get_frame_counter(void);

/**
 * @brief Get time until next TX slot in microseconds
 * @return Microseconds until next TX opportunity, or -1 if no slot assigned
 */
int32_t mesh_get_time_to_slot_us(void);

#ifdef __cplusplus
}
#endif

#endif /* OMI_MESH_H */
