/** @file mesh.h @brief ESP-NOW mesh networking interface. */

#ifndef OMI_MESH_H
#define OMI_MESH_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Shared on-air protocol definitions (single source of truth, also used by the
 * nRF52840/ESB build). Only ESP-NOW-specific items are defined below. */
#include "mesh_protocol_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Wire-protocol constants, enums, headers, and payloads come from
 * shared/mesh_protocol_defs.h. Only ESP-NOW-specific items are defined here. */

/** @brief Control window duration in milliseconds */
#define MESH_CONTROL_MS 2

/** @brief Number of voice slots per frame */
#define MESH_VOICE_SLOTS 8

/** @brief JOIN request retry interval in milliseconds */
#define MESH_JOIN_RETRY_MS 500

/** @brief Jitter buffer depth (number of frames) */
#define MESH_JITTER_BUFFER_DEPTH 4

/** @brief Maximum pending ESP-NOW control packets */
#define MESH_CONTROL_QUEUE_CAPACITY 12

/**
 * @brief SYNC payload - ESP-NOW variant (6-byte WiFi MAC for tiebreaking)
 *
 * Not in the shared header: the coordinator address width differs from the
 * nRF/ESB build (which uses a 5-byte ESB address).
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_counter;      /**< Current TDMA frame number */
    int16_t drift_ppm;           /**< Estimated clock drift */
    uint8_t coordinator_addr[6]; /**< Coordinator MAC address for tiebreaking */
} mesh_sync_payload_t;

_Static_assert(sizeof(mesh_sync_payload_t) == 12, "ESP-NOW SYNC wire size changed");

/**
 * @brief Mesh configuration
 */
typedef struct {
    uint8_t node_id;  /**< Local node ID (0 = auto-assign) */
    uint8_t tx_power; /**< TX power level (dBm) */
    uint8_t channel;  /**< WiFi channel (1-13) */
} mesh_config_t;

/**
 * @brief Default mesh configuration
 */
#define MESH_CONFIG_DEFAULT()                                                                      \
    {                                                                                              \
        .node_id = 0,                                                                              \
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
    int8_t rssi_dbm;      /**< Last reported RSSI */
    uint8_t peer_count;   /**< Last reported peer count */
    uint8_t fw_version;   /**< Last reported firmware version */
    int8_t temperature_c; /**< Last reported temperature */
    uint8_t heard_bitmap; /**< Last reported heard-source bitmap */
    uint8_t relay_bitmap; /**< Last reported relay-source bitmap */
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
    uint32_t join_attempts;               /**< JOIN requests sent */
    uint32_t join_successes;              /**< Successful joins */
    uint32_t node_timeouts;               /**< Peer timeout events */
    uint32_t control_queue_drops;         /**< Control packets dropped or evicted */
    uint8_t control_queue_depth;          /**< Current queued control packet count */
    uint8_t control_queue_high_watermark; /**< Maximum queued control packet count */
    uint32_t contention_tx;               /**< Unassigned JOIN contention transmissions */
    uint32_t contention_deferred;         /**< JOIN attempts deferred by contention pacing */
} mesh_stats_t;

/**
 * @brief Callback for received audio frames
 *
 * @param data Opus-encoded audio data
 * @param len Length of audio data
 * @param src_id Source node ID
 * @param audio_flags Per-frame audio activity flags
 * @param timestamp_us Original ESP-NOW receive timestamp (microseconds)
 */
typedef void (*mesh_audio_cb_t)(const uint8_t *data, uint16_t len, uint8_t src_id,
                                 uint8_t audio_flags, int64_t timestamp_us);

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
 * @brief Check if mesh subsystem is initialized
 * @return true if initialized
 */
bool mesh_is_initialized(void);

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
esp_err_t mesh_send_audio(const uint8_t *data, uint16_t len, uint8_t audio_flags);

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
