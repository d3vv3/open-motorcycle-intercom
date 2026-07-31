/**
 * @file mesh.c
 * @brief OMI Mesh Networking Implementation - Phase 2
 *
 * Phase 2: Single-hop RF link with dynamic join/leave
 * - ESP-NOW transport layer
 * - TDMA frame timing (20ms frames)
 * - Dynamic coordinator election (lowest MAC)
 * - Simple jitter buffer
 * - Time synchronization
 */

#include "mesh.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "audio.h"
#include "power.h"

static const char *TAG = "mesh";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define MESH_TASK_STACK_SIZE 4096
#define MESH_TASK_PRIORITY   6 /* Higher than audio (5) for timing */
#define MESH_TASK_CORE       0 /* Run on core 0, audio on core 1 */

#define MESH_TX_QUEUE_SIZE 4
#define MESH_RX_QUEUE_SIZE 32 /* Increased to handle packet floods during join */

#define MESH_SCAN_TIMEOUT_MS  2000 /* Time to listen before becoming coordinator */
#define MESH_JOIN_TIMEOUT_MS  5000 /* Time to wait for JOIN_ACK */
#define MESH_JOIN_RETRY_COUNT 10
#define MESH_STATUS_INTERVAL_MS 1000
#define SEEN_RING_SIZE          32
#define RELAY_RING_SIZE         16
#define TX_COMPLETION_RING_SIZE 16
#define ACTIVE_SPEAKER_TIMEOUT_MS 1500

/* Frame timing in microseconds */
#define MESH_FRAME_US   (MESH_FRAME_MS * 1000)
#define MESH_SLOT_US    (MESH_SLOT_MS * 1000)
#define MESH_CONTROL_US (MESH_CONTROL_MS * 1000)
#define ESPNOW_SYNC_RX_LATENCY_US 500

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief TX queue item
 */
typedef struct {
    uint8_t data[MESH_MAX_OPUS_BYTES];
    uint16_t len;
    uint8_t audio_flags;
    int64_t timestamp_us;
} mesh_tx_item_t;

/**
 * @brief RX queue item (for async processing)
 */
typedef struct {
    mesh_header_t header;
    uint8_t payload[128];
    uint8_t src_mac[6];
    int8_t rssi;
    int64_t timestamp_us;
} mesh_rx_item_t;

/**
 * @brief Jitter buffer entry
 */
typedef struct {
    uint8_t data[MESH_MAX_OPUS_BYTES];
    uint16_t len;
    uint8_t src_id;
    uint8_t seq;
    uint8_t audio_flags;
    int64_t timestamp_us;
    bool valid;
} jitter_entry_t;

/**
 * @brief Peer tracking structure
 */
typedef struct {
    mesh_peer_info_t info;
    uint8_t last_seq; /* Last sequence number seen */
    uint32_t packets_received;
    uint32_t packets_lost;
} peer_tracking_t;

typedef struct {
    uint8_t type;
    uint8_t src_id;
    uint8_t seq;
    bool valid;
} seen_entry_t;

typedef struct {
    uint8_t data[sizeof(mesh_header_t) + sizeof(mesh_audio_payload_t)];
    uint16_t len;
} relay_entry_t;

typedef struct {
    int64_t timestamp_us;
    uint32_t generation;
} frame_event_t;

typedef struct {
    uint8_t type;
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;
    bool valid;
} tx_completion_entry_t;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/* State */
static bool s_initialized = false;
static mesh_config_t s_config = MESH_CONFIG_DEFAULT();
static mesh_state_t s_state = MESH_STATE_IDLE;
static mesh_role_t s_role = MESH_ROLE_NONE;
static mesh_stats_t s_stats = {0};

/* Identity */
static uint8_t s_local_mac[6] = {0};
static uint8_t s_node_id = 0;
static int8_t s_slot_index = -1;
/* Speaker-grant and TDMA snapshots cross the mesh, timer, and Wi-Fi callback
 * contexts. Critical sections stay limited to plain state copies. */
static portMUX_TYPE s_speaker_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_tdma_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
static uint8_t s_relay_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
static uint8_t s_heard_bitmap = 0;
static uint8_t s_relay_bitmap = 0;
static int64_t s_active_speaker_deadline_ms[MESH_MAX_NODES + 1] = {0};

/* Peer tracking */
static peer_tracking_t s_peers[MESH_MAX_NODES] = {0};
static uint8_t s_peer_count = 0;
static uint8_t s_coordinator_id = 0;
static uint8_t s_coordinator_mac[6] = {0};
static SemaphoreHandle_t s_peer_mutex = NULL;

/* TDMA timing */
static uint32_t s_frame_counter = 0;
static int64_t s_frame_start_us = 0;
static esp_timer_handle_t s_frame_timer = NULL;
static esp_timer_handle_t s_slot_timer = NULL;
static esp_timer_handle_t s_control_timer = NULL;
static int64_t s_slot_start_us = 0;
static int64_t s_slot_deadline_us = 0;
static int64_t s_control_start_us = 0;
static int64_t s_control_deadline_us = 0;
static uint32_t s_tdma_generation = 0;
static uint32_t s_frame_timer_generation = 0;
static int64_t s_expected_frame_us = 0;
static uint32_t s_slot_generation = 0;
static uint32_t s_control_generation = 0;
static SemaphoreHandle_t s_slot_semaphore = NULL;
static SemaphoreHandle_t s_control_semaphore = NULL;
static QueueHandle_t s_frame_event_queue = NULL;
static QueueSetHandle_t s_timer_queue_set = NULL;

/* Queues */
static QueueHandle_t s_tx_queue = NULL;
static QueueHandle_t s_rx_queue = NULL;

/* Multi-hop state */
static seen_entry_t s_seen_ring[SEEN_RING_SIZE] = {0};
static uint8_t s_seen_head = 0;
static relay_entry_t s_relay_ring[RELAY_RING_SIZE] = {0};
static uint8_t s_relay_head = 0;
static uint8_t s_relay_tail = 0;
static tx_completion_entry_t s_tx_completion_ring[TX_COMPLETION_RING_SIZE] = {0};
static uint8_t s_tx_completion_head = 0;
static uint8_t s_tx_completion_tail = 0;

/* Jitter buffer */
static jitter_entry_t s_jitter_buffer[MESH_JITTER_BUFFER_DEPTH] = {0};
static uint8_t s_jitter_read_idx = 0;
static uint8_t s_jitter_write_idx = 0;
static SemaphoreHandle_t s_jitter_mutex = NULL;

/* Callbacks */
static mesh_audio_cb_t s_audio_cb = NULL;
static mesh_state_cb_t s_state_cb = NULL;
static mesh_peer_cb_t s_peer_cb = NULL;

/* Task handles */
static TaskHandle_t s_mesh_task = NULL;

/* ESP-NOW */
static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Protocol sequence numbers */
static uint8_t s_tx_seq = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void mesh_task(void *arg);
static void frame_timer_callback(void *arg);
static void slot_timer_callback(void *arg);
static void control_timer_callback(void *arg);
static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
static void esp_now_send_cb(const esp_now_send_info_t *send_info, esp_now_send_status_t status);

static void handle_packet(const mesh_rx_item_t *rx);
static void handle_audio_packet(const mesh_rx_item_t *rx);
static void handle_join_packet(const mesh_rx_item_t *rx);
static void handle_join_ack_packet(const mesh_rx_item_t *rx);
static void handle_sync_packet(const mesh_rx_item_t *rx);
static void handle_keepalive_packet(const mesh_rx_item_t *rx);
static void handle_slot_map_packet(const mesh_rx_item_t *rx);
static void handle_status_packet(const mesh_rx_item_t *rx);

static esp_err_t send_packet(mesh_pkt_type_t type, const void *payload, uint16_t len);
static esp_err_t tracked_esp_now_send(const uint8_t *dest_mac, const uint8_t *data, size_t len,
                                      uint8_t type, uint8_t heard_bitmap,
                                      uint8_t relay_bitmap);
static esp_err_t send_join_request(void);
static esp_err_t send_join_ack(uint8_t node_id, uint8_t slot, const uint8_t *dest_mac);
static esp_err_t send_sync(void);
static esp_err_t send_keepalive(void);
static esp_err_t send_slot_map(void);
static esp_err_t send_status(void);
static esp_err_t send_audio_in_slot(void);
static void service_tx_slot(void);
static void service_control_window(void);
static void service_frame_boundary(const frame_event_t *event);
static void drain_slot_signal(void);
static void clear_transient_mesh_state(void);
static void advance_tdma_generation(void);
static esp_err_t arm_frame_timer(int64_t delay_us);

static void set_state(mesh_state_t new_state);
static uint8_t assign_node_id(const uint8_t *mac);
static int8_t assign_slot(uint8_t node_id);
static void check_peer_timeouts(void);
static void jitter_buffer_insert(const uint8_t *data, uint16_t len, uint8_t src_id, uint8_t seq,
                                 uint8_t audio_flags);
static bool jitter_buffer_pop(uint8_t *data, uint16_t *len, uint8_t *src_id,
                              uint8_t *audio_flags);

static int compare_mac(const uint8_t *a, const uint8_t *b);
static void demote_to_participant(const uint8_t *coordinator_mac, uint8_t coordinator_id);
static uint8_t node_bit(uint8_t node_id);
static bool seen_packet(uint8_t type, uint8_t src_id, uint8_t seq);
static void remember_packet(uint8_t type, uint8_t src_id, uint8_t seq);
static bool relay_queue_empty(void);
static bool enqueue_relay_packet(const uint8_t *data, uint16_t len, uint8_t ttl, uint8_t flags);
static void clear_speaker_state(void);
static void note_audio_activity(uint8_t src_id, uint8_t audio_flags);
static uint8_t compute_relay_mask(uint8_t speaker_id);
static void send_speaker_release_for(uint8_t speaker_id);
static void update_speaker_grants(void);

static uint8_t node_bit(uint8_t node_id)
{
    if (node_id == 0 || node_id > MESH_MAX_NODES) {
        return 0;
    }

    return (uint8_t)(1U << (node_id - 1));
}

static bool seen_packet(uint8_t type, uint8_t src_id, uint8_t seq)
{
    for (size_t i = 0; i < SEEN_RING_SIZE; i++) {
        if (!s_seen_ring[i].valid) {
            continue;
        }

        if (s_seen_ring[i].type == type && s_seen_ring[i].src_id == src_id &&
            s_seen_ring[i].seq == seq) {
            return true;
        }
    }

    return false;
}

static void remember_packet(uint8_t type, uint8_t src_id, uint8_t seq)
{
    s_seen_ring[s_seen_head].type = type;
    s_seen_ring[s_seen_head].src_id = src_id;
    s_seen_ring[s_seen_head].seq = seq;
    s_seen_ring[s_seen_head].valid = true;
    s_seen_head = (uint8_t)((s_seen_head + 1) % SEEN_RING_SIZE);
}

static bool relay_queue_empty(void)
{
    return s_relay_head == s_relay_tail;
}

static bool enqueue_relay_packet(const uint8_t *data, uint16_t len, uint8_t ttl, uint8_t flags)
{
    if (data == NULL || len < sizeof(mesh_header_t) || ttl == 0) {
        return false;
    }

    if (len > sizeof(s_relay_ring[0].data)) {
        return false;
    }

    uint8_t next_head = (uint8_t)((s_relay_head + 1) % RELAY_RING_SIZE);
    if (next_head == s_relay_tail) {
        s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
    }

    relay_entry_t *entry = &s_relay_ring[s_relay_head];
    memcpy(entry->data, data, len);
    entry->len = len;

    mesh_header_t *header = (mesh_header_t *)entry->data;
    header->ttl = ttl;
    header->flags = flags;

    s_relay_head = next_head;
    return true;
}

static void clear_speaker_state(void)
{
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_ids, 0, sizeof(s_active_speaker_ids));
    memset(s_relay_masks, 0, sizeof(s_relay_masks));
    taskEXIT_CRITICAL(&s_speaker_mux);
}

/* --- speaker-grant state accessors (all guarded by s_speaker_mux) ---
 * Keep every critical section tiny: plain array copies only, never a send,
 * log, or mutex take inside taskENTER_CRITICAL/taskEXIT_CRITICAL. */
static void speaker_state_get(uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                              uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS])
{
    taskENTER_CRITICAL(&s_speaker_mux);
    if (ids) {
        memcpy(ids, s_active_speaker_ids, sizeof(s_active_speaker_ids));
    }
    if (masks) {
        memcpy(masks, s_relay_masks, sizeof(s_relay_masks));
    }
    taskEXIT_CRITICAL(&s_speaker_mux);
}

static void speaker_state_set(const uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                              const uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS])
{
    taskENTER_CRITICAL(&s_speaker_mux);
    memcpy(s_active_speaker_ids, ids, sizeof(s_active_speaker_ids));
    memcpy(s_relay_masks, masks, sizeof(s_relay_masks));
    taskEXIT_CRITICAL(&s_speaker_mux);
}

static void status_bitmaps_snapshot_and_clear(uint8_t *heard, uint8_t *relayed)
{
    taskENTER_CRITICAL(&s_speaker_mux);
    *heard = s_heard_bitmap;
    *relayed = s_relay_bitmap;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);
}

static void note_audio_activity(uint8_t src_id, uint8_t audio_flags)
{
    if ((audio_flags & MESH_AUDIO_FLAG_ACTIVE) == 0 || src_id == 0 || src_id > MESH_MAX_NODES) {
        return;
    }

    uint8_t bit = node_bit(src_id);
    int64_t deadline = (esp_timer_get_time() / 1000) + ACTIVE_SPEAKER_TIMEOUT_MS;
    taskENTER_CRITICAL(&s_speaker_mux);
    s_heard_bitmap |= bit;
    s_active_speaker_deadline_ms[src_id] = deadline;
    taskEXIT_CRITICAL(&s_speaker_mux);
}

static uint8_t compute_relay_mask(uint8_t speaker_id)
{
    uint8_t speaker_mask = node_bit(speaker_id);
    uint8_t members = 0;
    uint8_t heard = 0;
    uint8_t member_count = 0;
    uint8_t local_heard;

    taskENTER_CRITICAL(&s_speaker_mux);
    local_heard = s_heard_bitmap;
    taskEXIT_CRITICAL(&s_speaker_mux);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].info.active || s_peers[i].info.node_id == 0 ||
            s_peers[i].info.node_id == speaker_id) {
            continue;
        }

        uint8_t peer_bit = node_bit(s_peers[i].info.node_id);
        members |= peer_bit;
        member_count++;

        uint8_t heard_bitmap = s_peers[i].info.node_id == s_node_id
                                   ? local_heard
                                   : s_peers[i].info.heard_bitmap;
        if ((heard_bitmap & speaker_mask) != 0) {
            heard |= peer_bit;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    if (member_count <= 1 || (members & (uint8_t)~heard) == 0) {
        return 0;
    }

    return heard != 0 ? heard : members;
}

static void send_speaker_release_for(uint8_t speaker_id)
{
    if (speaker_id == 0) {
        return;
    }

    mesh_speaker_release_payload_t payload = {
        .speaker_count = 1,
        .speaker_ids = {speaker_id, 0},
    };

    (void)send_packet(MESH_PKT_SPEAKER_RELEASE, &payload, sizeof(payload));
}

static void update_speaker_grants(void)
{
    if (s_role != MESH_ROLE_COORDINATOR) {
        return;
    }

    uint8_t previous[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t prev_masks[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t selected[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    int64_t deadlines[MESH_MAX_NODES + 1];
    size_t idx = 0;
    int64_t now_ms = esp_timer_get_time() / 1000;

    /* Atomically snapshot the shared state, then compute outside the lock
     * (compute_relay_mask takes s_peer_mutex and must not run under the spinlock). */
    taskENTER_CRITICAL(&s_speaker_mux);
    memcpy(previous, s_active_speaker_ids, sizeof(previous));
    memcpy(prev_masks, s_relay_masks, sizeof(prev_masks));
    memcpy(deadlines, s_active_speaker_deadline_ms, sizeof(deadlines));
    taskEXIT_CRITICAL(&s_speaker_mux);

    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES && idx < MESH_MAX_ACTIVE_SPEAKERS; node_id++) {
        if (deadlines[node_id] > now_ms) {
            selected[idx] = node_id;
            relay_masks[idx] = compute_relay_mask(node_id);
            idx++;
        }
    }

    if (memcmp(previous, selected, sizeof(previous)) == 0 &&
        memcmp(prev_masks, relay_masks, sizeof(prev_masks)) == 0) {
        return;
    }

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (previous[i] == 0) {
            continue;
        }

        bool still_selected = false;
        for (int j = 0; j < MESH_MAX_ACTIVE_SPEAKERS; j++) {
            if (selected[j] == previous[i]) {
                still_selected = true;
                break;
            }
        }

        if (!still_selected) {
            send_speaker_release_for(previous[i]);
        }
    }

    speaker_state_set(selected, relay_masks);

    mesh_speaker_grant_payload_t payload = {0};
    memcpy(payload.speaker_ids, selected, sizeof(payload.speaker_ids));
    memcpy(payload.relay_masks, relay_masks, sizeof(payload.relay_masks));
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (selected[i] != 0) {
            payload.speaker_count++;
        }
    }

    (void)send_packet(MESH_PKT_SPEAKER_GRANT, &payload, sizeof(payload));
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t mesh_init(void)
{
    return mesh_init_with_config(NULL);
}

esp_err_t mesh_init_with_config(const mesh_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config != NULL) {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing mesh subsystem (Phase 2)");

    /* Get local MAC address */
    ESP_ERROR_CHECK(esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "Local MAC: " MACSTR, MAC2STR(s_local_mac));

    /* Create synchronization primitives */
    s_peer_mutex = xSemaphoreCreateMutex();
    s_jitter_mutex = xSemaphoreCreateMutex();
    s_slot_semaphore = xSemaphoreCreateBinary();
    s_control_semaphore = xSemaphoreCreateBinary();
    s_frame_event_queue = xQueueCreate(1, sizeof(frame_event_t));
    s_timer_queue_set = xQueueCreateSet(3);

    if (!s_peer_mutex || !s_jitter_mutex || !s_slot_semaphore || !s_control_semaphore ||
        !s_frame_event_queue || !s_timer_queue_set) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return ESP_ERR_NO_MEM;
    }
    xQueueAddToSet(s_slot_semaphore, s_timer_queue_set);
    xQueueAddToSet(s_control_semaphore, s_timer_queue_set);
    xQueueAddToSet(s_frame_event_queue, s_timer_queue_set);

    /* Create queues */
    s_tx_queue = xQueueCreate(MESH_TX_QUEUE_SIZE, sizeof(mesh_tx_item_t));
    s_rx_queue = xQueueCreate(MESH_RX_QUEUE_SIZE, sizeof(mesh_rx_item_t));

    if (!s_tx_queue || !s_rx_queue) {
        ESP_LOGE(TAG, "Failed to create queues");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize WiFi */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_config.channel, WIFI_SECOND_CHAN_NONE));

    /* Set TX power */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(s_config.tx_power * 4)); /* Unit: 0.25 dBm */

    /* Initialize ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    /* Add broadcast peer */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, 6);
    peer.channel = s_config.channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* Create frame timer */
    esp_timer_create_args_t timer_args = {
        .callback = frame_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tdma_frame",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_frame_timer));

    esp_timer_create_args_t slot_timer_args = {
        .callback = slot_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tdma_slot",
    };
    ESP_ERROR_CHECK(esp_timer_create(&slot_timer_args, &s_slot_timer));

    esp_timer_create_args_t control_timer_args = {
        .callback = control_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tdma_control",
    };
    ESP_ERROR_CHECK(esp_timer_create(&control_timer_args, &s_control_timer));

    /* Clear stats */
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;

    s_initialized = true;
    ESP_LOGI(TAG, "Mesh subsystem initialized");

    return ESP_OK;
}

esp_err_t mesh_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing mesh subsystem");

    /* Stop if running */
    if (s_state != MESH_STATE_IDLE) {
        mesh_stop();
    }

    /* Cleanup timer */
    if (s_frame_timer) {
        esp_timer_delete(s_frame_timer);
        s_frame_timer = NULL;
    }
    if (s_slot_timer) {
        esp_timer_delete(s_slot_timer);
        s_slot_timer = NULL;
    }
    if (s_control_timer) {
        esp_timer_delete(s_control_timer);
        s_control_timer = NULL;
    }

    /* Cleanup ESP-NOW */
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();

    /* Cleanup WiFi */
    esp_wifi_stop();
    esp_wifi_deinit();

    /* Cleanup queues */
    if (s_tx_queue) {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }
    if (s_rx_queue) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }

    /* Cleanup semaphores */
    if (s_peer_mutex) {
        vSemaphoreDelete(s_peer_mutex);
        s_peer_mutex = NULL;
    }
    if (s_jitter_mutex) {
        vSemaphoreDelete(s_jitter_mutex);
        s_jitter_mutex = NULL;
    }
    if (s_slot_semaphore) {
        xQueueRemoveFromSet(s_slot_semaphore, s_timer_queue_set);
        vSemaphoreDelete(s_slot_semaphore);
        s_slot_semaphore = NULL;
    }
    if (s_control_semaphore) {
        xQueueRemoveFromSet(s_control_semaphore, s_timer_queue_set);
        vSemaphoreDelete(s_control_semaphore);
        s_control_semaphore = NULL;
    }
    if (s_frame_event_queue) {
        xQueueRemoveFromSet(s_frame_event_queue, s_timer_queue_set);
        vQueueDelete(s_frame_event_queue);
        s_frame_event_queue = NULL;
    }
    if (s_timer_queue_set) {
        vQueueDelete(s_timer_queue_set);
        s_timer_queue_set = NULL;
    }

    s_initialized = false;
    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;

    return ESP_OK;
}

esp_err_t mesh_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state != MESH_STATE_IDLE) {
        ESP_LOGW(TAG, "Mesh already started");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting mesh networking");

    /* Create mesh task */
    BaseType_t ret = xTaskCreatePinnedToCore(mesh_task, "mesh", MESH_TASK_STACK_SIZE, NULL,
                                             MESH_TASK_PRIORITY, &s_mesh_task, MESH_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mesh task");
        return ESP_ERR_NO_MEM;
    }

    set_state(MESH_STATE_SCANNING);

    return ESP_OK;
}

esp_err_t mesh_stop(void)
{
    if (!s_initialized || s_state == MESH_STATE_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping mesh networking");

    advance_tdma_generation();
    /* Stop frame timer */
    esp_timer_stop(s_frame_timer);
    esp_timer_stop(s_slot_timer);
    esp_timer_stop(s_control_timer);
    drain_slot_signal();

    /* Send LEAVE if we were active */
    if (s_state == MESH_STATE_ACTIVE) {
        /* Send leave notification (best effort) */
        send_packet(MESH_PKT_LEAVE, NULL, 0);
    }

    /* Stop task */
    if (s_mesh_task) {
        vTaskDelete(s_mesh_task);
        s_mesh_task = NULL;
    }

    xQueueReset(s_rx_queue);
    clear_transient_mesh_state();

    /* Clear peer list */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    memset(s_peers, 0, sizeof(s_peers));
    s_peer_count = 0;
    xSemaphoreGive(s_peer_mutex);

    clear_speaker_state();
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    memset(s_seen_ring, 0, sizeof(s_seen_ring));
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    s_seen_head = 0;
    s_relay_head = 0;
    s_relay_tail = 0;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);

    /* Reset state */
    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
    s_frame_counter = 0;
    s_coordinator_id = 0;
    memset(s_coordinator_mac, 0, sizeof(s_coordinator_mac));

    set_state(MESH_STATE_IDLE);

    return ESP_OK;
}

bool mesh_is_initialized(void)
{
    return s_initialized;
}

mesh_role_t mesh_get_role(void)
{
    return s_role;
}

mesh_state_t mesh_get_state(void)
{
    return s_state;
}

int8_t mesh_get_slot(void)
{
    return s_slot_index;
}

uint8_t mesh_get_node_id(void)
{
    return s_node_id;
}

uint8_t mesh_get_node_count(void)
{
    uint8_t count = 0;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.active) {
            count++;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    /* Note: Self is already included in s_peers array (added when becoming
     * coordinator or receiving JOIN_ACK), so we don't add +1 here. */

    return count;
}

esp_err_t mesh_get_peer_info(uint8_t node_id, mesh_peer_info_t *info)
{
    if (info == NULL || node_id == 0 || node_id > MESH_MAX_NODES) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == node_id && s_peers[i].info.active) {
            *info = s_peers[i].info;
            xSemaphoreGive(s_peer_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(s_peer_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t mesh_send_audio(const uint8_t *data, uint16_t len, uint8_t audio_flags)
{
    if (!s_initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state != MESH_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len > MESH_MAX_OPUS_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    mesh_tx_item_t item;
    memcpy(item.data, data, len);
    item.len = len;
    item.audio_flags = audio_flags;
    item.timestamp_us = esp_timer_get_time();

    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        s_stats.packets_dropped++;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t mesh_register_audio_callback(mesh_audio_cb_t cb)
{
    s_audio_cb = cb;
    return ESP_OK;
}

esp_err_t mesh_register_state_callback(mesh_state_cb_t cb)
{
    s_state_cb = cb;
    return ESP_OK;
}

esp_err_t mesh_register_peer_callback(mesh_peer_cb_t cb)
{
    s_peer_cb = cb;
    return ESP_OK;
}

esp_err_t mesh_get_stats(mesh_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *stats = s_stats;
    return ESP_OK;
}

esp_err_t mesh_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;
    return ESP_OK;
}

uint32_t mesh_get_frame_counter(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    uint32_t frame_counter = s_frame_counter;
    taskEXIT_CRITICAL(&s_tdma_mux);
    return frame_counter;
}

int32_t mesh_get_time_to_slot_us(void)
{
    if (s_slot_index < 0 || s_state != MESH_STATE_ACTIVE) {
        return -1;
    }

    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t frame_start_us = s_frame_start_us;
    taskEXIT_CRITICAL(&s_tdma_mux);
    int64_t frame_elapsed = (now - frame_start_us) % MESH_FRAME_US;
    if (frame_elapsed < 0) {
        frame_elapsed += MESH_FRAME_US;
    }

    /* Calculate slot start within frame */
    int64_t slot_start = (int64_t)s_slot_index * MESH_SLOT_US;

    if (frame_elapsed < slot_start) {
        /* Slot is ahead in current frame */
        return (int32_t)(slot_start - frame_elapsed);
    } else {
        /* Slot passed, calculate for next frame */
        return (int32_t)(MESH_FRAME_US - frame_elapsed + slot_start);
    }
}

/* ============================================================================
 * Private Functions - Main Task
 * ============================================================================ */

static void mesh_task(void *arg)
{
    ESP_LOGI(TAG, "Mesh task started");

    int64_t scan_start = esp_timer_get_time();
    int64_t last_join_attempt = 0;
    int64_t last_keepalive = 0;
    int64_t last_status = 0;
    int64_t last_beacon = 0;
    int join_attempts = 0;
    bool saw_lower_mac = false;        /* Track if we saw a node with lower MAC */
    mesh_state_t prev_state = s_state; /* Track state changes for resets */

    while (1) {
        int64_t now = esp_timer_get_time();

        /* Reset scan variables when transitioning TO scanning state */
        if (s_state == MESH_STATE_SCANNING && prev_state != MESH_STATE_SCANNING) {
            ESP_LOGI(TAG, "Entering SCANNING state, resetting scan variables");
            scan_start = now;
            saw_lower_mac = false;
            last_beacon = 0;
        }
        if (s_state == MESH_STATE_JOINING && prev_state != MESH_STATE_JOINING) {
            join_attempts = 0;
            last_join_attempt = 0;
        }
        prev_state = s_state;

        switch (s_state) {
        case MESH_STATE_SCANNING:
            /* Listen for SYNC packets to find existing mesh */
            /* Also send periodic beacons (JOIN requests) for discovery */
            {
                mesh_rx_item_t rx;
                if (xQueueReceive(s_rx_queue, &rx, pdMS_TO_TICKS(50)) == pdTRUE) {
                    ESP_LOGI(TAG, "SCAN: RX packet type=0x%02X from " MACSTR " (RSSI=%d)",
                             rx.header.type, MAC2STR(rx.src_mac), rx.rssi);

                    handle_packet(&rx);

                    if (s_state != MESH_STATE_SCANNING) {
                        break;
                    }

                    /* If we received a SYNC, we found a mesh */
                    if (rx.header.type == MESH_PKT_SYNC &&
                        rx.header.payload_len == sizeof(mesh_sync_payload_t) &&
                        rx.header.src_id > 0 && rx.header.src_id <= MESH_MAX_NODES) {
                        ESP_LOGI(TAG, "Found existing mesh, coordinator=%d", rx.header.src_id);
                        s_coordinator_id = rx.header.src_id;
                        memcpy(s_coordinator_mac, rx.src_mac, sizeof(s_coordinator_mac));
                        set_state(MESH_STATE_JOINING);
                        join_attempts = 0;
                        last_join_attempt = 0;
                        break;
                    }
                    /* If we see a JOIN from another scanning node, check MAC for election */
                    else if (rx.header.type == MESH_PKT_JOIN) {
                        /* Compare MAC addresses - lower MAC will become coordinator */
                        if (memcmp(rx.src_mac, s_local_mac, 6) < 0) {
                            ESP_LOGI(TAG,
                                     "SCAN: Saw node with lower MAC, deferring coordinator role");
                            saw_lower_mac = true;
                            /* Reset scan timer to give lower MAC node time to become coordinator */
                            scan_start = now;
                        } else {
                            ESP_LOGI(TAG,
                                     "SCAN: Saw node with higher MAC, we may become coordinator");
                        }
                    }
                    /* Also transition if we see a JOIN_ACK (coordinator responded) */
                    else if (rx.header.type == MESH_PKT_JOIN_ACK) {
                        /* Already handled in handle_join_ack_packet */
                    }
                }

                /* Send periodic beacon (JOIN request) for discovery */
                /* Beacons help other scanning nodes discover us */
                if ((now - last_beacon) > (300 * 1000)) { /* Every 300ms */
                    send_join_request();
                    last_beacon = now;
                    ESP_LOGD(TAG, "SCAN: Sent beacon (JOIN request)");
                }

                /* Timeout - become coordinator */
                /* If we saw a lower MAC, extend timeout to let them become coordinator */
                int64_t timeout_us = MESH_SCAN_TIMEOUT_MS * 1000;
                if (saw_lower_mac) {
                    timeout_us += 1000 * 1000; /* Add 1 second if we saw lower MAC */
                }

                if ((now - scan_start) > timeout_us) {
                    ESP_LOGI(TAG, "No mesh found, becoming coordinator (saw_lower_mac=%d)",
                             saw_lower_mac);
                    s_role = MESH_ROLE_COORDINATOR;
                    s_node_id = 1;
                    s_slot_index = 0;
                    s_coordinator_id = 1;

                    /* Add self to peer list */
                    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
                    s_peers[0].info.node_id = 1;
                    memcpy(s_peers[0].info.mac_addr, s_local_mac, 6);
                    s_peers[0].info.slot_index = 0;
                    s_peers[0].info.active = true;
                    s_peers[0].info.last_seen_ms = now / 1000;
                    s_peer_count = 1;
                    xSemaphoreGive(s_peer_mutex);

                    /* Start TDMA frame timer */
                    advance_tdma_generation();
                    s_frame_start_us = esp_timer_get_time();
                    arm_frame_timer(MESH_FRAME_US);

                    set_state(MESH_STATE_ACTIVE);
                    ESP_LOGI(TAG, "ACTIVE as coordinator, node_id=%d, slot=%d", s_node_id,
                             s_slot_index);
                }
            }
            break;

        case MESH_STATE_JOINING:
            /* Send JOIN requests until we get ACK */
            {
                /* Process any incoming packets */
                mesh_rx_item_t rx;
                while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
                    handle_packet(&rx);
                    if (s_state != MESH_STATE_JOINING) {
                        break;
                    }
                }
                if (s_state != MESH_STATE_JOINING) {
                    break;
                }

                /* Send JOIN request periodically */
                if ((now - last_join_attempt) > (MESH_JOIN_RETRY_MS * 1000)) {
                    if (join_attempts < MESH_JOIN_RETRY_COUNT) {
                        send_join_request();
                        join_attempts++;
                        s_stats.join_attempts++;
                        last_join_attempt = now;
                        ESP_LOGI(TAG, "JOIN request #%d sent", join_attempts);
                    } else {
                        /* Give up and become coordinator */
                        ESP_LOGW(TAG, "JOIN timeout, becoming coordinator");
                        clear_transient_mesh_state();
                        s_role = MESH_ROLE_COORDINATOR;
                        s_node_id = 1;
                        s_slot_index = 0;
                        s_coordinator_id = 1;

                        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
                        memset(s_peers, 0, sizeof(s_peers));
                        s_peers[0].info.node_id = 1;
                        memcpy(s_peers[0].info.mac_addr, s_local_mac, 6);
                        s_peers[0].info.slot_index = 0;
                        s_peers[0].info.active = true;
                        s_peers[0].info.last_seen_ms = now / 1000;
                        s_peer_count = 1;
                        xSemaphoreGive(s_peer_mutex);

                        advance_tdma_generation();
                        s_frame_start_us = esp_timer_get_time();
                        arm_frame_timer(MESH_FRAME_US);

                        set_state(MESH_STATE_ACTIVE);
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }
            break;

        case MESH_STATE_ACTIVE:
            /* Normal operation */
            {
                /* Process incoming packets - drain all available
                 * Process more packets per iteration to prevent queue overflow
                 * during coordinator transitions */
                mesh_rx_item_t rx;
                int processed = 0;
                while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
                    handle_packet(&rx);
                    processed++;
                    /* Limit processing per iteration to avoid starving other tasks */
                    if (processed >= 32) {
                        break;
                    }
                }

                if (processed > 8) {
                    ESP_LOGD(TAG, "ACTIVE: processed %d packets (high load)", processed);
                }

                /* Pop from jitter buffer and deliver to audio callback */
                if (s_audio_cb) {
                    uint8_t audio_data[MESH_MAX_OPUS_BYTES];
                    uint16_t audio_len;
                    uint8_t src_id;
                    uint8_t audio_flags;

                    while (jitter_buffer_pop(audio_data, &audio_len, &src_id, &audio_flags)) {
                        /* Pass RSSI/Seq if available (requires updating struct/pop)
                         * For now, just pass timestamp for latency calc.
                         */
                        s_audio_cb(audio_data, audio_len, src_id, audio_flags, now);
                    }
                }

                /* Send KEEPALIVE periodically */
                if ((now - last_keepalive) > (MESH_KEEPALIVE_INTERVAL_MS * 1000)) {
                    send_keepalive();
                    last_keepalive = now;
                }

                /* Send STATUS periodically */
                if ((now - last_status) > (MESH_STATUS_INTERVAL_MS * 1000)) {
                    if (s_role == MESH_ROLE_COORDINATOR) {
                        update_speaker_grants();
                    }
                    send_status();
                    last_status = now;
                }

                /* Check for peer timeouts */
                check_peer_timeouts();

                QueueSetMemberHandle_t ready =
                    xQueueSelectFromSet(s_timer_queue_set, pdMS_TO_TICKS(5));
                if (ready == s_slot_semaphore && xSemaphoreTake(s_slot_semaphore, 0) == pdTRUE) {
                    service_tx_slot();
                } else if (ready == s_control_semaphore &&
                           xSemaphoreTake(s_control_semaphore, 0) == pdTRUE) {
                    service_control_window();
                } else if (ready == s_frame_event_queue) {
                    frame_event_t event;
                    if (xQueueReceive(s_frame_event_queue, &event, 0) == pdTRUE) {
                        service_frame_boundary(&event);
                    }
                }
            }
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

/* ============================================================================
 * Private Functions - TDMA Timer
 * ============================================================================ */

static void frame_timer_callback(void *arg)
{
    (void)arg;
    int64_t now_us = esp_timer_get_time();
    frame_event_t event;
    taskENTER_CRITICAL(&s_tdma_mux);
    event.timestamp_us = s_expected_frame_us;
    event.generation = s_frame_timer_generation;
    bool valid = event.generation == s_tdma_generation &&
                 llabs(now_us - event.timestamp_us) <= MESH_SLOT_US;
    if (valid) {
        s_expected_frame_us += MESH_FRAME_US;
    }
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid) {
        return;
    }
    if (!esp_timer_is_active(s_frame_timer)) {
        esp_timer_start_periodic(s_frame_timer, MESH_FRAME_US);
    }
    xQueueOverwrite(s_frame_event_queue, &event);
}

static void service_frame_boundary(const frame_event_t *event)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    bool valid = event->generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid || s_state != MESH_STATE_ACTIVE) {
        return;
    }

    int64_t frame_start_us = event->timestamp_us;
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t frame_delta_us = frame_start_us - s_frame_start_us;
    uint32_t elapsed_frames = frame_delta_us > MESH_FRAME_US
                                  ? (uint32_t)(frame_delta_us / MESH_FRAME_US)
                                  : 1;
    s_frame_counter += elapsed_frames;
    uint32_t frame_counter = s_frame_counter;
    s_frame_start_us = frame_start_us;
    taskEXIT_CRITICAL(&s_tdma_mux);

    if (s_role == MESH_ROLE_COORDINATOR &&
        (frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0) {
        taskENTER_CRITICAL(&s_tdma_mux);
        s_control_start_us = frame_start_us + (MESH_VOICE_SLOTS * MESH_SLOT_US);
        s_control_deadline_us = s_control_start_us + MESH_CONTROL_US - MESH_GUARD_US;
        s_control_generation = event->generation;
        taskEXIT_CRITICAL(&s_tdma_mux);
        int64_t control_delay_us = s_control_start_us - esp_timer_get_time();
        if (control_delay_us > 0) {
            esp_timer_start_once(s_control_timer, control_delay_us);
        } else {
            xSemaphoreGive(s_control_semaphore);
        }
    }

    /* Trigger audio TX in our slot */
    if (s_slot_index >= 0 && s_state == MESH_STATE_ACTIVE) {
        int64_t slot_offset_us = (int64_t)s_slot_index * MESH_SLOT_US;
        taskENTER_CRITICAL(&s_tdma_mux);
        s_slot_start_us = frame_start_us + slot_offset_us;
        s_slot_deadline_us = frame_start_us + slot_offset_us + MESH_SLOT_US - MESH_GUARD_US;
        s_slot_generation = event->generation;
        taskEXIT_CRITICAL(&s_tdma_mux);

        int64_t slot_delay_us = s_slot_start_us - esp_timer_get_time();
        if (slot_delay_us <= 0) {
            xSemaphoreGive(s_slot_semaphore);
        } else if (esp_timer_start_once(s_slot_timer, slot_delay_us) != ESP_OK) {
            s_stats.slot_misses++;
        }
    }

}

static void slot_timer_callback(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_slot_semaphore);
}

static void control_timer_callback(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_control_semaphore);
}

static void service_tx_slot(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t slot_start_us = s_slot_start_us;
    int64_t deadline_us = s_slot_deadline_us;
    bool valid = s_slot_generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);

    int64_t now_us = esp_timer_get_time();
    if (!valid || s_state != MESH_STATE_ACTIVE || s_slot_index < 0 || now_us < slot_start_us ||
        now_us > deadline_us) {
        s_stats.slot_misses++;
        return;
    }

    power_radio_slot_start();
    send_audio_in_slot();
    power_radio_slot_end();
}

static void service_control_window(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t control_start_us = s_control_start_us;
    int64_t control_deadline_us = s_control_deadline_us;
    bool valid = s_control_generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);

    int64_t now_us = esp_timer_get_time();
    if (valid && s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_COORDINATOR &&
        now_us >= control_start_us && now_us <= control_deadline_us) {
        send_sync();
    }
}

static void drain_slot_signal(void)
{
    QueueSetMemberHandle_t ready;
    while ((ready = xQueueSelectFromSet(s_timer_queue_set, 0)) != NULL) {
        if (ready == s_frame_event_queue) {
            frame_event_t event;
            xQueueReceive(s_frame_event_queue, &event, 0);
        } else {
            xSemaphoreTake((SemaphoreHandle_t)ready, 0);
        }
    }
}

static void advance_tdma_generation(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    s_tdma_generation++;
    taskEXIT_CRITICAL(&s_tdma_mux);
}

static esp_err_t arm_frame_timer(int64_t delay_us)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    s_frame_timer_generation = s_tdma_generation;
    s_expected_frame_us = esp_timer_get_time() + delay_us;
    taskEXIT_CRITICAL(&s_tdma_mux);
    return esp_timer_start_once(s_frame_timer, delay_us);
}

static void clear_transient_mesh_state(void)
{
    clear_speaker_state();
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    memset(s_seen_ring, 0, sizeof(s_seen_ring));
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    s_seen_head = 0;
    s_relay_head = 0;
    s_relay_tail = 0;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);

    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);
    memset(s_jitter_buffer, 0, sizeof(s_jitter_buffer));
    s_jitter_read_idx = 0;
    s_jitter_write_idx = 0;
    xSemaphoreGive(s_jitter_mutex);
    xQueueReset(s_tx_queue);
}

/* ============================================================================
 * Private Functions - ESP-NOW Callbacks
 * ============================================================================ */

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* Discard packets if mesh is not active (prevents queue overflow) */
    if (s_state == MESH_STATE_IDLE) {
        return;
    }

    if (len < sizeof(mesh_header_t)) {
        ESP_LOGW(TAG, "RX: packet too short (%d bytes)", len);
        return;
    }

    mesh_rx_item_t rx;
    memcpy(&rx.header, data, sizeof(mesh_header_t));
    memcpy(rx.src_mac, info->src_addr, 6);
    rx.rssi = info->rx_ctrl->rssi;
    rx.timestamp_us = esp_timer_get_time();

    /* Validate header */
    if (rx.header.version != MESH_PROTOCOL_VERSION) {
        ESP_LOGW(TAG, "RX: invalid version 0x%02X (expected 0x%02X)", rx.header.version,
                 MESH_PROTOCOL_VERSION);
        return;
    }

    /* Copy payload */
    uint16_t payload_len = rx.header.payload_len;
    if (payload_len > sizeof(rx.payload) ||
        payload_len > (uint16_t)(len - sizeof(mesh_header_t))) {
        ESP_LOGW(TAG, "RX: invalid payload length %u for %d-byte packet", payload_len, len);
        return;
    }
    if (payload_len > 0) {
        memcpy(rx.payload, data + sizeof(mesh_header_t), payload_len);
    }

    s_stats.packets_rx++;

    /* Only log non-audio packets at INFO level to reduce noise */
    if (rx.header.type != MESH_PKT_AUDIO) {
        ESP_LOGI(TAG, "RX: type=0x%02X src=%d seq=%d from " MACSTR " RSSI=%d", rx.header.type,
                 rx.header.src_id, rx.header.seq, MAC2STR(rx.src_mac), rx.rssi);
    } else {
        ESP_LOGD(TAG, "RX: AUDIO src=%d seq=%d RSSI=%d", rx.header.src_id, rx.header.seq, rx.rssi);
    }

    /* Queue for processing */
    if (xQueueSendFromISR(s_rx_queue, &rx, NULL) != pdTRUE) {
        s_stats.rx_queue_overflows++;
        /* Only log occasionally to avoid log flooding */
        if ((s_stats.rx_queue_overflows % 10) == 1) {
            ESP_LOGW(TAG, "RX queue full! (total overflows: %lu)", s_stats.rx_queue_overflows);
        }
    }
}

static void esp_now_send_cb(const esp_now_send_info_t *send_info, esp_now_send_status_t status)
{
    (void)send_info; /* Unused in Phase 2 */

    tx_completion_entry_t completion = {0};
    taskENTER_CRITICAL(&s_speaker_mux);
    while (s_tx_completion_tail != s_tx_completion_head) {
        tx_completion_entry_t *entry = &s_tx_completion_ring[s_tx_completion_tail];
        s_tx_completion_tail = (uint8_t)((s_tx_completion_tail + 1) % TX_COMPLETION_RING_SIZE);
        if (entry->valid) {
            completion = *entry;
            entry->valid = false;
            break;
        }
    }
    if (status != ESP_NOW_SEND_SUCCESS && completion.type == MESH_PKT_STATUS) {
        s_heard_bitmap |= completion.heard_bitmap;
        s_relay_bitmap |= completion.relay_bitmap;
    }
    taskEXIT_CRITICAL(&s_speaker_mux);

    if (status == ESP_NOW_SEND_SUCCESS) {
        s_stats.packets_tx++;
    } else {
        s_stats.packets_dropped++;
    }
}

/* ============================================================================
 * Private Functions - Packet Handling
 * ============================================================================ */

static void handle_packet(const mesh_rx_item_t *rx)
{
    ESP_LOGD(TAG, "handle_packet: type=0x%02X from src_id=%d", rx->header.type, rx->header.src_id);

    switch (rx->header.type) {
    case MESH_PKT_AUDIO:
        handle_audio_packet(rx);
        break;

    case MESH_PKT_JOIN:
        handle_join_packet(rx);
        break;

    case MESH_PKT_JOIN_ACK:
        handle_join_ack_packet(rx);
        break;

    case MESH_PKT_SYNC:
        handle_sync_packet(rx);
        break;

    case MESH_PKT_KEEPALIVE:
        handle_keepalive_packet(rx);
        break;

    case MESH_PKT_SLOT_MAP:
        handle_slot_map_packet(rx);
        break;

    case MESH_PKT_STATUS:
        handle_status_packet(rx);
        break;

    case MESH_PKT_SPEAKER_GRANT:
        if (rx->header.payload_len >= sizeof(mesh_speaker_grant_payload_t)) {
            const mesh_speaker_grant_payload_t *grant =
                (const mesh_speaker_grant_payload_t *)rx->payload;

            uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
            uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
            for (uint8_t i = 0; i < grant->speaker_count && i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
                ids[i] = grant->speaker_ids[i];
                masks[i] = grant->relay_masks[i];
            }
            speaker_state_set(ids, masks);
        }
        break;

    case MESH_PKT_SPEAKER_RELEASE:
        if (rx->header.payload_len >= sizeof(mesh_speaker_release_payload_t)) {
            const mesh_speaker_release_payload_t *release =
                (const mesh_speaker_release_payload_t *)rx->payload;

            uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS];
            uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS];
            speaker_state_get(ids, masks);
            for (uint8_t rel = 0; rel < release->speaker_count && rel < MESH_MAX_ACTIVE_SPEAKERS;
                 rel++) {
                for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
                    if (ids[i] == release->speaker_ids[rel]) {
                        ids[i] = 0;
                        masks[i] = 0;
                    }
                }
            }
            speaker_state_set(ids, masks);
        }
        break;

    case MESH_PKT_LEAVE:
        /* Handle peer leaving */
        ESP_LOGI(TAG, "Node %d leaving mesh", rx->header.src_id);
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        bool peer_removed = false;
        mesh_peer_info_t removed_peer = {0};
        for (int i = 0; i < MESH_MAX_NODES; i++) {
            if (s_peers[i].info.node_id == rx->header.src_id) {
                removed_peer = s_peers[i].info;
                s_peers[i].info.active = false;
                s_peer_count--;
                peer_removed = true;
                break;
            }
        }
        xSemaphoreGive(s_peer_mutex);
        if (peer_removed) {
            audio_play_notification(AUDIO_NOTIFY_PEER_LEAVE);
            if (s_peer_cb) {
                s_peer_cb(&removed_peer, false);
            }
        }
        if (s_role == MESH_ROLE_COORDINATOR) {
            send_slot_map();
        }
        break;

    default:
        ESP_LOGD(TAG, "Unknown packet type: 0x%02X", rx->header.type);
        break;
    }
}

static void handle_audio_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < 4) {
        return;
    }

    const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)rx->payload;
    uint16_t opus_len = rx->header.payload_len - 4;

    if (opus_len > MESH_MAX_OPUS_BYTES) {
        return;
    }

    /* Don't process our own packets */
    if (rx->header.src_id == s_node_id) {
        return;
    }

    if (seen_packet(rx->header.type, rx->header.src_id, rx->header.seq)) {
        return;
    }
    remember_packet(rx->header.type, rx->header.src_id, rx->header.seq);

    note_audio_activity(rx->header.src_id, audio->audio_flags);
    if (s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
    }

    /* Update peer last seen */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;

            /* Track packet loss */
            uint8_t expected_seq = s_peers[i].last_seq + 1;
            if (rx->header.seq != expected_seq && s_peers[i].packets_received > 0) {
                int lost = (rx->header.seq - expected_seq) & 0xFF;
                s_peers[i].packets_lost += lost;
                s_stats.audio_frames_lost += lost;
            }
            s_peers[i].last_seq = rx->header.seq;
            s_peers[i].packets_received++;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    /* Insert into jitter buffer */
    jitter_buffer_insert(audio->data, opus_len, rx->header.src_id, rx->header.seq,
                         audio->audio_flags);

    if (rx->header.ttl > 0 && (rx->header.flags & MESH_FLAG_RELAY_REQUEST) != 0) {
        bool relay_allowed = false;  /* Ungrated audio is not relayed (1-hop only) */

        if ((rx->header.flags & MESH_FLAG_SPEAKER_GRANTED) != 0) {
            /* Granted audio: check if we are in the relay mask for this speaker */
            uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS];
            uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS];
            speaker_state_get(ids, masks);
            for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
                if (ids[i] == rx->header.src_id &&
                    (masks[i] & node_bit(s_node_id)) != 0) {
                    relay_allowed = true;
                    break;
                }
            }
        }

        if (relay_allowed) {
            enqueue_relay_packet((const uint8_t *)&rx->header,
                                 (uint16_t)(sizeof(mesh_header_t) + rx->header.payload_len),
                                 (uint8_t)(rx->header.ttl - 1),
                                 (uint8_t)(rx->header.flags | MESH_FLAG_RELAYED));
        }
    }

    s_stats.audio_frames_rx++;

    /* Calculate latency (frame timestamp to receive time) */
    /* For now we just track receive timestamp - proper latency requires TX timestamp in packet */
}

static void handle_join_packet(const mesh_rx_item_t *rx)
{
    /* Only coordinator handles JOIN requests */
    if (s_role != MESH_ROLE_COORDINATOR) {
        ESP_LOGD(TAG, "Ignoring JOIN (not coordinator, role=%d)", s_role);
        return;
    }

    ESP_LOGI(TAG, "JOIN request from " MACSTR, MAC2STR(rx->src_mac));

    /* Check if already in mesh */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    bool already_joined = false;
    uint8_t existing_id = 0;
    int8_t existing_slot = -1;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (memcmp(s_peers[i].info.mac_addr, rx->src_mac, 6) == 0 && s_peers[i].info.active) {
            already_joined = true;
            existing_id = s_peers[i].info.node_id;
            existing_slot = s_peers[i].info.slot_index;
            break;
        }
    }

    if (already_joined) {
        xSemaphoreGive(s_peer_mutex);
        /* Re-send ACK in case previous was lost */
        send_join_ack(existing_id, existing_slot, rx->src_mac);
        return;
    }

    /* Assign new node ID and slot */
    uint8_t new_id = assign_node_id(rx->src_mac);
    int8_t new_slot = assign_slot(new_id);

    if (new_id == 0 || new_slot < 0) {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGW(TAG, "Cannot assign node - mesh full");
        return;
    }

    /* Add to peer list */
    bool peer_added = false;
    mesh_peer_info_t added_peer = {0};
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].info.active) {
            s_peers[i].info.node_id = new_id;
            memcpy(s_peers[i].info.mac_addr, rx->src_mac, 6);
            s_peers[i].info.slot_index = new_slot;
            s_peers[i].info.active = true;
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            s_peers[i].last_seq = 0;
            s_peers[i].packets_received = 0;
            s_peers[i].packets_lost = 0;
            s_peer_count++;
            added_peer = s_peers[i].info;
            peer_added = true;

            ESP_LOGI(TAG, "Assigned node_id=%d, slot=%d to " MACSTR, new_id, new_slot,
                     MAC2STR(rx->src_mac));

            break;
        }
    }

    xSemaphoreGive(s_peer_mutex);

    if (peer_added) {
        audio_play_notification(AUDIO_NOTIFY_PEER_JOIN);
        if (s_peer_cb) {
            s_peer_cb(&added_peer, true);
        }
    }

    /* Send JOIN_ACK */
    send_join_ack(new_id, new_slot, rx->src_mac);
    send_slot_map();
}

static void handle_join_ack_packet(const mesh_rx_item_t *rx)
{
    /* Accept JOIN_ACK in both SCANNING and JOINING states */
    /* SCANNING: We sent beacons and coordinator responded */
    /* JOINING: Normal join flow */
    if (s_state != MESH_STATE_JOINING && s_state != MESH_STATE_SCANNING) {
        return;
    }

    if (rx->header.payload_len != sizeof(mesh_join_ack_payload_t)) {
        return;
    }

    const mesh_join_ack_payload_t *ack = (const mesh_join_ack_payload_t *)rx->payload;
    if (ack->assigned_id == 0 || ack->assigned_id > MESH_MAX_NODES ||
        ack->slot_index == 0 || ack->slot_index >= MESH_VOICE_SLOTS ||
        ack->coordinator_id == 0 || ack->coordinator_id > MESH_MAX_NODES ||
        rx->header.src_id != ack->coordinator_id ||
        (s_state == MESH_STATE_JOINING &&
         memcmp(rx->src_mac, s_coordinator_mac, sizeof(s_coordinator_mac)) != 0)) {
        return;
    }

    ESP_LOGI(TAG, "JOIN_ACK received: node_id=%d, slot=%d, coordinator=%d", ack->assigned_id,
             ack->slot_index, ack->coordinator_id);

    s_node_id = ack->assigned_id;
    s_slot_index = ack->slot_index;
    s_coordinator_id = ack->coordinator_id;
    memcpy(s_coordinator_mac, rx->src_mac, sizeof(s_coordinator_mac));
    s_role = MESH_ROLE_PARTICIPANT;

    /* Add coordinator to peer list */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    s_peers[0].info.node_id = ack->coordinator_id;
    memcpy(s_peers[0].info.mac_addr, rx->src_mac, 6);
    s_peers[0].info.slot_index = 0; /* Coordinator is always slot 0 */
    s_peers[0].info.active = true;
    s_peers[0].info.last_seen_ms = rx->timestamp_us / 1000;
    s_peer_count = 1;
    xSemaphoreGive(s_peer_mutex);

    /* Add coordinator as ESP-NOW peer for unicast */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, rx->src_mac, 6);
    peer.channel = s_config.channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    s_stats.join_successes++;
    set_state(MESH_STATE_ACTIVE);

    ESP_LOGI(TAG, "ACTIVE as participant, node_id=%d, slot=%d", s_node_id, s_slot_index);
}

static void handle_sync_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len != sizeof(mesh_sync_payload_t) || rx->header.src_id == 0 ||
        rx->header.src_id > MESH_MAX_NODES) {
        return;
    }

    const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)rx->payload;

    if (s_role == MESH_ROLE_PARTICIPANT && s_state == MESH_STATE_ACTIVE &&
        (rx->header.src_id != s_coordinator_id ||
         memcmp(rx->src_mac, s_coordinator_mac, sizeof(s_coordinator_mac)) != 0)) {
        return;
    }
    if (memcmp(sync->coordinator_addr, rx->src_mac, sizeof(sync->coordinator_addr)) != 0) {
        return;
    }

    s_stats.sync_received++;

    /* Coordinator conflict resolution - if we're coordinator and receive SYNC from another node,
     * the node with lower MAC wins. Loser demotes to participant. */
    if (s_role == MESH_ROLE_COORDINATOR && rx->header.src_id != s_node_id) {
        const uint8_t *remote_addr = rx->src_mac;

        if (compare_mac(remote_addr, s_local_mac) < 0) {
            ESP_LOGW(TAG, "Coordinator conflict detected! Node %d has lower MAC - demoting",
                     rx->header.src_id);
            demote_to_participant(remote_addr, rx->header.src_id);
            /* After demotion, we're now a participant - process this SYNC normally below */
        } else {
            ESP_LOGW(TAG, "Coordinator conflict detected! We have lower MAC - ignoring their SYNC");
            return; /* Ignore SYNC from higher MAC, they should demote */
        }
    }

    /* Update frame counter and adjust timing */
    if (s_role == MESH_ROLE_PARTICIPANT) {
        taskENTER_CRITICAL(&s_tdma_mux);
        int64_t local_frame_start_us = s_frame_start_us;
        uint32_t local_frame_counter = s_frame_counter;
        taskEXIT_CRITICAL(&s_tdma_mux);

        int32_t frame_delta = (int32_t)(sync->frame_counter - local_frame_counter);
        int64_t expected_frame_start = local_frame_start_us + (MESH_FRAME_US * frame_delta);
        int64_t actual_frame_start =
            rx->timestamp_us - (MESH_VOICE_SLOTS * MESH_SLOT_US) -
            ESPNOW_SYNC_RX_LATENCY_US;
        int32_t drift_us = (int32_t)(actual_frame_start - expected_frame_start);

        /* Gradual drift correction (don't jump more than 500us per sync) */
        if (drift_us > 500)
            drift_us = 500;
        if (drift_us < -500)
            drift_us = -500;

        advance_tdma_generation();
        esp_timer_stop(s_slot_timer);
        esp_timer_stop(s_control_timer);
        drain_slot_signal();
        taskENTER_CRITICAL(&s_tdma_mux);
        s_frame_start_us = actual_frame_start;
        s_frame_counter = sync->frame_counter;
        taskEXIT_CRITICAL(&s_tdma_mux);
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = (now_us - actual_frame_start) % MESH_FRAME_US;
        if (elapsed_us < 0) {
            elapsed_us += MESH_FRAME_US;
        }
        int64_t next_frame_us = MESH_FRAME_US - elapsed_us;
        esp_timer_stop(s_frame_timer);
        arm_frame_timer(next_frame_us);
        s_stats.clock_drift_us = drift_us;

        if (abs(drift_us) > 100) {
            s_stats.sync_errors++;
            ESP_LOGD(TAG, "SYNC drift: %d us", drift_us);
        }
    }

    /* Update coordinator last seen */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

static void handle_keepalive_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < sizeof(mesh_keepalive_payload_t)) {
        return;
    }

    const mesh_keepalive_payload_t *ka = (const mesh_keepalive_payload_t *)rx->payload;

    /* Update peer info */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            s_peers[i].info.battery_pct = ka->battery_pct;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

static void handle_slot_map_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len != sizeof(mesh_slot_map_payload_t)) {
        return;
    }

    const mesh_slot_map_payload_t *slot_map = (const mesh_slot_map_payload_t *)rx->payload;
    uint8_t slot_count = slot_map->slot_count;
    if (slot_count > MESH_MAX_NODES) {
        slot_count = MESH_MAX_NODES;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (uint8_t slot = 0; slot < slot_count; slot++) {
        uint8_t node_id = slot_map->slot_ids[slot];
        if (node_id == 0) {
            continue;
        }

        if (node_id == s_node_id) {
            s_slot_index = (int8_t)slot;
        }

        for (int i = 0; i < MESH_MAX_NODES; i++) {
            if (s_peers[i].info.active && s_peers[i].info.node_id == node_id) {
                s_peers[i].info.slot_index = (int8_t)slot;
                break;
            }
        }
    }

    uint8_t sm_ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t sm_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    for (uint8_t i = 0; i < slot_map->active_speaker_count && i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        sm_ids[i] = slot_map->active_speaker_ids[i];
        sm_masks[i] = slot_map->relay_masks[i];
    }
    speaker_state_set(sm_ids, sm_masks);
    xSemaphoreGive(s_peer_mutex);
}

static void handle_status_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < sizeof(mesh_status_payload_t)) {
        return;
    }

    const mesh_status_payload_t *status = (const mesh_status_payload_t *)rx->payload;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            s_peers[i].info.rssi_dbm = rx->rssi;
            s_peers[i].info.heard_bitmap = status->heard_bitmap;
            s_peers[i].info.relay_bitmap = status->relay_bitmap;
            s_peers[i].info.battery_pct = status->battery_pct;
            s_peers[i].info.peer_count = status->peer_count;
            s_peers[i].info.fw_version = status->fw_version;
            s_peers[i].info.temperature_c = status->temperature_c;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

/* ============================================================================
 * Private Functions - Packet Transmission
 * ============================================================================ */

static esp_err_t send_packet(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    uint8_t buffer[256];
    mesh_header_t *header = (mesh_header_t *)buffer;

    header->version = MESH_PROTOCOL_VERSION;
    header->type = type;
    header->src_id = s_node_id;
    header->seq = s_tx_seq++;
    header->ttl = 0;
    header->flags = 0;
    header->payload_len = len;

    if (payload && len > 0) {
        memcpy(buffer + sizeof(mesh_header_t), payload, len);
    }

    uint8_t heard_bitmap = 0;
    uint8_t relay_bitmap = 0;
    if (type == MESH_PKT_STATUS && len == sizeof(mesh_status_payload_t)) {
        const mesh_status_payload_t *status = (const mesh_status_payload_t *)payload;
        heard_bitmap = status->heard_bitmap;
        relay_bitmap = status->relay_bitmap;
    }
    esp_err_t ret = tracked_esp_now_send(s_broadcast_mac, buffer, sizeof(mesh_header_t) + len,
                                         type, heard_bitmap, relay_bitmap);
    return ret;
}

static esp_err_t tracked_esp_now_send(const uint8_t *dest_mac, const uint8_t *data, size_t len,
                                      uint8_t type, uint8_t heard_bitmap, uint8_t relay_bitmap)
{
    taskENTER_CRITICAL(&s_speaker_mux);
    uint8_t entry_index = s_tx_completion_head;
    uint8_t next_head = (uint8_t)((entry_index + 1) % TX_COMPLETION_RING_SIZE);
    if (next_head == s_tx_completion_tail) {
        taskEXIT_CRITICAL(&s_speaker_mux);
        return ESP_ERR_NO_MEM;
    }
    s_tx_completion_ring[entry_index] = (tx_completion_entry_t){
        .type = type,
        .heard_bitmap = heard_bitmap,
        .relay_bitmap = relay_bitmap,
        .valid = true,
    };
    s_tx_completion_head = next_head;
    taskEXIT_CRITICAL(&s_speaker_mux);

    esp_err_t ret = esp_now_send(dest_mac, data, len);
    if (ret != ESP_OK) {
        taskENTER_CRITICAL(&s_speaker_mux);
        s_tx_completion_ring[entry_index].valid = false;
        while (s_tx_completion_tail != s_tx_completion_head &&
               !s_tx_completion_ring[s_tx_completion_tail].valid) {
            s_tx_completion_tail =
                (uint8_t)((s_tx_completion_tail + 1) % TX_COMPLETION_RING_SIZE);
        }
        taskEXIT_CRITICAL(&s_speaker_mux);
    }
    return ret;
}

static esp_err_t send_join_request(void)
{
    mesh_join_payload_t payload = {
        .capabilities = 0x01, /* Basic Opus support */
        .reserved = 0,
    };

    /* Use src_id=0 to indicate unassigned */
    uint8_t buffer[sizeof(mesh_header_t) + sizeof(mesh_join_payload_t)];
    mesh_header_t *header = (mesh_header_t *)buffer;

    header->version = MESH_PROTOCOL_VERSION;
    header->type = MESH_PKT_JOIN;
    header->src_id = 0; /* Unassigned */
    header->seq = s_tx_seq++;
    header->ttl = 0;
    header->flags = 0;
    header->payload_len = sizeof(payload);

    memcpy(buffer + sizeof(mesh_header_t), &payload, sizeof(payload));

    return tracked_esp_now_send(s_broadcast_mac, buffer, sizeof(buffer), MESH_PKT_JOIN, 0, 0);
}

static esp_err_t send_join_ack(uint8_t node_id, uint8_t slot, const uint8_t *dest_mac)
{
    mesh_join_ack_payload_t payload = {
        .assigned_id = node_id,
        .slot_index = slot,
        .coordinator_id = s_node_id,
    };

    /* Add peer for unicast if not already added */
    if (!esp_now_is_peer_exist(dest_mac)) {
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, dest_mac, 6);
        peer.channel = s_config.channel;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    uint8_t buffer[sizeof(mesh_header_t) + sizeof(mesh_join_ack_payload_t)];
    mesh_header_t *header = (mesh_header_t *)buffer;

    header->version = MESH_PROTOCOL_VERSION;
    header->type = MESH_PKT_JOIN_ACK;
    header->src_id = s_node_id;
    header->seq = s_tx_seq++;
    header->ttl = 0;
    header->flags = 0;
    header->payload_len = sizeof(payload);

    memcpy(buffer + sizeof(mesh_header_t), &payload, sizeof(payload));

    return tracked_esp_now_send(dest_mac, buffer, sizeof(buffer), MESH_PKT_JOIN_ACK, 0, 0);
}

static esp_err_t send_sync(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    uint32_t frame_counter = s_frame_counter;
    taskEXIT_CRITICAL(&s_tdma_mux);
    mesh_sync_payload_t payload = {
        .frame_counter = frame_counter,
        .drift_ppm = 0, /* TODO: Calculate actual drift */
    };
    memcpy(payload.coordinator_addr, s_local_mac, sizeof(payload.coordinator_addr));

    return send_packet(MESH_PKT_SYNC, &payload, sizeof(payload));
}

static esp_err_t send_keepalive(void)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 100, /* TODO: Get actual battery level */
        .reserved = 0,
    };

    return send_packet(MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
}

static esp_err_t send_slot_map(void)
{
    mesh_slot_map_payload_t payload = {0};
    payload.slot_count = MESH_MAX_NODES;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.active && s_peers[i].info.slot_index >= 0 &&
            s_peers[i].info.slot_index < MESH_MAX_NODES) {
            payload.slot_ids[s_peers[i].info.slot_index] = s_peers[i].info.node_id;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    uint8_t sm_ids[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t sm_masks[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(sm_ids, sm_masks);
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (sm_ids[i] != 0) {
            payload.active_speaker_count++;
        }
    }
    memcpy(payload.active_speaker_ids, sm_ids, sizeof(payload.active_speaker_ids));
    memcpy(payload.relay_masks, sm_masks, sizeof(payload.relay_masks));

    return send_packet(MESH_PKT_SLOT_MAP, &payload, sizeof(payload));
}

static esp_err_t send_status(void)
{
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;
    status_bitmaps_snapshot_and_clear(&heard_bitmap, &relay_bitmap);

    mesh_status_payload_t payload = {
        .battery_pct = 100,
        .rssi_dbm = 127,
        .peer_count = mesh_get_node_count(),
        .fw_version = MESH_PROTOCOL_VERSION,
        .temperature_c = 127,
        .heard_bitmap = heard_bitmap,
        .relay_bitmap = relay_bitmap,
        .active_speakers = 0,
    };

    uint8_t st_ids[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(st_ids, NULL);
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (st_ids[i] != 0) {
            payload.active_speakers++;
        }
    }

    esp_err_t ret = send_packet(MESH_PKT_STATUS, &payload, sizeof(payload));
    if (ret != ESP_OK) {
        taskENTER_CRITICAL(&s_speaker_mux);
        s_heard_bitmap |= heard_bitmap;
        s_relay_bitmap |= relay_bitmap;
        taskEXIT_CRITICAL(&s_speaker_mux);
    }
    return ret;
}

static esp_err_t send_audio_in_slot(void)
{
    mesh_tx_item_t tx_item;

    /* Check if we have audio to send */
    if (xQueueReceive(s_tx_queue, &tx_item, 0) != pdTRUE) {
        if (!relay_queue_empty()) {
            relay_entry_t *entry = &s_relay_ring[s_relay_tail];
            mesh_header_t *relay_header = (mesh_header_t *)entry->data;
            esp_err_t relay_ret = tracked_esp_now_send(
                s_broadcast_mac, entry->data, entry->len,
                ((const mesh_header_t *)entry->data)->type, 0, 0);

            if (relay_ret == ESP_OK) {
                taskENTER_CRITICAL(&s_speaker_mux);
                s_relay_bitmap |= node_bit(relay_header->src_id);
                taskEXIT_CRITICAL(&s_speaker_mux);
            } else {
                s_stats.slot_misses++;
            }

            s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
            return relay_ret;
        }

        /* No audio queued - don't TX silence */
        return ESP_OK;
    }

    /* Build audio packet */
    uint8_t buffer[sizeof(mesh_header_t) + sizeof(mesh_audio_payload_t)];
    mesh_header_t *header = (mesh_header_t *)buffer;
    mesh_audio_payload_t *audio = (mesh_audio_payload_t *)(buffer + sizeof(mesh_header_t));

    header->version = MESH_PROTOCOL_VERSION;
    header->type = MESH_PKT_AUDIO;
    header->src_id = s_node_id;
    header->seq = s_tx_seq++;
    header->ttl = MESH_AUDIO_TTL_DEFAULT;
    header->flags = MESH_FLAG_RELAY_REQUEST;
    header->payload_len = 4 + tx_item.len;

    uint8_t slot_ids[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(slot_ids, NULL);
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (slot_ids[i] == s_node_id) {
            header->flags |= MESH_FLAG_SPEAKER_GRANTED;
            break;
        }
    }

    note_audio_activity(s_node_id, tx_item.audio_flags);
    if (s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
    }

    audio->codec = 0x01; /* Opus */
    audio->frame_ms = MESH_FRAME_MS;
    audio->stream_id = s_node_id;
    audio->audio_flags = tx_item.audio_flags;
    memcpy(audio->data, tx_item.data, tx_item.len);

    esp_err_t ret = tracked_esp_now_send(s_broadcast_mac, buffer,
                                         sizeof(mesh_header_t) + 4 + tx_item.len,
                                         MESH_PKT_AUDIO, 0, 0);

    if (ret == ESP_OK) {
        s_stats.audio_frames_tx++;
    } else {
        s_stats.slot_misses++;
    }

    return ret;
}

/* ============================================================================
 * Private Functions - Utility
 * ============================================================================ */

static void set_state(mesh_state_t new_state)
{
    mesh_state_t old_state = s_state;
    s_state = new_state;

    if (s_state_cb) {
        s_state_cb(old_state, new_state);
    }

    ESP_LOGI(TAG, "State: %d -> %d", old_state, new_state);
}

static uint8_t assign_node_id(const uint8_t *mac)
{
    /* Find first available ID (1-8) */
    /* NOTE: Caller must hold s_peer_mutex! */
    bool used[MESH_MAX_NODES + 1] = {false};

    used[s_node_id] = true; /* Our ID is used */

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.active && s_peers[i].info.node_id <= MESH_MAX_NODES) {
            used[s_peers[i].info.node_id] = true;
        }
    }

    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        if (!used[id]) {
            return id;
        }
    }

    return 0; /* No ID available */
}

static int8_t assign_slot(uint8_t node_id)
{
    /* Simple slot assignment: node_id - 1 maps to slot */
    /* This can be made more sophisticated in Phase 4 */
    if (node_id == 0 || node_id > MESH_MAX_NODES) {
        return -1;
    }

    return (int8_t)(node_id - 1);
}

static void check_peer_timeouts(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    mesh_peer_info_t timed_out[MESH_MAX_NODES];
    int timed_out_count = 0;
    bool coordinator_lost = false;

    /* Only mutate the peer table while holding its mutex. Every operation that
     * can send, take another lock, or invoke application code happens below. */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].info.active || s_peers[i].info.node_id == s_node_id) {
            continue;
        }

        int64_t age = now_ms - s_peers[i].info.last_seen_ms;
        if (age <= MESH_NODE_TIMEOUT_MS) {
            continue;
        }

        ESP_LOGW(TAG, "Node %d timeout (last seen %lld ms ago)", s_peers[i].info.node_id, age);
        timed_out[timed_out_count++] = s_peers[i].info;
        coordinator_lost = coordinator_lost ||
                           (s_role == MESH_ROLE_PARTICIPANT &&
                            s_peers[i].info.node_id == s_coordinator_id);
        s_peers[i].info.active = false;
        if (s_peer_count > 0) {
            s_peer_count--;
        }
        s_stats.node_timeouts++;
    }

    xSemaphoreGive(s_peer_mutex);

    for (int t = 0; t < timed_out_count; t++) {
        uint8_t timed_out_id = timed_out[t].node_id;
        bool was_speaker = false;
        taskENTER_CRITICAL(&s_speaker_mux);
        s_active_speaker_deadline_ms[timed_out_id] = 0;
        for (int speaker = 0; speaker < MESH_MAX_ACTIVE_SPEAKERS; speaker++) {
            if (s_active_speaker_ids[speaker] == timed_out_id) {
                s_active_speaker_ids[speaker] = 0;
                s_relay_masks[speaker] = 0;
                was_speaker = true;
            }
        }
        taskEXIT_CRITICAL(&s_speaker_mux);

        if (was_speaker && s_role == MESH_ROLE_COORDINATOR) {
            send_speaker_release_for(timed_out_id);
        }
        audio_play_notification(AUDIO_NOTIFY_PEER_LEAVE);
        if (s_peer_cb) {
            s_peer_cb(&timed_out[t], false);
        }
    }

    if (timed_out_count > 0 && s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
        send_slot_map();
    }

    if (coordinator_lost) {
        ESP_LOGI(TAG, "Coordinator lost, returning to SCANNING");
        advance_tdma_generation();
        esp_timer_stop(s_slot_timer);
        esp_timer_stop(s_frame_timer);
        esp_timer_stop(s_control_timer);
        drain_slot_signal();
        xQueueReset(s_rx_queue);
        clear_transient_mesh_state();
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        memset(s_peers, 0, sizeof(s_peers));
        s_peer_count = 0;
        xSemaphoreGive(s_peer_mutex);
        s_role = MESH_ROLE_NONE;
        s_node_id = 0;
        s_slot_index = -1;
        s_coordinator_id = 0;
        memset(s_coordinator_mac, 0, sizeof(s_coordinator_mac));
        set_state(MESH_STATE_SCANNING);
    }
}

static void jitter_buffer_insert(const uint8_t *data, uint16_t len, uint8_t src_id, uint8_t seq,
                                 uint8_t audio_flags)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);

    /* Simple ring buffer insert */
    jitter_entry_t *entry = &s_jitter_buffer[s_jitter_write_idx];

    if (entry->valid) {
        /* Overwriting unread entry - buffer overrun */
        s_stats.jitter_overruns++;
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    entry->src_id = src_id;
    entry->seq = seq;
    entry->audio_flags = audio_flags;
    entry->timestamp_us = esp_timer_get_time();
    entry->valid = true;

    s_jitter_write_idx = (s_jitter_write_idx + 1) % MESH_JITTER_BUFFER_DEPTH;

    /* Update depth stat */
    uint8_t depth = 0;
    for (int i = 0; i < MESH_JITTER_BUFFER_DEPTH; i++) {
        if (s_jitter_buffer[i].valid)
            depth++;
    }
    s_stats.jitter_depth = depth;

    xSemaphoreGive(s_jitter_mutex);
}

static bool jitter_buffer_pop(uint8_t *data, uint16_t *len, uint8_t *src_id,
                              uint8_t *audio_flags)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);

    jitter_entry_t *entry = &s_jitter_buffer[s_jitter_read_idx];

    if (!entry->valid) {
        xSemaphoreGive(s_jitter_mutex);
        s_stats.jitter_underruns++;
        return false;
    }

    /* Check if packet is too old (more than 2 frames late) */
    int64_t age_us = esp_timer_get_time() - entry->timestamp_us;
    if (age_us > (MESH_FRAME_US * 3)) {
        s_stats.audio_frames_late++;
        entry->valid = false;
        s_jitter_read_idx = (s_jitter_read_idx + 1) % MESH_JITTER_BUFFER_DEPTH;
        xSemaphoreGive(s_jitter_mutex);
        return false; /* Drop late packet */
    }

    memcpy(data, entry->data, entry->len);
    *len = entry->len;
    *src_id = entry->src_id;
    *audio_flags = entry->audio_flags;

    entry->valid = false;
    s_jitter_read_idx = (s_jitter_read_idx + 1) % MESH_JITTER_BUFFER_DEPTH;

    xSemaphoreGive(s_jitter_mutex);
    return true;
}

static int compare_mac(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6);
}

static void demote_to_participant(const uint8_t *coordinator_mac, uint8_t coordinator_id)
{
    ESP_LOGW(TAG, "Demoting from coordinator to participant (new coord=%d)", coordinator_id);

    advance_tdma_generation();
    esp_timer_stop(s_slot_timer);
    esp_timer_stop(s_frame_timer);
    esp_timer_stop(s_control_timer);
    drain_slot_signal();
    clear_transient_mesh_state();

    /* Clear RX queue to prevent overflow from stale packets */
    mesh_rx_item_t rx;
    int cleared = 0;
    while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
        cleared++;
    }
    if (cleared > 0) {
        ESP_LOGI(TAG, "Cleared %d packets from RX queue during demotion", cleared);
    }

    /* Update role and coordinator tracking */
    s_role = MESH_ROLE_PARTICIPANT;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = coordinator_id;
    memcpy(s_coordinator_mac, coordinator_mac, sizeof(s_coordinator_mac));

    /* Clear peer list except for the new coordinator */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    /* First, clear all peers */
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        s_peers[i].info.active = false;
    }
    s_peer_count = 0;

    /* Add the new coordinator to our peer list */
    s_peers[0].info.node_id = coordinator_id;
    memcpy(s_peers[0].info.mac_addr, coordinator_mac, 6);
    s_peers[0].info.slot_index = 0;
    s_peers[0].info.active = true;
    s_peers[0].info.last_seen_ms = esp_timer_get_time() / 1000;
    s_peer_count = 1;

    xSemaphoreGive(s_peer_mutex);

    /* Transition to JOINING state to request a node_id from the new coordinator */
    set_state(MESH_STATE_JOINING);

    ESP_LOGI(TAG, "Now in JOINING state, will request node_id from coordinator %d", coordinator_id);
}
