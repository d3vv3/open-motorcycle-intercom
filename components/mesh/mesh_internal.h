#ifndef OMI_MESH_INTERNAL_H
#define OMI_MESH_INTERNAL_H

/**
 * @file mesh_internal.h
 * @brief Private composed state for the ESP-NOW mesh implementation.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_now.h"
#include "esp_timer.h"

#include "mesh.h"
#include "mesh_core.h"

#define MESH_TASK_STACK_SIZE 4096
#define MESH_TASK_PRIORITY   6
#define MESH_TASK_CORE       0

#define MESH_TX_QUEUE_SIZE           4
#define MESH_RX_QUEUE_SIZE           32
#define MESH_SCAN_TIMEOUT_MS         2000
#define MESH_JOIN_TIMEOUT_MS         5000
#define MESH_STATUS_INTERVAL_MS      1000
#define RELAY_RING_SIZE              16
#define ACTIVE_SPEAKER_TIMEOUT_MS    1500
#define CONTENTION_MIN_INTERVAL_MS   100
#define CONTENTION_JITTER_MS         100
#define TX_QUIESCE_TIMEOUT_MS        250
#define RX_QUIESCE_TIMEOUT_MS        50
#define TASK_QUIESCE_TIMEOUT_MS      250
#define CONTROL_MAX_PRIORITY_WAIT_MS 500

#define MESH_FRAME_US             (MESH_FRAME_MS * 1000)
#define MESH_SLOT_US              (MESH_SLOT_MS * 1000)
#define MESH_CONTROL_US           (MESH_CONTROL_MS * 1000)
#define ESPNOW_SYNC_RX_LATENCY_US 500

typedef struct {
    uint8_t data[MESH_MAX_OPUS_BYTES];
    uint16_t len;
    uint8_t audio_flags;
    int64_t timestamp_us;
} mesh_tx_item_t;

typedef struct {
    mesh_header_t header;
    uint8_t payload[128];
    uint8_t src_mac[6];
    int8_t rssi;
    int64_t timestamp_us;
} mesh_rx_item_t;

typedef struct {
    uint8_t data[MESH_MAX_OPUS_BYTES];
    uint16_t len;
    uint8_t src_id;
    uint8_t seq;
    uint8_t audio_flags;
    int64_t timestamp_us; /* Original ESP-NOW receive timestamp. */
    int64_t enqueued_us;  /* Local insertion time used for jitter expiry. */
    bool valid;
} jitter_entry_t;

typedef struct {
    mesh_peer_info_t info;
    mesh_core_seq8_t rx_seq;
    uint32_t packets_received;
    uint32_t packets_lost;
} peer_tracking_t;

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
    bool audio_origin;
    bool active;
} tx_inflight_t;

typedef struct {
    const uint8_t *dest_mac;
    uint8_t *data;
    size_t len;
    uint8_t type;
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;
    uint8_t *sequence;
    bool audio_origin;
} tracked_esp_now_send_request_t;

typedef enum {
    /* NOTE: Higher priorities are served first; overdue control packets retain FIFO order. */
    CONTROL_PRIORITY_PERIODIC = 1,
    CONTROL_PRIORITY_TOPOLOGY,
    CONTROL_PRIORITY_LIFECYCLE,
    CONTROL_PRIORITY_SYNC,
} control_priority_t;

typedef struct {
    uint8_t data[sizeof(mesh_header_t) + sizeof(mesh_slot_map_payload_t)];
    uint8_t dest_mac[6];
    uint16_t len;
    uint32_t order;
    int64_t enqueued_us;
    uint8_t type;
    uint8_t priority;
} control_tx_item_t;

typedef struct {
    bool initialized;
    mesh_config_t config;
    mesh_state_t state;
    mesh_role_t role;
    mesh_stats_t stats;
    portMUX_TYPE stats_mux;

    uint8_t local_mac[6];
    uint8_t node_id;
    int8_t slot_index;
    portMUX_TYPE speaker_mux;
    portMUX_TYPE tdma_mux;
    uint8_t active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;
    int64_t active_speaker_deadline_ms[MESH_MAX_NODES + 1];

    peer_tracking_t peers[MESH_MAX_NODES];
    uint8_t peer_count;
    uint8_t coordinator_id;
    uint8_t coordinator_mac[6];
    SemaphoreHandle_t peer_mutex;

    uint32_t frame_counter;
    int64_t frame_start_us;
    esp_timer_handle_t frame_timer;
    esp_timer_handle_t slot_timer;
    esp_timer_handle_t control_timer;
    int64_t slot_start_us;
    int64_t slot_deadline_us;
    int64_t control_start_us;
    int64_t control_deadline_us;
    uint32_t tdma_generation; /* Invalidates callbacks scheduled before resync or stop. */
    uint32_t frame_timer_generation;
    int64_t expected_frame_us;
    uint32_t slot_generation;
    uint32_t control_generation;
    SemaphoreHandle_t frame_timer_mutex;
    SemaphoreHandle_t slot_semaphore;
    SemaphoreHandle_t control_semaphore;
    QueueHandle_t frame_event_queue;
    QueueSetHandle_t timer_queue_set;

    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    portMUX_TYPE control_queue_mux;
    control_tx_item_t control_queue[MESH_CONTROL_QUEUE_CAPACITY];
    uint8_t control_queue_count;
    uint32_t control_queue_order;
    int64_t contention_next_tx_us;

    portMUX_TYPE transport_mux;
    tx_inflight_t tx_inflight;
    SemaphoreHandle_t tx_done_semaphore;
    SemaphoreHandle_t task_stopped_semaphore;
    SemaphoreHandle_t audio_producer_mutex;
    SemaphoreHandle_t stop_mutex;
    bool stopping;
    bool rx_enabled;
    uint8_t rx_callbacks_active;
    bool send_callback_enabled;
    uint8_t tx_callbacks_active;
    bool esp_now_ready;
    esp_now_send_status_t last_tx_status;

    mesh_core_dedupe_t dedupe;
    relay_entry_t relay_ring[RELAY_RING_SIZE];
    uint8_t relay_head;
    uint8_t relay_tail;
    jitter_entry_t jitter_buffer[MESH_JITTER_BUFFER_DEPTH];
    uint8_t jitter_read_idx;
    uint8_t jitter_write_idx;
    SemaphoreHandle_t jitter_mutex;

    mesh_audio_cb_t audio_cb;
    mesh_state_cb_t state_cb;
    mesh_peer_cb_t peer_cb;
    TaskHandle_t mesh_task;
    uint8_t control_tx_seq;
    uint8_t audio_tx_seq;
} mesh_context_t;

extern mesh_context_t s_mesh;
extern const char *const TAG;
extern const uint8_t s_broadcast_mac[6];

#define s_initialized                s_mesh.initialized
#define s_config                     s_mesh.config
#define s_state                      s_mesh.state
#define s_role                       s_mesh.role
#define s_stats                      s_mesh.stats
#define s_stats_mux                  s_mesh.stats_mux
#define s_local_mac                  s_mesh.local_mac
#define s_node_id                    s_mesh.node_id
#define s_slot_index                 s_mesh.slot_index
#define s_speaker_mux                s_mesh.speaker_mux
#define s_tdma_mux                   s_mesh.tdma_mux
#define s_active_speaker_ids         s_mesh.active_speaker_ids
#define s_relay_masks                s_mesh.relay_masks
#define s_heard_bitmap               s_mesh.heard_bitmap
#define s_relay_bitmap               s_mesh.relay_bitmap
#define s_active_speaker_deadline_ms s_mesh.active_speaker_deadline_ms
#define s_peers                      s_mesh.peers
#define s_peer_count                 s_mesh.peer_count
#define s_coordinator_id             s_mesh.coordinator_id
#define s_coordinator_mac            s_mesh.coordinator_mac
#define s_peer_mutex                 s_mesh.peer_mutex
#define s_frame_counter              s_mesh.frame_counter
#define s_frame_start_us             s_mesh.frame_start_us
#define s_frame_timer                s_mesh.frame_timer
#define s_slot_timer                 s_mesh.slot_timer
#define s_control_timer              s_mesh.control_timer
#define s_slot_start_us              s_mesh.slot_start_us
#define s_slot_deadline_us           s_mesh.slot_deadline_us
#define s_control_start_us           s_mesh.control_start_us
#define s_control_deadline_us        s_mesh.control_deadline_us
#define s_tdma_generation            s_mesh.tdma_generation
#define s_frame_timer_generation     s_mesh.frame_timer_generation
#define s_expected_frame_us          s_mesh.expected_frame_us
#define s_slot_generation            s_mesh.slot_generation
#define s_control_generation         s_mesh.control_generation
#define s_frame_timer_mutex          s_mesh.frame_timer_mutex
#define s_slot_semaphore             s_mesh.slot_semaphore
#define s_control_semaphore          s_mesh.control_semaphore
#define s_frame_event_queue          s_mesh.frame_event_queue
#define s_timer_queue_set            s_mesh.timer_queue_set
#define s_tx_queue                   s_mesh.tx_queue
#define s_rx_queue                   s_mesh.rx_queue
#define s_control_queue_mux          s_mesh.control_queue_mux
#define s_control_queue              s_mesh.control_queue
#define s_control_queue_count        s_mesh.control_queue_count
#define s_control_queue_order        s_mesh.control_queue_order
#define s_contention_next_tx_us      s_mesh.contention_next_tx_us
#define s_transport_mux              s_mesh.transport_mux
#define s_tx_inflight                s_mesh.tx_inflight
#define s_tx_done_semaphore          s_mesh.tx_done_semaphore
#define s_task_stopped_semaphore     s_mesh.task_stopped_semaphore
#define s_audio_producer_mutex       s_mesh.audio_producer_mutex
#define s_stop_mutex                 s_mesh.stop_mutex
#define s_stopping                   s_mesh.stopping
#define s_rx_enabled                 s_mesh.rx_enabled
#define s_rx_callbacks_active        s_mesh.rx_callbacks_active
#define s_send_callback_enabled      s_mesh.send_callback_enabled
#define s_tx_callbacks_active        s_mesh.tx_callbacks_active
#define s_esp_now_ready              s_mesh.esp_now_ready
#define s_last_tx_status             s_mesh.last_tx_status
#define s_dedupe                     s_mesh.dedupe
#define s_relay_ring                 s_mesh.relay_ring
#define s_relay_head                 s_mesh.relay_head
#define s_relay_tail                 s_mesh.relay_tail
#define s_jitter_buffer              s_mesh.jitter_buffer
#define s_jitter_read_idx            s_mesh.jitter_read_idx
#define s_jitter_write_idx           s_mesh.jitter_write_idx
#define s_jitter_mutex               s_mesh.jitter_mutex
#define s_audio_cb                   s_mesh.audio_cb
#define s_state_cb                   s_mesh.state_cb
#define s_peer_cb                    s_mesh.peer_cb
#define s_mesh_task                  s_mesh.mesh_task
#define s_control_tx_seq             s_mesh.control_tx_seq
#define s_audio_tx_seq               s_mesh.audio_tx_seq

#define STATS_ADD(field, value)                                                                    \
    do {                                                                                           \
        taskENTER_CRITICAL(&s_stats_mux);                                                          \
        s_stats.field += (value);                                                                  \
        taskEXIT_CRITICAL(&s_stats_mux);                                                           \
    } while (0)
#define STATS_INC(field) STATS_ADD(field, 1)
#define STATS_SET(field, value)                                                                    \
    do {                                                                                           \
        taskENTER_CRITICAL(&s_stats_mux);                                                          \
        s_stats.field = (value);                                                                   \
        taskEXIT_CRITICAL(&s_stats_mux);                                                           \
    } while (0)

bool relay_queue_empty(void);
bool enqueue_relay_packet(const uint8_t *data, uint16_t len, uint8_t ttl, uint8_t flags);
void clear_speaker_state(void);
void speaker_state_get(uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                       uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS]);
void speaker_state_set(const uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                       const uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS]);
void status_bitmaps_snapshot_and_clear(uint8_t *heard, uint8_t *relayed);
void note_audio_activity(uint8_t src_id, uint8_t audio_flags);
uint8_t compute_relay_mask(uint8_t speaker_id);
void send_speaker_release_for(uint8_t speaker_id);
void update_speaker_grants(void);
void clear_transient_mesh_state(void);
void jitter_buffer_insert(const uint8_t *data, uint16_t len, uint8_t src_id, uint8_t seq,
                          uint8_t audio_flags, int64_t timestamp_us);
bool jitter_buffer_pop(uint8_t *data, uint16_t *len, uint8_t *src_id, uint8_t *audio_flags,
                       int64_t *timestamp_us);
esp_err_t send_audio_in_slot(void);

esp_err_t init_esp_now_transport(void);
void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void esp_now_send_cb(const esp_now_send_info_t *send_info, esp_now_send_status_t status);
esp_err_t enqueue_control_packet(uint8_t type, const void *payload, uint16_t len,
                                 const uint8_t *dest_mac);
bool dequeue_control_packet(control_tx_item_t *item);
void reset_control_queue(void);
void mesh_transport_restore_status_bitmaps(const control_tx_item_t *item);
esp_err_t send_packet(mesh_pkt_type_t type, const void *payload, uint16_t len);
esp_err_t send_packet_immediate(mesh_pkt_type_t type, const void *payload, uint16_t len,
                                const uint8_t *dest_mac);
esp_err_t tracked_esp_now_send(const tracked_esp_now_send_request_t *request);
bool wait_for_tx_idle(TickType_t timeout_ticks);
bool wait_for_rx_quiesced(TickType_t timeout_ticks);
void force_cleanup_esp_now_transport(void);
esp_err_t send_join_request(void);
esp_err_t send_join_ack(uint8_t node_id, uint8_t slot, const uint8_t *dest_mac);
esp_err_t send_sync(void);
esp_err_t send_keepalive(void);
esp_err_t send_slot_map(void);
esp_err_t send_status(void);

bool owns_control_window(uint32_t frame_counter);
void frame_timer_callback(void *arg);
void slot_timer_callback(void *arg);
void control_timer_callback(void *arg);
void service_frame_boundary(const frame_event_t *event);
void service_tx_slot(void);
void service_control_window(void);
void drain_slot_signal(void);
void advance_tdma_generation(void);
esp_err_t arm_frame_timer(int64_t delay_us);
esp_err_t arm_frame_timer_at(int64_t boundary_us);
esp_err_t arm_frame_timer_at_generation(int64_t boundary_us, uint32_t generation);
esp_err_t arm_frame_timer_at_generation_locked(int64_t boundary_us, uint32_t generation);

void mesh_task(void *arg);
void handle_packet(const mesh_rx_item_t *rx);
void handle_join_packet(const mesh_rx_item_t *rx);
void handle_join_ack_packet(const mesh_rx_item_t *rx);
void handle_sync_packet(const mesh_rx_item_t *rx);
void handle_keepalive_packet(const mesh_rx_item_t *rx);
void handle_slot_map_packet(const mesh_rx_item_t *rx);
void handle_status_packet(const mesh_rx_item_t *rx);
void handle_audio_packet(const mesh_rx_item_t *rx);
void set_state(mesh_state_t new_state);
void check_peer_timeouts(void);
void demote_to_participant(const uint8_t *coordinator_mac, uint8_t coordinator_id);

#endif
