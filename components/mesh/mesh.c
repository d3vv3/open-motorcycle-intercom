/** @file mesh.c @brief ESP-NOW mesh implementation. */

#include <inttypes.h>
#include <string.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#ifdef MESH_S31_COEX_PREFER_WIFI
#include "esp_coexist.h"
#endif

#include "mesh_internal.h"

const char *const TAG = "mesh";
const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#ifdef MESH_S31_COEX_PREFER_WIFI
static bool s_coex_restore_pending;

static void restore_coex_preference(void)
{
    if (!s_coex_restore_pending) return;
    esp_err_t ret = esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    if (ret == ESP_OK) {
        s_coex_restore_pending = false;
        ESP_LOGI(TAG, "Coexistence preference BALANCE: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGE(TAG, "Coexistence preference BALANCE: %s", esp_err_to_name(ret));
    }
}
#endif

mesh_context_t s_mesh = {
    .config = MESH_CONFIG_DEFAULT(),
    .slot_index = -1,
    .stats_mux = portMUX_INITIALIZER_UNLOCKED,
    .speaker_mux = portMUX_INITIALIZER_UNLOCKED,
    .tdma_mux = portMUX_INITIALIZER_UNLOCKED,
    .slot_state = {.completed = true, .assigned_index = -1},
    .control_queue_mux = portMUX_INITIALIZER_UNLOCKED,
    .transport_mux = portMUX_INITIALIZER_UNLOCKED,
    .last_tx_status = ESP_NOW_SEND_SUCCESS,
};

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

    ESP_LOGI(TAG, "Initializing mesh subsystem");

    s_mesh.slot_state = (mesh_tx_slot_state_t){.completed = true, .assigned_index = -1};

    ESP_ERROR_CHECK(esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "Local MAC: " MACSTR, MAC2STR(s_local_mac));

    s_peer_mutex = xSemaphoreCreateMutex();
    s_jitter_mutex = xSemaphoreCreateMutex();
    s_slot_semaphore = xSemaphoreCreateBinary();
    s_control_semaphore = xSemaphoreCreateBinary();
    s_tx_done_semaphore = xSemaphoreCreateBinary();
    s_task_stopped_semaphore = xSemaphoreCreateBinary();
    s_audio_producer_mutex = xSemaphoreCreateMutex();
    s_stop_mutex = xSemaphoreCreateMutex();
    s_frame_timer_mutex = xSemaphoreCreateMutex();
    s_frame_event_queue = xQueueCreate(1, sizeof(frame_event_t));
    s_timer_queue_set = xQueueCreateSet(3);

    if (!s_peer_mutex || !s_jitter_mutex || !s_slot_semaphore || !s_control_semaphore ||
        !s_tx_done_semaphore || !s_task_stopped_semaphore || !s_audio_producer_mutex ||
        !s_stop_mutex || !s_frame_timer_mutex || !s_frame_event_queue || !s_timer_queue_set) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return ESP_ERR_NO_MEM;
    }
    xQueueAddToSet(s_slot_semaphore, s_timer_queue_set);
    xQueueAddToSet(s_control_semaphore, s_timer_queue_set);
    xQueueAddToSet(s_frame_event_queue, s_timer_queue_set);

    s_tx_queue = xQueueCreate(MESH_TX_QUEUE_SIZE, sizeof(mesh_tx_item_t));
    s_rx_queue = xQueueCreate(MESH_RX_QUEUE_SIZE, sizeof(mesh_rx_item_t));

    if (!s_tx_queue || !s_rx_queue) {
        ESP_LOGE(TAG, "Failed to create queues");
        return ESP_ERR_NO_MEM;
    }

    mesh_jitter_reset(&s_jitter_buffer);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_config.channel, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(s_config.tx_power * 4)); /* Unit: 0.25 dBm */

    ESP_ERROR_CHECK(init_esp_now_transport());

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

    uint32_t epoch_id = esp_random();
    taskENTER_CRITICAL(&s_stats_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;
    s_stats.epoch_id = epoch_id;
    taskEXIT_CRITICAL(&s_stats_mux);
    reset_control_queue();
    s_contention_next_tx_us = 0;
    taskENTER_CRITICAL(&s_transport_mux);
    s_tx_inflight = (tx_inflight_t){0};
    s_stopping = false;
    s_rx_enabled = false;
    s_rx_callbacks_active = 0;
    taskEXIT_CRITICAL(&s_transport_mux);

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

    esp_err_t result = ESP_OK;
    if (s_state != MESH_STATE_IDLE) {
        result = mesh_stop();
    }
#ifdef MESH_S31_COEX_PREFER_WIFI
    restore_coex_preference();
#endif

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

    if (s_esp_now_ready) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
        taskENTER_CRITICAL(&s_transport_mux);
        s_send_callback_enabled = false;
        s_esp_now_ready = false;
        taskEXIT_CRITICAL(&s_transport_mux);
    }

    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_tx_queue) {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }
    if (s_rx_queue) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    reset_control_queue();

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
    if (s_tx_done_semaphore) {
        vSemaphoreDelete(s_tx_done_semaphore);
        s_tx_done_semaphore = NULL;
    }
    if (s_task_stopped_semaphore) {
        vSemaphoreDelete(s_task_stopped_semaphore);
        s_task_stopped_semaphore = NULL;
    }
    if (s_audio_producer_mutex) {
        vSemaphoreDelete(s_audio_producer_mutex);
        s_audio_producer_mutex = NULL;
    }
    if (s_stop_mutex) {
        vSemaphoreDelete(s_stop_mutex);
        s_stop_mutex = NULL;
    }
    if (s_frame_timer_mutex) {
        vSemaphoreDelete(s_frame_timer_mutex);
        s_frame_timer_mutex = NULL;
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

    return result;
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

    if (!s_esp_now_ready) {
        esp_err_t ret = init_esp_now_transport();
        if (ret != ESP_OK) {
            return ret;
        }
    }

#ifdef MESH_S31_COEX_PREFER_WIFI
    /* Temporary RF experiment: more Wi-Fi opportunity may degrade Bluetooth audio. */
    esp_err_t coex_ret = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    if (coex_ret == ESP_OK) {
        s_coex_restore_pending = true;
        ESP_LOGI(TAG, "Coexistence preference WIFI: %s", esp_err_to_name(coex_ret));
    } else {
        ESP_LOGE(TAG, "Coexistence preference WIFI: %s", esp_err_to_name(coex_ret));
    }
#endif

    taskENTER_CRITICAL(&s_transport_mux);
    s_stopping = false;
    s_rx_enabled = true;
    taskEXIT_CRITICAL(&s_transport_mux);
    set_state(MESH_STATE_SCANNING);
    (void)xSemaphoreTake(s_task_stopped_semaphore, 0);

    BaseType_t ret = xTaskCreatePinnedToCore(mesh_task, "mesh", MESH_TASK_STACK_SIZE, NULL,
                                             MESH_TASK_PRIORITY, &s_mesh_task, MESH_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mesh task");
        taskENTER_CRITICAL(&s_transport_mux);
        s_rx_enabled = false;
        taskEXIT_CRITICAL(&s_transport_mux);
        set_state(MESH_STATE_IDLE);
#ifdef MESH_S31_COEX_PREFER_WIFI
        restore_coex_preference();
#endif
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

uint32_t drain_rx_queue_for_reset(void)
{
    mesh_rx_item_t rx;
    uint32_t cleared = 0;
    uint32_t audio_cleared = 0;
    while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
        cleared++;
        if (rx.header.type == MESH_PKT_AUDIO) audio_cleared++;
    }
    STATS_ADD(rx_audio_purge, audio_cleared);
    return cleared;
}

esp_err_t mesh_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_stop_mutex, portMAX_DELAY);
    if (s_state == MESH_STATE_IDLE && !s_stopping) {
#ifdef MESH_S31_COEX_PREFER_WIFI
        restore_coex_preference();
#endif
        xSemaphoreGive(s_stop_mutex);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping mesh networking");

    esp_err_t result = ESP_OK;
    bool was_active = s_state == MESH_STATE_ACTIVE;
    taskENTER_CRITICAL(&s_transport_mux);
    s_stopping = true;
    s_rx_enabled = false;
    taskEXIT_CRITICAL(&s_transport_mux);

    xSemaphoreTake(s_frame_timer_mutex, portMAX_DELAY);
    advance_tdma_generation();
    esp_timer_stop(s_frame_timer);
    esp_timer_stop(s_slot_timer);
    esp_timer_stop(s_control_timer);
    xSemaphoreGive(s_frame_timer_mutex);
    drain_slot_signal();

    xSemaphoreGive(s_slot_semaphore);
    xSemaphoreGive(s_control_semaphore);
    if (s_mesh_task != NULL && xSemaphoreTake(s_task_stopped_semaphore,
                                              pdMS_TO_TICKS(TASK_QUIESCE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Timed out stopping mesh task");
        TaskHandle_t task = s_mesh_task;
        s_mesh_task = NULL;
        if (task != NULL) {
            vTaskDelete(task);
        }
        result = ESP_ERR_TIMEOUT;
    }

    /* NOTE: Wait for an admitted producer before resetting its queue. */
    xSemaphoreTake(s_audio_producer_mutex, portMAX_DELAY);
    xSemaphoreGive(s_audio_producer_mutex);

    bool force_transport_cleanup = false;
    if (!wait_for_rx_quiesced(pdMS_TO_TICKS(RX_QUIESCE_TIMEOUT_MS)) ||
        !wait_for_tx_idle(pdMS_TO_TICKS(TX_QUIESCE_TIMEOUT_MS))) {
        ESP_LOGE(TAG, "Timed out quiescing ESP-NOW callbacks");
        result = ESP_ERR_TIMEOUT;
        force_transport_cleanup = true;
    }

    if (was_active && !force_transport_cleanup) {
        esp_err_t leave_ret = send_packet_immediate(MESH_PKT_LEAVE, NULL, 0, s_broadcast_mac);
        if (leave_ret != ESP_OK) {
            result = leave_ret;
        } else if (!wait_for_tx_idle(pdMS_TO_TICKS(TX_QUIESCE_TIMEOUT_MS))) {
            ESP_LOGE(TAG, "Timed out waiting for LEAVE completion");
            result = ESP_ERR_TIMEOUT;
            force_transport_cleanup = true;
        } else {
            taskENTER_CRITICAL(&s_transport_mux);
            esp_now_send_status_t leave_status = s_last_tx_status;
            taskEXIT_CRITICAL(&s_transport_mux);
            if (leave_status != ESP_NOW_SEND_SUCCESS) {
                result = ESP_FAIL;
            }
        }
    }

    if (force_transport_cleanup) {
        force_cleanup_esp_now_transport();
    }

    /* NOTE: No producer or admitted callback may reach a queue after this point. */
    drain_rx_queue_for_reset();
    xQueueReset(s_rx_queue);
    clear_transient_mesh_state();

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    memset(s_peers, 0, sizeof(s_peers));
    s_peer_count = 0;
    xSemaphoreGive(s_peer_mutex);

    clear_speaker_state();
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    mesh_core_dedupe_reset(&s_dedupe);
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    s_relay_head = 0;
    s_relay_tail = 0;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);

    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
    s_frame_counter = 0;
    s_coordinator_id = 0;
    memset(s_coordinator_mac, 0, sizeof(s_coordinator_mac));
    s_control_tx_seq = 0;
    s_audio_tx_seq = 0;

#ifdef MESH_S31_COEX_PREFER_WIFI
    restore_coex_preference();
#endif
    set_state(MESH_STATE_IDLE);
    taskENTER_CRITICAL(&s_transport_mux);
    s_stopping = false;
    taskEXIT_CRITICAL(&s_transport_mux);

    xSemaphoreGive(s_stop_mutex);
    return result;
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
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    uint8_t count = s_peer_count;
    if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT && s_node_id != 0) {
        count++;
    }
    xSemaphoreGive(s_peer_mutex);

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
    STATS_INC(tx_offer);
    if (!s_initialized || data == NULL || len == 0) {
        STATS_INC(tx_reject_invalid);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_audio_producer_mutex, portMAX_DELAY);
    taskENTER_CRITICAL(&s_transport_mux);
    bool stopping = s_stopping;
    taskEXIT_CRITICAL(&s_transport_mux);
    if (stopping || s_state != MESH_STATE_ACTIVE) {
        STATS_INC(tx_reject_state);
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    if (len > MESH_MAX_OPUS_BYTES) {
        STATS_INC(tx_reject_invalid);
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

#ifdef MESH_S31_LC3_WIRE
    if (len != MESH_LC3_FRAME_BYTES) {
        STATS_INC(tx_reject_invalid);
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_INVALID_SIZE;
    }
#endif

    mesh_tx_item_t item;
    memcpy(item.data, data, len);
    item.len = len;
    item.audio_flags = audio_flags;
    item.timestamp_us = esp_timer_get_time();

    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        STATS_INC(tx_queue_full);
        STATS_INC(packets_dropped);
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_NO_MEM;
    }

    STATS_INC(tx_enqueue);
    unsigned depth = (unsigned)uxQueueMessagesWaiting(s_tx_queue);
    taskENTER_CRITICAL(&s_stats_mux);
    if (depth > s_stats.tx_depth_high_water) s_stats.tx_depth_high_water = depth;
    taskEXIT_CRITICAL(&s_stats_mux);
    xSemaphoreGive(s_audio_producer_mutex);
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

    taskENTER_CRITICAL(&s_stats_mux);
    *stats = s_stats;
    taskEXIT_CRITICAL(&s_stats_mux);
    return ESP_OK;
}

esp_err_t mesh_reset_stats(void)
{
    uint32_t epoch_id = esp_random();
    uint8_t control_depth;
    taskENTER_CRITICAL(&s_control_queue_mux);
    control_depth = s_control_queue_count;
    taskEXIT_CRITICAL(&s_control_queue_mux);
    uint8_t jitter_depth = 0;
    if (s_jitter_mutex) {
        xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);
        jitter_depth = s_jitter_buffer.count;
    }
    uint32_t tx_depth = s_tx_queue ? uxQueueMessagesWaiting(s_tx_queue) : 0;
    taskENTER_CRITICAL(&s_stats_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;
    s_stats.epoch_id = epoch_id;
    /* Live queues are retained; compare same-epoch samples using depth deltas. */
    s_stats.jitter_depth = jitter_depth;
    s_stats.control_queue_depth = control_depth;
    s_stats.control_queue_high_watermark = control_depth;
    s_stats.tx_depth_high_water = tx_depth;
    taskEXIT_CRITICAL(&s_stats_mux);
    if (s_jitter_mutex) xSemaphoreGive(s_jitter_mutex);
    return ESP_OK;
}

void mesh_log_pipeline_stats(void)
{
    if (!s_initialized) return;

    mesh_stats_t st;
    taskENTER_CRITICAL(&s_stats_mux);
    st = s_stats;
    taskEXIT_CRITICAL(&s_stats_mux);
    /* Snapshot each gauge independently; do not hold a spinlock during formatting/logging. */
    unsigned tx_depth = s_tx_queue ? (unsigned)uxQueueMessagesWaiting(s_tx_queue) : 0;
    unsigned rx_depth = s_rx_queue ? (unsigned)uxQueueMessagesWaiting(s_rx_queue) : 0;
    taskENTER_CRITICAL(&s_transport_mux);
    unsigned tx_inflight = s_tx_inflight.active && s_tx_inflight.audio_origin ? 1 : 0;
    taskEXIT_CRITICAL(&s_transport_mux);
    uint64_t uptime_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const char *role = s_role == MESH_ROLE_COORDINATOR ? "coordinator" :
                       s_role == MESH_ROLE_PARTICIPANT ? "participant" : "none";

#define PIPE_META "PIPE v=1 dev=esp stage=espnow epoch_id=0x%08" PRIx32 \
                  " uptime_ms=%" PRIu64 " node_mac=" MACSTR " node_id=%u role=%s"
#define PIPE_ARGS st.epoch_id, uptime_ms, MAC2STR(s_local_mac), s_node_id, role
    ESP_LOGI(TAG, PIPE_META " part=tx tx_offer=%" PRIu32 " tx_enqueue=%" PRIu32
             " tx_reject_state=%" PRIu32 " tx_reject_invalid=%" PRIu32
              " tx_queue_full=%" PRIu32 " tx_purge=%" PRIu32
              " tx_submit_ok=%" PRIu32 " tx_submit_err=%" PRIu32
              " tx_unicast_submit_ok=%" PRIu32 " tx_broadcast_submit_ok=%" PRIu32
              " tx_radio_ok=%" PRIu32 " tx_radio_fail=%" PRIu32
             " tx_abandoned=%" PRIu32 " tx_depth=%u tx_inflight=%u relay_overwrite=%" PRIu32,
              PIPE_ARGS, st.tx_offer, st.tx_enqueue, st.tx_reject_state, st.tx_reject_invalid,
              st.tx_queue_full, st.tx_purge, st.tx_submit_ok, st.tx_submit_err,
              st.tx_unicast_submit_ok, st.tx_broadcast_submit_ok,
              st.audio_frames_tx, st.tx_radio_fail, st.tx_abandoned, tx_depth, tx_inflight,
             st.relay_overwrite);
    ESP_LOGI(TAG, PIPE_META " part=rx rx_short=%" PRIu32 " rx_audio_raw=%" PRIu32
             " rx_audio_bad_header=%" PRIu32 " rx_audio_disabled=%" PRIu32
             " rx_audio_queue_full=%" PRIu32 " rx_audio_queued=%" PRIu32
             " rx_audio_purge=%" PRIu32
             " rx_audio_seen=%" PRIu32 " rx_audio_invalid=%" PRIu32
             " rx_audio_self=%" PRIu32 " rx_audio_dup=%" PRIu32
             " rx_audio_accept=%" PRIu32 " jitter_overwrite=%" PRIu32
             " jitter_pop=%" PRIu32 " jitter_late=%" PRIu32 " jitter_purge=%" PRIu32
             " rx_deliver=%" PRIu32 " jitter_depth=%u rx_depth=%u"
             " seq_gap=%" PRIu32 " slot_misses=%" PRIu32
             " control_queue_drops=%" PRIu32 " rx_queue_overflows=%" PRIu32
             " control_queue_depth=%u",
             PIPE_ARGS, st.rx_short, st.rx_audio_raw, st.rx_audio_bad_header,
             st.rx_audio_disabled, st.rx_audio_queue_full, st.rx_audio_queued, st.rx_audio_purge,
             st.rx_audio_seen, st.rx_audio_invalid, st.rx_audio_self, st.rx_audio_dup,
             st.audio_frames_rx, st.jitter_overruns, st.jitter_pop, st.audio_frames_late,
             st.jitter_purge, st.rx_deliver, st.jitter_depth, rx_depth, st.audio_frames_lost,
             st.slot_misses, st.control_queue_drops, st.rx_queue_overflows,
              st.control_queue_depth);
#undef PIPE_ARGS
#undef PIPE_META

#define TIMING_META "PIPE v=1 dev=esp stage=espnow_timing epoch_id=0x%08" PRIx32 \
                    " uptime_ms=%" PRIu64 " node_mac=" MACSTR " node_id=%u role=%s"
#define TIMING_ARGS st.epoch_id, uptime_ms, MAC2STR(s_local_mac), s_node_id, role
#define TIMING_FIELDS(name) " " #name "_count=%" PRIu32 " " #name "_us_sum=%" PRIu64 \
                            " " #name "_us_max=%" PRIu32
#define TIMING_ARGS_FOR(name) st.name.count, st.name.us_sum, st.name.us_max
    const uint32_t heap_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    uint32_t internal_free = heap_caps_get_free_size(heap_caps);
    uint32_t internal_largest = heap_caps_get_largest_free_block(heap_caps);
    uint32_t internal_min = heap_caps_get_minimum_free_size(heap_caps);
    ESP_LOGI(TAG, TIMING_META " part=errors send_err_nomem=%" PRIu32
             " send_err_other=%" PRIu32 " send_last_error=%" PRId32
             " send_last_error_uptime_ms=%" PRIu64
             " send_heap_snapshot_uptime_ms=%" PRIu64
             " send_heap_snapshot_error=%" PRId32 " send_heap_snapshot_valid=%u"
             " internal_8bit_free=%" PRIu32 " internal_8bit_largest=%" PRIu32
             " internal_8bit_min=%" PRIu32 " send_error_internal_free=%" PRIu32
             " send_error_internal_largest=%" PRIu32 " send_error_internal_min=%" PRIu32,
             TIMING_ARGS, st.send_err_nomem, st.send_err_other, st.send_last_error,
             st.send_last_error_uptime_ms, st.send_heap_snapshot_uptime_ms,
             st.send_heap_snapshot_error, st.send_heap_snapshot_valid ? 1u : 0u,
             internal_free, internal_largest, internal_min,
             st.send_error_internal_free, st.send_error_internal_largest,
             st.send_error_internal_min);
    ESP_LOGI(TAG, TIMING_META " part=rx"
             TIMING_FIELDS(rx_loop_gap) " rx_loop_gap_over_5ms_count=%" PRIu32
             " rx_loop_gap_over_20ms_count=%" PRIu32 " rx_loop_gap_over_60ms_count=%" PRIu32
             TIMING_FIELDS(rx_dequeue_age) TIMING_FIELDS(rx_handle)
             TIMING_FIELDS(rx_audio_callback) " rx_audio_callback_over_20ms_count=%" PRIu32
             TIMING_FIELDS(jitter_deliver_age)
             " jitter_expired_age_us_max=%" PRIu32
             " jitter_expired_with_pending_count=%" PRIu32,
             TIMING_ARGS, TIMING_ARGS_FOR(rx_loop_gap), st.rx_loop_gap_over_5ms_count,
             st.rx_loop_gap_over_20ms_count, st.rx_loop_gap_over_60ms_count,
             TIMING_ARGS_FOR(rx_dequeue_age), TIMING_ARGS_FOR(rx_handle),
             TIMING_ARGS_FOR(rx_audio_callback), st.rx_audio_callback_over_20ms_count,
             TIMING_ARGS_FOR(jitter_deliver_age), st.jitter_expired_age_us_max,
             st.jitter_expired_with_pending_count);
    ESP_LOGI(TAG, TIMING_META " part=tx"
             TIMING_FIELDS(tx_frame_dispatch_late) TIMING_FIELDS(tx_slot_service_late)
             " tx_slot_late_count=%" PRIu32 " tx_slot_early_count=%" PRIu32
             " tx_slot_invalid_count=%" PRIu32 " tx_busy_count=%" PRIu32
             " tx_retry_armed=%" PRIu32 " tx_retry_recovered=%" PRIu32
             " tx_retry_exhausted=%" PRIu32 " tx_deadline_reject=%" PRIu32
             TIMING_FIELDS(tx_queue_age) TIMING_FIELDS(tx_esp_now_send)
             " tx_depth_high_water=%" PRIu32,
             TIMING_ARGS, TIMING_ARGS_FOR(tx_frame_dispatch_late),
             TIMING_ARGS_FOR(tx_slot_service_late), st.tx_slot_late_count, st.tx_slot_early_count,
             st.tx_slot_invalid_count, st.tx_busy_count, st.tx_retry_armed,
             st.tx_retry_recovered, st.tx_retry_exhausted, st.tx_deadline_reject,
             TIMING_ARGS_FOR(tx_queue_age),
              TIMING_ARGS_FOR(tx_esp_now_send), st.tx_depth_high_water);
    ESP_LOGI(TAG, TIMING_META " part=radio"
             TIMING_FIELDS(origin_complete) TIMING_FIELDS(control_complete)
             " tx_busy_audio_count=%" PRIu32 " tx_busy_control_count=%" PRIu32
             " tx_busy_age_us_max=%" PRIu32,
             TIMING_ARGS, TIMING_ARGS_FOR(origin_complete), TIMING_ARGS_FOR(control_complete),
             st.tx_busy_audio_count, st.tx_busy_control_count, st.tx_busy_age_us_max);
#undef TIMING_ARGS_FOR
#undef TIMING_FIELDS
#undef TIMING_ARGS
#undef TIMING_META
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

    int64_t slot_start = (int64_t)s_slot_index * MESH_SLOT_US;

    if (frame_elapsed < slot_start) {
        return (int32_t)(slot_start - frame_elapsed);
    } else {
        return (int32_t)(MESH_FRAME_US - frame_elapsed + slot_start);
    }
}
