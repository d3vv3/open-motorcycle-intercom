/** @file mesh.c @brief ESP-NOW mesh implementation. */

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "mesh_internal.h"

const char *const TAG = "mesh";
const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

mesh_context_t s_mesh = {
    .config = MESH_CONFIG_DEFAULT(),
    .slot_index = -1,
    .stats_mux = portMUX_INITIALIZER_UNLOCKED,
    .speaker_mux = portMUX_INITIALIZER_UNLOCKED,
    .tdma_mux = portMUX_INITIALIZER_UNLOCKED,
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

    taskENTER_CRITICAL(&s_stats_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;
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
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t mesh_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_stop_mutex, portMAX_DELAY);
    if (s_state == MESH_STATE_IDLE && !s_stopping) {
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
    if (!s_initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_audio_producer_mutex, portMAX_DELAY);
    taskENTER_CRITICAL(&s_transport_mux);
    bool stopping = s_stopping;
    taskEXIT_CRITICAL(&s_transport_mux);
    if (stopping || s_state != MESH_STATE_ACTIVE) {
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    if (len > MESH_MAX_OPUS_BYTES) {
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    mesh_tx_item_t item;
    memcpy(item.data, data, len);
    item.len = len;
    item.audio_flags = audio_flags;
    item.timestamp_us = esp_timer_get_time();

    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        STATS_INC(packets_dropped);
        xSemaphoreGive(s_audio_producer_mutex);
        return ESP_ERR_NO_MEM;
    }

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
    uint8_t control_depth;
    taskENTER_CRITICAL(&s_control_queue_mux);
    control_depth = s_control_queue_count;
    taskEXIT_CRITICAL(&s_control_queue_mux);
    taskENTER_CRITICAL(&s_stats_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.latency_min_us = UINT32_MAX;
    s_stats.control_queue_depth = control_depth;
    s_stats.control_queue_high_watermark = control_depth;
    taskEXIT_CRITICAL(&s_stats_mux);
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

    int64_t slot_start = (int64_t)s_slot_index * MESH_SLOT_US;

    if (frame_elapsed < slot_start) {
        return (int32_t)(slot_start - frame_elapsed);
    } else {
        return (int32_t)(MESH_FRAME_US - frame_elapsed + slot_start);
    }
}
