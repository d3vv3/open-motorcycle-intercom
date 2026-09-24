#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "mesh_internal.h"
#include "power.h"

static control_priority_t control_priority(uint8_t type)
{
    /* SYNC and lifecycle traffic preempt periodic control traffic. */
    switch (type) {
    case MESH_PKT_SYNC:
        return CONTROL_PRIORITY_SYNC;
    case MESH_PKT_JOIN_ACK:
    case MESH_PKT_LEAVE:
        return CONTROL_PRIORITY_LIFECYCLE;
    case MESH_PKT_SLOT_MAP:
    case MESH_PKT_SPEAKER_GRANT:
    case MESH_PKT_SPEAKER_RELEASE:
        return CONTROL_PRIORITY_TOPOLOGY;
    default:
        return CONTROL_PRIORITY_PERIODIC;
    }
}

static bool control_type_replaceable(uint8_t type)
{
    return type == MESH_PKT_STATUS || type == MESH_PKT_KEEPALIVE || type == MESH_PKT_SLOT_MAP ||
           type == MESH_PKT_SPEAKER_GRANT || type == MESH_PKT_JOIN_ACK;
}

static void stats_set_control_depth(uint8_t depth, bool update_high_watermark)
{
    taskENTER_CRITICAL(&s_stats_mux);
    s_stats.control_queue_depth = depth;
    if (update_high_watermark && depth > s_stats.control_queue_high_watermark) {
        s_stats.control_queue_high_watermark = depth;
    }
    taskEXIT_CRITICAL(&s_stats_mux);
}

void mesh_transport_restore_status_bitmaps(const control_tx_item_t *item)
{
    if (item == NULL || item->type != MESH_PKT_STATUS ||
        item->len != sizeof(mesh_header_t) + sizeof(mesh_status_payload_t)) {
        return;
    }

    const mesh_status_payload_t *status =
        (const mesh_status_payload_t *)(item->data + sizeof(mesh_header_t));
    taskENTER_CRITICAL(&s_speaker_mux);
    s_heard_bitmap |= status->heard_bitmap;
    s_relay_bitmap |= status->relay_bitmap;
    taskEXIT_CRITICAL(&s_speaker_mux);
}

esp_err_t enqueue_control_packet(uint8_t type, const void *payload, uint16_t len,
                                 const uint8_t *dest_mac)
{
    if (type == MESH_PKT_SYNC) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > sizeof(s_control_queue[0].data) - sizeof(mesh_header_t) ||
        (len > 0 && payload == NULL) || dest_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_transport_mux);
    bool stopping = s_stopping;
    taskEXIT_CRITICAL(&s_transport_mux);
    if (stopping) {
        return ESP_ERR_INVALID_STATE;
    }

    control_tx_item_t item = {
        .len = (uint16_t)(sizeof(mesh_header_t) + len),
        .enqueued_us = esp_timer_get_time(),
        .type = type,
        .priority = control_priority(type),
    };
    memcpy(item.dest_mac, dest_mac, sizeof(item.dest_mac));
    mesh_header_t *header = (mesh_header_t *)item.data;
    *header = (mesh_header_t){
        .version = MESH_PROTOCOL_VERSION,
        .type = type,
        .src_id = s_node_id,
        .seq = 0,
        .ttl = 0,
        .flags = 0,
        .payload_len = len,
    };
    if (len > 0) {
        memcpy(item.data + sizeof(mesh_header_t), payload, len);
    }

    control_tx_item_t evicted = {0};
    bool have_evicted = false;
    esp_err_t result = ESP_OK;

    taskENTER_CRITICAL(&s_control_queue_mux);
    if (control_type_replaceable(type)) {
        for (uint8_t i = 0; i < s_control_queue_count; i++) {
            control_tx_item_t *queued = &s_control_queue[i];
            if (queued->type != type ||
                memcmp(queued->dest_mac, dest_mac, sizeof(queued->dest_mac)) != 0) {
                continue;
            }

            if (type == MESH_PKT_STATUS) {
                mesh_status_payload_t *new_status =
                    (mesh_status_payload_t *)(item.data + sizeof(mesh_header_t));
                const mesh_status_payload_t *old_status =
                    (const mesh_status_payload_t *)(queued->data + sizeof(mesh_header_t));
                new_status->heard_bitmap |= old_status->heard_bitmap;
                new_status->relay_bitmap |= old_status->relay_bitmap;
            }
            item.order = queued->order;
            item.enqueued_us = queued->enqueued_us;
            *queued = item;
            taskEXIT_CRITICAL(&s_control_queue_mux);
            return ESP_OK;
        }
    }

    if (s_control_queue_count == MESH_CONTROL_QUEUE_CAPACITY) {
        uint8_t victim = 0;
        for (uint8_t i = 1; i < s_control_queue_count; i++) {
            if (s_control_queue[i].priority < s_control_queue[victim].priority ||
                (s_control_queue[i].priority == s_control_queue[victim].priority &&
                 s_control_queue[i].order < s_control_queue[victim].order)) {
                victim = i;
            }
        }
        if (s_control_queue[victim].priority > item.priority ||
            (s_control_queue[victim].priority == item.priority &&
             item.priority == CONTROL_PRIORITY_PERIODIC)) {
            STATS_INC(control_queue_drops);
            result = ESP_ERR_NO_MEM;
        } else {
            evicted = s_control_queue[victim];
            have_evicted = true;
            s_control_queue[victim] = s_control_queue[--s_control_queue_count];
            STATS_INC(control_queue_drops);
        }
    }

    if (result == ESP_OK) {
        item.order = s_control_queue_order++;
        s_control_queue[s_control_queue_count++] = item;
    }
    uint8_t depth = s_control_queue_count;
    taskEXIT_CRITICAL(&s_control_queue_mux);

    stats_set_control_depth(depth, result == ESP_OK);

    if (have_evicted) {
        mesh_transport_restore_status_bitmaps(&evicted);
    }
    return result;
}

bool dequeue_control_packet(control_tx_item_t *item)
{
    if (item == NULL) {
        return false;
    }

    taskENTER_CRITICAL(&s_control_queue_mux);
    if (s_control_queue_count == 0) {
        taskEXIT_CRITICAL(&s_control_queue_mux);
        return false;
    }

    uint8_t selected = 0;
    int64_t now_us = esp_timer_get_time();
    bool selected_overdue =
        (now_us - s_control_queue[0].enqueued_us) >= (CONTROL_MAX_PRIORITY_WAIT_MS * 1000);
    for (uint8_t i = 1; i < s_control_queue_count; i++) {
        bool overdue =
            (now_us - s_control_queue[i].enqueued_us) >= (CONTROL_MAX_PRIORITY_WAIT_MS * 1000);
        if ((overdue && !selected_overdue) ||
            (overdue == selected_overdue && overdue &&
             s_control_queue[i].order < s_control_queue[selected].order) ||
            (!overdue && !selected_overdue &&
             (s_control_queue[i].priority > s_control_queue[selected].priority ||
              (s_control_queue[i].priority == s_control_queue[selected].priority &&
               s_control_queue[i].order < s_control_queue[selected].order)))) {
            selected = i;
            selected_overdue = overdue;
        }
    }
    *item = s_control_queue[selected];
    s_control_queue[selected] = s_control_queue[--s_control_queue_count];
    uint8_t depth = s_control_queue_count;
    taskEXIT_CRITICAL(&s_control_queue_mux);
    stats_set_control_depth(depth, false);
    return true;
}

void reset_control_queue(void)
{
    taskENTER_CRITICAL(&s_control_queue_mux);
    memset(s_control_queue, 0, sizeof(s_control_queue));
    s_control_queue_count = 0;
    taskEXIT_CRITICAL(&s_control_queue_mux);
    stats_set_control_depth(0, false);
}

esp_err_t init_esp_now_transport(void)
{
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        return ret;
    }
    if ((ret = esp_now_register_recv_cb(esp_now_recv_cb)) != ESP_OK ||
        (ret = esp_now_register_send_cb(esp_now_send_cb)) != ESP_OK) {
        esp_now_deinit();
        return ret;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, sizeof(peer.peer_addr));
    peer.channel = s_config.channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if ((ret = esp_now_add_peer(&peer)) != ESP_OK) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
        return ret;
    }

    taskENTER_CRITICAL(&s_transport_mux);
    s_send_callback_enabled = true;
    s_esp_now_ready = true;
    taskEXIT_CRITICAL(&s_transport_mux);
    return ESP_OK;
}
void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* Type is byte 1 of the wire header; shorter packets cannot be classified. */
    bool audio = len >= 2 && data != NULL && data[1] == MESH_PKT_AUDIO;
    if (audio) STATS_INC(rx_audio_raw);
    else if (len < 2) STATS_INC(rx_short);
    taskENTER_CRITICAL(&s_transport_mux);
    if (!s_rx_enabled) {
        taskEXIT_CRITICAL(&s_transport_mux);
        if (audio) STATS_INC(rx_audio_disabled);
        return;
    }
    s_rx_callbacks_active++;
    taskEXIT_CRITICAL(&s_transport_mux);

    if (len < sizeof(mesh_header_t)) {
        if (audio) STATS_INC(rx_audio_bad_header);
        goto done;
    }

    mesh_rx_item_t rx;
    memcpy(&rx.header, data, sizeof(mesh_header_t));
    memcpy(rx.src_mac, info->src_addr, 6);
    rx.rssi = info->rx_ctrl->rssi;
    rx.timestamp_us = esp_timer_get_time();

    if (rx.header.version != MESH_PROTOCOL_VERSION) {
        if (audio) STATS_INC(rx_audio_bad_header);
        goto done;
    }

    uint16_t payload_len = rx.header.payload_len;
    if (payload_len > sizeof(rx.payload) || payload_len > (uint16_t)(len - sizeof(mesh_header_t))) {
        if (audio) STATS_INC(rx_audio_bad_header);
        goto done;
    }
    if (payload_len > 0) {
        memcpy(rx.payload, data + sizeof(mesh_header_t), payload_len);
    }

    STATS_INC(packets_rx);

    if (xQueueSend(s_rx_queue, &rx, 0) != pdTRUE) {
        STATS_INC(rx_queue_overflows);
        if (audio) STATS_INC(rx_audio_queue_full);
    } else if (audio) {
        STATS_INC(rx_audio_queued);
    }

done:
    taskENTER_CRITICAL(&s_transport_mux);
    s_rx_callbacks_active--;
    taskEXIT_CRITICAL(&s_transport_mux);
}

void esp_now_send_cb(const esp_now_send_info_t *send_info, esp_now_send_status_t status)
{
    (void)send_info;
    int64_t completed_us = esp_timer_get_time();

    taskENTER_CRITICAL(&s_transport_mux);
    if (!s_send_callback_enabled) {
        taskEXIT_CRITICAL(&s_transport_mux);
        return;
    }
    s_tx_callbacks_active++;
    if (!s_tx_inflight.active) {
        s_tx_callbacks_active--;
        taskEXIT_CRITICAL(&s_transport_mux);
        return;
    }
    tx_inflight_t completion = s_tx_inflight;
    uint32_t elapsed_us = mesh_timing_us(completed_us - completion.starts_us);

    if (status != ESP_NOW_SEND_SUCCESS && completion.type == MESH_PKT_STATUS) {
        taskENTER_CRITICAL(&s_speaker_mux);
        s_heard_bitmap |= completion.heard_bitmap;
        s_relay_bitmap |= completion.relay_bitmap;
        taskEXIT_CRITICAL(&s_speaker_mux);
    }

    taskENTER_CRITICAL(&s_stats_mux);
    mesh_timing_stat_t *timing = completion.audio_origin ? &s_stats.origin_complete :
                                                         &s_stats.control_complete;
    if (timing->count < UINT32_MAX) timing->count++;
    timing->us_sum = UINT64_MAX - timing->us_sum < elapsed_us ? UINT64_MAX :
                     timing->us_sum + elapsed_us;
    if (elapsed_us > timing->us_max) timing->us_max = elapsed_us;
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_stats.packets_tx++;
        if (completion.audio_origin) {
            s_stats.audio_frames_tx++;
        }
    } else {
        s_stats.packets_dropped++;
        if (completion.audio_origin) s_stats.tx_radio_fail++;
    }
    taskEXIT_CRITICAL(&s_stats_mux);

    s_last_tx_status = status;
    s_tx_inflight.active = false;
    s_tx_callbacks_active--;
    taskEXIT_CRITICAL(&s_transport_mux);
    xSemaphoreGive(s_tx_done_semaphore);
}

/* Called with transport mux held, then takes stats mux in the established order. */
static void record_audio_busy_locked(int64_t now_us)
{
    if (!s_tx_inflight.active) return;
    uint32_t age = mesh_timing_us(now_us - s_tx_inflight.starts_us);
    taskENTER_CRITICAL(&s_stats_mux);
    if (s_tx_inflight.audio_origin) s_stats.tx_busy_audio_count++;
    else s_stats.tx_busy_control_count++;
    if (age > s_stats.tx_busy_age_us_max) s_stats.tx_busy_age_us_max = age;
    taskEXIT_CRITICAL(&s_stats_mux);
}

void mesh_note_audio_tx_busy(void)
{
    int64_t now_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_transport_mux);
    record_audio_busy_locked(now_us);
    taskEXIT_CRITICAL(&s_transport_mux);
}

esp_err_t mesh_send_control_packet(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    return enqueue_control_packet(type, payload, len, s_broadcast_mac);
}

esp_err_t send_packet_immediate(mesh_pkt_type_t type, const void *payload, uint16_t len,
                                const uint8_t *dest_mac)
{
    if (len > sizeof(s_control_queue[0].data) - sizeof(mesh_header_t) ||
        (len > 0 && payload == NULL) || dest_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[sizeof(s_control_queue[0].data)];
    mesh_header_t *header = (mesh_header_t *)buffer;
    *header = (mesh_header_t){
        .version = MESH_PROTOCOL_VERSION,
        .type = type,
        .src_id = s_node_id,
        .seq = 0,
        .ttl = 0,
        .flags = 0,
        .payload_len = len,
    };
    if (len > 0) {
        memcpy(buffer + sizeof(mesh_header_t), payload, len);
    }
    return tracked_esp_now_send(&(tracked_esp_now_send_request_t){
        .dest_mac = dest_mac,
        .data = buffer,
        .len = sizeof(mesh_header_t) + len,
        .type = type,
        .heard_bitmap = 0,
        .relay_bitmap = 0,
        .sequence = &s_control_tx_seq,
        .audio_origin = false,
    });
}

esp_err_t tracked_esp_now_send(const tracked_esp_now_send_request_t *request)
{
    if (request == NULL || request->dest_mac == NULL || request->data == NULL ||
        request->len < sizeof(mesh_header_t)) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t reservation_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_transport_mux);
    if (s_tx_inflight.active || (s_stopping && request->type != MESH_PKT_LEAVE)) {
        bool busy = s_tx_inflight.active && request->audio_origin;
        if (busy) record_audio_busy_locked(reservation_us);
        taskEXIT_CRITICAL(&s_transport_mux);
        if (busy) STATS_INC(tx_busy_count);
        return ESP_ERR_INVALID_STATE;
    }
    mesh_header_t *header = (mesh_header_t *)request->data;
    if (request->sequence != NULL) {
        header->seq = *request->sequence;
        (*request->sequence)++;
    }
    s_tx_inflight = (tx_inflight_t){
        .type = request->type,
        .heard_bitmap = request->heard_bitmap,
        .relay_bitmap = request->relay_bitmap,
        .audio_origin = request->audio_origin,
        .active = true,
        .starts_us = reservation_us,
    };
    taskEXIT_CRITICAL(&s_transport_mux);

    /* Last software admission check: preparation/reservation may outlive the slot. */
    int64_t admission_us = esp_timer_get_time();
    esp_err_t ret;
    bool snapshot_heap = false;
    uint32_t error_epoch = 0;
    uint64_t snapshot_id = 0;
    uint64_t failure_ms = 0;
    if (mesh_tx_slot_deadline_passed(request->deadline_us, admission_us)) {
        STATS_INC(tx_deadline_reject);
        ret = ESP_ERR_TIMEOUT;
        goto cleanup;
    }

    int64_t send_start_us = request->audio_origin ? admission_us : 0;
    ret = esp_now_send(request->dest_mac, request->data, request->len);
    int64_t send_end_us = esp_timer_get_time();
    if (request->audio_origin) {
        mesh_timing_record(&s_stats.tx_esp_now_send, send_end_us - send_start_us);
    }
    if (ret != ESP_OK) {
        failure_ms = (uint64_t)(send_end_us / 1000);
        taskENTER_CRITICAL(&s_stats_mux);
        snapshot_heap = (s_stats.send_err_nomem == 0 && s_stats.send_err_other == 0) ||
                         s_stats.send_last_error != (int32_t)ret;
        if (snapshot_heap) {
            error_epoch = s_stats.epoch_id;
            snapshot_id = ++s_stats.send_heap_snapshot_id;
            s_stats.send_heap_snapshot_uptime_ms = failure_ms;
            s_stats.send_heap_snapshot_error = (int32_t)ret;
            s_stats.send_heap_snapshot_valid = false;
        }
        if (ret == ESP_ERR_ESPNOW_NO_MEM || ret == ESP_ERR_NO_MEM) {
            if (s_stats.send_err_nomem != UINT32_MAX) s_stats.send_err_nomem++;
        } else {
            if (s_stats.send_err_other != UINT32_MAX) s_stats.send_err_other++;
        }
        s_stats.send_last_error = (int32_t)ret;
        s_stats.send_last_error_uptime_ms = failure_ms;
        taskEXIT_CRITICAL(&s_stats_mux);
    }
    if (request->audio_origin) {
        if (ret == ESP_OK) STATS_INC(tx_submit_ok);
        else STATS_INC(tx_submit_err);
    }
cleanup:
    if (ret != ESP_OK) {
        taskENTER_CRITICAL(&s_transport_mux);
        s_tx_inflight.active = false;
        if (request->sequence != NULL) {
            (*request->sequence)--;
        }
        taskEXIT_CRITICAL(&s_transport_mux);
    }
    if (snapshot_heap) {
        const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
        uint32_t free_bytes = heap_caps_get_free_size(caps);
        uint32_t largest = heap_caps_get_largest_free_block(caps);
        uint32_t minimum = heap_caps_get_minimum_free_size(caps);
        taskENTER_CRITICAL(&s_stats_mux);
        if (s_stats.epoch_id == error_epoch && s_stats.send_heap_snapshot_id == snapshot_id) {
            s_stats.send_error_internal_free = free_bytes;
            s_stats.send_error_internal_largest = largest;
            s_stats.send_error_internal_min = minimum;
            s_stats.send_heap_snapshot_valid = true;
        }
        taskEXIT_CRITICAL(&s_stats_mux);
    }
    return ret;
}

bool wait_for_tx_idle(TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    while (true) {
        taskENTER_CRITICAL(&s_transport_mux);
        bool active = s_tx_inflight.active;
        taskEXIT_CRITICAL(&s_transport_mux);
        if (!active) {
            return true;
        }

        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks ||
            xSemaphoreTake(s_tx_done_semaphore, timeout_ticks - elapsed) != pdTRUE) {
            return false;
        }
    }
}

bool wait_for_rx_quiesced(TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    while (true) {
        taskENTER_CRITICAL(&s_transport_mux);
        uint8_t active = s_rx_callbacks_active;
        taskEXIT_CRITICAL(&s_transport_mux);
        if (active == 0) {
            return true;
        }
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            return false;
        }
        vTaskDelay(1);
    }
}

void force_cleanup_esp_now_transport(void)
{
    taskENTER_CRITICAL(&s_transport_mux);
    bool ready = s_esp_now_ready;
    s_rx_enabled = false;
    s_send_callback_enabled = false;
    taskEXIT_CRITICAL(&s_transport_mux);

    if (ready) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(RX_QUIESCE_TIMEOUT_MS);
    while (true) {
        taskENTER_CRITICAL(&s_transport_mux);
        uint8_t callbacks_active = s_rx_callbacks_active + s_tx_callbacks_active;
        taskEXIT_CRITICAL(&s_transport_mux);
        if (callbacks_active == 0 || xTaskGetTickCount() >= deadline) {
            break;
        }
        vTaskDelay(1);
    }

    taskENTER_CRITICAL(&s_transport_mux);
    tx_inflight_t abandoned = s_tx_inflight;
    s_tx_inflight = (tx_inflight_t){0};
    if (abandoned.active && abandoned.audio_origin) STATS_INC(tx_abandoned);
    s_esp_now_ready = false;
    taskEXIT_CRITICAL(&s_transport_mux);

    if (abandoned.active && abandoned.type == MESH_PKT_STATUS) {
        taskENTER_CRITICAL(&s_speaker_mux);
        s_heard_bitmap |= abandoned.heard_bitmap;
        s_relay_bitmap |= abandoned.relay_bitmap;
        taskEXIT_CRITICAL(&s_speaker_mux);
    }
    xSemaphoreGive(s_tx_done_semaphore);
}

esp_err_t send_join_request(void)
{
    if (s_state != MESH_STATE_SCANNING && s_state != MESH_STATE_JOINING) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t now_us = esp_timer_get_time();
    if (now_us < s_contention_next_tx_us) {
        STATS_INC(contention_deferred);
        return ESP_ERR_TIMEOUT;
    }

    mesh_join_payload_t payload = {
        .capabilities = 0x01, /* Basic Opus support */
        .reserved = 0,
    };

    /* Wire contract: unassigned JOIN requests use source ID 0. */
    uint8_t buffer[sizeof(mesh_header_t) + sizeof(mesh_join_payload_t)];
    mesh_header_t *header = (mesh_header_t *)buffer;

    header->version = MESH_PROTOCOL_VERSION;
    header->type = MESH_PKT_JOIN;
    header->src_id = 0; /* Unassigned */
    header->seq = 0;
    header->ttl = 0;
    header->flags = 0;
    header->payload_len = sizeof(payload);

    memcpy(buffer + sizeof(mesh_header_t), &payload, sizeof(payload));

    esp_err_t ret = tracked_esp_now_send(&(tracked_esp_now_send_request_t){
        .dest_mac = s_broadcast_mac,
        .data = buffer,
        .len = sizeof(buffer),
        .type = MESH_PKT_JOIN,
        .heard_bitmap = 0,
        .relay_bitmap = 0,
        .sequence = &s_control_tx_seq,
        .audio_origin = false,
    });
    uint32_t jitter_ms = esp_random() % (CONTENTION_JITTER_MS + 1);
    s_contention_next_tx_us = now_us + ((CONTENTION_MIN_INTERVAL_MS + jitter_ms) * 1000);
    if (ret == ESP_OK) {
        STATS_INC(contention_tx);
    }
    return ret;
}

esp_err_t send_join_ack(uint8_t node_id, uint8_t slot, const uint8_t *dest_mac)
{
    mesh_join_ack_payload_t payload = {
        .assigned_id = node_id,
        .slot_index = slot,
        .coordinator_id = s_node_id,
    };

    if (!esp_now_is_peer_exist(dest_mac)) {
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, dest_mac, 6);
        peer.channel = s_config.channel;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    return enqueue_control_packet(MESH_PKT_JOIN_ACK, &payload, sizeof(payload), dest_mac);
}

esp_err_t send_sync(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    uint32_t frame_counter = s_frame_counter;
    taskEXIT_CRITICAL(&s_tdma_mux);
    mesh_sync_payload_t payload = {
        .frame_counter = frame_counter,
        .drift_ppm = 0, /* TODO(sync): Set from a measured local clock offset. */
    };
    memcpy(payload.coordinator_addr, s_local_mac, sizeof(payload.coordinator_addr));

    return send_packet_immediate(MESH_PKT_SYNC, &payload, sizeof(payload), s_broadcast_mac);
}

esp_err_t send_keepalive(void)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 100, /* TODO(keepalive): Set from the battery monitor. */
        .reserved = 0,
    };

    return mesh_send_control_packet(MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
}

esp_err_t send_slot_map(void)
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

    return mesh_send_control_packet(MESH_PKT_SLOT_MAP, &payload, sizeof(payload));
}

esp_err_t send_status(void)
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

    esp_err_t ret = mesh_send_control_packet(MESH_PKT_STATUS, &payload, sizeof(payload));
    if (ret != ESP_OK) {
        taskENTER_CRITICAL(&s_speaker_mux);
        s_heard_bitmap |= heard_bitmap;
        s_relay_bitmap |= relay_bitmap;
        taskEXIT_CRITICAL(&s_speaker_mux);
    }
    return ret;
}
