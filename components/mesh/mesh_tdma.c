#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "mesh_internal.h"
#include "power.h"

bool owns_control_window(uint32_t frame_counter)
{
    /* Slot ownership rotates by frame. Every sync frame is reserved for the
     * coordinator in slot 0 so participants never contend with timing sync. */
    uint8_t owner_slot = (frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0
                             ? 0
                             : (uint8_t)(frame_counter % MESH_VOICE_SLOTS);
    return s_slot_index == (int8_t)owner_slot;
}

void frame_timer_callback(void *arg)
{
    (void)arg;
    int64_t now_us = esp_timer_get_time();
    frame_event_t event;
    /* NOTE: The generation rejects timer callbacks queued before resync or stop. */
    taskENTER_CRITICAL(&s_tdma_mux);
    event.timestamp_us = s_expected_frame_us;
    event.generation = s_frame_timer_generation;
    bool valid = event.generation == s_tdma_generation;
    if (valid) {
        event.timestamp_us =
            mesh_core_recover_frame_boundary(event.timestamp_us, now_us, MESH_FRAME_US);
        s_expected_frame_us = event.timestamp_us + MESH_FRAME_US;
    }
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid) {
        return;
    }
    xQueueOverwrite(s_frame_event_queue, &event);
}

void service_frame_boundary(const frame_event_t *event)
{
    xSemaphoreTake(s_frame_timer_mutex, portMAX_DELAY);
    taskENTER_CRITICAL(&s_tdma_mux);
    bool valid = event->generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid || s_state != MESH_STATE_ACTIVE) {
        xSemaphoreGive(s_frame_timer_mutex);
        return;
    }

    esp_err_t timer_ret = arm_frame_timer_at_generation_locked(event->timestamp_us + MESH_FRAME_US,
                                                               event->generation);
    if (timer_ret != ESP_OK) {
        xSemaphoreGive(s_frame_timer_mutex);
        if (timer_ret == ESP_ERR_INVALID_STATE) {
            return;
        }
        ESP_LOGE(TAG, "Failed to arm TDMA frame timer: %s", esp_err_to_name(timer_ret));
        return;
    }

    int64_t frame_start_us = event->timestamp_us;
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t frame_delta_us = frame_start_us - s_frame_start_us;
    uint32_t elapsed_frames =
        frame_delta_us > MESH_FRAME_US ? (uint32_t)(frame_delta_us / MESH_FRAME_US) : 1;
    s_frame_counter += elapsed_frames;
    uint32_t frame_counter = s_frame_counter;
    s_frame_start_us = frame_start_us;
    taskEXIT_CRITICAL(&s_tdma_mux);

    if (owns_control_window(frame_counter)) {
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
            STATS_INC(slot_misses);
        }
    }

    xSemaphoreGive(s_frame_timer_mutex);
}

void slot_timer_callback(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_slot_semaphore);
}

void control_timer_callback(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_control_semaphore);
}

void service_tx_slot(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t slot_start_us = s_slot_start_us;
    int64_t deadline_us = s_slot_deadline_us;
    bool valid = s_slot_generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);

    int64_t now_us = esp_timer_get_time();
    if (!valid || s_state != MESH_STATE_ACTIVE || s_slot_index < 0 || now_us < slot_start_us ||
        now_us > deadline_us) {
        STATS_INC(slot_misses);
        return;
    }

    power_radio_slot_start();
    send_audio_in_slot();
    power_radio_slot_end();
}

void service_control_window(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    int64_t control_start_us = s_control_start_us;
    int64_t control_deadline_us = s_control_deadline_us;
    bool valid = s_control_generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);

    int64_t now_us = esp_timer_get_time();
    if (!valid || s_state != MESH_STATE_ACTIVE || now_us < control_start_us ||
        now_us > control_deadline_us) {
        return;
    }

    uint32_t frame_counter = mesh_get_frame_counter();
    if (!owns_control_window(frame_counter)) {
        return;
    }
    bool sync_due =
        s_role == MESH_ROLE_COORDINATOR && (frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0;

    if (!wait_for_tx_idle(0)) {
        int64_t remaining_us = control_deadline_us - esp_timer_get_time();
        TickType_t wait_ticks = remaining_us > 0 ? pdMS_TO_TICKS((remaining_us + 999) / 1000) : 0;
        if (!wait_for_tx_idle(wait_ticks) || esp_timer_get_time() > control_deadline_us) {
            if (sync_due) {
                STATS_INC(control_queue_drops);
            }
            return;
        }
    }

    taskENTER_CRITICAL(&s_tdma_mux);
    valid = s_control_generation == s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid || s_state != MESH_STATE_ACTIVE || mesh_get_frame_counter() != frame_counter ||
        !owns_control_window(frame_counter) || esp_timer_get_time() > control_deadline_us) {
        return;
    }

    if (sync_due) {
        if (send_sync() != ESP_OK) {
            STATS_INC(control_queue_drops);
        }
        return;
    }

    control_tx_item_t item;
    if (wait_for_tx_idle(0) && dequeue_control_packet(&item)) {
        const mesh_status_payload_t *status = NULL;
        uint8_t heard_bitmap = 0;
        uint8_t relay_bitmap = 0;
        if (item.type == MESH_PKT_STATUS &&
            item.len == sizeof(mesh_header_t) + sizeof(mesh_status_payload_t)) {
            status = (const mesh_status_payload_t *)(item.data + sizeof(mesh_header_t));
            heard_bitmap = status->heard_bitmap;
            relay_bitmap = status->relay_bitmap;
        }
        esp_err_t ret = tracked_esp_now_send(&(tracked_esp_now_send_request_t){
            .dest_mac = item.dest_mac,
            .data = item.data,
            .len = item.len,
            .type = item.type,
            .heard_bitmap = heard_bitmap,
            .relay_bitmap = relay_bitmap,
            .sequence = &s_control_tx_seq,
            .audio_origin = false,
        });
        if (ret != ESP_OK) {
            mesh_transport_restore_status_bitmaps(&item);
            STATS_INC(control_queue_drops);
            STATS_INC(packets_dropped);
        }
    }
}

void drain_slot_signal(void)
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

void advance_tdma_generation(void)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    s_tdma_generation++;
    taskEXIT_CRITICAL(&s_tdma_mux);
}

esp_err_t arm_frame_timer(int64_t delay_us)
{
    return arm_frame_timer_at(esp_timer_get_time() + delay_us);
}

esp_err_t arm_frame_timer_at(int64_t boundary_us)
{
    taskENTER_CRITICAL(&s_tdma_mux);
    uint32_t generation = s_tdma_generation;
    taskEXIT_CRITICAL(&s_tdma_mux);
    return arm_frame_timer_at_generation(boundary_us, generation);
}

esp_err_t arm_frame_timer_at_generation(int64_t boundary_us, uint32_t generation)
{
    xSemaphoreTake(s_frame_timer_mutex, portMAX_DELAY);
    esp_err_t ret = arm_frame_timer_at_generation_locked(boundary_us, generation);
    xSemaphoreGive(s_frame_timer_mutex);
    return ret;
}

esp_err_t arm_frame_timer_at_generation_locked(int64_t boundary_us, uint32_t generation)
{
    taskENTER_CRITICAL(&s_transport_mux);
    bool stopping = s_stopping;
    taskEXIT_CRITICAL(&s_transport_mux);

    taskENTER_CRITICAL(&s_tdma_mux);
    uint32_t current_generation = s_tdma_generation;
    bool valid = !stopping && generation == current_generation;
    if (valid) {
        s_frame_timer_generation = current_generation;
        s_expected_frame_us = boundary_us;
    }
    taskEXIT_CRITICAL(&s_tdma_mux);
    if (!valid) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t delay_us = boundary_us - esp_timer_get_time();
    if (delay_us < 1) {
        delay_us = 1;
    }
    return esp_timer_start_once(s_frame_timer, delay_us);
}
