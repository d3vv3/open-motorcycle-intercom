/**
 * @file audio_rx.c
 * @brief RX packet admission and source reset handshakes.
 */

#include "audio_internal.h"
#include "audio_rx_source_select.h"

#include <string.h>

#include "esp_timer.h"

/* Caller holds rx_sources_mutex. Decoder and resampler resets remain playout-owned. */
void audio_rx_reset_source_metadata_locked(void)
{
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        audio_rx_source_t *source = &g_audio.rx_sources[i];
        source->assigned = false;
        source->source_id = 0;
        source->last_enqueue_ms = 0;
        source->last_active_ms = 0;
        source->decoder_reset_pending = true;
        audio_packet_store_reset(&source->packet_store);
    }
}

void audio_rx_reset_source_metadata(void)
{
    xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
    audio_rx_reset_source_metadata_locked();
    xSemaphoreGive(g_audio.rx_sources_mutex);
}

void audio_rx_reset_codecs_and_resamplers(void)
{
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        opus_decoder_ctl(g_audio.rx_sources[i].decoder, OPUS_RESET_STATE);
        audio_pcm_resampler_reset(&g_audio.rx_sources[i].resampler);
        g_audio.rx_sources[i].decoded_active = false;
    }
}

void audio_rx_service_reset_request(void)
{
    if (!atomic_exchange_explicit(&g_audio.rx_reset_requested, false, memory_order_acq_rel)) {
        return;
    }
    audio_rx_reset_source_metadata();
    audio_rx_reset_codecs_and_resamplers();
    xSemaphoreGive(g_audio.rx_reset_done);
}

static audio_packet_t packet_from_frame(const audio_frame_t *frame)
{
    audio_packet_t packet = {
        .length = frame->len,
        .sequence = frame->seq,
        .mode = frame->has_seq ? AUDIO_PACKET_MODE_SEQUENCED
                               : AUDIO_PACKET_MODE_ARRIVAL_ORDER,
        .active = frame->active,
        .received_us = frame->timestamp_ms > 0
                           ? (uint64_t)frame->timestamp_ms * UINT64_C(1000)
                           : 0u,
    };
    memcpy(packet.data, frame->data, frame->len);
    return packet;
}

/* Caller holds rx_sources_mutex. */
static audio_rx_source_t *admit_source_locked(const audio_frame_t *frame, uint8_t source_id,
                                              uint64_t now_ms)
{
    audio_rx_slot_snapshot_t slot_snapshot[AUDIO_MAX_RX_SOURCES];
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        slot_snapshot[i].assigned = g_audio.rx_sources[i].assigned;
        slot_snapshot[i].source_id = g_audio.rx_sources[i].source_id;
        slot_snapshot[i].last_active_ms = g_audio.rx_sources[i].last_active_ms;
    }
    audio_rx_select_decision_t decision = audio_rx_source_select(
        slot_snapshot, AUDIO_MAX_RX_SOURCES, source_id, frame->active, now_ms);
    if (decision.action == AUDIO_RX_SELECT_REJECT) {
        return NULL;
    }
    audio_rx_source_t *source = &g_audio.rx_sources[decision.slot_index];
    if (decision.action != AUDIO_RX_SELECT_MATCH) {
        if (decision.action == AUDIO_RX_SELECT_EVICT) {
            source->assigned = false;
            source->last_active_ms = 0;
            AUDIO_STATS_LOCK();
            g_audio.stats.rx_source_evictions++;
            AUDIO_STATS_UNLOCK();
        }
        source->assigned = true;
        source->source_id = source_id;
        source->decoder_reset_pending = true;
        audio_packet_store_reset(&source->packet_store);
    }
    if (frame->active) {
        source->last_active_ms = now_ms;
    }
    return source;
}

/* Caller holds rx_sources_mutex. */
static audio_packet_store_push_result_t push_packet_locked(audio_rx_source_t *source,
                                                           audio_packet_t *packet,
                                                           uint64_t now_ms, bool *reanchored)
{
    audio_packet_store_push_result_t result =
        audio_packet_store_push(&source->packet_store, packet, now_ms);
    if (result == AUDIO_PACKET_STORE_PUSH_FUTURE) {
        audio_packet_store_reset(&source->packet_store);
        audio_packet_store_push_result_t reanchor_result =
            audio_packet_store_push(&source->packet_store, packet, now_ms);
        if (reanchor_result == AUDIO_PACKET_STORE_PUSH_OK) {
            source->decoder_reset_pending = true;
            result = AUDIO_PACKET_STORE_PUSH_OK;
            *reanchored = true;
        } else {
            result = reanchor_result;
        }
    }
    if (result == AUDIO_PACKET_STORE_PUSH_OK) {
        source->last_enqueue_ms = now_ms;
    }
    return result;
}

static esp_err_t record_push_result(audio_packet_store_push_result_t result, bool reanchored)
{
    esp_err_t ret = ESP_OK;
    AUDIO_STATS_LOCK();
    if (reanchored) {
        g_audio.stats.packet_future_drops++;
        g_audio.stats.seq_resets++;
        g_audio.stats.glitches_detected++;
    }
    if (result != AUDIO_PACKET_STORE_PUSH_OK) {
        g_audio.stats.frames_dropped++;
        switch (result) {
        case AUDIO_PACKET_STORE_PUSH_DUPLICATE:
            g_audio.stats.packet_duplicate_drops++;
            g_audio.stats.seq_stale_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_LATE:
            g_audio.stats.packet_late_drops++;
            g_audio.stats.seq_stale_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_FUTURE:
            g_audio.stats.packet_future_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_FULL:
            g_audio.stats.rx_queue_overflows++;
            ret = ESP_ERR_NO_MEM;
            break;
        default:
            g_audio.stats.rx_source_rejections++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        }
    }
    AUDIO_STATS_UNLOCK();
    return ret;
}

esp_err_t audio_put_rx_frame(const audio_frame_t *frame, uint8_t source_id)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->len == 0 || frame->len > AUDIO_PACKET_MAX_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }

    audio_packet_t packet = packet_from_frame(frame);

    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (g_audio.deinitializing || g_audio.rx_sources_mutex == NULL) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(g_audio.rx_sources_mutex, pdMS_TO_TICKS(RX_ENQUEUE_LOCK_WAIT_MS)) !=
        pdTRUE) {
        AUDIO_STATS_LOCK();
        g_audio.stats.frames_dropped++;
        g_audio.stats.rx_lock_drops++;
        AUDIO_STATS_UNLOCK();
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_TIMEOUT;
    }
    if (!g_audio.initialized ||
        !atomic_load_explicit(&g_audio.running, memory_order_acquire) || source_id == 0 ||
        g_audio.config.mode != AUDIO_MODE_MESH) {
        xSemaphoreGive(g_audio.rx_sources_mutex);
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    int64_t now_us = esp_timer_get_time();
    uint64_t now_ms = (uint64_t)(now_us / 1000);
    audio_rx_source_t *source = admit_source_locked(frame, source_id, now_ms);
    if (source == NULL) {
        xSemaphoreGive(g_audio.rx_sources_mutex);
        AUDIO_STATS_LOCK();
        g_audio.stats.frames_dropped++;
        g_audio.stats.rx_source_rejections++;
        AUDIO_STATS_UNLOCK();
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }
    if (packet.received_us == 0u) {
        packet.received_us = (uint64_t)now_us;
    }
    bool reanchored = false;
    audio_packet_store_push_result_t result =
        push_packet_locked(source, &packet, now_ms, &reanchored);
    xSemaphoreGive(g_audio.rx_sources_mutex);

    esp_err_t ret = record_push_result(result, reanchored);
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ret;
}

void audio_clear_rx_frames(void)
{
    if (g_audio.lifecycle_mutex == NULL) {
        return;
    }
    if (audio_called_from_worker()) {
        return;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing ||
        g_audio.rx_reset_mutex == NULL) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return;
    }
    xSemaphoreTake(g_audio.rx_reset_mutex, portMAX_DELAY);
    xSemaphoreTake(g_audio.rx_reset_done, 0);
    portENTER_CRITICAL(&g_audio_task_lock);
    bool have_playout = g_audio.playout_task != NULL;
    if (have_playout) {
        atomic_store_explicit(&g_audio.rx_reset_requested, true, memory_order_release);
    }
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (have_playout) {
        xSemaphoreTake(g_audio.rx_reset_done, portMAX_DELAY);
    } else {
        audio_rx_reset_source_metadata();
    }
    xSemaphoreGive(g_audio.rx_reset_mutex);
    xSemaphoreGive(g_audio.lifecycle_mutex);
}
