/**
 * @file audio_notify.c
 * @brief Notification tone synthesis mixed into the playout frame.
 */

#include "audio_internal.h"
#include "audio_notification_tone.h"

static uint8_t notification_tone_count(audio_notify_t type)
{
    if (type == AUDIO_NOTIFY_STARTUP) {
        return 3;
    }
    if (type == AUDIO_NOTIFY_PEER_JOIN || type == AUDIO_NOTIFY_PEER_LEAVE ||
        type == AUDIO_NOTIFY_MESH_ENABLED || type == AUDIO_NOTIFY_MESH_DISABLED) {
        return 2;
    }
    if (type == AUDIO_NOTIFY_BLUETOOTH_PAIRING) return 3;
    return 1;
}

static uint32_t notification_frequency_millihz(audio_notify_t type, uint8_t tone_index)
{
    static const uint32_t startup[] = {261630u, 329630u, 392000u};
    static const uint32_t join[] = {440000u, 880000u};
    static const uint32_t leave[] = {880000u, 440000u};
    switch (type) {
    case AUDIO_NOTIFY_STARTUP:
        return startup[tone_index];
    case AUDIO_NOTIFY_PEER_JOIN:
        return join[tone_index];
    case AUDIO_NOTIFY_PEER_LEAVE:
        return leave[tone_index];
    case AUDIO_NOTIFY_MESH_ENABLED:
        return join[tone_index];
    case AUDIO_NOTIFY_MESH_DISABLED:
        return leave[tone_index];
    case AUDIO_NOTIFY_BLUETOOTH_PAIRING:
        return 988000u;
    default:
        return 0u;
    }
}

static void notification_start_tone(audio_notification_state_t *note)
{
    note->phase = 0u;
    note->phase_step = audio_notification_phase_step(
        notification_frequency_millihz(note->type, note->tone_index),
        g_audio.config.sample_rate);
}

size_t audio_notify_mix_frame(size_t base_present_samples, bool *request_consumed)
{
    audio_notification_state_t *note = &g_audio.notification;
    size_t contributed = 0u;
    *request_consumed = false;
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        if (!note->active) {
            audio_notification_request_t request;
            if (xQueueReceive(g_audio.notification_queue, &request, 0) != pdTRUE) {
                return contributed;
            }
            note->active = true;
            *request_consumed = true;
            AUDIO_STATS_LOCK();
            if (g_audio.stats.notify_started_count != UINT32_MAX)
                g_audio.stats.notify_started_count++;
            AUDIO_STATS_UNLOCK();
            note->type = (audio_notify_t)request.type;
            note->tone_index = 0;
            note->segment_sample = 0;
            note->in_gap = false;
            notification_start_tone(note);
        }
        int32_t tone = 0;
        if (!note->in_gap) {
            tone = audio_notification_tone_sample(note->phase);
            note->phase += note->phase_step;
            contributed++;
            g_audio.pcm_output[i] = audio_route_mix_sample(false, i < base_present_samples, true,
                                                            g_audio.pcm_output[i],
                                                            audio_route_saturate(tone), 0);
        }

        note->segment_sample++;
        uint16_t segment_length =
            note->in_gap ? NOTIFICATION_GAP_SAMPLES : NOTIFICATION_BEEP_SAMPLES;
        if (note->segment_sample < segment_length) {
            continue;
        }
        note->segment_sample = 0;
        if (note->in_gap) {
            note->in_gap = false;
            note->tone_index++;
            notification_start_tone(note);
        } else if (note->tone_index + 1u < notification_tone_count(note->type)) {
            note->in_gap = true;
        } else {
            note->active = false;
            AUDIO_STATS_LOCK();
            if (g_audio.stats.notify_completed_count != UINT32_MAX)
                g_audio.stats.notify_completed_count++;
            AUDIO_STATS_UNLOCK();
        }
        /* Start each tone or gap on a frame boundary so contribution remains a prefix. */
        return contributed;
    }
    return contributed;
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (type < AUDIO_NOTIFY_STARTUP || type > AUDIO_NOTIFY_BLUETOOTH_PAIRING) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing ||
        g_audio.notification_queue == NULL) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    audio_notification_request_t request = {.type = (uint8_t)type};
    if (xQueueSend(g_audio.notification_queue, &request, 0) != pdTRUE) {
        AUDIO_STATS_LOCK();
        g_audio.stats.notification_queue_overflows++;
        AUDIO_STATS_UNLOCK();
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(lifecycle_mutex);
    return ESP_OK;
}
