/**
 * @file audio_notify.c
 * @brief Notification tone synthesis mixed into the playout frame.
 */

#include <math.h>

#include "audio_internal.h"

static uint8_t notification_tone_count(audio_notify_t type)
{
    if (type == AUDIO_NOTIFY_STARTUP) {
        return 3;
    }
    if (type == AUDIO_NOTIFY_PEER_JOIN || type == AUDIO_NOTIFY_PEER_LEAVE) {
        return 2;
    }
    return 1;
}

static float notification_frequency(audio_notify_t type, uint8_t tone_index)
{
    static const float startup[] = {261.63f, 329.63f, 392.00f};
    static const float join[] = {440.0f, 880.0f};
    static const float leave[] = {880.0f, 440.0f};
    switch (type) {
    case AUDIO_NOTIFY_STARTUP:
        return startup[tone_index];
    case AUDIO_NOTIFY_PEER_JOIN:
        return join[tone_index];
    case AUDIO_NOTIFY_PEER_LEAVE:
        return leave[tone_index];
    case AUDIO_NOTIFY_MESH_ENABLED:
        return 329.63f;
    case AUDIO_NOTIFY_MESH_DISABLED:
        return 261.63f;
    default:
        return 0.0f;
    }
}

void audio_notify_mix_frame(void)
{
    audio_notification_state_t *note = &g_audio.notification;
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        if (!note->active) {
            audio_notification_request_t request;
            if (xQueueReceive(g_audio.notification_queue, &request, 0) != pdTRUE) {
                return;
            }
            note->active = true;
            note->type = (audio_notify_t)request.type;
            note->tone_index = 0;
            note->segment_sample = 0;
            note->in_gap = false;
        }
        int32_t tone = 0;
        if (!note->in_gap) {
            float phase = 2.0f * M_PI * notification_frequency(note->type, note->tone_index) *
                          note->segment_sample / g_audio.config.sample_rate;
            tone = (int32_t)(NOTIFICATION_AMPLITUDE * 32767.0f * sinf(phase));
        }
        int32_t mixed = (int32_t)g_audio.pcm_output[i] + tone;
        if (mixed > INT16_MAX) {
            mixed = INT16_MAX;
        } else if (mixed < INT16_MIN) {
            mixed = INT16_MIN;
        }
        g_audio.pcm_output[i] = (int16_t)mixed;

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
        } else if (note->tone_index + 1u < notification_tone_count(note->type)) {
            note->in_gap = true;
        } else {
            note->active = false;
        }
    }
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (type < AUDIO_NOTIFY_STARTUP || type > AUDIO_NOTIFY_MESH_DISABLED) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing ||
        g_audio.notification_queue == NULL) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    audio_notification_request_t request = {.type = (uint8_t)type};
    if (xQueueSend(g_audio.notification_queue, &request, 0) != pdTRUE) {
        AUDIO_STATS_LOCK();
        g_audio.stats.notification_queue_overflows++;
        AUDIO_STATS_UNLOCK();
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ESP_OK;
}
