/** Voice notifications mixed in the 16 kHz playout domain. */
#include "audio_internal.h"

size_t audio_notify_mix_frame(size_t base_present_samples, bool *request_consumed)
{
    audio_notification_state_t *note = &g_audio.notification;
    size_t contributed = 0u;
    *request_consumed = false;
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        if (!note->active) {
            audio_notification_request_t request;
            if (xQueueReceive(g_audio.notification_queue, &request, 0) != pdTRUE)
                return contributed;
            *request_consumed = true;
            note->active = audio_prompt_start(
                &note->player, audio_prompt_for_notification((audio_notify_t)request.type));
            if (!note->active) continue;
            note->mixed_samples = 0u;
            note->output_samples = (uint32_t)(((uint64_t)note->player.clip->samples *
                                               g_audio.config.sample_rate + 15999u) / 16000u);
            AUDIO_STATS_LOCK();
            if (g_audio.stats.notify_started_count != UINT32_MAX)
                g_audio.stats.notify_started_count++;
            AUDIO_STATS_UNLOCK();
        }
        int16_t sample;
        if (audio_prompt_next(&note->player, g_audio.config.sample_rate, &sample)) {
            g_audio.pcm_output[i] = audio_prompt_mix(
                i < base_present_samples ? g_audio.pcm_output[i] : 0, sample,
                note->mixed_samples++, note->output_samples, g_audio.config.sample_rate);
            contributed = i + 1u;
        }
        if (note->player.position >= note->player.clip->samples) {
            note->active = false;
            AUDIO_STATS_LOCK();
            if (g_audio.stats.notify_completed_count != UINT32_MAX)
                g_audio.stats.notify_completed_count++;
            AUDIO_STATS_UNLOCK();
        }
    }
    return contributed;
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) return ESP_ERR_INVALID_STATE;
    if (audio_called_from_worker()) return ESP_ERR_INVALID_STATE;
    if (audio_prompt_for_notification(type) == NULL) return ESP_ERR_INVALID_ARG;
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
