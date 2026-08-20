#include "vox.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

#define VOX_FRAME_MS 20U

static const char *TAG = "vox";

static float calculate_rms(const int16_t *samples, size_t count)
{
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float x = (float)samples[i] / 32768.0f;
        sum += x * x;
    }
    return (count > 0) ? sqrtf(sum / (float)count) : 0.0f;
}

void vox_init(vox_state_t *state, const audio_vox_config_t *config)
{
    if (!state || !config) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->activation_threshold = config->activation_threshold;
    state->deactivation_threshold = config->deactivation_threshold;
    state->hangover_frames = (uint16_t)((config->hangover_ms + (VOX_FRAME_MS - 1U)) / VOX_FRAME_MS);
    state->min_active_frames =
        (uint16_t)((config->min_active_ms + (VOX_FRAME_MS - 1U)) / VOX_FRAME_MS);
    if (state->hangover_frames == 0U) {
        state->hangover_frames = 1U;
    }
    if (state->min_active_frames == 0U) {
        state->min_active_frames = 1U;
    }

    ESP_LOGI(TAG, "Initialized");
    ESP_LOGI(TAG, "  Activation threshold: %.3f", config->activation_threshold);
    ESP_LOGI(TAG, "  Deactivation threshold: %.3f", config->deactivation_threshold);
    ESP_LOGI(TAG, "  Min active: %u ms (%u frames)", config->min_active_ms,
             state->min_active_frames);
    ESP_LOGI(TAG, "  Hangover: %u ms (%u frames)", config->hangover_ms, state->hangover_frames);
}

bool vox_process(vox_state_t *state, const int16_t *samples, size_t count)
{
    if (!state || !samples || count == 0) {
        return false;
    }

    float rms = calculate_rms(samples, count);

    if (rms > state->activation_threshold) {
        if (!state->active) {
            state->active = true;
            state->activation_count++;
            state->min_active_counter = state->min_active_frames;
            ESP_LOGD(TAG, "Activated (RMS: %.4f)", rms);
        }
        state->hangover_counter = state->hangover_frames;
    } else if (rms < state->deactivation_threshold) {
        if (state->active) {
            if (state->min_active_counter > 0) {
                state->min_active_counter--;
            }

            if (state->min_active_counter == 0) {
                if (state->hangover_counter > 0) {
                    state->hangover_counter--;
                } else {
                    state->active = false;
                    ESP_LOGD(TAG, "Deactivated");
                }
            }
        }
    } else if (state->active && state->min_active_counter > 0) {
        state->min_active_counter--;
    }

    return state->active;
}
