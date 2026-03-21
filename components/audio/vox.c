#include "vox.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "power.h"

#define VOX_HANGOVER_FRAMES 15

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

    ESP_LOGI(TAG, "Initialized");
    ESP_LOGI(TAG, "  Activation threshold: %.3f", config->activation_threshold);
    ESP_LOGI(TAG, "  Deactivation threshold: %.3f", config->deactivation_threshold);
    ESP_LOGI(TAG, "  Hangover: %u ms", config->hangover_ms);
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
            power_notify_voice_start();
            ESP_LOGD(TAG, "Activated (RMS: %.4f)", rms);
        }
        state->hangover_counter = VOX_HANGOVER_FRAMES;
    } else if (rms < state->deactivation_threshold) {
        if (state->hangover_counter > 0) {
            state->hangover_counter--;
        } else if (state->active) {
            state->active = false;
            power_notify_voice_end();
            ESP_LOGD(TAG, "Deactivated");
        }
    }

    return state->active;
}
