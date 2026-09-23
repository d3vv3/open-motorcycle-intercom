#include "voice_cleanup.h"

#include <math.h>
#include <stdbool.h>

#define NS_MIN_GAIN                0.18f
#define NS_NOISE_UPDATE_IDLE_ALPHA 0.04f
#define NS_NOISE_UPDATE_TALK_ALPHA 0.001f
#define NS_GAIN_ATTACK_ALPHA       0.35f
#define NS_GAIN_RELEASE_ALPHA      0.04f
#define NS_SPEECH_MULTIPLIER       2.8f

static float calculate_rms(const int16_t *samples, size_t count)
{
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float x = (float)samples[i];
        sum += x * x;
    }
    return (count > 0) ? sqrtf(sum / (float)count) : 0.0f;
}

void voice_cleanup_init(voice_cleanup_state_t *state)
{
    if (!state) {
        return;
    }
    state->ns_noise_rms = 800.0f;
    state->ns_gain = 1.0f;
}

void voice_cleanup_process(voice_cleanup_state_t *state, int16_t *mic_samples, size_t count)
{
    if (!state || !mic_samples || count == 0) {
        return;
    }

    float rms = calculate_rms(mic_samples, count);
    bool speech_like = (rms > (state->ns_noise_rms * NS_SPEECH_MULTIPLIER));
    float noise_alpha = speech_like ? NS_NOISE_UPDATE_TALK_ALPHA : NS_NOISE_UPDATE_IDLE_ALPHA;
    state->ns_noise_rms = (1.0f - noise_alpha) * state->ns_noise_rms + noise_alpha * rms;

    float target_gain = 1.0f;
    if (rms > 1.0f) {
        float snr_ratio = (rms - state->ns_noise_rms) / rms;
        if (snr_ratio < 0.0f) {
            snr_ratio = 0.0f;
        }
        target_gain = snr_ratio;
    }

    if (target_gain < NS_MIN_GAIN) {
        target_gain = NS_MIN_GAIN;
    }
    if (target_gain > 1.0f) {
        target_gain = 1.0f;
    }

    float gain_alpha =
        (target_gain < state->ns_gain) ? NS_GAIN_ATTACK_ALPHA : NS_GAIN_RELEASE_ALPHA;
    state->ns_gain = (1.0f - gain_alpha) * state->ns_gain + gain_alpha * target_gain;

    for (size_t i = 0; i < count; i++) {
        float y = (float)mic_samples[i] * state->ns_gain;
        if (y > 32767.0f) {
            y = 32767.0f;
        }
        if (y < -32768.0f) {
            y = -32768.0f;
        }
        mic_samples[i] = (int16_t)y;
    }
}
