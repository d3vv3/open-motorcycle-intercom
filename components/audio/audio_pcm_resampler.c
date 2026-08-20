#include "audio_pcm_resampler.h"

#include <string.h>

#define PHASE_ONE_Q32                    UINT64_C(4294967296)
#define DEPTH_FILTER_SHIFT               3
#define PROPORTIONAL_GAIN_PPM_PER_SAMPLE 20
#define INTEGRAL_GAIN_Q16_PER_ERROR_Q8   4

static audio_pcm_resampler_telemetry_t telemetry(const audio_pcm_resampler_t *resampler)
{
    audio_pcm_resampler_telemetry_t result = {0};

    if (resampler != NULL) {
        result.started = resampler->started;
        result.depth_before = (uint16_t)resampler->depth;
        result.depth_after = (uint16_t)resampler->depth;
        result.correction_ppm = resampler->correction_ppm;
    }
    return result;
}

static int32_t clamp_correction_q16(int64_t value)
{
    const int64_t limit = (int64_t)AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM * 65536;

    if (value > limit) {
        return (int32_t)limit;
    }
    if (value < -limit) {
        return (int32_t)-limit;
    }
    return (int32_t)value;
}

static int32_t clamp_output_q16(int64_t value, int32_t positive_limit_ppm)
{
    const int64_t positive_limit = (int64_t)positive_limit_ppm * 65536;
    const int64_t negative_limit = -(int64_t)AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM * 65536;

    if (value > positive_limit) {
        return (int32_t)positive_limit;
    }
    if (value < negative_limit) {
        return (int32_t)negative_limit;
    }
    return (int32_t)value;
}

static void update_controller(audio_pcm_resampler_t *resampler, size_t depth, bool recovery_active)
{
    const int32_t measured_q8 = (int32_t)depth * 256;
    const int32_t target_q8 = (int32_t)AUDIO_PCM_RESAMPLER_TARGET_SAMPLES * 256;
    int32_t error_q8;
    int64_t proportional_q16;
    int64_t candidate_integrator;
    int64_t candidate_output;
    const int32_t positive_limit_ppm = recovery_active ? AUDIO_PCM_RESAMPLER_MAX_RECOVERY_PPM
                                                       : AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM;
    const int64_t positive_limit_q16 = (int64_t)positive_limit_ppm * 65536;
    const int64_t negative_limit_q16 = -(int64_t)AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM * 65536;

    resampler->filtered_depth_q8 +=
        (measured_q8 - resampler->filtered_depth_q8) / (1 << DEPTH_FILTER_SHIFT);
    error_q8 = resampler->filtered_depth_q8 - target_q8;
    proportional_q16 = (int64_t)error_q8 * PROPORTIONAL_GAIN_PPM_PER_SAMPLE * 256;
    candidate_integrator =
        resampler->integrator_q16 + (int64_t)error_q8 * INTEGRAL_GAIN_Q16_PER_ERROR_Q8;
    candidate_integrator = clamp_correction_q16(candidate_integrator);
    candidate_output = proportional_q16 + candidate_integrator;

    if (!((candidate_output > positive_limit_q16 && error_q8 > 0) ||
          (candidate_output < negative_limit_q16 && error_q8 < 0))) {
        resampler->integrator_q16 = (int32_t)candidate_integrator;
    }

    resampler->correction_ppm =
        clamp_output_q16(proportional_q16 + resampler->integrator_q16, positive_limit_ppm) / 65536;
}

static size_t ring_offset(const audio_pcm_resampler_t *resampler, size_t offset)
{
    return (resampler->read_index + offset) % AUDIO_PCM_RESAMPLER_RING_SAMPLES;
}

static int16_t interpolate(int16_t first, int16_t second, uint32_t fraction)
{
    const int32_t difference = (int32_t)second - (int32_t)first;
    const int64_t scaled = (int64_t)difference * (int64_t)fraction;
    const int32_t value = (int32_t)first + (int32_t)(scaled / (int64_t)PHASE_ONE_Q32);

    return (int16_t)value;
}

static void render_fade_to_zero(audio_pcm_resampler_t *resampler,
                                int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES])
{
    size_t i;

    for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        const int64_t remaining = (int64_t)(AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u - i);
        output[i] = (int16_t)((int64_t)resampler->last_output * remaining /
                              (AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u));
    }
    resampler->last_output = 0;
}

void audio_pcm_resampler_reset(audio_pcm_resampler_t *resampler)
{
    if (resampler == NULL) {
        return;
    }

    memset(resampler, 0, sizeof(*resampler));
    resampler->filtered_depth_q8 = (int32_t)AUDIO_PCM_RESAMPLER_TARGET_SAMPLES * 256;
}

size_t audio_pcm_resampler_available(const audio_pcm_resampler_t *resampler)
{
    return resampler == NULL ? 0u : AUDIO_PCM_RESAMPLER_RING_SAMPLES - resampler->depth;
}

size_t audio_pcm_resampler_depth(const audio_pcm_resampler_t *resampler)
{
    return resampler == NULL ? 0u : resampler->depth;
}

audio_pcm_resampler_telemetry_t audio_pcm_resampler_push(audio_pcm_resampler_t *resampler,
                                                         const int16_t *samples,
                                                         size_t sample_count, bool active)
{
    audio_pcm_resampler_telemetry_t result = telemetry(resampler);
    size_t first_count;
    size_t second_count;

    if (resampler == NULL) {
        result.rejected_push = true;
        return result;
    }

    result.depth_before = (uint16_t)resampler->depth;
    if ((samples == NULL && sample_count != 0u) ||
        sample_count > AUDIO_PCM_RESAMPLER_RING_SAMPLES - resampler->depth) {
        result.rejected_push = true;
        return result;
    }

    first_count = sample_count;
    if (first_count > AUDIO_PCM_RESAMPLER_RING_SAMPLES - resampler->write_index) {
        first_count = AUDIO_PCM_RESAMPLER_RING_SAMPLES - resampler->write_index;
    }
    second_count = sample_count - first_count;
    if (first_count != 0u) {
        memcpy(&resampler->ring[resampler->write_index], samples, first_count * sizeof(samples[0]));
        memset(&resampler->activity[resampler->write_index], active ? 1 : 0, first_count);
    }
    if (second_count != 0u) {
        memcpy(resampler->ring, &samples[first_count], second_count * sizeof(samples[0]));
        memset(resampler->activity, active ? 1 : 0, second_count);
    }
    resampler->write_index =
        (resampler->write_index + sample_count) % AUDIO_PCM_RESAMPLER_RING_SAMPLES;
    resampler->depth += sample_count;

    result.depth_after = (uint16_t)resampler->depth;
    result.started = resampler->started;
    return result;
}

audio_pcm_resampler_telemetry_t
audio_pcm_resampler_render(audio_pcm_resampler_t *resampler,
                           int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES],
                           size_t upstream_samples)
{
    audio_pcm_resampler_telemetry_t result = telemetry(resampler);
    uint64_t step_q32;
    int64_t step_adjustment_q32;
    uint64_t last_phase;
    uint64_t final_phase;
    size_t required;
    size_t consumed;
    size_t i;
    bool apply_restart_fade;
    bool last_rendered_activity = false;

    if (resampler == NULL || output == NULL) {
        return result;
    }

    result.depth_before = (uint16_t)resampler->depth;
    result.recovery_active = upstream_samples > 0u;
    if (!resampler->started) {
        if (resampler->depth < AUDIO_PCM_RESAMPLER_START_SAMPLES) {
            for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
                output[i] = 0;
            }
            result.depth_after = (uint16_t)resampler->depth;
            return result;
        }
        resampler->started = true;
    }

    update_controller(resampler, resampler->depth + upstream_samples, result.recovery_active);
    result.correction_ppm = resampler->correction_ppm;
    step_adjustment_q32 = (int64_t)resampler->correction_ppm * (int64_t)PHASE_ONE_Q32 / 1000000;
    step_q32 = (uint64_t)((int64_t)PHASE_ONE_Q32 + step_adjustment_q32);
    last_phase = resampler->phase_q32 + step_q32 * (AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u);
    final_phase = last_phase + step_q32;
    required = (size_t)(last_phase / PHASE_ONE_Q32) + 2u;
    consumed = (size_t)(final_phase / PHASE_ONE_Q32);

    if (required > resampler->depth) {
        result.audible_active = resampler->last_activity;
        render_fade_to_zero(resampler, output);
        resampler->started = false;
        resampler->restart_fade_pending = true;
        resampler->phase_q32 = 0;
        resampler->integrator_q16 = 0;
        resampler->correction_ppm = 0;
        resampler->filtered_depth_q8 = (int32_t)AUDIO_PCM_RESAMPLER_TARGET_SAMPLES * 256;
        resampler->last_activity = false;
        result.started = false;
        result.underrun = true;
        result.correction_ppm = 0;
        result.depth_after = (uint16_t)resampler->depth;
        return result;
    }

    apply_restart_fade = resampler->restart_fade_pending;
    for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        const size_t offset = (size_t)(resampler->phase_q32 / PHASE_ONE_Q32);
        const uint32_t fraction = (uint32_t)resampler->phase_q32;
        const size_t first_index = ring_offset(resampler, offset);
        const size_t second_index = ring_offset(resampler, offset + 1u);
        int16_t sample =
            interpolate(resampler->ring[ring_offset(resampler, offset)],
                        resampler->ring[ring_offset(resampler, offset + 1u)], fraction);
        bool sample_active = resampler->activity[first_index] != 0u ||
                             (fraction != 0u && resampler->activity[second_index] != 0u);

        if (apply_restart_fade && i < AUDIO_PCM_RESAMPLER_RESTART_FADE_SAMPLES) {
            sample = (int16_t)((int64_t)sample * (int64_t)(i + 1u) /
                               AUDIO_PCM_RESAMPLER_RESTART_FADE_SAMPLES);
        }
        output[i] = sample;
        result.audible_active = result.audible_active || sample_active;
        last_rendered_activity = sample_active;
        resampler->phase_q32 += step_q32;
    }
    resampler->restart_fade_pending = false;
    resampler->read_index = (resampler->read_index + consumed) % AUDIO_PCM_RESAMPLER_RING_SAMPLES;
    resampler->depth -= consumed;
    resampler->phase_q32 -= (uint64_t)consumed * PHASE_ONE_Q32;
    resampler->last_output = output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u];
    resampler->last_activity = last_rendered_activity;

    result.started = true;
    result.depth_after = (uint16_t)resampler->depth;
    result.consumed_count = (uint16_t)consumed;
    return result;
}
