#ifndef OMI_AUDIO_PCM_RESAMPLER_H
#define OMI_AUDIO_PCM_RESAMPLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_PCM_RESAMPLER_RATE_HZ 16000u
#define AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES 320u
#define AUDIO_PCM_RESAMPLER_RING_SAMPLES 2880u
#define AUDIO_PCM_RESAMPLER_TARGET_SAMPLES 960u
#define AUDIO_PCM_RESAMPLER_START_SAMPLES 960u
#define AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM 1000
#define AUDIO_PCM_RESAMPLER_MAX_RECOVERY_PPM 20000
#define AUDIO_PCM_RESAMPLER_RESTART_FADE_SAMPLES 32u

typedef struct {
    bool started;
    bool underrun;
    bool rejected_push;
    uint16_t depth_before;
    uint16_t depth_after;
    uint16_t consumed_count;
    int32_t correction_ppm;
    bool audible_active;
    bool recovery_active;
} audio_pcm_resampler_telemetry_t;

typedef struct {
    int16_t ring[AUDIO_PCM_RESAMPLER_RING_SAMPLES];
    uint8_t activity[AUDIO_PCM_RESAMPLER_RING_SAMPLES];
    size_t read_index;
    size_t write_index;
    size_t depth;
    uint64_t phase_q32;
    int32_t filtered_depth_q8;
    int32_t integrator_q16;
    int32_t correction_ppm;
    int16_t last_output;
    bool last_activity;
    bool started;
    bool restart_fade_pending;
} audio_pcm_resampler_t;

void audio_pcm_resampler_reset(audio_pcm_resampler_t *resampler);

/** Return the number of PCM samples that can be pushed without rejection. */
size_t audio_pcm_resampler_available(const audio_pcm_resampler_t *resampler);
size_t audio_pcm_resampler_depth(const audio_pcm_resampler_t *resampler);

audio_pcm_resampler_telemetry_t audio_pcm_resampler_push(
    audio_pcm_resampler_t *resampler, const int16_t *samples, size_t sample_count, bool active);

audio_pcm_resampler_telemetry_t audio_pcm_resampler_render(
    audio_pcm_resampler_t *resampler,
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES], size_t upstream_samples);

#endif
