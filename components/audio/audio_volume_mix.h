#ifndef AUDIO_VOLUME_MIX_H
#define AUDIO_VOLUME_MIX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

typedef struct {
    atomic_uchar mesh;
    atomic_uchar bluetooth;
} audio_volume_levels_t;

void audio_volume_levels_reset(audio_volume_levels_t *levels);
void audio_volume_levels_set(audio_volume_levels_t *levels, bool bluetooth, uint8_t percent);
uint8_t audio_volume_levels_get(const audio_volume_levels_t *levels, bool bluetooth);

typedef struct {
    int32_t current_q16;
    int32_t step_q16;
    int32_t destination_q16;
    uint32_t remaining;
} audio_gain_ramp_t;

void audio_gain_reset(audio_gain_ramp_t *ramp);
void audio_gain_apply(audio_gain_ramp_t *ramp, int16_t *samples, size_t count,
                      uint8_t percent, uint32_t rate);

/* Used before enqueueing the voice presence marker (including prompt tails). */
size_t audio_voice_contribution_samples(const int16_t *post_gain, size_t base_samples,
                                         size_t prompt_samples);

typedef struct {
    audio_gain_ramp_t voice_weight;
    audio_gain_ramp_t music_weight;
    bool initialized;
} audio_program_mix_t;

/* Contribution is decided once per buffer after source gain; the weights then
 * ramp for 5 ms. A mute can take one extra buffer before normalization starts.
 * Voice presence comes from the converter's queued marker, preserving prompt tails. */
void audio_program_mix(audio_program_mix_t *state, int16_t *voice, size_t frames,
                       size_t voice_present, const int16_t *music, size_t music_samples,
                       uint32_t rate);

typedef enum {
    AUDIO_CUE_IDLE = 0,
    AUDIO_CUE_LIMIT = 1,
    AUDIO_CUE_END = 2,
    AUDIO_CUE_LIMIT_WITH_PENDING_END = 3,
} audio_cue_state_t;

typedef struct {
    atomic_uint state;
    uint32_t position;
    uint32_t phase;
} audio_limit_cue_t;

void audio_limit_cue_reset(audio_limit_cue_t *cue);
bool audio_limit_cue_busy(const audio_limit_cue_t *cue);
bool audio_limit_cue_request(audio_limit_cue_t *cue);
/* Returns true when an end beep is scheduled or coalesced with one in progress. */
bool audio_limit_cue_request_end(audio_limit_cue_t *cue);
void audio_limit_cue_mix(audio_limit_cue_t *cue, int16_t *interleaved,
                         size_t frames, uint8_t channels, uint32_t rate);

#endif
