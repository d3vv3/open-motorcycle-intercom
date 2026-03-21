#ifndef OMI_AUDIO_VOICE_CLEANUP_H
#define OMI_AUDIO_VOICE_CLEANUP_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    float aec_gain;
    float ns_noise_rms;
    float ns_gain;
} voice_cleanup_state_t;

void voice_cleanup_init(voice_cleanup_state_t *state);
void voice_cleanup_process(voice_cleanup_state_t *state, int16_t *mic_samples,
                           const int16_t *far_ref, size_t count);

#endif
