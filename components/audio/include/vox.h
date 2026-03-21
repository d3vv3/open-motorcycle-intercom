#ifndef OMI_AUDIO_VOX_H
#define OMI_AUDIO_VOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio.h"

typedef struct {
    bool active;
    uint16_t hangover_counter;
    float activation_threshold;
    float deactivation_threshold;
    uint32_t activation_count;
} vox_state_t;

void vox_init(vox_state_t *state, const audio_vox_config_t *config);
bool vox_process(vox_state_t *state, const int16_t *samples, size_t count);

#endif
