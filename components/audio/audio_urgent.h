#ifndef AUDIO_URGENT_H
#define AUDIO_URGENT_H

#include <stdatomic.h>
#include "audio_prompt.h"

typedef struct {
    atomic_uint intent; /* epoch in upper bits; eligible, pending, claimed in low bits */
    uint32_t playing_epoch;
    bool playing;
    audio_prompt_player_t player;
    uint32_t mixed_samples;
    uint32_t output_samples;
} audio_urgent_t;

uint32_t audio_urgent_set_incoming(audio_urgent_t *urgent, bool incoming);
bool audio_urgent_request(audio_urgent_t *urgent);
bool audio_urgent_active(const audio_urgent_t *urgent);
/** Returns the generation mixed (zero if no prompt). If intent changes before
 * the hardware write, the caller must restore its unmodified PCM. */
uint32_t audio_urgent_mix(audio_urgent_t *urgent, const audio_prompt_clip_t *clip,
                          int16_t *output, size_t frames, uint8_t channels, uint32_t rate);
bool audio_urgent_epoch_valid(const audio_urgent_t *urgent, uint32_t epoch);

#endif
