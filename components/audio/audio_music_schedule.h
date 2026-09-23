#ifndef AUDIO_MUSIC_SCHEDULE_H
#define AUDIO_MUSIC_SCHEDULE_H

#include <stddef.h>
#include <stdint.h>

#include "audio_rate_converter.h"

#define AUDIO_MUSIC_MAX_INPUT_CHUNKS 4u

static inline size_t audio_music_input_chunk_limit(uint32_t input_rate_hz,
                                                   size_t output_samples,
                                                   size_t remaining_input)
{
    size_t wanted = ((size_t)input_rate_hz * output_samples + 47999u) / 48000u;
    size_t limit = wanted;
    size_t max_input = AUDIO_MUSIC_MAX_INPUT_CHUNKS * AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES;
    if (limit > max_input) limit = max_input;
    if (limit > remaining_input) limit = remaining_input;
    return limit;
}

#endif
