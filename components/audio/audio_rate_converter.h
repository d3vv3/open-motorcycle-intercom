#ifndef AUDIO_RATE_CONVERTER_H
#define AUDIO_RATE_CONVERTER_H

#include <stddef.h>
#include <stdint.h>

#define AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES 320u

typedef struct audio_rate_converter audio_rate_converter_t;

typedef struct {
    uint32_t count;
    uint64_t total_us;
    uint32_t max_us;
    uint32_t source_rate_hz;
    uint32_t destination_rate_hz;
    int32_t last_vendor_result;
} audio_rate_converter_timing_t;

/* Supported rates are 16000, 32000, 44100, and 48000 Hz. HFP's 8 kHz
 * conversion is performed by the voice-route conversion path. */
int audio_rate_converter_create(audio_rate_converter_t **converter,
                                uint32_t source_rate_hz,
                                uint32_t destination_rate_hz);

/* Returns the maximum number of output frames for a chunk of input_frames.
 * input_frames must be at most AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES. */
int audio_rate_converter_get_max_output_frames(audio_rate_converter_t *converter,
                                               size_t input_frames,
                                               size_t *output_frames);

/* Converts one bounded chunk. On failure, *output_frames is zero. A processing
 * failure resets converter state before returning. On success, *output_frames reports
 * the number of samples actually produced. Input and output must not overlap
 * for rate conversion; the vendor API does not guarantee in-place operation. */
int audio_rate_converter_process(audio_rate_converter_t *converter,
                                const int16_t *input, size_t input_frames,
                                int16_t *output, size_t output_capacity_frames,
                                 size_t *output_frames);

void audio_rate_converter_timing_snapshot(audio_rate_converter_t *converter,
                                          audio_rate_converter_timing_t *snapshot);
void audio_rate_converter_timing_reset(audio_rate_converter_t *converter);

int audio_rate_converter_reset(audio_rate_converter_t *converter);
void audio_rate_converter_close(audio_rate_converter_t **converter);

#endif
