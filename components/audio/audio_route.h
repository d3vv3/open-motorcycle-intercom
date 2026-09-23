#ifndef AUDIO_ROUTE_H
#define AUDIO_ROUTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_ROUTE_INPUT_CAPACITY 16384u
#define AUDIO_ROUTE_OUTPUT_SAMPLES 320u
/* Public route calls are bounded so producers cannot hold route_mutex over a large copy. */
#ifndef AUDIO_ROUTE_MAX_ENQUEUE_FRAMES
#define AUDIO_ROUTE_MAX_ENQUEUE_FRAMES 1024u
#endif
/* HFP callers request at most one 20 ms hardware frame per call. */
#ifndef AUDIO_ROUTE_MAX_MIC_READ_SAMPLES
#define AUDIO_ROUTE_MAX_MIC_READ_SAMPLES 320u
#endif

typedef struct {
    int16_t *samples;
    size_t capacity;
    size_t read_index;
    size_t write_index;
    size_t depth;
    uint32_t sample_rate;
    uint8_t channels;
    bool configured;
    bool active;
    uint64_t phase_q32;
    uint64_t step_q32;
    uint32_t overflow_count;
    uint32_t underrun_count;
} audio_route_stream_t;

bool audio_route_stream_init(audio_route_stream_t *stream, int16_t *storage,
                             size_t capacity);
void audio_route_stream_reset(audio_route_stream_t *stream);
bool audio_route_stream_configure(audio_route_stream_t *stream, uint32_t sample_rate,
                                  uint8_t channels);
void audio_route_stream_set_active(audio_route_stream_t *stream, bool active);
size_t audio_route_stream_enqueue(audio_route_stream_t *stream, const int16_t *interleaved,
                                  size_t frames);
/* Render a contiguous prefix, silence-fill the suffix, and return the prefix length. */
size_t audio_route_stream_render(audio_route_stream_t *stream, int16_t *output, size_t samples);
size_t audio_route_stream_read_raw(audio_route_stream_t *stream, int16_t *output, size_t samples);

void audio_route_mic_reset(audio_route_stream_t *stream);
size_t audio_route_mic_write(audio_route_stream_t *stream, const int16_t *samples, size_t count);
size_t audio_route_mic_read(audio_route_stream_t *stream, int16_t *output, size_t count,
                            uint32_t output_rate);

int16_t audio_route_saturate(int32_t sample);
int16_t audio_route_mix_sample(bool call_active, bool base_present, bool music_present,
                              int16_t base, int16_t music, int16_t call);

#endif
