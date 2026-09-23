#ifndef AUDIO_REBLOCK_H
#define AUDIO_REBLOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_REBLOCK_MAX_CHUNK_SIZE 512u
#define AUDIO_REBLOCK_FRAME_SAMPLES 320u

typedef void (*audio_reblock_process_fn)(void *context, const int16_t *mic,
                                         const int16_t *ref, int16_t *out);

typedef struct {
    int16_t *mic_fifo;
    int16_t *ref_fifo;
    int16_t *out_fifo;
    size_t capacity;
    size_t chunk_size;
    size_t mic_depth;
    size_t ref_depth;
    size_t out_depth;
    size_t mic_read;
    size_t mic_write;
    size_t ref_read;
    size_t ref_write;
    size_t out_read;
    size_t out_write;
    bool primed;
    bool overflowed;
    uint32_t chunks_processed;
    uint32_t startup_delay_frames;
    uint64_t input_total;
    uint64_t output_total;
    _Alignas(16) int16_t mic_chunk[AUDIO_REBLOCK_MAX_CHUNK_SIZE];
    _Alignas(16) int16_t ref_chunk[AUDIO_REBLOCK_MAX_CHUNK_SIZE];
    _Alignas(16) int16_t out_chunk[AUDIO_REBLOCK_MAX_CHUNK_SIZE];
} audio_reblock_t;

/* Compatibility name for callers that used the old terminology. */
#define startup_fallback_frames startup_delay_frames

bool audio_reblock_init(audio_reblock_t *state, int16_t *mic_fifo, int16_t *ref_fifo,
                        int16_t *out_fifo, size_t capacity, size_t chunk_size);
void audio_reblock_reset(audio_reblock_t *state);
size_t audio_reblock_process_frame(audio_reblock_t *state, const int16_t *mic,
                                   const int16_t *ref, int16_t *out, size_t frame_samples,
                                    audio_reblock_process_fn process, void *context);

#endif
