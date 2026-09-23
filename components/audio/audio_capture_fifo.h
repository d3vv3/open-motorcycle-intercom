#ifndef AUDIO_CAPTURE_FIFO_H
#define AUDIO_CAPTURE_FIFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t *samples;
    size_t capacity;
    size_t read_index;
    size_t write_index;
    size_t depth;
    uint64_t samples_written;
    uint64_t samples_read;
    uint32_t startup_silence_frames;
} audio_capture_fifo_t;

bool audio_capture_fifo_init(audio_capture_fifo_t *fifo, int16_t *storage, size_t capacity);
void audio_capture_fifo_reset(audio_capture_fifo_t *fifo);
bool audio_capture_fifo_push(audio_capture_fifo_t *fifo, const int16_t *samples, size_t count);
bool audio_capture_fifo_pop_frame(audio_capture_fifo_t *fifo, int16_t *frame, size_t frame_samples);

#endif
