#ifndef AUDIO_SAMPLE_FIFO_H
#define AUDIO_SAMPLE_FIFO_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t *samples;
    size_t capacity;
    size_t read_index;
    size_t write_index;
    size_t depth;
} audio_sample_fifo_t;

void audio_sample_fifo_init(audio_sample_fifo_t *fifo, int16_t *storage, size_t capacity);
void audio_sample_fifo_reset(audio_sample_fifo_t *fifo);
size_t audio_sample_fifo_write(audio_sample_fifo_t *fifo, const int16_t *samples, size_t count);
size_t audio_sample_fifo_read(audio_sample_fifo_t *fifo, int16_t *samples, size_t count);

#endif
