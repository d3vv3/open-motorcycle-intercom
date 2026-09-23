#include "audio_sample_fifo.h"

#include <string.h>

void audio_sample_fifo_init(audio_sample_fifo_t *fifo, int16_t *storage, size_t capacity)
{
    fifo->samples = storage;
    fifo->capacity = capacity;
    audio_sample_fifo_reset(fifo);
}

void audio_sample_fifo_reset(audio_sample_fifo_t *fifo)
{
    fifo->read_index = 0u;
    fifo->write_index = 0u;
    fifo->depth = 0u;
}

size_t audio_sample_fifo_write(audio_sample_fifo_t *fifo, const int16_t *samples, size_t count)
{
    size_t accepted = 0u;
    if (fifo == NULL || fifo->samples == NULL || samples == NULL || fifo->capacity == 0u) {
        return 0u;
    }
    if (count > fifo->capacity - fifo->depth) {
        count = fifo->capacity - fifo->depth;
    }
    while (accepted < count) {
        size_t contiguous = fifo->capacity - fifo->write_index;
        if (contiguous > count - accepted) contiguous = count - accepted;
        memcpy(&fifo->samples[fifo->write_index], &samples[accepted], contiguous * sizeof(int16_t));
        fifo->write_index = (fifo->write_index + contiguous) % fifo->capacity;
        fifo->depth += contiguous;
        accepted += contiguous;
    }
    return accepted;
}

size_t audio_sample_fifo_read(audio_sample_fifo_t *fifo, int16_t *samples, size_t count)
{
    size_t produced = 0u;
    if (fifo == NULL || samples == NULL) return 0u;
    if (count > fifo->depth) count = fifo->depth;
    while (produced < count) {
        size_t contiguous = fifo->capacity - fifo->read_index;
        if (contiguous > count - produced) contiguous = count - produced;
        memcpy(&samples[produced], &fifo->samples[fifo->read_index], contiguous * sizeof(int16_t));
        fifo->read_index = (fifo->read_index + contiguous) % fifo->capacity;
        fifo->depth -= contiguous;
        produced += contiguous;
    }
    return produced;
}
