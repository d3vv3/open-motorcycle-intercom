#include "audio_capture_fifo.h"

#include <string.h>

bool audio_capture_fifo_init(audio_capture_fifo_t *fifo, int16_t *storage, size_t capacity)
{
    if (fifo == NULL || storage == NULL || capacity == 0u) {
        return false;
    }
    memset(fifo, 0, sizeof(*fifo));
    fifo->samples = storage;
    fifo->capacity = capacity;
    return true;
}

void audio_capture_fifo_reset(audio_capture_fifo_t *fifo)
{
    if (fifo == NULL) {
        return;
    }
    int16_t *samples = fifo->samples;
    size_t capacity = fifo->capacity;
    memset(fifo, 0, sizeof(*fifo));
    fifo->samples = samples;
    fifo->capacity = capacity;
}

bool audio_capture_fifo_push(audio_capture_fifo_t *fifo, const int16_t *samples, size_t count)
{
    if (fifo == NULL || fifo->samples == NULL || fifo->depth > fifo->capacity ||
        (count != 0u && samples == NULL) || count > fifo->capacity - fifo->depth) {
        return false;
    }
    for (size_t i = 0u; i < count; ++i) {
        fifo->samples[fifo->write_index] = samples[i];
        fifo->write_index = (fifo->write_index + 1u) % fifo->capacity;
    }
    fifo->depth += count;
    fifo->samples_written += count;
    return true;
}

bool audio_capture_fifo_pop_frame(audio_capture_fifo_t *fifo, int16_t *frame, size_t frame_samples)
{
    if (fifo == NULL || fifo->samples == NULL || frame == NULL || frame_samples == 0u ||
        frame_samples > fifo->capacity) {
        return false;
    }
    if (fifo->depth < frame_samples) {
        memset(frame, 0, frame_samples * sizeof(*frame));
        if (fifo->startup_silence_frames != UINT32_MAX) {
            fifo->startup_silence_frames++;
        }
        return true;
    }
    for (size_t i = 0u; i < frame_samples; ++i) {
        frame[i] = fifo->samples[fifo->read_index];
        fifo->read_index = (fifo->read_index + 1u) % fifo->capacity;
    }
    fifo->depth -= frame_samples;
    fifo->samples_read += frame_samples;
    return true;
}
