#include "audio_reblock.h"

#include <string.h>

static size_t advance(size_t index, size_t capacity)
{
    return index + 1u == capacity ? 0u : index + 1u;
}

static void push(int16_t *fifo, size_t *write, size_t *depth, size_t capacity,
                 const int16_t *samples, size_t count)
{
    for (size_t i = 0u; i < count; ++i) {
        fifo[*write] = samples[i];
        *write = advance(*write, capacity);
        (*depth)++;
    }
}

static void pop(int16_t *fifo, size_t *read, size_t *depth, size_t capacity,
                int16_t *samples, size_t count)
{
    for (size_t i = 0u; i < count; ++i) {
        samples[i] = fifo[*read];
        *read = advance(*read, capacity);
        (*depth)--;
    }
}

static bool pop_chunk(audio_reblock_t *state, int16_t *mic, int16_t *ref)
{
    if (state->mic_depth < state->chunk_size || state->ref_depth < state->chunk_size) {
        return false;
    }
    pop(state->mic_fifo, &state->mic_read, &state->mic_depth, state->capacity, mic,
        state->chunk_size);
    pop(state->ref_fifo, &state->ref_read, &state->ref_depth, state->capacity, ref,
        state->chunk_size);
    return true;
}

bool audio_reblock_init(audio_reblock_t *state, int16_t *mic_fifo, int16_t *ref_fifo,
                        int16_t *out_fifo, size_t capacity, size_t chunk_size)
{
    if (state == NULL || mic_fifo == NULL || ref_fifo == NULL || out_fifo == NULL ||
        capacity == 0u || chunk_size == 0u || chunk_size > capacity ||
        chunk_size > AUDIO_REBLOCK_MAX_CHUNK_SIZE || chunk_size > AUDIO_REBLOCK_FRAME_SAMPLES) {
        return false;
    }
    memset(state, 0, sizeof(*state));
    state->mic_fifo = mic_fifo;
    state->ref_fifo = ref_fifo;
    state->out_fifo = out_fifo;
    state->capacity = capacity;
    state->chunk_size = chunk_size;
    return true;
}

void audio_reblock_reset(audio_reblock_t *state)
{
    if (state == NULL) {
        return;
    }
    int16_t *mic_fifo = state->mic_fifo;
    int16_t *ref_fifo = state->ref_fifo;
    int16_t *out_fifo = state->out_fifo;
    size_t capacity = state->capacity;
    size_t chunk_size = state->chunk_size;
    memset(state, 0, sizeof(*state));
    state->mic_fifo = mic_fifo;
    state->ref_fifo = ref_fifo;
    state->out_fifo = out_fifo;
    state->capacity = capacity;
    state->chunk_size = chunk_size;
}

size_t audio_reblock_process_frame(audio_reblock_t *state, const int16_t *mic,
                                   const int16_t *ref, int16_t *out, size_t frame_samples,
                                   audio_reblock_process_fn process, void *context)
{
    if (state == NULL || mic == NULL || ref == NULL || out == NULL || frame_samples == 0u ||
        process == NULL || frame_samples > state->capacity ||
        frame_samples > state->capacity - state->mic_depth ||
        frame_samples > state->capacity - state->ref_depth) {
        if (state != NULL) {
            state->overflowed = true;
        }
        return 0u;
    }
    push(state->mic_fifo, &state->mic_write, &state->mic_depth, state->capacity, mic,
         frame_samples);
    push(state->ref_fifo, &state->ref_write, &state->ref_depth, state->capacity, ref,
         frame_samples);
    state->input_total += frame_samples;
    while (state->out_depth <= state->capacity - state->chunk_size &&
           pop_chunk(state, state->mic_chunk, state->ref_chunk)) {
        process(context, state->mic_chunk, state->ref_chunk, state->out_chunk);
        if (state->chunks_processed != UINT32_MAX) {
            state->chunks_processed++;
        }
        push(state->out_fifo, &state->out_write, &state->out_depth, state->capacity,
             state->out_chunk, state->chunk_size);
    }

    if (!state->primed && state->out_depth >= frame_samples) {
        state->primed = true;
    }
    if (state->out_depth < frame_samples) {
        memset(out, 0, frame_samples * sizeof(*out));
        if (state->startup_delay_frames != UINT32_MAX) {
            state->startup_delay_frames++;
        }
    } else {
        pop(state->out_fifo, &state->out_read, &state->out_depth, state->capacity, out,
            frame_samples);
    }
    state->output_total += frame_samples;
    return frame_samples;
}
