#include "audio_capture_fifo.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void test_variable_output_and_frame_order(void)
{
    int16_t storage[640];
    int16_t output[320];
    int16_t input[320];
    audio_capture_fifo_t fifo;
    assert(audio_capture_fifo_init(&fifo, storage, 640u));

    for (size_t i = 0u; i < 210u; ++i) input[i] = (int16_t)i;
    assert(audio_capture_fifo_push(&fifo, input, 210u));
    assert(audio_capture_fifo_pop_frame(&fifo, output, 320u));
    for (size_t i = 0u; i < 320u; ++i) assert(output[i] == 0);
    assert(fifo.depth == 210u && fifo.samples_read == 0u);

    for (size_t i = 210u; i < 320u; ++i) input[i] = (int16_t)i;
    assert(audio_capture_fifo_push(&fifo, input + 210u, 110u));
    assert(audio_capture_fifo_pop_frame(&fifo, output, 320u));
    for (size_t i = 0u; i < 320u; ++i) assert(output[i] == (int16_t)i);

    for (size_t i = 0u; i < 320u; ++i) input[i] = (int16_t)(320u + i);
    assert(audio_capture_fifo_push(&fifo, input, 317u));
    assert(audio_capture_fifo_push(&fifo, input + 317u, 3u));
    assert(audio_capture_fifo_pop_frame(&fifo, output, 320u));
    for (size_t i = 0u; i < 320u; ++i) assert(output[i] == (int16_t)(320u + i));
    assert(fifo.samples_written == 640u && fifo.samples_read == 640u);
    assert(fifo.startup_silence_frames == 1u && fifo.depth == 0u);
}

static void test_capacity_and_reset(void)
{
    int16_t storage[8];
    int16_t input[9] = {0};
    int16_t output[8];
    audio_capture_fifo_t fifo;
    assert(audio_capture_fifo_init(&fifo, storage, 8u));
    assert(!audio_capture_fifo_init(&fifo, storage, 0u));
    assert(audio_capture_fifo_init(&fifo, storage, 8u));
    assert(!audio_capture_fifo_push(&fifo, input, 9u));
    assert(fifo.depth == 0u && fifo.samples_written == 0u);
    assert(audio_capture_fifo_push(&fifo, input, 8u));
    assert(!audio_capture_fifo_push(&fifo, input, 1u));
    assert(!audio_capture_fifo_pop_frame(&fifo, output, 9u));
    audio_capture_fifo_reset(&fifo);
    assert(fifo.samples == storage && fifo.capacity == 8u && fifo.depth == 0u);
    assert(fifo.samples_written == 0u && fifo.samples_read == 0u);
    assert(audio_capture_fifo_pop_frame(&fifo, output, 8u));
    for (size_t i = 0u; i < 8u; ++i) assert(output[i] == 0);
}

int main(void)
{
    test_variable_output_and_frame_order();
    test_capacity_and_reset();
    puts("audio capture FIFO tests passed");
    return 0;
}
