#include "audio_sample_fifo.h"
#include "audio_route.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_voice_frame_buffering(void)
{
    int16_t storage[1024], input[960], output[960];
    audio_sample_fifo_t fifo;
    audio_sample_fifo_init(&fifo, storage, 1024u);
    for (size_t loop = 0u; loop < 8u; ++loop) {
        for (size_t i = 0u; i < 960u; ++i) input[i] = (int16_t)(loop * 960u + i);
        assert(audio_sample_fifo_write(&fifo, input, 317u) == 317u);
        assert(audio_sample_fifo_write(&fifo, &input[317], 643u) == 643u);
        assert(audio_sample_fifo_read(&fifo, output, 960u) == 960u);
        assert(memcmp(input, output, sizeof(input)) == 0);
    }
}

static void test_irregular_music_chunks_and_reset(void)
{
    int16_t storage[1024], chunk[300], output[1024];
    static const size_t produced[] = {147u, 160u, 153u, 149u, 161u, 150u};
    audio_sample_fifo_t fifo;
    audio_sample_fifo_init(&fifo, storage, 1024u);
    uint32_t next = 0u;
    for (size_t c = 0u; c < sizeof(produced) / sizeof(produced[0]); ++c) {
        for (size_t i = 0u; i < produced[c]; ++i) chunk[i] = (int16_t)(uint16_t)next++;
        assert(audio_sample_fifo_write(&fifo, chunk, produced[c]) == produced[c]);
    }
    assert(audio_sample_fifo_read(&fifo, output, (size_t)next) == (size_t)next);
    for (uint32_t i = 0u; i < next; ++i) assert((uint16_t)output[i] == (uint16_t)i);
    assert(fifo.depth == 0u);
    audio_sample_fifo_reset(&fifo);
    assert(audio_sample_fifo_read(&fifo, output, 1u) == 0u);
}

static void test_bounds_underrun_and_mix_policy(void)
{
    int16_t storage[8], input[12] = {0}, output[8] = {0};
    audio_sample_fifo_t fifo;
    audio_sample_fifo_init(&fifo, storage, 8u);
    assert(audio_sample_fifo_write(&fifo, input, 12u) == 8u);
    assert(audio_sample_fifo_read(&fifo, output, 12u) == 8u);
    assert(audio_sample_fifo_read(&fifo, output, 1u) == 0u);
    assert(audio_route_mix_sample(true, true, true, 30000, 20000, -8000) == -8000);
    assert(audio_route_mix_sample(false, false, true, 0, 1234, 0) == 1234);
    assert(audio_route_mix_sample(false, true, true, 1000, 2000, 0) == 1500);
    audio_sample_fifo_reset(&fifo);
    assert(fifo.depth == 0u);
}

int main(void)
{
    test_voice_frame_buffering();
    test_irregular_music_chunks_and_reset();
    test_bounds_underrun_and_mix_policy();
    puts("audio sample FIFO tests passed");
    return 0;
}
