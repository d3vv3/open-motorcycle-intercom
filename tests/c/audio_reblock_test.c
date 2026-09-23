#include "audio_reblock.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    size_t chunk_size;
    size_t chunks;
} test_context_t;

static void copy_chunk(void *context, const int16_t *mic, const int16_t *ref, int16_t *out)
{
    test_context_t *test = context;
    for (size_t i = 0u; i < test->chunk_size; ++i) {
        assert(ref[i] == (int16_t)(mic[i] ^ 0x55));
        out[i] = mic[i];
    }
    test->chunks++;
}

static void test_reblocks_continuously(void)
{
    int16_t mic_fifo[1024], ref_fifo[1024], out_fifo[1024];
    int16_t mic[320], ref[320], out[320];
    audio_reblock_t state;
    test_context_t context = {.chunk_size = 256u};
    assert(audio_reblock_init(&state, mic_fifo, ref_fifo, out_fifo, 1024u, 256u));
    assert(((uintptr_t)state.mic_chunk % 16u) == 0u);
    assert(((uintptr_t)state.ref_chunk % 16u) == 0u);
    assert(((uintptr_t)state.out_chunk % 16u) == 0u);
    for (size_t frame = 0u; frame < 12u; ++frame) {
        for (size_t i = 0u; i < 320u; ++i) {
            mic[i] = (int16_t)(frame * 320u + i);
            ref[i] = (int16_t)(mic[i] ^ 0x55);
        }
        assert(audio_reblock_process_frame(&state, mic, ref, out, 320u, copy_chunk,
                                           &context) == 320u);
        for (size_t i = 0u; i < 320u; ++i) {
            int expected = frame == 0u ? 0 : (int)(frame - 1u) * 320 + (int)i;
            assert(out[i] == (int16_t)expected);
        }
    }
    assert(state.chunks_processed == 15u);
    assert(context.chunks == 15u);
    assert(state.startup_delay_frames == 1u);
    assert(state.mic_depth < state.capacity);
    assert(state.ref_depth < state.capacity);
    assert(state.out_depth < state.capacity);
}

static void test_reset_and_overflow(void)
{
    int16_t mic_fifo[16], ref_fifo[16], out_fifo[16], mic[8], ref[8], out[8];
    audio_reblock_t state;
    memset(mic, 1, sizeof(mic));
    memset(ref, 2, sizeof(ref));
    assert(audio_reblock_init(&state, mic_fifo, ref_fifo, out_fifo, 16u, 8u));
    test_context_t context = {.chunk_size = 8u};
    for (size_t i = 0u; i < 8u; ++i) {
        mic[i] = (int16_t)(100 + i);
        ref[i] = (int16_t)(mic[i] ^ 0x55);
    }
    assert(audio_reblock_process_frame(&state, mic, ref, out, 8u, copy_chunk, &context) == 8u);
    audio_reblock_reset(&state);
    assert(state.mic_depth == 0u && state.ref_depth == 0u && state.out_depth == 0u);
    assert(state.chunks_processed == 0u && !state.overflowed);
    assert(audio_reblock_process_frame(&state, mic, ref, out, 17u, copy_chunk, &context) == 0u);
    assert(state.overflowed);
    assert(!audio_reblock_init(&state, mic_fifo, ref_fifo, out_fifo, 1024u, 512u));
}

int main(void)
{
    test_reblocks_continuously();
    test_reset_and_overflow();
    puts("audio_reblock tests passed");
    return 0;
}
