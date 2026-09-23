#include "audio_route.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static void push_in_chunks(audio_route_stream_t *stream, const int16_t *samples, size_t count)
{
    size_t offset = 0u;
    static const size_t chunks[] = {7u, 113u, 19u, 251u, 31u};
    size_t chunk = 0u;
    while (offset < count) {
        size_t length = chunks[chunk++ % (sizeof(chunks) / sizeof(chunks[0]))];
        if (length > count - offset) {
            length = count - offset;
        }
        assert(audio_route_stream_enqueue(stream, &samples[offset], length) == length);
        offset += length;
    }
}

#define TEST_STREAM(name) \
    int16_t name##_storage[AUDIO_ROUTE_INPUT_CAPACITY]; \
    audio_route_stream_t name = {0}; \
    assert(audio_route_stream_init(&name, name##_storage, AUDIO_ROUTE_INPUT_CAPACITY))

static void test_continuous_resampling(void)
{
    TEST_STREAM(stream);
    int16_t input[1600];
    int16_t output[320];
    for (size_t i = 0u; i < 1600u; ++i) {
        input[i] = (int16_t)i;
    }
    assert(audio_route_stream_configure(&stream, 8000u, 1u));
    audio_route_stream_set_active(&stream, true);
    push_in_chunks(&stream, input, 1600u);
    assert(audio_route_stream_render(&stream, output, 320u) == 320u);
    assert(output[0] == 0 && output[1] == 0);
    assert(output[319] >= output[318]);
    assert(audio_route_stream_render(&stream, output, 320u) == 320u);
    assert(output[319] > output[0]);
}

static void test_stereo_downmix_and_rates(void)
{
    TEST_STREAM(stream);
    int16_t stereo[2400];
    int16_t output[320];
    for (size_t i = 0u; i < 1200u; ++i) {
        stereo[i * 2u] = (int16_t)i;
        stereo[i * 2u + 1u] = (int16_t)(i + 2u);
    }
    assert(audio_route_stream_configure(&stream, 44100u, 2u));
    audio_route_stream_set_active(&stream, true);
    assert(audio_route_stream_enqueue(&stream, stereo, 400u) == 400u);
    assert(audio_route_stream_render(&stream, output, 100u) == 100u);
    push_in_chunks(&stream, &stereo[800u], 800u);
    assert(audio_route_stream_render(&stream, &output[100u], 220u) == 220u);
    assert(output[100] > output[99]);
    assert(audio_route_stream_configure(&stream, 48000u, 2u));
    assert(stream.step_q32 > UINT32_MAX);
    assert(!stream.active);
    assert(stream.depth == 0u);
    assert(stream.phase_q32 == 0u);
}

static void test_call_and_microphone_conversion(void)
{
    TEST_STREAM(call);
    TEST_STREAM(mic);
    int16_t input[641];
    int16_t output[320];
    for (size_t i = 0u; i < 641u; ++i) {
        input[i] = (int16_t)(i * 2u);
    }
    assert(audio_route_stream_configure(&call, 8000u, 1u));
    audio_route_stream_set_active(&call, true);
    assert(audio_route_stream_enqueue(&call, input, 641u) == 641u);
    assert(audio_route_stream_render(&call, output, 320u) == 320u);
    assert(output[1] == 1 && output[319] > output[0]);

    audio_route_mic_reset(&mic);
    audio_route_stream_set_active(&mic, true);
    assert(audio_route_mic_write(&mic, input, 641u) == 641u);
    assert(audio_route_mic_read(&mic, output, 320u, 8000u) == 320u);
    assert(output[0] == 0 && output[1] == 4);
}

static void test_policy_and_bounded_queue(void)
{
    TEST_STREAM(stream);
    int16_t input[AUDIO_ROUTE_INPUT_CAPACITY + 1u] = {0};
    assert(audio_route_stream_configure(&stream, 16000u, 1u));
    assert(audio_route_stream_enqueue(&stream, input, sizeof(input) / sizeof(input[0])) ==
           AUDIO_ROUTE_INPUT_CAPACITY);
    assert(stream.overflow_count == 1u);
    TEST_STREAM(empty);
    int16_t silence = 1;
    assert(audio_route_stream_configure(&empty, 16000u, 1u));
    audio_route_stream_set_active(&empty, true);
    assert(audio_route_stream_render(&empty, &silence, 1u) == 0u);
    assert(silence == 0 && empty.underrun_count == 1u);
    assert(audio_route_mix_sample(false, true, false, 30000, 0, -1000) == 30000);
    assert(audio_route_mix_sample(false, false, true, 0, 10000, -1000) == 10000);
    assert(audio_route_mix_sample(false, true, true, 30000, 10000, -1000) == 20000);
    assert(audio_route_mix_sample(false, true, true, 0, 10000, -1000) == 5000);
    assert(audio_route_mix_sample(false, true, true, 1, -1, -1000) == 0);
    assert(audio_route_mix_sample(false, true, true, INT16_MAX, INT16_MAX, -1000) == INT16_MAX);
    assert(audio_route_mix_sample(false, true, true, INT16_MIN, INT16_MIN, -1000) == INT16_MIN);
    assert(audio_route_mix_sample(true, true, true, 30000, 10000, -1000) == -1000);

    int16_t partial[4] = {0};
    TEST_STREAM(short_stream);
    assert(audio_route_stream_configure(&short_stream, 16000u, 1u));
    audio_route_stream_set_active(&short_stream, true);
    int16_t short_input[2] = {1000, 2000};
    assert(audio_route_stream_enqueue(&short_stream, short_input, 2u) == 2u);
    assert(audio_route_stream_render(&short_stream, partial, 4u) == 1u);
    assert(partial[0] == 1000 && partial[1] == 0 && partial[2] == 0 && partial[3] == 0);
    assert(short_stream.underrun_count == 1u);

    assert(audio_route_mix_sample(false, true, false, 1000, 0, 0) == 1000);
    assert(audio_route_mix_sample(false, false, true, 0, 2000, 0) == 2000);
    assert(audio_route_mix_sample(false, false, false, 0, 0, 0) == 0);

    int16_t base[4] = {1000, 1000, 1000, 1000};
    int16_t music[4] = {2000, 0, 0, 0};
    for (size_t i = 0u; i < 4u; ++i) {
        base[i] = audio_route_mix_sample(false, i < 4u, i < 1u, base[i], music[i], 0);
    }
    assert(base[0] == 1500 && base[1] == 1000 && base[2] == 1000 && base[3] == 1000);
}

static void test_inactive_flushes_queued_pcm_and_phase(void)
{
    TEST_STREAM(stream);
    int16_t input[32] = {0};
    int16_t output[8] = {0};
    assert(audio_route_stream_configure(&stream, 44100u, 1u));
    audio_route_stream_set_active(&stream, true);
    assert(audio_route_stream_enqueue(&stream, input, 32u) == 32u);
    assert(audio_route_stream_render(&stream, output, 8u) == 8u);
    assert(stream.depth != 0u || stream.phase_q32 != 0u);
    audio_route_stream_set_active(&stream, false);
    assert(stream.depth == 0u);
    assert(stream.phase_q32 == 0u);
    assert(!stream.active);
    audio_route_stream_set_active(&stream, true);
    assert(audio_route_stream_render(&stream, output, 1u) == 0u);
}

static void test_storage_validation(void)
{
    audio_route_stream_t stream = {0};
    int16_t storage[AUDIO_ROUTE_INPUT_CAPACITY] = {0};
    assert(!audio_route_stream_init(&stream, NULL, AUDIO_ROUTE_INPUT_CAPACITY));
    assert(!audio_route_stream_init(&stream, storage, AUDIO_ROUTE_INPUT_CAPACITY - 1u));
    assert(audio_route_stream_init(&stream, storage, AUDIO_ROUTE_INPUT_CAPACITY));
}

int main(void)
{
    test_continuous_resampling();
    test_stereo_downmix_and_rates();
    test_call_and_microphone_conversion();
    test_policy_and_bounded_queue();
    test_inactive_flushes_queued_pcm_and_phase();
    test_storage_validation();
    puts("audio route tests passed");
    return 0;
}
