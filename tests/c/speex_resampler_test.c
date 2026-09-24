#include "speex_resampler.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define INPUT_FRAMES 3200u
#define OUTPUT_CAPACITY 11000u

static spx_uint32_t convert_chunks(spx_uint32_t source_rate, spx_uint32_t destination_rate,
                                   const spx_int16_t *input, const spx_uint32_t *chunks,
                                   size_t chunk_count, spx_int16_t *output)
{
    int error = RESAMPLER_ERR_SUCCESS;
    SpeexResamplerState *state = speex_resampler_init(1u, source_rate, destination_rate,
                                                      3, &error);
    assert(state != NULL && error == RESAMPLER_ERR_SUCCESS);

    spx_uint32_t consumed = 0u;
    spx_uint32_t produced = 0u;
    for (size_t i = 0u; consumed < INPUT_FRAMES; ++i) {
        spx_uint32_t in_len = chunks[i % chunk_count];
        if (in_len > INPUT_FRAMES - consumed) in_len = INPUT_FRAMES - consumed;
        spx_uint32_t out_len = OUTPUT_CAPACITY - produced;
        spx_uint32_t requested = in_len;
        assert(speex_resampler_process_int(state, 0u, input + consumed, &in_len,
                                           output + produced, &out_len) == RESAMPLER_ERR_SUCCESS);
        assert(in_len == requested);
        assert(out_len <= OUTPUT_CAPACITY - produced);
        consumed += in_len;
        produced += out_len;
    }
    assert(consumed == INPUT_FRAMES);
    speex_resampler_destroy(state);
    return produced;
}

static void test_rate_pair(spx_uint32_t source_rate, spx_uint32_t destination_rate)
{
    static const spx_uint32_t regular[] = {320u};
    static const spx_uint32_t irregular[] = {1u, 71u, 3u, 199u, 46u, 320u, 13u};
    spx_int16_t input[INPUT_FRAMES];
    spx_int16_t regular_output[OUTPUT_CAPACITY];
    spx_int16_t irregular_output[OUTPUT_CAPACITY];

    for (size_t i = 0u; i < INPUT_FRAMES; ++i)
        input[i] = (spx_int16_t)(12000.0 * sin(2.0 * 3.14159265358979323846 *
                                                1000.0 * i / source_rate));

    spx_uint32_t regular_count = convert_chunks(source_rate, destination_rate, input,
                                                 regular, 1u, regular_output);
    spx_uint32_t irregular_count = convert_chunks(source_rate, destination_rate, input,
                                                   irregular, sizeof(irregular) / sizeof(irregular[0]),
                                                   irregular_output);
    assert(regular_count == irregular_count);
    assert(memcmp(regular_output, irregular_output,
                  regular_count * sizeof(regular_output[0])) == 0);

    spx_uint32_t expected = (spx_uint32_t)((uint64_t)INPUT_FRAMES * destination_rate /
                                            source_rate);
    assert(regular_count >= expected - 100u && regular_count <= expected + 1u);

    double sum_squares = 0.0;
    size_t steady_count = 0u;
    for (spx_uint32_t i = 200u; i < regular_count; ++i) {
        double sample = regular_output[i];
        assert(sample >= -14000.0 && sample <= 14000.0);
        sum_squares += sample * sample;
        ++steady_count;
    }
    assert(steady_count > 800u);
    double rms = sqrt(sum_squares / steady_count);
    assert(rms > 7000.0 && rms < 10000.0);
}

int main(void)
{
    test_rate_pair(44100u, 48000u);
    test_rate_pair(48000u, 16000u);
    test_rate_pair(16000u, 48000u);
    puts("speex resampler tests passed");
    return 0;
}
