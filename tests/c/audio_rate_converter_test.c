#define _POSIX_C_SOURCE 200112L
#include "audio_rate_converter.h"
#include "audio_rate_converter_vendor_mock.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int64_t mock_time_us;

int64_t audio_rate_converter_test_timer_get_time(void)
{
    return mock_time_us++;
}

static size_t convert(audio_rate_converter_t *converter, const int16_t *input,
                      size_t input_frames, int16_t *output)
{
    size_t capacity = 0u;
    size_t produced = 0u;
    assert(audio_rate_converter_get_max_output_frames(converter, input_frames,
                                                      &capacity) == 0);
    assert(audio_rate_converter_process(converter, input, input_frames, output,
                                        capacity, &produced) == 0);
    assert(produced <= capacity);
    return produced;
}

static void test_rate_pair(uint32_t source_rate, uint32_t destination_rate)
{
    audio_rate_converter_t *converter = NULL;
    int16_t input[960];
    int16_t output[3000];
    size_t total_output = 0u;
    size_t position = 0u;
    const size_t chunks[] = {1u, 37u, 113u, 7u, 159u, 83u, 320u, 240u};

    for (size_t i = 0u; i < sizeof(input) / sizeof(input[0]); ++i)
        input[i] = (int16_t)((int)(i % 251u) * 100 - 12500);
    assert(audio_rate_converter_create(&converter, source_rate, destination_rate) == 0);
    for (size_t i = 0u; position < 960u; ++i) {
        size_t count = chunks[i % (sizeof(chunks) / sizeof(chunks[0]))];
        if (count > 960u - position) count = 960u - position;
        total_output += convert(converter, &input[position], count, &output[total_output]);
        position += count;
    }
    double expected = (double)960u * destination_rate / source_rate;
    assert(total_output <= sizeof(output) / sizeof(output[0]));
    assert((double)total_output > expected * 0.90);
    assert((double)total_output < expected * 1.01);
    audio_rate_converter_timing_t timing;
    audio_rate_converter_timing_snapshot(converter, &timing);
    assert(timing.count == sizeof(chunks) / sizeof(chunks[0]));
    assert(timing.total_us != 0u && timing.max_us != 0u);
    assert(timing.source_rate_hz == source_rate && timing.destination_rate_hz == destination_rate);
    audio_rate_converter_close(&converter);
    assert(converter == NULL);
}

static size_t convert_signal(uint32_t frequency_hz, int16_t *output, size_t sample_count)
{
    audio_rate_converter_t *converter = NULL;
    int16_t input[AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES];
    size_t total_output = 0u;
    size_t input_position = 0u;
    assert(audio_rate_converter_create(&converter, 48000u, 16000u) == 0);
    while (input_position < sample_count) {
        size_t count = sample_count - input_position;
        if (count > AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES)
            count = AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES;
        for (size_t i = 0u; i < count; ++i) {
            double phase = 2.0 * 3.14159265358979323846 * frequency_hz *
                          (input_position + i) / 48000.0;
            input[i] = (int16_t)(12000.0 * sin(phase));
        }
        total_output += convert(converter, input, count, &output[total_output]);
        input_position += count;
    }
    audio_rate_converter_close(&converter);
    return total_output;
}

static double rms(const int16_t *samples, size_t begin, size_t end)
{
    double sum = 0.0;
    for (size_t i = begin; i < end; ++i) sum += (double)samples[i] * samples[i];
    return sqrt(sum / (end - begin));
}

static void test_reset_capacity_and_identity(void)
{
    audio_rate_converter_t *converter = NULL;
    audio_rate_converter_t *fresh = NULL;
    int16_t input[320];
    int16_t first[300];
    int16_t after_reset[300];
    int16_t fresh_output[300];
    size_t first_count, reset_count, fresh_count, max_output;

    for (size_t i = 0u; i < 320u; ++i) input[i] = (int16_t)(i * 61u);
    assert(audio_rate_converter_create(&converter, 48000u, 32000u) == 0);
    assert(audio_rate_converter_get_max_output_frames(converter, 320u, &max_output) == 0);
    memset(first, 0x5a, sizeof(first));
    assert(audio_rate_converter_process(converter, input, 320u, first,
                                        max_output - 1u, &first_count) != 0);
    assert(first_count == 0u && (uint8_t)first[0] == 0x5a);
    first_count = convert(converter, input, 320u, first);
    assert(audio_rate_converter_reset(converter) == 0);
    reset_count = convert(converter, input, 320u, after_reset);
    assert(audio_rate_converter_create(&fresh, 48000u, 32000u) == 0);
    fresh_count = convert(fresh, input, 320u, fresh_output);
    assert(reset_count == fresh_count);
    assert(memcmp(after_reset, fresh_output, fresh_count * sizeof(int16_t)) == 0);
    assert(first_count == reset_count);
    audio_rate_converter_close(&fresh);
    audio_rate_converter_close(&converter);

    int16_t identity[4] = {1, -2, 300, -400};
    assert(audio_rate_converter_create(&converter, 48000u, 48000u) == 0);
    size_t identity_count = 0u;
    assert(audio_rate_converter_process(converter, identity, 4u, identity, 4u,
                                        &identity_count) == 0);
    assert(identity_count == 4u && identity[0] == 1 && identity[3] == -400);
    audio_rate_converter_close(&converter);
}

static size_t convert_chunks(audio_rate_converter_t *converter, const int16_t *input,
                             const size_t *chunks, size_t chunk_count, int16_t *output)
{
    size_t input_position = 0u;
    size_t output_position = 0u;
    for (size_t i = 0u; input_position < 640u; ++i) {
        size_t count = chunks[i % chunk_count];
        if (count > 640u - input_position) count = 640u - input_position;
        output_position += convert(converter, &input[input_position], count,
                                   &output[output_position]);
        input_position += count;
    }
    return output_position;
}

static void test_chunk_continuity(void)
{
    const size_t regular_chunks[] = {320u};
    const size_t awkward_chunks[] = {3u, 71u, 1u, 199u, 46u, 320u};
    audio_rate_converter_t *regular = NULL;
    audio_rate_converter_t *awkward = NULL;
    int16_t input[640];
    int16_t regular_output[800];
    int16_t awkward_output[800];

    for (size_t i = 0u; i < 640u; ++i)
        input[i] = (int16_t)((int32_t)((i * 173u) % 24001u) - 12000);
    assert(audio_rate_converter_create(&regular, 48000u, 32000u) == 0);
    assert(audio_rate_converter_create(&awkward, 48000u, 32000u) == 0);
    size_t regular_count = convert_chunks(regular, input, regular_chunks, 1u, regular_output);
    size_t awkward_count = convert_chunks(awkward, input, awkward_chunks,
                                          sizeof(awkward_chunks) / sizeof(awkward_chunks[0]),
                                          awkward_output);
    assert(regular_count == awkward_count);
    assert(memcmp(regular_output, awkward_output,
                  regular_count * sizeof(regular_output[0])) == 0);
    audio_rate_converter_close(&regular);
    audio_rate_converter_close(&awkward);
}

static void test_48k_alias_rejection(void)
{
    int16_t passband[4000];
    int16_t alias[4000];
    size_t passband_count = convert_signal(1000u, passband, 12000u);
    size_t alias_count = convert_signal(12000u, alias, 12000u);
    size_t usable = passband_count < alias_count ? passband_count : alias_count;
    assert(usable > 3000u);
    assert(rms(alias, 400u, usable - 100u) < rms(passband, 400u, usable - 100u) * 0.08);
}

int main(void)
{
    test_rate_pair(48000u, 16000u);
    test_rate_pair(16000u, 48000u);
    test_rate_pair(48000u, 32000u);
    test_rate_pair(32000u, 48000u);
    test_rate_pair(44100u, 48000u);
    test_reset_capacity_and_identity();
    test_chunk_continuity();
    test_48k_alias_rejection();
    puts("audio rate converter tests passed");
    return 0;
}
