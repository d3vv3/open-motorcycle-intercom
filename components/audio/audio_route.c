#include "audio_route.h"

#include <limits.h>
#include <string.h>

#define ROUTE_PHASE_ONE UINT64_C(4294967296)

static bool supported_step(uint32_t rate, uint32_t output_rate, uint64_t *step)
{
    if (rate == 0u || output_rate == 0u) {
        return false;
    }
    *step = ((uint64_t)rate * ROUTE_PHASE_ONE) / output_rate;
    return *step != 0u;
}

static size_t ring_index(const audio_route_stream_t *stream, size_t offset)
{
    return (stream->read_index + offset) % stream->capacity;
}

bool audio_route_stream_init(audio_route_stream_t *stream, int16_t *storage, size_t capacity)
{
    if (stream == NULL || storage == NULL || capacity < AUDIO_ROUTE_INPUT_CAPACITY) {
        return false;
    }
    memset(stream, 0, sizeof(*stream));
    stream->samples = storage;
    stream->capacity = capacity;
    return true;
}

void audio_route_stream_reset(audio_route_stream_t *stream)
{
    uint32_t rate = stream->sample_rate;
    uint8_t channels = stream->channels;
    bool configured = stream->configured;
    bool active = stream->active;
    int16_t *samples = stream->samples;
    size_t capacity = stream->capacity;
    memset(stream, 0, sizeof(*stream));
    stream->samples = samples;
    stream->capacity = capacity;
    stream->sample_rate = rate;
    stream->channels = channels;
    stream->configured = configured;
    stream->active = active;
    (void)supported_step(rate, 16000u, &stream->step_q32);
}

bool audio_route_stream_configure(audio_route_stream_t *stream, uint32_t sample_rate,
                                  uint8_t channels)
{
    bool rate_ok = sample_rate == 16000u || sample_rate == 32000u || sample_rate == 44100u ||
                   sample_rate == 48000u || sample_rate == 8000u;
    if (stream == NULL || stream->samples == NULL || stream->capacity < AUDIO_ROUTE_INPUT_CAPACITY ||
        !rate_ok || (channels != 1u && channels != 2u) ||
        !supported_step(sample_rate, 16000u, &stream->step_q32)) {
        return false;
    }
    stream->sample_rate = sample_rate;
    stream->channels = channels;
    stream->configured = true;
    audio_route_stream_reset(stream);
    stream->active = false;
    return true;
}

void audio_route_stream_set_active(audio_route_stream_t *stream, bool active)
{
    if (!active) {
        /* Suspending a route is a hard boundary: queued PCM and fractional phase
         * must not survive into a later stream or call. */
        audio_route_stream_reset(stream);
    }
    stream->active = active;
}

size_t audio_route_stream_enqueue(audio_route_stream_t *stream, const int16_t *interleaved,
                                  size_t frames)
{
    size_t accepted = 0u;
    if (!stream->configured || interleaved == NULL || stream->channels == 0u) {
        return 0u;
    }
    if (stream->samples == NULL || stream->capacity < AUDIO_ROUTE_INPUT_CAPACITY) {
        return 0u;
    }
    while (accepted < frames && stream->depth < stream->capacity) {
        const int16_t *frame = &interleaved[accepted * stream->channels];
        int32_t value = frame[0];
        if (stream->channels == 2u) {
            /* Widen before averaging so opposite full-scale channels cannot overflow. */
            value = ((int32_t)frame[0] + (int32_t)frame[1]) / 2;
        }
        stream->samples[stream->write_index] = audio_route_saturate(value);
        stream->write_index = (stream->write_index + 1u) % stream->capacity;
        stream->depth++;
        accepted++;
    }
    if (accepted != frames) {
        size_t rejected = frames - accepted;
        uint32_t add = rejected > UINT32_MAX ? UINT32_MAX : (uint32_t)rejected;
        stream->overflow_count = UINT32_MAX - stream->overflow_count < add
                                    ? UINT32_MAX
                                    : stream->overflow_count + add;
    }
    return accepted;
}

static int16_t interpolate(int16_t first, int16_t second, uint32_t fraction)
{
    int64_t value = (int64_t)first + ((int64_t)(second - first) * fraction) / ROUTE_PHASE_ONE;
    return audio_route_saturate((int32_t)value);
}

size_t audio_route_stream_render(audio_route_stream_t *stream, int16_t *output, size_t samples)
{
    size_t produced = 0u;
    size_t consumed = 0u;
    if (output == NULL || stream->samples == NULL || stream->capacity < AUDIO_ROUTE_INPUT_CAPACITY ||
        !stream->configured || !stream->active) {
        if (output != NULL) {
            memset(output, 0, samples * sizeof(*output));
        }
        return 0u;
    }
    for (size_t i = 0u; i < samples; ++i) {
        size_t offset = (size_t)(stream->phase_q32 / ROUTE_PHASE_ONE);
        if (offset + 1u >= stream->depth) {
            memset(&output[i], 0, (samples - i) * sizeof(*output));
            break;
        }
        size_t first = ring_index(stream, offset);
        size_t second = ring_index(stream, offset + 1u);
        output[i] = interpolate(stream->samples[first], stream->samples[second],
                                (uint32_t)stream->phase_q32);
        stream->phase_q32 += stream->step_q32;
        consumed = (size_t)(stream->phase_q32 / ROUTE_PHASE_ONE);
        produced++;
    }
    if (consumed > stream->depth) {
        consumed = stream->depth;
    }
    stream->read_index = (stream->read_index + consumed) % stream->capacity;
    stream->depth -= consumed;
    stream->phase_q32 -= (uint64_t)consumed * ROUTE_PHASE_ONE;
    if (produced != samples) {
        stream->underrun_count++;
    }
    return produced;
}

size_t audio_route_stream_read_raw(audio_route_stream_t *stream, int16_t *output, size_t samples)
{
    size_t count = 0u;
    if (stream == NULL || output == NULL || !stream->configured || !stream->active ||
        stream->samples == NULL || stream->capacity == 0u) return 0u;
    if (samples > stream->depth) samples = stream->depth;
    while (count < samples) {
        size_t contiguous = stream->capacity - stream->read_index;
        if (contiguous > samples - count) contiguous = samples - count;
        memcpy(&output[count], &stream->samples[stream->read_index], contiguous * sizeof(*output));
        stream->read_index = (stream->read_index + contiguous) % stream->capacity;
        stream->depth -= contiguous;
        count += contiguous;
    }
    return count;
}

void audio_route_mic_reset(audio_route_stream_t *stream)
{
    audio_route_stream_reset(stream);
    stream->configured = true;
    stream->sample_rate = 16000u;
    stream->channels = 1u;
    stream->step_q32 = ROUTE_PHASE_ONE;
}

size_t audio_route_mic_write(audio_route_stream_t *stream, const int16_t *samples, size_t count)
{
    return audio_route_stream_enqueue(stream, samples, count);
}

size_t audio_route_mic_read(audio_route_stream_t *stream, int16_t *output, size_t count,
                            uint32_t output_rate)
{
    uint64_t saved_step;
    size_t produced = 0u;
    if (output == NULL || stream->samples == NULL || stream->capacity < AUDIO_ROUTE_INPUT_CAPACITY ||
        count == 0u || (output_rate != 8000u && output_rate != 16000u) ||
        !supported_step(16000u, output_rate, &saved_step)) {
        return 0u;
    }
    stream->step_q32 = saved_step;
    for (size_t i = 0u; i < count; ++i) {
        size_t offset = (size_t)(stream->phase_q32 / ROUTE_PHASE_ONE);
        if (offset + 1u >= stream->depth) {
            break;
        }
        output[i] = interpolate(stream->samples[ring_index(stream, offset)],
                                stream->samples[ring_index(stream, offset + 1u)],
                                (uint32_t)stream->phase_q32);
        stream->phase_q32 += stream->step_q32;
        produced++;
    }
    size_t consumed = (size_t)(stream->phase_q32 / ROUTE_PHASE_ONE);
    if (consumed > stream->depth) {
        consumed = stream->depth;
    }
    stream->read_index = (stream->read_index + consumed) % stream->capacity;
    stream->depth -= consumed;
    stream->phase_q32 -= (uint64_t)consumed * ROUTE_PHASE_ONE;
    return produced;
}

int16_t audio_route_saturate(int32_t sample)
{
    if (sample > INT16_MAX) {
        return INT16_MAX;
    }
    if (sample < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)sample;
}

int16_t audio_route_mix_sample(bool call_active, bool base_present, bool music_present,
                              int16_t base, int16_t music, int16_t call)
{
    if (call_active) {
        return call;
    }
    if (base_present && music_present) {
        int32_t sum = (int32_t)base + (int32_t)music;
        /* Round halves symmetrically; present zero samples retain the shared gain. */
        int32_t rounded = sum > 0 ? (sum + 1) / 2 : (sum < 0 ? (sum - 1) / 2 : 0);
        return audio_route_saturate(rounded);
    }
    return base_present ? base : music;
}
