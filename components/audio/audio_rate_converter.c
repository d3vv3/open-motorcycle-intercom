#include "audio_rate_converter.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#ifdef AUDIO_RATE_CONVERTER_HOST_TEST
#include "audio_rate_converter_vendor_mock.h"
#else
#include "esp_log.h"
#include "esp_timer.h"
#endif
#include "speex_resampler.h"

#define RATE_CVT_QUALITY 3
#define RATE_CVT_OUTPUT_MARGIN 16u

struct audio_rate_converter {
    uint32_t source_rate_hz;
    uint32_t destination_rate_hz;
    size_t output_capacity_frames;
    bool process_failure_logged;
    SpeexResamplerState *resampler;
    int16_t *input_scratch;
    int16_t *output_scratch;
    atomic_uint_least32_t timing_count;
    atomic_uint_least64_t timing_total_us;
    atomic_uint_least32_t timing_max_us;
    atomic_int_least32_t last_resampler_result;
};

#ifndef AUDIO_RATE_CONVERTER_HOST_TEST
static const char *TAG = "audio_rate_cvt";
#endif

static int64_t timer_get_time(void)
{
#ifdef AUDIO_RATE_CONVERTER_HOST_TEST
    return audio_rate_converter_test_timer_get_time();
#else
    return esp_timer_get_time();
#endif
}

static bool supported_rate(uint32_t rate)
{
    return rate == 16000u || rate == 32000u || rate == 44100u || rate == 48000u;
}

static size_t output_bound(uint32_t source_rate, uint32_t destination_rate,
                           size_t input_frames, size_t latency)
{
    uint64_t scaled = (uint64_t)input_frames * destination_rate;
    return (size_t)((scaled + source_rate - 1u) / source_rate) + latency +
           RATE_CVT_OUTPUT_MARGIN;
}

static void record_timing(audio_rate_converter_t *converter, int64_t elapsed_us)
{
    uint32_t elapsed = elapsed_us > UINT32_MAX ? UINT32_MAX :
                       (uint32_t)(elapsed_us > 0 ? elapsed_us : 0);
    uint_least32_t count = atomic_load_explicit(&converter->timing_count, memory_order_relaxed);
    while (count != UINT32_MAX && !atomic_compare_exchange_weak_explicit(
               &converter->timing_count, &count, count + 1u, memory_order_relaxed,
               memory_order_relaxed)) {}
    uint_least64_t total = atomic_load_explicit(&converter->timing_total_us,
                                               memory_order_relaxed);
    while (total != UINT64_MAX) {
        uint_least64_t next = UINT64_MAX - total < elapsed ? UINT64_MAX : total + elapsed;
        if (atomic_compare_exchange_weak_explicit(&converter->timing_total_us, &total,
                                                  next, memory_order_relaxed,
                                                  memory_order_relaxed)) break;
    }
    uint_least32_t maximum = atomic_load_explicit(&converter->timing_max_us,
                                                 memory_order_relaxed);
    while (elapsed > maximum && !atomic_compare_exchange_weak_explicit(
               &converter->timing_max_us, &maximum, elapsed, memory_order_relaxed,
               memory_order_relaxed)) {}
}

int audio_rate_converter_create(audio_rate_converter_t **converter,
                                uint32_t source_rate_hz,
                                uint32_t destination_rate_hz)
{
    audio_rate_converter_t *created;
    size_t output_capacity = AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES;

    if (converter == NULL) return -1;
    *converter = NULL;
    if (!supported_rate(source_rate_hz) || !supported_rate(destination_rate_hz)) return -1;

    created = calloc(1u, sizeof(*created));
    if (created == NULL) return -1;
    created->source_rate_hz = source_rate_hz;
    created->destination_rate_hz = destination_rate_hz;

    if (source_rate_hz != destination_rate_hz) {
        int error = RESAMPLER_ERR_SUCCESS;
        created->resampler = omi_speex_resampler_init(1u, source_rate_hz,
                                                     destination_rate_hz,
                                                     RATE_CVT_QUALITY, &error);
        if (created->resampler == NULL || error != RESAMPLER_ERR_SUCCESS) {
            audio_rate_converter_close(&created);
            return -1;
        }
        output_capacity = output_bound(source_rate_hz, destination_rate_hz,
                                       AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES,
                                       (size_t)omi_speex_resampler_get_output_latency(
                                           created->resampler));
    }
    created->input_scratch = calloc(AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES, sizeof(int16_t));
    created->output_scratch = calloc(output_capacity, sizeof(int16_t));
    if (created->input_scratch == NULL || created->output_scratch == NULL) {
        audio_rate_converter_close(&created);
        return -1;
    }
    created->output_capacity_frames = output_capacity;
    *converter = created;
    return 0;
}

int audio_rate_converter_get_max_output_frames(audio_rate_converter_t *converter,
                                               size_t input_frames,
                                               size_t *output_frames)
{
    if (output_frames != NULL) *output_frames = 0u;
    if (converter == NULL || output_frames == NULL ||
        input_frames > AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES) return -1;
    if (converter->resampler == NULL) {
        *output_frames = input_frames;
        return 0;
    }
    *output_frames = output_bound(converter->source_rate_hz,
                                  converter->destination_rate_hz, input_frames,
                                  (size_t)omi_speex_resampler_get_output_latency(
                                      converter->resampler));
    return *output_frames <= converter->output_capacity_frames ? 0 : -1;
}

int audio_rate_converter_process(audio_rate_converter_t *converter,
                                 const int16_t *input, size_t input_frames,
                                 int16_t *output, size_t output_capacity_frames,
                                 size_t *output_frames)
{
    if (output_frames != NULL) *output_frames = 0u;
    if (converter == NULL || output_frames == NULL ||
        (input_frames != 0u && (input == NULL || output == NULL)) ||
        input_frames > AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES) return -1;
    if (input_frames == 0u) return 0;
    if (converter->resampler == NULL) {
        if (output_capacity_frames < input_frames) return -1;
        int64_t start = timer_get_time();
        memmove(output, input, input_frames * sizeof(*output));
        *output_frames = input_frames;
        record_timing(converter, timer_get_time() - start);
        return 0;
    }

    size_t required_capacity;
    if (audio_rate_converter_get_max_output_frames(converter, input_frames,
                                                   &required_capacity) != 0 ||
        output_capacity_frames < required_capacity) return -1;
    int64_t start = timer_get_time();
    memcpy(converter->input_scratch, input, input_frames * sizeof(*input));
    spx_uint32_t consumed = (spx_uint32_t)input_frames;
    spx_uint32_t produced = (spx_uint32_t)converter->output_capacity_frames;
    int result = omi_speex_resampler_process_int(converter->resampler, 0u,
                                                 converter->input_scratch, &consumed,
                                                 converter->output_scratch, &produced);
    atomic_store_explicit(&converter->last_resampler_result, result, memory_order_relaxed);
    if (result != RESAMPLER_ERR_SUCCESS || consumed != input_frames ||
        produced > converter->output_capacity_frames || produced > output_capacity_frames) {
        (void)omi_speex_resampler_reset_mem(converter->resampler);
        record_timing(converter, timer_get_time() - start);
#ifndef AUDIO_RATE_CONVERTER_HOST_TEST
        if (!converter->process_failure_logged) {
            converter->process_failure_logged = true;
            ESP_LOGW(TAG, "resampler failed src=%lu dst=%lu rc=%d consumed=%lu/%lu produced=%lu",
                     (unsigned long)converter->source_rate_hz,
                     (unsigned long)converter->destination_rate_hz, result,
                     (unsigned long)consumed, (unsigned long)input_frames,
                     (unsigned long)produced);
        }
#endif
        return -1;
    }
    memcpy(output, converter->output_scratch, produced * sizeof(*output));
    *output_frames = produced;
    record_timing(converter, timer_get_time() - start);
    return 0;
}

void audio_rate_converter_timing_snapshot(audio_rate_converter_t *converter,
                                          audio_rate_converter_timing_t *snapshot)
{
    if (snapshot == NULL) return;
    memset(snapshot, 0, sizeof(*snapshot));
    if (converter == NULL) return;
    snapshot->count = atomic_load_explicit(&converter->timing_count, memory_order_relaxed);
    snapshot->total_us = atomic_load_explicit(&converter->timing_total_us, memory_order_relaxed);
    snapshot->max_us = atomic_load_explicit(&converter->timing_max_us, memory_order_relaxed);
    snapshot->source_rate_hz = converter->source_rate_hz;
    snapshot->destination_rate_hz = converter->destination_rate_hz;
    snapshot->last_vendor_result = atomic_load_explicit(&converter->last_resampler_result,
                                                        memory_order_relaxed);
}

void audio_rate_converter_timing_reset(audio_rate_converter_t *converter)
{
    if (converter == NULL) return;
    atomic_store_explicit(&converter->timing_count, 0u, memory_order_relaxed);
    atomic_store_explicit(&converter->timing_total_us, 0u, memory_order_relaxed);
    atomic_store_explicit(&converter->timing_max_us, 0u, memory_order_relaxed);
}

int audio_rate_converter_reset(audio_rate_converter_t *converter)
{
    if (converter == NULL) return -1;
    return converter->resampler == NULL ? 0 :
           (omi_speex_resampler_reset_mem(converter->resampler) == RESAMPLER_ERR_SUCCESS ? 0 : -1);
}

void audio_rate_converter_close(audio_rate_converter_t **converter)
{
    if (converter == NULL || *converter == NULL) return;
    audio_rate_converter_t *instance = *converter;
    *converter = NULL;
    if (instance->resampler != NULL) omi_speex_resampler_destroy(instance->resampler);
    free(instance->input_scratch);
    free(instance->output_scratch);
    free(instance);
}
