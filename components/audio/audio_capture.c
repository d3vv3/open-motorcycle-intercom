/**
 * @file audio_capture.c
 * @brief Capture task: codec read, DSP chain, VOX, encode, TX handoff.
 */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_internal.h"
#include "audio_opus_stage_profile.h"
#include "cpu_profile.h"
#if defined(AUDIO_LC3_BENCH)
#include "lc3_bench.h"
#endif
#if defined(AUDIO_ESP_LC3_BENCH)
#include "esp_lc3_bench.h"
#endif
static const char *TAG = "audio";

#define AUDIO_CAPTURE_ERROR_BACKOFF_MS 20
#define AUDIO_CAPTURE_CONVERTER_RETRY_MS 1000

typedef enum { CAPTURE_READ_TIMING, CAPTURE_CONVERT_TIMING, CAPTURE_AEC_TIMING,
               CAPTURE_LOOP_TIMING, CAPTURE_TIMING_COUNT } capture_timing_t;

typedef struct {
    uint32_t count;
    uint64_t time_sum_us;
    uint32_t time_max_us;
} encode_size_profile_t;

typedef struct {
    encode_size_profile_t buckets[3];
    uint32_t successful_encodes;
} encode_profile_t;

#define ENCODE_PROFILE_LOG_INTERVAL 250u

static uint64_t *const capture_timing_sums[CAPTURE_TIMING_COUNT] = {
    &g_audio.capture_read_us_sum, &g_audio.capture_convert_us_sum,
    &g_audio.capture_aec_us_sum, &g_audio.capture_loop_us_sum,
};

static uint32_t *const capture_timing_counts[CAPTURE_TIMING_COUNT] = {
    &g_audio.stats.capture_read_count, &g_audio.stats.capture_convert_count,
    &g_audio.stats.capture_aec_count, &g_audio.stats.capture_loop_count,
};

static uint32_t *const capture_timing_avgs[CAPTURE_TIMING_COUNT] = {
    &g_audio.stats.capture_read_us_avg, &g_audio.stats.capture_convert_us_avg,
    &g_audio.stats.capture_aec_us_avg, &g_audio.stats.capture_loop_us_avg,
};

static uint32_t *const capture_timing_maxes[CAPTURE_TIMING_COUNT] = {
    &g_audio.stats.capture_read_us_max, &g_audio.stats.capture_convert_us_max,
    &g_audio.stats.capture_aec_us_max, &g_audio.stats.capture_loop_us_max,
};

static void record_capture_timing(capture_timing_t timing, int64_t elapsed_us)
{
    uint32_t elapsed = elapsed_us > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us;
    AUDIO_STATS_LOCK();
    if (*capture_timing_counts[timing] != UINT32_MAX) (*capture_timing_counts[timing])++;
    uint64_t *sum = capture_timing_sums[timing];
    *sum = UINT64_MAX - *sum < elapsed ? UINT64_MAX : *sum + elapsed;
    uint32_t count = *capture_timing_counts[timing];
    if (count != 0u) *capture_timing_avgs[timing] = (uint32_t)(*sum / count);
    if (elapsed > *capture_timing_maxes[timing]) *capture_timing_maxes[timing] = elapsed;
    AUDIO_STATS_UNLOCK();
}

static void hpf_init(audio_hpf_state_t *state, float cutoff_hz, float sample_rate)
{
    memset(state, 0, sizeof(*state));
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float alpha = sinf(omega) / (2.0f * 0.7071f);
    float cosine = cosf(omega);
    float a0 = 1.0f + alpha;
    state->b0 = (1.0f + cosine) / (2.0f * a0);
    state->b1 = -(1.0f + cosine) / a0;
    state->b2 = state->b0;
    state->a1 = (-2.0f * cosine) / a0;
    state->a2 = (1.0f - alpha) / a0;
}

static void hpf_process(audio_hpf_state_t *state, int16_t *samples, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        float input = (float)samples[i] / 32768.0f;
        float output = state->b0 * input + state->b1 * state->x1 + state->b2 * state->x2 -
                       state->a1 * state->y1 - state->a2 * state->y2;
        state->x2 = state->x1;
        state->x1 = input;
        state->y2 = state->y1;
        state->y1 = output;
        float scaled = output * 32768.0f;
        if (scaled > INT16_MAX) {
            scaled = INT16_MAX;
        } else if (scaled < INT16_MIN) {
            scaled = INT16_MIN;
        }
        samples[i] = (int16_t)scaled;
    }
}

void audio_capture_init_dsp(void)
{
    hpf_init(&g_audio.hpf, g_audio.config.hpf_cutoff_hz, g_audio.config.sample_rate);
    vox_init(&g_audio.vox, &g_audio.config.vox_config);
    voice_cleanup_init(&g_audio.voice_cleanup);
}

static void apply_voice_cleanup(void)
{
#if AUDIO_ENABLE_AEC_NS
    if (g_audio.config.mode == AUDIO_MODE_MESH) {
        int64_t cleanup_span = cpu_profile_span_begin();
        int16_t far_reference[AUDIO_FRAME_SAMPLES];
        portENTER_CRITICAL(&g_audio_far_ref_lock);
        memcpy(far_reference, g_audio.far_ref_frame, sizeof(far_reference));
        portEXIT_CRITICAL(&g_audio_far_ref_lock);
        /* TODO: Calibrate the physical playback-to-microphone correlation delay. */
        int64_t aec_start_us = esp_timer_get_time();
        bool aec_active = audio_aec_process(&g_audio.aec, g_audio.pcm_input, far_reference,
                                            AUDIO_FRAME_SAMPLES);
        if (!aec_active) {
            voice_cleanup_process(&g_audio.voice_cleanup, g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
        }
        record_capture_timing(CAPTURE_AEC_TIMING, esp_timer_get_time() - aec_start_us);
        AUDIO_STATS_LOCK();
        g_audio.stats.aec_chunks_processed = g_audio.aec.reblock.chunks_processed;
        g_audio.stats.aec_startup_delay_frames = g_audio.aec.reblock.startup_delay_frames;
        g_audio.stats.aec_startup_fallback_frames = g_audio.stats.aec_startup_delay_frames;
        AUDIO_STATS_UNLOCK();
        cpu_profile_span_end(CPU_PROFILE_SPAN_CAP_CLEANUP, cleanup_span);
    }
#endif
}

static void capture_peak_abs_update(void)
{
    int32_t peak = 0;
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        int32_t sample = g_audio.pcm_input[i];
        /* The widened value makes abs(INT16_MIN) safe. */
        int32_t absolute = sample < 0 ? -sample : sample;
        if (absolute > peak) {
            peak = absolute;
        }
    }
    AUDIO_STATS_LOCK();
    if ((uint16_t)peak > g_audio.stats.capture_peak_abs) {
        g_audio.stats.capture_peak_abs = (uint16_t)peak;
    }
    AUDIO_STATS_UNLOCK();
}

static bool detect_voice_activity(void)
{
    bool was_active = g_audio.vox.active;
    bool vox_active = vox_process(&g_audio.vox, g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
    portENTER_CRITICAL(&g_audio_task_lock);
    audio_activity_cb_t activity_callback = g_audio.activity_callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (vox_active != was_active && activity_callback != NULL) {
        activity_callback(vox_active);
    }
    AUDIO_STATS_LOCK();
    g_audio.stats.vox_active = vox_active;
    g_audio.stats.vox_activations = g_audio.vox.activation_count;
    AUDIO_STATS_UNLOCK();
    return vox_active;
}

static void record_encode_profile(encode_profile_t *profile, int encoded, int64_t encode_time_us)
{
    if (profile->successful_encodes == UINT32_MAX) return;
    size_t bucket = encoded <= 2 ? 0u : (encoded <= 16 ? 1u : 2u);
    encode_size_profile_t *stats = &profile->buckets[bucket];
    uint32_t elapsed = encode_time_us > UINT32_MAX ? UINT32_MAX : (uint32_t)encode_time_us;
    if (stats->count != UINT32_MAX) {
        stats->count++;
    }
    stats->time_sum_us = UINT64_MAX - stats->time_sum_us < elapsed
                             ? UINT64_MAX
                             : stats->time_sum_us + elapsed;
    if (elapsed > stats->time_max_us) {
        stats->time_max_us = elapsed;
    }
    profile->successful_encodes++;
    if (profile->successful_encodes % ENCODE_PROFILE_LOG_INTERVAL == 0u) {
        audio_opus_stage_snapshot_t stages = audio_opus_stage_profile_snapshot();
        ESP_LOGI(TAG, "Opus encode profile (%lu successful): <=2B count=%lu avg=%llu us max=%lu us, "
                      "3-16B count=%lu avg=%llu us max=%lu us, >16B count=%lu avg=%llu us max=%lu us",
                 (unsigned long)profile->successful_encodes,
                 (unsigned long)profile->buckets[0].count,
                 (unsigned long long)(profile->buckets[0].count == 0u ? 0u :
                     profile->buckets[0].time_sum_us / profile->buckets[0].count),
                 (unsigned long)profile->buckets[0].time_max_us,
                 (unsigned long)profile->buckets[1].count,
                 (unsigned long long)(profile->buckets[1].count == 0u ? 0u :
                     profile->buckets[1].time_sum_us / profile->buckets[1].count),
                 (unsigned long)profile->buckets[1].time_max_us,
                 (unsigned long)profile->buckets[2].count,
                 (unsigned long long)(profile->buckets[2].count == 0u ? 0u :
                     profile->buckets[2].time_sum_us / profile->buckets[2].count),
                 (unsigned long)profile->buckets[2].time_max_us);
        ESP_LOGI(TAG, "Opus stages: SILK count=%lu avg=%llu us max=%lu us, "
                      "CELT count=%lu avg=%llu us max=%lu us",
                 (unsigned long)stages.silk.count,
                 (unsigned long long)(stages.silk.count == 0u ? 0u :
                     stages.silk.time_sum_us / stages.silk.count),
                 (unsigned long)stages.silk.time_max_us,
                 (unsigned long)stages.celt.count,
                 (unsigned long long)(stages.celt.count == 0u ? 0u :
                     stages.celt.time_sum_us / stages.celt.count),
                 (unsigned long)stages.celt.time_max_us);
    }
}

static int encode_frame(const int16_t *input, int64_t *encode_time_sum, uint32_t *encoded_frames,
                        encode_profile_t *profile)
{
    int64_t encode_start = esp_timer_get_time();
#if defined(AUDIO_S31_LC3_WIRE)
    bool lc3_wire = g_audio.config.mode == AUDIO_MODE_MESH;
    int encoded = lc3_wire
                      ? esp_lc3_codec_encode20(g_audio.lc3_encoder, input, g_audio.opus_buffer)
                      : opus_encode(g_audio.opus_encoder, input, AUDIO_FRAME_SAMPLES,
                                    g_audio.opus_buffer, sizeof(g_audio.opus_buffer));
#else
    int encoded = opus_encode(g_audio.opus_encoder, input, AUDIO_FRAME_SAMPLES, g_audio.opus_buffer,
                              sizeof(g_audio.opus_buffer));
#endif
    int64_t encode_time = esp_timer_get_time() - encode_start;
    if (encoded <= 0) {
        AUDIO_STATS_LOCK();
        g_audio.stats.encode_errors++;
        AUDIO_STATS_UNLOCK();
#if defined(AUDIO_S31_LC3_WIRE)
        if (g_audio.config.mode == AUDIO_MODE_MESH) ESP_LOGW(TAG, "LC3 encode failed");
        else
#endif
        ESP_LOGW(TAG, "Opus encode failed: %s", opus_strerror(encoded));
        return encoded;
    }
#if defined(AUDIO_S31_LC3_WIRE)
    if (g_audio.config.mode != AUDIO_MODE_MESH)
#endif
    record_encode_profile(profile, encoded, encode_time);
    *encode_time_sum += encode_time;
    AUDIO_STATS_LOCK();
    g_audio.stats.frames_encoded++;
    g_audio.stats.encode_time_us_avg = (uint32_t)(*encode_time_sum / g_audio.stats.frames_encoded);
    if ((uint32_t)encode_time > g_audio.stats.encode_time_us_max) {
        g_audio.stats.encode_time_us_max = (uint32_t)encode_time;
    }
    *encoded_frames = g_audio.stats.frames_encoded;
    AUDIO_STATS_UNLOCK();
    return encoded;
}

#if defined(AUDIO_S31_LC3_WIRE)
static void skip_idle_lc3_frame(int64_t frame_start_us)
{
    AUDIO_STATS_LOCK();
    if (g_audio.stats.vox_suppressed_frames != UINT32_MAX) {
        g_audio.stats.vox_suppressed_frames++;
    }
    AUDIO_STATS_UNLOCK();
    portENTER_CRITICAL(&g_audio_task_lock);
    audio_tx_idle_cb_t callback = g_audio.tx_idle_callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (callback != NULL) {
        callback(frame_start_us);
    }
}
#endif

static void deliver_encoded_frame(int encoded, bool tx_active, int64_t frame_start_us)
{
#if defined(AUDIO_S31_LC3_WIRE)
    bool lc3_wire = g_audio.config.mode == AUDIO_MODE_MESH;
    if (lc3_wire && !tx_active) {
        skip_idle_lc3_frame(frame_start_us);
        return;
    }
    bool comfort_update = encoded > OPUS_DTX_FRAME_MAX_BYTES;
#else
    bool comfort_update = encoded > OPUS_DTX_FRAME_MAX_BYTES;
#endif
    if (g_audio.config.mode == AUDIO_MODE_MESH) {
        if (tx_active || comfort_update) {
            portENTER_CRITICAL(&g_audio_task_lock);
            audio_tx_cb_t tx_callback = g_audio.tx_callback;
            portEXIT_CRITICAL(&g_audio_task_lock);
            if (tx_callback != NULL) {
                AUDIO_STATS_LOCK();
                g_audio.stats.tx_handoff++;
                AUDIO_STATS_UNLOCK();
                tx_callback(g_audio.opus_buffer, (uint16_t)encoded, tx_active, frame_start_us);
            } else {
                AUDIO_STATS_LOCK();
                g_audio.stats.tx_no_cb++;
                AUDIO_STATS_UNLOCK();
            }
        } else {
            AUDIO_STATS_LOCK();
            g_audio.stats.tx_dtx_suppressed++;
            AUDIO_STATS_UNLOCK();
        }
        return;
    }
    audio_loopback_item_t item = {
        .length = (uint16_t)encoded,
        .timestamp_us = frame_start_us,
    };
    memcpy(item.data, g_audio.opus_buffer, (size_t)encoded);
    if (xQueueSend(g_audio.loopback_queue, &item, 0) != pdTRUE) {
        AUDIO_STATS_LOCK();
        g_audio.stats.frames_dropped++;
        AUDIO_STATS_UNLOCK();
    }
}

static void record_frame_latency(int64_t frame_start_us, int64_t *latency_sum,
                                 uint32_t encoded_frames)
{
    int64_t processing_time_us = esp_timer_get_time() - frame_start_us;
    /* NOTE: latency stats cover local processing plus this DMA estimate;
     * they do not measure mouth-to-ear latency over the radio. Two
     * descriptors bound queued speaker data to roughly one or two frames. */
    int64_t dma_latency_us = ((int64_t)I2S_DMA_BUFFER_COUNT / 2) * 20000;
    int64_t total_latency_us = processing_time_us + dma_latency_us;
    *latency_sum += total_latency_us;
    AUDIO_STATS_LOCK();
    g_audio.stats.latency_ms_avg = (uint32_t)((*latency_sum / (int64_t)encoded_frames) / 1000);
    uint32_t latency_ms = (uint32_t)(total_latency_us / 1000);
    if (latency_ms > g_audio.stats.latency_ms_max) {
        g_audio.stats.latency_ms_max = latency_ms;
    }
    AUDIO_STATS_UNLOCK();
}

static void capture_task_finish(void)
{
    portENTER_CRITICAL(&g_audio_task_lock);
    g_audio.capture_task = NULL;
    portEXIT_CRITICAL(&g_audio_task_lock);
    xSemaphoreGive(g_audio.capture_done);
    vTaskDelete(NULL);
}

static void reset_capture_fifo_with_loss(size_t failed_push_samples)
{
    size_t discarded = g_audio.capture_fifo.depth + failed_push_samples;
    AUDIO_STATS_LOCK();
    g_audio.stats.capture_fifo_discard_samples += (uint32_t)discarded;
    AUDIO_STATS_UNLOCK();
    audio_capture_fifo_reset(&g_audio.capture_fifo);
}

static bool read_codec_frame(uint32_t *consecutive_hard_errors, int64_t *last_error_log_us,
                             int64_t *converter_retry_after_us, bool *emit_silence)
{
    *emit_silence = false;
    /* beta5 may block inside this call until its internal read timeout; stop waits for that
     * read to return before it can close the record device. */
    int64_t read_start_us = esp_timer_get_time();
    int ret = esp_codec_dev_read(g_audio.record_dev, (uint8_t *)g_audio.capture_hw_frame,
                                 sizeof(g_audio.capture_hw_frame));
    record_capture_timing(CAPTURE_READ_TIMING, esp_timer_get_time() - read_start_us);
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        return false;
    }
    if (ret == ESP_CODEC_DEV_OK) {
        int64_t now_us = esp_timer_get_time();
        if (now_us < *converter_retry_after_us) {
            *emit_silence = true;
            return false;
        }
        int16_t converted[AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES];
        size_t produced;
        for (size_t offset = 0u; offset < AUDIO_HW_FRAME_SAMPLES;
             offset += AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES) {
            int64_t convert_start_us = esp_timer_get_time();
            int64_t convert_span = cpu_profile_span_begin();
            int convert_result = audio_rate_converter_process(
                g_audio.capture_rate_converter, &g_audio.capture_hw_frame[offset],
                AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES, converted,
                AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES, &produced);
            cpu_profile_span_end(CPU_PROFILE_SPAN_CAP_CONVERT, convert_span);
            bool pushed = convert_result == 0 &&
                audio_capture_fifo_push(&g_audio.capture_fifo, converted, produced);
            if (!pushed) {
                record_capture_timing(CAPTURE_CONVERT_TIMING,
                                      esp_timer_get_time() - convert_start_us);
                (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
                reset_capture_fifo_with_loss(convert_result == 0 ? produced : 0u);
                AUDIO_STATS_LOCK();
                g_audio.stats.capture_errors++;
                AUDIO_STATS_UNLOCK();
                *converter_retry_after_us = now_us +
                    (int64_t)AUDIO_CAPTURE_CONVERTER_RETRY_MS * 1000LL;
                *emit_silence = true;
                return false;
            }
            record_capture_timing(CAPTURE_CONVERT_TIMING,
                                  esp_timer_get_time() - convert_start_us);
        }
        uint32_t startup_silence_before = g_audio.capture_fifo.startup_silence_frames;
        if (!audio_capture_fifo_pop_frame(&g_audio.capture_fifo, g_audio.pcm_input,
                                          AUDIO_FRAME_SAMPLES)) {
            (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
            reset_capture_fifo_with_loss(0u);
            AUDIO_STATS_LOCK();
            g_audio.stats.capture_errors++;
            AUDIO_STATS_UNLOCK();
            return false;
        }
        AUDIO_STATS_LOCK();
        if (g_audio.capture_fifo.startup_silence_frames != startup_silence_before) {
            g_audio.stats.capture_short_reads++;
        } else {
            g_audio.stats.capture_frames_ok++;
        }
        AUDIO_STATS_UNLOCK();
        *consecutive_hard_errors = 0;
        return true;
    }
    if (ret == ESP_CODEC_DEV_TIMEOUT) {
        (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
        reset_capture_fifo_with_loss(0u);
        AUDIO_STATS_LOCK();
        g_audio.stats.capture_timeouts++;
        AUDIO_STATS_UNLOCK();
        vTaskDelay(pdMS_TO_TICKS(5));
        return false;
    }

    AUDIO_STATS_LOCK();
    g_audio.stats.capture_errors++;
    AUDIO_STATS_UNLOCK();
    (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
    reset_capture_fifo_with_loss(0u);
    if (*consecutive_hard_errors < UINT32_MAX) {
        (*consecutive_hard_errors)++;
    }
    int64_t now_us = esp_timer_get_time();
    if (*last_error_log_us == 0 || now_us - *last_error_log_us >= 1000000LL) {
        ESP_LOGW(TAG, "Codec capture hard error %d (consecutive=%lu)", ret,
                 (unsigned long)*consecutive_hard_errors);
        *last_error_log_us = now_us;
    }
    vTaskDelay(pdMS_TO_TICKS(AUDIO_CAPTURE_ERROR_BACKOFF_MS));
    return false;
}

void audio_capture_task(void *arg)
{
#if defined(AUDIO_S31_LC3_SPLIT_CORES)
    ESP_LOGI(TAG, "LC3 split cores=1 capture entry mode=%d actual_core=%d affinity_core=%d",
             (int)g_audio.config.mode, (int)xPortGetCoreID(), (int)xTaskGetCoreID(NULL));
#endif
    (void)arg;
    static int16_t silence_frame[AUDIO_FRAME_SAMPLES];
    int64_t encode_time_sum = 0;
    encode_profile_t encode_profile = {0};
    int64_t latency_sum = 0;
    uint32_t consecutive_hard_errors = 0;
    int64_t last_error_log_us = 0;
    int64_t converter_retry_after_us = 0;
    uint32_t frame_loops = 0;
    bool stack_logged = false;
#if defined(AUDIO_S31_LC3_WIRE)
    bool last_tx_active = g_audio.config.force_tx_always;
#endif

    audio_opus_stage_profile_reset();
#if defined(AUDIO_LC3_BENCH)
    lc3_bench_reset();
#endif
#if defined(AUDIO_ESP_LC3_BENCH) && !defined(AUDIO_S31_LC3_WIRE)
    esp_lc3_bench_init();
#endif
#if defined(AUDIO_S31_LC3_SPLIT_CORES)
    ESP_LOGI(TAG, "LC3 split cores=1 capture before encoder reset mode=%d actual_core=%d affinity_core=%d",
             (int)g_audio.config.mode, (int)xPortGetCoreID(), (int)xTaskGetCoreID(NULL));
#endif
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_RESET_STATE);
#if defined(AUDIO_S31_LC3_WIRE)
    int lc3_reset = g_audio.config.mode == AUDIO_MODE_MESH
                        ? esp_lc3_codec_reset(g_audio.lc3_encoder) : 0;
#endif
#if defined(AUDIO_S31_LC3_SPLIT_CORES)
    ESP_LOGI(TAG, "LC3 split cores=1 capture after encoder reset mode=%d actual_core=%d affinity_core=%d",
             (int)g_audio.config.mode, (int)xPortGetCoreID(), (int)xTaskGetCoreID(NULL));
#endif
#if defined(AUDIO_S31_LC3_WIRE)
    if (lc3_reset != 0) {
        atomic_store_explicit(&g_audio.capture_ready, false, memory_order_release);
        xSemaphoreGive(g_audio.capture_started);
        capture_task_finish();
        return;
    }
#endif
    bool aec_enabled = audio_aec_init(
        &g_audio.aec, AUDIO_ENABLE_ESP_SR_AEC && g_audio.config.mode == AUDIO_MODE_MESH);
    if (!AUDIO_ENABLE_ESP_SR_AEC) {
        ESP_LOGI(TAG, "ESP-SR AEC trial disabled (122 ms per 20 ms capture frame on S31); "
                       "noise-only voice cleanup active, HFP call echo unqualified");
    }
    AUDIO_STATS_LOCK();
    g_audio.stats.aec_available = aec_enabled;
    g_audio.stats.aec_active = aec_enabled;
    g_audio.stats.aec_chunk_size = aec_enabled ? (uint16_t)g_audio.aec.reblock.chunk_size : 0u;
    AUDIO_STATS_UNLOCK();
    atomic_store_explicit(&g_audio.capture_ready, true, memory_order_release);
    xSemaphoreGive(g_audio.capture_started);
    ESP_LOGI(TAG, "Capture task started on core %d", xPortGetCoreID());

    while (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        /* The codec can return from a buffered read immediately. Reserve one tick
         * for the idle task so Core 1 still services the task watchdog. */
        vTaskDelay(1);
        int64_t frame_start_us = esp_timer_get_time();
        AUDIO_STATS_LOCK();
        g_audio.stats.task_loops++;
        AUDIO_STATS_UNLOCK();
        if (!stack_logged && ++frame_loops >= 50u) {
            ESP_LOGI(TAG, "Capture task stack high water: %u bytes",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
            stack_logged = true;
        }

        bool emit_silence = false;
        if (!read_codec_frame(&consecutive_hard_errors, &last_error_log_us,
                              &converter_retry_after_us, &emit_silence)) {
            if (emit_silence && atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
#if defined(AUDIO_S31_LC3_WIRE)
                if (g_audio.config.mode == AUDIO_MODE_MESH && !last_tx_active) {
                    skip_idle_lc3_frame(frame_start_us);
                } else
#endif
                {
                    uint32_t encoded_frames = 0;
                    int encoded = encode_frame(silence_frame, &encode_time_sum, &encoded_frames,
                                               &encode_profile);
                    if (encoded > 0) {
                        deliver_encoded_frame(encoded, true, frame_start_us);
                        record_frame_latency(frame_start_us, &latency_sum, encoded_frames);
                    }
                }
            }
            record_capture_timing(CAPTURE_LOOP_TIMING, esp_timer_get_time() - frame_start_us);
            continue;
        }
        if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
            break;
        }
        if (g_audio.config.enable_hpf) {
            int64_t hpf_span = cpu_profile_span_begin();
            hpf_process(&g_audio.hpf, g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
            cpu_profile_span_end(CPU_PROFILE_SPAN_CAP_HPF, hpf_span);
        }
        apply_voice_cleanup();
        audio_route_capture_frame(g_audio.pcm_input, AUDIO_FRAME_SAMPLES);
        capture_peak_abs_update();

#if defined(AUDIO_LC3_BENCH)
#if !defined(AUDIO_S31_LC3_WIRE)
        lc3_bench_capture_frame(g_audio.pcm_input);
#endif
#endif
#if defined(AUDIO_ESP_LC3_BENCH)
#if !defined(AUDIO_S31_LC3_WIRE)
        esp_lc3_bench_capture_frame(g_audio.pcm_input);
#endif
#endif

        bool vox_active = detect_voice_activity();
        bool tx_active = vox_active || g_audio.config.force_tx_always;
#if defined(AUDIO_S31_LC3_WIRE)
        last_tx_active = tx_active;
        if (g_audio.config.mode == AUDIO_MODE_MESH && !tx_active) {
            skip_idle_lc3_frame(frame_start_us);
            record_capture_timing(CAPTURE_LOOP_TIMING, esp_timer_get_time() - frame_start_us);
            continue;
        }
#endif
        const int16_t *encode_input = tx_active ? g_audio.pcm_input : silence_frame;
        uint32_t encoded_frames = 0;
        int encoded = encode_frame(encode_input, &encode_time_sum, &encoded_frames,
                                   &encode_profile);
        if (encoded <= 0) {
            record_capture_timing(CAPTURE_LOOP_TIMING, esp_timer_get_time() - frame_start_us);
            continue;
        }
        deliver_encoded_frame(encoded, tx_active, frame_start_us);
        record_frame_latency(frame_start_us, &latency_sum, encoded_frames);
        record_capture_timing(CAPTURE_LOOP_TIMING, esp_timer_get_time() - frame_start_us);
    }

    ESP_LOGI(TAG, "Capture task stopped");
#if defined(AUDIO_LC3_BENCH)
    lc3_bench_reset();
#endif
#if defined(AUDIO_ESP_LC3_BENCH) && !defined(AUDIO_S31_LC3_WIRE)
    esp_lc3_bench_deinit();
#endif
    audio_aec_deinit(&g_audio.aec);
    AUDIO_STATS_LOCK();
    g_audio.stats.aec_available = false;
    g_audio.stats.aec_active = false;
    g_audio.stats.aec_chunk_size = 0u;
    AUDIO_STATS_UNLOCK();
    capture_task_finish();
}
