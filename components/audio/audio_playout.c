/**
 * @file audio_playout.c
 * @brief Playout task: decode, adaptive render, mixing, I2S writes, heartbeat.
 */

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_internal.h"
#include "cpu_profile.h"

static const char *TAG = "audio";
static uint32_t successful_remote_decodes;

typedef struct {
    uint32_t attempted;
    uint32_t successful;
    uint32_t failed;
    uint32_t nonzero_frames;
    int32_t peak_abs;
} audio_output_window_t;

#if defined(AUDIO_S31_LC3_SELFTEST)
static bool run_lc3_startup_selftest(void)
{
    static int16_t zero_pcm[AUDIO_FRAME_SAMPLES] = {0};
    static int16_t patterned_pcm[AUDIO_FRAME_SAMPLES];
    static uint8_t packet[48];
    static int16_t decoded[AUDIO_FRAME_SAMPLES];

    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        patterned_pcm[i] = ((i / 8u) & 1u) != 0u ? -10000 : 10000;
    }
    const int16_t *tests[] = {zero_pcm, patterned_pcm};
    const char *names[] = {"zero", "patterned"};
    for (size_t test = 0; test < 2u; ++test) {
        ESP_LOGI(TAG, "LC3 self-test %s encode/decode start", names[test]);
        int64_t encode_start = esp_timer_get_time();
        int encoded_bytes = esp_lc3_codec_encode20(g_audio.lc3_encoder, tests[test], packet);
        int64_t encode_us = esp_timer_get_time() - encode_start;
        int output_samples = -1;
        int64_t decode_us = 0;
        if (encoded_bytes == (int)sizeof(packet)) {
            int64_t decode_start = esp_timer_get_time();
            output_samples = esp_lc3_codec_decode20(g_audio.rx_sources[0].lc3_decoder,
                                                    packet, false, decoded);
            decode_us = esp_timer_get_time() - decode_start;
        }
        int32_t peak = 0;
        if (output_samples == AUDIO_FRAME_SAMPLES) {
            for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
                int32_t sample = decoded[i];
                int32_t magnitude = sample < 0 ? -sample : sample;
                if (magnitude > peak) peak = magnitude;
            }
        }
        ESP_LOGI(TAG, "LC3 self-test %s encode/decode done: samples=%d bytes=%d"
                      " encode_us=%" PRId64 " decode_us=%" PRId64 " peak=%" PRId32,
                 names[test], output_samples, encoded_bytes, encode_us, decode_us, peak);
        if (encoded_bytes != (int)sizeof(packet) || output_samples != AUDIO_FRAME_SAMPLES ||
            (test == 1u && peak == 0)) {
            ESP_LOGE(TAG, "LC3 self-test %s failed", names[test]);
            return false;
        }
    }

    /* Captured 48B peer diagnostic samples only; these are never transmitted. */
    static const uint8_t peer_packets[][48] = {
        {
            0x06, 0x00, 0x1a, 0x9e, 0x0c, 0x0e, 0xfa, 0xe3,
            0x6a, 0xb3, 0x2d, 0x83, 0x65, 0xf7, 0xe9, 0xfe,
            0x0e, 0x2e, 0x7b, 0xe5, 0x33, 0x51, 0x10, 0x79,
            0x00, 0x00, 0x10, 0x33, 0x22, 0x29, 0x21, 0xad,
            0xf1, 0x84, 0x20, 0x9a, 0xe1, 0xc5, 0x56, 0xba,
            0x4f, 0xd9, 0xd3, 0x59, 0x33, 0x53, 0x1e, 0x37,
        },
        {
            0x2c, 0xbc, 0x25, 0x88, 0x90, 0x21, 0x67, 0xe3,
            0xf0, 0xed, 0xd6, 0x35, 0xad, 0xc8, 0x68, 0x4f,
            0x0e, 0x66, 0x2a, 0xeb, 0x33, 0x51, 0x1a, 0x65,
            0x00, 0x00, 0x16, 0x77, 0xa5, 0xbe, 0xcd, 0x15,
            0x9e, 0xf7, 0x8d, 0x6c, 0xfd, 0x7b, 0x57, 0xfe,
            0xa8, 0x67, 0x21, 0xaa, 0x8b, 0x57, 0x26, 0x4b,
        },
    };
    const char *peer_names[] = {"peer-old", "peer-new"};
    for (size_t test = 0; test < 2u; ++test) {
        int reset = esp_lc3_codec_reset(g_audio.rx_sources[0].lc3_decoder);
        if (reset != 0) {
            ESP_LOGE(TAG, "LC3 self-test %s decoder reset failed: %d", peer_names[test], reset);
            return false;
        }
        ESP_LOGI(TAG, "LC3 self-test %s decode start", peer_names[test]);
        int64_t decode_start = esp_timer_get_time();
        int output_samples = esp_lc3_codec_decode20(g_audio.rx_sources[0].lc3_decoder,
                                                    peer_packets[test], false, decoded);
        int64_t decode_us = esp_timer_get_time() - decode_start;
        int32_t peak = 0;
        if (output_samples == AUDIO_FRAME_SAMPLES) {
            for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
                int32_t sample = decoded[i];
                int32_t magnitude = sample < 0 ? -sample : sample;
                if (magnitude > peak) peak = magnitude;
            }
        }
        ESP_LOGI(TAG, "LC3 self-test %s decode done: samples=%d decode_us=%" PRId64
                      " peak=%" PRId32,
                 peer_names[test], output_samples, decode_us, peak);
        if (output_samples != AUDIO_FRAME_SAMPLES) {
            ESP_LOGE(TAG, "LC3 self-test %s failed", peer_names[test]);
            return false;
        }
    }

    int decoder_reset = esp_lc3_codec_reset(g_audio.rx_sources[0].lc3_decoder);
    int encoder_reset = esp_lc3_codec_reset(g_audio.lc3_encoder);
    if (decoder_reset != 0 || encoder_reset != 0) {
        ESP_LOGE(TAG, "LC3 self-test reset failed: decoder=%d encoder=%d",
                 decoder_reset, encoder_reset);
        return false;
    }
    ESP_LOGI(TAG, "LC3 startup self-tests passed; encoder and decoder reset");
    return true;
}
#endif

static void add_timing_us(uint64_t *sum, uint64_t value)
{
    *sum = UINT64_MAX - *sum < value ? UINT64_MAX : *sum + value;
}

static void record_timing(uint32_t *count, uint64_t *sum, uint32_t *maximum, uint32_t us)
{
    if (*count != UINT32_MAX) (*count)++;
    add_timing_us(sum, us);
    if (us > *maximum) *maximum = us;
}

typedef struct {
    audio_packet_store_pop_result_t result;
    audio_packet_t packet;
} audio_playout_event_t;

void audio_playout_reset_far_reference(void)
{
    portENTER_CRITICAL(&g_audio_far_ref_lock);
    memset(g_audio.far_ref_frame, 0, sizeof(g_audio.far_ref_frame));
    memset(g_audio.far_ref_shadows, 0, sizeof(g_audio.far_ref_shadows));
    g_audio.far_ref_shadow_head = 0u;
    portEXIT_CRITICAL(&g_audio_far_ref_lock);
}

/*
 * Each shadow follows one preloaded/submitted DMA descriptor. A successful
 * blocking write means the oldest descriptor completed and can become the AEC
 * reference; the new final mix takes its place in that descriptor's shadow.
 */
static void advance_far_reference_after_write(void)
{
#if AUDIO_ENABLE_AEC_NS && AUDIO_ENABLE_ESP_SR_AEC
    /* No reference conversion is needed when AEC is compiled out. */
    for (size_t offset = 0u; offset < AUDIO_HW_FRAME_SAMPLES;
         offset += AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES) {
        size_t converted = 0u;
        if (audio_rate_converter_process(g_audio.far_reference_converter,
                                         &g_audio.hw_output[offset],
                                         AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES,
                                         g_audio.far_converted, AUDIO_FRAME_SAMPLES,
                                         &converted) != 0 ||
            audio_sample_fifo_write(&g_audio.far_fifo, g_audio.far_converted, converted) != converted) {
            audio_playout_reset_far_reference();
            (void)audio_rate_converter_reset(g_audio.far_reference_converter);
            audio_sample_fifo_reset(&g_audio.far_fifo);
            return;
        }
    }
    memset(g_audio.far_pending_frame, 0, AUDIO_FRAME_SAMPLES * sizeof(int16_t));
    (void)audio_sample_fifo_read(&g_audio.far_fifo, g_audio.far_pending_frame,
                                 AUDIO_FRAME_SAMPLES);
    portENTER_CRITICAL(&g_audio_far_ref_lock);
    memcpy(g_audio.far_converted, g_audio.far_pending_frame,
           AUDIO_FRAME_SAMPLES * sizeof(int16_t));
    memcpy(g_audio.far_ref_frame, g_audio.far_ref_shadows[g_audio.far_ref_shadow_head],
           sizeof(g_audio.far_ref_frame));
    memcpy(g_audio.far_ref_shadows[g_audio.far_ref_shadow_head], g_audio.far_converted,
           sizeof(g_audio.far_ref_shadows[g_audio.far_ref_shadow_head]));
    g_audio.far_ref_shadow_head = (g_audio.far_ref_shadow_head + 1u) % I2S_DMA_BUFFER_COUNT;
    portEXIT_CRITICAL(&g_audio_far_ref_lock);
#endif
}

static void record_decode_result(int samples, int64_t decode_time_us, int64_t *decode_time_sum)
{
    if (samples != AUDIO_FRAME_SAMPLES) {
        AUDIO_STATS_LOCK();
        g_audio.stats.decode_errors++;
        AUDIO_STATS_UNLOCK();
        ESP_LOGW(TAG, "Opus decode failed: %d", samples);
        return;
    }
    if (decode_time_us > 0) {
        uint64_t sum = *decode_time_sum > 0 ? (uint64_t)*decode_time_sum : 0u;
        sum = UINT64_MAX - sum < (uint64_t)decode_time_us
            ? UINT64_MAX : sum + (uint64_t)decode_time_us;
        *decode_time_sum = sum > INT64_MAX ? INT64_MAX : (int64_t)sum;
    }
    AUDIO_STATS_LOCK();
    if (g_audio.stats.frames_decoded != UINT32_MAX) g_audio.stats.frames_decoded++;
    if (g_audio.stats.frames_decoded != 0u) {
        g_audio.stats.decode_time_us_avg = (uint32_t)(
            (uint64_t)*decode_time_sum / g_audio.stats.frames_decoded);
    }
    if ((uint32_t)decode_time_us > g_audio.stats.decode_time_us_max) {
        g_audio.stats.decode_time_us_max = (uint32_t)decode_time_us;
    }
    AUDIO_STATS_UNLOCK();
}

static void update_depth_stats(uint16_t packet_depth)
{
    uint8_t depth = packet_depth > UINT8_MAX ? UINT8_MAX : (uint8_t)packet_depth;
    AUDIO_STATS_LOCK();
    g_audio.stats.jitter_buffer_depth = depth;
    if (g_audio.stats.playout_task_loops == 1 || depth < g_audio.stats.rx_q_depth_min) {
        g_audio.stats.rx_q_depth_min = depth;
    }
    if (depth > g_audio.stats.rx_q_depth_max) {
        g_audio.stats.rx_q_depth_max = depth;
    }
    uint64_t previous =
        (uint64_t)g_audio.stats.rx_q_depth_avg * (uint64_t)(g_audio.stats.playout_task_loops - 1u);
    g_audio.stats.rx_q_depth_avg = (uint8_t)((previous + depth) / g_audio.stats.playout_task_loops);
    AUDIO_STATS_UNLOCK();
}

static void record_rx_pipe_latency(const audio_packet_t *packet, int64_t decode_start)
{
    if (packet->received_us == 0u || (uint64_t)decode_start < packet->received_us) {
        return;
    }
    uint64_t rx_pipe_us = (uint64_t)decode_start - packet->received_us;
    AUDIO_STATS_LOCK();
    if (g_audio.rx_pipe_count != UINT32_MAX) g_audio.rx_pipe_count++;
    add_timing_us(&g_audio.rx_pipe_sum_us, rx_pipe_us);
    if (g_audio.rx_pipe_count != 0u) {
        g_audio.stats.rx_pipe_us_avg = (uint32_t)(g_audio.rx_pipe_sum_us / g_audio.rx_pipe_count);
    }
    if (rx_pipe_us > g_audio.stats.rx_pipe_us_max) {
        g_audio.stats.rx_pipe_us_max = rx_pipe_us > UINT32_MAX ? UINT32_MAX : (uint32_t)rx_pipe_us;
    }
    AUDIO_STATS_UNLOCK();
}

/* Unassigns the source after 1 s without packets; reports whether decode may run. */
static bool refresh_source_assignment(audio_rx_source_t *source, uint64_t now_ms)
{
    bool assigned;
    bool reset_decoder = false;
    bool was_pending = false;

    xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
    assigned = source->assigned;
    if (source->decoder_reset_pending) {
        was_pending = true;
        source->decoder_reset_pending = false;
        reset_decoder = true;
    }
    if (assigned && audio_packet_store_depth(&source->packet_store) == 0u &&
        now_ms - source->last_enqueue_ms >= RX_SOURCE_IDLE_TIMEOUT_MS) {
        source->assigned = false;
        source->source_id = 0;
        source->last_active_ms = 0;
        source->decoder_reset_pending = false;
        audio_packet_store_reset(&source->packet_store);
        assigned = false;
        reset_decoder = true;
    }
    xSemaphoreGive(g_audio.rx_sources_mutex);

    if (reset_decoder) {
        bool reset_ok = true;
#if defined(AUDIO_S31_LC3_WIRE)
        if (g_audio.config.mode == AUDIO_MODE_MESH) {
            reset_ok = esp_lc3_codec_reset(source->lc3_decoder) == 0;
        } else
#endif
        {
            opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
        }
        audio_pcm_resampler_reset(&source->resampler);
        source->decoded_active = false;
        if (!reset_ok && was_pending) {
            xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
            if (source->assigned) source->decoder_reset_pending = true;
            xSemaphoreGive(g_audio.rx_sources_mutex);
            return false;
        }
    }
    return assigned;
}

/* Pops due packets while resampler room allows; returns false when unassigned. */
static bool pop_due_events(audio_rx_source_t *source, uint64_t now_ms,
                           audio_playout_event_t *events, size_t *event_count,
                           uint16_t *packet_depth, size_t *upstream_samples)
{
    size_t event_limit = audio_pcm_resampler_admission_blocks(&source->resampler);
    if (event_limit > AUDIO_PACKET_STORE_CAPACITY) {
        event_limit = AUDIO_PACKET_STORE_CAPACITY;
    }

    bool assigned = true;
    *event_count = 0;
    xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
    if (source->assigned) {
        size_t count;
        uint32_t popped = 0u;
        for (count = 0; count < event_limit; ++count) {
            audio_playout_event_t *event = &events[count];
            event->result = audio_packet_store_pop(&source->packet_store, now_ms, &event->packet);
            if (event->result == AUDIO_PACKET_STORE_POP_NOT_DUE) {
                break;
            }
            if (event->result == AUDIO_PACKET_STORE_POP_PACKET) {
                popped++;
            }
        }
        if (popped != 0u) {
            AUDIO_STATS_LOCK();
            g_audio.stats.rx_store_pop += popped;
            AUDIO_STATS_UNLOCK();
        }
        *event_count = count;
        size_t remaining_depth = audio_packet_store_depth(&source->packet_store);
        *packet_depth += (uint16_t)remaining_depth;
        *upstream_samples = remaining_depth * AUDIO_FRAME_SAMPLES;
    } else {
        assigned = false;
    }
    xSemaphoreGive(g_audio.rx_sources_mutex);
    return assigned;
}

static void decode_event_into_resampler(audio_rx_source_t *source, audio_playout_event_t *event,
                                        int64_t *decode_time_sum)
{
    bool packet_event = event->result == AUDIO_PACKET_STORE_POP_PACKET;
    if (packet_event) {
        source->decoded_active = event->packet.active;
    } else if (event->result == AUDIO_PACKET_STORE_POP_DTX_IDLE) {
        source->decoded_active = false;
    }

    int16_t decoded[AUDIO_FRAME_SAMPLES];
    int64_t decode_start = esp_timer_get_time();
#if defined(AUDIO_S31_LC3_WIRE)
    bool lc3_wire = g_audio.config.mode == AUDIO_MODE_MESH;
    int samples;
#if defined(AUDIO_S31_LC3_SKIP_RX)
    if (lc3_wire) {
        memset(decoded, 0, sizeof(decoded));
        samples = AUDIO_FRAME_SAMPLES;
    } else {
        samples = packet_event
                      ? opus_decode(source->decoder, event->packet.data,
                                    (opus_int32)event->packet.length, decoded,
                                    AUDIO_FRAME_SAMPLES, 0)
                      : opus_decode(source->decoder, NULL, 0, decoded,
                                    AUDIO_FRAME_SAMPLES, 0);
    }
#else
    samples = lc3_wire
                      ? esp_lc3_codec_decode20(source->lc3_decoder,
                                               packet_event ? event->packet.data : NULL,
                                               !packet_event, decoded)
                      : (packet_event
                             ? opus_decode(source->decoder, event->packet.data,
                                           (opus_int32)event->packet.length, decoded,
                                           AUDIO_FRAME_SAMPLES, 0)
                             : opus_decode(source->decoder, NULL, 0, decoded,
                                           AUDIO_FRAME_SAMPLES, 0));
#endif
#else
    int samples =
        packet_event
            ? opus_decode(source->decoder, event->packet.data, (opus_int32)event->packet.length,
                          decoded, AUDIO_FRAME_SAMPLES, 0)
            : opus_decode(source->decoder, NULL, 0, decoded, AUDIO_FRAME_SAMPLES, 0);
#endif
    int64_t decode_time = esp_timer_get_time() - decode_start;

    if (packet_event) {
        record_rx_pipe_latency(&event->packet, decode_start);
#if defined(AUDIO_S31_LC3_SKIP_RX)
        if (!lc3_wire) {
            record_decode_result(samples, decode_time, decode_time_sum);
        }
#else
        record_decode_result(samples, decode_time, decode_time_sum);
#endif
#if !defined(AUDIO_S31_LC3_SKIP_RX)
        if (samples == AUDIO_FRAME_SAMPLES && successful_remote_decodes < 100u) {
            successful_remote_decodes++;
            if (successful_remote_decodes == 1u || successful_remote_decodes == 100u) {
                ESP_LOGI(TAG, "Remote audio decode count=%" PRIu32
                         " playout stack high water: %u bytes",
                         successful_remote_decodes,
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
        }
#endif
    } else if (samples != AUDIO_FRAME_SAMPLES) {
        AUDIO_STATS_LOCK();
        g_audio.stats.decode_errors++;
        AUDIO_STATS_UNLOCK();
        ESP_LOGW(TAG, "Audio PLC failed: %d", samples);
    } else if (event->result == AUDIO_PACKET_STORE_POP_MISSING) {
        AUDIO_STATS_LOCK();
        g_audio.stats.seq_gap_frames++;
        g_audio.stats.conceal_loss_frames++;
        AUDIO_STATS_UNLOCK();
    } else {
        AUDIO_STATS_LOCK();
        g_audio.stats.plc_frames++;
        AUDIO_STATS_UNLOCK();
    }

    if (samples == AUDIO_FRAME_SAMPLES) {
        audio_pcm_resampler_telemetry_t push = audio_pcm_resampler_push(
            &source->resampler, decoded, AUDIO_FRAME_SAMPLES, source->decoded_active);
        if (push.rejected_push) {
            AUDIO_STATS_LOCK();
            g_audio.stats.pcm_fifo_overflows++;
            g_audio.stats.frames_dropped++;
            g_audio.stats.glitches_detected++;
            AUDIO_STATS_UNLOCK();
        }
    }
}

static bool decode_and_buffer_source(audio_rx_source_t *source, uint64_t now_ms,
                                     int64_t *decode_time_sum, uint16_t *packet_depth,
                                     size_t *upstream_samples)
{
    if (!refresh_source_assignment(source, now_ms)) {
        return false;
    }
    audio_playout_event_t events[AUDIO_PACKET_STORE_CAPACITY];
    size_t event_count = 0;
    if (!pop_due_events(source, now_ms, events, &event_count, packet_depth, upstream_samples)) {
        return false;
    }
    for (size_t i = 0; i < event_count; ++i) {
        decode_event_into_resampler(source, &events[i], decode_time_sum);
    }
    return true;
}

static bool render_remote_sources(uint64_t now_ms, int64_t *decode_time_sum)
{
    uint8_t active_sources = 0;
    uint8_t mixed_sources = 0;
    uint16_t packet_depth = 0;
    int32_t current_ppm = 0;
    int32_t current_abs_ppm = 0;
    bool recovery_active = false;
    memset(g_audio.mix_frame, 0, sizeof(g_audio.mix_frame));

    for (size_t source_index = 0; source_index < AUDIO_MAX_RX_SOURCES; ++source_index) {
        audio_rx_source_t *source = &g_audio.rx_sources[source_index];
        size_t upstream_samples = 0u;
        bool assigned = decode_and_buffer_source(source, now_ms, decode_time_sum, &packet_depth,
                                                 &upstream_samples);
        if (!assigned) {
            continue;
        }
        int16_t rendered[AUDIO_FRAME_SAMPLES];
        bool was_started = source->resampler.started;
        audio_pcm_resampler_telemetry_t render =
            audio_pcm_resampler_render(&source->resampler, rendered, upstream_samples);
        recovery_active = recovery_active || render.recovery_active;
        if (render.underrun) {
            AUDIO_STATS_LOCK();
            g_audio.stats.pcm_underruns++;
            g_audio.stats.rx_queue_underruns++;
            g_audio.stats.glitches_detected++;
            AUDIO_STATS_UNLOCK();
        }
        int32_t abs_ppm =
            render.correction_ppm < 0 ? -render.correction_ppm : render.correction_ppm;
        if (abs_ppm > current_abs_ppm) {
            current_abs_ppm = abs_ppm;
            current_ppm = render.correction_ppm;
        }
        if (render.audible_active && (render.started || (was_started && render.underrun))) {
            active_sources++;
            for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
                g_audio.mix_frame[i] += rendered[i];
            }
            mixed_sources++;
        }
    }

    AUDIO_STATS_LOCK();
    g_audio.stats.active_rx_sources = active_sources;
    g_audio.stats.asrc_correction_ppm = current_ppm;
    g_audio.stats.asrc_recovery_active = recovery_active;
    if ((uint32_t)current_abs_ppm > g_audio.stats.asrc_correction_abs_max_ppm) {
        g_audio.stats.asrc_correction_abs_max_ppm = (uint32_t)current_abs_ppm;
    }
    AUDIO_STATS_UNLOCK();
    update_depth_stats(packet_depth);

    if (mixed_sources == 0) {
        memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        return false;
    }
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        int32_t sample = g_audio.mix_frame[i];
        if (mixed_sources > 1) {
            sample /= mixed_sources;
        }
        if (sample > INT16_MAX) {
            sample = INT16_MAX;
        } else if (sample < INT16_MIN) {
            sample = INT16_MIN;
        }
        g_audio.pcm_output[i] = (int16_t)sample;
    }
    return true;
}

static bool render_loopback(int64_t *decode_time_sum)
{
    audio_loopback_item_t item;
    if (xQueueReceive(g_audio.loopback_queue, &item, 0) != pdTRUE) {
        memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        return false;
    }
    int64_t decode_start = esp_timer_get_time();
    int samples = opus_decode(g_audio.loopback_decoder, item.data, item.length, g_audio.pcm_output,
                              AUDIO_FRAME_SAMPLES, 0);
    int64_t decode_time = esp_timer_get_time() - decode_start;
    record_decode_result(samples, decode_time, decode_time_sum);
    if (samples != AUDIO_FRAME_SAMPLES) {
        memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
    }
    return samples == AUDIO_FRAME_SAMPLES;
}

void audio_log_stats(void)
{
    audio_stats_t stats = audio_stats_snapshot();
    /* Fall back to the previous playout sample when either nonblocking lock is busy. */
    uint32_t rx_store_depth = stats.jitter_buffer_depth;
    bool rx_store_depth_valid = false;
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex != NULL && xSemaphoreTake(lifecycle_mutex, 0) == pdTRUE) {
        if (g_audio.rx_sources_mutex != NULL &&
            xSemaphoreTake(g_audio.rx_sources_mutex, 0) == pdTRUE) {
            rx_store_depth = 0u;
            for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
                rx_store_depth += (uint32_t)audio_packet_store_depth(
                    &g_audio.rx_sources[i].packet_store);
            }
            rx_store_depth_valid = true;
            xSemaphoreGive(g_audio.rx_sources_mutex);
        }
        xSemaphoreGive(lifecycle_mutex);
    }
    uint64_t uptime_ms = (uint64_t)esp_timer_get_time() / 1000u;
    audio_rate_converter_timing_t music_cvt = {0};
    bool music_active = false;
    bool music_snapshot_valid = false;
    bool music_format_valid = audio_route_lock_acquire(0);
    if (music_format_valid) {
        music_active = g_audio.bluetooth_music.active && g_audio.bluetooth_music.configured &&
            !(g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured);
        stats.music_rate_hz = music_active ? g_audio.bluetooth_music.sample_rate : 0u;
        stats.music_channels = music_active ? g_audio.bluetooth_music.channels : 0u;
        /* Route users pin the lifecycle; playback takes this mutex before the route lock. */
        if (g_audio.music_playback_mutex != NULL &&
            xSemaphoreTake(g_audio.music_playback_mutex, 0) == pdTRUE) {
            if (music_active) {
                audio_rate_converter_timing_snapshot(g_audio.music_playback_converter, &music_cvt);
                music_snapshot_valid = true;
            }
            xSemaphoreGive(g_audio.music_playback_mutex);
        }
        audio_route_lock_release();
    } else {
        stats.music_rate_hz = 0u;
        stats.music_channels = 0u;
    }
    ESP_LOGI(TAG, "Audio loops capture=%lu playout=%lu encoded=%lu decoded=%lu", stats.task_loops,
             stats.playout_task_loops, stats.frames_encoded, stats.frames_decoded);
    ESP_LOGI(TAG, "  Encoded: %lu frames", stats.frames_encoded);
    ESP_LOGI(TAG, "  Decoded: %lu frames", stats.frames_decoded);
    ESP_LOGI(TAG, "  VOX activations: %lu (active: %s)", stats.vox_activations,
             stats.vox_active ? "YES" : "no");
    ESP_LOGI(TAG, "  Encode time: avg=%lu us, max=%lu us", stats.encode_time_us_avg,
             stats.encode_time_us_max);
    ESP_LOGI(TAG, "  Decode time: avg=%lu us, max=%lu us", stats.decode_time_us_avg,
             stats.decode_time_us_max);
    ESP_LOGI(TAG, "  TX pipeline: avg=%lu us, max=%lu us", stats.tx_pipe_us_avg,
             stats.tx_pipe_us_max);
    ESP_LOGI(TAG, "  RX pipeline: avg=%lu us, max=%lu us", stats.rx_pipe_us_avg,
             stats.rx_pipe_us_max);
    ESP_LOGI(TAG, "  Capture reads: timeouts=%lu errors=%lu", stats.capture_timeouts,
             stats.capture_errors);
    ESP_LOGI(TAG, "  Latency: avg=%lu ms, max=%lu ms", stats.latency_ms_avg, stats.latency_ms_max);
    ESP_LOGI(TAG, "  Glitches: %lu (rx_und=%lu i2s_inc=%lu), ADC overruns: %lu",
             stats.glitches_detected, stats.rx_queue_underruns, stats.i2s_write_incomplete,
             stats.adc_overruns);
    ESP_LOGI(TAG,
             "  Concealment: plc=%lu grace_empty=%lu conceal=%lu seq_gap=%lu "
             "seq_reset=%lu seq_stale=%lu",
             stats.plc_frames, stats.grace_empty_polls, stats.conceal_loss_frames,
             stats.seq_gap_frames, stats.seq_resets, stats.seq_stale_drops);
    ESP_LOGI(TAG, "  Adaptive playout: hold=%lu catchup=%lu sources=%u", stats.hold_frames,
             stats.catchup_frames, stats.active_rx_sources);
    ESP_LOGI(TAG, "  RX queue depth/source: min=%u avg=%u max=%u (total now=%u)",
             stats.rx_q_depth_min, stats.rx_q_depth_avg, stats.rx_q_depth_max,
             stats.jitter_buffer_depth);
    ESP_LOGI(TAG, "Packet drops duplicate=%lu late=%lu future=%lu full=%lu source=%lu lock=%lu",
             stats.packet_duplicate_drops, stats.packet_late_drops, stats.packet_future_drops,
             stats.rx_queue_overflows, stats.rx_source_rejections, stats.rx_lock_drops);
    ESP_LOGI(TAG,
             "Playout plc=%lu conceal=%lu seq_gap=%lu pcm_overflow=%lu pcm_underrun=%lu "
             "asrc_ppm=%ld asrc_abs_max=%lu",
             stats.plc_frames, stats.conceal_loss_frames, stats.seq_gap_frames,
              stats.pcm_fifo_overflows, stats.pcm_underruns, (long)stats.asrc_correction_ppm,
              stats.asrc_correction_abs_max_ppm);
    ESP_LOGI(TAG,
              "Bluetooth route music_overflow=%lu music_underrun=%lu music_convert_err=%lu music_lock=%lu "
              "call_overflow=%lu call_underrun=%lu call_lock=%lu playout_lock=%lu "
              "mic_overflow=%lu mic_underrun=%lu mic_write_lock=%lu mic_read_lock=%lu",
              stats.bluetooth_music_overflows, stats.bluetooth_music_underruns,
              stats.bluetooth_music_conversion_errors,
              stats.bluetooth_music_enqueue_route_lock_misses, stats.bluetooth_call_overflows,
             stats.bluetooth_call_underruns, stats.bluetooth_call_enqueue_route_lock_misses,
             stats.bluetooth_playout_route_lock_misses, stats.bluetooth_mic_overflows,
              stats.bluetooth_mic_underruns, stats.bluetooth_mic_capture_write_route_lock_misses,
              stats.bluetooth_mic_read_route_lock_misses);
     ESP_LOGI(TAG, "  AEC: available=%u active=%u chunk=%u chunks=%lu startup_delay=%lu",
              stats.aec_available ? 1u : 0u, stats.aec_active ? 1u : 0u, stats.aec_chunk_size,
              stats.aec_chunks_processed, stats.aec_startup_delay_frames);
      ESP_LOGI(TAG,
              "audio_log_stats music_converter_fail=%lu music_fifo_overflow=%lu music_render_us_avg=%lu music_render_us_max=%lu playout_work_us_avg=%lu playout_work_us_max=%lu capture_ok=%lu capture_err=%lu",
               stats.bluetooth_music_converter_failures,
               stats.bluetooth_music_output_fifo_overflows,
               stats.bluetooth_music_conversion_us_avg,
               stats.bluetooth_music_conversion_us_max,
               stats.playout_work_us_avg, stats.playout_work_us_max,
               stats.capture_frames_ok, stats.capture_errors);
      ESP_LOGI(TAG,
               "Audio timing read avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32
               " convert avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32
               " aec avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32
               " loop avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32
               " work avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32
               " write avg=%" PRIu32 "us max=%" PRIu32 "us count=%" PRIu32,
               stats.capture_read_us_avg, stats.capture_read_us_max, stats.capture_read_count,
               stats.capture_convert_us_avg, stats.capture_convert_us_max,
               stats.capture_convert_count, stats.capture_aec_us_avg, stats.capture_aec_us_max,
               stats.capture_aec_count, stats.capture_loop_us_avg, stats.capture_loop_us_max,
              stats.capture_loop_count, stats.playout_work_us_avg, stats.playout_work_us_max,
              stats.playout_work_count, stats.playout_write_us_avg,
               stats.playout_write_us_max, stats.playout_write_count);
      if (music_snapshot_valid) {
          ESP_LOGD(TAG, "music_converter count=%" PRIu32 " total_us=%" PRIu64
                   " max_us=%" PRIu32 " src_hz=%" PRIu32 " dst_hz=%" PRIu32 " rc=%" PRId32,
                   music_cvt.count, music_cvt.total_us, music_cvt.max_us,
                   music_cvt.source_rate_hz, music_cvt.destination_rate_hz,
                   music_cvt.last_vendor_result);
     }
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio part=tx epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " tx_handoff=%" PRIu32 " tx_no_cb=%" PRIu32
             " capture_fifo_discard_samples=%" PRIu32
             " capture_ok=%lu capture_short=%lu capture_timeout=%lu capture_err=%lu"
             " encode_ok=%lu encode_err=%lu dtx_drop=%lu",
             stats.pipeline_epoch, uptime_ms, stats.tx_handoff, stats.tx_no_cb,
             stats.capture_fifo_discard_samples, stats.capture_frames_ok,
             stats.capture_short_reads, stats.capture_timeouts, stats.capture_errors,
             stats.frames_encoded, stats.encode_errors, stats.tx_dtx_suppressed);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio part=rx epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " rx_offer=%" PRIu32 " rx_store_ok=%" PRIu32
             " rx_store_reject=%" PRIu32 " rx_invalid=%" PRIu32
             " rx_inactive=%" PRIu32 " rx_store_pop=%" PRIu32
             " rx_store_purge=%" PRIu32 " rx_store_depth=%" PRIu32
             " rx_store_depth_valid=%u rx_q_drop=%lu rx_lock_drop=%lu"
             " rx_src_drop=%lu rx_src_evict=%lu jitter_drop=%lu"
             " seq_reset=%lu seq_stale=%lu rx_sources=%u"
             " packet_dup=%lu packet_late=%lu packet_future=%lu",
             stats.pipeline_epoch, uptime_ms, stats.rx_offer, stats.rx_store_ok,
             stats.rx_store_reject, stats.rx_invalid, stats.rx_inactive,
             stats.rx_store_pop, stats.rx_store_purge, rx_store_depth,
             rx_store_depth_valid ? 1u : 0u, stats.rx_queue_overflows,
             stats.rx_lock_drops, stats.rx_source_rejections, stats.rx_source_evictions,
             stats.jitter_trim_frames, stats.seq_resets, stats.seq_stale_drops,
             stats.active_rx_sources, stats.packet_duplicate_drops,
             stats.packet_late_drops, stats.packet_future_drops);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio part=playout epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " decode_ok=%lu decode_err=%lu plc=%lu"
             " hold=%lu catchup=%lu conceal=%lu seq_gap=%lu glitch=%lu"
             " play_ok=%lu i2s_err=%lu notify_drop=%lu pcm_overflow=%lu"
             " pcm_underrun=%lu asrc_ppm=%ld asrc_abs_max_ppm=%lu"
             " asrc_recovery=%u playout_loops=%lu",
             stats.pipeline_epoch, uptime_ms, stats.frames_decoded, stats.decode_errors,
             stats.plc_frames, stats.hold_frames, stats.catchup_frames,
             stats.conceal_loss_frames, stats.seq_gap_frames, stats.glitches_detected,
             stats.playback_frames, stats.i2s_write_incomplete,
             stats.notification_queue_overflows, stats.pcm_fifo_overflows,
             stats.pcm_underruns, (long)stats.asrc_correction_ppm,
             stats.asrc_correction_abs_max_ppm, stats.asrc_recovery_active ? 1u : 0u,
             stats.playout_task_loops);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio part=bt epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " bt_music_overflow=%lu bt_music_underrun=%lu"
             " bt_music_lock=%lu bt_call_overflow=%lu bt_call_underrun=%lu"
             " bt_call_lock=%lu bt_playout_lock=%lu bt_mic_overflow=%lu"
             " bt_mic_underrun=%lu bt_mic_write_lock=%lu bt_mic_read_lock=%lu",
             stats.pipeline_epoch, uptime_ms, stats.bluetooth_music_overflows,
             stats.bluetooth_music_underruns,
             stats.bluetooth_music_enqueue_route_lock_misses,
             stats.bluetooth_call_overflows, stats.bluetooth_call_underruns,
             stats.bluetooth_call_enqueue_route_lock_misses,
             stats.bluetooth_playout_route_lock_misses, stats.bluetooth_mic_overflows,
             stats.bluetooth_mic_underruns,
             stats.bluetooth_mic_capture_write_route_lock_misses,
              stats.bluetooth_mic_read_route_lock_misses);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio_timing part=notify epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " notify_started_count=%" PRIu32
             " notify_completed_count=%" PRIu32 " notify_mix_count=%" PRIu32
             " notify_mix_us_sum=%" PRIu64 " notify_mix_us_max=%" PRIu32
             " notify_frame_gap_count=%" PRIu32 " notify_frame_gap_us_sum=%" PRIu64
             " notify_frame_gap_us_max=%" PRIu32
             " notify_frame_gap_over25ms_count=%" PRIu32
             " notify_work_count=%" PRIu32 " notify_work_us_sum=%" PRIu64
             " notify_work_us_max=%" PRIu32 " notify_write_count=%" PRIu32
             " notify_write_us_sum=%" PRIu64 " notify_write_us_max=%" PRIu32
             " notify_write_gap_count=%" PRIu32 " notify_write_gap_us_sum=%" PRIu64
             " notify_write_gap_us_max=%" PRIu32,
             stats.pipeline_epoch, uptime_ms, stats.notify_started_count,
             stats.notify_completed_count, stats.notify_mix_count, stats.notify_mix_us_sum,
             stats.notify_mix_us_max, stats.notify_frame_gap_count,
             stats.notify_frame_gap_us_sum, stats.notify_frame_gap_us_max,
             stats.notify_frame_gap_over25ms_count, stats.notify_work_count,
             stats.notify_work_us_sum, stats.notify_work_us_max, stats.notify_write_count,
             stats.notify_write_us_sum, stats.notify_write_us_max, stats.notify_write_gap_count,
              stats.notify_write_gap_us_sum, stats.notify_write_gap_us_max);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio_timing part=music epoch_id=0x%08" PRIx32
             " uptime_ms=%" PRIu64 " music_mutex_wait_count=%" PRIu32
             " music_mutex_wait_us_sum=%" PRIu64 " music_mutex_wait_us_max=%" PRIu32
             " music_route_read_count=%" PRIu32 " music_route_read_us_sum=%" PRIu64
             " music_route_read_us_max=%" PRIu32 " music_convert_count=%" PRIu32
             " music_convert_us_sum=%" PRIu64 " music_convert_us_max=%" PRIu32
             " music_mix_count=%" PRIu32 " music_mix_us_sum=%" PRIu64
             " music_mix_us_max=%" PRIu32 " music_render_over20ms_count=%" PRIu32
             " music_iterations_count=%" PRIu32 " music_chunks_count=%" PRIu32 " music_input_frames=%" PRIu64
             " music_output_frames=%" PRIu64 " music_rate_hz=%" PRIu32
             " music_channels=%u music_format_valid=%u",
             stats.pipeline_epoch, uptime_ms,
             stats.music_mutex_wait.count, stats.music_mutex_wait.us_sum,
             stats.music_mutex_wait.us_max, stats.music_route_read.count,
             stats.music_route_read.us_sum, stats.music_route_read.us_max,
             stats.music_convert.count, stats.music_convert.us_sum, stats.music_convert.us_max,
             stats.music_mix.count, stats.music_mix.us_sum, stats.music_mix.us_max,
             stats.music_render_over20ms_count, stats.music_iterations_count, stats.music_chunks_count,
             stats.music_input_frames, stats.music_output_frames, stats.music_rate_hz,
             stats.music_channels, music_format_valid ? 1u : 0u);
}

static void playout_task_finish(void)
{
    atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
    portENTER_CRITICAL(&g_audio_task_lock);
    bool reset_requested =
        atomic_exchange_explicit(&g_audio.rx_reset_requested, false, memory_order_acq_rel);
    g_audio.playout_task = NULL;
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (reset_requested) {
        audio_rx_reset_source_metadata();
        audio_rx_reset_codecs_and_resamplers();
        xSemaphoreGive(g_audio.rx_reset_done);
    }
    xSemaphoreGive(g_audio.playout_done);
    vTaskDelete(NULL);
}

static void write_playout_frame(bool notify_frame, bool notify_continues,
                                int64_t *previous_notify_write_start_us,
                                audio_output_window_t *output_window)
{
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        *previous_notify_write_start_us = 0;
        return;
    }
    int32_t frame_peak = 0;
    for (size_t i = 0; i < AUDIO_HW_FRAME_SAMPLES; ++i) {
        int32_t sample = g_audio.hw_output[i];
        int32_t magnitude = sample < 0 ? -sample : sample;
        if (magnitude > frame_peak) frame_peak = magnitude;
    }
    output_window->attempted++;
    if (frame_peak != 0) output_window->nonzero_frames++;
    if (frame_peak > output_window->peak_abs) output_window->peak_abs = frame_peak;
    int64_t write_start_us = esp_timer_get_time();
    int ret = esp_codec_dev_write(g_audio.play_dev, (uint8_t *)g_audio.hw_output,
                                  sizeof(g_audio.hw_output));
    uint32_t write_us = (uint32_t)(esp_timer_get_time() - write_start_us);
    AUDIO_STATS_LOCK();
    if (g_audio.stats.playout_write_count != UINT32_MAX) g_audio.stats.playout_write_count++;
    if (UINT64_MAX - g_audio.playout_write_us_sum < write_us)
        g_audio.playout_write_us_sum = UINT64_MAX;
    else
        g_audio.playout_write_us_sum += write_us;
    if (g_audio.stats.playout_write_count != 0u)
        g_audio.stats.playout_write_us_avg = (uint32_t)(g_audio.playout_write_us_sum /
                                                        g_audio.stats.playout_write_count);
    if (write_us > g_audio.stats.playout_write_us_max)
        g_audio.stats.playout_write_us_max = write_us;
    if (notify_frame) {
        record_timing(&g_audio.stats.notify_write_count, &g_audio.stats.notify_write_us_sum,
                      &g_audio.stats.notify_write_us_max, write_us);
        if (*previous_notify_write_start_us != 0 &&
            write_start_us >= *previous_notify_write_start_us) {
            uint32_t gap_us = (uint32_t)(write_start_us - *previous_notify_write_start_us);
            record_timing(&g_audio.stats.notify_write_gap_count,
                          &g_audio.stats.notify_write_gap_us_sum,
                          &g_audio.stats.notify_write_gap_us_max, gap_us);
        }
    }
    AUDIO_STATS_UNLOCK();
    *previous_notify_write_start_us = notify_continues ? write_start_us : 0;
    if (ret != ESP_CODEC_DEV_OK) {
        output_window->failed++;
        AUDIO_STATS_LOCK();
        g_audio.stats.i2s_write_incomplete++;
        g_audio.stats.glitches_detected++;
        AUDIO_STATS_UNLOCK();
        audio_playout_reset_far_reference();
        audio_sample_fifo_reset(&g_audio.far_fifo);
        (void)audio_rate_converter_reset(g_audio.far_reference_converter);
        ESP_LOGW(TAG, "Codec playback write failed: %d", ret);
    } else {
        output_window->successful++;
        int64_t far_span = cpu_profile_span_begin();
        advance_far_reference_after_write();
        cpu_profile_span_end(CPU_PROFILE_SPAN_PLAY_FAR_REFERENCE, far_span);
        AUDIO_STATS_LOCK();
        g_audio.stats.playback_frames++;
        AUDIO_STATS_UNLOCK();
    }
}

void audio_playout_task(void *arg)
{
#if defined(AUDIO_S31_LC3_SPLIT_CORES)
    ESP_LOGI(TAG, "LC3 split cores=1 playout entry mode=%d actual_core=%d affinity_core=%d",
             (int)g_audio.config.mode, (int)xPortGetCoreID(), (int)xTaskGetCoreID(NULL));
#endif
    (void)arg;
    successful_remote_decodes = 0u;
#if defined(AUDIO_S31_LC3_WIRE)
    if (g_audio.config.mode == AUDIO_MODE_MESH) {
#if defined(AUDIO_S31_LC3_SKIP_RX)
        ESP_LOGW(TAG, "Diagnostic mode: LC3 decoder is bypassed; remote audio is silent");
#endif
        for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
            if (esp_lc3_codec_reset(g_audio.rx_sources[i].lc3_decoder) != 0) {
                ESP_LOGE(TAG, "LC3 decoder reset failed for source %zu", i);
                atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
                xSemaphoreGive(g_audio.playout_started);
                playout_task_finish();
                return;
            }
        }
#if defined(AUDIO_S31_LC3_SELFTEST)
        bool selftest_ok = run_lc3_startup_selftest();
#if defined(AUDIO_S31_LC3_SPLIT_CORES)
        ESP_LOGI(TAG, "LC3 split cores=1 playout after selftest mode=%d actual_core=%d affinity_core=%d",
                 (int)g_audio.config.mode, (int)xPortGetCoreID(), (int)xTaskGetCoreID(NULL));
#endif
        if (!selftest_ok) {
            atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
            xSemaphoreGive(g_audio.playout_started);
            playout_task_finish();
            return;
        }
#endif
    }
#endif
    int64_t decode_time_sum = 0;
    uint32_t frame_loops = 0;
    bool stack_logged = false;
    int64_t previous_notify_work_start_us = 0;
    int64_t previous_notify_write_start_us = 0;
    audio_output_window_t output_window = {0};

    opus_decoder_ctl(g_audio.loopback_decoder, OPUS_RESET_STATE);
    audio_rx_reset_codecs_and_resamplers();
    (void)audio_rate_converter_reset(g_audio.voice_playback_converter);
    (void)audio_rate_converter_reset(g_audio.far_reference_converter);
    audio_sample_fifo_reset(&g_audio.music_fifo);
    audio_sample_fifo_reset(&g_audio.far_fifo);
    audio_sample_fifo_reset(&g_audio.voice_fifo);
    audio_sample_fifo_reset(&g_audio.voice_presence_fifo);
    audio_playout_reset_far_reference();
    atomic_store_explicit(&g_audio.playout_ready, true, memory_order_release);
    xSemaphoreGive(g_audio.playout_started);
    ESP_LOGI(TAG, "Playout task started on core %d", xPortGetCoreID());
    int64_t output_window_start_us = esp_timer_get_time();

    while (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        int64_t work_start_us = esp_timer_get_time();
        bool notify_frame = g_audio.notification.active;
        audio_rx_service_reset_request();
        AUDIO_STATS_LOCK();
        g_audio.stats.playout_task_loops++;
        AUDIO_STATS_UNLOCK();
        if (!stack_logged && ++frame_loops >= 50u) {
            ESP_LOGI(TAG, "Playout task stack high water: %u bytes",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
            stack_logged = true;
        }
        if (atomic_exchange_explicit(&g_audio.voice_playback_reset_requested, false,
                                     memory_order_acq_rel)) {
            audio_sample_fifo_reset(&g_audio.voice_fifo);
            audio_sample_fifo_reset(&g_audio.voice_presence_fifo);
            (void)audio_rate_converter_reset(g_audio.voice_playback_converter);
        }
        bool call_priority = atomic_load_explicit(&g_audio.call_priority_active,
                                                  memory_order_acquire);
        size_t base_present_samples = 0u;
        if (call_priority) {
            memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        } else {
            bool base_present;
            if (g_audio.config.mode == AUDIO_MODE_LOOPBACK) {
                base_present = render_loopback(&decode_time_sum);
            } else {
                int64_t remote_span = cpu_profile_span_begin();
                base_present = render_remote_sources((uint64_t)(esp_timer_get_time() / 1000),
                                                     &decode_time_sum);
                cpu_profile_span_end(CPU_PROFILE_SPAN_PLAY_REMOTE, remote_span);
            }
            base_present_samples = base_present ? AUDIO_FRAME_SAMPLES : 0u;
            bool request_consumed;
            int64_t mix_start_us = esp_timer_get_time();
            size_t notification_samples = audio_notify_mix_frame(base_present_samples,
                                                                  &request_consumed);
            uint32_t mix_us = (uint32_t)(esp_timer_get_time() - mix_start_us);
            notify_frame = notify_frame || request_consumed || notification_samples != 0u ||
                           g_audio.notification.active;
            if (notify_frame) {
                AUDIO_STATS_LOCK();
                record_timing(&g_audio.stats.notify_mix_count, &g_audio.stats.notify_mix_us_sum,
                              &g_audio.stats.notify_mix_us_max, mix_us);
                AUDIO_STATS_UNLOCK();
            }
            if (base_present_samples == 0u) base_present_samples = notification_samples;
        }
        bool call_active = audio_route_render_call_frame();
        size_t voice_samples = call_active ? AUDIO_FRAME_SAMPLES : base_present_samples;
        if (!call_active && voice_samples == 0u) {
            memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        }
        size_t converted = 0u;
        int64_t voice_convert_span = cpu_profile_span_begin();
        int convert_result = audio_rate_converter_process(g_audio.voice_playback_converter,
                                                          g_audio.pcm_output, AUDIO_FRAME_SAMPLES,
                                                          g_audio.voice_converted,
                                                          AUDIO_PLAYBACK_CONVERTED_CAPACITY,
                                                          &converted);
        cpu_profile_span_end(CPU_PROFILE_SPAN_PLAY_VOICE_CONVERT, voice_convert_span);
        if (convert_result != 0 ||
            audio_sample_fifo_write(&g_audio.voice_fifo, g_audio.voice_converted,
                                    converted) != converted) {
            audio_sample_fifo_reset(&g_audio.voice_fifo);
            audio_sample_fifo_reset(&g_audio.voice_presence_fifo);
            audio_sample_fifo_reset(&g_audio.far_fifo);
            audio_playout_reset_far_reference();
            (void)audio_rate_converter_reset(g_audio.voice_playback_converter);
            (void)audio_rate_converter_reset(g_audio.far_reference_converter);
            audio_playout_reset_far_reference();
            AUDIO_STATS_LOCK();
            g_audio.stats.pcm_fifo_overflows++;
            g_audio.stats.glitches_detected++;
            AUDIO_STATS_UNLOCK();
        } else {
            size_t present_count = (converted * voice_samples) / AUDIO_FRAME_SAMPLES;
            for (size_t i = 0u; i < converted; ++i) {
                g_audio.voice_presence_staging[i] = i < present_count ? 1 : 0;
            }
            if (audio_sample_fifo_write(&g_audio.voice_presence_fifo,
                                        g_audio.voice_presence_staging, converted) != converted) {
                audio_sample_fifo_reset(&g_audio.voice_fifo);
                audio_sample_fifo_reset(&g_audio.voice_presence_fifo);
                (void)audio_rate_converter_reset(g_audio.voice_playback_converter);
                AUDIO_STATS_LOCK();
                g_audio.stats.pcm_fifo_overflows++;
                g_audio.stats.glitches_detected++;
                AUDIO_STATS_UNLOCK();
            }
        }
        memset(g_audio.hw_output, 0, sizeof(g_audio.hw_output));
        size_t voice_output = audio_sample_fifo_read(&g_audio.voice_fifo, g_audio.hw_output,
                                                     AUDIO_HW_FRAME_SAMPLES);
        (void)audio_sample_fifo_read(&g_audio.voice_presence_fifo, g_audio.voice_presence_frame,
                                     voice_output);
        size_t voice_present = 0u;
        for (size_t i = 0u; i < voice_output; ++i) {
            if (g_audio.voice_presence_frame[i] != 0) voice_present = i + 1u;
        }
        if (!call_active) audio_route_mix_music_48k(voice_present);
        uint32_t work_us = (uint32_t)(esp_timer_get_time() - work_start_us);
        AUDIO_STATS_LOCK();
        add_timing_us(&g_audio.playout_work_us_sum, work_us);
        if (g_audio.playout_work_frames != UINT32_MAX) g_audio.playout_work_frames++;
        g_audio.stats.playout_work_count = g_audio.playout_work_frames;
        if (g_audio.playout_work_frames != 0u) {
            g_audio.stats.playout_work_us_avg = (uint32_t)(g_audio.playout_work_us_sum /
                                                            g_audio.playout_work_frames);
        }
        if (work_us > g_audio.stats.playout_work_us_max)
            g_audio.stats.playout_work_us_max = work_us;
        if (notify_frame) {
            record_timing(&g_audio.stats.notify_work_count, &g_audio.stats.notify_work_us_sum,
                          &g_audio.stats.notify_work_us_max, work_us);
            if (previous_notify_work_start_us != 0 &&
                work_start_us >= previous_notify_work_start_us) {
                uint32_t gap_us = (uint32_t)(work_start_us - previous_notify_work_start_us);
                record_timing(&g_audio.stats.notify_frame_gap_count,
                              &g_audio.stats.notify_frame_gap_us_sum,
                              &g_audio.stats.notify_frame_gap_us_max, gap_us);
                if (gap_us > 25000u &&
                    g_audio.stats.notify_frame_gap_over25ms_count != UINT32_MAX)
                    g_audio.stats.notify_frame_gap_over25ms_count++;
            }
        }
        AUDIO_STATS_UNLOCK();
        bool notify_continues = notify_frame && g_audio.notification.active;
        previous_notify_work_start_us = notify_continues ? work_start_us : 0;
        write_playout_frame(notify_frame, notify_continues, &previous_notify_write_start_us,
                            &output_window);
        int64_t output_now_us = esp_timer_get_time();
        if (output_window.attempted >= 500u ||
            output_now_us - output_window_start_us >= 10000000LL) {
            ESP_LOGI(TAG, "AUDIO_OUT window_attempted=%" PRIu32 " successful=%" PRIu32
                          " failed=%" PRIu32 " attempted_nonzero_frames=%" PRIu32
                          " attempted_peak_abs=%" PRId32,
                     output_window.attempted, output_window.successful, output_window.failed,
                     output_window.nonzero_frames, output_window.peak_abs);
            audio_hw_log_output_state();
            output_window = (audio_output_window_t){0};
            output_window_start_us = esp_timer_get_time();
        }

    }

    audio_rx_service_reset_request();
    ESP_LOGI(TAG, "Playout task stopped");
    playout_task_finish();
}
