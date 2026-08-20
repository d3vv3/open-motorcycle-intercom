/**
 * @file audio_playout.c
 * @brief Playout task: decode, adaptive render, mixing, I2S writes, heartbeat.
 */

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_internal.h"

static const char *TAG = "audio";

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
    portENTER_CRITICAL(&g_audio_far_ref_lock);
    memcpy(g_audio.far_ref_frame, g_audio.far_ref_shadows[g_audio.far_ref_shadow_head],
           sizeof(g_audio.far_ref_frame));
    memcpy(g_audio.far_ref_shadows[g_audio.far_ref_shadow_head], g_audio.pcm_output,
           sizeof(g_audio.far_ref_shadows[g_audio.far_ref_shadow_head]));
    g_audio.far_ref_shadow_head = (g_audio.far_ref_shadow_head + 1u) % I2S_DMA_BUFFER_COUNT;
    portEXIT_CRITICAL(&g_audio_far_ref_lock);
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
    *decode_time_sum += decode_time_us;
    AUDIO_STATS_LOCK();
    g_audio.stats.frames_decoded++;
    g_audio.stats.decode_time_us_avg = (uint32_t)(*decode_time_sum / g_audio.stats.frames_decoded);
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
    g_audio.rx_pipe_count++;
    g_audio.rx_pipe_sum_us += rx_pipe_us;
    g_audio.stats.rx_pipe_us_avg = (uint32_t)(g_audio.rx_pipe_sum_us / g_audio.rx_pipe_count);
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

    xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
    assigned = source->assigned;
    if (source->decoder_reset_pending) {
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
        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
        audio_pcm_resampler_reset(&source->resampler);
        source->decoded_active = false;
    }
    return assigned;
}

/* Pops due packets while resampler room allows; returns false when unassigned. */
static bool pop_due_events(audio_rx_source_t *source, uint64_t now_ms,
                           audio_playout_event_t *events, size_t *event_count,
                           uint16_t *packet_depth, size_t *upstream_samples)
{
    size_t pcm_depth = audio_pcm_resampler_depth(&source->resampler);
    size_t target_room = pcm_depth < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES
                             ? AUDIO_PCM_RESAMPLER_TARGET_SAMPLES - pcm_depth
                             : 0u;
    size_t event_limit = target_room / AUDIO_FRAME_SAMPLES;
    size_t available_events =
        audio_pcm_resampler_available(&source->resampler) / AUDIO_FRAME_SAMPLES;
    if (event_limit > available_events) {
        event_limit = available_events;
    }
    if (event_limit > AUDIO_PACKET_STORE_CAPACITY) {
        event_limit = AUDIO_PACKET_STORE_CAPACITY;
    }

    bool assigned = true;
    *event_count = 0;
    xSemaphoreTake(g_audio.rx_sources_mutex, portMAX_DELAY);
    if (source->assigned) {
        size_t count;
        for (count = 0; count < event_limit; ++count) {
            audio_playout_event_t *event = &events[count];
            event->result = audio_packet_store_pop(&source->packet_store, now_ms, &event->packet);
            if (event->result == AUDIO_PACKET_STORE_POP_NOT_DUE) {
                break;
            }
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
    int samples =
        packet_event
            ? opus_decode(source->decoder, event->packet.data, (opus_int32)event->packet.length,
                          decoded, AUDIO_FRAME_SAMPLES, 0)
            : opus_decode(source->decoder, NULL, 0, decoded, AUDIO_FRAME_SAMPLES, 0);
    int64_t decode_time = esp_timer_get_time() - decode_start;

    if (packet_event) {
        record_rx_pipe_latency(&event->packet, decode_start);
        record_decode_result(samples, decode_time, decode_time_sum);
    } else if (samples != AUDIO_FRAME_SAMPLES) {
        AUDIO_STATS_LOCK();
        g_audio.stats.decode_errors++;
        AUDIO_STATS_UNLOCK();
        ESP_LOGW(TAG, "Opus PLC failed: %d", samples);
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

static void render_loopback(int64_t *decode_time_sum)
{
    audio_loopback_item_t item;
    if (xQueueReceive(g_audio.loopback_queue, &item, 0) != pdTRUE) {
        memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        return;
    }
    int64_t decode_start = esp_timer_get_time();
    int samples = opus_decode(g_audio.loopback_decoder, item.data, item.length, g_audio.pcm_output,
                              AUDIO_FRAME_SAMPLES, 0);
    int64_t decode_time = esp_timer_get_time() - decode_start;
    record_decode_result(samples, decode_time, decode_time_sum);
    if (samples != AUDIO_FRAME_SAMPLES) {
        memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
    }
}

static void log_audio_stats(void)
{
    audio_stats_t stats = audio_stats_snapshot();
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
             "PIPE v=1 dev=esp stage=audio capture_ok=%lu capture_short=%lu "
             "capture_timeout=%lu capture_err=%lu encode_ok=%lu encode_err=%lu dtx_drop=%lu "
             "rx_q_drop=%lu rx_lock_drop=%lu rx_src_drop=%lu rx_src_evict=%lu "
             "jitter_drop=%lu decode_ok=%lu "
             "decode_err=%lu plc=%lu hold=%lu catchup=%lu conceal=%lu seq_gap=%lu "
             "seq_reset=%lu seq_stale=%lu glitch=%lu play_ok=%lu i2s_err=%lu notify_drop=%lu "
             "rx_sources=%u packet_dup=%lu packet_late=%lu packet_future=%lu pcm_overflow=%lu "
             "pcm_underrun=%lu asrc_ppm=%ld asrc_abs_max_ppm=%lu asrc_recovery=%u "
             "playout_loops=%lu",
             stats.capture_frames_ok, stats.capture_short_reads, stats.capture_timeouts,
             stats.adc_overruns, stats.frames_encoded, stats.encode_errors, stats.tx_dtx_suppressed,
             stats.rx_queue_overflows, stats.rx_lock_drops, stats.rx_source_rejections,
             stats.rx_source_evictions, stats.jitter_trim_frames, stats.frames_decoded,
             stats.decode_errors, stats.plc_frames, stats.hold_frames, stats.catchup_frames,
             stats.conceal_loss_frames, stats.seq_gap_frames, stats.seq_resets,
             stats.seq_stale_drops, stats.glitches_detected, stats.playback_frames,
             stats.i2s_write_incomplete, stats.notification_queue_overflows,
             stats.active_rx_sources, stats.packet_duplicate_drops, stats.packet_late_drops,
             stats.packet_future_drops, stats.pcm_fifo_overflows, stats.pcm_underruns,
             (long)stats.asrc_correction_ppm, stats.asrc_correction_abs_max_ppm,
             stats.asrc_recovery_active ? 1u : 0u, stats.playout_task_loops);
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

static bool prepare_i2s_output(void)
{
    for (size_t i = 0; i < I2S_DMA_BUFFER_COUNT; ++i) {
        size_t bytes_loaded = 0;
        esp_err_t preload = i2s_channel_preload_data(g_audio.tx_chan, g_audio.i2s_silence,
                                                     sizeof(g_audio.i2s_silence), &bytes_loaded);
        if (preload != ESP_OK || bytes_loaded != sizeof(g_audio.i2s_silence)) {
            ESP_LOGE(TAG, "I2S silence preload %zu failed: %s (%zu bytes)", i,
                     esp_err_to_name(preload), bytes_loaded);
            return false;
        }
    }
    esp_err_t ret = i2s_channel_enable(g_audio.tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

static void write_playout_frame(void)
{
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        g_audio.pcm_stereo[i * 2] = g_audio.pcm_output[i];
        g_audio.pcm_stereo[i * 2 + 1] = g_audio.pcm_output[i];
    }
    size_t bytes_written = 0;
    esp_err_t ret =
        i2s_channel_write(g_audio.tx_chan, g_audio.pcm_stereo, sizeof(g_audio.pcm_stereo),
                          &bytes_written, I2S_WRITE_TIMEOUT_MS);
    if (ret != ESP_OK || bytes_written != sizeof(g_audio.pcm_stereo)) {
        AUDIO_STATS_LOCK();
        g_audio.stats.i2s_write_incomplete++;
        g_audio.stats.glitches_detected++;
        AUDIO_STATS_UNLOCK();
        audio_playout_reset_far_reference();
        ESP_LOGW(TAG, "I2S write incomplete: %s (%zu bytes)", esp_err_to_name(ret), bytes_written);
    } else {
        advance_far_reference_after_write();
        AUDIO_STATS_LOCK();
        g_audio.stats.playback_frames++;
        AUDIO_STATS_UNLOCK();
    }
}

void audio_playout_task(void *arg)
{
    (void)arg;
    int64_t decode_time_sum = 0;
    int64_t last_heartbeat_ms = esp_timer_get_time() / 1000;

    opus_decoder_ctl(g_audio.loopback_decoder, OPUS_RESET_STATE);
    audio_rx_reset_codecs_and_resamplers();
    audio_playout_reset_far_reference();
    if (!prepare_i2s_output()) {
        atomic_store_explicit(&g_audio.running, false, memory_order_release);
        xSemaphoreGive(g_audio.playout_started);
        playout_task_finish();
    }

    atomic_store_explicit(&g_audio.playout_ready, true, memory_order_release);
    xSemaphoreGive(g_audio.playout_started);
    ESP_LOGI(TAG, "Playout task started on core %d", xPortGetCoreID());

    while (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        audio_rx_service_reset_request();
        AUDIO_STATS_LOCK();
        g_audio.stats.playout_task_loops++;
        AUDIO_STATS_UNLOCK();
        if (g_audio.config.mode == AUDIO_MODE_LOOPBACK) {
            render_loopback(&decode_time_sum);
        } else {
            (void)render_remote_sources((uint64_t)(esp_timer_get_time() / 1000), &decode_time_sum);
        }
        audio_notify_mix_frame();
        write_playout_frame();

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_heartbeat_ms >= AUDIO_HEARTBEAT_INTERVAL_MS) {
            log_audio_stats();
            last_heartbeat_ms = now_ms;
        }
    }

    audio_rx_service_reset_request();
    i2s_channel_disable(g_audio.tx_chan);
    ESP_LOGI(TAG, "Playout task stopped");
    playout_task_finish();
}
