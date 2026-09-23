/**
 * @file audio_playout.c
 * @brief Playout task: decode, adaptive render, mixing, I2S writes, heartbeat.
 */

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_internal.h"

static const char *TAG = "audio";

static void add_timing_us(uint64_t *sum, uint64_t value)
{
    *sum = UINT64_MAX - *sum < value ? UINT64_MAX : *sum + value;
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
    audio_rate_converter_timing_t music_cvt = {0};
    bool music_active = false;
    bool music_snapshot_valid = false;
    if (audio_route_lock_acquire(0)) {
        music_active = g_audio.bluetooth_music.active && g_audio.bluetooth_music.configured &&
            !(g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured);
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
              "PIPE v=1 dev=esp stage=audio capture_ok=%lu capture_short=%lu "
              "capture_timeout=%lu capture_err=%lu encode_ok=%lu encode_err=%lu dtx_drop=%lu "
              "rx_q_drop=%lu rx_lock_drop=%lu rx_src_drop=%lu rx_src_evict=%lu "
             "jitter_drop=%lu decode_ok=%lu "
             "decode_err=%lu plc=%lu hold=%lu catchup=%lu conceal=%lu seq_gap=%lu "
             "seq_reset=%lu seq_stale=%lu glitch=%lu play_ok=%lu i2s_err=%lu notify_drop=%lu "
             "rx_sources=%u packet_dup=%lu packet_late=%lu packet_future=%lu pcm_overflow=%lu "
             "pcm_underrun=%lu asrc_ppm=%ld asrc_abs_max_ppm=%lu asrc_recovery=%u "
              "playout_loops=%lu bt_music_overflow=%lu bt_music_underrun=%lu "
              "bt_music_lock=%lu bt_call_overflow=%lu bt_call_underrun=%lu bt_call_lock=%lu "
              "bt_playout_lock=%lu bt_mic_overflow=%lu bt_mic_underrun=%lu "
              "bt_mic_write_lock=%lu bt_mic_read_lock=%lu",
              stats.capture_frames_ok, stats.capture_short_reads, stats.capture_timeouts,
              stats.capture_errors, stats.frames_encoded, stats.encode_errors,
              stats.tx_dtx_suppressed,
             stats.rx_queue_overflows, stats.rx_lock_drops, stats.rx_source_rejections,
             stats.rx_source_evictions, stats.jitter_trim_frames, stats.frames_decoded,
             stats.decode_errors, stats.plc_frames, stats.hold_frames, stats.catchup_frames,
             stats.conceal_loss_frames, stats.seq_gap_frames, stats.seq_resets,
             stats.seq_stale_drops, stats.glitches_detected, stats.playback_frames,
             stats.i2s_write_incomplete, stats.notification_queue_overflows,
             stats.active_rx_sources, stats.packet_duplicate_drops, stats.packet_late_drops,
             stats.packet_future_drops, stats.pcm_fifo_overflows, stats.pcm_underruns,
             (long)stats.asrc_correction_ppm, stats.asrc_correction_abs_max_ppm,
              stats.asrc_recovery_active ? 1u : 0u, stats.playout_task_loops,
               stats.bluetooth_music_overflows, stats.bluetooth_music_underruns,
               stats.bluetooth_music_enqueue_route_lock_misses, stats.bluetooth_call_overflows,
              stats.bluetooth_call_underruns, stats.bluetooth_call_enqueue_route_lock_misses,
              stats.bluetooth_playout_route_lock_misses, stats.bluetooth_mic_overflows,
              stats.bluetooth_mic_underruns, stats.bluetooth_mic_capture_write_route_lock_misses,
              stats.bluetooth_mic_read_route_lock_misses);
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

static void write_playout_frame(void)
{
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        return;
    }
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
    AUDIO_STATS_UNLOCK();
    if (ret != ESP_CODEC_DEV_OK) {
        AUDIO_STATS_LOCK();
        g_audio.stats.i2s_write_incomplete++;
        g_audio.stats.glitches_detected++;
        AUDIO_STATS_UNLOCK();
        audio_playout_reset_far_reference();
        audio_sample_fifo_reset(&g_audio.far_fifo);
        (void)audio_rate_converter_reset(g_audio.far_reference_converter);
        ESP_LOGW(TAG, "Codec playback write failed: %d", ret);
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
    uint32_t frame_loops = 0;
    bool stack_logged = false;

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

    while (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        int64_t work_start_us = esp_timer_get_time();
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
                base_present = render_remote_sources((uint64_t)(esp_timer_get_time() / 1000),
                                                     &decode_time_sum);
            }
            base_present_samples = base_present ? AUDIO_FRAME_SAMPLES : 0u;
            size_t notification_samples = audio_notify_mix_frame(base_present_samples);
            if (base_present_samples == 0u) base_present_samples = notification_samples;
        }
        bool call_active = audio_route_render_call_frame();
        size_t voice_samples = call_active ? AUDIO_FRAME_SAMPLES : base_present_samples;
        if (!call_active && voice_samples == 0u) {
            memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
        }
        size_t converted = 0u;
        if (audio_rate_converter_process(g_audio.voice_playback_converter, g_audio.pcm_output,
                                         AUDIO_FRAME_SAMPLES, g_audio.voice_converted,
                                          AUDIO_PLAYBACK_CONVERTED_CAPACITY, &converted) != 0 ||
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
        AUDIO_STATS_UNLOCK();
        write_playout_frame();

    }

    audio_rx_service_reset_request();
    ESP_LOGI(TAG, "Playout task stopped");
    playout_task_finish();
}
