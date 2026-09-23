#include "audio_internal.h"
#include "audio_music_schedule.h"

#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

static void add_route_stat(uint32_t *counter, size_t quantity)
{
    uint32_t add = quantity > UINT32_MAX ? UINT32_MAX : (uint32_t)quantity;
    *counter = UINT32_MAX - *counter < add ? UINT32_MAX : *counter + add;
}

static void add_music_render_time(uint32_t elapsed)
{
    uint64_t max_sum = UINT64_MAX - g_audio.bluetooth_music_conversion_us_sum;
    g_audio.bluetooth_music_conversion_us_sum += max_sum < elapsed ? max_sum : elapsed;
    if (g_audio.bluetooth_music_conversion_frames != UINT32_MAX) {
        g_audio.bluetooth_music_conversion_frames++;
    }
    if (g_audio.bluetooth_music_conversion_frames != 0u) {
        g_audio.stats.bluetooth_music_conversion_us_avg = (uint32_t)(
            g_audio.bluetooth_music_conversion_us_sum / g_audio.bluetooth_music_conversion_frames);
    }
    if (elapsed > g_audio.stats.bluetooth_music_conversion_us_max) {
        g_audio.stats.bluetooth_music_conversion_us_max = elapsed;
    }
}

static bool s_music_fifo_failure_logged;

bool audio_route_init(void)
{
    if (!audio_route_stream_init(&g_audio.bluetooth_music, g_audio.bluetooth_music.samples,
                                 g_audio.bluetooth_music.capacity) ||
        !audio_route_stream_init(&g_audio.bluetooth_call, g_audio.bluetooth_call.samples,
                                 g_audio.bluetooth_call.capacity) ||
        !audio_route_stream_init(&g_audio.bluetooth_mic, g_audio.bluetooth_mic.samples,
                                 g_audio.bluetooth_mic.capacity)) return false;
    audio_route_mic_reset(&g_audio.bluetooth_mic);
    g_audio.bluetooth_mic.active = false;
    g_audio.bluetooth_mic_rate = 16000u;
    audio_sample_fifo_init(&g_audio.music_fifo, g_audio.music_fifo_storage,
                           AUDIO_PLAYBACK_FIFO_CAPACITY);
    audio_sample_fifo_init(&g_audio.far_fifo, g_audio.far_fifo_storage,
                           AUDIO_FAR_FIFO_CAPACITY);
    audio_sample_fifo_init(&g_audio.voice_fifo, g_audio.voice_fifo_storage,
                           AUDIO_PLAYBACK_FIFO_CAPACITY);
    audio_sample_fifo_init(&g_audio.voice_presence_fifo, g_audio.voice_presence_fifo_storage,
                           AUDIO_PLAYBACK_FIFO_CAPACITY);
    atomic_store_explicit(&g_audio.call_priority_active, false, memory_order_release);
    return true;
}

void audio_route_reset_streams(void)
{
    if (!audio_route_lock_acquire(portMAX_DELAY)) return;
    audio_route_stream_reset(&g_audio.bluetooth_music);
    audio_route_stream_reset(&g_audio.bluetooth_call);
    audio_route_stream_reset(&g_audio.bluetooth_mic);
    audio_sample_fifo_reset(&g_audio.music_fifo);
    atomic_fetch_add_explicit(&g_audio.music_playback_generation, 1u, memory_order_acq_rel);
    atomic_store_explicit(&g_audio.call_priority_active, false, memory_order_release);
    audio_route_lock_release();
}

void audio_route_capture_frame(const int16_t *samples, size_t count)
{
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        AUDIO_STATS_LOCK();
        add_route_stat(&g_audio.stats.bluetooth_mic_capture_write_route_lock_misses, 1u);
        AUDIO_STATS_UNLOCK();
        return;
    }
    if (g_audio.bluetooth_mic.active) {
        size_t accepted = audio_route_mic_write(&g_audio.bluetooth_mic, samples, count);
        if (accepted != count) {
            AUDIO_STATS_LOCK();
            add_route_stat(&g_audio.stats.bluetooth_mic_overflows, count - accepted);
            AUDIO_STATS_UNLOCK();
        }
    }
    audio_route_lock_release();
}

bool audio_route_render_call_frame(void)
{
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        AUDIO_STATS_LOCK();
        add_route_stat(&g_audio.stats.bluetooth_playout_route_lock_misses, 1u);
        AUDIO_STATS_UNLOCK();
        if (atomic_load_explicit(&g_audio.call_priority_active, memory_order_acquire)) {
            memset(g_audio.pcm_output, 0, sizeof(g_audio.pcm_output));
            return true;
        }
        return false;
    }
    bool active = g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured;
    if (active) {
        size_t produced = audio_route_stream_render(&g_audio.bluetooth_call, g_audio.pcm_output,
                                                    AUDIO_FRAME_SAMPLES);
        if (produced != AUDIO_FRAME_SAMPLES) {
            AUDIO_STATS_LOCK();
            g_audio.stats.bluetooth_call_underruns++;
            g_audio.stats.glitches_detected++;
            AUDIO_STATS_UNLOCK();
        }
    }
    audio_route_lock_release();
    return active;
}

void audio_route_mix_music_48k(size_t voice_present_samples)
{
    int64_t render_start_us = esp_timer_get_time();
    size_t music_samples = 0u;
    size_t chunks_processed = 0u;
    uint32_t generation = atomic_load_explicit(&g_audio.music_playback_generation,
                                               memory_order_acquire);
    bool active = false;
    uint32_t input_rate = 0u;
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        AUDIO_STATS_LOCK();
        add_route_stat(&g_audio.stats.bluetooth_playout_route_lock_misses, 1u);
        AUDIO_STATS_UNLOCK();
        return;
    }
    if (g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured) {
        audio_route_lock_release();
        return;
    }
    audio_route_stream_t *music = &g_audio.bluetooth_music;
    active = music->active && music->configured;
    input_rate = music->sample_rate;
    audio_route_lock_release();

    if (active && xSemaphoreTake(g_audio.music_playback_mutex, portMAX_DELAY) == pdTRUE) {
        if (generation == atomic_load_explicit(&g_audio.music_playback_generation,
                                               memory_order_acquire)) {
            size_t available = 0u;
            if (audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
                if (g_audio.bluetooth_music.active && g_audio.bluetooth_music.configured &&
                    !(g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured)) {
                    available = g_audio.bluetooth_music.depth;
                }
                audio_route_lock_release();
            }
            size_t output_needed = g_audio.music_fifo.depth < AUDIO_HW_FRAME_SAMPLES
                ? AUDIO_HW_FRAME_SAMPLES - g_audio.music_fifo.depth : 0u;
            size_t input_limit = audio_music_input_chunk_limit(input_rate,
                output_needed, available);
            size_t input_consumed = 0u;
            bool conversion_failed = false;
            while (chunks_processed < AUDIO_MUSIC_MAX_INPUT_CHUNKS) {
                size_t fifo_short = g_audio.music_fifo.depth < AUDIO_HW_FRAME_SAMPLES
                    ? AUDIO_HW_FRAME_SAMPLES - g_audio.music_fifo.depth : 0u;
                size_t scheduled_remaining = input_limit > input_consumed
                    ? input_limit - input_consumed : 0u;
                if (scheduled_remaining == 0u && fifo_short != 0u && available > input_consumed) {
                    scheduled_remaining = audio_music_input_chunk_limit(input_rate,
                        fifo_short, available - input_consumed);
                }
                if (scheduled_remaining == 0u) break;
                int16_t *input = g_audio.music_input;
                size_t input_count = scheduled_remaining;
                if (input_count > AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES)
                    input_count = AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES;
                if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) break;
                bool stream_active = g_audio.bluetooth_music.active &&
                    g_audio.bluetooth_music.configured &&
                    !(g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured);
                if (!stream_active || generation != atomic_load_explicit(
                        &g_audio.music_playback_generation, memory_order_acquire)) {
                    audio_route_lock_release();
                    break;
                }
                input_count = audio_route_stream_read_raw(&g_audio.bluetooth_music,
                                                           input, input_count);
                audio_route_lock_release();
                if (input_count == 0u) break;
                input_consumed += input_count;
                size_t output_count = 0u;
                int convert_result = g_audio.music_playback_converter == NULL ? -1 :
                    audio_rate_converter_process(g_audio.music_playback_converter,
                        input, input_count, g_audio.music_converted,
                        AUDIO_PLAYBACK_CONVERTED_CAPACITY, &output_count);
                if (convert_result != 0) {
                    AUDIO_STATS_LOCK();
                    add_route_stat(&g_audio.stats.bluetooth_music_converter_failures, 1u);
                    AUDIO_STATS_UNLOCK();
                    (void)audio_rate_converter_reset(g_audio.music_playback_converter);
                    audio_sample_fifo_reset(&g_audio.music_fifo);
                    conversion_failed = true;
                    break;
                }
                if (audio_sample_fifo_write(&g_audio.music_fifo, g_audio.music_converted,
                                            output_count) != output_count) {
                    AUDIO_STATS_LOCK();
                    add_route_stat(&g_audio.stats.bluetooth_music_output_fifo_overflows, 1u);
                    AUDIO_STATS_UNLOCK();
                    if (!s_music_fifo_failure_logged) {
                        s_music_fifo_failure_logged = true;
                        ESP_LOGW("audio", "music conversion failed reason=output_fifo_capacity rc=0");
                    }
                    audio_sample_fifo_reset(&g_audio.music_fifo);
                    conversion_failed = true;
                    break;
                }
                chunks_processed++;
                if (output_count == 0u) break;
            }
            if (conversion_failed) {
                AUDIO_STATS_LOCK();
                add_route_stat(&g_audio.stats.bluetooth_music_conversion_errors, 1u);
                AUDIO_STATS_UNLOCK();
            }
            music_samples = audio_sample_fifo_read(&g_audio.music_fifo,
                                                    g_audio.music_converted,
                                                    AUDIO_HW_FRAME_SAMPLES);
        }
        xSemaphoreGive(g_audio.music_playback_mutex);
    }
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        AUDIO_STATS_LOCK();
        add_route_stat(&g_audio.stats.bluetooth_playout_route_lock_misses, 1u);
        AUDIO_STATS_UNLOCK();
        return;
    }
    bool still_active = g_audio.bluetooth_music.active && g_audio.bluetooth_music.configured &&
        !(g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured);
    audio_route_lock_release();
    if (!still_active || generation != atomic_load_explicit(&g_audio.music_playback_generation,
                                                            memory_order_acquire)) {
        music_samples = 0u;
    } else if (music_samples < AUDIO_HW_FRAME_SAMPLES) {
        AUDIO_STATS_LOCK();
        g_audio.stats.bluetooth_music_underruns++;
        AUDIO_STATS_UNLOCK();
    }
    for (size_t i = 0u; i < AUDIO_HW_FRAME_SAMPLES; ++i) {
        g_audio.hw_output[i] = audio_route_mix_sample(
            false, i < voice_present_samples, i < music_samples, g_audio.hw_output[i],
            i < music_samples ? g_audio.music_converted[i] : 0, 0);
    }
    if (active) {
        int64_t elapsed_us = esp_timer_get_time() - render_start_us;
        uint32_t elapsed = elapsed_us > UINT32_MAX ? UINT32_MAX :
            (uint32_t)(elapsed_us > 0 ? elapsed_us : 0);
        AUDIO_STATS_LOCK();
        add_music_render_time(elapsed);
        AUDIO_STATS_UNLOCK();
    }
}
