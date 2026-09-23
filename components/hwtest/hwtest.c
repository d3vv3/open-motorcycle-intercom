/**
 * @file hwtest.c
 * @brief Hardware validation tests for OMI.
 *
 * These tests exercise the production audio path, including the ES8311 codec.
 */

#include "hwtest.h"

#include <stdbool.h>
#include <stddef.h>

#include "audio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hwtest";

#define MIC_TEST_DURATION_SEC 5
#define SPEAKER_TEST_DURATION_SEC 4
#define AUDIO_FRAMES_PER_SEC 50
#define MIC_MIN_FRAME_PERCENT 80
#define LOOPBACK_MIN_FRAME_PERCENT 75
#define SPEAKER_MIN_FRAME_PERCENT 75
#define MAX_ERROR_PERCENT 20
#define MAX_IO_ERROR_PERCENT 5
#define MIC_MIN_PEAK_ABS 512
#define LOOPBACK_MIN_PEAK_ABS MIC_MIN_PEAK_ABS
#define TEST_DELAY_CHUNK_MS 100

static esp_err_t hwtest_audio_start(const audio_config_t *config)
{
    return audio_init_and_start_with_config(config);
}

static esp_err_t hwtest_audio_stop(void)
{
    esp_err_t ret = ESP_OK;
    if (audio_is_running()) {
        ret = audio_stop();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Audio stop failed: %s", esp_err_to_name(ret));
        }
    }
    esp_err_t deinit_ret = audio_deinit();
    if (deinit_ret == ESP_ERR_INVALID_STATE && !audio_is_running()) {
        /* An external deinit may have completed the exclusive test already. */
        deinit_ret = ESP_OK;
    } else if (deinit_ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio deinit failed: %s", esp_err_to_name(deinit_ret));
    }
    return ret != ESP_OK ? ret : deinit_ret;
}

static uint32_t minimum_frames(uint64_t expected, uint32_t percent)
{
    if (expected <= 4u) {
        return 1u;
    }
    uint64_t minimum = (expected * percent + 99u) / 100u;
    return minimum > UINT32_MAX ? UINT32_MAX : (uint32_t)minimum;
}

static uint32_t maximum_errors(uint64_t expected, uint32_t percent)
{
    uint64_t limit64 = (expected * percent) / 100u;
    uint32_t limit = limit64 > UINT32_MAX ? UINT32_MAX : (uint32_t)limit64;
    return limit == 0u ? 1u : limit;
}

static bool wait_for_audio(uint64_t duration_ms)
{
    while (duration_ms != 0u && audio_is_running()) {
        uint32_t chunk_ms = duration_ms > TEST_DELAY_CHUNK_MS ? TEST_DELAY_CHUNK_MS : (uint32_t)duration_ms;
        vTaskDelay(pdMS_TO_TICKS(chunk_ms));
        duration_ms -= chunk_ms;
    }
    return audio_is_running();
}

/**
 * Run the production notification path through the ES8311 speaker.
 *
 * The counters validate that the digital codec/I2S write path is operating.
 * Audible output is still a required physical observation; software cannot
 * detect a disconnected speaker or amplifier.
 */
esp_err_t hwtest_speaker(void)
{
    ESP_LOGI(TAG, "=== SPEAKER TEST ===");
    ESP_LOGI(TAG, "Playing production audio notifications for about %d seconds; verify audible output",
             SPEAKER_TEST_DURATION_SEC);

    audio_config_t config = AUDIO_CONFIG_DEFAULT();
    esp_err_t ret = hwtest_audio_start(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start production audio: %s", esp_err_to_name(ret));
        return ret;
    }

    const audio_notify_t sequence[] = {
        AUDIO_NOTIFY_STARTUP,
        AUDIO_NOTIFY_PEER_JOIN,
        AUDIO_NOTIFY_PEER_LEAVE,
    };
    for (size_t i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i) {
        ret = audio_play_notification(sequence[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue speaker notification: %s", esp_err_to_name(ret));
            (void)hwtest_audio_stop();
            return ret;
        }
    }

    (void)wait_for_audio((uint64_t)SPEAKER_TEST_DURATION_SEC * 1000u);
    audio_stats_t stats;
    ret = audio_get_stats(&stats);
    if (ret == ESP_OK) {
        const uint64_t expected = (uint64_t)SPEAKER_TEST_DURATION_SEC * AUDIO_FRAMES_PER_SEC;
        const uint32_t minimum = minimum_frames(expected, SPEAKER_MIN_FRAME_PERCENT);
        const uint32_t error_limit = maximum_errors(expected, MAX_IO_ERROR_PERCENT);
        if (stats.playback_frames < minimum || stats.i2s_write_incomplete > error_limit) {
            ESP_LOGE(TAG, "Digital speaker path invalid: playback=%lu minimum=%lu incomplete=%lu limit=%lu",
                     (unsigned long)stats.playback_frames, (unsigned long)minimum,
                     (unsigned long)stats.i2s_write_incomplete, (unsigned long)error_limit);
            ret = ESP_FAIL;
        }
    }
    if (ret != ESP_OK) {
        (void)hwtest_audio_stop();
        return ret;
    }
    ret = hwtest_audio_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop production audio: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "=== SPEAKER TEST DONE ===");
    return ESP_OK;
}

/**
 * Run production codec capture and report public audio pipeline statistics.
 */
esp_err_t hwtest_mic(void)
{
    ESP_LOGI(TAG, "=== MICROPHONE TEST ===");
    ESP_LOGI(TAG, "Capturing and encoding through the ES8311 for %d seconds",
             MIC_TEST_DURATION_SEC);
    ESP_LOGI(TAG, "Speak near the microphone during this five-second test");

    audio_config_t config = AUDIO_CONFIG_DEFAULT();
    config.force_tx_always = true;

    esp_err_t ret = hwtest_audio_start(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start production audio: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int second = 0; second < MIC_TEST_DURATION_SEC; ++second) {
        (void)wait_for_audio(1000u);

        audio_stats_t stats;
        ret = audio_get_stats(&stats);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read audio statistics: %s", esp_err_to_name(ret));
            (void)hwtest_audio_stop();
            return ret;
        }
        ESP_LOGI(TAG, "Audio stats: capture=%lu encoded=%lu capture_errors=%lu timeouts=%lu",
                 (unsigned long)stats.capture_frames_ok, (unsigned long)stats.frames_encoded,
                 (unsigned long)stats.capture_errors, (unsigned long)stats.capture_timeouts);
    }

    audio_stats_t stats;
    ret = audio_get_stats(&stats);
    if (ret == ESP_OK) {
        const uint64_t expected = (uint64_t)MIC_TEST_DURATION_SEC * AUDIO_FRAMES_PER_SEC;
        const uint32_t minimum = minimum_frames(expected, MIC_MIN_FRAME_PERCENT);
        const uint32_t error_limit = maximum_errors(expected, MAX_ERROR_PERCENT);
        if (stats.capture_frames_ok < minimum || stats.frames_encoded < minimum) {
            ESP_LOGE(TAG, "Too few mic frames: capture=%lu encode=%lu minimum=%lu",
                     (unsigned long)stats.capture_frames_ok, (unsigned long)stats.frames_encoded,
                     (unsigned long)minimum);
            ret = ESP_FAIL;
        } else if (stats.capture_peak_abs < MIC_MIN_PEAK_ABS) {
            ESP_LOGE(TAG, "Mic input peak too low: peak=%u minimum=%u; speak during the test",
                     stats.capture_peak_abs, MIC_MIN_PEAK_ABS);
            ret = ESP_FAIL;
        } else if (stats.capture_errors > error_limit || stats.capture_timeouts > error_limit ||
                   stats.encode_errors > error_limit) {
            ESP_LOGE(TAG, "Too many mic pipeline errors: capture=%lu timeout=%lu encode=%lu limit=%lu",
                     (unsigned long)stats.capture_errors, (unsigned long)stats.capture_timeouts,
                     (unsigned long)stats.encode_errors, (unsigned long)error_limit);
            ret = ESP_FAIL;
        }
    }

    esp_err_t cleanup_ret = hwtest_audio_stop();
    if (ret == ESP_OK && cleanup_ret != ESP_OK) {
        ret = cleanup_ret;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "=== MICROPHONE TEST FAILED: %s ===", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "=== MICROPHONE TEST DONE ===");
    return ESP_OK;
}

/**
 * Run the production codec capture -> Opus -> playout loopback path.
 */
esp_err_t hwtest_loopback(uint32_t duration_sec)
{
    ESP_LOGI(TAG, "=== LOOPBACK TEST ===");
    ESP_LOGI(TAG, "Codec capture -> Opus -> playout for %lu seconds", (unsigned long)duration_sec);
    ESP_LOGI(TAG, "(duration=0 runs until an external stop or deinit)");

    audio_config_t config = AUDIO_CONFIG_DEFAULT();
    config.mode = AUDIO_MODE_LOOPBACK;
    config.force_tx_always = true;

    esp_err_t ret = hwtest_audio_start(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start production audio: %s", esp_err_to_name(ret));
        return ret;
    }

    if (duration_sec == 0) {
        while (audio_is_running()) {
            (void)wait_for_audio(1000u);
        }
        /* An external stop ends this exclusive/manual test. Deinit only after
         * observing that the workers are already stopped. */
        ret = hwtest_audio_stop();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clean up externally stopped audio: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "=== LOOPBACK TEST STOPPED EXTERNALLY ===");
        return ESP_OK;
    }

    (void)wait_for_audio((uint64_t)duration_sec * 1000u);
    audio_stats_t stats;
    ret = audio_get_stats(&stats);
    if (ret == ESP_OK) {
        const uint64_t expected = (uint64_t)duration_sec * AUDIO_FRAMES_PER_SEC;
        const uint32_t minimum = minimum_frames(expected, LOOPBACK_MIN_FRAME_PERCENT);
        const uint32_t error_limit = maximum_errors(expected, MAX_ERROR_PERCENT);
        if (stats.capture_frames_ok < minimum || stats.frames_encoded < minimum ||
            stats.frames_decoded < minimum || stats.playback_frames < minimum) {
            ESP_LOGE(TAG,
                     "Too few loopback frames: capture=%lu encode=%lu decode=%lu playback=%lu "
                     "minimum=%lu",
                     (unsigned long)stats.capture_frames_ok, (unsigned long)stats.frames_encoded,
                     (unsigned long)stats.frames_decoded, (unsigned long)stats.playback_frames,
                     (unsigned long)minimum);
            ret = ESP_FAIL;
        } else if (stats.capture_peak_abs < LOOPBACK_MIN_PEAK_ABS) {
            ESP_LOGE(TAG, "Loopback input peak too low: peak=%u minimum=%u; provide non-zero input",
                     stats.capture_peak_abs, LOOPBACK_MIN_PEAK_ABS);
            ret = ESP_FAIL;
        } else if (stats.capture_errors > error_limit || stats.capture_timeouts > error_limit ||
                   stats.encode_errors > error_limit || stats.decode_errors > error_limit ||
                   stats.i2s_write_incomplete > error_limit) {
            ESP_LOGE(TAG,
                     "Too many loopback errors: capture=%lu timeout=%lu encode=%lu decode=%lu "
                     "playback=%lu limit=%lu",
                     (unsigned long)stats.capture_errors, (unsigned long)stats.capture_timeouts,
                     (unsigned long)stats.encode_errors, (unsigned long)stats.decode_errors,
                     (unsigned long)stats.i2s_write_incomplete, (unsigned long)error_limit);
            ret = ESP_FAIL;
        }
    }
    esp_err_t cleanup_ret = hwtest_audio_stop();
    if (ret == ESP_OK && cleanup_ret != ESP_OK) {
        ret = cleanup_ret;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "=== LOOPBACK TEST FAILED: %s ===", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "=== LOOPBACK TEST DONE ===");
    return ESP_OK;
}
