/**
 * @file main.c
 * @brief OMI - Open Motorcycle Intercom
 *
 * Main entry point for the ESP32-S3 firmware.
 * Phase 1: Audio pipeline with loopback (ADC -> HPF -> Opus -> I2S)
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "audio.h"
#include "hwtest.h"
#include "nvs_flash.h"

static const char *TAG = "omi";

/**
 * @brief Get monotonic timestamp in milliseconds
 */
static inline int64_t get_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/**
 * @brief Initialize NVS (required for WiFi/ESP-NOW and Bluetooth)
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    int64_t boot_time = get_time_ms();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "OMI - Open Motorcycle Intercom");
    ESP_LOGI(TAG, "Phase 1: Audio Pipeline Loopback");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Boot time: %" PRId64 " ms", boot_time);
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    /* Initialize NVS */
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "[%" PRId64 " ms] NVS initialized", get_time_ms());

    /* Initialize audio subsystem */
    ESP_LOGI(TAG, "");
    esp_err_t ret = audio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* Start audio pipeline */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Starting audio loopback...");
    ESP_LOGI(TAG, "Speak into the microphone - you should hear your voice with ~20-50ms delay");
    ESP_LOGI(TAG, "");

    ret = audio_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio: %s", esp_err_to_name(ret));
        audio_deinit();
        ESP_LOGE(TAG, "System halted");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Audio pipeline running!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Phase 1 Exit Criteria ===");
    ESP_LOGI(TAG, "1. Latency must be < 50 ms");
    ESP_LOGI(TAG, "2. No audio glitches over 30 minutes");
    ESP_LOGI(TAG, "3. Encode time: 5-10 ms");
    ESP_LOGI(TAG, "4. Decode time: 5-8 ms");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Monitor the audio stats logs every 10 seconds");
    ESP_LOGI(TAG, "========================================");

    /* Main loop - log system health every 60 seconds */
    int64_t last_health_check = get_time_ms();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int64_t now_ms = get_time_ms();
        if ((now_ms - last_health_check) >= 60000) {
            audio_stats_t stats;
            audio_get_stats(&stats);

            uint32_t free_heap = esp_get_free_heap_size();
            uint32_t min_heap = esp_get_minimum_free_heap_size();

            ESP_LOGI(TAG, "=== System Health ===");
            ESP_LOGI(TAG, "  Uptime: %" PRId64 " seconds", (now_ms - boot_time) / 1000);
            ESP_LOGI(TAG, "  Free heap: %lu bytes (min: %lu)", free_heap, min_heap);
            ESP_LOGI(TAG, "  Audio loops: %lu", stats.task_loops);
            ESP_LOGI(TAG, "  Phase 1 Status:");

            if (stats.latency_ms_avg < 50) {
                ESP_LOGI(TAG, "    ✓ Latency < 50ms (avg: %lu ms)", stats.latency_ms_avg);
            } else {
                ESP_LOGW(TAG, "    ✗ Latency >= 50ms (avg: %lu ms) - FAILS EXIT CRITERIA",
                         stats.latency_ms_avg);
            }

            if (stats.glitches_detected == 0) {
                ESP_LOGI(TAG, "    ✓ No glitches detected");
            } else {
                ESP_LOGW(TAG, "    ! %lu glitches detected", stats.glitches_detected);
            }

            if (stats.encode_time_us_avg >= 5000 && stats.encode_time_us_avg <= 10000) {
                ESP_LOGI(TAG, "    ✓ Encode time in range (avg: %lu us)", stats.encode_time_us_avg);
            } else {
                ESP_LOGI(TAG, "    i Encode time: %lu us (expected 5000-10000)",
                         stats.encode_time_us_avg);
            }

            if (stats.decode_time_us_avg >= 5000 && stats.decode_time_us_avg <= 8000) {
                ESP_LOGI(TAG, "    ✓ Decode time in range (avg: %lu us)", stats.decode_time_us_avg);
            } else {
                ESP_LOGI(TAG, "    i Decode time: %lu us (expected 5000-8000)",
                         stats.decode_time_us_avg);
            }

            ESP_LOGI(TAG, "");

            last_health_check = now_ms;
        }
    }

    /* Cleanup (unreachable in normal operation) */
    audio_stop();
    audio_deinit();
}
