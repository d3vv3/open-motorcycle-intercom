/**
 * @file main.c
 * @brief OMI - Open Motorcycle Intercom
 *
 * Main entry point for the ESP32-S3 firmware.
 * Phase 0: Basic initialization and logging framework.
 */

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "audio.h"
#include "mesh.h"

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
    ESP_LOGI(TAG, "Boot time: %" PRId64 " ms", boot_time);
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    // Initialize NVS
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "[%" PRId64 " ms] NVS initialized", get_time_ms());

    // Initialize audio subsystem
    esp_err_t ret = audio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[%" PRId64 " ms] Audio initialized", get_time_ms());
    }

    // Initialize mesh subsystem
    ret = mesh_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mesh init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[%" PRId64 " ms] Mesh initialized", get_time_ms());
    }

    ESP_LOGI(TAG, "[%" PRId64 " ms] System ready", get_time_ms());

    // Main loop - placeholder for future state machine
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
