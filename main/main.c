/**
 * @file main.c
 * @brief OMI - Open Motorcycle Intercom
 *
 * Main entry point for the ESP32-S3 firmware.
 * Phase 2: Single-hop RF link with TDMA mesh
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "audio.h"
#include "mesh.h"
#include "nvs_flash.h"

static const char *TAG = "omi";

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Set to true to enable mesh mode, false for local loopback testing
 */
#define ENABLE_MESH_MODE 1

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

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

/* ============================================================================
 * Audio <-> Mesh Integration
 * ============================================================================ */

/**
 * @brief Callback from audio subsystem when encoded frame is ready
 *
 * This is called from the audio task when a VOX-gated Opus frame
 * is ready for transmission. We queue it to the mesh for TX in
 * our assigned TDMA slot.
 */
static void audio_tx_callback(const uint8_t *data, uint16_t len, int64_t timestamp_us)
{
    (void)timestamp_us;

    if (mesh_get_state() != MESH_STATE_ACTIVE) {
        return;
    }

    esp_err_t ret = mesh_send_audio(data, len);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to queue audio for TX: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback from mesh subsystem when audio frame is received
 *
 * This is called from the mesh task when an audio packet is received
 * from another node. We decode it and queue for playback.
 */
static void mesh_audio_callback(const uint8_t *data, uint16_t len, uint8_t src_id,
                                int64_t timestamp_us)
{
    audio_frame_t frame;

    if (len > sizeof(frame.data)) {
        ESP_LOGW(TAG, "Audio frame too large: %u bytes", len);
        return;
    }

    memcpy(frame.data, data, len);
    frame.len = len;
    frame.timestamp_ms = timestamp_us / 1000;

    esp_err_t ret = audio_put_rx_frame(&frame, src_id);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to queue RX audio: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback for mesh state changes
 */
static void mesh_state_callback(mesh_state_t old_state, mesh_state_t new_state)
{
    const char *state_names[] = {"IDLE", "SCANNING", "JOINING", "ACTIVE"};

    ESP_LOGI(TAG, "Mesh state: %s -> %s", state_names[old_state], state_names[new_state]);

    if (new_state == MESH_STATE_ACTIVE) {
        mesh_role_t role = mesh_get_role();
        uint8_t node_id = mesh_get_node_id();
        int8_t slot = mesh_get_slot();

        ESP_LOGI(TAG, "=== Mesh Active ===");
        ESP_LOGI(TAG, "  Role: %s", role == MESH_ROLE_COORDINATOR ? "COORDINATOR" : "PARTICIPANT");
        ESP_LOGI(TAG, "  Node ID: %u", node_id);
        ESP_LOGI(TAG, "  Slot: %d", slot);
        ESP_LOGI(TAG, "");
    }
}

/**
 * @brief Callback for peer join/leave events
 */
static void mesh_peer_callback(const mesh_peer_info_t *peer, bool joined)
{
    if (joined) {
        ESP_LOGI(TAG, "Peer JOINED: node_id=%u, slot=%d, MAC=" MACSTR, peer->node_id,
                 peer->slot_index, MAC2STR(peer->mac_addr));
    } else {
        ESP_LOGI(TAG, "Peer LEFT: node_id=%u", peer->node_id);
    }
}

/* ============================================================================
 * Main Application
 * ============================================================================ */

void app_main(void)
{
    int64_t boot_time = get_time_ms();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "OMI - Open Motorcycle Intercom");
#if ENABLE_MESH_MODE
    ESP_LOGI(TAG, "Phase 2: Single-Hop RF Link");
#else
    ESP_LOGI(TAG, "Phase 1: Audio Pipeline Loopback");
#endif
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
        goto error_halt;
    }

#if ENABLE_MESH_MODE
    /* Configure audio for mesh mode */
    ret = audio_set_mode(AUDIO_MODE_MESH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set audio mode: %s", esp_err_to_name(ret));
        goto error_halt;
    }

    /* Register audio TX callback */
    audio_register_tx_callback(audio_tx_callback);

    /* Initialize mesh subsystem */
    ESP_LOGI(TAG, "");
    ret = mesh_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh: %s", esp_err_to_name(ret));
        goto error_halt;
    }

    /* Register mesh callbacks */
    mesh_register_audio_callback(mesh_audio_callback);
    mesh_register_state_callback(mesh_state_callback);
    mesh_register_peer_callback(mesh_peer_callback);
#endif

    /* Start audio pipeline */
    ESP_LOGI(TAG, "");
#if ENABLE_MESH_MODE
    ESP_LOGI(TAG, "Starting audio pipeline (mesh mode)...");
#else
    ESP_LOGI(TAG, "Starting audio loopback...");
    ESP_LOGI(TAG, "Speak into the microphone - you should hear your voice with ~20-50ms delay");
#endif
    ESP_LOGI(TAG, "");

    ret = audio_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }

#if ENABLE_MESH_MODE
    /* Start mesh networking */
    ESP_LOGI(TAG, "Starting mesh networking...");
    ESP_LOGI(TAG, "Scanning for existing mesh (2 seconds)...");
    ESP_LOGI(TAG, "");

    ret = mesh_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start mesh: %s", esp_err_to_name(ret));
        goto error_halt;
    }
#endif

    ESP_LOGI(TAG, "System running!");
    ESP_LOGI(TAG, "");

#if ENABLE_MESH_MODE
    ESP_LOGI(TAG, "=== Phase 2 Exit Criteria ===");
    ESP_LOGI(TAG, "1. Clear voice at 50-100m LOS");
    ESP_LOGI(TAG, "2. Packet loss < 10%% sustained");
    ESP_LOGI(TAG, "3. Dynamic join/leave working");
#else
    ESP_LOGI(TAG, "=== Phase 1 Exit Criteria ===");
    ESP_LOGI(TAG, "1. Latency must be < 50 ms");
    ESP_LOGI(TAG, "2. No audio glitches over 30 minutes");
#endif
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");

    /* Main loop - log system health periodically
     * - Quick stats every 10 seconds for active debugging
     * - Full health check every 60 seconds
     */
    int64_t last_health_check = get_time_ms();
    int64_t last_quick_stats = get_time_ms();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int64_t now_ms = get_time_ms();

#if ENABLE_MESH_MODE
        /* Quick stats every 10 seconds for Phase 2 validation */
        if ((now_ms - last_quick_stats) >= 10000) {
            mesh_stats_t mesh_stats;
            mesh_get_stats(&mesh_stats);
            audio_stats_t audio_stats;
            audio_get_stats(&audio_stats);

            /* Calculate packet loss rate */
            uint32_t total_expected = mesh_stats.audio_frames_rx + mesh_stats.audio_frames_lost;
            float loss_pct = 0.0f;
            if (total_expected > 0) {
                loss_pct = (float)mesh_stats.audio_frames_lost / total_expected * 100.0f;
            }

            ESP_LOGI(TAG, "[STATS] TX:%lu RX:%lu Lost:%lu (%.1f%%) | Enc:%lu Dec:%lu | Jitter:%u",
                     mesh_stats.audio_frames_tx, mesh_stats.audio_frames_rx,
                     mesh_stats.audio_frames_lost, loss_pct, audio_stats.frames_encoded,
                     audio_stats.frames_decoded, mesh_stats.jitter_depth);

            last_quick_stats = now_ms;
        }
#endif

        if ((now_ms - last_health_check) >= 60000) {
            audio_stats_t audio_stats;
            audio_get_stats(&audio_stats);

            uint32_t free_heap = esp_get_free_heap_size();
            uint32_t min_heap = esp_get_minimum_free_heap_size();

            ESP_LOGI(TAG, "=== System Health ===");
            ESP_LOGI(TAG, "  Uptime: %" PRId64 " seconds", (now_ms - boot_time) / 1000);
            ESP_LOGI(TAG, "  Free heap: %lu bytes (min: %lu)", free_heap, min_heap);
            ESP_LOGI(TAG, "  Audio loops: %lu", audio_stats.task_loops);

#if ENABLE_MESH_MODE
            mesh_stats_t mesh_stats;
            mesh_get_stats(&mesh_stats);

            ESP_LOGI(TAG, "  Mesh Status:");
            ESP_LOGI(TAG, "    Role: %s",
                     mesh_get_role() == MESH_ROLE_COORDINATOR   ? "COORDINATOR"
                     : mesh_get_role() == MESH_ROLE_PARTICIPANT ? "PARTICIPANT"
                                                                : "NONE");
            ESP_LOGI(TAG, "    Nodes: %u", mesh_get_node_count());
            ESP_LOGI(TAG, "    Frame: %lu", mesh_get_frame_counter());
            ESP_LOGI(TAG, "    TX: %lu packets, RX: %lu packets", mesh_stats.packets_tx,
                     mesh_stats.packets_rx);
            ESP_LOGI(TAG, "    Audio TX: %lu, RX: %lu, Lost: %lu", mesh_stats.audio_frames_tx,
                     mesh_stats.audio_frames_rx, mesh_stats.audio_frames_lost);
            ESP_LOGI(TAG, "    Jitter depth: %u, underruns: %lu", mesh_stats.jitter_depth,
                     mesh_stats.jitter_underruns);

            /* Calculate packet loss rate */
            uint32_t total_expected = mesh_stats.audio_frames_rx + mesh_stats.audio_frames_lost;
            if (total_expected > 0) {
                float loss_pct = (float)mesh_stats.audio_frames_lost / total_expected * 100.0f;
                ESP_LOGI(TAG, "    Packet loss: %.1f%%", loss_pct);

                if (loss_pct < 10.0f) {
                    ESP_LOGI(TAG, "    ✓ Loss < 10%% - EXIT CRITERIA MET");
                } else {
                    ESP_LOGW(TAG, "    ✗ Loss >= 10%% - FAILS EXIT CRITERIA");
                }
            }
#else
            ESP_LOGI(TAG, "  Audio Stats:");
            if (audio_stats.latency_ms_avg < 50) {
                ESP_LOGI(TAG, "    ✓ Latency < 50ms (avg: %lu ms)", audio_stats.latency_ms_avg);
            } else {
                ESP_LOGW(TAG, "    ✗ Latency >= 50ms (avg: %lu ms)", audio_stats.latency_ms_avg);
            }

            if (audio_stats.glitches_detected == 0) {
                ESP_LOGI(TAG, "    ✓ No glitches detected");
            } else {
                ESP_LOGW(TAG, "    ! %lu glitches detected", audio_stats.glitches_detected);
            }
#endif

            ESP_LOGI(TAG, "");
            last_health_check = now_ms;
        }
    }

    /* Cleanup (unreachable in normal operation) */
#if ENABLE_MESH_MODE
    mesh_stop();
    mesh_deinit();
#endif
    audio_stop();
    audio_deinit();
    return;

error_halt:
    ESP_LOGE(TAG, "System halted due to initialization error");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
