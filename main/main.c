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
#include "button.h"
#include "mesh.h"
#include "nvs_flash.h"
#include "power.h"
#include "uart_bridge.h"

static const char *TAG = "omi";

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Set to true to enable mesh mode, false for local loopback testing
 */
#define ENABLE_MESH_MODE 1

/* ============================================================================
 * State
 * ============================================================================ */

static bool s_mesh_active = false;

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
 * @brief Initialize NVS (required for persistent storage)
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
 * Audio <-> Transport Integration
 *
 * Runtime transport selection: nRF52840 (ESB via UART) or ESP-NOW (WiFi).
 * On boot, attempt UART detection. If nRF52840 responds, use it.
 * Otherwise, fallback to ESP-NOW mesh.
 * ============================================================================ */

typedef enum {
    TRANSPORT_NONE,
    TRANSPORT_ESP_NOW,  /* ESP-NOW mesh (WiFi) */
    TRANSPORT_NRF52840, /* nRF52840 ESB (via UART bridge) */
} transport_type_t;

static transport_type_t s_active_transport = TRANSPORT_NONE;

/**
 * @brief Callback from audio subsystem when encoded frame is ready
 */
static void audio_tx_callback(const uint8_t *data, uint16_t len, int64_t timestamp_us)
{
    (void)timestamp_us;

    switch (s_active_transport) {
    case TRANSPORT_ESP_NOW:
        if (mesh_get_state() == MESH_STATE_ACTIVE) {
            esp_err_t ret = mesh_send_audio(data, len);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Failed to queue audio for TX: %s", esp_err_to_name(ret));
            }
        }
        break;

    case TRANSPORT_NRF52840:
        if (uart_bridge_is_connected()) {
            esp_err_t ret = uart_bridge_send_audio(data, len);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Failed to send audio via UART: %s", esp_err_to_name(ret));
            }
        }
        break;

    default:
        /* No transport active */
        break;
    }
}

/**
 * @brief Callback from mesh subsystem when audio frame is received (ESP-NOW)
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
 * @brief Callback from UART bridge when audio is received (nRF52840)
 */
static void bridge_audio_callback(uint8_t src_id, const uint8_t *data, uint16_t len,
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
 * @brief Callback for mesh state changes (ESP-NOW)
 */
static void mesh_state_callback(mesh_state_t old_state, mesh_state_t new_state)
{
    const char *state_names[] = {"IDLE", "SCANNING", "JOINING", "ACTIVE"};

    ESP_LOGI(TAG, "Mesh state: %s -> %s", state_names[old_state], state_names[new_state]);

    if (new_state == MESH_STATE_ACTIVE) {
        mesh_role_t role = mesh_get_role();
        uint8_t node_id = mesh_get_node_id();
        int8_t slot = mesh_get_slot();

        power_set_state(POWER_STATE_MESH_IDLE);

        ESP_LOGI(TAG, "=== Mesh Active ===");
        ESP_LOGI(TAG, "  Role: %s", role == MESH_ROLE_COORDINATOR ? "COORDINATOR" : "PARTICIPANT");
        ESP_LOGI(TAG, "  Node ID: %u", node_id);
        ESP_LOGI(TAG, "  Slot: %d", slot);
        ESP_LOGI(TAG, "");
    }
}

/**
 * @brief Callback for peer join/leave events (ESP-NOW)
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

/**
 * @brief Callback for mesh events from nRF52840
 */
static void bridge_event_callback(uart_bridge_event_t event, const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;

    switch (event) {
    case BRIDGE_EVENT_MESH_READY:
        ESP_LOGI(TAG, "nRF52840 mesh ready");
        s_mesh_active = true;
        audio_play_notification(AUDIO_NOTIFY_MESH_ENABLED);
        break;
    case BRIDGE_EVENT_PEER_JOINED:
        ESP_LOGI(TAG, "Peer joined mesh");
        audio_play_notification(AUDIO_NOTIFY_PEER_JOIN);
        break;
    case BRIDGE_EVENT_PEER_LEFT:
        ESP_LOGI(TAG, "Peer left mesh");
        audio_play_notification(AUDIO_NOTIFY_PEER_LEAVE);
        break;
    default:
        break;
    }
}

/**
 * @brief Callback for button long press - toggles mesh on/off
 */
static void button_long_press_callback(int gpio)
{
    ESP_LOGI(TAG, "Button long press detected on GPIO %d - toggling mesh", gpio);

    if (s_mesh_active) {
        /* Disable mesh */
        ESP_LOGI(TAG, "Disabling mesh networking...");
        esp_err_t ret = ESP_OK;

        if (s_active_transport == TRANSPORT_ESP_NOW) {
            ret = mesh_stop();
        }
        /* For nRF52840, we could send a disable command, but for now just track state */

        if (ret == ESP_OK) {
            s_mesh_active = false;
            audio_play_notification(AUDIO_NOTIFY_MESH_DISABLED);
            ESP_LOGI(TAG, "Mesh disabled");
        } else {
            ESP_LOGE(TAG, "Failed to stop mesh: %s", esp_err_to_name(ret));
        }
    } else {
        /* Enable mesh */
        ESP_LOGI(TAG, "Enabling mesh networking...");
        esp_err_t ret = ESP_OK;

        if (s_active_transport == TRANSPORT_ESP_NOW) {
            ret = mesh_start();
        }
        /* For nRF52840, we could send an enable command */

        if (ret == ESP_OK) {
            s_mesh_active = true;
            audio_play_notification(AUDIO_NOTIFY_MESH_ENABLED);
            ESP_LOGI(TAG, "Mesh enabled");
        } else {
            ESP_LOGE(TAG, "Failed to start mesh: %s", esp_err_to_name(ret));
        }
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

    /* Initialize power management (Phase 3) */
    esp_err_t ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management: %s", esp_err_to_name(ret));
        goto error_halt;
    }
    ESP_LOGI(TAG, "[%" PRId64 " ms] Power management initialized", get_time_ms());

    /* Initialize button handler */
    ESP_LOGI(TAG, "");
    ret = button_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button: %s", esp_err_to_name(ret));
        goto error_halt;
    }
    ESP_LOGI(TAG, "[%" PRId64 " ms] Button handler initialized", get_time_ms());

#if ENABLE_MESH_MODE
    /* Detect mesh transport BEFORE audio init.
     * SPI slave needs a GDMA channel — if audio (I2S + ADC) initializes
     * first, it may exhaust all available DMA channels. */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Detecting mesh transport...");

    /* Try UART bridge first */
    ret = uart_bridge_init();
    /* Probe for nRF52840 (send ping and wait up to 2s, with retries) */
    if (ret == ESP_OK && uart_bridge_probe(2000)) {
        /* nRF52840 detected - use ESB via UART */
        s_active_transport = TRANSPORT_NRF52840;
        ESP_LOGI(TAG, "nRF52840 detected on UART - using ESB transport");

        uart_bridge_set_audio_callback(bridge_audio_callback);
        uart_bridge_set_event_callback(bridge_event_callback);
    } else {
        /* No nRF52840 - fallback to ESP-NOW */
        s_active_transport = TRANSPORT_ESP_NOW;
        ESP_LOGI(TAG, "nRF52840 not detected - using ESP-NOW transport");
    }

    /* Initialize audio subsystem (after SPI so DMA channels are available) */
    ESP_LOGI(TAG, "");
    ret = audio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }

    /* Configure audio for mesh mode */
    ret = audio_set_mode(AUDIO_MODE_MESH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set audio mode: %s", esp_err_to_name(ret));
        goto error_halt;
    }

    /* Register audio TX callback */
    audio_register_tx_callback(audio_tx_callback);

    /* Now initialize ESP-NOW mesh if needed (after audio) */
    if (s_active_transport == TRANSPORT_ESP_NOW) {
        ret = mesh_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize mesh: %s", esp_err_to_name(ret));
            goto error_halt;
        }

        mesh_register_audio_callback(mesh_audio_callback);
        mesh_register_state_callback(mesh_state_callback);
        mesh_register_peer_callback(mesh_peer_callback);
    }

    /* Register button callback for mesh toggle */
    button_register_long_press_callback(button_long_press_callback);
#else
    /* Non-mesh: initialize audio directly (no SPI contention) */
    ESP_LOGI(TAG, "");
    ret = audio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }
#endif /* ENABLE_MESH_MODE */

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

    /* Play startup sound (after I2S TX is enabled) */
    audio_play_notification(AUDIO_NOTIFY_STARTUP);

#if ENABLE_MESH_MODE
    /* Mesh networking - starts disabled, enabled by holding boot button for 2 seconds */
    ESP_LOGI(TAG, "Mesh networking ready (hold boot button for 2s to enable)");
    ESP_LOGI(TAG, "");
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
        if (s_mesh_active && mesh_is_initialized() && (now_ms - last_quick_stats) >= 10000) {
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

            const char *transport_str = (s_active_transport == TRANSPORT_NRF52840)  ? "ESB"
                                        : (s_active_transport == TRANSPORT_ESP_NOW) ? "ESP-NOW"
                                                                                    : "NONE";

            ESP_LOGI(
                TAG,
                "[STATS] Trans:%s | TX:%lu RX:%lu Lost:%lu (%.1f%%) | Enc:%lu Dec:%lu | Jitter:%u",
                transport_str, mesh_stats.audio_frames_tx, mesh_stats.audio_frames_rx,
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
            if (s_mesh_active && mesh_is_initialized()) {
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
            }
            if (s_active_transport == TRANSPORT_NRF52840) {
                ESP_LOGI(TAG, "  Transport: ESB via nRF52840");
                ESP_LOGI(TAG, "    Connected: %s", uart_bridge_is_connected() ? "yes" : "no");
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
