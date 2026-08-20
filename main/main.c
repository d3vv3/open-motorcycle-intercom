/**
 * @file main.c
 * @brief OMI - Open Motorcycle Intercom
 *
 * Boot orchestration for the ESP32-S3 firmware: transport detection,
 * subsystem wiring, the mesh toggle button, and the periodic health loop.
 */

#include <inttypes.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "app_state.h"
#include "audio.h"
#include "button.h"
#include "e2e_diag.h"
#include "mesh.h"
#include "mesh_intent.h"
#include "nvs_flash.h"
#include "power.h"
#include "rtt_probe.h"
#include "transport_espnow.h"
#include "transport_nrf.h"
#include "uart_bridge.h"

static const char *TAG = "omi";

/* Debug instrumentation knobs */
#define REDUCED_LOGGING_MODE 1

/* Test knob: bypass VOX gating and always transmit microphone frames.
 * 0 = normal VOX behavior (DTX silence suppression active), 1 = force continuous TX. */
#define FORCE_TX_ALWAYS_FOR_TEST 1

/* RTT log cadence while using nRF transport */
#define RTT_LOG_INTERVAL_MS 10000

_Atomic bool g_mesh_active = false;

/*
 * Runtime transport selection: nRF52840 (ESB via SPI bridge) or ESP-NOW (WiFi).
 * On boot, attempt SPI-bridge detection. If nRF52840 responds, use it.
 * Otherwise, fallback to ESP-NOW mesh.
 */

typedef enum {
    TRANSPORT_NONE,
    TRANSPORT_ESP_NOW,  /* ESP-NOW mesh (WiFi) */
    TRANSPORT_NRF52840, /* nRF52840 ESB via SPI bridge */
} transport_type_t;

static transport_type_t s_active_transport = TRANSPORT_NONE;

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

static void disable_esp_radios_for_nrf_transport(void)
{
#if CONFIG_ESP_WIFI_ENABLED
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT && ret != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "WiFi stop failed during nRF handoff: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "WiFi deinit failed during nRF handoff: %s", esp_err_to_name(ret));
    }

    ret = esp_event_loop_delete_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Event loop delete failed during nRF handoff: %s", esp_err_to_name(ret));
    }

    ret = esp_netif_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Netif deinit failed during nRF handoff: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "ESP WiFi/ESP-NOW disabled for nRF transport");
#else
    ESP_LOGI(TAG, "ESP WiFi disabled in build config");
#endif

#if CONFIG_BT_ENABLED
    ESP_LOGW(TAG, "Bluetooth support is enabled in this build; no runtime BT stack is started");
#else
    ESP_LOGI(TAG, "ESP Bluetooth disabled in build config");
#endif
}

static esp_err_t init_audio_with_test_flags(void)
{
    audio_config_t audio_cfg = AUDIO_CONFIG_DEFAULT();
    audio_cfg.force_tx_always = (FORCE_TX_ALWAYS_FOR_TEST != 0);
    return audio_init_with_config(&audio_cfg);
}

/**
 * @brief Callback from audio subsystem when encoded frame is ready
 */
static void audio_tx_callback(const uint8_t *data, uint16_t len, bool active, int64_t timestamp_us)
{
    switch (s_active_transport) {
    case TRANSPORT_ESP_NOW:
        transport_espnow_send_audio(data, len, active);
        break;

    case TRANSPORT_NRF52840:
        transport_nrf_send_audio(data, len, active, timestamp_us);
        break;

    default:
        /* No transport active */
        break;
    }
}

static void audio_activity_callback(bool active)
{
    if (active) {
        power_notify_voice_start();
    } else {
        power_notify_voice_end();
    }
}

static void disable_mesh_from_button(void)
{
    ESP_LOGI(TAG, "Disabling mesh networking...");

    esp_err_t ret = ESP_OK;
    if (s_active_transport == TRANSPORT_ESP_NOW) {
        ret = mesh_stop();
    }
    /* The nRF transport is reconciled from app_main, never from this callback. */

    if (ret == ESP_OK) {
        atomic_store(&g_mesh_active, false);
        transport_nrf_cancel_enable_notification();
        transport_nrf_reset_membership_tracking();
        audio_clear_rx_frames();
        (void)audio_play_notification(AUDIO_NOTIFY_MESH_DISABLED);
        ESP_LOGI(TAG, "Mesh disabled");
    } else {
        transport_nrf_set_user_enabled(true);
        transport_nrf_cancel_enable_notification();
        esp_err_t persist_ret = mesh_intent_persist(true);
        if (persist_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore enabled mesh intent: %s",
                     esp_err_to_name(persist_ret));
        }
        ESP_LOGE(TAG, "Failed to stop mesh: %s", esp_err_to_name(ret));
    }
}

static void enable_mesh_from_button(void)
{
    ESP_LOGI(TAG, "Enabling mesh networking...");
    atomic_store(&g_mesh_active, false);

    esp_err_t ret = ESP_OK;
    if (s_active_transport == TRANSPORT_ESP_NOW) {
        ret = mesh_start();
    }
    /* The nRF transport is reconciled from app_main, never from this callback. */

    if (ret == ESP_OK) {
        if (s_active_transport == TRANSPORT_ESP_NOW) {
            atomic_store(&g_mesh_active, true);
            transport_nrf_cancel_enable_notification();
            (void)audio_play_notification(AUDIO_NOTIFY_MESH_ENABLED);
        }
        ESP_LOGI(TAG, "Mesh enable accepted; waiting for active state");
    } else {
        transport_nrf_set_user_enabled(false);
        esp_err_t persist_ret = mesh_intent_persist(false);
        if (persist_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore disabled mesh intent: %s",
                     esp_err_to_name(persist_ret));
        }
        atomic_store(&g_mesh_active, false);
        ESP_LOGE(TAG, "Failed to start mesh: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback for button long press - toggles mesh on/off
 */
static void button_long_press_callback(int gpio)
{
    ESP_LOGI(TAG, "Button long press detected on GPIO %d - toggling mesh", gpio);

    bool requested_enabled = !mesh_intent_enabled();
    esp_err_t ret = mesh_intent_persist(requested_enabled);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist mesh intent: %s", esp_err_to_name(ret));
        return;
    }
    transport_nrf_set_user_enabled(requested_enabled);
    transport_nrf_reset_reconciliation();

    if (requested_enabled) {
        enable_mesh_from_button();
    } else {
        disable_mesh_from_button();
    }
}

/* ============================================================================
 * Main Application
 * ============================================================================ */

static void log_quick_stats(void)
{
    mesh_stats_t mesh_stats;
    mesh_get_stats(&mesh_stats);
    audio_stats_t audio_stats;
    audio_get_stats(&audio_stats);

    uint32_t total_expected = mesh_stats.audio_frames_rx + mesh_stats.audio_frames_lost;
    float loss_pct = 0.0f;
    if (total_expected > 0) {
        loss_pct = (float)mesh_stats.audio_frames_lost / total_expected * 100.0f;
    }

    const char *transport_str = (s_active_transport == TRANSPORT_NRF52840)  ? "ESB"
                                : (s_active_transport == TRANSPORT_ESP_NOW) ? "ESP-NOW"
                                                                            : "NONE";

    ESP_LOGI(TAG,
             "[STATS] Trans:%s | TX:%lu RX:%lu Lost:%lu (%.1f%%) | Enc:%lu Dec:%lu | Jitter:%u",
             transport_str, mesh_stats.audio_frames_tx, mesh_stats.audio_frames_rx,
             mesh_stats.audio_frames_lost, loss_pct, audio_stats.frames_encoded,
             audio_stats.frames_decoded, mesh_stats.jitter_depth);

    if (s_active_transport == TRANSPORT_NRF52840) {
        rtt_probe_stats_t rtt = {0};
        rtt_probe_get_stats(&rtt);
        ESP_LOGI(TAG, "[RTT] sent=%lu recv=%lu lost=%lu rtt=%lums/%lums jit=%lums/%lums", rtt.sent,
                 rtt.recv, rtt.lost, rtt.rtt_ms_avg, rtt.rtt_ms_max, rtt.jitter_ms_avg,
                 rtt.jitter_ms_max);
    }
}

static void log_system_health(int64_t now_ms, int64_t boot_time)
{
    audio_stats_t audio_stats;
    audio_get_stats(&audio_stats);

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

    ESP_LOGI(TAG, "=== System Health ===");
    ESP_LOGI(TAG, "  Uptime: %" PRId64 " seconds", (now_ms - boot_time) / 1000);
    ESP_LOGI(TAG, "  Free heap: %lu bytes (min: %lu)", free_heap, min_heap);
    ESP_LOGI(TAG, "  Audio loops: %lu", audio_stats.task_loops);

    if (g_mesh_active && mesh_is_initialized()) {
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
        }
    }
    if (s_active_transport == TRANSPORT_NRF52840) {
        ESP_LOGI(TAG, "  Transport: ESB via nRF52840");
        ESP_LOGI(TAG, "    Connected: %s", uart_bridge_is_connected() ? "yes" : "no");
    }

    ESP_LOGI(TAG, "");
}

static void select_transport(void)
{
    /* Detect mesh transport BEFORE audio init.
     * SPI slave needs a GDMA channel - if audio (I2S + ADC) initializes
     * first, it may exhaust all available DMA channels. */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Detecting mesh transport...");

    /* Try the nRF SPI bridge first. */
    esp_err_t ret = uart_bridge_init();
    /* Probe for nRF52840 (send ping and wait up to 2s, with retries) */
    if (ret == ESP_OK && uart_bridge_probe(2000)) {
        /* nRF52840 detected: use ESB via SPI bridge. */
        s_active_transport = TRANSPORT_NRF52840;
        ESP_LOGI(TAG, "nRF52840 detected on SPI bridge - using ESB transport");

        transport_nrf_attach();
        disable_esp_radios_for_nrf_transport();
    } else {
        /* No nRF52840 - fallback to ESP-NOW */
        uart_bridge_deinit();
        transport_nrf_reset_tx_cache();
        s_active_transport = TRANSPORT_ESP_NOW;
        ESP_LOGI(TAG, "nRF52840 not detected - using ESP-NOW transport");
    }
}

static esp_err_t initialize_application(int64_t boot_time)
{
    rtt_probe_init();

    ESP_LOGI(TAG, "OMI - Open Motorcycle Intercom");
    ESP_LOGI(TAG, "Boot time: %" PRId64 " ms", boot_time);
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());

    /* Initialize NVS */
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "[%" PRId64 " ms] NVS initialized", get_time_ms());
    ESP_ERROR_CHECK(mesh_intent_load());
    ESP_LOGI(TAG, "Persisted mesh intent: %s", mesh_intent_enabled() ? "enabled" : "disabled");
    ESP_ERROR_CHECK(transport_nrf_init());

    /* Initialize power management. */
    esp_err_t ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "[%" PRId64 " ms] Power management initialized", get_time_ms());

    /* Initialize button handler */
    ESP_LOGI(TAG, "");
    ret = button_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "[%" PRId64 " ms] Button handler initialized", get_time_ms());

    select_transport();

    /* Initialize audio subsystem (after SPI so DMA channels are available) */
    ESP_LOGI(TAG, "");
    ret = init_audio_with_test_flags();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Audio test flags: force_tx_always=%s", FORCE_TX_ALWAYS_FOR_TEST ? "YES" : "no");

    /* Configure audio for mesh mode */
    ret = audio_set_mode(AUDIO_MODE_MESH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set audio mode: %s", esp_err_to_name(ret));
        return ret;
    }

    audio_register_tx_callback(audio_tx_callback);
    audio_register_activity_callback(audio_activity_callback);

    /* Now initialize ESP-NOW mesh if needed (after audio) */
    if (s_active_transport == TRANSPORT_ESP_NOW) {
        ret = transport_espnow_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize mesh: %s", esp_err_to_name(ret));
            return ret;
        }

        if (mesh_intent_enabled()) {
            ret = mesh_start();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore persisted mesh intent: %s", esp_err_to_name(ret));
                return ret;
            }
            atomic_store(&g_mesh_active, true);
        }
    }

    /* Register button callback for mesh toggle */
    button_register_long_press_callback(button_long_press_callback);

    /* Start audio pipeline */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Starting audio pipeline (mesh mode)...");
    ESP_LOGI(TAG, "");

    /* Queue startup notification; the audio task owns generation and I2S playback. */
    ret = audio_play_notification(AUDIO_NOTIFY_STARTUP);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to play startup notification: %s", esp_err_to_name(ret));
    }

    ret = audio_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio: %s", esp_err_to_name(ret));
        return ret;
    }

    /* The persisted user policy is reconciled with the selected transport. */
    if (s_active_transport == TRANSPORT_NRF52840) {
        atomic_store(&g_mesh_active, false);
    }
    ESP_LOGI(TAG, "Mesh networking ready (desired state: %s)",
             mesh_intent_enabled() ? "enabled" : "disabled");
    ESP_LOGI(TAG, "");

    ESP_LOGI(TAG, "System running!");
    ESP_LOGI(TAG, "");

    return ESP_OK;
}

static void run_runtime_health_loop(int64_t boot_time)
{
    /* Main loop - log system health periodically */
    int64_t last_health_check = get_time_ms();
    int64_t last_quick_stats = get_time_ms();

#if REDUCED_LOGGING_MODE
    const int64_t quick_stats_interval_ms = 20000;
    const int64_t health_interval_ms = 120000;
#else
    const int64_t quick_stats_interval_ms = 10000;
    const int64_t health_interval_ms = 60000;
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int64_t now_ms = get_time_ms();

        if (g_mesh_active && mesh_is_initialized() &&
            (now_ms - last_quick_stats) >= quick_stats_interval_ms) {
            log_quick_stats();
            last_quick_stats = now_ms;
        }

        if (s_active_transport == TRANSPORT_NRF52840) {
            static int64_t last_rtt_log_ms = 0;

            transport_nrf_tick(now_ms);
            rtt_probe_tick(now_ms, g_mesh_active, uart_bridge_is_connected());

            if ((now_ms - last_rtt_log_ms) >= RTT_LOG_INTERVAL_MS) {
                rtt_probe_stats_t rtt = {0};
                rtt_probe_get_stats(&rtt);

                ESP_LOGI(TAG, "[RTT] sent=%lu recv=%lu lost=%lu rtt=%lums/%lums jit=%lums/%lums",
                         rtt.sent, rtt.recv, rtt.lost, rtt.rtt_ms_avg, rtt.rtt_ms_max,
                         rtt.jitter_ms_avg, rtt.jitter_ms_max);
                e2e_diag_log(transport_nrf_node_id());
                last_rtt_log_ms = now_ms;
            }
        }

        if ((now_ms - last_health_check) >= health_interval_ms) {
            log_system_health(now_ms, boot_time);
            last_health_check = now_ms;
        }
    }
}

void app_main(void)
{
    int64_t boot_time = get_time_ms();

    if (initialize_application(boot_time) != ESP_OK) {
        goto error_halt;
    }

    run_runtime_health_loop(boot_time);

    /* Cleanup (unreachable in normal operation) */
    mesh_stop();
    mesh_deinit();
    audio_stop();
    audio_deinit();
    return;

error_halt:
    ESP_LOGE(TAG, "System halted due to initialization error");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
