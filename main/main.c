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
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "app_state.h"
#include "audio.h"
#include "button.h"
#include "cpu_profile.h"
#include "e2e_diag.h"
#include "mesh.h"
#include "mesh_intent.h"
#include "media_toggle_guard.h"
#include "nvs_flash.h"
#include "power.h"
#include "phone_audio.h"
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
static atomic_bool s_short_press_pending = ATOMIC_VAR_INIT(false);
static atomic_bool s_pairing_pending = ATOMIC_VAR_INIT(false);
static media_toggle_guard_t s_media_toggle_guard;

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

static void disable_esp_wifi_for_nrf_transport(void)
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
    ESP_LOGI(TAG, "Bluetooth remains available for phone audio");
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
 * @brief Handle release-classified BOOT gestures.
 */
static void button_gesture_callback(button_gesture_t gesture, int gpio)
{
    if (gesture == BUTTON_GESTURE_BLUETOOTH_PAIRING) {
        atomic_store_explicit(&s_pairing_pending, true, memory_order_release);
        return;
    }
    if (gesture == BUTTON_GESTURE_SHORT_PRESS) {
        atomic_store_explicit(&s_short_press_pending, true, memory_order_release);
        return;
    }
    if (gesture != BUTTON_GESTURE_MESH_TOGGLE) return;

    ESP_LOGI(TAG, "BOOT mesh gesture detected on GPIO %d - toggling mesh", gpio);

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

static void process_pairing_request(void)
{
    esp_err_t ret = phone_audio_set_discoverable(true);
    if (ret == ESP_OK) {
        (void)audio_play_notification(AUDIO_NOTIFY_BLUETOOTH_PAIRING);
        ESP_LOGI(TAG, "BOOT gesture opened Bluetooth pairing window for 120 seconds");
    } else {
        ESP_LOGW(TAG, "BOOT pairing gesture unavailable: %s", esp_err_to_name(ret));
    }
}

static void process_media_toggle(bool request_pending, int64_t now_ms)
{
    phone_audio_state_t phone_state;
    if (phone_audio_get_state(&phone_state) != ESP_OK) {
        media_toggle_guard_reset(&s_media_toggle_guard);
        return;
    }

    bool valid = phone_state.call.phase == PHONE_AUDIO_CALL_PHASE_IDLE &&
                 phone_state.a2dp_connected && phone_state.avrcp_connected;
    now_ms = get_time_ms();
    media_toggle_decision_t decision = media_toggle_guard_step(
        &s_media_toggle_guard, request_pending, valid, phone_state.media_streaming,
        phone_state.media_transition_ms, now_ms);
    if (decision == MEDIA_TOGGLE_DROPPED) {
        ESP_LOGW(TAG, "BOOT media toggle dropped: A2DP state transition was not observed");
    } else if (decision == MEDIA_TOGGLE_DISPATCH) {
        bool sent_streaming = phone_state.media_streaming;
        esp_err_t ret = sent_streaming ? phone_audio_pause() : phone_audio_play();
        bool arm_guard = ret != ESP_ERR_NO_MEM;
        media_toggle_guard_command_result(&s_media_toggle_guard, arm_guard,
                                          sent_streaming, get_time_ms());
        ESP_LOGI(TAG, "BOOT short press: %s media (%s)",
                 sent_streaming ? "pause" : "play", esp_err_to_name(ret));
    }
}

static void process_short_press(int64_t now_ms)
{
    phone_audio_state_t phone_state;
    esp_err_t ret = phone_audio_get_state(&phone_state);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BOOT short press state unavailable: %s", esp_err_to_name(ret));
        return;
    }

    switch (phone_state.call.phase) {
        case PHONE_AUDIO_CALL_PHASE_INCOMING:
            media_toggle_guard_reset(&s_media_toggle_guard);
            ret = phone_audio_answer_call();
            ESP_LOGI(TAG, "BOOT short press: answer call (%s)", esp_err_to_name(ret));
            return;
        case PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING:
        case PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING:
        case PHONE_AUDIO_CALL_PHASE_ACTIVE:
        case PHONE_AUDIO_CALL_PHASE_HELD:
            media_toggle_guard_reset(&s_media_toggle_guard);
            ret = phone_audio_reject_call();
            ESP_LOGI(TAG, "BOOT short press: end call (%s)", esp_err_to_name(ret));
            return;
        case PHONE_AUDIO_CALL_PHASE_IDLE:
            break;
    }

    if (phone_audio_get_call_state(&phone_state.call) != ESP_OK ||
        phone_state.call.phase != PHONE_AUDIO_CALL_PHASE_IDLE) {
        media_toggle_guard_reset(&s_media_toggle_guard);
        ESP_LOGI(TAG, "BOOT short press: media action skipped because call is no longer idle");
        return;
    }
    process_media_toggle(true, now_ms);
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

#if defined(APP_S31_LC3_WIRE)
    /* Try the nRF SPI bridge first. */
    esp_err_t ret = uart_bridge_init();
    bool probed = ret == ESP_OK && uart_bridge_probe(2000);
    uart_bridge_status_t status = {0};
    bool fresh_status = probed && uart_bridge_get_status(&status) == ESP_OK;
    if (fresh_status && bridge_audio_supports_lc3(status.protocol_version, status.audio_codec,
                                                   status.audio_frame_ms)) {
        s_active_transport = TRANSPORT_NRF52840;
        ESP_LOGI(TAG, "Using nRF ESB transport: bridge protocol=%u codec=LC3(%u) frame=%u ms",
                 status.protocol_version, status.audio_codec, status.audio_frame_ms);

        transport_nrf_attach();
        disable_esp_wifi_for_nrf_transport();
    } else {
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Using ESP-NOW: SPI bridge init failed: %s", esp_err_to_name(ret));
        } else if (!fresh_status) {
            ESP_LOGW(TAG, "Using ESP-NOW: no fresh nRF status after probe");
        } else {
            ESP_LOGW(TAG, "Using ESP-NOW: incompatible nRF bridge protocol=%u codec=%u frame=%u ms (need v%u LC3/20 ms)",
                     status.protocol_version, status.audio_codec, status.audio_frame_ms,
                     BRIDGE_PROTOCOL_VERSION);
        }
        uart_bridge_deinit();
        transport_nrf_reset_tx_cache();
        s_active_transport = TRANSPORT_ESP_NOW;
    }
#else
    s_active_transport = TRANSPORT_ESP_NOW;
    ESP_LOGI(TAG, "Using ESP-NOW: nRF LC3 transport requires S31_LC3_WIRE=ON");
#endif
}

static esp_err_t initialize_application(int64_t boot_time)
{
    rtt_probe_init();

    ESP_LOGI(TAG, "OMI - Open Motorcycle Intercom");
    ESP_LOGI(TAG, "Boot time: %" PRId64 " ms", boot_time);
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    cpu_profile_init();

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

    /* Reserve Classic Bluetooth controller memory before Wi-Fi pools and audio stacks. */
    ESP_LOGI(TAG, "Before phone_audio_init: internal free/largest=%lu/%lu bytes, PSRAM free=%lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    esp_err_t phone_ret = phone_audio_init();
    if (phone_ret != ESP_OK) {
        ESP_LOGW(TAG, "Phone audio unavailable; continuing without Bluetooth: %s",
                 esp_err_to_name(phone_ret));
    }

    /* Now initialize ESP-NOW mesh if needed (after audio) */
    if (s_active_transport == TRANSPORT_ESP_NOW) {
        ret = transport_espnow_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize mesh: %s", esp_err_to_name(ret));
            if (phone_ret == ESP_OK) (void)phone_audio_deinit();
            return ret;
        }

        if (mesh_intent_enabled()) {
            ret = mesh_start();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore persisted mesh intent: %s", esp_err_to_name(ret));
                if (phone_ret == ESP_OK) (void)phone_audio_deinit();
                return ret;
            }
            atomic_store(&g_mesh_active, true);
        }
    }

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
        if (phone_ret == ESP_OK) (void)phone_audio_deinit();
        return ret;
    }
    button_register_gesture_callback(button_gesture_callback);

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
    int64_t last_audio_stats = get_time_ms();

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

        bool short_press = atomic_exchange_explicit(&s_short_press_pending, false,
                                                    memory_order_acq_rel);
        if (short_press) process_short_press(now_ms);
        else process_media_toggle(false, now_ms);

        /* When both intents are pending, call/media handling takes priority. */
        if (atomic_exchange_explicit(&s_pairing_pending, false, memory_order_acq_rel)) {
            process_pairing_request();
        }

        now_ms = get_time_ms();

        if (audio_is_running() && (now_ms - last_audio_stats) >= 10000) {
            audio_log_stats();
            cpu_profile_log();
            if (s_active_transport == TRANSPORT_ESP_NOW && mesh_is_initialized()) {
                mesh_log_pipeline_stats();
            }
            phone_audio_state_t phone_state;
            if (phone_audio_get_state(&phone_state) == ESP_OK && phone_state.initialized) {
                phone_audio_log_stats();
            }
            last_audio_stats = now_ms;
        }

        if (g_mesh_active && mesh_is_initialized() &&
            (now_ms - last_quick_stats) >= quick_stats_interval_ms) {
            log_quick_stats();
            last_quick_stats = now_ms;
        }

        if (s_active_transport == TRANSPORT_NRF52840) {
            static int64_t last_rtt_log_ms = 0;

            transport_nrf_tick(now_ms);
#if !defined(APP_S31_LC3_WIRE)
            /* LC3 nRF firmware does not ACK legacy BRIDGE_PKT_AUDIO RTT probes. */
            rtt_probe_tick(now_ms, g_mesh_active, uart_bridge_is_connected());
#endif

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
    phone_audio_deinit();
    audio_stop();
    audio_deinit();
    return;

error_halt:
    ESP_LOGE(TAG, "System halted due to initialization error");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
