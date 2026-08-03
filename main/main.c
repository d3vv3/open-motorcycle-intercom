/**
 * @file main.c
 * @brief OMI - Open Motorcycle Intercom
 *
 * Main entry point for the ESP32-S3 firmware.
 * Phase 2: Single-hop RF link with TDMA mesh
 */

#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "audio.h"
#include "audio_bundle.h"
#include "audio_rx_tracker.h"
#include "audio_tx_cache.h"
#include "button.h"
#include "mesh.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "power.h"
#include "rtt_probe.h"
#include "uart_bridge.h"

static const char *TAG = "omi";

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Set to true to enable mesh mode, false for local loopback testing
 */
#define ENABLE_MESH_MODE 1

/* Debug instrumentation knobs */
#define REDUCED_LOGGING_MODE 1

/* Test knob: bypass VOX gating and always transmit microphone frames.
 * 0 = normal VOX behavior (DTX silence suppression active), 1 = force continuous TX. */
#define FORCE_TX_ALWAYS_FOR_TEST 0

/* ============================================================================
 * State
 * ============================================================================ */

static _Atomic bool s_mesh_active = false;

/* End-to-end audio sequence diagnostics (nRF transport path) */
static uint16_t s_e2e_tx_seq = 0;
static uint32_t s_e2e_tx_frames = 0;
static audio_rx_tracker_t s_e2e_rx_tracker = {0};
static portMUX_TYPE s_e2e_rx_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_pipe_source_frames = 0;
static uint32_t s_pipe_gate_drops = 0;
static uint32_t s_pipe_spi_attempts = 0;
static uint32_t s_pipe_spi_enqueue_ok = 0;
static uint32_t s_pipe_spi_enqueue_fail = 0;
static uint32_t s_pipe_spi_oversize = 0;
static uint32_t s_pipe_spi_rx = 0;
static uint32_t s_pipe_spi_rx_invalid = 0;
static uint32_t s_pipe_spi_rx_self = 0;
static uint32_t s_pipe_probe_rx = 0;
static uint32_t s_pipe_play_queue_ok = 0;
static uint32_t s_pipe_play_queue_drop = 0;
static uint32_t s_pipe_bundle_tx = 0;
static uint32_t s_pipe_prev1_attached = 0;
static uint32_t s_pipe_prev2_attached = 0;
static uint32_t s_pipe_bundle_rx = 0;
static uint32_t s_pipe_bundle_bad = 0;
static uint32_t s_pipe_prev1_offer = 0;
static uint32_t s_pipe_prev1_accept = 0;
static uint32_t s_pipe_prev1_reject = 0;
static uint32_t s_pipe_prev2_offer = 0;
static uint32_t s_pipe_prev2_accept = 0;
static uint32_t s_pipe_prev2_reject = 0;

static void reset_e2e_rx_source(uint8_t source_id)
{
    portENTER_CRITICAL(&s_e2e_rx_lock);
    audio_rx_tracker_reset_source(&s_e2e_rx_tracker, source_id);
    portEXIT_CRITICAL(&s_e2e_rx_lock);
}

static void reset_all_e2e_rx_sources(void)
{
    portENTER_CRITICAL(&s_e2e_rx_lock);
    audio_rx_tracker_reset_sources(&s_e2e_rx_tracker);
    portEXIT_CRITICAL(&s_e2e_rx_lock);
}

static audio_tx_cache_t s_nrf_previous_audio;


/* RTT log cadence while using nRF transport */
#define RTT_LOG_INTERVAL_MS 10000

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

static uint8_t get_bridge_node_id(void)
{
    uart_bridge_status_t status;
    if (uart_bridge_get_status(&status) == ESP_OK) {
        return status.node_id;
    }

    return 0;
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
static _Atomic bool s_mesh_user_enabled = false;

#define MESH_NVS_NAMESPACE "omi"
#define MESH_NVS_ENABLED_KEY "mesh_enabled"
#define NRF_RECONCILE_INTERVAL_MS 2000
#define NRF_RECONCILE_MAX_ATTEMPTS 3

static uint8_t s_nrf_reconcile_attempts = 0;
static bool s_nrf_status_observed = false;
static uint8_t s_nrf_last_mesh_state = BRIDGE_MESH_STATE_IDLE;
static bool s_nrf_membership_observed = false;
static uint8_t s_nrf_last_peer_count = 0;
static SemaphoreHandle_t s_nrf_membership_mutex = NULL;
static _Atomic bool s_mesh_enable_notification_pending = false;
#define NRF_NOTIFICATION_QUEUE_SIZE 8
typedef struct {
    audio_notify_t type;
    uint32_t generation;
} nrf_notification_t;
static nrf_notification_t s_nrf_notification_queue[NRF_NOTIFICATION_QUEUE_SIZE];
static uint8_t s_nrf_notification_head = 0;
static uint8_t s_nrf_notification_tail = 0;
static uint32_t s_nrf_membership_generation = 0;
static int64_t s_mesh_restart_attempt_ms = 0;

static esp_err_t load_mesh_user_intent(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MESH_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        s_mesh_user_enabled = false;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t enabled = 0;
    ret = nvs_get_u8(handle, MESH_NVS_ENABLED_KEY, &enabled);
    nvs_close(handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        s_mesh_user_enabled = false;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }
    if (enabled > 1) {
        ESP_LOGW(TAG, "Invalid persisted mesh intent %u; defaulting to disabled", enabled);
        enabled = 0;
    }
    s_mesh_user_enabled = enabled != 0;
    return ESP_OK;
}

static esp_err_t persist_mesh_user_intent(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MESH_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_u8(handle, MESH_NVS_ENABLED_KEY, enabled ? 1 : 0);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void reset_nrf_reconciliation(void)
{
    s_nrf_reconcile_attempts = 0;
    s_mesh_restart_attempt_ms = 0;
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
        if (mesh_get_state() == MESH_STATE_ACTIVE) {
            uint8_t audio_flags = active ? MESH_AUDIO_FLAG_ACTIVE : 0;
            esp_err_t ret = mesh_send_audio(data, len, audio_flags);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Failed to queue audio for TX: %s", esp_err_to_name(ret));
            }
        }
        break;

    case TRANSPORT_NRF52840:
        s_pipe_source_frames++;
        uint16_t seq = s_e2e_tx_seq++;
        s_e2e_tx_frames++;
        if (len > MESH_MAX_OPUS_BYTES) {
            s_pipe_spi_oversize++;
            audio_tx_cache_reset(&s_nrf_previous_audio);
            ESP_LOGW(TAG, "Audio frame too large for E2E wrapper: %u", len);
            break;
        }

        /* Single-predecessor redundancy: prev1 recovers isolated losses, and
         * Opus PLC covers the rare two-in-a-row loss. prev2 measured near-zero
         * additional recovery for ~1/3 of the airtime budget. */
        uint16_t previous1_len = 0;
        const uint8_t *previous1_data =
            audio_tx_cache_previous(&s_nrf_previous_audio, seq, &previous1_len);
        bool attach_previous1 = previous1_data != NULL;
        uint8_t bundle_buf[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
        size_t bundle_len = 0;
        audio_bundle_view_t bundle = {
            .previous1_data = previous1_data,
            .previous2_data = NULL,
            .current_data = data,
            .previous1_len = previous1_len,
            .previous2_len = 0,
            .current_len = len,
            .current_seq = seq,
            .stream_id = get_bridge_node_id(),
            .flags = active ? AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE : 0,
        };
        if (attach_previous1) {
            bundle.flags |= AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                            AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
        }
        bool bundle_encoded = audio_bundle_encode(&bundle, bundle_buf, sizeof(bundle_buf),
                                                  &bundle_len);

        audio_tx_cache_store(&s_nrf_previous_audio, data, len, active, seq,
                             atomic_load(&s_mesh_user_enabled));

        xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
        if (s_mesh_user_enabled && uart_bridge_is_mesh_ready()) {
            s_pipe_spi_attempts++;
            esp_err_t ret = bundle_encoded
                                ? uart_bridge_send_audio_v2(bundle_buf, (uint16_t)bundle_len)
                                : ESP_ERR_INVALID_SIZE;
            if (ret == ESP_OK) {
                s_pipe_bundle_tx++;
                if (attach_previous1) {
                    s_pipe_prev1_attached++;
                }
            }
            if (ret != ESP_OK) {
                s_pipe_spi_enqueue_fail++;
                ESP_LOGD(TAG, "Failed to send audio via UART: %s", esp_err_to_name(ret));
            } else {
                s_pipe_spi_enqueue_ok++;
                int64_t now_us = esp_timer_get_time();
                if (now_us >= timestamp_us) {
                    (void)audio_record_tx_pipeline_latency_us((uint32_t)(now_us - timestamp_us));
                }
                /* Rate limit logs */
                static int64_t last_log = 0;
                int64_t now = now_us;
                if (now - last_log > 5000000) { /* Every 5s */
                    ESP_LOGI(TAG, "Audio sent via SPI bridge (len=%d)", len);
                    last_log = now;
                }
            }
        } else {
            s_pipe_gate_drops++;
            /* Log if we're trying to send but bridge thinks it's disconnected */
            static int64_t last_log = 0;
            int64_t now = esp_timer_get_time();
            if (now - last_log > 1000000) { /* Every 1s */
                ESP_LOGW(TAG, "Audio dropped - SPI bridge not connected");
                last_log = now;
            }
        }
        xSemaphoreGive(s_nrf_membership_mutex);
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

/**
 * @brief Callback from mesh subsystem when audio frame is received (ESP-NOW)
 */
static void mesh_audio_callback(const uint8_t *data, uint16_t len, uint8_t src_id,
                                 uint8_t audio_flags, int64_t timestamp_us)
{
    audio_frame_t frame;

    if (len > sizeof(frame.data)) {
        ESP_LOGW(TAG, "Audio frame too large: %u bytes", len);
        return;
    }

    memcpy(frame.data, data, len);
    frame.len = len;
    frame.timestamp_ms = timestamp_us / 1000;
    frame.active = (audio_flags & MESH_AUDIO_FLAG_ACTIVE) != 0;
    /* ESP-NOW carries no end-to-end audio sequence, so playout cannot detect holes. */
    frame.seq = 0;
    frame.has_seq = false;

    esp_err_t ret = audio_put_rx_frame(&frame, src_id);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to queue RX audio: %s", esp_err_to_name(ret));
    }
}

static esp_err_t submit_nrf_audio_frame(const audio_frame_t *frame, uint8_t src_id,
                                        bool *admitted)
{
    xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
    bool ready = s_mesh_user_enabled && uart_bridge_is_mesh_ready();
    if (admitted != NULL) {
        *admitted = ready;
    }
    esp_err_t ret = ready ? audio_put_rx_frame(frame, src_id) : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(s_nrf_membership_mutex);
    return ret;
}

static void offer_nrf_predecessor(uint8_t src_id, const uint8_t *data, size_t len,
                                  uint16_t seq, bool active, int64_t timestamp_us,
                                  uint32_t *offer_count, uint32_t *accept_count,
                                  uint32_t *reject_count)
{
    audio_frame_t frame;

    (*offer_count)++;
    memcpy(frame.data, data, len);
    frame.len = (uint16_t)len;
    frame.timestamp_ms = timestamp_us / 1000;
    frame.active = active;
    frame.seq = seq;
    frame.has_seq = true;

    bool queue_succeeded = submit_nrf_audio_frame(&frame, src_id, NULL) == ESP_OK;
    if (queue_succeeded) {
        (*accept_count)++;
    } else {
        (*reject_count)++;
    }
    portENTER_CRITICAL(&s_e2e_rx_lock);
    (void)audio_rx_tracker_credit_predecessor(&s_e2e_rx_tracker, src_id,
                                              queue_succeeded);
    portEXIT_CRITICAL(&s_e2e_rx_lock);
}

/**
 * @brief Callback from UART bridge when audio is received (nRF52840)
 */
static void bridge_audio_callback(uint8_t src_id, const uint8_t *data, uint16_t len,
                                   int64_t timestamp_us, bool redundant_bundle)
{
    if (!s_mesh_user_enabled || !uart_bridge_is_mesh_ready()) {
        s_pipe_spi_rx_invalid++;
        return;
    }

    if (src_id == 0 || src_id > MESH_MAX_NODES) {
        s_pipe_spi_rx_invalid++;
        return;
    }

    uint8_t local_node_id = get_bridge_node_id();

    s_pipe_spi_rx++;
    if (local_node_id != 0 && src_id == local_node_id) {
        s_pipe_spi_rx_self++;
        return;
    }

    if (!redundant_bundle) {
        if (len < 2 || !rtt_probe_handle_packet(src_id, data + 1, (uint16_t)(len - 1))) {
            s_pipe_spi_rx_invalid++;
        } else {
            s_pipe_probe_rx++;
        }
        return;
    }

    audio_bundle_view_t bundle = {0};
    s_pipe_bundle_rx++;
    if (!audio_bundle_parse(data, len, &bundle) ||
        (bundle.stream_id != 0 && bundle.stream_id != src_id)) {
        s_pipe_bundle_bad++;
        s_pipe_spi_rx_invalid++;
        return;
    }
    uint16_t e2e_seq = bundle.current_seq;

    portENTER_CRITICAL(&s_e2e_rx_lock);
    (void)audio_rx_tracker_accept(&s_e2e_rx_tracker, src_id, e2e_seq);
    portEXIT_CRITICAL(&s_e2e_rx_lock);

    audio_frame_t frame;
    if (bundle.current_len > sizeof(frame.data)) {
        s_pipe_spi_rx_invalid++;
        ESP_LOGW(TAG, "Audio frame too large: %u bytes", (unsigned)bundle.current_len);
        return;
    }

    if (bundle.previous2_len != 0) {
        offer_nrf_predecessor(
            src_id, bundle.previous2_data, bundle.previous2_len,
            (uint16_t)(e2e_seq - 2u),
            (bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0,
            timestamp_us - 40000, &s_pipe_prev2_offer,
            &s_pipe_prev2_accept, &s_pipe_prev2_reject);
    }
    if (bundle.previous1_len != 0) {
        offer_nrf_predecessor(
            src_id, bundle.previous1_data, bundle.previous1_len,
            (uint16_t)(e2e_seq - 1u),
            (bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0,
            timestamp_us - 20000, &s_pipe_prev1_offer,
            &s_pipe_prev1_accept, &s_pipe_prev1_reject);
    }

    memcpy(frame.data, bundle.current_data, bundle.current_len);
    frame.len = (uint16_t)bundle.current_len;
    frame.timestamp_ms = timestamp_us / 1000;
    frame.active = (bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) != 0;
    /* Hand the end-to-end sequence to playout so it can conceal missing frames. */
    frame.seq = e2e_seq;
    frame.has_seq = true;

    bool admitted = false;
    esp_err_t ret = submit_nrf_audio_frame(&frame, src_id, &admitted);
    if (!admitted) {
        s_pipe_spi_rx_invalid++;
        return;
    }
    if (ret != ESP_OK) {
        s_pipe_play_queue_drop++;
        ESP_LOGD(TAG, "Failed to queue RX audio: %s", esp_err_to_name(ret));
    } else {
        s_pipe_play_queue_ok++;
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
        (void)audio_play_notification(AUDIO_NOTIFY_PEER_JOIN);
    } else {
        ESP_LOGI(TAG, "Peer LEFT: node_id=%u", peer->node_id);
        (void)audio_play_notification(AUDIO_NOTIFY_PEER_LEAVE);
    }
}

static void reset_nrf_membership_tracking(void)
{
    xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
    s_nrf_membership_observed = false;
    s_nrf_last_peer_count = 0;
    s_nrf_notification_head = 0;
    s_nrf_notification_tail = 0;
    s_nrf_membership_generation++;
    xSemaphoreGive(s_nrf_membership_mutex);
}

static void queue_nrf_notification_locked(audio_notify_t type)
{
    uint8_t next = (uint8_t)((s_nrf_notification_head + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
    if (next == s_nrf_notification_tail) {
        s_nrf_notification_tail =
            (uint8_t)((s_nrf_notification_tail + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
    }
    s_nrf_notification_queue[s_nrf_notification_head] = (nrf_notification_t){
        .type = type,
        .generation = s_nrf_membership_generation,
    };
    s_nrf_notification_head = next;
}

static void drain_nrf_notifications(void)
{
    xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
    while (s_nrf_notification_tail != s_nrf_notification_head) {
        nrf_notification_t item = s_nrf_notification_queue[s_nrf_notification_tail];
        s_nrf_notification_tail =
            (uint8_t)((s_nrf_notification_tail + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
        if (item.generation == s_nrf_membership_generation &&
            atomic_load(&s_mesh_user_enabled)) {
            (void)audio_play_notification(item.type);
        }
    }
    xSemaphoreGive(s_nrf_membership_mutex);
}

static void set_mesh_user_runtime_state(bool enabled)
{
    xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
    atomic_store(&s_mesh_user_enabled, enabled);
    atomic_store(&s_mesh_enable_notification_pending, enabled);
    s_nrf_membership_generation++;
    s_nrf_notification_head = 0;
    s_nrf_notification_tail = 0;
    if (!enabled) {
        audio_tx_cache_reset(&s_nrf_previous_audio);
        s_nrf_membership_observed = false;
        s_nrf_last_peer_count = 0;
        atomic_store(&s_mesh_active, false);
    }
    xSemaphoreGive(s_nrf_membership_mutex);
}

static void bridge_status_callback(const uart_bridge_status_t *status)
{
    bool ready = status->mesh_state == BRIDGE_MESH_STATE_ACTIVE && status->node_id != 0;
    bool user_enabled;
    bool logged_join = false;
    bool logged_leave = false;

    xSemaphoreTake(s_nrf_membership_mutex, portMAX_DELAY);
    user_enabled = atomic_load(&s_mesh_user_enabled);
    if (!user_enabled) {
        s_nrf_membership_observed = false;
        s_nrf_last_peer_count = 0;
    } else if (!ready || status->continuity_lost ||
               status->peer_count == BRIDGE_PEER_COUNT_UNKNOWN) {
        s_nrf_membership_observed = false;
        s_nrf_last_peer_count = 0;
    } else if (!s_nrf_membership_observed) {
        s_nrf_membership_observed = true;
        s_nrf_last_peer_count = status->peer_count;
    } else {
        if (status->peer_count > s_nrf_last_peer_count) {
            queue_nrf_notification_locked(AUDIO_NOTIFY_PEER_JOIN);
            logged_join = true;
        } else if (status->peer_count < s_nrf_last_peer_count) {
            queue_nrf_notification_locked(AUDIO_NOTIFY_PEER_LEAVE);
            logged_leave = true;
        }
        s_nrf_last_peer_count = status->peer_count;
    }
    atomic_store(&s_mesh_active, user_enabled && ready);
    if (user_enabled && ready && atomic_exchange(&s_mesh_enable_notification_pending, false)) {
        queue_nrf_notification_locked(AUDIO_NOTIFY_MESH_ENABLED);
    }
    xSemaphoreGive(s_nrf_membership_mutex);

    if (logged_join) {
        ESP_LOGI(TAG, "nRF peer count increased to %u", status->peer_count);
    } else if (logged_leave) {
        ESP_LOGI(TAG, "nRF peer count decreased to %u", status->peer_count);
    }
}

/**
 * @brief Callback for mesh events from nRF52840
 */
static void bridge_event_callback(uart_bridge_event_t event, const uint8_t *data, uint16_t len)
{
    switch (event) {
    case BRIDGE_EVENT_MESH_READY:
        ESP_LOGI(TAG, "nRF52840 mesh ready");
        if (!s_mesh_user_enabled) {
            /* Reconciliation runs in app_main, never in this bridge RX callback. */
            ESP_LOGI(TAG, "Mesh-ready conflicts with disabled user intent");
            s_mesh_active = false;
            break;
        }
        break;
    case BRIDGE_EVENT_PEER_JOINED:
        if (data != NULL && len >= 1 && data[0] >= 1 && data[0] <= MESH_MAX_NODES) {
            reset_e2e_rx_source(data[0]);
            ESP_LOGI(TAG, "Peer %u join event received; waiting for status confirmation", data[0]);
        } else {
            ESP_LOGW(TAG, "Ignoring peer join event with invalid node ID");
        }
        break;
    case BRIDGE_EVENT_PEER_LEFT:
        if (data != NULL && len >= 1 && data[0] >= 1 && data[0] <= MESH_MAX_NODES) {
            reset_e2e_rx_source(data[0]);
            ESP_LOGI(TAG, "Peer %u leave event received; waiting for status confirmation", data[0]);
        } else {
            ESP_LOGW(TAG, "Ignoring peer leave event with invalid node ID");
        }
        break;
    case BRIDGE_EVENT_MESH_STOPPED:
        ESP_LOGI(TAG, "nRF52840 mesh stopped");
        reset_all_e2e_rx_sources();
        audio_tx_cache_reset(&s_nrf_previous_audio);
        s_mesh_active = false;
        atomic_store(&s_mesh_enable_notification_pending, false);
        reset_nrf_membership_tracking();
        break;
    case BRIDGE_EVENT_SYNC_LOST:
        ESP_LOGW(TAG, "nRF sync lost/rescanning; waiting for status confirmation");
        reset_all_e2e_rx_sources();
        break;
    case BRIDGE_EVENT_COMMAND_ACK:
    case BRIDGE_EVENT_BECAME_COORDINATOR:
        break;
    default:
        break;
    }
}

static void reconcile_nrf_mesh_state(int64_t now_ms)
{
    uart_bridge_status_t status;
    if (uart_bridge_get_status(&status) != ESP_OK) {
        atomic_store(&s_mesh_active, false);
        if (s_nrf_status_observed) {
            reset_nrf_membership_tracking();
        }
        s_nrf_status_observed = false;
        return;
    }

    bool ready = status.mesh_state == BRIDGE_MESH_STATE_ACTIVE && status.node_id != 0;
    if (!s_nrf_status_observed || status.mesh_state != s_nrf_last_mesh_state) {
        s_nrf_status_observed = true;
        s_nrf_last_mesh_state = status.mesh_state;
        s_nrf_reconcile_attempts = 0;
    }

    if (s_mesh_user_enabled && ready) {
        atomic_store(&s_mesh_active, true);
        return;
    }

    s_mesh_active = false;
    bool disabled = status.mesh_state == BRIDGE_MESH_STATE_IDLE;
    if ((!s_mesh_user_enabled && disabled) ||
        (s_mesh_user_enabled && status.has_mesh_state && !disabled)) {
        s_nrf_reconcile_attempts = 0;
        return;
    }

    if (s_nrf_reconcile_attempts >= NRF_RECONCILE_MAX_ATTEMPTS ||
        now_ms - s_mesh_restart_attempt_ms < NRF_RECONCILE_INTERVAL_MS) {
        return;
    }

    s_mesh_restart_attempt_ms = now_ms;
    s_nrf_reconcile_attempts++;
    esp_err_t ret = s_mesh_user_enabled ? uart_bridge_mesh_enable() : uart_bridge_mesh_disable();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Mesh %s reconcile failed (%u/%u): %s",
                 s_mesh_user_enabled ? "enable" : "disable", s_nrf_reconcile_attempts,
                 NRF_RECONCILE_MAX_ATTEMPTS, esp_err_to_name(ret));
    }
}

/**
 * @brief Callback for button long press - toggles mesh on/off
 */
static void button_long_press_callback(int gpio)
{
    ESP_LOGI(TAG, "Button long press detected on GPIO %d - toggling mesh", gpio);

    bool requested_enabled = !s_mesh_user_enabled;
    esp_err_t ret = persist_mesh_user_intent(requested_enabled);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist mesh intent: %s", esp_err_to_name(ret));
        return;
    }
    set_mesh_user_runtime_state(requested_enabled);
    reset_nrf_reconciliation();

    if (!requested_enabled) {
        /* Disable mesh */
        ESP_LOGI(TAG, "Disabling mesh networking...");

        if (s_active_transport == TRANSPORT_ESP_NOW) {
            ret = mesh_stop();
        } else if (s_active_transport == TRANSPORT_NRF52840) {
            /* app_main reconciles this intent outside bridge and button callbacks. */
            ret = ESP_OK;
        }

        if (ret == ESP_OK) {
            s_mesh_active = false;
            atomic_store(&s_mesh_enable_notification_pending, false);
            reset_nrf_membership_tracking();
            audio_clear_rx_frames();
            (void)audio_play_notification(AUDIO_NOTIFY_MESH_DISABLED);
            ESP_LOGI(TAG, "Mesh disabled");
        } else {
            set_mesh_user_runtime_state(true);
            atomic_store(&s_mesh_enable_notification_pending, false);
            esp_err_t persist_ret = persist_mesh_user_intent(true);
            if (persist_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore enabled mesh intent: %s",
                         esp_err_to_name(persist_ret));
            }
            ESP_LOGE(TAG, "Failed to stop mesh: %s", esp_err_to_name(ret));
        }
    } else {
        /* Enable mesh */
        ESP_LOGI(TAG, "Enabling mesh networking...");
        s_mesh_active = false;

        if (s_active_transport == TRANSPORT_ESP_NOW) {
            ret = mesh_start();
        } else if (s_active_transport == TRANSPORT_NRF52840) {
            ret = ESP_OK;
        }

        if (ret == ESP_OK) {
            if (s_active_transport == TRANSPORT_ESP_NOW) {
                s_mesh_active = true;
                atomic_store(&s_mesh_enable_notification_pending, false);
                (void)audio_play_notification(AUDIO_NOTIFY_MESH_ENABLED);
            }
            ESP_LOGI(TAG, "Mesh enable accepted; waiting for active state");
        } else {
            set_mesh_user_runtime_state(false);
            esp_err_t persist_ret = persist_mesh_user_intent(false);
            if (persist_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore disabled mesh intent: %s",
                         esp_err_to_name(persist_ret));
            }
            s_mesh_active = false;
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
    rtt_probe_init();

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
    ESP_ERROR_CHECK(load_mesh_user_intent());
    ESP_LOGI(TAG, "Persisted mesh intent: %s", s_mesh_user_enabled ? "enabled" : "disabled");
    s_nrf_membership_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_nrf_membership_mutex ? ESP_OK : ESP_ERR_NO_MEM);

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
        uart_bridge_set_status_callback(bridge_status_callback);

        s_mesh_active = false;
        reset_nrf_reconciliation();
        disable_esp_radios_for_nrf_transport();
    } else {
        /* No nRF52840 - fallback to ESP-NOW */
        uart_bridge_deinit();
        audio_tx_cache_reset(&s_nrf_previous_audio);
        s_active_transport = TRANSPORT_ESP_NOW;
        ESP_LOGI(TAG, "nRF52840 not detected - using ESP-NOW transport");
    }

    /* Initialize audio subsystem (after SPI so DMA channels are available) */
    ESP_LOGI(TAG, "");
    ret = init_audio_with_test_flags();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }
    ESP_LOGI(TAG, "Audio test flags: force_tx_always=%s",
             FORCE_TX_ALWAYS_FOR_TEST ? "YES" : "no");

    /* Configure audio for mesh mode */
    ret = audio_set_mode(AUDIO_MODE_MESH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set audio mode: %s", esp_err_to_name(ret));
        goto error_halt;
    }

    /* Register audio TX callback */
    audio_register_tx_callback(audio_tx_callback);
    audio_register_activity_callback(audio_activity_callback);

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

        if (s_mesh_user_enabled) {
            ret = mesh_start();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore persisted mesh intent: %s", esp_err_to_name(ret));
                goto error_halt;
            }
            s_mesh_active = true;
        }
    }

    /* Register button callback for mesh toggle */
    button_register_long_press_callback(button_long_press_callback);
#else
    /* Non-mesh: initialize audio directly (no SPI contention) */
    ESP_LOGI(TAG, "");
    ret = init_audio_with_test_flags();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }
    ESP_LOGI(TAG, "Audio test flags: force_tx_always=%s",
             FORCE_TX_ALWAYS_FOR_TEST ? "YES" : "no");
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

    /* Queue startup notification; the audio task owns generation and I2S playback. */
    ret = audio_play_notification(AUDIO_NOTIFY_STARTUP);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to play startup notification: %s", esp_err_to_name(ret));
    }

    ret = audio_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio: %s", esp_err_to_name(ret));
        goto error_halt;
    }

#if ENABLE_MESH_MODE
    /* The persisted user policy is reconciled with the selected transport. */
    if (s_active_transport == TRANSPORT_NRF52840) {
        s_mesh_active = false;
    }
    ESP_LOGI(TAG, "Mesh networking ready (desired state: %s)",
             s_mesh_user_enabled ? "enabled" : "disabled");
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

#if ENABLE_MESH_MODE
        /* Quick stats every 10 seconds for Phase 2 validation */
        if (s_mesh_active && mesh_is_initialized() &&
            (now_ms - last_quick_stats) >= quick_stats_interval_ms) {
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

            if (s_active_transport == TRANSPORT_NRF52840) {
                rtt_probe_stats_t rtt = {0};
                rtt_probe_get_stats(&rtt);
                ESP_LOGI(TAG, "[RTT] sent=%lu recv=%lu lost=%lu rtt=%lums/%lums jit=%lums/%lums",
                         rtt.sent, rtt.recv, rtt.lost, rtt.rtt_ms_avg, rtt.rtt_ms_max,
                         rtt.jitter_ms_avg, rtt.jitter_ms_max);
            }

            last_quick_stats = now_ms;
        }

        if (s_active_transport == TRANSPORT_NRF52840) {
            static int64_t last_rtt_log_ms = 0;

            reconcile_nrf_mesh_state(now_ms);
            drain_nrf_notifications();
            rtt_probe_tick(now_ms, s_mesh_active, uart_bridge_is_connected());

            if ((now_ms - last_rtt_log_ms) >= RTT_LOG_INTERVAL_MS) {
                rtt_probe_stats_t rtt = {0};
                uint32_t rx_frames;
                uint32_t rx_gap_events;
                uint32_t rx_gap_frames;
                uint32_t rx_reordered;
                uint32_t rx_recovered;
                rtt_probe_get_stats(&rtt);
                portENTER_CRITICAL(&s_e2e_rx_lock);
                rx_frames = s_e2e_rx_tracker.frames;
                rx_gap_events = s_e2e_rx_tracker.gap_events;
                rx_gap_frames = s_e2e_rx_tracker.gap_frames;
                rx_reordered = s_e2e_rx_tracker.reordered_or_old;
                rx_recovered = s_e2e_rx_tracker.recovered_frames;
                portEXIT_CRITICAL(&s_e2e_rx_lock);

                ESP_LOGI(TAG, "[RTT] sent=%lu recv=%lu lost=%lu rtt=%lums/%lums jit=%lums/%lums",
                         rtt.sent, rtt.recv, rtt.lost, rtt.rtt_ms_avg, rtt.rtt_ms_max,
                         rtt.jitter_ms_avg, rtt.jitter_ms_max);
                ESP_LOGI(TAG,
                         "[E2E_ESP] tx=%lu rx=%lu gap_evt=%lu gap_fr=%lu reset_evt=%lu recovered=%lu effective_gap=%lu",
                          s_e2e_tx_frames, rx_frames, rx_gap_events, rx_gap_frames,
                          rx_reordered, rx_recovered,
                          rx_gap_frames > rx_recovered
                             ? rx_gap_frames - rx_recovered
                             : 0);
                ESP_LOGI(TAG,
                         "PIPE v=1 dev=esp stage=transport node=%u source=%lu gate_drop=%lu spi_try=%lu spi_ok=%lu spi_fail=%lu spi_oversize=%lu spi_rx=%lu spi_gap=%lu spi_invalid=%lu spi_self=%lu probe_rx=%lu play_q_ok=%lu play_q_drop=%lu bundle_tx=%lu bundle_rx=%lu bundle_bad=%lu prev1_attached=%lu prev2_attached=%lu prev1_offer=%lu prev1_accept=%lu prev1_reject=%lu prev2_offer=%lu prev2_accept=%lu prev2_reject=%lu recovered=%lu",
                         get_bridge_node_id(), s_pipe_source_frames, s_pipe_gate_drops,
                         s_pipe_spi_attempts, s_pipe_spi_enqueue_ok, s_pipe_spi_enqueue_fail,
                          s_pipe_spi_oversize, s_pipe_spi_rx, rx_gap_frames,
                         s_pipe_spi_rx_invalid, s_pipe_spi_rx_self, s_pipe_probe_rx,
                         s_pipe_play_queue_ok, s_pipe_play_queue_drop, s_pipe_bundle_tx,
                         s_pipe_bundle_rx, s_pipe_bundle_bad, s_pipe_prev1_attached,
                         s_pipe_prev2_attached, s_pipe_prev1_offer, s_pipe_prev1_accept,
                         s_pipe_prev1_reject, s_pipe_prev2_offer, s_pipe_prev2_accept,
                          s_pipe_prev2_reject, rx_recovered);
                ESP_LOGI(TAG,
                         "Redundancy: tx=%lu rx=%lu attached=%lu/%lu prev1=%lu/%lu/%lu prev2=%lu/%lu/%lu recovered=%lu",
                         s_pipe_bundle_tx, s_pipe_bundle_rx, s_pipe_prev1_attached,
                         s_pipe_prev2_attached, s_pipe_prev1_offer, s_pipe_prev1_accept,
                         s_pipe_prev1_reject, s_pipe_prev2_offer, s_pipe_prev2_accept,
                          s_pipe_prev2_reject, rx_recovered);
                last_rtt_log_ms = now_ms;
            }
        }
#endif

        if ((now_ms - last_health_check) >= health_interval_ms) {
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
