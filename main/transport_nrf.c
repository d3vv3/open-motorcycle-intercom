/**
 * @file transport_nrf.c
 * @brief nRF52840 transport controller (ESB via SPI bridge).
 */

#include "transport_nrf.h"

#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_state.h"
#include "audio.h"
#include "audio_bundle.h"
#include "audio_tx_cache.h"
#include "e2e_diag.h"
#include "mesh.h"
#include "mesh_intent.h"
#include "rtt_probe.h"
#include "uart_bridge.h"

static const char *TAG = "omi";

#define NRF_RECONCILE_INTERVAL_MS   2000
#define NRF_RECONCILE_MAX_ATTEMPTS  3
#define NRF_NOTIFICATION_QUEUE_SIZE 8

typedef struct {
    audio_notify_t type;
    uint32_t generation;
} nrf_notification_t;

static audio_tx_cache_t s_previous_audio;

static uint8_t s_reconcile_attempts = 0;
static bool s_status_observed = false;
static uint8_t s_last_mesh_state = BRIDGE_MESH_STATE_IDLE;
static int64_t s_restart_attempt_ms = 0;

/* Membership tracking and the tone queue share one mutex; the generation
 * counter invalidates queued tones from a previous enable/disable cycle. */
static SemaphoreHandle_t s_membership_mutex = NULL;
static bool s_membership_observed = false;
static uint8_t s_last_peer_count = 0;
static nrf_notification_t s_notification_queue[NRF_NOTIFICATION_QUEUE_SIZE];
static uint8_t s_notification_head = 0;
static uint8_t s_notification_tail = 0;
static uint32_t s_membership_generation = 0;
static _Atomic bool s_enable_notification_pending = false;

uint8_t transport_nrf_node_id(void)
{
    uart_bridge_status_t status;
    if (uart_bridge_get_status(&status) == ESP_OK) {
        return status.node_id;
    }

    return 0;
}

void transport_nrf_reset_reconciliation(void)
{
    s_reconcile_attempts = 0;
    s_restart_attempt_ms = 0;
}

void transport_nrf_reset_tx_cache(void)
{
    audio_tx_cache_reset(&s_previous_audio);
}

void transport_nrf_cancel_enable_notification(void)
{
    atomic_store(&s_enable_notification_pending, false);
}

static void reset_membership_tracking(void)
{
    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    s_membership_observed = false;
    s_last_peer_count = 0;
    s_notification_head = 0;
    s_notification_tail = 0;
    s_membership_generation++;
    xSemaphoreGive(s_membership_mutex);
}

static void queue_notification_locked(audio_notify_t type)
{
    uint8_t next = (uint8_t)((s_notification_head + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
    if (next == s_notification_tail) {
        s_notification_tail = (uint8_t)((s_notification_tail + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
    }
    s_notification_queue[s_notification_head] = (nrf_notification_t){
        .type = type,
        .generation = s_membership_generation,
    };
    s_notification_head = next;
}

static void drain_notifications(void)
{
    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    while (s_notification_tail != s_notification_head) {
        nrf_notification_t item = s_notification_queue[s_notification_tail];
        s_notification_tail = (uint8_t)((s_notification_tail + 1) % NRF_NOTIFICATION_QUEUE_SIZE);
        if (item.generation == s_membership_generation && mesh_intent_enabled()) {
            (void)audio_play_notification(item.type);
        }
    }
    xSemaphoreGive(s_membership_mutex);
}

void transport_nrf_set_user_enabled(bool enabled)
{
    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    mesh_intent_set(enabled);
    atomic_store(&s_enable_notification_pending, enabled);
    s_membership_generation++;
    s_notification_head = 0;
    s_notification_tail = 0;
    if (!enabled) {
        audio_tx_cache_reset(&s_previous_audio);
        s_membership_observed = false;
        s_last_peer_count = 0;
        atomic_store(&g_mesh_active, false);
    }
    xSemaphoreGive(s_membership_mutex);
}

void transport_nrf_send_audio(const uint8_t *data, uint16_t len, bool active, int64_t timestamp_us)
{
    e2e_pipe_counters_t *pipe = e2e_diag_counters();
    uint16_t seq = e2e_diag_next_tx_seq();
    if (len > MESH_MAX_OPUS_BYTES) {
        pipe->spi_oversize++;
        audio_tx_cache_reset(&s_previous_audio);
        ESP_LOGW(TAG, "Audio frame too large for E2E wrapper: %u", len);
        return;
    }

    /* Single-predecessor redundancy: prev1 recovers isolated losses, and
     * Opus PLC covers the rare two-in-a-row loss. prev2 measured near-zero
     * additional recovery for ~1/3 of the airtime budget. */
    uint16_t previous1_len = 0;
    const uint8_t *previous1_data = audio_tx_cache_previous(&s_previous_audio, seq, &previous1_len);
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
        .stream_id = transport_nrf_node_id(),
        .flags = active ? AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE : 0,
    };
    if (attach_previous1) {
        bundle.flags |= AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT | AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    }
    bool bundle_encoded = audio_bundle_encode(&bundle, bundle_buf, sizeof(bundle_buf), &bundle_len);

    audio_tx_cache_store(&s_previous_audio, data, len, active, seq, mesh_intent_enabled());

    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    if (mesh_intent_enabled() && uart_bridge_is_mesh_ready()) {
        pipe->spi_attempts++;
        esp_err_t ret = bundle_encoded ? uart_bridge_send_audio_v2(bundle_buf, (uint16_t)bundle_len)
                                       : ESP_ERR_INVALID_SIZE;
        if (ret == ESP_OK) {
            pipe->bundle_tx++;
            if (attach_previous1) {
                pipe->prev1_attached++;
            }
        }
        if (ret != ESP_OK) {
            pipe->spi_enqueue_fail++;
            ESP_LOGD(TAG, "Failed to send audio via SPI bridge: %s", esp_err_to_name(ret));
        } else {
            pipe->spi_enqueue_ok++;
            int64_t now_us = esp_timer_get_time();
            if (now_us >= timestamp_us) {
                (void)audio_record_tx_pipeline_latency_us((uint32_t)(now_us - timestamp_us));
            }
            /* Rate limit logs */
            static int64_t last_log = 0;
            if (now_us - last_log > 5000000) { /* Every 5s */
                ESP_LOGI(TAG, "Audio sent via SPI bridge (len=%d)", len);
                last_log = now_us;
            }
        }
    } else {
        pipe->gate_drops++;
        /* Log if we're trying to send but bridge thinks it's disconnected */
        static int64_t last_log = 0;
        int64_t now = esp_timer_get_time();
        if (now - last_log > 1000000) { /* Every 1s */
            ESP_LOGW(TAG, "Audio dropped - SPI bridge not connected");
            last_log = now;
        }
    }
    xSemaphoreGive(s_membership_mutex);
}

static esp_err_t submit_audio_frame(const audio_frame_t *frame, uint8_t src_id, bool *admitted)
{
    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    bool ready = mesh_intent_enabled() && uart_bridge_is_mesh_ready();
    if (admitted != NULL) {
        *admitted = ready;
    }
    esp_err_t ret = ready ? audio_put_rx_frame(frame, src_id) : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(s_membership_mutex);
    return ret;
}

typedef struct {
    uint32_t *offer_count;
    uint32_t *accept_count;
    uint32_t *reject_count;
} predecessor_counter_targets_t;

typedef struct {
    uint8_t src_id;
    const uint8_t *data;
    size_t len;
    uint16_t seq;
    bool active;
    int64_t timestamp_us;
    predecessor_counter_targets_t counters;
} predecessor_offer_t;

static void offer_predecessor(const predecessor_offer_t *offer)
{
    audio_frame_t frame;

    (*offer->counters.offer_count)++;
    memcpy(frame.data, offer->data, offer->len);
    frame.len = (uint16_t)offer->len;
    frame.timestamp_ms = offer->timestamp_us / 1000;
    frame.active = offer->active;
    frame.seq = offer->seq;
    frame.has_seq = true;

    bool queue_succeeded = submit_audio_frame(&frame, offer->src_id, NULL) == ESP_OK;
    if (queue_succeeded) {
        (*offer->counters.accept_count)++;
    } else {
        (*offer->counters.reject_count)++;
    }
    e2e_diag_credit_predecessor(offer->src_id, queue_succeeded);
}

/**
 * @brief Callback from SPI bridge when audio is received (nRF52840)
 */
static void bridge_audio_callback(uint8_t src_id, const uint8_t *data, uint16_t len,
                                  int64_t timestamp_us, bool redundant_bundle)
{
    e2e_pipe_counters_t *pipe = e2e_diag_counters();

    if (!mesh_intent_enabled() || !uart_bridge_is_mesh_ready()) {
        pipe->spi_rx_invalid++;
        return;
    }

    if (src_id == 0 || src_id > MESH_MAX_NODES) {
        pipe->spi_rx_invalid++;
        return;
    }

    uint8_t local_node_id = transport_nrf_node_id();

    pipe->spi_rx++;
    if (local_node_id != 0 && src_id == local_node_id) {
        pipe->spi_rx_self++;
        return;
    }

    if (!redundant_bundle) {
        if (len < 2 || !rtt_probe_handle_packet(src_id, data + 1, (uint16_t)(len - 1))) {
            pipe->spi_rx_invalid++;
        } else {
            pipe->probe_rx++;
        }
        return;
    }

    audio_bundle_view_t bundle = {0};
    pipe->bundle_rx++;
    if (!audio_bundle_parse(data, len, &bundle) ||
        (bundle.stream_id != 0 && bundle.stream_id != src_id)) {
        pipe->bundle_bad++;
        pipe->spi_rx_invalid++;
        return;
    }
    uint16_t e2e_seq = bundle.current_seq;

    e2e_diag_track_rx(src_id, e2e_seq);

    audio_frame_t frame;
    if (bundle.current_len > sizeof(frame.data)) {
        pipe->spi_rx_invalid++;
        ESP_LOGW(TAG, "Audio frame too large: %u bytes", (unsigned)bundle.current_len);
        return;
    }

    if (bundle.previous2_len != 0) {
        offer_predecessor(&(predecessor_offer_t){
            .src_id = src_id,
            .data = bundle.previous2_data,
            .len = bundle.previous2_len,
            .seq = (uint16_t)(e2e_seq - 2u),
            .active = (bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0,
            .timestamp_us = timestamp_us - 40000,
            .counters =
                {
                    .offer_count = &pipe->prev2_offer,
                    .accept_count = &pipe->prev2_accept,
                    .reject_count = &pipe->prev2_reject,
                },
        });
    }
    if (bundle.previous1_len != 0) {
        offer_predecessor(&(predecessor_offer_t){
            .src_id = src_id,
            .data = bundle.previous1_data,
            .len = bundle.previous1_len,
            .seq = (uint16_t)(e2e_seq - 1u),
            .active = (bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0,
            .timestamp_us = timestamp_us - 20000,
            .counters =
                {
                    .offer_count = &pipe->prev1_offer,
                    .accept_count = &pipe->prev1_accept,
                    .reject_count = &pipe->prev1_reject,
                },
        });
    }

    memcpy(frame.data, bundle.current_data, bundle.current_len);
    frame.len = (uint16_t)bundle.current_len;
    frame.timestamp_ms = timestamp_us / 1000;
    frame.active = (bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) != 0;
    /* Hand the end-to-end sequence to playout so it can conceal missing frames. */
    frame.seq = e2e_seq;
    frame.has_seq = true;

    bool admitted = false;
    esp_err_t ret = submit_audio_frame(&frame, src_id, &admitted);
    if (!admitted) {
        pipe->spi_rx_invalid++;
        return;
    }
    if (ret != ESP_OK) {
        pipe->play_queue_drop++;
        ESP_LOGD(TAG, "Failed to queue RX audio: %s", esp_err_to_name(ret));
    } else {
        pipe->play_queue_ok++;
    }
}

static void bridge_status_callback(const uart_bridge_status_t *status)
{
    bool ready = status->mesh_state == BRIDGE_MESH_STATE_ACTIVE && status->node_id != 0;
    bool user_enabled;
    bool logged_join = false;
    bool logged_leave = false;

    xSemaphoreTake(s_membership_mutex, portMAX_DELAY);
    user_enabled = mesh_intent_enabled();
    if (!user_enabled) {
        s_membership_observed = false;
        s_last_peer_count = 0;
    } else if (!ready || status->continuity_lost ||
               status->peer_count == BRIDGE_PEER_COUNT_UNKNOWN) {
        s_membership_observed = false;
        s_last_peer_count = 0;
    } else if (!s_membership_observed) {
        s_membership_observed = true;
        s_last_peer_count = status->peer_count;
    } else {
        if (status->peer_count > s_last_peer_count) {
            queue_notification_locked(AUDIO_NOTIFY_PEER_JOIN);
            logged_join = true;
        } else if (status->peer_count < s_last_peer_count) {
            queue_notification_locked(AUDIO_NOTIFY_PEER_LEAVE);
            logged_leave = true;
        }
        s_last_peer_count = status->peer_count;
    }
    atomic_store(&g_mesh_active, user_enabled && ready);
    if (user_enabled && ready && atomic_exchange(&s_enable_notification_pending, false)) {
        queue_notification_locked(AUDIO_NOTIFY_MESH_ENABLED);
    }
    xSemaphoreGive(s_membership_mutex);

    if (logged_join || logged_leave) {
        ESP_LOGI(TAG, "nRF peer count is now %u", status->peer_count);
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
        if (!mesh_intent_enabled()) {
            /* Reconciliation runs in app_main, never in this bridge RX callback. */
            ESP_LOGI(TAG, "Mesh-ready conflicts with disabled user intent");
            atomic_store(&g_mesh_active, false);
        }
        break;
    case BRIDGE_EVENT_PEER_JOINED:
        if (data != NULL && len >= 1 && data[0] >= 1 && data[0] <= MESH_MAX_NODES) {
            e2e_diag_reset_source(data[0]);
            ESP_LOGI(TAG, "Peer %u join event received; waiting for status confirmation", data[0]);
        } else {
            ESP_LOGW(TAG, "Ignoring peer join event with invalid node ID");
        }
        break;
    case BRIDGE_EVENT_PEER_LEFT:
        if (data != NULL && len >= 1 && data[0] >= 1 && data[0] <= MESH_MAX_NODES) {
            e2e_diag_reset_source(data[0]);
            ESP_LOGI(TAG, "Peer %u leave event received; waiting for status confirmation", data[0]);
        } else {
            ESP_LOGW(TAG, "Ignoring peer leave event with invalid node ID");
        }
        break;
    case BRIDGE_EVENT_MESH_STOPPED:
        ESP_LOGI(TAG, "nRF52840 mesh stopped");
        e2e_diag_reset_all_sources();
        audio_tx_cache_reset(&s_previous_audio);
        atomic_store(&g_mesh_active, false);
        atomic_store(&s_enable_notification_pending, false);
        reset_membership_tracking();
        break;
    case BRIDGE_EVENT_SYNC_LOST:
        ESP_LOGW(TAG, "nRF sync lost/rescanning; waiting for status confirmation");
        e2e_diag_reset_all_sources();
        break;
    case BRIDGE_EVENT_COMMAND_ACK:
    case BRIDGE_EVENT_BECAME_COORDINATOR:
        break;
    default:
        break;
    }
}

static void reconcile_mesh_state(int64_t now_ms)
{
    uart_bridge_status_t status;
    if (uart_bridge_get_status(&status) != ESP_OK) {
        atomic_store(&g_mesh_active, false);
        if (s_status_observed) {
            reset_membership_tracking();
        }
        s_status_observed = false;
        return;
    }

    bool ready = status.mesh_state == BRIDGE_MESH_STATE_ACTIVE && status.node_id != 0;
    if (!s_status_observed || status.mesh_state != s_last_mesh_state) {
        s_status_observed = true;
        s_last_mesh_state = status.mesh_state;
        s_reconcile_attempts = 0;
    }

    if (mesh_intent_enabled() && ready) {
        atomic_store(&g_mesh_active, true);
        return;
    }

    atomic_store(&g_mesh_active, false);
    bool disabled = status.mesh_state == BRIDGE_MESH_STATE_IDLE;
    if ((!mesh_intent_enabled() && disabled) ||
        (mesh_intent_enabled() && status.has_mesh_state && !disabled)) {
        s_reconcile_attempts = 0;
        return;
    }

    if (s_reconcile_attempts >= NRF_RECONCILE_MAX_ATTEMPTS ||
        now_ms - s_restart_attempt_ms < NRF_RECONCILE_INTERVAL_MS) {
        return;
    }

    s_restart_attempt_ms = now_ms;
    s_reconcile_attempts++;
    esp_err_t ret = mesh_intent_enabled() ? uart_bridge_mesh_enable() : uart_bridge_mesh_disable();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Mesh %s reconcile failed (%u/%u): %s",
                 mesh_intent_enabled() ? "enable" : "disable", s_reconcile_attempts,
                 NRF_RECONCILE_MAX_ATTEMPTS, esp_err_to_name(ret));
    }
}

void transport_nrf_reset_membership_tracking(void)
{
    reset_membership_tracking();
}

void transport_nrf_tick(int64_t now_ms)
{
    reconcile_mesh_state(now_ms);
    drain_notifications();
}

esp_err_t transport_nrf_init(void)
{
    s_membership_mutex = xSemaphoreCreateMutex();
    return s_membership_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

void transport_nrf_attach(void)
{
    uart_bridge_set_audio_callback(bridge_audio_callback);
    uart_bridge_set_event_callback(bridge_event_callback);
    uart_bridge_set_status_callback(bridge_status_callback);

    atomic_store(&g_mesh_active, false);
    transport_nrf_reset_reconciliation();
}
