#include "phone_audio.h"

#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "audio.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "phone_audio";

#define PROFILE_WAIT_MS 5000
#define PAIRING_WINDOW_MS 120000
#define CONTROL_QUEUE_LENGTH 8
#define CONTROL_POLL_MS 100
#define SCO_INTERVAL_US 7500
#define SCO_PENDING_TIMEOUT_US 30000
#define CONTROL_RETRY_LIMIT 80
#define ROUTE_FAILURE_LOG_MS 1000
#define TIMER_COMMAND_WAIT_MS 100
#define COMMAND_RESPONSE_WAIT_MS 2000
#define MAX_CLASSIC_BONDS 16
#define MAX_PENDING_SBC_CONFIGS 4
#define PROFILE_CT_INIT BIT0
#define PROFILE_TG_INIT BIT1
#define PROFILE_A2DP_INIT BIT2
#define PROFILE_CT_DEINIT BIT3
#define PROFILE_TG_DEINIT BIT4
#define PROFILE_A2DP_DEINIT BIT5
#define PROFILE_HF_INIT BIT6
#define PROFILE_HF_DEINIT BIT7

/* These labels are never used for another command while the corresponding
 * notification registration is alive. Passthrough commands use 3..15. A
 * timed-out label stays quarantined until its response or CT disconnect; the
 * bounded 13-label pool therefore fails closed instead of aliasing a stale
 * response to a newer command. */
#define AVRCP_LABEL_CAPABILITIES 0
#define AVRCP_LABEL_PLAY_STATUS 1
#define AVRCP_LABEL_TRACK 2
#define AVRCP_LABEL_COMMAND_FIRST 3
#define AVRCP_LABEL_COUNT 16

typedef enum {
    CONTROL_CONFIGURE,
    CONTROL_ACTIVE,
    CONTROL_CALL_CONFIGURE,
    CONTROL_CALL_ACTIVE,
} control_kind_t;

typedef struct {
    control_kind_t kind;
    uint32_t rate;
    uint8_t channels;
    bool active;
    uint32_t generation;
} control_intent_t;

typedef struct {
    uint32_t generation;
    bool configured;
    bool active;
    bool call_active;
    uint32_t rate;
    uint8_t channels;
    uint32_t call_rate;
    bool force_reconfigure;
} desired_snapshot_t;

typedef struct {
    bool valid;
    uint8_t bda[ESP_BD_ADDR_LEN];
    uint32_t rate;
    uint8_t channels;
} pending_sbc_config_t;

typedef struct {
    atomic_bool initialized;
    atomic_bool terminal;
    atomic_bool a2dp_connected;
    atomic_bool media_streaming;
    int64_t media_transition_ms;
    atomic_bool a2dp_audio_active;
    atomic_bool hf_slc_connected;
    atomic_bool sco_active;
    atomic_bool desired_call_active;
    atomic_uint call_sample_rate;
    atomic_uint pcm_drops;
    atomic_uint silence_padding;
    atomic_uint hfp_incoming_frames;
    atomic_uint hfp_incoming_drops;
    atomic_uint hfp_outgoing_callbacks;
    atomic_uint hfp_padded_samples;
    atomic_uint hfp_ready_notifications;
    atomic_uint hfp_pending_timeouts;
    atomic_uint expected_sco_samples;
    atomic_bool outgoing_ready_pending;
    atomic_llong outgoing_ready_pending_since_us;
    atomic_llong next_sco_ready_us;
    atomic_bool avrc_ct_connected;
    atomic_bool avrc_tg_connected;
    atomic_bool ct_play_status_supported;
    atomic_bool ct_track_supported;
    atomic_bool discoverable;
    atomic_bool bluetooth_live;
    atomic_bool desired_configured;
    atomic_bool desired_active;
    atomic_bool force_reconfigure;
    atomic_bool first_pcm_logged;
    atomic_bool inactive_pcm_logged;
    atomic_uint desired_generation;
    atomic_uint sample_rate;
    atomic_uchar channels;
    atomic_uchar pcm_channels;
    atomic_uchar command_transaction;
    atomic_ushort command_labels_in_use;
    SemaphoreHandle_t lifecycle_mutex;
    StaticSemaphore_t lifecycle_mutex_storage;
    uint8_t peer[ESP_BD_ADDR_LEN];
    uint8_t ct_peer[ESP_BD_ADDR_LEN];
    uint8_t tg_peer[ESP_BD_ADDR_LEN];
    pending_sbc_config_t pending_sbc[MAX_PENDING_SBC_CONFIGS];
    bool ct_peer_pending;
    bool tg_peer_pending;
    portMUX_TYPE peer_lock;
    EventGroupHandle_t profile_events;
    StaticEventGroup_t profile_event_storage;
    QueueHandle_t control_queue;
    StaticQueue_t control_queue_storage;
    uint8_t control_queue_data[CONTROL_QUEUE_LENGTH * sizeof(control_intent_t)];
    TaskHandle_t control_task;
    StaticTask_t control_task_storage;
    StackType_t control_task_stack[3072 / sizeof(StackType_t)];
    SemaphoreHandle_t control_done;
    StaticSemaphore_t control_done_storage;
    SemaphoreHandle_t command_response;
    StaticSemaphore_t command_response_storage;
    atomic_uchar expected_command_label;
    atomic_uchar command_response_code;
    phone_audio_call_state_t call_state;
    phone_audio_call_indicators_t call_indicators;
    portMUX_TYPE call_state_lock;
    atomic_uint pairing_deadline_tick;
    TimerHandle_t pairing_timer;
    StaticTimer_t pairing_timer_storage;
    bool controller_ready;
    bool bluedroid_ready;
    bool avrc_ct_ready;
    bool avrc_tg_ready;
    bool a2dp_ready;
    bool hf_ready;
    bool controller_enabled;
    bool bluedroid_enabled;
    bool applied_configured;
    bool applied_active;
    uint32_t applied_rate;
    uint8_t applied_channels;
    uint32_t applied_generation;
    bool applied_call_configured;
    bool applied_call_playback_active;
    bool applied_mic_active;
    uint32_t applied_call_rate;
    portMUX_TYPE desired_lock;
    atomic_bool shutdown_success;
    TickType_t route_failure_log_tick;
    bool route_failure_logged;
    _Alignas(4) int16_t a2dp_pcm_scratch[AUDIO_BLUETOOTH_PLAYBACK_MAX_FRAMES * 2u];
    _Alignas(4) int16_t hfp_pcm_incoming_scratch[320];
    _Alignas(4) int16_t hfp_pcm_outgoing_scratch[320];
} phone_audio_context_t;

static phone_audio_context_t s_phone = {
    .peer_lock = portMUX_INITIALIZER_UNLOCKED,
    .call_state_lock = portMUX_INITIALIZER_UNLOCKED,
    .desired_lock = portMUX_INITIALIZER_UNLOCKED,
};
static portMUX_TYPE s_lifecycle_init_lock = portMUX_INITIALIZER_UNLOCKED;

static bool peer_is_selected(const uint8_t *bda)
{
    bool same;
    portENTER_CRITICAL(&s_phone.peer_lock);
    same = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
           memcmp(s_phone.peer, bda, ESP_BD_ADDR_LEN) == 0;
    portEXIT_CRITICAL(&s_phone.peer_lock);
    return same;
}

static bool selected_peer_copy(uint8_t *bda)
{
    bool selected;
    portENTER_CRITICAL(&s_phone.peer_lock);
    selected = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire);
    if (selected) memcpy(bda, s_phone.peer, ESP_BD_ADDR_LEN);
    portEXIT_CRITICAL(&s_phone.peer_lock);
    return selected;
}

static bool avrc_role_matches_locked(const uint8_t *bda, bool ct)
{
    return (ct ? s_phone.ct_peer_pending : s_phone.tg_peer_pending) &&
           memcmp(ct ? s_phone.ct_peer : s_phone.tg_peer, bda, ESP_BD_ADDR_LEN) == 0;
}

static bool allocate_command_label(uint8_t *label)
{
    for (unsigned attempt = 0u; attempt < AVRCP_LABEL_COUNT - AVRCP_LABEL_COMMAND_FIRST; ++attempt) {
        uint8_t value = atomic_fetch_add_explicit(&s_phone.command_transaction, 1u,
                                                   memory_order_relaxed);
        uint8_t candidate = (uint8_t)(AVRCP_LABEL_COMMAND_FIRST +
                                      (value % (AVRCP_LABEL_COUNT - AVRCP_LABEL_COMMAND_FIRST)));
        uint16_t mask = (uint16_t)(1u << candidate);
        uint16_t old = atomic_fetch_or_explicit(&s_phone.command_labels_in_use, mask,
                                                memory_order_acq_rel);
        if ((old & mask) == 0u) {
            *label = candidate;
            return true;
        }
    }
    return false;
}

static void release_command_label(uint8_t label)
{
    if (label < AVRCP_LABEL_COMMAND_FIRST || label >= AVRCP_LABEL_COUNT) return;
    atomic_fetch_and_explicit(&s_phone.command_labels_in_use,
                              (uint16_t)~(1u << label), memory_order_acq_rel);
}

static esp_err_t set_discoverable_internal(bool discoverable)
{
    esp_err_t ret = esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        discoverable ? ESP_BT_GENERAL_DISCOVERABLE : ESP_BT_NON_DISCOVERABLE);
    if (ret == ESP_OK) atomic_store_explicit(&s_phone.discoverable, discoverable, memory_order_release);
    return ret;
}

static esp_err_t open_pairing_window_locked(void)
{
    esp_err_t ret = set_discoverable_internal(true);
    if (ret != ESP_OK) return ret;
    atomic_store_explicit(&s_phone.pairing_deadline_tick,
                          (uint32_t)(xTaskGetTickCount() + pdMS_TO_TICKS(PAIRING_WINDOW_MS)),
                          memory_order_release);
    if (xTimerReset(s_phone.pairing_timer, pdMS_TO_TICKS(TIMER_COMMAND_WAIT_MS)) != pdPASS) {
        (void)set_discoverable_internal(false);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void pairing_timer_callback(TimerHandle_t timer)
{
    (void)timer;
    if (atomic_load_explicit(&s_phone.initialized, memory_order_acquire) &&
        atomic_load_explicit(&s_phone.bluetooth_live, memory_order_acquire) &&
        !atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) {
        TickType_t now = xTaskGetTickCount();
        TickType_t deadline = (TickType_t)atomic_load_explicit(
            &s_phone.pairing_deadline_tick, memory_order_acquire);
        if ((int32_t)(now - deadline) < 0) {
            if (xTimerChangePeriod(s_phone.pairing_timer, deadline - now, 0) != pdPASS) {
                ESP_LOGW(TAG, "Pairing window timer refresh failed");
            }
            return;
        }
        esp_err_t ret = set_discoverable_internal(false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Pairing window close failed: %s", esp_err_to_name(ret));
        }
    }
}

static bool pairing_allowed(const uint8_t *bda)
{
    return atomic_load_explicit(&s_phone.initialized, memory_order_acquire) &&
           (atomic_load_explicit(&s_phone.discoverable, memory_order_acquire) ||
            peer_is_selected(bda));
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_CFM_REQ_EVT:
        (void)esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, pairing_allowed(param->cfm_req.bda));
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        bool allowed = !param->pin_req.min_16_digit && pairing_allowed(param->pin_req.bda);
        (void)esp_bt_gap_pin_reply(param->pin_req.bda, allowed, allowed ? 4 : 0, pin);
        break;
    }
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "Bluetooth authentication %s",
                 param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS ? "succeeded" : "failed");
        break;
    default:
        break;
    }
}

static uint32_t sbc_sample_rate(uint8_t value)
{
    if (value & ESP_A2D_SBC_CIE_SF_48K) return 48000u;
    if (value & ESP_A2D_SBC_CIE_SF_44K) return 44100u;
    if (value & ESP_A2D_SBC_CIE_SF_32K) return 32000u;
    if (value & ESP_A2D_SBC_CIE_SF_16K) return 16000u;
    return 0u;
}

static void log_a2dp_peer(const char *event, const uint8_t *bda)
{
    ESP_LOGI(TAG, "A2DP %s peer=%02x:%02x:%02x:%02x:%02x:%02x", event,
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool sbc_channel_mode_supported(uint8_t mode)
{
    return (mode & (ESP_A2D_SBC_CIE_CH_MODE_MONO | ESP_A2D_SBC_CIE_CH_MODE_STEREO |
                    ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL | ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO)) != 0u;
}

static void clear_pending_sbc_locked(const uint8_t *bda)
{
    for (size_t i = 0; i < MAX_PENDING_SBC_CONFIGS; ++i) {
        if (s_phone.pending_sbc[i].valid &&
            (bda == NULL || memcmp(s_phone.pending_sbc[i].bda, bda, ESP_BD_ADDR_LEN) == 0)) {
            s_phone.pending_sbc[i].valid = false;
        }
    }
}

static void clear_mismatched_pending_sbc_locked(const uint8_t *bda)
{
    for (size_t i = 0; i < MAX_PENDING_SBC_CONFIGS; ++i) {
        if (s_phone.pending_sbc[i].valid &&
            memcmp(s_phone.pending_sbc[i].bda, bda, ESP_BD_ADDR_LEN) != 0) {
            s_phone.pending_sbc[i].valid = false;
        }
    }
}

static bool take_pending_sbc_locked(const uint8_t *bda, uint32_t *rate, uint8_t *channels)
{
    for (size_t i = 0; i < MAX_PENDING_SBC_CONFIGS; ++i) {
        if (s_phone.pending_sbc[i].valid &&
            memcmp(s_phone.pending_sbc[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            *rate = s_phone.pending_sbc[i].rate;
            *channels = s_phone.pending_sbc[i].channels;
            s_phone.pending_sbc[i].valid = false;
            return true;
        }
    }
    return false;
}

static void cache_pending_sbc(const uint8_t *bda, uint32_t rate, uint8_t channels)
{
    size_t slot = MAX_PENDING_SBC_CONFIGS;
    for (size_t i = 0; i < MAX_PENDING_SBC_CONFIGS; ++i) {
        if (s_phone.pending_sbc[i].valid &&
            memcmp(s_phone.pending_sbc[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            slot = i;
            break;
        }
        if (slot == MAX_PENDING_SBC_CONFIGS && !s_phone.pending_sbc[i].valid) slot = i;
    }
    if (slot == MAX_PENDING_SBC_CONFIGS) {
        return;
    }
    memcpy(s_phone.pending_sbc[slot].bda, bda, ESP_BD_ADDR_LEN);
    s_phone.pending_sbc[slot].rate = rate;
    s_phone.pending_sbc[slot].channels = channels;
    s_phone.pending_sbc[slot].valid = true;
}

static void query_capabilities_if_selected(void);
static void queue_control(control_intent_t intent);

static void set_call_indicator(uint8_t *indicator, uint8_t value)
{
    portENTER_CRITICAL(&s_phone.call_state_lock);
    *indicator = value;
    phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
    portEXIT_CRITICAL(&s_phone.call_state_lock);
}

static void reset_sco_pacing(void)
{
    uint32_t rate = atomic_load_explicit(&s_phone.call_sample_rate, memory_order_acquire);
    atomic_store_explicit(&s_phone.expected_sco_samples, rate == 8000u ? 60u : 120u,
                          memory_order_release);
    atomic_store_explicit(&s_phone.outgoing_ready_pending, false, memory_order_release);
    atomic_store_explicit(&s_phone.outgoing_ready_pending_since_us, 0, memory_order_release);
    atomic_store_explicit(&s_phone.next_sco_ready_us, esp_timer_get_time() + SCO_INTERVAL_US,
                          memory_order_release);
}

/* IDF v6.1 legacy HCI PCM hooks. These APIs are deprecated upstream; keep this
 * adapter isolated so a future external-codec migration changes only this block. */
static void hfp_pcm_incoming(const uint8_t *data, uint32_t len)
{
    if (data == NULL || (len & 1u) != 0u ||
        !atomic_load_explicit(&s_phone.sco_active, memory_order_acquire)) return;
    atomic_fetch_add_explicit(&s_phone.hfp_incoming_frames, len / 2u, memory_order_relaxed);
    while (len != 0u) {
        uint32_t bytes = len > sizeof(s_phone.hfp_pcm_incoming_scratch)
                              ? sizeof(s_phone.hfp_pcm_incoming_scratch) : len;
        bytes &= ~1u;
        memcpy(s_phone.hfp_pcm_incoming_scratch, data, bytes);
        size_t accepted = audio_bluetooth_playback_enqueue(AUDIO_BLUETOOTH_CALL,
                                                            s_phone.hfp_pcm_incoming_scratch, bytes / 2u);
        if (accepted != bytes / 2u) {
            unsigned dropped = (unsigned)((bytes / 2u) - accepted);
            atomic_fetch_add_explicit(&s_phone.pcm_drops, dropped, memory_order_relaxed);
            atomic_fetch_add_explicit(&s_phone.hfp_incoming_drops, dropped,
                                      memory_order_relaxed);
        }
        data += bytes;
        len -= bytes;
    }
}

static uint32_t hfp_pcm_outgoing(uint8_t *data, uint32_t len)
{
    if (data == NULL || (len & 1u) != 0u) return 0u;
    atomic_store_explicit(&s_phone.outgoing_ready_pending, false, memory_order_release);
    atomic_store_explicit(&s_phone.outgoing_ready_pending_since_us, 0, memory_order_release);
    atomic_fetch_add_explicit(&s_phone.hfp_outgoing_callbacks, 1u, memory_order_relaxed);
    if (!atomic_load_explicit(&s_phone.sco_active, memory_order_acquire)) {
        memset(data, 0, len);
        atomic_fetch_add_explicit(&s_phone.silence_padding, len / 2u, memory_order_relaxed);
        atomic_fetch_add_explicit(&s_phone.hfp_padded_samples, len / 2u, memory_order_relaxed);
        return len;
    }
    uint32_t requested_samples = len / 2u;
    if (requested_samples > 0u && requested_samples <= AUDIO_ROUTE_MAX_MIC_READ_SAMPLES) {
        atomic_store_explicit(&s_phone.expected_sco_samples, requested_samples,
                              memory_order_release);
    }
    uint32_t remaining = len;
    uint8_t *destination = data;
    while (remaining != 0u) {
        uint32_t bytes = remaining > sizeof(s_phone.hfp_pcm_outgoing_scratch)
                              ? sizeof(s_phone.hfp_pcm_outgoing_scratch) : remaining;
        size_t samples = audio_bluetooth_mic_read(s_phone.hfp_pcm_outgoing_scratch, bytes / 2u);
        memcpy(destination, s_phone.hfp_pcm_outgoing_scratch,
               samples * sizeof(*s_phone.hfp_pcm_outgoing_scratch));
        if (samples * sizeof(*s_phone.hfp_pcm_outgoing_scratch) < bytes) {
            memset(destination + samples * sizeof(*s_phone.hfp_pcm_outgoing_scratch), 0,
                   bytes - samples * sizeof(*s_phone.hfp_pcm_outgoing_scratch));
        }
        if (samples != bytes / 2u) {
            unsigned padded = (unsigned)(bytes / 2u - samples);
            atomic_fetch_add_explicit(&s_phone.silence_padding, padded, memory_order_relaxed);
            atomic_fetch_add_explicit(&s_phone.hfp_padded_samples, padded,
                                      memory_order_relaxed);
        }
        destination += bytes;
        remaining -= bytes;
    }
    return len;
}

static void request_call_route(bool active, uint32_t rate)
{
    portENTER_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.desired_call_active, active, memory_order_release);
    if (rate != 0u) atomic_store_explicit(&s_phone.call_sample_rate, rate, memory_order_release);
    uint32_t generation = atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_acq_rel) + 1u;
    portEXIT_CRITICAL(&s_phone.desired_lock);
    queue_control((control_intent_t){CONTROL_CALL_ACTIVE, rate, 1u, active, generation});
}

static void queue_control(control_intent_t intent)
{
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        s_phone.control_queue == NULL) return;
    if (xQueueSend(s_phone.control_queue, &intent, 0) != pdPASS) {
        ESP_LOGW(TAG, "Audio control queue full; latest intent will be retried");
    }
}

static void request_configure(uint32_t rate, uint8_t channels)
{
    portENTER_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
    atomic_store_explicit(&s_phone.force_reconfigure, true, memory_order_release);
    atomic_store_explicit(&s_phone.sample_rate, rate, memory_order_release);
    atomic_store_explicit(&s_phone.channels, channels, memory_order_release);
    atomic_store_explicit(&s_phone.desired_configured, true, memory_order_release);
    uint32_t generation = atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_acq_rel) + 1u;
    portEXIT_CRITICAL(&s_phone.desired_lock);
    queue_control((control_intent_t){CONTROL_CONFIGURE, rate, channels, false, generation});
}

static void request_active(bool active)
{
    portENTER_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.desired_active, active, memory_order_release);
    bool was_active = atomic_load_explicit(&s_phone.media_streaming, memory_order_relaxed);
    if (was_active != active) s_phone.media_transition_ms = esp_timer_get_time() / 1000;
    atomic_store_explicit(&s_phone.media_streaming, active, memory_order_release);
    if (!active) atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
    uint32_t generation = atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_acq_rel) + 1u;
    portEXIT_CRITICAL(&s_phone.desired_lock);
    queue_control((control_intent_t){CONTROL_ACTIVE, 0, 0, active, generation});
}

static void request_unconfigure(void)
{
    portENTER_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.desired_configured, false, memory_order_release);
    atomic_store_explicit(&s_phone.desired_active, false, memory_order_release);
    atomic_store_explicit(&s_phone.media_streaming, false, memory_order_relaxed);
    s_phone.media_transition_ms = 0;
    atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
    uint32_t generation = atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_acq_rel) + 1u;
    portEXIT_CRITICAL(&s_phone.desired_lock);
    queue_control((control_intent_t){CONTROL_CONFIGURE, 0u, 0u, false, generation});
}

static bool retryable(esp_err_t ret)
{
    return ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_INVALID_STATE;
}

static esp_err_t route_result(esp_err_t ret)
{
    if (ret != ESP_OK && retryable(ret)) {
        TickType_t now = xTaskGetTickCount();
        if (!s_phone.route_failure_logged ||
            (TickType_t)(now - s_phone.route_failure_log_tick) >=
            pdMS_TO_TICKS(ROUTE_FAILURE_LOG_MS)) {
            ESP_LOGW(TAG, "Audio route intent remains pending: %s", esp_err_to_name(ret));
            s_phone.route_failure_log_tick = now;
            s_phone.route_failure_logged = true;
        }
    }
    return ret;
}

static esp_err_t apply_route_intent(void)
{
    desired_snapshot_t desired;
    portENTER_CRITICAL(&s_phone.desired_lock);
    desired.generation = atomic_load_explicit(&s_phone.desired_generation, memory_order_relaxed);
    desired.configured = atomic_load_explicit(&s_phone.desired_configured, memory_order_relaxed);
    desired.active = atomic_load_explicit(&s_phone.desired_active, memory_order_relaxed);
    desired.call_active = atomic_load_explicit(&s_phone.desired_call_active, memory_order_relaxed);
    desired.rate = atomic_load_explicit(&s_phone.sample_rate, memory_order_relaxed);
    desired.channels = atomic_load_explicit(&s_phone.channels, memory_order_relaxed);
    desired.call_rate = atomic_load_explicit(&s_phone.call_sample_rate, memory_order_relaxed);
    desired.force_reconfigure = atomic_load_explicit(&s_phone.force_reconfigure, memory_order_relaxed);
    portEXIT_CRITICAL(&s_phone.desired_lock);
    uint32_t generation = desired.generation;
    bool configured = desired.configured;
    bool active = desired.active;
    bool call_active = desired.call_active;
    uint32_t call_rate = desired.call_rate;
    uint32_t rate = desired.rate;
    uint8_t channels = desired.channels;
    esp_err_t ret = ESP_OK;
    if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) {
        configured = false;
        active = false;
        call_active = false;
    }
    if (atomic_load_explicit(&s_phone.desired_generation, memory_order_acquire) != generation) {
        return route_result(ESP_ERR_INVALID_STATE);
    }

    bool format_changed = desired.force_reconfigure ||
                          !s_phone.applied_configured || s_phone.applied_rate != rate ||
                          s_phone.applied_channels != channels;
    if (call_active) {
        ret = audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_MUSIC, false);
        if (ret != ESP_OK) return route_result(ret);
        s_phone.applied_active = false;
        if (!s_phone.applied_call_configured || s_phone.applied_call_rate != call_rate) {
            if (s_phone.applied_mic_active) {
                ret = audio_bluetooth_mic_set_active(false);
                if (ret != ESP_OK) return route_result(ret);
                s_phone.applied_mic_active = false;
            }
            if (s_phone.applied_call_playback_active) {
                ret = audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_CALL, false);
                if (ret != ESP_OK) return route_result(ret);
                s_phone.applied_call_playback_active = false;
            }
            ret = audio_bluetooth_playback_configure(AUDIO_BLUETOOTH_CALL, call_rate, 1u);
            if (ret == ESP_OK) ret = audio_bluetooth_mic_configure(call_rate);
            if (ret != ESP_OK) return route_result(ret);
            s_phone.applied_call_configured = true;
            s_phone.applied_call_rate = call_rate;
        }
        if (!s_phone.applied_call_playback_active) {
            ret = audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_CALL, true);
            if (ret != ESP_OK) return route_result(ret);
            s_phone.applied_call_playback_active = true;
        }
        if (!s_phone.applied_mic_active) {
            ret = audio_bluetooth_mic_set_active(true);
            if (ret != ESP_OK) {
                if (s_phone.applied_call_playback_active) {
                    esp_err_t call_off = audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_CALL, false);
                    if (call_off == ESP_OK) s_phone.applied_call_playback_active = false;
                }
                atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
                return route_result(ret);
            }
            s_phone.applied_mic_active = true;
        }
        if (s_phone.applied_call_playback_active && s_phone.applied_mic_active) {
            atomic_store_explicit(&s_phone.sco_active, true, memory_order_release);
        } else {
            atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
        }
    } else {
        esp_err_t mic_ret = s_phone.applied_mic_active ? audio_bluetooth_mic_set_active(false) : ESP_OK;
        if (mic_ret == ESP_OK) s_phone.applied_mic_active = false;
        esp_err_t call_ret = s_phone.applied_call_playback_active
                                 ? audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_CALL, false)
                                 : ESP_OK;
        if (call_ret == ESP_OK) s_phone.applied_call_playback_active = false;
        if (mic_ret == ESP_OK && call_ret == ESP_OK) {
            s_phone.applied_call_configured = false;
            atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
        } else {
            atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
            return route_result(mic_ret != ESP_OK ? mic_ret : call_ret);
        }
    }
    if (configured && format_changed) {
        ret = audio_bluetooth_playback_configure(AUDIO_BLUETOOTH_MUSIC, rate, channels);
        if (ret != ESP_OK) return route_result(ret);
        s_phone.applied_configured = true;
        s_phone.applied_rate = rate;
        s_phone.applied_channels = channels;
        s_phone.applied_active = false;
        portENTER_CRITICAL(&s_phone.desired_lock);
        if (atomic_load_explicit(&s_phone.desired_generation, memory_order_relaxed) == generation) {
            atomic_store_explicit(&s_phone.force_reconfigure, false, memory_order_relaxed);
        }
        portEXIT_CRITICAL(&s_phone.desired_lock);
        atomic_store_explicit(&s_phone.pcm_channels, channels, memory_order_release);
        ESP_LOGI(TAG, "A2DP route format applied: rate=%lu channels=%u", rate, channels);
    }
    bool route_active = !call_active && active && s_phone.applied_configured;
    if (!configured && !call_active) route_active = false;
    if (s_phone.applied_active != route_active) {
        ret = audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_MUSIC, route_active);
        if (ret != ESP_OK) return route_result(ret);
        s_phone.applied_active = route_active;
        ESP_LOGI(TAG, "A2DP route active=%s applied", route_active ? "yes" : "no");
    }
    if (!configured && !call_active) {
        s_phone.applied_configured = false;
        atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
    } else {
        atomic_store_explicit(&s_phone.a2dp_audio_active, route_active, memory_order_release);
    }
    if (atomic_load_explicit(&s_phone.desired_generation, memory_order_acquire) != generation) {
        atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
        return route_result(ESP_ERR_INVALID_STATE);
    }
    s_phone.applied_generation = generation;
    return ESP_OK;
}

static void hfp_callback(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event == ESP_HF_CLIENT_PROF_STATE_EVT) {
        if (s_phone.profile_events != NULL) {
            if (param->prof_stat.state == ESP_HF_INIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_HF_INIT);
            else if (param->prof_stat.state == ESP_HF_DEINIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_HF_DEINIT);
        }
        return;
    }
    if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) return;
    if (event == ESP_HF_CLIENT_CONNECTION_STATE_EVT && !peer_is_selected(param->conn_stat.remote_bda)) {
        if (param->conn_stat.state != ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
            (void)esp_hf_client_disconnect(param->conn_stat.remote_bda);
        }
        return;
    }
    if (event == ESP_HF_CLIENT_AUDIO_STATE_EVT && !peer_is_selected(param->audio_stat.remote_bda)) return;
    if ((event == ESP_HF_CLIENT_CIND_CALL_EVT || event == ESP_HF_CLIENT_CIND_CALL_SETUP_EVT ||
         event == ESP_HF_CLIENT_CIND_CALL_HELD_EVT || event == ESP_HF_CLIENT_RING_IND_EVT ||
         event == ESP_HF_CLIENT_CLIP_EVT || event == ESP_HF_CLIENT_VOLUME_CONTROL_EVT ||
         event == ESP_HF_CLIENT_AT_RESPONSE_EVT) &&
        !atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire)) return;
    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
            portENTER_CRITICAL(&s_phone.call_state_lock);
            memset(&s_phone.call_indicators, 0, sizeof(s_phone.call_indicators));
            portEXIT_CRITICAL(&s_phone.call_state_lock);
            atomic_store_explicit(&s_phone.hf_slc_connected, true, memory_order_release);
            portENTER_CRITICAL(&s_phone.call_state_lock);
            s_phone.call_indicators.slc_connected = true;
            phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
            portEXIT_CRITICAL(&s_phone.call_state_lock);
        }
        atomic_store_explicit(&s_phone.hf_slc_connected,
                              param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED,
                              memory_order_release);
        if (param->conn_stat.state != ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
            portENTER_CRITICAL(&s_phone.call_state_lock);
            memset(&s_phone.call_indicators, 0, sizeof(s_phone.call_indicators));
            phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
            portEXIT_CRITICAL(&s_phone.call_state_lock);
        }
        if (param->conn_stat.state != ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
            reset_sco_pacing();
        }
        if (!atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire)) request_call_route(false, 0u);
        ESP_LOGI(TAG, "HFP connection state=%d", param->conn_stat.state);
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
        bool connected = param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                         param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC;
        uint32_t rate = param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC ? 16000u : 8000u;
        atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
        portENTER_CRITICAL(&s_phone.call_state_lock);
        s_phone.call_indicators.audio_connected = connected;
        s_phone.call_indicators.wideband = param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC;
        s_phone.call_indicators.sample_rate = connected ? rate : 0u;
        phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
        portEXIT_CRITICAL(&s_phone.call_state_lock);
        request_call_route(connected, connected ? rate : 0u);
        reset_sco_pacing();
        ESP_LOGI(TAG, "HFP audio state=%d rate=%lu", param->audio_stat.state, (unsigned long)rate);
        break;
    }
    case ESP_HF_CLIENT_CIND_CALL_EVT:
        set_call_indicator(&s_phone.call_indicators.call, (uint8_t)param->call.status);
        ESP_LOGI(TAG, "HFP call=%u", (unsigned)param->call.status);
        break;
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        set_call_indicator(&s_phone.call_indicators.call_setup, (uint8_t)param->call_setup.status);
        ESP_LOGI(TAG, "HFP callsetup=%u", (unsigned)param->call_setup.status);
        break;
    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        set_call_indicator(&s_phone.call_indicators.call_held, (uint8_t)param->call_held.status);
        ESP_LOGI(TAG, "HFP callheld=%u", (unsigned)param->call_held.status);
        break;
    case ESP_HF_CLIENT_RING_IND_EVT:
        ESP_LOGI(TAG, "HFP ring");
        break;
    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGI(TAG, "HFP caller ID received (redacted)");
        break;
    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "HFP volume target=%d level=%d", param->volume_control.type, param->volume_control.volume);
        break;
    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGI(TAG, "HFP AT response=%d", param->at_response.code);
        break;
    default:
        break;
    }
}

static void control_worker(void *arg)
{
    (void)arg;
    control_intent_t intent;
    unsigned terminal_cycles = 0u;
    atomic_store_explicit(&s_phone.shutdown_success, false, memory_order_release);
    while (true) {
        bool terminal = atomic_load_explicit(&s_phone.terminal, memory_order_acquire);
        bool generation_changed;
        portENTER_CRITICAL(&s_phone.desired_lock);
        generation_changed = atomic_load_explicit(&s_phone.desired_generation, memory_order_relaxed) !=
                             s_phone.applied_generation;
        portEXIT_CRITICAL(&s_phone.desired_lock);
        if (generation_changed || terminal) {
            (void)apply_route_intent();
        }

        int64_t now_us = esp_timer_get_time();
        bool sco_ready = !terminal && s_phone.applied_call_playback_active &&
                         s_phone.applied_mic_active &&
                         atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire) &&
                         atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                         atomic_load_explicit(&s_phone.sco_active, memory_order_acquire);
        if (!sco_ready) {
            reset_sco_pacing();
        } else {
            bool pending = atomic_load_explicit(&s_phone.outgoing_ready_pending, memory_order_acquire);
            int64_t pending_since = atomic_load_explicit(&s_phone.outgoing_ready_pending_since_us,
                                                         memory_order_acquire);
            if (pending && pending_since != 0 && now_us - pending_since >= SCO_PENDING_TIMEOUT_US) {
                bool expected = true;
                if (atomic_compare_exchange_strong_explicit(&s_phone.outgoing_ready_pending, &expected,
                                                            false, memory_order_acq_rel,
                                                            memory_order_acquire)) {
                    atomic_store_explicit(&s_phone.outgoing_ready_pending_since_us, 0,
                                          memory_order_release);
                    atomic_fetch_add_explicit(&s_phone.hfp_pending_timeouts, 1u, memory_order_relaxed);
                    atomic_store_explicit(&s_phone.next_sco_ready_us, now_us + SCO_INTERVAL_US,
                                          memory_order_release);
                }
            }
            int64_t deadline = atomic_load_explicit(&s_phone.next_sco_ready_us, memory_order_acquire);
            if (deadline == 0) {
                deadline = now_us + SCO_INTERVAL_US;
                atomic_store_explicit(&s_phone.next_sco_ready_us, deadline, memory_order_release);
            }
            uint32_t expected_samples = atomic_load_explicit(&s_phone.expected_sco_samples,
                                                               memory_order_acquire);
            uint32_t call_rate = atomic_load_explicit(&s_phone.call_sample_rate,
                                                      memory_order_acquire);
            size_t required_input_samples = call_rate == 8000u
                                                ? (size_t)expected_samples * 2u + 1u
                                                : (size_t)expected_samples + 1u;
            if (!atomic_load_explicit(&s_phone.outgoing_ready_pending, memory_order_acquire) &&
                now_us >= deadline &&
                audio_bluetooth_mic_available_samples() >= required_input_samples) {
                bool expected_pending = false;
                if (atomic_compare_exchange_strong_explicit(&s_phone.outgoing_ready_pending,
                                                            &expected_pending, true,
                                                            memory_order_acq_rel,
                                                            memory_order_acquire)) {
                    int64_t notify_us = esp_timer_get_time();
                    atomic_store_explicit(&s_phone.outgoing_ready_pending_since_us, notify_us,
                                          memory_order_release);
                    atomic_fetch_add_explicit(&s_phone.hfp_ready_notifications, 1u,
                                              memory_order_relaxed);
                    esp_hf_client_outgoing_data_ready();
                    atomic_store_explicit(&s_phone.next_sco_ready_us,
                                           notify_us - deadline >= SCO_INTERVAL_US
                                              ? notify_us + SCO_INTERVAL_US
                                              : deadline + SCO_INTERVAL_US,
                                          memory_order_release);
                }
            }
        }

        if (terminal) {
            terminal_cycles++;
            bool inactive = !s_phone.applied_active && !s_phone.applied_call_playback_active &&
                            !s_phone.applied_mic_active &&
                            !s_phone.applied_call_configured;
            if (inactive || terminal_cycles >= CONTROL_RETRY_LIMIT) break;
            (void)xQueueReceive(s_phone.control_queue, &intent, 1);
        } else {
            TickType_t wait_ticks = pdMS_TO_TICKS(CONTROL_POLL_MS);
            if (sco_ready) {
                int64_t wait_us = atomic_load_explicit(&s_phone.next_sco_ready_us, memory_order_acquire) -
                                  esp_timer_get_time();
                int64_t pending_since = atomic_load_explicit(&s_phone.outgoing_ready_pending_since_us,
                                                             memory_order_acquire);
                if (atomic_load_explicit(&s_phone.outgoing_ready_pending, memory_order_acquire) &&
                    pending_since != 0) {
                    int64_t timeout_wait = pending_since + SCO_PENDING_TIMEOUT_US -
                                           esp_timer_get_time();
                    if (timeout_wait < wait_us) wait_us = timeout_wait;
                }
                uint32_t wait_ms = wait_us > 0 ? (uint32_t)((wait_us + 999) / 1000) : 1u;
                TickType_t until_ready = pdMS_TO_TICKS(wait_ms);
                if (until_ready == 0) until_ready = 1;
                wait_ticks = until_ready < wait_ticks ? until_ready : wait_ticks;
            }
            (void)xQueueReceive(s_phone.control_queue, &intent, wait_ticks);
        }
    }
    atomic_store_explicit(&s_phone.a2dp_audio_active, false, memory_order_release);
    for (unsigned attempt = 0u; attempt < 2u &&
                                 (s_phone.applied_mic_active || s_phone.applied_call_playback_active);
         ++attempt) {
        if (s_phone.applied_mic_active && audio_bluetooth_mic_set_active(false) == ESP_OK) {
            s_phone.applied_mic_active = false;
        }
        if (s_phone.applied_call_playback_active &&
            audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_CALL, false) == ESP_OK) {
            s_phone.applied_call_playback_active = false;
        }
        if (s_phone.applied_mic_active || s_phone.applied_call_playback_active) vTaskDelay(1);
    }
    if (!s_phone.applied_mic_active && !s_phone.applied_call_playback_active && s_phone.applied_active) {
        if (audio_bluetooth_playback_set_active(AUDIO_BLUETOOTH_MUSIC, false) == ESP_OK) {
            s_phone.applied_active = false;
        }
    }
    atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
    atomic_store_explicit(&s_phone.shutdown_success,
                          !s_phone.applied_mic_active && !s_phone.applied_call_playback_active &&
                              !s_phone.applied_active,
                          memory_order_release);
    xSemaphoreGive(s_phone.control_done);
    vTaskDelete(NULL);
}

static void a2dp_data_callback(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len < sizeof(int16_t)) return;
    if (!atomic_load_explicit(&s_phone.a2dp_audio_active, memory_order_acquire)) {
        bool expected = false;
        if (atomic_compare_exchange_strong_explicit(&s_phone.inactive_pcm_logged, &expected, true,
                                                    memory_order_acq_rel, memory_order_acquire)) {
            ESP_LOGW(TAG, "A2DP PCM callback received while route is not active; waiting for format");
        }
        return;
    }
    uint8_t channels = atomic_load_explicit(&s_phone.pcm_channels, memory_order_acquire);
    if (channels != 1u && channels != 2u) return;
    size_t frame_bytes = (size_t)channels * sizeof(int16_t);
    size_t frames = len / frame_bytes;
    size_t received = frames;
    size_t accepted_total = 0u;
    while (frames != 0u) {
        size_t chunk = frames > AUDIO_BLUETOOTH_PLAYBACK_MAX_FRAMES ? AUDIO_BLUETOOTH_PLAYBACK_MAX_FRAMES : frames;
        size_t chunk_bytes = chunk * frame_bytes;
        memcpy(s_phone.a2dp_pcm_scratch, data, chunk_bytes);
        size_t accepted = audio_bluetooth_playback_enqueue(AUDIO_BLUETOOTH_MUSIC,
                                                            s_phone.a2dp_pcm_scratch, chunk);
        accepted_total += accepted;
        if (accepted != chunk) break;
        data += chunk_bytes;
        frames -= chunk;
    }
    bool expected = false;
    if (accepted_total != 0u && atomic_compare_exchange_strong_explicit(
                                   &s_phone.first_pcm_logged, &expected, true,
                                   memory_order_acq_rel, memory_order_acquire)) {
        ESP_LOGI(TAG, "A2DP first PCM callback: received_frames=%u accepted_frames=%u",
                 (unsigned)received, (unsigned)accepted_total);
    }
}

static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
        if (s_phone.profile_events != NULL) {
            if (param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_A2DP_INIT);
            else if (param->a2d_prof_stat.init_state == ESP_A2D_DEINIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_A2DP_DEINIT);
        }
        break;
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            if (atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire)) {
                (void)esp_a2d_sink_disconnect(param->conn_stat.remote_bda);
                break;
            }
            portENTER_CRITICAL(&s_phone.peer_lock);
            memcpy(s_phone.peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            bool reject_ct = s_phone.ct_peer_pending &&
                             memcmp(s_phone.ct_peer, s_phone.peer, ESP_BD_ADDR_LEN) != 0;
            bool reject_tg = s_phone.tg_peer_pending &&
                             memcmp(s_phone.tg_peer, s_phone.peer, ESP_BD_ADDR_LEN) != 0;
            portEXIT_CRITICAL(&s_phone.peer_lock);
            if (reject_ct || reject_tg) {
                ESP_LOGW(TAG, "Split-role AVRCP peer observed; resetting the selected A2DP session");
                (void)esp_a2d_sink_disconnect(param->conn_stat.remote_bda);
                break;
            }
            atomic_store_explicit(&s_phone.a2dp_connected, true, memory_order_release);
            atomic_store_explicit(&s_phone.first_pcm_logged, false, memory_order_release);
            atomic_store_explicit(&s_phone.inactive_pcm_logged, false, memory_order_release);
            log_a2dp_peer("connected", param->conn_stat.remote_bda);
            if (s_phone.hf_ready) {
                esp_err_t hf_ret = esp_hf_client_connect(param->conn_stat.remote_bda);
                if (hf_ret != ESP_OK) ESP_LOGW(TAG, "HFP SLC connect request failed: %s", esp_err_to_name(hf_ret));
            }
            atomic_store_explicit(&s_phone.avrc_ct_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.avrc_tg_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.ct_play_status_supported, false, memory_order_release);
            atomic_store_explicit(&s_phone.ct_track_supported, false, memory_order_release);
            (void)set_discoverable_internal(false);
            bool matching_ct = false;
            portENTER_CRITICAL(&s_phone.peer_lock);
            if (s_phone.ct_peer_pending &&
                memcmp(s_phone.ct_peer, s_phone.peer, ESP_BD_ADDR_LEN) == 0) {
                atomic_store_explicit(&s_phone.avrc_ct_connected, true, memory_order_release);
                matching_ct = true;
            }
            if (s_phone.tg_peer_pending &&
                memcmp(s_phone.tg_peer, s_phone.peer, ESP_BD_ADDR_LEN) == 0) {
                atomic_store_explicit(&s_phone.avrc_tg_connected, true, memory_order_release);
            }
            portEXIT_CRITICAL(&s_phone.peer_lock);
            if (matching_ct) query_capabilities_if_selected();
            uint32_t pending_rate = 0u;
            uint8_t pending_channels = 0u;
            bool have_pending = false;
            portENTER_CRITICAL(&s_phone.peer_lock);
            clear_mismatched_pending_sbc_locked(param->conn_stat.remote_bda);
            have_pending = take_pending_sbc_locked(param->conn_stat.remote_bda,
                                                   &pending_rate, &pending_channels);
            portEXIT_CRITICAL(&s_phone.peer_lock);
            if (have_pending) request_configure(pending_rate, pending_channels);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED &&
                   peer_is_selected(param->conn_stat.remote_bda)) {
            if (s_phone.hf_ready) (void)esp_hf_client_disconnect(param->conn_stat.remote_bda);
            atomic_store_explicit(&s_phone.hf_slc_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
            request_call_route(false, 0u);
            request_active(false);
            portENTER_CRITICAL(&s_phone.peer_lock);
            atomic_store_explicit(&s_phone.a2dp_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.first_pcm_logged, false, memory_order_release);
            atomic_store_explicit(&s_phone.inactive_pcm_logged, false, memory_order_release);
            clear_pending_sbc_locked(NULL);
            portEXIT_CRITICAL(&s_phone.peer_lock);
            log_a2dp_peer("disconnected", param->conn_stat.remote_bda);
            atomic_store_explicit(&s_phone.avrc_ct_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.avrc_tg_connected, false, memory_order_release);
            atomic_store_explicit(&s_phone.ct_play_status_supported, false, memory_order_release);
            atomic_store_explicit(&s_phone.ct_track_supported, false, memory_order_release);
            request_unconfigure();
            portENTER_CRITICAL(&s_phone.call_state_lock);
            memset(&s_phone.call_indicators, 0, sizeof(s_phone.call_indicators));
            phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
            portEXIT_CRITICAL(&s_phone.call_state_lock);
            portENTER_CRITICAL(&s_phone.peer_lock);
            s_phone.ct_peer_pending = false;
            s_phone.tg_peer_pending = false;
            portEXIT_CRITICAL(&s_phone.peer_lock);
            (void)set_discoverable_internal(false);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            portENTER_CRITICAL(&s_phone.peer_lock);
            clear_pending_sbc_locked(param->conn_stat.remote_bda);
            portEXIT_CRITICAL(&s_phone.peer_lock);
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        {
            const uint8_t *bda = param->audio_cfg.remote_bda;
            uint8_t codec_type = param->audio_cfg.mcc.type;
            if (codec_type != ESP_A2D_MCT_SBC) {
                ESP_LOGW(TAG, "A2DP AUDIO_CFG peer=%02x:%02x:%02x:%02x:%02x:%02x rejected: codec_type=%u",
                         bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], codec_type);
                portENTER_CRITICAL(&s_phone.peer_lock);
                bool selected = atomic_load_explicit(&s_phone.a2dp_connected,
                                                     memory_order_acquire) &&
                                memcmp(s_phone.peer, bda, ESP_BD_ADDR_LEN) == 0;
                clear_pending_sbc_locked(bda);
                portEXIT_CRITICAL(&s_phone.peer_lock);
                if (selected) {
                    request_unconfigure();
                }
                break;
            }
            const esp_a2d_cie_sbc_t *sbc = &param->audio_cfg.mcc.cie.sbc_info;
            uint32_t rate = sbc_sample_rate(sbc->samp_freq);
            bool channels_valid = sbc_channel_mode_supported(sbc->ch_mode);
            uint8_t channels = (sbc->ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) != 0u ? 1u : 2u;
            if (rate == 0u || !channels_valid) {
                ESP_LOGW(TAG, "A2DP AUDIO_CFG peer=%02x:%02x:%02x:%02x:%02x:%02x rejected: %s%s",
                         bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                         rate == 0u ? "unsupported rate" : "",
                         !channels_valid ? (rate == 0u ? ", unsupported channels" : "unsupported channels") : "");
                portENTER_CRITICAL(&s_phone.peer_lock);
                bool selected = atomic_load_explicit(&s_phone.a2dp_connected,
                                                     memory_order_acquire) &&
                                memcmp(s_phone.peer, bda, ESP_BD_ADDR_LEN) == 0;
                clear_pending_sbc_locked(bda);
                portEXIT_CRITICAL(&s_phone.peer_lock);
                if (selected) {
                    request_unconfigure();
                }
                break;
            }
            ESP_LOGI(TAG, "A2DP AUDIO_CFG peer=%02x:%02x:%02x:%02x:%02x:%02x accepted: SBC rate=%lu channels=%u mode=0x%02x",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], rate, channels, sbc->ch_mode);
            bool selected = false;
            portENTER_CRITICAL(&s_phone.peer_lock);
            selected = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                       memcmp(s_phone.peer, bda, ESP_BD_ADDR_LEN) == 0;
            if (!selected) cache_pending_sbc(bda, rate, channels);
            portEXIT_CRITICAL(&s_phone.peer_lock);
            if (selected) request_configure(rate, channels);
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        if (peer_is_selected(param->audio_stat.remote_bda)) {
            const char *state = param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED
                                    ? "started"
                                     : "suspended";
            ESP_LOGI(TAG, "A2DP audio %s", state);
            request_active(param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        }
        break;
    default:
        break;
    }
}

static void register_remote_notification(uint8_t event_id)
{
    uint8_t label = event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE ? AVRCP_LABEL_PLAY_STATUS : AVRCP_LABEL_TRACK;
    esp_err_t ret = esp_avrc_ct_send_register_notification_cmd(label, event_id, 0);
    if (ret != ESP_OK) ESP_LOGW(TAG, "AVRCP notification registration failed: %s", esp_err_to_name(ret));
}

static void query_capabilities_if_selected(void)
{
    uint8_t peer[ESP_BD_ADDR_LEN];
    if (!selected_peer_copy(peer) || !atomic_load_explicit(&s_phone.avrc_ct_connected, memory_order_acquire)) return;
    esp_err_t ret = esp_avrc_ct_send_get_rn_capabilities_cmd(AVRCP_LABEL_CAPABILITIES);
    if (ret != ESP_OK) ESP_LOGW(TAG, "AVRCP capability query failed: %s", esp_err_to_name(ret));
}

static void avrc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_PROF_STATE_EVT:
        if (s_phone.profile_events != NULL) {
            if (param->avrc_ct_init_stat.state == ESP_AVRC_INIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_CT_INIT);
            else if (param->avrc_ct_init_stat.state == ESP_AVRC_DEINIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_CT_DEINIT);
        }
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        uint8_t expected = atomic_load_explicit(&s_phone.expected_command_label, memory_order_acquire);
        release_command_label(param->psth_rsp.tl);
        if (param->psth_rsp.tl == expected && s_phone.command_response != NULL) {
            atomic_store_explicit(&s_phone.command_response_code, (uint8_t)param->psth_rsp.rsp_code,
                                  memory_order_release);
            xSemaphoreGive(s_phone.command_response);
        }
        break;
    }
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        portENTER_CRITICAL(&s_phone.peer_lock);
        if (param->conn_stat.connected) {
            if (atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                memcmp(s_phone.peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN) != 0) {
                uint8_t selected_peer[ESP_BD_ADDR_LEN];
                memcpy(selected_peer, s_phone.peer, ESP_BD_ADDR_LEN);
                s_phone.ct_peer_pending = false;
                atomic_store_explicit(&s_phone.avrc_ct_connected, false, memory_order_release);
                atomic_store_explicit(&s_phone.ct_play_status_supported, false, memory_order_release);
                atomic_store_explicit(&s_phone.ct_track_supported, false, memory_order_release);
                portEXIT_CRITICAL(&s_phone.peer_lock);
                ESP_LOGW(TAG, "Split-role AVRCP CT peer observed; resetting the selected A2DP session");
                (void)esp_a2d_sink_disconnect(selected_peer);
                break;
            }
            memcpy(s_phone.ct_peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            s_phone.ct_peer_pending = true;
        } else if (avrc_role_matches_locked(param->conn_stat.remote_bda, true)) {
            s_phone.ct_peer_pending = false;
            atomic_store_explicit(&s_phone.ct_play_status_supported, false, memory_order_release);
            atomic_store_explicit(&s_phone.ct_track_supported, false, memory_order_release);
            atomic_store_explicit(&s_phone.command_labels_in_use, 0u, memory_order_release);
            atomic_store_explicit(&s_phone.expected_command_label, UINT8_MAX,
                                  memory_order_release);
        }
        bool selected = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                        memcmp(s_phone.peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN) == 0;
        portEXIT_CRITICAL(&s_phone.peer_lock);
        if (!selected) break;
        atomic_store_explicit(&s_phone.avrc_ct_connected, param->conn_stat.connected, memory_order_release);
        if (param->conn_stat.connected) query_capabilities_if_selected();
        break;
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        if (!atomic_load_explicit(&s_phone.avrc_ct_connected, memory_order_acquire)) break;
        if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &param->get_rn_caps_rsp.evt_set, ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
            atomic_store_explicit(&s_phone.ct_play_status_supported, true, memory_order_release);
            register_remote_notification(ESP_AVRC_RN_PLAY_STATUS_CHANGE);
        }
        if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &param->get_rn_caps_rsp.evt_set, ESP_AVRC_RN_TRACK_CHANGE)) {
            atomic_store_explicit(&s_phone.ct_track_supported, true, memory_order_release);
            register_remote_notification(ESP_AVRC_RN_TRACK_CHANGE);
        }
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        if (param->change_ntf.event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE && atomic_load_explicit(&s_phone.ct_play_status_supported, memory_order_acquire)) register_remote_notification(param->change_ntf.event_id);
        else if (param->change_ntf.event_id == ESP_AVRC_RN_TRACK_CHANGE && atomic_load_explicit(&s_phone.ct_track_supported, memory_order_acquire)) register_remote_notification(param->change_ntf.event_id);
        break;
    default:
        break;
    }
}

static void avrc_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_PROF_STATE_EVT:
        if (s_phone.profile_events != NULL) {
            if (param->avrc_tg_init_stat.state == ESP_AVRC_INIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_TG_INIT);
            else if (param->avrc_tg_init_stat.state == ESP_AVRC_DEINIT_SUCCESS) xEventGroupSetBits(s_phone.profile_events, PROFILE_TG_DEINIT);
        }
        break;
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        portENTER_CRITICAL(&s_phone.peer_lock);
        if (param->conn_stat.connected) {
            if (atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                memcmp(s_phone.peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN) != 0) {
                uint8_t selected_peer[ESP_BD_ADDR_LEN];
                memcpy(selected_peer, s_phone.peer, ESP_BD_ADDR_LEN);
                s_phone.tg_peer_pending = false;
                atomic_store_explicit(&s_phone.avrc_tg_connected, false, memory_order_release);
                portEXIT_CRITICAL(&s_phone.peer_lock);
                ESP_LOGW(TAG, "Split-role AVRCP TG peer observed; resetting the selected A2DP session");
                (void)esp_a2d_sink_disconnect(selected_peer);
                break;
            }
            memcpy(s_phone.tg_peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            s_phone.tg_peer_pending = true;
        } else if (avrc_role_matches_locked(param->conn_stat.remote_bda, false)) {
            s_phone.tg_peer_pending = false;
        }
        bool selected = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire) &&
                        memcmp(s_phone.peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN) == 0;
        portEXIT_CRITICAL(&s_phone.peer_lock);
        if (selected) atomic_store_explicit(&s_phone.avrc_tg_connected, param->conn_stat.connected, memory_order_release);
        break;
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire)) break;
        ESP_LOGD(TAG, "Ignoring AVRCP absolute volume request");
        break;
    default:
        break;
    }
}

static esp_err_t wait_profile(EventBits_t bit)
{
    EventBits_t result = xEventGroupWaitBits(s_phone.profile_events, bit, pdTRUE, pdTRUE, pdMS_TO_TICKS(PROFILE_WAIT_MS));
    return (result & bit) != 0u ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t stop_control_resources(void)
{
    esp_err_t first_error = ESP_OK;
    if (s_phone.pairing_timer != NULL) {
        if (xTimerStop(s_phone.pairing_timer, pdMS_TO_TICKS(TIMER_COMMAND_WAIT_MS)) != pdPASS) {
            ESP_LOGW(TAG, "Pairing timer stop failed");
            first_error = ESP_FAIL;
        }
    }
    if (s_phone.control_task != NULL) {
        portENTER_CRITICAL(&s_phone.desired_lock);
        atomic_store_explicit(&s_phone.desired_active, false, memory_order_relaxed);
        atomic_store_explicit(&s_phone.desired_configured, false, memory_order_relaxed);
        atomic_store_explicit(&s_phone.desired_call_active, false, memory_order_relaxed);
        uint32_t generation = atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_relaxed) + 1u;
        portEXIT_CRITICAL(&s_phone.desired_lock);
        control_intent_t intent = {CONTROL_ACTIVE, 0, 0, false, generation};
        if (s_phone.control_queue != NULL) (void)xQueueSend(s_phone.control_queue, &intent, 0);
        if (xSemaphoreTake(s_phone.control_done, pdMS_TO_TICKS(PROFILE_WAIT_MS)) != pdTRUE) {
            ESP_LOGW(TAG, "Audio control worker did not stop");
            if (first_error == ESP_OK) first_error = ESP_ERR_TIMEOUT;
        } else {
            s_phone.control_task = NULL;
            if (!atomic_load_explicit(&s_phone.shutdown_success, memory_order_acquire) &&
                first_error == ESP_OK) first_error = ESP_ERR_TIMEOUT;
        }
    }
    return first_error;
}

static esp_err_t deinit_profile(bool *ready, EventBits_t bit, esp_err_t (*deinit)(void))
{
    if (!*ready) return ESP_OK;
    if ((xEventGroupGetBits(s_phone.profile_events) & bit) != 0u) {
        xEventGroupClearBits(s_phone.profile_events, bit);
        *ready = false;
        return ESP_OK;
    }
    esp_err_t ret = deinit();
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        EventBits_t result = xEventGroupWaitBits(s_phone.profile_events, bit, pdFALSE, pdTRUE,
                                                  pdMS_TO_TICKS(PROFILE_WAIT_MS));
        if ((result & bit) != 0u) {
            xEventGroupClearBits(s_phone.profile_events, bit);
            *ready = false;
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Profile deinit completion was not observed");
        return ret == ESP_OK ? ESP_ERR_TIMEOUT : ret;
    }
    return ret;
}

static esp_err_t phone_audio_cleanup_locked(void)
{
    esp_err_t first_error = ESP_OK;
    bool sco_was_active = atomic_load_explicit(&s_phone.sco_active, memory_order_acquire);
    atomic_store_explicit(&s_phone.terminal, true, memory_order_release);
    atomic_store_explicit(&s_phone.bluetooth_live, false, memory_order_release);
    atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
    portENTER_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.desired_call_active, false, memory_order_relaxed);
    atomic_store_explicit(&s_phone.desired_active, false, memory_order_relaxed);
    atomic_store_explicit(&s_phone.desired_configured, false, memory_order_relaxed);
    atomic_fetch_add_explicit(&s_phone.desired_generation, 1u, memory_order_relaxed);
    portEXIT_CRITICAL(&s_phone.desired_lock);
    atomic_store_explicit(&s_phone.avrc_ct_connected, false, memory_order_release);
    atomic_store_explicit(&s_phone.avrc_tg_connected, false, memory_order_release);
    atomic_store_explicit(&s_phone.ct_play_status_supported, false, memory_order_release);
    atomic_store_explicit(&s_phone.ct_track_supported, false, memory_order_release);

    if (s_phone.bluedroid_ready) {
        esp_err_t ret = set_discoverable_internal(false);
        if (ret != ESP_OK) first_error = ret;
    }
    esp_err_t ret = stop_control_resources();
    if (ret != ESP_OK && first_error == ESP_OK) first_error = ret;
    if (s_phone.control_task != NULL) {
        /* Route shutdown is a prerequisite for tearing down profile callbacks. */
        return first_error == ESP_OK ? ESP_ERR_TIMEOUT : first_error;
    }
    uint8_t hf_peer[ESP_BD_ADDR_LEN];
    if (s_phone.hf_ready && selected_peer_copy(hf_peer)) {
        if (sco_was_active) {
            (void)esp_hf_client_disconnect_audio(hf_peer);
        }
        if (atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire)) {
            (void)esp_hf_client_disconnect(hf_peer);
        }
    }
    atomic_store_explicit(&s_phone.hf_slc_connected, false, memory_order_release);
    ret = deinit_profile(&s_phone.hf_ready, PROFILE_HF_DEINIT, esp_hf_client_deinit);
    if (ret != ESP_OK && first_error == ESP_OK) first_error = ret;
    ret = deinit_profile(&s_phone.avrc_ct_ready, PROFILE_CT_DEINIT, esp_avrc_ct_deinit);
    if (ret != ESP_OK && first_error == ESP_OK) first_error = ret;
    ret = deinit_profile(&s_phone.avrc_tg_ready, PROFILE_TG_DEINIT, esp_avrc_tg_deinit);
    if (ret != ESP_OK && first_error == ESP_OK) first_error = ret;
    ret = deinit_profile(&s_phone.a2dp_ready, PROFILE_A2DP_DEINIT, esp_a2d_sink_deinit);
    if (ret != ESP_OK && first_error == ESP_OK) first_error = ret;
    if (s_phone.hf_ready || s_phone.avrc_ct_ready || s_phone.avrc_tg_ready || s_phone.a2dp_ready) {
        /* Keep lower layers alive until every profile callback confirms deinit. */
        return first_error == ESP_OK ? ESP_ERR_TIMEOUT : first_error;
    }
    if (s_phone.bluedroid_ready && s_phone.bluedroid_enabled) {
        ret = esp_bluedroid_disable();
        if (ret == ESP_OK) s_phone.bluedroid_enabled = false;
        else if (first_error == ESP_OK) first_error = ret;
    }
    if (s_phone.bluedroid_ready && !s_phone.bluedroid_enabled) {
        ret = esp_bluedroid_deinit();
        if (ret == ESP_OK) s_phone.bluedroid_ready = false;
        else if (first_error == ESP_OK) first_error = ret;
    }
    if (s_phone.bluedroid_ready || s_phone.bluedroid_enabled) {
        /* The controller must remain intact until Bluedroid has fully gone away. */
        return first_error == ESP_OK ? ESP_ERR_INVALID_STATE : first_error;
    }
    if (s_phone.controller_ready && s_phone.controller_enabled) {
        ret = esp_bt_controller_disable();
        if (ret == ESP_OK) s_phone.controller_enabled = false;
        else if (first_error == ESP_OK) first_error = ret;
    }
    if (s_phone.controller_ready && !s_phone.controller_enabled) {
        ret = esp_bt_controller_deinit();
        if (ret == ESP_OK) s_phone.controller_ready = false;
        else if (first_error == ESP_OK) first_error = ret;
    }
    if (!s_phone.avrc_ct_ready && !s_phone.avrc_tg_ready && !s_phone.a2dp_ready && !s_phone.hf_ready &&
        !s_phone.bluedroid_ready && !s_phone.controller_ready && s_phone.control_task == NULL) {
        portENTER_CRITICAL(&s_phone.desired_lock);
        atomic_store_explicit(&s_phone.media_streaming, false, memory_order_relaxed);
        s_phone.media_transition_ms = 0;
        portEXIT_CRITICAL(&s_phone.desired_lock);
        atomic_store_explicit(&s_phone.a2dp_connected, false, memory_order_release);
        atomic_store_explicit(&s_phone.avrc_ct_connected, false, memory_order_release);
        atomic_store_explicit(&s_phone.avrc_tg_connected, false, memory_order_release);
        atomic_store_explicit(&s_phone.hf_slc_connected, false, memory_order_release);
        atomic_store_explicit(&s_phone.sco_active, false, memory_order_release);
        portENTER_CRITICAL(&s_phone.peer_lock);
        memset(s_phone.peer, 0, sizeof(s_phone.peer));
        memset(s_phone.ct_peer, 0, sizeof(s_phone.ct_peer));
        memset(s_phone.tg_peer, 0, sizeof(s_phone.tg_peer));
        s_phone.ct_peer_pending = false;
        s_phone.tg_peer_pending = false;
        portEXIT_CRITICAL(&s_phone.peer_lock);
        portENTER_CRITICAL(&s_phone.call_state_lock);
        memset(&s_phone.call_indicators, 0, sizeof(s_phone.call_indicators));
        phone_audio_call_state_reduce(&s_phone.call_indicators, &s_phone.call_state);
        portEXIT_CRITICAL(&s_phone.call_state_lock);
        atomic_store_explicit(&s_phone.initialized, false, memory_order_release);
    }
    return first_error;
}

esp_err_t phone_audio_init(void)
{
    portENTER_CRITICAL(&s_lifecycle_init_lock);
    if (s_phone.lifecycle_mutex == NULL) {
        s_phone.lifecycle_mutex =
            xSemaphoreCreateMutexStatic(&s_phone.lifecycle_mutex_storage);
        if (s_phone.lifecycle_mutex == NULL) {
            portEXIT_CRITICAL(&s_lifecycle_init_lock);
            return ESP_ERR_NO_MEM;
        }
    }
    SemaphoreHandle_t lifecycle_mutex = s_phone.lifecycle_mutex;
    portEXIT_CRITICAL(&s_lifecycle_init_lock);
    if (xSemaphoreTake(lifecycle_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.initialized, memory_order_acquire)) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    atomic_store_explicit(&s_phone.initialized, true, memory_order_release);
    esp_err_t ret = ESP_OK;
    if (s_phone.profile_events == NULL) s_phone.profile_events = xEventGroupCreateStatic(&s_phone.profile_event_storage);
    if (s_phone.control_done == NULL) s_phone.control_done = xSemaphoreCreateBinaryStatic(&s_phone.control_done_storage);
    if (s_phone.command_response == NULL) s_phone.command_response = xSemaphoreCreateBinaryStatic(&s_phone.command_response_storage);
    atomic_store_explicit(&s_phone.expected_command_label, UINT8_MAX, memory_order_release);
    if (s_phone.control_queue == NULL) s_phone.control_queue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(control_intent_t),
                                                                                     s_phone.control_queue_data,
                                                                                     &s_phone.control_queue_storage);
    s_phone.control_task = xTaskCreateStatic(control_worker, "phone_ctrl", 3072, NULL, 5,
                                             s_phone.control_task_stack,
                                             &s_phone.control_task_storage);
    if (s_phone.pairing_timer == NULL) s_phone.pairing_timer = xTimerCreateStatic("pair_window", pdMS_TO_TICKS(PAIRING_WINDOW_MS),
                                                                                    pdFALSE, NULL, pairing_timer_callback,
                                                                                    &s_phone.pairing_timer_storage);
    if (s_phone.profile_events == NULL || s_phone.control_done == NULL ||
        s_phone.command_response == NULL || s_phone.control_queue == NULL || s_phone.control_task == NULL ||
        s_phone.pairing_timer == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) goto fail;
    esp_bt_controller_config_t controller_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&controller_config);
    if (ret != ESP_OK) goto fail;
    s_phone.controller_ready = true;
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) goto fail;
    s_phone.controller_enabled = true;
    esp_bluedroid_config_t bluedroid_config = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_config);
    if (ret != ESP_OK) goto fail;
    s_phone.bluedroid_ready = true;
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) goto fail;
    s_phone.bluedroid_enabled = true;
    atomic_store_explicit(&s_phone.bluetooth_live, true, memory_order_release);
    ret = esp_hf_client_register_callback(hfp_callback);
    if (ret != ESP_OK) goto fail;
    /* Legacy HCI CVSD/mSBC PCM callbacks are registered while Bluedroid is
     * enabled and remain static for the terminal lifetime. */
    ret = esp_hf_client_register_data_callback(hfp_pcm_incoming, hfp_pcm_outgoing);
    if (ret != ESP_OK) goto fail;
    ret = esp_bt_gap_set_device_name("OMI ESP32-S31");
    if (ret != ESP_OK) goto fail;
    esp_bt_io_cap_t io_cap = ESP_BT_IO_CAP_NONE;
    ret = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_cap, sizeof(io_cap));
    if (ret != ESP_OK) goto fail;
    ret = esp_bt_gap_register_callback(gap_callback);
    if (ret != ESP_OK) goto fail;
    ret = esp_avrc_ct_register_callback(avrc_ct_callback);
    if (ret != ESP_OK) goto fail;
    ret = esp_avrc_ct_init();
    if (ret != ESP_OK) goto fail;
    s_phone.avrc_ct_ready = true;
    ret = wait_profile(PROFILE_CT_INIT);
    if (ret != ESP_OK) goto fail;
    ret = esp_avrc_tg_register_callback(avrc_tg_callback);
    if (ret != ESP_OK) goto fail;
    ret = esp_avrc_tg_init();
    if (ret != ESP_OK) goto fail;
    s_phone.avrc_tg_ready = true;
    ret = wait_profile(PROFILE_TG_INIT);
    if (ret != ESP_OK) goto fail;
    esp_avrc_rn_evt_cap_mask_t capabilities = {0};
    ret = esp_avrc_tg_set_rn_evt_cap(&capabilities);
    if (ret != ESP_OK) goto fail;
    esp_avrc_psth_bit_mask_t commands = {0};
    ret = esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &commands);
    if (ret != ESP_OK) goto fail;
    ret = esp_a2d_register_callback(a2dp_callback);
    if (ret != ESP_OK) goto fail;
    ret = esp_a2d_sink_register_data_callback(a2dp_data_callback);
    if (ret != ESP_OK) goto fail;
    ret = esp_a2d_sink_init();
    if (ret != ESP_OK) goto fail;
    s_phone.a2dp_ready = true;
    ret = wait_profile(PROFILE_A2DP_INIT);
    if (ret != ESP_OK) goto fail;
    ret = esp_hf_client_init();
    if (ret != ESP_OK) goto fail;
    s_phone.hf_ready = true;
    ret = wait_profile(PROFILE_HF_INIT);
    if (ret != ESP_OK) goto fail;
    ESP_LOGI(TAG, "A2DP path: internal SBC (external codec disabled)");
    int bond_count = esp_bt_gap_get_bond_device_num();
    if (bond_count < 0) {
        ret = ESP_FAIL;
        goto fail;
    }
    ESP_LOGI(TAG, "Bluetooth bonds: %d", bond_count);
    ret = set_discoverable_internal(bond_count == 0);
    if (ret != ESP_OK) goto fail;
    if (bond_count == 0) {
        atomic_store_explicit(&s_phone.pairing_deadline_tick,
                              (uint32_t)(xTaskGetTickCount() +
                                         pdMS_TO_TICKS(PAIRING_WINDOW_MS)),
                              memory_order_release);
        if (xTimerStart(s_phone.pairing_timer, pdMS_TO_TICKS(TIMER_COMMAND_WAIT_MS)) != pdPASS) {
            (void)set_discoverable_internal(false);
            ret = ESP_FAIL;
            goto fail;
        }
        ESP_LOGI(TAG, "Pairing window opened for 120 seconds");
    } else {
        ESP_LOGI(TAG, "Pairing window closed; device remains connectable and non-discoverable");
    }
    xSemaphoreGive(lifecycle_mutex);
    phone_audio_state_t state;
    if (phone_audio_get_state(&state) == ESP_OK) {
        ESP_LOGI(TAG, "Phone audio initialized: initialized=%s discoverable=%s pairing=%s",
                 state.initialized ? "yes" : "no",
                 state.discoverable ? "yes" : "no",
                 state.pairing_window_open ? "open" : "closed");
    } else {
        ESP_LOGW(TAG, "Phone audio initialized, but state query failed");
    }
    return ESP_OK;
fail:
    ESP_LOGE(TAG, "Bluetooth phone audio initialization failed: %s", esp_err_to_name(ret));
    (void)phone_audio_cleanup_locked();
    xSemaphoreGive(lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_deinit(void)
{
    if (s_phone.lifecycle_mutex == NULL ||
        xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t ret = phone_audio_cleanup_locked();
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_get_state(phone_audio_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    state->initialized = atomic_load_explicit(&s_phone.initialized, memory_order_acquire);
    state->terminal = atomic_load_explicit(&s_phone.terminal, memory_order_acquire);
    state->a2dp_connected = atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire);
    portENTER_CRITICAL(&s_phone.desired_lock);
    state->media_streaming = atomic_load_explicit(&s_phone.media_streaming, memory_order_relaxed);
    state->media_transition_ms = s_phone.media_transition_ms;
    portEXIT_CRITICAL(&s_phone.desired_lock);
    state->a2dp_audio_active = atomic_load_explicit(&s_phone.a2dp_audio_active, memory_order_acquire);
    state->avrcp_connected = atomic_load_explicit(&s_phone.avrc_ct_connected, memory_order_acquire);
    state->discoverable = atomic_load_explicit(&s_phone.discoverable, memory_order_acquire);
    state->pairing_window_open = state->discoverable;
    state->volume_control_limited = true;
    state->sample_rate = atomic_load_explicit(&s_phone.sample_rate, memory_order_acquire);
    state->channels = atomic_load_explicit(&s_phone.channels, memory_order_acquire);
    portENTER_CRITICAL(&s_phone.call_state_lock);
    state->call = s_phone.call_state;
    portEXIT_CRITICAL(&s_phone.call_state_lock);
    return ESP_OK;
}

void phone_audio_log_stats(void)
{
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire)) return;
    ESP_LOGI(TAG,
             "HFP stats incoming_frames=%lu incoming_drops=%lu outgoing_callbacks=%lu "
             "padded_samples=%lu ready_notifications=%lu pending_timeouts=%lu "
             "expected_samples=%lu slc=%u sco=%u ready_pending=%u",
             (unsigned long)atomic_load_explicit(&s_phone.hfp_incoming_frames, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.hfp_incoming_drops, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.hfp_outgoing_callbacks, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.hfp_padded_samples, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.hfp_ready_notifications, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.hfp_pending_timeouts, memory_order_relaxed),
             (unsigned long)atomic_load_explicit(&s_phone.expected_sco_samples, memory_order_relaxed),
             (unsigned)atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_relaxed),
             (unsigned)atomic_load_explicit(&s_phone.sco_active, memory_order_relaxed),
             (unsigned)atomic_load_explicit(&s_phone.outgoing_ready_pending, memory_order_relaxed));
}

esp_err_t phone_audio_get_call_state(phone_audio_call_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_phone.call_state_lock);
    *state = s_phone.call_state;
    portEXIT_CRITICAL(&s_phone.call_state_lock);
    return ESP_OK;
}

static phone_audio_call_phase_t call_phase_snapshot(void)
{
    phone_audio_call_phase_t phase;
    portENTER_CRITICAL(&s_phone.call_state_lock);
    phase = s_phone.call_state.phase;
    portEXIT_CRITICAL(&s_phone.call_state_lock);
    return phase;
}

static esp_err_t call_command(esp_err_t (*command)(void), bool answer)
{
    if (s_phone.lifecycle_mutex == NULL || xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t peer[ESP_BD_ADDR_LEN];
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        !atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire) ||
        !selected_peer_copy(peer) ||
        (answer ? call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_INCOMING
                : (call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_INCOMING &&
                   call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING &&
                   call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING &&
                   call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_ACTIVE &&
                   call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_HELD))) goto done;
    ret = command();
done:
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_answer_call(void)
{
    return call_command(esp_hf_client_answer_call, true);
}

esp_err_t phone_audio_reject_call(void)
{
    return call_command(esp_hf_client_reject_call, false);
}

esp_err_t phone_audio_dial(const char *number)
{
    if (number != NULL && strnlen(number, ESP_BT_HF_CLIENT_NUMBER_LEN) >= ESP_BT_HF_CLIENT_NUMBER_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_phone.lifecycle_mutex == NULL || xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t peer[ESP_BD_ADDR_LEN];
    char copied_number[ESP_BT_HF_CLIENT_NUMBER_LEN];
    const char *argument = NULL;
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        !atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire) ||
        !selected_peer_copy(peer) ||
        call_phase_snapshot() != PHONE_AUDIO_CALL_PHASE_IDLE) goto done;
    if (number != NULL) {
        strncpy(copied_number, number, sizeof(copied_number) - 1u);
        copied_number[sizeof(copied_number) - 1u] = '\0';
        argument = copied_number;
    }
    ret = esp_hf_client_dial(argument);
done:
    memset(copied_number, 0, sizeof(copied_number));
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_redial(void)
{
    return phone_audio_dial(NULL);
}

esp_err_t phone_audio_connect_audio(void)
{
    if (s_phone.lifecycle_mutex == NULL || xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t peer[ESP_BD_ADDR_LEN];
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire) && selected_peer_copy(peer)) {
        ret = esp_hf_client_connect_audio(peer);
    }
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_disconnect_audio(void)
{
    if (s_phone.lifecycle_mutex == NULL || xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t peer[ESP_BD_ADDR_LEN];
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (atomic_load_explicit(&s_phone.hf_slc_connected, memory_order_acquire) && selected_peer_copy(peer)) {
        /* The HFP audio callback owns route convergence; do not pre-clear local state. */
        ret = esp_hf_client_disconnect_audio(peer);
    }
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_forget_all_bonds(void)
{
    if (s_phone.lifecycle_mutex == NULL ||
        xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t ret = ESP_OK;
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        !s_phone.bluedroid_enabled) {
        ret = ESP_ERR_INVALID_STATE;
        goto done;
    }
    if (atomic_load_explicit(&s_phone.a2dp_connected, memory_order_acquire)) {
        ret = ESP_ERR_INVALID_STATE;
        ESP_LOGW(TAG, "Forget all bonds rejected while a phone is connected");
        goto done;
    }

    int bond_count = esp_bt_gap_get_bond_device_num();
    if (bond_count < 0 || bond_count > MAX_CLASSIC_BONDS) {
        ret = ESP_FAIL;
        ESP_LOGW(TAG, "Forget all bonds failed: invalid bond count");
        goto done;
    }
    esp_bd_addr_t bonds[MAX_CLASSIC_BONDS];
    int listed = bond_count;
    if (bond_count != 0) {
        ret = esp_bt_gap_get_bond_device_list(&listed, bonds);
        if (ret != ESP_OK || listed < 0 || listed > MAX_CLASSIC_BONDS) {
            ESP_LOGW(TAG, "Forget all bonds failed while listing %d bonds: %s", bond_count,
                     ret == ESP_OK ? "invalid bond count" : esp_err_to_name(ret));
            goto done;
        }
    }
    for (int i = 0; i < listed; ++i) {
        ret = esp_bt_gap_remove_bond_device(bonds[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Forget all bonds failed after removing %d of %d bonds: %s", i,
                     listed, esp_err_to_name(ret));
            goto done;
        }
    }
    ESP_LOGI(TAG, "Forgot all Bluetooth bonds: removed %d", listed);
    ret = open_pairing_window_locked();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Replacement-phone pairing window failed to open: %s", esp_err_to_name(ret));
    }
done:
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_set_discoverable(bool discoverable)
{
    if (s_phone.lifecycle_mutex == NULL ||
        xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!atomic_load_explicit(&s_phone.initialized, memory_order_acquire) ||
        atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        s_phone.pairing_timer == NULL) {
        xSemaphoreGive(s_phone.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = set_discoverable_internal(discoverable);
    if (ret == ESP_OK) {
        if (discoverable) {
            atomic_store_explicit(&s_phone.pairing_deadline_tick,
                                  (uint32_t)(xTaskGetTickCount() +
                                             pdMS_TO_TICKS(PAIRING_WINDOW_MS)),
                                  memory_order_release);
        }
        BaseType_t timer_result = discoverable
                                      ? xTimerReset(s_phone.pairing_timer, pdMS_TO_TICKS(TIMER_COMMAND_WAIT_MS))
                                      : xTimerStop(s_phone.pairing_timer, pdMS_TO_TICKS(TIMER_COMMAND_WAIT_MS));
        if (timer_result != pdPASS) {
            if (discoverable) (void)set_discoverable_internal(false);
            ret = ESP_FAIL;
        }
    }
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

static bool media_command_allowed_now(void)
{
    phone_audio_call_phase_t phase;
    portENTER_CRITICAL(&s_phone.call_state_lock);
    phase = s_phone.call_state.phase;
    bool sco_active = atomic_load_explicit(&s_phone.sco_active, memory_order_relaxed);
    portEXIT_CRITICAL(&s_phone.call_state_lock);
    return phase == PHONE_AUDIO_CALL_PHASE_IDLE && !sco_active;
}

static esp_err_t send_media_command(esp_avrc_pt_cmd_t command)
{
    if (s_phone.lifecycle_mutex == NULL ||
        xSemaphoreTake(s_phone.lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t peer[ESP_BD_ADDR_LEN];
    esp_err_t ret = ESP_OK;
    if (atomic_load_explicit(&s_phone.terminal, memory_order_acquire) ||
        !atomic_load_explicit(&s_phone.avrc_ct_connected, memory_order_acquire) ||
        !selected_peer_copy(peer)) {
        ret = ESP_ERR_INVALID_STATE;
        goto done;
    }
    (void)xSemaphoreTake(s_phone.command_response, 0);
    uint8_t pressed_transaction;
    uint8_t released_transaction;
    if (!allocate_command_label(&pressed_transaction)) {
        ret = ESP_ERR_NO_MEM;
        goto done;
    }
    if (!allocate_command_label(&released_transaction)) {
        release_command_label(pressed_transaction);
        ret = ESP_ERR_NO_MEM;
        goto done;
    }
    atomic_store_explicit(&s_phone.expected_command_label, released_transaction, memory_order_release);
    if (!media_command_allowed_now()) {
        release_command_label(pressed_transaction);
        release_command_label(released_transaction);
        atomic_store_explicit(&s_phone.expected_command_label, UINT8_MAX, memory_order_release);
        ret = ESP_ERR_INVALID_STATE;
        goto done;
    }
    ret = esp_avrc_ct_send_passthrough_cmd(pressed_transaction, command,
                                            ESP_AVRC_PT_CMD_STATE_PRESSED);
    if (ret != ESP_OK) {
        release_command_label(pressed_transaction);
        release_command_label(released_transaction);
        ret = ESP_ERR_INVALID_STATE;
    } else {
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_err_t release_result = esp_avrc_ct_send_passthrough_cmd(
            released_transaction, command, ESP_AVRC_PT_CMD_STATE_RELEASED);
        if (release_result != ESP_OK) {
            release_command_label(released_transaction);
            ESP_LOGW(TAG, "AVRCP passthrough release send failed: %s",
                     esp_err_to_name(release_result));
            ret = release_result;
        } else if (xSemaphoreTake(s_phone.command_response,
                                  pdMS_TO_TICKS(COMMAND_RESPONSE_WAIT_MS)) != pdTRUE) {
            atomic_store_explicit(&s_phone.expected_command_label, UINT8_MAX,
                                  memory_order_release);
            ret = ESP_ERR_TIMEOUT;
        } else if (atomic_load_explicit(&s_phone.command_response_code,
                                       memory_order_acquire) != ESP_AVRC_RSP_ACCEPT) {
            ret = ESP_FAIL;
        } else {
            ret = ESP_OK;
        }
    }
    atomic_store_explicit(&s_phone.expected_command_label, UINT8_MAX, memory_order_release);
done:
    xSemaphoreGive(s_phone.lifecycle_mutex);
    return ret;
}

esp_err_t phone_audio_play(void)
{
    return send_media_command(ESP_AVRC_PT_CMD_PLAY);
}

esp_err_t phone_audio_pause(void)
{
    return send_media_command(ESP_AVRC_PT_CMD_PAUSE);
}

esp_err_t phone_audio_stop(void)
{
    return send_media_command(ESP_AVRC_PT_CMD_STOP);
}

esp_err_t phone_audio_next(void)
{
    return send_media_command(ESP_AVRC_PT_CMD_FORWARD);
}

esp_err_t phone_audio_previous(void)
{
    return send_media_command(ESP_AVRC_PT_CMD_BACKWARD);
}
