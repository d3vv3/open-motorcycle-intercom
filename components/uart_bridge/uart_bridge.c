/**
 * @file uart_bridge.c
 * @brief SPI Bridge to nRF52840 Mesh Radio (ESP32-S3 = SPI Slave)
 *
 * The nRF52840 is the SPI master and polls the ESP32 periodically.
 * Each SPI transaction is full-duplex: while the master clocks out a
 * packet to the slave, the slave simultaneously sends its queued packet
 * (or a zero-filled idle frame) to the master.
 *
 * Packet wire format:
 *   [0xAA] [LEN] [SEQ] [TYPE] [PAYLOAD...] [CRC8]
 * LEN covers: [SEQ] [TYPE] [PAYLOAD...]
 * CRC8 is calculated over: [LEN] [SEQ] [TYPE] [PAYLOAD...]
 * Padded with zeros to BRIDGE_SPI_MAX_XFER bytes.
 *
 * "uart_bridge" naming retained for API compatibility.
 */

#include "uart_bridge.h"

#include <string.h>

#include "audio_bundle.h"
#include "bridge_frame.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

static const char *TAG = "spi_bridge";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define RX_TASK_STACK_SIZE 4096
#define RX_TASK_PRIORITY   5
#define RX_TASK_CORE       0
#define TX_AUDIO_QUEUE_SIZE 16
#define TX_AUDIO_MAX_AGE_US 120000
#define COMMAND_ACK_TIMEOUT_MS 300
#define COMMAND_MAX_ATTEMPTS   3
#define STATUS_STALE_TIMEOUT_US 5000000
#define SPI_MAX_PAYLOAD (BRIDGE_SPI_MAX_XFER - BRIDGE_FRAME_OVERHEAD)

_Static_assert(SPI_MAX_PAYLOAD <= BRIDGE_FRAME_MAX_PAYLOAD,
               "SPI bridge payload exceeds frame codec capacity");

_Static_assert(MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 6 <= BRIDGE_SPI_MAX_XFER,
               "source-prefixed audio v2 bundle must fit one SPI transfer");

typedef struct {
    uint8_t buf[BRIDGE_SPI_MAX_XFER];
    uint16_t len;
    int64_t queued_at_us;
} tx_entry_t;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static TaskHandle_t s_rx_task = NULL;

static uart_bridge_audio_cb_t s_audio_cb = NULL;
static uart_bridge_event_cb_t s_event_cb = NULL;
static uart_bridge_status_cb_t s_status_cb = NULL;

static uart_bridge_status_t s_status = {0};
static bool s_connected = false;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ack_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_command_in_progress = false;
static volatile bool s_command_ack_received = false;
static volatile uint8_t s_command_ack_generation = 0;
static volatile uint8_t s_command_ack_command = 0;
static volatile int8_t s_command_ack_result = 0;
static uint8_t s_command_generation = 0;

/*
 * TX double-buffer: s_tx_buf is the staging area where the app writes
 * new packets.  s_tx_dma_buf is what the SPI DMA actually reads from.
 * prepare_tx_buf() copies staging -> DMA so the audio callback can
 * safely write to s_tx_buf while a DMA transfer is in flight.
 */
static WORD_ALIGNED_ATTR uint8_t s_tx_buf[BRIDGE_SPI_MAX_XFER];     /* Staging buffer */
static WORD_ALIGNED_ATTR uint8_t s_tx_dma_buf[BRIDGE_SPI_MAX_XFER]; /* DMA buffer */
static WORD_ALIGNED_ATTR uint8_t s_rx_buf[BRIDGE_SPI_MAX_XFER];

/* Mutex protecting packet construction and the multi-producer audio ring head. */
static SemaphoreHandle_t s_tx_mutex;

/* TX audio ring. Producers share s_tx_mutex; the SPI task is the sole consumer. */
static tx_entry_t s_audio_q[TX_AUDIO_QUEUE_SIZE];
static volatile uint8_t s_audio_head = 0;
static volatile uint8_t s_audio_tail = 0;
static SemaphoreHandle_t s_audio_slots_free;
static SemaphoreHandle_t s_audio_slots_used;

/* Control slot: protected by s_tx_mutex (low-frequency path) */
static tx_entry_t s_ctrl_pending;
static volatile bool s_ctrl_pending_valid = false;

/* Pipeline diagnostics */
static uint32_t s_audio_tx_queued = 0;    /* Audio frames queued for SPI TX */
static uint32_t s_audio_tx_overwrite = 0; /* Audio frames that overwrote unsent ones */
static uint32_t s_audio_tx_enqueue_timeout = 0;
static uint32_t s_audio_tx_invalid_size = 0;
static uint32_t s_audio_tx_stale_drop = 0;
static uint32_t s_audio_tx_wait_count = 0;
static uint64_t s_audio_tx_wait_sum_us = 0;
static uint32_t s_audio_tx_wait_max_us = 0;
static uint32_t s_audio_rx_count = 0;     /* Audio frames received from nRF */
static uint8_t s_bridge_tx_seq = 0;
static uint8_t s_bridge_rx_expected = 0;
static bool s_bridge_rx_seq_init = false;
static uint32_t s_bridge_rx_seq_gaps = 0;
static uint32_t s_bridge_rx_crc_fail = 0;
static uint32_t s_bridge_rx_bad_sync = 0;
static uint32_t s_bridge_rx_bad_len = 0;
static uint32_t s_bridge_rx_trunc = 0;
static uint32_t s_status_rx_valid = 0;
static uint32_t s_status_expiration_count = 0;
static uint32_t s_status_age_current_ms = 0;
static uint32_t s_status_age_max_ms = 0;
static uint32_t s_status_expired_generation = 0;
static uint8_t s_status_expired_state = BRIDGE_MESH_STATE_IDLE;
static bool s_status_expired = false;
static uint32_t s_audio_gate_stale = 0;
static uint32_t s_audio_gate_inactive = 0;
static uint32_t s_audio_gate_disconnected = 0;
static uint32_t s_audio_gate_invalid_node = 0;
static uint8_t s_audio_tx_waiting_ack_seq = 0;
static volatile bool s_audio_tx_waiting_ack = false;
static tx_entry_t s_audio_tx_inflight;
static volatile bool s_audio_tx_inflight_valid = false;
static volatile uint32_t s_ack_irq_count = 0;
static volatile uint32_t s_ack_release_count = 0;
static volatile uint32_t s_ack_spurious_count = 0;
static int64_t s_ack_wait_start_us = 0;          /* Timestamp when waiting_ack was set */
static volatile uint32_t s_ack_timeout_count = 0; /* Forced releases due to timeout */
#define ACK_TIMEOUT_US  50000  /* 50 ms — force-release if ACK not received */

static void IRAM_ATTR ack_gpio_isr_handler(void *arg)
{
    BaseType_t woke = pdFALSE;
    bool release_slot = false;

    (void)arg;
    s_ack_irq_count++;

    portENTER_CRITICAL_ISR(&s_ack_lock);
    if (s_audio_tx_waiting_ack) {
        s_audio_tx_waiting_ack = false;
        s_audio_tx_inflight_valid = false;
        s_ack_release_count++;
        release_slot = true;
    } else {
        s_ack_spurious_count++;
    }
    portEXIT_CRITICAL_ISR(&s_ack_lock);

    if (release_slot) {
        xSemaphoreGiveFromISR(s_audio_slots_free, &woke);
    }

    if (woke == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static uint8_t audio_queue_depth(void)
{
    return s_audio_slots_used ? (uint8_t)uxSemaphoreGetCount(s_audio_slots_used) : 0;
}

static void discard_pending_audio(void)
{
    if (s_tx_mutex == NULL || s_audio_slots_used == NULL || s_audio_slots_free == NULL) {
        return;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);

    while (xSemaphoreTake(s_audio_slots_used, 0) == pdTRUE) {
        s_audio_tail = (uint8_t)((s_audio_tail + 1) % TX_AUDIO_QUEUE_SIZE);
        xSemaphoreGive(s_audio_slots_free);
        s_audio_tx_stale_drop++;
    }
    s_audio_head = s_audio_tail;

    bool release_inflight = false;
    portENTER_CRITICAL(&s_ack_lock);
    if (s_audio_tx_waiting_ack) {
        s_audio_tx_waiting_ack = false;
        release_inflight = true;
    }
    s_audio_tx_inflight_valid = false;
    portEXIT_CRITICAL(&s_ack_lock);
    if (release_inflight) {
        xSemaphoreGive(s_audio_slots_free);
    }

    xSemaphoreGive(s_tx_mutex);
}

static void discard_audio_for_expired_status(uint32_t generation)
{
    if (s_tx_mutex == NULL || s_audio_slots_used == NULL || s_audio_slots_free == NULL) {
        return;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    portENTER_CRITICAL(&s_status_lock);
    bool still_expired = !s_connected && s_status.generation == generation;
    portEXIT_CRITICAL(&s_status_lock);
    if (!still_expired) {
        xSemaphoreGive(s_tx_mutex);
        return;
    }

    while (xSemaphoreTake(s_audio_slots_used, 0) == pdTRUE) {
        s_audio_tail = (uint8_t)((s_audio_tail + 1) % TX_AUDIO_QUEUE_SIZE);
        xSemaphoreGive(s_audio_slots_free);
        s_audio_tx_stale_drop++;
    }
    s_audio_head = s_audio_tail;

    bool release_inflight = false;
    portENTER_CRITICAL(&s_ack_lock);
    if (s_audio_tx_waiting_ack) {
        s_audio_tx_waiting_ack = false;
        release_inflight = true;
    }
    s_audio_tx_inflight_valid = false;
    portEXIT_CRITICAL(&s_ack_lock);
    if (release_inflight) {
        xSemaphoreGive(s_audio_slots_free);
    }
    xSemaphoreGive(s_tx_mutex);
}

static bool status_is_fresh_locked(int64_t now_us)
{
    return s_connected && s_status.received_at_us > 0 && now_us >= s_status.received_at_us &&
           now_us - s_status.received_at_us <= STATUS_STALE_TIMEOUT_US;
}

static void update_status_age_locked(int64_t now_us)
{
    if (s_status.received_at_us <= 0 || now_us < s_status.received_at_us) {
        s_status_age_current_ms = 0;
        return;
    }

    int64_t age_ms = (now_us - s_status.received_at_us) / 1000;
    s_status_age_current_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
    if (s_status_age_current_ms > s_status_age_max_ms) {
        s_status_age_max_ms = s_status_age_current_ms;
    }
}

static void log_status_telemetry(int64_t now_us)
{
    static int64_t last_log_us = 0;
    if (now_us - last_log_us <= 5000000) {
        return;
    }

    uint32_t valid_rx;
    uint32_t expiration_count;
    uint32_t age_ms;
    uint32_t max_age_ms;
    uint32_t generation;
    uint8_t state;
    uint32_t expired_generation;
    uint8_t expired_state;
    uint32_t gate_stale;
    uint32_t gate_inactive;
    uint32_t gate_disconnected;
    uint32_t gate_invalid_node;

    portENTER_CRITICAL(&s_status_lock);
    update_status_age_locked(now_us);
    valid_rx = s_status_rx_valid;
    expiration_count = s_status_expiration_count;
    age_ms = s_status_age_current_ms;
    max_age_ms = s_status_age_max_ms;
    generation = s_status.generation;
    state = s_status.mesh_state;
    expired_generation = s_status_expired_generation;
    expired_state = s_status_expired_state;
    gate_stale = s_audio_gate_stale;
    gate_inactive = s_audio_gate_inactive;
    gate_disconnected = s_audio_gate_disconnected;
    gate_invalid_node = s_audio_gate_invalid_node;
    portEXIT_CRITICAL(&s_status_lock);

    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=bridge_status valid_rx=%lu expire=%lu age_ms=%lu max_age_ms=%lu gen=%lu state=%u exp_gen=%lu exp_state=%u gate_stale=%lu gate_inactive=%lu gate_disconnected=%lu gate_invalid_node=%lu",
             valid_rx, expiration_count, age_ms, max_age_ms, generation, state,
             expired_generation, expired_state, gate_stale, gate_inactive, gate_disconnected,
             gate_invalid_node);
    last_log_us = now_us;
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void handle_rx_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    switch (type) {
    case BRIDGE_PKT_AUDIO:
    case BRIDGE_PKT_AUDIO_V2:
        if (s_audio_cb && len > 1) {
            /* nRF prefixes both audio versions with src_id; preserve the bundle itself. */
            uint8_t src_id = payload[0];
            s_audio_rx_count++;
            s_audio_cb(src_id, payload + 1, len - 1, esp_timer_get_time(),
                       type == BRIDGE_PKT_AUDIO_V2);
        }
        break;

    case BRIDGE_PKT_STATUS:
        if (len == sizeof(bridge_status_payload_t)) {
            bridge_status_payload_t status;
            memcpy(&status, payload, sizeof(status));
            if (status.version != BRIDGE_PROTOCOL_VERSION ||
                status.marker != BRIDGE_STATUS_V2_MARKER ||
                status.mesh_state > BRIDGE_MESH_STATE_ACTIVE) {
                ESP_LOGW(TAG, "Ignoring invalid bridge v2 status (version=%u marker=0x%02X)",
                         status.version, status.marker);
                break;
            }

            int64_t received_at_us = esp_timer_get_time();
            portENTER_CRITICAL(&s_status_lock);
            bool continuity_lost =
                s_status.received_at_us > 0 && received_at_us >= s_status.received_at_us &&
                received_at_us - s_status.received_at_us > STATUS_STALE_TIMEOUT_US;
            if (continuity_lost && !s_status_expired) {
                update_status_age_locked(received_at_us);
                s_status_expiration_count++;
                s_status_expired_generation = s_status.generation;
                s_status_expired_state = s_status.mesh_state;
            }
            /* Check for changes before updating */
            bool changed = (!s_connected) || (s_status.mesh_state != status.mesh_state) ||
                           (s_status.role != status.role) ||
                           (s_status.peer_count != status.peer_count) ||
                           (s_status.node_id != status.node_id);

            uint32_t generation = s_status.generation + 1;
            memset(&s_status, 0, sizeof(s_status));
            s_status.mesh_state = status.mesh_state;
            s_status.role = status.role;
            s_status.node_id = status.node_id;
            s_status.slot_index = (uint8_t)status.slot_index;
            s_status.coordinator_id = status.coordinator_id;
            s_status.peer_count = status.peer_count;
            s_status.is_coordinator = status.role == 1;
            s_status.has_mesh_state = true;
            s_status.protocol_version = status.version;
            s_status.generation = generation;
            s_status.received_at_us = received_at_us;
            s_status.continuity_lost = continuity_lost;
            s_connected = true;
            s_status_expired = false;
            s_status_rx_valid++;
            s_status_age_current_ms = 0;
            uart_bridge_status_t published_status = s_status;
            portEXIT_CRITICAL(&s_status_lock);

            if (s_status_cb) {
                s_status_cb(&published_status);
            }

            if (changed) {
                ESP_LOGI(TAG, "Status: state=%u role=%u node=%u slot=%d coord=%u peers=%u",
                         status.mesh_state, status.role, status.node_id, status.slot_index,
                         status.coordinator_id, status.peer_count);
            }
            if (status.mesh_state != BRIDGE_MESH_STATE_ACTIVE || status.node_id == 0) {
                discard_pending_audio();
            }
        } else if (len == 3) {
            int64_t received_at_us = esp_timer_get_time();
            portENTER_CRITICAL(&s_status_lock);
            bool continuity_lost =
                s_status.received_at_us > 0 && received_at_us >= s_status.received_at_us &&
                received_at_us - s_status.received_at_us > STATUS_STALE_TIMEOUT_US;
            if (continuity_lost && !s_status_expired) {
                update_status_age_locked(received_at_us);
                s_status_expiration_count++;
                s_status_expired_generation = s_status.generation;
                s_status_expired_state = s_status.mesh_state;
            }
            uint32_t generation = s_status.generation + 1;
            memset(&s_status, 0, sizeof(s_status));
            s_status.role = payload[0];
            s_status.is_coordinator = payload[0] == 1;
            s_status.peer_count = payload[1];
            s_status.node_id = payload[2];
            s_status.mesh_state = payload[0] != 0 && payload[2] != 0
                                      ? BRIDGE_MESH_STATE_ACTIVE
                                      : BRIDGE_MESH_STATE_IDLE;
            s_status.slot_index = UINT8_MAX;
            s_status.protocol_version = 1;
            s_status.has_mesh_state = false;
            s_status.generation = generation;
            s_status.received_at_us = received_at_us;
            s_status.continuity_lost = continuity_lost;
            s_connected = true;
            s_status_expired = false;
            s_status_rx_valid++;
            s_status_age_current_ms = 0;
            uart_bridge_status_t published_status = s_status;
            portEXIT_CRITICAL(&s_status_lock);

            if (s_status_cb) {
                s_status_cb(&published_status);
            }

            if (payload[0] == 0 || payload[2] == 0) {
                discard_pending_audio();
            }
        } else {
            ESP_LOGW(TAG, "Ignoring bridge status with invalid length %u", len);
        }
        break;

    case BRIDGE_PKT_MESH_EVENT:
        if (len > 0 && payload[0] == BRIDGE_EVENT_COMMAND_ACK &&
            len >= 1 + sizeof(bridge_command_ack_payload_t)) {
            bridge_command_ack_payload_t ack;
            memcpy(&ack, payload + 1, sizeof(ack));
            s_command_ack_command = ack.command;
            s_command_ack_generation = ack.generation;
            s_command_ack_result = ack.result;
            __atomic_store_n(&s_command_ack_received, true, __ATOMIC_RELEASE);
        }
        if (s_event_cb && len > 0) {
            uart_bridge_event_t event = (uart_bridge_event_t)payload[0];
            s_event_cb(event, payload + 1, len - 1);
        }
        break;

    case BRIDGE_PKT_LOG:
        if (len > 0) {
            /* Print nRF52 log with prefix, ensure null termination */
            char log_msg[129];
            uint16_t copy_len = (len < sizeof(log_msg) - 1) ? len : sizeof(log_msg) - 1;
            memcpy(log_msg, payload, copy_len);
            log_msg[copy_len] = '\0';
            ESP_LOGI("nRF52", "%s", log_msg);
        }
        break;

    default:
        ESP_LOGD(TAG, "Unknown packet type: 0x%02X", type);
        break;
    }
}

/**
 * @brief Parse a received SPI buffer for a framed packet.
 *
 * The first byte must be the sync byte.  If the buffer starts with
 * 0x00 it was an idle frame from the master and is silently ignored.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    bridge_frame_view_t frame;
    bridge_frame_decode_result_t result =
        bridge_frame_decode(buf, len, SPI_MAX_PAYLOAD, &frame);

    if (result == BRIDGE_FRAME_IDLE) {
        return;
    }
    if (result == BRIDGE_FRAME_TRUNCATED) {
        s_bridge_rx_trunc++;
        if (len >= BRIDGE_FRAME_OVERHEAD) {
            ESP_LOGW(TAG, "Truncated packet: need %u, got %u", buf[1] + 3u,
                     (unsigned)len);
        }
        return;
    }
    if (result == BRIDGE_FRAME_BAD_SYNC) {
        s_bridge_rx_bad_sync++;
        return;
    }
    if (result == BRIDGE_FRAME_BAD_LENGTH) {
        s_bridge_rx_bad_len++;
        ESP_LOGW(TAG, "Bad packet length: %u", len >= 2u ? buf[1] : 0u);
        return;
    }
    if (result == BRIDGE_FRAME_BAD_CRC) {
        uint8_t pkt_len = buf[1];
        uint8_t rx_crc = buf[2u + pkt_len];
        uint8_t calc_crc = bridge_frame_crc8(buf + 1u, (size_t)pkt_len + 1u);

        s_bridge_rx_crc_fail++;
        if ((s_bridge_rx_crc_fail % 50) == 1) {
            ESP_LOGW(TAG, "CRC mismatch: rx=0x%02X calc=0x%02X fail=%lu", rx_crc, calc_crc,
                     s_bridge_rx_crc_fail);
        }
        return;
    }

    uint8_t seq = frame.seq;
    if (!s_bridge_rx_seq_init) {
        s_bridge_rx_expected = seq;
        s_bridge_rx_seq_init = true;
    } else {
        uint8_t expected_next = (uint8_t)(s_bridge_rx_expected + 1);
        if (seq != expected_next) {
            s_bridge_rx_seq_gaps++;
        }
    }
    s_bridge_rx_expected = seq;

    handle_rx_packet(frame.type, frame.payload, (uint16_t)frame.payload_len);
}

/**
 * @brief Prepare the DMA TX buffer for the next SPI transaction.
 *
 * Audio dequeue and queue purging share the TX mutex so semaphore ownership
 * and ring indices move together.
 */
static void prepare_tx_buf(void)
{
    /* ---- ACK timeout recovery ----
     * If the nRF ACK pulse was missed (noise, reboot, glitch), force-release
     * the inflight frame so the pipeline doesn't deadlock permanently. */
    bool release_timed_out_slot = false;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_ack_lock);
    if (s_audio_tx_waiting_ack && (now_us - s_ack_wait_start_us) > ACK_TIMEOUT_US) {
        s_audio_tx_waiting_ack = false;
        s_audio_tx_inflight_valid = false;
        s_ack_timeout_count++;
        release_timed_out_slot = true;
    }
    portEXIT_CRITICAL(&s_ack_lock);
    if (release_timed_out_slot) {
        xSemaphoreGive(s_audio_slots_free);
    }

    /* Lifecycle controls must pass even while an audio frame awaits ACK. */
    if (s_ctrl_pending_valid && xSemaphoreTake(s_tx_mutex, 0) == pdTRUE) {
        if (s_ctrl_pending_valid) {
            memcpy(s_tx_dma_buf, s_ctrl_pending.buf, BRIDGE_SPI_MAX_XFER);
            s_ctrl_pending_valid = false;
            xSemaphoreGive(s_tx_mutex);
            return;
        }
        xSemaphoreGive(s_tx_mutex);
    }

    /* Fast path: audio queue with GPIO ACK-driven stop-and-wait. */
    portENTER_CRITICAL(&s_ack_lock);
    if (s_audio_tx_waiting_ack && s_audio_tx_inflight_valid) {
        memcpy(s_tx_dma_buf, s_audio_tx_inflight.buf, BRIDGE_SPI_MAX_XFER);
        portEXIT_CRITICAL(&s_ack_lock);
        return;
    }
    portEXIT_CRITICAL(&s_ack_lock);

    while (xSemaphoreTake(s_tx_mutex, 0) == pdTRUE) {
        if (xSemaphoreTake(s_audio_slots_used, 0) != pdTRUE) {
            xSemaphoreGive(s_tx_mutex);
            break;
        }

        uint8_t t = s_audio_tail;
        tx_entry_t entry = s_audio_q[t];
        s_audio_tail = (uint8_t)((t + 1) % TX_AUDIO_QUEUE_SIZE);

        int64_t now_us = esp_timer_get_time();
        uint32_t wait_us = entry.queued_at_us > 0 && now_us > entry.queued_at_us
                               ? (uint32_t)(now_us - entry.queued_at_us)
                               : 0;
        if (wait_us > TX_AUDIO_MAX_AGE_US) {
            s_audio_tx_stale_drop++;
            xSemaphoreGive(s_audio_slots_free);
            xSemaphoreGive(s_tx_mutex);
            continue;
        }

        portENTER_CRITICAL(&s_status_lock);
        bool mesh_ready = status_is_fresh_locked(now_us) &&
                          s_status.mesh_state == BRIDGE_MESH_STATE_ACTIVE &&
                          s_status.node_id != 0;
        portEXIT_CRITICAL(&s_status_lock);
        if (!mesh_ready) {
            s_audio_tx_stale_drop++;
            xSemaphoreGive(s_audio_slots_free);
            xSemaphoreGive(s_tx_mutex);
            continue;
        }

        s_audio_tx_wait_count++;
        s_audio_tx_wait_sum_us += wait_us;
        if (wait_us > s_audio_tx_wait_max_us) {
            s_audio_tx_wait_max_us = wait_us;
        }

        memcpy(s_tx_dma_buf, entry.buf, BRIDGE_SPI_MAX_XFER);

        memcpy(s_audio_tx_inflight.buf, entry.buf, BRIDGE_SPI_MAX_XFER);
        s_audio_tx_inflight.len = entry.len;
        s_audio_tx_waiting_ack_seq = entry.buf[2];
        s_ack_wait_start_us = now_us;

        /* Critical section: set inflight + waiting_ack atomically so the
         * ISR cannot see a half-updated state and count a valid ACK as
         * spurious (which would leak a semaphore slot permanently). */
        portENTER_CRITICAL(&s_ack_lock);
        s_audio_tx_inflight_valid = true;
        s_audio_tx_waiting_ack = true;
        portEXIT_CRITICAL(&s_ack_lock);
        xSemaphoreGive(s_tx_mutex);
        return;
    }

    /* Nothing pending, send idle frame */
    memset(s_tx_dma_buf, 0, BRIDGE_SPI_MAX_XFER);
}

/**
 * @brief SPI slave RX task
 *
 * Continuously queues SPI slave transactions.  Each transaction is
 * full-duplex: we send s_tx_buf and receive into s_rx_buf simultaneously.
 */
static void spi_slave_task(void *arg)
{
    ESP_LOGI(TAG, "SPI slave task started on core %d", xPortGetCoreID());

    spi_slave_transaction_t txn;
    memset(&txn, 0, sizeof(txn));

    uint32_t txn_count = 0;

    while (1) {
        /* Prepare outgoing data */
        prepare_tx_buf();

        memset(s_rx_buf, 0, BRIDGE_SPI_MAX_XFER);
        txn.length = BRIDGE_SPI_MAX_XFER * 8; /* length in bits */
        txn.tx_buffer = s_tx_dma_buf;
        txn.rx_buffer = s_rx_buf;

        /* Block until the master performs a transaction */
        esp_err_t err = spi_slave_transmit(BRIDGE_SPI_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        txn_count++;

        /* Parse what the master sent us */
        size_t rx_bytes = txn.trans_len / 8;

        /* Debug: log first 3 transactions for boot verification */
        if (txn_count <= 3) {
            ESP_LOGI(TAG, "SPI txn #%lu: %u bytes, rx[0..3]: %02X %02X %02X %02X", txn_count,
                     (unsigned)rx_bytes, s_rx_buf[0], s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
        }

        if (rx_bytes > 0) {
            parse_rx(s_rx_buf, rx_bytes);
        }
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t uart_bridge_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }

    s_audio_slots_free = xSemaphoreCreateCounting(TX_AUDIO_QUEUE_SIZE - 1, TX_AUDIO_QUEUE_SIZE - 1);
    s_audio_slots_used = xSemaphoreCreateCounting(TX_AUDIO_QUEUE_SIZE - 1, 0);
    if (s_audio_slots_free == NULL || s_audio_slots_used == NULL) {
        ESP_LOGE(TAG, "Failed to create audio queue semaphores");
        if (s_audio_slots_free) {
            vSemaphoreDelete(s_audio_slots_free);
            s_audio_slots_free = NULL;
        }
        if (s_audio_slots_used) {
            vSemaphoreDelete(s_audio_slots_used);
            s_audio_slots_used = NULL;
        }
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t ack_gpio_cfg = {
        .pin_bit_mask = (1ULL << BRIDGE_ACK_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&ack_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ACK GPIO: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
    }

    err = gpio_isr_handler_add(BRIDGE_ACK_GPIO_PIN, ack_gpio_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ACK GPIO ISR: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
    }

    /* SPI slave bus configuration */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BRIDGE_SPI_MOSI_PIN,
        .miso_io_num = BRIDGE_SPI_MISO_PIN,
        .sclk_io_num = BRIDGE_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BRIDGE_SPI_MAX_XFER,
    };

    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = BRIDGE_SPI_CS_PIN,
        .flags = 0,
        .queue_size = 2,
        .mode = BRIDGE_SPI_MODE,
    };

    err = spi_slave_initialize(BRIDGE_SPI_HOST, &bus_cfg, &slave_cfg, BRIDGE_SPI_DMA_CHAN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(err));
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        vSemaphoreDelete(s_tx_mutex);
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        s_tx_mutex = NULL;
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        return err;
    }

    /* Pull-ups on input lines — required for reliable CS/SCK detection.
     * Must be set AFTER spi_slave_initialize so the SPI driver owns the pins first. */
    gpio_set_pull_mode(BRIDGE_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_SCLK_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_CS_PIN, GPIO_PULLUP_ONLY);

    /* Start SPI slave task on core 0 so audio stays on core 1 */
    BaseType_t ret = xTaskCreatePinnedToCore(spi_slave_task, "spi_bridge_rx", RX_TASK_STACK_SIZE,
                                             NULL, RX_TASK_PRIORITY, &s_rx_task, RX_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI task");
        spi_slave_free(BRIDGE_SPI_HOST);
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        vSemaphoreDelete(s_tx_mutex);
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        s_tx_mutex = NULL;
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    s_audio_head = 0;
    s_audio_tail = 0;
    s_audio_tx_waiting_ack = false;
    s_audio_tx_waiting_ack_seq = 0;
    s_audio_tx_inflight_valid = false;
    s_command_in_progress = false;
    s_command_ack_received = false;
    ESP_LOGI(TAG, "SPI bridge initialized (MOSI=%d, MISO=%d, SCK=%d, CS=%d)", BRIDGE_SPI_MOSI_PIN,
             BRIDGE_SPI_MISO_PIN, BRIDGE_SPI_SCLK_PIN, BRIDGE_SPI_CS_PIN);

    return ESP_OK;
}

void uart_bridge_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_rx_task) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }

    gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
    spi_slave_free(BRIDGE_SPI_HOST);

    if (s_tx_mutex) {
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
    }

    if (s_audio_slots_free) {
        vSemaphoreDelete(s_audio_slots_free);
        s_audio_slots_free = NULL;
    }
    if (s_audio_slots_used) {
        vSemaphoreDelete(s_audio_slots_used);
        s_audio_slots_used = NULL;
    }

    s_initialized = false;
    portENTER_CRITICAL(&s_status_lock);
    s_connected = false;
    memset(&s_status, 0, sizeof(s_status));
    s_status_expired = false;
    s_status_age_current_ms = 0;
    portEXIT_CRITICAL(&s_status_lock);
    s_audio_tx_waiting_ack = false;
    s_audio_tx_inflight_valid = false;
    s_command_in_progress = false;
    s_command_ack_received = false;

    ESP_LOGI(TAG, "SPI bridge deinitialized");
}

/**
 * @brief Queue a framed packet into the TX buffer for the next SPI transaction.
 */
static esp_err_t queue_tx_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    size_t total;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((size_t)len > SPI_MAX_PAYLOAD || (len != 0u && payload == NULL)) {
        s_audio_tx_invalid_size++;
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    total = bridge_frame_encode(s_tx_buf, sizeof(s_tx_buf), SPI_MAX_PAYLOAD,
                                s_bridge_tx_seq, type, payload, len);
    if (total == 0u) {
        xSemaphoreGive(s_tx_mutex);
        s_audio_tx_invalid_size++;
        return ESP_ERR_INVALID_SIZE;
    }
    s_bridge_tx_seq++;
    memcpy(s_ctrl_pending.buf, s_tx_buf, BRIDGE_SPI_MAX_XFER);
    s_ctrl_pending.len = (uint16_t)total;
    s_ctrl_pending_valid = true;

    xSemaphoreGive(s_tx_mutex);
    return ESP_OK;
}

static esp_err_t send_audio_packet(uint8_t type, const uint8_t *data, uint16_t len)
{
    size_t total;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!uart_bridge_is_mesh_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((size_t)len > SPI_MAX_PAYLOAD || (len != 0u && data == NULL)) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_audio_slots_free, 0) != pdTRUE) {
        s_audio_tx_enqueue_timeout++;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(s_tx_mutex, 0) != pdTRUE) {
        xSemaphoreGive(s_audio_slots_free);
        s_audio_tx_enqueue_timeout++;
        return ESP_ERR_TIMEOUT;
    }

    uint8_t cur_head = s_audio_head;
    uint8_t next_head = (uint8_t)((cur_head + 1) % TX_AUDIO_QUEUE_SIZE);

    tx_entry_t *entry = &s_audio_q[cur_head];
    memset(entry->buf, 0, BRIDGE_SPI_MAX_XFER);
    total = bridge_frame_encode(entry->buf, sizeof(entry->buf), SPI_MAX_PAYLOAD,
                                s_bridge_tx_seq, type, data, len);
    if (total == 0u) {
        xSemaphoreGive(s_tx_mutex);
        xSemaphoreGive(s_audio_slots_free);
        return ESP_ERR_INVALID_SIZE;
    }
    s_bridge_tx_seq++;
    entry->len = (uint16_t)total;
    entry->queued_at_us = esp_timer_get_time();

    /* Memory barrier: ensure all writes to entry are visible before
     * the consumer sees the new head. On ESP32 (Xtensa) this is implicit
     * because volatile reads/writes are sequentially consistent within
     * a single core, but the consumer runs on a different core. */
    __sync_synchronize();
    s_audio_head = next_head;
    xSemaphoreGive(s_audio_slots_used);
    s_audio_tx_queued++;
    xSemaphoreGive(s_tx_mutex);

    /* Log stats periodically — OUTSIDE any critical section */
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 5000000) { /* Every 5s */
        ESP_LOGI(TAG,
                 "Audio pipe: tx_queued=%lu tx_overwr=%lu rx_from_nrf=%lu tx_q=%u ctrl_pending=%d bad_sync=%lu bad_len=%lu trunc=%lu crc_fail=%lu seq_gap=%lu ack_irq=%lu ack_rel=%lu ack_spur=%lu ack_to=%lu waiting=%d",
                 s_audio_tx_queued, s_audio_tx_overwrite, s_audio_rx_count,
                 audio_queue_depth(), s_ctrl_pending_valid ? 1 : 0,
                 s_bridge_rx_bad_sync, s_bridge_rx_bad_len, s_bridge_rx_trunc,
                 s_bridge_rx_crc_fail, s_bridge_rx_seq_gaps, s_ack_irq_count,
                 s_ack_release_count, s_ack_spurious_count, s_ack_timeout_count,
                 s_audio_tx_waiting_ack ? 1 : 0);
        ESP_LOGI(TAG,
                 "PIPE v=1 dev=esp stage=spi tx_q_ok=%lu tx_q_timeout=%lu tx_size_drop=%lu tx_stale_drop=%lu tx_wait_avg_us=%lu tx_wait_max_us=%lu rx_ok=%lu crc_drop=%lu sync_drop=%lu len_drop=%lu trunc_drop=%lu seq_gap=%lu ack_timeout=%lu q_depth=%u",
                 s_audio_tx_queued, s_audio_tx_enqueue_timeout, s_audio_tx_invalid_size,
                 s_audio_tx_stale_drop,
                 s_audio_tx_wait_count ? (uint32_t)(s_audio_tx_wait_sum_us / s_audio_tx_wait_count) : 0,
                 s_audio_tx_wait_max_us,
                 s_audio_rx_count, s_bridge_rx_crc_fail, s_bridge_rx_bad_sync,
                 s_bridge_rx_bad_len, s_bridge_rx_trunc, s_bridge_rx_seq_gaps,
                 s_ack_timeout_count, audio_queue_depth());
        last_log = now;
    }

    return ESP_OK;
}

esp_err_t uart_bridge_send_audio(const uint8_t *data, uint16_t len)
{
    return send_audio_packet(BRIDGE_PKT_AUDIO, data, len);
}

esp_err_t uart_bridge_send_audio_v2(const uint8_t *data, uint16_t len)
{
    audio_bundle_view_t bundle;

    if (len > MESH_AUDIO_V2_MAX_BUNDLE_SIZE ||
        !audio_bundle_parse(data, len, &bundle)) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_audio_packet(BRIDGE_PKT_AUDIO_V2, data, len);
}

void uart_bridge_set_audio_callback(uart_bridge_audio_cb_t cb)
{
    s_audio_cb = cb;
}

void uart_bridge_set_event_callback(uart_bridge_event_cb_t cb)
{
    s_event_cb = cb;
}

void uart_bridge_set_status_callback(uart_bridge_status_cb_t cb)
{
    s_status_cb = cb;
}

esp_err_t uart_bridge_get_status(uart_bridge_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool expired = false;
    uint32_t expired_generation = 0;
    uint8_t expired_state = BRIDGE_MESH_STATE_IDLE;
    uint32_t expired_age_ms = 0;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_status_lock);
    update_status_age_locked(now_us);
    if (!status_is_fresh_locked(now_us)) {
        expired = s_connected;
        expired_generation = s_status.generation;
        expired_state = s_status.mesh_state;
        expired_age_ms = s_status_age_current_ms;
        if (expired) {
            s_status_expiration_count++;
            s_status_expired_generation = expired_generation;
            s_status_expired_state = expired_state;
            s_status_expired = true;
        }
        s_connected = false;
        portEXIT_CRITICAL(&s_status_lock);
        if (expired) {
            ESP_LOGW(TAG, "Status expired: age_ms=%lu generation=%lu state=%u", expired_age_ms,
                     expired_generation, expired_state);
            discard_audio_for_expired_status(expired_generation);
        }
        log_status_telemetry(now_us);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(status, &s_status, sizeof(uart_bridge_status_t));
    portEXIT_CRITICAL(&s_status_lock);
    log_status_telemetry(now_us);
    return ESP_OK;
}

bool uart_bridge_is_connected(void)
{
    uart_bridge_status_t status;
    return uart_bridge_get_status(&status) == ESP_OK;
}

bool uart_bridge_is_mesh_ready(void)
{
    uart_bridge_status_t status;
    esp_err_t err = uart_bridge_get_status(&status);

    portENTER_CRITICAL(&s_status_lock);
    if (err != ESP_OK) {
        if (s_status_expired) {
            s_audio_gate_stale++;
        } else {
            s_audio_gate_disconnected++;
        }
    } else if (status.mesh_state != BRIDGE_MESH_STATE_ACTIVE) {
        s_audio_gate_inactive++;
    } else if (status.node_id == 0) {
        s_audio_gate_invalid_node++;
    }
    portEXIT_CRITICAL(&s_status_lock);

    return err == ESP_OK && status.mesh_state == BRIDGE_MESH_STATE_ACTIVE && status.node_id != 0;
}

bool uart_bridge_probe(uint32_t timeout_ms)
{
    if (!s_initialized) {
        return false;
    }

    if (uart_bridge_is_connected()) {
        return true;
    }

    const int MAX_PROBE_ATTEMPTS = 3;

    for (int attempt = 1; attempt <= MAX_PROBE_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing for nRF52840 (attempt %d/%d)...", attempt, MAX_PROBE_ATTEMPTS);

        /* Queue a PING command for the master to pick up on its next poll */
        uint8_t ping_payload[] = {0x03}; /* CMD_ID = PING */
        queue_tx_packet(BRIDGE_PKT_CONTROL, ping_payload, sizeof(ping_payload));

        /* Wait for the master to respond with a STATUS packet */
        int64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) < ((int64_t)timeout_ms * 1000)) {
            if (uart_bridge_is_connected()) {
                ESP_LOGI(TAG, "nRF52840 probe response received!");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGW(TAG, "Probe attempt %d timed out", attempt);

        /* If not the last attempt, tear down SPI and reinit to reset DMA state.
         * This handles the case where the ESP32 rebooted while the nRF52
         * was mid-transaction, leaving the SPI peripheral desynced. */
        if (attempt < MAX_PROBE_ATTEMPTS) {
            ESP_LOGI(TAG, "Reinitializing SPI bus...");
            uart_bridge_deinit();
            vTaskDelay(pdMS_TO_TICKS(100)); /* Let bus settle */
            esp_err_t err = uart_bridge_init();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "SPI reinit failed: %s", esp_err_to_name(err));
                return false;
            }
        }
    }

    ESP_LOGW(TAG, "nRF52840 probe timed out after %d attempts", MAX_PROBE_ATTEMPTS);
    return false;
}

static esp_err_t send_mesh_command(bridge_command_t command)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (__atomic_exchange_n(&s_command_in_progress, true, __ATOMIC_ACQUIRE)) {
        return ESP_ERR_INVALID_STATE;
    }

    bridge_command_payload_t payload = {
        .command = command,
        .generation = ++s_command_generation,
    };
    esp_err_t result = ESP_ERR_TIMEOUT;
    uart_bridge_status_t initial_status = {0};
    uint32_t initial_status_generation =
        uart_bridge_get_status(&initial_status) == ESP_OK ? initial_status.generation : 0;
    __atomic_store_n(&s_command_ack_received, false, __ATOMIC_RELEASE);

    for (int attempt = 1; attempt <= COMMAND_MAX_ATTEMPTS; attempt++) {
        result = queue_tx_packet(BRIDGE_PKT_CONTROL, (const uint8_t *)&payload, sizeof(payload));
        if (result != ESP_OK) {
            break;
        }

        int64_t deadline_us = esp_timer_get_time() + COMMAND_ACK_TIMEOUT_MS * 1000LL;
        while (esp_timer_get_time() < deadline_us) {
            if (__atomic_load_n(&s_command_ack_received, __ATOMIC_ACQUIRE) &&
                s_command_ack_command == command &&
                s_command_ack_generation == payload.generation) {
                result = s_command_ack_result == 0 ? ESP_OK : ESP_FAIL;
                goto done;
            }

            uart_bridge_status_t status;
            if (uart_bridge_get_status(&status) == ESP_OK &&
                status.generation != initial_status_generation) {
                bool observed = command == BRIDGE_COMMAND_MESH_START
                                    ? (status.has_mesh_state
                                           ? status.mesh_state != BRIDGE_MESH_STATE_IDLE
                                           : status.mesh_state == BRIDGE_MESH_STATE_ACTIVE &&
                                                 status.node_id != 0)
                                    : status.mesh_state == BRIDGE_MESH_STATE_IDLE;
                if (observed) {
                    ESP_LOGI(TAG, "Command 0x%02X confirmed by bridge status (legacy ACK fallback)",
                             command);
                    result = ESP_OK;
                    goto done;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        ESP_LOGW(TAG, "Command 0x%02X generation %u ACK timeout (%d/%d)", command,
                 payload.generation, attempt, COMMAND_MAX_ATTEMPTS);
    }

done:
    __atomic_store_n(&s_command_in_progress, false, __ATOMIC_RELEASE);
    return result;
}

esp_err_t uart_bridge_mesh_enable(void)
{
    ESP_LOGI(TAG, "Requesting mesh enable from nRF52840");
    return send_mesh_command(BRIDGE_COMMAND_MESH_START);
}

esp_err_t uart_bridge_mesh_disable(void)
{
    ESP_LOGI(TAG, "Requesting mesh disable from nRF52840");
    return send_mesh_command(BRIDGE_COMMAND_MESH_STOP);
}
