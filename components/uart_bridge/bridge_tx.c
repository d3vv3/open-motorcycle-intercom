/**
 * @file bridge_tx.c
 * @brief SPI TX path: audio ring, control slot, and ACK stop-and-wait flow.
 *
 * Audio frames go through a ring drained by the SPI task. Each sent frame
 * stays "inflight" until the nRF pulses the ACK GPIO; only then is the ring
 * slot released. The ISR, the timeout recovery, and the purge paths all
 * hand slots back through the same semaphore, so this file keeps them
 * together on purpose.
 */

#include "bridge_internal.h"

#include <string.h>

#include "audio_bundle.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "spi_bridge";

_Static_assert(MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 6 <= BRIDGE_SPI_MAX_XFER,
               "source-prefixed audio v2 bundle must fit one SPI transfer");

void IRAM_ATTR bridge_tx_ack_isr(void *arg)
{
    BaseType_t woke = pdFALSE;
    bool release_slot = false;

    (void)arg;
    g_bridge.ack_irq_count++;

    portENTER_CRITICAL_ISR(&g_bridge_ack_lock);
    if (g_bridge.waiting_ack) {
        g_bridge.waiting_ack = false;
        g_bridge.inflight_valid = false;
        g_bridge.ack_release_count++;
        release_slot = true;
    } else {
        g_bridge.ack_spurious_count++;
    }
    portEXIT_CRITICAL_ISR(&g_bridge_ack_lock);

    if (release_slot) {
        xSemaphoreGiveFromISR(g_bridge.audio_slots_free, &woke);
    }

    if (woke == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

uint8_t bridge_tx_audio_depth(void)
{
    return g_bridge.audio_slots_used ? (uint8_t)uxSemaphoreGetCount(g_bridge.audio_slots_used) : 0;
}

/* Caller must hold tx_mutex. */
static void drain_audio_ring_locked(void)
{
    while (xSemaphoreTake(g_bridge.audio_slots_used, 0) == pdTRUE) {
        g_bridge.audio_tail = (uint8_t)((g_bridge.audio_tail + 1) % BRIDGE_TX_AUDIO_QUEUE_SIZE);
        xSemaphoreGive(g_bridge.audio_slots_free);
        g_bridge.audio_tx_stale_drop++;
    }
    g_bridge.audio_head = g_bridge.audio_tail;

    bool release_inflight = false;
    portENTER_CRITICAL(&g_bridge_ack_lock);
    if (g_bridge.waiting_ack) {
        g_bridge.waiting_ack = false;
        release_inflight = true;
    }
    g_bridge.inflight_valid = false;
    portEXIT_CRITICAL(&g_bridge_ack_lock);
    if (release_inflight) {
        xSemaphoreGive(g_bridge.audio_slots_free);
    }
}

void bridge_tx_discard_pending_audio(void)
{
    if (g_bridge.tx_mutex == NULL || g_bridge.audio_slots_used == NULL ||
        g_bridge.audio_slots_free == NULL) {
        return;
    }

    xSemaphoreTake(g_bridge.tx_mutex, portMAX_DELAY);
    drain_audio_ring_locked();
    xSemaphoreGive(g_bridge.tx_mutex);
}

void bridge_tx_discard_for_expired_status(uint32_t generation)
{
    if (g_bridge.tx_mutex == NULL || g_bridge.audio_slots_used == NULL ||
        g_bridge.audio_slots_free == NULL) {
        return;
    }

    xSemaphoreTake(g_bridge.tx_mutex, portMAX_DELAY);
    portENTER_CRITICAL(&g_bridge_status_lock);
    bool still_expired = !g_bridge.connected && g_bridge.status.generation == generation;
    portEXIT_CRITICAL(&g_bridge_status_lock);
    if (still_expired) {
        drain_audio_ring_locked();
    }
    xSemaphoreGive(g_bridge.tx_mutex);
}

/**
 * @brief Prepare the DMA TX buffer for the next SPI transaction.
 *
 * Audio dequeue and queue purging share the TX mutex so semaphore ownership
 * and ring indices move together.
 */
void bridge_tx_prepare_dma(void)
{
    /* ACK timeout recovery: if the nRF ACK pulse was missed (noise, reboot,
     * glitch), force-release the inflight frame so the pipeline doesn't
     * deadlock permanently. */
    bool release_timed_out_slot = false;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&g_bridge_ack_lock);
    if (g_bridge.waiting_ack && (now_us - g_bridge.ack_wait_start_us) > BRIDGE_ACK_TIMEOUT_US) {
        g_bridge.waiting_ack = false;
        g_bridge.inflight_valid = false;
        g_bridge.ack_timeout_count++;
        release_timed_out_slot = true;
    }
    portEXIT_CRITICAL(&g_bridge_ack_lock);
    if (release_timed_out_slot) {
        xSemaphoreGive(g_bridge.audio_slots_free);
    }

    /* Lifecycle controls must pass even while an audio frame awaits ACK. */
    if (g_bridge.ctrl_pending_valid && xSemaphoreTake(g_bridge.tx_mutex, 0) == pdTRUE) {
        if (g_bridge.ctrl_pending_valid) {
            memcpy(g_bridge_tx_dma, g_bridge.ctrl_pending.buf, BRIDGE_SPI_MAX_XFER);
            g_bridge.ctrl_pending_valid = false;
            xSemaphoreGive(g_bridge.tx_mutex);
            return;
        }
        xSemaphoreGive(g_bridge.tx_mutex);
    }

    /* While waiting for the ACK, keep re-sending the inflight frame. */
    portENTER_CRITICAL(&g_bridge_ack_lock);
    if (g_bridge.waiting_ack && g_bridge.inflight_valid) {
        memcpy(g_bridge_tx_dma, g_bridge.inflight.buf, BRIDGE_SPI_MAX_XFER);
        portEXIT_CRITICAL(&g_bridge_ack_lock);
        return;
    }
    portEXIT_CRITICAL(&g_bridge_ack_lock);

    while (xSemaphoreTake(g_bridge.tx_mutex, 0) == pdTRUE) {
        if (xSemaphoreTake(g_bridge.audio_slots_used, 0) != pdTRUE) {
            xSemaphoreGive(g_bridge.tx_mutex);
            break;
        }

        uint8_t t = g_bridge.audio_tail;
        bridge_tx_entry_t entry = g_bridge.audio_q[t];
        g_bridge.audio_tail = (uint8_t)((t + 1) % BRIDGE_TX_AUDIO_QUEUE_SIZE);

        int64_t dequeue_us = esp_timer_get_time();
        uint32_t wait_us = entry.queued_at_us > 0 && dequeue_us > entry.queued_at_us
                               ? (uint32_t)(dequeue_us - entry.queued_at_us)
                               : 0;
        if (wait_us > BRIDGE_TX_AUDIO_MAX_AGE_US) {
            g_bridge.audio_tx_stale_drop++;
            xSemaphoreGive(g_bridge.audio_slots_free);
            xSemaphoreGive(g_bridge.tx_mutex);
            continue;
        }

        portENTER_CRITICAL(&g_bridge_status_lock);
        bool mesh_ready = bridge_status_is_fresh_locked(dequeue_us) &&
                          g_bridge.status.mesh_state == BRIDGE_MESH_STATE_ACTIVE &&
                          g_bridge.status.node_id != 0;
        portEXIT_CRITICAL(&g_bridge_status_lock);
        if (!mesh_ready) {
            g_bridge.audio_tx_stale_drop++;
            xSemaphoreGive(g_bridge.audio_slots_free);
            xSemaphoreGive(g_bridge.tx_mutex);
            continue;
        }

        g_bridge.audio_tx_wait_count++;
        g_bridge.audio_tx_wait_sum_us += wait_us;
        if (wait_us > g_bridge.audio_tx_wait_max_us) {
            g_bridge.audio_tx_wait_max_us = wait_us;
        }

        memcpy(g_bridge_tx_dma, entry.buf, BRIDGE_SPI_MAX_XFER);

        memcpy(g_bridge.inflight.buf, entry.buf, BRIDGE_SPI_MAX_XFER);
        g_bridge.ack_wait_start_us = dequeue_us;

        /* Set inflight + waiting_ack atomically so the ISR cannot see a
         * half-updated state and count a valid ACK as spurious (which would
         * leak a semaphore slot permanently). */
        portENTER_CRITICAL(&g_bridge_ack_lock);
        g_bridge.inflight_valid = true;
        g_bridge.waiting_ack = true;
        portEXIT_CRITICAL(&g_bridge_ack_lock);
        xSemaphoreGive(g_bridge.tx_mutex);
        return;
    }

    /* Nothing pending, send idle frame */
    memset(g_bridge_tx_dma, 0, BRIDGE_SPI_MAX_XFER);
}

/**
 * @brief Queue a control packet for the next SPI transaction.
 *
 * There is a single control slot; a newer control packet replaces an unsent
 * older one. Control packets bypass the audio ACK flow.
 */
esp_err_t bridge_tx_queue_control(uint8_t type, const uint8_t *payload, uint16_t len)
{
    size_t total;

    if (!g_bridge.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((size_t)len > BRIDGE_SPI_MAX_PAYLOAD || (len != 0u && payload == NULL)) {
        g_bridge.audio_tx_invalid_size++;
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(g_bridge.tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(g_bridge_tx_staging, 0, BRIDGE_SPI_MAX_XFER);
    total = bridge_frame_encode(g_bridge_tx_staging, sizeof(g_bridge_tx_staging),
                                BRIDGE_SPI_MAX_PAYLOAD, g_bridge.tx_seq, type, payload, len);
    if (total == 0u) {
        xSemaphoreGive(g_bridge.tx_mutex);
        g_bridge.audio_tx_invalid_size++;
        return ESP_ERR_INVALID_SIZE;
    }
    g_bridge.tx_seq++;
    memcpy(g_bridge.ctrl_pending.buf, g_bridge_tx_staging, BRIDGE_SPI_MAX_XFER);
    g_bridge.ctrl_pending_valid = true;

    xSemaphoreGive(g_bridge.tx_mutex);
    return ESP_OK;
}

static void log_audio_pipe_stats(void)
{
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log <= 5000000) {
        return;
    }

    ESP_LOGI(TAG,
             "Audio pipe: tx_queued=%lu tx_overwr=%lu rx_from_nrf=%lu tx_q=%u ctrl_pending=%d bad_sync=%lu bad_len=%lu trunc=%lu crc_fail=%lu seq_gap=%lu ack_irq=%lu ack_rel=%lu ack_spur=%lu ack_to=%lu waiting=%d",
             g_bridge.audio_tx_queued, g_bridge.audio_tx_overwrite, g_bridge.audio_rx_count,
             bridge_tx_audio_depth(), g_bridge.ctrl_pending_valid ? 1 : 0, g_bridge.rx_bad_sync,
             g_bridge.rx_bad_len, g_bridge.rx_trunc, g_bridge.rx_crc_fail, g_bridge.rx_seq_gaps,
             g_bridge.ack_irq_count, g_bridge.ack_release_count, g_bridge.ack_spurious_count,
             g_bridge.ack_timeout_count, g_bridge.waiting_ack ? 1 : 0);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=spi tx_q_ok=%lu tx_q_timeout=%lu tx_size_drop=%lu tx_stale_drop=%lu tx_wait_avg_us=%lu tx_wait_max_us=%lu rx_ok=%lu crc_drop=%lu sync_drop=%lu len_drop=%lu trunc_drop=%lu seq_gap=%lu ack_timeout=%lu q_depth=%u",
             g_bridge.audio_tx_queued, g_bridge.audio_tx_enqueue_timeout,
             g_bridge.audio_tx_invalid_size, g_bridge.audio_tx_stale_drop,
             g_bridge.audio_tx_wait_count
                 ? (uint32_t)(g_bridge.audio_tx_wait_sum_us / g_bridge.audio_tx_wait_count)
                 : 0,
             g_bridge.audio_tx_wait_max_us, g_bridge.audio_rx_count, g_bridge.rx_crc_fail,
             g_bridge.rx_bad_sync, g_bridge.rx_bad_len, g_bridge.rx_trunc, g_bridge.rx_seq_gaps,
             g_bridge.ack_timeout_count, bridge_tx_audio_depth());
    last_log = now;
}

static esp_err_t send_audio_packet(uint8_t type, const uint8_t *data, uint16_t len)
{
    size_t total;

    if (!g_bridge.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!uart_bridge_is_mesh_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((size_t)len > BRIDGE_SPI_MAX_PAYLOAD || (len != 0u && data == NULL)) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(g_bridge.audio_slots_free, 0) != pdTRUE) {
        g_bridge.audio_tx_enqueue_timeout++;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(g_bridge.tx_mutex, 0) != pdTRUE) {
        xSemaphoreGive(g_bridge.audio_slots_free);
        g_bridge.audio_tx_enqueue_timeout++;
        return ESP_ERR_TIMEOUT;
    }

    uint8_t cur_head = g_bridge.audio_head;
    uint8_t next_head = (uint8_t)((cur_head + 1) % BRIDGE_TX_AUDIO_QUEUE_SIZE);

    bridge_tx_entry_t *entry = &g_bridge.audio_q[cur_head];
    memset(entry->buf, 0, BRIDGE_SPI_MAX_XFER);
    total = bridge_frame_encode(entry->buf, sizeof(entry->buf), BRIDGE_SPI_MAX_PAYLOAD,
                                g_bridge.tx_seq, type, data, len);
    if (total == 0u) {
        xSemaphoreGive(g_bridge.tx_mutex);
        xSemaphoreGive(g_bridge.audio_slots_free);
        return ESP_ERR_INVALID_SIZE;
    }
    g_bridge.tx_seq++;
    entry->queued_at_us = esp_timer_get_time();

    /* Memory barrier: ensure all writes to entry are visible before the
     * consumer sees the new head. The consumer runs on the other core. */
    __sync_synchronize();
    g_bridge.audio_head = next_head;
    xSemaphoreGive(g_bridge.audio_slots_used);
    g_bridge.audio_tx_queued++;
    xSemaphoreGive(g_bridge.tx_mutex);

    log_audio_pipe_stats();
    return ESP_OK;
}

esp_err_t uart_bridge_send_audio(const uint8_t *data, uint16_t len)
{
    return send_audio_packet(BRIDGE_PKT_AUDIO, data, len);
}

esp_err_t uart_bridge_send_audio_v2(const uint8_t *data, uint16_t len)
{
    audio_bundle_view_t bundle;

    if (len > MESH_AUDIO_V2_MAX_BUNDLE_SIZE || !audio_bundle_parse(data, len, &bundle)) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_audio_packet(BRIDGE_PKT_AUDIO_V2, data, len);
}
