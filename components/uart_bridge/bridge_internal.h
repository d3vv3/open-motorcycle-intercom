/**
 * @file bridge_internal.h
 * @brief Private state and interfaces of the SPI bridge component.
 *
 * Module ownership:
 *   uart_bridge.c      - facade: SPI slave lifecycle, transfer task, callbacks
 *   bridge_tx.c        - TX rings, ACK stop-and-wait flow, DMA buffer staging
 *   bridge_rx.c        - received frame validation and dispatch
 *   bridge_status.c    - nRF status registry, freshness gating, telemetry
 *   bridge_commands.c  - probe and mesh start/stop command exchange
 *
 * NOTE: g_bridge must stay zero-initialized (.bss). Spinlocks live outside
 * the struct because portMUX_INITIALIZER_UNLOCKED is nonzero and would move
 * the whole context into .data.
 *
 * Lock order: s_tx_mutex (g_bridge.tx_mutex) before g_bridge_ack_lock or
 * g_bridge_status_lock. Never take a mutex while holding a spinlock.
 */

#ifndef OMI_BRIDGE_INTERNAL_H
#define OMI_BRIDGE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bridge_frame.h"
#include "uart_bridge.h"

#define BRIDGE_RX_TASK_STACK_SIZE 4096
#define BRIDGE_RX_TASK_PRIORITY   5
#define BRIDGE_RX_TASK_CORE       0

#define BRIDGE_TX_AUDIO_QUEUE_SIZE 16
#define BRIDGE_TX_AUDIO_MAX_AGE_US 120000
#define BRIDGE_COMMAND_ACK_TIMEOUT_MS 300
#define BRIDGE_COMMAND_MAX_ATTEMPTS   3
#define BRIDGE_STATUS_STALE_TIMEOUT_US 5000000
#define BRIDGE_ACK_TIMEOUT_US 50000 /* Force-release inflight audio if the ACK pulse is missed */

#define BRIDGE_SPI_MAX_PAYLOAD (BRIDGE_SPI_MAX_XFER - BRIDGE_FRAME_OVERHEAD)

typedef struct {
    uint8_t buf[BRIDGE_SPI_MAX_XFER];
    int64_t queued_at_us;
} bridge_tx_entry_t;

typedef struct {
    /* Lifecycle */
    bool initialized;
    TaskHandle_t rx_task;

    /* Application callbacks */
    uart_bridge_audio_cb_t audio_cb;
    uart_bridge_event_cb_t event_cb;
    uart_bridge_status_cb_t status_cb;

    /* Status registry (g_bridge_status_lock) */
    uart_bridge_status_t status;
    bool connected;
    bool status_expired;
    uint32_t status_expired_generation;
    uint8_t status_expired_state;
    uint32_t status_rx_valid;
    uint32_t status_expiration_count;
    uint32_t status_age_current_ms;
    uint32_t status_age_max_ms;

    /* Audio admission gate counters (g_bridge_status_lock) */
    uint32_t gate_stale;
    uint32_t gate_inactive;
    uint32_t gate_disconnected;
    uint32_t gate_invalid_node;

    /* Command exchange (bridge_commands.c; ACK fields written by bridge_rx.c) */
    volatile bool command_in_progress;
    volatile bool command_ack_received;
    volatile uint8_t command_ack_generation;
    volatile uint8_t command_ack_command;
    volatile int8_t command_ack_result;
    uint8_t command_generation;

    /* TX audio ring. Producers share tx_mutex; the SPI task is the sole
     * consumer. Slot accounting uses the two counting semaphores. */
    SemaphoreHandle_t tx_mutex;
    bridge_tx_entry_t audio_q[BRIDGE_TX_AUDIO_QUEUE_SIZE];
    volatile uint8_t audio_head;
    volatile uint8_t audio_tail;
    SemaphoreHandle_t audio_slots_free;
    SemaphoreHandle_t audio_slots_used;

    /* Control slot: single pending packet, protected by tx_mutex */
    bridge_tx_entry_t ctrl_pending;
    volatile bool ctrl_pending_valid;

    /* ACK stop-and-wait state (g_bridge_ack_lock, shared with the GPIO ISR) */
    bridge_tx_entry_t inflight;
    volatile bool inflight_valid;
    volatile bool waiting_ack;
    int64_t ack_wait_start_us;
    volatile uint32_t ack_irq_count;
    volatile uint32_t ack_release_count;
    volatile uint32_t ack_spurious_count;
    volatile uint32_t ack_timeout_count;

    /* Wire sequence tracking */
    uint8_t tx_seq;
    uint8_t rx_expected;
    bool rx_seq_init;

    /* Diagnostics counters, logged periodically.
     * NOTE: audio_tx_overwrite is never incremented anymore (enqueue fails
     * instead of overwriting) but stays in the log line for parser schema
     * compatibility. */
    uint32_t audio_tx_queued;
    uint32_t audio_tx_overwrite;
    uint32_t audio_tx_enqueue_timeout;
    uint32_t audio_tx_invalid_size;
    uint32_t audio_tx_stale_drop;
    uint32_t audio_tx_wait_count;
    uint64_t audio_tx_wait_sum_us;
    uint32_t audio_tx_wait_max_us;
    uint32_t audio_rx_count;
    uint32_t rx_seq_gaps;
    uint32_t rx_crc_fail;
    uint32_t rx_bad_sync;
    uint32_t rx_bad_len;
    uint32_t rx_trunc;
} bridge_context_t;

extern bridge_context_t g_bridge;
extern portMUX_TYPE g_bridge_status_lock;
extern portMUX_TYPE g_bridge_ack_lock;

/* DMA-facing buffers, defined in uart_bridge.c (word-aligned, internal RAM) */
extern uint8_t g_bridge_tx_staging[BRIDGE_SPI_MAX_XFER];
extern uint8_t g_bridge_tx_dma[BRIDGE_SPI_MAX_XFER];
extern uint8_t g_bridge_rx_dma[BRIDGE_SPI_MAX_XFER];

/* bridge_tx.c */
void bridge_tx_ack_isr(void *arg);
void bridge_tx_prepare_dma(void);
void bridge_tx_discard_pending_audio(void);
void bridge_tx_discard_for_expired_status(uint32_t generation);
esp_err_t bridge_tx_queue_control(uint8_t type, const uint8_t *payload, uint16_t len);
uint8_t bridge_tx_audio_depth(void);

/* bridge_rx.c */
void bridge_rx_parse(const uint8_t *buf, size_t len);

/* bridge_status.c */
bool bridge_status_is_fresh_locked(int64_t now_us);
void bridge_status_apply_v2(const bridge_status_payload_t *payload);
void bridge_status_apply_legacy(const uint8_t payload[3]);
void bridge_status_log_telemetry(int64_t now_us);

/* bridge_commands.c */
void bridge_commands_note_ack(const bridge_command_ack_payload_t *ack);

#endif /* OMI_BRIDGE_INTERNAL_H */
