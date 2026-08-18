/**
 * @file e2e_diag.h
 * @brief End-to-end audio pipeline diagnostics for the nRF transport path.
 *
 * Tracks per-source RX sequence continuity and the stage counters reported
 * in the periodic [E2E_ESP], PIPE, and Redundancy log lines.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Stage counters for the SPI/bundle pipeline.
 *
 * NOTE: Fields are incremented directly by the transport paths without
 * synchronization. They are best-effort diagnostics, not exact accounting.
 */
typedef struct {
    uint32_t source_frames;
    uint32_t gate_drops;
    uint32_t spi_attempts;
    uint32_t spi_enqueue_ok;
    uint32_t spi_enqueue_fail;
    uint32_t spi_oversize;
    uint32_t spi_rx;
    uint32_t spi_rx_invalid;
    uint32_t spi_rx_self;
    uint32_t probe_rx;
    uint32_t play_queue_ok;
    uint32_t play_queue_drop;
    uint32_t bundle_tx;
    uint32_t prev1_attached;
    uint32_t prev2_attached;
    uint32_t bundle_rx;
    uint32_t bundle_bad;
    uint32_t prev1_offer;
    uint32_t prev1_accept;
    uint32_t prev1_reject;
    uint32_t prev2_offer;
    uint32_t prev2_accept;
    uint32_t prev2_reject;
} e2e_pipe_counters_t;

/** @brief Direct access to the pipe counters (single module owns the layout). */
e2e_pipe_counters_t *e2e_diag_counters(void);

/** @brief Allocate the next TX sequence number and count the frame. */
uint16_t e2e_diag_next_tx_seq(void);

/** @brief Record an accepted current frame for RX continuity tracking. */
void e2e_diag_track_rx(uint8_t source_id, uint16_t seq);

/** @brief Credit a predecessor frame offer against tracked gaps. */
void e2e_diag_credit_predecessor(uint8_t source_id, bool queued);

/** @brief Forget continuity state for one source (peer join/leave). */
void e2e_diag_reset_source(uint8_t source_id);

/** @brief Forget continuity state for all sources (mesh stop/sync loss). */
void e2e_diag_reset_all_sources(void);

/** @brief Emit the [E2E_ESP], PIPE, and Redundancy diagnostic log lines. */
void e2e_diag_log(uint8_t node_id);
