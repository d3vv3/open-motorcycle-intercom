/**
 * @file e2e_diag.c
 * @brief End-to-end audio pipeline diagnostics for the nRF transport path.
 */

#include "e2e_diag.h"

#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include "audio_rx_tracker.h"

static const char *TAG = "omi";

static uint16_t s_tx_seq = 0;
static uint32_t s_tx_frames = 0;
static e2e_pipe_counters_t s_pipe = {0};

/* The tracker is shared between the bridge RX callback and the health loop. */
static audio_rx_tracker_t s_rx_tracker = {0};
static portMUX_TYPE s_rx_lock = portMUX_INITIALIZER_UNLOCKED;

e2e_pipe_counters_t *e2e_diag_counters(void)
{
    return &s_pipe;
}

uint16_t e2e_diag_next_tx_seq(void)
{
    s_pipe.source_frames++;
    s_tx_frames++;
    return s_tx_seq++;
}

void e2e_diag_track_rx(uint8_t source_id, uint16_t seq)
{
    portENTER_CRITICAL(&s_rx_lock);
    (void)audio_rx_tracker_accept(&s_rx_tracker, source_id, seq);
    portEXIT_CRITICAL(&s_rx_lock);
}

void e2e_diag_credit_predecessor(uint8_t source_id, bool queued)
{
    portENTER_CRITICAL(&s_rx_lock);
    (void)audio_rx_tracker_credit_predecessor(&s_rx_tracker, source_id, queued);
    portEXIT_CRITICAL(&s_rx_lock);
}

void e2e_diag_reset_source(uint8_t source_id)
{
    portENTER_CRITICAL(&s_rx_lock);
    audio_rx_tracker_reset_source(&s_rx_tracker, source_id);
    portEXIT_CRITICAL(&s_rx_lock);
}

void e2e_diag_reset_all_sources(void)
{
    portENTER_CRITICAL(&s_rx_lock);
    audio_rx_tracker_reset_sources(&s_rx_tracker);
    portEXIT_CRITICAL(&s_rx_lock);
}

void e2e_diag_log(uint8_t node_id)
{
    uint32_t rx_frames;
    uint32_t rx_gap_events;
    uint32_t rx_gap_frames;
    uint32_t rx_reordered;
    uint32_t rx_recovered;

    portENTER_CRITICAL(&s_rx_lock);
    rx_frames = s_rx_tracker.frames;
    rx_gap_events = s_rx_tracker.gap_events;
    rx_gap_frames = s_rx_tracker.gap_frames;
    rx_reordered = s_rx_tracker.reordered_or_old;
    rx_recovered = s_rx_tracker.recovered_frames;
    portEXIT_CRITICAL(&s_rx_lock);

    ESP_LOGI(TAG,
             "[E2E_ESP] tx=%lu rx=%lu gap_evt=%lu gap_fr=%lu reset_evt=%lu recovered=%lu effective_gap=%lu",
             s_tx_frames, rx_frames, rx_gap_events, rx_gap_frames, rx_reordered, rx_recovered,
             rx_gap_frames > rx_recovered ? rx_gap_frames - rx_recovered : 0);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=transport node=%u source=%lu gate_drop=%lu spi_try=%lu spi_ok=%lu spi_fail=%lu spi_oversize=%lu spi_rx=%lu spi_gap=%lu spi_invalid=%lu spi_self=%lu probe_rx=%lu play_q_ok=%lu play_q_drop=%lu bundle_tx=%lu bundle_rx=%lu bundle_bad=%lu prev1_attached=%lu prev2_attached=%lu prev1_offer=%lu prev1_accept=%lu prev1_reject=%lu prev2_offer=%lu prev2_accept=%lu prev2_reject=%lu recovered=%lu",
             node_id, s_pipe.source_frames, s_pipe.gate_drops, s_pipe.spi_attempts,
             s_pipe.spi_enqueue_ok, s_pipe.spi_enqueue_fail, s_pipe.spi_oversize, s_pipe.spi_rx,
             rx_gap_frames, s_pipe.spi_rx_invalid, s_pipe.spi_rx_self, s_pipe.probe_rx,
             s_pipe.play_queue_ok, s_pipe.play_queue_drop, s_pipe.bundle_tx, s_pipe.bundle_rx,
             s_pipe.bundle_bad, s_pipe.prev1_attached, s_pipe.prev2_attached, s_pipe.prev1_offer,
             s_pipe.prev1_accept, s_pipe.prev1_reject, s_pipe.prev2_offer, s_pipe.prev2_accept,
             s_pipe.prev2_reject, rx_recovered);
    ESP_LOGI(TAG,
             "Redundancy: tx=%lu rx=%lu attached=%lu/%lu prev1=%lu/%lu/%lu prev2=%lu/%lu/%lu recovered=%lu",
             s_pipe.bundle_tx, s_pipe.bundle_rx, s_pipe.prev1_attached, s_pipe.prev2_attached,
             s_pipe.prev1_offer, s_pipe.prev1_accept, s_pipe.prev1_reject, s_pipe.prev2_offer,
             s_pipe.prev2_accept, s_pipe.prev2_reject, rx_recovered);
}
