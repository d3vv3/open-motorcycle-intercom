/**
 * @file ws_sync.c
 * @brief I2S WS Sync Capture — disciplines TDMA from ESP32 playout clock
 *
 * Uses GPIOTE + PPI + TIMER1 in counter mode to count WS rising edges
 * from the ESP32's I2S output (16 kHz).  Each TDMA frame (~20 ms) we
 * snapshot the count and compare to the expected 320 edges.  The delta
 * drives a correction fed into tdma_tune_timing().
 *
 * Hardware wiring: ESP32 GPIO5 (I2S WS / LCK) → nRF XIAO D0 (P0.02)
 */

#include "ws_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <nrfx_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>

LOG_MODULE_REGISTER(ws_sync, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* GPIO pin receiving ESP32 I2S WS signal (XIAO D0 = P0.02) */
#define WS_PIN       2
#define WS_PORT      0   /* NRF_P0 */

/* Expected WS edges per 20 ms TDMA frame (16 kHz × 0.020 s) */
#define EXPECTED_EDGES_PER_FRAME  320

/* GPIOTE channel for edge detection (0-7 available; ESB doesn't use GPIOTE) */
#define GPIOTE_CH    0

/* PPI channel — ESB uses 15-17, we take 0 */
static nrf_ppi_channel_t s_ppi_channel;

/* TIMER1 instance for counting (TIMER0 reserved by ESB) */
static const nrfx_timer_t s_timer = NRFX_TIMER_INSTANCE(1);

/* ============================================================================
 * State
 * ============================================================================ */

static bool     s_initialized  = false;
static bool     s_running      = false;
static uint32_t s_last_count   = 0;   /* Previous snapshot of edge counter   */
static bool     s_first_sample = true; /* Skip first delta (no baseline)      */
static int32_t  s_last_correction_us  = 0;
static int32_t  s_cumulative_drift_us = 0;
static uint32_t s_total_edges         = 0;

/* ============================================================================
 * Dummy handler — TIMER1 interrupt not used, but nrfx requires one
 * ============================================================================ */
static void timer_dummy_handler(nrf_timer_event_t event, void *ctx)
{
    ARG_UNUSED(event);
    ARG_UNUSED(ctx);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int ws_sync_init(void)
{
    if (s_initialized) {
        return 0;
    }

    nrfx_err_t err;

    /* --- TIMER1 in counter mode (counts external events via PPI) --- */
    nrfx_timer_config_t tcfg = {
        .frequency  = NRFX_MHZ_TO_HZ(1),       /* irrelevant in counter mode */
        .mode       = NRF_TIMER_MODE_COUNTER,
        .bit_width  = NRF_TIMER_BIT_WIDTH_32,
        .p_context  = NULL,
    };
    err = nrfx_timer_init(&s_timer, &tcfg, timer_dummy_handler);
    if (err != NRFX_SUCCESS && err != NRFX_ERROR_INVALID_STATE) {
        LOG_ERR("TIMER1 init failed: 0x%x", err);
        return -EIO;
    }

    /* --- GPIOTE channel: IN event on rising edge of P0.02 --- */
    nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_CH,
                               NRF_GPIO_PIN_MAP(WS_PORT, WS_PIN),
                               NRF_GPIOTE_POLARITY_LOTOHI);
    nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_CH);

    /* --- PPI: GPIOTE IN event → TIMER1 COUNT task --- */
    err = nrfx_ppi_channel_alloc(&s_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("PPI alloc failed: 0x%x", err);
        return -EIO;
    }

    uint32_t gpiote_evt_addr =
        nrf_gpiote_event_address_get(NRF_GPIOTE,
                                     nrf_gpiote_in_event_get(GPIOTE_CH));
    uint32_t timer_task_addr =
        nrfx_timer_task_address_get(&s_timer, NRF_TIMER_TASK_COUNT);

    err = nrfx_ppi_channel_assign(s_ppi_channel, gpiote_evt_addr, timer_task_addr);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("PPI assign failed: 0x%x", err);
        return -EIO;
    }

    s_initialized = true;
    LOG_INF("WS sync capture initialized (P%d.%02d, GPIOTE ch %d, TIMER1 counter)",
            WS_PORT, WS_PIN, GPIOTE_CH);
    return 0;
}

void ws_sync_start(void)
{
    if (!s_initialized || s_running) {
        return;
    }

    /* Clear and enable counter */
    nrfx_timer_clear(&s_timer);
    nrfx_timer_enable(&s_timer);

    /* Enable PPI channel */
    nrfx_ppi_channel_enable(s_ppi_channel);

    s_last_count   = 0;
    s_first_sample = true;
    s_last_correction_us  = 0;
    s_cumulative_drift_us = 0;
    s_total_edges         = 0;
    s_running = true;

    LOG_INF("WS sync capture started");
}

void ws_sync_stop(void)
{
    if (!s_running) {
        return;
    }

    nrfx_ppi_channel_disable(s_ppi_channel);
    nrfx_timer_disable(&s_timer);
    s_running = false;

    LOG_INF("WS sync capture stopped");
}

bool ws_sync_sample(int32_t *correction_us)
{
    if (!s_running || !correction_us) {
        return false;
    }

    /* Capture current counter value without stopping it.
     * We use CC[1] as the capture register (CC[0] is sometimes
     * used internally by nrfx_timer for comparison). */
    nrfx_timer_capture(&s_timer, NRF_TIMER_CC_CHANNEL1);
    uint32_t count = nrfx_timer_capture_get(&s_timer, NRF_TIMER_CC_CHANNEL1);

    uint32_t delta = count - s_last_count;
    s_last_count = count;
    s_total_edges += delta;

    if (s_first_sample) {
        /* Need at least one baseline sample */
        s_first_sample = false;
        *correction_us = 0;
        return false;
    }

    /* If WS signal is absent (delta == 0), don't try to correct */
    if (delta == 0) {
        *correction_us = 0;
        return false;
    }

    /* delta should be ~320 per 20ms frame.
     * If delta > 320: ESP clock ran more edges than expected → ESP is
     *   faster → nRF needs to speed up (negative correction = shrink frame).
     * If delta < 320: ESP is slower → nRF needs to slow down (positive
     *   correction = stretch frame).
     *
     * Each edge is 62.5 us (1/16000).  So the timing error in us:
     *   error_us = (EXPECTED - delta) * 62.5
     * We use fixed-point: (EXPECTED - delta) * 625 / 10
     */
    int32_t edge_error = (int32_t)EXPECTED_EDGES_PER_FRAME - (int32_t)delta;
    int32_t error_us = (edge_error * 625) / 10;

    /* Low-pass filter: apply 25% of the error per frame to avoid
     * overcorrection from single-frame jitter */
    int32_t correction = error_us / 4;

    /* Clamp to avoid wild swings */
    if (correction > 500) {
        correction = 500;
    } else if (correction < -500) {
        correction = -500;
    }

    s_last_correction_us = correction;
    s_cumulative_drift_us += error_us;
    *correction_us = correction;

    return true;
}

void ws_sync_get_diag(uint32_t *total_edges, int32_t *last_correction_us,
                      int32_t *cumulative_drift_us)
{
    if (total_edges) {
        *total_edges = s_total_edges;
    }
    if (last_correction_us) {
        *last_correction_us = s_last_correction_us;
    }
    if (cumulative_drift_us) {
        *cumulative_drift_us = s_cumulative_drift_us;
    }
}
