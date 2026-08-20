/**
 * @file ws_sync.c
 * @brief I2S WS Sync Capture diagnostics
 *
 * Uses GPIOTE + PPI + TIMER1 in counter mode to count WS rising edges
 * from the ESP32's I2S output (16 kHz).  Each TDMA frame (~20 ms) we
 * snapshot the count and compare to the expected 320 edges.
 *
 * Hardware wiring: ESP32 GPIO5 (I2S WS / LCK) → nRF XIAO D0 (P0.02)
 */

#include "ws_sync.h"

#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <nrfx_timer.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ws_sync, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* GPIO pin receiving ESP32 I2S WS signal (XIAO D0 = P0.02) */
#define WS_PIN  2
#define WS_PORT 0 /* NRF_P0 */

/* Expected WS edges per 20 ms TDMA frame (16 kHz × 0.020 s) */
#define EXPECTED_EDGES_PER_FRAME 320

/* GPIOTE and PPI channels are allocated to avoid colliding with other nrfx users. */
static uint8_t s_gpiote_channel;
/* PPI channel — ESB uses 15-17, we take 0 */
static nrf_ppi_channel_t s_ppi_channel;

/* TIMER1 instance for counting (TIMER0 reserved by ESB) */
static const nrfx_timer_t s_timer = NRFX_TIMER_INSTANCE(1);

/* ============================================================================
 * State
 * ============================================================================ */

static bool s_initialized = false;
static bool s_running = false;
static uint32_t s_last_count = 0;  /* Previous snapshot of edge counter   */
static bool s_first_sample = true; /* Skip first delta (no baseline)      */
static uint32_t s_last_frame_counter;
static ws_sync_diag_t s_diag;

/* ============================================================================
 * Dummy handler — TIMER1 interrupt not used, but nrfx requires one
 * ============================================================================ */
static void timer_dummy_handler(nrf_timer_event_t event, void *ctx)
{
    ARG_UNUSED(event);
    ARG_UNUSED(ctx);
}

#if NRFX_API_VER_AT_LEAST(3, 2, 0)
static const nrfx_gpiote_t s_gpiote = NRFX_GPIOTE_INSTANCE(0);

static nrfx_err_t gpiote_channel_alloc(uint8_t *channel)
{
    return nrfx_gpiote_channel_alloc(&s_gpiote, channel);
}

static nrfx_err_t gpiote_channel_free(uint8_t channel)
{
    return nrfx_gpiote_channel_free(&s_gpiote, channel);
}
#else
static nrfx_err_t gpiote_channel_alloc(uint8_t *channel)
{
    return nrfx_gpiote_channel_alloc(channel);
}

static nrfx_err_t gpiote_channel_free(uint8_t channel)
{
    return nrfx_gpiote_channel_free(channel);
}
#endif

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
        .frequency = NRFX_MHZ_TO_HZ(1), /* irrelevant in counter mode */
        .mode = NRF_TIMER_MODE_COUNTER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .p_context = NULL,
    };
    err = nrfx_timer_init(&s_timer, &tcfg, timer_dummy_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("TIMER1 init failed: 0x%x", err);
        return err == NRFX_ERROR_INVALID_STATE ? -EBUSY : -EIO;
    }

    err = gpiote_channel_alloc(&s_gpiote_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("GPIOTE channel alloc failed: 0x%x", err);
        nrfx_timer_uninit(&s_timer);
        return -EIO;
    }

    /* Connect the input buffer; reset-state GPIO inputs are disconnected. */
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(WS_PORT, WS_PIN), NRF_GPIO_PIN_NOPULL);
    nrf_gpiote_event_configure(NRF_GPIOTE, s_gpiote_channel, NRF_GPIO_PIN_MAP(WS_PORT, WS_PIN),
                               NRF_GPIOTE_POLARITY_LOTOHI);
    nrf_gpiote_event_enable(NRF_GPIOTE, s_gpiote_channel);

    /* --- PPI: GPIOTE IN event → TIMER1 COUNT task --- */
    err = nrfx_ppi_channel_alloc(&s_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("PPI alloc failed: 0x%x", err);
        nrf_gpiote_event_disable(NRF_GPIOTE, s_gpiote_channel);
        (void)gpiote_channel_free(s_gpiote_channel);
        nrfx_timer_uninit(&s_timer);
        return -EIO;
    }

    uint32_t gpiote_evt_addr =
        nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(s_gpiote_channel));
    uint32_t timer_task_addr = nrfx_timer_task_address_get(&s_timer, NRF_TIMER_TASK_COUNT);

    err = nrfx_ppi_channel_assign(s_ppi_channel, gpiote_evt_addr, timer_task_addr);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("PPI assign failed: 0x%x", err);
        (void)nrfx_ppi_channel_free(s_ppi_channel);
        nrf_gpiote_event_disable(NRF_GPIOTE, s_gpiote_channel);
        (void)gpiote_channel_free(s_gpiote_channel);
        nrfx_timer_uninit(&s_timer);
        return -EIO;
    }

    s_initialized = true;
    LOG_INF("WS sync capture initialized (P%d.%02d, GPIOTE ch %d, TIMER1 counter)", WS_PORT, WS_PIN,
            s_gpiote_channel);
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

    s_last_count = 0;
    s_first_sample = true;
    s_last_frame_counter = 0;
    memset(&s_diag, 0, sizeof(s_diag));
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

bool ws_sync_capture(uint32_t *edge_count)
{
    if (!s_running || edge_count == NULL) {
        return false;
    }

    *edge_count = nrfx_timer_capture(&s_timer, NRF_TIMER_CC_CHANNEL1);
    return true;
}

bool ws_sync_sample(uint32_t frame_counter, uint32_t edge_count, int32_t *correction_us)
{
    if (!s_running || correction_us == NULL) {
        return false;
    }

    uint32_t delta = edge_count - s_last_count;
    s_last_count = edge_count;
    s_diag.total_edges += delta;
    s_diag.sample_count++;
    s_diag.last_delta_edges = delta;

    if (s_first_sample) {
        /* Need at least one baseline sample */
        s_first_sample = false;
        s_last_frame_counter = frame_counter;
        *correction_us = 0;
        return false;
    }

    uint32_t elapsed_frames = frame_counter - s_last_frame_counter;
    s_last_frame_counter = frame_counter;
    if (elapsed_frames == 0 || elapsed_frames > 50) {
        s_diag.rejected_count++;
        *correction_us = 0;
        return false;
    }

    /* If WS signal is absent (delta == 0), don't try to correct */
    if (delta == 0) {
        s_diag.no_signal_count++;
        *correction_us = 0;
        return false;
    }

    uint32_t expected_edges = elapsed_frames * EXPECTED_EDGES_PER_FRAME;
    if (delta < (expected_edges * 3U) / 4U || delta > (expected_edges * 5U) / 4U) {
        s_diag.rejected_count++;
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
    int32_t edge_error = (int32_t)expected_edges - (int32_t)delta;
    int32_t error_us = (edge_error * 625) / 10;

    /* Apply 25% of the observed phase error to avoid single-sample swings. */
    int32_t correction = error_us / 4;

    /* Clamp to avoid wild swings */
    if (correction > 500) {
        correction = 500;
    } else if (correction < -500) {
        correction = -500;
    }

    s_diag.last_correction_us = correction;
    s_diag.cumulative_drift_us =
        CLAMP((int64_t)s_diag.cumulative_drift_us + error_us, INT32_MIN, INT32_MAX);
    s_diag.valid_count++;
    *correction_us = correction;

    return true;
}

void ws_sync_get_diag(ws_sync_diag_t *diag)
{
    if (diag != NULL) {
        *diag = s_diag;
    }
}
