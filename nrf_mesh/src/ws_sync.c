/**
 * @file ws_sync.c
 * @brief I2S WS Sync Capture diagnostics
 *
 * Uses GPIOTE + GPPI + TIMER1 in counter mode to count WS rising edges
 * from the ESP32-S31's physical 48 kHz I2S WS output. Each 20 ms TDMA frame
 * should contain 960 rising edges; this is separate from 16 kHz Opus mesh audio.
 *
 * Hardware wiring: ESP32 GPIO5 (I2S WS / LCK) -> nRF XIAO D0 (P0.02)
 */

#include "ws_sync.h"
#include "ws_sync_math.h"

#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_timer.h>
#include <gpiote_nrfx.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ws_sync, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* GPIO pin receiving the S31 GPIO47 WS mirror of GPIO55 (XIAO D0 = P0.02) */
#define WS_PIN  2
#define WS_PORT 0 /* NRF_P0 */

/* Expected WS edges per 20 ms TDMA frame (48 kHz * 0.020 s = 960) */

/* GPIOTE channel and GPPI connection are allocated to avoid other nrfx users. */
static uint8_t s_gpiote_channel;
static nrfx_gppi_handle_t s_gppi_handle;

/* TIMER1 instance for counting (ESB owns TIMER2). */
static nrfx_timer_t s_timer = NRFX_TIMER_INSTANCE(NRF_TIMER1);

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
 * Public API
 * ============================================================================ */

int ws_sync_init(void)
{
    if (s_initialized) {
        return 0;
    }

    int err;
    nrfx_gpiote_t *gpiote = &GPIOTE_NRFX_INST_BY_NODE(DT_NODELABEL(gpiote0));

    /* --- TIMER1 in counter mode (counts external events via GPPI) --- */
    nrfx_timer_config_t tcfg = {
        .frequency = NRFX_MHZ_TO_HZ(1), /* irrelevant in counter mode */
        .mode = NRF_TIMER_MODE_COUNTER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .p_context = NULL,
    };
    err = nrfx_timer_init(&s_timer, &tcfg, NULL);
    if (err < 0) {
        LOG_ERR("TIMER1 init failed: %d", err);
        return err == -EALREADY ? -EBUSY : -EIO;
    }

    err = nrfx_gpiote_channel_alloc(gpiote, &s_gpiote_channel);
    if (err < 0) {
        LOG_ERR("GPIOTE channel alloc failed: %d", err);
        nrfx_timer_uninit(&s_timer);
        return -EIO;
    }

    /* Connect the input buffer; reset-state GPIO inputs are disconnected. */
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(WS_PORT, WS_PIN), NRF_GPIO_PIN_NOPULL);
    nrf_gpiote_event_configure(NRF_GPIOTE, s_gpiote_channel, NRF_GPIO_PIN_MAP(WS_PORT, WS_PIN),
                               NRF_GPIOTE_POLARITY_LOTOHI);
    nrf_gpiote_event_enable(NRF_GPIOTE, s_gpiote_channel);

    /* --- GPPI: GPIOTE IN event to TIMER1 COUNT task --- */
    uint32_t gpiote_evt_addr =
        nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(s_gpiote_channel));
    uint32_t timer_task_addr = nrfx_timer_task_address_get(&s_timer, NRF_TIMER_TASK_COUNT);

    err = nrfx_gppi_conn_alloc(gpiote_evt_addr, timer_task_addr, &s_gppi_handle);
    if (err < 0) {
        LOG_ERR("GPPI connection alloc failed: %d", err);
        nrf_gpiote_event_disable(NRF_GPIOTE, s_gpiote_channel);
        (void)nrfx_gpiote_channel_free(gpiote, s_gpiote_channel);
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

    /* Enable GPPI connection */
    nrfx_gppi_conn_enable(s_gppi_handle);

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

    nrfx_gppi_conn_disable(s_gppi_handle);
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
    if (elapsed_frames == 0 || elapsed_frames > WS_SYNC_MAX_ELAPSED_FRAMES) {
        s_diag.rejected_count++;
        *correction_us = 0;
        return false;
    }

    int32_t error_us;
    int32_t correction;
    ws_sync_math_result_t result = ws_sync_calculate_correction(
        delta, elapsed_frames, &error_us, &correction);
    if (result == WS_SYNC_MATH_NO_SIGNAL) {
        s_diag.no_signal_count++;
        *correction_us = 0;
        return false;
    }

    if (result != WS_SYNC_MATH_VALID) {
        s_diag.rejected_count++;
        *correction_us = 0;
        return false;
    }

    /* Expect 960 WS edges per 20 ms frame (48 kHz), scaled by elapsed_frames.
     * The helper converts the edge difference to microseconds at the physical
     * 48 kHz WS rate and bounds the resulting timing correction.
     */

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
