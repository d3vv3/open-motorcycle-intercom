/**
 * @file tdma.c
 * @brief TDMA Frame Timing Implementation
 *
 * 20ms frames, 2ms slots, matching ESP32 mesh timing
 * Uses Zephyr kernel timer instead of counter device for simplicity
 */

#include "tdma.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mesh_protocol.h"

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define FRAME_US (MESH_FRAME_MS * 1000)
#define SLOT_US  (MESH_SLOT_MS * 1000)

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static struct k_timer s_frame_timer;
static struct k_work s_slot_work;
static tdma_slot_callback_t s_slot_callback = NULL;

static uint32_t s_frame_counter = 0;
static int64_t s_frame_start_us = 0;
static int32_t s_tune_offset_us = 0;
static int8_t s_slot_index = -1;
static bool s_running = false;
static uint8_t s_small_sync_count = 0;

/* ============================================================================
 * Work Handler (runs in system workqueue thread context)
 * ============================================================================ */

static void slot_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Call slot callback - safe to call blocking functions here */
    if (s_slot_callback && s_slot_index >= 0 && s_running) {
        s_slot_callback(s_slot_index, s_frame_counter);
    }
}

/* ============================================================================
 * Timer Callback (ISR context - keep minimal)
 * ============================================================================ */

static void frame_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    s_frame_counter++;

    /* Approximate start time for slot calc */
    s_frame_start_us = k_uptime_get() * 1000;

    /* Base period */
    int32_t period_us = FRAME_US;

    /* Apply manual tuning */
    if (s_tune_offset_us != 0) {
        period_us += s_tune_offset_us;
        s_tune_offset_us = 0;
    }

    /* Clamp minimal period */
    if (period_us < 1000)
        period_us = 1000;
    k_timer_start(&s_frame_timer, K_USEC(period_us), K_NO_WAIT);

    /* Schedule slot work to run in thread context */
    if (s_slot_index >= 0) {
        /* NOTE: Don't printk here - ISR context, can deadlock with USB/UART */
        k_work_submit(&s_slot_work);
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int tdma_init(void)
{
    k_timer_init(&s_frame_timer, frame_timer_handler, NULL);
    k_work_init(&s_slot_work, slot_work_handler);
    LOG_INF("TDMA timing initialized");
    return 0;
}

int tdma_start(int8_t slot_index)
{
    if (s_running) {
        LOG_WRN("TDMA already running");
        return -EALREADY;
    }

    s_slot_index = slot_index;
    s_frame_counter = 0;
    s_frame_start_us = k_uptime_get() * 1000;
    s_tune_offset_us = 0;
    s_small_sync_count = 0;
    s_running = true;

    /* Start one-shot timer for first frame end */
    k_timer_start(&s_frame_timer, K_MSEC(MESH_FRAME_MS), K_NO_WAIT);

    LOG_INF("TDMA started (adaptive relative), slot=%d", slot_index);
    return 0;
}

void tdma_stop(void)
{
    if (!s_running) {
        return;
    }

    s_running = false;
    k_timer_stop(&s_frame_timer);
    k_work_cancel(&s_slot_work);

    LOG_INF("TDMA stopped");
}

void tdma_set_slot_callback(tdma_slot_callback_t cb)
{
    s_slot_callback = cb;
}

uint32_t tdma_get_frame_counter(void)
{
    return s_frame_counter;
}

int32_t tdma_get_time_to_slot_us(void)
{
    if (s_slot_index < 0 || !s_running) {
        return -1;
    }

    int64_t now_us = k_uptime_get() * 1000;
    int64_t frame_elapsed = now_us - s_frame_start_us;
    int64_t slot_start = (int64_t)s_slot_index * SLOT_US;

    if (frame_elapsed < slot_start) {
        return (int32_t)(slot_start - frame_elapsed);
    } else {
        return (int32_t)(FRAME_US - frame_elapsed + slot_start);
    }
}

void tdma_tune_timing(int32_t offset_us)
{
    s_tune_offset_us += offset_us;
}

void tdma_sync(uint32_t frame_counter, int16_t drift_ppm)
{
    /* Adjust our frame counter based on coordinator's SYNC */
    int32_t frame_diff = (int32_t)frame_counter - (int32_t)s_frame_counter;

    if (frame_diff > 1 || frame_diff < -1) {
        LOG_INF("SYNC: adjusting frame counter by %d (drift=%d ppm)", frame_diff, drift_ppm);
        s_frame_counter = frame_counter;
        s_small_sync_count = 0;

        /* For relative timing, we don't strictly need to snap local timer
         * unless we want to align slots perfectly.
         * Snapping relative timer is hard without restarting it.
         * For now, just accept the frame counter update.
         */
    } else if (frame_diff == 1 || frame_diff == -1) {
        /* Ignore +/-1 jitter corrections to avoid slot-phase thrash on participant. */
        s_small_sync_count++;
        if (s_small_sync_count >= 4) {
            s_small_sync_count = 0;
        }
    } else {
        s_small_sync_count = 0;
    }

    ARG_UNUSED(drift_ppm);
}
