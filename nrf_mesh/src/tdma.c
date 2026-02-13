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
static tdma_slot_callback_t s_slot_callback = NULL;

static uint32_t s_frame_counter = 0;
static int64_t s_frame_start_us = 0;
static int8_t s_slot_index = -1;
static bool s_running = false;

/* ============================================================================
 * Timer Callback
 * ============================================================================ */

static void frame_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    s_frame_counter++;
    s_frame_start_us = k_uptime_get() * 1000;

    /* Call slot callback if we have an assigned slot */
    if (s_slot_callback && s_slot_index >= 0) {
        s_slot_callback(s_slot_index, s_frame_counter);
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int tdma_init(void)
{
    k_timer_init(&s_frame_timer, frame_timer_handler, NULL);
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
    s_running = true;

    /* Start periodic timer: MESH_FRAME_MS interval */
    k_timer_start(&s_frame_timer, K_MSEC(MESH_FRAME_MS), K_MSEC(MESH_FRAME_MS));

    LOG_INF("TDMA started, slot=%d", slot_index);
    return 0;
}

void tdma_stop(void)
{
    if (!s_running) {
        return;
    }

    s_running = false;
    k_timer_stop(&s_frame_timer);

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

void tdma_sync(uint32_t frame_counter, int16_t drift_ppm)
{
    /* Adjust our frame counter based on coordinator's SYNC */
    int32_t frame_diff = (int32_t)frame_counter - (int32_t)s_frame_counter;

    if (frame_diff != 0) {
        LOG_INF("SYNC: adjusting frame counter by %d (drift=%d ppm)", frame_diff, drift_ppm);
        s_frame_counter = frame_counter;
    }

    /* TODO: Apply clock drift correction */
    ARG_UNUSED(drift_ppm);
}
