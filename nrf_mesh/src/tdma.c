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
#include "ws_sync.h"

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define FRAME_US (MESH_FRAME_MS * 1000)
#define SLOT_US  (MESH_SLOT_MS * 1000)
#define CONTROL_OFFSET_US (MESH_MAX_NODES * SLOT_US)
#define CONTROL_WINDOW_US 2000

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static struct k_timer s_frame_timer;
static struct k_timer s_slot_timer;
static struct k_timer s_control_timer;
static struct k_work s_slot_work;
static struct k_work s_control_work;
static tdma_slot_callback_t s_slot_callback = NULL;
static tdma_control_callback_t s_control_callback = NULL;
static struct k_spinlock s_tdma_lock;

static uint32_t s_frame_counter = 0;
static int64_t s_frame_start_us = 0;
static int32_t s_tune_offset_us = 0;
static int8_t s_slot_index = -1;
static bool s_running = false;
static bool s_synchronized = false;
static uint8_t s_small_sync_count = 0;
static uint32_t s_last_sync_apply_frame = 0;
static uint32_t s_generation = 0;
static uint32_t s_slot_generation = 0;
static uint32_t s_slot_frame = 0;
static int64_t s_slot_start_us = 0;
static int64_t s_slot_deadline_us = 0;
static uint32_t s_control_generation = 0;
static uint32_t s_control_frame = 0;
static int64_t s_control_start_us = 0;
static int64_t s_control_deadline_us = 0;
static tdma_stats_t s_stats;

/* ============================================================================
 * Work Handler (runs in system workqueue thread context)
 * ============================================================================ */

static void slot_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    tdma_slot_callback_t callback;
    int8_t slot_index;
    uint32_t frame_counter;
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    bool current = s_running && s_slot_generation == s_generation;
    bool valid = current && now_us >= s_slot_start_us && now_us <= s_slot_deadline_us;
    if (current && now_us > s_slot_deadline_us) {
        s_stats.slot_late_drop++;
    }
    callback = s_slot_callback;
    slot_index = s_slot_index;
    frame_counter = s_slot_frame;
    k_spin_unlock(&s_tdma_lock, key);

    if (valid && callback != NULL && slot_index >= 0) {
        callback(slot_index, frame_counter);
    }
}

static void slot_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    s_stats.slot_due++;
    k_spin_unlock(&s_tdma_lock, key);
    if (k_work_submit(&s_slot_work) <= 0) {
        key = k_spin_lock(&s_tdma_lock);
        s_stats.slot_submit_drop++;
        k_spin_unlock(&s_tdma_lock, key);
    }
}

static void control_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    tdma_control_callback_t callback;
    uint32_t frame_counter;
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    bool current = s_running && s_synchronized && s_control_generation == s_generation;
    bool valid = current && now_us >= s_control_start_us && now_us <= s_control_deadline_us;
    if (current && now_us > s_control_deadline_us) {
        s_stats.control_late_drop++;
    }
    callback = s_control_callback;
    frame_counter = s_control_frame;
    k_spin_unlock(&s_tdma_lock, key);

    if (valid && callback != NULL) {
        callback(frame_counter);
    }
}

static void control_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    s_stats.control_due++;
    k_spin_unlock(&s_tdma_lock, key);
    if (k_work_submit(&s_control_work) <= 0) {
        key = k_spin_lock(&s_tdma_lock);
        s_stats.control_submit_drop++;
        k_spin_unlock(&s_tdma_lock, key);
    }
}

/* ============================================================================
 * Timer Callback (ISR context - keep minimal)
 * ============================================================================ */

static void frame_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int64_t frame_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int8_t slot_index;

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    s_frame_counter++;
    s_frame_start_us = frame_start_us;
    s_tune_offset_us = 0;
    slot_index = s_synchronized ? s_slot_index : -1;
    s_slot_generation = s_generation;
    s_slot_frame = s_frame_counter;
    if (slot_index >= 0) {
        int64_t slot_offset_us = (int64_t)slot_index * SLOT_US;
        s_slot_start_us = frame_start_us + slot_offset_us;
        s_slot_deadline_us = s_slot_start_us + SLOT_US - MESH_GUARD_US;
    }
    s_control_generation = s_generation;
    s_control_frame = s_frame_counter;
    s_control_start_us = frame_start_us + CONTROL_OFFSET_US;
    s_control_deadline_us = s_control_start_us + CONTROL_WINDOW_US - MESH_GUARD_US;

    if (slot_index == 0) {
        k_timer_start(&s_slot_timer, K_NO_WAIT, K_NO_WAIT);
    } else if (slot_index > 0) {
        k_timer_start(&s_slot_timer, K_USEC((int64_t)slot_index * SLOT_US), K_NO_WAIT);
    }
    k_timer_start(&s_control_timer, K_USEC(CONTROL_OFFSET_US), K_NO_WAIT);
    k_spin_unlock(&s_tdma_lock, key);
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int tdma_init(void)
{
    k_timer_init(&s_frame_timer, frame_timer_handler, NULL);
    k_timer_init(&s_slot_timer, slot_timer_handler, NULL);
    k_timer_init(&s_control_timer, control_timer_handler, NULL);
    k_work_init(&s_slot_work, slot_work_handler);
    k_work_init(&s_control_work, control_work_handler);
    LOG_INF("TDMA timing initialized");
    return 0;
}

int tdma_start(int8_t slot_index, bool synchronized)
{
    if (slot_index < 0 || slot_index >= MESH_MAX_NODES) {
        return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        LOG_WRN("TDMA already running");
        return -EALREADY;
    }

    s_slot_index = slot_index;
    s_frame_counter = 0;
    s_frame_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
    s_tune_offset_us = 0;
    s_small_sync_count = 0;
    s_last_sync_apply_frame = 0;
    s_generation++;
    s_running = true;
    s_synchronized = synchronized;
    k_spin_unlock(&s_tdma_lock, key);

    k_timer_start(&s_frame_timer, K_MSEC(MESH_FRAME_MS), K_MSEC(MESH_FRAME_MS));

    LOG_INF("TDMA started (adaptive relative), slot=%d", slot_index);
    return 0;
}

void tdma_stop(void)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    s_running = false;
    s_synchronized = false;
    s_generation++;
    k_spin_unlock(&s_tdma_lock, key);
    k_timer_stop(&s_frame_timer);
    k_timer_stop(&s_slot_timer);
    k_timer_stop(&s_control_timer);
    k_work_cancel(&s_slot_work);
    k_work_cancel(&s_control_work);

    LOG_INF("TDMA stopped");
}

void tdma_set_slot_callback(tdma_slot_callback_t cb)
{
    s_slot_callback = cb;
}

void tdma_set_control_callback(tdma_control_callback_t cb)
{
    s_control_callback = cb;
}

void tdma_set_slot_index(int8_t slot_index)
{
    if (slot_index < 0 || slot_index >= MESH_MAX_NODES) {
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    s_slot_index = slot_index;
    k_spin_unlock(&s_tdma_lock, key);
}

void tdma_get_stats(tdma_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    *stats = s_stats;
    k_spin_unlock(&s_tdma_lock, key);
}

uint32_t tdma_get_frame_counter(void)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    uint32_t frame_counter = s_frame_counter;
    k_spin_unlock(&s_tdma_lock, key);
    return frame_counter;
}

int32_t tdma_get_time_to_slot_us(void)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (s_slot_index < 0 || !s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return -1;
    }

    int8_t slot_index = s_slot_index;
    int64_t frame_start_us = s_frame_start_us;
    k_spin_unlock(&s_tdma_lock, key);

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t frame_elapsed = (now_us - frame_start_us) % FRAME_US;
    if (frame_elapsed < 0) {
        frame_elapsed += FRAME_US;
    }
    int64_t slot_start = (int64_t)slot_index * SLOT_US;

    if (frame_elapsed < slot_start) {
        return (int32_t)(slot_start - frame_elapsed);
    } else {
        return (int32_t)(FRAME_US - frame_elapsed + slot_start);
    }
}

void tdma_tune_timing(int32_t offset_us)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    s_tune_offset_us += offset_us;
    k_spin_unlock(&s_tdma_lock, key);
}

void tdma_sync(uint32_t frame_counter, int16_t drift_ppm, int64_t frame_start_us)
{
    bool log_adjustment = false;
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t elapsed_us = (now_us - frame_start_us) % FRAME_US;
    if (elapsed_us < 0) {
        elapsed_us += FRAME_US;
    }
    int64_t next_frame_us = FRAME_US - elapsed_us;

    k_timer_stop(&s_frame_timer);
    k_timer_stop(&s_slot_timer);
    k_timer_stop(&s_control_timer);
    k_work_cancel(&s_slot_work);
    k_work_cancel(&s_control_work);
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    /* Adjust our frame counter based on coordinator's SYNC */
    int32_t frame_diff = (int32_t)frame_counter - (int32_t)s_frame_counter;

    if (frame_diff > 1 || frame_diff < -1) {
        log_adjustment = true;
        s_frame_counter = frame_counter;
        s_small_sync_count = 0;
        s_last_sync_apply_frame = frame_counter;

    } else if (frame_diff == 1 || frame_diff == -1) {
        /* Ignore +/-1 jitter corrections to avoid slot-phase thrash on participant. */
        s_small_sync_count++;
        if (s_small_sync_count >= 4) {
            s_small_sync_count = 0;
        }
    } else {
        s_small_sync_count = 0;
    }
    s_frame_counter = frame_counter;
    s_frame_start_us = frame_start_us;
    s_synchronized = true;
    s_generation++;

    k_spin_unlock(&s_tdma_lock, key);

    k_timer_start(&s_frame_timer, K_USEC(next_frame_us), K_USEC(FRAME_US));

    if (log_adjustment) {
        LOG_INF("SYNC: adjusting frame counter by %d (drift=%d ppm)", frame_diff, drift_ppm);
    }

    ARG_UNUSED(drift_ppm);
}
