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
#include <zephyr/sys/util.h>

#include "mesh_protocol.h"
#include "ws_sync.h"

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define FRAME_US                          (MESH_FRAME_MS * 1000)
#define SLOT_US                           (MESH_SLOT_MS * 1000)
#define CONTROL_OFFSET_US                 (MESH_MAX_NODES * SLOT_US)
#define CONTROL_WINDOW_US                 2000
#define TIMER_QUANTUM_US                  (1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_FRAME_CORRECTION_US           TIMER_QUANTUM_US
#define MAX_PENDING_CORRECTION_US         2000
#define MAX_SYNC_DRIFT_PPM                5000
#define SYNC_REACQUIRE_FRAME_THRESHOLD    4
#define SYNC_REACQUIRE_PHASE_THRESHOLD_US MESH_GUARD_US
#define FRAME_HISTORY_SIZE                16

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static struct k_timer s_frame_timer;
static struct k_timer s_slot_timer;
static struct k_timer s_control_timer;
static struct k_work s_slot_work;
static struct k_work s_control_work;
static struct k_work s_discipline_work;
static tdma_slot_callback_t s_slot_callback = NULL;
static tdma_control_callback_t s_control_callback = NULL;
static struct k_spinlock s_tdma_lock;

static uint32_t s_frame_counter = 0;
static int64_t s_frame_start_us = 0;
static int32_t s_pending_correction_us = 0;
static int32_t s_rate_whole_us = 0;
static int32_t s_rate_fraction = 0;
static int64_t s_rate_residual = 0;
static int8_t s_slot_index = -1;
static bool s_running = false;
static bool s_synchronized = false;
static bool s_local_clock_source = false;
static bool s_timer_quiesced = false;
static uint32_t s_generation = 0;
static int64_t s_next_frame_deadline_us = 0;
static int64_t s_last_callback_us = 0;
static uint32_t s_slot_generation = 0;
static uint32_t s_slot_frame = 0;
static int64_t s_slot_start_us = 0;
static int64_t s_slot_deadline_us = 0;
static uint32_t s_control_generation = 0;
static uint32_t s_control_frame = 0;
static int64_t s_control_start_us = 0;
static int64_t s_control_deadline_us = 0;
static tdma_stats_t s_stats;
static uint32_t s_discipline_frame = 0;
static uint32_t s_discipline_edge_count = 0;

struct frame_boundary {
    uint32_t frame_counter;
    int64_t start_us;
    bool valid;
};

static struct frame_boundary s_frame_history[FRAME_HISTORY_SIZE];
static uint8_t s_frame_history_head;

static void frame_history_invalidate_locked(void)
{
    for (int i = 0; i < FRAME_HISTORY_SIZE; i++) {
        s_frame_history[i].valid = false;
    }
    s_frame_history_head = 0;
}

static void frame_history_record_locked(uint32_t frame_counter, int64_t start_us)
{
    s_frame_history[s_frame_history_head] = (struct frame_boundary){
        .frame_counter = frame_counter,
        .start_us = start_us,
        .valid = true,
    };
    s_frame_history_head = (uint8_t)((s_frame_history_head + 1U) % FRAME_HISTORY_SIZE);
}

static bool frame_history_lookup_locked(uint32_t frame_counter, int64_t *start_us)
{
    for (int i = 0; i < FRAME_HISTORY_SIZE; i++) {
        if (s_frame_history[i].valid && s_frame_history[i].frame_counter == frame_counter) {
            *start_us = s_frame_history[i].start_us;
            return true;
        }
    }
    return false;
}

static int32_t clamp_pending_correction(int64_t correction_us)
{
    return (int32_t)CLAMP(correction_us, -MAX_PENDING_CORRECTION_US, MAX_PENDING_CORRECTION_US);
}

static void set_rate_correction_locked(int32_t drift_ppm)
{
    int32_t scaled = CLAMP(drift_ppm, -MAX_SYNC_DRIFT_PPM, MAX_SYNC_DRIFT_PPM) * FRAME_US;
    s_rate_whole_us = scaled / 1000000;
    s_rate_fraction = scaled % 1000000;
}

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

static void discipline_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    bool sample = s_running && s_local_clock_source;
    uint32_t frame_counter = s_discipline_frame;
    uint32_t edge_count = s_discipline_edge_count;
    k_spin_unlock(&s_tdma_lock, key);

    int32_t correction_us;
    if (sample && ws_sync_sample(frame_counter, edge_count, &correction_us)) {
        tdma_tune_timing(correction_us);
    }
}

/* ============================================================================
 * Timer Callback (ISR context - keep minimal)
 * ============================================================================ */

static void frame_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int8_t slot_index;
    int32_t applied_correction_us = 0;
    bool submit_discipline = false;
    uint32_t captured_edges = 0;
    bool captured = false;

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running || s_timer_quiesced) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    int64_t scheduled_start_us = s_next_frame_deadline_us;
    int64_t callback_jitter_us = now_us - scheduled_start_us;
    s_stats.callback_jitter_us = (int32_t)CLAMP(callback_jitter_us, INT32_MIN, INT32_MAX);
    uint64_t jitter_abs =
        callback_jitter_us >= 0 ? (uint64_t)callback_jitter_us : (uint64_t)-callback_jitter_us;
    uint32_t jitter_abs_us = (uint32_t)MIN(jitter_abs, (uint64_t)UINT32_MAX);
    s_stats.callback_jitter_max_us = MAX(s_stats.callback_jitter_max_us, jitter_abs_us);
    if (s_last_callback_us != 0) {
        s_stats.measured_interval_us = (uint32_t)CLAMP(now_us - s_last_callback_us, 0, UINT32_MAX);
    }
    s_last_callback_us = now_us;

    s_frame_counter++;
    int64_t frame_start_us = scheduled_start_us;
    frame_history_record_locked(s_frame_counter, frame_start_us);
    if (scheduled_start_us + FRAME_US <= now_us) {
        uint32_t skipped = (uint32_t)((now_us - scheduled_start_us) / FRAME_US);
        frame_start_us += (int64_t)skipped * FRAME_US;
        if (skipped >= FRAME_HISTORY_SIZE) {
            frame_history_invalidate_locked();
            s_frame_counter += skipped;
            frame_history_record_locked(s_frame_counter, frame_start_us);
        } else {
            for (uint32_t i = 0; i < skipped; i++) {
                s_frame_counter++;
                frame_history_record_locked(s_frame_counter,
                                            scheduled_start_us + (int64_t)(i + 1U) * FRAME_US);
            }
        }
        s_stats.skipped_frame_count += skipped;
    }

    int32_t rate_correction_us = s_rate_whole_us;
    s_rate_residual += s_rate_fraction;
    if (s_rate_residual >= 1000000) {
        rate_correction_us++;
        s_rate_residual -= 1000000;
    } else if (s_rate_residual <= -1000000) {
        rate_correction_us--;
        s_rate_residual += 1000000;
    }
    s_pending_correction_us =
        clamp_pending_correction((int64_t)s_pending_correction_us + rate_correction_us);
    if (s_pending_correction_us >= TIMER_QUANTUM_US) {
        applied_correction_us = MAX_FRAME_CORRECTION_US;
    } else if (s_pending_correction_us <= -TIMER_QUANTUM_US) {
        applied_correction_us = -MAX_FRAME_CORRECTION_US;
    }
    s_pending_correction_us -= applied_correction_us;
    int64_t next_deadline_us = frame_start_us + FRAME_US + applied_correction_us;
    if (next_deadline_us <= now_us) {
        /* The shortened boundary has already passed; retain it for a future frame. */
        s_pending_correction_us += applied_correction_us;
        applied_correction_us = 0;
        next_deadline_us = frame_start_us + FRAME_US;
    }

    s_stats.correction_pending_us = s_pending_correction_us;
    s_stats.last_correction_us = applied_correction_us;
    s_stats.commanded_period_us = FRAME_US + applied_correction_us;
    s_stats.correction_applied_us += applied_correction_us;
    if (applied_correction_us != 0) {
        s_stats.correction_apply_count++;
    }

    s_frame_start_us = frame_start_us;
    s_next_frame_deadline_us = next_deadline_us;

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

    if (slot_index >= 0 && now_us <= s_slot_deadline_us) {
        int64_t slot_delay_us = MAX(s_slot_start_us - now_us, 0);
        k_timer_start(&s_slot_timer, K_USEC(slot_delay_us), K_NO_WAIT);
    } else if (slot_index >= 0) {
        s_stats.slot_late_drop++;
    }
    if (s_synchronized && now_us <= s_control_deadline_us) {
        int64_t control_delay_us = MAX(s_control_start_us - now_us, 0);
        k_timer_start(&s_control_timer, K_USEC(control_delay_us), K_NO_WAIT);
    } else if (s_synchronized) {
        s_stats.control_late_drop++;
    }
    k_timer_start(&s_frame_timer, K_USEC(next_deadline_us - now_us), K_NO_WAIT);
    if (s_local_clock_source) {
        s_stats.discipline_due++;
        captured = ws_sync_capture(&captured_edges);
        if (captured) {
            s_discipline_frame = s_frame_counter;
            s_discipline_edge_count = captured_edges;
            submit_discipline = true;
        } else {
            s_stats.discipline_capture_drop++;
        }
    }
    k_spin_unlock(&s_tdma_lock, key);

    if (submit_discipline && k_work_submit(&s_discipline_work) <= 0) {
        key = k_spin_lock(&s_tdma_lock);
        s_stats.discipline_submit_drop++;
        k_spin_unlock(&s_tdma_lock, key);
    }
}

static void acquire_sync(uint32_t frame_counter, int16_t drift_ppm, int64_t frame_start_us,
                         bool reacquire)
{
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t next_deadline_us = frame_start_us + FRAME_US;
    if (next_deadline_us <= now_us) {
        uint32_t elapsed_frames = (uint32_t)((now_us - next_deadline_us) / FRAME_US) + 1U;
        frame_counter += elapsed_frames;
        frame_start_us = next_deadline_us + (int64_t)(elapsed_frames - 1U) * FRAME_US;
        next_deadline_us += (int64_t)elapsed_frames * FRAME_US;
    }

    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    s_timer_quiesced = true;
    s_synchronized = false;
    uint32_t generation = ++s_generation;
    k_spin_unlock(&s_tdma_lock, key);

    k_timer_stop(&s_frame_timer);
    k_timer_stop(&s_slot_timer);
    k_timer_stop(&s_control_timer);
    k_work_cancel(&s_slot_work);
    k_work_cancel(&s_control_work);
    k_work_cancel(&s_discipline_work);

    key = k_spin_lock(&s_tdma_lock);
    if (!s_running || s_generation != generation) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    if (next_deadline_us <= now_us) {
        uint32_t elapsed_frames = (uint32_t)((now_us - next_deadline_us) / FRAME_US) + 1U;
        frame_counter += elapsed_frames;
        frame_start_us = next_deadline_us + (int64_t)(elapsed_frames - 1U) * FRAME_US;
        next_deadline_us += (int64_t)elapsed_frames * FRAME_US;
    }

    s_frame_counter = frame_counter;
    s_frame_start_us = frame_start_us;
    frame_history_invalidate_locked();
    frame_history_record_locked(frame_counter, frame_start_us);
    s_next_frame_deadline_us = next_deadline_us;
    s_last_callback_us = 0;
    s_pending_correction_us = 0;
    s_stats.correction_pending_us = 0;
    s_stats.last_correction_us = 0;
    s_stats.commanded_period_us = FRAME_US;
    s_stats.sync_phase_correction_us = 0;
    if (!reacquire) {
        s_stats.sync_frame_diff = 0;
    }
    set_rate_correction_locked(drift_ppm);
    s_rate_residual = 0;
    s_synchronized = true;
    s_timer_quiesced = false;
    if (reacquire) {
        s_stats.sync_reacquire_count++;
    } else {
        s_stats.sync_acquire_count++;
    }
    k_timer_start(&s_frame_timer, K_USEC(next_deadline_us - now_us), K_NO_WAIT);
    k_spin_unlock(&s_tdma_lock, key);

    LOG_INF("SYNC %s: frame=%u drift=%d ppm", reacquire ? "reacquired" : "acquired", frame_counter,
            drift_ppm);
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
    k_work_init(&s_discipline_work, discipline_work_handler);
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
    frame_history_invalidate_locked();
    frame_history_record_locked(s_frame_counter, s_frame_start_us);
    s_next_frame_deadline_us = s_frame_start_us + FRAME_US;
    s_last_callback_us = 0;
    s_pending_correction_us = 0;
    set_rate_correction_locked(0);
    s_rate_residual = 0;
    s_stats.correction_pending_us = 0;
    s_stats.last_correction_us = 0;
    s_stats.commanded_period_us = FRAME_US;
    s_stats.measured_interval_us = 0;
    s_stats.callback_jitter_us = 0;
    s_generation++;
    s_running = true;
    s_synchronized = synchronized;
    s_local_clock_source = false;
    s_timer_quiesced = false;
    int64_t arm_now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    k_timer_start(&s_frame_timer,
                  K_USEC(MAX(s_next_frame_deadline_us - arm_now_us, TIMER_QUANTUM_US)), K_NO_WAIT);
    k_spin_unlock(&s_tdma_lock, key);

    LOG_INF("TDMA started (adaptive absolute), slot=%d", slot_index);
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
    s_local_clock_source = false;
    s_timer_quiesced = true;
    s_generation++;
    k_spin_unlock(&s_tdma_lock, key);
    k_timer_stop(&s_frame_timer);
    k_timer_stop(&s_slot_timer);
    k_timer_stop(&s_control_timer);
    k_work_cancel(&s_slot_work);
    k_work_cancel(&s_control_work);
    k_work_cancel(&s_discipline_work);

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
    int64_t next_frame_deadline_us = s_next_frame_deadline_us;
    k_spin_unlock(&s_tdma_lock, key);

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t slot_start = (int64_t)slot_index * SLOT_US;

    if (now_us < frame_start_us + slot_start) {
        return (int32_t)(frame_start_us + slot_start - now_us);
    }
    return (int32_t)(MAX(next_frame_deadline_us - now_us, 0) + slot_start);
}

uint32_t tdma_get_current_slot_remaining_us(void)
{
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running || s_slot_index < 0 || now_us < s_slot_start_us) {
        k_spin_unlock(&s_tdma_lock, key);
        return 0;
    }

    uint32_t remaining_us = (uint32_t)CLAMP(s_slot_deadline_us - now_us, 0, UINT32_MAX);
    k_spin_unlock(&s_tdma_lock, key);
    return remaining_us;
}

void tdma_tune_timing(int32_t offset_us)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }
    int64_t requested = (int64_t)s_pending_correction_us + offset_us;
    int32_t bounded = clamp_pending_correction(requested);
    s_stats.tune_request_count++;
    if (bounded != requested) {
        s_stats.tune_clamp_count++;
    }
    s_pending_correction_us = bounded;
    s_stats.correction_pending_us = bounded;
    k_spin_unlock(&s_tdma_lock, key);
}

void tdma_sync(uint32_t frame_counter, int16_t drift_ppm, int64_t frame_start_us)
{
    k_spinlock_key_t key = k_spin_lock(&s_tdma_lock);
    if (!s_running) {
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    if (!s_synchronized) {
        k_spin_unlock(&s_tdma_lock, key);
        acquire_sync(frame_counter, drift_ppm, frame_start_us, false);
        return;
    }

    uint32_t modular_diff = frame_counter - s_frame_counter;
    int32_t frame_diff = (int32_t)modular_diff;
    s_stats.sync_frame_diff = frame_diff;
    if (frame_diff > SYNC_REACQUIRE_FRAME_THRESHOLD ||
        frame_diff < -SYNC_REACQUIRE_FRAME_THRESHOLD) {
        k_spin_unlock(&s_tdma_lock, key);
        acquire_sync(frame_counter, drift_ppm, frame_start_us, true);
        return;
    }

    int64_t local_frame_start_us;
    if (!frame_history_lookup_locked(frame_counter, &local_frame_start_us)) {
        /* A small unresolved delta is safer to defer than to invent an epoch. */
        s_stats.sync_history_miss_count++;
        s_stats.sync_phase_correction_us = 0;
        k_spin_unlock(&s_tdma_lock, key);
        return;
    }

    int32_t phase_error_us = (int32_t)CLAMP(frame_start_us - local_frame_start_us,
                                            -MAX_PENDING_CORRECTION_US, MAX_PENDING_CORRECTION_US);
    s_stats.sync_phase_correction_us = phase_error_us;
    if (phase_error_us > SYNC_REACQUIRE_PHASE_THRESHOLD_US ||
        phase_error_us < -SYNC_REACQUIRE_PHASE_THRESHOLD_US) {
        k_spin_unlock(&s_tdma_lock, key);
        acquire_sync(frame_counter, drift_ppm, frame_start_us, true);
        return;
    }

    s_pending_correction_us = phase_error_us;
    s_stats.correction_pending_us = s_pending_correction_us;
    set_rate_correction_locked(drift_ppm);
    k_spin_unlock(&s_tdma_lock, key);
}
