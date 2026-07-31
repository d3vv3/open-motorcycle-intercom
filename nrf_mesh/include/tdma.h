/**
 * @file tdma.h
 * @brief TDMA Frame Timing for OMI Mesh
 */

#ifndef OMI_TDMA_H
#define OMI_TDMA_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Callback for slot timing events
 */
typedef void (*tdma_slot_callback_t)(uint8_t slot_index, uint32_t frame_counter);
typedef void (*tdma_control_callback_t)(uint32_t frame_counter);

typedef struct {
    uint32_t slot_due;
    uint32_t slot_submit_drop;
    uint32_t slot_late_drop;
    uint32_t control_due;
    uint32_t control_submit_drop;
    uint32_t control_late_drop;
    uint32_t discipline_due;
    uint32_t discipline_submit_drop;
    uint32_t discipline_capture_drop;
    uint32_t tune_request_count;
    uint32_t tune_clamp_count;
    uint32_t correction_apply_count;
    uint32_t skipped_frame_count;
    uint32_t sync_acquire_count;
    uint32_t sync_reacquire_count;
    uint32_t sync_history_miss_count;
    int64_t correction_applied_us;
    int32_t correction_pending_us;
    int32_t last_correction_us;
    int32_t sync_frame_diff;
    int32_t sync_phase_correction_us;
    uint32_t commanded_period_us;
    uint32_t measured_interval_us;
    int32_t callback_jitter_us;
    uint32_t callback_jitter_max_us;
} tdma_stats_t;

/**
 * @brief Initialize TDMA timing
 * @return 0 on success
 */
int tdma_init(void);

/**
 * @brief Start TDMA frame timer
 * @param slot_index Assigned slot (0-7)
 */
int tdma_start(int8_t slot_index, bool synchronized);

/**
 * @brief Stop TDMA timing
 */
void tdma_stop(void);

/**
 * @brief Set slot callback (called when it's our TX slot)
 */
void tdma_set_slot_callback(tdma_slot_callback_t cb);
void tdma_set_control_callback(tdma_control_callback_t cb);
void tdma_set_slot_index(int8_t slot_index);
void tdma_get_stats(tdma_stats_t *stats);

/**
 * @brief Get current frame counter
 */
uint32_t tdma_get_frame_counter(void);

/**
 * @brief Get microseconds until our TX slot
 */
int32_t tdma_get_time_to_slot_us(void);

/**
 * @brief Synchronize to coordinator timing
 * @param frame_counter Coordinator's frame counter
 * @param drift_ppm Coordinator period correction in ppm; positive is longer.
 */
void tdma_sync(uint32_t frame_counter, int16_t drift_ppm, int64_t frame_start_us);

/**
 * @brief Queue a bounded phase correction for upcoming frame periods.
 * @param offset_us Positive stretches upcoming periods; negative shortens them.
 */
void tdma_tune_timing(int32_t offset_us);

#endif /* OMI_TDMA_H */
