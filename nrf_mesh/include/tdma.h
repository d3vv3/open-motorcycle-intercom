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

/**
 * @brief Initialize TDMA timing
 * @return 0 on success
 */
int tdma_init(void);

/**
 * @brief Start TDMA frame timer
 * @param slot_index Assigned slot (0-7)
 */
int tdma_start(int8_t slot_index);

/**
 * @brief Stop TDMA timing
 */
void tdma_stop(void);

/**
 * @brief Set slot callback (called when it's our TX slot)
 */
void tdma_set_slot_callback(tdma_slot_callback_t cb);

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
 * @param drift_ppm Clock drift correction
 */
void tdma_sync(uint32_t frame_counter, int16_t drift_ppm);

/**
 * @brief Tune the next frame interval by adding offset_us
 * @param offset_us Microseconds to add (can be negative, though implementation might clamp)
 */
void tdma_tune_timing(int32_t offset_us);

#endif /* OMI_TDMA_H */
