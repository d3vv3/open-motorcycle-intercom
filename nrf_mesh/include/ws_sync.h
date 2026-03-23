/**
 * @file ws_sync.h
 * @brief I2S Word-Select (WS) Sync Capture
 *
 * Captures the ESP32's I2S WS signal (16 kHz) on nRF GPIO P0.02 (XIAO D0)
 * using GPIOTE + PPI + TIMER1 to measure the ESP32 playout clock rate
 * relative to the nRF local clock.  The measured drift is used to discipline
 * the TDMA frame timer so both boards stay in lock-step.
 */

#ifndef OMI_WS_SYNC_H
#define OMI_WS_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize WS sync capture hardware (GPIOTE + PPI + TIMER1).
 * @return 0 on success, negative errno on failure.
 */
int ws_sync_init(void);

/**
 * @brief Start capturing WS edges.
 */
void ws_sync_start(void);

/**
 * @brief Stop capturing WS edges.
 */
void ws_sync_stop(void);

/**
 * @brief Sample the WS edge counter and compute drift.
 *
 * Call this once per TDMA frame (every 20 ms).  Returns the measured
 * timing correction in microseconds that should be fed to
 * tdma_tune_timing() to keep the TDMA frame aligned with the ESP32
 * I2S playout clock.
 *
 * @param[out] correction_us  Signed correction: positive = nRF frame
 *             was too fast (stretch it), negative = too slow (shrink it).
 * @return true if a valid measurement was produced, false if not enough
 *         data yet (e.g. first call, or WS signal absent).
 */
bool ws_sync_sample(int32_t *correction_us);

/**
 * @brief Get diagnostic counters for logging.
 */
void ws_sync_get_diag(uint32_t *total_edges, int32_t *last_correction_us,
                      int32_t *cumulative_drift_us);

#endif /* OMI_WS_SYNC_H */
