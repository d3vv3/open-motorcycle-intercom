/**
 * @file ws_sync.h
 * @brief I2S Word-Select (WS) Sync Capture
 *
 * Captures the ESP32's I2S WS signal (16 kHz) on nRF GPIO P0.02 (XIAO D0)
 * using GPIOTE + PPI + TIMER1 for playout-clock diagnostics. RF TDMA timing
 * remains on its fixed 20 ms packet cadence.
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
 * @brief Capture the WS edge counter at a TDMA boundary.
 *
 * This is safe to call from the frame timer ISR. The caller must keep the
 * returned count paired with that frame's sequence number.
 */
bool ws_sync_capture(uint32_t *edge_count);

/**
 * @brief Process a captured WS edge count for diagnostics.
 *
 * Process each frame-boundary capture from deferred context. Returns the
 * measured timing difference in microseconds.
 *
 * @param frame_counter Frame sequence paired with edge_count at capture time.
 * @param edge_count Absolute WS edge count captured at the frame boundary.
 * @param[out] correction_us  Signed correction: positive = nRF frame
 *             was too fast (stretch it), negative = too slow (shrink it).
 * @return true if a valid measurement was produced, false if not enough
 *         data yet (e.g. first call, or WS signal absent).
 */
bool ws_sync_sample(uint32_t frame_counter, uint32_t edge_count,
                    int32_t *correction_us);

typedef struct {
    uint32_t total_edges;
    uint32_t sample_count;
    uint32_t valid_count;
    uint32_t no_signal_count;
    uint32_t rejected_count;
    uint32_t last_delta_edges;
    int32_t last_correction_us;
    int32_t cumulative_drift_us;
} ws_sync_diag_t;

/**
 * @brief Get diagnostic counters for logging.
 */
void ws_sync_get_diag(ws_sync_diag_t *diag);

#endif /* OMI_WS_SYNC_H */
