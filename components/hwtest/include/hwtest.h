/**
 * @file hwtest.h
 * @brief Hardware validation tests for OMI
 */

#ifndef OMI_HWTEST_H
#define OMI_HWTEST_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the production audio notification path through the speaker
 *
 * Software counters validate the digital codec write path. Audible output
 * must also be verified physically; disconnected speaker/amplifier hardware
 * cannot be detected by this test.
 * @return ESP_OK on success
 */
esp_err_t hwtest_speaker(void);

/**
 * @brief Run production codec capture and encoding for 5 seconds
 * @return ESP_OK on success
 */
esp_err_t hwtest_mic(void);

/**
 * @brief Run production codec capture -> Opus -> playout loopback
 * @param duration_sec Duration in seconds; 0 runs until an external stop or deinit.
 *                     An already-completed external deinit is successful cleanup.
 * @return ESP_OK on success
 */
esp_err_t hwtest_loopback(uint32_t duration_sec);

#ifdef __cplusplus
}
#endif

#endif /* OMI_HWTEST_H */
