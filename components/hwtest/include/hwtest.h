/**
 * @file hwtest.h
 * @brief Hardware validation tests for OMI
 */

#ifndef OMI_HWTEST_H
#define OMI_HWTEST_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run speaker test (1kHz tone for 2 seconds)
 * @return ESP_OK on success
 */
esp_err_t hwtest_speaker(void);

/**
 * @brief Run microphone test (read ADC and log levels for 5 seconds)
 * @return ESP_OK on success
 */
esp_err_t hwtest_mic(void);

/**
 * @brief Run loopback test (mic -> speaker passthrough)
 * @param duration_sec Duration in seconds (0 = indefinite until stopped)
 * @return ESP_OK on success
 */
esp_err_t hwtest_loopback(uint32_t duration_sec);

#ifdef __cplusplus
}
#endif

#endif /* OMI_HWTEST_H */
