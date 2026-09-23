/**
 * @file button.h
 * @brief Button handler for release-classified boot-button gestures
 *
 * This module provides button handling with timestamped debounce and release
 * classified gestures.
 * The Function CoreBoard-1 boot button is on the board-defined GPIO.
 */

#ifndef OMI_BUTTON_H
#define OMI_BUTTON_H

#include <stdbool.h>

#include "esp_err.h"

#include "button_gesture.h"
#include "omi_board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Boot button GPIO on the Function CoreBoard-1
 */
#define BUTTON_BOOT_GPIO OMI_BOARD_GPIO_BOOT_BUTTON

/**
 * Gesture thresholds in milliseconds. Classification happens only on release.
 */
/**
 * @brief Button debounce time in milliseconds
 */
#define BUTTON_DEBOUNCE_MS 50

/* ============================================================================
 * Callbacks
 * ============================================================================ */

/**
 * @brief Release-classified boot-button gesture
 *
 * Releases from 50 ms to less than 2000 ms produce SHORT_PRESS. A hold of
 * exactly 6000 ms is a pairing gesture. Releases under 50 ms produce NONE.
 */
typedef void (*button_gesture_cb_t)(button_gesture_t gesture, int button_gpio);

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Initialize button handler
 *
 * Sets up GPIO and interrupt handler for the boot button.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_init(void);

/**
 * @brief Deinitialize button handler
 *
 * Cleans up GPIO and interrupt handlers.
 */
void button_deinit(void);

/**
 * @brief Register callback for release-classified gestures
 *
 * The callback runs in the button task after a debounced release.
 *
 * @param callback Function to call for a classified gesture, or NULL to unregister
 */
void button_register_gesture_callback(button_gesture_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* OMI_BUTTON_H */
