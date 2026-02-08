/**
 * @file button.h
 * @brief Button handler for boot button with long press detection
 *
 * This module provides button handling with debouncing and long press detection.
 * On ESP32-S3, GPIO 0 is the boot button.
 */

#ifndef OMI_BUTTON_H
#define OMI_BUTTON_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Boot button GPIO (GPIO 0 on ESP32-S3)
 */
#define BUTTON_BOOT_GPIO 0

/**
 * @brief Long press duration in milliseconds (2 seconds)
 */
#define BUTTON_LONG_PRESS_MS 2000

/**
 * @brief Button debounce time in milliseconds
 */
#define BUTTON_DEBOUNCE_MS 50

/* ============================================================================
 * Callbacks
 * ============================================================================ */

/**
 * @brief Button long press callback function type
 *
 * Called when the boot button has been held for BUTTON_LONG_PRESS_MS.
 *
 * @param button_gpio GPIO number of the button that was pressed
 */
typedef void (*button_long_press_cb_t)(int button_gpio);

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
 * @brief Register callback for long press events
 *
 * The callback will be invoked from the button task when a long press
 * is detected (button held for BUTTON_LONG_PRESS_MS).
 *
 * @param callback Function to call on long press, or NULL to unregister
 */
void button_register_long_press_callback(button_long_press_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* OMI_BUTTON_H */
