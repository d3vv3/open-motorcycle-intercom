/**
 * @file transport_espnow.h
 * @brief ESP-NOW transport glue (fallback when no nRF52840 is detected).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief Initialize the ESP-NOW mesh and register its callbacks. */
esp_err_t transport_espnow_init(void);

/** @brief Queue one encoded frame for the next TDMA slot. */
void transport_espnow_send_audio(const uint8_t *data, uint16_t len, bool active);
