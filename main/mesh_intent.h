/**
 * @file mesh_intent.h
 * @brief Persisted user intent for mesh networking (NVS-backed).
 *
 * The intent survives reboots; transports reconcile their runtime state
 * against it. Only the button handler and boot code change it.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

/** @brief Load the persisted intent; missing NVS entries default to disabled. */
esp_err_t mesh_intent_load(void);

/** @brief Persist a new intent to NVS. Does not change the runtime value. */
esp_err_t mesh_intent_persist(bool enabled);

/** @brief Update the runtime intent value. */
void mesh_intent_set(bool enabled);

/** @brief Current runtime intent value. */
bool mesh_intent_enabled(void);
