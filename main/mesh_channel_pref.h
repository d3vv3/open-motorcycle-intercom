#ifndef OMI_MESH_CHANNEL_PREF_H
#define OMI_MESH_CHANNEL_PREF_H

#include <stdint.h>
#include "esp_err.h"
#include "channel_control.h"

/* Load uses the configured fallback for a missing or invalid persisted value.
 * Errors other than missing/invalid are returned to the caller. */
esp_err_t mesh_channel_pref_load(uint8_t fallback);
/* Commit before changing the in-memory selection; on error selection is unchanged. */
esp_err_t mesh_channel_pref_persist(uint8_t channel);
uint8_t mesh_channel_pref_selected(void);

#endif
