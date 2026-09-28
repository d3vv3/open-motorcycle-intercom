#ifndef OMI_TRANSPORT_NRF_RECONCILE_H
#define OMI_TRANSPORT_NRF_RECONCILE_H

#include <stdbool.h>
#include <stdint.h>

#include "bridge_protocol_defs.h"

typedef enum {
    NRF_MESH_NO_COMMAND,
    NRF_MESH_STOP,
    NRF_MESH_START,
} nrf_mesh_reconcile_action_t;

#define NRF_RECONCILE_INTERVAL_MS      2000
#define NRF_RECONCILE_MAX_ATTEMPTS     3
#define NRF_RECONCILE_SLOW_INTERVAL_MS 10000

static inline int64_t nrf_mesh_reconcile_interval_ms(uint8_t attempts)
{
    return attempts < NRF_RECONCILE_MAX_ATTEMPTS ? NRF_RECONCILE_INTERVAL_MS :
                                                    NRF_RECONCILE_SLOW_INTERVAL_MS;
}

static inline nrf_mesh_reconcile_action_t nrf_mesh_reconcile_action(bool enabled, bool confirmed,
                                                                     uint8_t bridge_state)
{
    if (!enabled) {
        return bridge_state == BRIDGE_MESH_STATE_IDLE ? NRF_MESH_NO_COMMAND : NRF_MESH_STOP;
    }
    if (confirmed) {
        return bridge_state == BRIDGE_MESH_STATE_IDLE ? NRF_MESH_START : NRF_MESH_NO_COMMAND;
    }
    return bridge_state == BRIDGE_MESH_STATE_IDLE ? NRF_MESH_START : NRF_MESH_STOP;
}

#endif
