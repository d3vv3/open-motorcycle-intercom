#ifndef OMI_CHANNEL_CONTROL_H
#define OMI_CHANNEL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_channel.h"

static inline uint8_t mesh_channel_cycle(uint8_t current, int direction)
{
    if (!mesh_channel_valid(current)) return 0;
    return (uint8_t)(direction < 0 ? (current == 1 ? 3 : current - 1) :
                                 (current == 3 ? 1 : current + 1));
}

static inline uint8_t mesh_channel_restore(uint8_t persisted, uint8_t fallback)
{
    return mesh_channel_valid(persisted) ? persisted :
           (mesh_channel_valid(fallback) ? fallback : MESH_CHANNEL_DEFAULT);
}

static inline uint8_t mesh_volume_step(uint8_t current, int direction)
{
    return direction < 0 ? (current < 5 ? 0 : current - 5) :
            (current > 95 ? 100 : current + 5);
}

/* A successful step reaching an endpoint, including a push past it, gets a cue. */
static inline bool mesh_volume_at_limit(uint8_t current, uint8_t next, int direction)
{
    return (direction < 0 && current >= next && next == 0) ||
           (direction > 0 && current <= next && next == 100);
}

#endif
