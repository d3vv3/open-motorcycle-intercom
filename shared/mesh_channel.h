#ifndef OMI_MESH_CHANNEL_H
#define OMI_MESH_CHANNEL_H

#include <stdint.h>

/* Group IDs travel in every packet; each group also has its own RF frequency. */
#define MESH_CHANNEL_DEFAULT 1
#define MESH_CHANNEL_COUNT   3

static inline int mesh_channel_valid(uint8_t channel)
{
    return channel >= MESH_CHANNEL_DEFAULT && channel <= MESH_CHANNEL_COUNT;
}

static inline uint8_t mesh_channel_espnow_rf(uint8_t channel)
{
    switch (channel) {
    case 1: return 1;  /* 2412 MHz */
    case 2: return 6;  /* 2437 MHz */
    case 3: return 11; /* 2462 MHz */
    default: return 0;
    }
}

static inline uint8_t mesh_channel_esb_rf(uint8_t channel)
{
    switch (channel) {
    case 1: return 40; /* 2440 MHz */
    case 2: return 20; /* 2420 MHz */
    case 3: return 60; /* 2460 MHz */
    default: return 0;
    }
}

#endif
