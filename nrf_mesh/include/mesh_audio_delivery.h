#ifndef OMI_MESH_AUDIO_DELIVERY_H
#define OMI_MESH_AUDIO_DELIVERY_H

#include <stdbool.h>
#include <stdint.h>

/* Compare RF ISR receipt time, not the later SPI delivery time. */
static inline bool mesh_audio_delivery_allowed(bool enabled, int64_t received_us,
                                               int64_t cutoff_us)
{
    return enabled && received_us >= cutoff_us;
}

#endif /* OMI_MESH_AUDIO_DELIVERY_H */
