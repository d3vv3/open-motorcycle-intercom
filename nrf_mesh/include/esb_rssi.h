#ifndef OMI_ESB_RSSI_H
#define OMI_ESB_RSSI_H

#include <stdint.h>

static inline int8_t esb_rssi_to_dbm(int16_t magnitude)
{
    /* NCS 3.4.1 ESB stores nrf_radio_rssi_sample_get() as positive magnitude. */
    return magnitude >= 1 && magnitude <= 127 ? (int8_t)-magnitude : INT8_MAX;
}

#endif /* OMI_ESB_RSSI_H */
