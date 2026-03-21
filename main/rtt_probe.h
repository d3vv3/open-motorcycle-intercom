#ifndef OMI_RTT_PROBE_H
#define OMI_RTT_PROBE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t sent;
    uint32_t recv;
    uint32_t lost;
    uint32_t rtt_ms_avg;
    uint32_t rtt_ms_max;
    uint32_t jitter_ms_avg;
    uint32_t jitter_ms_max;
} rtt_probe_stats_t;

void rtt_probe_init(void);
void rtt_probe_tick(int64_t now_ms, bool mesh_active, bool bridge_connected);
bool rtt_probe_handle_packet(uint8_t src_id, const uint8_t *data, uint16_t len);
void rtt_probe_get_stats(rtt_probe_stats_t *stats);

#endif
