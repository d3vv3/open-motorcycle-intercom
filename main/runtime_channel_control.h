#ifndef OMI_RUNTIME_CHANNEL_CONTROL_H
#define OMI_RUNTIME_CHANNEL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "bridge_protocol_defs.h"

typedef enum {
    OMI_ROLE_QUIET,
    OMI_ROLE_COORDINATOR,
    OMI_ROLE_PARTICIPANT,
} omi_role_voice_t;

static inline omi_role_voice_t omi_role_announcement(bool ready, bool announced,
                                                      bool last_coordinator, bool coordinator)
{
    if (!ready || (announced && last_coordinator == coordinator)) return OMI_ROLE_QUIET;
    return coordinator ? OMI_ROLE_COORDINATOR : OMI_ROLE_PARTICIPANT;
}

/* An error reporting a failed LEAVE is admissible only after the mesh has
 * independently proved cooperative task exit and callback/queue quiescence. */
static inline bool omi_esp_channel_can_apply(bool idle, bool quiesced)
{
    return idle && quiesced;
}

/* Serial comparison handles uint32 status-generation wrap; timestamp also
 * prevents an old cached ACTIVE status from satisfying a new START ACK. */
static inline bool omi_status_after_start(uint32_t generation, int64_t received_at_us,
                                           uint32_t ack_generation, int64_t ack_at_us,
                                           bool have_ack_generation)
{
    return ack_at_us > 0 && received_at_us > ack_at_us &&
           (!have_ack_generation || ((uint32_t)(generation - ack_generation) != 0 &&
                                    (uint32_t)(generation - ack_generation) < UINT32_C(0x80000000)));
}

static inline bool omi_nrf_session_ready(bool start_ack_confirmed, bool user_enabled,
                                          bool fresh_status, uint8_t mesh_state,
                                          uint8_t node_id, uint8_t protocol_version,
                                          uint8_t codec, uint8_t frame_ms,
                                          uint32_t generation, int64_t received_at_us,
                                          uint32_t ack_generation, int64_t ack_at_us,
                                          bool have_ack_generation)
{
    return start_ack_confirmed && user_enabled && fresh_status &&
           mesh_state == BRIDGE_MESH_STATE_ACTIVE && node_id != 0 &&
           protocol_version == BRIDGE_PROTOCOL_VERSION && codec == MESH_AUDIO_CODEC_LC3 &&
           frame_ms == MESH_AUDIO_V2_FRAME_MS &&
           omi_status_after_start(generation, received_at_us, ack_generation,
                                  ack_at_us, have_ack_generation);
}

#endif
