#ifndef OMI_AUDIO_MESH_CALL_H
#define OMI_AUDIO_MESH_CALL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

/* One atomic word prevents an epoch/desired-state snapshot from tearing. */
typedef struct { atomic_uint state; } audio_mesh_call_state_t;

uint32_t audio_mesh_call_state_update(audio_mesh_call_state_t *state, bool active);
bool audio_mesh_call_state_resume(audio_mesh_call_state_t *state, uint32_t epoch);
uint32_t audio_mesh_call_state_epoch(const audio_mesh_call_state_t *state);
bool audio_mesh_call_state_active(const audio_mesh_call_state_t *state);
bool audio_mesh_call_state_blocked(const audio_mesh_call_state_t *state);
bool audio_mesh_call_state_tx_allowed(const audio_mesh_call_state_t *state, uint32_t epoch);
/* timestamp_ms is local receive time, not a sender/capture timestamp. */
bool audio_mesh_call_rx_allowed(const audio_mesh_call_state_t *state, int64_t timestamp_ms,
                                uint32_t cutoff_ms);

#endif
