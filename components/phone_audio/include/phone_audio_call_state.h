#ifndef OMI_PHONE_AUDIO_CALL_STATE_H
#define OMI_PHONE_AUDIO_CALL_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PHONE_AUDIO_CALL_PHASE_IDLE = 0,
    PHONE_AUDIO_CALL_PHASE_INCOMING,
    PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING,
    PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING,
    PHONE_AUDIO_CALL_PHASE_ACTIVE,
    PHONE_AUDIO_CALL_PHASE_HELD,
} phone_audio_call_phase_t;

typedef struct {
    bool slc_connected;
    bool audio_connected;
    bool wideband;
    uint32_t sample_rate;
    uint8_t call;
    uint8_t call_setup;
    uint8_t call_held;
} phone_audio_call_indicators_t;

typedef struct {
    bool slc_connected;
    bool audio_connected;
    bool wideband;
    uint32_t sample_rate;
    uint8_t call;
    uint8_t call_setup;
    uint8_t call_held;
    phone_audio_call_phase_t phase;
} phone_audio_call_state_t;

void phone_audio_call_state_reduce(const phone_audio_call_indicators_t *indicators,
                                   phone_audio_call_state_t *state);

/* HFP disconnect remains attributable to its bound peer after A2DP is gone. */
bool phone_audio_hfp_connection_allowed(bool a2dp_selected, const uint8_t *a2dp_peer,
                                        bool hf_pending, bool hf_bound, bool teardown_requested,
                                        const uint8_t *hf_peer, const uint8_t *event_peer,
                                        bool disconnected);
bool phone_audio_a2dp_selection_allowed(bool a2dp_selected, bool hf_pending, bool hf_bound);

#endif
