#include "phone_audio_call_state.h"

void phone_audio_call_state_reduce(const phone_audio_call_indicators_t *indicators,
                                   phone_audio_call_state_t *state)
{
    if (indicators == 0 || state == 0) return;
    state->slc_connected = indicators->slc_connected;
    state->audio_connected = indicators->slc_connected && indicators->audio_connected;
    state->wideband = state->audio_connected && indicators->wideband;
    state->sample_rate = state->audio_connected ? indicators->sample_rate : 0u;
    state->call = indicators->slc_connected ? indicators->call : 0u;
    state->call_setup = indicators->slc_connected ? indicators->call_setup : 0u;
    state->call_held = indicators->slc_connected ? indicators->call_held : 0u;
    if (!indicators->slc_connected) {
        state->phase = PHONE_AUDIO_CALL_PHASE_IDLE;
    } else if (indicators->call_setup == 1u) {
        state->phase = PHONE_AUDIO_CALL_PHASE_INCOMING;
    } else if (indicators->call_setup == 2u) {
        state->phase = PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING;
    } else if (indicators->call_setup == 3u) {
        state->phase = PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING;
    } else if (indicators->call != 0u && indicators->call_held == 2u) {
        state->phase = PHONE_AUDIO_CALL_PHASE_HELD;
    } else if (indicators->call != 0u) {
        state->phase = PHONE_AUDIO_CALL_PHASE_ACTIVE;
    } else {
        state->phase = PHONE_AUDIO_CALL_PHASE_IDLE;
    }
}
