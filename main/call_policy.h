#ifndef OMI_CALL_POLICY_H
#define OMI_CALL_POLICY_H

#include "phone_audio_call_state.h"

typedef struct {
    bool incoming;
    bool incoming_started_answered;
    bool incoming_inhibited;
    bool answered;
} omi_call_policy_t;

typedef struct {
    bool privacy;
    bool current_incoming;
    bool announce;
    bool cancel_announcement;
    bool end_beep;
} omi_call_decision_t;

static inline omi_call_decision_t omi_call_policy_step(omi_call_policy_t *policy,
                                                       const phone_audio_call_state_t *state)
{
    bool incoming = state->slc_connected && state->call_setup == 1u;
    bool answered = state->slc_connected && (state->call != 0u || state->call_held != 0u);
    bool previously_eligible = policy->incoming && !policy->incoming_inhibited;
    if (incoming && !policy->incoming) {
        policy->incoming_started_answered = policy->answered;
        policy->incoming_inhibited = false;
    }
    if (incoming && answered && !policy->incoming_started_answered)
        policy->incoming_inhibited = true;
    bool eligible = incoming && !policy->incoming_inhibited;
    omi_call_decision_t result = {
        .privacy = answered,
        .current_incoming = eligible,
        .announce = eligible && !previously_eligible,
        .cancel_announcement = previously_eligible && !eligible,
        .end_beep = policy->answered && !answered,
    };
    policy->answered = answered;
    policy->incoming = incoming;
    if (!incoming) {
        policy->incoming_started_answered = false;
        policy->incoming_inhibited = false;
    }
    return result;
}

#endif
