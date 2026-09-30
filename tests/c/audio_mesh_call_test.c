#include <assert.h>
#include <stdio.h>

#include "audio_mesh_call.h"

int main(void)
{
    audio_mesh_call_state_t state = {0};
    assert(!audio_mesh_call_state_active(&state));
    assert(!audio_mesh_call_state_blocked(&state));
    assert(audio_mesh_call_state_update(&state, false) == 0u);
    assert(audio_mesh_call_state_tx_allowed(&state, 0u));
    uint32_t call = audio_mesh_call_state_update(&state, true);
    assert(call == 1u && audio_mesh_call_state_active(&state));
    assert(audio_mesh_call_state_update(&state, true) == call);
    assert(!audio_mesh_call_state_tx_allowed(&state, 0u));
    assert(!audio_mesh_call_rx_allowed(&state, 1001, 0u));
    uint32_t ended = audio_mesh_call_state_update(&state, false);
    assert(ended == 2u && !audio_mesh_call_state_active(&state));
    assert(audio_mesh_call_state_blocked(&state));
    assert(!audio_mesh_call_state_resume(&state, call));
    assert(!audio_mesh_call_state_tx_allowed(&state, ended));
    assert(audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_tx_allowed(&state, 0u));
    assert(audio_mesh_call_state_tx_allowed(&state, ended));
    assert(!audio_mesh_call_rx_allowed(&state, 1000, 1000));
    assert(!audio_mesh_call_rx_allowed(&state, 999, 1000));
    assert(!audio_mesh_call_rx_allowed(&state, 0, 1000));
    assert(audio_mesh_call_rx_allowed(&state, 1001, 1000));
    /* Predecessors in a redundant bundle must be checked independently. */
    assert(!audio_mesh_call_rx_allowed(&state, 980, 1000));
    uint32_t rapid = audio_mesh_call_state_update(&state, true);
    assert(audio_mesh_call_state_update(&state, false) == rapid + 1u);
    assert(!audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_tx_allowed(&state, ended));
    assert(audio_mesh_call_state_resume(&state, rapid + 1u));
    puts("audio mesh call tests passed");
    return 0;
}
