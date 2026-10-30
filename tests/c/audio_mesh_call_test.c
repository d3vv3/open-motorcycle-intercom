#include <assert.h>
#include <stdio.h>

#include "audio_mesh_call.h"

static uint32_t s_cutoff_ms;
static unsigned s_cutoff_loads;
static uint32_t load_cutoff(void)
{
    s_cutoff_loads++;
    return s_cutoff_ms;
}

static bool rx_allowed(const audio_mesh_call_state_t *state, int64_t timestamp_ms, uint32_t cutoff_ms)
{
    s_cutoff_ms = cutoff_ms;
    return audio_mesh_call_rx_allowed(state, timestamp_ms, load_cutoff);
}

/* Privacy: the gate word must be loaded before the RX cutoff. Loading the
 * cutoff first could pair the stale pre-resume cutoff with a reopened gate
 * and admit a frame received while paused. */
static void test_gate_word_loaded_before_cutoff(void)
{
    audio_mesh_call_state_t state = {0};
    audio_mesh_call_state_update(&state, true);
    uint32_t paused = audio_mesh_call_state_update(&state, false);
    assert(audio_mesh_call_state_blocked(&state));

    s_cutoff_loads = 0;
    s_cutoff_ms = 0u;
    assert(!audio_mesh_call_rx_allowed(&state, 4000, load_cutoff));
    assert(s_cutoff_loads == 0u); /* blocked gate never reads the cutoff */

    /* A resume publishes the cutoff, then reopens the word (release). A frame
     * that loads the reopened word (acquire) must then load the new cutoff,
     * never the stale pre-resume one it would have read had it loaded first. */
    s_cutoff_ms = 5000u;
    assert(audio_mesh_call_state_resume(&state, paused));
    assert(!audio_mesh_call_rx_allowed(&state, 4000, load_cutoff));
    assert(s_cutoff_loads == 1u);
    assert(audio_mesh_call_rx_allowed(&state, 5001, load_cutoff));
}

int main(void)
{
    test_gate_word_loaded_before_cutoff();

    audio_mesh_call_state_t state = {0};
    assert(!audio_mesh_call_state_active(&state));
    assert(!audio_mesh_call_state_blocked(&state));
    assert(audio_mesh_call_state_update(&state, false) == 0u);
    assert(audio_mesh_call_state_tx_allowed(&state, 0u));
    uint32_t call = audio_mesh_call_state_update(&state, true);
    assert(call == 1u && audio_mesh_call_state_active(&state));
    assert(audio_mesh_call_state_update(&state, true) == call);
    assert(!audio_mesh_call_state_tx_allowed(&state, 0u));
    assert(!rx_allowed(&state, 1001, 0u));
    uint32_t ended = audio_mesh_call_state_update(&state, false);
    assert(ended == 2u && !audio_mesh_call_state_active(&state));
    assert(audio_mesh_call_state_blocked(&state));
    assert(!audio_mesh_call_state_resume(&state, call));
    assert(!audio_mesh_call_state_tx_allowed(&state, ended));
    assert(audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_tx_allowed(&state, 0u));
    assert(audio_mesh_call_state_tx_allowed(&state, ended));
    assert(!rx_allowed(&state, 1000, 1000));
    assert(!rx_allowed(&state, 999, 1000));
    assert(!rx_allowed(&state, 0, 1000));
    assert(rx_allowed(&state, 1001, 1000));
    /* Predecessors in a redundant bundle must be checked independently. */
    assert(!rx_allowed(&state, 980, 1000));
    uint32_t rapid = audio_mesh_call_state_update(&state, true);
    assert(audio_mesh_call_state_update(&state, false) == rapid + 1u);
    assert(!audio_mesh_call_state_resume(&state, ended));
    assert(!audio_mesh_call_state_tx_allowed(&state, ended));
    assert(audio_mesh_call_state_resume(&state, rapid + 1u));
    puts("audio mesh call tests passed");
    return 0;
}
