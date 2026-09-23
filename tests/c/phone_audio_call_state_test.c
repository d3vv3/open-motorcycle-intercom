#include <assert.h>
#include <string.h>

#include "phone_audio_call_state.h"

static phone_audio_call_state_t reduce(uint8_t call, uint8_t setup, uint8_t held)
{
    phone_audio_call_indicators_t input = {
        .slc_connected = true, .audio_connected = true, .wideband = false,
        .sample_rate = 8000, .call = call, .call_setup = setup, .call_held = held,
    };
    phone_audio_call_state_t output;
    memset(&output, 0, sizeof(output));
    phone_audio_call_state_reduce(&input, &output);
    return output;
}

static phone_audio_call_state_t reduce_disconnected(void)
{
    phone_audio_call_indicators_t input = {
        .slc_connected = false, .audio_connected = true, .wideband = true,
        .sample_rate = 16000, .call = 1, .call_setup = 1, .call_held = 2,
    };
    phone_audio_call_state_t output;
    memset(&output, 0, sizeof(output));
    phone_audio_call_state_reduce(&input, &output);
    return output;
}

int main(void)
{
    for (uint8_t call = 0; call <= 1; ++call) {
        for (uint8_t setup = 0; setup <= 3; ++setup) {
            for (uint8_t held = 0; held <= 2; ++held) {
                phone_audio_call_state_t state = reduce(call, setup, held);
                phone_audio_call_phase_t expected = PHONE_AUDIO_CALL_PHASE_IDLE;
                if (setup == 1) expected = PHONE_AUDIO_CALL_PHASE_INCOMING;
                else if (setup == 2) expected = PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING;
                else if (setup == 3) expected = PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING;
                else if (call != 0 && held == 2) expected = PHONE_AUDIO_CALL_PHASE_HELD;
                else if (call != 0) expected = PHONE_AUDIO_CALL_PHASE_ACTIVE;
                assert(state.phase == expected);
            }
        }
    }
    assert(reduce(0, 0, 0).phase == PHONE_AUDIO_CALL_PHASE_IDLE);
    assert(reduce(0, 1, 0).phase == PHONE_AUDIO_CALL_PHASE_INCOMING);
    assert(reduce(0, 2, 0).phase == PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING);
    assert(reduce(0, 3, 0).phase == PHONE_AUDIO_CALL_PHASE_OUTGOING_ALERTING);
    assert(reduce(1, 0, 0).phase == PHONE_AUDIO_CALL_PHASE_ACTIVE);
    assert(reduce(1, 0, 1).phase == PHONE_AUDIO_CALL_PHASE_ACTIVE);
    assert(reduce(1, 0, 2).phase == PHONE_AUDIO_CALL_PHASE_HELD);
    assert(reduce(0, 0, 2).phase == PHONE_AUDIO_CALL_PHASE_IDLE);
    assert(reduce(1, 2, 2).phase == PHONE_AUDIO_CALL_PHASE_OUTGOING_DIALING);
    phone_audio_call_state_t disconnected = reduce_disconnected();
    assert(disconnected.phase == PHONE_AUDIO_CALL_PHASE_IDLE);
    assert(!disconnected.audio_connected && !disconnected.wideband);
    assert(disconnected.sample_rate == 0u && disconnected.call == 0u);
    return 0;
}
