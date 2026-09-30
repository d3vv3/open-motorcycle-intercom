#include <assert.h>
#include "call_policy.h"
#include "call_cues.h"
#include "call_privacy.h"
#include "button_control.h"

static omi_call_decision_t step(omi_call_policy_t *policy, uint8_t call, uint8_t setup,
                                uint8_t held, bool connected)
{
    phone_audio_call_state_t state = {
        .slc_connected = connected, .call = call, .call_setup = setup, .call_held = held,
    };
    return omi_call_policy_step(policy, &state);
}

int main(void)
{
    omi_call_policy_t p = {0};
    omi_call_decision_t d = step(&p, 0, 1, 0, true);
    assert(d.announce && d.current_incoming && !d.privacy && !d.end_beep);
    d = step(&p, 0, 1, 0, true);
    assert(!d.announce && d.current_incoming && !d.end_beep);
    d = step(&p, 1, 1, 0, true); /* Answer before call_setup clears. */
    assert(d.privacy && !d.current_incoming && d.cancel_announcement);
    d = step(&p, 1, 1, 0, true);
    assert(!d.announce && !d.cancel_announcement && !d.current_incoming); /* Repeated RING stays canceled. */
    d = step(&p, 1, 0, 0, true);
    assert(!d.cancel_announcement && d.privacy && !d.end_beep);
    d = step(&p, 1, 0, 2, true);
    assert(d.privacy && !d.end_beep);
    d = step(&p, 1, 1, 2, true); /* New waiting call while first call is held. */
    assert(d.announce && d.current_incoming && d.privacy);
    d = step(&p, 1, 1, 2, true);
    assert(!d.announce && d.current_incoming);
    d = step(&p, 1, 0, 2, true);
    assert(d.cancel_announcement && !d.end_beep);
    d = step(&p, 0, 0, 0, true);
    assert(d.end_beep && !d.privacy);
    assert(!step(&p, 0, 0, 0, true).end_beep);

    p = (omi_call_policy_t){0};
    step(&p, 0, 1, 0, true);
    d = step(&p, 0, 0, 0, true);
    assert(d.cancel_announcement && !d.current_incoming && !d.end_beep); /* Rejected/missed ring. */
    step(&p, 0, 1, 0, true);
    d = step(&p, 0, 0, 0, false);
    assert(d.cancel_announcement && !d.end_beep);
    step(&p, 0, 2, 0, true);
    d = step(&p, 0, 3, 0, true);
    assert(!d.privacy && !d.end_beep);
    d = step(&p, 1, 0, 0, true);
    assert(d.privacy && !d.announce);
    d = step(&p, 0, 0, 0, false);
    assert(d.end_beep && !d.privacy);

    p = (omi_call_policy_t){0};
    step(&p, 1, 0, 0, true);
    d = step(&p, 1, 1, 0, true); /* Waiting call during an active call. */
    assert(d.announce && d.current_incoming && d.privacy);
    d = step(&p, 0, 1, 0, true); /* The answered call ends; waiting call is still ringing. */
    assert(d.end_beep && d.current_incoming && !d.cancel_announcement);
    d = step(&p, 0, 1, 0, true);
    assert(!d.end_beep && !d.announce);
    d = step(&p, 0, 0, 0, true); /* Unanswered waiting call ends without a beep. */
    assert(d.cancel_announcement && !d.end_beep);
    d = step(&p, 0, 0, 0, true);
    assert(!d.cancel_announcement && !d.end_beep);

    p = (omi_call_policy_t){0};
    step(&p, 1, 0, 0, true);
    d = step(&p, 1, 0, 2, true);
    assert(d.privacy && !d.end_beep);
    d = step(&p, 1, 1, 2, true); /* Waiting ring also works during a held call. */
    assert(d.announce && d.current_incoming && d.privacy);
    d = step(&p, 1, 0, 2, true);
    assert(d.cancel_announcement && !d.end_beep);
    d = step(&p, 0, 0, 0, true);
    assert(d.end_beep);

    /* Failed lifecycle-lock acquisition must leave the cue for a later tick. */
    unsigned pending_beeps = 1;
    if (omi_call_end_acknowledged(pending_beeps, false)) pending_beeps--;
    assert(pending_beeps == 1);
    pending_beeps++; /* A new call ends while the first beep is pending. */
    if (omi_call_end_acknowledged(pending_beeps, true)) pending_beeps--;
    assert(pending_beeps == 1);
    if (omi_call_end_acknowledged(pending_beeps, true)) pending_beeps--;
    assert(pending_beeps == 0);
    assert(!omi_call_end_acknowledged(pending_beeps, true));

    omi_privacy_reconcile_t privacy = {0};
    assert(omi_privacy_next(&privacy, 0, false, false) == OMI_PRIVACY_NONE);
    assert(omi_privacy_next(&privacy, 1, true, true) == OMI_PRIVACY_PAUSE);
    assert(omi_privacy_next(&privacy, 1, true, true) == OMI_PRIVACY_PAUSE); /* Retry. */
    privacy.pause_acked = true;
    assert(omi_privacy_next(&privacy, 1, true, true) == OMI_PRIVACY_NONE);
    assert(omi_privacy_next(&privacy, 2, false, true) == OMI_PRIVACY_PAUSE);
    privacy.pause_acked = true;
    assert(omi_privacy_next(&privacy, 2, false, true) == OMI_PRIVACY_RESUME);
    assert(!omi_privacy_resume_settled(1199, 1000));
    assert(omi_privacy_resume_settled(1200, 1000));
    assert(omi_privacy_next(&privacy, 3, true, true) == OMI_PRIVACY_PAUSE);
    /* The old resume ACK cannot mark this new epoch complete. */
    assert(privacy.epoch == 3 && !privacy.pause_acked);
    assert(omi_privacy_next(&privacy, 4, false, true) == OMI_PRIVACY_PAUSE);
    privacy.pause_acked = true;
    assert(omi_privacy_next(&privacy, 4, false, true) == OMI_PRIVACY_RESUME);
    assert(omi_call_volume_target(OMI_ACTION_VOLUME_UP, false) == OMI_VOLUME_MESH);
    assert(omi_call_volume_target(OMI_ACTION_VOLUME_UP, true) == OMI_VOLUME_BLUETOOTH);
    assert(omi_call_volume_target(OMI_ACTION_BLUETOOTH_VOLUME_DOWN, false) == OMI_VOLUME_BLUETOOTH);
    assert(omi_call_volume_target(OMI_ACTION_BLUETOOTH_VOLUME_DOWN, true) == OMI_VOLUME_BLUETOOTH);
    return 0;
}
