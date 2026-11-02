#include <assert.h>
#include <stdio.h>
#include "phone_link_cue.h"

/* Steps once and, like main.c, treats a fired beep as played. */
static bool fire(omi_phone_link_cue_t *cue, bool a2dp, bool hfp)
{
    bool owed = omi_phone_link_cue_step(cue, a2dp, hfp);
    if (owed) omi_phone_link_cue_done(cue);
    return owed;
}

static void test_second_profile_fires_once(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(!fire(&cue, false, false));
    assert(!fire(&cue, true, false)); /* A2DP alone. */
    assert(fire(&cue, true, true));   /* HFP is the second profile. */
    for (int i = 0; i < 5; ++i) assert(!fire(&cue, true, true));

    cue = (omi_phone_link_cue_t){0};
    assert(!fire(&cue, false, true)); /* HFP first, then A2DP. */
    assert(fire(&cue, true, true));
    assert(!fire(&cue, true, true));

    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, true)); /* Both appear in one poll. */
}

static void test_single_profile_reconnect_stays_silent(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true));
    assert(!fire(&cue, false, true)); /* A2DP drops, e.g. audio re-config. */
    assert(!fire(&cue, true, true));
    assert(!fire(&cue, true, false)); /* HFP drops and returns. */
    assert(!fire(&cue, true, true));
    assert(!fire(&cue, false, true));
    assert(!fire(&cue, true, false)); /* Swap without both ever being down. */
    assert(!fire(&cue, true, true));
}

static void test_rearms_after_both_drop(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true));
    assert(!fire(&cue, false, true));
    assert(!fire(&cue, false, false)); /* Both down: re-armed. */
    assert(!fire(&cue, false, true));
    assert(fire(&cue, true, true));
    assert(!fire(&cue, true, true));
    assert(!fire(&cue, false, false));
    assert(fire(&cue, true, true)); /* Both return together. */
}

static void test_refused_beep_retries_until_link_breaks(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(omi_phone_link_cue_step(&cue, true, true));
    assert(omi_phone_link_cue_step(&cue, true, true)); /* Busy cue: still owed. */
    omi_phone_link_cue_done(&cue);
    assert(!omi_phone_link_cue_step(&cue, true, true));

    cue = (omi_phone_link_cue_t){0};
    assert(omi_phone_link_cue_step(&cue, true, true));
    assert(!omi_phone_link_cue_step(&cue, true, false)); /* Stale once a profile drops. */
    assert(!omi_phone_link_cue_step(&cue, true, true));  /* And it does not re-fire. */
    assert(!omi_phone_link_cue_step(&cue, false, false));
    assert(omi_phone_link_cue_step(&cue, true, true));
}

int main(void)
{
    test_second_profile_fires_once();
    test_single_profile_reconnect_stays_silent();
    test_rearms_after_both_drop();
    test_refused_beep_retries_until_link_breaks();
    puts("phone link cue tests passed");
    return 0;
}
