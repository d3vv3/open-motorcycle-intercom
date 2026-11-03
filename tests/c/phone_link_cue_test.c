#include <assert.h>
#include <stdio.h>
#include "phone_link_cue.h"

#define NONE OMI_PHONE_CUE_NONE
#define CONNECTED OMI_PHONE_CUE_CONNECTED
#define DISCONNECTED OMI_PHONE_CUE_DISCONNECTED

/* Steps once and, like main.c, treats an owed beep as played. */
static omi_phone_cue_t fire(omi_phone_link_cue_t *cue, bool a2dp, bool hfp)
{
    omi_phone_cue_t owed = omi_phone_link_cue_step(cue, a2dp, hfp);
    if (owed != NONE) omi_phone_link_cue_done(cue);
    return owed;
}

static void test_second_profile_fires_once(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, false, false) == NONE);
    assert(fire(&cue, true, false) == NONE); /* A2DP alone. */
    assert(fire(&cue, true, true) == CONNECTED); /* HFP is the second profile. */
    for (int i = 0; i < 5; ++i) assert(fire(&cue, true, true) == NONE);

    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, false, true) == NONE); /* HFP first, then A2DP. */
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, true, true) == NONE);

    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, true) == CONNECTED); /* Both appear in one poll. */
}

static void test_single_profile_reconnect_stays_silent(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, false, true) == NONE); /* A2DP drops, e.g. audio re-config. */
    assert(fire(&cue, true, true) == NONE);
    assert(fire(&cue, true, false) == NONE); /* HFP drops and returns. */
    assert(fire(&cue, true, true) == NONE);
    assert(fire(&cue, false, true) == NONE);
    assert(fire(&cue, true, false) == NONE); /* Swap without both ever being down. */
    assert(fire(&cue, true, true) == NONE);
}

static void test_rearms_after_both_drop(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, false, true) == NONE);
    assert(fire(&cue, false, false) == DISCONNECTED); /* Both down: re-armed. */
    assert(fire(&cue, false, true) == NONE);
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, true, true) == NONE);
    assert(fire(&cue, false, false) == DISCONNECTED);
    assert(fire(&cue, true, true) == CONNECTED); /* Both return together. */
}

static void test_refused_beep_retries_until_link_breaks(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED);
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED); /* Busy cue: still owed. */
    omi_phone_link_cue_done(&cue);
    assert(omi_phone_link_cue_step(&cue, true, true) == NONE);

    cue = (omi_phone_link_cue_t){0};
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED);
    assert(omi_phone_link_cue_step(&cue, true, false) == NONE); /* Stale once a profile drops. */
    assert(omi_phone_link_cue_step(&cue, true, true) == NONE);  /* And it does not re-fire. */
    assert(omi_phone_link_cue_step(&cue, false, false) == DISCONNECTED);
    omi_phone_link_cue_done(&cue);
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED);
}

/* The disconnect beep plays once per earned connection, and a busy cue keeps it owed. */
static void test_disconnect_beeps_once(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, false, false) == DISCONNECTED);
    for (int i = 0; i < 5; ++i) assert(fire(&cue, false, false) == NONE);

    /* Both profiles drop one poll apart: still one beep, on the second drop. */
    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, true, false) == NONE);
    assert(fire(&cue, false, false) == DISCONNECTED);
    assert(fire(&cue, false, false) == NONE);

    /* A refused beep stays owed while the link stays down. */
    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(omi_phone_link_cue_step(&cue, false, false) == DISCONNECTED);
    assert(omi_phone_link_cue_step(&cue, false, false) == DISCONNECTED);
    assert(omi_phone_link_cue_step(&cue, true, false) == DISCONNECTED); /* Half a reconnect. */
    omi_phone_link_cue_done(&cue);
    assert(omi_phone_link_cue_step(&cue, false, false) == NONE);

    /* A connection that never had both profiles earns no disconnect beep. */
    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, false) == NONE);
    assert(fire(&cue, false, false) == NONE);
    assert(fire(&cue, false, true) == NONE);
    assert(fire(&cue, false, false) == NONE);

    /* Even a connect beep skipped for a call earns the disconnect beep. */
    cue = (omi_phone_link_cue_t){0};
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED);
    omi_phone_link_cue_done(&cue);
    assert(fire(&cue, false, false) == DISCONNECTED);
}

static void test_single_profile_blip_no_disconnect_beep(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true) == CONNECTED);
    for (int i = 0; i < 3; ++i) {
        assert(fire(&cue, false, true) == NONE); /* A2DP blip. */
        assert(fire(&cue, true, true) == NONE);
        assert(fire(&cue, true, false) == NONE); /* HFP blip. */
        assert(fire(&cue, true, true) == NONE);
    }
}

static void test_disconnect_we_caused_is_silent(void)
{
    omi_phone_link_cue_t cue = {0};
    assert(fire(&cue, true, true) == CONNECTED);
    omi_phone_link_cue_suppress_disconnect(&cue); /* Mesh on: Bluetooth off. */
    assert(fire(&cue, true, true) == NONE);
    assert(fire(&cue, false, true) == NONE);
    assert(fire(&cue, false, false) == NONE);
    assert(fire(&cue, false, false) == NONE);

    /* The suppression covers that drop only: the next connection beeps both ways. */
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, false, false) == DISCONNECTED);

    /* Suppressing with no full connection does not leak into the next one. */
    cue = (omi_phone_link_cue_t){0};
    omi_phone_link_cue_suppress_disconnect(&cue);
    assert(fire(&cue, true, false) == NONE);
    omi_phone_link_cue_suppress_disconnect(&cue);
    assert(fire(&cue, true, true) == CONNECTED);
    assert(fire(&cue, false, false) == DISCONNECTED);
}

static void test_connect_and_disconnect_alternate(void)
{
    omi_phone_link_cue_t cue = {0};
    omi_phone_cue_t expected = CONNECTED;
    for (int i = 0; i < 6; ++i) {
        bool up = expected == CONNECTED;
        assert(fire(&cue, up, up) == expected);
        assert(fire(&cue, up, up) == NONE);
        expected = up ? DISCONNECTED : CONNECTED;
    }

    /* Reconnected before the disconnect beep was accepted: only the connect beep. */
    cue = (omi_phone_link_cue_t){0};
    assert(fire(&cue, true, true) == CONNECTED);
    assert(omi_phone_link_cue_step(&cue, false, false) == DISCONNECTED); /* Busy cue. */
    assert(omi_phone_link_cue_step(&cue, true, true) == CONNECTED);
    omi_phone_link_cue_done(&cue);
    assert(omi_phone_link_cue_step(&cue, true, true) == NONE);
    assert(fire(&cue, false, false) == DISCONNECTED);
}

int main(void)
{
    test_second_profile_fires_once();
    test_single_profile_reconnect_stays_silent();
    test_rearms_after_both_drop();
    test_refused_beep_retries_until_link_breaks();
    test_disconnect_beeps_once();
    test_single_profile_blip_no_disconnect_beep();
    test_disconnect_we_caused_is_silent();
    test_connect_and_disconnect_alternate();
    puts("phone link cue tests passed");
    return 0;
}
