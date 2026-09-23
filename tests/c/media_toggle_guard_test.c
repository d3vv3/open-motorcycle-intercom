#include <assert.h>
#include <stdio.h>

#include "media_toggle_guard.h"

static void test_pause_then_early_play(void)
{
    media_toggle_guard_t guard = {0};
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 0) == MEDIA_TOGGLE_DISPATCH);
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 1000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 2000, 2000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 2000, 6999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 2000, 7000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_play_then_early_pause_and_boundaries(void)
{
    media_toggle_guard_t guard = {0};
    assert(media_toggle_guard_step(&guard, true, true, false, 0, 0) == MEDIA_TOGGLE_DISPATCH);
    media_toggle_guard_command_result(&guard, true, false, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 5000, 5000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, true, 5000, 5001) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, true, 5000, 10000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_timeout_disconnect_and_call(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 10000) == MEDIA_TOGGLE_DROPPED);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 10001) == MEDIA_TOGGLE_DISPATCH);
    media_toggle_guard_command_result(&guard, true, true, 10001);
    assert(media_toggle_guard_step(&guard, true, false, true, 0, 11000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 11001) == MEDIA_TOGGLE_DISPATCH);
}

static void test_coalesces_repeated_requests(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, false, 100);
    assert(media_toggle_guard_step(&guard, true, true, false, 0, 200) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, true, true, true, 1000, 1000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, true, 1000, 6000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_late_transition_gets_full_settle_time(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 8999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 9000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 13999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 14000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_bounce_requires_new_transition_and_settle(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, false, 1000, 1000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, true, 2000, 2000) == MEDIA_TOGGLE_NONE);
    assert(!guard.transition_observed);
    assert(media_toggle_guard_step(&guard, false, true, true, 2000, 10000) == MEDIA_TOGGLE_DROPPED);

    media_toggle_guard_command_result(&guard, true, true, 11000);
    assert(media_toggle_guard_step(&guard, true, true, false, 19000, 19000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, true, 20000, 20000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 21000, 21000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 21000, 25999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 21000, 26000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_hidden_bounce_uses_latest_callback_transition(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    /* Polling missed an opposite-state transition at 1000 and bounce at 2000. */
    assert(media_toggle_guard_step(&guard, true, true, false, 3000, 6999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 3000, 7999) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 3000, 8000) == MEDIA_TOGGLE_DISPATCH);
}

static void test_late_transition_drops_queued_toggle(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 1000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 10001, 10001) ==
           MEDIA_TOGGLE_DROPPED);
    assert(!guard.waiting && !guard.queued);
}

static void test_poll_observes_early_transition_late(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, true, 0);
    assert(media_toggle_guard_step(&guard, true, true, true, 0, 1000) == MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 11000) ==
           MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 13999) ==
           MEDIA_TOGGLE_NONE);
    assert(media_toggle_guard_step(&guard, false, true, false, 9000, 14000) ==
           MEDIA_TOGGLE_DISPATCH);
}

static void test_ambiguous_failure_is_armed(void)
{
    media_toggle_guard_t guard = {0};
    media_toggle_guard_command_result(&guard, true, false, 100);
    assert(media_toggle_guard_step(&guard, true, true, false, 0, 200) == MEDIA_TOGGLE_NONE);
    assert(guard.waiting && guard.armed);
    assert(media_toggle_guard_step(&guard, true, true, false, 0, 10100) == MEDIA_TOGGLE_DROPPED);
}

int main(void)
{
    test_pause_then_early_play();
    test_play_then_early_pause_and_boundaries();
    test_timeout_disconnect_and_call();
    test_coalesces_repeated_requests();
    test_late_transition_gets_full_settle_time();
    test_bounce_requires_new_transition_and_settle();
    test_hidden_bounce_uses_latest_callback_transition();
    test_late_transition_drops_queued_toggle();
    test_poll_observes_early_transition_late();
    test_ambiguous_failure_is_armed();
    puts("media_toggle_guard_test: ok");
    return 0;
}
