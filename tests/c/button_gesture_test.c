#include <assert.h>
#include <stdint.h>

#include "button_gesture.h"

static button_state_t state;
static bool keys[BUTTON_COUNT];
static button_id_t emitted_id;
static button_event_t emitted_event;
/* Replay mode: hold events enabled, skipped by sample(), so the release
 * scenarios prove release classification is unchanged by them. */
static bool g_holds;
static unsigned g_skipped_holds;

static bool is_hold(button_event_t event)
{
    return event == BUTTON_EVENT_HOLD_LONG || event == BUTTON_EVENT_HOLD_EXTRA_LONG;
}

static void init_keys(uint32_t at)
{
    button_state_init(&state, at, keys);
    button_state_enable_hold_events(&state, g_holds);
}

static void start(uint32_t at)
{
    for (int i = 0; i < BUTTON_COUNT; ++i) keys[i] = false;
    init_keys(at);
    assert(!button_state_sample(&state, at + 20, keys, &emitted_id, &emitted_event));
}

static bool sample(uint32_t at)
{
    while (button_state_sample(&state, at, keys, &emitted_id, &emitted_event)) {
        if (!is_hold(emitted_event)) return true;
        assert(g_holds);
        g_skipped_holds++;
    }
    return false;
}

static void press(button_id_t id, uint32_t at)
{
    keys[id] = true;
    assert(!sample(at));
    assert(!sample(at + 20));
}

static void release_expect(button_id_t id, uint32_t at, bool expected,
                           button_event_t event)
{
    keys[id] = false;
    assert(!sample(at));
    assert(sample(at + 20) == expected);
    if (expected) {
        assert(emitted_id == id);
        assert(emitted_event == event);
    }
    assert(!sample(at + 30));
}

static void release_scenarios(void)
{
    start(0);
    press(BUTTON_CENTER, 100);
    release_expect(BUTTON_CENTER, 149, false, BUTTON_EVENT_SHORT_PRESS);
    press(BUTTON_CENTER, 300);
    release_expect(BUTTON_CENTER, 350, true, BUTTON_EVENT_SHORT_PRESS);
    press(BUTTON_CENTER, 500);
    release_expect(BUTTON_CENTER, 2499, true, BUTTON_EVENT_SHORT_PRESS);
    press(BUTTON_CENTER, 2600);
    release_expect(BUTTON_CENTER, 4600, true, BUTTON_EVENT_LONG_PRESS);
    press(BUTTON_CENTER, 4700);
    release_expect(BUTTON_CENTER, 10699, true, BUTTON_EVENT_LONG_PRESS);
    press(BUTTON_CENTER, 10800);
    release_expect(BUTTON_CENTER, 16800, true, BUTTON_EVENT_EXTRA_LONG_PRESS);

    start(0);
    press(BUTTON_MINUS, 100);
    release_expect(BUTTON_MINUS, 1099, true, BUTTON_EVENT_SHORT_PRESS);
    press(BUTTON_PLUS, 1200);
    assert(!sample(2200)); /* No repeat or long event while held. */
    assert(!sample(7200));
    release_expect(BUTTON_PLUS, 7200, true, BUTTON_EVENT_LONG_PRESS);
    press(BUTTON_MINUS, 7300);
    release_expect(BUTTON_MINUS, 8300, true, BUTTON_EVENT_LONG_PRESS);

    start(0);
    press(BUTTON_CENTER, 100);
    press(BUTTON_MINUS, 200);
    release_expect(BUTTON_MINUS, 250, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 300, false, BUTTON_EVENT_SHORT_PRESS);
    press(BUTTON_CENTER, 400);
    press(BUTTON_PLUS, 500);
    release_expect(BUTTON_PLUS, 550, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 600, false, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    press(BUTTON_CENTER, 100);
    press(BUTTON_MINUS, 200);
    release_expect(BUTTON_MINUS, 249, false, BUTTON_EVENT_MODIFIED_PRESS);
    press(BUTTON_MINUS, 300);
    release_expect(BUTTON_MINUS, 350, true, BUTTON_EVENT_MODIFIED_PRESS);
    press(BUTTON_PLUS, 450);
    release_expect(BUTTON_PLUS, 500, true, BUTTON_EVENT_MODIFIED_PRESS);
    press(BUTTON_MINUS, 550);
    assert(!sample(2500));
    assert(!sample(6600));
    release_expect(BUTTON_MINUS, 6600, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 6700, false, BUTTON_EVENT_EXTRA_LONG_PRESS);

    start(0);
    press(BUTTON_CENTER, 100);
    press(BUTTON_MINUS, 200);
    keys[BUTTON_MINUS] = false;
    keys[BUTTON_PLUS] = true; /* Switch sides before release debounce finishes. */
    assert(!sample(250));
    assert(sample(270));
    assert(emitted_id == BUTTON_MINUS && emitted_event == BUTTON_EVENT_MODIFIED_PRESS);
    assert(!sample(280));
    release_expect(BUTTON_PLUS, 320, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 400, false, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    press(BUTTON_PLUS, 100); /* Either press order is a valid modifier. */
    press(BUTTON_CENTER, 200);
    release_expect(BUTTON_CENTER, 6400, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    assert(!sample(7500)); /* No side hold/channel event while held. */
    release_expect(BUTTON_PLUS, 7600, true, BUTTON_EVENT_MODIFIED_PRESS);
    press(BUTTON_CENTER, 7700);
    release_expect(BUTTON_CENTER, 7750, true, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    keys[BUTTON_CENTER] = keys[BUTTON_PLUS] = true;
    assert(!sample(100));
    assert(!sample(120)); /* Same-sample stable presses form a chord. */
    release_expect(BUTTON_PLUS, 2200, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 2300, false, BUTTON_EVENT_LONG_PRESS);

    start(0);
    press(BUTTON_CENTER, 100);
    press(BUTTON_MINUS, 200);
    keys[BUTTON_PLUS] = true;
    assert(!sample(300));
    assert(!sample(320));
    release_expect(BUTTON_MINUS, 400, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_PLUS, 500, false, BUTTON_EVENT_MODIFIED_PRESS);
    press(BUTTON_PLUS, 600);
    release_expect(BUTTON_PLUS, 650, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 6500, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_PLUS, 6600);
    release_expect(BUTTON_PLUS, 6650, true, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    keys[BUTTON_MINUS] = keys[BUTTON_CENTER] = keys[BUTTON_PLUS] = true;
    assert(!sample(100));
    assert(!sample(120));
    release_expect(BUTTON_MINUS, 200, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_PLUS, 300, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 400, false, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    keys[BUTTON_MINUS] = true;
    assert(!sample(100));
    keys[BUTTON_MINUS] = false;
    assert(!sample(110));
    assert(!sample(130)); /* Press bounce below 20 ms. */
    press(BUTTON_MINUS, 200);
    keys[BUTTON_MINUS] = false;
    assert(!sample(230));
    keys[BUTTON_MINUS] = true;
    assert(!sample(240)); /* Release bounce must not emit. */
    assert(!sample(260));
    release_expect(BUTTON_MINUS, 280, true, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    press(BUTTON_CENTER, 100);
    keys[BUTTON_PLUS] = true;
    assert(!sample(2100)); /* Even a short overlapping raw edge cancels center. */
    keys[BUTTON_PLUS] = false;
    assert(!sample(2110));
    release_expect(BUTTON_CENTER, 6200, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_PLUS, 6400);
    release_expect(BUTTON_PLUS, 6450, true, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    keys[BUTTON_CENTER] = true;
    assert(!sample(100));
    keys[BUTTON_MINUS] = true;
    assert(!sample(110)); /* Raw overlap before center press debounces. */
    keys[BUTTON_MINUS] = false;
    assert(!sample(115));
    assert(!sample(119));
    assert(!sample(120)); /* Center becomes stable; suppression must survive. */
    assert(!sample(6100));
    release_expect(BUTTON_CENTER, 6200, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_CENTER, 6300);
    release_expect(BUTTON_CENTER, 6350, true, BUTTON_EVENT_SHORT_PRESS);

    start(0);
    keys[BUTTON_PLUS] = true;
    assert(!sample(100));
    keys[BUTTON_CENTER] = true;
    assert(!sample(110)); /* Raw overlap before side press debounces. */
    keys[BUTTON_CENTER] = false;
    assert(!sample(115));
    assert(!sample(119));
    assert(!sample(120)); /* Side becomes stable; no modified chord was valid. */
    assert(!sample(1100));
    release_expect(BUTTON_PLUS, 1200, false, BUTTON_EVENT_LONG_PRESS);
    press(BUTTON_PLUS, 1300);
    release_expect(BUTTON_PLUS, 2300, true, BUTTON_EVENT_LONG_PRESS);

    start(0);
    keys[BUTTON_MINUS] = keys[BUTTON_PLUS] = true;
    assert(!sample(100));
    assert(!sample(120));
    release_expect(BUTTON_MINUS, 1200, false, BUTTON_EVENT_LONG_PRESS);
    release_expect(BUTTON_PLUS, 1300, false, BUTTON_EVENT_LONG_PRESS);

    start(0);
    keys[BUTTON_MINUS] = true;
    assert(!sample(100));
    keys[BUTTON_PLUS] = true;
    assert(!sample(110)); /* Adjacent polls during press debounce. */
    assert(!sample(130));
    release_expect(BUTTON_MINUS, 2100, false, BUTTON_EVENT_LONG_PRESS);
    release_expect(BUTTON_PLUS, 2200, false, BUTTON_EVENT_LONG_PRESS);

    keys[BUTTON_CENTER] = true;
    init_keys(0);
    assert(!sample(6000));
    release_expect(BUTTON_CENTER, 7000, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_CENTER, 7200);
    release_expect(BUTTON_CENTER, 7250, true, BUTTON_EVENT_SHORT_PRESS);

    start(UINT32_MAX - 200);
    press(BUTTON_MINUS, UINT32_MAX - 100);
    release_expect(BUTTON_MINUS, 950, true, BUTTON_EVENT_LONG_PRESS);
    press(BUTTON_CENTER, 1100);
    release_expect(BUTTON_CENTER, 1150, true, BUTTON_EVENT_SHORT_PRESS);
    start(UINT32_MAX - 200);
    press(BUTTON_CENTER, UINT32_MAX - 100);
    press(BUTTON_PLUS, UINT32_MAX - 50);
    release_expect(BUTTON_CENTER, 100, false, BUTTON_EVENT_SHORT_PRESS);
    release_expect(BUTTON_PLUS, 1050, true, BUTTON_EVENT_MODIFIED_PRESS);

    keys[BUTTON_CENTER] = keys[BUTTON_MINUS] = true;
    init_keys(0);
    assert(!sample(7000));
    release_expect(BUTTON_MINUS, 7100, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 7200, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_CENTER, 7300);
    press(BUTTON_PLUS, 7400);
    release_expect(BUTTON_PLUS, 7450, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 7500, false, BUTTON_EVENT_SHORT_PRESS);
}

/* ---- While-held hold events, sampled like the driver (every poll). ---- */
#define MAX_EVENTS 16
static button_id_t ev_id[MAX_EVENTS];
static button_event_t ev_kind[MAX_EVENTS];
static uint32_t ev_at[MAX_EVENTS];
static unsigned ev_n;

static void drain(uint32_t at)
{
    button_id_t id;
    button_event_t event;
    while (button_state_sample(&state, at, keys, &id, &event)) {
        assert(ev_n < MAX_EVENTS);
        ev_id[ev_n] = id;
        ev_kind[ev_n] = event;
        ev_at[ev_n++] = at;
    }
}

static void hold_start(uint32_t at)
{
    for (int i = 0; i < BUTTON_COUNT; ++i) keys[i] = false;
    button_state_init(&state, at, keys);
    button_state_enable_hold_events(&state, true);
    ev_n = 0;
    drain(at + 20);
    assert(ev_n == 0);
}

/* Press `id` at `at`, poll every `step` ms, release at at + duration, settle. */
static uint32_t hold_press(button_id_t id, uint32_t at, uint32_t duration, uint32_t step)
{
    keys[id] = true;
    uint32_t t = at;
    for (; t < at + duration; t += step) drain(t);
    keys[id] = false;
    for (t = at + duration; t <= at + duration + 60; t += step) drain(t);
    return t;
}

static void expect(unsigned i, button_id_t id, button_event_t event)
{
    assert(i < ev_n);
    assert(ev_id[i] == id);
    assert(ev_kind[i] == event);
}

static void hold_scenarios(void)
{
    /* Center: HOLD_LONG once at 2 s, HOLD_EXTRA_LONG once at 6 s, then release. */
    hold_start(0);
    hold_press(BUTTON_CENTER, 100, 7000, BUTTON_POLL_MS);
    assert(ev_n == 3);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG);
    assert(ev_at[0] == 100 + BUTTON_MESH_GESTURE_MS);
    expect(1, BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG);
    assert(ev_at[1] == 100 + BUTTON_PAIRING_GESTURE_MS);
    expect(2, BUTTON_CENTER, BUTTON_EVENT_EXTRA_LONG_PRESS);

    /* 2..6 s: one HOLD_LONG, then LONG_PRESS. */
    hold_start(0);
    hold_press(BUTTON_CENTER, 100, 4000, BUTTON_POLL_MS);
    assert(ev_n == 2);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG);
    expect(1, BUTTON_CENTER, BUTTON_EVENT_LONG_PRESS);

    /* Short presses, including just under 2 s: no hold event, no sound. */
    hold_start(0);
    uint32_t t = hold_press(BUTTON_CENTER, 100, 300, BUTTON_POLL_MS);
    t = hold_press(BUTTON_CENTER, t + 100, 1990, BUTTON_POLL_MS);
    assert(ev_n == 2);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_SHORT_PRESS);
    expect(1, BUTTON_CENTER, BUTTON_EVENT_SHORT_PRESS);
    /* Raw release 5 ms before the threshold, debounce still pending at 2 s:
     * the hold event must not fire for what releases as a short press. */
    hold_start(0);
    hold_press(BUTTON_CENTER, 100, 1995, 5);
    assert(ev_n == 1);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_SHORT_PRESS);

    /* Each new press emits its own hold events again. */
    hold_start(0);
    t = hold_press(BUTTON_CENTER, 100, 2500, BUTTON_POLL_MS);
    hold_press(BUTTON_CENTER, t + 100, 2500, BUTTON_POLL_MS);
    assert(ev_n == 4);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG);
    expect(1, BUTTON_CENTER, BUTTON_EVENT_LONG_PRESS);
    expect(2, BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG);
    expect(3, BUTTON_CENTER, BUTTON_EVENT_LONG_PRESS);

    /* Sides: HOLD_LONG once at 1 s, never an extra-long hold. */
    for (int side = BUTTON_MINUS; side < BUTTON_COUNT; side += 2) {
        hold_start(0);
        t = hold_press((button_id_t)side, 100, 7000, BUTTON_POLL_MS);
        assert(ev_n == 2);
        expect(0, (button_id_t)side, BUTTON_EVENT_HOLD_LONG);
        assert(ev_at[0] == 100 + BUTTON_SIDE_HOLD_MS);
        expect(1, (button_id_t)side, BUTTON_EVENT_LONG_PRESS);
        hold_press((button_id_t)side, t + 100, 990, BUTTON_POLL_MS);
        assert(ev_n == 3);
        expect(2, (button_id_t)side, BUTTON_EVENT_SHORT_PRESS);
    }

    /* No poll between threshold and release: the hold event is caught up
     * just before the release, so a long release is always previewed. */
    hold_start(0);
    keys[BUTTON_CENTER] = true;
    drain(100);
    drain(120);
    keys[BUTTON_CENTER] = false;
    drain(100 + 6005);
    drain(100 + 6025);
    assert(ev_n == 2);
    expect(0, BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG);
    expect(1, BUTTON_CENTER, BUTTON_EVENT_EXTRA_LONG_PRESS);
    hold_start(0);
    keys[BUTTON_PLUS] = true;
    drain(100);
    drain(1099);
    keys[BUTTON_PLUS] = false;
    drain(1100);
    drain(1120);
    assert(ev_n == 2);
    expect(0, BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG);
    expect(1, BUTTON_PLUS, BUTTON_EVENT_LONG_PRESS);

    /* Modified press (center + side): no hold events for either key. */
    hold_start(0);
    keys[BUTTON_CENTER] = true;
    for (t = 100; t < 300; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_MINUS] = true;
    for (; t < 8000; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_MINUS] = false;
    for (; t < 8100; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_CENTER] = false;
    for (; t < 8200; t += BUTTON_POLL_MS) drain(t);
    assert(ev_n == 1);
    expect(0, BUTTON_MINUS, BUTTON_EVENT_MODIFIED_PRESS);

    /* Both sides (chord): silent throughout. */
    hold_start(0);
    keys[BUTTON_MINUS] = keys[BUTTON_PLUS] = true;
    for (t = 100; t < 3000; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_MINUS] = keys[BUTTON_PLUS] = false;
    for (; t < 3100; t += BUTTON_POLL_MS) drain(t);
    assert(ev_n == 0);

    /* A side hold already previewed, then the center joins: the release is
     * the modified press (Bluetooth volume), unchanged by the earlier hold. */
    hold_start(0);
    keys[BUTTON_PLUS] = true;
    for (t = 100; t < 1500; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_CENTER] = true;
    for (; t < 1700; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_PLUS] = false;
    for (; t < 1800; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_CENTER] = false;
    for (; t < 1900; t += BUTTON_POLL_MS) drain(t);
    assert(ev_n == 2);
    expect(0, BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG);
    expect(1, BUTTON_PLUS, BUTTON_EVENT_MODIFIED_PRESS);

    /* Held at boot: gated until released, so no hold events either. */
    keys[BUTTON_CENTER] = true;
    keys[BUTTON_MINUS] = keys[BUTTON_PLUS] = false;
    button_state_init(&state, 0, keys);
    button_state_enable_hold_events(&state, true);
    ev_n = 0;
    for (t = 0; t < 7000; t += BUTTON_POLL_MS) drain(t);
    keys[BUTTON_CENTER] = false;
    for (; t < 7100; t += BUTTON_POLL_MS) drain(t);
    assert(ev_n == 0);

    /* Release classification with hold events on matches it with them off,
     * for every duration around the thresholds, at the driver's poll rate. */
    static const uint32_t durations[] = {
        30, 49, 50, 51, 999, 1000, 1001, 1999, 2000, 2001, 5999, 6000, 6001, 9000,
    };
    for (int id = 0; id < BUTTON_COUNT; ++id) {
        for (unsigned d = 0; d < sizeof(durations) / sizeof(durations[0]); ++d) {
            button_event_t release[2] = {BUTTON_EVENT_SHORT_PRESS, BUTTON_EVENT_SHORT_PRESS};
            unsigned releases[2] = {0, 0};
            unsigned holds = 0;
            for (int mode = 0; mode < 2; ++mode) {
                hold_start(0);
                button_state_enable_hold_events(&state, mode == 1);
                hold_press((button_id_t)id, 100, durations[d], BUTTON_POLL_MS);
                for (unsigned i = 0; i < ev_n; ++i) {
                    assert(ev_id[i] == (button_id_t)id);
                    if (is_hold(ev_kind[i])) {
                        assert(mode == 1);
                        assert(releases[mode] == 0); /* Holds precede the release. */
                        holds++;
                    } else {
                        release[mode] = ev_kind[i];
                        releases[mode]++;
                    }
                }
            }
            assert(releases[0] == releases[1] && release[0] == release[1]);
            unsigned expected_holds = id == BUTTON_CENTER
                ? (durations[d] >= BUTTON_PAIRING_GESTURE_MS ? 2u :
                   durations[d] >= BUTTON_MESH_GESTURE_MS ? 1u : 0u)
                : (durations[d] >= BUTTON_SIDE_HOLD_MS ? 1u : 0u);
            assert(holds == expected_holds);
        }
    }
}

int main(void)
{
    assert(button_classify_release_ms(49) == BUTTON_GESTURE_NONE);
    assert(button_classify_release_ms(50) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(1999) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(2000) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(5999) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(6000) == BUTTON_GESTURE_BLUETOOTH_PAIRING);

    g_holds = false;
    release_scenarios();
    assert(g_skipped_holds == 0);
    g_holds = true;
    release_scenarios(); /* Same assertions with hold events interleaved. */
    assert(g_skipped_holds > 0);
    hold_scenarios();
    return 0;
}
