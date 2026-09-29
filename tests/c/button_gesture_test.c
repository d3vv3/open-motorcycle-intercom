#include <assert.h>
#include <stdint.h>

#include "button_gesture.h"

static button_state_t state;
static bool keys[BUTTON_COUNT];
static button_id_t emitted_id;
static button_event_t emitted_event;

static void start(uint32_t at)
{
    for (int i = 0; i < BUTTON_COUNT; ++i) keys[i] = false;
    button_state_init(&state, at, keys);
    assert(!button_state_sample(&state, at + 20, keys, &emitted_id, &emitted_event));
}

static bool sample(uint32_t at)
{
    return button_state_sample(&state, at, keys, &emitted_id, &emitted_event);
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

int main(void)
{
    assert(button_classify_release_ms(49) == BUTTON_GESTURE_NONE);
    assert(button_classify_release_ms(50) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(1999) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(2000) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(5999) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(6000) == BUTTON_GESTURE_BLUETOOTH_PAIRING);

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
    button_state_init(&state, 0, keys);
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
    button_state_init(&state, 0, keys);
    assert(!sample(7000));
    release_expect(BUTTON_MINUS, 7100, false, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 7200, false, BUTTON_EVENT_EXTRA_LONG_PRESS);
    press(BUTTON_CENTER, 7300);
    press(BUTTON_PLUS, 7400);
    release_expect(BUTTON_PLUS, 7450, true, BUTTON_EVENT_MODIFIED_PRESS);
    release_expect(BUTTON_CENTER, 7500, false, BUTTON_EVENT_SHORT_PRESS);
    return 0;
}
