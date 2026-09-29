/**
 * @file button_gesture.c
 * @brief Pure button release classification and debounced state engine.
 */

#include "button_gesture.h"

button_gesture_t button_classify_release_ms(int64_t duration_ms)
{
    if (duration_ms >= BUTTON_PAIRING_GESTURE_MS) {
        return BUTTON_GESTURE_BLUETOOTH_PAIRING;
    }
    if (duration_ms >= BUTTON_MESH_GESTURE_MS) {
        return BUTTON_GESTURE_MESH_TOGGLE;
    }
    if (duration_ms >= BUTTON_SHORT_PRESS_MIN_MS) {
        return BUTTON_GESTURE_SHORT_PRESS;
    }
    return BUTTON_GESTURE_NONE;
}

void button_state_init(button_state_t *state, uint32_t now_ms,
                       const bool pressed[BUTTON_COUNT])
{
    *state = (button_state_t){0};
    for (int i = 0; i < BUTTON_COUNT; ++i) {
        state->keys[i].raw_pressed = pressed[i];
        state->keys[i].stable_pressed = pressed[i];
        state->keys[i].raw_since_ms = now_ms;
    }
}

bool button_state_sample(button_state_t *state, uint32_t now_ms,
                          const bool pressed[BUTTON_COUNT],
                          button_id_t *id, button_event_t *event)
{
    unsigned raw_count = 0;
    bool any_stable = false;
    bool released[BUTTON_COUNT] = {false};
    bool active[BUTTON_COUNT];

    for (int i = 0; i < BUTTON_COUNT; ++i) {
        active[i] = pressed[i] || state->keys[i].raw_pressed ||
                    state->keys[i].stable_pressed;
        raw_count += pressed[i] ? 1u : 0u;
        /* Start a new gesture only after this key was fully released and idle.
         * Clearing on stable press would erase raw-overlap suppression. */
        if (pressed[i] && !state->keys[i].raw_pressed &&
            !state->keys[i].stable_pressed &&
            (uint32_t)(now_ms - state->keys[i].raw_since_ms) >= BUTTON_DEBOUNCE_MS) {
            if (i == BUTTON_CENTER) state->center_used = false;
            else {
                state->side_modified[i] = false;
                state->side_suppressed[i] = false;
            }
        }
    }

    /* Only concurrently down sides invalidate the chord. A side waiting for
     * release debounce must not block an alternating tap of the other side. */
    if (state->ready && pressed[BUTTON_MINUS] && pressed[BUTTON_PLUS]) {
        state->chord = true;
    }
    if (state->ready && active[BUTTON_CENTER]) {
        if (active[BUTTON_MINUS] || active[BUTTON_PLUS]) {
            state->center_used = true;
        }
        for (int i = BUTTON_MINUS; i < BUTTON_COUNT; i += 2) {
            if (active[i]) state->side_suppressed[i] = true;
        }
    }

    for (int i = 0; i < BUTTON_COUNT; ++i) {
        button_key_state_t *key = &state->keys[i];
        if (pressed[i] != key->raw_pressed) {
            key->raw_pressed = pressed[i];
            key->raw_since_ms = now_ms;
        }
        if (key->raw_pressed != key->stable_pressed &&
            (uint32_t)(now_ms - key->raw_since_ms) >= BUTTON_DEBOUNCE_MS) {
            key->stable_pressed = key->raw_pressed;
            if (key->stable_pressed) {
                key->press_since_ms = key->raw_since_ms;
            } else {
                released[i] = true;
            }
        }
        any_stable |= key->stable_pressed;
    }

    /* A chord is valid only after both keys have debounced as pressed.
     * The latch survives either release order and successive side presses. */
    if (state->ready && !state->chord && state->keys[BUTTON_CENTER].stable_pressed) {
        for (int i = BUTTON_MINUS; i < BUTTON_COUNT; i += 2) {
            if (state->keys[i].stable_pressed || released[i]) {
                state->side_modified[i] = true;
                state->center_used = true;
            }
        }
    }

    if (state->ready && !state->chord) {
        for (int i = 0; i < BUTTON_COUNT; ++i) {
            if (!released[i]) continue;
            button_key_state_t *key = &state->keys[i];
            uint32_t duration = key->raw_since_ms - key->press_since_ms;
            if (duration < BUTTON_SHORT_PRESS_MIN_MS ||
                (i == BUTTON_CENTER ? state->center_used :
                 state->side_suppressed[i] && !state->side_modified[i])) continue;
            button_event_t kind = i == BUTTON_CENTER
                ? (duration >= BUTTON_PAIRING_GESTURE_MS ? BUTTON_EVENT_EXTRA_LONG_PRESS :
                   duration >= BUTTON_MESH_GESTURE_MS ? BUTTON_EVENT_LONG_PRESS : BUTTON_EVENT_SHORT_PRESS)
                : (state->side_modified[i] ? BUTTON_EVENT_MODIFIED_PRESS :
                   duration >= BUTTON_SIDE_HOLD_MS ? BUTTON_EVENT_LONG_PRESS : BUTTON_EVENT_SHORT_PRESS);
            if (state->pending_count < BUTTON_COUNT) {
                unsigned n = state->pending_count++;
                state->pending_ids[n] = (button_id_t)i;
                state->pending_events[n] = kind;
            }
        }
    }

    if (raw_count == 0 && !any_stable) {
        state->ready = true;
        state->chord = false;
        state->center_used = false;
        for (int i = BUTTON_MINUS; i < BUTTON_COUNT; i += 2) {
            state->side_modified[i] = false;
            state->side_suppressed[i] = false;
        }
    }
    if (!state->pending_count) return false;
    *id = state->pending_ids[0];
    *event = state->pending_events[0];
    --state->pending_count;
    for (unsigned i = 0; i < state->pending_count; ++i) {
        state->pending_ids[i] = state->pending_ids[i + 1];
        state->pending_events[i] = state->pending_events[i + 1];
    }
    return true;
}
