/**
 * @file button_gesture.h
 * @brief Pure button release classification and debounced three-button state.
 */

#ifndef OMI_BUTTON_GESTURE_H
#define OMI_BUTTON_GESTURE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUTTON_SHORT_PRESS_MIN_MS 50
#define BUTTON_MESH_GESTURE_MS 2000
#define BUTTON_PAIRING_GESTURE_MS 6000
#define BUTTON_SIDE_HOLD_MS 1000
#define BUTTON_DEBOUNCE_MS 20
#define BUTTON_POLL_MS 10

typedef enum {
    BUTTON_MINUS = 0,
    BUTTON_CENTER,
    BUTTON_PLUS,
    BUTTON_COUNT
} button_id_t;

typedef enum {
    BUTTON_EVENT_SHORT_PRESS = 0,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_EXTRA_LONG_PRESS,
    BUTTON_EVENT_MODIFIED_PRESS
} button_event_t;

/* Release-only events: center 50/2000/6000 ms; sides 50/1000 ms.
 * Sides never emit EXTRA_LONG_PRESS, including holds of 6000 ms or more.
 * A debounced center + one-side overlap emits MODIFIED_PRESS for the side
 * on release (once, for any hold >= 50 ms), and consumes the center press. */

/* Short aliases for consumers using the event rather than gesture terminology. */
#define BUTTON_EVENT_SHORT BUTTON_EVENT_SHORT_PRESS
#define BUTTON_EVENT_LONG BUTTON_EVENT_LONG_PRESS
#define BUTTON_EVENT_EXTRA_LONG BUTTON_EVENT_EXTRA_LONG_PRESS

typedef struct {
    bool raw_pressed;
    bool stable_pressed;
    uint32_t raw_since_ms;
    uint32_t press_since_ms;
} button_key_state_t;

typedef struct {
    button_key_state_t keys[BUTTON_COUNT];
    bool ready;
    bool chord; /* Both sides active: ignore everything until all are released. */
    bool center_used;
    bool side_modified[BUTTON_COUNT];
    bool side_suppressed[BUTTON_COUNT];
    button_id_t pending_ids[BUTTON_COUNT];
    button_event_t pending_events[BUTTON_COUNT];
    unsigned pending_count;
} button_state_t;

/** Start with all gestures gated until all three keys have been stably released. */
void button_state_init(button_state_t *state, uint32_t now_ms,
                       const bool pressed[BUTTON_COUNT]);

/** Sample all keys together. Returns one event at a time; call again with the
 * same sample to drain any additional release events before the next poll. */
bool button_state_sample(button_state_t *state, uint32_t now_ms,
                         const bool pressed[BUTTON_COUNT],
                         button_id_t *id, button_event_t *event);

typedef enum {
    BUTTON_GESTURE_NONE = 0,
    BUTTON_GESTURE_SHORT_PRESS,
    BUTTON_GESTURE_MESH_TOGGLE,
    BUTTON_GESTURE_BLUETOOTH_PAIRING,
} button_gesture_t;

/** Classify one stable button release. Exactly one gesture is returned. */
button_gesture_t button_classify_release_ms(int64_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* OMI_BUTTON_GESTURE_H */
