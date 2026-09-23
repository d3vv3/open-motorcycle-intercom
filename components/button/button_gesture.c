/**
 * @file button_gesture.c
 * @brief Pure BOOT-button release gesture classification.
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
