/**
 * @file button_gesture.h
 * @brief Pure BOOT-button release gesture classification.
 */

#ifndef OMI_BUTTON_GESTURE_H
#define OMI_BUTTON_GESTURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUTTON_SHORT_PRESS_MIN_MS 50
#define BUTTON_MESH_GESTURE_MS 2000
#define BUTTON_PAIRING_GESTURE_MS 6000

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
