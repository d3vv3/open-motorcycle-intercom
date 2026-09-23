#include <assert.h>

#include "button_gesture.h"

int main(void)
{
    assert(button_classify_release_ms(49) == BUTTON_GESTURE_NONE);
    assert(button_classify_release_ms(50) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(1999) == BUTTON_GESTURE_SHORT_PRESS);
    assert(button_classify_release_ms(2000) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(5999) == BUTTON_GESTURE_MESH_TOGGLE);
    assert(button_classify_release_ms(6000) == BUTTON_GESTURE_BLUETOOTH_PAIRING);
    return 0;
}
