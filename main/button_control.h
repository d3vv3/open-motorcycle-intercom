#ifndef OMI_BUTTON_CONTROL_H
#define OMI_BUTTON_CONTROL_H

#include "button_gesture.h"

typedef enum {
    OMI_ACTION_NONE,
    OMI_ACTION_VOLUME_DOWN,
    OMI_ACTION_VOLUME_UP,
    OMI_ACTION_BLUETOOTH_VOLUME_DOWN,
    OMI_ACTION_BLUETOOTH_VOLUME_UP,
    OMI_ACTION_CHANNEL_PREVIOUS,
    OMI_ACTION_CHANNEL_NEXT,
    OMI_ACTION_PREVIOUS_TRACK,
    OMI_ACTION_NEXT_TRACK,
    OMI_ACTION_CALL_MEDIA,
    OMI_ACTION_MESH_TOGGLE,
    OMI_ACTION_PAIRING,
} omi_action_t;

typedef enum {
    OMI_VOLUME_NONE,
    OMI_VOLUME_MESH,
    OMI_VOLUME_BLUETOOTH,
} omi_volume_target_t;

static inline omi_volume_target_t omi_volume_action_target(omi_action_t action)
{
    if (action == OMI_ACTION_VOLUME_DOWN || action == OMI_ACTION_VOLUME_UP)
        return OMI_VOLUME_MESH;
    if (action == OMI_ACTION_BLUETOOTH_VOLUME_DOWN ||
        action == OMI_ACTION_BLUETOOTH_VOLUME_UP)
        return OMI_VOLUME_BLUETOOTH;
    return OMI_VOLUME_NONE;
}

static inline int omi_volume_action_direction(omi_action_t action)
{
    if (action == OMI_ACTION_VOLUME_DOWN || action == OMI_ACTION_BLUETOOTH_VOLUME_DOWN)
        return -1;
    if (action == OMI_ACTION_VOLUME_UP || action == OMI_ACTION_BLUETOOTH_VOLUME_UP)
        return 1;
    return 0;
}

static inline omi_action_t omi_button_action(button_id_t id, button_event_t event)
{
    if (id == BUTTON_CENTER) {
        if (event == BUTTON_EVENT_SHORT_PRESS) return OMI_ACTION_CALL_MEDIA;
        if (event == BUTTON_EVENT_LONG_PRESS) return OMI_ACTION_MESH_TOGGLE;
        if (event == BUTTON_EVENT_EXTRA_LONG_PRESS) return OMI_ACTION_PAIRING;
    } else if (id == BUTTON_MINUS || id == BUTTON_PLUS) {
        if (event == BUTTON_EVENT_MODIFIED_PRESS)
            return id == BUTTON_MINUS ? OMI_ACTION_BLUETOOTH_VOLUME_DOWN :
                                        OMI_ACTION_BLUETOOTH_VOLUME_UP;
        if (event == BUTTON_EVENT_SHORT_PRESS)
            return id == BUTTON_MINUS ? OMI_ACTION_VOLUME_DOWN : OMI_ACTION_VOLUME_UP;
        if (event == BUTTON_EVENT_LONG_PRESS)
            return id == BUTTON_MINUS ? OMI_ACTION_CHANNEL_PREVIOUS : OMI_ACTION_CHANNEL_NEXT;
    }
    return OMI_ACTION_NONE;
}

static inline omi_action_t omi_side_hold_action(omi_action_t action, bool media_state_known,
                                                 bool media_streaming, bool call_idle)
{
    if (action != OMI_ACTION_CHANNEL_PREVIOUS && action != OMI_ACTION_CHANNEL_NEXT)
        return action;
    if (!media_state_known) return OMI_ACTION_NONE;
    if (media_streaming && call_idle)
        return action == OMI_ACTION_CHANNEL_NEXT ? OMI_ACTION_NEXT_TRACK : OMI_ACTION_PREVIOUS_TRACK;
    return action;
}

#endif
