#ifndef OMI_BUTTON_CONTROL_H
#define OMI_BUTTON_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "button_gesture.h"
#include "channel_control.h"

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

static inline omi_volume_target_t omi_call_volume_target(omi_action_t action, bool call_active)
{
    omi_volume_target_t target = omi_volume_action_target(action);
    return call_active && target == OMI_VOLUME_MESH ? OMI_VOLUME_BLUETOOTH : target;
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

/* Action a release right now would run, for a while-held HOLD_* event. Uses
 * the release mapping, so preview and release cannot disagree on the gesture. */
static inline omi_action_t omi_button_hold_action(button_id_t id, button_event_t event)
{
    if (event == BUTTON_EVENT_HOLD_LONG) return omi_button_action(id, BUTTON_EVENT_LONG_PRESS);
    if (event == BUTTON_EVENT_HOLD_EXTRA_LONG && id == BUTTON_CENTER)
        return omi_button_action(id, BUTTON_EVENT_EXTRA_LONG_PRESS);
    return OMI_ACTION_NONE;
}

typedef enum {
    OMI_PREVIEW_NONE,
    OMI_PREVIEW_MESH_ON,
    OMI_PREVIEW_MESH_OFF,
    OMI_PREVIEW_PHONE_PAIRING,
    OMI_PREVIEW_CHANNEL,
} omi_preview_kind_t;

typedef struct {
    omi_preview_kind_t kind;
    uint8_t channel; /* OMI_PREVIEW_CHANNEL: the channel release would select. */
} omi_preview_t;

/* What releasing now would announce. `action` is already resolved through
 * omi_side_hold_action; mesh_on is the current mesh intent and channel the
 * selected talk channel. Track control, volume and media stay silent. */
static inline omi_preview_t omi_action_preview(omi_action_t action, bool mesh_on, uint8_t channel)
{
    omi_preview_t preview = {OMI_PREVIEW_NONE, 0u};
    if (action == OMI_ACTION_MESH_TOGGLE) {
        preview.kind = mesh_on ? OMI_PREVIEW_MESH_OFF : OMI_PREVIEW_MESH_ON;
    } else if (action == OMI_ACTION_PAIRING) {
        preview.kind = OMI_PREVIEW_PHONE_PAIRING;
    } else if (action == OMI_ACTION_CHANNEL_PREVIOUS || action == OMI_ACTION_CHANNEL_NEXT) {
        uint8_t target = mesh_channel_cycle(channel, action == OMI_ACTION_CHANNEL_NEXT ? 1 : -1);
        if (mesh_channel_valid(target)) {
            preview.kind = OMI_PREVIEW_CHANNEL;
            preview.channel = target;
        }
    }
    return preview;
}

#endif
