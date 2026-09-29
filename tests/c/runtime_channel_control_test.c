#include "main/button_control.h"
#include "main/channel_control.h"
#include "main/transport_nrf_reconcile.h"
#include "main/runtime_channel_control.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(mesh_channel_restore(0, 2) == 2);
    assert(mesh_channel_restore(4, 3) == 3);
    assert(mesh_channel_restore(2, 1) == 2);
    assert(mesh_channel_restore(0, 0) == 1);
    assert(mesh_channel_cycle(1, -1) == 3);
    assert(mesh_channel_cycle(3, 1) == 1);
    assert(mesh_channel_cycle(2, -1) == 1);
    assert(mesh_channel_cycle(2, 1) == 3);
    assert(mesh_channel_cycle(0, 1) == 0);
    assert(mesh_volume_step(0, -1) == 0);
    assert(mesh_volume_step(3, -1) == 0);
    assert(mesh_volume_step(60, 1) == 65);
    assert(mesh_volume_step(98, 1) == 100);
    assert(mesh_volume_step(100, 1) == 100);
    assert(mesh_volume_step(95, 1) == 100);
    assert(mesh_volume_step(5, -1) == 0);
    assert(mesh_volume_at_limit(95, mesh_volume_step(95, 1), 1));
    assert(mesh_volume_at_limit(5, mesh_volume_step(5, -1), -1));
    assert(mesh_volume_at_limit(100, mesh_volume_step(100, 1), 1));
    assert(mesh_volume_at_limit(0, mesh_volume_step(0, -1), -1));
    assert(!mesh_volume_at_limit(50, mesh_volume_step(50, 1), 1));
    assert(!mesh_volume_at_limit(50, mesh_volume_step(50, -1), -1));
    assert(!mesh_volume_at_limit(50, 50, 0));

    assert(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_SHORT_PRESS) == OMI_ACTION_VOLUME_DOWN);
    assert(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_SHORT_PRESS) == OMI_ACTION_VOLUME_UP);
    assert(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_LONG_PRESS) == OMI_ACTION_CHANNEL_PREVIOUS);
    assert(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_LONG_PRESS) == OMI_ACTION_CHANNEL_NEXT);
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_SHORT_PRESS) == OMI_ACTION_CALL_MEDIA);
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_LONG_PRESS) == OMI_ACTION_MESH_TOGGLE);
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_EXTRA_LONG_PRESS) == OMI_ACTION_PAIRING);
    assert(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_EXTRA_LONG_PRESS) == OMI_ACTION_NONE);
    assert(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_MODIFIED_PRESS) == OMI_ACTION_BLUETOOTH_VOLUME_DOWN);
    assert(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_MODIFIED_PRESS) == OMI_ACTION_BLUETOOTH_VOLUME_UP);
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_MODIFIED_PRESS) == OMI_ACTION_NONE);
    /* No AVRCP parameter: streaming selects track control even when the send cannot succeed. */
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_NEXT, true, true, true) == OMI_ACTION_NEXT_TRACK);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_PREVIOUS, true, true, true) == OMI_ACTION_PREVIOUS_TRACK);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_NEXT, true, false, true) == OMI_ACTION_CHANNEL_NEXT);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_PREVIOUS, true, false, true) == OMI_ACTION_CHANNEL_PREVIOUS);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_NEXT, true, true, false) == OMI_ACTION_CHANNEL_NEXT);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_PREVIOUS, true, true, false) == OMI_ACTION_CHANNEL_PREVIOUS);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_NEXT, false, false, false) == OMI_ACTION_NONE);
    assert(omi_side_hold_action(OMI_ACTION_CHANNEL_PREVIOUS, false, true, true) == OMI_ACTION_NONE);
    assert(omi_side_hold_action(OMI_ACTION_VOLUME_UP, false, true, true) == OMI_ACTION_VOLUME_UP);
    assert(omi_side_hold_action(OMI_ACTION_BLUETOOTH_VOLUME_DOWN, false, true, true) == OMI_ACTION_BLUETOOTH_VOLUME_DOWN);
    assert(omi_side_hold_action(OMI_ACTION_CALL_MEDIA, false, true, true) == OMI_ACTION_CALL_MEDIA);
    assert(omi_volume_action_target(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_SHORT_PRESS)) == OMI_VOLUME_MESH);
    assert(omi_volume_action_target(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_SHORT_PRESS)) == OMI_VOLUME_MESH);
    assert(omi_volume_action_target(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_MODIFIED_PRESS)) == OMI_VOLUME_BLUETOOTH);
    assert(omi_volume_action_target(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_MODIFIED_PRESS)) == OMI_VOLUME_BLUETOOTH);
    assert(omi_volume_action_target(OMI_ACTION_PAIRING) == OMI_VOLUME_NONE);
    assert(omi_volume_action_direction(OMI_ACTION_VOLUME_DOWN) == -1);
    assert(omi_volume_action_direction(OMI_ACTION_VOLUME_UP) == 1);
    assert(omi_volume_action_direction(OMI_ACTION_BLUETOOTH_VOLUME_DOWN) == -1);
    assert(omi_volume_action_direction(OMI_ACTION_BLUETOOTH_VOLUME_UP) == 1);
    assert(omi_volume_action_direction(OMI_ACTION_CHANNEL_NEXT) == 0);
    uint8_t levels[] = {[OMI_VOLUME_MESH] = 50, [OMI_VOLUME_BLUETOOTH] = 80};
    omi_action_t mesh_up = omi_button_action(BUTTON_PLUS, BUTTON_EVENT_SHORT_PRESS);
    levels[omi_volume_action_target(mesh_up)] =
        mesh_volume_step(levels[omi_volume_action_target(mesh_up)],
                         omi_volume_action_direction(mesh_up));
    assert(levels[OMI_VOLUME_MESH] == 55 && levels[OMI_VOLUME_BLUETOOTH] == 80);
    omi_action_t bt_down = omi_button_action(BUTTON_MINUS, BUTTON_EVENT_MODIFIED_PRESS);
    levels[omi_volume_action_target(bt_down)] =
        mesh_volume_step(levels[omi_volume_action_target(bt_down)],
                         omi_volume_action_direction(bt_down));
    assert(levels[OMI_VOLUME_MESH] == 55 && levels[OMI_VOLUME_BLUETOOTH] == 75);
    assert(omi_role_announcement(false, false, false, true) == OMI_ROLE_QUIET);
    assert(omi_role_announcement(true, false, false, true) == OMI_ROLE_COORDINATOR);
    assert(omi_role_announcement(true, true, true, true) == OMI_ROLE_QUIET);
    assert(omi_role_announcement(true, true, true, false) == OMI_ROLE_PARTICIPANT);
    assert(omi_role_announcement(true, false, true, false) == OMI_ROLE_PARTICIPANT);

    assert(nrf_channel_reconcile_action(false, false, false, BRIDGE_MESH_STATE_IDLE) ==
           NRF_MESH_NO_COMMAND);
    assert(nrf_channel_reconcile_action(true, false, true, BRIDGE_MESH_STATE_IDLE) == NRF_MESH_STOP);
    assert(nrf_channel_reconcile_action(true, false, false, BRIDGE_MESH_STATE_IDLE) == NRF_MESH_START);
    assert(nrf_channel_reconcile_action(true, false, false, BRIDGE_MESH_STATE_ACTIVE) == NRF_MESH_STOP);
    assert(!nrf_channel_start_confirmed(true, 1, 3, true));
    assert(!nrf_channel_start_confirmed(false, 3, 3, true));
    assert(!nrf_channel_start_confirmed(true, 3, 3, false));
    assert(nrf_channel_start_confirmed(true, 3, 3, true));

    /* A matched START ACK is not a ready session until a *later* ACTIVE status. */
    assert(!omi_nrf_session_ready(true, true, true, BRIDGE_MESH_STATE_ACTIVE, 1,
                                  BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_CODEC_LC3,
                                  MESH_AUDIO_V2_FRAME_MS, 17, 1000, 17, 1100, true));
    assert(!omi_nrf_session_ready(true, true, true, BRIDGE_MESH_STATE_SCANNING, 1,
                                  BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_CODEC_LC3,
                                  MESH_AUDIO_V2_FRAME_MS, 18, 1200, 17, 1100, true));
    assert(omi_nrf_session_ready(true, true, true, BRIDGE_MESH_STATE_ACTIVE, 1,
                                 BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_CODEC_LC3,
                                 MESH_AUDIO_V2_FRAME_MS, 19, 1300, 17, 1100, true));
    assert(!omi_nrf_session_ready(false, true, true, BRIDGE_MESH_STATE_ACTIVE, 1,
                                  BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_CODEC_LC3,
                                  MESH_AUDIO_V2_FRAME_MS, 19, 1300, 17, 1100, true));
    assert(!omi_nrf_session_ready(true, true, true, BRIDGE_MESH_STATE_ACTIVE, 1,
                                  BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_CODEC_LC3,
                                  MESH_AUDIO_V2_FRAME_MS, 19, 1300, 17, 1400, true));
    assert(omi_status_after_start(0, 1300, UINT32_MAX, 1100, true));
    assert(!omi_status_after_start(UINT32_MAX, 1300, 0, 1100, true));
    assert(!omi_status_after_start(17, 1300, 17, 1100, true));
    assert(!omi_status_after_start(18, 1100, 17, 1100, true));

    /* LEAVE can fail after full cleanup; an IDLE state after any timeout
     * does not establish task/callback quiescence. Late completion + retry does. */
    assert(omi_esp_channel_can_apply(true, true));
    assert(!omi_esp_channel_can_apply(true, false));
    assert(!omi_esp_channel_can_apply(false, false));
    bool quiesced_after_timeout = false;
    assert(!omi_esp_channel_can_apply(true, quiesced_after_timeout));
    quiesced_after_timeout = true;
    assert(omi_esp_channel_can_apply(true, quiesced_after_timeout));
    puts("runtime channel control tests passed");
    return 0;
}
