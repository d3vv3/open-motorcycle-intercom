#include "main/hold_preview.h"

#include <assert.h>
#include <stdio.h>

static omi_preview_t preview(omi_preview_kind_t kind, uint8_t channel)
{
    omi_preview_t p = {kind, channel};
    return p;
}

static const omi_preview_t none = {OMI_PREVIEW_NONE, 0u};

/* Hold event -> action -> preview, the way main.c chains them. */
static omi_preview_t hold_preview(button_id_t id, button_event_t event, bool media_known,
                                  bool streaming, bool call_idle, bool mesh_on, uint8_t channel)
{
    omi_action_t action = omi_side_hold_action(omi_button_hold_action(id, event),
                                               media_known, streaming, call_idle);
    return omi_action_preview(action, mesh_on, channel);
}

static void expect_prompts(omi_radio_plan_t plan, uint8_t count, omi_radio_prompt_t a,
                           omi_radio_prompt_t b)
{
    assert(plan.prompt_count == count);
    if (count > 0) assert(plan.prompts[0] == a);
    if (count > 1) assert(plan.prompts[1] == b);
}

static void test_hold_action_mapping(void)
{
    assert(omi_button_hold_action(BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_MESH_TOGGLE);
    assert(omi_button_hold_action(BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG) == OMI_ACTION_PAIRING);
    assert(omi_button_hold_action(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_CHANNEL_PREVIOUS);
    assert(omi_button_hold_action(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_CHANNEL_NEXT);
    assert(omi_button_hold_action(BUTTON_PLUS, BUTTON_EVENT_HOLD_EXTRA_LONG) == OMI_ACTION_NONE);
    assert(omi_button_hold_action(BUTTON_CENTER, BUTTON_EVENT_LONG_PRESS) == OMI_ACTION_NONE);
    assert(omi_button_hold_action(BUTTON_MINUS, BUTTON_EVENT_SHORT_PRESS) == OMI_ACTION_NONE);
    assert(omi_button_hold_action(BUTTON_MINUS, BUTTON_EVENT_MODIFIED_PRESS) == OMI_ACTION_NONE);
    /* Hold events never run an action themselves. */
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_NONE);
    assert(omi_button_action(BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG) == OMI_ACTION_NONE);
    assert(omi_button_action(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_NONE);
    assert(omi_button_action(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG) == OMI_ACTION_NONE);
}

static void test_center_previews(void)
{
    omi_preview_t p = hold_preview(BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 1);
    assert(p.kind == OMI_PREVIEW_MESH_ON);
    p = hold_preview(BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG, true, false, true, true, 1);
    assert(p.kind == OMI_PREVIEW_MESH_OFF);
    /* Center previews do not depend on media or call state. */
    p = hold_preview(BUTTON_CENTER, BUTTON_EVENT_HOLD_LONG, false, true, false, true, 2);
    assert(p.kind == OMI_PREVIEW_MESH_OFF);
    p = hold_preview(BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG, true, false, true, true, 1);
    assert(p.kind == OMI_PREVIEW_PHONE_PAIRING);
    p = hold_preview(BUTTON_CENTER, BUTTON_EVENT_HOLD_EXTRA_LONG, true, false, true, false, 1);
    assert(p.kind == OMI_PREVIEW_PHONE_PAIRING);
}

static void test_side_previews(void)
{
    /* Same wrap as the release path (mesh_channel_cycle). */
    for (uint8_t channel = 1; channel <= 3; ++channel) {
        omi_preview_t next = hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, channel);
        omi_preview_t prev = hold_preview(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, channel);
        assert(next.kind == OMI_PREVIEW_CHANNEL && next.channel == mesh_channel_cycle(channel, 1));
        assert(prev.kind == OMI_PREVIEW_CHANNEL && prev.channel == mesh_channel_cycle(channel, -1));
    }
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 3).channel == 1);
    assert(hold_preview(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 1).channel == 3);
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 1).channel == 2);
    assert(hold_preview(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 3).channel == 2);
    /* Streaming with no call: the hold is track control, so stay silent. */
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, true, true, false, 1).kind == OMI_PREVIEW_NONE);
    assert(hold_preview(BUTTON_MINUS, BUTTON_EVENT_HOLD_LONG, true, true, true, false, 1).kind == OMI_PREVIEW_NONE);
    /* Streaming during a call keeps the channel change, so it is previewed. */
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, true, false, false, 1).kind == OMI_PREVIEW_CHANNEL);
    /* Media state unknown: the hold is suppressed, so is the preview. */
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, false, false, true, false, 1).kind == OMI_PREVIEW_NONE);
    /* An invalid selected channel would not be selected either. */
    assert(hold_preview(BUTTON_PLUS, BUTTON_EVENT_HOLD_LONG, true, false, true, false, 0).kind == OMI_PREVIEW_NONE);
    /* Volume, modified presses and media never preview. */
    assert(omi_action_preview(OMI_ACTION_VOLUME_UP, false, 1).kind == OMI_PREVIEW_NONE);
    assert(omi_action_preview(OMI_ACTION_BLUETOOTH_VOLUME_DOWN, false, 1).kind == OMI_PREVIEW_NONE);
    assert(omi_action_preview(OMI_ACTION_NEXT_TRACK, false, 1).kind == OMI_PREVIEW_NONE);
    assert(omi_action_preview(OMI_ACTION_CALL_MEDIA, false, 1).kind == OMI_PREVIEW_NONE);
}

static void test_memo(void)
{
    omi_preview_memo_t memo = {0};
    /* No hold, nothing to skip. */
    assert(omi_preview_memo_release(&memo, BUTTON_CENTER).kind == OMI_PREVIEW_NONE);
    /* 2 s then 6 s preview: the release takes the latest one, once. */
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_MESH_OFF, 0), true);
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_PHONE_PAIRING, 0), true);
    assert(omi_preview_memo_release(&memo, BUTTON_CENTER).kind == OMI_PREVIEW_PHONE_PAIRING);
    assert(omi_preview_memo_release(&memo, BUTTON_CENTER).kind == OMI_PREVIEW_NONE);
    /* A preview that could not be queued is not skipped at release. */
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_MESH_ON, 0), true);
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_PHONE_PAIRING, 0), false);
    assert(omi_preview_memo_release(&memo, BUTTON_CENTER).kind == OMI_PREVIEW_NONE);
    /* A silent preview (track control) clears a stale one. */
    omi_preview_memo_hold(&memo, BUTTON_PLUS, preview(OMI_PREVIEW_CHANNEL, 2), true);
    omi_preview_memo_hold(&memo, BUTTON_PLUS, none, false);
    assert(omi_preview_memo_release(&memo, BUTTON_PLUS).kind == OMI_PREVIEW_NONE);
    /* Per button: a side release does not consume the center preview. */
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_MESH_ON, 0), true);
    omi_preview_memo_hold(&memo, BUTTON_MINUS, preview(OMI_PREVIEW_CHANNEL, 3), true);
    assert(omi_preview_memo_release(&memo, BUTTON_PLUS).kind == OMI_PREVIEW_NONE);
    omi_preview_t side = omi_preview_memo_release(&memo, BUTTON_MINUS);
    assert(side.kind == OMI_PREVIEW_CHANNEL && side.channel == 3);
    assert(omi_preview_memo_release(&memo, BUTTON_CENTER).kind == OMI_PREVIEW_MESH_ON);
    assert(omi_preview_memo_release(&memo, BUTTON_COUNT).kind == OMI_PREVIEW_NONE);
}

static void test_release_prompts(void)
{
    /* ESP-NOW mesh on: the preview said Mesh On; only Bluetooth off follows. */
    omi_radio_plan_t on = omi_radio_mesh_on_result(true, true, false);
    expect_prompts(omi_radio_plan_after_preview(on, preview(OMI_PREVIEW_MESH_ON, 0)), 1,
                   OMI_RADIO_PROMPT_BLUETOOTH_OFF, 0);
    omi_radio_plan_t on_toggle = omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, true, false, false);
    omi_radio_plan_t on_left = omi_radio_plan_after_preview(on_toggle, preview(OMI_PREVIEW_MESH_ON, 0));
    expect_prompts(on_left, 1, OMI_RADIO_PROMPT_BLUETOOTH_OFF, 0);
    assert(on_left.bluetooth_off); /* Actions untouched. */
    /* ESP-NOW mesh off: only Bluetooth on follows. */
    omi_radio_plan_t off = omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, true, true, true);
    omi_radio_plan_t off_left = omi_radio_plan_after_preview(off, preview(OMI_PREVIEW_MESH_OFF, 0));
    expect_prompts(off_left, 1, OMI_RADIO_PROMPT_BLUETOOTH_ON, 0);
    assert(off_left.bluetooth_on && off_left.reconnect);
    /* ESP-NOW pairing with mesh on: Phone Pairing was the preview, Mesh Off
     * (what the release does) still plays. */
    omi_radio_plan_t pair = omi_radio_plan(OMI_RADIO_EVENT_PAIRING, true, true, false);
    omi_radio_plan_t pair_left = omi_radio_plan_after_preview(pair, preview(OMI_PREVIEW_PHONE_PAIRING, 0));
    expect_prompts(pair_left, 1, OMI_RADIO_PROMPT_MESH_OFF, 0);
    assert(pair_left.stop_mesh && pair_left.bluetooth_on && pair_left.open_pairing);
    /* ESP-NOW pairing with mesh off, and nRF: nothing follows. */
    expect_prompts(omi_radio_plan_after_preview(omi_radio_plan(OMI_RADIO_EVENT_PAIRING, true, false, false),
                                                preview(OMI_PREVIEW_PHONE_PAIRING, 0)), 0, 0, 0);
    expect_prompts(omi_radio_plan_after_preview(omi_radio_plan(OMI_RADIO_EVENT_PAIRING, false, true, false),
                                                preview(OMI_PREVIEW_PHONE_PAIRING, 0)), 0, 0, 0);
    expect_prompts(omi_radio_plan_after_preview(omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, false, false, false),
                                                preview(OMI_PREVIEW_MESH_ON, 0)), 0, 0, 0);
    expect_prompts(omi_radio_plan_after_preview(omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, false, true, false),
                                                preview(OMI_PREVIEW_MESH_OFF, 0)), 0, 0, 0);
    /* No preview (released between polls, or the queue was full): all prompts. */
    expect_prompts(omi_radio_plan_after_preview(pair, none), 2,
                   OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_PHONE_PAIRING);
    expect_prompts(omi_radio_plan_after_preview(off, none), 2,
                   OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_BLUETOOTH_ON);
    /* State changed after the preview: the release's own prompt is not eaten. */
    expect_prompts(omi_radio_plan_after_preview(off, preview(OMI_PREVIEW_MESH_ON, 0)), 2,
                   OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_BLUETOOTH_ON);
    expect_prompts(omi_radio_plan_after_preview(on_toggle, preview(OMI_PREVIEW_MESH_OFF, 0)), 2,
                   OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_BLUETOOTH_OFF);
    /* The 2 s Mesh Off preview does not stand in for pairing's Mesh Off:
     * only the latest preview (Phone Pairing) is skipped. */
    omi_preview_memo_t memo = {0};
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_MESH_OFF, 0), true);
    omi_preview_memo_hold(&memo, BUTTON_CENTER, preview(OMI_PREVIEW_PHONE_PAIRING, 0), true);
    expect_prompts(omi_radio_plan_after_preview(pair, omi_preview_memo_release(&memo, BUTTON_CENTER)), 1,
                   OMI_RADIO_PROMPT_MESH_OFF, 0);
    /* Failed mesh start rollback has no prompt either way. */
    expect_prompts(omi_radio_plan_after_preview(omi_radio_mesh_on_result(false, true, true),
                                                preview(OMI_PREVIEW_MESH_ON, 0)), 0, 0, 0);

    /* Channel: the previewed channel is not repeated; a different one plays. */
    assert(!omi_channel_prompt_after_preview(preview(OMI_PREVIEW_CHANNEL, 2), 2));
    assert(omi_channel_prompt_after_preview(preview(OMI_PREVIEW_CHANNEL, 2), 3));
    assert(omi_channel_prompt_after_preview(none, 2));
    assert(omi_channel_prompt_after_preview(preview(OMI_PREVIEW_MESH_ON, 0), 1));

    /* Mismatch detection for the log. */
    assert(!omi_preview_mismatch(none, preview(OMI_PREVIEW_MESH_ON, 0)));
    assert(!omi_preview_mismatch(preview(OMI_PREVIEW_MESH_ON, 0), preview(OMI_PREVIEW_MESH_ON, 0)));
    assert(omi_preview_mismatch(preview(OMI_PREVIEW_MESH_ON, 0), preview(OMI_PREVIEW_MESH_OFF, 0)));
    assert(!omi_preview_mismatch(preview(OMI_PREVIEW_CHANNEL, 2), preview(OMI_PREVIEW_CHANNEL, 2)));
    assert(omi_preview_mismatch(preview(OMI_PREVIEW_CHANNEL, 2), preview(OMI_PREVIEW_CHANNEL, 3)));
}

int main(void)
{
    test_hold_action_mapping();
    test_center_previews();
    test_side_previews();
    test_memo();
    test_release_prompts();
    puts("hold preview tests passed");
    return 0;
}
