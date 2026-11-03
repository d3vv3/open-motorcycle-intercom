#ifndef OMI_HOLD_PREVIEW_H
#define OMI_HOLD_PREVIEW_H

#include <stdbool.h>
#include <stdint.h>

#include "button_control.h"
#include "radio_policy.h"

/* Hold previews: while a key is held past a threshold the device says what
 * releasing now would do. The release then runs the action without repeating
 * the prompt the preview already said; every other prompt (follow-ups such as
 * "Bluetooth off", or "Mesh Off" before pairing) still plays, in plan order.
 *
 * The memo keeps, per button, the latest preview that actually played during
 * the current press. The button engine emits the matching hold event before
 * every LONG/EXTRA_LONG release, so the memo the release takes belongs to
 * that press; any hold event overwrites it, including with NONE when the
 * preview stayed silent or could not be queued. */
typedef struct {
    omi_preview_t played[BUTTON_COUNT];
} omi_preview_memo_t;

static inline void omi_preview_memo_hold(omi_preview_memo_t *memo, button_id_t id,
                                         omi_preview_t preview, bool played)
{
    if ((unsigned)id >= BUTTON_COUNT) return;
    omi_preview_t none = {OMI_PREVIEW_NONE, 0u};
    memo->played[id] = played ? preview : none;
}

/* Release of `id`: returns the preview that played for this press and clears it. */
static inline omi_preview_t omi_preview_memo_release(omi_preview_memo_t *memo, button_id_t id)
{
    omi_preview_t none = {OMI_PREVIEW_NONE, 0u};
    if ((unsigned)id >= BUTTON_COUNT) return none;
    omi_preview_t played = memo->played[id];
    memo->played[id] = none;
    return played;
}

static inline bool omi_preview_said_radio_prompt(omi_preview_t previewed, omi_radio_prompt_t prompt)
{
    switch (previewed.kind) {
    case OMI_PREVIEW_MESH_ON: return prompt == OMI_RADIO_PROMPT_MESH_ON;
    case OMI_PREVIEW_MESH_OFF: return prompt == OMI_RADIO_PROMPT_MESH_OFF;
    case OMI_PREVIEW_PHONE_PAIRING: return prompt == OMI_RADIO_PROMPT_PHONE_PAIRING;
    case OMI_PREVIEW_NONE:
    case OMI_PREVIEW_CHANNEL: break;
    }
    return false;
}

/* The plan's prompts minus the one already previewed; order is kept. Only
 * prompts are touched; the plan's radio actions are unchanged. */
static inline omi_radio_plan_t omi_radio_plan_after_preview(omi_radio_plan_t plan,
                                                            omi_preview_t previewed)
{
    uint8_t kept = 0u;
    for (uint8_t i = 0u; i < plan.prompt_count && i < OMI_RADIO_MAX_PROMPTS; ++i) {
        if (omi_preview_said_radio_prompt(previewed, plan.prompts[i])) continue;
        plan.prompts[kept++] = plan.prompts[i];
    }
    plan.prompt_count = kept;
    return plan;
}

/* True when the release's channel prompt still has to play. */
static inline bool omi_channel_prompt_after_preview(omi_preview_t previewed, uint8_t target)
{
    return previewed.kind != OMI_PREVIEW_CHANNEL || previewed.channel != target;
}

/* True when the preview and the action the release actually ran disagree
 * (state changed in between); the caller logs it. */
static inline bool omi_preview_mismatch(omi_preview_t previewed, omi_preview_t actual)
{
    if (previewed.kind == OMI_PREVIEW_NONE) return false;
    return previewed.kind != actual.kind ||
           (actual.kind == OMI_PREVIEW_CHANNEL && previewed.channel != actual.channel);
}

#endif
