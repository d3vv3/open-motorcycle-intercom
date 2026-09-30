#ifndef OMI_CALL_PRIVACY_H
#define OMI_CALL_PRIVACY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OMI_PRIVACY_NONE,
    OMI_PRIVACY_PAUSE,
    OMI_PRIVACY_RESUME,
} omi_privacy_action_t;

typedef struct {
    uint32_t epoch;
    bool pause_acked;
    bool resumed;
} omi_privacy_reconcile_t;

/* Allow a brief HFP indicator sequence to settle before acknowledging resume. */
static inline bool omi_privacy_resume_settled(int64_t now_ms, int64_t transition_ms)
{
    return now_ms - transition_ms >= 200;
}

static inline omi_privacy_action_t omi_privacy_next(omi_privacy_reconcile_t *state,
                                                    uint32_t epoch, bool active, bool blocked)
{
    if (state->epoch != epoch) {
        state->epoch = epoch;
        state->pause_acked = false;
        state->resumed = false;
    }
    if (!blocked) return OMI_PRIVACY_NONE;
    if (!state->pause_acked) return OMI_PRIVACY_PAUSE;
    if (!active && !state->resumed) return OMI_PRIVACY_RESUME;
    return OMI_PRIVACY_NONE;
}

#endif
