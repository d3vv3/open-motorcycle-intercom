#ifndef OMI_PHONE_LINK_CUE_H
#define OMI_PHONE_LINK_CUE_H

#include <stdbool.h>

/* Phone-connected beep edge: fires once when the second of A2DP and HFP (SLC)
 * comes up. A single-profile drop and reconnect, or an audio re-config, keeps it
 * disarmed; it re-arms only once both profiles are down. */
typedef struct {
    bool fired;   /* Both profiles came up since they were last both down. */
    bool pending; /* Fired but the beep has not been accepted yet. */
} omi_phone_link_cue_t;

/* Returns true while a beep is owed for the current connection. */
static inline bool omi_phone_link_cue_step(omi_phone_link_cue_t *cue, bool a2dp, bool hfp)
{
    if (a2dp && hfp) {
        if (!cue->fired) {
            cue->fired = true;
            cue->pending = true;
        }
    } else {
        /* Not fully connected any more: an unplayed beep is stale. */
        cue->pending = false;
        if (!a2dp && !hfp) cue->fired = false;
    }
    return cue->pending;
}

/* The beep was played or deliberately skipped; a transient refusal keeps it owed. */
static inline void omi_phone_link_cue_done(omi_phone_link_cue_t *cue)
{
    cue->pending = false;
}

#endif
