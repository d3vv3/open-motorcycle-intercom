#ifndef OMI_PHONE_LINK_CUE_H
#define OMI_PHONE_LINK_CUE_H

#include <stdbool.h>

/* Phone link beeps. Connected fires once when the second of A2DP and HFP (SLC)
 * comes up. Disconnected fires once when a connection that earned the connect
 * beep goes fully down (both profiles). A single-profile drop and reconnect, or
 * an audio re-config, plays neither. A disconnect we caused (mesh on turning
 * Bluetooth off) is silent: "Bluetooth off" already explains it. */
typedef enum {
    OMI_PHONE_CUE_NONE,
    OMI_PHONE_CUE_CONNECTED,
    OMI_PHONE_CUE_DISCONNECTED,
} omi_phone_cue_t;

typedef struct {
    bool fired;               /* Both profiles came up since they were last both down. */
    bool pending;             /* Connect beep owed, not accepted yet. */
    bool disconnect_pending;  /* Disconnect beep owed, not accepted yet. */
    bool suppress_disconnect; /* This connection's drop is ours; stay silent. */
} omi_phone_link_cue_t;

/* Returns the beep owed for the current link state, if any. */
static inline omi_phone_cue_t omi_phone_link_cue_step(omi_phone_link_cue_t *cue, bool a2dp, bool hfp)
{
    if (a2dp && hfp) {
        if (!cue->fired) {
            cue->fired = true;
            cue->pending = true;
            /* Back before the disconnect beep played: it is stale. */
            cue->disconnect_pending = false;
        }
    } else {
        /* Not fully connected any more: an unplayed connect beep is stale. */
        cue->pending = false;
        if (!a2dp && !hfp && cue->fired) {
            cue->fired = false;
            cue->disconnect_pending = !cue->suppress_disconnect;
            cue->suppress_disconnect = false;
        }
    }
    if (cue->pending) return OMI_PHONE_CUE_CONNECTED;
    return cue->disconnect_pending ? OMI_PHONE_CUE_DISCONNECTED : OMI_PHONE_CUE_NONE;
}

/* The owed beep was played or deliberately skipped; a transient refusal keeps it owed. */
static inline void omi_phone_link_cue_done(omi_phone_link_cue_t *cue)
{
    if (cue->pending) cue->pending = false;
    else cue->disconnect_pending = false;
}

/* We are dropping the phone ourselves (Bluetooth off for mesh on). Only the
 * current connection is affected; with no full connection there is no beep. */
static inline void omi_phone_link_cue_suppress_disconnect(omi_phone_link_cue_t *cue)
{
    if (cue->fired) cue->suppress_disconnect = true;
}

#endif
