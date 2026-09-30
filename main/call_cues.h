#ifndef OMI_CALL_CUES_H
#define OMI_CALL_CUES_H

#include <stdbool.h>

/* The caller is the only consumer; concurrent producers may increment pending. */
static inline bool omi_call_end_acknowledged(unsigned pending, bool enqueue_succeeded)
{
    return pending != 0u && enqueue_succeeded;
}

#endif
