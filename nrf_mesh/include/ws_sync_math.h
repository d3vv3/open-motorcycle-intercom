#ifndef OMI_WS_SYNC_MATH_H
#define OMI_WS_SYNC_MATH_H

#include <stdbool.h>
#include <stdint.h>

#define WS_SYNC_EXPECTED_EDGES_PER_FRAME 960U
#define WS_SYNC_MAX_ELAPSED_FRAMES 50U

typedef enum {
    WS_SYNC_MATH_INVALID,
    WS_SYNC_MATH_NO_SIGNAL,
    WS_SYNC_MATH_VALID,
} ws_sync_math_result_t;

static inline ws_sync_math_result_t ws_sync_calculate_correction(uint32_t delta,
                                                                 uint32_t elapsed_frames,
                                                                 int32_t *error_us,
                                                                 int32_t *correction_us)
{
    if (elapsed_frames == 0U || elapsed_frames > WS_SYNC_MAX_ELAPSED_FRAMES) {
        return WS_SYNC_MATH_INVALID;
    }
    if (delta == 0U) {
        return WS_SYNC_MATH_NO_SIGNAL;
    }

    uint32_t expected = elapsed_frames * WS_SYNC_EXPECTED_EDGES_PER_FRAME;
    if (delta < (expected * 3U) / 4U || delta > (expected * 5U) / 4U) {
        return WS_SYNC_MATH_INVALID;
    }

    int64_t numerator = ((int64_t)expected - (int64_t)delta) * 125;
    int64_t rounded = numerator >= 0 ? (numerator + 3) / 6 : (numerator - 3) / 6;
    *error_us = (int32_t)rounded;

    int32_t correction = *error_us / 4;
    if (correction > 500) {
        correction = 500;
    } else if (correction < -500) {
        correction = -500;
    }
    *correction_us = correction;
    return WS_SYNC_MATH_VALID;
}

#endif
