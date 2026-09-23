#include "ws_sync_math.h"

#include <assert.h>
#include <stdio.h>

static void test_nominal_and_missing_extra_edges(void)
{
    int32_t error_us, correction_us;
    assert(ws_sync_calculate_correction(960U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(error_us == 0 && correction_us == 0);
    assert(ws_sync_calculate_correction(720U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(ws_sync_calculate_correction(1200U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(ws_sync_calculate_correction(719U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_INVALID);
    assert(ws_sync_calculate_correction(1201U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_INVALID);
}

static void test_elapsed_limit_and_frame_wrap(void)
{
    int32_t error_us, correction_us;
    assert(ws_sync_calculate_correction(48000U, 50U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(ws_sync_calculate_correction(48960U, 51U, &error_us, &correction_us) ==
           WS_SYNC_MATH_INVALID);
    uint32_t elapsed = 3U - UINT32_MAX;
    assert(elapsed == 4U);
    assert(ws_sync_calculate_correction(3840U, elapsed, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
}

static void test_rounding_clamp_and_no_signal(void)
{
    int32_t error_us, correction_us;
    assert(ws_sync_calculate_correction(956U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(error_us == 83 && correction_us == 20);
    assert(ws_sync_calculate_correction(964U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(error_us == -83 && correction_us == -20);
    assert(ws_sync_calculate_correction(0U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_NO_SIGNAL);
    assert(ws_sync_calculate_correction(1200U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(correction_us == -500);
    assert(ws_sync_calculate_correction(720U, 1U, &error_us, &correction_us) ==
           WS_SYNC_MATH_VALID);
    assert(correction_us == 500);
}

int main(void)
{
    test_nominal_and_missing_extra_edges();
    test_elapsed_limit_and_frame_wrap();
    test_rounding_clamp_and_no_signal();
    puts("ws_sync math tests passed");
    return 0;
}
