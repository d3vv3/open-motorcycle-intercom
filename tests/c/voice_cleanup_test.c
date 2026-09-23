#include "voice_cleanup.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define SAMPLE_COUNT 320u

static void test_far_end_cannot_enter_silent_mic(void)
{
    voice_cleanup_state_t state;
    int16_t mic[SAMPLE_COUNT] = {0};
    int16_t far_end[SAMPLE_COUNT];
    voice_cleanup_init(&state);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        far_end[i] = (i & 1u) ? INT16_MAX : INT16_MIN;
    }
    assert(far_end[0] == INT16_MIN && far_end[1] == INT16_MAX);

    voice_cleanup_process(&state, mic, SAMPLE_COUNT);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        assert(mic[i] == 0);
    }
}

static void test_speech_is_retained_with_bounded_gain(void)
{
    voice_cleanup_state_t state;
    int16_t mic[SAMPLE_COUNT];
    voice_cleanup_init(&state);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        mic[i] = (i & 1u) ? 2400 : -2400;
    }

    voice_cleanup_process(&state, mic, SAMPLE_COUNT);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        assert(mic[i] != 0);
        assert(mic[i] >= -2400 && mic[i] <= 2400);
    }
    assert(state.ns_gain >= 0.18f && state.ns_gain <= 1.0f);
}

static void test_int16_extremes_do_not_overflow(void)
{
    voice_cleanup_state_t state;
    int16_t mic[SAMPLE_COUNT];
    voice_cleanup_init(&state);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        mic[i] = (i & 1u) ? INT16_MAX : INT16_MIN;
    }

    voice_cleanup_process(&state, mic, SAMPLE_COUNT);
    for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
        assert(mic[i] >= INT16_MIN && mic[i] <= INT16_MAX);
    }
}

int main(void)
{
    test_far_end_cannot_enter_silent_mic();
    test_speech_is_retained_with_bounded_gain();
    test_int16_extremes_do_not_overflow();
    puts("voice cleanup tests passed");
    return 0;
}
