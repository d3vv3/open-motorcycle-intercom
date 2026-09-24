#include "audio_notification_tone.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define RATE 16000u
#define FRAME 320u
#define BEEP 1600u
#define GAP 400u
#define PI 3.14159265358979323846

static void check_note(uint32_t frequency)
{
    uint32_t step = audio_notification_phase_step(frequency, RATE);
    uint32_t phase = 0u;
    int max_error = 0;
    double cycles = 0.0;
    int positive_peak = 0;
    int negative_peak = 0;

    assert(step == (uint32_t)(((uint64_t)frequency * UINT64_C(4294967296)) /
                              (RATE * 1000u)));
    assert(step != 0u);
    for (unsigned frame = 0; frame < BEEP / FRAME; ++frame) {
        for (unsigned i = 0; i < FRAME; ++i) {
            int sample = audio_notification_tone_sample(phase);
            int expected = (int)(AUDIO_NOTIFICATION_GAIN * sin(2.0 * PI * cycles));
            int error = sample - expected;
            if (error < 0) error = -error;
            if (error > max_error) max_error = error;
            if (sample > positive_peak) positive_peak = sample;
            if (sample < negative_peak) negative_peak = sample;
            phase += step;
            cycles += (double)frequency / (RATE * 1000u);
        }
    }
    assert(max_error <= 3);
    assert(positive_peak >= AUDIO_NOTIFICATION_GAIN - 185);
    assert(negative_peak <= -AUDIO_NOTIFICATION_GAIN + 185);
}

static void check_sequence(const uint32_t *frequencies, unsigned count)
{
    for (unsigned tone = 0; tone < count; ++tone) {
        uint32_t phase = 0u;
        uint32_t step = audio_notification_phase_step(frequencies[tone], RATE);
        assert(audio_notification_tone_sample(phase) == 0);
        for (unsigned frame = 0; frame < BEEP / FRAME; ++frame) {
            assert(phase == (uint32_t)((uint64_t)frame * FRAME * step));
            for (unsigned i = 0; i < FRAME; ++i) {
                assert(phase == (uint32_t)(((uint64_t)frame * FRAME + i) * step));
                phase += step;
            }
        }
        if (tone + 1u < count) {
            /* Gap frames advance segment time but must not advance oscillator phase. */
            uint32_t end_phase = phase;
            for (unsigned i = 0; i < GAP; ++i) assert(phase == end_phase);
        }
    }
}

int main(void)
{
    static const uint32_t startup[] = {261630u, 329630u, 392000u};
    static const uint32_t join[] = {440000u, 880000u};
    static const uint32_t leave[] = {880000u, 440000u};
    static const uint32_t pairing[] = {988000u, 988000u, 988000u};
    static const uint32_t all[] = {261630u, 329630u, 392000u, 440000u, 880000u, 988000u};

    assert(audio_notification_tone_sample(0u) == 0);
    assert(audio_notification_tone_sample(UINT32_C(0x40000000)) == AUDIO_NOTIFICATION_GAIN);
    assert(audio_notification_tone_sample(UINT32_C(0x80000000)) == 0);
    assert(audio_notification_tone_sample(UINT32_C(0xc0000000)) == -AUDIO_NOTIFICATION_GAIN);
    assert(audio_notification_tone_sample(UINT32_MAX) >= -2);
    assert(audio_notification_tone_sample(UINT32_MAX) <= 0);
    {
        uint32_t wrapped = UINT32_MAX;
        wrapped += 1u;
        assert(wrapped == 0u);
        assert(audio_notification_tone_sample(wrapped) == 0);
    }
    for (unsigned i = 0; i < sizeof(all) / sizeof(all[0]); ++i) check_note(all[i]);
    check_sequence(startup, 3u);
    check_sequence(join, 2u);
    check_sequence(leave, 2u);
    check_sequence(pairing, 3u);
    puts("audio notification tone tests passed");
    return 0;
}
