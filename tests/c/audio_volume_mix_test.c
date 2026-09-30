#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>

#include "audio_volume_mix.h"

static void test_gains(void)
{
    audio_volume_levels_t levels = {0};
    audio_volume_levels_reset(&levels);
    assert(audio_volume_levels_get(&levels, false) == 100u);
    assert(audio_volume_levels_get(&levels, true) == 100u);
    audio_volume_levels_set(&levels, false, 0u);
    assert(audio_volume_levels_get(&levels, false) == 0u);
    assert(audio_volume_levels_get(&levels, true) == 100u);
    audio_volume_levels_set(&levels, true, 50u);
    assert(audio_volume_levels_get(&levels, false) == 0u);
    assert(audio_volume_levels_get(&levels, true) == 50u);
    audio_gain_ramp_t mesh, bt;
    audio_gain_reset(&mesh);
    audio_gain_reset(&bt);
    int16_t m[320], b[320];
    for (size_t i = 0; i < 320; ++i) m[i] = b[i] = 20000;
    audio_gain_apply(&mesh, m, 320, 0, 16000);
    audio_gain_apply(&bt, b, 320, 100, 16000);
    assert(m[319] == 0 && b[319] == 20000);
    audio_gain_apply(&bt, b, 320, 0, 16000);
    audio_gain_apply(&mesh, m, 320, 100, 16000);
    assert(b[319] == 0 && m[319] == 0); /* previously muted input remains muted */
    m[0] = 20000;
    audio_gain_apply(&mesh, m, 1, 100, 16000);
    assert(m[0] == 20000);
    int16_t call[320];
    for (size_t i = 0; i < 320; ++i) call[i] = 20000;
    audio_gain_apply(&bt, call, 320, 50, 16000);
    assert(call[319] == 10000);
    for (size_t i = 0; i < 320; ++i) call[i] = 20000;
    audio_gain_apply(&bt, call, 320, 100, 16000);
    assert(call[319] == 20000);
    for (size_t i = 0; i < 320; ++i) call[i] = 20000;
    audio_gain_apply(&bt, call, 320, 0, 16000);
    assert(call[319] == 0);
    for (size_t i = 0; i < 320; ++i) call[i] = 20000;
    audio_gain_apply(&bt, call, 320, 255, 16000);
    assert(call[319] == 20000); /* invalid gain is saturated */
    audio_gain_reset(&bt);
    for (size_t i = 0; i < 320; ++i) call[i] = INT16_MIN;
    audio_gain_apply(&bt, call, 320, 50, 16000);
    assert(call[319] == -16384);
}

static void test_cue(uint32_t rate, uint8_t channels)
{
    audio_limit_cue_t cue = {0};
    audio_limit_cue_reset(&cue);
    assert(audio_limit_cue_request(&cue));
    assert(!audio_limit_cue_request(&cue));
    const uint32_t on = rate * 80u / 1000u;
    const uint32_t gap = rate * 60u / 1000u;
    const uint32_t end = 3u * on + 2u * gap;
    int16_t frame[2];
    unsigned bursts = 0;
    bool previous = false;
    for (uint32_t pos = 0; pos < end + rate / 10u; ++pos) {
        for (uint8_t c = 0; c < channels; ++c) frame[c] = 0;
        audio_limit_cue_mix(&cue, frame, 1, channels, rate);
        bool in_burst = pos < end && pos % (on + gap) < on;
        if (in_burst && !previous) bursts++;
        previous = in_burst;
        if (!in_burst || pos >= end) assert(frame[0] == 0);
        for (uint8_t c = 1; c < channels; ++c) assert(frame[c] == frame[0]);
        if (pos == on + gap / 2u) assert(!audio_limit_cue_request(&cue));
        if (pos == end - 1u) assert(!audio_limit_cue_busy(&cue));
    }
    assert(bursts == 3u);
    assert(audio_limit_cue_request(&cue));
    bool audible = false;
    for (unsigned i = 0; i < rate / 100u; ++i) {
        for (uint8_t c = 0; c < channels; ++c) frame[c] = 0;
        audio_limit_cue_mix(&cue, frame, 1, channels, rate);
        if (frame[0]) audible = true;
    }
    assert(audible);
    audio_limit_cue_reset(&cue);
    assert(!audio_limit_cue_busy(&cue));
    assert(audio_limit_cue_request(&cue));
    for (uint32_t pos = 0; pos < end; ++pos) {
        for (uint8_t c = 0; c < channels; ++c) frame[c] = INT16_MAX;
        audio_limit_cue_mix(&cue, frame, 1, channels, rate);
        assert(frame[0] >= 20000); /* ducking leaves room even for a full-scale source */
    }
    assert(!audio_limit_cue_busy(&cue));
}

static void test_cue_non_aligned_completion(uint32_t rate, uint8_t channels, size_t chunk)
{
    const uint32_t on = rate * 80u / 1000u;
    const uint32_t gap = rate * 60u / 1000u;
    const uint32_t end = on * 3u + gap * 2u;
    const size_t total = end + rate / 4u;
    audio_limit_cue_t cue = {0};
    audio_limit_cue_reset(&cue);
    assert(audio_limit_cue_request(&cue));
    int16_t *long_block = malloc(total * channels * sizeof(*long_block));
    assert(long_block != NULL);
    for (size_t i = 0; i < total * channels; ++i) long_block[i] = 12345;
    audio_limit_cue_mix(&cue, long_block, total, channels, rate);
    for (size_t i = end * channels; i < total * channels; ++i)
        assert(long_block[i] == 12345);
    free(long_block);
    assert(!audio_limit_cue_busy(&cue) && cue.position == 0u && cue.phase == 0u);

    /* A second request must render all three bursts, with no extra tail even
     * when completion falls in the middle of an output block. */
    assert(audio_limit_cue_request(&cue));
    int16_t block[997 * 2];
    bool seen[3] = {false, false, false};
    for (size_t start = 0; start < total; start += chunk) {
        size_t frames = chunk < total - start ? chunk : total - start;
        for (size_t i = 0; i < frames * channels; ++i) block[i] = 0;
        audio_limit_cue_mix(&cue, block, frames, channels, rate);
        for (size_t i = 0; i < frames; ++i) {
            size_t position = start + i;
            size_t burst = position / (on + gap);
            bool sounding = position < end && burst < 3u && position % (on + gap) < on;
            if (sounding && block[i * channels] != 0) seen[burst] = true;
            if (!sounding) assert(block[i * channels] == 0);
            for (uint8_t c = 1; c < channels; ++c)
                assert(block[i * channels + c] == block[i * channels]);
        }
        if (start == 0u) assert(!audio_limit_cue_request(&cue));
        if (start + frames >= end)
            assert(!audio_limit_cue_busy(&cue) && cue.position == 0u && cue.phase == 0u);
    }
    assert(seen[0] && seen[1] && seen[2]);

    /* Recover an impossible stale position before making ownership available. */
    assert(audio_limit_cue_request(&cue));
    cue.position = end;
    cue.phase = 17u;
    block[0] = 12345;
    audio_limit_cue_mix(&cue, block, 1u, 1u, rate);
    assert(block[0] == 12345);
    assert(!audio_limit_cue_busy(&cue) && cue.position == 0u && cue.phase == 0u);
    assert(audio_limit_cue_request(&cue));
    block[0] = 0;
    audio_limit_cue_mix(&cue, block, 1u, 1u, rate);
    assert(cue.position == 1u && audio_limit_cue_busy(&cue));
}

static void test_program_mix(void)
{
    audio_gain_ramp_t mesh_gain, bt_gain;
    audio_program_mix_t mixer = {0};
    audio_gain_reset(&mesh_gain);
    audio_gain_reset(&bt_gain);
    int16_t voice[960], music[960], mesh[320];
    for (size_t i = 0; i < 320; ++i) mesh[i] = 12000;
    audio_gain_apply(&mesh_gain, mesh, 320, 100, 16000);
    assert(audio_voice_contribution_samples(mesh, 320, 0) == 320);
    for (size_t i = 0; i < 960; ++i) voice[i] = 12000, music[i] = 16000;
    audio_gain_apply(&bt_gain, music, 960, 100, 48000);
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    assert(voice[959] == 14000);

    /* BT mute reaches zero after 5 ms, but the mesh mix must not jump at
     * the following frame boundary. It eventually equals mesh-only output. */
    for (size_t i = 0; i < 960; ++i) voice[i] = 12000, music[i] = 16000;
    audio_gain_apply(&bt_gain, music, 960, 0, 48000);
    assert(music[1] != 0 && music[959] == 0);
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    int16_t boundary = voice[959];
    for (size_t i = 0; i < 960; ++i) voice[i] = 12000, music[i] = 16000;
    audio_gain_apply(&bt_gain, music, 960, 0, 48000);
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    assert(voice[0] >= boundary - 100 && voice[0] <= boundary + 100);
    assert(voice[959] == 12000);

    /* Mesh mute cannot halve Bluetooth music, even with a decoded mesh
     * frame queued. A separately overlaid prompt remains a voice contributor. */
    for (size_t i = 0; i < 320; ++i) mesh[i] = 12000;
    audio_gain_apply(&mesh_gain, mesh, 320, 0, 16000);
    assert(mesh[0] != 0 && mesh[319] == 0);
    assert(audio_voice_contribution_samples(mesh, 320, 0) == 320);
    for (size_t i = 0; i < 320; ++i) mesh[i] = 12000;
    audio_gain_apply(&mesh_gain, mesh, 320, 0, 16000);
    assert(audio_voice_contribution_samples(mesh, 320, 0) == 0);
    for (size_t i = 0; i < 960; ++i) voice[i] = 0, music[i] = 16000;
    audio_gain_apply(&bt_gain, music, 960, 100, 48000);
    audio_program_mix(&mixer, voice, 960, 0, music, 960, 48000);
    int16_t music_boundary = voice[959];
    for (size_t i = 0; i < 960; ++i) voice[i] = 0, music[i] = 16000;
    audio_gain_apply(&bt_gain, music, 960, 100, 48000);
    audio_program_mix(&mixer, voice, 960, 0, music, 960, 48000);
    assert(voice[0] >= music_boundary - 100 && voice[0] <= music_boundary + 100);
    assert(voice[959] == 16000);

    size_t voice_present = audio_voice_contribution_samples(mesh, 320, 0);
    assert(voice_present == 0);
    mesh[0] = 4000; /* Prompt mixed after mesh gain; pending voice FIFO tail. */
    if (voice_present == 0) voice_present = 1; /* audio_notify_mix_frame contribution */
    assert(voice_present == 1);
    for (size_t i = 0; i < 960; ++i) voice[i] = 4000, music[i] = 16000;
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    for (size_t i = 0; i < 960; ++i) voice[i] = 4000, music[i] = 16000;
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    assert(voice[959] == 10000); /* Prompt flag may already be inactive. */

    for (size_t i = 0; i < 960; ++i) voice[i] = music[i] = INT16_MAX;
    audio_program_mix(&mixer, voice, 960, 960, music, 960, 48000);
    assert(voice[959] == INT16_MAX);

    /* The final cue has its own path even when both program levels are zero. */
    audio_limit_cue_t cue = {0};
    audio_limit_cue_reset(&cue);
    assert(audio_limit_cue_request(&cue));
    for (size_t i = 0; i < 960; ++i) voice[i] = music[i] = 0;
    audio_program_mix(&mixer, voice, 960, 0, music, 960, 48000);
    audio_limit_cue_mix(&cue, voice, 960, 1, 48000);
    bool audible = false;
    for (size_t i = 0; i < 960; ++i) if (voice[i]) audible = true;
    assert(audible);
}

static void test_call_end(uint32_t rate, uint8_t channels, size_t chunk)
{
    audio_limit_cue_t cue = {0};
    audio_limit_cue_reset(&cue);
    assert(audio_limit_cue_request_end(&cue));
    assert(audio_limit_cue_request_end(&cue));
    const uint32_t end = rate * 80u / 1000u;
    int16_t frame[997 * 2];
    bool audible = false;
    for (size_t start = 0; start < end + chunk; start += chunk) {
        size_t count = chunk;
        for (size_t i = 0; i < count * channels; ++i) frame[i] = 0;
        audio_limit_cue_mix(&cue, frame, count, channels, rate);
        for (size_t i = 0; i < count; ++i) {
            if (start + i >= end) assert(frame[i * channels] == 0);
            else if (frame[i * channels] != 0) audible = true;
            for (uint8_t c = 1; c < channels; ++c)
                assert(frame[i * channels + c] == frame[i * channels]);
        }
    }
    assert(audible && !audio_limit_cue_busy(&cue));
    audio_limit_cue_reset(&cue);
    assert(audio_limit_cue_request(&cue));
    assert(audio_limit_cue_request_end(&cue));
    assert(audio_limit_cue_request_end(&cue));
    size_t duration = rate * 80u / 1000u * 3u + rate * 60u / 1000u * 2u;
    for (size_t start = 0; start < duration + end + chunk; start += chunk) {
        for (size_t i = 0; i < chunk * channels; ++i) frame[i] = 0;
        audio_limit_cue_mix(&cue, frame, chunk, channels, rate);
    }
    assert(!audio_limit_cue_busy(&cue));
}

static void test_pending_end_transition(uint32_t rate, uint8_t channels)
{
    audio_limit_cue_t cue = {0};
    audio_limit_cue_reset(&cue);
    const uint32_t on = rate * 80u / 1000u;
    const uint32_t gap = rate * 60u / 1000u;
    const uint32_t limit_end = on * 3u + gap * 2u;
    int16_t frame[2];
    bool sounded[4] = {false};
    assert(audio_limit_cue_request(&cue));
    assert(atomic_load(&cue.state) == AUDIO_CUE_LIMIT);
    assert(audio_limit_cue_request_end(&cue));
    assert(audio_limit_cue_request_end(&cue));
    assert(atomic_load(&cue.state) == AUDIO_CUE_LIMIT_WITH_PENDING_END);
    assert(!audio_limit_cue_request(&cue));
    for (uint32_t pos = 0; pos < limit_end + on + gap; ++pos) {
        if (pos == limit_end - 1u) {
            assert(audio_limit_cue_request_end(&cue));
            assert(atomic_load(&cue.state) == AUDIO_CUE_LIMIT_WITH_PENDING_END);
        }
        if (pos == limit_end) {
            assert(atomic_load(&cue.state) == AUDIO_CUE_END);
            assert(cue.position == 0u && cue.phase == 0u);
            assert(audio_limit_cue_request_end(&cue));
            assert(!audio_limit_cue_request(&cue));
        }
        for (uint8_t c = 0; c < channels; ++c) frame[c] = 0;
        audio_limit_cue_mix(&cue, frame, 1u, channels, rate);
        uint32_t burst = pos < limit_end ? pos / (on + gap) : 3u;
        bool sounding = pos < limit_end
            ? pos % (on + gap) < on : pos - limit_end < on;
        if (sounding && frame[0] != 0) sounded[burst] = true;
        if (!sounding) assert(frame[0] == 0);
        for (uint8_t c = 1; c < channels; ++c) assert(frame[c] == frame[0]);
        if (pos == limit_end + on - 1u) {
            assert(!audio_limit_cue_busy(&cue));
            assert(cue.position == 0u && cue.phase == 0u);
        }
    }
    for (size_t i = 0; i < 4u; ++i) assert(sounded[i]);
    assert(atomic_load(&cue.state) == AUDIO_CUE_IDLE);
}

int main(void)
{
    test_gains();
    test_program_mix();
    test_cue(16000u, 1u);
    test_cue(48000u, 1u);
    test_cue(48000u, 2u);
    test_cue_non_aligned_completion(16000u, 1u, 137u);
    test_cue_non_aligned_completion(48000u, 2u, 997u);
    test_call_end(16000u, 1u, 137u);
    test_call_end(48000u, 2u, 997u);
    test_pending_end_transition(16000u, 1u);
    test_pending_end_transition(48000u, 2u);
    puts("audio volume mix tests passed");
    return 0;
}
