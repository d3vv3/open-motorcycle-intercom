#include "audio_pcm_resampler.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

_Static_assert(AUDIO_PCM_RESAMPLER_TARGET_SAMPLES ==
                   4u * AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
               "PCM target must hold four 20 ms frames");
_Static_assert(AUDIO_PCM_RESAMPLER_START_SAMPLES == AUDIO_PCM_RESAMPLER_TARGET_SAMPLES,
               "PCM start threshold must match its target");

static void fill_constant(int16_t *samples, size_t count, int16_t value)
{
    size_t i;

    for (i = 0; i < count; ++i) {
        samples[i] = value;
    }
}

static void test_nominal(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_START_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t i;

    audio_pcm_resampler_reset(&state);
    for (i = 0; i < AUDIO_PCM_RESAMPLER_START_SAMPLES; ++i) {
        input[i] = (int16_t)i;
    }
    status = audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true);
    assert(!status.rejected_push);
    assert(status.depth_before == 0u);
    assert(status.depth_after == AUDIO_PCM_RESAMPLER_START_SAMPLES);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started);
    assert(!status.underrun);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_START_SAMPLES);
    assert(status.depth_after == AUDIO_PCM_RESAMPLER_START_SAMPLES -
                                     AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES);
    assert(status.consumed_count == AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES);
    assert(status.correction_ppm == 0);
    for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        assert(output[i] == (int16_t)i);
    }
}

static void test_fractional_direction_and_clamp(void)
{
    audio_pcm_resampler_t high;
    audio_pcm_resampler_t low;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_RING_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t i;
    int32_t saturated_integrator;
    bool saw_faster_consumption = false;
    bool saw_slower_consumption = false;

    fill_constant(input, AUDIO_PCM_RESAMPLER_RING_SAMPLES, 1000);
    audio_pcm_resampler_reset(&high);
    assert(!audio_pcm_resampler_push(&high, input, AUDIO_PCM_RESAMPLER_RING_SAMPLES, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&high, output, 0u);
    assert(status.correction_ppm == AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
    for (i = 0; i < 40u; ++i) {
        assert(!audio_pcm_resampler_push(&high, input, status.consumed_count, true)
                    .rejected_push);
        status = audio_pcm_resampler_render(&high, output, 0u);
        assert(status.correction_ppm == AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
        if (status.consumed_count == AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 1u) {
            saw_faster_consumption = true;
        }
    }
    saturated_integrator = high.integrator_q16;
    for (i = 0; i < 40u; ++i) {
        assert(!audio_pcm_resampler_push(&high, input, status.consumed_count, true)
                    .rejected_push);
        status = audio_pcm_resampler_render(&high, output, 0u);
    }
    assert(high.integrator_q16 == saturated_integrator);
    assert(saw_faster_consumption);

    audio_pcm_resampler_reset(&low);
    assert(!audio_pcm_resampler_push(&low, input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&low, output, 0u);
    assert(status.correction_ppm == 0);
    status = audio_pcm_resampler_render(&low, output, 0u);
    assert(status.started && !status.underrun);
    for (i = 0; i < 80u; ++i) {
        assert(!audio_pcm_resampler_push(&low, input, status.consumed_count, true)
                    .rejected_push);
        status = audio_pcm_resampler_render(&low, output, 0u);
        if (status.consumed_count == AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u) {
            saw_slower_consumption = true;
        }
    }
    assert(status.correction_ppm == -AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
    assert(saw_slower_consumption);

    while (high.depth > AUDIO_PCM_RESAMPLER_TARGET_SAMPLES) {
        status = audio_pcm_resampler_render(&high, output, 0u);
        assert(!status.underrun);
    }
    for (i = 0; i < 400u && status.correction_ppm == AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM;
         ++i) {
        size_t needed = AUDIO_PCM_RESAMPLER_TARGET_SAMPLES - high.depth;
        assert(!audio_pcm_resampler_push(&high, input, needed, true).rejected_push);
        status = audio_pcm_resampler_render(&high, output, 0u);
    }
    assert(status.correction_ppm < AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
}

static void simulate_producer(int32_t producer_ppm)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_START_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    uint64_t producer_phase = 0;
    uint64_t produced_total = 0;
    uint64_t consumed_total = 0;
    size_t minimum_depth = AUDIO_PCM_RESAMPLER_RING_SAMPLES;
    size_t maximum_depth = 0;
    size_t i;

    fill_constant(input, AUDIO_PCM_RESAMPLER_START_SAMPLES, 77);
    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun);
    consumed_total = status.consumed_count;

    for (i = 0; i < 30000u; ++i) {
        size_t produced;

        producer_phase += (uint64_t)AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES *
                          (uint64_t)(1000000 + producer_ppm);
        produced = (size_t)(producer_phase / UINT64_C(1000000));
        producer_phase %= UINT64_C(1000000);
        assert(produced <= AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 1u);
        assert(!audio_pcm_resampler_push(&state, input, produced, true).rejected_push);
        produced_total += produced;
        status = audio_pcm_resampler_render(&state, output, 0u);
        assert(status.started);
        assert(!status.underrun);
        assert(!status.rejected_push);
        assert(!status.recovery_active);
        assert(status.correction_ppm >= -AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
        assert(status.correction_ppm <= AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
        consumed_total += status.consumed_count;
        if (i >= 2000u) {
            if (state.depth < minimum_depth) {
                minimum_depth = state.depth;
            }
            if (state.depth > maximum_depth) {
                maximum_depth = state.depth;
            }
        }
    }

    assert(minimum_depth > AUDIO_PCM_RESAMPLER_TARGET_SAMPLES -
                               AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 240u);
    assert(maximum_depth < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES -
                               AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 400u);
    assert(state.depth > AUDIO_PCM_RESAMPLER_TARGET_SAMPLES -
                             AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 140u);
    assert(state.depth < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES -
                             AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 260u);
    assert(status.depth_before > AUDIO_PCM_RESAMPLER_TARGET_SAMPLES - 160u);
    assert(status.depth_before < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES + 140u);
    assert(produced_total + AUDIO_PCM_RESAMPLER_START_SAMPLES ==
           consumed_total + state.depth);
    if (producer_ppm > 0) {
        assert(status.correction_ppm > 0);
    } else {
        assert(status.correction_ppm < 0);
    }
}

static void test_ring_wrap_and_overflow(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_RING_SAMPLES];
    int16_t block[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int expected = 0;
    size_t i;
    size_t iteration;
    size_t saved_write;

    for (i = 0; i < AUDIO_PCM_RESAMPLER_RING_SAMPLES; ++i) {
        input[i] = (int16_t)(i % 20000u);
    }
    audio_pcm_resampler_reset(&state);
    assert(audio_pcm_resampler_available(NULL) == 0u);
    assert(audio_pcm_resampler_depth(NULL) == 0u);
    assert(audio_pcm_resampler_available(&state) == AUDIO_PCM_RESAMPLER_RING_SAMPLES);
    assert(audio_pcm_resampler_depth(&state) == 0u);
    status = audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_RING_SAMPLES, true);
    assert(!status.rejected_push);
    assert(audio_pcm_resampler_available(&state) == 0u);
    saved_write = state.write_index;
    status = audio_pcm_resampler_push(&state, input, 1u, true);
    assert(status.rejected_push);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_RING_SAMPLES);
    assert(status.depth_after == AUDIO_PCM_RESAMPLER_RING_SAMPLES);
    assert(state.write_index == saved_write);
    assert(audio_pcm_resampler_available(&state) == 0u);

    audio_pcm_resampler_reset(&state);
    for (i = 0; i < AUDIO_PCM_RESAMPLER_START_SAMPLES; ++i) {
        input[i] = (int16_t)expected++;
    }
    assert(!audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    assert(audio_pcm_resampler_available(&state) ==
           AUDIO_PCM_RESAMPLER_RING_SAMPLES - AUDIO_PCM_RESAMPLER_START_SAMPLES);
    expected = 0;
    for (iteration = 0; iteration < 30u; ++iteration) {
        status = audio_pcm_resampler_render(&state, output, 0u);
        assert(audio_pcm_resampler_available(&state) ==
               AUDIO_PCM_RESAMPLER_RING_SAMPLES - state.depth);
        assert(status.correction_ppm == 0);
        for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
            assert(output[i] == (int16_t)(expected++));
            block[i] =
                (int16_t)(((int)iteration +
                           (int)(AUDIO_PCM_RESAMPLER_TARGET_SAMPLES /
                                 AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES)) *
                              (int)AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES +
                          (int)i);
        }
        assert(!audio_pcm_resampler_push(&state, block, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                    .rejected_push);
    }
    assert(state.read_index < AUDIO_PCM_RESAMPLER_RING_SAMPLES);
    assert(state.write_index < AUDIO_PCM_RESAMPLER_RING_SAMPLES);
}

static void test_int16_limits(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_RING_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t i;

    for (i = 0; i < AUDIO_PCM_RESAMPLER_RING_SAMPLES; ++i) {
        input[i] = (i % 2u == 0u) ? INT16_MIN : INT16_MAX;
    }
    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_RING_SAMPLES, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.correction_ppm == AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
    assert(output[0] == INT16_MIN);
    assert(output[1] > 30000);
    assert(output[2] < -30000);
}

static void test_underrun_and_restart(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t input[AUDIO_PCM_RESAMPLER_START_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t i;

    fill_constant(input, AUDIO_PCM_RESAMPLER_START_SAMPLES, 10000);
    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    assert(audio_pcm_resampler_render(&state, output, 0u).started);
    do {
        status = audio_pcm_resampler_render(&state, output, 0u);
    } while (!status.underrun);
    assert(status.underrun);
    assert(!status.started);
    assert(status.audible_active);
    assert(status.consumed_count == 0u);
    assert(output[0] == 10000);
    assert(output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u] == 0);
    for (i = 1; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        assert(output[i] <= output[i - 1u]);
        assert(output[i] >= 0);
    }

    assert(state.depth < AUDIO_PCM_RESAMPLER_START_SAMPLES);
    assert(!audio_pcm_resampler_push(
                &state, input, AUDIO_PCM_RESAMPLER_START_SAMPLES - state.depth - 1u, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(!status.started && !status.underrun);
    assert(!status.audible_active);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_START_SAMPLES - 1u);
    assert(status.depth_after == AUDIO_PCM_RESAMPLER_START_SAMPLES - 1u);
    for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        assert(output[i] == 0);
    }
    assert(!audio_pcm_resampler_push(&state, input, 1u, true).rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun);
    assert(output[0] == 312);
    assert(output[AUDIO_PCM_RESAMPLER_RESTART_FADE_SAMPLES - 1u] == 10000);
    assert(output[AUDIO_PCM_RESAMPLER_RESTART_FADE_SAMPLES] == 10000);
}

static void test_activity_follows_pcm_timeline(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t first[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t remainder[AUDIO_PCM_RESAMPLER_START_SAMPLES -
                      AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];

    fill_constant(first, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, 1000);
    fill_constant(remainder, AUDIO_PCM_RESAMPLER_START_SAMPLES -
                                 AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
                  1000);

    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, first, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                .rejected_push);
    assert(!audio_pcm_resampler_push(&state, remainder,
                                     AUDIO_PCM_RESAMPLER_START_SAMPLES -
                                         AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
                                     false)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun && status.audible_active);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_TARGET_SAMPLES);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun && !status.audible_active);

    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, first, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, false)
                .rejected_push);
    assert(!audio_pcm_resampler_push(&state, remainder,
                                     AUDIO_PCM_RESAMPLER_START_SAMPLES -
                                         AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
                                     true)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun && !status.audible_active);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_TARGET_SAMPLES);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun && status.audible_active);
}

static void test_two_producer_gaps_are_bridged(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t initial[AUDIO_PCM_RESAMPLER_TARGET_SAMPLES];
    int16_t replacement[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];

    fill_constant(initial, AUDIO_PCM_RESAMPLER_TARGET_SAMPLES, 1234);
    fill_constant(replacement, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, 1234);
    audio_pcm_resampler_reset(&state);
    assert(!audio_pcm_resampler_push(&state, initial,
                                     AUDIO_PCM_RESAMPLER_TARGET_SAMPLES, true)
                .rejected_push);

    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun);

    assert(!audio_pcm_resampler_push(&state, replacement,
                                     AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                .rejected_push);
    assert(!audio_pcm_resampler_push(&state, replacement,
                                     AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                .rejected_push);
    status = audio_pcm_resampler_render(&state, output, 0u);
    assert(status.started && !status.underrun);
}

static void test_upstream_burst_recovery(void)
{
    audio_pcm_resampler_t state;
    audio_pcm_resampler_telemetry_t status;
    int16_t block[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t queued_frames = 7u;
    size_t pushed_samples = AUDIO_PCM_RESAMPLER_TARGET_SAMPLES;
    size_t consumed_samples = 0u;
    bool saw_burst_correction = false;
    size_t iteration;
    size_t i;

    fill_constant(block, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, 500);
    audio_pcm_resampler_reset(&state);
    for (iteration = 0u; iteration < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES /
                                      AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES;
         ++iteration) {
        assert(!audio_pcm_resampler_push(&state, block,
                                         AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                    .rejected_push);
    }
    assert(audio_pcm_resampler_depth(&state) == AUDIO_PCM_RESAMPLER_TARGET_SAMPLES);

    for (iteration = 0u; iteration < 12u * 50u && queued_frames > 0u; ++iteration) {
        if (audio_pcm_resampler_depth(&state) + AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES <=
            AUDIO_PCM_RESAMPLER_TARGET_SAMPLES) {
            assert(!audio_pcm_resampler_push(&state, block,
                                              AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, true)
                         .rejected_push);
            pushed_samples += AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES;
            --queued_frames;
        }

        fill_constant(output, AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES, INT16_MIN);
        status = audio_pcm_resampler_render(&state, output,
                                            queued_frames * AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES);
        assert(status.started);
        assert(!status.underrun);
        assert(!status.rejected_push);
        assert(status.recovery_active == (queued_frames > 0u));
        assert(status.correction_ppm >= -AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
        assert(status.correction_ppm <= AUDIO_PCM_RESAMPLER_MAX_RECOVERY_PPM);
        assert(status.consumed_count >= AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u);
        assert(status.consumed_count <= AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 7u);
        consumed_samples += status.consumed_count;
        if (status.correction_ppm > AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM) {
            saw_burst_correction = true;
        }
        for (i = 0u; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
            assert(output[i] == 500);
        }
    }

    assert(queued_frames == 0u);
    assert(iteration < 12u * 50u);
    assert(saw_burst_correction);
    assert(!status.recovery_active);
    assert(status.correction_ppm <= AUDIO_PCM_RESAMPLER_MAX_CORRECTION_PPM);
    assert(pushed_samples == consumed_samples + audio_pcm_resampler_depth(&state));
}

static void test_reset_and_instance_independence(void)
{
    audio_pcm_resampler_t first;
    audio_pcm_resampler_t second;
    audio_pcm_resampler_t clean;
    int16_t first_input[AUDIO_PCM_RESAMPLER_START_SAMPLES];
    int16_t second_input[AUDIO_PCM_RESAMPLER_START_SAMPLES];
    int16_t first_output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t second_output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    int16_t repeated_output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    audio_pcm_resampler_telemetry_t first_status;
    audio_pcm_resampler_telemetry_t repeated_status;
    size_t i;

    fill_constant(first_input, AUDIO_PCM_RESAMPLER_START_SAMPLES, 1111);
    fill_constant(second_input, AUDIO_PCM_RESAMPLER_START_SAMPLES, -2222);
    audio_pcm_resampler_reset(&first);
    audio_pcm_resampler_reset(&second);
    audio_pcm_resampler_reset(&clean);
    assert(memcmp(&first, &clean, sizeof(first)) == 0);

    assert(!audio_pcm_resampler_push(&first, first_input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    assert(!audio_pcm_resampler_push(&second, second_input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    first_status = audio_pcm_resampler_render(&first, first_output, 0u);
    assert(audio_pcm_resampler_render(&second, second_output, 0u).started);
    for (i = 0; i < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++i) {
        assert(first_output[i] == 1111);
        assert(second_output[i] == -2222);
    }
    assert(first.depth == second.depth);

    audio_pcm_resampler_reset(&first);
    assert(!audio_pcm_resampler_push(&first, first_input, AUDIO_PCM_RESAMPLER_START_SAMPLES, true)
                .rejected_push);
    repeated_status = audio_pcm_resampler_render(&first, repeated_output, 0u);
    assert(memcmp(first_output, repeated_output, sizeof(first_output)) == 0);
    assert(first_status.depth_before == repeated_status.depth_before);
    assert(first_status.depth_after == repeated_status.depth_after);
    assert(first_status.consumed_count == repeated_status.consumed_count);
    assert(first_status.correction_ppm == repeated_status.correction_ppm);
}

int main(void)
{
    test_nominal();
    test_fractional_direction_and_clamp();
    simulate_producer(500);
    simulate_producer(-750);
    test_ring_wrap_and_overflow();
    test_int16_limits();
    test_underrun_and_restart();
    test_activity_follows_pcm_timeline();
    test_two_producer_gaps_are_bridged();
    test_upstream_burst_recovery();
    test_reset_and_instance_independence();
    puts("audio_pcm_resampler tests passed");
    return 0;
}
