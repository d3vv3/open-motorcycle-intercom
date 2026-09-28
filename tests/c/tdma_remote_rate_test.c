#include "tdma_remote_rate.h"

#include <assert.h>
#include <stdio.h>

static void test_phase_reacquire_keeps_learned_period(void)
{
    tdma_remote_rate_t rate = {0};
    int64_t participant_start = 0;
    unsigned reacquires = 0;
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    for (uint32_t frame = 10; frame <= 120; frame += 10) {
        int64_t remote_start = (int64_t)frame * 20100;
        participant_start += 10 * (20000 + (rate.valid ? rate.ppm / 50 : 0));
        int64_t phase_us = remote_start - participant_start;
        assert(tdma_remote_rate_observe(&rate, frame, remote_start));
        if (phase_us > 500 || phase_us < -500) {
            reacquires++;
            participant_start = remote_start;
            /* tdma_sync reacquires phase without resetting the rate estimator. */
        }
        assert(rate.ppm == 5000);
        if (frame > 10) assert(phase_us == 0);
    }
    assert(reacquires == 1);
}

static void test_negative_nominal_and_range_clamp(void)
{
    tdma_remote_rate_t rate = {0};
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    assert(tdma_remote_rate_observe(&rate, 10, 199000));
    assert(rate.ppm == -5000);
    tdma_remote_rate_reset(&rate);
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    assert(tdma_remote_rate_observe(&rate, 10, 200000));
    assert(rate.ppm == 0);
    tdma_remote_rate_reset(&rate);
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    assert(tdma_remote_rate_observe(&rate, 10, 201200));
    assert(rate.ppm == 5000);
    tdma_remote_rate_reset(&rate);
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    assert(tdma_remote_rate_observe(&rate, 10, 198800));
    assert(rate.ppm == -5000);
}

static void test_jitter_and_bad_observation(void)
{
    tdma_remote_rate_t rate = {0};
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    for (uint32_t frame = 10; frame <= 110; frame += 10) {
        int64_t time = (int64_t)frame * 20100 + ((frame / 10) % 2 ? 100 : -100);
        assert(tdma_remote_rate_observe(&rate, frame, time));
        assert(rate.ppm >= 4500 && rate.ppm <= 5000);
    }
    int16_t learned = rate.ppm;
    assert(tdma_remote_rate_observe(&rate, 120, (int64_t)120 * 20100 + 3000));
    assert(rate.ppm == learned);
    assert(tdma_remote_rate_observe(&rate, 120, (int64_t)120 * 20100));
    assert(rate.ppm >= 4500 && rate.ppm <= 5000);
}

static void test_missing_duplicate_reorder_gap_and_wrap(void)
{
    tdma_remote_rate_t rate = {0};
    assert(!tdma_remote_rate_observe(&rate, 0, 0));
    assert(tdma_remote_rate_observe(&rate, 30, 603000));
    assert(rate.ppm == 5000);
    assert(tdma_remote_rate_observe(&rate, 30, 1));
    assert(tdma_remote_rate_observe(&rate, 20, 402000));
    assert(tdma_remote_rate_observe(&rate, 40, 603000)); /* zero elapsed: ignored */
    assert(rate.ppm == 5000);
    assert(tdma_remote_rate_observe(&rate, 300, 1)); /* bad long-gap timestamp: ignored */
    assert(rate.ppm == 5000);
    assert(!tdma_remote_rate_observe(&rate, 300, 6030000)); /* gap: new baseline */
    assert(!rate.valid);
    assert(tdma_remote_rate_observe(&rate, 330, 6633000));
    assert(rate.ppm == 5000);

    tdma_remote_rate_reset(&rate);
    assert(!tdma_remote_rate_observe(&rate, UINT32_MAX - 9U, 1000000));
    assert(tdma_remote_rate_observe(&rate, 0, 1201000));
    assert(rate.ppm == 5000);
}

static void test_long_gap_new_clock_epoch(void)
{
    tdma_remote_rate_t rate = {0};
    assert(!tdma_remote_rate_observe(&rate, 0, 1000000));
    assert(tdma_remote_rate_observe(&rate, 10, 1201000));
    assert(rate.ppm == 5000);

    /* Old-anchor tolerance cannot validate the new leader's shifted epoch. */
    assert(!tdma_remote_rate_observe(&rate, 300, 8000000));
    assert(!rate.valid && rate.ppm == 0);
    assert(tdma_remote_rate_observe(&rate, 330, 8597000));
    assert(rate.valid && rate.ppm == -5000);

    /* A within-window outlier must still leave the new estimate intact. */
    assert(tdma_remote_rate_observe(&rate, 340, 9000000));
    assert(rate.ppm == -5000);
}

static void test_frequent_syncs_keep_a_long_window(void)
{
    tdma_remote_rate_t rate = {0};
    for (uint32_t frame = 0; frame <= 100; frame++) {
        (void)tdma_remote_rate_observe(&rate, frame, (int64_t)frame * 20100);
    }
    assert(rate.valid && rate.ppm == 5000);
}

int main(void)
{
    test_phase_reacquire_keeps_learned_period();
    test_negative_nominal_and_range_clamp();
    test_jitter_and_bad_observation();
    test_missing_duplicate_reorder_gap_and_wrap();
    test_long_gap_new_clock_epoch();
    test_frequent_syncs_keep_a_long_window();
    puts("TDMA remote rate tests passed");
    return 0;
}
