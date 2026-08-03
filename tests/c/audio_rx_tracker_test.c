#include "shared/audio_rx_tracker.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static audio_rx_tracker_result_t accept(audio_rx_tracker_t *tracker, uint16_t seq)
{
    return audio_rx_tracker_accept(tracker, 7u, seq);
}

static void expect(audio_rx_tracker_result_t result,
                   audio_rx_tracker_classification_t classification,
                   uint32_t gap_frames)
{
    assert(result.classification == classification);
    assert(result.gap_frames == gap_frames);
}

static void test_first_in_order_and_gap(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    expect(accept(&tracker, 100u), AUDIO_RX_TRACKER_FIRST, 0u);
    expect(accept(&tracker, 101u), AUDIO_RX_TRACKER_IN_ORDER, 0u);
    expect(accept(&tracker, 105u), AUDIO_RX_TRACKER_FORWARD_GAP, 3u);
    assert(tracker.frames == 3u);
    assert(tracker.gap_events == 1u);
    assert(tracker.gap_frames == 3u);
    assert(tracker.sources[7].last_seq == 105u);
    assert(tracker.sources[7].outstanding_gap_frames == 3u);
}

static void test_duplicate_and_late_do_not_rewind(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 100u);
    expect(accept(&tracker, 100u), AUDIO_RX_TRACKER_REORDERED_OR_OLD, 0u);
    (void)accept(&tracker, 104u);
    expect(accept(&tracker, 102u), AUDIO_RX_TRACKER_REORDERED_OR_OLD, 0u);
    assert(tracker.frames == 4u);
    assert(tracker.reordered_or_old == 2u);
    assert(tracker.sources[7].last_seq == 104u);
    assert(tracker.sources[7].outstanding_gap_frames == 3u);
}

static void test_wraparound(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 65535u);
    expect(accept(&tracker, 0u), AUDIO_RX_TRACKER_IN_ORDER, 0u);

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 65534u);
    expect(accept(&tracker, 2u), AUDIO_RX_TRACKER_FORWARD_GAP, 3u);
    assert(tracker.sources[7].last_seq == 2u);
}

static void test_half_range_boundary(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 100u);
    expect(accept(&tracker, (uint16_t)(100u + 32768u)),
           AUDIO_RX_TRACKER_FORWARD_GAP, 32767u);

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 100u);
    expect(accept(&tracker, (uint16_t)(100u + 32769u)),
           AUDIO_RX_TRACKER_REORDERED_OR_OLD, 0u);
    assert(tracker.sources[7].last_seq == 100u);
}

static void test_gap_and_aggregate_saturation(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 10u);
    tracker.frames = UINT32_MAX;
    tracker.gap_events = UINT32_MAX;
    tracker.gap_frames = UINT32_MAX - 1u;
    tracker.reordered_or_old = UINT32_MAX;
    tracker.sources[7].outstanding_gap_frames = UINT32_MAX - 1u;

    expect(accept(&tracker, 14u), AUDIO_RX_TRACKER_FORWARD_GAP, 3u);
    expect(accept(&tracker, 14u), AUDIO_RX_TRACKER_REORDERED_OR_OLD, 0u);
    assert(tracker.frames == UINT32_MAX);
    assert(tracker.gap_events == UINT32_MAX);
    assert(tracker.gap_frames == UINT32_MAX);
    assert(tracker.reordered_or_old == UINT32_MAX);
    assert(tracker.sources[7].outstanding_gap_frames == UINT32_MAX);
}

static void test_recovery_accounting(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    assert(!audio_rx_tracker_credit_predecessor(&tracker, 7u, true));
    (void)accept(&tracker, 20u);
    (void)accept(&tracker, 23u);
    assert(!audio_rx_tracker_credit_predecessor(&tracker, 7u, false));
    assert(tracker.sources[7].outstanding_gap_frames == 2u);
    assert(tracker.recovered_frames == 0u);
    assert(audio_rx_tracker_credit_predecessor(&tracker, 7u, true));
    assert(audio_rx_tracker_credit_predecessor(&tracker, 7u, true));
    assert(!audio_rx_tracker_credit_predecessor(&tracker, 7u, true));
    assert(tracker.sources[7].outstanding_gap_frames == 0u);
    assert(tracker.recovered_frames == 2u);

    tracker.sources[7].outstanding_gap_frames = 1u;
    tracker.recovered_frames = UINT32_MAX;
    assert(audio_rx_tracker_credit_predecessor(&tracker, 7u, true));
    assert(tracker.sources[7].outstanding_gap_frames == 0u);
    assert(tracker.recovered_frames == UINT32_MAX);
}

static void test_source_reset_then_join_midstream(void)
{
    audio_rx_tracker_t tracker;

    audio_rx_tracker_reset(&tracker);
    (void)accept(&tracker, 40u);
    (void)accept(&tracker, 43u);
    audio_rx_tracker_reset_source(&tracker, 7u);
    assert(!tracker.sources[7].initialized);
    assert(tracker.frames == 2u);
    assert(tracker.gap_events == 1u);
    assert(tracker.gap_frames == 2u);
    expect(accept(&tracker, 50000u), AUDIO_RX_TRACKER_FIRST, 0u);
    assert(tracker.frames == 3u);
    assert(tracker.gap_events == 1u);
    assert(tracker.gap_frames == 2u);
    assert(tracker.sources[7].last_seq == 50000u);

    audio_rx_tracker_reset_sources(&tracker);
    assert(!tracker.sources[7].initialized);
    assert(tracker.frames == 3u);
    assert(tracker.gap_frames == 2u);
}

int main(void)
{
    test_first_in_order_and_gap();
    test_duplicate_and_late_do_not_rewind();
    test_wraparound();
    test_half_range_boundary();
    test_gap_and_aggregate_saturation();
    test_recovery_accounting();
    test_source_reset_then_join_midstream();
    puts("audio_rx_tracker_test: all tests passed");
    return 0;
}
