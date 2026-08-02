/* Host unit tests for the pure RX source slot selection/eviction policy.
 *
 * Build:
 *   cc -std=c11 -Wall -Wextra -Werror -pedantic -Icomponents/audio/include \
 *      components/audio/audio_rx_source_select.c \
 *      tests/c/audio_rx_source_select_test.c -o /tmp/rxsel && /tmp/rxsel
 */
#include "audio_rx_source_select.h"

#include <stdio.h>
#include <stdlib.h>

#define NUM_SLOTS 3u

static int s_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            s_failures++;                                                      \
        }                                                                      \
    } while (0)

static audio_rx_slot_snapshot_t slot(bool assigned, uint8_t source_id,
                                     uint64_t last_active_ms)
{
    audio_rx_slot_snapshot_t s = {
        .assigned = assigned,
        .source_id = source_id,
        .last_active_ms = last_active_ms,
    };
    return s;
}

/* 1. Existing-source match wins even when free slots exist. */
static void test_match_wins_over_free_slot(void)
{
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(false, 0, 0),      /* free slot before the match */
        slot(true, 7, 1000),    /* matching slot */
        slot(false, 0, 0),
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 7, true, 2000);
    CHECK(d.action == AUDIO_RX_SELECT_MATCH);
    CHECK(d.slot_index == 1);

    /* Also with an inactive (keepalive) frame. */
    d = audio_rx_source_select(slots, NUM_SLOTS, 7, false, 2000);
    CHECK(d.action == AUDIO_RX_SELECT_MATCH);
    CHECK(d.slot_index == 1);
}

/* 2. Free slot preferred over eviction, even when eviction would be legal. */
static void test_free_slot_preferred_over_eviction(void)
{
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(true, 1, 0),       /* silent forever -> evictable */
        slot(false, 0, 0),      /* free */
        slot(true, 2, 0),       /* silent forever -> evictable */
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 9, true, 100000);
    CHECK(d.action == AUDIO_RX_SELECT_ASSIGN_FREE);
    CHECK(d.slot_index == 1);
}

/* 3. Keepalive (active=false) claims a free slot but NEVER evicts. */
static void test_keepalive_claims_free_but_never_evicts(void)
{
    /* Free slot available: keepalive claims it. */
    audio_rx_slot_snapshot_t with_free[NUM_SLOTS] = {
        slot(true, 1, 5000),
        slot(true, 2, 5000),
        slot(false, 0, 0),
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(with_free, NUM_SLOTS, 9, false, 6000);
    CHECK(d.action == AUDIO_RX_SELECT_ASSIGN_FREE);
    CHECK(d.slot_index == 2);

    /* No free slot: keepalive is rejected even though every slot has been
     * silent far beyond the eviction threshold. */
    audio_rx_slot_snapshot_t all_stale[NUM_SLOTS] = {
        slot(true, 1, 0),
        slot(true, 2, 0),
        slot(true, 3, 0),
    };
    d = audio_rx_source_select(all_stale, NUM_SLOTS, 9, false, 100000);
    CHECK(d.action == AUDIO_RX_SELECT_REJECT);
}

/* 4. Eviction only at silence >= RX_SOURCE_EVICT_SILENCE_MS (399 vs 400). */
static void test_eviction_silence_threshold_boundary(void)
{
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(true, 1, 1000),    /* the potential victim */
        slot(true, 2, 2000),
        slot(true, 3, 2000),
    };
    /* 399 ms of silence: NOT evictable. */
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 9, true, 1000 + 399);
    CHECK(d.action == AUDIO_RX_SELECT_REJECT);

    /* Exactly 400 ms of silence: evictable. */
    d = audio_rx_source_select(slots, NUM_SLOTS, 9, true,
                               1000 + RX_SOURCE_EVICT_SILENCE_MS);
    CHECK(RX_SOURCE_EVICT_SILENCE_MS == 400);
    CHECK(d.action == AUDIO_RX_SELECT_EVICT);
    CHECK(d.slot_index == 0);
}

/* 5. Victim is the longest-silent slot (smallest last_active_ms). */
static void test_victim_is_longest_silent(void)
{
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(true, 1, 3000),
        slot(true, 2, 1000),    /* longest silent */
        slot(true, 3, 2000),
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 9, true, 10000);
    CHECK(d.action == AUDIO_RX_SELECT_EVICT);
    CHECK(d.slot_index == 1);
}

/* 6. All slots recently active + new active talker -> REJECT
 *    (first speaker wins). */
static void test_all_recently_active_rejects_new_talker(void)
{
    uint64_t now = 5000;
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(true, 1, now - 100),
        slot(true, 2, now - 200),
        slot(true, 3, now - 399),   /* even the oldest is < 400 ms silent */
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 9, true, now);
    CHECK(d.action == AUDIO_RX_SELECT_REJECT);
}

/* 7. A freshly assigned keepalive-only stream (last_active_ms still 0) is
 *    evictable immediately by an active talker. */
static void test_fresh_keepalive_stream_is_immediately_evictable(void)
{
    uint64_t now = 5000;
    audio_rx_slot_snapshot_t slots[NUM_SLOTS] = {
        slot(true, 1, now - 50),
        slot(true, 2, 0),       /* assigned via keepalive, never VOX-active */
        slot(true, 3, now - 50),
    };
    audio_rx_select_decision_t d =
        audio_rx_source_select(slots, NUM_SLOTS, 9, true, now);
    CHECK(d.action == AUDIO_RX_SELECT_EVICT);
    CHECK(d.slot_index == 1);
}

int main(void)
{
    test_match_wins_over_free_slot();
    test_free_slot_preferred_over_eviction();
    test_keepalive_claims_free_but_never_evicts();
    test_eviction_silence_threshold_boundary();
    test_victim_is_longest_silent();
    test_all_recently_active_rejects_new_talker();
    test_fresh_keepalive_stream_is_immediately_evictable();

    if (s_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("audio_rx_source_select_test: all checks passed\n");
    return EXIT_SUCCESS;
}
