#include "shared/audio_tx_cache.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void fill(uint8_t *data, size_t len, uint8_t first)
{
    size_t index;
    for (index = 0u; index < len; ++index) {
        data[index] = (uint8_t)(first + index);
    }
}

static void expect_no_attach(const audio_tx_cache_t *cache, uint16_t current_seq)
{
    uint16_t len = 0xFFFFu;
    const uint8_t *data = audio_tx_cache_previous(cache, current_seq, &len);
    assert(data == NULL);
    assert(len == 0u);
}

static void test_consecutive_seq_attaches(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];
    uint16_t len = 0u;
    const uint8_t *previous;

    fill(frame, sizeof(frame), 0x10u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 100u, true);

    previous = audio_tx_cache_previous(&cache, 101u, &len);
    assert(previous != NULL);
    assert(len == sizeof(frame));
    assert(memcmp(previous, frame, sizeof(frame)) == 0);
}

static void test_seq_gap_does_not_attach(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];

    fill(frame, sizeof(frame), 0x20u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 100u, true);

    expect_no_attach(&cache, 102u); /* gap */
    expect_no_attach(&cache, 100u); /* same seq */
    expect_no_attach(&cache, 99u);  /* backwards */
}

static void test_inactive_cached_frame_does_not_attach(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];

    fill(frame, sizeof(frame), 0x30u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), false, 200u, true);

    expect_no_attach(&cache, 201u);
}

static void test_not_eligible_does_not_attach(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];

    fill(frame, sizeof(frame), 0x40u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 300u, false);

    expect_no_attach(&cache, 301u);
}

static void test_zero_len_resets(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];

    fill(frame, sizeof(frame), 0x50u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 400u, true);
    audio_tx_cache_store(&cache, frame, 0u, true, 401u, true);

    expect_no_attach(&cache, 401u);
    expect_no_attach(&cache, 402u);
}

static void test_oversize_resets(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[MESH_AUDIO_V2_MAX_FRAME_BYTES + 1u];

    fill(frame, sizeof(frame), 0x60u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, MESH_AUDIO_V2_MAX_FRAME_BYTES, true, 500u, true);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 501u, true);

    expect_no_attach(&cache, 501u);
    expect_no_attach(&cache, 502u);
}

static void test_max_len_frame_attaches(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[MESH_AUDIO_V2_MAX_FRAME_BYTES];
    uint16_t len = 0u;
    const uint8_t *previous;

    fill(frame, sizeof(frame), 0x70u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 600u, true);

    previous = audio_tx_cache_previous(&cache, 601u, &len);
    assert(previous != NULL);
    assert(len == MESH_AUDIO_V2_MAX_FRAME_BYTES);
    assert(memcmp(previous, frame, sizeof(frame)) == 0);
}

static void test_seq_wraparound_attaches(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];
    uint16_t len = 0u;
    const uint8_t *previous;

    fill(frame, sizeof(frame), 0x80u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 65535u, true);

    previous = audio_tx_cache_previous(&cache, 0u, &len);
    assert(previous != NULL);
    assert(len == sizeof(frame));
    assert(memcmp(previous, frame, sizeof(frame)) == 0);
}

static void test_reset_clears(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24];

    fill(frame, sizeof(frame), 0x90u);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, (uint16_t)sizeof(frame), true, 700u, true);
    audio_tx_cache_reset(&cache);

    expect_no_attach(&cache, 701u);
}

static void test_idle_advances_and_invalidates_at_wrap(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24] = {1u};
    uint16_t next_seq = UINT16_MAX;

    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, sizeof(frame), true, 65534u, true);
    audio_tx_cache_skip_frame(&cache, &next_seq);
    assert(next_seq == 0u);
    expect_no_attach(&cache, next_seq);
    audio_tx_cache_skip_frame(&cache, &next_seq);
    assert(next_seq == 1u);
    expect_no_attach(&cache, next_seq);
}

static void test_quiet_cap_and_lifecycle_reset(void)
{
    audio_tx_cache_t cache;
    uint8_t frame[24] = {1u};
    uint16_t next_seq = 101u;
    unsigned index;

    audio_tx_cache_reset(&cache);
    audio_tx_cache_store(&cache, frame, sizeof(frame), true, 100u, true);
    for (index = 0u; index < 65537u; ++index) {
        audio_tx_cache_skip_frame(&cache, &next_seq);
    }
    assert(next_seq == 101u + AUDIO_TX_QUIET_SEQUENCE_SLOTS);
    expect_no_attach(&cache, next_seq);
    audio_tx_cache_store(&cache, frame, sizeof(frame), true, next_seq++, true);
    assert(next_seq == 102u + AUDIO_TX_QUIET_SEQUENCE_SLOTS);
    audio_tx_cache_skip_frame(&cache, &next_seq);
    assert(next_seq == 103u + AUDIO_TX_QUIET_SEQUENCE_SLOTS);
    expect_no_attach(&cache, next_seq);

    for (index = 1u; index < AUDIO_TX_QUIET_SEQUENCE_SLOTS; ++index) {
        audio_tx_cache_skip_frame(&cache, &next_seq);
    }
    uint16_t held_seq = next_seq;
    audio_tx_cache_skip_frame(&cache, &next_seq);
    assert(next_seq == held_seq);
    audio_tx_cache_reset(&cache);
    audio_tx_cache_skip_frame(&cache, &next_seq);
    assert(next_seq == (uint16_t)(held_seq + 1u));
}

int main(void)
{
    test_consecutive_seq_attaches();
    test_seq_gap_does_not_attach();
    test_inactive_cached_frame_does_not_attach();
    test_not_eligible_does_not_attach();
    test_zero_len_resets();
    test_oversize_resets();
    test_max_len_frame_attaches();
    test_seq_wraparound_attaches();
    test_reset_clears();
    test_idle_advances_and_invalidates_at_wrap();
    test_quiet_cap_and_lifecycle_reset();
    printf("audio_tx_cache_test: all tests passed\n");
    return 0;
}
