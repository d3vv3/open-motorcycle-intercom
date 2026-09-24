#include "shared/audio_bundle.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fill(uint8_t *data, size_t len, uint8_t first)
{
    size_t index;
    for (index = 0u; index < len; ++index) {
        data[index] = (uint8_t)(first + index);
    }
}

static audio_bundle_view_t make_bundle(const uint8_t *previous2, size_t previous2_len,
                                       const uint8_t *previous1, size_t previous1_len,
                                       const uint8_t *current, size_t current_len,
                                       uint16_t sequence, uint8_t flags)
{
    audio_bundle_view_t bundle = {0};
    bundle.previous2_data = previous2;
    bundle.previous2_len = previous2_len;
    bundle.previous1_data = previous1;
    bundle.previous1_len = previous1_len;
    bundle.current_data = current;
    bundle.current_len = current_len;
    bundle.current_seq = sequence;
    bundle.stream_id = 9u;
    bundle.flags = flags;
    bundle.codec = MESH_AUDIO_V2_CODEC_LC3;
    return bundle;
}

static void expect_parse_rejected(const uint8_t *wire, size_t wire_len)
{
    audio_bundle_view_t parsed;
    assert(!audio_bundle_parse(wire, wire_len, &parsed));
}

static void test_zero_predecessors(void)
{
    uint8_t current[MESH_LC3_FRAME_BYTES];
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    size_t wire_len;
    audio_bundle_view_t parsed;
    audio_bundle_view_t input;

    fill(current, sizeof(current), 0x11u);
    input = make_bundle(NULL, 0u, NULL, 0u, current, sizeof(current), 7u,
                        AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == MESH_AUDIO_V2_FIXED_HEADER_SIZE + MESH_LC3_FRAME_BYTES);
    assert(wire[0] == MESH_AUDIO_V2_CODEC_LC3 && wire[1] == 20u && wire[2] == 9u);
    assert(wire[3] == AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
    assert(wire[4] == 0u && wire[5] == 7u);
    assert(wire[6] == sizeof(current) && wire[7] == 0u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3);
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(parsed.previous1_data == NULL && parsed.previous1_len == 0u);
    assert(parsed.current_data == wire + MESH_AUDIO_V2_FIXED_HEADER_SIZE);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_one_predecessor(void)
{
    uint8_t previous1[MESH_LC3_FRAME_BYTES];
    uint8_t current[MESH_LC3_FRAME_BYTES];
    uint8_t wire[MESH_AUDIO_V2_FIXED_HEADER_SIZE + 2 * MESH_LC3_FRAME_BYTES];
    size_t wire_len;
    audio_bundle_view_t parsed;
    uint8_t flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                    AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    audio_bundle_view_t input;

    fill(previous1, sizeof(previous1), 0x31u);
    fill(current, sizeof(current), 0x41u);
    input = make_bundle(NULL, 0u, previous1, sizeof(previous1), current,
                        sizeof(current), 12u, flags);
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == sizeof(wire) && wire[7] == sizeof(previous1));
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3);
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(parsed.previous1_data == wire + 8u);
    assert(parsed.current_data == wire + 8u + sizeof(previous1));
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_max_and_pointer_order(void)
{
    uint8_t previous2[MESH_LC3_FRAME_BYTES];
    uint8_t previous1[MESH_LC3_FRAME_BYTES];
    uint8_t current[MESH_LC3_FRAME_BYTES];
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    size_t wire_len;
    audio_bundle_view_t parsed;
    audio_bundle_view_t input;

    fill(previous2, sizeof(previous2), 0x10u);
    fill(previous1, sizeof(previous1), 0x50u);
    fill(current, sizeof(current), 0x90u);
    input = make_bundle(previous2, sizeof(previous2), previous1, sizeof(previous1),
                        current, sizeof(current), 0u, AUDIO_BUNDLE_FLAG_MASK);
    assert(!audio_bundle_encode(&input, wire,
                                MESH_AUDIO_V2_FIXED_HEADER_SIZE + 3 * MESH_LC3_FRAME_BYTES - 1u,
                                &wire_len));
    assert(wire_len == 0u);
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == 152u && wire[6] == 48u && wire[7] == 48u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3);
    assert(parsed.current_seq == 0u);
    assert((uint16_t)(parsed.current_seq - 1u) == UINT16_C(65535));
    assert((uint16_t)(parsed.current_seq - 2u) == UINT16_C(65534));
    assert(parsed.previous2_data == wire + 8u);
    assert(parsed.previous1_data == wire + 8u + sizeof(previous2));
    assert(parsed.current_data == wire + 8u + sizeof(previous2) + sizeof(previous1));
    assert(parsed.previous2_len == 48u && parsed.previous1_len == 48u &&
           parsed.current_len == 48u);
    assert(memcmp(parsed.previous2_data, previous2, sizeof(previous2)) == 0);
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_all_presence_flag_combinations(void)
{
    uint8_t wire[MESH_AUDIO_V2_FIXED_HEADER_SIZE + 3 * MESH_LC3_FRAME_BYTES] = {
        MESH_AUDIO_V2_CODEC_LC3, MESH_AUDIO_V2_FRAME_MS, 3u, 0u, 0u, 1u,
        MESH_LC3_FRAME_BYTES, 0u
    };
    unsigned int flags;
    size_t previous1_len;
    size_t previous2_len;

    for (flags = 0u; flags <= UINT8_MAX; ++flags) {
        for (previous1_len = 0u; previous1_len <= MESH_LC3_FRAME_BYTES;
             previous1_len += MESH_LC3_FRAME_BYTES) {
            for (previous2_len = 0u; previous2_len <= MESH_LC3_FRAME_BYTES;
                 previous2_len += MESH_LC3_FRAME_BYTES) {
                audio_bundle_view_t parsed;
                bool previous1_present =
                    (flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) != 0u;
                bool previous2_present =
                    (flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT) != 0u;
                bool expected = (flags & (unsigned int)~AUDIO_BUNDLE_FLAG_MASK) == 0u &&
                                previous1_present == (previous1_len != 0u) &&
                                previous2_present == (previous2_len != 0u) &&
                                (!previous2_present || previous1_present) &&
                                ((flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) == 0u ||
                                 previous1_present) &&
                                ((flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) == 0u ||
                                 previous2_present);
                size_t wire_len = MESH_AUDIO_V2_FIXED_HEADER_SIZE + previous2_len +
                                  previous1_len + MESH_LC3_FRAME_BYTES;

                wire[3] = (uint8_t)flags;
                wire[7] = (uint8_t)previous1_len;
                assert(audio_bundle_parse(wire, wire_len, &parsed) == expected);
            }
        }
    }
}

static void test_boundary_active_flags(void)
{
    uint8_t frames[3][MESH_LC3_FRAME_BYTES] = {{0}};
    uint8_t wire[MESH_AUDIO_V2_FIXED_HEADER_SIZE + sizeof(frames)];
    uint8_t flags;
    size_t wire_len;
    audio_bundle_view_t input;
    audio_bundle_view_t parsed;

    for (flags = 0u; flags < 8u; ++flags) {
        uint8_t wire_flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                             AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
        if ((flags & 1u) != 0u) {
            wire_flags |= AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE;
        }
        if ((flags & 2u) != 0u) {
            wire_flags |= AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
        }
        if ((flags & 4u) != 0u) {
            wire_flags |= AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE;
        }
        input = make_bundle(frames[0], MESH_LC3_FRAME_BYTES,
                            frames[1], MESH_LC3_FRAME_BYTES,
                            frames[2], MESH_LC3_FRAME_BYTES,
                            UINT16_C(65535), wire_flags);
        assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
        assert(audio_bundle_parse(wire, wire_len, &parsed));
        assert(parsed.flags == wire_flags);
    }
}

static void test_malformed_wire(void)
{
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 1u] = {
        MESH_AUDIO_V2_CODEC_LC3, 20u, 3u, 0u, 0u, 1u, 48u, 0u
    };
    const size_t one_len = MESH_AUDIO_V2_FIXED_HEADER_SIZE + MESH_LC3_FRAME_BYTES;

    wire[0] = MESH_AUDIO_V2_CODEC_OPUS;
    expect_parse_rejected(wire, one_len);
    wire[0] = MESH_AUDIO_V2_CODEC_LC3;
    wire[1] = 10u;
    expect_parse_rejected(wire, one_len);
    wire[1] = 20u;
    wire[3] = 0x20u;
    expect_parse_rejected(wire, one_len);
    wire[3] = 0u;
    wire[6] = 0u;
    expect_parse_rejected(wire, 8u);
    wire[6] = 65u;
    expect_parse_rejected(wire, 73u);
    wire[6] = 47u;
    expect_parse_rejected(wire, one_len);
    wire[6] = 49u;
    expect_parse_rejected(wire, one_len + 1u);
    wire[6] = 48u;
    expect_parse_rejected(wire, one_len - 1u);
    wire[7] = 65u;
    expect_parse_rejected(wire, one_len + 65u);
    wire[7] = 47u;
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    expect_parse_rejected(wire, one_len + 47u);
    wire[7] = 49u;
    expect_parse_rejected(wire, one_len + 49u);
    wire[7] = 48u;
    expect_parse_rejected(wire, one_len + 47u);
    expect_parse_rejected(wire, one_len + 49u);
    wire[7] = 1u;
    expect_parse_rejected(wire, one_len + 1u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    expect_parse_rejected(wire, one_len);
    wire[7] = 0u;
    expect_parse_rejected(wire, one_len + 1u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    expect_parse_rejected(wire, one_len);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    expect_parse_rejected(wire, one_len + 1u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE;
    expect_parse_rejected(wire, one_len);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
               AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    wire[7] = 48u;
    expect_parse_rejected(wire, one_len + 48u + 47u);
    expect_parse_rejected(wire, one_len + 48u + 49u);
    expect_parse_rejected(wire, one_len + 48u);
    expect_parse_rejected(wire, one_len + 1u);
    expect_parse_rejected(wire, 201u);
    expect_parse_rejected(wire, 7u);
    assert(!audio_bundle_parse(NULL, one_len, &(audio_bundle_view_t){0}));
    assert(!audio_bundle_parse(wire, one_len, NULL));
}

static void test_encode_rejections_and_bounds(void)
{
    uint8_t frame[65] = {0};
    uint8_t wire[200];
    uint8_t before[200];
    size_t wire_len = 42u;
    audio_bundle_view_t input = make_bundle(NULL, 0u, NULL, 0u, frame,
                                            MESH_LC3_FRAME_BYTES, 1u, 0u);

    memset(wire, 0x5au, sizeof(wire));
    memcpy(before, wire, sizeof(wire));
    assert(!audio_bundle_encode(&input, wire, 8u, &wire_len));
    assert(wire_len == 0u && memcmp(wire, before, sizeof(wire)) == 0);
    input.codec = MESH_AUDIO_V2_CODEC_OPUS;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.codec = 0u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.codec = MESH_AUDIO_V2_CODEC_LC3;
    input.current_len = 0u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = 47u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = 49u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = MESH_LC3_FRAME_BYTES;
    input.current_data = NULL;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_data = frame;
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_data = frame;
    input.previous1_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_len = 47u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_len = 49u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_len = MESH_LC3_FRAME_BYTES;
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                  AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_data = frame;
    input.previous2_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_len = 47u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_len = 49u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_len = MESH_LC3_FRAME_BYTES;
    input.flags = 0x20u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    input.previous1_len = 0u;
    input.previous2_len = 0u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.flags = 0u;
    input.previous1_data = NULL;
    input.previous2_data = NULL;
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(!audio_bundle_encode(&input, NULL, sizeof(wire), &wire_len));
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), NULL));
    assert(!audio_bundle_encode(NULL, wire, sizeof(wire), &wire_len));
}

static void test_strip_two_to_one_to_zero(void)
{
    uint8_t previous2[MESH_LC3_FRAME_BYTES];
    uint8_t previous1[MESH_LC3_FRAME_BYTES];
    uint8_t current[MESH_LC3_FRAME_BYTES];
    uint8_t wire[200];
    uint8_t unchanged[MESH_AUDIO_V2_FIXED_HEADER_SIZE + MESH_LC3_FRAME_BYTES];
    size_t wire_len;
    audio_bundle_view_t parsed;
    audio_bundle_view_t input;

    fill(previous2, sizeof(previous2), 1u);
    fill(previous1, sizeof(previous1), 65u);
    fill(current, sizeof(current), 129u);
    input = make_bundle(previous2, sizeof(previous2), previous1, sizeof(previous1),
                        current, sizeof(current), 0u, AUDIO_BUNDLE_FLAG_MASK);
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));

    assert(audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == 104u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3 && wire[0] == parsed.codec);
    assert(parsed.flags == (AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                            AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                            AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE));
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);

    assert(audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == 56u && wire[7] == 0u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3 && wire[0] == parsed.codec);
    assert(parsed.flags == AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
    assert(parsed.previous1_data == NULL && parsed.previous1_len == 0u);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);

    memcpy(unchanged, wire, wire_len);
    assert(!audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == sizeof(unchanged));
    assert(memcmp(wire, unchanged, sizeof(unchanged)) == 0);
    assert(!audio_bundle_strip_oldest(wire, NULL));
    wire[0] = MESH_AUDIO_V2_CODEC_OPUS;
    assert(!audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == sizeof(unchanged));
}

/*
 * Deterministic mini-fuzz coverage below. No libc rand(): a fixed-seed
 * xorshift64 PRNG makes every run bit-identical.
 */

#define FUZZ_RANDOM_ITERATIONS   50000u
#define FUZZ_MUTATION_ITERATIONS 50000u
#define FUZZ_MAX_EXTRA_LEN       16u

static uint64_t fuzz_state = UINT64_C(0x9e3779b97f4a7c15);

static uint64_t fuzz_next(void)
{
    fuzz_state ^= fuzz_state << 13;
    fuzz_state ^= fuzz_state >> 7;
    fuzz_state ^= fuzz_state << 17;
    return fuzz_state;
}

/* Uniform-ish value in [0, bound); bound must be > 0. */
static size_t fuzz_below(size_t bound)
{
    return (size_t)(fuzz_next() % (uint64_t)bound);
}

/*
 * Invariants every successful parse must satisfy:
 *  - current and every present predecessor have exactly MESH_LC3_FRAME_BYTES
 *  - fixed header + sum of frame lengths equals input length
 *  - flags contain no bits outside AUDIO_BUNDLE_FLAG_MASK
 *  - PRESENT flags match non-zero frame lengths; previous2 implies previous1;
 *    ACTIVE flags only alongside the matching PRESENT flag
 *  - every returned data pointer lies inside [buf, buf + len); pointer + its
 *    length stays within buf + len; absent frames have NULL pointers
 */
static void assert_parsed_invariants(const uint8_t *buf, size_t len,
                                     const audio_bundle_view_t *parsed)
{
    const uint8_t *frame_start = buf + MESH_AUDIO_V2_FIXED_HEADER_SIZE;
    size_t total_frames =
        parsed->previous2_len + parsed->previous1_len + parsed->current_len;
    bool previous1_present =
        (parsed->flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) != 0u;
    bool previous2_present =
        (parsed->flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT) != 0u;

    assert(len >= MESH_AUDIO_V2_FIXED_HEADER_SIZE);
    assert(parsed->codec == MESH_AUDIO_V2_CODEC_LC3);
    assert(buf[0] == parsed->codec && buf[1] == MESH_AUDIO_V2_FRAME_MS);
    assert(parsed->current_len == MESH_LC3_FRAME_BYTES);
    assert(parsed->previous1_len == 0u || parsed->previous1_len == MESH_LC3_FRAME_BYTES);
    assert(parsed->previous2_len == 0u || parsed->previous2_len == MESH_LC3_FRAME_BYTES);
    assert(MESH_AUDIO_V2_FIXED_HEADER_SIZE + total_frames == len);

    assert((parsed->flags & (uint8_t)~AUDIO_BUNDLE_FLAG_MASK) == 0u);
    assert(previous1_present == (parsed->previous1_len != 0u));
    assert(previous2_present == (parsed->previous2_len != 0u));
    assert(!previous2_present || previous1_present);
    assert((parsed->flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) == 0u ||
           previous1_present);
    assert((parsed->flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) == 0u ||
           previous2_present);

    if (previous2_present) {
        assert(parsed->previous2_data == frame_start);
        assert(parsed->previous2_data + parsed->previous2_len <= buf + len);
    } else {
        assert(parsed->previous2_data == NULL);
    }
    if (previous1_present) {
        assert(parsed->previous1_data == frame_start + parsed->previous2_len);
        assert(parsed->previous1_data + parsed->previous1_len <= buf + len);
    } else {
        assert(parsed->previous1_data == NULL);
    }
    assert(parsed->current_data ==
           frame_start + parsed->previous2_len + parsed->previous1_len);
    assert(parsed->current_data + parsed->current_len <= buf + len);
}

/*
 * Corpus 1: pure random bytes at random lengths. Each buffer is heap
 * allocated at its exact length so address-sanitized runs catch any
 * out-of-bounds read inside audio_bundle_parse. A fraction of inputs get
 * plausible header bytes so the deeper validation paths are exercised too.
 */
static void test_fuzz_random_bytes(void)
{
    size_t iteration;

    for (iteration = 0u; iteration < FUZZ_RANDOM_ITERATIONS; ++iteration) {
        size_t len =
            fuzz_below(MESH_AUDIO_V2_MAX_BUNDLE_SIZE + FUZZ_MAX_EXTRA_LEN + 1u);
        uint8_t *buf = malloc(len != 0u ? len : 1u);
        audio_bundle_view_t parsed;
        size_t index;

        assert(buf != NULL);
        for (index = 0u; index < len; ++index) {
            buf[index] = (uint8_t)fuzz_next();
        }
        /* Bias some inputs past the cheap magic-byte checks. */
        if (len >= 2u && fuzz_below(4u) != 0u) {
            buf[0] = MESH_AUDIO_V2_CODEC_LC3;
            buf[1] = MESH_AUDIO_V2_FRAME_MS;
        }
        if (len >= 4u && fuzz_below(2u) != 0u) {
            buf[3] &= AUDIO_BUNDLE_FLAG_MASK;
        }
        if (len >= MESH_AUDIO_V2_FIXED_HEADER_SIZE && fuzz_below(2u) != 0u) {
            buf[6] = (uint8_t)fuzz_below(MESH_LC3_FRAME_BYTES + 2u);
            buf[7] = (uint8_t)fuzz_below(MESH_LC3_FRAME_BYTES + 2u);
        }

        if (audio_bundle_parse(buf, len, &parsed)) {
            assert_parsed_invariants(buf, len, &parsed);
        }
        free(buf);
    }
}

/*
 * Corpus 2: encode a valid randomized bundle (fixed-size frames with
 * absent previous frames, random flags/seq/stream id),
 * then apply 0-4 mutations (bit flip, byte set, truncated parse length,
 * extended parse length over trailing garbage) before parsing an
 * exact-length heap slice. Zero mutations must round-trip exactly.
 */
static void test_fuzz_mutated_bundles(void)
{
    uint8_t previous2[MESH_LC3_FRAME_BYTES];
    uint8_t previous1[MESH_LC3_FRAME_BYTES];
    uint8_t current[MESH_LC3_FRAME_BYTES];
    size_t iteration;

    for (iteration = 0u; iteration < FUZZ_MUTATION_ITERATIONS; ++iteration) {
        bool previous1_present = fuzz_below(4u) != 0u;
        bool previous2_present = previous1_present && fuzz_below(2u) != 0u;
        size_t current_len = MESH_LC3_FRAME_BYTES;
        size_t previous1_len =
            previous1_present ? MESH_LC3_FRAME_BYTES : 0u;
        size_t previous2_len =
            previous2_present ? MESH_LC3_FRAME_BYTES : 0u;
        uint16_t sequence = (uint16_t)fuzz_next();
        uint8_t flags = 0u;
        size_t encoded_len;
        size_t capacity;
        size_t parse_len;
        size_t mutation_count;
        size_t mutation;
        size_t index;
        size_t wire_len = 0u;
        uint8_t *buf;
        uint8_t *slice;
        audio_bundle_view_t input;
        audio_bundle_view_t parsed;
        bool ok;

        if (fuzz_below(2u) != 0u) {
            flags |= AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE;
        }
        if (previous1_present) {
            flags |= AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
            if (fuzz_below(2u) != 0u) {
                flags |= AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
            }
        }
        if (previous2_present) {
            flags |= AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
            if (fuzz_below(2u) != 0u) {
                flags |= AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE;
            }
        }

        for (index = 0u; index < previous2_len; ++index) {
            previous2[index] = (uint8_t)fuzz_next();
        }
        for (index = 0u; index < previous1_len; ++index) {
            previous1[index] = (uint8_t)fuzz_next();
        }
        for (index = 0u; index < current_len; ++index) {
            current[index] = (uint8_t)fuzz_next();
        }

        input = make_bundle(previous2_present ? previous2 : NULL, previous2_len,
                            previous1_present ? previous1 : NULL, previous1_len,
                            current, current_len, sequence, flags);
        input.stream_id = (uint8_t)fuzz_next();

        encoded_len = MESH_AUDIO_V2_FIXED_HEADER_SIZE + previous2_len +
                      previous1_len + current_len;
        capacity = encoded_len + fuzz_below(FUZZ_MAX_EXTRA_LEN + 1u);
        buf = malloc(capacity);
        assert(buf != NULL);
        for (index = encoded_len; index < capacity; ++index) {
            buf[index] = (uint8_t)fuzz_next(); /* trailing garbage */
        }
        assert(audio_bundle_encode(&input, buf, capacity, &wire_len));
        assert(wire_len == encoded_len);

        parse_len = encoded_len;
        mutation_count = fuzz_below(5u); /* 0 = pristine round-trip check */
        for (mutation = 0u; mutation < mutation_count; ++mutation) {
            switch (fuzz_below(4u)) {
            case 0u: /* bit flip */
                buf[fuzz_below(encoded_len)] ^=
                    (uint8_t)(1u << fuzz_below(8u));
                break;
            case 1u: /* byte set */
                buf[fuzz_below(encoded_len)] = (uint8_t)fuzz_next();
                break;
            case 2u: /* truncate claimed length */
                parse_len = fuzz_below(encoded_len + 1u);
                break;
            default: /* extend claimed length into trailing garbage */
                parse_len = encoded_len + fuzz_below(capacity - encoded_len + 1u);
                break;
            }
        }

        /* Exact-length slice so sanitizers see the true input boundary. */
        slice = malloc(parse_len != 0u ? parse_len : 1u);
        assert(slice != NULL);
        memcpy(slice, buf, parse_len);
        ok = audio_bundle_parse(slice, parse_len, &parsed);

        if (mutation_count == 0u) {
            assert(ok);
            assert(parsed.current_seq == sequence);
            assert(parsed.codec == input.codec);
            assert(parsed.flags == flags);
            assert(parsed.stream_id == input.stream_id);
            assert(parsed.previous2_len == previous2_len);
            assert(parsed.previous1_len == previous1_len);
            assert(parsed.current_len == current_len);
            assert(previous2_len == 0u ||
                   memcmp(parsed.previous2_data, previous2, previous2_len) == 0);
            assert(previous1_len == 0u ||
                   memcmp(parsed.previous1_data, previous1, previous1_len) == 0);
            assert(memcmp(parsed.current_data, current, current_len) == 0);
        }
        if (ok) {
            assert_parsed_invariants(slice, parse_len, &parsed);
        }
        free(slice);
        free(buf);
    }
}

int main(void)
{
    test_zero_predecessors();
    test_one_predecessor();
    test_max_and_pointer_order();
    test_boundary_active_flags();
    test_all_presence_flag_combinations();
    test_malformed_wire();
    test_encode_rejections_and_bounds();
    test_strip_two_to_one_to_zero();
    test_fuzz_random_bytes();
    test_fuzz_mutated_bundles();
    puts("audio_bundle tests passed");
    return 0;
}
