#include "shared/audio_bundle.h"

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
    return bundle;
}

static void expect_parse_rejected(const uint8_t *wire, size_t wire_len)
{
    audio_bundle_view_t parsed;
    assert(!audio_bundle_parse(wire, wire_len, &parsed));
}

static void test_zero_predecessors(void)
{
    uint8_t current[] = {0x11u, 0x22u};
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    size_t wire_len;
    audio_bundle_view_t parsed;
    audio_bundle_view_t input = make_bundle(NULL, 0u, NULL, 0u, current,
                                            sizeof(current), 7u,
                                            AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);

    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == 10u);
    assert(wire[0] == 1u && wire[1] == 20u && wire[2] == 9u);
    assert(wire[3] == AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
    assert(wire[4] == 0u && wire[5] == 7u);
    assert(wire[6] == sizeof(current) && wire[7] == 0u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(parsed.previous1_data == NULL && parsed.previous1_len == 0u);
    assert(parsed.current_data == wire + MESH_AUDIO_V2_FIXED_HEADER_SIZE);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_one_predecessor(void)
{
    uint8_t previous1[] = {0x31u, 0x32u};
    uint8_t current[] = {0x41u, 0x42u, 0x43u};
    uint8_t wire[13];
    size_t wire_len;
    audio_bundle_view_t parsed;
    uint8_t flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                    AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    audio_bundle_view_t input = make_bundle(NULL, 0u, previous1, sizeof(previous1),
                                            current, sizeof(current), 12u, flags);

    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == sizeof(wire) && wire[7] == sizeof(previous1));
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(parsed.previous1_data == wire + 8u);
    assert(parsed.current_data == wire + 8u + sizeof(previous1));
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_max_and_pointer_order(void)
{
    uint8_t previous2[64];
    uint8_t previous1[64];
    uint8_t current[64];
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    size_t wire_len;
    audio_bundle_view_t parsed;
    audio_bundle_view_t input;

    fill(previous2, sizeof(previous2), 0x10u);
    fill(previous1, sizeof(previous1), 0x50u);
    fill(current, sizeof(current), 0x90u);
    input = make_bundle(previous2, sizeof(previous2), previous1, sizeof(previous1),
                        current, sizeof(current), 0u, AUDIO_BUNDLE_FLAG_MASK);
    assert(!audio_bundle_encode(&input, wire, sizeof(wire) - 1u, &wire_len));
    assert(wire_len == 0u);
    assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    assert(wire_len == 200u && wire[6] == 64u && wire[7] == 64u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.current_seq == 0u);
    assert((uint16_t)(parsed.current_seq - 1u) == UINT16_C(65535));
    assert((uint16_t)(parsed.current_seq - 2u) == UINT16_C(65534));
    assert(parsed.previous2_data == wire + 8u);
    assert(parsed.previous1_data == wire + 8u + sizeof(previous2));
    assert(parsed.current_data == wire + 8u + sizeof(previous2) + sizeof(previous1));
    assert(parsed.previous2_len == 64u && parsed.previous1_len == 64u &&
           parsed.current_len == 64u);
    assert(memcmp(parsed.previous2_data, previous2, sizeof(previous2)) == 0);
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);
}

static void test_all_presence_flag_combinations(void)
{
    uint8_t wire[11] = {1u, 20u, 3u, 0u, 0u, 1u, 1u, 0u, 1u, 2u, 3u};
    unsigned int flags;
    size_t previous1_len;
    size_t previous2_len;

    for (flags = 0u; flags <= UINT8_MAX; ++flags) {
        for (previous1_len = 0u; previous1_len <= 1u; ++previous1_len) {
            for (previous2_len = 0u; previous2_len <= 1u; ++previous2_len) {
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
                                  previous1_len + 1u;

                wire[3] = (uint8_t)flags;
                wire[7] = (uint8_t)previous1_len;
                assert(audio_bundle_parse(wire, wire_len, &parsed) == expected);
            }
        }
    }
}

static void test_boundary_active_flags(void)
{
    uint8_t frames[3] = {1u, 2u, 3u};
    uint8_t wire[11];
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
        input = make_bundle(&frames[0], 1u, &frames[1], 1u, &frames[2], 1u,
                            UINT16_C(65535), wire_flags);
        assert(audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
        assert(audio_bundle_parse(wire, wire_len, &parsed));
        assert(parsed.flags == wire_flags);
    }
}

static void test_malformed_wire(void)
{
    uint8_t wire[201] = {1u, 20u, 3u, 0u, 0u, 1u, 1u, 0u, 0xaau};

    wire[0] = 2u;
    expect_parse_rejected(wire, 9u);
    wire[0] = 1u;
    wire[1] = 10u;
    expect_parse_rejected(wire, 9u);
    wire[1] = 20u;
    wire[3] = 0x20u;
    expect_parse_rejected(wire, 9u);
    wire[3] = 0u;
    wire[6] = 0u;
    expect_parse_rejected(wire, 8u);
    wire[6] = 65u;
    expect_parse_rejected(wire, 73u);
    wire[6] = 2u;
    expect_parse_rejected(wire, 9u);
    wire[6] = 1u;
    wire[7] = 65u;
    expect_parse_rejected(wire, 74u);
    wire[7] = 1u;
    expect_parse_rejected(wire, 10u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    expect_parse_rejected(wire, 9u);
    wire[7] = 0u;
    expect_parse_rejected(wire, 10u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;
    expect_parse_rejected(wire, 9u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    expect_parse_rejected(wire, 10u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE;
    expect_parse_rejected(wire, 9u);
    wire[3] = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
              AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    wire[7] = 1u;
    expect_parse_rejected(wire, 10u);
    wire[74] = 0xbbu;
    expect_parse_rejected(wire, 75u);
    expect_parse_rejected(wire, 201u);
    expect_parse_rejected(wire, 7u);
    assert(!audio_bundle_parse(NULL, 9u, &(audio_bundle_view_t){0}));
    assert(!audio_bundle_parse(wire, 9u, NULL));
}

static void test_encode_rejections_and_bounds(void)
{
    uint8_t frame[65] = {0};
    uint8_t wire[200];
    uint8_t before[200];
    size_t wire_len = 42u;
    audio_bundle_view_t input = make_bundle(NULL, 0u, NULL, 0u, frame, 1u, 1u, 0u);

    memset(wire, 0x5au, sizeof(wire));
    memcpy(before, wire, sizeof(wire));
    assert(!audio_bundle_encode(&input, wire, 8u, &wire_len));
    assert(wire_len == 0u && memcmp(wire, before, sizeof(wire)) == 0);
    input.current_len = 0u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_len = 1u;
    input.current_data = NULL;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.current_data = frame;
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_data = frame;
    input.previous1_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous1_len = 1u;
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                  AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_data = frame;
    input.previous2_len = 65u;
    assert(!audio_bundle_encode(&input, wire, sizeof(wire), &wire_len));
    input.previous2_len = 1u;
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
    uint8_t previous2[64];
    uint8_t previous1[64];
    uint8_t current[64];
    uint8_t wire[200];
    uint8_t unchanged[72];
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
    assert(wire_len == 136u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.flags == (AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                            AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                            AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE));
    assert(parsed.previous2_data == NULL && parsed.previous2_len == 0u);
    assert(memcmp(parsed.previous1_data, previous1, sizeof(previous1)) == 0);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);

    assert(audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == 72u && wire[7] == 0u);
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.flags == AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
    assert(parsed.previous1_data == NULL && parsed.previous1_len == 0u);
    assert(memcmp(parsed.current_data, current, sizeof(current)) == 0);

    memcpy(unchanged, wire, wire_len);
    assert(!audio_bundle_strip_oldest(wire, &wire_len));
    assert(wire_len == sizeof(unchanged));
    assert(memcmp(wire, unchanged, sizeof(unchanged)) == 0);
    assert(!audio_bundle_strip_oldest(wire, NULL));
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
    puts("audio_bundle tests passed");
    return 0;
}
