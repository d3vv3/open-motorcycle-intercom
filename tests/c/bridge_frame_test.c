#include "shared/bridge_frame.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t fuzz_state = UINT32_C(0x6d2b79f5);

static uint32_t fuzz_next(void)
{
    fuzz_state ^= fuzz_state << 13;
    fuzz_state ^= fuzz_state >> 17;
    fuzz_state ^= fuzz_state << 5;
    return fuzz_state;
}

static void test_crc_vector(void)
{
    static const uint8_t vector[] = "123456789";

    assert(bridge_frame_crc8(vector, sizeof(vector) - 1u) == UINT8_C(0xF4));
    assert(bridge_frame_crc8(NULL, 0u) == 0u);
    assert(bridge_frame_crc8(NULL, 1u) == 0u);
}

static void test_zero_payload_and_sequence_255(void)
{
    uint8_t frame[BRIDGE_FRAME_OVERHEAD];
    bridge_frame_view_t view;
    size_t len = bridge_frame_encode(frame, sizeof(frame), 0u, UINT8_MAX,
                                     UINT8_C(0x42), NULL, 0u);

    assert(len == sizeof(frame));
    assert(frame[0] == BRIDGE_FRAME_SYNC_BYTE && frame[1] == 2u);
    assert(frame[2] == UINT8_MAX && frame[3] == UINT8_C(0x42));
    assert(bridge_frame_decode(frame, len, 0u, &view) == BRIDGE_FRAME_OK);
    assert(view.seq == UINT8_MAX && view.type == UINT8_C(0x42));
    assert(view.payload == frame + 4u && view.payload_len == 0u);
}

static void test_max_payload(void)
{
    uint8_t payload[BRIDGE_FRAME_MAX_PAYLOAD];
    uint8_t frame[BRIDGE_FRAME_MAX_SIZE];
    bridge_frame_view_t view;
    size_t i;
    size_t len;

    for (i = 0u; i < sizeof(payload); ++i) {
        payload[i] = (uint8_t)i;
    }
    len = bridge_frame_encode(frame, sizeof(frame), BRIDGE_FRAME_MAX_PAYLOAD,
                              7u, 9u, payload, sizeof(payload));
    assert(len == sizeof(frame) && frame[1] == UINT8_MAX);
    assert(bridge_frame_decode(frame, len, BRIDGE_FRAME_MAX_PAYLOAD, &view) ==
           BRIDGE_FRAME_OK);
    assert(view.payload_len == sizeof(payload));
    assert(memcmp(view.payload, payload, sizeof(payload)) == 0);
}

static void test_encode_rejections(void)
{
    uint8_t dst[16];
    uint8_t before[sizeof(dst)];
    uint8_t payload = 1u;

    memset(dst, 0x5au, sizeof(dst));
    memcpy(before, dst, sizeof(dst));
    assert(bridge_frame_encode(dst, 5u, 1u, 0u, 0u, &payload, 1u) == 0u);
    assert(memcmp(dst, before, sizeof(dst)) == 0);
    assert(bridge_frame_encode(NULL, sizeof(dst), 1u, 0u, 0u, &payload, 1u) == 0u);
    assert(bridge_frame_encode(dst, sizeof(dst), 1u, 0u, 0u, NULL, 1u) == 0u);
    assert(bridge_frame_encode(dst, sizeof(dst), 0u, 0u, 0u, &payload, 1u) == 0u);
    assert(bridge_frame_encode(dst, sizeof(dst), BRIDGE_FRAME_MAX_PAYLOAD + 1u,
                               0u, 0u, NULL, 0u) == 0u);
    assert(bridge_frame_encode(dst, SIZE_MAX, BRIDGE_FRAME_MAX_PAYLOAD, 0u, 0u,
                               &payload, SIZE_MAX) == 0u);
}

static void test_decode_results(void)
{
    uint8_t frame[16] = {0};
    uint8_t payload[] = {1u, 2u, 3u};
    bridge_frame_view_t view = {(const uint8_t *)1, 99u, 1u, 1u};
    size_t len;

    assert(bridge_frame_decode(NULL, 0u, 3u, &view) == BRIDGE_FRAME_TRUNCATED);
    assert(view.payload == NULL && view.payload_len == 0u);
    assert(bridge_frame_decode(NULL, 5u, 3u, &view) == BRIDGE_FRAME_TRUNCATED);
    assert(bridge_frame_decode(frame, 1u, 3u, &view) == BRIDGE_FRAME_IDLE);
    frame[0] = 0x55u;
    assert(bridge_frame_decode(frame, 4u, 3u, &view) == BRIDGE_FRAME_TRUNCATED);
    assert(bridge_frame_decode(frame, 5u, 3u, &view) == BRIDGE_FRAME_BAD_SYNC);

    frame[0] = BRIDGE_FRAME_SYNC_BYTE;
    frame[1] = 0u;
    assert(bridge_frame_decode(frame, 5u, 3u, &view) == BRIDGE_FRAME_BAD_LENGTH);
    frame[1] = 1u;
    assert(bridge_frame_decode(frame, 5u, 3u, &view) == BRIDGE_FRAME_BAD_LENGTH);
    frame[1] = 6u;
    assert(bridge_frame_decode(frame, sizeof(frame), 3u, &view) ==
           BRIDGE_FRAME_BAD_LENGTH);
    assert(bridge_frame_decode(frame, sizeof(frame), BRIDGE_FRAME_MAX_PAYLOAD + 1u,
                               &view) == BRIDGE_FRAME_BAD_LENGTH);
    assert(view.payload == NULL && view.payload_len == 0u);
    assert(bridge_frame_decode(frame, sizeof(frame), 3u, NULL) ==
           BRIDGE_FRAME_BAD_LENGTH);

    len = bridge_frame_encode(frame, sizeof(frame), 3u, 4u, 5u,
                              payload, sizeof(payload));
    assert(len == 8u);
    assert(bridge_frame_decode(frame, 5u, 3u, &view) == BRIDGE_FRAME_TRUNCATED);
    assert(bridge_frame_decode(frame, len - 1u, 3u, &view) ==
           BRIDGE_FRAME_TRUNCATED);
    frame[len - 1u] ^= 1u;
    assert(bridge_frame_decode(frame, len, 3u, &view) == BRIDGE_FRAME_BAD_CRC);
}

static void test_trailing_padding(void)
{
    uint8_t frame[32];
    uint8_t payload[] = {0xa1u, 0xb2u};
    bridge_frame_view_t view;
    size_t len;

    memset(frame, 0xceu, sizeof(frame));
    len = bridge_frame_encode(frame, sizeof(frame), sizeof(payload), 8u, 3u,
                              payload, sizeof(payload));
    assert(len == 7u);
    assert(bridge_frame_decode(frame, sizeof(frame), sizeof(payload), &view) ==
           BRIDGE_FRAME_OK);
    assert(view.seq == 8u && view.type == 3u && view.payload_len == 2u);
    assert(memcmp(view.payload, payload, sizeof(payload)) == 0);
}

static void test_fuzz_exact_length_buffers(void)
{
    size_t iteration;

    for (iteration = 0u; iteration < 20000u; ++iteration) {
        size_t len = (size_t)(fuzz_next() % (BRIDGE_FRAME_MAX_SIZE + 16u));
        size_t max_payload = (size_t)(fuzz_next() % (BRIDGE_FRAME_MAX_PAYLOAD + 1u));
        uint8_t *buffer = malloc(len == 0u ? 1u : len);
        bridge_frame_view_t view;
        bridge_frame_decode_result_t result;
        size_t i;

        assert(buffer != NULL);
        for (i = 0u; i < len; ++i) {
            buffer[i] = (uint8_t)fuzz_next();
        }
        result = bridge_frame_decode(buffer, len, max_payload, &view);
        assert(result >= BRIDGE_FRAME_OK && result <= BRIDGE_FRAME_BAD_CRC);
        if (result == BRIDGE_FRAME_OK) {
            size_t declared_len = view.payload_len + BRIDGE_FRAME_OVERHEAD;
            uint8_t roundtrip[BRIDGE_FRAME_MAX_SIZE];
            size_t encoded;

            assert(len >= declared_len);
            assert(view.payload == buffer + 4u);
            assert(view.payload_len <= max_payload);
            encoded = bridge_frame_encode(roundtrip, sizeof(roundtrip), max_payload,
                                          view.seq, view.type, view.payload,
                                          view.payload_len);
            assert(encoded == declared_len);
            assert(memcmp(roundtrip, buffer, declared_len) == 0);
        } else {
            assert(view.payload == NULL && view.payload_len == 0u);
        }
        free(buffer);
    }
}

int main(void)
{
    test_crc_vector();
    test_zero_payload_and_sequence_255();
    test_max_payload();
    test_encode_rejections();
    test_decode_results();
    test_trailing_padding();
    test_fuzz_exact_length_buffers();
    puts("bridge_frame_test: PASS");
    return 0;
}
