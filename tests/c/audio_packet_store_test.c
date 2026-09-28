#include "audio_packet_store.h"

#include <assert.h>
#include <stdio.h>

static audio_packet_t packet(uint16_t sequence, audio_packet_mode_t mode, bool active)
{
    audio_packet_t value = {0};
    value.data[0] = (uint8_t)sequence;
    value.data[1] = (uint8_t)(sequence >> 8);
    value.length = 2u;
    value.sequence = sequence;
    value.mode = mode;
    value.active = active;
    value.hop_count = 1u; /* Existing recovery/deadline cases exercise relay bounds. */
    value.received_us = (uint64_t)sequence * 1000u + 17u;
    return value;
}

static void push_ok(audio_packet_store_t *store, uint16_t sequence, uint64_t now_ms)
{
    audio_packet_t value = packet(sequence, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(store, &value, now_ms) == AUDIO_PACKET_STORE_PUSH_OK);
}

static void expect_packet(audio_packet_store_t *store, uint64_t now_ms, uint16_t sequence)
{
    audio_packet_t output = {0};
    assert(audio_packet_store_pop(store, now_ms, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == sequence);
    assert(output.length == 2u);
    assert(output.received_us == (uint64_t)sequence * 1000u + 17u);
}

static void test_sequential_and_prefill(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    push_ok(&store, 10u, 0u);
    push_ok(&store, 11u, 0u);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    push_ok(&store, 12u, 0u);
    assert(audio_packet_store_depth(&store) == 3u);
    expect_packet(&store, 0u, 10u);
    expect_packet(&store, 0u, 11u);
    expect_packet(&store, 0u, 12u);
    assert(audio_packet_store_depth(&store) == 0u);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 59u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 79u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 99u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 100u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
}

static void test_prefill_timeout(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    push_ok(&store, 90u, 100u);
    assert(audio_packet_store_pop(&store, 159u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    expect_packet(&store, 160u, 90u);
}

static void test_reorder_and_missing_deadline(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    push_ok(&store, 100u, 0u);
    push_ok(&store, 102u, 0u);
    push_ok(&store, 101u, 0u);
    expect_packet(&store, 0u, 100u);
    expect_packet(&store, 0u, 101u);
    expect_packet(&store, 0u, 102u);

    audio_packet_store_reset(&store);
    push_ok(&store, 100u, 0u);
    push_ok(&store, 103u, 0u);
    push_ok(&store, 104u, 0u);
    expect_packet(&store, 0u, 100u);
    assert(audio_packet_store_pop(&store, 20u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    value = packet(101u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 40u) == AUDIO_PACKET_STORE_PUSH_OK);
    expect_packet(&store, 40u, 101u);
    assert(audio_packet_store_pop(&store, 79u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    value = packet(102u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 79u) == AUDIO_PACKET_STORE_PUSH_OK);
    expect_packet(&store, 79u, 102u);

    audio_packet_store_reset(&store);
    push_ok(&store, 100u, 0u);
    push_ok(&store, 102u, 0u);
    push_ok(&store, 103u, 0u);
    expect_packet(&store, 0u, 100u);
    assert(audio_packet_store_pop(&store, 20u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 59u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    expect_packet(&store, 60u, 102u);
}

static void test_two_frame_loss_recovery(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};

    audio_packet_store_reset(&store);
    push_ok(&store, 100u, 0u);
    push_ok(&store, 103u, 0u);
    push_ok(&store, 104u, 0u);
    expect_packet(&store, 0u, 100u);
    assert(audio_packet_store_pop(&store, 59u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 102u);
    assert(audio_packet_store_pop(&store, 79u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 80u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 103u);
    expect_packet(&store, 80u, 103u);
}

static void test_duplicate_and_late(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_store_reset(&store);
    value = packet(7u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_DUPLICATE);
    push_ok(&store, 8u, 0u);
    push_ok(&store, 9u, 0u);
    expect_packet(&store, 0u, 7u);
    assert(audio_packet_store_push(&store, &value, 1u) == AUDIO_PACKET_STORE_PUSH_DUPLICATE);

    value = packet(6u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 1u) == AUDIO_PACKET_STORE_PUSH_LATE);
    expect_packet(&store, 20u, 8u);
    value = packet(7u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 21u) == AUDIO_PACKET_STORE_PUSH_DUPLICATE);
}

static void test_stale_slot_collision(void)
{
    audio_packet_store_t store;
    audio_packet_t value;

    audio_packet_store_reset(&store);
    push_ok(&store, 100u, 0u);
    push_ok(&store, 112u, 0u);
    push_ok(&store, 101u, 0u);
    expect_packet(&store, 0u, 100u);
    value = packet(96u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 1u) == AUDIO_PACKET_STORE_PUSH_LATE);
    assert(audio_packet_store_depth(&store) == 2u);

    audio_packet_store_reset(&store);
    push_ok(&store, UINT16_C(65535), 0u);
    push_ok(&store, 12u, 0u);
    push_ok(&store, 0u, 0u);
    expect_packet(&store, 0u, UINT16_C(65535));
    expect_packet(&store, 20u, 0u);
    value = packet(UINT16_C(65532), AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 21u) == AUDIO_PACKET_STORE_PUSH_LATE);
    assert(audio_packet_store_depth(&store) == 1u);
}

static void test_future_and_full(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    uint16_t sequence;
    audio_packet_store_reset(&store);
    push_ok(&store, 1000u, 0u);
    value = packet(1016u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_FUTURE);

    for (sequence = 1001u; sequence < 1016u; ++sequence) {
        push_ok(&store, sequence, 0u);
    }
    assert(audio_packet_store_depth(&store) == AUDIO_PACKET_STORE_CAPACITY);
    value = packet(1000u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_DUPLICATE);
    value = packet(1016u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_FUTURE);
    assert(audio_packet_store_depth(&store) == AUDIO_PACKET_STORE_CAPACITY);
    for (sequence = 1000u; sequence < 1016u; ++sequence) {
        expect_packet(&store, 0u, sequence);
    }
    assert(audio_packet_store_depth(&store) == 0u);

    audio_packet_store_reset(&store);
    for (sequence = 0u; sequence < AUDIO_PACKET_STORE_CAPACITY; ++sequence) {
        value = packet(sequence, AUDIO_PACKET_MODE_ARRIVAL_ORDER, true);
        assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    }
    value = packet(99u, AUDIO_PACKET_MODE_ARRIVAL_ORDER, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_FULL);
    assert(audio_packet_store_depth(&store) == AUDIO_PACKET_STORE_CAPACITY);
    expect_packet(&store, 0u, 0u);
}

static void test_wrap(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    uint16_t sequence = UINT16_C(65528);
    size_t index;

    audio_packet_store_reset(&store);
    push_ok(&store, sequence, 0u);
    value = packet(8u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_FUTURE);
    sequence = (uint16_t)(sequence + 1u);
    for (index = 1u; index < AUDIO_PACKET_STORE_CAPACITY; ++index) {
        push_ok(&store, sequence, 0u);
        sequence = (uint16_t)(sequence + 1u);
    }
    assert(audio_packet_store_depth(&store) == AUDIO_PACKET_STORE_CAPACITY);
    sequence = UINT16_C(65528);
    for (index = 0u; index < AUDIO_PACKET_STORE_CAPACITY; ++index) {
        expect_packet(&store, 0u, sequence);
        sequence = (uint16_t)(sequence + 1u);
    }
    assert(audio_packet_store_depth(&store) == 0u);
}

static void test_overdue_events(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    push_ok(&store, 30u, 0u);
    push_ok(&store, 33u, 0u);
    push_ok(&store, 36u, 0u);
    expect_packet(&store, 0u, 30u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 32u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 33u);
    expect_packet(&store, 140u, 33u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 35u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 36u);
    expect_packet(&store, 140u, 36u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
}

static void test_dtx_inactive_payload_and_active_restart_loss(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};

    audio_packet_store_reset(&store);
    value = packet(50u, AUDIO_PACKET_MODE_SEQUENCED, false);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    push_ok(&store, 52u, 0u);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 50u);
    assert(!output.active);
    assert(output.length == 2u);
    assert(output.data[0] == 50u);
    assert(store.expected_sequence == 51u);
    assert(store.dtx_idle);
    assert(audio_packet_store_pop(&store, 80u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 119u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 120u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 52u);
    assert(store.dtx_idle);
    assert(audio_packet_store_pop(&store, 120u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 52u);
    assert(output.active);
    assert(store.expected_sequence == 53u);
    assert(!store.dtx_idle);
}

static void test_dtx_lost_comfort_update(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};

    audio_packet_store_reset(&store);
    value = packet(60u, AUDIO_PACKET_MODE_SEQUENCED, false);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 60u && !output.active);
    assert(store.expected_sequence == 61u);
    assert(audio_packet_store_pop(&store, 80u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 120u, &output) == AUDIO_PACKET_STORE_POP_DTX_IDLE);
    assert(store.expected_sequence == 61u);

    value = packet(62u, AUDIO_PACKET_MODE_SEQUENCED, false);
    assert(audio_packet_store_push(&store, &value, 121u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&store, 120u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(store.expected_sequence == 62u);
    assert(audio_packet_store_pop(&store, 140u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 62u && !output.active);
    assert(store.expected_sequence == 63u);
    assert(store.dtx_idle);
}

static void test_dtx_buffered_early_and_empty_deadline(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    uint16_t sequence;

    audio_packet_store_reset(&store);
    for (sequence = 70u; sequence < 73u; ++sequence) {
        value = packet(sequence, AUDIO_PACKET_MODE_SEQUENCED, false);
        assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    }
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 70u && !output.active);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 71u && !output.active);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 72u && !output.active);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 59u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 99u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 100u, &output) == AUDIO_PACKET_STORE_POP_DTX_IDLE);
    assert(store.expected_sequence == 73u);
}

static void test_lost_dtx_transition_active_resume(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    uint64_t deadline;

    audio_packet_store_reset(&store);
    push_ok(&store, 50u, 0u);
    expect_packet(&store, 60u, 50u);
    for (deadline = 120u; deadline <= 200u; deadline += AUDIO_PACKET_STORE_FRAME_MS) {
        assert(audio_packet_store_pop(&store, deadline, &output) ==
               AUDIO_PACKET_STORE_POP_MISSING);
    }
    assert(store.expected_sequence == 56u);
    assert(store.sequence_uncertain);
    assert(audio_packet_store_pop(&store, 220u, &output) == AUDIO_PACKET_STORE_POP_DTX_IDLE);
    assert(store.expected_sequence == 56u);

    value = packet(49u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&store, &value, 221u) == AUDIO_PACKET_STORE_PUSH_LATE);
    assert(store.expected_sequence == 56u);
    assert(store.sequence_uncertain);
    push_ok(&store, 51u, 221u);
    assert(store.expected_sequence == 51u);
    expect_packet(&store, 221u, 51u);
    assert(store.expected_sequence == 52u);
    assert(!store.dtx_idle);
    assert(!store.sequence_uncertain);
    assert(store.consecutive_empty_missing == 0u);
}

static void test_lost_dtx_transition_comfort_resume(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    uint64_t deadline;

    audio_packet_store_reset(&store);
    push_ok(&store, 50u, 0u);
    expect_packet(&store, 60u, 50u);
    for (deadline = 120u; deadline <= 200u; deadline += AUDIO_PACKET_STORE_FRAME_MS) {
        assert(audio_packet_store_pop(&store, deadline, &output) ==
               AUDIO_PACKET_STORE_POP_MISSING);
    }
    value = packet(51u, AUDIO_PACKET_MODE_SEQUENCED, false);
    assert(audio_packet_store_push(&store, &value, 201u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&store, 201u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 51u && !output.active);
    assert(store.expected_sequence == 52u);
    assert(store.dtx_idle);
    assert(!store.sequence_uncertain);
}

static void test_buffered_six_frame_burst_loss(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};
    uint16_t missing_sequence;
    uint64_t deadline = 120u;

    audio_packet_store_reset(&store);
    push_ok(&store, 50u, 0u);
    push_ok(&store, 56u, 0u);
    expect_packet(&store, 60u, 50u);
    for (missing_sequence = 51u; missing_sequence < 56u; ++missing_sequence) {
        assert(store.expected_sequence == missing_sequence);
        assert(audio_packet_store_pop(&store, deadline, &output) ==
               AUDIO_PACKET_STORE_POP_MISSING);
        assert(!store.dtx_idle);
        assert(!store.sequence_uncertain);
        deadline += AUDIO_PACKET_STORE_FRAME_MS;
    }
    expect_packet(&store, 200u, 56u);
    assert(store.expected_sequence == 57u);
}

static void test_lost_dtx_transition_wrap(void)
{
    audio_packet_store_t store;
    audio_packet_t output = {0};
    uint64_t deadline;

    audio_packet_store_reset(&store);
    push_ok(&store, UINT16_C(65534), 0u);
    expect_packet(&store, 60u, UINT16_C(65534));
    for (deadline = 120u; deadline <= 200u; deadline += AUDIO_PACKET_STORE_FRAME_MS) {
        assert(audio_packet_store_pop(&store, deadline, &output) ==
               AUDIO_PACKET_STORE_POP_MISSING);
    }
    assert(store.expected_sequence == 4u);
    push_ok(&store, UINT16_C(65535), 201u);
    expect_packet(&store, 201u, UINT16_C(65535));
    assert(store.expected_sequence == 0u);
    assert(!store.sequence_uncertain);
}

static void test_arrival_order(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    value = packet(9u, AUDIO_PACKET_MODE_ARRIVAL_ORDER, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    value = packet(7u, AUDIO_PACKET_MODE_ARRIVAL_ORDER, true);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    value = packet(8u, AUDIO_PACKET_MODE_ARRIVAL_ORDER, false);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    expect_packet(&store, 0u, 9u);
    expect_packet(&store, 0u, 7u);
    assert(audio_packet_store_pop(&store, 0u, &output) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(output.sequence == 8u && !output.active);
    assert(audio_packet_store_pop(&store, 59u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 60u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 100u, &output) == AUDIO_PACKET_STORE_POP_DTX_IDLE);
}

static void test_mode_mismatch_reset_and_lengths(void)
{
    audio_packet_store_t store;
    audio_packet_t value;
    audio_packet_t output = {0};
    audio_packet_store_reset(&store);
    value = packet(1u, AUDIO_PACKET_MODE_SEQUENCED, true);
    value.length = 0u;
    assert(audio_packet_store_push(&store, &value, 0u) ==
           AUDIO_PACKET_STORE_PUSH_INVALID_LENGTH);
    value.length = AUDIO_PACKET_MAX_SIZE + 1u;
    assert(audio_packet_store_push(&store, &value, 0u) ==
           AUDIO_PACKET_STORE_PUSH_INVALID_LENGTH);
    assert(audio_packet_store_depth(&store) == 0u);

    value.length = 2u;
    value.hop_count = 2u;
    assert(audio_packet_store_push(&store, &value, 0u) ==
           AUDIO_PACKET_STORE_PUSH_INVALID_ARGUMENT);
    assert(audio_packet_store_depth(&store) == 0u);
    value.hop_count = 1u;

    value.length = AUDIO_PACKET_MAX_SIZE;
    assert(audio_packet_store_push(&store, &value, 5u) == AUDIO_PACKET_STORE_PUSH_OK);
    value = packet(2u, AUDIO_PACKET_MODE_ARRIVAL_ORDER, true);
    assert(audio_packet_store_push(&store, &value, 5u) ==
           AUDIO_PACKET_STORE_PUSH_MODE_MISMATCH);
    assert(audio_packet_store_depth(&store) == 1u);

    audio_packet_store_reset(&store);
    assert(audio_packet_store_depth(&store) == 0u);
    assert(audio_packet_store_pop(&store, 100u, &output) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_push(NULL, &value, 0u) ==
           AUDIO_PACKET_STORE_PUSH_INVALID_ARGUMENT);
    assert(audio_packet_store_push(&store, NULL, 0u) ==
           AUDIO_PACKET_STORE_PUSH_INVALID_ARGUMENT);
    assert(audio_packet_store_depth(NULL) == 0u);
    audio_packet_store_reset(NULL);
}

static void test_independent_path_policies(void)
{
    audio_packet_store_t direct, relay;
    audio_packet_t out = {0};
    audio_packet_t value;
    audio_packet_store_reset(&direct);
    audio_packet_store_reset(&relay);

    value = packet(10u, AUDIO_PACKET_MODE_SEQUENCED, true);
    value.hop_count = 0u;
    assert(audio_packet_store_push(&direct, &value, 100u) == AUDIO_PACKET_STORE_PUSH_OK);
    value = packet(20u, AUDIO_PACKET_MODE_SEQUENCED, true);
    assert(audio_packet_store_push(&relay, &value, 100u) == AUDIO_PACKET_STORE_PUSH_OK);
    value = packet(11u, AUDIO_PACKET_MODE_SEQUENCED, true);
    value.hop_count = 0u;
    assert(audio_packet_store_push(&direct, &value, 101u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&relay, 101u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&direct, 101u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(out.sequence == 10u && out.hop_count == 0u);
    assert(audio_packet_store_pop(&direct, 101u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(out.sequence == 11u);
    assert(audio_packet_store_pop(&direct, 160u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&direct, 161u, &out) == AUDIO_PACKET_STORE_POP_MISSING);
    assert(audio_packet_store_pop(&relay, 159u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&relay, 160u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(out.sequence == 20u && out.hop_count == 1u);
    assert(audio_packet_store_pop(&relay, 219u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&relay, 220u, &out) == AUDIO_PACKET_STORE_POP_MISSING);

    /* A switched path retains its sequence/decode continuity and latched grace;
     * the other source's timing is untouched. Rebuffer/reset chooses the new policy. */
    value = packet(13u, AUDIO_PACKET_MODE_SEQUENCED, true);
    value.hop_count = 1u;
    assert(audio_packet_store_push(&direct, &value, 201u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(direct.policy_hop_count == 0u && direct.expected_sequence == 13u);
    assert(audio_packet_store_pop(&direct, 201u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(out.hop_count == 1u);
    assert(relay.policy_hop_count == 1u && relay.expected_sequence == 22u);
    audio_packet_store_reset(&direct);
    assert(audio_packet_store_push(&direct, &value, 202u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(direct.policy_hop_count == 1u && direct.prefill_deadline_ms == 262u);
}

static void test_direct_prefill_timeout_and_late_recovery(void)
{
    audio_packet_store_t store;
    audio_packet_t value = packet(30u, AUDIO_PACKET_MODE_SEQUENCED, true);
    audio_packet_t out = {0};
    value.hop_count = 0u;
    audio_packet_store_reset(&store);
    assert(audio_packet_store_push(&store, &value, 0u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&store, 39u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 40u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(audio_packet_store_pop(&store, 59u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    value.sequence = 31u;
    assert(audio_packet_store_push(&store, &value, 59u) == AUDIO_PACKET_STORE_PUSH_OK);
    assert(audio_packet_store_pop(&store, 59u, &out) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(out.sequence == 31u);
    assert(audio_packet_store_pop(&store, 99u, &out) == AUDIO_PACKET_STORE_POP_NOT_DUE);
    assert(audio_packet_store_pop(&store, 100u, &out) == AUDIO_PACKET_STORE_POP_MISSING);
    value.sequence = 32u;
    assert(audio_packet_store_push(&store, &value, 100u) == AUDIO_PACKET_STORE_PUSH_LATE);
}

int main(void)
{
    test_sequential_and_prefill();
    test_prefill_timeout();
    test_reorder_and_missing_deadline();
    test_two_frame_loss_recovery();
    test_duplicate_and_late();
    test_stale_slot_collision();
    test_future_and_full();
    test_wrap();
    test_overdue_events();
    test_dtx_inactive_payload_and_active_restart_loss();
    test_dtx_lost_comfort_update();
    test_dtx_buffered_early_and_empty_deadline();
    test_lost_dtx_transition_active_resume();
    test_lost_dtx_transition_comfort_resume();
    test_buffered_six_frame_burst_loss();
    test_lost_dtx_transition_wrap();
    test_arrival_order();
    test_mode_mismatch_reset_and_lengths();
    test_independent_path_policies();
    test_direct_prefill_timeout_and_late_recovery();
    puts("audio_packet_store tests passed");
    return 0;
}
