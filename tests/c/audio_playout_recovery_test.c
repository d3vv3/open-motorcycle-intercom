#include "audio_packet_store.h"
#include "audio_pcm_resampler.h"
#include "shared/audio_bundle.h"

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

enum {
    FIRST_SEQUENCE = 96,
    FIRST_LOST_SEQUENCE = 100,
    BUNDLE_SEQUENCE = 102,
    BUNDLE_ARRIVAL_MS = 119
};

_Static_assert(AUDIO_PCM_RESAMPLER_TARGET_SAMPLES ==
                   4u * AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
               "playout reserve must contain four frames");
_Static_assert(AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES == 320u,
               "each render must produce one 20 ms PCM block");

static audio_packet_t make_packet(uint16_t sequence, bool active)
{
    audio_packet_t packet = {0};

    packet.data[0] = (uint8_t)sequence;
    packet.length = 1u;
    packet.sequence = sequence;
    packet.mode = AUDIO_PACKET_MODE_SEQUENCED;
    packet.active = active;
    return packet;
}

static void decode_and_push(audio_pcm_resampler_t *resampler,
                            const audio_packet_t *packet)
{
    int16_t pcm[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    size_t index;

    assert(packet->length == 1u);
    assert(packet->data[0] == (uint8_t)packet->sequence);
    for (index = 0u; index < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++index) {
        pcm[index] = (int16_t)(1000 + packet->sequence);
    }
    assert(!audio_pcm_resampler_push(resampler, pcm,
                                     AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES,
                                     packet->active)
                .rejected_push);
}

static audio_pcm_resampler_telemetry_t render_block(audio_pcm_resampler_t *resampler,
                                                     size_t upstream_samples,
                                                     bool expected_active)
{
    int16_t output[AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES];
    audio_pcm_resampler_telemetry_t status;
    size_t index;

    for (index = 0u; index < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++index) {
        output[index] = INT16_MIN;
    }
    status = audio_pcm_resampler_render(resampler, output, upstream_samples);
    assert(status.started);
    assert(!status.underrun);
    assert(status.audible_active == expected_active);
    assert(status.consumed_count >= AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES - 1u);
    assert(status.consumed_count <= AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 1u);
    for (index = 0u; index < AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES; ++index) {
        assert(output[index] != INT16_MIN);
    }
    return status;
}

static void prime_playout(audio_packet_store_t *store,
                          audio_pcm_resampler_t *resampler)
{
    uint16_t sequence;

    audio_packet_store_reset(store);
    audio_pcm_resampler_reset(resampler);
    for (sequence = FIRST_SEQUENCE; sequence < FIRST_LOST_SEQUENCE; ++sequence) {
        audio_packet_t packet = make_packet(sequence, sequence != FIRST_SEQUENCE + 2u);
        assert(audio_packet_store_push(store, &packet, 0u) ==
               AUDIO_PACKET_STORE_PUSH_OK);
        assert(audio_packet_store_depth(store) <= AUDIO_PACKET_STORE_CAPACITY);
    }
    for (sequence = FIRST_SEQUENCE; sequence < FIRST_LOST_SEQUENCE; ++sequence) {
        audio_packet_t packet = {0};
        assert(audio_packet_store_pop(store, 0u, &packet) ==
               AUDIO_PACKET_STORE_POP_PACKET);
        assert(packet.sequence == sequence);
        decode_and_push(resampler, &packet);
    }
    assert(audio_packet_store_depth(store) == 0u);
    assert(audio_pcm_resampler_depth(resampler) ==
           AUDIO_PCM_RESAMPLER_TARGET_SAMPLES);
}

static void push_redundant_bundle_oldest_first(audio_packet_store_t *store)
{
    const uint8_t previous2[] = {FIRST_LOST_SEQUENCE};
    const uint8_t previous1[] = {FIRST_LOST_SEQUENCE + 1u};
    const uint8_t current[] = {BUNDLE_SEQUENCE};
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    size_t wire_length;
    audio_bundle_view_t parsed;
    const audio_bundle_view_t bundle = {
        .previous1_data = previous1,
        .previous2_data = previous2,
        .current_data = current,
        .previous1_len = sizeof(previous1),
        .previous2_len = sizeof(previous2),
        .current_len = sizeof(current),
        .current_seq = BUNDLE_SEQUENCE,
        .stream_id = 1u,
        .flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                 AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE |
                 AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT |
                 AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE,
    };
    audio_packet_t packets[3];
    size_t index;

    assert(audio_bundle_encode(&bundle, wire, sizeof(wire), &wire_length));
    assert(audio_bundle_parse(wire, wire_length, &parsed));
    packets[0] = make_packet((uint16_t)(parsed.current_seq - 2u),
                             (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0u);
    packets[0].data[0] = parsed.previous2_data[0];
    packets[1] = make_packet((uint16_t)(parsed.current_seq - 1u),
                             (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0u);
    packets[1].data[0] = parsed.previous1_data[0];
    packets[2] = make_packet(parsed.current_seq,
                             (parsed.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) != 0u);
    packets[2].data[0] = parsed.current_data[0];

    for (index = 0u; index < 3u; ++index) {
        assert(packets[index].sequence == FIRST_LOST_SEQUENCE + index);
        assert(packets[index].active == (index != 0u));
        assert(audio_packet_store_push(store, &packets[index], BUNDLE_ARRIVAL_MS) ==
               AUDIO_PACKET_STORE_PUSH_OK);
        assert(audio_packet_store_depth(store) == index + 1u);
    }
}

static void test_redundant_bundle_recovers_two_losses(void)
{
    audio_packet_store_t store;
    audio_pcm_resampler_t resampler;
    audio_packet_t packet = {0};
    audio_pcm_resampler_telemetry_t status;
    audio_packet_store_pop_result_t result;
    size_t missing_count = 0u;
    size_t index;

    prime_playout(&store, &resampler);
    result = audio_packet_store_pop(&store, 80u, &packet);
    missing_count += result == AUDIO_PACKET_STORE_POP_MISSING ? 1u : 0u;
    assert(result == AUDIO_PACKET_STORE_POP_NOT_DUE);
    status = render_block(&resampler, 0u, true);
    assert(status.depth_before == AUDIO_PCM_RESAMPLER_TARGET_SAMPLES);

    result = audio_packet_store_pop(&store, 100u, &packet);
    missing_count += result == AUDIO_PACKET_STORE_POP_MISSING ? 1u : 0u;
    assert(result == AUDIO_PACKET_STORE_POP_NOT_DUE);
    status = render_block(&resampler, 0u, true);
    assert(status.depth_after >= 2u * AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES);

    push_redundant_bundle_oldest_first(&store);
    for (index = 0u; index < 3u; ++index) {
        result = audio_packet_store_pop(&store, BUNDLE_ARRIVAL_MS, &packet);
        missing_count += result == AUDIO_PACKET_STORE_POP_MISSING ? 1u : 0u;
        assert(result == AUDIO_PACKET_STORE_POP_PACKET);
        assert(packet.sequence == FIRST_LOST_SEQUENCE + index);
        assert(packet.active == (index != 0u));
        decode_and_push(&resampler, &packet);
    }
    assert(missing_count == 0u);
    assert(audio_packet_store_depth(&store) == 0u);
    assert(audio_pcm_resampler_depth(&resampler) <=
           AUDIO_PCM_RESAMPLER_TARGET_SAMPLES +
               AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES + 1u);
    status = render_block(&resampler, 0u, true);
    assert(status.depth_after <= AUDIO_PCM_RESAMPLER_TARGET_SAMPLES + 1u);
}

static void push_plc(audio_pcm_resampler_t *resampler, uint16_t missing_sequence)
{
    audio_packet_t plc = make_packet(missing_sequence, true);

    decode_and_push(resampler, &plc);
}

static void test_unrecovered_losses_use_two_plc_blocks(void)
{
    audio_packet_store_t store;
    audio_pcm_resampler_t resampler;
    audio_packet_t packet = {0};
    uint16_t sequence;
    size_t missing_count = 0u;

    prime_playout(&store, &resampler);
    for (sequence = FIRST_LOST_SEQUENCE;
         sequence < FIRST_LOST_SEQUENCE + 2u; ++sequence) {
        uint64_t deadline_ms = 120u +
                               (uint64_t)(sequence - FIRST_LOST_SEQUENCE) *
                                   AUDIO_PACKET_STORE_FRAME_MS;
        audio_packet_store_pop_result_t result =
            audio_packet_store_pop(&store, deadline_ms, &packet);

        assert(result == AUDIO_PACKET_STORE_POP_MISSING);
        assert(store.expected_sequence == (uint16_t)(sequence + 1u));
        ++missing_count;
        push_plc(&resampler, sequence);
        (void)render_block(&resampler, 0u, true);
        assert(audio_pcm_resampler_depth(&resampler) <=
               AUDIO_PCM_RESAMPLER_TARGET_SAMPLES + 1u);
    }
    assert(missing_count == 2u);
    assert(audio_pcm_resampler_depth(&resampler) >=
           3u * AUDIO_PCM_RESAMPLER_BLOCK_SAMPLES);
}

int main(void)
{
    test_redundant_bundle_recovers_two_losses();
    test_unrecovered_losses_use_two_plc_blocks();
    puts("audio playout recovery tests passed");
    return 0;
}
