#include "audio_packet_store.h"
#include "audio_pcm_resampler.h"
#include "shared/audio_bundle.h"
#include "shared/audio_tx_cache.h"

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
_Static_assert(AUDIO_TX_QUIET_SEQUENCE_SLOTS == AUDIO_PACKET_STORE_EMPTY_MISSING_LIMIT,
               "TX quiet sequence cap must match RX DTX inference threshold");

static audio_packet_t make_packet(uint16_t sequence, bool active)
{
    audio_packet_t packet = {0};

    packet.data[0] = (uint8_t)sequence;
    packet.length = MESH_LC3_FRAME_BYTES;
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

    assert(packet->length == MESH_LC3_FRAME_BYTES);
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
    uint8_t previous2[MESH_LC3_FRAME_BYTES] = {FIRST_LOST_SEQUENCE};
    uint8_t previous1[MESH_LC3_FRAME_BYTES] = {FIRST_LOST_SEQUENCE + 1u};
    uint8_t current[MESH_LC3_FRAME_BYTES] = {BUNDLE_SEQUENCE};
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
        .codec = MESH_AUDIO_V2_CODEC_LC3,
        .flags = AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                 AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE |
                 AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT |
                 AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE,
    };
    audio_packet_t packets[3];
    size_t index;

    assert(audio_bundle_encode(&bundle, wire, sizeof(wire), &wire_length));
    assert(audio_bundle_parse(wire, wire_length, &parsed));
    assert(parsed.codec == MESH_AUDIO_V2_CODEC_LC3);
    assert(parsed.current_len == MESH_LC3_FRAME_BYTES);
    packets[0] = make_packet((uint16_t)(parsed.current_seq - 2u),
                             (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0u);
    memcpy(packets[0].data, parsed.previous2_data, MESH_LC3_FRAME_BYTES);
    packets[1] = make_packet((uint16_t)(parsed.current_seq - 1u),
                             (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0u);
    memcpy(packets[1].data, parsed.previous1_data, MESH_LC3_FRAME_BYTES);
    packets[2] = make_packet(parsed.current_seq,
                             (parsed.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) != 0u);
    memcpy(packets[2].data, parsed.current_data, MESH_LC3_FRAME_BYTES);

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

static void send_frame(audio_packet_store_t *store, audio_tx_cache_t *cache,
                       uint16_t *next_seq, uint64_t arrival_ms, bool expect_previous)
{
    uint8_t payload[MESH_LC3_FRAME_BYTES] = {0};
    uint8_t wire[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    uint16_t previous_len = 0u;
    uint16_t seq = (*next_seq)++;
    const uint8_t *previous = audio_tx_cache_previous(cache, seq, &previous_len);
    size_t wire_len = 0u;
    audio_bundle_view_t parsed;
    audio_packet_t packet;
    audio_bundle_view_t bundle = {
        .previous1_data = previous,
        .previous1_len = previous_len,
        .current_data = payload,
        .current_len = sizeof(payload),
        .current_seq = seq,
        .stream_id = 7u,
        .flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                 (previous != NULL ? AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                                     AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE : 0u),
        .codec = MESH_AUDIO_V2_CODEC_LC3,
    };

    assert((previous != NULL) == expect_previous);
    payload[0] = (uint8_t)seq;
    assert(audio_bundle_encode(&bundle, wire, sizeof(wire), &wire_len));
    assert(audio_bundle_parse(wire, wire_len, &parsed));
    assert(parsed.stream_id == 7u);
    assert(parsed.current_seq == seq);
    assert(((parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) != 0u) == expect_previous);
    packet = make_packet(parsed.current_seq, true);
    memcpy(packet.data, parsed.current_data, parsed.current_len);
    assert(audio_packet_store_push(store, &packet, arrival_ms) == AUDIO_PACKET_STORE_PUSH_OK);
    audio_tx_cache_store(cache, payload, sizeof(payload), true, seq, true);
}

static void test_vox_idle_resume(uint16_t first_seq, unsigned idle_frames)
{
    audio_packet_store_t store;
    audio_tx_cache_t cache;
    audio_packet_t packet = {0};
    uint16_t next_seq = first_seq;
    unsigned sequence_slots = idle_frames < AUDIO_TX_QUIET_SEQUENCE_SLOTS
                                  ? idle_frames : AUDIO_TX_QUIET_SEQUENCE_SLOTS;
    unsigned index;

    audio_packet_store_reset(&store);
    audio_tx_cache_reset(&cache);
    for (index = 0u; index < 3u; ++index) {
        send_frame(&store, &cache, &next_seq, 0u, index != 0u);
    }
    for (index = 0u; index < 3u; ++index) {
        assert(audio_packet_store_pop(&store, index * 20u, &packet) ==
               AUDIO_PACKET_STORE_POP_PACKET);
        assert(packet.sequence == (uint16_t)(first_seq + index));
    }
    for (index = 0u; index < idle_frames; ++index) {
        audio_tx_cache_skip_frame(&cache, &next_seq);
    }
    assert(next_seq == (uint16_t)(first_seq + 3u + sequence_slots));
    /* Past the cap, another quiet callback must still leave the cache invalid. */
    if (idle_frames > AUDIO_TX_QUIET_SEQUENCE_SLOTS) {
        uint16_t previous_len = 1u;
        assert(audio_tx_cache_previous(&cache, next_seq, &previous_len) == NULL);
        assert(previous_len == 0u);
    }
    if (idle_frames < AUDIO_PACKET_STORE_EMPTY_MISSING_LIMIT) {
        send_frame(&store, &cache, &next_seq, 80u, false);
        for (index = 0u; index < idle_frames; ++index) {
            assert(audio_packet_store_pop(&store, 100u + index * 20u, &packet) ==
                   AUDIO_PACKET_STORE_POP_MISSING);
        }
        assert(!store.dtx_idle);
        assert(audio_packet_store_pop(&store, 100u + idle_frames * 20u, &packet) ==
               AUDIO_PACKET_STORE_POP_PACKET);
    } else {
        for (index = 0u; index < AUDIO_PACKET_STORE_EMPTY_MISSING_LIMIT; ++index) {
            assert(audio_packet_store_pop(&store, 100u + index * 20u, &packet) ==
                   AUDIO_PACKET_STORE_POP_MISSING);
        }
        assert(store.dtx_idle);
        assert(audio_packet_store_pop(&store, 200u, &packet) ==
               AUDIO_PACKET_STORE_POP_DTX_IDLE);
        send_frame(&store, &cache, &next_seq, 210u, false);
        assert(audio_packet_store_pop(&store, 220u, &packet) ==
               AUDIO_PACKET_STORE_POP_PACKET);
    }
    assert(packet.sequence == (uint16_t)(first_seq + 3u + sequence_slots));
    assert(packet.data[0] == (uint8_t)packet.sequence);
    assert(packet.active);
    assert(!store.dtx_idle);
    assert(audio_packet_store_depth(&store) == 0u);
    send_frame(&store, &cache, &next_seq, 260u, true);
    assert(audio_packet_store_pop(&store, 260u, &packet) == AUDIO_PACKET_STORE_POP_PACKET);
    assert(packet.sequence == (uint16_t)(first_seq + 4u + sequence_slots));
}

int main(void)
{
    test_redundant_bundle_recovers_two_losses();
    test_unrecovered_losses_use_two_plc_blocks();
    test_vox_idle_resume(100u, 2u);
    test_vox_idle_resume(300u, AUDIO_TX_QUIET_SEQUENCE_SLOTS);
    test_vox_idle_resume(300u, 6u);
    test_vox_idle_resume(65533u, 2u);
    test_vox_idle_resume(65533u, 65537u);
    puts("audio playout recovery tests passed");
    return 0;
}
