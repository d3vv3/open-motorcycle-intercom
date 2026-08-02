#include "shared/mesh_core.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_ids_and_bitmaps(void)
{
    assert(!mesh_core_node_id_valid(0));
    assert(mesh_core_node_id_valid(1));
    assert(mesh_core_node_id_valid(8));
    assert(!mesh_core_node_id_valid(9));
    assert(mesh_core_node_bit(1) == 0x01);
    assert(mesh_core_node_bit(8) == 0x80);
    assert(mesh_core_node_bit(9) == 0);
    assert(mesh_core_bitmap_contains(0x81, 1));
    assert(mesh_core_bitmap_contains(0x81, 8));
    assert(!mesh_core_bitmap_contains(0x81, 2));
    assert(mesh_core_first_free_node_id(0x05) == 2);
    assert(mesh_core_first_free_node_id(0xff) == 0);
    assert(mesh_core_slot_for_node_id(8) == 7);
    assert(mesh_core_slot_for_node_id(0) == -1);
}

static void test_dedupe(void)
{
    mesh_core_dedupe_t cache;
    mesh_core_dedupe_reset(&cache);
    assert(mesh_core_dedupe_accept(&cache, 1, 2, 250));
    assert(mesh_core_dedupe_contains(&cache, 1, 2, 250));
    assert(!mesh_core_dedupe_accept(&cache, 1, 2, 250));
    assert(mesh_core_dedupe_accept(&cache, 1, 2, 251));
    assert(mesh_core_dedupe_accept(&cache, 1, 2, 0));
    for (uint8_t i = 1; i <= 30; i++) {
        assert(mesh_core_dedupe_accept(&cache, 2, 3, i));
    }
    assert(!mesh_core_dedupe_contains(&cache, 1, 2, 250));
    assert(mesh_core_dedupe_contains(&cache, 1, 2, 0));
    mesh_core_dedupe_purge_node(&cache, 2);
    assert(!mesh_core_dedupe_contains(&cache, 1, 2, 0));
    assert(mesh_core_dedupe_contains(&cache, 2, 3, 1));
    mesh_core_dedupe_reset(&cache);
    assert(!mesh_core_dedupe_contains(&cache, 2, 3, 1));
}

static void test_sequences(void)
{
    mesh_core_seq8_t seq8 = {0};
    mesh_core_seq_result_t result = mesh_core_seq8_accept(&seq8, 254);
    assert(result.classification == MESH_CORE_SEQ_FIRST);
    assert(mesh_core_seq8_accept(&seq8, 255).classification == MESH_CORE_SEQ_IN_ORDER);
    assert(mesh_core_seq8_accept(&seq8, 0).classification == MESH_CORE_SEQ_IN_ORDER);
    result = mesh_core_seq8_accept(&seq8, 3);
    assert(result.classification == MESH_CORE_SEQ_GAP && result.gap == 2);
    assert(mesh_core_seq8_accept(&seq8, 3).classification == MESH_CORE_SEQ_DUPLICATE);
    assert(mesh_core_seq8_accept(&seq8, 2).classification == MESH_CORE_SEQ_OLD_RESET);
    assert(mesh_core_seq8_accept(&seq8, 3).classification == MESH_CORE_SEQ_IN_ORDER);
    mesh_core_seq8_reset(&seq8);
    assert(mesh_core_seq8_accept(&seq8, 8).classification == MESH_CORE_SEQ_FIRST);

    mesh_core_seq16_t seq16 = {0};
    assert(mesh_core_seq16_accept(&seq16, 65534).classification == MESH_CORE_SEQ_FIRST);
    assert(mesh_core_seq16_accept(&seq16, 65535).classification == MESH_CORE_SEQ_IN_ORDER);
    assert(mesh_core_seq16_accept(&seq16, 0).classification == MESH_CORE_SEQ_IN_ORDER);
    result = mesh_core_seq16_accept(&seq16, 4);
    assert(result.classification == MESH_CORE_SEQ_GAP && result.gap == 3);
    assert(mesh_core_seq16_accept(&seq16, 4).classification == MESH_CORE_SEQ_DUPLICATE);
    assert(mesh_core_seq16_accept(&seq16, 2).classification == MESH_CORE_SEQ_OLD_RESET);
    assert(mesh_core_seq16_accept(&seq16, 3).classification == MESH_CORE_SEQ_IN_ORDER);
}

static void test_election(void)
{
    const uint8_t a5[5] = {1, 2, 3, 4, 5};
    const uint8_t b5[5] = {1, 2, 3, 4, 6};
    const uint8_t a6[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01};
    const uint8_t b6[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x00};
    assert(mesh_core_address_compare(a5, b5, sizeof(a5)) < 0);
    assert(mesh_core_address_compare(b5, a5, sizeof(a5)) > 0);
    assert(mesh_core_address_compare(a5, a5, sizeof(a5)) == 0);
    assert(mesh_core_address_compare(a6, b6, sizeof(a6)) > 0);
}

static void test_frame_boundary_recovery(void)
{
    const int64_t expected = 1000000;
    const int64_t frame_us = 20000;

    assert(mesh_core_recover_frame_boundary(expected, expected - 100, frame_us) == expected);
    assert(mesh_core_recover_frame_boundary(expected, expected + 2100, frame_us) == expected);
    assert(mesh_core_recover_frame_boundary(expected, expected + 10000, frame_us) == expected);
    assert(mesh_core_recover_frame_boundary(expected, expected + 20000, frame_us) ==
           expected + frame_us);
    assert(mesh_core_recover_frame_boundary(expected, expected + 25000, frame_us) ==
           expected + frame_us);
    assert(mesh_core_recover_frame_boundary(expected, expected + 65000, frame_us) ==
           expected + 3 * frame_us);
}

static void test_relay_masks(void)
{
    mesh_core_peer_snapshot_t peers[] = {
        {.node_id = 1, .active = true},
        {.node_id = 2, .active = true},
        {.node_id = 3, .heard_bitmap = 0x02, .active = true},
        {.node_id = 4, .heard_bitmap = 0x00, .active = true},
    };
    assert(mesh_core_relay_mask(2, 1, 0x02, peers, 2) == 0);
    assert(mesh_core_relay_mask(2, 1, 0x02, peers, 4) == (uint8_t)(0x01 | 0x04));
    peers[3].heard_bitmap = 0x02;
    assert(mesh_core_relay_mask(2, 1, 0x02, peers, 4) == 0);
    peers[0].heard_bitmap = 0;
    assert(mesh_core_relay_mask(2, 1, 0x00, peers, 4) == (uint8_t)(0x04 | 0x08));
    peers[2].heard_bitmap = 0;
    peers[3].heard_bitmap = 0;
    assert(mesh_core_relay_mask(2, 1, 0x00, peers, 4) == (uint8_t)(0x01 | 0x04 | 0x08));
}

static void test_join_assignments(void)
{
    mesh_join_ack_payload_t ack = {.assigned_id = 3, .slot_index = 2, .coordinator_id = 1};
    assert(mesh_core_join_assignment_valid(&ack, 1, 1, MESH_MAX_NODES));
    assert(mesh_core_join_assignment_valid(&ack, 1, 0, MESH_MAX_NODES));
    ack.assigned_id = 0;
    assert(!mesh_core_join_assignment_valid(&ack, 1, 1, MESH_MAX_NODES));
    ack = (mesh_join_ack_payload_t){.assigned_id = 1, .slot_index = 2, .coordinator_id = 1};
    assert(!mesh_core_join_assignment_valid(&ack, 1, 1, MESH_MAX_NODES));
    ack = (mesh_join_ack_payload_t){.assigned_id = 3, .slot_index = 0, .coordinator_id = 1};
    assert(!mesh_core_join_assignment_valid(&ack, 1, 1, MESH_MAX_NODES));
    ack.slot_index = MESH_MAX_NODES;
    assert(!mesh_core_join_assignment_valid(&ack, 1, 1, MESH_MAX_NODES));
    ack.slot_index = 2;
    assert(!mesh_core_join_assignment_valid(&ack, 2, 1, MESH_MAX_NODES));
    assert(!mesh_core_join_assignment_valid(&ack, 1, 2, MESH_MAX_NODES));
}

static mesh_slot_map_payload_t valid_slot_map(void)
{
    mesh_slot_map_payload_t map = {0};
    map.slot_count = MESH_MAX_NODES;
    map.slot_ids[0] = 1;
    map.slot_ids[2] = 3;
    map.slot_ids[4] = 5;
    map.active_speaker_count = 2;
    map.active_speaker_ids[0] = 3;
    map.active_speaker_ids[1] = 5;
    return map;
}

static void test_slot_maps(void)
{
    mesh_core_slot_map_result_t result;
    mesh_slot_map_payload_t map = valid_slot_map();
    assert(mesh_core_slot_map_valid(&map, 3, 1, &result));
    assert(result.local_slot == 2 && result.member_bitmap == 0x15 && result.member_count == 3);
    map.slot_ids[4] = 3;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.slot_ids[2] = 0;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.slot_ids[2] = 9;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.active_speaker_ids[0] = 0;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.active_speaker_ids[1] = 3;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.active_speaker_ids[1] = 7;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
    map = valid_slot_map();
    map.active_speaker_count = 3;
    assert(!mesh_core_slot_map_valid(&map, 3, 1, NULL));
}

static void test_speaker_selection(void)
{
    uint8_t previous[2] = {0, 0};
    uint8_t selected[2] = {0, 0};
    int64_t since[MESH_MAX_NODES + 1] = {0};

    /* No activity selects nobody. */
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 0);
    assert(selected[0] == 0 && selected[1] == 0);

    /* Earliest talk spurt wins the free slots, not the lowest node id. */
    since[7] = 100;
    since[2] = 200;
    since[5] = 300;
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 2);
    assert(selected[0] == 7 && selected[1] == 2);

    /* Granted speakers keep their slot while active, newcomers wait. */
    previous[0] = 7;
    previous[1] = 2;
    since[1] = 50;
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 2);
    assert(selected[0] == 7 && selected[1] == 2);

    /* A grant frees only when its holder goes silent. */
    since[2] = 0;
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 2);
    assert(selected[0] == 7 && selected[1] == 1);

    /* Tie on start time falls back to the lower node id. */
    memset(since, 0, sizeof(since));
    previous[0] = 0;
    previous[1] = 0;
    since[6] = 400;
    since[3] = 400;
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 2);
    assert(selected[0] == 3 && selected[1] == 6);

    /* Invalid previous ids and duplicate entries are ignored. */
    previous[0] = 9;
    previous[1] = 6;
    assert(mesh_core_select_speakers(previous, 2, since, selected) == 2);
    assert(selected[0] == 6 && selected[1] == 3);

    /* Degenerate arguments select nobody. */
    assert(mesh_core_select_speakers(NULL, 2, since, selected) == 0);
    assert(mesh_core_select_speakers(previous, 0, since, selected) == 0);
    assert(mesh_core_select_speakers(previous, 2, NULL, selected) == 0);
}

/* Firmware airtime model constants (mesh_protocol.c): 129 us ramp,
 * 4 us/byte at 2 Mbps, 11 bytes on-air overhead, 100 us TX margin. */
#define TEST_RAMP_US 129u
#define TEST_US_PER_BYTE 4u
#define TEST_OVERHEAD_BYTES 11u
#define TEST_MARGIN_US 100u

static int fit(uint32_t remaining_us, const uint32_t *candidates, size_t count)
{
    return mesh_core_fit_airtime(remaining_us, TEST_MARGIN_US, TEST_RAMP_US,
                                 TEST_US_PER_BYTE, TEST_OVERHEAD_BYTES, candidates, count);
}

static void test_fit_airtime(void)
{
    /* Full bundle, after stripping prev2, after stripping prev1. */
    const uint32_t candidates[3] = {208, 150, 90};
    /* required = 129 + 4 * (len + 11) + 100 */
    const uint32_t required_full = 1105;  /* len 208 */
    const uint32_t required_one = 873;    /* len 150 */
    const uint32_t required_two = 633;    /* len 90 */

    assert(mesh_core_esb_tx_us(TEST_RAMP_US, TEST_US_PER_BYTE, TEST_OVERHEAD_BYTES, 208) ==
           required_full - TEST_MARGIN_US);

    /* Generous slot: no strip. */
    assert(fit(5000, candidates, 3) == 0);
    /* Boundary: required == remaining still transmits (the firmware only
     * strips/drops on the strict `required > remaining`). */
    assert(fit(required_full, candidates, 3) == 0);
    /* One microsecond short of the full bundle: exactly one strip. */
    assert(fit(required_full - 1, candidates, 3) == 1);
    assert(fit(required_one, candidates, 3) == 1);
    /* Tighter still: strip both predecessors. */
    assert(fit(required_one - 1, candidates, 3) == 2);
    assert(fit(required_two, candidates, 3) == 2);
    /* Impossible: even the bare bundle misses the slot -> late drop. */
    assert(fit(required_two - 1, candidates, 3) == -1);
    assert(fit(0, candidates, 3) == -1);

    /* Bundle without predecessors: fit or drop, never strip. */
    assert(fit(required_full, candidates, 1) == 0);
    assert(fit(required_full - 1, candidates, 1) == -1);

    /* Degenerate arguments drop. */
    assert(fit(5000, NULL, 3) == -1);
    assert(fit(5000, candidates, 0) == -1);
}

static void test_defer_local_tail(void)
{
    assert(mesh_core_defer_local_tail(true, true, true, true));
    assert(!mesh_core_defer_local_tail(false, true, true, true));
    assert(!mesh_core_defer_local_tail(true, false, true, true));
    assert(!mesh_core_defer_local_tail(true, true, false, true));
    assert(!mesh_core_defer_local_tail(true, true, true, false));
}

static void test_successor_carries_prev1(void)
{
    static const uint8_t frame[4] = {0x11, 0x22, 0x33, 0x44};
    static const uint8_t other[4] = {0x11, 0x22, 0x33, 0x45};
    audio_bundle_view_t tail = {
        .current_data = frame,
        .current_len = sizeof(frame),
        .current_seq = 0xFFFF,
        .flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE,
    };
    audio_bundle_view_t successor = {
        .previous1_data = frame,
        .previous1_len = sizeof(frame),
        .current_data = other,
        .current_len = sizeof(other),
        .current_seq = 0x0000,
        .flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                 AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                 AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE,
    };

    /* Proven successor across the uint16 sequence wraparound. */
    assert(mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));

    /* Tail no longer holds the deferred sequence. */
    assert(!mesh_core_successor_carries_prev1(0xFFFE, &tail, &successor));

    /* Successor is not the immediate next sequence. */
    successor.current_seq = 1;
    assert(!mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    successor.current_seq = 0;

    /* prev1 not present. */
    successor.flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE;
    assert(!mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    successor.flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                      AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                      AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;

    /* prev1 length mismatch. */
    successor.previous1_len = sizeof(frame) - 1;
    assert(!mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    successor.previous1_len = sizeof(frame);

    /* Active state must match: tail active but successor prev1 inactive. */
    successor.flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                      AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT;
    assert(!mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    /* ...and inactive/inactive is an equally valid proof. */
    tail.flags = 0;
    assert(mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    tail.flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE;
    successor.flags = AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE |
                      AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT |
                      AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE;

    /* Payload bytes must be identical. */
    successor.previous1_data = other;
    assert(!mesh_core_successor_carries_prev1(0xFFFF, &tail, &successor));
    successor.previous1_data = frame;

    /* Non-wraparound sequence also works. */
    tail.current_seq = 41;
    successor.current_seq = 42;
    assert(mesh_core_successor_carries_prev1(41, &tail, &successor));

    /* Degenerate arguments never prove a successor. */
    assert(!mesh_core_successor_carries_prev1(41, NULL, &successor));
    assert(!mesh_core_successor_carries_prev1(41, &tail, NULL));
}

int main(void)
{
    test_ids_and_bitmaps();
    test_dedupe();
    test_sequences();
    test_election();
    test_frame_boundary_recovery();
    test_relay_masks();
    test_join_assignments();
    test_slot_maps();
    test_speaker_selection();
    test_fit_airtime();
    test_defer_local_tail();
    test_successor_carries_prev1();
    puts("mesh_core tests passed");
    return 0;
}
