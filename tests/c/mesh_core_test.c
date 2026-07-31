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

int main(void)
{
    test_ids_and_bitmaps();
    test_dedupe();
    test_sequences();
    test_election();
    test_relay_masks();
    test_join_assignments();
    test_slot_maps();
    puts("mesh_core tests passed");
    return 0;
}
