#ifndef OMI_MESH_CORE_H
#define OMI_MESH_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mesh_protocol_defs.h"

#define MESH_CORE_DEDUPE_CAPACITY 32

typedef struct {
    uint8_t type;
    uint8_t src_id;
    uint8_t seq;
    bool valid;
} mesh_core_packet_key_t;

typedef struct {
    mesh_core_packet_key_t entries[MESH_CORE_DEDUPE_CAPACITY];
    uint8_t next;
} mesh_core_dedupe_t;

typedef enum {
    MESH_CORE_SEQ_FIRST,
    MESH_CORE_SEQ_IN_ORDER,
    MESH_CORE_SEQ_DUPLICATE,
    MESH_CORE_SEQ_GAP,
    MESH_CORE_SEQ_OLD_RESET,
} mesh_core_seq_class_t;

typedef struct {
    mesh_core_seq_class_t classification;
    uint16_t gap;
} mesh_core_seq_result_t;

typedef struct {
    uint8_t last;
    bool initialized;
} mesh_core_seq8_t;

typedef struct {
    uint16_t last;
    bool initialized;
} mesh_core_seq16_t;

typedef struct {
    uint8_t node_id;
    uint8_t heard_bitmap;
    bool active;
} mesh_core_peer_snapshot_t;

typedef struct {
    int8_t local_slot;
    uint8_t member_bitmap;
    uint8_t member_count;
} mesh_core_slot_map_result_t;

bool mesh_core_node_id_valid(uint8_t node_id);
uint8_t mesh_core_node_bit(uint8_t node_id);
bool mesh_core_bitmap_contains(uint8_t bitmap, uint8_t node_id);
uint8_t mesh_core_first_free_node_id(uint8_t occupied_bitmap);
int8_t mesh_core_slot_for_node_id(uint8_t node_id);

void mesh_core_dedupe_reset(mesh_core_dedupe_t *cache);
bool mesh_core_dedupe_contains(const mesh_core_dedupe_t *cache, uint8_t type, uint8_t src_id,
                               uint8_t seq);
bool mesh_core_dedupe_accept(mesh_core_dedupe_t *cache, uint8_t type, uint8_t src_id, uint8_t seq);
void mesh_core_dedupe_purge_node(mesh_core_dedupe_t *cache, uint8_t node_id);

void mesh_core_seq8_reset(mesh_core_seq8_t *state);
mesh_core_seq_result_t mesh_core_seq8_accept(mesh_core_seq8_t *state, uint8_t seq);
void mesh_core_seq16_reset(mesh_core_seq16_t *state);
mesh_core_seq_result_t mesh_core_seq16_accept(mesh_core_seq16_t *state, uint16_t seq);

int mesh_core_address_compare(const uint8_t *a, const uint8_t *b, size_t address_len);
int64_t mesh_core_recover_frame_boundary(int64_t expected_us, int64_t now_us, int64_t frame_us);

uint8_t mesh_core_relay_mask(uint8_t speaker_id, uint8_t local_node_id,
                             uint8_t local_heard_bitmap,
                             const mesh_core_peer_snapshot_t *peers, size_t peer_count);

bool mesh_core_join_assignment_valid(const mesh_join_ack_payload_t *assignment,
                                     uint8_t sender_id, uint8_t expected_coordinator_id,
                                     uint8_t slot_count);
bool mesh_core_slot_map_valid(const mesh_slot_map_payload_t *slot_map, uint8_t local_node_id,
                              uint8_t coordinator_id, mesh_core_slot_map_result_t *result);

#endif /* OMI_MESH_CORE_H */
