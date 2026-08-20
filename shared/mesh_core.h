#ifndef OMI_MESH_CORE_H
#define OMI_MESH_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_bundle.h"
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

uint8_t mesh_core_relay_mask(uint8_t speaker_id, uint8_t local_node_id, uint8_t local_heard_bitmap,
                             const mesh_core_peer_snapshot_t *peers, size_t peer_count);

/**
 * Select up to slot_count speaker grants with first-speaker precedence.
 *
 * active_since_ms is indexed by node id (index 0 unused, valid indices
 * 1..MESH_MAX_NODES); a value greater than zero marks the node as an
 * active speaker and records when its current talk spurt started.
 *
 * Nodes granted in previous[] keep their grant while they stay active.
 * Free slots go to the remaining active nodes, earliest talk-spurt start
 * first; ties fall back to the lower node id. Returns the number of
 * selected speakers; selected[] is zero-filled beyond that count.
 */
size_t mesh_core_select_speakers(const uint8_t *previous, size_t slot_count,
                                 const int64_t *active_since_ms, uint8_t *selected);

/**
 * Estimated ESB on-air TX time for one outer packet.
 *
 * Mirrors the nRF radio model used by mesh_protocol.c:
 * ramp_us + us_per_byte * (outer_packet_bytes + overhead_bytes).
 */
uint32_t mesh_core_esb_tx_us(uint32_t ramp_us, uint32_t us_per_byte, uint32_t overhead_bytes,
                             uint32_t outer_packet_bytes);

/**
 * Decide how many predecessor frames must be stripped from an audio bundle
 * so its outer packet fits the remaining TDMA slot airtime.
 *
 * candidate_outer_lens[0] is the full outer packet length in bytes; each
 * following entry is the outer length after stripping one more predecessor
 * frame (oldest first: prev2, then prev1). Returns the index of the first
 * candidate whose estimated TX time plus margin_us fits remaining_us -
 * i.e. the number of strips required - or -1 when even the last candidate
 * does not fit (late drop).
 *
 * Fit uses required_us <= remaining_us: a packet landing exactly on the
 * margin boundary is still sent, mirroring the strict
 * `required_us > remaining_us` strip/drop condition in mesh_protocol.c.
 */
int mesh_core_fit_airtime(uint32_t remaining_us, uint32_t margin_us, uint32_t ramp_us,
                          uint32_t us_per_byte, uint32_t overhead_bytes,
                          const uint32_t *candidate_outer_lens, size_t candidate_count);

/**
 * Decide whether the local audio tail is deferred for one slot while relay
 * traffic contends for airtime. The tail is only ever deferred when it is
 * an active AUDIO_V2 bundle: an inactive or non-bundle tail transmits
 * immediately because no successor bundle will re-carry it as prev1.
 */
bool mesh_core_defer_local_tail(bool local_pending, bool relay_pending, bool relay_contention_turn,
                                bool tail_is_active_bundle);

/**
 * Proof that a deferred local tail is safe to skip: its immediate successor
 * bundle must provably carry the deferred current frame as prev1. Checks
 * that the tail still holds the deferred sequence, the successor is the
 * next sequence (uint16_t wraparound-safe), prev1 is present with matching
 * length and active state, and the payload bytes are identical.
 *
 * Both views must come from successful audio_bundle_parse() calls (parse
 * guarantees previous1_data is non-NULL whenever PREVIOUS1_PRESENT is set).
 */
bool mesh_core_successor_carries_prev1(uint16_t deferred_seq, const audio_bundle_view_t *tail,
                                       const audio_bundle_view_t *successor);

bool mesh_core_join_assignment_valid(const mesh_join_ack_payload_t *assignment, uint8_t sender_id,
                                     uint8_t expected_coordinator_id, uint8_t slot_count);
bool mesh_core_slot_map_valid(const mesh_slot_map_payload_t *slot_map, uint8_t local_node_id,
                              uint8_t coordinator_id, mesh_core_slot_map_result_t *result);

#endif /* OMI_MESH_CORE_H */
