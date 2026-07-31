#include "mesh_core.h"

#include <string.h>

bool mesh_core_node_id_valid(uint8_t node_id)
{
    return node_id > 0 && node_id <= MESH_MAX_NODES;
}

uint8_t mesh_core_node_bit(uint8_t node_id)
{
    return mesh_core_node_id_valid(node_id) ? (uint8_t)(1U << (node_id - 1U)) : 0;
}

bool mesh_core_bitmap_contains(uint8_t bitmap, uint8_t node_id)
{
    uint8_t bit = mesh_core_node_bit(node_id);
    return bit != 0 && (bitmap & bit) != 0;
}

uint8_t mesh_core_first_free_node_id(uint8_t occupied_bitmap)
{
    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES; node_id++) {
        if (!mesh_core_bitmap_contains(occupied_bitmap, node_id)) {
            return node_id;
        }
    }
    return 0;
}

int8_t mesh_core_slot_for_node_id(uint8_t node_id)
{
    return mesh_core_node_id_valid(node_id) ? (int8_t)(node_id - 1U) : -1;
}

void mesh_core_dedupe_reset(mesh_core_dedupe_t *cache)
{
    if (cache != NULL) {
        memset(cache, 0, sizeof(*cache));
    }
}

bool mesh_core_dedupe_contains(const mesh_core_dedupe_t *cache, uint8_t type, uint8_t src_id,
                               uint8_t seq)
{
    if (cache == NULL) {
        return false;
    }
    for (size_t i = 0; i < MESH_CORE_DEDUPE_CAPACITY; i++) {
        const mesh_core_packet_key_t *entry = &cache->entries[i];
        if (entry->valid && entry->type == type && entry->src_id == src_id && entry->seq == seq) {
            return true;
        }
    }
    return false;
}

bool mesh_core_dedupe_accept(mesh_core_dedupe_t *cache, uint8_t type, uint8_t src_id, uint8_t seq)
{
    if (cache == NULL || mesh_core_dedupe_contains(cache, type, src_id, seq)) {
        return false;
    }
    cache->entries[cache->next] = (mesh_core_packet_key_t){
        .type = type,
        .src_id = src_id,
        .seq = seq,
        .valid = true,
    };
    cache->next = (uint8_t)((cache->next + 1U) % MESH_CORE_DEDUPE_CAPACITY);
    return true;
}

void mesh_core_dedupe_purge_node(mesh_core_dedupe_t *cache, uint8_t node_id)
{
    if (cache == NULL) {
        return;
    }
    for (size_t i = 0; i < MESH_CORE_DEDUPE_CAPACITY; i++) {
        if (cache->entries[i].valid && cache->entries[i].src_id == node_id) {
            cache->entries[i].valid = false;
        }
    }
}

void mesh_core_seq8_reset(mesh_core_seq8_t *state)
{
    if (state != NULL) {
        *state = (mesh_core_seq8_t){0};
    }
}

mesh_core_seq_result_t mesh_core_seq8_accept(mesh_core_seq8_t *state, uint8_t seq)
{
    mesh_core_seq_result_t result = {.classification = MESH_CORE_SEQ_FIRST, .gap = 0};
    if (state == NULL) {
        return result;
    }
    if (!state->initialized) {
        state->initialized = true;
        state->last = seq;
        return result;
    }

    uint8_t delta = (uint8_t)(seq - state->last);
    if (delta == 0) {
        result.classification = MESH_CORE_SEQ_DUPLICATE;
        return result;
    }
    if (delta == 1) {
        result.classification = MESH_CORE_SEQ_IN_ORDER;
    } else if (delta < 0x80U) {
        result.classification = MESH_CORE_SEQ_GAP;
        result.gap = (uint16_t)(delta - 1U);
    } else {
        result.classification = MESH_CORE_SEQ_OLD_RESET;
    }
    state->last = seq;
    return result;
}

void mesh_core_seq16_reset(mesh_core_seq16_t *state)
{
    if (state != NULL) {
        *state = (mesh_core_seq16_t){0};
    }
}

mesh_core_seq_result_t mesh_core_seq16_accept(mesh_core_seq16_t *state, uint16_t seq)
{
    mesh_core_seq_result_t result = {.classification = MESH_CORE_SEQ_FIRST, .gap = 0};
    if (state == NULL) {
        return result;
    }
    if (!state->initialized) {
        state->initialized = true;
        state->last = seq;
        return result;
    }

    uint16_t delta = (uint16_t)(seq - state->last);
    if (delta == 0) {
        result.classification = MESH_CORE_SEQ_DUPLICATE;
        return result;
    }
    if (delta == 1) {
        result.classification = MESH_CORE_SEQ_IN_ORDER;
    } else if (delta < 0x8000U) {
        result.classification = MESH_CORE_SEQ_GAP;
        result.gap = (uint16_t)(delta - 1U);
    } else {
        result.classification = MESH_CORE_SEQ_OLD_RESET;
    }
    state->last = seq;
    return result;
}

int mesh_core_address_compare(const uint8_t *a, const uint8_t *b, size_t address_len)
{
    if (address_len == 0 || a == b) {
        return 0;
    }
    if (a == NULL) {
        return -1;
    }
    if (b == NULL) {
        return 1;
    }
    for (size_t i = 0; i < address_len; i++) {
        if (a[i] != b[i]) {
            return a[i] < b[i] ? -1 : 1;
        }
    }
    return 0;
}

uint8_t mesh_core_relay_mask(uint8_t speaker_id, uint8_t local_node_id,
                             uint8_t local_heard_bitmap,
                             const mesh_core_peer_snapshot_t *peers, size_t peer_count)
{
    uint8_t speaker_bit = mesh_core_node_bit(speaker_id);
    uint8_t members = 0;
    uint8_t heard = 0;
    uint8_t member_count = 0;
    if (speaker_bit == 0 || (peers == NULL && peer_count != 0)) {
        return 0;
    }

    for (size_t i = 0; i < peer_count; i++) {
        if (!peers[i].active || !mesh_core_node_id_valid(peers[i].node_id) ||
            peers[i].node_id == speaker_id) {
            continue;
        }
        uint8_t peer_bit = mesh_core_node_bit(peers[i].node_id);
        uint8_t heard_bitmap = peers[i].node_id == local_node_id
                                   ? local_heard_bitmap
                                   : peers[i].heard_bitmap;
        members |= peer_bit;
        member_count++;
        if ((heard_bitmap & speaker_bit) != 0) {
            heard |= peer_bit;
        }
    }

    if (member_count <= 1 || (members & (uint8_t)~heard) == 0) {
        return 0;
    }
    return heard != 0 ? heard : members;
}

bool mesh_core_join_assignment_valid(const mesh_join_ack_payload_t *assignment,
                                     uint8_t sender_id, uint8_t expected_coordinator_id,
                                     uint8_t slot_count)
{
    return assignment != NULL && slot_count > 1 && slot_count <= MESH_MAX_NODES &&
           mesh_core_node_id_valid(assignment->assigned_id) && assignment->slot_index > 0 &&
           assignment->slot_index < slot_count &&
           mesh_core_node_id_valid(assignment->coordinator_id) &&
           assignment->assigned_id != assignment->coordinator_id &&
           sender_id == assignment->coordinator_id &&
           (expected_coordinator_id == 0 ||
            expected_coordinator_id == assignment->coordinator_id);
}

bool mesh_core_slot_map_valid(const mesh_slot_map_payload_t *slot_map, uint8_t local_node_id,
                              uint8_t coordinator_id, mesh_core_slot_map_result_t *result)
{
    mesh_core_slot_map_result_t parsed = {.local_slot = -1, .member_bitmap = 0, .member_count = 0};
    if (slot_map == NULL || !mesh_core_node_id_valid(local_node_id) ||
        !mesh_core_node_id_valid(coordinator_id) || slot_map->slot_count == 0 ||
        slot_map->slot_count > MESH_MAX_NODES || slot_map->slot_ids[0] != coordinator_id ||
        slot_map->active_speaker_count > MESH_MAX_ACTIVE_SPEAKERS) {
        return false;
    }

    for (uint8_t slot = 0; slot < slot_map->slot_count; slot++) {
        uint8_t node_id = slot_map->slot_ids[slot];
        if (node_id == 0) {
            continue;
        }
        uint8_t bit = mesh_core_node_bit(node_id);
        if (bit == 0 || (parsed.member_bitmap & bit) != 0) {
            return false;
        }
        parsed.member_bitmap |= bit;
        parsed.member_count++;
        if (node_id == local_node_id) {
            parsed.local_slot = (int8_t)slot;
        }
    }
    if (parsed.local_slot <= 0) {
        return false;
    }

    uint8_t speaker_bitmap = 0;
    for (uint8_t i = 0; i < slot_map->active_speaker_count; i++) {
        uint8_t speaker_id = slot_map->active_speaker_ids[i];
        uint8_t bit = mesh_core_node_bit(speaker_id);
        if (bit == 0 || (parsed.member_bitmap & bit) == 0 || (speaker_bitmap & bit) != 0) {
            return false;
        }
        speaker_bitmap |= bit;
    }

    if (result != NULL) {
        *result = parsed;
    }
    return true;
}
