#include "audio_packet_store.h"

#include <string.h>

static bool audio_packet_mode_valid(audio_packet_mode_t mode)
{
    return mode == AUDIO_PACKET_MODE_SEQUENCED || mode == AUDIO_PACKET_MODE_ARRIVAL_ORDER;
}

static bool audio_packet_sequence_ahead(uint16_t sequence, uint16_t reference)
{
    uint16_t distance = (uint16_t)(sequence - reference);
    return distance != 0u && distance < UINT16_C(0x8000);
}

static size_t audio_packet_sequence_slot(uint16_t sequence)
{
    return (size_t)(sequence % AUDIO_PACKET_STORE_CAPACITY);
}

static void audio_packet_store_advance_sequence(audio_packet_store_t *store, uint16_t count,
                                                bool delivered)
{
    if (count >= 16u) {
        store->delivered_history = 0u;
    } else {
        store->delivered_history = (uint16_t)(store->delivered_history << count);
    }
    if (delivered) {
        store->delivered_history |= 1u;
    }
    store->expected_sequence = (uint16_t)(store->expected_sequence + count);
}

static bool audio_packet_store_was_delivered(const audio_packet_store_t *store, uint16_t sequence)
{
    uint16_t age = (uint16_t)(store->expected_sequence - sequence);
    if (age == 0u || age > 16u) {
        return false;
    }
    return (store->delivered_history & (uint16_t)(UINT16_C(1) << (age - 1u))) != 0u;
}

static bool audio_packet_store_can_move_start(const audio_packet_store_t *store, uint16_t sequence)
{
    size_t index;

    for (index = 0u; index < AUDIO_PACKET_STORE_CAPACITY; ++index) {
        uint16_t distance;
        if (!store->occupied[index]) {
            continue;
        }
        distance = (uint16_t)(store->packets[index].sequence - sequence);
        if (distance >= AUDIO_PACKET_STORE_CAPACITY) {
            return false;
        }
    }
    return true;
}

void audio_packet_store_reset(audio_packet_store_t *store)
{
    if (store != NULL) {
        (void)memset(store, 0, sizeof(*store));
    }
}

size_t audio_packet_store_depth(const audio_packet_store_t *store)
{
    return store == NULL ? 0u : store->depth;
}

audio_packet_store_push_result_t
audio_packet_store_push(audio_packet_store_t *store, const audio_packet_t *packet, uint64_t now_ms)
{
    size_t slot;

    if (store == NULL || packet == NULL || !audio_packet_mode_valid(packet->mode)) {
        return AUDIO_PACKET_STORE_PUSH_INVALID_ARGUMENT;
    }
    if (packet->length == 0u || packet->length > AUDIO_PACKET_MAX_SIZE) {
        return AUDIO_PACKET_STORE_PUSH_INVALID_LENGTH;
    }
    if (store->mode_set && store->mode != packet->mode) {
        return AUDIO_PACKET_STORE_PUSH_MODE_MISMATCH;
    }
    if (packet->mode == AUDIO_PACKET_MODE_ARRIVAL_ORDER) {
        if (store->depth == AUDIO_PACKET_STORE_CAPACITY) {
            return AUDIO_PACKET_STORE_PUSH_FULL;
        }
        slot = store->arrival_tail;
        store->packets[slot] = *packet;
        store->occupied[slot] = true;
        store->arrival_tail = (store->arrival_tail + 1u) % AUDIO_PACKET_STORE_CAPACITY;
    } else {
        bool clear_uncertainty_on_accept = false;
        uint16_t distance;

        if (!store->sequence_set) {
            store->expected_sequence = packet->sequence;
            store->sequence_set = true;
        } else {
            distance = (uint16_t)(packet->sequence - store->expected_sequence);
            if (store->sequence_uncertain) {
                uint16_t speculative_age = (uint16_t)(store->expected_sequence - packet->sequence);
                if (distance >= UINT16_C(0x8000) && store->depth == 0u &&
                    speculative_age <= store->consecutive_empty_missing) {
                    store->expected_sequence = packet->sequence;
                    store->delivered_history = 0u;
                    store->sequence_uncertain = false;
                    distance = 0u;
                } else if (distance < UINT16_C(0x8000)) {
                    clear_uncertainty_on_accept = true;
                }
            }
            if (distance >= UINT16_C(0x8000)) {
                if (!store->playout_started &&
                    audio_packet_sequence_ahead(store->expected_sequence, packet->sequence) &&
                    audio_packet_store_can_move_start(store, packet->sequence)) {
                    store->expected_sequence = packet->sequence;
                } else if (audio_packet_store_was_delivered(store, packet->sequence)) {
                    return AUDIO_PACKET_STORE_PUSH_DUPLICATE;
                } else {
                    return AUDIO_PACKET_STORE_PUSH_LATE;
                }
            } else if (distance >= AUDIO_PACKET_STORE_CAPACITY) {
                return AUDIO_PACKET_STORE_PUSH_FUTURE;
            }
        }
        slot = audio_packet_sequence_slot(packet->sequence);
        if (store->occupied[slot]) {
            if (store->packets[slot].sequence == packet->sequence) {
                return AUDIO_PACKET_STORE_PUSH_DUPLICATE;
            }
            if (store->depth == AUDIO_PACKET_STORE_CAPACITY) {
                return AUDIO_PACKET_STORE_PUSH_FULL;
            }
            return AUDIO_PACKET_STORE_PUSH_FUTURE;
        }
        if (store->depth == AUDIO_PACKET_STORE_CAPACITY) {
            return AUDIO_PACKET_STORE_PUSH_FULL;
        }
        store->packets[slot] = *packet;
        store->occupied[slot] = true;
        if (clear_uncertainty_on_accept) {
            store->sequence_uncertain = false;
            store->dtx_idle = false;
        }
    }

    if (!store->mode_set) {
        store->mode = packet->mode;
        store->mode_set = true;
        store->prefill_deadline_ms = now_ms + AUDIO_PACKET_STORE_PREFILL_MS;
    }
    ++store->depth;
    return AUDIO_PACKET_STORE_PUSH_OK;
}

static audio_packet_store_pop_result_t audio_packet_store_pop_arrival(audio_packet_store_t *store,
                                                                      audio_packet_t *packet)
{
    audio_packet_t current;

    if (store->depth == 0u) {
        return store->dtx_idle ? AUDIO_PACKET_STORE_POP_DTX_IDLE : AUDIO_PACKET_STORE_POP_MISSING;
    }
    current = store->packets[store->arrival_head];
    store->occupied[store->arrival_head] = false;
    store->arrival_head = (store->arrival_head + 1u) % AUDIO_PACKET_STORE_CAPACITY;
    --store->depth;
    *packet = current;
    store->dtx_idle = !current.active;
    store->consecutive_empty_missing = 0u;
    store->sequence_uncertain = false;
    return AUDIO_PACKET_STORE_POP_PACKET;
}

static audio_packet_store_pop_result_t audio_packet_store_pop_sequenced(audio_packet_store_t *store,
                                                                        audio_packet_t *packet)
{
    size_t slot = audio_packet_sequence_slot(store->expected_sequence);

    if (store->occupied[slot] && store->packets[slot].sequence == store->expected_sequence) {
        audio_packet_t current = store->packets[slot];
        store->occupied[slot] = false;
        --store->depth;
        *packet = current;
        store->dtx_idle = !current.active;
        store->consecutive_empty_missing = 0u;
        store->sequence_uncertain = false;
        audio_packet_store_advance_sequence(store, 1u, true);
        return AUDIO_PACKET_STORE_POP_PACKET;
    }

    if (store->dtx_idle && store->depth == 0u) {
        return AUDIO_PACKET_STORE_POP_DTX_IDLE;
    }

    if (store->depth == 0u) {
        ++store->consecutive_empty_missing;
    } else {
        store->consecutive_empty_missing = 0u;
        store->sequence_uncertain = false;
    }
    audio_packet_store_advance_sequence(store, 1u, false);
    if (store->consecutive_empty_missing >= AUDIO_PACKET_STORE_EMPTY_MISSING_LIMIT) {
        store->dtx_idle = true;
        store->sequence_uncertain = true;
    }
    return AUDIO_PACKET_STORE_POP_MISSING;
}

audio_packet_store_pop_result_t audio_packet_store_pop(audio_packet_store_t *store, uint64_t now_ms,
                                                       audio_packet_t *packet)
{
    bool packet_ready;
    audio_packet_store_pop_result_t result;

    if (store == NULL || packet == NULL || !store->mode_set) {
        return AUDIO_PACKET_STORE_POP_NOT_DUE;
    }
    if (!store->playout_started) {
        if (store->depth < AUDIO_PACKET_STORE_PREFILL_PACKETS &&
            now_ms < store->prefill_deadline_ms) {
            return AUDIO_PACKET_STORE_POP_NOT_DUE;
        }
        store->playout_started = true;
        store->next_deadline_ms = now_ms;
    }

    if (store->mode == AUDIO_PACKET_MODE_ARRIVAL_ORDER) {
        packet_ready = store->depth > 0u;
    } else {
        size_t slot = audio_packet_sequence_slot(store->expected_sequence);
        packet_ready =
            store->occupied[slot] && store->packets[slot].sequence == store->expected_sequence;
    }
    if (!packet_ready && now_ms < store->next_deadline_ms + AUDIO_PACKET_STORE_LATE_GRACE_MS) {
        return AUDIO_PACKET_STORE_POP_NOT_DUE;
    }

    store->next_deadline_ms += AUDIO_PACKET_STORE_FRAME_MS;
    if (store->mode == AUDIO_PACKET_MODE_ARRIVAL_ORDER) {
        result = audio_packet_store_pop_arrival(store, packet);
    } else {
        result = audio_packet_store_pop_sequenced(store, packet);
    }
    return result;
}
