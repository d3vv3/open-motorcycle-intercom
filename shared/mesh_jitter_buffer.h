#ifndef OMI_MESH_JITTER_BUFFER_H
#define OMI_MESH_JITTER_BUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "mesh_protocol_defs.h"

#define MESH_JITTER_CAPACITY 4
#define MESH_JITTER_EXPIRY_US (MESH_FRAME_MS * 3 * 1000)

typedef struct {
    uint8_t data[MESH_MAX_OPUS_BYTES];
    uint16_t len;
    uint8_t src_id;
    uint8_t seq;
    uint8_t audio_flags;
    int64_t timestamp_us;
    int64_t enqueued_us;
} mesh_jitter_entry_t;

typedef struct {
    mesh_jitter_entry_t entries[MESH_JITTER_CAPACITY];
    uint8_t read_idx;
    uint8_t write_idx;
    uint8_t count;
} mesh_jitter_buffer_t;

typedef struct {
    bool delivered;
    uint8_t expired;
    uint8_t expired_with_pending;
    uint32_t expired_age_us_max;
} mesh_jitter_pop_result_t;

static inline void mesh_jitter_reset(mesh_jitter_buffer_t *buffer)
{
    memset(buffer, 0, sizeof(*buffer));
}

/* Returns true when the oldest queued frame was overwritten. */
static inline bool mesh_jitter_push(mesh_jitter_buffer_t *buffer, const uint8_t *data,
                                    uint16_t len, uint8_t src_id, uint8_t seq,
                                    uint8_t audio_flags, int64_t timestamp_us, int64_t enqueued_us)
{
    bool overwritten = buffer->count == MESH_JITTER_CAPACITY;
    if (overwritten) {
        buffer->read_idx = (uint8_t)((buffer->read_idx + 1) % MESH_JITTER_CAPACITY);
    } else {
        buffer->count++;
    }
    mesh_jitter_entry_t *entry = &buffer->entries[buffer->write_idx];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->src_id = src_id;
    entry->seq = seq;
    entry->audio_flags = audio_flags;
    entry->timestamp_us = timestamp_us;
    entry->enqueued_us = enqueued_us;
    buffer->write_idx = (uint8_t)((buffer->write_idx + 1) % MESH_JITTER_CAPACITY);
    return overwritten;
}

static inline mesh_jitter_pop_result_t mesh_jitter_pop(mesh_jitter_buffer_t *buffer,
                                                       int64_t now_us, mesh_jitter_entry_t *out)
{
    mesh_jitter_pop_result_t result = {false, 0, 0, 0};
    for (uint8_t scanned = 0; scanned < MESH_JITTER_CAPACITY && buffer->count; scanned++) {
        mesh_jitter_entry_t *entry = &buffer->entries[buffer->read_idx];
        int64_t age_us = now_us - entry->enqueued_us;
        if (age_us > MESH_JITTER_EXPIRY_US) {
            uint32_t age = (uint64_t)age_us > UINT32_MAX ? UINT32_MAX : (uint32_t)age_us;
            if (age > result.expired_age_us_max) result.expired_age_us_max = age;
            for (uint8_t offset = 1; offset < buffer->count; offset++) {
                const mesh_jitter_entry_t *later =
                    &buffer->entries[(buffer->read_idx + offset) % MESH_JITTER_CAPACITY];
                if (now_us - later->enqueued_us <= MESH_JITTER_EXPIRY_US) {
                    result.expired_with_pending++;
                    break;
                }
            }
            result.expired++;
        } else {
            *out = *entry;
            result.delivered = true;
        }
        buffer->read_idx = (uint8_t)((buffer->read_idx + 1) % MESH_JITTER_CAPACITY);
        buffer->count--;
        if (result.delivered) break;
    }
    return result;
}

#endif
