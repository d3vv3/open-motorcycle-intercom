#ifndef OMI_AUDIO_PACKET_STORE_H
#define OMI_AUDIO_PACKET_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUDIO_PACKET_STORE_CAPACITY 16u
#define AUDIO_PACKET_MAX_SIZE 64u
#define AUDIO_PACKET_STORE_PREFILL_PACKETS 3u
#define AUDIO_PACKET_STORE_PREFILL_MS 60u
#define AUDIO_PACKET_STORE_FRAME_MS 20u
#define AUDIO_PACKET_STORE_EMPTY_MISSING_LIMIT 5u

_Static_assert(AUDIO_PACKET_STORE_CAPACITY < UINT16_C(0x8000),
               "packet store capacity must stay below the sequence half range");

typedef enum {
    AUDIO_PACKET_MODE_SEQUENCED = 0,
    AUDIO_PACKET_MODE_ARRIVAL_ORDER = 1
} audio_packet_mode_t;

typedef struct {
    uint8_t data[AUDIO_PACKET_MAX_SIZE];
    size_t length;
    uint16_t sequence;
    audio_packet_mode_t mode;
    bool active;
    uint64_t received_us;
} audio_packet_t;

typedef enum {
    AUDIO_PACKET_STORE_PUSH_OK = 0,
    AUDIO_PACKET_STORE_PUSH_INVALID_ARGUMENT,
    AUDIO_PACKET_STORE_PUSH_INVALID_LENGTH,
    AUDIO_PACKET_STORE_PUSH_DUPLICATE,
    AUDIO_PACKET_STORE_PUSH_LATE,
    AUDIO_PACKET_STORE_PUSH_MODE_MISMATCH,
    AUDIO_PACKET_STORE_PUSH_FUTURE,
    AUDIO_PACKET_STORE_PUSH_FULL
} audio_packet_store_push_result_t;

typedef enum {
    AUDIO_PACKET_STORE_POP_PACKET = 0,
    AUDIO_PACKET_STORE_POP_MISSING,
    AUDIO_PACKET_STORE_POP_DTX_IDLE,
    AUDIO_PACKET_STORE_POP_NOT_DUE
} audio_packet_store_pop_result_t;

typedef struct {
    audio_packet_t packets[AUDIO_PACKET_STORE_CAPACITY];
    bool occupied[AUDIO_PACKET_STORE_CAPACITY];
    size_t depth;
    size_t arrival_head;
    size_t arrival_tail;
    audio_packet_mode_t mode;
    bool mode_set;
    bool sequence_set;
    uint16_t expected_sequence;
    uint16_t delivered_history;
    uint8_t consecutive_empty_missing;
    bool playout_started;
    bool dtx_idle;
    bool sequence_uncertain;
    uint64_t prefill_deadline_ms;
    uint64_t next_deadline_ms;
} audio_packet_store_t;

void audio_packet_store_reset(audio_packet_store_t *store);
size_t audio_packet_store_depth(const audio_packet_store_t *store);
audio_packet_store_push_result_t audio_packet_store_push(audio_packet_store_t *store,
                                                         const audio_packet_t *packet,
                                                         uint64_t now_ms);
audio_packet_store_pop_result_t audio_packet_store_pop(audio_packet_store_t *store,
                                                       uint64_t now_ms,
                                                       audio_packet_t *packet);

#endif
