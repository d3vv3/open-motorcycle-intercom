#ifndef OMI_AUDIO_JITTER_BUFFER_H
#define OMI_AUDIO_JITTER_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "audio.h"

#define AUDIO_RX_QUEUE_SIZE 8

typedef struct {
    uint8_t data[64];
    uint16_t len;
    uint8_t source_id;
    int64_t timestamp_us;
    bool active; /* false => sender is in intentional silence (DTX), not packet loss */
} audio_rx_item_t;

typedef struct {
    bool playout_started;
    int64_t last_rx_packet_us;
    uint32_t depth_sum;
    uint32_t depth_samples;
    uint8_t consecutive_empty;  /* Consecutive empty polls (for underrun grace) */
    bool hold_next;             /* Skip next consume to let queue refill */
    uint8_t hold_budget;        /* Accumulated hold-frames to burn off */
    bool stream_silent;         /* Last decoded frame marked intentional silence (DTX) */
} audio_jitter_state_t;

void audio_jitter_reset(audio_jitter_state_t *state);
void audio_jitter_record_depth(audio_jitter_state_t *state, audio_stats_t *stats, UBaseType_t items);
void audio_jitter_update_playout_start(audio_jitter_state_t *state, UBaseType_t items);
void audio_jitter_trim_backlog(QueueHandle_t queue, audio_jitter_state_t *state, audio_stats_t *stats,
                               audio_rx_item_t *scratch_item);
bool audio_jitter_should_count_underrun(const audio_jitter_state_t *state, int64_t now_us);

#endif
