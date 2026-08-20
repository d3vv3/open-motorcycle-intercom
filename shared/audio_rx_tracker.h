#ifndef OMI_AUDIO_RX_TRACKER_H
#define OMI_AUDIO_RX_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#define AUDIO_RX_TRACKER_SOURCE_COUNT 256u

typedef struct {
    bool initialized;
    uint16_t last_seq;
    uint32_t outstanding_gap_frames;
} audio_rx_tracker_source_t;

typedef struct {
    audio_rx_tracker_source_t sources[AUDIO_RX_TRACKER_SOURCE_COUNT];
    uint32_t frames;
    uint32_t gap_events;
    uint32_t gap_frames;
    uint32_t reordered_or_old;
    uint32_t recovered_frames;
} audio_rx_tracker_t;

typedef enum {
    AUDIO_RX_TRACKER_FIRST,
    AUDIO_RX_TRACKER_IN_ORDER,
    AUDIO_RX_TRACKER_FORWARD_GAP,
    AUDIO_RX_TRACKER_REORDERED_OR_OLD,
} audio_rx_tracker_classification_t;

typedef struct {
    audio_rx_tracker_classification_t classification;
    uint32_t gap_frames;
} audio_rx_tracker_result_t;

/* Reset source sequence state while retaining cumulative counters. */
void audio_rx_tracker_reset_source(audio_rx_tracker_t *tracker, uint8_t source_id);
void audio_rx_tracker_reset_sources(audio_rx_tracker_t *tracker);

/* Reset both source state and cumulative counters. */
void audio_rx_tracker_reset(audio_rx_tracker_t *tracker);

audio_rx_tracker_result_t audio_rx_tracker_accept(audio_rx_tracker_t *tracker, uint8_t source_id,
                                                  uint16_t seq);

/* Credit one queued predecessor when it corresponds to an outstanding gap. */
bool audio_rx_tracker_credit_predecessor(audio_rx_tracker_t *tracker, uint8_t source_id,
                                         bool queue_succeeded);

#endif /* OMI_AUDIO_RX_TRACKER_H */
