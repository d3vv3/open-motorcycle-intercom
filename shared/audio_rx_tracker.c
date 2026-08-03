#include "audio_rx_tracker.h"

#include <limits.h>
#include <string.h>

static uint32_t saturating_add(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

void audio_rx_tracker_reset_source(audio_rx_tracker_t *tracker, uint8_t source_id)
{
    memset(&tracker->sources[source_id], 0, sizeof(tracker->sources[source_id]));
}

void audio_rx_tracker_reset_sources(audio_rx_tracker_t *tracker)
{
    memset(tracker->sources, 0, sizeof(tracker->sources));
}

void audio_rx_tracker_reset(audio_rx_tracker_t *tracker)
{
    memset(tracker, 0, sizeof(*tracker));
}

audio_rx_tracker_result_t audio_rx_tracker_accept(audio_rx_tracker_t *tracker,
                                                  uint8_t source_id,
                                                  uint16_t seq)
{
    audio_rx_tracker_source_t *source = &tracker->sources[source_id];
    audio_rx_tracker_result_t result = {
        .classification = AUDIO_RX_TRACKER_FIRST,
        .gap_frames = 0u,
    };

    tracker->frames = saturating_add(tracker->frames, 1u);
    if (!source->initialized) {
        source->initialized = true;
        source->last_seq = seq;
        return result;
    }

    if (seq == (uint16_t)(source->last_seq + 1u)) {
        source->last_seq = seq;
        result.classification = AUDIO_RX_TRACKER_IN_ORDER;
        return result;
    }

    uint16_t gap = (uint16_t)(seq - (uint16_t)(source->last_seq + 1u));
    if (gap != 0u && gap < UINT16_C(0x8000)) {
        result.classification = AUDIO_RX_TRACKER_FORWARD_GAP;
        result.gap_frames = gap;
        tracker->gap_events = saturating_add(tracker->gap_events, 1u);
        tracker->gap_frames = saturating_add(tracker->gap_frames, result.gap_frames);
        source->outstanding_gap_frames =
            saturating_add(source->outstanding_gap_frames, result.gap_frames);
        source->last_seq = seq;
        return result;
    }

    result.classification = AUDIO_RX_TRACKER_REORDERED_OR_OLD;
    tracker->reordered_or_old = saturating_add(tracker->reordered_or_old, 1u);
    return result;
}

bool audio_rx_tracker_credit_predecessor(audio_rx_tracker_t *tracker,
                                         uint8_t source_id,
                                         bool queue_succeeded)
{
    audio_rx_tracker_source_t *source = &tracker->sources[source_id];

    if (!queue_succeeded || source->outstanding_gap_frames == 0u) {
        return false;
    }
    source->outstanding_gap_frames--;
    tracker->recovered_frames = saturating_add(tracker->recovered_frames, 1u);
    return true;
}
