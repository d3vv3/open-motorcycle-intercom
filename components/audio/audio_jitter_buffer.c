#include "audio_jitter_buffer.h"

#define MESH_RX_PREFILL_FRAMES    2
#define MESH_RX_MAX_TARGET_FRAMES 4

void audio_jitter_reset(audio_jitter_state_t *state)
{
    if (!state) {
        return;
    }
    state->playout_started = false;
    state->last_rx_packet_us = 0;
    state->depth_sum = 0;
    state->depth_samples = 0;
    state->consecutive_empty = 0;
    state->hold_next = false;
    state->hold_budget = 0;
    state->stream_silent = false;
    state->next_seq = 0;
    state->next_seq_valid = false;
}

void audio_jitter_record_depth(audio_jitter_state_t *state, audio_stats_t *stats, UBaseType_t items)
{
    if (!state || !stats) {
        return;
    }

    uint8_t depth = (uint8_t)items;
    if (state->depth_samples == 0) {
        stats->rx_q_depth_min = depth;
        stats->rx_q_depth_max = depth;
    } else {
        if (depth < stats->rx_q_depth_min) {
            stats->rx_q_depth_min = depth;
        }
        if (depth > stats->rx_q_depth_max) {
            stats->rx_q_depth_max = depth;
        }
    }

    state->depth_sum += depth;
    state->depth_samples++;
    stats->rx_q_depth_avg = (uint8_t)(state->depth_sum / state->depth_samples);
}

void audio_jitter_update_playout_start(audio_jitter_state_t *state, UBaseType_t items)
{
    if (!state) {
        return;
    }

    if (!state->playout_started && items >= MESH_RX_PREFILL_FRAMES) {
        state->playout_started = true;
    }
}

void audio_jitter_trim_backlog(QueueHandle_t queue, audio_jitter_state_t *state, audio_stats_t *stats,
                               audio_rx_item_t *scratch_item)
{
    if (!queue || !state || !stats || !scratch_item || !state->playout_started) {
        return;
    }

    UBaseType_t items = uxQueueMessagesWaiting(queue);
    while (items > MESH_RX_MAX_TARGET_FRAMES) {
        if (xQueueReceive(queue, scratch_item, 0) == pdTRUE) {
            stats->frames_dropped++;
            stats->jitter_trim_frames++;
            /* Deliberate discard: re-baseline so playout does not conceal it as loss. */
            state->next_seq_valid = false;
            items = uxQueueMessagesWaiting(queue);
        } else {
            break;
        }
    }
}

bool audio_jitter_should_count_underrun(const audio_jitter_state_t *state, int64_t now_us)
{
    if (!state) {
        return false;
    }
    return (now_us - state->last_rx_packet_us) < 200000;
}
