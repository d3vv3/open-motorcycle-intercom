#include "audio_rx_source_select.h"

audio_rx_select_decision_t audio_rx_source_select(const audio_rx_slot_snapshot_t *slots,
                                                  size_t slot_count, uint8_t source_id,
                                                  bool frame_active, uint64_t now_ms)
{
    audio_rx_select_decision_t decision = {
        .action = AUDIO_RX_SELECT_REJECT,
        .slot_index = 0,
    };
    if (slots == NULL || slot_count == 0) {
        return decision;
    }

    bool have_free = false;
    size_t free_index = 0;
    for (size_t i = 0; i < slot_count; ++i) {
        if (slots[i].assigned && slots[i].source_id == source_id) {
            decision.action = AUDIO_RX_SELECT_MATCH;
            decision.slot_index = i;
            return decision;
        }
        if (!slots[i].assigned && !have_free) {
            have_free = true;
            free_index = i;
        }
    }

    if (have_free) {
        decision.action = AUDIO_RX_SELECT_ASSIGN_FREE;
        decision.slot_index = free_index;
        return decision;
    }

    if (frame_active) {
        /* Top-N selection: a new active talker displaces the source
         * that has been VOX-silent the longest. Recently active
         * sources keep their slot (first speaker wins). */
        size_t victim = 0;
        for (size_t i = 1; i < slot_count; ++i) {
            if (slots[i].last_active_ms < slots[victim].last_active_ms) {
                victim = i;
            }
        }
        if (now_ms - slots[victim].last_active_ms >= RX_SOURCE_EVICT_SILENCE_MS) {
            decision.action = AUDIO_RX_SELECT_EVICT;
            decision.slot_index = victim;
            return decision;
        }
    }

    return decision;
}
