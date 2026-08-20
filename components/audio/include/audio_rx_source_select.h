/* Pure RX source slot selection/eviction policy.
 *
 * Extracted from audio_put_rx_frame() so the Top-N talker selection can be
 * unit-tested on the host. This module is pure C11: no ESP-IDF, no FreeRTOS,
 * no side effects. The caller snapshots slot state under its own lock, asks
 * for a decision, and applies it.
 */
#ifndef AUDIO_RX_SOURCE_SELECT_H
#define AUDIO_RX_SOURCE_SELECT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A source must be VOX-silent this long before a new talker may displace it. */
#define RX_SOURCE_EVICT_SILENCE_MS 400

/* Minimal snapshot of one RX source slot, as needed by the policy. */
typedef struct {
    bool assigned;
    uint8_t source_id;
    uint64_t last_active_ms;
} audio_rx_slot_snapshot_t;

typedef enum {
    /* Slot slot_index is already assigned to this source_id. */
    AUDIO_RX_SELECT_MATCH,
    /* Slot slot_index is unassigned; claim it. */
    AUDIO_RX_SELECT_ASSIGN_FREE,
    /* Evict the source in slot slot_index (longest VOX-silent), then claim it. */
    AUDIO_RX_SELECT_EVICT,
    /* No slot available; drop the frame. slot_index is not meaningful. */
    AUDIO_RX_SELECT_REJECT,
} audio_rx_select_action_t;

typedef struct {
    audio_rx_select_action_t action;
    size_t slot_index;
} audio_rx_select_decision_t;

/* Decide which slot (if any) an incoming frame should use.
 *
 * Policy (exact behavior of the original audio_put_rx_frame() logic):
 *  1. An assigned slot whose source_id matches wins (MATCH).
 *  2. Otherwise the first free slot is claimed (ASSIGN_FREE).
 *  3. Otherwise, only if the frame is VOX-active, the slot with the smallest
 *     last_active_ms is evicted -- but only when
 *     now_ms - last_active_ms >= RX_SOURCE_EVICT_SILENCE_MS (EVICT).
 *     Recently active sources keep their slot: first speaker wins.
 *  4. Otherwise the frame is rejected (REJECT). Keepalive (inactive) frames
 *     never evict.
 */
audio_rx_select_decision_t audio_rx_source_select(const audio_rx_slot_snapshot_t *slots,
                                                  size_t slot_count, uint8_t source_id,
                                                  bool frame_active, uint64_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_RX_SOURCE_SELECT_H */
