#ifndef OMI_AUDIO_TX_CACHE_H
#define OMI_AUDIO_TX_CACHE_H

/**
 * @file audio_tx_cache.h
 * @brief Audio TX predecessor-frame cache policy (pure C11, host-testable).
 *
 * Caches the most recently transmitted audio frame so the next bundle can
 * attach it as previous1 redundancy. The attach rule is:
 *   valid && cached.active && (uint16_t)(cached.seq + 1) == current_seq
 *
 * Thread-safety: on target, store + query run in the same task (the audio TX
 * callback), but reset is invoked from other task contexts (bridge event
 * callbacks, mesh user enable/disable, transport selection). The `valid`
 * flag is therefore _Atomic with release/acquire ordering, mirroring the
 * original main.c implementation. The frame payload itself is only written
 * by the TX task, so it needs no additional synchronization.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "mesh_protocol_defs.h"

typedef struct {
    uint8_t data[MESH_AUDIO_V2_MAX_FRAME_BYTES];
    uint16_t len;
    uint16_t seq;
    bool active;
    _Atomic bool valid;
} audio_tx_cache_t;

/**
 * @brief Invalidate the cached predecessor frame.
 *
 * Safe to call from any task context.
 */
void audio_tx_cache_reset(audio_tx_cache_t *cache);

/**
 * @brief Cache a just-transmitted frame as the predecessor candidate.
 *
 * A zero-length or oversize frame (len > MESH_AUDIO_V2_MAX_FRAME_BYTES)
 * invalidates the cache instead of storing. Otherwise the frame is stored
 * and the cache is marked valid iff @p eligible is true.
 *
 * @param cache    Cache instance.
 * @param data     Encoded frame payload.
 * @param len      Payload length in bytes.
 * @param active   Whether the frame carried active (non-DTX) audio.
 * @param seq      End-to-end sequence number assigned to the frame.
 * @param eligible Whether the frame may later be attached as previous1
 *                 (in main.c: the mesh-user-enabled gate at TX time).
 */
void audio_tx_cache_store(audio_tx_cache_t *cache, const uint8_t *data,
                          uint16_t len, bool active, uint16_t seq,
                          bool eligible);

/**
 * @brief Fetch the cached frame if it qualifies as previous1 for @p current_seq.
 *
 * Returns a pointer to the cached payload and writes its length to
 * @p len_out when the cache is valid, the cached frame was active, and its
 * sequence number immediately precedes @p current_seq (uint16 wraparound
 * aware). Returns NULL (and *len_out = 0) otherwise.
 */
const uint8_t *audio_tx_cache_previous(const audio_tx_cache_t *cache,
                                       uint16_t current_seq,
                                       uint16_t *len_out);

#endif /* OMI_AUDIO_TX_CACHE_H */
