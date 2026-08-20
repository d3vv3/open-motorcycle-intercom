#include "audio_tx_cache.h"

#include <string.h>

void audio_tx_cache_reset(audio_tx_cache_t *cache)
{
    atomic_store_explicit(&cache->valid, false, memory_order_release);
}

void audio_tx_cache_store(audio_tx_cache_t *cache, const uint8_t *data, uint16_t len, bool active,
                          uint16_t seq, bool eligible)
{
    if (len == 0u || len > sizeof(cache->data)) {
        audio_tx_cache_reset(cache);
        return;
    }
    memcpy(cache->data, data, len);
    cache->len = len;
    cache->seq = seq;
    cache->active = active;
    atomic_store_explicit(&cache->valid, eligible, memory_order_release);
}

const uint8_t *audio_tx_cache_previous(const audio_tx_cache_t *cache, uint16_t current_seq,
                                       uint16_t *len_out)
{
    bool attach = atomic_load_explicit(&cache->valid, memory_order_acquire) && cache->active &&
                  (uint16_t)(cache->seq + 1u) == current_seq;
    if (!attach) {
        *len_out = 0u;
        return NULL;
    }
    *len_out = cache->len;
    return cache->data;
}
