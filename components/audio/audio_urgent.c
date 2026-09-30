#include "audio_urgent.h"

#define ELIGIBLE 1u
#define PENDING 2u
#define CLAIMED 4u
#define EPOCH_SHIFT 3u
#define EPOCH_MASK 0x1fffffffu

uint32_t audio_urgent_set_incoming(audio_urgent_t *urgent, bool incoming)
{
    unsigned old = atomic_load_explicit(&urgent->intent, memory_order_acquire);
    for (;;) {
        if (incoming && (old & ELIGIBLE)) return old >> EPOCH_SHIFT;
        unsigned epoch = ((old >> EPOCH_SHIFT) + 1u) & EPOCH_MASK;
        unsigned next = (epoch << EPOCH_SHIFT) | (incoming ? ELIGIBLE | PENDING : 0u);
        if (atomic_compare_exchange_weak_explicit(&urgent->intent, &old, next,
                                                  memory_order_acq_rel, memory_order_acquire))
            return epoch;
    }
}

bool audio_urgent_request(audio_urgent_t *urgent)
{
    unsigned old = atomic_load_explicit(&urgent->intent, memory_order_acquire);
    for (;;) {
        if (!(old & ELIGIBLE) || (old & CLAIMED)) return false;
        if (old & PENDING) return true;
        if (atomic_compare_exchange_weak_explicit(&urgent->intent, &old, old | PENDING,
                                                  memory_order_acq_rel, memory_order_acquire))
            return true;
    }
}

bool audio_urgent_active(const audio_urgent_t *urgent)
{
    unsigned state = atomic_load_explicit(&urgent->intent, memory_order_acquire);
    return (state & (ELIGIBLE | PENDING)) == (ELIGIBLE | PENDING) ||
           ((state & (ELIGIBLE | CLAIMED)) == (ELIGIBLE | CLAIMED) && urgent->playing);
}

bool audio_urgent_epoch_valid(const audio_urgent_t *urgent, uint32_t epoch)
{
    unsigned state = atomic_load_explicit(&urgent->intent, memory_order_acquire);
    return (state & (ELIGIBLE | CLAIMED)) == (ELIGIBLE | CLAIMED) &&
           (state >> EPOCH_SHIFT) == epoch;
}

uint32_t audio_urgent_mix(audio_urgent_t *urgent, const audio_prompt_clip_t *clip,
                          int16_t *output, size_t frames, uint8_t channels, uint32_t rate)
{
    if (output == NULL || channels == 0u || rate == 0u) return 0u;
    unsigned state = atomic_load_explicit(&urgent->intent, memory_order_acquire);
    if ((state & (ELIGIBLE | PENDING)) == (ELIGIBLE | PENDING)) {
        unsigned expected = state;
        if (atomic_compare_exchange_strong_explicit(&urgent->intent, &expected,
                (state & ~PENDING) | CLAIMED, memory_order_acq_rel, memory_order_acquire)) {
            urgent->playing_epoch = state >> EPOCH_SHIFT;
            urgent->playing = audio_prompt_start(&urgent->player, clip);
            urgent->mixed_samples = 0u;
            urgent->output_samples = urgent->playing
                ? (uint32_t)(((uint64_t)clip->samples * rate + 15999u) / 16000u) : 0u;
        }
    }
    if (!urgent->playing || !audio_urgent_epoch_valid(urgent, urgent->playing_epoch)) {
        urgent->playing = false;
        return 0u;
    }
    uint32_t epoch = urgent->playing_epoch;
    for (size_t i = 0; i < frames; ++i) {
        if (!audio_urgent_epoch_valid(urgent, epoch)) {
            urgent->playing = false;
            break;
        }
        int16_t sample;
        if (audio_prompt_next(&urgent->player, rate, &sample)) {
            for (uint8_t channel = 0; channel < channels; ++channel) {
                size_t index = i * channels + channel;
                output[index] = audio_prompt_mix(output[index], sample, urgent->mixed_samples,
                                                  urgent->output_samples, rate);
            }
            urgent->mixed_samples++;
        }
        if (urgent->player.position >= urgent->player.clip->samples) {
            urgent->playing = false;
            break;
        }
    }
    return epoch;
}
