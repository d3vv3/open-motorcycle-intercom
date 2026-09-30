#include "audio_volume_mix.h"

#include <limits.h>

static bool has_signal(const int16_t *samples, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (samples[i] != 0) return true;
    }
    return false;
}

size_t audio_voice_contribution_samples(const int16_t *post_gain, size_t base_samples,
                                         size_t prompt_samples)
{
    return (base_samples && has_signal(post_gain, base_samples))
               ? base_samples : prompt_samples;
}

void audio_volume_levels_reset(audio_volume_levels_t *levels)
{
    atomic_store_explicit(&levels->mesh, 100u, memory_order_release);
    atomic_store_explicit(&levels->bluetooth, 100u, memory_order_release);
}

void audio_volume_levels_set(audio_volume_levels_t *levels, bool bluetooth, uint8_t percent)
{
    atomic_store_explicit(bluetooth ? &levels->bluetooth : &levels->mesh,
                          percent, memory_order_release);
}

uint8_t audio_volume_levels_get(const audio_volume_levels_t *levels, bool bluetooth)
{
    return atomic_load_explicit(bluetooth ? &levels->bluetooth : &levels->mesh,
                                memory_order_acquire);
}

void audio_gain_reset(audio_gain_ramp_t *ramp)
{
    *ramp = (audio_gain_ramp_t){.current_q16 = 100 * 65536,
                                .destination_q16 = 100 * 65536};
}

static void gain_target(audio_gain_ramp_t *ramp, uint8_t percent, uint32_t rate)
{
    int32_t destination = (int32_t)percent * 65536;
    if (destination != ramp->destination_q16) {
        ramp->destination_q16 = destination;
        ramp->remaining = rate / 200u;
        if (ramp->remaining == 0u) ramp->remaining = 1u;
        ramp->step_q16 = (destination - ramp->current_q16) / (int32_t)ramp->remaining;
    }
}

static int32_t gain_next(audio_gain_ramp_t *ramp)
{
    if (ramp->remaining != 0u) {
        ramp->current_q16 += ramp->step_q16;
        if (--ramp->remaining == 0u) ramp->current_q16 = ramp->destination_q16;
    }
    return ramp->current_q16;
}

void audio_gain_apply(audio_gain_ramp_t *ramp, int16_t *samples, size_t count,
                      uint8_t percent, uint32_t rate)
{
    if (percent > 100u) percent = 100u;
    gain_target(ramp, percent, rate);
    for (size_t i = 0; i < count; ++i) {
        samples[i] = (int16_t)((int64_t)samples[i] * gain_next(ramp) / (100 * 65536));
    }
}

void audio_program_mix(audio_program_mix_t *state, int16_t *voice, size_t frames,
                       size_t voice_present, const int16_t *music, size_t music_samples,
                       uint32_t rate)
{
    bool has_voice = voice_present != 0u;
    bool has_music = has_signal(music, music_samples);
    uint8_t voice_percent = has_voice ? (has_music ? 50u : 100u) : 0u;
    uint8_t music_percent = has_music ? (has_voice ? 50u : 100u) : 0u;
    if (!state->initialized) {
        audio_gain_reset(&state->voice_weight);
        audio_gain_reset(&state->music_weight);
        state->voice_weight.current_q16 = state->voice_weight.destination_q16 =
            (int32_t)voice_percent * 65536;
        state->music_weight.current_q16 = state->music_weight.destination_q16 =
            (int32_t)music_percent * 65536;
        state->initialized = true;
    }
    gain_target(&state->voice_weight, voice_percent, rate);
    gain_target(&state->music_weight, music_percent, rate);
    for (size_t i = 0u; i < frames; ++i) {
        int32_t voice_q8 = gain_next(&state->voice_weight) * 256 / (100 * 65536);
        int32_t music_q8 = gain_next(&state->music_weight) * 256 / (100 * 65536);
        if (i >= voice_present) voice_q8 = 0;
        if (i >= music_samples) music_q8 = 0;
        /* Both weights together must never amplify a full-scale pair. */
        int32_t denominator = voice_q8 + music_q8 > 256 ? voice_q8 + music_q8 : 256;
        int32_t value = ((int32_t)voice[i] * voice_q8 +
                         (int32_t)(i < music_samples ? music[i] : 0) * music_q8) / denominator;
        voice[i] = (int16_t)value;
    }
}
void audio_limit_cue_reset(audio_limit_cue_t *cue)
{
    cue->position = 0u;
    cue->phase = 0u;
    atomic_store_explicit(&cue->state, AUDIO_CUE_IDLE, memory_order_release);
}

bool audio_limit_cue_busy(const audio_limit_cue_t *cue)
{
    return atomic_load_explicit(&cue->state, memory_order_acquire) != AUDIO_CUE_IDLE;
}

bool audio_limit_cue_request(audio_limit_cue_t *cue)
{
    unsigned expected = AUDIO_CUE_IDLE;
    return atomic_compare_exchange_strong_explicit(&cue->state, &expected, AUDIO_CUE_LIMIT,
                                                   memory_order_acq_rel, memory_order_acquire);
}

bool audio_limit_cue_request_end(audio_limit_cue_t *cue)
{
    unsigned state = atomic_load_explicit(&cue->state, memory_order_acquire);
    for (;;) {
        if (state == AUDIO_CUE_END || state == AUDIO_CUE_LIMIT_WITH_PENDING_END) return true;
        unsigned next = state == AUDIO_CUE_IDLE ? AUDIO_CUE_END :
                        AUDIO_CUE_LIMIT_WITH_PENDING_END;
        if (atomic_compare_exchange_weak_explicit(&cue->state, &state, next,
                                                  memory_order_acq_rel, memory_order_acquire))
            return true;
    }
}

static void cue_finish(audio_limit_cue_t *cue)
{
    cue->position = 0u;
    cue->phase = 0u;
    unsigned state = atomic_load_explicit(&cue->state, memory_order_acquire);
    for (;;) {
        unsigned next = state == AUDIO_CUE_LIMIT_WITH_PENDING_END ? AUDIO_CUE_END : AUDIO_CUE_IDLE;
        if (atomic_compare_exchange_weak_explicit(&cue->state, &state, next,
                                                  memory_order_acq_rel, memory_order_acquire))
            return;
    }
}

static int16_t cue_wave(uint32_t phase)
{
    static const int16_t quarter[] = {0, 799, 1567, 2276, 2896, 3406, 3784, 4017, 4096};
    uint32_t index = phase >> 27;
    uint32_t quadrant = index / 8u;
    uint32_t offset = index % 8u;
    int16_t sample = quarter[(quadrant & 1u) ? 8u - offset : offset];
    return quadrant >= 2u ? -sample : sample;
}

void audio_limit_cue_mix(audio_limit_cue_t *cue, int16_t *interleaved,
                         size_t frames, uint8_t channels, uint32_t rate)
{
    if (channels == 0u || rate == 0u) return;
    unsigned kind = atomic_load_explicit(&cue->state, memory_order_acquire);
    if (kind == AUDIO_CUE_IDLE) return;
    const uint32_t on = rate * 80u / 1000u;
    const uint32_t gap = rate * 60u / 1000u;
    const uint32_t end = kind == AUDIO_CUE_END ? on : on * 3u + gap * 2u;
    const uint32_t fade = rate / 200u;
    const uint32_t increment = (uint32_t)((880ull << 32) / rate);
    for (size_t frame = 0; frame < frames; ++frame) {
        uint32_t pos = cue->position;
        if (pos >= end) {
            cue_finish(cue);
            break;
        }
        uint32_t burst = pos / (on + gap);
        uint32_t within = pos % (on + gap);
        bool sounding = burst < 3u && within < on;
        uint32_t envelope = within;
        if (on - within - 1u < envelope) envelope = on - within - 1u;
        if (envelope > fade) envelope = fade;
        uint32_t cue_gain = sounding ? (fade ? envelope * 256u / fade : 256u) : 0u;
        uint32_t duck = fade == 0u ? 64u : pos < fade ? pos * 64u / fade :
                        end - pos <= fade ? (end - pos - 1u) * 64u / fade : 64u;
        int32_t tone = (int32_t)cue_wave(cue->phase) * (int32_t)cue_gain / 256;
        if (sounding) cue->phase += increment;
        for (uint8_t channel = 0; channel < channels; ++channel) {
            size_t index = frame * channels + channel;
            int32_t mixed = (int32_t)interleaved[index] * (int32_t)(256u - duck) / 256 + tone;
            if (mixed > INT16_MAX) mixed = INT16_MAX;
            if (mixed < INT16_MIN) mixed = INT16_MIN;
            interleaved[index] = (int16_t)mixed;
        }
        cue->position++;
        if (cue->position == end) {
            cue_finish(cue);
            break;
        }
    }
}
