#include "audio_mesh_call.h"

#define ACTIVE_BIT  1u
#define BLOCKED_BIT 2u
#define EPOCH_SHIFT 2u
#define EPOCH_MASK  0x3fffffffu

uint32_t audio_mesh_call_state_update(audio_mesh_call_state_t *state, bool active)
{
    unsigned old = atomic_load_explicit(&state->state, memory_order_acquire);
    for (;;) {
        if ((old & ACTIVE_BIT) == (active ? ACTIVE_BIT : 0u)) return old >> EPOCH_SHIFT;
        unsigned epoch = ((old >> EPOCH_SHIFT) + 1u) & EPOCH_MASK;
        unsigned next = (epoch << EPOCH_SHIFT) | BLOCKED_BIT | (active ? ACTIVE_BIT : 0u);
        if (atomic_compare_exchange_weak_explicit(&state->state, &old, next,
                                                  memory_order_acq_rel, memory_order_acquire))
            return epoch;
    }
}

bool audio_mesh_call_state_resume(audio_mesh_call_state_t *state, uint32_t epoch)
{
    unsigned expected = (epoch & EPOCH_MASK) << EPOCH_SHIFT | BLOCKED_BIT;
    return epoch <= EPOCH_MASK && atomic_compare_exchange_strong_explicit(
        &state->state, &expected, expected & ~BLOCKED_BIT,
        memory_order_acq_rel, memory_order_acquire);
}

uint32_t audio_mesh_call_state_epoch(const audio_mesh_call_state_t *state)
{
    return atomic_load_explicit(&state->state, memory_order_acquire) >> EPOCH_SHIFT;
}

bool audio_mesh_call_state_active(const audio_mesh_call_state_t *state)
{
    return (atomic_load_explicit(&state->state, memory_order_acquire) & ACTIVE_BIT) != 0;
}

bool audio_mesh_call_state_blocked(const audio_mesh_call_state_t *state)
{
    return (atomic_load_explicit(&state->state, memory_order_acquire) & BLOCKED_BIT) != 0;
}

bool audio_mesh_call_state_tx_allowed(const audio_mesh_call_state_t *state, uint32_t epoch)
{
    unsigned value = atomic_load_explicit(&state->state, memory_order_acquire);
    return (value & (ACTIVE_BIT | BLOCKED_BIT)) == 0 && (value >> EPOCH_SHIFT) == epoch;
}

bool audio_mesh_call_rx_allowed(const audio_mesh_call_state_t *state, int64_t timestamp_ms,
                                uint32_t cutoff_ms)
{
    if (audio_mesh_call_state_blocked(state)) return false;
    /* Unknown timestamps cannot establish that a frame arrived after resume. */
    if (cutoff_ms != 0u && (timestamp_ms <= 0 ||
        (int32_t)((uint32_t)timestamp_ms - cutoff_ms) <= 0)) return false;
    return true;
}
