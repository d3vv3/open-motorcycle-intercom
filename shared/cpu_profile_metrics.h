#ifndef CPU_PROFILE_METRICS_H
#define CPU_PROFILE_METRICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uintptr_t handle;
    uint32_t creation_id;
    uint64_t runtime_us;
} cpu_profile_sample_t;

static inline bool cpu_profile_same_task(const cpu_profile_sample_t *a,
                                         const cpu_profile_sample_t *b)
{
    return a->handle == b->handle && a->creation_id == b->creation_id;
}

static inline bool cpu_profile_delta(const cpu_profile_sample_t *previous,
                                     const cpu_profile_sample_t *current,
                                     uint64_t interval_us, uint64_t *delta_us)
{
    if (!cpu_profile_same_task(previous, current) || interval_us == 0 ||
        current->runtime_us < previous->runtime_us) {
        return false;
    }
    *delta_us = current->runtime_us - previous->runtime_us;
    return true;
}

/* Divide before scaling to avoid overflow on long-running U64 counters. */
static inline uint64_t cpu_profile_permille(uint64_t runtime_us, uint64_t interval_us,
                                            uint32_t cores)
{
    if (!interval_us || !cores) return 0;
    uint64_t whole = runtime_us / interval_us;
    uint64_t remainder = runtime_us % interval_us;
    uint64_t fraction = interval_us <= UINT64_MAX / 1000
                            ? remainder * 1000 / interval_us
                            : remainder / (interval_us / 1000 + 1);
    uint64_t one_core = whole > (UINT64_MAX - 999) / 1000
                            ? UINT64_MAX : whole * 1000 + fraction;
    return one_core / cores;
}

#endif
