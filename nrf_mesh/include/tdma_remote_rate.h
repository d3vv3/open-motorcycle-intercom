#ifndef OMI_TDMA_REMOTE_RATE_H
#define OMI_TDMA_REMOTE_RATE_H

#include <stdbool.h>
#include <stdint.h>

#define TDMA_REMOTE_RATE_FRAME_US 20000U
#define TDMA_REMOTE_RATE_MAX_PPM 5000
#define TDMA_REMOTE_RATE_MAX_GAP_FRAMES 250U
#define TDMA_REMOTE_RATE_WINDOW_FRAMES 50U
#define TDMA_REMOTE_RATE_MIN_FRAMES 10U
#define TDMA_REMOTE_RATE_SAMPLES 8U

typedef struct {
    uint32_t frame;
    int64_t start_us;
} tdma_remote_rate_sample_t;

typedef struct {
    tdma_remote_rate_sample_t samples[TDMA_REMOTE_RATE_SAMPLES];
    tdma_remote_rate_sample_t last;
    uint8_t count;
    bool seen;
    bool valid;
    int16_t ppm;
} tdma_remote_rate_t;

static inline void tdma_remote_rate_reset(tdma_remote_rate_t *rate)
{
    rate->count = 0;
    rate->seen = false;
    rate->valid = false;
    rate->ppm = 0;
}

/* Returns true once at least ten remote frames have supplied a rate estimate.
 * Invalid/stale observations do not perturb the last good estimate. */
static inline bool tdma_remote_rate_observe(tdma_remote_rate_t *rate, uint32_t frame,
                                             int64_t start_us)
{
    if (rate->seen) {
        tdma_remote_rate_sample_t last = rate->last;
        uint32_t frames = frame - last.frame;
        if (!frames || frames >= UINT32_C(0x80000000)) return rate->valid;
        if (start_us <= last.start_us) return rate->valid;
        if (frames > TDMA_REMOTE_RATE_MAX_GAP_FRAMES) {
            tdma_remote_rate_reset(rate);
        } else {
            uint64_t elapsed = (uint64_t)start_us - (uint64_t)last.start_us;
            uint64_t expected = (uint64_t)frames * TDMA_REMOTE_RATE_FRAME_US;
            uint64_t tolerance = expected * TDMA_REMOTE_RATE_MAX_PPM / 1000000U + 300U;
            if (elapsed < expected - tolerance || elapsed > expected + tolerance)
                return rate->valid;
        }
    }

    rate->last = (tdma_remote_rate_sample_t){frame, start_us};
    rate->seen = true;
    /* Sample at most every ten frames so frequent SYNCs retain a useful window. */
    if (rate->count && frame - rate->samples[rate->count - 1U].frame <
        TDMA_REMOTE_RATE_MIN_FRAMES) return rate->valid;

    /* Retain a rolling ~1 s baseline without losing a first 30-frame relay interval. */
    while (rate->count >= 2U &&
           frame - rate->samples[1].frame >= TDMA_REMOTE_RATE_WINDOW_FRAMES) {
        for (uint8_t i = 1; i < rate->count; i++) rate->samples[i - 1U] = rate->samples[i];
        rate->count--;
    }
    if (rate->count == TDMA_REMOTE_RATE_SAMPLES) {
        for (uint8_t i = 1; i < rate->count; i++) rate->samples[i - 1U] = rate->samples[i];
        rate->count--;
    }
    rate->samples[rate->count++] = (tdma_remote_rate_sample_t){frame, start_us};
    uint32_t frames = frame - rate->samples[0].frame;
    if (frames >= TDMA_REMOTE_RATE_MIN_FRAMES) {
        uint64_t elapsed = (uint64_t)start_us - (uint64_t)rate->samples[0].start_us;
        int64_t expected = (int64_t)frames * TDMA_REMOTE_RATE_FRAME_US;
        int64_t ppm = ((int64_t)elapsed - expected) * 1000000 / expected;
        if (ppm > TDMA_REMOTE_RATE_MAX_PPM) ppm = TDMA_REMOTE_RATE_MAX_PPM;
        if (ppm < -TDMA_REMOTE_RATE_MAX_PPM) ppm = -TDMA_REMOTE_RATE_MAX_PPM;
        rate->ppm = (int16_t)ppm;
        rate->valid = true;
    }
    return rate->valid;
}

#endif /* OMI_TDMA_REMOTE_RATE_H */
