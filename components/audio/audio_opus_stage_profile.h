#ifndef AUDIO_OPUS_STAGE_PROFILE_H
#define AUDIO_OPUS_STAGE_PROFILE_H

#include <stdint.h>

typedef struct {
    uint32_t count;
    uint64_t time_sum_us;
    uint32_t time_max_us;
} audio_opus_stage_stats_t;

typedef struct {
    audio_opus_stage_stats_t silk;
    audio_opus_stage_stats_t celt;
} audio_opus_stage_snapshot_t;

void audio_opus_stage_profile_reset(void);
audio_opus_stage_snapshot_t audio_opus_stage_profile_snapshot(void);

#endif
