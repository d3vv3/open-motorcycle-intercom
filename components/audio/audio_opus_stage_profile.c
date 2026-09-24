#include "audio_opus_stage_profile.h"

#if defined(AUDIO_OPUS_STAGE_PROFILE)

#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "API.h"
#include "celt.h"

static TaskHandle_t s_owner;
static audio_opus_stage_snapshot_t s_stats;

static void record_stage(audio_opus_stage_stats_t *stats, int64_t elapsed_us)
{
    uint32_t elapsed = elapsed_us > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us;
    if (stats->count != UINT32_MAX) stats->count++;
    stats->time_sum_us = UINT64_MAX - stats->time_sum_us < elapsed
                             ? UINT64_MAX
                             : stats->time_sum_us + elapsed;
    if (elapsed > stats->time_max_us) stats->time_max_us = elapsed;
}

opus_int __real_silk_Encode(void *encState, silk_EncControlStruct *encControl,
                            const opus_res *samplesIn, opus_int nSamplesIn,
                            ec_enc *psRangeEnc, opus_int32 *nBytesOut,
                            const opus_int prefillFlag, int activity);

opus_int __wrap_silk_Encode(void *encState, silk_EncControlStruct *encControl,
                            const opus_res *samplesIn, opus_int nSamplesIn,
                            ec_enc *psRangeEnc, opus_int32 *nBytesOut,
                            const opus_int prefillFlag, int activity)
{
    bool owned = xTaskGetCurrentTaskHandle() == s_owner;
    int64_t start = owned ? esp_timer_get_time() : 0;
    opus_int result = __real_silk_Encode(encState, encControl, samplesIn, nSamplesIn,
                                         psRangeEnc, nBytesOut, prefillFlag, activity);
    if (owned) record_stage(&s_stats.silk, esp_timer_get_time() - start);
    return result;
}

int __real_celt_encode_with_ec(OpusCustomEncoder *st, const opus_res *pcm, int frame_size,
                               unsigned char *compressed, int nbCompressedBytes, ec_enc *enc);

int __wrap_celt_encode_with_ec(OpusCustomEncoder *st, const opus_res *pcm, int frame_size,
                               unsigned char *compressed, int nbCompressedBytes, ec_enc *enc)
{
    bool owned = xTaskGetCurrentTaskHandle() == s_owner;
    int64_t start = owned ? esp_timer_get_time() : 0;
    int result = __real_celt_encode_with_ec(st, pcm, frame_size, compressed,
                                            nbCompressedBytes, enc);
    if (owned) record_stage(&s_stats.celt, esp_timer_get_time() - start);
    return result;
}

void audio_opus_stage_profile_reset(void)
{
    s_owner = xTaskGetCurrentTaskHandle();
    s_stats = (audio_opus_stage_snapshot_t){0};
}

audio_opus_stage_snapshot_t audio_opus_stage_profile_snapshot(void)
{
    if (xTaskGetCurrentTaskHandle() != s_owner) return (audio_opus_stage_snapshot_t){0};
    return s_stats;
}

#else

void audio_opus_stage_profile_reset(void) {}
audio_opus_stage_snapshot_t audio_opus_stage_profile_snapshot(void)
{
    return (audio_opus_stage_snapshot_t){0};
}

#endif
