#ifndef OMI_AUDIO_PROMPT_H
#define OMI_AUDIO_PROMPT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    AUDIO_NOTIFY_STARTUP,
    AUDIO_NOTIFY_PEER_JOIN,
    AUDIO_NOTIFY_PEER_LEAVE,
    AUDIO_NOTIFY_MESH_ENABLED,
    AUDIO_NOTIFY_MESH_DISABLED,
    AUDIO_NOTIFY_BLUETOOTH_PAIRING,
    AUDIO_NOTIFY_CHANNEL_GREEN,
    AUDIO_NOTIFY_CHANNEL_RED,
    AUDIO_NOTIFY_CHANNEL_BLUE,
    AUDIO_NOTIFY_ROLE_COORDINATOR,
    AUDIO_NOTIFY_ROLE_PARTICIPANT,
    AUDIO_NOTIFY_COUNT,
} audio_notify_t;

typedef struct {
    const uint8_t *data;
    size_t bytes;
    uint32_t samples;
    int16_t predictor;
    uint8_t index;
} audio_prompt_clip_t;

typedef struct {
    const audio_prompt_clip_t *clip;
    uint32_t position;
    int32_t predictor;
    uint8_t index;
    uint64_t phase;
} audio_prompt_player_t;

extern const audio_prompt_clip_t audio_prompt_clips[AUDIO_NOTIFY_COUNT];
const audio_prompt_clip_t *audio_prompt_for_notification(audio_notify_t type);
bool audio_prompt_start(audio_prompt_player_t *player, const audio_prompt_clip_t *clip);
bool audio_prompt_next(audio_prompt_player_t *player, uint32_t output_rate, int16_t *sample);
/** Mix one active prompt sample with a 5 ms entry/exit duck. The background
 * is unchanged at both boundaries; interior gains sum to unity. */
int16_t audio_prompt_mix(int16_t base, int16_t prompt, uint32_t output_position,
                         uint32_t output_samples, uint32_t output_rate);

#endif
