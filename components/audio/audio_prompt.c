#include "audio_prompt.h"

#include <limits.h>

static const int16_t steps[] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
    45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
    209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724,
    796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272,
    2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
    7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818,
    18500, 20350, 22385, 24623, 27086, 29794, 32767,
};
static const int8_t index_delta[] = {-1, -1, -1, -1, 2, 4, 6, 8};

const audio_prompt_clip_t *audio_prompt_for_notification(audio_notify_t type)
{
    if ((unsigned)type >= AUDIO_NOTIFY_COUNT) return NULL;
    return &audio_prompt_clips[type];
}

bool audio_prompt_start(audio_prompt_player_t *player, const audio_prompt_clip_t *clip)
{
    if (player == NULL || clip == NULL || clip->data == NULL || clip->samples == 0u ||
        clip->samples / 2u + (clip->samples & 1u) > clip->bytes || clip->index > 88u) return false;
    *player = (audio_prompt_player_t){.clip = clip, .predictor = clip->predictor,
                                       .index = clip->index};
    return true;
}

static int16_t decode(audio_prompt_player_t *player)
{
    uint8_t code = player->clip->data[player->position / 2u];
    if (player->position & 1u) code >>= 4;
    code &= 15u;
    int32_t step = steps[player->index];
    int32_t diff = step >> 3;
    if (code & 4u) diff += step;
    if (code & 2u) diff += step >> 1;
    if (code & 1u) diff += step >> 2;
    int32_t value = player->predictor + ((code & 8u) ? -diff : diff);
    if (value > INT16_MAX) value = INT16_MAX;
    if (value < INT16_MIN) value = INT16_MIN;
    player->predictor = value;
    int32_t index = (int32_t)player->index + index_delta[code & 7u];
    if (index < 0) index = 0;
    if (index > 88) index = 88;
    player->index = (uint8_t)index;
    player->position++;
    return (int16_t)value;
}

bool audio_prompt_next(audio_prompt_player_t *player, uint32_t output_rate, int16_t *sample)
{
    if (player == NULL || player->clip == NULL || sample == NULL || output_rate == 0u ||
        player->position >= player->clip->samples) return false;
    *sample = (int16_t)player->predictor;
    /* Phase is bounded by output_rate on each call; step supports any output rate. */
    player->phase += 16000u;
    while (player->phase >= output_rate && player->position < player->clip->samples) {
        player->phase -= output_rate;
        *sample = decode(player);
    }
    return true;
}

int16_t audio_prompt_mix(int16_t base, int16_t prompt, uint32_t output_position,
                         uint32_t output_samples, uint32_t output_rate)
{
    if (output_samples == 0u || output_position >= output_samples || output_rate == 0u)
        return base;
    uint32_t fade_samples = output_rate / 200u;
    if (fade_samples == 0u) fade_samples = 1u;
    uint32_t distance = output_position;
    uint32_t remaining = output_samples - 1u - output_position;
    if (remaining < distance) distance = remaining;
    if (distance > fade_samples) distance = fade_samples;
    uint32_t fade_q8 = (uint32_t)((uint64_t)distance * 256u / fade_samples);
    uint32_t prompt_gain = fade_q8 * 192u / 256u;
    int32_t mixed = ((int32_t)base * (int32_t)(256u - prompt_gain) +
                     (int32_t)prompt * (int32_t)prompt_gain) / 256;
    return (int16_t)mixed;
}
