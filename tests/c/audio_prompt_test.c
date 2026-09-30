#include "audio_prompt.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static void known_vectors(void)
{
    const uint8_t data[] = {0x77, 0xf7};
    audio_prompt_clip_t clip = {data, sizeof(data), 3, 0, 0};
    audio_prompt_player_t player;
    int16_t sample;
    assert(audio_prompt_start(&player, &clip));
    assert(audio_prompt_next(&player, 16000, &sample) && sample == 11);
    assert(audio_prompt_next(&player, 16000, &sample) && sample == 41);
    assert(audio_prompt_next(&player, 16000, &sample) && sample == 104);
    assert(!audio_prompt_next(&player, 16000, &sample));
    assert(!audio_prompt_next(&player, 0, &sample));
    assert(audio_prompt_start(&player, &clip));
    assert(audio_prompt_next(&player, 32000, &sample) && sample == 0);
    assert(audio_prompt_next(&player, 32000, &sample) && sample == 11);
    assert(audio_prompt_next(&player, 32000, &sample) && sample == 11);
    assert(audio_prompt_next(&player, 32000, &sample) && sample == 41);

    clip.samples = 5;
    assert(!audio_prompt_start(&player, &clip));
    clip.samples = 1;
    clip.index = 89;
    assert(!audio_prompt_start(&player, &clip));
    clip.index = 0;
    clip.predictor = 32760;
    assert(audio_prompt_start(&player, &clip));
    assert(audio_prompt_next(&player, 16000, &sample) && sample == INT16_MAX);
    clip.predictor = -32760;
    const uint8_t negative[] = {0x0f};
    clip.data = negative;
    assert(audio_prompt_start(&player, &clip));
    assert(audio_prompt_next(&player, 16000, &sample) && sample == INT16_MIN);
}

static void mix_headroom(void)
{
    const uint32_t total = 400u;
    assert(audio_prompt_mix(INT16_MAX, INT16_MIN, 0, total, 16000) == INT16_MAX);
    assert(audio_prompt_mix(INT16_MAX, INT16_MIN, total - 1u, total, 16000) == INT16_MAX);
    assert(audio_prompt_mix(INT16_MAX, INT16_MIN, 80, total, 16000) == -16384);
    assert(audio_prompt_mix(INT16_MIN, INT16_MAX, 80, total, 16000) == 16383);
    assert(audio_prompt_mix(INT16_MAX, INT16_MAX, 80, total, 16000) == INT16_MAX);
    assert(audio_prompt_mix(INT16_MIN, INT16_MIN, 80, total, 16000) == INT16_MIN);
    assert(audio_prompt_mix(INT16_MAX, INT16_MAX, total, total, 16000) == INT16_MAX);
    assert(audio_prompt_mix(1234, -20000, 0, 0, 16000) == 1234);
    assert(audio_prompt_mix(1234, -20000, 0, total, 0) == 1234);
    assert(audio_prompt_mix(20000, -20000, 1, total, 16000) < 20000);
    assert(audio_prompt_mix(20000, -20000, 1, total, 16000) >
           audio_prompt_mix(20000, -20000, 40, total, 16000));
    assert(audio_prompt_mix(20000, -20000, total - 2u, total, 16000) < 20000);
    assert(audio_prompt_mix(20000, -20000, total - 2u, total, 16000) >
           audio_prompt_mix(20000, -20000, total - 41u, total, 16000));
}

static void clip_duration(const audio_prompt_clip_t *clip, uint32_t rate)
{
    audio_prompt_player_t player;
    int16_t sample;
    uint32_t count = 0;
    uint32_t total = (uint32_t)(((uint64_t)clip->samples * rate + 15999u) / 16000u);
    assert(audio_prompt_start(&player, clip));
    while (audio_prompt_next(&player, rate, &sample)) {
        int16_t mixed = audio_prompt_mix(24000, sample, count, total, rate);
        if (count == 0u || count + 1u == total) assert(mixed == 24000);
        count++;
        assert(count <= (uint64_t)clip->samples * rate / 16000u + 2u);
    }
    assert(count == ((uint64_t)clip->samples * rate + 15999u) / 16000u);
    assert(player.position == clip->samples);
    assert(audio_prompt_mix(24000, sample, count, total, rate) == 24000);
}

int main(void)
{
    assert(AUDIO_NOTIFY_COUNT == 12);
    known_vectors();
    mix_headroom();
    for (int type = 0; type < AUDIO_NOTIFY_COUNT; ++type) {
        const audio_prompt_clip_t *clip = audio_prompt_for_notification((audio_notify_t)type);
        assert(clip == &audio_prompt_clips[type]);
        assert(clip->data != NULL && clip->bytes > 0 && clip->samples > 0);
        assert(clip->bytes == (clip->samples + 1u) / 2u);
        clip_duration(clip, 16000);
        clip_duration(clip, 32000);
        clip_duration(clip, 8000);
    }
    assert(audio_prompt_for_notification(AUDIO_NOTIFY_COUNT) == NULL);
    assert(audio_prompt_for_notification(AUDIO_NOTIFY_INCOMING_CALL) ==
           &audio_prompt_clips[11]);
    assert(audio_prompt_for_notification((audio_notify_t)-1) == NULL);
    puts("audio prompt tests passed");
    return 0;
}
