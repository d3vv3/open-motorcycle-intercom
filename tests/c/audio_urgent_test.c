#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "audio_mesh_call.h"
#include "audio_urgent.h"

int main(void)
{
    static const uint8_t data[] = {0x77, 0x77, 0x77, 0x77};
    const audio_prompt_clip_t clip = {data, sizeof(data), 8u, 0, 0};
    audio_urgent_t urgent = {0}; /* No audio_init, mutex or playback task. */
    int16_t stereo[32] = {0};
    assert(!audio_urgent_active(&urgent));
    assert(!audio_urgent_request(&urgent));
    uint32_t first = audio_urgent_set_incoming(&urgent, true);
    assert(first != 0u && audio_urgent_set_incoming(&urgent, true) == first);
    assert(audio_urgent_request(&urgent));
    assert(audio_urgent_active(&urgent));
    uint32_t canceled = audio_urgent_set_incoming(&urgent, false);
    assert(canceled != first && !audio_urgent_active(&urgent));
    assert(!audio_urgent_request(&urgent));
    assert(audio_urgent_mix(&urgent, &clip, stereo, 4u, 2u, 32000u) == 0u);
    for (size_t i = 0; i < 8u; ++i) assert(stereo[i] == 0);

    /* An answered call does not suppress a separate waiting-call cue. */
    audio_mesh_call_state_t privacy = {0};
    audio_mesh_call_state_update(&privacy, true);
    assert(audio_mesh_call_state_active(&privacy));
    uint32_t waiting = audio_urgent_set_incoming(&urgent, true);
    assert(waiting != canceled);
    assert(audio_urgent_mix(&urgent, &clip, stereo, 4u, 2u, 32000u) == waiting);
    assert(urgent.playing && stereo[2] == stereo[3]);
    assert(audio_urgent_epoch_valid(&urgent, waiting));
    assert(audio_urgent_set_incoming(&urgent, true) == waiting);
    assert(!audio_urgent_request(&urgent)); /* already claimed, no restart */

    /* Answer/cancel after render but before output write invalidates that mix. */
    audio_urgent_set_incoming(&urgent, false);
    assert(!audio_urgent_epoch_valid(&urgent, waiting));
    uint32_t next = audio_urgent_set_incoming(&urgent, true);
    assert(next != waiting && audio_urgent_active(&urgent));
    memset(stereo, 0, sizeof(stereo));
    assert(audio_urgent_mix(&urgent, &clip, stereo, 4u, 2u, 32000u) == next);
    assert(audio_urgent_epoch_valid(&urgent, next));
    for (unsigned i = 0; i < 3u; ++i)
        (void)audio_urgent_mix(&urgent, &clip, stereo, 4u, 2u, 32000u);
    assert(!urgent.playing && !audio_urgent_active(&urgent));
    assert(audio_urgent_set_incoming(&urgent, true) == next);
    assert(!audio_urgent_request(&urgent)); /* completed indication cannot replay */
    assert(audio_urgent_mix(&urgent, &clip, stereo, 4u, 2u, 32000u) == 0u);
    puts("audio urgent tests passed");
    return 0;
}
