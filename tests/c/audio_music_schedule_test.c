#include "audio_music_schedule.h"

#include <assert.h>
#include <stdio.h>

static void test_schedule(uint32_t rate, size_t expected_input, size_t expected_chunks)
{
    size_t limit = audio_music_input_chunk_limit(rate, 960u, 2048u);
    size_t consumed = 0u;
    size_t chunks = 0u;
    size_t output = 0u;

    assert(limit == expected_input);
    while (consumed < limit && chunks < AUDIO_MUSIC_MAX_INPUT_CHUNKS) {
        size_t count = limit - consumed;
        if (count > AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES)
            count = AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES;
        consumed += count;
        chunks++;
        output = consumed * 48000u / rate;
    }
    assert(consumed == expected_input);
    assert(chunks == expected_chunks);
    assert(output <= 960u);
}

int main(void)
{
    test_schedule(48000u, 960u, 3u);
    test_schedule(44100u, 882u, 3u);
    assert(audio_music_input_chunk_limit(48000u, 960u, 100u) == 100u);
    assert(audio_music_input_chunk_limit(48000u, 0u, 100u) == 0u);
    assert(audio_music_input_chunk_limit(44100u, 959u, 4096u) == 882u);
    assert(audio_music_input_chunk_limit(48000u, 960u, 4096u) == 960u);
    assert(audio_music_input_chunk_limit(48000u, 2000u, 4096u) == 1280u);
    assert(audio_music_input_chunk_limit(44100u, 1u, 100u) == 1u);
    puts("audio music schedule tests passed");
    return 0;
}
