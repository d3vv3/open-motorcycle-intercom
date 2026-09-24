#include "shared/bridge_protocol_defs.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_V2_CODEC_LC3,
                                      MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION_V2, MESH_AUDIO_V2_CODEC_LC3,
                                       MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION_V2, 0u, 0u));
    assert(!bridge_audio_supports_lc3(1u, MESH_AUDIO_V2_CODEC_LC3, MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(1u, 0u, 0u));
    assert(!bridge_audio_supports_lc3(0u, 0u, 0u));
    assert(!bridge_audio_supports_lc3(4u, MESH_AUDIO_V2_CODEC_LC3, MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION, 0u, MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_V2_CODEC_OPUS,
                                       MESH_FRAME_MS));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_V2_CODEC_LC3, 10u));
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION, MESH_AUDIO_V2_CODEC_LC3, 0u));

    puts("bridge_audio_capability_test: PASS");
    return 0;
}
