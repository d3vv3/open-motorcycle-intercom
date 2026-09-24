#include "shared/mesh_protocol_defs.h"

#include <assert.h>
#include <stdio.h>

static void test_legacy_audio_wire_validation(void)
{
    assert(mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS, 1,
                                         MESH_AUDIO_CODEC_OPUS));
    assert(mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS,
                                         MESH_MAX_OPUS_BYTES, MESH_AUDIO_CODEC_OPUS));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS, 0,
                                          MESH_AUDIO_CODEC_OPUS));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS,
                                          MESH_MAX_OPUS_BYTES + 1, MESH_AUDIO_CODEC_OPUS));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS - 1, 20,
                                          MESH_AUDIO_CODEC_OPUS));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_OPUS, MESH_FRAME_MS, 20,
                                          MESH_AUDIO_CODEC_LC3));
    assert(mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_LC3, MESH_FRAME_MS,
                                         MESH_LC3_FRAME_BYTES, MESH_AUDIO_CODEC_LC3));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_LC3, MESH_FRAME_MS,
                                          MESH_LC3_FRAME_BYTES - 1, MESH_AUDIO_CODEC_LC3));
    assert(!mesh_audio_wire_payload_valid(MESH_AUDIO_CODEC_LC3, MESH_FRAME_MS,
                                          MESH_LC3_FRAME_BYTES + 1, MESH_AUDIO_CODEC_LC3));
    assert(sizeof(mesh_audio_payload_t) == 70);
    assert(MESH_AUDIO_V2_MAX_BUNDLE_SIZE == 200);
    assert(MESH_AUDIO_V2_MAX_PACKET_SIZE == 208);
}

int main(void)
{
    test_legacy_audio_wire_validation();
    puts("mesh wire tests passed");
    return 0;
}
