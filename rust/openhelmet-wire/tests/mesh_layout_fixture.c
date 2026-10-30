#include <stdio.h>
#include <string.h>
#include "components/mesh/include/mesh.h"

static void emit(unsigned id, const void *object, size_t size)
{
    putchar((int)id);
    putchar((int)size);
    fwrite(object, 1, size, stdout);
}

/* Populate the live ESP packed objects, then memcpy their representations.
 * This is a layout check against shipping C, not a firmware acceptance validator. */
#define RECORD(id, type, assignments) do { \
    type value; \
    unsigned char bytes[sizeof(value)]; \
    for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = (unsigned char)(0x81u + i); \
    memcpy(&value, bytes, sizeof(value)); \
    assignments; \
    memcpy(bytes, &value, sizeof(value)); \
    emit(id, bytes, sizeof(bytes)); \
} while (0)

int main(void)
{
    const unsigned short endian = 1;
    if (*(const unsigned char *)&endian != 1) return 2;
    RECORD(0, mesh_header_t,
           value.version = 5; value.type = 0xd7; value.src_id = 0x29;
           value.seq = 0x4b; value.ttl = 0x6d; value.flags = 0x8f;
           value.talk_channel = 0xb1; value.payload_len = 0x1234);
    RECORD(1, mesh_audio_payload_t, (void)value);
    RECORD(2, mesh_join_payload_t, (void)value);
    RECORD(3, mesh_join_ack_payload_t, (void)value);
    RECORD(5, mesh_sync_payload_t, value.frame_counter = 0x78563412; value.drift_ppm = -12345);
    RECORD(6, mesh_slot_map_payload_t, (void)value);
    RECORD(7, mesh_status_payload_t, value.rssi_dbm = -101; value.temperature_c = -37);
    RECORD(8, mesh_keepalive_payload_t, (void)value);
    RECORD(9, mesh_speaker_grant_payload_t, (void)value);
    RECORD(10, mesh_speaker_release_payload_t, (void)value);
    RECORD(14, mesh_topology_payload_t, value.report_seq = 0x78563412; value.term = 0xefcdab89; value.rssi_dbm[0] = -128; value.rssi_dbm[7] = -1);
    RECORD(15, mesh_join_v3_payload_t, (void)value);
    RECORD(16, mesh_join_ack_v3_payload_t, value.term = 0x78563412);
    RECORD(17, mesh_sync_v3_payload_t, value.term = 0x78563412; value.frame_counter = 0xefcdab89; value.phase_us = 0xfedc);
    RECORD(18, mesh_handover_payload_t, value.next_term = 0x78563412; value.switch_frame = 0xefcdab89);
    RECORD(19, mesh_handover_ack_payload_t, value.next_term = 0x78563412; value.switch_frame = 0xefcdab89);
    RECORD(20, mesh_handover_payload_t, value.next_term = 0x78563412; value.switch_frame = 0xefcdab89);
    RECORD(21, mesh_handover_payload_t, value.next_term = 0x78563412; value.switch_frame = 0xefcdab89);
    RECORD(22, mesh_membership_v3_payload_t, value.term = 0x78563412; value.revision = 0xefcdab89);
    RECORD(23, mesh_speaker_request_payload_t, value.term = 0x78563412; value.request_seq = 0xefcdab89; value.frame_counter = 0x12345678);
    RECORD(24, mesh_wire_identity_t, (void)value);
    const unsigned char offsets[] = {
        offsetof(mesh_header_t, talk_channel), offsetof(mesh_header_t, payload_len),
        offsetof(mesh_membership_v3_payload_t, identities),
        offsetof(mesh_topology_payload_t, rssi_dbm), offsetof(mesh_topology_payload_t, term),
        offsetof(mesh_sync_v3_payload_t, phase_us), offsetof(mesh_handover_payload_t, slot_map),
    };
    emit(25, offsets, sizeof(offsets));
    putchar(255);
    /* Invoke only the ORIGINAL shared helper for semantic audio validation. */
    const unsigned short lengths[] = {0, 1, 47, 48, 49, 64, 65, 196, 65535};
    for (unsigned codec = 0; codec < 256; ++codec)
        for (unsigned local = 0; local < 4; ++local)
            for (unsigned ms = 19; ms <= 21; ++ms)
                for (unsigned i = 0; i < sizeof(lengths)/sizeof(lengths[0]); ++i)
                    putchar(mesh_audio_wire_payload_valid(codec, ms, lengths[i], local));
    return ferror(stdout) ? 1 : 0;
}
