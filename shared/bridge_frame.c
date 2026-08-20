#include "bridge_frame.h"

#include <string.h>

uint8_t bridge_frame_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0u;
    size_t i;

    if (data == NULL && len != 0u) {
        return 0u;
    }

    for (i = 0u; i < len; ++i) {
        unsigned int bit;

        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & UINT8_C(0x80)) != 0u ? (uint8_t)((uint8_t)(crc << 1) ^ UINT8_C(0x07))
                                              : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

size_t bridge_frame_encode(uint8_t *dst, size_t dst_capacity, size_t max_payload, uint8_t seq,
                           uint8_t type, const uint8_t *payload, size_t payload_len)
{
    size_t frame_len;
    uint8_t wire_len;

    if (dst == NULL || max_payload > BRIDGE_FRAME_MAX_PAYLOAD || payload_len > max_payload ||
        (payload_len != 0u && payload == NULL) || payload_len > SIZE_MAX - BRIDGE_FRAME_OVERHEAD) {
        return 0u;
    }

    frame_len = payload_len + BRIDGE_FRAME_OVERHEAD;
    if (dst_capacity < frame_len) {
        return 0u;
    }

    wire_len = (uint8_t)(payload_len + 2u);
    dst[0] = BRIDGE_FRAME_SYNC_BYTE;
    dst[1] = wire_len;
    dst[2] = seq;
    dst[3] = type;
    if (payload_len != 0u) {
        memmove(dst + 4u, payload, payload_len);
    }
    dst[frame_len - 1u] = bridge_frame_crc8(dst + 1u, payload_len + 3u);
    return frame_len;
}

bridge_frame_decode_result_t bridge_frame_decode(const uint8_t *frame, size_t frame_len,
                                                 size_t max_payload, bridge_frame_view_t *view)
{
    bridge_frame_view_t decoded = {0};
    size_t payload_len;
    size_t declared_len;

    if (view == NULL) {
        return BRIDGE_FRAME_BAD_LENGTH;
    }
    *view = decoded;
    if (max_payload > BRIDGE_FRAME_MAX_PAYLOAD) {
        return BRIDGE_FRAME_BAD_LENGTH;
    }

    if (frame == NULL || frame_len == 0u) {
        return BRIDGE_FRAME_TRUNCATED;
    }
    if (frame[0] == 0u) {
        return BRIDGE_FRAME_IDLE;
    }
    if (frame_len < BRIDGE_FRAME_OVERHEAD) {
        return BRIDGE_FRAME_TRUNCATED;
    }
    if (frame[0] != BRIDGE_FRAME_SYNC_BYTE) {
        return BRIDGE_FRAME_BAD_SYNC;
    }
    if (frame[1] < 2u) {
        return BRIDGE_FRAME_BAD_LENGTH;
    }

    payload_len = (size_t)frame[1] - 2u;
    if (payload_len > max_payload) {
        return BRIDGE_FRAME_BAD_LENGTH;
    }

    declared_len = payload_len + BRIDGE_FRAME_OVERHEAD;
    if (frame_len < declared_len) {
        return BRIDGE_FRAME_TRUNCATED;
    }
    if (frame[declared_len - 1u] != bridge_frame_crc8(frame + 1u, payload_len + 3u)) {
        return BRIDGE_FRAME_BAD_CRC;
    }

    decoded.seq = frame[2];
    decoded.type = frame[3];
    decoded.payload = frame + 4u;
    decoded.payload_len = payload_len;
    *view = decoded;
    return BRIDGE_FRAME_OK;
}
