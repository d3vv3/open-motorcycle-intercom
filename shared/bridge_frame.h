#ifndef OMI_BRIDGE_FRAME_H
#define OMI_BRIDGE_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define BRIDGE_FRAME_SYNC_BYTE   UINT8_C(0xAA)
#define BRIDGE_FRAME_MAX_PAYLOAD 253u
#define BRIDGE_FRAME_OVERHEAD    5u
#define BRIDGE_FRAME_MAX_SIZE    (BRIDGE_FRAME_MAX_PAYLOAD + BRIDGE_FRAME_OVERHEAD)

typedef enum {
    BRIDGE_FRAME_OK = 0,
    BRIDGE_FRAME_IDLE,
    BRIDGE_FRAME_TRUNCATED,
    BRIDGE_FRAME_BAD_SYNC,
    BRIDGE_FRAME_BAD_LENGTH,
    BRIDGE_FRAME_BAD_CRC,
} bridge_frame_decode_result_t;

typedef struct {
    const uint8_t *payload;
    size_t payload_len;
    uint8_t seq;
    uint8_t type;
} bridge_frame_view_t;

uint8_t bridge_frame_crc8(const uint8_t *data, size_t len);

size_t bridge_frame_encode(uint8_t *dst, size_t dst_capacity, size_t max_payload, uint8_t seq,
                           uint8_t type, const uint8_t *payload, size_t payload_len);

bridge_frame_decode_result_t bridge_frame_decode(const uint8_t *frame, size_t frame_len,
                                                 size_t max_payload, bridge_frame_view_t *view);

#endif /* OMI_BRIDGE_FRAME_H */
