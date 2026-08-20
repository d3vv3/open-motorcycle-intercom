#include "audio_bundle.h"

#include <string.h>

enum {
    AUDIO_BUNDLE_CODEC_OFFSET = 0,
    AUDIO_BUNDLE_FRAME_MS_OFFSET = 1,
    AUDIO_BUNDLE_STREAM_ID_OFFSET = 2,
    AUDIO_BUNDLE_FLAGS_OFFSET = 3,
    AUDIO_BUNDLE_SEQUENCE_MSB_OFFSET = 4,
    AUDIO_BUNDLE_SEQUENCE_LSB_OFFSET = 5,
    AUDIO_BUNDLE_CURRENT_LEN_OFFSET = 6,
    AUDIO_BUNDLE_PREVIOUS1_LEN_OFFSET = 7,
};

static bool audio_bundle_fields_valid(const audio_bundle_view_t *bundle)
{
    bool previous1_present;
    bool previous2_present;

    if (bundle == NULL || bundle->current_data == NULL || bundle->current_len == 0u ||
        bundle->current_len > MESH_AUDIO_V2_MAX_FRAME_BYTES ||
        (bundle->flags & (uint8_t)~AUDIO_BUNDLE_FLAG_MASK) != 0u) {
        return false;
    }

    previous1_present = (bundle->flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) != 0u;
    previous2_present = (bundle->flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT) != 0u;
    if ((!previous1_present && (bundle->previous1_len != 0u ||
                                (bundle->flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0u)) ||
        (previous1_present && (bundle->previous1_data == NULL || bundle->previous1_len == 0u ||
                               bundle->previous1_len > MESH_AUDIO_V2_MAX_FRAME_BYTES)) ||
        (!previous2_present && (bundle->previous2_len != 0u ||
                                (bundle->flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0u)) ||
        (previous2_present &&
         (!previous1_present || bundle->previous2_data == NULL || bundle->previous2_len == 0u ||
          bundle->previous2_len > MESH_AUDIO_V2_MAX_FRAME_BYTES))) {
        return false;
    }

    return bundle->previous2_len + bundle->previous1_len + bundle->current_len <=
           MESH_AUDIO_V2_MAX_FRAME_DATA;
}

bool audio_bundle_encode(const audio_bundle_view_t *bundle, uint8_t *output, size_t output_capacity,
                         size_t *output_len)
{
    size_t encoded_len;
    size_t offset;

    if (output_len == NULL) {
        return false;
    }
    *output_len = 0u;
    if (output == NULL || !audio_bundle_fields_valid(bundle)) {
        return false;
    }

    encoded_len = MESH_AUDIO_V2_FIXED_HEADER_SIZE + bundle->previous2_len + bundle->previous1_len +
                  bundle->current_len;
    if (encoded_len > MESH_AUDIO_V2_MAX_BUNDLE_SIZE || output_capacity < encoded_len) {
        return false;
    }

    output[AUDIO_BUNDLE_CODEC_OFFSET] = MESH_AUDIO_V2_CODEC_OPUS;
    output[AUDIO_BUNDLE_FRAME_MS_OFFSET] = MESH_AUDIO_V2_FRAME_MS;
    output[AUDIO_BUNDLE_STREAM_ID_OFFSET] = bundle->stream_id;
    output[AUDIO_BUNDLE_FLAGS_OFFSET] = bundle->flags;
    output[AUDIO_BUNDLE_SEQUENCE_MSB_OFFSET] = (uint8_t)(bundle->current_seq >> 8);
    output[AUDIO_BUNDLE_SEQUENCE_LSB_OFFSET] = (uint8_t)bundle->current_seq;
    output[AUDIO_BUNDLE_CURRENT_LEN_OFFSET] = (uint8_t)bundle->current_len;
    output[AUDIO_BUNDLE_PREVIOUS1_LEN_OFFSET] = (uint8_t)bundle->previous1_len;

    offset = MESH_AUDIO_V2_FIXED_HEADER_SIZE;
    if (bundle->previous2_len != 0u) {
        memmove(output + offset, bundle->previous2_data, bundle->previous2_len);
        offset += bundle->previous2_len;
    }
    if (bundle->previous1_len != 0u) {
        memmove(output + offset, bundle->previous1_data, bundle->previous1_len);
        offset += bundle->previous1_len;
    }
    memmove(output + offset, bundle->current_data, bundle->current_len);
    *output_len = encoded_len;
    return true;
}

bool audio_bundle_parse(const uint8_t *data, size_t data_len, audio_bundle_view_t *bundle)
{
    audio_bundle_view_t parsed;
    size_t frame_data_len;
    bool previous1_present;
    bool previous2_present;

    if (data == NULL || bundle == NULL || data_len < MESH_AUDIO_V2_FIXED_HEADER_SIZE ||
        data_len > MESH_AUDIO_V2_MAX_BUNDLE_SIZE ||
        data[AUDIO_BUNDLE_CODEC_OFFSET] != MESH_AUDIO_V2_CODEC_OPUS ||
        data[AUDIO_BUNDLE_FRAME_MS_OFFSET] != MESH_AUDIO_V2_FRAME_MS ||
        (data[AUDIO_BUNDLE_FLAGS_OFFSET] & (uint8_t)~AUDIO_BUNDLE_FLAG_MASK) != 0u) {
        return false;
    }

    parsed.current_len = data[AUDIO_BUNDLE_CURRENT_LEN_OFFSET];
    parsed.previous1_len = data[AUDIO_BUNDLE_PREVIOUS1_LEN_OFFSET];
    frame_data_len = data_len - MESH_AUDIO_V2_FIXED_HEADER_SIZE;
    if (parsed.current_len == 0u || parsed.current_len > MESH_AUDIO_V2_MAX_FRAME_BYTES ||
        parsed.previous1_len > MESH_AUDIO_V2_MAX_FRAME_BYTES ||
        parsed.current_len + parsed.previous1_len > frame_data_len) {
        return false;
    }

    parsed.previous2_len = frame_data_len - parsed.previous1_len - parsed.current_len;
    parsed.flags = data[AUDIO_BUNDLE_FLAGS_OFFSET];
    previous1_present = (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) != 0u;
    previous2_present = (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT) != 0u;
    if ((!previous1_present && (parsed.previous1_len != 0u ||
                                (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0u)) ||
        (previous1_present && parsed.previous1_len == 0u) ||
        (!previous2_present && (parsed.previous2_len != 0u ||
                                (parsed.flags & AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE) != 0u)) ||
        (previous2_present && (!previous1_present || parsed.previous2_len == 0u ||
                               parsed.previous2_len > MESH_AUDIO_V2_MAX_FRAME_BYTES))) {
        return false;
    }

    parsed.stream_id = data[AUDIO_BUNDLE_STREAM_ID_OFFSET];
    parsed.current_seq = (uint16_t)((uint16_t)data[AUDIO_BUNDLE_SEQUENCE_MSB_OFFSET] << 8) |
                         data[AUDIO_BUNDLE_SEQUENCE_LSB_OFFSET];
    parsed.previous2_data = previous2_present ? data + MESH_AUDIO_V2_FIXED_HEADER_SIZE : NULL;
    parsed.previous1_data =
        previous1_present ? data + MESH_AUDIO_V2_FIXED_HEADER_SIZE + parsed.previous2_len : NULL;
    parsed.current_data =
        data + MESH_AUDIO_V2_FIXED_HEADER_SIZE + parsed.previous2_len + parsed.previous1_len;
    *bundle = parsed;
    return true;
}

bool audio_bundle_strip_oldest(uint8_t *data, size_t *data_len)
{
    audio_bundle_view_t bundle;
    size_t retained_len;

    if (data_len == NULL || !audio_bundle_parse(data, *data_len, &bundle)) {
        return false;
    }

    if (bundle.previous2_len != 0u) {
        retained_len = bundle.previous1_len + bundle.current_len;
        memmove(data + MESH_AUDIO_V2_FIXED_HEADER_SIZE, bundle.previous1_data, retained_len);
        data[AUDIO_BUNDLE_FLAGS_OFFSET] &=
            (uint8_t) ~(AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT | AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE);
        *data_len -= bundle.previous2_len;
        return true;
    }
    if (bundle.previous1_len != 0u) {
        memmove(data + MESH_AUDIO_V2_FIXED_HEADER_SIZE, bundle.current_data, bundle.current_len);
        data[AUDIO_BUNDLE_FLAGS_OFFSET] &=
            (uint8_t) ~(AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT | AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE);
        data[AUDIO_BUNDLE_PREVIOUS1_LEN_OFFSET] = 0u;
        *data_len = MESH_AUDIO_V2_FIXED_HEADER_SIZE + bundle.current_len;
        return true;
    }

    return false;
}
