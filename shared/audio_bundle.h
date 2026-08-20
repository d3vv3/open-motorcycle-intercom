#ifndef OMI_AUDIO_BUNDLE_H
#define OMI_AUDIO_BUNDLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mesh_protocol_defs.h"

#define AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE    MESH_AUDIO_V2_FLAG_CURRENT_ACTIVE
#define AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT MESH_AUDIO_V2_FLAG_PREVIOUS1_PRESENT
#define AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE  MESH_AUDIO_V2_FLAG_PREVIOUS1_ACTIVE
#define AUDIO_BUNDLE_FLAG_PREVIOUS2_PRESENT MESH_AUDIO_V2_FLAG_PREVIOUS2_PRESENT
#define AUDIO_BUNDLE_FLAG_PREVIOUS2_ACTIVE  MESH_AUDIO_V2_FLAG_PREVIOUS2_ACTIVE
#define AUDIO_BUNDLE_FLAG_MASK              MESH_AUDIO_V2_FLAG_MASK

typedef struct {
    const uint8_t *previous1_data;
    const uint8_t *previous2_data;
    const uint8_t *current_data;
    size_t previous1_len;
    size_t previous2_len;
    size_t current_len;
    uint16_t current_seq;
    uint8_t stream_id;
    uint8_t flags;
} audio_bundle_view_t;

bool audio_bundle_encode(const audio_bundle_view_t *bundle, uint8_t *output, size_t output_capacity,
                         size_t *output_len);
bool audio_bundle_parse(const uint8_t *data, size_t data_len, audio_bundle_view_t *bundle);
bool audio_bundle_strip_oldest(uint8_t *data, size_t *data_len);

#endif /* OMI_AUDIO_BUNDLE_H */
