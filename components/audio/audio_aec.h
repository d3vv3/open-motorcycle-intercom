#ifndef AUDIO_AEC_H
#define AUDIO_AEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_aec.h"
#include "audio_reblock.h"

#define AUDIO_AEC_FIFO_CAPACITY 1024u
#define AUDIO_AEC_MAX_CHUNK_SIZE AUDIO_REBLOCK_MAX_CHUNK_SIZE

typedef struct {
    aec_handle_t *handle;
    int16_t *mic_fifo;
    int16_t *ref_fifo;
    int16_t *out_fifo;
    audio_reblock_t reblock;
    bool available;
    bool active;
} audio_aec_t;

bool audio_aec_init(audio_aec_t *aec, bool enabled);
void audio_aec_deinit(audio_aec_t *aec);
bool audio_aec_process(audio_aec_t *aec, int16_t *mic, const int16_t *far_ref,
                       size_t count);

#endif
