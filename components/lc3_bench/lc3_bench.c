#include "lc3_bench.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lc3.h"

#define LC3_FRAME_SAMPLES 160
#define LC3_FRAME_BYTES 24
#define LC3_BENCH_INTERVAL 5u
#define LC3_LOG_INTERVAL 50u

static const char *TAG = "lc3_bench";

typedef struct {
    lc3_encoder_t encoder;
    lc3_decoder_t decoder;
    void *encoder_mem;
    void *decoder_mem;
    uint8_t encoded[2][LC3_FRAME_BYTES];
    int16_t decoded[LC3_FRAME_SAMPLES * 2];
    uint32_t frame_count;
    uint32_t sample_count;
    uint32_t errors;
    uint64_t encode_sum_us;
    uint64_t decode_sum_us;
    uint32_t encode_max_us;
    uint32_t decode_max_us;
} lc3_bench_state_t;

static lc3_bench_state_t state;

void lc3_bench_reset(void)
{
    free(state.encoder_mem);
    free(state.decoder_mem);
    state = (lc3_bench_state_t){0};
}

static bool initialize(void)
{
    if (state.encoder != NULL && state.decoder != NULL) return true;

    unsigned encoder_size = lc3_encoder_size(10000, 16000);
    unsigned decoder_size = lc3_decoder_size(10000, 16000);
    state.encoder_mem = heap_caps_malloc(encoder_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    state.decoder_mem = heap_caps_malloc(decoder_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (state.encoder_mem != NULL) {
        state.encoder = lc3_setup_encoder(10000, 16000, 0, state.encoder_mem);
    }
    if (state.decoder_mem != NULL) {
        state.decoder = lc3_setup_decoder(10000, 16000, 0, state.decoder_mem);
    }
    if (state.encoder == NULL || state.decoder == NULL) {
        ESP_LOGE(TAG, "LC3 allocation/setup failed (encoder=%s decoder=%s)",
                 state.encoder != NULL ? "ok" : "failed", state.decoder != NULL ? "ok" : "failed");
        state.encoder = NULL;
        state.decoder = NULL;
        free(state.encoder_mem);
        free(state.decoder_mem);
        state.encoder_mem = NULL;
        state.decoder_mem = NULL;
        return false;
    }
    ESP_LOGI(TAG, "Google liblc3 1.1.3 configured: 16 kHz, 10 ms, 24 bytes/frame; contexts ready");
    return true;
}

void lc3_bench_capture_frame(const int16_t *pcm_20ms)
{
    if (pcm_20ms == NULL) return;
    state.frame_count++;
    if (state.frame_count % LC3_BENCH_INTERVAL != 0u || !initialize()) return;

    int64_t start_us = esp_timer_get_time();
    int encode_result = 0;
    for (size_t frame = 0; frame < 2; ++frame) {
        encode_result |= lc3_encode(state.encoder, LC3_PCM_FORMAT_S16,
                                    pcm_20ms + frame * LC3_FRAME_SAMPLES, 1,
                                    LC3_FRAME_BYTES, state.encoded[frame]);
    }
    uint32_t encode_us = (uint32_t)(esp_timer_get_time() - start_us);
    start_us = esp_timer_get_time();
    int decode_result = 0;
    if (encode_result == 0) {
        for (size_t frame = 0; frame < 2; ++frame) {
            decode_result |= lc3_decode(state.decoder, state.encoded[frame], LC3_FRAME_BYTES,
                                        LC3_PCM_FORMAT_S16,
                                        state.decoded + frame * LC3_FRAME_SAMPLES, 1);
        }
    } else {
        decode_result = -1;
    }
    uint32_t decode_us = (uint32_t)(esp_timer_get_time() - start_us);

    state.sample_count++;
    if (encode_result != 0 || decode_result != 0) state.errors++;
    state.encode_sum_us += encode_us;
    state.decode_sum_us += decode_us;
    if (encode_us > state.encode_max_us) state.encode_max_us = encode_us;
    if (decode_us > state.decode_max_us) state.decode_max_us = decode_us;
    if (state.sample_count % LC3_LOG_INTERVAL == 0u) {
        ESP_LOGI(TAG, "samples=%" PRIu32 " roundtrip=2x%d samples (%d decoded) encode avg/max=%" PRIu64 "/%" PRIu32
                      " us decode avg/max=%" PRIu64 "/%" PRIu32 " us errors=%" PRIu32,
                 state.sample_count, LC3_FRAME_SAMPLES, LC3_FRAME_SAMPLES * 2,
                 state.encode_sum_us / state.sample_count, state.encode_max_us,
                 state.decode_sum_us / state.sample_count, state.decode_max_us, state.errors);
    }
}
