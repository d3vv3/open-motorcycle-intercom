#include "esp_lc3_bench.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "esp_audio_types.h"
#include "esp_lc3_dec.h"
#include "esp_lc3_enc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LC3_FRAME_SAMPLES 160u
#define LC3_FRAME_BYTES 24u
#define LC3_PCM_BYTES (LC3_FRAME_SAMPLES * sizeof(int16_t))
#define LC3_BENCH_INTERVAL 5u
#define LC3_LOG_INTERVAL 50u

static const char *TAG = "esp_lc3_bench";

typedef struct {
    void *encoder;
    void *decoder;
    uint8_t encoded[2][LC3_FRAME_BYTES];
    int16_t decoded[LC3_FRAME_SAMPLES * 2u];
    uint32_t capture_frames;
    uint32_t samples;
    uint32_t errors;
    uint32_t last_encoded_bytes[2];
    uint32_t last_consumed[2];
    uint32_t last_decoded_bytes[2];
    uint64_t encode_sum_us;
    uint64_t decode_sum_us;
    uint32_t encode_max_us;
    uint32_t decode_max_us;
    bool ready;
} esp_lc3_bench_state_t;

static esp_lc3_bench_state_t state;

void esp_lc3_bench_init(void)
{
    const esp_lc3_enc_config_t enc_cfg = {
        .sample_rate = 16000,
        .channel = 1,
        .bits_per_sample = 16,
        .frame_dms = 100,
        .nbyte = LC3_FRAME_BYTES,
        .len_prefixed = false,
    };
    const esp_lc3_dec_cfg_t dec_cfg = {
        .sample_rate = 16000,
        .channel = 1,
        .bits_per_sample = 16,
        .frame_dms = 100,
        .nbyte = LC3_FRAME_BYTES,
        .is_cbr = true,
        .len_prefixed = false,
        .enable_plc = true,
    };
    int input_size = 0;
    int output_size = 0;

    memset(&state, 0, sizeof(state));
    esp_audio_err_t enc_result = esp_lc3_enc_open((void *)&enc_cfg, sizeof(enc_cfg), &state.encoder);
    esp_audio_err_t dec_result = esp_lc3_dec_open((void *)&dec_cfg, sizeof(dec_cfg), &state.decoder);
    if (enc_result != ESP_AUDIO_ERR_OK || dec_result != ESP_AUDIO_ERR_OK ||
        state.encoder == NULL || state.decoder == NULL ||
        esp_lc3_enc_get_frame_size(state.encoder, &input_size, &output_size) != ESP_AUDIO_ERR_OK ||
        input_size != (int)LC3_PCM_BYTES || output_size != (int)LC3_FRAME_BYTES) {
        ESP_LOGE(TAG, "LC3 init failed enc=%d dec=%d frame_sizes=%d/%d", (int)enc_result,
                 (int)dec_result, input_size, output_size);
        esp_lc3_bench_deinit();
        return;
    }

    state.ready = true;
    ESP_LOGI(TAG, "Espressif LC3 2.6.2 ready: 16 kHz mono, 10 ms, 24 bytes/frame, core=%d stack_hwm=%u",
             xPortGetCoreID(), (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

void esp_lc3_bench_capture_frame(const int16_t *pcm_20ms)
{
    if (pcm_20ms == NULL || !state.ready) {
        return;
    }
    if (++state.capture_frames % LC3_BENCH_INTERVAL != 0u) {
        return;
    }

    uint32_t encoded_bytes[2] = {0};
    uint32_t consumed[2] = {0};
    uint32_t decoded_bytes[2] = {0};
    bool failed = false;
    int64_t start_us = esp_timer_get_time();
    for (size_t frame = 0; frame < 2u; ++frame) {
        esp_audio_enc_in_frame_t input = {
            .buffer = (uint8_t *)(pcm_20ms + frame * LC3_FRAME_SAMPLES),
            .len = LC3_PCM_BYTES,
        };
        esp_audio_enc_out_frame_t output = {
            .buffer = state.encoded[frame],
            .len = LC3_FRAME_BYTES,
        };
        if (esp_lc3_enc_process(state.encoder, &input, &output) != ESP_AUDIO_ERR_OK) {
            failed = true;
            break;
        }
        encoded_bytes[frame] = output.encoded_bytes;
        if (output.encoded_bytes != LC3_FRAME_BYTES) {
            failed = true;
            break;
        }
    }
    uint32_t encode_us = (uint32_t)(esp_timer_get_time() - start_us);

    start_us = esp_timer_get_time();
    if (!failed) {
        for (size_t frame = 0; frame < 2u; ++frame) {
            esp_audio_dec_in_raw_t input = {
                .buffer = state.encoded[frame],
                .len = encoded_bytes[frame],
                .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
            };
            esp_audio_dec_out_frame_t output = {
                .buffer = (uint8_t *)(state.decoded + frame * LC3_FRAME_SAMPLES),
                .len = LC3_PCM_BYTES,
            };
            esp_audio_dec_info_t info = {0};
            if (esp_lc3_dec_decode(state.decoder, &input, &output, &info) != ESP_AUDIO_ERR_OK) {
                failed = true;
                break;
            }
            consumed[frame] = input.consumed;
            decoded_bytes[frame] = output.decoded_size;
            if (input.consumed != LC3_FRAME_BYTES || output.decoded_size != LC3_PCM_BYTES) {
                failed = true;
                break;
            }
        }
    }
    uint32_t decode_us = (uint32_t)(esp_timer_get_time() - start_us);

    state.samples++;
    state.errors += failed;
    state.encode_sum_us += encode_us;
    state.decode_sum_us += decode_us;
    if (encode_us > state.encode_max_us) state.encode_max_us = encode_us;
    if (decode_us > state.decode_max_us) state.decode_max_us = decode_us;
    memcpy(state.last_encoded_bytes, encoded_bytes, sizeof(encoded_bytes));
    memcpy(state.last_consumed, consumed, sizeof(consumed));
    memcpy(state.last_decoded_bytes, decoded_bytes, sizeof(decoded_bytes));

    if (state.samples % LC3_LOG_INTERVAL == 0u) {
        ESP_LOGI(TAG, "samples=%" PRIu32 " encode avg/max=%" PRIu64 "/%" PRIu32
                      " us decode avg/max=%" PRIu64 "/%" PRIu32
                      " us errors=%" PRIu32 " encoded=%" PRIu32 "+%" PRIu32
                      " consumed=%" PRIu32 "+%" PRIu32 " decoded=%" PRIu32 "+%" PRIu32,
                 state.samples, state.encode_sum_us / state.samples, state.encode_max_us,
                 state.decode_sum_us / state.samples, state.decode_max_us, state.errors,
                 state.last_encoded_bytes[0], state.last_encoded_bytes[1],
                 state.last_consumed[0], state.last_consumed[1],
                 state.last_decoded_bytes[0], state.last_decoded_bytes[1]);
    }
}

void esp_lc3_bench_deinit(void)
{
    if (state.encoder != NULL) {
        esp_lc3_enc_close(state.encoder);
    }
    if (state.decoder != NULL) {
        (void)esp_lc3_dec_close(state.decoder);
    }
    state.encoder = NULL;
    state.decoder = NULL;
    state.ready = false;
}
