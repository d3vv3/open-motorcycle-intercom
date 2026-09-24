#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct esp_lc3_codec esp_lc3_codec_t;

esp_lc3_codec_t *esp_lc3_codec_open(bool encoder);
void esp_lc3_codec_close(esp_lc3_codec_t **codec);
int esp_lc3_codec_reset(esp_lc3_codec_t *codec);
int esp_lc3_codec_encode20(esp_lc3_codec_t *codec, const int16_t pcm[320], uint8_t packet[48]);
int esp_lc3_codec_decode20(esp_lc3_codec_t *codec, const uint8_t packet[48], bool plc,
                           int16_t pcm[320]);
