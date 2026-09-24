#pragma once

#include <stdint.h>

void esp_lc3_bench_init(void);
void esp_lc3_bench_capture_frame(const int16_t *pcm_20ms);
void esp_lc3_bench_deinit(void);
