#ifndef TEST_ESP_SLEEP_H
#define TEST_ESP_SLEEP_H
#include <stdint.h>
#include "esp_err.h"
enum { ESP_SLEEP_WAKEUP_EXT0 = 2, ESP_SLEEP_WAKEUP_EXT1 = 3, ESP_SLEEP_WAKEUP_TIMER = 4,
       ESP_SLEEP_WAKEUP_GPIO = 7 };
uint64_t esp_sleep_get_wakeup_causes(void);
esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us);
void esp_deep_sleep_start(void);
#endif
