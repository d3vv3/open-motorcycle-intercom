#ifndef TEST_ESP_ERR_H
#define TEST_ESP_ERR_H
#include <stdint.h>
typedef int32_t esp_err_t;
enum { ESP_OK = 0, ESP_ERR_NO_MEM = 0x101, ESP_ERR_INVALID_ARG = 0x102,
       ESP_ERR_INVALID_STATE = 0x103, ESP_ERR_INVALID_SIZE = 0x104,
       ESP_ERR_NOT_FOUND = 0x105, ESP_ERR_TIMEOUT = 0x107 };
const char *esp_err_to_name(esp_err_t result);
#endif
