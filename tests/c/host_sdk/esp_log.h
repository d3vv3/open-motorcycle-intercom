#ifndef TEST_ESP_LOG_H
#define TEST_ESP_LOG_H
void test_sdk_log(const char *tag, const char *format, ...);
#define ESP_LOGI(...) test_sdk_log(__VA_ARGS__)
#define ESP_LOGW(...) test_sdk_log(__VA_ARGS__)
#define ESP_LOGD(...) test_sdk_log(__VA_ARGS__)
#endif
