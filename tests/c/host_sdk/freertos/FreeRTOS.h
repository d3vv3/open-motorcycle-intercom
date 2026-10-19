#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <pthread.h>
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define IRAM_ATTR
void test_critical_enter(portMUX_TYPE *lock);
void test_critical_exit(portMUX_TYPE *lock);
#define portENTER_CRITICAL(lock) test_critical_enter(lock)
#define portEXIT_CRITICAL(lock) test_critical_exit(lock)
#define portENTER_CRITICAL_ISR(lock) test_critical_enter(lock)
#define portEXIT_CRITICAL_ISR(lock) test_critical_exit(lock)
#define portYIELD_FROM_ISR() ((void)0)
#endif
