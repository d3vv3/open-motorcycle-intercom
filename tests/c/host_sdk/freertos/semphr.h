#ifndef TEST_SEMPHR_H
#define TEST_SEMPHR_H
#include "FreeRTOS.h"
typedef struct test_semaphore *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t max, UBaseType_t initial);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t wait);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t *woke);
UBaseType_t uxSemaphoreGetCount(SemaphoreHandle_t sem);
#endif
