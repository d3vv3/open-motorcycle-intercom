/** @file button.c External three-button polling driver. */
#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "button";
static const int s_pins[BUTTON_COUNT] = {
    BUTTON_MINUS_GPIO, BUTTON_CENTER_GPIO, BUTTON_PLUS_GPIO
};
static TaskHandle_t s_task;
static SemaphoreHandle_t s_stopped;
static button_callback_t s_callback;
static void *s_context;
static portMUX_TYPE s_callback_lock = portMUX_INITIALIZER_UNLOCKED;

static void read_buttons(bool pressed[BUTTON_COUNT])
{
    for (int i = 0; i < BUTTON_COUNT; ++i) {
        pressed[i] = gpio_get_level(s_pins[i]) == 0;
    }
}

static void button_task(void *arg)
{
    (void)arg;
    button_state_t state;
    bool pressed[BUTTON_COUNT];
    read_buttons(pressed);
    button_state_init(&state, (uint32_t)(esp_timer_get_time() / 1000), pressed);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BUTTON_POLL_MS)) != 0) {
            break;
        }
        read_buttons(pressed);
        button_id_t id;
        button_event_t event;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        while (button_state_sample(&state, now_ms, pressed, &id, &event)) {
            button_callback_t callback;
            void *context;
            taskENTER_CRITICAL(&s_callback_lock);
            callback = s_callback;
            context = s_context;
            taskEXIT_CRITICAL(&s_callback_lock);
            if (callback) {
                callback(id, event, context);
            }
        }
    }
    xSemaphoreGive(s_stopped);
    vTaskSuspend(NULL); /* deinit owns task deletion after the stop handshake. */
}

esp_err_t button_init(void)
{
    if (s_task || s_stopped) {
        return ESP_ERR_INVALID_STATE;
    }
    s_stopped = xSemaphoreCreateBinary();
    if (!s_stopped) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t config = {
        .pin_bit_mask = (1ULL << BUTTON_MINUS_GPIO) | (1ULL << BUTTON_CENTER_GPIO) |
                        (1ULL << BUTTON_PLUS_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Button GPIO configuration failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    if (xTaskCreate(button_task, "button", 3072, NULL, 2, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "Button task creation failed");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    return ESP_OK;

fail:
    for (int i = 0; i < BUTTON_COUNT; ++i) {
        gpio_reset_pin(s_pins[i]);
    }
    vSemaphoreDelete(s_stopped);
    s_stopped = NULL;
    return ret;
}

void button_deinit(void)
{
    if (!s_stopped) {
        return;
    }
    if (xTaskGetCurrentTaskHandle() == s_task) {
        ESP_LOGE(TAG, "button_deinit cannot run from the button callback");
        return;
    }
    xTaskNotifyGive(s_task);
    xSemaphoreTake(s_stopped, portMAX_DELAY);
    vTaskDelete(s_task);
    s_task = NULL;
    vSemaphoreDelete(s_stopped);
    s_stopped = NULL;
    for (int i = 0; i < BUTTON_COUNT; ++i) {
        gpio_reset_pin(s_pins[i]);
    }
    button_register_callback(NULL, NULL);
}

void button_register_callback(button_callback_t callback, void *context)
{
    taskENTER_CRITICAL(&s_callback_lock);
    s_callback = callback;
    s_context = context;
    taskEXIT_CRITICAL(&s_callback_lock);
}
