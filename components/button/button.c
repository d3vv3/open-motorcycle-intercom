/**
 * @file button.c
 * @brief Button handler implementation
 */

#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"

static const char *TAG = "button";

/* ============================================================================
 * State
 * ============================================================================ */

static QueueHandle_t s_button_queue = NULL;
static TaskHandle_t s_button_task = NULL;
static button_gesture_cb_t s_gesture_callback = NULL;
static bool s_initialized = false;
static atomic_uint s_queue_overflows;

/* Button event structure for queue */
typedef struct {
    int gpio;
    int level;
    int64_t timestamp_us;
} button_event_t;

/* ============================================================================
 * GPIO Interrupt Handler
 * ============================================================================ */

/**
 * @brief GPIO ISR handler
 *
 * Runs in interrupt context - just queue the event for processing.
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    int gpio = (int)arg;
    button_event_t evt = {
        .gpio = gpio,
        .level = gpio_get_level(gpio),
        .timestamp_us = esp_timer_get_time(),
    };

    if (xQueueSendFromISR(s_button_queue, &evt, NULL) != pdTRUE) {
        atomic_fetch_add_explicit(&s_queue_overflows, 1u, memory_order_relaxed);
    }
}

/* ============================================================================
 * Button Task
 * ============================================================================ */

/**
 * @brief Button processing task
 *
 * Handles debouncing and long press detection.
 */
static void button_task(void *arg)
{
    button_event_t evt;
    int64_t press_start_us = 0;
    bool button_pressed = false;
    unsigned last_reported_overflows = 0u;

    ESP_LOGI(TAG, "Button task started");

    while (1) {
        if (xQueueReceive(s_button_queue, &evt, pdMS_TO_TICKS(100))) {
            while (xQueueReceive(s_button_queue, &evt,
                                 pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) == pdTRUE) {
                /* Restart the quiet interval after each bounce edge. */
            }
            int stable_level = gpio_get_level(BUTTON_BOOT_GPIO);
            int64_t stable_timestamp_us = esp_timer_get_time();
            /* Button pressed (active low). */
            if (stable_level == 0 && !button_pressed) {
                press_start_us = stable_timestamp_us;
                button_pressed = true;
                ESP_LOGD(TAG, "Button pressed");
            }
            /* Button released. */
            else if (stable_level == 1 && button_pressed) {
                int64_t press_duration_ms = (stable_timestamp_us - press_start_us) / 1000;
                button_pressed = false;
                ESP_LOGD(TAG, "Button released after %lld ms", press_duration_ms);

                button_gesture_t gesture = button_classify_release_ms(press_duration_ms);
                if (gesture != BUTTON_GESTURE_NONE && s_gesture_callback != NULL) {
                    s_gesture_callback(gesture, BUTTON_BOOT_GPIO);
                }
            }
        }
        unsigned overflows = atomic_load_explicit(&s_queue_overflows, memory_order_relaxed);
        if (overflows != last_reported_overflows) {
            ESP_LOGW(TAG, "Button event queue overflowed %u time(s)",
                     overflows - last_reported_overflows);
            last_reported_overflows = overflows;
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t button_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Button already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing button handler (GPIO %d)", BUTTON_BOOT_GPIO);

    /* Configure GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create event queue */
    s_button_queue = xQueueCreate(10, sizeof(button_event_t));
    if (s_button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return ESP_ERR_NO_MEM;
    }

    /* Install GPIO ISR service */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means service already installed, which is OK */
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        vQueueDelete(s_button_queue);
        s_button_queue = NULL;
        return ret;
    }

    /* Hook ISR handler for specific GPIO */
    ret = gpio_isr_handler_add(BUTTON_BOOT_GPIO, button_isr_handler, (void *)BUTTON_BOOT_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        vQueueDelete(s_button_queue);
        s_button_queue = NULL;
        return ret;
    }

    /* Create button task */
    BaseType_t task_ret = xTaskCreate(button_task, "button", 3072, NULL, 5, &s_button_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        gpio_isr_handler_remove(BUTTON_BOOT_GPIO);
        vQueueDelete(s_button_queue);
        s_button_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Button handler initialized successfully");

    return ESP_OK;
}

void button_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing button handler");

    /* Remove ISR handler */
    gpio_isr_handler_remove(BUTTON_BOOT_GPIO);

    /* Delete task */
    if (s_button_task) {
        vTaskDelete(s_button_task);
        s_button_task = NULL;
    }

    /* Delete queue */
    if (s_button_queue) {
        vQueueDelete(s_button_queue);
        s_button_queue = NULL;
    }

    s_gesture_callback = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "Button handler deinitialized");
}

void button_register_gesture_callback(button_gesture_cb_t callback)
{
    s_gesture_callback = callback;
    if (callback) {
        ESP_LOGI(TAG, "Gesture callback registered");
    } else {
        ESP_LOGI(TAG, "Gesture callback unregistered");
    }
}
