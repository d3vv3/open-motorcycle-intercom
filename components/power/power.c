/**
 * @file power.c
 * @brief OMI Power Management Implementation - Phase 3
 *
 * Implements power state machine with:
 * - Radio duty cycling (slot-based on/off)
 * - Light sleep between TDMA frames
 * - Deep sleep with RTC wake
 * - Voice activity driven state transitions
 */

#include "power.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"

static const char *TAG = "power";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define POWER_TASK_STACK_SIZE 2048
#define POWER_TASK_PRIORITY   3 /* Lower priority than audio/mesh */

/* Light sleep threshold - don't sleep for less than this */
#define MIN_SLEEP_TIME_US 1000

/* Radio wake lead time before slot (microseconds) */
#define RADIO_WAKE_LEAD_US 500

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static power_config_t s_config = POWER_CONFIG_DEFAULT();
static power_state_t s_state = POWER_STATE_STANDBY;
static power_stats_t s_stats = {0};

/* State tracking */
static int64_t s_state_enter_time_ms = 0;
static int64_t s_last_activity_ms = 0;

/* Radio control */
static bool s_radio_active = true;
static SemaphoreHandle_t s_state_mutex = NULL;

/* Power management lock (prevents light sleep during critical sections) */
static esp_pm_lock_handle_t s_pm_lock = NULL;

/* Deep sleep wake tracking */
static bool s_is_deep_sleep_wake = false;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void update_state_time(power_state_t old_state);
static esp_err_t configure_wifi_power_save(bool aggressive);

/* ============================================================================
 * Power State Names
 * ============================================================================ */

static const char *s_state_names[] = {
    [POWER_STATE_DEEP_SLEEP] = "DEEP_SLEEP",
    [POWER_STATE_STANDBY] = "STANDBY",
    [POWER_STATE_MESH_IDLE] = "MESH_IDLE",
    [POWER_STATE_ACTIVE_VOICE] = "ACTIVE_VOICE",
};

/* ============================================================================
 * Public Functions - Initialization
 * ============================================================================ */

esp_err_t power_init(void)
{
    return power_init_with_config(NULL);
}

esp_err_t power_init_with_config(const power_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config != NULL) {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing power management (Phase 3)");
    ESP_LOGI(TAG, "  Radio duty cycle: %s", s_config.enable_radio_duty_cycle ? "ON" : "OFF");
    ESP_LOGI(TAG, "  Light sleep: %s", s_config.enable_light_sleep ? "ON" : "OFF");
    ESP_LOGI(TAG, "  Deep sleep timeout: %lu sec", (unsigned long)s_config.deep_sleep_timeout_sec);

    /* Check wake reason */
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    s_is_deep_sleep_wake =
        (wake_cause == ESP_SLEEP_WAKEUP_TIMER || wake_cause == ESP_SLEEP_WAKEUP_GPIO ||
         wake_cause == ESP_SLEEP_WAKEUP_EXT0 || wake_cause == ESP_SLEEP_WAKEUP_EXT1);

    if (s_is_deep_sleep_wake) {
        ESP_LOGI(TAG, "Woke from deep sleep (cause: %d)", wake_cause);
    }

    /* Create state mutex */
    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Create PM lock for preventing light sleep during critical sections */
#if CONFIG_PM_ENABLE
    esp_err_t ret = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "power_lock", &s_pm_lock);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create PM lock (PM may not be enabled): %s", esp_err_to_name(ret));
        s_pm_lock = NULL;
    }
#endif

    /* Initialize state */
    s_state = POWER_STATE_STANDBY;
    s_state_enter_time_ms = esp_timer_get_time() / 1000;
    s_last_activity_ms = s_state_enter_time_ms;
    s_radio_active = true;

    /* Clear stats */
    memset(&s_stats, 0, sizeof(s_stats));

    s_initialized = true;
    ESP_LOGI(TAG, "Power management initialized");

    return ESP_OK;
}

esp_err_t power_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing power management");

    /* Release PM lock if held */
#if CONFIG_PM_ENABLE
    if (s_pm_lock) {
        esp_pm_lock_delete(s_pm_lock);
        s_pm_lock = NULL;
    }
#endif

    /* Cleanup mutex */
    if (s_state_mutex) {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
    }

    s_initialized = false;

    return ESP_OK;
}

/* ============================================================================
 * Public Functions - State Control
 * ============================================================================ */

esp_err_t power_set_state(power_state_t state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == s_state) {
        return ESP_OK; /* Already in this state */
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    power_state_t old_state = s_state;
    update_state_time(old_state);

    ESP_LOGI(TAG, "State transition: %s -> %s", s_state_names[old_state], s_state_names[state]);

    s_state = state;
    s_state_enter_time_ms = esp_timer_get_time() / 1000;
    s_stats.state_changes++;

    /* State-specific actions */
    switch (state) {
    case POWER_STATE_DEEP_SLEEP:
        /* Will be handled by power_enter_deep_sleep() */
        break;

    case POWER_STATE_STANDBY:
        /* Aggressive power save - radio mostly off */
        configure_wifi_power_save(true);
        break;

    case POWER_STATE_MESH_IDLE:
        /* Normal power save - radio on for slots only */
        configure_wifi_power_save(true);
        break;

    case POWER_STATE_ACTIVE_VOICE:
        /* Performance mode - radio always ready */
        configure_wifi_power_save(false);
        s_stats.voice_activations++;
        break;
    }

    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}

power_state_t power_get_state(void)
{
    return s_state;
}

const char *power_state_str(power_state_t state)
{
    if (state < sizeof(s_state_names) / sizeof(s_state_names[0])) {
        return s_state_names[state];
    }
    return "UNKNOWN";
}

/* ============================================================================
 * Public Functions - Voice Activity
 * ============================================================================ */

void power_notify_voice_start(void)
{
    if (!s_initialized) {
        return;
    }

    power_activity_ping();

    if (s_state == POWER_STATE_MESH_IDLE) {
        power_set_state(POWER_STATE_ACTIVE_VOICE);
    }

    ESP_LOGD(TAG, "Voice started");
}

void power_notify_voice_end(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_state == POWER_STATE_ACTIVE_VOICE) {
        power_set_state(POWER_STATE_MESH_IDLE);
    }

    ESP_LOGD(TAG, "Voice ended");
}

/* ============================================================================
 * Public Functions - Radio Control
 * ============================================================================ */

void power_radio_slot_start(void)
{
    if (!s_initialized || !s_config.enable_radio_duty_cycle) {
        return;
    }

    if (!s_radio_active) {
        /* Wake radio from power save */
        esp_wifi_set_ps(WIFI_PS_NONE);
        s_radio_active = true;
        s_stats.radio_duty_cycles++;
        ESP_LOGD(TAG, "Radio: ON (slot start)");
    }

    /* Acquire PM lock to prevent sleep during slot */
#if CONFIG_PM_ENABLE
    if (s_pm_lock) {
        esp_pm_lock_acquire(s_pm_lock);
    }
#endif
}

void power_radio_slot_end(void)
{
    if (!s_initialized || !s_config.enable_radio_duty_cycle) {
        return;
    }

    /* Release PM lock */
#if CONFIG_PM_ENABLE
    if (s_pm_lock) {
        esp_pm_lock_release(s_pm_lock);
    }
#endif

    /* Only put radio to sleep in MESH_IDLE state */
    if (s_state == POWER_STATE_MESH_IDLE && s_radio_active) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        s_radio_active = false;
        ESP_LOGD(TAG, "Radio: SLEEP (slot end)");
    }
}

bool power_is_radio_active(void)
{
    return s_radio_active;
}

/* ============================================================================
 * Public Functions - Activity Tracking
 * ============================================================================ */

void power_activity_ping(void)
{
    s_last_activity_ms = esp_timer_get_time() / 1000;
}

uint32_t power_get_idle_time_ms(void)
{
    int64_t now = esp_timer_get_time() / 1000;
    return (uint32_t)(now - s_last_activity_ms);
}

/* ============================================================================
 * Public Functions - Statistics
 * ============================================================================ */

esp_err_t power_get_stats(power_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Update current state time before returning */
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    update_state_time(s_state);
    *stats = s_stats;
    stats->last_activity_ms = s_last_activity_ms;
    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}

esp_err_t power_reset_stats(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memset(&s_stats, 0, sizeof(s_stats));
    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}

/* ============================================================================
 * Public Functions - Deep Sleep
 * ============================================================================ */

void power_enter_deep_sleep(uint32_t wake_after_sec)
{
    ESP_LOGI(TAG, "Entering deep sleep (wake after %lu sec)", (unsigned long)wake_after_sec);

    /* Update state time */
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    update_state_time(s_state);
    s_state = POWER_STATE_DEEP_SLEEP;
    xSemaphoreGive(s_state_mutex);

    /* Configure wake source */
    if (wake_after_sec > 0) {
        esp_sleep_enable_timer_wakeup((uint64_t)wake_after_sec * 1000000ULL);
        ESP_LOGI(TAG, "RTC timer wake configured for %lu seconds", (unsigned long)wake_after_sec);
    }

    /* Could also configure GPIO wake here if needed:
     * esp_sleep_enable_ext0_wakeup(GPIO_NUM, level);
     */

    /* Enter deep sleep - this does not return */
    esp_deep_sleep_start();
}

bool power_is_deep_sleep_wake(void)
{
    return s_is_deep_sleep_wake;
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Update time tracking for state transitions
 */
static void update_state_time(power_state_t old_state)
{
    int64_t now = esp_timer_get_time() / 1000;
    int64_t duration = now - s_state_enter_time_ms;

    if (old_state < 4 && duration > 0) {
        s_stats.time_in_state_ms[old_state] += duration;
    }

    s_state_enter_time_ms = now;
}

/**
 * @brief Configure WiFi power save mode
 *
 * Note: WiFi power save is DISABLED for ESP-NOW mesh operation because
 * PS_MIN_MODEM causes significant packet loss (~16%+). The radio must
 * be always listening to reliably receive ESP-NOW packets.
 *
 * Power savings will be achieved in Phase 6 with nRF54 duty cycling.
 */
static esp_err_t configure_wifi_power_save(bool aggressive)
{
    (void)aggressive; /* Ignored - always use PS_NONE for reliability */

    esp_err_t ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi PS mode: %s", esp_err_to_name(ret));
    }

    return ret;
}
