/**
 * @file audio.c
 * @brief Audio subsystem facade: lifecycle, configuration, and public API.
 */

#include "audio_internal.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "audio";

audio_context_t g_audio;
portMUX_TYPE g_audio_stats_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_audio_task_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_audio_far_ref_lock = portMUX_INITIALIZER_UNLOCKED;

audio_stats_t audio_stats_snapshot(void)
{
    audio_stats_t snapshot;
    AUDIO_STATS_LOCK();
    snapshot = g_audio.stats;
    AUDIO_STATS_UNLOCK();
    return snapshot;
}

bool audio_called_from_worker(void)
{
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&g_audio_task_lock);
    bool is_worker = current == g_audio.capture_task || current == g_audio.playout_task;
    portEXIT_CRITICAL(&g_audio_task_lock);
    return is_worker;
}

static void delete_sync_resources(void)
{
    if (g_audio.notification_queue != NULL) {
        vQueueDelete(g_audio.notification_queue);
        g_audio.notification_queue = NULL;
    }
    if (g_audio.loopback_queue != NULL) {
        vQueueDelete(g_audio.loopback_queue);
        g_audio.loopback_queue = NULL;
    }
    SemaphoreHandle_t *semaphores[] = {
        &g_audio.rx_sources_mutex, &g_audio.rx_reset_mutex, &g_audio.rx_reset_done,
        &g_audio.playout_started, &g_audio.capture_started, &g_audio.capture_done,
        &g_audio.playout_done,
    };
    for (size_t i = 0; i < sizeof(semaphores) / sizeof(semaphores[0]); ++i) {
        if (*semaphores[i] != NULL) {
            vSemaphoreDelete(*semaphores[i]);
            *semaphores[i] = NULL;
        }
    }
}

static esp_err_t create_sync_resources(void)
{
    g_audio.playout_started = xSemaphoreCreateBinary();
    g_audio.capture_started = xSemaphoreCreateBinary();
    g_audio.capture_done = xSemaphoreCreateBinary();
    g_audio.playout_done = xSemaphoreCreateBinary();
    g_audio.rx_reset_done = xSemaphoreCreateBinary();
    g_audio.rx_reset_mutex = xSemaphoreCreateMutex();
    g_audio.rx_sources_mutex = xSemaphoreCreateMutex();
    g_audio.loopback_queue = xQueueCreate(LOOPBACK_QUEUE_SIZE, sizeof(audio_loopback_item_t));
    g_audio.notification_queue =
        xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(audio_notification_request_t));
    if (g_audio.playout_started == NULL || g_audio.capture_started == NULL ||
        g_audio.capture_done == NULL || g_audio.playout_done == NULL ||
        g_audio.rx_reset_done == NULL || g_audio.rx_reset_mutex == NULL ||
        g_audio.rx_sources_mutex == NULL || g_audio.loopback_queue == NULL ||
        g_audio.notification_queue == NULL) {
        delete_sync_resources();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static bool config_supported(const audio_config_t *config)
{
    return config->sample_rate == 16000 && config->channels == 1 &&
           config->bits_per_sample == 16 && config->frame_size_ms == 20;
}

static esp_err_t audio_init_with_config_locked(const audio_config_t *config)
{
    if (g_audio.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    audio_config_t requested = AUDIO_CONFIG_DEFAULT();
    if (config != NULL) {
        requested = *config;
    }
    if (!config_supported(&requested)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (requested.mode != AUDIO_MODE_LOOPBACK && requested.mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_ARG;
    }
    g_audio.config = requested;

    esp_err_t ret = audio_hw_adc_init(&g_audio.config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = audio_hw_i2s_init(&g_audio.config);
    if (ret != ESP_OK) {
        audio_hw_adc_deinit();
        return ret;
    }
    ret = audio_hw_opus_init(&g_audio.config);
    if (ret != ESP_OK) {
        audio_hw_opus_deinit();
        audio_hw_i2s_deinit();
        audio_hw_adc_deinit();
        return ret;
    }
    ret = create_sync_resources();
    if (ret != ESP_OK) {
        audio_hw_opus_deinit();
        audio_hw_i2s_deinit();
        audio_hw_adc_deinit();
        return ret;
    }

    AUDIO_STATS_LOCK();
    memset(&g_audio.stats, 0, sizeof(g_audio.stats));
    g_audio.tx_pipe_sum_us = 0u;
    g_audio.tx_pipe_count = 0u;
    g_audio.rx_pipe_sum_us = 0u;
    g_audio.rx_pipe_count = 0u;
    AUDIO_STATS_UNLOCK();
    memset(&g_audio.notification, 0, sizeof(g_audio.notification));
    audio_playout_reset_far_reference();
    audio_capture_init_dsp();
    audio_rx_reset_source_metadata();
    g_audio.initialized = true;
    ESP_LOGI(TAG, "Audio subsystem initialized");
    return ESP_OK;
}

esp_err_t audio_init(void)
{
    return audio_init_with_config(NULL);
}

esp_err_t audio_init_with_config(const audio_config_t *config)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.lifecycle_mutex == NULL) {
        g_audio.lifecycle_mutex =
            xSemaphoreCreateMutexStatic(&g_audio.lifecycle_mutex_storage);
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret = g_audio.deinitializing ? ESP_ERR_INVALID_STATE
                                           : audio_init_with_config_locked(config);
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ret;
}

static esp_err_t audio_stop_locked(void)
{
    if (!g_audio.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    portENTER_CRITICAL(&g_audio_task_lock);
    bool wait_capture = g_audio.capture_task != NULL;
    bool wait_playout = g_audio.playout_task != NULL;
    if (wait_capture) {
        xTaskNotifyGive(g_audio.capture_task);
    }
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (wait_capture) {
        xSemaphoreTake(g_audio.capture_done, portMAX_DELAY);
    }
    if (wait_playout) {
        xSemaphoreTake(g_audio.playout_done, portMAX_DELAY);
    }
    audio_rx_reset_source_metadata();
    xQueueReset(g_audio.loopback_queue);
    xQueueReset(g_audio.notification_queue);
    memset(&g_audio.notification, 0, sizeof(g_audio.notification));
    audio_playout_reset_far_reference();
    AUDIO_STATS_LOCK();
    g_audio.stats.asrc_correction_ppm = 0;
    g_audio.stats.asrc_recovery_active = false;
    g_audio.stats.rx_pipe_us_avg = 0u;
    g_audio.stats.rx_pipe_us_max = 0u;
    g_audio.rx_pipe_sum_us = 0u;
    g_audio.rx_pipe_count = 0u;
    AUDIO_STATS_UNLOCK();
    return ESP_OK;
}

esp_err_t audio_deinit(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    g_audio.deinitializing = true;
    g_audio.stopping = true;
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    if (ret != ESP_OK) {
        g_audio.deinitializing = false;
        g_audio.stopping = false;
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ret;
    }
    audio_rx_reset_source_metadata();
    audio_hw_opus_deinit();
    audio_hw_i2s_deinit();
    audio_hw_adc_deinit();
    delete_sync_resources();
    g_audio.initialized = false;
    g_audio.deinitializing = false;
    g_audio.stopping = false;
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ESP_OK;
}

static esp_err_t audio_start_locked(void)
{
    if (!g_audio.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&g_audio_task_lock);
    bool tasks_exist = g_audio.capture_task != NULL || g_audio.playout_task != NULL;
    portEXIT_CRITICAL(&g_audio_task_lock);
    if (atomic_load_explicit(&g_audio.running, memory_order_acquire) || tasks_exist) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.playout_started, 0);
    xSemaphoreTake(g_audio.capture_started, 0);
    xSemaphoreTake(g_audio.playout_done, 0);
    xSemaphoreTake(g_audio.capture_done, 0);
    xQueueReset(g_audio.loopback_queue);
    atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.capture_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.running, true, memory_order_release);

    BaseType_t created = xTaskCreatePinnedToCore(
        audio_playout_task, "audio_playout", AUDIO_PLAYOUT_TASK_STACK_SIZE, NULL,
        AUDIO_PLAYOUT_TASK_PRIORITY, &g_audio.playout_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        g_audio.playout_task = NULL;
        atomic_store_explicit(&g_audio.running, false, memory_order_release);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_audio.playout_started, portMAX_DELAY);
    if (!atomic_load_explicit(&g_audio.playout_ready, memory_order_acquire)) {
        xSemaphoreTake(g_audio.playout_done, portMAX_DELAY);
        return ESP_FAIL;
    }

    created = xTaskCreatePinnedToCore(
        audio_capture_task, "audio_capture", AUDIO_CAPTURE_TASK_STACK_SIZE, NULL,
        AUDIO_CAPTURE_TASK_PRIORITY, &g_audio.capture_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        g_audio.capture_task = NULL;
        atomic_store_explicit(&g_audio.running, false, memory_order_release);
        xSemaphoreTake(g_audio.playout_done, portMAX_DELAY);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_audio.capture_started, portMAX_DELAY);
    if (!atomic_load_explicit(&g_audio.capture_ready, memory_order_acquire)) {
        xSemaphoreTake(g_audio.capture_done, portMAX_DELAY);
        xSemaphoreTake(g_audio.playout_done, portMAX_DELAY);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret = (g_audio.stopping || g_audio.deinitializing) ? ESP_ERR_INVALID_STATE
                                                                 : audio_start_locked();
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ret;
}

esp_err_t audio_stop(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.stopping) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_OK;
    }
    g_audio.stopping = true;
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    g_audio.stopping = false;
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ret;
}

bool audio_vox_active(void)
{
    audio_stats_t stats = audio_stats_snapshot();
    return stats.vox_active;
}

/* FIXME(api): legacy pull API kept only for link compatibility; frames flow
 * through the TX callback. Remove together with the header declaration. */
esp_err_t audio_get_tx_frame(audio_frame_t *frame, uint32_t timeout_ms)
{
    if (!g_audio.initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t audio_register_tx_callback(audio_tx_cb_t callback)
{
    portENTER_CRITICAL(&g_audio_task_lock);
    g_audio.tx_callback = callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    return ESP_OK;
}

esp_err_t audio_register_activity_callback(audio_activity_cb_t callback)
{
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&g_audio_task_lock);
    g_audio.activity_callback = callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ESP_OK;
}

esp_err_t audio_set_mode(audio_mode_t mode)
{
    if (g_audio.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    if (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (mode != AUDIO_MODE_LOOPBACK && mode != AUDIO_MODE_MESH) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(g_audio.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    g_audio.config.mode = mode;
    audio_rx_reset_source_metadata();
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return ESP_OK;
}

audio_mode_t audio_get_mode(void)
{
    if (g_audio.lifecycle_mutex == NULL) {
        return AUDIO_MODE_LOOPBACK;
    }
    if (audio_called_from_worker()) {
        return g_audio.config.mode;
    }
    xSemaphoreTake(g_audio.lifecycle_mutex, portMAX_DELAY);
    audio_mode_t mode = g_audio.config.mode;
    xSemaphoreGive(g_audio.lifecycle_mutex);
    return mode;
}

esp_err_t audio_get_stats(audio_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *stats = audio_stats_snapshot();
    return ESP_OK;
}

esp_err_t audio_record_tx_pipeline_latency_us(uint32_t latency_us)
{
    AUDIO_STATS_LOCK();
    g_audio.tx_pipe_count++;
    g_audio.tx_pipe_sum_us += latency_us;
    g_audio.stats.tx_pipe_us_avg = (uint32_t)(g_audio.tx_pipe_sum_us / g_audio.tx_pipe_count);
    if (latency_us > g_audio.stats.tx_pipe_us_max) {
        g_audio.stats.tx_pipe_us_max = latency_us;
    }
    AUDIO_STATS_UNLOCK();
    return ESP_OK;
}
