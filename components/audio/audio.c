/**
 * @file audio.c
 * @brief Audio subsystem facade: lifecycle, configuration, and public API.
 */

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#include "audio_internal.h"

static const char *TAG = "audio";

audio_context_t g_audio;
portMUX_TYPE g_audio_stats_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_audio_task_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_audio_far_ref_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_audio_lifecycle_init_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_audio_route_handle_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_audio_route_users;
static bool s_audio_route_closing;

static void free_audio_route_storage(void)
{
    int16_t *storage[] = {g_audio.bluetooth_music.samples, g_audio.bluetooth_call.samples,
                          g_audio.bluetooth_mic.samples};
    memset(&g_audio.bluetooth_music, 0, sizeof(g_audio.bluetooth_music));
    memset(&g_audio.bluetooth_call, 0, sizeof(g_audio.bluetooth_call));
    memset(&g_audio.bluetooth_mic, 0, sizeof(g_audio.bluetooth_mic));
    for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); ++i) {
        heap_caps_free(storage[i]);
    }
    heap_caps_free(g_audio.playback_psram_storage);
    g_audio.playback_psram_storage = NULL;
    g_audio.voice_converted = NULL;
    g_audio.voice_fifo_storage = NULL;
    g_audio.voice_presence_fifo_storage = NULL;
    g_audio.voice_presence_staging = NULL;
    g_audio.voice_presence_frame = NULL;
    g_audio.music_input = NULL;
    g_audio.music_converted = NULL;
    g_audio.music_fifo_storage = NULL;
    g_audio.far_fifo_storage = NULL;
    g_audio.far_converted = NULL;
    g_audio.far_pending_frame = NULL;
}

static esp_err_t allocate_playback_psram_storage(void)
{
    const size_t playback_samples =
        4u * AUDIO_PLAYBACK_CONVERTED_CAPACITY + AUDIO_HW_FRAME_SAMPLES +
        2u * AUDIO_PLAYBACK_FIFO_CAPACITY + AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES +
        AUDIO_FAR_FIFO_CAPACITY + 2u * AUDIO_FRAME_SAMPLES;
    int16_t *storage = heap_caps_calloc(playback_samples, sizeof(*storage),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (storage == NULL) {
        return ESP_ERR_NO_MEM;
    }
    g_audio.playback_psram_storage = storage;
    size_t offset = 0u;
#define ASSIGN_PLAYBACK_BUFFER(field, count) \
    do { \
        g_audio.field = &storage[offset]; \
        offset += (count); \
    } while (0)
    ASSIGN_PLAYBACK_BUFFER(voice_converted, AUDIO_PLAYBACK_CONVERTED_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(voice_fifo_storage, AUDIO_PLAYBACK_FIFO_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(voice_presence_fifo_storage, AUDIO_PLAYBACK_FIFO_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(voice_presence_staging, AUDIO_PLAYBACK_CONVERTED_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(voice_presence_frame, AUDIO_HW_FRAME_SAMPLES);
    ASSIGN_PLAYBACK_BUFFER(music_converted, AUDIO_PLAYBACK_CONVERTED_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(music_fifo_storage, AUDIO_PLAYBACK_FIFO_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(music_input, AUDIO_RATE_CONVERTER_MAX_INPUT_FRAMES);
    ASSIGN_PLAYBACK_BUFFER(far_fifo_storage, AUDIO_FAR_FIFO_CAPACITY);
    ASSIGN_PLAYBACK_BUFFER(far_converted, AUDIO_FRAME_SAMPLES);
    ASSIGN_PLAYBACK_BUFFER(far_pending_frame, AUDIO_FRAME_SAMPLES);
#undef ASSIGN_PLAYBACK_BUFFER
    configASSERT(offset == playback_samples);
    ESP_LOGI(TAG, "Playback staging: PSRAM allocated=%u bytes; internal free=%u largest=%u",
             (unsigned)(playback_samples * sizeof(*storage)),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return ESP_OK;
}

static esp_err_t allocate_audio_route_storage(void)
{
    size_t psram_size = esp_psram_get_size();
    if (!esp_psram_is_initialized() || psram_size < (16u * 1024u * 1024u)) {
        ESP_LOGE(TAG, "PSRAM unavailable or unexpected size: %u bytes", (unsigned)psram_size);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PSRAM: %u bytes", (unsigned)psram_size);

    int16_t **storage[] = {&g_audio.bluetooth_music.samples, &g_audio.bluetooth_call.samples,
                           &g_audio.bluetooth_mic.samples};
    audio_route_stream_t *streams[] = {&g_audio.bluetooth_music, &g_audio.bluetooth_call,
                                       &g_audio.bluetooth_mic};
    for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); ++i) {
        *storage[i] = heap_caps_calloc(AUDIO_ROUTE_INPUT_CAPACITY, sizeof(int16_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (*storage[i] == NULL) {
            free_audio_route_storage();
            return ESP_ERR_NO_MEM;
        }
        streams[i]->capacity = AUDIO_ROUTE_INPUT_CAPACITY;
    }
    return ESP_OK;
}

static void log_audio_worker_memory(const char *worker)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGI(TAG, "%s: internal free=%u largest=%u", worker,
             (unsigned)heap_caps_get_free_size(caps),
             (unsigned)heap_caps_get_largest_free_block(caps));
}

SemaphoreHandle_t audio_lifecycle_mutex_get(void)
{
    portENTER_CRITICAL(&s_audio_lifecycle_init_lock);
    if (g_audio.lifecycle_mutex == NULL) {
        g_audio.lifecycle_mutex = xSemaphoreCreateMutexStatic(&g_audio.lifecycle_mutex_storage);
    }
    SemaphoreHandle_t mutex = g_audio.lifecycle_mutex;
    portEXIT_CRITICAL(&s_audio_lifecycle_init_lock);
    return mutex;
}

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
    portENTER_CRITICAL(&s_audio_route_handle_lock);
    s_audio_route_closing = true;
    portEXIT_CRITICAL(&s_audio_route_handle_lock);
    /* Route callers count themselves before taking the mutex, so deletion waits for every
     * in-flight caller before deleting the mutex or any route-backed storage. */
    for (;;) {
        portENTER_CRITICAL(&s_audio_route_handle_lock);
        bool users_done = s_audio_route_users == 0u;
        portEXIT_CRITICAL(&s_audio_route_handle_lock);
        if (users_done) {
            break;
        }
        vTaskDelay(1);
    }
    if (g_audio.route_mutex != NULL) {
        vSemaphoreDelete(g_audio.route_mutex);
        g_audio.route_mutex = NULL;
    }
    if (g_audio.music_playback_mutex != NULL) {
        vSemaphoreDelete(g_audio.music_playback_mutex);
        g_audio.music_playback_mutex = NULL;
    }
    if (g_audio.notification_queue != NULL) {
        vQueueDelete(g_audio.notification_queue);
        g_audio.notification_queue = NULL;
    }
    if (g_audio.loopback_queue != NULL) {
        vQueueDelete(g_audio.loopback_queue);
        g_audio.loopback_queue = NULL;
    }
    SemaphoreHandle_t *semaphores[] = {
        &g_audio.rx_sources_mutex, &g_audio.rx_reset_mutex,  &g_audio.rx_reset_done,
        &g_audio.playout_started,  &g_audio.capture_started, &g_audio.capture_done,
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
    g_audio.route_mutex = xSemaphoreCreateMutex();
    g_audio.music_playback_mutex = xSemaphoreCreateMutex();
    g_audio.loopback_queue = xQueueCreate(LOOPBACK_QUEUE_SIZE, sizeof(audio_loopback_item_t));
    g_audio.notification_queue =
        xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(audio_notification_request_t));
    if (g_audio.playout_started == NULL || g_audio.capture_started == NULL ||
        g_audio.capture_done == NULL || g_audio.playout_done == NULL ||
        g_audio.rx_reset_done == NULL || g_audio.rx_reset_mutex == NULL ||
        g_audio.rx_sources_mutex == NULL || g_audio.route_mutex == NULL ||
        g_audio.music_playback_mutex == NULL ||
        g_audio.loopback_queue == NULL ||
        g_audio.notification_queue == NULL) {
        delete_sync_resources();
        return ESP_ERR_NO_MEM;
    }
    portENTER_CRITICAL(&s_audio_route_handle_lock);
    s_audio_route_users = 0u;
    s_audio_route_closing = false;
    portEXIT_CRITICAL(&s_audio_route_handle_lock);
    return ESP_OK;
}

bool audio_route_lock_acquire(TickType_t wait_ticks)
{
    portENTER_CRITICAL(&s_audio_route_handle_lock);
    SemaphoreHandle_t mutex = s_audio_route_closing ? NULL : g_audio.route_mutex;
    if (mutex != NULL) {
        s_audio_route_users++;
    }
    portEXIT_CRITICAL(&s_audio_route_handle_lock);
    if (mutex == NULL || xSemaphoreTake(mutex, wait_ticks) != pdTRUE) {
        if (mutex != NULL) {
            portENTER_CRITICAL(&s_audio_route_handle_lock);
            s_audio_route_users--;
            portEXIT_CRITICAL(&s_audio_route_handle_lock);
        }
        return false;
    }
    return true;
}

void audio_route_lock_release(void)
{
    xSemaphoreGive(g_audio.route_mutex);
    portENTER_CRITICAL(&s_audio_route_handle_lock);
    s_audio_route_users--;
    portEXIT_CRITICAL(&s_audio_route_handle_lock);
}

static bool config_supported(const audio_config_t *config)
{
    return config->sample_rate == 16000 && config->channels == 1 && config->bits_per_sample == 16 &&
           config->frame_size_ms == 20;
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

    if (audio_rate_converter_create(&g_audio.capture_rate_converter, AUDIO_HW_SAMPLE_RATE,
                                    g_audio.config.sample_rate) != 0 ||
        audio_rate_converter_create(&g_audio.voice_playback_converter, g_audio.config.sample_rate,
                                    AUDIO_HW_SAMPLE_RATE) != 0 ||
        audio_rate_converter_create(&g_audio.far_reference_converter, AUDIO_HW_SAMPLE_RATE,
                                    g_audio.config.sample_rate) != 0 ||
        !audio_capture_fifo_init(&g_audio.capture_fifo, g_audio.capture_fifo_storage,
                                 AUDIO_CAPTURE_FIFO_CAPACITY)) {
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = allocate_playback_psram_storage();
    if (ret != ESP_OK) {
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }
    ret = allocate_audio_route_storage();
    if (ret != ESP_OK) {
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }
    ret = audio_hw_i2s_init(&g_audio.config);
    if (ret != ESP_OK) {
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }
    ret = audio_hw_codec_init(&g_audio.config);
    if (ret != ESP_OK) {
        audio_hw_i2s_deinit();
        audio_hw_i2c_deinit();
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }
    ret = audio_hw_opus_init(&g_audio.config);
    if (ret != ESP_OK) {
        audio_hw_opus_deinit();
        audio_hw_codec_deinit();
        audio_hw_i2s_deinit();
        audio_hw_i2c_deinit();
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }
    ret = create_sync_resources();
    if (ret != ESP_OK) {
        audio_hw_opus_deinit();
        audio_hw_codec_deinit();
        audio_hw_i2s_deinit();
        audio_hw_i2c_deinit();
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ret;
    }

    AUDIO_STATS_LOCK();
    memset(&g_audio.stats, 0, sizeof(g_audio.stats));
    g_audio.tx_pipe_sum_us = 0u;
    g_audio.tx_pipe_count = 0u;
    g_audio.rx_pipe_sum_us = 0u;
    g_audio.rx_pipe_count = 0u;
    g_audio.bluetooth_music_conversion_us_sum = 0u;
    g_audio.bluetooth_music_conversion_frames = 0u;
    g_audio.playout_work_us_sum = 0u;
    g_audio.playout_work_frames = 0u;
    g_audio.capture_read_us_sum = 0u;
    g_audio.capture_convert_us_sum = 0u;
    g_audio.capture_aec_us_sum = 0u;
    g_audio.capture_loop_us_sum = 0u;
    g_audio.playout_write_us_sum = 0u;
    g_audio.stats.capture_peak_abs = 0u;
    AUDIO_STATS_UNLOCK();
    memset(&g_audio.notification, 0, sizeof(g_audio.notification));
    if (!audio_route_init()) {
        delete_sync_resources();
        audio_hw_opus_deinit();
        audio_hw_codec_deinit();
        audio_hw_i2s_deinit();
        audio_hw_i2c_deinit();
        free_audio_route_storage();
        audio_rate_converter_close(&g_audio.capture_rate_converter);
        audio_rate_converter_close(&g_audio.voice_playback_converter);
        audio_rate_converter_close(&g_audio.far_reference_converter);
        return ESP_ERR_NO_MEM;
    }
    audio_playout_reset_far_reference();
    audio_capture_init_dsp();
    audio_rx_reset_source_metadata();
    g_audio.initialized = true;
    ESP_LOGI(TAG, "Audio subsystem initialized");
    return ESP_OK;
}

static void audio_join_workers_locked(void);
static esp_err_t audio_start_locked(void);

static void audio_unwind_locked(void)
{
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    atomic_store_explicit(&g_audio.call_priority_active, false, memory_order_release);
    audio_join_workers_locked();
    audio_hw_codec_stop();
    (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
    audio_capture_fifo_reset(&g_audio.capture_fifo);
    audio_rx_reset_source_metadata();
    audio_hw_opus_deinit();
    audio_hw_codec_deinit();
    audio_hw_i2s_deinit();
    audio_hw_i2c_deinit();
    audio_rate_converter_close(&g_audio.capture_rate_converter);
    audio_rate_converter_close(&g_audio.voice_playback_converter);
    audio_rate_converter_close(&g_audio.far_reference_converter);
    if (g_audio.music_playback_mutex != NULL &&
        xSemaphoreTake(g_audio.music_playback_mutex, portMAX_DELAY) == pdTRUE) {
        audio_rate_converter_close(&g_audio.music_playback_converter);
        xSemaphoreGive(g_audio.music_playback_mutex);
    } else if (g_audio.music_playback_mutex == NULL) {
        audio_rate_converter_close(&g_audio.music_playback_converter);
    }
    delete_sync_resources();
    free_audio_route_storage();
    atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.capture_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.rx_reset_requested, false, memory_order_release);
    g_audio.initialized = false;
    g_audio.stopping = false;
    g_audio.deinitializing = false;
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
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret =
        g_audio.deinitializing ? ESP_ERR_INVALID_STATE : audio_init_with_config_locked(config);
    xSemaphoreGive(lifecycle_mutex);
    return ret;
}

esp_err_t audio_init_and_start_with_config(const audio_config_t *config)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    bool initialized_by_call = false;
    if (g_audio.deinitializing) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        ret = audio_init_with_config_locked(config);
        if (ret == ESP_OK) {
            initialized_by_call = true;
            ret = audio_start_locked();
        }
        if (ret != ESP_OK && initialized_by_call) {
            audio_unwind_locked();
        }
    }
    xSemaphoreGive(lifecycle_mutex);
    return ret;
}

static void audio_join_workers_locked(void)
{
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
}

static esp_err_t audio_stop_locked(void)
{
    if (!g_audio.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    audio_join_workers_locked();
    audio_hw_codec_stop();
    (void)audio_rate_converter_reset(g_audio.capture_rate_converter);
    (void)audio_rate_converter_reset(g_audio.voice_playback_converter);
    (void)audio_rate_converter_reset(g_audio.far_reference_converter);
    audio_capture_fifo_reset(&g_audio.capture_fifo);
    audio_rx_reset_source_metadata();
    xQueueReset(g_audio.loopback_queue);
    xQueueReset(g_audio.notification_queue);
    memset(&g_audio.notification, 0, sizeof(g_audio.notification));
    audio_route_reset_streams();
    audio_sample_fifo_reset(&g_audio.far_fifo);
    audio_sample_fifo_reset(&g_audio.voice_fifo);
    audio_sample_fifo_reset(&g_audio.voice_presence_fifo);
    audio_playout_reset_far_reference();
    AUDIO_STATS_LOCK();
    g_audio.stats.asrc_correction_ppm = 0;
    g_audio.stats.asrc_recovery_active = false;
    g_audio.stats.rx_pipe_us_avg = 0u;
    g_audio.stats.rx_pipe_us_max = 0u;
    g_audio.rx_pipe_sum_us = 0u;
    g_audio.rx_pipe_count = 0u;
    g_audio.stats.capture_peak_abs = 0u;
    AUDIO_STATS_UNLOCK();
    return ESP_OK;
}

esp_err_t audio_deinit(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    g_audio.deinitializing = true;
    g_audio.stopping = true;
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    atomic_store_explicit(&g_audio.call_priority_active, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    if (ret != ESP_OK) {
        g_audio.deinitializing = false;
        g_audio.stopping = false;
        xSemaphoreGive(lifecycle_mutex);
        return ret;
    }
    audio_rx_reset_source_metadata();
    audio_hw_opus_deinit();
    audio_hw_codec_deinit();
    audio_hw_i2s_deinit();
    audio_hw_i2c_deinit();
    audio_rate_converter_close(&g_audio.capture_rate_converter);
    audio_rate_converter_close(&g_audio.voice_playback_converter);
    audio_rate_converter_close(&g_audio.far_reference_converter);
    if (g_audio.music_playback_mutex != NULL &&
        xSemaphoreTake(g_audio.music_playback_mutex, portMAX_DELAY) == pdTRUE) {
        audio_rate_converter_close(&g_audio.music_playback_converter);
        xSemaphoreGive(g_audio.music_playback_mutex);
    }
    delete_sync_resources();
    free_audio_route_storage();
    g_audio.initialized = false;
    g_audio.deinitializing = false;
    g_audio.stopping = false;
    xSemaphoreGive(lifecycle_mutex);
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
    if (audio_rate_converter_reset(g_audio.capture_rate_converter) != 0) {
        return ESP_FAIL;
    }
    if (audio_rate_converter_reset(g_audio.voice_playback_converter) != 0 ||
        audio_rate_converter_reset(g_audio.far_reference_converter) != 0) return ESP_FAIL;
    audio_sample_fifo_reset(&g_audio.far_fifo);
    audio_sample_fifo_reset(&g_audio.voice_fifo);
    audio_sample_fifo_reset(&g_audio.music_fifo);
    audio_playout_reset_far_reference();
    if (g_audio.music_playback_converter != NULL &&
        audio_rate_converter_reset(g_audio.music_playback_converter) != 0) return ESP_FAIL;
    audio_capture_fifo_reset(&g_audio.capture_fifo);
    xQueueReset(g_audio.loopback_queue);
    AUDIO_STATS_LOCK();
    g_audio.stats.capture_peak_abs = 0u;
    AUDIO_STATS_UNLOCK();
    esp_err_t ret = audio_hw_codec_start(&g_audio.config);
    if (ret != ESP_OK) {
        return ret;
    }
    atomic_store_explicit(&g_audio.playout_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.capture_ready, false, memory_order_release);
    atomic_store_explicit(&g_audio.running, true, memory_order_release);
    if (audio_route_lock_acquire(portMAX_DELAY)) {
        atomic_store_explicit(&g_audio.call_priority_active,
                              g_audio.bluetooth_call.active && g_audio.bluetooth_call.configured,
                              memory_order_release);
        audio_route_lock_release();
    }

    log_audio_worker_memory("audio_playout task");
    BaseType_t created = xTaskCreatePinnedToCore(
        audio_playout_task, "audio_playout", AUDIO_PLAYOUT_TASK_STACK_SIZE, NULL,
        AUDIO_PLAYOUT_TASK_PRIORITY, &g_audio.playout_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        g_audio.playout_task = NULL;
        atomic_store_explicit(&g_audio.running, false, memory_order_release);
        audio_hw_codec_stop();
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_audio.playout_started, portMAX_DELAY);
    if (!atomic_load_explicit(&g_audio.playout_ready, memory_order_acquire)) {
        xSemaphoreTake(g_audio.playout_done, portMAX_DELAY);
        audio_hw_codec_stop();
        return ESP_FAIL;
    }

    log_audio_worker_memory("audio_capture task");
    created = xTaskCreatePinnedToCore(
        audio_capture_task, "audio_capture", AUDIO_CAPTURE_TASK_STACK_SIZE, NULL,
        AUDIO_CAPTURE_TASK_PRIORITY, &g_audio.capture_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        g_audio.capture_task = NULL;
        audio_join_workers_locked();
        audio_hw_codec_stop();
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_audio.capture_started, portMAX_DELAY);
    if (!atomic_load_explicit(&g_audio.capture_ready, memory_order_acquire)) {
        audio_join_workers_locked();
        audio_hw_codec_stop();
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret =
        (g_audio.stopping || g_audio.deinitializing) ? ESP_ERR_INVALID_STATE : audio_start_locked();
    xSemaphoreGive(lifecycle_mutex);
    return ret;
}

esp_err_t audio_stop(void)
{
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (g_audio.stopping) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_OK;
    }
    g_audio.stopping = true;
    atomic_store_explicit(&g_audio.running, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    g_audio.stopping = false;
    xSemaphoreGive(lifecycle_mutex);
    return ret;
}

bool audio_is_running(void)
{
    return atomic_load_explicit(&g_audio.running, memory_order_acquire);
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
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    if (!g_audio.initialized || g_audio.deinitializing) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&g_audio_task_lock);
    g_audio.activity_callback = callback;
    portEXIT_CRITICAL(&g_audio_task_lock);
    xSemaphoreGive(lifecycle_mutex);
    return ESP_OK;
}

esp_err_t audio_set_mode(audio_mode_t mode)
{
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    if (atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (mode != AUDIO_MODE_LOOPBACK && mode != AUDIO_MODE_MESH) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    g_audio.config.mode = mode;
    audio_rx_reset_source_metadata();
    xSemaphoreGive(lifecycle_mutex);
    return ESP_OK;
}

audio_mode_t audio_get_mode(void)
{
    SemaphoreHandle_t lifecycle_mutex = audio_lifecycle_mutex_get();
    if (lifecycle_mutex == NULL) {
        return AUDIO_MODE_LOOPBACK;
    }
    if (audio_called_from_worker()) {
        return g_audio.config.mode;
    }
    xSemaphoreTake(lifecycle_mutex, portMAX_DELAY);
    audio_mode_t mode = g_audio.config.mode;
    xSemaphoreGive(lifecycle_mutex);
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

static audio_route_stream_t *playback_route(audio_bluetooth_playback_t route)
{
    if (route == AUDIO_BLUETOOTH_MUSIC) {
        return &g_audio.bluetooth_music;
    }
    if (route == AUDIO_BLUETOOTH_CALL) {
        return &g_audio.bluetooth_call;
    }
    return NULL;
}

static bool playback_format_supported(audio_bluetooth_playback_t route, uint32_t sample_rate,
                                      uint8_t channels)
{
    if (route == AUDIO_BLUETOOTH_CALL) {
        return channels == 1u && (sample_rate == 8000u || sample_rate == 16000u);
    }
    return route == AUDIO_BLUETOOTH_MUSIC && (channels == 1u || channels == 2u) &&
           (sample_rate == 16000u || sample_rate == 32000u || sample_rate == 44100u ||
            sample_rate == 48000u);
}

static void add_route_drop(uint32_t *counter, size_t quantity)
{
    uint32_t add = quantity > UINT32_MAX ? UINT32_MAX : (uint32_t)quantity;
    *counter = UINT32_MAX - *counter < add ? UINT32_MAX : *counter + add;
}

esp_err_t audio_bluetooth_playback_configure(audio_bluetooth_playback_t route,
                                             uint32_t sample_rate, uint8_t channels)
{
    SemaphoreHandle_t mutex = audio_lifecycle_mutex_get();
    audio_route_stream_t *stream = playback_route(route);
    if (mutex == NULL || stream == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!playback_format_supported(route, sample_rate, channels)) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NOT_SUPPORTED;
    }
    audio_rate_converter_t *replacement = NULL;
    if (route == AUDIO_BLUETOOTH_MUSIC &&
        audio_rate_converter_create(&replacement, sample_rate, AUDIO_HW_SAMPLE_RATE) != 0) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }
    if (route == AUDIO_BLUETOOTH_MUSIC) {
        xSemaphoreTake(g_audio.music_playback_mutex, portMAX_DELAY);
    }
    if (!audio_route_lock_acquire(0)) {
        audio_rate_converter_close(&replacement);
        if (route == AUDIO_BLUETOOTH_MUSIC) {
            xSemaphoreGive(g_audio.music_playback_mutex);
        }
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    bool configured = audio_route_stream_configure(stream, sample_rate, channels);
    audio_rate_converter_t *old_converter = NULL;
    if (configured && route == AUDIO_BLUETOOTH_MUSIC) {
        old_converter = g_audio.music_playback_converter;
        g_audio.music_playback_converter = replacement;
        replacement = NULL;
        audio_sample_fifo_reset(&g_audio.music_fifo);
        atomic_fetch_add_explicit(&g_audio.music_playback_generation, 1u, memory_order_acq_rel);
    }
    if (configured && route == AUDIO_BLUETOOTH_CALL) {
        atomic_store_explicit(&g_audio.call_priority_active, false, memory_order_release);
        atomic_store_explicit(&g_audio.voice_playback_reset_requested, true,
                              memory_order_release);
    }
    audio_route_lock_release();
    if (route == AUDIO_BLUETOOTH_MUSIC) {
        xSemaphoreGive(g_audio.music_playback_mutex);
    }
    audio_rate_converter_close(&old_converter);
    audio_rate_converter_close(&replacement);
    xSemaphoreGive(mutex);
    return configured ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t audio_bluetooth_playback_set_active(audio_bluetooth_playback_t route, bool active)
{
    SemaphoreHandle_t mutex = audio_lifecycle_mutex_get();
    audio_route_stream_t *stream = playback_route(route);
    if (mutex == NULL || stream == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!audio_route_lock_acquire(0)) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (active && !stream->configured) {
        audio_route_lock_release();
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    audio_route_stream_set_active(stream, active);
    if (route == AUDIO_BLUETOOTH_CALL) {
        atomic_store_explicit(&g_audio.voice_playback_reset_requested, true, memory_order_release);
    }
    if (route == AUDIO_BLUETOOTH_MUSIC && !active) {
        atomic_fetch_add_explicit(&g_audio.music_playback_generation, 1u, memory_order_acq_rel);
    }
    if (route == AUDIO_BLUETOOTH_CALL) {
        atomic_store_explicit(&g_audio.call_priority_active, active, memory_order_release);
    }
    audio_route_lock_release();
    if (route == AUDIO_BLUETOOTH_MUSIC) {
        xSemaphoreTake(g_audio.music_playback_mutex, portMAX_DELAY);
        audio_sample_fifo_reset(&g_audio.music_fifo);
        if (!active) (void)audio_rate_converter_reset(g_audio.music_playback_converter);
        xSemaphoreGive(g_audio.music_playback_mutex);
    }
    xSemaphoreGive(mutex);
    return ESP_OK;
}

size_t audio_bluetooth_playback_enqueue(audio_bluetooth_playback_t route,
                                        const int16_t *interleaved, size_t frames)
{
    audio_route_stream_t *stream = playback_route(route);
    if (stream == NULL || interleaved == NULL || frames == 0u) {
        return 0u;
    }
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        return 0u;
    }
    TickType_t lock_wait_ticks = route == AUDIO_BLUETOOTH_MUSIC ? 1 : 0;
    if (!audio_route_lock_acquire(lock_wait_ticks)) {
        AUDIO_STATS_LOCK();
        add_route_drop(route == AUDIO_BLUETOOTH_MUSIC
                            ? &g_audio.stats.bluetooth_music_enqueue_route_lock_misses
                            : &g_audio.stats.bluetooth_call_enqueue_route_lock_misses,
                       frames);
        if (route == AUDIO_BLUETOOTH_MUSIC) {
            add_route_drop(&g_audio.stats.bluetooth_music_overflows, frames);
        } else {
            add_route_drop(&g_audio.stats.bluetooth_call_overflows, frames);
        }
        AUDIO_STATS_UNLOCK();
        return 0u;
    }
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire) || !stream->configured ||
        !stream->active) {
        audio_route_lock_release();
        return 0u;
    }
    size_t input_frames = frames > AUDIO_ROUTE_MAX_ENQUEUE_FRAMES
                              ? AUDIO_ROUTE_MAX_ENQUEUE_FRAMES
                              : frames;
    size_t accepted = audio_route_stream_enqueue(stream, interleaved, input_frames);
    audio_route_lock_release();
    if (accepted != frames) {
        AUDIO_STATS_LOCK();
        if (route == AUDIO_BLUETOOTH_MUSIC) {
            add_route_drop(&g_audio.stats.bluetooth_music_overflows, frames - accepted);
        } else {
            add_route_drop(&g_audio.stats.bluetooth_call_overflows, frames - accepted);
        }
        AUDIO_STATS_UNLOCK();
    }
    return accepted;
}

esp_err_t audio_bluetooth_mic_configure(uint32_t sample_rate)
{
    SemaphoreHandle_t mutex = audio_lifecycle_mutex_get();
    if (mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (sample_rate != 8000u && sample_rate != 16000u) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        xSemaphoreGive(mutex);
        return ESP_ERR_TIMEOUT;
    }
    audio_route_mic_reset(&g_audio.bluetooth_mic);
    g_audio.bluetooth_mic_rate = sample_rate;
    audio_route_lock_release();
    xSemaphoreGive(mutex);
    return ESP_OK;
}

esp_err_t audio_bluetooth_mic_set_active(bool active)
{
    SemaphoreHandle_t mutex = audio_lifecycle_mutex_get();
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_called_from_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!g_audio.initialized || g_audio.stopping || g_audio.deinitializing) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!audio_route_lock_acquire(pdMS_TO_TICKS(AUDIO_ROUTE_LOCK_WAIT_MS))) {
        xSemaphoreGive(mutex);
        return ESP_ERR_TIMEOUT;
    }
    audio_route_stream_set_active(&g_audio.bluetooth_mic, active);
    audio_route_lock_release();
    xSemaphoreGive(mutex);
    return ESP_OK;
}

size_t audio_bluetooth_mic_read(int16_t *samples, size_t requested_samples)
{
    if (samples == NULL || requested_samples == 0u) {
        return 0u;
    }
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        return 0u;
    }
    if (!audio_route_lock_acquire(0)) {
        AUDIO_STATS_LOCK();
        g_audio.stats.bluetooth_mic_read_route_lock_misses++;
        AUDIO_STATS_UNLOCK();
        return 0u;
    }
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire) ||
        !g_audio.bluetooth_mic.configured || !g_audio.bluetooth_mic.active) {
        audio_route_lock_release();
        return 0u;
    }
    size_t read_request = requested_samples > AUDIO_ROUTE_MAX_MIC_READ_SAMPLES
                              ? AUDIO_ROUTE_MAX_MIC_READ_SAMPLES
                              : requested_samples;
    size_t count = audio_route_mic_read(&g_audio.bluetooth_mic, samples, read_request,
                                        g_audio.bluetooth_mic_rate);
    audio_route_lock_release();
    if (count != read_request) {
        AUDIO_STATS_LOCK();
        g_audio.stats.bluetooth_mic_underruns++;
        AUDIO_STATS_UNLOCK();
    }
    return count;
}

size_t audio_bluetooth_mic_available_samples(void)
{
    if (!atomic_load_explicit(&g_audio.running, memory_order_acquire)) {
        return 0u;
    }
    if (!audio_route_lock_acquire(0)) {
        AUDIO_STATS_LOCK();
        g_audio.stats.bluetooth_mic_read_route_lock_misses++;
        AUDIO_STATS_UNLOCK();
        return 0u;
    }
    size_t available = 0u;
    if (atomic_load_explicit(&g_audio.running, memory_order_acquire) &&
        g_audio.bluetooth_mic.configured && g_audio.bluetooth_mic.active) {
        available = g_audio.bluetooth_mic.depth;
    }
    audio_route_lock_release();
    return available;
}
