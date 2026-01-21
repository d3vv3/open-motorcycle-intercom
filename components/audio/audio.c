/**
 * @file audio.c
 * @brief OMI Audio Subsystem Implementation
 *
 * Phase 0: I2S driver setup (disabled state), idle task, Opus stub
 * Phase 1: Full implementation with I2S streaming, Opus encode/decode, VOX
 */

#include "audio.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2s_std.h"

#include "opus.h"

static const char *TAG = "audio";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define AUDIO_TASK_STACK_SIZE 4096
#define AUDIO_TASK_PRIORITY   5
#define AUDIO_TASK_CORE       1

#define AUDIO_HEARTBEAT_INTERVAL_MS 10000 /* Log heartbeat every 10s */
#define AUDIO_TASK_LOOP_DELAY_MS    100   /* Idle loop delay */

/* I2S DMA buffer configuration */
#define I2S_DMA_BUFFER_COUNT 4
#define I2S_DMA_BUFFER_SIZE  256 /* Samples per buffer */

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static audio_config_t s_config = AUDIO_CONFIG_DEFAULT();
static audio_stats_t s_stats = {0};

/* I2S channel handles */
static i2s_chan_handle_t s_rx_chan = NULL; /* Microphone input */
static i2s_chan_handle_t s_tx_chan = NULL; /* Speaker output */

/* Audio task handle */
static TaskHandle_t s_audio_task = NULL;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static esp_err_t i2s_init_channels(const audio_config_t *config);
static void i2s_deinit_channels(void);
static void audio_task(void *arg);

/* ============================================================================
 * I2S Initialization (Disabled State)
 * ============================================================================ */

/**
 * @brief Initialize I2S channels in disabled state
 *
 * Sets up I2S RX (microphone) and TX (speaker) channels with the new
 * ESP-IDF 5.x I2S driver API. Channels are created but NOT enabled.
 *
 * @param config Audio configuration with I2S pin assignments
 * @return ESP_OK on success
 */
static esp_err_t i2s_init_channels(const audio_config_t *config)
{
    esp_err_t ret;

    /*
     * I2S Channel Configuration
     * Using I2S_NUM_0 for both RX and TX (full-duplex mode)
     */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUFFER_SIZE;

    /* Create I2S channel in full-duplex mode (both RX and TX) */
    ret = i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * I2S Standard Mode Configuration
     * - 16kHz sample rate (optimal for voice)
     * - 16-bit samples
     * - Mono (single channel)
     * - Philips format (standard I2S)
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)config->i2s_pins.bclk_gpio,
                .ws = (gpio_num_t)config->i2s_pins.ws_gpio,
                .dout = (gpio_num_t)config->i2s_pins.dout_gpio,
                .din = (gpio_num_t)config->i2s_pins.din_gpio,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };

    /* Initialize TX channel (speaker output) */
    ret = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S TX: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        i2s_del_channel(s_rx_chan);
        s_tx_chan = NULL;
        s_rx_chan = NULL;
        return ret;
    }

    /* Initialize RX channel (microphone input) */
    ret = i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S RX: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        i2s_del_channel(s_rx_chan);
        s_tx_chan = NULL;
        s_rx_chan = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2S channels initialized (disabled state)");
    ESP_LOGI(TAG, "  BCLK: GPIO %d", config->i2s_pins.bclk_gpio);
    ESP_LOGI(TAG, "  WS:   GPIO %d", config->i2s_pins.ws_gpio);
    ESP_LOGI(TAG, "  DIN:  GPIO %d", config->i2s_pins.din_gpio);
    ESP_LOGI(TAG, "  DOUT: GPIO %d", config->i2s_pins.dout_gpio);

    /*
     * NOTE: Channels are NOT enabled here (disabled state)
     * Call audio_start() to enable I2S streaming
     * TODO: Phase 1 - Enable channels and start streaming
     */

    return ESP_OK;
}

/**
 * @brief Deinitialize I2S channels
 */
static void i2s_deinit_channels(void)
{
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    ESP_LOGI(TAG, "I2S channels deinitialized");
}

/* ============================================================================
 * Audio Task
 * ============================================================================ */

/**
 * @brief Audio processing task (idle loop for Phase 0)
 *
 * In Phase 0, this task simply idles and logs periodic heartbeats.
 * In Phase 1, this task will:
 * - Read audio from I2S RX (microphone)
 * - Apply HPF and VOX detection
 * - Encode with Opus
 * - Queue frames for mesh transmission
 * - Decode received frames from mesh
 * - Write audio to I2S TX (speaker)
 *
 * @param arg Unused
 */
static void audio_task(void *arg)
{
    (void)arg;

    int64_t last_heartbeat = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Audio task started (idle mode)");

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        s_stats.task_loops++;

        /* Periodic heartbeat log */
        if ((now_ms - last_heartbeat) >= AUDIO_HEARTBEAT_INTERVAL_MS) {
            ESP_LOGI(TAG,
                     "Audio task heartbeat: loops=%" PRIu32 ", encoded=%" PRIu32
                     ", decoded=%" PRIu32,
                     s_stats.task_loops, s_stats.frames_encoded, s_stats.frames_decoded);
            last_heartbeat = now_ms;
        }

        /*
         * TODO: Phase 1 - Audio processing pipeline
         *
         * 1. Read samples from I2S RX (microphone)
         *    size_t bytes_read;
         *    i2s_channel_read(s_rx_chan, buffer, size, &bytes_read, timeout);
         *
         * 2. Apply high-pass filter (HPF) to remove low-frequency noise
         *
         * 3. VOX detection - check if speech is present
         *    if (detect_speech(buffer, bytes_read)) {
         *        s_stats.vox_activations++;
         *    }
         *
         * 4. Opus encode (20ms frames)
         *    opus_encode(encoder, pcm_buffer, frame_size, opus_buffer, max_size);
         *    s_stats.frames_encoded++;
         *
         * 5. Queue encoded frame for mesh transmission
         *    xQueueSend(tx_queue, &frame, 0);
         *
         * 6. Check for received frames from mesh
         *    if (xQueueReceive(rx_queue, &frame, 0)) {
         *        opus_decode(decoder, frame.data, frame.len, pcm_buffer, frame_size, 0);
         *        s_stats.frames_decoded++;
         *    }
         *
         * 7. Manage jitter buffer for smooth playback
         *
         * 8. Write samples to I2S TX (speaker)
         *    size_t bytes_written;
         *    i2s_channel_write(s_tx_chan, buffer, size, &bytes_written, timeout);
         */

        /* Idle delay - will be replaced with actual audio timing in Phase 1 */
        vTaskDelay(pdMS_TO_TICKS(AUDIO_TASK_LOOP_DELAY_MS));
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t audio_init(void)
{
    return audio_init_with_config(NULL);
}

esp_err_t audio_init_with_config(const audio_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Apply configuration */
    if (config != NULL) {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing audio subsystem");
    ESP_LOGI(TAG, "  Sample rate:  %" PRIu32 " Hz", s_config.sample_rate);
    ESP_LOGI(TAG, "  Channels:     %u", s_config.channels);
    ESP_LOGI(TAG, "  Bits/sample:  %u", s_config.bits_per_sample);
    ESP_LOGI(TAG, "  Frame size:   %u ms", s_config.frame_size_ms);
    ESP_LOGI(TAG, "  Opus bitrate: %" PRIu32 " bps", s_config.opus_bitrate);

    /* Initialize I2S in disabled state */
    esp_err_t ret = i2s_init_channels(&s_config);
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     * TODO: Phase 1 - Initialize Opus encoder/decoder
     *
     * int error;
     * OpusEncoder *encoder = opus_encoder_create(
     *     s_config.sample_rate,
     *     s_config.channels,
     *     OPUS_APPLICATION_VOIP,
     *     &error
     * );
     * opus_encoder_ctl(encoder, OPUS_SET_BITRATE(s_config.opus_bitrate));
     * opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(5));  // Balance CPU/quality
     * opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(1));  // Enable FEC
     *
     * OpusDecoder *decoder = opus_decoder_create(
     *     s_config.sample_rate,
     *     s_config.channels,
     *     &error
     * );
     */

    /* Log Opus library info (proves it's linked correctly) */
    ESP_LOGI(TAG, "Opus version: %s", opus_get_version_string());

    /* Reset statistics */
    memset(&s_stats, 0, sizeof(s_stats));

    /* Create audio task */
    BaseType_t task_ret =
        xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK_SIZE, NULL,
                                AUDIO_TASK_PRIORITY, &s_audio_task, AUDIO_TASK_CORE);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
        i2s_deinit_channels();
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Audio subsystem initialized successfully");

    return ESP_OK;
}

esp_err_t audio_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing audio subsystem");

    /* Delete audio task */
    if (s_audio_task) {
        vTaskDelete(s_audio_task);
        s_audio_task = NULL;
    }

    /*
     * TODO: Phase 1 - Destroy Opus encoder/decoder
     * opus_encoder_destroy(encoder);
     * opus_decoder_destroy(decoder);
     */

    /* Deinitialize I2S */
    i2s_deinit_channels();

    s_initialized = false;
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting audio capture/playback");

    /*
     * TODO: Phase 1 - Enable I2S channels
     *
     * esp_err_t ret;
     * ret = i2s_channel_enable(s_rx_chan);
     * if (ret != ESP_OK) {
     *     ESP_LOGE(TAG, "Failed to enable I2S RX: %s", esp_err_to_name(ret));
     *     return ret;
     * }
     * ret = i2s_channel_enable(s_tx_chan);
     * if (ret != ESP_OK) {
     *     ESP_LOGE(TAG, "Failed to enable I2S TX: %s", esp_err_to_name(ret));
     *     i2s_channel_disable(s_rx_chan);
     *     return ret;
     * }
     */

    return ESP_OK;
}

esp_err_t audio_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping audio capture/playback");

    /*
     * TODO: Phase 1 - Disable I2S channels
     *
     * i2s_channel_disable(s_rx_chan);
     * i2s_channel_disable(s_tx_chan);
     */

    return ESP_OK;
}

bool audio_vox_active(void)
{
    /*
     * TODO: Phase 1 - Implement VOX detection
     *
     * VOX (Voice Operated Exchange) detects speech presence based on:
     * - Signal energy above threshold
     * - Sustained energy for minimum duration
     * - Hysteresis to avoid rapid on/off switching
     */
    return false;
}

esp_err_t audio_get_tx_frame(audio_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * TODO: Phase 1 - Get encoded frame from TX queue
     *
     * if (xQueueReceive(s_tx_queue, frame, pdMS_TO_TICKS(timeout_ms))) {
     *     return ESP_OK;
     * }
     */

    (void)timeout_ms;
    return ESP_ERR_TIMEOUT;
}

esp_err_t audio_put_rx_frame(const audio_frame_t *frame, uint8_t source_id)
{
    if (!s_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * TODO: Phase 1 - Put frame in jitter buffer for playback
     *
     * jitter_buffer_put(s_jitter_buf, frame, source_id);
     */

    (void)source_id;
    s_stats.frames_decoded++;

    return ESP_OK;
}

esp_err_t audio_get_stats(audio_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *stats = s_stats;
    return ESP_OK;
}
