/**
 * @file audio.c
 * @brief OMI Audio Subsystem Implementation - Phase 1
 *
 * Phase 1: Full audio pipeline with ADC input, HPF, Opus encode/decode, VOX, loopback
 */

#include "audio.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2s_std.h"

#include "hal/adc_types.h"
#include "opus.h"
#include "power.h"
#include "soc/soc.h"

static const char *TAG = "audio";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define AUDIO_TASK_STACK_SIZE 32768 /* 32KB - Opus encoder needs significant stack */
#define AUDIO_TASK_PRIORITY   5
#define AUDIO_TASK_CORE       1

#define AUDIO_HEARTBEAT_INTERVAL_MS 10000 /* Log heartbeat every 10s */

/* I2S DMA buffer configuration - reduced for lower latency */
#define I2S_DMA_BUFFER_COUNT 2   /* Minimum for double-buffering, saves ~20ms latency */
#define I2S_DMA_BUFFER_SIZE  320 /* Samples per buffer (20ms @ 16kHz) */

/* ADC configuration */
/* Audio frame size (20ms @ 16kHz) */
#define AUDIO_FRAME_SAMPLES 320

/* ADC configuration */
#define ADC_OVERSAMPLE_FACTOR 4
#define ADC_CONV_FRAME_SIZE                                                                        \
    (AUDIO_FRAME_SAMPLES * ADC_OVERSAMPLE_FACTOR * 4) /* 4 bytes/sample (S3) */
#define ADC_READ_TIMEOUT_MS 100

/* Maximum Opus packet size */
#define MAX_OPUS_PACKET_SIZE 64

/* VOX state machine */
#define VOX_HANGOVER_FRAMES 15 /* 15 frames @ 20ms = 300ms */

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief High-pass filter state (2nd order IIR Butterworth)
 */
typedef struct {
    float x1, x2;     /* Input history */
    float y1, y2;     /* Output history */
    float b0, b1, b2; /* Numerator coefficients */
    float a1, a2;     /* Denominator coefficients (a0 = 1) */
} hpf_state_t;

/**
 * @brief VOX detector state
 */
typedef struct {
    bool active;                  /* Current VOX state */
    uint16_t hangover_counter;    /* Frames remaining in hangover */
    float activation_threshold;   /* RMS threshold for activation */
    float deactivation_threshold; /* RMS threshold for deactivation */
    uint32_t activation_count;    /* Total activations */
} vox_state_t;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static bool s_running = false;
static audio_config_t s_config = AUDIO_CONFIG_DEFAULT();
static audio_stats_t s_stats = {0};

/* I2S channel handles */
static i2s_chan_handle_t s_rx_chan = NULL;
static i2s_chan_handle_t s_tx_chan = NULL;

/* ADC handle */
static adc_continuous_handle_t s_adc_handle = NULL;

/* Audio task handle */
static TaskHandle_t s_audio_task = NULL;

/* ADC notification handle */
static TaskHandle_t s_adc_notify_task = NULL;

/* Opus encoder/decoder */
static OpusEncoder *s_opus_encoder = NULL;
static OpusDecoder *s_opus_decoder = NULL;

/* Audio processing state */
static hpf_state_t s_hpf_state = {0};
static vox_state_t s_vox_state = {0};

/* DC blocker and low-pass filter state */
static float s_dc_estimate = 0.0f; /* Running DC offset estimate */
static int16_t s_lpf_prev = 0;     /* Previous sample for LPF */

/* Audio buffers */
static int16_t s_pcm_input[AUDIO_FRAME_SAMPLES];
static int16_t s_pcm_output[AUDIO_FRAME_SAMPLES];
static uint8_t s_opus_buffer[MAX_OPUS_PACKET_SIZE];

/* Phase 2: Mesh mode support */
static audio_tx_cb_t s_tx_callback = NULL;
static QueueHandle_t s_rx_queue = NULL;

#define AUDIO_RX_QUEUE_SIZE 8

/**
 * @brief RX frame queue item
 */
typedef struct {
    uint8_t data[MAX_OPUS_PACKET_SIZE];
    uint16_t len;
    uint8_t source_id;
    int64_t timestamp_us;
} audio_rx_item_t;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static esp_err_t adc_init(const audio_config_t *config);
static void adc_deinit(void);
static esp_err_t i2s_init_channels(const audio_config_t *config);
static void i2s_deinit_channels(void);
static esp_err_t opus_init(const audio_config_t *config);
static void opus_deinit(void);
static void hpf_init(hpf_state_t *state, float cutoff_hz, float sample_rate);
static void hpf_process(hpf_state_t *state, int16_t *samples, size_t count);
static void vox_init(vox_state_t *state, const audio_vox_config_t *config);
static bool vox_process(vox_state_t *state, const int16_t *samples, size_t count);
static float calculate_rms(const int16_t *samples, size_t count);
static void audio_task(void *arg);

/* ============================================================================
 * ADC Callbacks
 * ============================================================================ */

/**
 * @brief ADC conversion complete callback (called from ISR context)
 */
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    /* Notify audio task that ADC data is ready */
    /* Guard against callback firing before audio task sets its handle */
    if (s_adc_notify_task != NULL) {
        vTaskNotifyGiveFromISR(s_adc_notify_task, &mustYield);
    }
    return (mustYield == pdTRUE);
}

/* ============================================================================
 * ADC Initialization (Continuous Mode)
 * ============================================================================ */

static esp_err_t adc_init(const audio_config_t *config)
{
    esp_err_t ret;

    /* ADC continuous driver configuration */
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_CONV_FRAME_SIZE * 2,
        .conv_frame_size = ADC_CONV_FRAME_SIZE,
    };

    ret = adc_continuous_new_handle(&adc_config, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC handle: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure ADC for continuous conversion */
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = config->sample_rate * ADC_OVERSAMPLE_FACTOR,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern = {
        .atten = config->adc_config.adc_atten,
        .channel = config->adc_config.adc_channel & 0x7,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &adc_pattern;

    ret = adc_continuous_config(s_adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC: %s", esp_err_to_name(ret));
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    /* Register ADC event callbacks */
    adc_continuous_evt_cbs_t cbs = {0}; /* Zero-initialize entire struct first */
    cbs.on_conv_done = adc_conv_done_cb;
    ret = adc_continuous_register_event_callbacks(s_adc_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ADC callbacks: %s", esp_err_to_name(ret));
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "ADC continuous mode initialized");
    ESP_LOGI(TAG, "  Channel: %d (GPIO1)", config->adc_config.adc_channel);
    ESP_LOGI(TAG, "  Sample rate: %lu Hz", config->sample_rate);
    ESP_LOGI(TAG, "  Bit width: 12-bit");

    return ESP_OK;
}

static void adc_deinit(void)
{
    if (s_adc_handle) {
        adc_continuous_stop(s_adc_handle);
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
        ESP_LOGI(TAG, "ADC deinitialized");
    }
}

/* ============================================================================
 * I2S Initialization
 * ============================================================================ */

static esp_err_t i2s_init_channels(const audio_config_t *config)
{
    esp_err_t ret;

    /* I2S Channel Configuration */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUFFER_SIZE;

    /* Create I2S TX channel (speaker only for Phase 1) */
    ret = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    /* I2S Standard Mode Configuration */
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
                .din = I2S_GPIO_UNUSED,
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
        s_tx_chan = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2S TX channel initialized");
    ESP_LOGI(TAG, "  BCLK: GPIO %d", config->i2s_pins.bclk_gpio);
    ESP_LOGI(TAG, "  WS:   GPIO %d", config->i2s_pins.ws_gpio);
    ESP_LOGI(TAG, "  DOUT: GPIO %d", config->i2s_pins.dout_gpio);

    return ESP_OK;
}

static void i2s_deinit_channels(void)
{
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
    ESP_LOGI(TAG, "I2S channels deinitialized");
}

/* ============================================================================
 * Opus Encoder/Decoder Initialization
 * ============================================================================ */

static esp_err_t opus_init(const audio_config_t *config)
{
    int error;

    /* Create Opus encoder */
    s_opus_encoder =
        opus_encoder_create(config->sample_rate, config->channels, OPUS_APPLICATION_VOIP, &error);

    if (error != OPUS_OK || s_opus_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(error));
        return ESP_FAIL;
    }

    /* Configure encoder */
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_BITRATE(config->opus_bitrate));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_COMPLEXITY(5));

    /* Create Opus decoder */
    s_opus_decoder = opus_decoder_create(config->sample_rate, config->channels, &error);

    if (error != OPUS_OK || s_opus_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to create Opus decoder: %s", opus_strerror(error));
        opus_encoder_destroy(s_opus_encoder);
        s_opus_encoder = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Opus encoder/decoder initialized");
    ESP_LOGI(TAG, "  Version: %s", opus_get_version_string());
    ESP_LOGI(TAG, "  Mode: VoIP");
    ESP_LOGI(TAG, "  Bitrate: %lu bps", config->opus_bitrate);
    ESP_LOGI(TAG, "  Frame size: %u ms (%d samples)", config->frame_size_ms, AUDIO_FRAME_SAMPLES);

    return ESP_OK;
}

static void opus_deinit(void)
{
    if (s_opus_encoder) {
        opus_encoder_destroy(s_opus_encoder);
        s_opus_encoder = NULL;
    }
    if (s_opus_decoder) {
        opus_decoder_destroy(s_opus_decoder);
        s_opus_decoder = NULL;
    }
    ESP_LOGI(TAG, "Opus encoder/decoder destroyed");
}

/* ============================================================================
 * High-Pass Filter Implementation
 * ============================================================================ */

/**
 * @brief Initialize 2nd order Butterworth high-pass filter
 *
 * @param state Filter state structure
 * @param cutoff_hz Cutoff frequency in Hz
 * @param sample_rate Sample rate in Hz
 */
static void hpf_init(hpf_state_t *state, float cutoff_hz, float sample_rate)
{
    memset(state, 0, sizeof(hpf_state_t));

    /* Calculate filter coefficients for 2nd order Butterworth HPF */
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float alpha = sin_omega / (2.0f * 0.7071f); /* Q = 0.7071 for Butterworth */

    float a0 = 1.0f + alpha;
    state->b0 = (1.0f + cos_omega) / (2.0f * a0);
    state->b1 = -(1.0f + cos_omega) / a0;
    state->b2 = (1.0f + cos_omega) / (2.0f * a0);
    state->a1 = (-2.0f * cos_omega) / a0;
    state->a2 = (1.0f - alpha) / a0;

    ESP_LOGI(TAG, "HPF initialized: cutoff=%.1f Hz", cutoff_hz);
}

/**
 * @brief Process samples through high-pass filter
 *
 * @param state Filter state
 * @param samples Input/output sample buffer (in-place processing)
 * @param count Number of samples
 */
static void hpf_process(hpf_state_t *state, int16_t *samples, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        /* Normalize to [-1.0, 1.0] */
        float x = (float)samples[i] / 32768.0f;

        /* Direct Form II Transposed */
        float y = state->b0 * x + state->b1 * state->x1 + state->b2 * state->x2 -
                  state->a1 * state->y1 - state->a2 * state->y2;

        /* Update state */
        state->x2 = state->x1;
        state->x1 = x;
        state->y2 = state->y1;
        state->y1 = y;

        /* Convert back to int16_t with clamping */
        float scaled = y * 32768.0f;
        if (scaled > 32767.0f)
            scaled = 32767.0f;
        if (scaled < -32768.0f)
            scaled = -32768.0f;
        samples[i] = (int16_t)scaled;
    }
}

/* ============================================================================
 * VOX Detection Implementation
 * ============================================================================ */

/**
 * @brief Initialize VOX detector
 */
static void vox_init(vox_state_t *state, const audio_vox_config_t *config)
{
    memset(state, 0, sizeof(vox_state_t));
    state->activation_threshold = config->activation_threshold;
    state->deactivation_threshold = config->deactivation_threshold;

    ESP_LOGI(TAG, "VOX initialized");
    ESP_LOGI(TAG, "  Activation threshold: %.3f", config->activation_threshold);
    ESP_LOGI(TAG, "  Deactivation threshold: %.3f", config->deactivation_threshold);
    ESP_LOGI(TAG, "  Hangover: %u ms", config->hangover_ms);
}

/**
 * @brief Calculate RMS (Root Mean Square) of audio samples
 */
static float calculate_rms(const int16_t *samples, size_t count)
{
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float normalized = (float)samples[i] / 32768.0f;
        sum += normalized * normalized;
    }
    return sqrtf(sum / (float)count);
}

/**
 * @brief Process VOX detection
 *
 * @param state VOX state
 * @param samples Input samples
 * @param count Number of samples
 * @return true if speech is detected (VOX active)
 */
static bool vox_process(vox_state_t *state, const int16_t *samples, size_t count)
{
    float rms = calculate_rms(samples, count);

    if (rms > state->activation_threshold) {
        /* Speech detected - activate VOX */
        if (!state->active) {
            state->active = true;
            state->activation_count++;
            power_notify_voice_start(); /* Phase 3: Notify power manager */
            ESP_LOGD(TAG, "VOX activated (RMS: %.4f)", rms);
        }
        state->hangover_counter = VOX_HANGOVER_FRAMES;
    } else if (rms < state->deactivation_threshold) {
        /* No speech - decrement hangover counter */
        if (state->hangover_counter > 0) {
            state->hangover_counter--;
        } else if (state->active) {
            state->active = false;
            power_notify_voice_end(); /* Phase 3: Notify power manager */
            ESP_LOGD(TAG, "VOX deactivated");
        }
    }

    return state->active;
}

/* ============================================================================
 * Audio Task - Main Processing Loop
 * ============================================================================ */

static void audio_task(void *arg)
{
    (void)arg;

    int64_t last_heartbeat = esp_timer_get_time() / 1000;
    int64_t encode_time_sum = 0;
    int64_t decode_time_sum = 0;
    int64_t latency_sum = 0;

    /* Store task handle for ADC callback notification */
    s_adc_notify_task = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Audio task started - Phase 1 loopback mode");

    /* Start ADC */
    esp_err_t ret = adc_continuous_start(s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* Start I2S TX */
    ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX: %s", esp_err_to_name(ret));
        adc_continuous_stop(s_adc_handle);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Audio pipeline active");

    while (s_running) {
        int64_t frame_start_us = esp_timer_get_time();
        s_stats.task_loops++;

        /* ====================================================================
         * STEP 1: Read audio from ADC (microphone)
         * ==================================================================== */

        /* Wait for ADC data ready notification (with timeout) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADC_READ_TIMEOUT_MS));

        static uint8_t adc_buffer[ADC_CONV_FRAME_SIZE]; /* Static to save stack */
        uint32_t bytes_read = 0;

        /* Read available ADC data (non-blocking after notification) */
        ret = adc_continuous_read(s_adc_handle, adc_buffer, ADC_CONV_FRAME_SIZE, &bytes_read, 0);

        if (ret == ESP_OK && bytes_read > 0) {
            /* Calculate number of raw ADC samples available */
            size_t num_adc_samples = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;

            /* Convert ADC data to PCM samples with proper oversampling/averaging */
            /* Average every ADC_OVERSAMPLE_FACTOR (4) samples into one PCM sample */
            /* This reduces noise floor by ~6dB (sqrt(4) = 2x noise reduction) */
            for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
                int32_t sum = 0;
                size_t valid_samples = 0;

                for (size_t j = 0; j < ADC_OVERSAMPLE_FACTOR; j++) {
                    size_t idx = i * ADC_OVERSAMPLE_FACTOR + j;
                    if (idx < num_adc_samples) {
                        /* ESP32-S3 uses type2 format */
                        adc_digi_output_data_t *p =
                            (adc_digi_output_data_t *)&adc_buffer[idx * SOC_ADC_DIGI_RESULT_BYTES];
                        sum += p->type2.data;
                        valid_samples++;
                    }
                }

                if (valid_samples > 0) {
                    /* Average the oversampled values */
                    int32_t avg = sum / (int32_t)valid_samples;

                    /* Convert 12-bit unsigned (0-4095) to 16-bit signed */
                    /* Center around 2048, moderate gain (12x) with 2.5dB attenuation */
                    int16_t sample = (int16_t)((avg - 2048) * 12);

                    /* DC blocker - removes DC offset drift (critical for ADC) */
                    s_dc_estimate = s_dc_estimate * 0.999f + (float)sample * 0.001f;
                    sample = sample - (int16_t)s_dc_estimate;

                    /* Simple low-pass filter - reduces high-frequency noise */
                    sample = (int16_t)((s_lpf_prev * 3 + sample) / 4);
                    s_lpf_prev = sample;

                    s_pcm_input[i] = sample;
                } else {
                    s_pcm_input[i] = 0;
                }
            }

            /* ================================================================
             * STEP 2: Apply High-Pass Filter
             * ================================================================ */
            if (s_config.enable_hpf) {
                hpf_process(&s_hpf_state, s_pcm_input, AUDIO_FRAME_SAMPLES);
            }

            /* ================================================================
             * STEP 3: VOX Detection
             * ================================================================ */
            bool vox_active = vox_process(&s_vox_state, s_pcm_input, AUDIO_FRAME_SAMPLES);
            s_stats.vox_active = vox_active;
            s_stats.vox_activations = s_vox_state.activation_count;

            /* ================================================================
             * STEP 4: Opus Encode (only if VOX active)
             * ================================================================ */
            int opus_bytes = 0;
            if (vox_active) {
                int64_t encode_start = esp_timer_get_time();

                opus_bytes = opus_encode(s_opus_encoder, s_pcm_input, AUDIO_FRAME_SAMPLES,
                                         s_opus_buffer, MAX_OPUS_PACKET_SIZE);

                int64_t encode_time = esp_timer_get_time() - encode_start;

                if (opus_bytes > 0) {
                    s_stats.frames_encoded++;
                    encode_time_sum += encode_time;
                    s_stats.encode_time_us_avg = encode_time_sum / s_stats.frames_encoded;
                    if (encode_time > s_stats.encode_time_us_max) {
                        s_stats.encode_time_us_max = encode_time;
                    }

                    /* Mesh mode: Send to TX callback instead of loopback */
                    if (s_config.mode == AUDIO_MODE_MESH && s_tx_callback != NULL) {
                        s_tx_callback(s_opus_buffer, (uint16_t)opus_bytes, frame_start_us);
                    }
                } else {
                    ESP_LOGW(TAG, "Opus encode failed: %s", opus_strerror(opus_bytes));
                }
            } else {
                /* VOX inactive - no transmission */
                opus_bytes = 0;
            }

            /* ================================================================
             * STEP 5: Get audio for playback
             * In loopback mode: decode our own encoded audio
             * In mesh mode: decode from RX queue (received from other nodes)
             * ================================================================ */
            int samples_decoded = 0;
            bool have_audio_to_play = false;

            if (s_config.mode == AUDIO_MODE_LOOPBACK) {
                /* Loopback: decode our own audio */
                if (opus_bytes > 0) {
                    int64_t decode_start = esp_timer_get_time();

                    samples_decoded = opus_decode(s_opus_decoder, s_opus_buffer, opus_bytes,
                                                  s_pcm_output, AUDIO_FRAME_SAMPLES, 0);

                    int64_t decode_time = esp_timer_get_time() - decode_start;

                    if (samples_decoded == AUDIO_FRAME_SAMPLES) {
                        s_stats.frames_decoded++;
                        decode_time_sum += decode_time;
                        s_stats.decode_time_us_avg = decode_time_sum / s_stats.frames_decoded;
                        if (decode_time > s_stats.decode_time_us_max) {
                            s_stats.decode_time_us_max = decode_time;
                        }
                        have_audio_to_play = true;
                    } else {
                        ESP_LOGW(TAG, "Opus decode failed: %d", samples_decoded);
                    }
                }
            } else {
                /* Mesh mode: check RX queue for incoming audio */
                audio_rx_item_t rx_item;
                if (s_rx_queue != NULL && xQueueReceive(s_rx_queue, &rx_item, 0) == pdTRUE) {
                    int64_t decode_start = esp_timer_get_time();

                    samples_decoded = opus_decode(s_opus_decoder, rx_item.data, rx_item.len,
                                                  s_pcm_output, AUDIO_FRAME_SAMPLES, 0);

                    int64_t decode_time = esp_timer_get_time() - decode_start;

                    if (samples_decoded == AUDIO_FRAME_SAMPLES) {
                        s_stats.frames_decoded++;
                        decode_time_sum += decode_time;
                        s_stats.decode_time_us_avg = decode_time_sum / s_stats.frames_decoded;
                        if (decode_time > s_stats.decode_time_us_max) {
                            s_stats.decode_time_us_max = decode_time;
                        }
                        have_audio_to_play = true;
                    } else {
                        ESP_LOGW(TAG, "Opus decode failed: %d", samples_decoded);
                    }
                }
            }

            if (!have_audio_to_play) {
                /* No audio to play - output silence */
                memset(s_pcm_output, 0, AUDIO_FRAME_SAMPLES * sizeof(int16_t));
            }

            /* ================================================================
             * STEP 6: Write to I2S (speaker)
             * ================================================================ */
            size_t bytes_written = 0;
            ret = i2s_channel_write(s_tx_chan, s_pcm_output, AUDIO_FRAME_SAMPLES * sizeof(int16_t),
                                    &bytes_written, portMAX_DELAY);

            if (ret != ESP_OK || bytes_written != AUDIO_FRAME_SAMPLES * sizeof(int16_t)) {
                ESP_LOGW(TAG, "I2S write incomplete: %zu bytes", bytes_written);
                s_stats.glitches_detected++;
            }

            /* ================================================================
             * STEP 7: Calculate end-to-end latency (only for encoded frames)
             * ================================================================ */
            int64_t frame_end_us = esp_timer_get_time();
            int64_t processing_time_us = frame_end_us - frame_start_us;

            /* Only measure latency when we actually encoded a frame */
            if (opus_bytes > 0) {
                /* Total latency = processing time + I2S DMA buffer latency
                 * I2S has I2S_DMA_BUFFER_COUNT buffers of I2S_DMA_BUFFER_SIZE samples
                 * At 16kHz, each 320 sample buffer = 20ms
                 * With 4 buffers, add ~40ms (2 buffers worth) average DMA latency */
                int64_t dma_latency_us = ((int64_t)I2S_DMA_BUFFER_COUNT / 2) * 20 * 1000;
                int64_t total_latency_us = processing_time_us + dma_latency_us;

                latency_sum += total_latency_us;
                s_stats.latency_ms_avg = (latency_sum / s_stats.frames_encoded) / 1000;

                uint32_t latency_ms = total_latency_us / 1000;
                if (latency_ms > s_stats.latency_ms_max) {
                    s_stats.latency_ms_max = latency_ms;
                }
            }

        } else if (ret == ESP_ERR_TIMEOUT) {
            /* ADC timeout - continue loop */
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(ret));
            s_stats.adc_overruns++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* Periodic heartbeat log */
        int64_t now_ms = esp_timer_get_time() / 1000;
        if ((now_ms - last_heartbeat) >= AUDIO_HEARTBEAT_INTERVAL_MS) {
            ESP_LOGI(TAG, "=== Audio Stats ===");
            ESP_LOGI(TAG, "  Loops: %lu", s_stats.task_loops);
            ESP_LOGI(TAG, "  Encoded: %lu frames", s_stats.frames_encoded);
            ESP_LOGI(TAG, "  Decoded: %lu frames", s_stats.frames_decoded);
            ESP_LOGI(TAG, "  VOX activations: %lu (active: %s)", s_stats.vox_activations,
                     s_stats.vox_active ? "YES" : "no");
            ESP_LOGI(TAG, "  Encode time: avg=%lu us, max=%lu us", s_stats.encode_time_us_avg,
                     s_stats.encode_time_us_max);
            ESP_LOGI(TAG, "  Decode time: avg=%lu us, max=%lu us", s_stats.decode_time_us_avg,
                     s_stats.decode_time_us_max);
            ESP_LOGI(TAG, "  Latency: avg=%lu ms, max=%lu ms", s_stats.latency_ms_avg,
                     s_stats.latency_ms_max);
            ESP_LOGI(TAG, "  Glitches: %lu, ADC overruns: %lu", s_stats.glitches_detected,
                     s_stats.adc_overruns);
            last_heartbeat = now_ms;
        }
    }

    /* Stop audio */
    adc_continuous_stop(s_adc_handle);
    i2s_channel_disable(s_tx_chan);

    ESP_LOGI(TAG, "Audio task stopped");
    vTaskDelete(NULL);
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

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing Audio Subsystem - Phase 1");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Sample rate:  %lu Hz", s_config.sample_rate);
    ESP_LOGI(TAG, "  Channels:     %u", s_config.channels);
    ESP_LOGI(TAG, "  Bits/sample:  %u", s_config.bits_per_sample);
    ESP_LOGI(TAG, "  Frame size:   %u ms (%d samples)", s_config.frame_size_ms,
             AUDIO_FRAME_SAMPLES);
    ESP_LOGI(TAG, "  Opus bitrate: %lu bps", s_config.opus_bitrate);
    ESP_LOGI(TAG, "  HPF enabled:  %s (%.1f Hz)", s_config.enable_hpf ? "yes" : "no",
             s_config.hpf_cutoff_hz);

    /* Initialize ADC for microphone */
    esp_err_t ret = adc_init(&s_config);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Initialize I2S for speaker */
    ret = i2s_init_channels(&s_config);
    if (ret != ESP_OK) {
        adc_deinit();
        return ret;
    }

    /* Initialize Opus encoder/decoder */
    ret = opus_init(&s_config);
    if (ret != ESP_OK) {
        i2s_deinit_channels();
        adc_deinit();
        return ret;
    }

    /* Initialize HPF */
    if (s_config.enable_hpf) {
        hpf_init(&s_hpf_state, s_config.hpf_cutoff_hz, s_config.sample_rate);
    }

    /* Initialize VOX */
    vox_init(&s_vox_state, &s_config.vox_config);

    /* Reset statistics */
    memset(&s_stats, 0, sizeof(s_stats));

    s_initialized = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Audio subsystem initialized successfully");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

esp_err_t audio_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing audio subsystem");

    /* Stop if running */
    if (s_running) {
        audio_stop();
    }

    /* Destroy Opus encoder/decoder */
    opus_deinit();

    /* Deinitialize I2S */
    i2s_deinit_channels();

    /* Deinitialize ADC */
    adc_deinit();

    s_initialized = false;
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting audio pipeline");

    s_running = true;

    /* Create audio task */
    BaseType_t task_ret =
        xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK_SIZE, NULL,
                                AUDIO_TASK_PRIORITY, &s_audio_task, AUDIO_TASK_CORE);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
        s_running = false;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t audio_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping audio pipeline");

    s_running = false;

    /* Wait for task to finish */
    if (s_audio_task) {
        vTaskDelay(pdMS_TO_TICKS(100));
        s_audio_task = NULL;
    }

    return ESP_OK;
}

bool audio_vox_active(void)
{
    return s_stats.vox_active;
}

esp_err_t audio_get_tx_frame(audio_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* TODO: Phase 2 - Implement TX queue for mesh transmission */
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t audio_put_rx_frame(const audio_frame_t *frame, uint8_t source_id)
{
    if (!s_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_config.mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_rx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (frame->len > MAX_OPUS_PACKET_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    audio_rx_item_t rx_item;
    memcpy(rx_item.data, frame->data, frame->len);
    rx_item.len = frame->len;
    rx_item.source_id = source_id;
    rx_item.timestamp_us = frame->timestamp_ms * 1000;

    if (xQueueSend(s_rx_queue, &rx_item, 0) != pdTRUE) {
        s_stats.frames_dropped++;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t audio_register_tx_callback(audio_tx_cb_t cb)
{
    s_tx_callback = cb;
    return ESP_OK;
}

esp_err_t audio_set_mode(audio_mode_t mode)
{
    if (s_running) {
        ESP_LOGE(TAG, "Cannot change mode while running");
        return ESP_ERR_INVALID_STATE;
    }

    s_config.mode = mode;

    /* Create RX queue for mesh mode */
    if (mode == AUDIO_MODE_MESH && s_rx_queue == NULL) {
        s_rx_queue = xQueueCreate(AUDIO_RX_QUEUE_SIZE, sizeof(audio_rx_item_t));
        if (s_rx_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create RX queue");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Audio mode set to MESH");
    } else if (mode == AUDIO_MODE_LOOPBACK) {
        ESP_LOGI(TAG, "Audio mode set to LOOPBACK");
    }

    return ESP_OK;
}

audio_mode_t audio_get_mode(void)
{
    return s_config.mode;
}

esp_err_t audio_get_stats(audio_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *stats = s_stats;
    return ESP_OK;
}

/* ============================================================================
 * Notification Sounds
 * ============================================================================ */

/**
 * @brief Generate a sine wave tone into a buffer
 *
 * @param buffer Output buffer for PCM samples
 * @param samples Number of samples to generate
 * @param freq_hz Tone frequency in Hz
 * @param sample_rate Sample rate in Hz
 * @param amplitude Amplitude (0.0-1.0)
 */
static void generate_tone(int16_t *buffer, size_t samples, float freq_hz, uint32_t sample_rate,
                          float amplitude)
{
    for (size_t i = 0; i < samples; i++) {
        float t = (float)i / (float)sample_rate;
        float value = amplitude * sinf(2.0f * M_PI * freq_hz * t);
        buffer[i] = (int16_t)(value * 32767.0f);
    }
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    if (!s_initialized || s_tx_chan == NULL) {
        ESP_LOGW(TAG, "Audio not initialized, cannot play notification");
        return ESP_ERR_INVALID_STATE;
    }

    /* Tone configuration */
    const float FREQ_LOW = 440.0f;    /* A4 - lower beep */
    const float FREQ_HIGH = 880.0f;   /* A5 - higher beep */
    const size_t BEEP_SAMPLES = 1600; /* 100ms @ 16kHz */
    const size_t GAP_SAMPLES = 400;   /* 25ms gap between beeps */
    const float AMPLITUDE = 0.3f;     /* 30% volume to avoid clipping */

    /* Allocate buffer for both beeps + gap */
    size_t total_samples = BEEP_SAMPLES * 2 + GAP_SAMPLES;
    int16_t *buffer = malloc(total_samples * sizeof(int16_t));
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate notification buffer");
        return ESP_ERR_NO_MEM;
    }

    /* Generate the beeps based on notification type */
    float freq1, freq2;
    if (type == AUDIO_NOTIFY_PEER_JOIN) {
        /* Ascending: low then high */
        freq1 = FREQ_LOW;
        freq2 = FREQ_HIGH;
        ESP_LOGI(TAG, "Playing peer JOIN notification (low-high)");
    } else {
        /* Descending: high then low */
        freq1 = FREQ_HIGH;
        freq2 = FREQ_LOW;
        ESP_LOGI(TAG, "Playing peer LEAVE notification (high-low)");
    }

    /* First beep */
    generate_tone(buffer, BEEP_SAMPLES, freq1, s_config.sample_rate, AMPLITUDE);

    /* Gap (silence) */
    memset(buffer + BEEP_SAMPLES, 0, GAP_SAMPLES * sizeof(int16_t));

    /* Second beep */
    generate_tone(buffer + BEEP_SAMPLES + GAP_SAMPLES, BEEP_SAMPLES, freq2, s_config.sample_rate,
                  AMPLITUDE);

    /* Write to I2S (blocking) */
    size_t bytes_written = 0;
    esp_err_t ret =
        i2s_channel_write(s_tx_chan, buffer, total_samples * sizeof(int16_t), &bytes_written, 500);

    free(buffer);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to play notification: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
