/**
 * @file audio.h
 * @brief OMI Audio Subsystem Interface
 *
 * This component handles:
 * - Microphone capture (I2S)
 * - Speaker output (I2S)
 * - Opus encode/decode
 * - VOX detection
 * - Jitter buffer
 */

#ifndef OMI_AUDIO_H
#define OMI_AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I2S GPIO Pin Definitions
 *
 * Default pin assignments for ESP32-S3-DevKitC-1:
 *   - BCLK (bit clock):  GPIO 4 - shared between mic and speaker
 *   - WS (word select):  GPIO 5 - shared between mic and speaker
 *   - DIN (data in):     GPIO 6 - from INMP441 microphone
 *   - DOUT (data out):   GPIO 7 - to MAX98357A amplifier
 */
#define AUDIO_I2S_BCLK_GPIO 4 /**< I2S bit clock GPIO */
#define AUDIO_I2S_WS_GPIO   5 /**< I2S word select (LRCLK) GPIO */
#define AUDIO_I2S_DIN_GPIO  6 /**< I2S data in (from mic) GPIO */
#define AUDIO_I2S_DOUT_GPIO 7 /**< I2S data out (to speaker) GPIO */

/**
 * @brief I2S GPIO pin configuration
 */
typedef struct {
    int bclk_gpio; /**< Bit clock GPIO */
    int ws_gpio;   /**< Word select (LRCLK) GPIO */
    int din_gpio;  /**< Data in GPIO (microphone) */
    int dout_gpio; /**< Data out GPIO (speaker) */
} audio_i2s_pins_t;

/**
 * @brief Default I2S pin configuration
 */
#define AUDIO_I2S_PINS_DEFAULT()                                                                   \
    {                                                                                              \
        .bclk_gpio = AUDIO_I2S_BCLK_GPIO,                                                          \
        .ws_gpio = AUDIO_I2S_WS_GPIO,                                                              \
        .din_gpio = AUDIO_I2S_DIN_GPIO,                                                            \
        .dout_gpio = AUDIO_I2S_DOUT_GPIO,                                                          \
    }

/**
 * @brief ADC configuration for analog microphone
 */
typedef struct {
    int adc_channel; /**< ADC channel (default: ADC_CHANNEL_0 = GPIO1) */
    int adc_unit;    /**< ADC unit (default: ADC_UNIT_1) */
    int adc_atten;   /**< Attenuation (default: ADC_ATTEN_DB_12) */
} audio_adc_config_t;

/**
 * @brief Default ADC configuration
 */
#define AUDIO_ADC_CONFIG_DEFAULT()                                                                 \
    {                                                                                              \
        .adc_channel = 0, /* ADC1_CHANNEL_0 = GPIO1 */                                             \
        .adc_unit = 1,    /* ADC_UNIT_1 */                                                         \
        .adc_atten = 1,   /* ADC_ATTEN_DB_2_5 - better SNR with 0.6V bias divider */               \
    }

/**
 * @brief VOX configuration parameters
 */
typedef struct {
    float activation_threshold;   /**< RMS threshold for speech detection (0.0-1.0) */
    float deactivation_threshold; /**< RMS threshold for speech end (0.0-1.0) */
    uint16_t hangover_ms;         /**< Time to keep VOX active after speech ends */
} audio_vox_config_t;

/**
 * @brief Default VOX configuration (medium sensitivity)
 */
#define AUDIO_VOX_CONFIG_DEFAULT()                                                                 \
    {                                                                                              \
        .activation_threshold = 0.03f,    /* Normal sensitive threshold */                         \
        .deactivation_threshold = 0.010f, /* 67% of activation (hysteresis) */                     \
        .hangover_ms = 300,               /* 300ms hangover */                                     \
    }

/**
 * @brief Audio operating mode
 */
typedef enum {
    AUDIO_MODE_LOOPBACK = 0, /**< Phase 1: Local loopback for testing */
    AUDIO_MODE_MESH,         /**< Phase 2+: Send/receive via mesh */
} audio_mode_t;

/**
 * @brief Callback for encoded audio frames (TX)
 *
 * Called when an Opus-encoded frame is ready to be transmitted.
 * The callback should queue the frame for mesh transmission.
 *
 * @param data Opus-encoded audio data
 * @param len Length of encoded data (typically 20-40 bytes)
 * @param timestamp_us Capture timestamp in microseconds
 */
typedef void (*audio_tx_cb_t)(const uint8_t *data, uint16_t len, int64_t timestamp_us);

/**
 * @brief Audio configuration parameters
 */
typedef struct {
    uint32_t sample_rate;          /**< Sample rate in Hz (default: 16000) */
    uint8_t channels;              /**< Number of channels (default: 1) */
    uint8_t bits_per_sample;       /**< Bits per sample (default: 16) */
    uint16_t frame_size_ms;        /**< Frame size in ms (default: 20) */
    uint32_t opus_bitrate;         /**< Opus bitrate in bps (default: 16000) */
    audio_i2s_pins_t i2s_pins;     /**< I2S GPIO pin configuration */
    audio_adc_config_t adc_config; /**< ADC configuration for mic input */
    audio_vox_config_t vox_config; /**< VOX detection configuration */
    bool enable_hpf;               /**< Enable high-pass filter */
    float hpf_cutoff_hz;           /**< HPF cutoff frequency (default: 120 Hz) */
    audio_mode_t mode;             /**< Operating mode (default: LOOPBACK) */
} audio_config_t;

/**
 * @brief Default audio configuration
 */
#define AUDIO_CONFIG_DEFAULT()                                                                     \
    {                                                                                              \
        .sample_rate = 16000,                                                                      \
        .channels = 1,                                                                             \
        .bits_per_sample = 16,                                                                     \
        .frame_size_ms = 20,                                                                       \
        .opus_bitrate = 16000,                                                                     \
        .i2s_pins = AUDIO_I2S_PINS_DEFAULT(),                                                      \
        .adc_config = AUDIO_ADC_CONFIG_DEFAULT(),                                                  \
        .vox_config = AUDIO_VOX_CONFIG_DEFAULT(),                                                  \
        .enable_hpf = true,                                                                        \
        .hpf_cutoff_hz = 80.0f,                                                                    \
        .mode = AUDIO_MODE_LOOPBACK,                                                               \
    }

/**
 * @brief Audio frame (one Opus frame)
 */
typedef struct {
    uint8_t data[64];     /**< Encoded Opus data (max ~40 bytes typical) */
    uint16_t len;         /**< Actual length of encoded data */
    int64_t timestamp_ms; /**< Capture timestamp */
} audio_frame_t;

/**
 * @brief Audio statistics
 */
typedef struct {
    uint32_t frames_encoded;     /**< Total frames encoded */
    uint32_t frames_decoded;     /**< Total frames decoded */
    uint32_t frames_dropped;     /**< Frames dropped due to overflow */
    uint32_t vox_activations;    /**< Number of VOX activations */
    uint32_t encode_time_us_avg; /**< Average encode time in microseconds */
    uint32_t encode_time_us_max; /**< Maximum encode time in microseconds */
    uint32_t decode_time_us_avg; /**< Average decode time in microseconds */
    uint32_t decode_time_us_max; /**< Maximum decode time in microseconds */
    uint32_t latency_ms_avg;     /**< Average end-to-end latency in milliseconds */
    uint32_t latency_ms_max;     /**< Maximum end-to-end latency in milliseconds */
    uint32_t glitches_detected;  /**< Number of audio glitches detected */
    uint8_t jitter_buffer_depth; /**< Current jitter buffer depth */
    uint32_t task_loops;         /**< Audio task loop count (health indicator) */
    uint32_t adc_overruns;       /**< ADC buffer overrun count */
    bool vox_active;             /**< Current VOX state */
} audio_stats_t;

/**
 * @brief Initialize the audio subsystem with default configuration
 * @return ESP_OK on success
 */
esp_err_t audio_init(void);

/**
 * @brief Initialize with custom configuration
 * @param config Audio configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t audio_init_with_config(const audio_config_t *config);

/**
 * @brief Deinitialize the audio subsystem
 * @return ESP_OK on success
 */
esp_err_t audio_deinit(void);

/**
 * @brief Start audio capture and playback
 * @return ESP_OK on success
 * @note TODO: Phase 1 - Enable I2S streams
 */
esp_err_t audio_start(void);

/**
 * @brief Stop audio capture and playback
 * @return ESP_OK on success
 * @note TODO: Phase 1 - Disable I2S streams
 */
esp_err_t audio_stop(void);

/**
 * @brief Check if VOX is currently active (speech detected)
 * @return true if speech detected
 * @note TODO: Phase 1 - Implement VOX detection
 */
bool audio_vox_active(void);

/**
 * @brief Get the next encoded audio frame for transmission
 * @param frame Output frame buffer
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK if frame available, ESP_ERR_TIMEOUT otherwise
 * @note TODO: Phase 1 - Implement Opus encoding
 */
esp_err_t audio_get_tx_frame(audio_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Submit a received audio frame for playback
 * @param frame Received frame
 * @param source_id Source node ID
 * @return ESP_OK on success
 * @note TODO: Phase 1 - Implement jitter buffer and Opus decoding
 */
esp_err_t audio_put_rx_frame(const audio_frame_t *frame, uint8_t source_id);

/**
 * @brief Register callback for encoded TX frames (mesh mode)
 *
 * In mesh mode, this callback is called for each encoded audio frame.
 * The callback should send the frame to the mesh for transmission.
 *
 * @param cb Callback function (NULL to disable)
 * @return ESP_OK on success
 */
esp_err_t audio_register_tx_callback(audio_tx_cb_t cb);

/**
 * @brief Set audio operating mode
 * @param mode AUDIO_MODE_LOOPBACK or AUDIO_MODE_MESH
 * @return ESP_OK on success
 */
esp_err_t audio_set_mode(audio_mode_t mode);

/**
 * @brief Get current audio operating mode
 * @return Current mode
 */
audio_mode_t audio_get_mode(void);

/**
 * @brief Get audio statistics
 * @param stats Output statistics structure
 * @return ESP_OK on success
 */
esp_err_t audio_get_stats(audio_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* OMI_AUDIO_H */
