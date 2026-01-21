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
 * @brief Audio configuration parameters
 */
typedef struct {
    uint32_t sample_rate;      /**< Sample rate in Hz (default: 16000) */
    uint8_t channels;          /**< Number of channels (default: 1) */
    uint8_t bits_per_sample;   /**< Bits per sample (default: 16) */
    uint16_t frame_size_ms;    /**< Frame size in ms (default: 20) */
    uint32_t opus_bitrate;     /**< Opus bitrate in bps (default: 16000) */
    audio_i2s_pins_t i2s_pins; /**< I2S GPIO pin configuration */
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
    uint32_t decode_time_us_avg; /**< Average decode time in microseconds */
    uint8_t jitter_buffer_depth; /**< Current jitter buffer depth */
    uint32_t task_loops;         /**< Audio task loop count (health indicator) */
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
 * @brief Get audio statistics
 * @param stats Output statistics structure
 * @return ESP_OK on success
 */
esp_err_t audio_get_stats(audio_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* OMI_AUDIO_H */
