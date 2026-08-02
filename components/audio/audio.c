/**
 * @file audio.c
 * @brief OMI Audio Subsystem Implementation - Phase 1
 *
 * Phase 1: Full audio pipeline with ADC input, HPF, Opus encode/decode, VOX, loopback
 */

#include "audio.h"
#include "audio_jitter_buffer.h"
#include "voice_cleanup.h"
#include "vox.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2s_std.h"

#include "hal/adc_types.h"
#include "opus.h"
#include "soc/soc.h"

static const char *TAG = "audio";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define AUDIO_TASK_STACK_SIZE 32768 /* 32KB - Opus encoder needs significant stack */
#define AUDIO_TASK_PRIORITY   7
#define AUDIO_TASK_CORE       1

#define AUDIO_HEARTBEAT_INTERVAL_MS 10000 /* Log heartbeat every 10s */

/* Stage-1 voice cleanup (lightweight AEC + NS) */
#define AUDIO_ENABLE_AEC_NS 1

/* I2S DMA buffer configuration - keep enough headroom without over-buffering output */
#define I2S_DMA_BUFFER_COUNT 4
#define I2S_DMA_BUFFER_SIZE  320 /* Samples per buffer (20ms @ 16kHz) */

/* Debug isolation: allow longer speaker write wait before counting glitch */
#define I2S_WRITE_TIMEOUT_MS 10

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

/* With DTX enabled, opus_encode() returns a 1-2 byte frame when it has nothing
 * to send (pure silence between comfort-noise updates).  Frames at or below
 * this size are dropped before transmit so the radio stays quiet during
 * silence; larger frames (speech or periodic SID/comfort-noise updates) are
 * sent so the receiver can sustain comfort noise. */
#define OPUS_DTX_FRAME_MAX_BYTES 2

#define RX_SOURCE_IDLE_TIMEOUT_US 1000000
#define HOLD_BUDGET_MAX           1
#define GRACE_EMPTY_MAX           5

/* Longest run of lost frames rebuilt one-by-one before the decoder is resynchronised.
 * Beyond this the dropout is long enough that concealment no longer sounds better
 * than restarting cleanly. */
#define MAX_CONCEAL_FRAMES 8

/* Expected network loss declared to the encoder. See opus_init() for what
 * this does and does not enable at the configured bitrate. */
#define OPUS_EXPECTED_LOSS_PERC 5

/* Bounded waits: the source lock is only ever held for short queue operations, so a
 * short wait removes contention drops without risking the caller's deadline. The
 * enqueue wait stays below the 2 ms nRF SPI poll period because the bridge receive
 * task must re-arm its next transaction. */
#define RX_ENQUEUE_LOCK_WAIT_MS 1
#define RX_DECODE_LOCK_WAIT_MS  2

#define NOTIFICATION_QUEUE_SIZE   4
#define NOTIFICATION_BEEP_SAMPLES 1600
#define NOTIFICATION_GAP_SAMPLES  400
#define NOTIFICATION_AMPLITUDE    0.3f

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

typedef enum {
    PLAYOUT_DECODE,  /* Decode this packet now */
    PLAYOUT_CONCEAL, /* Fill one missing frame; packet is held in pending */
    PLAYOUT_DROP,    /* Stale duplicate or late reorder; discard the packet */
} playout_action_t;

typedef struct {
    bool assigned;
    uint8_t source_id;
    int64_t last_enqueue_us;
    OpusDecoder *decoder;
    QueueHandle_t queue;
    audio_jitter_state_t jitter;
    /* Packet held back while the frames missing before it are concealed. */
    audio_rx_item_t pending;
    bool pending_valid;
} audio_rx_source_t;

typedef struct {
    uint8_t type;
} audio_notification_request_t;

typedef struct {
    bool active;
    audio_notify_t type;
    uint8_t tone_index;
    uint16_t segment_sample;
    bool in_gap;
} audio_notification_state_t;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static volatile bool s_running = false;
static audio_config_t s_config = AUDIO_CONFIG_DEFAULT();
static audio_stats_t s_stats = {0};

/* I2S channel handles */
static i2s_chan_handle_t s_tx_chan = NULL;

/* ADC handle */
static adc_continuous_handle_t s_adc_handle = NULL;

/* Audio task handle */
static TaskHandle_t s_audio_task = NULL;
static portMUX_TYPE s_audio_task_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_audio_task_done = NULL;
static SemaphoreHandle_t s_rx_reset_done = NULL;
static SemaphoreHandle_t s_rx_reset_mutex = NULL;

/* ADC notification handle */
static TaskHandle_t s_adc_notify_task = NULL;

/* Opus encoder and dedicated loopback decoder */
static OpusEncoder *s_opus_encoder = NULL;
static OpusDecoder *s_loopback_decoder = NULL;

/* Audio processing state */
static hpf_state_t s_hpf_state = {0};
static vox_state_t s_vox_state = {0};

/* DC blocker and low-pass filter state */
static float s_dc_estimate = 0.0f; /* Running DC offset estimate */
static int16_t s_lpf_prev = 0;     /* Previous sample for LPF */

/* Audio buffers */
static int16_t s_pcm_input[AUDIO_FRAME_SAMPLES];
static int16_t s_pcm_output[AUDIO_FRAME_SAMPLES];
static int16_t s_pcm_stereo[AUDIO_FRAME_SAMPLES * 2]; /* Stereo output for I2S */
static uint8_t s_opus_buffer[MAX_OPUS_PACKET_SIZE];
static int16_t s_far_ref_frame[AUDIO_FRAME_SAMPLES];
static int16_t s_decode_frame[AUDIO_FRAME_SAMPLES];
static int32_t s_mix_frame[AUDIO_FRAME_SAMPLES];
static voice_cleanup_state_t s_voice_cleanup = {0};

/* RX source slots and notification playback are owned by the audio task. */
static audio_rx_source_t s_rx_sources[AUDIO_MAX_RX_SOURCES] = {0};
static SemaphoreHandle_t s_rx_sources_mutex = NULL;
static atomic_bool s_rx_reset_requested = false;
static QueueHandle_t s_notification_queue = NULL;
static audio_notification_state_t s_notification = {0};

/* Pipeline latency accumulators (microseconds) */
static uint64_t s_tx_pipe_sum_us = 0;
static uint32_t s_tx_pipe_count = 0;
static uint64_t s_rx_pipe_sum_us = 0;
static uint32_t s_rx_pipe_count = 0;

/* Phase 2: Mesh mode support */
static audio_tx_cb_t s_tx_callback = NULL;
static audio_activity_cb_t s_activity_callback = NULL;

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
static void audio_task(void *arg);
static void audio_task_await_delete(void);
static void reset_rx_sources(void);
static bool decode_rx_source(audio_rx_source_t *source, int64_t now_us,
                             int64_t *decode_time_sum);
static void mix_notification_frame(void);

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
    /* Note: Use STEREO slot mode for PCM5102A DAC compatibility -
     * PCM5102A expects standard stereo I2S frames even for mono audio */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
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
    /* Declaring the expected loss rate weakens long-term prediction (silk LTP
     * scaling), so the decoder recovers faster after a concealed frame.
     * It does NOT buy in-band FEC here: decide_fec() needs an equivalent rate
     * above the 16 kbit/s wideband threshold scaled by (125 - loss)/100, which at
     * 12 kbit/s is unreachable at any loss percentage. Playout therefore conceals
     * with PLC, not LBRR. Raising the bitrate past ~25 kbit/s would change that. */
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_PACKET_LOSS_PERC(OPUS_EXPECTED_LOSS_PERC));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_COMPLEXITY(5));
    /* Discontinuous transmission: during silence the encoder emits a periodic
     * comfort-noise (SID) frame and otherwise returns a 1-2 byte DTX frame that
     * we drop at the transport layer (see audio_task TX gating).  The decoder
     * generates comfort noise from the last SID, so receivers hear a natural
     * quiet rather than a dead link. */
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_DTX(1));

    /* Loopback must not share decoder state with any remote source. */
    s_loopback_decoder = opus_decoder_create(config->sample_rate, config->channels, &error);

    if (error != OPUS_OK || s_loopback_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to create loopback Opus decoder: %s", opus_strerror(error));
        opus_encoder_destroy(s_opus_encoder);
        s_opus_encoder = NULL;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        s_rx_sources[i].decoder =
            opus_decoder_create(config->sample_rate, config->channels, &error);
        if (error != OPUS_OK || s_rx_sources[i].decoder == NULL) {
            ESP_LOGE(TAG, "Failed to create source Opus decoder %zu: %s", i,
                     opus_strerror(error));
            opus_deinit();
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Opus encoder and %u independent decoders initialized",
             AUDIO_MAX_RX_SOURCES + 1);
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
    if (s_loopback_decoder) {
        opus_decoder_destroy(s_loopback_decoder);
        s_loopback_decoder = NULL;
    }
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        if (s_rx_sources[i].decoder) {
            opus_decoder_destroy(s_rx_sources[i].decoder);
            s_rx_sources[i].decoder = NULL;
        }
    }
    ESP_LOGI(TAG, "Opus encoder/decoders destroyed");
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

static void reset_rx_sources(void)
{
    if (s_rx_sources_mutex) {
        xSemaphoreTake(s_rx_sources_mutex, portMAX_DELAY);
    }
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        audio_rx_source_t *source = &s_rx_sources[i];
        source->assigned = false;
        source->source_id = 0;
        source->last_enqueue_us = 0;
        source->pending_valid = false;
        audio_jitter_reset(&source->jitter);
        if (source->queue) {
            xQueueReset(source->queue);
        }
        if (source->decoder) {
            opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
        }
    }
    if (s_rx_sources_mutex) {
        xSemaphoreGive(s_rx_sources_mutex);
    }
}

static void record_decode_result(int samples, int64_t decode_time, int64_t *decode_time_sum)
{
    if (samples == AUDIO_FRAME_SAMPLES) {
        s_stats.frames_decoded++;
        *decode_time_sum += decode_time;
        s_stats.decode_time_us_avg = *decode_time_sum / s_stats.frames_decoded;
        if (decode_time > s_stats.decode_time_us_max) {
            s_stats.decode_time_us_max = decode_time;
        }
    } else {
        s_stats.decode_errors++;
        ESP_LOGW(TAG, "Opus decode failed: %d", samples);
    }
}

/* Pop the next packet for this source: a held-back packet first, then the queue.
 * Caller must hold s_rx_sources_mutex. */
static bool take_next_item(audio_rx_source_t *source, audio_rx_item_t *item)
{
    if (source->pending_valid) {
        *item = source->pending;
        source->pending_valid = false;
        return true;
    }
    return xQueueReceive(source->queue, item, 0) == pdTRUE;
}

/* Decide how to play a packet whose sequence may not follow the previous one.
 * Caller must hold s_rx_sources_mutex. */
static playout_action_t resolve_playout_sequence(audio_rx_source_t *source,
                                                 const audio_rx_item_t *item)
{
    audio_jitter_state_t *jitter = &source->jitter;

    if (!item->has_seq) {
        jitter->next_seq_valid = false;
        return PLAYOUT_DECODE;
    }

    if (!jitter->next_seq_valid) {
        jitter->next_seq = (uint16_t)(item->seq + 1);
        jitter->next_seq_valid = true;
        return PLAYOUT_DECODE;
    }

    int16_t delta = (int16_t)(item->seq - jitter->next_seq);

    if (delta == 0) {
        jitter->next_seq = (uint16_t)(item->seq + 1);
        return PLAYOUT_DECODE;
    }

    if (delta > 0 && delta <= MAX_CONCEAL_FRAMES) {
        /* Conceal exactly one missing frame now and replay this packet next tick,
         * so the playout timeline keeps one output frame per sender frame. */
        source->pending = *item;
        source->pending_valid = true;
        jitter->next_seq++;
        s_stats.seq_gap_frames++;
        s_stats.conceal_loss_frames++;
        return PLAYOUT_CONCEAL;
    }

    if (delta < 0 && delta >= -MAX_CONCEAL_FRAMES) {
        /* Duplicate or late reordered packet. Playout already moved past it, so
         * decoding it would corrupt predictor state. Discard it and keep going. */
        s_stats.seq_stale_drops++;
        return PLAYOUT_DROP;
    }

    /* Dropout too long to conceal, or the sender restarted: resynchronise cleanly. */
    if (delta > 0) {
        s_stats.seq_gap_frames += (uint32_t)delta;
        s_stats.glitches_detected++;
    }
    s_stats.seq_resets++;
    opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
    jitter->next_seq = (uint16_t)(item->seq + 1);
    return PLAYOUT_DECODE;
}

static bool decode_rx_source(audio_rx_source_t *source, int64_t now_us,
                             int64_t *decode_time_sum)
{
    audio_rx_item_t item;
    bool decode_plc = false;
    bool have_item = false;
    bool conceal_loss = false;

    if (xSemaphoreTake(s_rx_sources_mutex, pdMS_TO_TICKS(RX_DECODE_LOCK_WAIT_MS)) != pdTRUE) {
        return false;
    }
    if (!source->assigned) {
        xSemaphoreGive(s_rx_sources_mutex);
        return false;
    }

    UBaseType_t items = uxQueueMessagesWaiting(source->queue);
    audio_jitter_record_depth(&source->jitter, &s_stats, items);
    audio_jitter_update_playout_start(&source->jitter, items);
    uint32_t trim_count = s_stats.jitter_trim_frames;
    audio_jitter_trim_backlog(source->queue, &source->jitter, &s_stats, &item);
    if (s_stats.jitter_trim_frames != trim_count) {
        /* Trim is an explicit resynchronisation point. A held-back packet is older
         * than everything trim kept, so it would otherwise consume the re-baseline
         * and make the discarded frames reappear as a gap. */
        if (source->pending_valid) {
            source->pending_valid = false;
            s_stats.frames_dropped++;
        }
        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
    }
    items = uxQueueMessagesWaiting(source->queue);

    if (source->jitter.playout_started && source->jitter.hold_next) {
        source->jitter.hold_next = false;
        source->jitter.consecutive_empty = 0;
        s_stats.hold_frames++;
        s_stats.plc_frames++;
        decode_plc = true;
    } else if (source->jitter.playout_started) {
        playout_action_t action = PLAYOUT_DROP;
        unsigned stale_dropped = 0;
        /* Bounded by queue capacity plus the held packet. */
        for (unsigned attempt = 0; attempt <= AUDIO_RX_QUEUE_SIZE; attempt++) {
            if (!take_next_item(source, &item)) {
                action = PLAYOUT_DROP;
                break;
            }
            action = resolve_playout_sequence(source, &item);
            if (action != PLAYOUT_DROP) {
                break;
            }
            s_stats.frames_dropped++;
            stale_dropped++;
        }

        if (action == PLAYOUT_DECODE) {
            source->jitter.last_rx_packet_us = now_us;
            source->jitter.consecutive_empty = 0;
            source->jitter.stream_silent = !item.active;
            have_item = true;

            UBaseType_t remaining = uxQueueMessagesWaiting(source->queue);
            if (remaining == 0 && !source->pending_valid &&
                source->jitter.hold_budget < HOLD_BUDGET_MAX) {
                source->jitter.hold_next = true;
                source->jitter.hold_budget++;
            }
        } else if (action == PLAYOUT_CONCEAL) {
            source->jitter.last_rx_packet_us = now_us;
            source->jitter.consecutive_empty = 0;
            conceal_loss = true;
        } else {
            source->jitter.consecutive_empty++;
            s_stats.grace_empty_polls++;
            if (source->jitter.consecutive_empty <= GRACE_EMPTY_MAX) {
                decode_plc = true;
                s_stats.plc_frames++;
                if (stale_dropped > 0) {
                    /* Everything queued was already behind playout, so the frames
                     * assumed lost were only late. Stop guessing and let the next
                     * packet re-anchor the timeline. */
                    source->jitter.next_seq_valid = false;
                } else if (source->jitter.next_seq_valid && !source->jitter.stream_silent) {
                    /* The sender is transmitting, so this slot really was a lost
                     * packet: consume its sequence number, otherwise a later burst
                     * is concealed a second time. During intentional DTX silence
                     * the sender consumes no sequence number, so leave it alone or
                     * the first frame of the resumed utterance looks stale. */
                    source->jitter.next_seq++;
                }
            } else {
                if (!source->jitter.stream_silent &&
                    audio_jitter_should_count_underrun(&source->jitter, now_us)) {
                    s_stats.rx_queue_underruns++;
                    s_stats.glitches_detected++;
                }
                source->jitter.playout_started = false;
                /* Playout stopped: the break is already accounted for, so re-baseline
                 * instead of reporting the resulting sequence jump a second time. */
                source->jitter.next_seq_valid = false;
            }
        }
    }

    if (!source->jitter.playout_started && items == 0 &&
        now_us - source->last_enqueue_us >= RX_SOURCE_IDLE_TIMEOUT_US) {
        source->assigned = false;
        source->source_id = 0;
        source->pending_valid = false;
        audio_jitter_reset(&source->jitter);
        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
    }
    xSemaphoreGive(s_rx_sources_mutex);

    if (!have_item && !decode_plc && !conceal_loss) {
        return false;
    }

    int64_t decode_start = esp_timer_get_time();
    int samples = have_item ? opus_decode(source->decoder, item.data, item.len, s_decode_frame,
                                          AUDIO_FRAME_SAMPLES, 0)
                            : opus_decode(source->decoder, NULL, 0, s_decode_frame,
                                          AUDIO_FRAME_SAMPLES, 0);
    int64_t decode_time = esp_timer_get_time() - decode_start;
    if (have_item) {
        record_decode_result(samples, decode_time, decode_time_sum);
    } else if (samples != AUDIO_FRAME_SAMPLES) {
        /* Keep decode_ok comparable with the sender's encode_ok: only real packets
         * count as decodes, but concealment failures are still errors. */
        s_stats.decode_errors++;
        ESP_LOGW(TAG, "Opus concealment failed: %d", samples);
    }
    if (samples != AUDIO_FRAME_SAMPLES) {
        return false;
    }

    if (have_item) {
        int64_t rx_pipe_us = decode_start - item.timestamp_us;
        if (rx_pipe_us >= 0) {
            s_rx_pipe_count++;
            s_rx_pipe_sum_us += (uint64_t)rx_pipe_us;
            s_stats.rx_pipe_us_avg = (uint32_t)(s_rx_pipe_sum_us / s_rx_pipe_count);
            if ((uint32_t)rx_pipe_us > s_stats.rx_pipe_us_max) {
                s_stats.rx_pipe_us_max = (uint32_t)rx_pipe_us;
            }
        }
    }

    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
        s_mix_frame[i] += s_decode_frame[i];
    }

    if (have_item) {
        /* Repay hold debt by decoding in sequence so Opus predictor state stays continuous. */
        audio_rx_item_t catchup_item;
        bool have_catchup_item = false;

        if (xSemaphoreTake(s_rx_sources_mutex, 0) == pdTRUE) {
            if (source->assigned && source->jitter.playout_started &&
                source->jitter.hold_budget > 0 && uxQueueMessagesWaiting(source->queue) >= 3 &&
                xQueueReceive(source->queue, &catchup_item, 0) == pdTRUE) {
                source->jitter.hold_budget--;
                have_catchup_item = true;

                /* The catch-up frame is decoded here, so it consumes its sequence
                 * slot too. Resolve the delta first: a stale packet must not reach
                 * the decoder, and a real hole must stay visible in telemetry. */
                bool seq_known = catchup_item.has_seq && source->jitter.next_seq_valid;
                int16_t cdelta =
                    seq_known ? (int16_t)(catchup_item.seq - source->jitter.next_seq) : 0;

                if (seq_known && cdelta < 0) {
                    have_catchup_item = false;
                    s_stats.seq_stale_drops++;
                    s_stats.frames_dropped++;
                } else {
                    if (cdelta > MAX_CONCEAL_FRAMES) {
                        /* Same hard resynchronisation the main path performs. */
                        s_stats.seq_resets++;
                        s_stats.glitches_detected++;
                        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
                    }
                    if (cdelta > 0) {
                        s_stats.seq_gap_frames += (uint32_t)cdelta;
                    }
                    if (catchup_item.has_seq) {
                        source->jitter.next_seq = (uint16_t)(catchup_item.seq + 1);
                        source->jitter.next_seq_valid = true;
                    } else {
                        source->jitter.next_seq_valid = false;
                    }
                    /* Keep the DTX view local to the last frame actually consumed. */
                    source->jitter.stream_silent = !catchup_item.active;
                    s_stats.catchup_frames++;
                }
            }
            xSemaphoreGive(s_rx_sources_mutex);
        }

        if (have_catchup_item) {
            int catchup_samples =
                opus_decode(source->decoder, catchup_item.data, catchup_item.len, s_decode_frame,
                            AUDIO_FRAME_SAMPLES, 0);
            if (catchup_samples != AUDIO_FRAME_SAMPLES) {
                s_stats.decode_errors++;
                ESP_LOGW(TAG, "Catch-up Opus decode failed: %d", catchup_samples);

                if (catchup_samples < 0) {
                    int plc_samples = opus_decode(source->decoder, NULL, 0, s_decode_frame,
                                                  AUDIO_FRAME_SAMPLES, 0);
                    if (plc_samples == AUDIO_FRAME_SAMPLES) {
                        s_stats.plc_frames++;
                    } else {
                        s_stats.decode_errors++;
                        ESP_LOGW(TAG, "Catch-up PLC failed: %d", plc_samples);
                    }
                }
            }
        }
    }
    return true;
}

static void audio_task_await_delete(void)
{
    portENTER_CRITICAL(&s_audio_task_lock);
    bool reset_requested = atomic_exchange(&s_rx_reset_requested, false);
    s_audio_task = NULL;
    portEXIT_CRITICAL(&s_audio_task_lock);
    if (reset_requested) {
        reset_rx_sources();
        xSemaphoreGive(s_rx_reset_done);
    }
    s_adc_notify_task = NULL;
    s_running = false;
    xSemaphoreGive(s_audio_task_done);
    vTaskDelete(NULL);
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

    ESP_LOGI(TAG, "Audio task started on core %d", xPortGetCoreID());

    /* Start ADC */
    esp_err_t ret = adc_continuous_start(s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        audio_task_await_delete();
    }

    /* Start I2S TX */
    ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX: %s", esp_err_to_name(ret));
        adc_continuous_stop(s_adc_handle);
        audio_task_await_delete();
    }

    ESP_LOGI(TAG, "Audio pipeline active");

    while (s_running) {
        int64_t frame_start_us = esp_timer_get_time();
        s_stats.task_loops++;

        /* ====================================================================
         * STEP 1: Read audio from ADC (microphone)
         * ==================================================================== */

        /* Wait for ADC data ready notification (with timeout) */
        bool capture_notified =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADC_READ_TIMEOUT_MS)) != 0;

        if (atomic_exchange(&s_rx_reset_requested, false)) {
            reset_rx_sources();
            xSemaphoreGive(s_rx_reset_done);
        }
        if (!capture_notified) {
            s_stats.capture_timeouts++;
        }

        static uint8_t adc_buffer[ADC_CONV_FRAME_SIZE]; /* Static to save stack */
        uint32_t bytes_read = 0;

        /* Read available ADC data (non-blocking after notification) */
        ret = adc_continuous_read(s_adc_handle, adc_buffer, ADC_CONV_FRAME_SIZE, &bytes_read, 0);

        if (ret == ESP_OK && bytes_read > 0) {
            if (bytes_read == ADC_CONV_FRAME_SIZE) {
                s_stats.capture_frames_ok++;
            } else {
                s_stats.capture_short_reads++;
            }
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
                    /* Center around 2048, gain (8x) for MAX9814 with 12dB atten */
                    int16_t sample = (int16_t)((avg - 2048) * 8);

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

#if AUDIO_ENABLE_AEC_NS
            if (s_config.mode == AUDIO_MODE_MESH) {
                voice_cleanup_process(&s_voice_cleanup, s_pcm_input, s_far_ref_frame,
                                      AUDIO_FRAME_SAMPLES);
            }
#endif

            /* ================================================================
             * STEP 3: VOX Detection
             * ================================================================ */
            bool was_vox_active = s_vox_state.active;
            bool vox_active = vox_process(&s_vox_state, s_pcm_input, AUDIO_FRAME_SAMPLES);
            if (vox_active != was_vox_active && s_activity_callback != NULL) {
                s_activity_callback(vox_active);
            }
            bool tx_active = vox_active || s_config.force_tx_always;
            s_stats.vox_active = vox_active;
            s_stats.vox_activations = s_vox_state.activation_count;

            /* ================================================================
             * STEP 4: Opus Encode (only if VOX active)
             * ================================================================ */
            int opus_bytes = 0;
            if (tx_active) {
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
                        s_tx_callback(s_opus_buffer, (uint16_t)opus_bytes, true, frame_start_us);
                    }
                } else {
                    s_stats.encode_errors++;
                    ESP_LOGW(TAG, "Opus encode failed: %s", opus_strerror(opus_bytes));
                }
            } else {
                /* VOX inactive - feed a zeroed frame through the DTX-enabled
                 * encoder.  Opus emits a small comfort-noise (SID) update at the
                 * start of the silence period and roughly every 400 ms after,
                 * returning a 1-2 byte DTX frame in between.  We transmit only
                 * the comfort-noise updates and drop the DTX frames, so the
                 * radio goes silent during silence while the receiver still has
                 * enough to generate comfort noise.  The audio_flags inactive
                 * bit (active=false) tells the receiver this is intentional
                 * silence, not packet loss. */
                static int16_t s_silence_frame[AUDIO_FRAME_SAMPLES]; /* BSS-zeroed */
                int64_t encode_start = esp_timer_get_time();
                opus_bytes = opus_encode(s_opus_encoder, s_silence_frame, AUDIO_FRAME_SAMPLES,
                                         s_opus_buffer, MAX_OPUS_PACKET_SIZE);
                int64_t encode_time = esp_timer_get_time() - encode_start;

                if (opus_bytes > 0) {
                    s_stats.frames_encoded++;
                    encode_time_sum += encode_time;
                    s_stats.encode_time_us_avg = encode_time_sum / s_stats.frames_encoded;
                    if (encode_time > s_stats.encode_time_us_max) {
                        s_stats.encode_time_us_max = encode_time;
                    }

                    bool comfort_update = (opus_bytes > OPUS_DTX_FRAME_MAX_BYTES);
                    if (comfort_update) {
                        if (s_config.mode == AUDIO_MODE_MESH && s_tx_callback != NULL) {
                            s_tx_callback(s_opus_buffer, (uint16_t)opus_bytes, false,
                                          frame_start_us);
                        }
                    } else {
                        /* Pure DTX frame: keep the radio quiet. */
                        s_stats.tx_dtx_suppressed++;
                    }
                } else {
                    s_stats.encode_errors++;
                }
            }

            /* ================================================================
             * STEP 5: Get audio for playback
             * In loopback mode: decode our own encoded audio
             * In mesh mode: decode from RX queue (received from other nodes)
             * ================================================================ */
            bool have_audio_to_play = false;

            if (s_config.mode == AUDIO_MODE_LOOPBACK) {
                /* Loopback: decode our own audio */
                if (opus_bytes > 0) {
                    int64_t decode_start = esp_timer_get_time();

                    int samples_decoded = opus_decode(s_loopback_decoder, s_opus_buffer, opus_bytes,
                                                      s_pcm_output, AUDIO_FRAME_SAMPLES, 0);

                    int64_t decode_time = esp_timer_get_time() - decode_start;

                    if (samples_decoded == AUDIO_FRAME_SAMPLES) {
                        record_decode_result(samples_decoded, decode_time, &decode_time_sum);
                        have_audio_to_play = true;
                    } else {
                        record_decode_result(samples_decoded, decode_time, &decode_time_sum);
                    }
                }
            } else {
                memset(s_mix_frame, 0, sizeof(s_mix_frame));
                uint8_t active_sources = 0;
                uint8_t mixed_sources = 0;
                uint16_t total_depth = 0;
                int64_t now_us = esp_timer_get_time();
                for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
                    if (decode_rx_source(&s_rx_sources[i], now_us, &decode_time_sum)) {
                        have_audio_to_play = true;
                        mixed_sources++;
                    }
                    if (s_rx_sources[i].assigned) {
                        active_sources++;
                        total_depth += uxQueueMessagesWaiting(s_rx_sources[i].queue);
                    }
                }
                s_stats.active_rx_sources = active_sources;
                s_stats.jitter_buffer_depth = total_depth > UINT8_MAX ? UINT8_MAX : total_depth;

                for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
                    int32_t sample = s_mix_frame[i];
                    /* Preserve one-source level; average only sources that produced PCM. */
                    if (mixed_sources > 1) {
                        sample /= mixed_sources;
                    }
                    if (sample > INT16_MAX) {
                        sample = INT16_MAX;
                    } else if (sample < INT16_MIN) {
                        sample = INT16_MIN;
                    }
                    s_pcm_output[i] = (int16_t)sample;
                }
            }

            if (!have_audio_to_play) {
                /* No audio to play - output silence */
                memset(s_pcm_output, 0, AUDIO_FRAME_SAMPLES * sizeof(int16_t));
            }

            mix_notification_frame();

            /* Keep the final speaker mix as the next mic frame's AEC reference. */
            memcpy(s_far_ref_frame, s_pcm_output, sizeof(s_far_ref_frame));

            /* ================================================================
             * STEP 6: Write to I2S (speaker)
             * Convert mono to stereo for PCM5102A/MAX98357A compatibility
             * ================================================================ */
            for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
                s_pcm_stereo[i * 2] = s_pcm_output[i];     /* Left */
                s_pcm_stereo[i * 2 + 1] = s_pcm_output[i]; /* Right */
            }

            size_t bytes_written = 0;
            ret = i2s_channel_write(s_tx_chan, s_pcm_stereo,
                                    AUDIO_FRAME_SAMPLES * 2 * sizeof(int16_t), &bytes_written,
                                    pdMS_TO_TICKS(I2S_WRITE_TIMEOUT_MS));

            if (ret != ESP_OK || bytes_written != AUDIO_FRAME_SAMPLES * 2 * sizeof(int16_t)) {
                ESP_LOGW(TAG, "I2S write incomplete: %zu bytes", bytes_written);
                s_stats.i2s_write_incomplete++;
                s_stats.glitches_detected++;
            } else {
                s_stats.playback_frames++;
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
            if (capture_notified) {
                s_stats.capture_timeouts++;
            }
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
            ESP_LOGI(TAG, "  TX pipeline: avg=%lu us, max=%lu us", s_stats.tx_pipe_us_avg,
                     s_stats.tx_pipe_us_max);
            ESP_LOGI(TAG, "  RX pipeline: avg=%lu us, max=%lu us", s_stats.rx_pipe_us_avg,
                     s_stats.rx_pipe_us_max);
            ESP_LOGI(TAG, "  Latency: avg=%lu ms, max=%lu ms", s_stats.latency_ms_avg,
                     s_stats.latency_ms_max);
            ESP_LOGI(TAG, "  Glitches: %lu (rx_und=%lu i2s_inc=%lu), ADC overruns: %lu",
                     s_stats.glitches_detected, s_stats.rx_queue_underruns,
                     s_stats.i2s_write_incomplete, s_stats.adc_overruns);
            ESP_LOGI(TAG,
                     "  Concealment: plc=%lu grace_empty=%lu conceal=%lu seq_gap=%lu "
                     "seq_reset=%lu seq_stale=%lu",
                     s_stats.plc_frames, s_stats.grace_empty_polls, s_stats.conceal_loss_frames,
                     s_stats.seq_gap_frames, s_stats.seq_resets, s_stats.seq_stale_drops);
            ESP_LOGI(TAG, "  Adaptive playout: hold=%lu catchup=%lu sources=%u",
                     s_stats.hold_frames, s_stats.catchup_frames, s_stats.active_rx_sources);
            ESP_LOGI(TAG, "  RX queue depth/source: min=%u avg=%u max=%u (total now=%u)",
                     s_stats.rx_q_depth_min, s_stats.rx_q_depth_avg, s_stats.rx_q_depth_max,
                     s_stats.jitter_buffer_depth);
            ESP_LOGI(TAG,
                     "PIPE v=1 dev=esp stage=audio capture_ok=%lu capture_short=%lu capture_timeout=%lu capture_err=%lu encode_ok=%lu encode_err=%lu dtx_drop=%lu rx_q_drop=%lu rx_lock_drop=%lu rx_src_drop=%lu jitter_drop=%lu decode_ok=%lu decode_err=%lu plc=%lu hold=%lu catchup=%lu conceal=%lu seq_gap=%lu seq_reset=%lu seq_stale=%lu glitch=%lu play_ok=%lu i2s_err=%lu notify_drop=%lu rx_sources=%u",
                     s_stats.capture_frames_ok, s_stats.capture_short_reads,
                     s_stats.capture_timeouts, s_stats.adc_overruns, s_stats.frames_encoded,
                     s_stats.encode_errors, s_stats.tx_dtx_suppressed,
                     s_stats.rx_queue_overflows, s_stats.rx_lock_drops,
                     s_stats.rx_source_rejections,
                     s_stats.jitter_trim_frames, s_stats.frames_decoded, s_stats.decode_errors,
                     s_stats.plc_frames, s_stats.hold_frames, s_stats.catchup_frames,
                     s_stats.conceal_loss_frames, s_stats.seq_gap_frames, s_stats.seq_resets,
                     s_stats.seq_stale_drops, s_stats.glitches_detected,
                     s_stats.playback_frames, s_stats.i2s_write_incomplete,
                     s_stats.notification_queue_overflows, s_stats.active_rx_sources);
            last_heartbeat = now_ms;
        }

        /* Frame pacing:
         * We are paced by the ADC data availability (ulTaskNotifyTake at top of loop).
         * We do NOT want to sleep here, because that would double-pace the loop
         * and cause drift if the ADC clock != system clock.
         * Just loop back and wait for next ADC buffer.
         */
    }

    /* Stop audio before signalling completion so teardown cannot race drivers. */
    adc_continuous_stop(s_adc_handle);
    i2s_channel_disable(s_tx_chan);

    ESP_LOGI(TAG, "Audio task stopped");
    audio_task_await_delete();
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

    audio_config_t requested = AUDIO_CONFIG_DEFAULT();
    if (config != NULL) {
        requested = *config;
    }

    /* Buffers and hardware setup are fixed for 16 kHz mono, 16-bit, 20 ms frames. */
    if (requested.sample_rate != 16000 || requested.channels != 1 ||
        requested.bits_per_sample != 16 || requested.frame_size_ms != 20) {
        ESP_LOGE(TAG, "Unsupported audio format (requires 16kHz mono 16-bit 20ms)");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (requested.mode != AUDIO_MODE_LOOPBACK && requested.mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Apply configuration */
    s_config = requested;

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

    /* Initialize voice cleanup (AEC + noise suppression) */
    voice_cleanup_init(&s_voice_cleanup);

    /* Reset statistics */
    memset(&s_stats, 0, sizeof(s_stats));

    s_audio_task_done = xSemaphoreCreateBinary();
    s_rx_reset_done = xSemaphoreCreateBinary();
    s_rx_reset_mutex = xSemaphoreCreateMutex();
    s_rx_sources_mutex = xSemaphoreCreateMutex();
    s_notification_queue =
        xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(audio_notification_request_t));
    if (!s_audio_task_done || !s_rx_reset_done || !s_rx_reset_mutex || !s_rx_sources_mutex ||
        !s_notification_queue) {
        ESP_LOGE(TAG, "Failed to create audio synchronization resources");
        if (s_audio_task_done) {
            vSemaphoreDelete(s_audio_task_done);
            s_audio_task_done = NULL;
        }
        if (s_rx_sources_mutex) {
            vSemaphoreDelete(s_rx_sources_mutex);
            s_rx_sources_mutex = NULL;
        }
        if (s_rx_reset_done) {
            vSemaphoreDelete(s_rx_reset_done);
            s_rx_reset_done = NULL;
        }
        if (s_rx_reset_mutex) {
            vSemaphoreDelete(s_rx_reset_mutex);
            s_rx_reset_mutex = NULL;
        }
        if (s_notification_queue) {
            vQueueDelete(s_notification_queue);
            s_notification_queue = NULL;
        }
        opus_deinit();
        i2s_deinit_channels();
        adc_deinit();
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        s_rx_sources[i].queue = xQueueCreate(AUDIO_RX_QUEUE_SIZE, sizeof(audio_rx_item_t));
        if (s_rx_sources[i].queue == NULL) {
            ESP_LOGE(TAG, "Failed to create RX source queue %zu", i);
            for (size_t j = 0; j < i; j++) {
                vQueueDelete(s_rx_sources[j].queue);
                s_rx_sources[j].queue = NULL;
            }
            vQueueDelete(s_notification_queue);
            s_notification_queue = NULL;
            vSemaphoreDelete(s_rx_sources_mutex);
            s_rx_sources_mutex = NULL;
            vSemaphoreDelete(s_rx_reset_mutex);
            s_rx_reset_mutex = NULL;
            vSemaphoreDelete(s_rx_reset_done);
            s_rx_reset_done = NULL;
            vSemaphoreDelete(s_audio_task_done);
            s_audio_task_done = NULL;
            opus_deinit();
            i2s_deinit_channels();
            adc_deinit();
            return ESP_ERR_NO_MEM;
        }
    }
    memset(&s_notification, 0, sizeof(s_notification));
    reset_rx_sources();

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
    if (s_running || s_audio_task != NULL) {
        esp_err_t stop_ret = audio_stop();
        if (stop_ret != ESP_OK) {
            return stop_ret;
        }
    }

    reset_rx_sources();
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        if (s_rx_sources[i].queue) {
            vQueueDelete(s_rx_sources[i].queue);
            s_rx_sources[i].queue = NULL;
        }
    }
    if (s_notification_queue) {
        vQueueDelete(s_notification_queue);
        s_notification_queue = NULL;
    }

    /* Destroy Opus encoder/decoder */
    opus_deinit();

    /* Deinitialize I2S */
    i2s_deinit_channels();

    /* Deinitialize ADC */
    adc_deinit();

    if (s_rx_sources_mutex) {
        vSemaphoreDelete(s_rx_sources_mutex);
        s_rx_sources_mutex = NULL;
    }
    if (s_audio_task_done) {
        vSemaphoreDelete(s_audio_task_done);
        s_audio_task_done = NULL;
    }
    if (s_rx_reset_done) {
        vSemaphoreDelete(s_rx_reset_done);
        s_rx_reset_done = NULL;
    }
    if (s_rx_reset_mutex) {
        vSemaphoreDelete(s_rx_reset_mutex);
        s_rx_reset_mutex = NULL;
    }

    s_initialized = false;
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_running || s_audio_task != NULL) {
        ESP_LOGW(TAG, "Already running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting audio pipeline");

    xSemaphoreTake(s_audio_task_done, 0);
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

    if (!s_running && s_audio_task == NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping audio pipeline");

    s_running = false;

    /* Wake the ADC wait and wait for the task's explicit completion signal. */
    TaskHandle_t task = s_audio_task;
    if (task) {
        xTaskNotifyGive(task);
        xSemaphoreTake(s_audio_task_done, portMAX_DELAY);
    }

    reset_rx_sources();
    xQueueReset(s_notification_queue);
    memset(&s_notification, 0, sizeof(s_notification));
    opus_encoder_ctl(s_opus_encoder, OPUS_RESET_STATE);
    opus_decoder_ctl(s_loopback_decoder, OPUS_RESET_STATE);

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
    if (!s_running || source_id == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_config.mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_rx_sources_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (frame->len > MAX_OPUS_PACKET_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    audio_rx_item_t rx_item = {0};
    memcpy(rx_item.data, frame->data, frame->len);
    rx_item.len = frame->len;
    rx_item.source_id = source_id;
    rx_item.timestamp_us = frame->timestamp_ms * 1000;
    rx_item.active = frame->active;
    rx_item.seq = frame->seq;
    rx_item.has_seq = frame->has_seq;

    if (xSemaphoreTake(s_rx_sources_mutex, pdMS_TO_TICKS(RX_ENQUEUE_LOCK_WAIT_MS)) != pdTRUE) {
        s_stats.frames_dropped++;
        s_stats.rx_lock_drops++;
        return ESP_ERR_TIMEOUT;
    }

    audio_rx_source_t *source = NULL;
    audio_rx_source_t *free_source = NULL;
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; i++) {
        if (s_rx_sources[i].assigned && s_rx_sources[i].source_id == source_id) {
            source = &s_rx_sources[i];
            break;
        }
        if (!s_rx_sources[i].assigned && free_source == NULL) {
            free_source = &s_rx_sources[i];
        }
    }

    if (source == NULL) {
        source = free_source;
        if (source == NULL) {
            xSemaphoreGive(s_rx_sources_mutex);
            s_stats.frames_dropped++;
            s_stats.rx_source_rejections++;
            return ESP_ERR_NO_MEM;
        }
        source->assigned = true;
        source->source_id = source_id;
        source->pending_valid = false;
        audio_jitter_reset(&source->jitter);
        xQueueReset(source->queue);
        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
    }
    source->last_enqueue_us = esp_timer_get_time();

    if (xQueueSend(source->queue, &rx_item, 0) != pdTRUE) {
        xSemaphoreGive(s_rx_sources_mutex);
        s_stats.frames_dropped++;
        s_stats.rx_queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_rx_sources_mutex);

    return ESP_OK;
}

void audio_clear_rx_frames(void)
{
    if (!s_initialized || s_rx_reset_mutex == NULL || s_rx_reset_done == NULL) {
        return;
    }
    xSemaphoreTake(s_rx_reset_mutex, portMAX_DELAY);
    xSemaphoreTake(s_rx_reset_done, 0);
    portENTER_CRITICAL(&s_audio_task_lock);
    TaskHandle_t task = s_audio_task;
    if (task != NULL) {
        atomic_store(&s_rx_reset_requested, true);
    }
    portEXIT_CRITICAL(&s_audio_task_lock);
    if (task != NULL) {
        xSemaphoreTake(s_rx_reset_done, portMAX_DELAY);
    } else {
        reset_rx_sources();
    }
    xSemaphoreGive(s_rx_reset_mutex);
}

esp_err_t audio_register_tx_callback(audio_tx_cb_t cb)
{
    s_tx_callback = cb;
    return ESP_OK;
}

esp_err_t audio_register_activity_callback(audio_activity_cb_t cb)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    s_activity_callback = cb;
    return ESP_OK;
}

esp_err_t audio_set_mode(audio_mode_t mode)
{
    if (s_running) {
        ESP_LOGE(TAG, "Cannot change mode while running");
        return ESP_ERR_INVALID_STATE;
    }

    if (mode != AUDIO_MODE_LOOPBACK && mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config.mode = mode;

    /* Queues, decoder, and jitter state remain slot-local. */
    if (mode == AUDIO_MODE_MESH) {
        reset_rx_sources();
        ESP_LOGI(TAG, "Audio mode set to MESH");
    } else if (mode == AUDIO_MODE_LOOPBACK) {
        reset_rx_sources();
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

esp_err_t audio_record_tx_pipeline_latency_us(uint32_t latency_us)
{
    s_tx_pipe_count++;
    s_tx_pipe_sum_us += latency_us;
    s_stats.tx_pipe_us_avg = (uint32_t)(s_tx_pipe_sum_us / s_tx_pipe_count);
    if (latency_us > s_stats.tx_pipe_us_max) {
        s_stats.tx_pipe_us_max = latency_us;
    }
    return ESP_OK;
}

/* ============================================================================
 * Notification Sounds
 * ============================================================================ */

static uint8_t notification_tone_count(audio_notify_t type)
{
    if (type == AUDIO_NOTIFY_STARTUP) {
        return 3;
    }
    if (type == AUDIO_NOTIFY_PEER_JOIN || type == AUDIO_NOTIFY_PEER_LEAVE) {
        return 2;
    }
    return 1;
}

static float notification_frequency(audio_notify_t type, uint8_t tone_index)
{
    static const float startup[] = {261.63f, 329.63f, 392.00f};
    static const float join[] = {440.0f, 880.0f};
    static const float leave[] = {880.0f, 440.0f};

    switch (type) {
    case AUDIO_NOTIFY_STARTUP:
        return startup[tone_index];
    case AUDIO_NOTIFY_PEER_JOIN:
        return join[tone_index];
    case AUDIO_NOTIFY_PEER_LEAVE:
        return leave[tone_index];
    case AUDIO_NOTIFY_MESH_ENABLED:
        return 329.63f;
    case AUDIO_NOTIFY_MESH_DISABLED:
        return 261.63f;
    default:
        return 0.0f;
    }
}

static void mix_notification_frame(void)
{
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
        if (!s_notification.active) {
            audio_notification_request_t request;
            if (xQueueReceive(s_notification_queue, &request, 0) != pdTRUE) {
                return;
            }
            s_notification.active = true;
            s_notification.type = (audio_notify_t)request.type;
            s_notification.tone_index = 0;
            s_notification.segment_sample = 0;
            s_notification.in_gap = false;
            ESP_LOGI(TAG, "Playing notification type %u", request.type);
        }

        int32_t notification_sample = 0;
        if (!s_notification.in_gap) {
            float phase = 2.0f * M_PI * notification_frequency(s_notification.type,
                                                               s_notification.tone_index) *
                          s_notification.segment_sample / s_config.sample_rate;
            notification_sample =
                (int32_t)(NOTIFICATION_AMPLITUDE * 32767.0f * sinf(phase));
        }

        int32_t mixed = (int32_t)s_pcm_output[i] + notification_sample;
        if (mixed > INT16_MAX) {
            mixed = INT16_MAX;
        } else if (mixed < INT16_MIN) {
            mixed = INT16_MIN;
        }
        s_pcm_output[i] = (int16_t)mixed;

        s_notification.segment_sample++;
        uint16_t segment_length = s_notification.in_gap ? NOTIFICATION_GAP_SAMPLES
                                                        : NOTIFICATION_BEEP_SAMPLES;
        if (s_notification.segment_sample < segment_length) {
            continue;
        }

        s_notification.segment_sample = 0;
        if (s_notification.in_gap) {
            s_notification.in_gap = false;
            s_notification.tone_index++;
        } else if (s_notification.tone_index + 1 <
                   notification_tone_count(s_notification.type)) {
            s_notification.in_gap = true;
        } else {
            s_notification.active = false;
        }
    }
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    if (!s_initialized || s_notification_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (type < AUDIO_NOTIFY_STARTUP || type > AUDIO_NOTIFY_MESH_DISABLED) {
        return ESP_ERR_INVALID_ARG;
    }

    audio_notification_request_t request = {.type = (uint8_t)type};
    if (xQueueSend(s_notification_queue, &request, 0) != pdTRUE) {
        s_stats.notification_queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
