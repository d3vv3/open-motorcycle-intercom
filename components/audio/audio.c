/**
 * @file audio.c
 * @brief Audio capture, transport playout, and notification synthesis.
 */

#include "audio.h"
#include "audio_packet_store.h"
#include "audio_pcm_resampler.h"
#include "voice_cleanup.h"
#include "vox.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "opus.h"
#include "soc/soc.h"

static const char *TAG = "audio";

#define AUDIO_CAPTURE_TASK_STACK_SIZE 32768
#define AUDIO_PLAYOUT_TASK_STACK_SIZE 32768
#define AUDIO_CAPTURE_TASK_PRIORITY   7
#define AUDIO_PLAYOUT_TASK_PRIORITY   8
#define AUDIO_TASK_CORE               1

#define AUDIO_HEARTBEAT_INTERVAL_MS 10000
#define AUDIO_ENABLE_AEC_NS          1

/* Two 20 ms descriptors: preload both, then each steady write waits for availability. */
#define I2S_DMA_BUFFER_COUNT 2
#define I2S_DMA_BUFFER_SIZE  320
#define I2S_WRITE_TIMEOUT_MS 50

#define AUDIO_FRAME_SAMPLES 320
#define ADC_OVERSAMPLE_FACTOR 4
#define ADC_CONV_FRAME_SIZE                                                                        \
    (AUDIO_FRAME_SAMPLES * ADC_OVERSAMPLE_FACTOR * 4)
#define ADC_READ_TIMEOUT_MS 100

#define MAX_OPUS_PACKET_SIZE     64
#define OPUS_DTX_FRAME_MAX_BYTES 2
#define OPUS_EXPECTED_LOSS_PERC  5
#define RX_SOURCE_IDLE_TIMEOUT_MS 1000
/* A source must be VOX-silent this long before a new talker may displace it. */
#define RX_SOURCE_EVICT_SILENCE_MS 400
#define RX_ENQUEUE_LOCK_WAIT_MS   1

#define LOOPBACK_QUEUE_SIZE      8
#define NOTIFICATION_QUEUE_SIZE  4
#define NOTIFICATION_BEEP_SAMPLES 1600
#define NOTIFICATION_GAP_SAMPLES  400
#define NOTIFICATION_AMPLITUDE    0.3f

typedef struct {
    float x1, x2;
    float y1, y2;
    float b0, b1, b2;
    float a1, a2;
} hpf_state_t;

typedef struct {
    bool assigned;
    uint8_t source_id;
    uint64_t last_enqueue_ms;
    uint64_t last_active_ms;
    bool decoder_reset_pending;
    bool decoded_active;
    OpusDecoder *decoder;
    audio_packet_store_t packet_store;
    audio_pcm_resampler_t resampler;
} audio_rx_source_t;

typedef struct {
    audio_packet_store_pop_result_t result;
    audio_packet_t packet;
} audio_playout_event_t;

typedef struct {
    uint8_t data[MAX_OPUS_PACKET_SIZE];
    uint16_t length;
    int64_t timestamp_us;
} audio_loopback_item_t;

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

static bool s_initialized;
static bool s_stopping;
static bool s_deinitializing;
static atomic_bool s_running = ATOMIC_VAR_INIT(false);
static atomic_bool s_playout_ready = ATOMIC_VAR_INIT(false);
static atomic_bool s_capture_ready = ATOMIC_VAR_INIT(false);
static atomic_bool s_rx_reset_requested = ATOMIC_VAR_INIT(false);
static audio_config_t s_config = AUDIO_CONFIG_DEFAULT();
static audio_stats_t s_stats;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;

#define STATS_LOCK()   portENTER_CRITICAL(&s_stats_lock)
#define STATS_UNLOCK() portEXIT_CRITICAL(&s_stats_lock)

static i2s_chan_handle_t s_tx_chan;
static adc_continuous_handle_t s_adc_handle;

static TaskHandle_t s_capture_task;
static TaskHandle_t s_playout_task;
static _Atomic(TaskHandle_t) s_adc_notify_task = ATOMIC_VAR_INIT(NULL);
static portMUX_TYPE s_task_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_playout_started;
static SemaphoreHandle_t s_capture_started;
static SemaphoreHandle_t s_capture_done;
static SemaphoreHandle_t s_playout_done;
static SemaphoreHandle_t s_rx_reset_done;
static SemaphoreHandle_t s_rx_reset_mutex;
static StaticSemaphore_t s_lifecycle_mutex_storage;
static SemaphoreHandle_t s_lifecycle_mutex;

static OpusEncoder *s_opus_encoder;
static OpusDecoder *s_loopback_decoder;
static hpf_state_t s_hpf_state;
static vox_state_t s_vox_state;
static voice_cleanup_state_t s_voice_cleanup;
static float s_dc_estimate;
static int16_t s_lpf_prev;

static int16_t s_pcm_input[AUDIO_FRAME_SAMPLES];
static uint8_t s_opus_buffer[MAX_OPUS_PACKET_SIZE];
static int16_t s_pcm_output[AUDIO_FRAME_SAMPLES];
static int16_t s_pcm_stereo[AUDIO_FRAME_SAMPLES * 2];
static int16_t s_i2s_silence[AUDIO_FRAME_SAMPLES * 2];
static int16_t s_far_ref_frame[AUDIO_FRAME_SAMPLES];
static int16_t s_far_ref_shadows[I2S_DMA_BUFFER_COUNT][AUDIO_FRAME_SAMPLES];
static size_t s_far_ref_shadow_head;
static int32_t s_mix_frame[AUDIO_FRAME_SAMPLES];
static portMUX_TYPE s_far_ref_lock = portMUX_INITIALIZER_UNLOCKED;

static audio_rx_source_t s_rx_sources[AUDIO_MAX_RX_SOURCES];
static SemaphoreHandle_t s_rx_sources_mutex;
static QueueHandle_t s_loopback_queue;
static QueueHandle_t s_notification_queue;
static audio_notification_state_t s_notification;

static uint64_t s_tx_pipe_sum_us;
static uint32_t s_tx_pipe_count;
static uint64_t s_rx_pipe_sum_us;
static uint32_t s_rx_pipe_count;
static audio_tx_cb_t s_tx_callback;
static audio_activity_cb_t s_activity_callback;

static void capture_task(void *arg);
static void playout_task(void *arg);
static void mix_notification_frame(void);
static esp_err_t audio_stop_locked(void);

/*
 * API lock order is lifecycle -> RX reset -> RX sources -> short portMUX locks.
 * Task code never holds task/stats/far-reference portMUX locks while taking a mutex.
 */
static audio_stats_t audio_stats_snapshot(void)
{
    audio_stats_t snapshot;
    STATS_LOCK();
    snapshot = s_stats;
    STATS_UNLOCK();
    return snapshot;
}

static bool called_from_audio_worker(void)
{
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&s_task_lock);
    bool is_worker = current == s_capture_task || current == s_playout_task;
    portEXIT_CRITICAL(&s_task_lock);
    return is_worker;
}

static void reset_far_reference_shadows(void)
{
    portENTER_CRITICAL(&s_far_ref_lock);
    memset(s_far_ref_frame, 0, sizeof(s_far_ref_frame));
    memset(s_far_ref_shadows, 0, sizeof(s_far_ref_shadows));
    s_far_ref_shadow_head = 0u;
    portEXIT_CRITICAL(&s_far_ref_lock);
}

/*
 * Each shadow follows one preloaded/submitted DMA descriptor. A successful
 * blocking write means the oldest descriptor completed and can become the AEC
 * reference; the new final mix takes its place in that descriptor's shadow.
 */
static void advance_far_reference_after_write(void)
{
    portENTER_CRITICAL(&s_far_ref_lock);
    memcpy(s_far_ref_frame, s_far_ref_shadows[s_far_ref_shadow_head],
           sizeof(s_far_ref_frame));
    memcpy(s_far_ref_shadows[s_far_ref_shadow_head], s_pcm_output,
           sizeof(s_far_ref_shadows[s_far_ref_shadow_head]));
    s_far_ref_shadow_head = (s_far_ref_shadow_head + 1u) % I2S_DMA_BUFFER_COUNT;
    portEXIT_CRITICAL(&s_far_ref_lock);
}

static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata, void *user_data)
{
    (void)handle;
    (void)edata;
    (void)user_data;
    BaseType_t must_yield = pdFALSE;
    TaskHandle_t task = atomic_load_explicit(&s_adc_notify_task, memory_order_relaxed);
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &must_yield);
    }
    return must_yield == pdTRUE;
}

static esp_err_t adc_init(const audio_config_t *config)
{
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = ADC_CONV_FRAME_SIZE * 2,
        .conv_frame_size = ADC_CONV_FRAME_SIZE,
    };
    esp_err_t ret = adc_continuous_new_handle(&handle_config, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC handle: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_continuous_config_t config_data = {
        .sample_freq_hz = config->sample_rate * ADC_OVERSAMPLE_FACTOR,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    adc_digi_pattern_config_t pattern = {
        .atten = config->adc_config.adc_atten,
        .channel = config->adc_config.adc_channel & 0x7,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    config_data.pattern_num = 1;
    config_data.adc_pattern = &pattern;
    ret = adc_continuous_config(s_adc_handle, &config_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %s", esp_err_to_name(ret));
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    adc_continuous_evt_cbs_t callbacks = {.on_conv_done = adc_conv_done_cb};
    ret = adc_continuous_register_event_callbacks(s_adc_handle, &callbacks, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ADC callbacks: %s", esp_err_to_name(ret));
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "ADC continuous mode initialized at %lu Hz", config->sample_rate);
    return ESP_OK;
}

static void adc_deinit(void)
{
    if (s_adc_handle != NULL) {
        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

static esp_err_t i2s_init_channels(const audio_config_t *config)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    channel_config.dma_frame_num = I2S_DMA_BUFFER_SIZE;
    channel_config.auto_clear_after_cb = true;

    esp_err_t ret = i2s_new_channel(&channel_config, &s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)config->i2s_pins.bclk_gpio,
            .ws = (gpio_num_t)config->i2s_pins.ws_gpio,
            .dout = (gpio_num_t)config->i2s_pins.dout_gpio,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };
    ret = i2s_channel_init_std_mode(s_tx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S TX: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "I2S TX initialized on BCLK=%d WS=%d DOUT=%d",
             config->i2s_pins.bclk_gpio, config->i2s_pins.ws_gpio,
             config->i2s_pins.dout_gpio);
    return ESP_OK;
}

static void i2s_deinit_channels(void)
{
    if (s_tx_chan != NULL) {
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
}

static esp_err_t opus_init(const audio_config_t *config)
{
    int error = OPUS_OK;
    s_opus_encoder =
        opus_encoder_create(config->sample_rate, config->channels, OPUS_APPLICATION_VOIP, &error);
    if (error != OPUS_OK || s_opus_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(error));
        return ESP_FAIL;
    }
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_BITRATE(config->opus_bitrate));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_PACKET_LOSS_PERC(OPUS_EXPECTED_LOSS_PERC));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_COMPLEXITY(5));
    opus_encoder_ctl(s_opus_encoder, OPUS_SET_DTX(1));

    s_loopback_decoder = opus_decoder_create(config->sample_rate, config->channels, &error);
    if (error != OPUS_OK || s_loopback_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to create loopback decoder: %s", opus_strerror(error));
        opus_encoder_destroy(s_opus_encoder);
        s_opus_encoder = NULL;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        s_rx_sources[i].decoder =
            opus_decoder_create(config->sample_rate, config->channels, &error);
        if (error != OPUS_OK || s_rx_sources[i].decoder == NULL) {
            ESP_LOGE(TAG, "Failed to create source decoder %zu: %s", i, opus_strerror(error));
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "Opus encoder and %u independent decoders initialized",
             AUDIO_MAX_RX_SOURCES + 1);
    return ESP_OK;
}

static void opus_deinit(void)
{
    if (s_opus_encoder != NULL) {
        opus_encoder_destroy(s_opus_encoder);
        s_opus_encoder = NULL;
    }
    if (s_loopback_decoder != NULL) {
        opus_decoder_destroy(s_loopback_decoder);
        s_loopback_decoder = NULL;
    }
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        if (s_rx_sources[i].decoder != NULL) {
            opus_decoder_destroy(s_rx_sources[i].decoder);
            s_rx_sources[i].decoder = NULL;
        }
    }
}

static void hpf_init(hpf_state_t *state, float cutoff_hz, float sample_rate)
{
    memset(state, 0, sizeof(*state));
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float alpha = sinf(omega) / (2.0f * 0.7071f);
    float cosine = cosf(omega);
    float a0 = 1.0f + alpha;
    state->b0 = (1.0f + cosine) / (2.0f * a0);
    state->b1 = -(1.0f + cosine) / a0;
    state->b2 = state->b0;
    state->a1 = (-2.0f * cosine) / a0;
    state->a2 = (1.0f - alpha) / a0;
}

static void hpf_process(hpf_state_t *state, int16_t *samples, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        float input = (float)samples[i] / 32768.0f;
        float output = state->b0 * input + state->b1 * state->x1 +
                       state->b2 * state->x2 - state->a1 * state->y1 -
                       state->a2 * state->y2;
        state->x2 = state->x1;
        state->x1 = input;
        state->y2 = state->y1;
        state->y1 = output;
        float scaled = output * 32768.0f;
        if (scaled > INT16_MAX) {
            scaled = INT16_MAX;
        } else if (scaled < INT16_MIN) {
            scaled = INT16_MIN;
        }
        samples[i] = (int16_t)scaled;
    }
}

/* Caller holds s_rx_sources_mutex. Decoder and resampler resets remain playout-owned. */
static void reset_rx_source_metadata_locked(void)
{
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        audio_rx_source_t *source = &s_rx_sources[i];
        source->assigned = false;
        source->source_id = 0;
        source->last_enqueue_ms = 0;
        source->last_active_ms = 0;
        source->decoder_reset_pending = true;
        audio_packet_store_reset(&source->packet_store);
    }
}

static void reset_rx_source_metadata(void)
{
    xSemaphoreTake(s_rx_sources_mutex, portMAX_DELAY);
    reset_rx_source_metadata_locked();
    xSemaphoreGive(s_rx_sources_mutex);
}

static void reset_rx_codecs_and_resamplers(void)
{
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        opus_decoder_ctl(s_rx_sources[i].decoder, OPUS_RESET_STATE);
        audio_pcm_resampler_reset(&s_rx_sources[i].resampler);
        s_rx_sources[i].decoded_active = false;
    }
}

static void service_rx_reset_request(void)
{
    if (!atomic_exchange_explicit(&s_rx_reset_requested, false, memory_order_acq_rel)) {
        return;
    }
    reset_rx_source_metadata();
    reset_rx_codecs_and_resamplers();
    xSemaphoreGive(s_rx_reset_done);
}

static void record_decode_result(int samples, int64_t decode_time_us,
                                 int64_t *decode_time_sum)
{
    if (samples != AUDIO_FRAME_SAMPLES) {
        STATS_LOCK();
        s_stats.decode_errors++;
        STATS_UNLOCK();
        ESP_LOGW(TAG, "Opus decode failed: %d", samples);
        return;
    }
    *decode_time_sum += decode_time_us;
    STATS_LOCK();
    s_stats.frames_decoded++;
    s_stats.decode_time_us_avg = (uint32_t)(*decode_time_sum / s_stats.frames_decoded);
    if ((uint32_t)decode_time_us > s_stats.decode_time_us_max) {
        s_stats.decode_time_us_max = (uint32_t)decode_time_us;
    }
    STATS_UNLOCK();
}

static void update_depth_stats(uint16_t packet_depth)
{
    uint8_t depth = packet_depth > UINT8_MAX ? UINT8_MAX : (uint8_t)packet_depth;
    STATS_LOCK();
    s_stats.jitter_buffer_depth = depth;
    if (s_stats.playout_task_loops == 1 || depth < s_stats.rx_q_depth_min) {
        s_stats.rx_q_depth_min = depth;
    }
    if (depth > s_stats.rx_q_depth_max) {
        s_stats.rx_q_depth_max = depth;
    }
    uint64_t previous = (uint64_t)s_stats.rx_q_depth_avg *
                        (uint64_t)(s_stats.playout_task_loops - 1u);
    s_stats.rx_q_depth_avg = (uint8_t)((previous + depth) / s_stats.playout_task_loops);
    STATS_UNLOCK();
}

static bool decode_and_buffer_source(audio_rx_source_t *source, uint64_t now_ms,
                                     int64_t *decode_time_sum, uint16_t *packet_depth,
                                     size_t *upstream_samples)
{
    audio_playout_event_t events[AUDIO_PACKET_STORE_CAPACITY];
    size_t event_count = 0;
    bool assigned;
    bool reset_decoder = false;

    xSemaphoreTake(s_rx_sources_mutex, portMAX_DELAY);
    assigned = source->assigned;
    if (source->decoder_reset_pending) {
        source->decoder_reset_pending = false;
        reset_decoder = true;
    }
    if (assigned && audio_packet_store_depth(&source->packet_store) == 0u &&
        now_ms - source->last_enqueue_ms >= RX_SOURCE_IDLE_TIMEOUT_MS) {
        source->assigned = false;
        source->source_id = 0;
        source->last_active_ms = 0;
        source->decoder_reset_pending = false;
        audio_packet_store_reset(&source->packet_store);
        assigned = false;
        reset_decoder = true;
    }
    xSemaphoreGive(s_rx_sources_mutex);

    if (reset_decoder) {
        opus_decoder_ctl(source->decoder, OPUS_RESET_STATE);
        audio_pcm_resampler_reset(&source->resampler);
        source->decoded_active = false;
    }
    if (!assigned) {
        return false;
    }

    size_t pcm_depth = audio_pcm_resampler_depth(&source->resampler);
    size_t target_room = pcm_depth < AUDIO_PCM_RESAMPLER_TARGET_SAMPLES
                             ? AUDIO_PCM_RESAMPLER_TARGET_SAMPLES - pcm_depth
                             : 0u;
    size_t event_limit = target_room / AUDIO_FRAME_SAMPLES;
    size_t available_events =
        audio_pcm_resampler_available(&source->resampler) / AUDIO_FRAME_SAMPLES;
    if (event_limit > available_events) {
        event_limit = available_events;
    }
    if (event_limit > AUDIO_PACKET_STORE_CAPACITY) {
        event_limit = AUDIO_PACKET_STORE_CAPACITY;
    }
    xSemaphoreTake(s_rx_sources_mutex, portMAX_DELAY);
    if (source->assigned) {
        for (event_count = 0; event_count < event_limit; ++event_count) {
            audio_playout_event_t *event = &events[event_count];
            event->result =
                audio_packet_store_pop(&source->packet_store, now_ms, &event->packet);
            if (event->result == AUDIO_PACKET_STORE_POP_NOT_DUE) {
                break;
            }
        }
        size_t remaining_depth = audio_packet_store_depth(&source->packet_store);
        *packet_depth += (uint16_t)remaining_depth;
        *upstream_samples = remaining_depth * AUDIO_FRAME_SAMPLES;
    } else {
        assigned = false;
    }
    xSemaphoreGive(s_rx_sources_mutex);
    if (!assigned) {
        return false;
    }

    int16_t decoded[AUDIO_FRAME_SAMPLES];
    for (size_t i = 0; i < event_count; ++i) {
        audio_playout_event_t *event = &events[i];
        bool packet_event = event->result == AUDIO_PACKET_STORE_POP_PACKET;
        if (packet_event) {
            source->decoded_active = event->packet.active;
        } else if (event->result == AUDIO_PACKET_STORE_POP_DTX_IDLE) {
            source->decoded_active = false;
        }
        int64_t decode_start = esp_timer_get_time();
        int samples = packet_event
                          ? opus_decode(source->decoder, event->packet.data,
                                        (opus_int32)event->packet.length, decoded,
                                        AUDIO_FRAME_SAMPLES, 0)
                          : opus_decode(source->decoder, NULL, 0, decoded,
                                        AUDIO_FRAME_SAMPLES, 0);
        int64_t decode_time = esp_timer_get_time() - decode_start;
        if (packet_event) {
            if (event->packet.received_us != 0u &&
                (uint64_t)decode_start >= event->packet.received_us) {
                uint64_t rx_pipe_us = (uint64_t)decode_start - event->packet.received_us;
                STATS_LOCK();
                s_rx_pipe_count++;
                s_rx_pipe_sum_us += rx_pipe_us;
                s_stats.rx_pipe_us_avg =
                    (uint32_t)(s_rx_pipe_sum_us / s_rx_pipe_count);
                if (rx_pipe_us > s_stats.rx_pipe_us_max) {
                    s_stats.rx_pipe_us_max =
                        rx_pipe_us > UINT32_MAX ? UINT32_MAX : (uint32_t)rx_pipe_us;
                }
                STATS_UNLOCK();
            }
            record_decode_result(samples, decode_time, decode_time_sum);
        } else if (samples != AUDIO_FRAME_SAMPLES) {
            STATS_LOCK();
            s_stats.decode_errors++;
            STATS_UNLOCK();
            ESP_LOGW(TAG, "Opus PLC failed: %d", samples);
        } else if (event->result == AUDIO_PACKET_STORE_POP_MISSING) {
            STATS_LOCK();
            s_stats.seq_gap_frames++;
            s_stats.conceal_loss_frames++;
            STATS_UNLOCK();
        } else {
            STATS_LOCK();
            s_stats.plc_frames++;
            STATS_UNLOCK();
        }
        if (samples == AUDIO_FRAME_SAMPLES) {
            audio_pcm_resampler_telemetry_t push =
                audio_pcm_resampler_push(&source->resampler, decoded, AUDIO_FRAME_SAMPLES,
                                         source->decoded_active);
            if (push.rejected_push) {
                STATS_LOCK();
                s_stats.pcm_fifo_overflows++;
                s_stats.frames_dropped++;
                s_stats.glitches_detected++;
                STATS_UNLOCK();
            }
        }
    }
    return true;
}

static bool render_remote_sources(uint64_t now_ms, int64_t *decode_time_sum)
{
    uint8_t active_sources = 0;
    uint8_t mixed_sources = 0;
    uint16_t packet_depth = 0;
    int32_t current_ppm = 0;
    int32_t current_abs_ppm = 0;
    bool recovery_active = false;
    memset(s_mix_frame, 0, sizeof(s_mix_frame));

    for (size_t source_index = 0; source_index < AUDIO_MAX_RX_SOURCES; ++source_index) {
        audio_rx_source_t *source = &s_rx_sources[source_index];
        size_t upstream_samples = 0u;
        bool assigned = decode_and_buffer_source(source, now_ms, decode_time_sum,
                                                 &packet_depth, &upstream_samples);
        if (!assigned) {
            continue;
        }
        int16_t rendered[AUDIO_FRAME_SAMPLES];
        bool was_started = source->resampler.started;
        audio_pcm_resampler_telemetry_t render =
            audio_pcm_resampler_render(&source->resampler, rendered, upstream_samples);
        recovery_active = recovery_active || render.recovery_active;
        if (render.underrun) {
            STATS_LOCK();
            s_stats.pcm_underruns++;
            s_stats.rx_queue_underruns++;
            s_stats.glitches_detected++;
            STATS_UNLOCK();
        }
        int32_t abs_ppm = render.correction_ppm < 0 ? -render.correction_ppm
                                                    : render.correction_ppm;
        if (abs_ppm > current_abs_ppm) {
            current_abs_ppm = abs_ppm;
            current_ppm = render.correction_ppm;
        }
        if (render.audible_active &&
            (render.started || (was_started && render.underrun))) {
            active_sources++;
            for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
                s_mix_frame[i] += rendered[i];
            }
            mixed_sources++;
        }
    }

    STATS_LOCK();
    s_stats.active_rx_sources = active_sources;
    s_stats.asrc_correction_ppm = current_ppm;
    s_stats.asrc_recovery_active = recovery_active;
    if ((uint32_t)current_abs_ppm > s_stats.asrc_correction_abs_max_ppm) {
        s_stats.asrc_correction_abs_max_ppm = (uint32_t)current_abs_ppm;
    }
    STATS_UNLOCK();
    update_depth_stats(packet_depth);

    if (mixed_sources == 0) {
        memset(s_pcm_output, 0, sizeof(s_pcm_output));
        return false;
    }
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
        int32_t sample = s_mix_frame[i];
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
    return true;
}

static void render_loopback(int64_t *decode_time_sum)
{
    audio_loopback_item_t item;
    if (xQueueReceive(s_loopback_queue, &item, 0) != pdTRUE) {
        memset(s_pcm_output, 0, sizeof(s_pcm_output));
        return;
    }
    int64_t decode_start = esp_timer_get_time();
    int samples = opus_decode(s_loopback_decoder, item.data, item.length, s_pcm_output,
                              AUDIO_FRAME_SAMPLES, 0);
    int64_t decode_time = esp_timer_get_time() - decode_start;
    record_decode_result(samples, decode_time, decode_time_sum);
    if (samples != AUDIO_FRAME_SAMPLES) {
        memset(s_pcm_output, 0, sizeof(s_pcm_output));
    }
}

static void log_audio_stats(void)
{
    audio_stats_t stats = audio_stats_snapshot();
    ESP_LOGI(TAG, "Audio loops capture=%lu playout=%lu encoded=%lu decoded=%lu",
             stats.task_loops, stats.playout_task_loops, stats.frames_encoded,
             stats.frames_decoded);
    ESP_LOGI(TAG, "  Encoded: %lu frames", stats.frames_encoded);
    ESP_LOGI(TAG, "  Decoded: %lu frames", stats.frames_decoded);
    ESP_LOGI(TAG, "  VOX activations: %lu (active: %s)", stats.vox_activations,
             stats.vox_active ? "YES" : "no");
    ESP_LOGI(TAG, "  Encode time: avg=%lu us, max=%lu us", stats.encode_time_us_avg,
             stats.encode_time_us_max);
    ESP_LOGI(TAG, "  Decode time: avg=%lu us, max=%lu us", stats.decode_time_us_avg,
             stats.decode_time_us_max);
    ESP_LOGI(TAG, "  TX pipeline: avg=%lu us, max=%lu us", stats.tx_pipe_us_avg,
             stats.tx_pipe_us_max);
    ESP_LOGI(TAG, "  RX pipeline: avg=%lu us, max=%lu us", stats.rx_pipe_us_avg,
             stats.rx_pipe_us_max);
    ESP_LOGI(TAG, "  Latency: avg=%lu ms, max=%lu ms", stats.latency_ms_avg,
             stats.latency_ms_max);
    ESP_LOGI(TAG, "  Glitches: %lu (rx_und=%lu i2s_inc=%lu), ADC overruns: %lu",
             stats.glitches_detected, stats.rx_queue_underruns,
             stats.i2s_write_incomplete, stats.adc_overruns);
    ESP_LOGI(TAG,
             "  Concealment: plc=%lu grace_empty=%lu conceal=%lu seq_gap=%lu "
             "seq_reset=%lu seq_stale=%lu",
             stats.plc_frames, stats.grace_empty_polls, stats.conceal_loss_frames,
             stats.seq_gap_frames, stats.seq_resets, stats.seq_stale_drops);
    ESP_LOGI(TAG, "  Adaptive playout: hold=%lu catchup=%lu sources=%u",
             stats.hold_frames, stats.catchup_frames, stats.active_rx_sources);
    ESP_LOGI(TAG, "  RX queue depth/source: min=%u avg=%u max=%u (total now=%u)",
             stats.rx_q_depth_min, stats.rx_q_depth_avg, stats.rx_q_depth_max,
             stats.jitter_buffer_depth);
    ESP_LOGI(TAG,
             "Packet drops duplicate=%lu late=%lu future=%lu full=%lu source=%lu lock=%lu",
             stats.packet_duplicate_drops, stats.packet_late_drops,
             stats.packet_future_drops, stats.rx_queue_overflows,
             stats.rx_source_rejections, stats.rx_lock_drops);
    ESP_LOGI(TAG,
             "Playout plc=%lu conceal=%lu seq_gap=%lu pcm_overflow=%lu pcm_underrun=%lu "
             "asrc_ppm=%ld asrc_abs_max=%lu",
             stats.plc_frames, stats.conceal_loss_frames, stats.seq_gap_frames,
             stats.pcm_fifo_overflows, stats.pcm_underruns,
             (long)stats.asrc_correction_ppm, stats.asrc_correction_abs_max_ppm);
    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=audio capture_ok=%lu capture_short=%lu "
             "capture_timeout=%lu capture_err=%lu encode_ok=%lu encode_err=%lu dtx_drop=%lu "
             "rx_q_drop=%lu rx_lock_drop=%lu rx_src_drop=%lu rx_src_evict=%lu "
             "jitter_drop=%lu decode_ok=%lu "
             "decode_err=%lu plc=%lu hold=%lu catchup=%lu conceal=%lu seq_gap=%lu "
             "seq_reset=%lu seq_stale=%lu glitch=%lu play_ok=%lu i2s_err=%lu notify_drop=%lu "
             "rx_sources=%u packet_dup=%lu packet_late=%lu packet_future=%lu pcm_overflow=%lu "
             "pcm_underrun=%lu asrc_ppm=%ld asrc_abs_max_ppm=%lu asrc_recovery=%u "
             "playout_loops=%lu",
             stats.capture_frames_ok, stats.capture_short_reads, stats.capture_timeouts,
             stats.adc_overruns, stats.frames_encoded, stats.encode_errors,
             stats.tx_dtx_suppressed, stats.rx_queue_overflows, stats.rx_lock_drops,
             stats.rx_source_rejections, stats.rx_source_evictions,
             stats.jitter_trim_frames, stats.frames_decoded,
             stats.decode_errors, stats.plc_frames, stats.hold_frames,
             stats.catchup_frames, stats.conceal_loss_frames, stats.seq_gap_frames,
             stats.seq_resets, stats.seq_stale_drops, stats.glitches_detected,
             stats.playback_frames, stats.i2s_write_incomplete,
             stats.notification_queue_overflows, stats.active_rx_sources,
             stats.packet_duplicate_drops, stats.packet_late_drops,
             stats.packet_future_drops, stats.pcm_fifo_overflows, stats.pcm_underruns,
             (long)stats.asrc_correction_ppm, stats.asrc_correction_abs_max_ppm,
             stats.asrc_recovery_active ? 1u : 0u,
             stats.playout_task_loops);
}

static void playout_task_finish(void)
{
    atomic_store_explicit(&s_playout_ready, false, memory_order_release);
    portENTER_CRITICAL(&s_task_lock);
    bool reset_requested =
        atomic_exchange_explicit(&s_rx_reset_requested, false, memory_order_acq_rel);
    s_playout_task = NULL;
    portEXIT_CRITICAL(&s_task_lock);
    if (reset_requested) {
        reset_rx_source_metadata();
        reset_rx_codecs_and_resamplers();
        xSemaphoreGive(s_rx_reset_done);
    }
    xSemaphoreGive(s_playout_done);
    vTaskDelete(NULL);
}

static void playout_task(void *arg)
{
    (void)arg;
    int64_t decode_time_sum = 0;
    int64_t last_heartbeat_ms = esp_timer_get_time() / 1000;

    opus_decoder_ctl(s_loopback_decoder, OPUS_RESET_STATE);
    reset_rx_codecs_and_resamplers();
    reset_far_reference_shadows();
    for (size_t i = 0; i < I2S_DMA_BUFFER_COUNT; ++i) {
        size_t bytes_loaded = 0;
        esp_err_t preload = i2s_channel_preload_data(
            s_tx_chan, s_i2s_silence, sizeof(s_i2s_silence), &bytes_loaded);
        if (preload != ESP_OK || bytes_loaded != sizeof(s_i2s_silence)) {
            ESP_LOGE(TAG, "I2S silence preload %zu failed: %s (%zu bytes)", i,
                     esp_err_to_name(preload), bytes_loaded);
            atomic_store_explicit(&s_running, false, memory_order_release);
            xSemaphoreGive(s_playout_started);
            playout_task_finish();
        }
    }
    esp_err_t ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX: %s", esp_err_to_name(ret));
        atomic_store_explicit(&s_running, false, memory_order_release);
        xSemaphoreGive(s_playout_started);
        playout_task_finish();
    }

    atomic_store_explicit(&s_playout_ready, true, memory_order_release);
    xSemaphoreGive(s_playout_started);
    ESP_LOGI(TAG, "Playout task started on core %d", xPortGetCoreID());

    while (atomic_load_explicit(&s_running, memory_order_acquire)) {
        service_rx_reset_request();
        STATS_LOCK();
        s_stats.playout_task_loops++;
        STATS_UNLOCK();
        if (s_config.mode == AUDIO_MODE_LOOPBACK) {
            render_loopback(&decode_time_sum);
        } else {
            (void)render_remote_sources((uint64_t)(esp_timer_get_time() / 1000),
                                        &decode_time_sum);
        }
        mix_notification_frame();

        for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
            s_pcm_stereo[i * 2] = s_pcm_output[i];
            s_pcm_stereo[i * 2 + 1] = s_pcm_output[i];
        }
        size_t bytes_written = 0;
        ret = i2s_channel_write(s_tx_chan, s_pcm_stereo, sizeof(s_pcm_stereo),
                                &bytes_written, I2S_WRITE_TIMEOUT_MS);
        if (ret != ESP_OK || bytes_written != sizeof(s_pcm_stereo)) {
            STATS_LOCK();
            s_stats.i2s_write_incomplete++;
            s_stats.glitches_detected++;
            STATS_UNLOCK();
            reset_far_reference_shadows();
            ESP_LOGW(TAG, "I2S write incomplete: %s (%zu bytes)", esp_err_to_name(ret),
                     bytes_written);
        } else {
            advance_far_reference_after_write();
            STATS_LOCK();
            s_stats.playback_frames++;
            STATS_UNLOCK();
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_heartbeat_ms >= AUDIO_HEARTBEAT_INTERVAL_MS) {
            log_audio_stats();
            last_heartbeat_ms = now_ms;
        }
    }

    service_rx_reset_request();
    i2s_channel_disable(s_tx_chan);
    ESP_LOGI(TAG, "Playout task stopped");
    playout_task_finish();
}

static void capture_task_finish(void)
{
    atomic_store_explicit(&s_adc_notify_task, NULL, memory_order_release);
    portENTER_CRITICAL(&s_task_lock);
    s_capture_task = NULL;
    portEXIT_CRITICAL(&s_task_lock);
    xSemaphoreGive(s_capture_done);
    vTaskDelete(NULL);
}

static void capture_task(void *arg)
{
    (void)arg;
    static uint8_t adc_buffer[ADC_CONV_FRAME_SIZE];
    static int16_t silence_frame[AUDIO_FRAME_SAMPLES];
    int64_t encode_time_sum = 0;
    int64_t latency_sum = 0;

    atomic_store_explicit(&s_adc_notify_task, xTaskGetCurrentTaskHandle(),
                          memory_order_release);
    opus_encoder_ctl(s_opus_encoder, OPUS_RESET_STATE);
    esp_err_t ret = adc_continuous_start(s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        atomic_store_explicit(&s_running, false, memory_order_release);
        xSemaphoreGive(s_capture_started);
        capture_task_finish();
    }
    atomic_store_explicit(&s_capture_ready, true, memory_order_release);
    xSemaphoreGive(s_capture_started);
    ESP_LOGI(TAG, "Capture task started on core %d", xPortGetCoreID());

    while (atomic_load_explicit(&s_running, memory_order_acquire)) {
        int64_t frame_start_us = esp_timer_get_time();
        STATS_LOCK();
        s_stats.task_loops++;
        STATS_UNLOCK();
        bool notified =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADC_READ_TIMEOUT_MS)) != 0;
        if (!atomic_load_explicit(&s_running, memory_order_acquire)) {
            break;
        }
        if (!notified) {
            STATS_LOCK();
            s_stats.capture_timeouts++;
            STATS_UNLOCK();
        }

        uint32_t bytes_read = 0;
        ret = adc_continuous_read(s_adc_handle, adc_buffer, sizeof(adc_buffer),
                                  &bytes_read, 0);
        if (ret == ESP_ERR_TIMEOUT) {
            if (notified) {
                STATS_LOCK();
                s_stats.capture_timeouts++;
                STATS_UNLOCK();
            }
            continue;
        }
        if (ret != ESP_OK || bytes_read == 0) {
            STATS_LOCK();
            s_stats.adc_overruns++;
            STATS_UNLOCK();
            ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(ret));
            continue;
        }
        if (bytes_read == ADC_CONV_FRAME_SIZE) {
            STATS_LOCK();
            s_stats.capture_frames_ok++;
            STATS_UNLOCK();
        } else {
            STATS_LOCK();
            s_stats.capture_short_reads++;
            STATS_UNLOCK();
        }

        size_t adc_samples = bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
        for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
            int32_t sum = 0;
            size_t valid = 0;
            for (size_t j = 0; j < ADC_OVERSAMPLE_FACTOR; ++j) {
                size_t index = i * ADC_OVERSAMPLE_FACTOR + j;
                if (index < adc_samples) {
                    adc_digi_output_data_t *sample = (adc_digi_output_data_t *)&adc_buffer[
                        index * SOC_ADC_DIGI_RESULT_BYTES];
                    sum += sample->type2.data;
                    valid++;
                }
            }
            if (valid == 0) {
                s_pcm_input[i] = 0;
                continue;
            }
            int16_t sample = (int16_t)(((sum / (int32_t)valid) - 2048) * 8);
            s_dc_estimate = s_dc_estimate * 0.999f + (float)sample * 0.001f;
            sample -= (int16_t)s_dc_estimate;
            sample = (int16_t)((s_lpf_prev * 3 + sample) / 4);
            s_lpf_prev = sample;
            s_pcm_input[i] = sample;
        }
        if (s_config.enable_hpf) {
            hpf_process(&s_hpf_state, s_pcm_input, AUDIO_FRAME_SAMPLES);
        }
#if AUDIO_ENABLE_AEC_NS
        if (s_config.mode == AUDIO_MODE_MESH) {
            int16_t far_reference[AUDIO_FRAME_SAMPLES];
            portENTER_CRITICAL(&s_far_ref_lock);
            memcpy(far_reference, s_far_ref_frame, sizeof(far_reference));
            portEXIT_CRITICAL(&s_far_ref_lock);
            voice_cleanup_process(&s_voice_cleanup, s_pcm_input, far_reference,
                                  AUDIO_FRAME_SAMPLES);
        }
#endif

        bool was_active = s_vox_state.active;
        bool vox_active = vox_process(&s_vox_state, s_pcm_input, AUDIO_FRAME_SAMPLES);
        portENTER_CRITICAL(&s_task_lock);
        audio_activity_cb_t activity_callback = s_activity_callback;
        portEXIT_CRITICAL(&s_task_lock);
        if (vox_active != was_active && activity_callback != NULL) {
            activity_callback(vox_active);
        }
        STATS_LOCK();
        s_stats.vox_active = vox_active;
        s_stats.vox_activations = s_vox_state.activation_count;
        STATS_UNLOCK();
        bool tx_active = vox_active || s_config.force_tx_always;
        const int16_t *encode_input = tx_active ? s_pcm_input : silence_frame;
        int64_t encode_start = esp_timer_get_time();
        int encoded = opus_encode(s_opus_encoder, encode_input, AUDIO_FRAME_SAMPLES,
                                  s_opus_buffer, sizeof(s_opus_buffer));
        int64_t encode_time = esp_timer_get_time() - encode_start;
        if (encoded <= 0) {
            STATS_LOCK();
            s_stats.encode_errors++;
            STATS_UNLOCK();
            ESP_LOGW(TAG, "Opus encode failed: %s", opus_strerror(encoded));
            continue;
        }
        encode_time_sum += encode_time;
        STATS_LOCK();
        s_stats.frames_encoded++;
        s_stats.encode_time_us_avg = (uint32_t)(encode_time_sum / s_stats.frames_encoded);
        if ((uint32_t)encode_time > s_stats.encode_time_us_max) {
            s_stats.encode_time_us_max = (uint32_t)encode_time;
        }
        uint32_t encoded_frames = s_stats.frames_encoded;
        STATS_UNLOCK();

        bool comfort_update = encoded > OPUS_DTX_FRAME_MAX_BYTES;
        if (s_config.mode == AUDIO_MODE_MESH) {
            if (tx_active || comfort_update) {
                portENTER_CRITICAL(&s_task_lock);
                audio_tx_cb_t tx_callback = s_tx_callback;
                portEXIT_CRITICAL(&s_task_lock);
                if (tx_callback != NULL) {
                    tx_callback(s_opus_buffer, (uint16_t)encoded, tx_active,
                                frame_start_us);
                }
            } else {
                STATS_LOCK();
                s_stats.tx_dtx_suppressed++;
                STATS_UNLOCK();
            }
        } else {
            audio_loopback_item_t item = {
                .length = (uint16_t)encoded,
                .timestamp_us = frame_start_us,
            };
            memcpy(item.data, s_opus_buffer, (size_t)encoded);
            if (xQueueSend(s_loopback_queue, &item, 0) != pdTRUE) {
                STATS_LOCK();
                s_stats.frames_dropped++;
                STATS_UNLOCK();
            }
        }

        int64_t processing_time_us = esp_timer_get_time() - frame_start_us;
        /* Two descriptors bound queued speaker data to roughly one or two frames. */
        int64_t dma_latency_us = ((int64_t)I2S_DMA_BUFFER_COUNT / 2) * 20000;
        int64_t total_latency_us = processing_time_us + dma_latency_us;
        latency_sum += total_latency_us;
        STATS_LOCK();
        s_stats.latency_ms_avg =
            (uint32_t)((latency_sum / (int64_t)encoded_frames) / 1000);
        uint32_t latency_ms = (uint32_t)(total_latency_us / 1000);
        if (latency_ms > s_stats.latency_ms_max) {
            s_stats.latency_ms_max = latency_ms;
        }
        STATS_UNLOCK();
    }

    adc_continuous_stop(s_adc_handle);
    ESP_LOGI(TAG, "Capture task stopped");
    capture_task_finish();
}

static void delete_sync_resources(void)
{
    if (s_notification_queue != NULL) {
        vQueueDelete(s_notification_queue);
        s_notification_queue = NULL;
    }
    if (s_loopback_queue != NULL) {
        vQueueDelete(s_loopback_queue);
        s_loopback_queue = NULL;
    }
    SemaphoreHandle_t *semaphores[] = {
        &s_rx_sources_mutex, &s_rx_reset_mutex, &s_rx_reset_done,
        &s_playout_started, &s_capture_started, &s_capture_done, &s_playout_done,
    };
    for (size_t i = 0; i < sizeof(semaphores) / sizeof(semaphores[0]); ++i) {
        if (*semaphores[i] != NULL) {
            vSemaphoreDelete(*semaphores[i]);
            *semaphores[i] = NULL;
        }
    }
}

esp_err_t audio_init(void)
{
    return audio_init_with_config(NULL);
}

static esp_err_t audio_init_with_config_locked(const audio_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    audio_config_t requested = AUDIO_CONFIG_DEFAULT();
    if (config != NULL) {
        requested = *config;
    }
    if (requested.sample_rate != 16000 || requested.channels != 1 ||
        requested.bits_per_sample != 16 || requested.frame_size_ms != 20) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (requested.mode != AUDIO_MODE_LOOPBACK && requested.mode != AUDIO_MODE_MESH) {
        return ESP_ERR_INVALID_ARG;
    }
    s_config = requested;

    esp_err_t ret = adc_init(&s_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2s_init_channels(&s_config);
    if (ret != ESP_OK) {
        adc_deinit();
        return ret;
    }
    ret = opus_init(&s_config);
    if (ret != ESP_OK) {
        opus_deinit();
        i2s_deinit_channels();
        adc_deinit();
        return ret;
    }

    s_playout_started = xSemaphoreCreateBinary();
    s_capture_started = xSemaphoreCreateBinary();
    s_capture_done = xSemaphoreCreateBinary();
    s_playout_done = xSemaphoreCreateBinary();
    s_rx_reset_done = xSemaphoreCreateBinary();
    s_rx_reset_mutex = xSemaphoreCreateMutex();
    s_rx_sources_mutex = xSemaphoreCreateMutex();
    s_loopback_queue = xQueueCreate(LOOPBACK_QUEUE_SIZE, sizeof(audio_loopback_item_t));
    s_notification_queue =
        xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(audio_notification_request_t));
    if (s_playout_started == NULL || s_capture_started == NULL || s_capture_done == NULL ||
        s_playout_done == NULL || s_rx_reset_done == NULL || s_rx_reset_mutex == NULL ||
        s_rx_sources_mutex == NULL || s_loopback_queue == NULL ||
        s_notification_queue == NULL) {
        delete_sync_resources();
        opus_deinit();
        i2s_deinit_channels();
        adc_deinit();
        return ESP_ERR_NO_MEM;
    }

    STATS_LOCK();
    memset(&s_stats, 0, sizeof(s_stats));
    s_tx_pipe_sum_us = 0u;
    s_tx_pipe_count = 0u;
    s_rx_pipe_sum_us = 0u;
    s_rx_pipe_count = 0u;
    STATS_UNLOCK();
    memset(&s_notification, 0, sizeof(s_notification));
    reset_far_reference_shadows();
    s_dc_estimate = 0.0f;
    s_lpf_prev = 0;
    hpf_init(&s_hpf_state, s_config.hpf_cutoff_hz, s_config.sample_rate);
    vox_init(&s_vox_state, &s_config.vox_config);
    voice_cleanup_init(&s_voice_cleanup);
    reset_rx_source_metadata();
    s_initialized = true;
    ESP_LOGI(TAG, "Audio subsystem initialized");
    return ESP_OK;
}

esp_err_t audio_init_with_config(const audio_config_t *config)
{
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lifecycle_mutex == NULL) {
        s_lifecycle_mutex = xSemaphoreCreateMutexStatic(&s_lifecycle_mutex_storage);
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret = s_deinitializing ? ESP_ERR_INVALID_STATE
                                    : audio_init_with_config_locked(config);
    xSemaphoreGive(s_lifecycle_mutex);
    return ret;
}

esp_err_t audio_deinit(void)
{
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_deinitializing) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_deinitializing = true;
    s_stopping = true;
    atomic_store_explicit(&s_running, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    if (ret != ESP_OK) {
        s_deinitializing = false;
        s_stopping = false;
        xSemaphoreGive(s_lifecycle_mutex);
        return ret;
    }
    reset_rx_source_metadata();
    opus_deinit();
    i2s_deinit_channels();
    adc_deinit();
    delete_sync_resources();
    s_initialized = false;
    s_deinitializing = false;
    s_stopping = false;
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}

static esp_err_t audio_start_locked(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_task_lock);
    bool tasks_exist = s_capture_task != NULL || s_playout_task != NULL;
    portEXIT_CRITICAL(&s_task_lock);
    if (atomic_load_explicit(&s_running, memory_order_acquire) || tasks_exist) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_playout_started, 0);
    xSemaphoreTake(s_capture_started, 0);
    xSemaphoreTake(s_playout_done, 0);
    xSemaphoreTake(s_capture_done, 0);
    xQueueReset(s_loopback_queue);
    atomic_store_explicit(&s_playout_ready, false, memory_order_release);
    atomic_store_explicit(&s_capture_ready, false, memory_order_release);
    atomic_store_explicit(&s_running, true, memory_order_release);

    BaseType_t created = xTaskCreatePinnedToCore(
        playout_task, "audio_playout", AUDIO_PLAYOUT_TASK_STACK_SIZE, NULL,
        AUDIO_PLAYOUT_TASK_PRIORITY, &s_playout_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        s_playout_task = NULL;
        atomic_store_explicit(&s_running, false, memory_order_release);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_playout_started, portMAX_DELAY);
    if (!atomic_load_explicit(&s_playout_ready, memory_order_acquire)) {
        xSemaphoreTake(s_playout_done, portMAX_DELAY);
        return ESP_FAIL;
    }

    created = xTaskCreatePinnedToCore(
        capture_task, "audio_capture", AUDIO_CAPTURE_TASK_STACK_SIZE, NULL,
        AUDIO_CAPTURE_TASK_PRIORITY, &s_capture_task, AUDIO_TASK_CORE);
    if (created != pdPASS) {
        s_capture_task = NULL;
        atomic_store_explicit(&s_running, false, memory_order_release);
        xSemaphoreTake(s_playout_done, portMAX_DELAY);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_capture_started, portMAX_DELAY);
    if (!atomic_load_explicit(&s_capture_ready, memory_order_acquire)) {
        xSemaphoreTake(s_capture_done, portMAX_DELAY);
        xSemaphoreTake(s_playout_done, portMAX_DELAY);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t audio_start(void)
{
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    esp_err_t ret = (s_stopping || s_deinitializing) ? ESP_ERR_INVALID_STATE
                                                     : audio_start_locked();
    xSemaphoreGive(s_lifecycle_mutex);
    return ret;
}

static esp_err_t audio_stop_locked(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    atomic_store_explicit(&s_running, false, memory_order_release);
    portENTER_CRITICAL(&s_task_lock);
    bool wait_capture = s_capture_task != NULL;
    bool wait_playout = s_playout_task != NULL;
    if (wait_capture) {
        xTaskNotifyGive(s_capture_task);
    }
    portEXIT_CRITICAL(&s_task_lock);
    if (wait_capture) {
        xSemaphoreTake(s_capture_done, portMAX_DELAY);
    }
    if (wait_playout) {
        xSemaphoreTake(s_playout_done, portMAX_DELAY);
    }
    reset_rx_source_metadata();
    xQueueReset(s_loopback_queue);
    xQueueReset(s_notification_queue);
    memset(&s_notification, 0, sizeof(s_notification));
    reset_far_reference_shadows();
    STATS_LOCK();
    s_stats.asrc_correction_ppm = 0;
    s_stats.asrc_recovery_active = false;
    s_stats.rx_pipe_us_avg = 0u;
    s_stats.rx_pipe_us_max = 0u;
    s_rx_pipe_sum_us = 0u;
    s_rx_pipe_count = 0u;
    STATS_UNLOCK();
    return ESP_OK;
}

esp_err_t audio_stop(void)
{
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_deinitializing) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_stopping) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_OK;
    }
    s_stopping = true;
    atomic_store_explicit(&s_running, false, memory_order_release);
    esp_err_t ret = audio_stop_locked();
    s_stopping = false;
    xSemaphoreGive(s_lifecycle_mutex);
    return ret;
}

bool audio_vox_active(void)
{
    audio_stats_t stats = audio_stats_snapshot();
    return stats.vox_active;
}

esp_err_t audio_get_tx_frame(audio_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t audio_put_rx_frame(const audio_frame_t *frame, uint8_t source_id)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->len == 0 || frame->len > AUDIO_PACKET_MAX_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }

    audio_packet_t packet = {
        .length = frame->len,
        .sequence = frame->seq,
        .mode = frame->has_seq ? AUDIO_PACKET_MODE_SEQUENCED
                               : AUDIO_PACKET_MODE_ARRIVAL_ORDER,
        .active = frame->active,
        .received_us = frame->timestamp_ms > 0
                           ? (uint64_t)frame->timestamp_ms * UINT64_C(1000)
                           : 0u,
    };
    memcpy(packet.data, frame->data, frame->len);

    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (s_deinitializing || s_rx_sources_mutex == NULL) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_rx_sources_mutex, pdMS_TO_TICKS(RX_ENQUEUE_LOCK_WAIT_MS)) !=
        pdTRUE) {
        STATS_LOCK();
        s_stats.frames_dropped++;
        s_stats.rx_lock_drops++;
        STATS_UNLOCK();
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_TIMEOUT;
    }
    if (!s_initialized || !atomic_load_explicit(&s_running, memory_order_acquire) ||
        source_id == 0 || s_config.mode != AUDIO_MODE_MESH) {
        xSemaphoreGive(s_rx_sources_mutex);
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    int64_t now_us = esp_timer_get_time();
    uint64_t alloc_now_ms = (uint64_t)(now_us / 1000);
    audio_rx_source_t *source = NULL;
    audio_rx_source_t *free_source = NULL;
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
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
        if (source == NULL && frame->active) {
            /* Top-N selection: a new active talker displaces the source
             * that has been VOX-silent the longest. Recently active
             * sources keep their slot (first speaker wins). */
            audio_rx_source_t *victim = NULL;
            for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
                if (victim == NULL ||
                    s_rx_sources[i].last_active_ms < victim->last_active_ms) {
                    victim = &s_rx_sources[i];
                }
            }
            if (victim != NULL &&
                alloc_now_ms - victim->last_active_ms >= RX_SOURCE_EVICT_SILENCE_MS) {
                source = victim;
                source->assigned = false;
                source->last_active_ms = 0;
                STATS_LOCK();
                s_stats.rx_source_evictions++;
                STATS_UNLOCK();
            }
        }
        if (source == NULL) {
            xSemaphoreGive(s_rx_sources_mutex);
            STATS_LOCK();
            s_stats.frames_dropped++;
            s_stats.rx_source_rejections++;
            STATS_UNLOCK();
            xSemaphoreGive(s_lifecycle_mutex);
            return ESP_ERR_NO_MEM;
        }
        source->assigned = true;
        source->source_id = source_id;
        source->decoder_reset_pending = true;
        audio_packet_store_reset(&source->packet_store);
    }
    if (frame->active) {
        source->last_active_ms = alloc_now_ms;
    }
    if (packet.received_us == 0u) {
        packet.received_us = (uint64_t)now_us;
    }
    uint64_t now_ms = (uint64_t)(now_us / 1000);
    audio_packet_store_push_result_t result =
        audio_packet_store_push(&source->packet_store, &packet, now_ms);
    bool reanchored = false;
    if (result == AUDIO_PACKET_STORE_PUSH_FUTURE) {
        audio_packet_store_reset(&source->packet_store);
        audio_packet_store_push_result_t reanchor_result =
            audio_packet_store_push(&source->packet_store, &packet, now_ms);
        if (reanchor_result == AUDIO_PACKET_STORE_PUSH_OK) {
            source->decoder_reset_pending = true;
            result = AUDIO_PACKET_STORE_PUSH_OK;
            reanchored = true;
        } else {
            result = reanchor_result;
        }
    }
    if (result == AUDIO_PACKET_STORE_PUSH_OK) {
        source->last_enqueue_ms = now_ms;
    }
    xSemaphoreGive(s_rx_sources_mutex);

    esp_err_t ret = ESP_OK;
    STATS_LOCK();
    if (reanchored) {
        s_stats.packet_future_drops++;
        s_stats.seq_resets++;
        s_stats.glitches_detected++;
    }
    if (result == AUDIO_PACKET_STORE_PUSH_OK) {
        ret = ESP_OK;
    } else {
        s_stats.frames_dropped++;
        switch (result) {
        case AUDIO_PACKET_STORE_PUSH_DUPLICATE:
            s_stats.packet_duplicate_drops++;
            s_stats.seq_stale_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_LATE:
            s_stats.packet_late_drops++;
            s_stats.seq_stale_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_FUTURE:
            s_stats.packet_future_drops++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        case AUDIO_PACKET_STORE_PUSH_FULL:
            s_stats.rx_queue_overflows++;
            ret = ESP_ERR_NO_MEM;
            break;
        default:
            s_stats.rx_source_rejections++;
            ret = ESP_ERR_INVALID_STATE;
            break;
        }
    }
    STATS_UNLOCK();
    xSemaphoreGive(s_lifecycle_mutex);
    return ret;
}

void audio_clear_rx_frames(void)
{
    if (s_lifecycle_mutex == NULL) {
        return;
    }
    if (called_from_audio_worker()) {
        return;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_stopping || s_deinitializing || s_rx_reset_mutex == NULL) {
        xSemaphoreGive(s_lifecycle_mutex);
        return;
    }
    xSemaphoreTake(s_rx_reset_mutex, portMAX_DELAY);
    xSemaphoreTake(s_rx_reset_done, 0);
    portENTER_CRITICAL(&s_task_lock);
    bool have_playout = s_playout_task != NULL;
    if (have_playout) {
        atomic_store_explicit(&s_rx_reset_requested, true, memory_order_release);
    }
    portEXIT_CRITICAL(&s_task_lock);
    if (have_playout) {
        xSemaphoreTake(s_rx_reset_done, portMAX_DELAY);
    } else {
        reset_rx_source_metadata();
    }
    xSemaphoreGive(s_rx_reset_mutex);
    xSemaphoreGive(s_lifecycle_mutex);
}

esp_err_t audio_register_tx_callback(audio_tx_cb_t callback)
{
    portENTER_CRITICAL(&s_task_lock);
    s_tx_callback = callback;
    portEXIT_CRITICAL(&s_task_lock);
    return ESP_OK;
}

esp_err_t audio_register_activity_callback(audio_activity_cb_t callback)
{
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_deinitializing) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_task_lock);
    s_activity_callback = callback;
    portEXIT_CRITICAL(&s_task_lock);
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}

esp_err_t audio_set_mode(audio_mode_t mode)
{
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (atomic_load_explicit(&s_running, memory_order_acquire)) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (mode != AUDIO_MODE_LOOPBACK && mode != AUDIO_MODE_MESH) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || s_stopping || s_deinitializing) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_config.mode = mode;
    reset_rx_source_metadata();
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}

audio_mode_t audio_get_mode(void)
{
    if (s_lifecycle_mutex == NULL) {
        return AUDIO_MODE_LOOPBACK;
    }
    if (called_from_audio_worker()) {
        return s_config.mode;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    audio_mode_t mode = s_config.mode;
    xSemaphoreGive(s_lifecycle_mutex);
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
    STATS_LOCK();
    s_tx_pipe_count++;
    s_tx_pipe_sum_us += latency_us;
    s_stats.tx_pipe_us_avg = (uint32_t)(s_tx_pipe_sum_us / s_tx_pipe_count);
    if (latency_us > s_stats.tx_pipe_us_max) {
        s_stats.tx_pipe_us_max = latency_us;
    }
    STATS_UNLOCK();
    return ESP_OK;
}

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
    for (size_t i = 0; i < AUDIO_FRAME_SAMPLES; ++i) {
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
        }
        int32_t tone = 0;
        if (!s_notification.in_gap) {
            float phase = 2.0f * M_PI *
                          notification_frequency(s_notification.type,
                                                 s_notification.tone_index) *
                          s_notification.segment_sample / s_config.sample_rate;
            tone = (int32_t)(NOTIFICATION_AMPLITUDE * 32767.0f * sinf(phase));
        }
        int32_t mixed = (int32_t)s_pcm_output[i] + tone;
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
        } else if (s_notification.tone_index + 1u <
                   notification_tone_count(s_notification.type)) {
            s_notification.in_gap = true;
        } else {
            s_notification.active = false;
        }
    }
}

esp_err_t audio_play_notification(audio_notify_t type)
{
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (called_from_audio_worker()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (type < AUDIO_NOTIFY_STARTUP || type > AUDIO_NOTIFY_MESH_DISABLED) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_stopping || s_deinitializing || s_notification_queue == NULL) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    audio_notification_request_t request = {.type = (uint8_t)type};
    if (xQueueSend(s_notification_queue, &request, 0) != pdTRUE) {
        STATS_LOCK();
        s_stats.notification_queue_overflows++;
        STATS_UNLOCK();
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}
