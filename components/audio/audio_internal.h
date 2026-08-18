/**
 * @file audio_internal.h
 * @brief Private state and interfaces shared by the audio modules.
 *
 * Module ownership:
 * - audio.c: lifecycle, configuration, public API guards, stats snapshots.
 * - audio_hw.c: ADC, I2S, and Opus codec resources.
 * - audio_capture.c: capture task, DSP chain, encode, TX callback.
 * - audio_playout.c: playout task, decode, mixing, I2S writes, heartbeat.
 * - audio_rx.c: RX packet admission and source reset handshakes.
 * - audio_notify.c: notification tone synthesis.
 *
 * NOTE: API lock order is lifecycle -> RX reset -> RX sources -> short
 * portMUX locks. Task code never holds a portMUX lock while taking a mutex.
 */

#ifndef AUDIO_INTERNAL_H
#define AUDIO_INTERNAL_H

#include "audio.h"
#include "audio_packet_store.h"
#include "audio_pcm_resampler.h"
#include "voice_cleanup.h"
#include "vox.h"

#include <stdatomic.h>

#include "driver/i2s_std.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "opus.h"

#define AUDIO_CAPTURE_TASK_STACK_SIZE 32768
#define AUDIO_PLAYOUT_TASK_STACK_SIZE 32768
#define AUDIO_CAPTURE_TASK_PRIORITY   7
#define AUDIO_PLAYOUT_TASK_PRIORITY   8
#define AUDIO_TASK_CORE               1

#define AUDIO_HEARTBEAT_INTERVAL_MS 10000
#define AUDIO_ENABLE_AEC_NS         1

/* Two 20 ms descriptors: preload both, then each steady write waits for availability. */
#define I2S_DMA_BUFFER_COUNT 2
#define I2S_DMA_BUFFER_SIZE  320
#define I2S_WRITE_TIMEOUT_MS 50

#define AUDIO_FRAME_SAMPLES   320
#define ADC_OVERSAMPLE_FACTOR 4
#define ADC_CONV_FRAME_SIZE   (AUDIO_FRAME_SAMPLES * ADC_OVERSAMPLE_FACTOR * 4)
#define ADC_READ_TIMEOUT_MS   100

#define MAX_OPUS_PACKET_SIZE      64
#define OPUS_DTX_FRAME_MAX_BYTES  2
#define OPUS_EXPECTED_LOSS_PERC   5
#define RX_SOURCE_IDLE_TIMEOUT_MS 1000
/* RX_SOURCE_EVICT_SILENCE_MS lives in audio_rx_source_select.h. */
#define RX_ENQUEUE_LOCK_WAIT_MS   1

#define LOOPBACK_QUEUE_SIZE       8
#define NOTIFICATION_QUEUE_SIZE   4
#define NOTIFICATION_BEEP_SAMPLES 1600
#define NOTIFICATION_GAP_SAMPLES  400
#define NOTIFICATION_AMPLITUDE    0.3f

typedef struct {
    float x1, x2;
    float y1, y2;
    float b0, b1, b2;
    float a1, a2;
} audio_hpf_state_t;

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

/*
 * NOTE: keep this struct free of nonzero initializers so it stays in .bss;
 * the spinlocks live outside because their initializer is a nonzero pattern.
 */
typedef struct {
    /* Lifecycle */
    bool initialized;
    bool stopping;
    bool deinitializing;
    atomic_bool running;
    atomic_bool playout_ready;
    atomic_bool capture_ready;
    atomic_bool rx_reset_requested;
    audio_config_t config;
    SemaphoreHandle_t lifecycle_mutex;
    StaticSemaphore_t lifecycle_mutex_storage;

    /* Statistics */
    audio_stats_t stats;
    uint64_t tx_pipe_sum_us;
    uint32_t tx_pipe_count;
    uint64_t rx_pipe_sum_us;
    uint32_t rx_pipe_count;

    /* Hardware */
    i2s_chan_handle_t tx_chan;
    adc_continuous_handle_t adc_handle;

    /* Tasks and handshakes */
    TaskHandle_t capture_task;
    TaskHandle_t playout_task;
    _Atomic(TaskHandle_t) adc_notify_task;
    SemaphoreHandle_t playout_started;
    SemaphoreHandle_t capture_started;
    SemaphoreHandle_t capture_done;
    SemaphoreHandle_t playout_done;
    SemaphoreHandle_t rx_reset_done;
    SemaphoreHandle_t rx_reset_mutex;
    SemaphoreHandle_t rx_sources_mutex;

    /* Codecs and DSP */
    OpusEncoder *opus_encoder;
    OpusDecoder *loopback_decoder;
    audio_hpf_state_t hpf;
    vox_state_t vox;
    voice_cleanup_state_t voice_cleanup;
    float dc_estimate;
    int16_t lpf_prev;

    /* Frame buffers */
    int16_t pcm_input[AUDIO_FRAME_SAMPLES];
    uint8_t opus_buffer[MAX_OPUS_PACKET_SIZE];
    int16_t pcm_output[AUDIO_FRAME_SAMPLES];
    int16_t pcm_stereo[AUDIO_FRAME_SAMPLES * 2];
    int16_t i2s_silence[AUDIO_FRAME_SAMPLES * 2];
    int16_t far_ref_frame[AUDIO_FRAME_SAMPLES];
    int16_t far_ref_shadows[I2S_DMA_BUFFER_COUNT][AUDIO_FRAME_SAMPLES];
    size_t far_ref_shadow_head;
    int32_t mix_frame[AUDIO_FRAME_SAMPLES];

    /* RX sources and queues */
    audio_rx_source_t rx_sources[AUDIO_MAX_RX_SOURCES];
    QueueHandle_t loopback_queue;
    QueueHandle_t notification_queue;
    audio_notification_state_t notification;

    /* Callbacks */
    audio_tx_cb_t tx_callback;
    audio_activity_cb_t activity_callback;
} audio_context_t;

extern audio_context_t g_audio;
extern portMUX_TYPE g_audio_stats_lock;
extern portMUX_TYPE g_audio_task_lock;
extern portMUX_TYPE g_audio_far_ref_lock;

#define AUDIO_STATS_LOCK()   portENTER_CRITICAL(&g_audio_stats_lock)
#define AUDIO_STATS_UNLOCK() portEXIT_CRITICAL(&g_audio_stats_lock)

/* audio.c */
audio_stats_t audio_stats_snapshot(void);
bool audio_called_from_worker(void);

/* audio_hw.c */
esp_err_t audio_hw_adc_init(const audio_config_t *config);
void audio_hw_adc_deinit(void);
esp_err_t audio_hw_i2s_init(const audio_config_t *config);
void audio_hw_i2s_deinit(void);
esp_err_t audio_hw_opus_init(const audio_config_t *config);
void audio_hw_opus_deinit(void);

/* audio_capture.c */
void audio_capture_task(void *arg);
void audio_capture_init_dsp(void);

/* audio_playout.c */
void audio_playout_task(void *arg);
void audio_playout_reset_far_reference(void);

/* audio_rx.c */
void audio_rx_reset_source_metadata(void);
void audio_rx_reset_source_metadata_locked(void);
void audio_rx_reset_codecs_and_resamplers(void);
void audio_rx_service_reset_request(void);

/* audio_notify.c */
void audio_notify_mix_frame(void);

#endif /* AUDIO_INTERNAL_H */
