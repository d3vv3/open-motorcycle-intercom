/**
 * @file audio_internal.h
 * @brief Private state and interfaces shared by the audio modules.
 *
 * Module ownership:
 * - audio.c: lifecycle, configuration, public API guards, stats snapshots.
 * - audio_hw.c: ES8311, I2C, I2S, and Opus codec resources.
 * - audio_capture.c: capture task, DSP chain, encode, TX callback.
 * - audio_playout.c: playout task, decode, mixing, and I2S writes.
 * - audio_rx.c: RX packet admission and source reset handshakes.
 * - audio_notify.c: notification tone synthesis.
 *
 * NOTE: API lock order is lifecycle -> RX reset -> RX sources -> short
 * portMUX locks. Task code never holds a portMUX lock while taking a mutex.
 */

#ifndef AUDIO_INTERNAL_H
#define AUDIO_INTERNAL_H

#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

#include "audio.h"
#include "audio_packet_store.h"
#include "audio_pcm_resampler.h"
#include "audio_route.h"
#include "audio_aec.h"
#include "audio_capture_fifo.h"
#include "audio_rate_converter.h"
#include "audio_sample_fifo.h"
#include "opus.h"
#include "voice_cleanup.h"
#include "vox.h"

#define AUDIO_CAPTURE_TASK_STACK_SIZE 28672
#define AUDIO_PLAYOUT_TASK_STACK_SIZE 8192
#define AUDIO_CAPTURE_TASK_PRIORITY   7
#define AUDIO_PLAYOUT_TASK_PRIORITY   8
#define AUDIO_TASK_CORE               1

#define AUDIO_ENABLE_AEC_NS         1
/* Trial only: ESP-SR took 122 ms per 20 ms capture frame on S31, so it is not
 * qualified for the 50 fps capture deadline. Keep scalar voice cleanup active. */
#define AUDIO_ENABLE_ESP_SR_AEC      0

/* Each DMA descriptor holds one 20 ms hardware frame at 48 kHz. */
#define I2S_DMA_BUFFER_COUNT 2
#define I2S_DMA_BUFFER_SIZE  960

#define AUDIO_FRAME_SAMPLES 320
#define AUDIO_HW_SAMPLE_RATE 48000
#define AUDIO_HW_FRAME_SAMPLES 960
#define AUDIO_CAPTURE_FIFO_CAPACITY 640
#define AUDIO_PLAYBACK_CONVERTED_CAPACITY (AUDIO_HW_FRAME_SAMPLES * 2u)
#define AUDIO_PLAYBACK_FIFO_CAPACITY      (AUDIO_HW_FRAME_SAMPLES * 2u)
#define AUDIO_FAR_FIFO_CAPACITY           (AUDIO_FRAME_SAMPLES * 2u)

#define MAX_OPUS_PACKET_SIZE      64
#define OPUS_DTX_FRAME_MAX_BYTES  2
#define OPUS_EXPECTED_LOSS_PERC   5
#define RX_SOURCE_IDLE_TIMEOUT_MS 1000
/* RX_SOURCE_EVICT_SILENCE_MS lives in audio_rx_source_select.h. */
/* Runtime route callbacks wait at most this long while holding route_mutex. */
#define AUDIO_ROUTE_LOCK_WAIT_MS 1
#define RX_ENQUEUE_LOCK_WAIT_MS 1

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
    atomic_bool call_priority_active;
    atomic_bool playout_ready;
    atomic_bool capture_ready;
    atomic_bool rx_reset_requested;
    atomic_bool voice_playback_reset_requested;
    atomic_uint music_playback_generation;
    audio_config_t config;
    SemaphoreHandle_t lifecycle_mutex;
    StaticSemaphore_t lifecycle_mutex_storage;

    /* Statistics */
    audio_stats_t stats;
    uint64_t tx_pipe_sum_us;
    uint32_t tx_pipe_count;
    uint64_t rx_pipe_sum_us;
    uint32_t rx_pipe_count;
    uint64_t bluetooth_music_conversion_us_sum;
    uint32_t bluetooth_music_conversion_frames;
    uint64_t playout_work_us_sum;
    uint32_t playout_work_frames;
    uint64_t capture_read_us_sum, capture_convert_us_sum, capture_aec_us_sum;
    uint64_t capture_loop_us_sum, playout_write_us_sum;

    /* Hardware */
    i2s_chan_handle_t tx_chan;
    i2s_chan_handle_t rx_chan;
    i2c_master_bus_handle_t i2c_bus;
    const audio_codec_data_if_t *tx_data_if;
    const audio_codec_data_if_t *rx_data_if;
    const audio_codec_ctrl_if_t *ctrl_if;
    const audio_codec_gpio_if_t *gpio_if;
    const audio_codec_if_t *codec_if;
    esp_codec_dev_handle_t play_dev;
    esp_codec_dev_handle_t record_dev;
    bool play_open;
    bool record_open;
    bool ws_sync_connected;

    /* Tasks and handshakes */
    TaskHandle_t capture_task;
    TaskHandle_t playout_task;
    SemaphoreHandle_t playout_started;
    SemaphoreHandle_t capture_started;
    SemaphoreHandle_t capture_done;
    SemaphoreHandle_t playout_done;
    SemaphoreHandle_t rx_reset_done;
    SemaphoreHandle_t rx_reset_mutex;
    SemaphoreHandle_t rx_sources_mutex;
    SemaphoreHandle_t route_mutex;
    SemaphoreHandle_t music_playback_mutex;

    /* Codecs and DSP */
    OpusEncoder *opus_encoder;
    OpusDecoder *loopback_decoder;
    audio_hpf_state_t hpf;
    vox_state_t vox;
    voice_cleanup_state_t voice_cleanup;
    audio_aec_t aec;
    audio_rate_converter_t *capture_rate_converter;
    audio_rate_converter_t *voice_playback_converter;
    audio_rate_converter_t *far_reference_converter;
    audio_rate_converter_t *music_playback_converter;
    audio_capture_fifo_t capture_fifo;
    int16_t capture_fifo_storage[AUDIO_CAPTURE_FIFO_CAPACITY];
    /* Frame buffers */
    int16_t capture_hw_frame[AUDIO_HW_FRAME_SAMPLES];
    int16_t pcm_input[AUDIO_FRAME_SAMPLES];
    uint8_t opus_buffer[MAX_OPUS_PACKET_SIZE];
    int16_t pcm_output[AUDIO_FRAME_SAMPLES];
    int16_t far_ref_frame[AUDIO_FRAME_SAMPLES];
    int16_t far_ref_shadows[I2S_DMA_BUFFER_COUNT][AUDIO_FRAME_SAMPLES];
    size_t far_ref_shadow_head;
    int16_t hw_output[AUDIO_HW_FRAME_SAMPLES];
    /* Software-only playback staging is allocated from PSRAM during init. */
    int16_t *playback_psram_storage;
    int16_t *voice_converted;
    int16_t *voice_fifo_storage;
    audio_sample_fifo_t voice_fifo;
    int16_t *voice_presence_fifo_storage;
    int16_t *voice_presence_staging;
    int16_t *voice_presence_frame;
    audio_sample_fifo_t voice_presence_fifo;
    int16_t *music_input;
    int16_t *music_converted;
    int16_t *music_fifo_storage;
    audio_sample_fifo_t music_fifo;
    int16_t *far_converted;
    int16_t *far_pending_frame;
    int16_t *far_fifo_storage;
    audio_sample_fifo_t far_fifo;
    int32_t mix_frame[AUDIO_FRAME_SAMPLES];

    /* RX sources and queues */
    audio_rx_source_t rx_sources[AUDIO_MAX_RX_SOURCES];
    QueueHandle_t loopback_queue;
    QueueHandle_t notification_queue;
    audio_notification_state_t notification;

    /* Bluetooth route state. These queues are fixed storage and are reset at lifecycle edges. */
    audio_route_stream_t bluetooth_music;
    audio_route_stream_t bluetooth_call;
    audio_route_stream_t bluetooth_mic;
    uint32_t bluetooth_mic_rate;

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
SemaphoreHandle_t audio_lifecycle_mutex_get(void);
bool audio_route_lock_acquire(TickType_t wait_ticks);
void audio_route_lock_release(void);
audio_stats_t audio_stats_snapshot(void);
bool audio_called_from_worker(void);

/* audio_hw.c */
esp_err_t audio_hw_i2s_init(const audio_config_t *config);
void audio_hw_i2s_deinit(void);
esp_err_t audio_hw_codec_init(const audio_config_t *config);
void audio_hw_codec_deinit(void);
esp_err_t audio_hw_codec_start(const audio_config_t *config);
void audio_hw_codec_stop(void);
void audio_hw_i2c_deinit(void);
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
size_t audio_notify_mix_frame(size_t base_present_samples);

/* audio_route.c */
bool audio_route_init(void);
void audio_route_reset_streams(void);
void audio_route_capture_frame(const int16_t *samples, size_t count);
bool audio_route_render_call_frame(void);
void audio_route_mix_music_48k(size_t voice_present_samples);

#endif /* AUDIO_INTERNAL_H */
