#include "esp_lc3_codec.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_audio_types.h"
#include "esp_attr.h"
#include "esp_lc3_dec.h"
#include "esp_lc3_enc.h"
#include "esp_log.h"
#include "esp_system.h"

#if defined(ESP_LC3_INTERNAL_BUFFERS)
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#if defined(ESP_LC3_SERIALIZE)
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#if defined(ESP_LC3_SINGLE_OWNER)
#if !defined(ESP_LC3_SERIALIZE)
#error "ESP_LC3_SINGLE_OWNER requires ESP_LC3_SERIALIZE"
#endif
#include "freertos/queue.h"
#include "freertos/task.h"
#endif

#define LC3_FRAME_SAMPLES 160u
#define LC3_FRAME_BYTES 24u
#define LC3_PCM_BYTES (LC3_FRAME_SAMPLES * sizeof(int16_t))
#define LC3_BREADCRUMB_MAGIC 0x4c433342u
#define LC3_BREADCRUMB_VERSION 1u

static const char *TAG = "esp_lc3_codec";

#if defined(ESP_LC3_INTERNAL_BUFFERS)
#if !defined(ESP_LC3_SERIALIZE)
#error "ESP_LC3_INTERNAL_BUFFERS requires ESP_LC3_SERIALIZE"
#endif
static DRAM_ATTR uint8_t s_decode_input[LC3_FRAME_BYTES] __attribute__((aligned(16)));
static DRAM_ATTR int16_t s_decode_output[LC3_FRAME_SAMPLES] __attribute__((aligned(16)));
static unsigned s_decode20_calls;

static void report_decode_buffers(unsigned call, const uint8_t *input0, const uint8_t *input1,
                                  int16_t *output0, int16_t *output1, bool plc)
{
    ESP_LOGI(TAG,
             "decode20 buffers call=%u plc=%d "
             "in0=%p mod16=%u internal=%d external=%d out0=%p mod16=%u internal=%d external=%d "
             "in1=%p mod16=%u internal=%d external=%d out1=%p mod16=%u internal=%d external=%d "
             "scratch_in=%p mod16=%u internal=%d external=%d "
             "scratch_out=%p mod16=%u internal=%d external=%d stack_hwm=%u",
             call, (int)plc,
             (const void *)input0, (unsigned)((uintptr_t)input0 % 16u),
             esp_ptr_internal(input0), esp_ptr_external_ram(input0),
             (void *)output0, (unsigned)((uintptr_t)output0 % 16u),
             esp_ptr_internal(output0), esp_ptr_external_ram(output0),
             (const void *)input1, (unsigned)((uintptr_t)input1 % 16u),
             esp_ptr_internal(input1), esp_ptr_external_ram(input1),
             (void *)output1, (unsigned)((uintptr_t)output1 % 16u),
             esp_ptr_internal(output1), esp_ptr_external_ram(output1),
             (void *)s_decode_input, (unsigned)((uintptr_t)s_decode_input % 16u),
             esp_ptr_internal(s_decode_input), esp_ptr_external_ram(s_decode_input),
             (void *)s_decode_output, (unsigned)((uintptr_t)s_decode_output % 16u),
             esp_ptr_internal(s_decode_output), esp_ptr_external_ram(s_decode_output),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
}
#endif

#if defined(ESP_LC3_SERIALIZE)
static StaticSemaphore_t s_vendor_mutex_storage;
static SemaphoreHandle_t s_vendor_mutex;
static portMUX_TYPE s_vendor_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_serialization_reported;

static bool vendor_lock(void)
{
    portENTER_CRITICAL(&s_vendor_mutex_init_lock);
    if (s_vendor_mutex == NULL)
        s_vendor_mutex = xSemaphoreCreateMutexStatic(&s_vendor_mutex_storage);
    SemaphoreHandle_t mutex = s_vendor_mutex;
    portEXIT_CRITICAL(&s_vendor_mutex_init_lock);
    if (mutex == NULL || xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) return false;
    if (!s_serialization_reported) {
        s_serialization_reported = true;
        ESP_LOGI(TAG, "LC3 vendor serialization enabled (shared priority-inheriting mutex)");
    }
    return true;
}

static void vendor_unlock(void)
{
    xSemaphoreGive(s_vendor_mutex);
}
#else
static inline bool vendor_lock(void) { return true; }
static inline void vendor_unlock(void) {}
#endif

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t encode_before_count;
    uint32_t encode_after_count;
    uint32_t decode_before_count;
    uint32_t decode_after_count;
    uint32_t encoder_reset_before_count;
    uint32_t encoder_reset_after_count;
    uint32_t decoder_reset_before_count;
    uint32_t decoder_reset_after_count;
    uint32_t last_decode_plc;
    uint8_t last_decode_packet[48];
    uint32_t last_decode_packet_valid;
} lc3_breadcrumbs_t;

static RTC_NOINIT_ATTR volatile lc3_breadcrumbs_t s_breadcrumbs;
static bool s_breadcrumbs_reported;

static inline void breadcrumb_barrier(void)
{
    __asm__ __volatile__("" ::: "memory");
}

static void report_and_initialize_breadcrumbs(void)
{
    if (s_breadcrumbs_reported) return;
    s_breadcrumbs_reported = true;

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    if (s_breadcrumbs.magic == LC3_BREADCRUMB_MAGIC &&
        s_breadcrumbs.version == LC3_BREADCRUMB_VERSION) {
        ESP_LOGI(TAG,
                 "previous breadcrumbs reset_reason=%d magic=0x%08" PRIx32
                 " version=%" PRIu32 " enc_before=%" PRIu32 " enc_after=%" PRIu32
                  " dec_before=%" PRIu32 " dec_after=%" PRIu32 " enc_reset_before=%" PRIu32
                  " enc_reset_after=%" PRIu32 " dec_reset_before=%" PRIu32
                  " dec_reset_after=%" PRIu32 " last_decode_plc=%" PRIu32,
                 (int)reset_reason, s_breadcrumbs.magic, s_breadcrumbs.version,
                 s_breadcrumbs.encode_before_count, s_breadcrumbs.encode_after_count,
                 s_breadcrumbs.decode_before_count, s_breadcrumbs.decode_after_count,
                  s_breadcrumbs.encoder_reset_before_count,
                  s_breadcrumbs.encoder_reset_after_count,
                   s_breadcrumbs.decoder_reset_before_count,
                   s_breadcrumbs.decoder_reset_after_count,
                   s_breadcrumbs.last_decode_plc);
        if (s_breadcrumbs.decode_before_count > s_breadcrumbs.decode_after_count &&
            s_breadcrumbs.last_decode_plc == 0u &&
            s_breadcrumbs.last_decode_packet_valid == 1u) {
            static const char hex[] = "0123456789abcdef";
            char packet_hex[49];
            for (size_t half = 0; half < 2u; ++half) {
                for (size_t i = 0; i < LC3_FRAME_BYTES; ++i) {
                    uint8_t byte = s_breadcrumbs.last_decode_packet[
                        half * LC3_FRAME_BYTES + i];
                    packet_hex[i * 2u] = hex[byte >> 4];
                    packet_hex[i * 2u + 1u] = hex[byte & 0x0fu];
                }
                packet_hex[sizeof(packet_hex) - 1u] = '\0';
                ESP_LOGI(TAG, "retained decode packet half=%u hex=%s",
                         (unsigned)(half + 1u), packet_hex);
            }
        }
    } else {
        ESP_LOGI(TAG,
                 "previous breadcrumbs invalid/unretained reset_reason=%d magic=0x%08" PRIx32
                 " version=%" PRIu32,
                 (int)reset_reason, s_breadcrumbs.magic, s_breadcrumbs.version);
    }

    s_breadcrumbs.magic = 0;
    s_breadcrumbs.version = LC3_BREADCRUMB_VERSION;
    s_breadcrumbs.encode_before_count = 0;
    s_breadcrumbs.encode_after_count = 0;
    s_breadcrumbs.decode_before_count = 0;
    s_breadcrumbs.decode_after_count = 0;
    s_breadcrumbs.encoder_reset_before_count = 0;
    s_breadcrumbs.encoder_reset_after_count = 0;
    s_breadcrumbs.decoder_reset_before_count = 0;
    s_breadcrumbs.decoder_reset_after_count = 0;
    s_breadcrumbs.last_decode_plc = 0;
    s_breadcrumbs.last_decode_packet_valid = 0;
    breadcrumb_barrier();
    s_breadcrumbs.magic = LC3_BREADCRUMB_MAGIC;
}

struct esp_lc3_codec {
    void *handle;
    bool encoder;
};

static int codec_open_handle(esp_lc3_codec_t *codec)
{
    if (codec->encoder) {
        const esp_lc3_enc_config_t config = {
            .sample_rate = 16000, .channel = 1, .bits_per_sample = 16,
            .frame_dms = 100, .nbyte = LC3_FRAME_BYTES, .len_prefixed = false,
        };
        return esp_lc3_enc_open((void *)&config, sizeof(config), &codec->handle) == ESP_AUDIO_ERR_OK
                   ? 0 : -1;
    }
    const esp_lc3_dec_cfg_t config = {
        .sample_rate = 16000, .channel = 1, .bits_per_sample = 16,
        .frame_dms = 100, .nbyte = LC3_FRAME_BYTES, .is_cbr = true,
        .len_prefixed = false, .enable_plc = true,
    };
    return esp_lc3_dec_open((void *)&config, sizeof(config), &codec->handle) == ESP_AUDIO_ERR_OK
               ? 0 : -1;
}

static esp_lc3_codec_t *codec_open_direct(bool encoder)
{
#if !defined(ESP_LC3_SERIALIZE)
    report_and_initialize_breadcrumbs();
#endif
    esp_lc3_codec_t *codec = calloc(1, sizeof(*codec));
    if (codec == NULL) return NULL;
    codec->encoder = encoder;
    if (!vendor_lock()) {
        free(codec);
        return NULL;
    }
#if defined(ESP_LC3_SERIALIZE)
    report_and_initialize_breadcrumbs();
#endif
    int result = codec_open_handle(codec);
    vendor_unlock();
    if (result != 0) {
        free(codec);
        return NULL;
    }
    return codec;
}

static void codec_close_direct(esp_lc3_codec_t **codec_ptr)
{
    if (codec_ptr == NULL || *codec_ptr == NULL) return;
    esp_lc3_codec_t *codec = *codec_ptr;
    if (!vendor_lock()) return;
    if (codec->handle != NULL) {
        if (codec->encoder) esp_lc3_enc_close(codec->handle);
        else (void)esp_lc3_dec_close(codec->handle);
    }
    vendor_unlock();
    free(codec);
    *codec_ptr = NULL;
}

static int codec_reset_direct(esp_lc3_codec_t *codec)
{
    if (codec == NULL) return -1;
    if (!vendor_lock()) return -1;
    if (codec->handle == NULL) {
        vendor_unlock();
        return -1;
    }
    if (codec->encoder) ++s_breadcrumbs.encoder_reset_before_count;
    else ++s_breadcrumbs.decoder_reset_before_count;
    breadcrumb_barrier();
    if (!codec->encoder) {
        if (esp_lc3_dec_reset(codec->handle) == ESP_AUDIO_ERR_OK) {
            ++s_breadcrumbs.decoder_reset_after_count;
            vendor_unlock();
            return 0;
        }
        (void)esp_lc3_dec_close(codec->handle);
        codec->handle = NULL;
        int result = codec_open_handle(codec);
        ++s_breadcrumbs.decoder_reset_after_count;
        vendor_unlock();
        return result;
    }
    esp_lc3_enc_close(codec->handle);
    codec->handle = NULL;
    int result = codec_open_handle(codec);
    ++s_breadcrumbs.encoder_reset_after_count;
    vendor_unlock();
    return result;
}

static int codec_encode20_direct(esp_lc3_codec_t *codec, const int16_t pcm[320], uint8_t packet[48])
{
    if (codec == NULL || pcm == NULL || packet == NULL) return -1;
    if (!vendor_lock()) return -1;
    if (codec->handle == NULL || !codec->encoder) {
        vendor_unlock();
        return -1;
    }
    int encoded = 48;
    for (size_t i = 0; i < 2u; ++i) {
        esp_audio_enc_in_frame_t input = {
            .buffer = (uint8_t *)(pcm + i * LC3_FRAME_SAMPLES), .len = LC3_PCM_BYTES,
        };
        esp_audio_enc_out_frame_t output = {
            .buffer = packet + i * LC3_FRAME_BYTES, .len = LC3_FRAME_BYTES,
        };
        ++s_breadcrumbs.encode_before_count;
        breadcrumb_barrier();
        const int result = esp_lc3_enc_process(codec->handle, &input, &output);
        ++s_breadcrumbs.encode_after_count;
        if (result != ESP_AUDIO_ERR_OK || output.encoded_bytes != LC3_FRAME_BYTES) {
            encoded = -1;
            break;
        }
    }
    vendor_unlock();
    return encoded;
}

static int codec_decode20_direct(esp_lc3_codec_t *codec, const uint8_t packet[48], bool plc,
                                 int16_t pcm[320])
{
    if (codec == NULL || (!plc && packet == NULL) || pcm == NULL) return -1;
    if (!vendor_lock()) return -1;
    if (codec->handle == NULL || codec->encoder) {
        vendor_unlock();
        return -1;
    }
    if (!plc) {
        s_breadcrumbs.last_decode_packet_valid = 0;
        breadcrumb_barrier();
        for (size_t i = 0; i < sizeof(s_breadcrumbs.last_decode_packet); ++i)
            s_breadcrumbs.last_decode_packet[i] = packet[i];
        breadcrumb_barrier();
        s_breadcrumbs.last_decode_packet_valid = 1u;
    }
    static const uint8_t plc_frame[LC3_FRAME_BYTES] = {0};
#if defined(ESP_LC3_INTERNAL_BUFFERS)
    if (s_decode20_calls < 5u) {
        report_decode_buffers(++s_decode20_calls, plc ? plc_frame : packet,
                              plc ? plc_frame : packet + LC3_FRAME_BYTES,
                              pcm, pcm + LC3_FRAME_SAMPLES, plc);
    }
#endif
    int decoded = 320;
    for (size_t i = 0; i < 2u; ++i) {
#if defined(ESP_LC3_INTERNAL_BUFFERS)
        memcpy(s_decode_input, plc ? plc_frame : packet + i * LC3_FRAME_BYTES,
               LC3_FRAME_BYTES);
#endif
        esp_audio_dec_in_raw_t input = {
#if defined(ESP_LC3_INTERNAL_BUFFERS)
            .buffer = s_decode_input,
#else
            .buffer = plc ? (uint8_t *)plc_frame : (uint8_t *)(packet + i * LC3_FRAME_BYTES),
#endif
            .len = LC3_FRAME_BYTES,
            .frame_recover = plc ? ESP_AUDIO_DEC_RECOVERY_PLC : ESP_AUDIO_DEC_RECOVERY_NONE,
        };
        esp_audio_dec_out_frame_t output = {
#if defined(ESP_LC3_INTERNAL_BUFFERS)
            .buffer = (uint8_t *)s_decode_output, .len = LC3_PCM_BYTES,
#else
            .buffer = (uint8_t *)(pcm + i * LC3_FRAME_SAMPLES), .len = LC3_PCM_BYTES,
#endif
        };
        esp_audio_dec_info_t info = {0};
        s_breadcrumbs.last_decode_plc = plc ? 1u : 0u;
        ++s_breadcrumbs.decode_before_count;
        breadcrumb_barrier();
        const int result = esp_lc3_dec_decode(codec->handle, &input, &output, &info);
        ++s_breadcrumbs.decode_after_count;
        if (result != ESP_AUDIO_ERR_OK || (!plc && input.consumed != LC3_FRAME_BYTES) ||
            output.decoded_size != LC3_PCM_BYTES) {
            decoded = -1;
            break;
        }
#if defined(ESP_LC3_INTERNAL_BUFFERS)
        memcpy(pcm + i * LC3_FRAME_SAMPLES, s_decode_output, LC3_PCM_BYTES);
#endif
    }
    vendor_unlock();
    return decoded;
}

#if defined(ESP_LC3_SINGLE_OWNER)
typedef enum {
    LC3_OP_OPEN,
    LC3_OP_CLOSE,
    LC3_OP_RESET,
    LC3_OP_ENCODE20,
    LC3_OP_DECODE20,
} lc3_op_t;

typedef struct {
    lc3_op_t op;
    union {
        struct { bool encoder; } open;
        struct { esp_lc3_codec_t **codec_ptr; } close;
        struct { esp_lc3_codec_t *codec; } reset;
        struct { esp_lc3_codec_t *codec; const int16_t *pcm; uint8_t *packet; } encode;
        struct { esp_lc3_codec_t *codec; const uint8_t *packet; bool plc; int16_t *pcm; } decode;
    } args;
    union { esp_lc3_codec_t *codec; int status; } result;
    StaticSemaphore_t completion_storage;
    SemaphoreHandle_t completion;
} lc3_request_t;

static StaticQueue_t s_worker_queue_storage;
static uint8_t s_worker_queue_items[4 * sizeof(lc3_request_t *)];
static QueueHandle_t s_worker_queue;
static TaskHandle_t s_worker_task;

static void codec_worker(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "LC3 single owner started core=%d priority=%u",
             xPortGetCoreID(), (unsigned)uxTaskPriorityGet(NULL));
    for (;;) {
        lc3_request_t *request;
        if (xQueueReceive(queue, &request, portMAX_DELAY) != pdTRUE) continue;
        switch (request->op) {
        case LC3_OP_OPEN:
            request->result.codec = codec_open_direct(request->args.open.encoder);
            break;
        case LC3_OP_CLOSE:
            codec_close_direct(request->args.close.codec_ptr);
            break;
        case LC3_OP_RESET:
            request->result.status = codec_reset_direct(request->args.reset.codec);
            break;
        case LC3_OP_ENCODE20:
            request->result.status = codec_encode20_direct(request->args.encode.codec,
                                                             request->args.encode.pcm,
                                                             request->args.encode.packet);
            break;
        case LC3_OP_DECODE20:
            request->result.status = codec_decode20_direct(request->args.decode.codec,
                                                             request->args.decode.packet,
                                                             request->args.decode.plc,
                                                             request->args.decode.pcm);
            break;
        }
        SemaphoreHandle_t completion = request->completion;
        xSemaphoreGive(completion);
        // The caller may release its stack request as soon as the semaphore is given.
    }
}

static bool codec_start_worker(void)
{
    if (!vendor_lock()) return false;
    if (s_worker_queue == NULL) {
        QueueHandle_t queue = xQueueCreateStatic(4, sizeof(lc3_request_t *),
                                                 s_worker_queue_items, &s_worker_queue_storage);
        if (queue != NULL) {
            if (xTaskCreatePinnedToCore(codec_worker, "lc3_owner", 24576, queue, 8,
                                        &s_worker_task, 1) == pdPASS) {
                s_worker_queue = queue;
            } else {
                vQueueDelete(queue);
                s_worker_task = NULL;
            }
        }
    }
    bool ready = s_worker_queue != NULL;
    vendor_unlock();
    return ready;
}

static bool codec_dispatch(lc3_request_t *request)
{
    if (!vendor_lock()) return false;
    QueueHandle_t queue = s_worker_queue;
    vendor_unlock();
    if (queue == NULL) return false;

    request->completion = xSemaphoreCreateBinaryStatic(&request->completion_storage);
    if (request->completion == NULL) return false;
    if (xQueueSend(queue, &request, portMAX_DELAY) != pdTRUE) {
        vSemaphoreDelete(request->completion);
        return false;
    }
    while (xSemaphoreTake(request->completion, portMAX_DELAY) != pdTRUE) {}
    vSemaphoreDelete(request->completion);
    return true;
}
#endif

esp_lc3_codec_t *esp_lc3_codec_open(bool encoder)
{
#if defined(ESP_LC3_SINGLE_OWNER)
    if (!codec_start_worker()) return NULL;
    lc3_request_t request = { .op = LC3_OP_OPEN, .args.open.encoder = encoder };
    if (!codec_dispatch(&request)) return NULL;
    return request.result.codec;
#else
    return codec_open_direct(encoder);
#endif
}

void esp_lc3_codec_close(esp_lc3_codec_t **codec_ptr)
{
#if defined(ESP_LC3_SINGLE_OWNER)
    if (codec_ptr == NULL || *codec_ptr == NULL) return;
    lc3_request_t request = { .op = LC3_OP_CLOSE, .args.close.codec_ptr = codec_ptr };
    (void)codec_dispatch(&request);
#else
    codec_close_direct(codec_ptr);
#endif
}

int esp_lc3_codec_reset(esp_lc3_codec_t *codec)
{
#if defined(ESP_LC3_SINGLE_OWNER)
    lc3_request_t request = { .op = LC3_OP_RESET, .args.reset.codec = codec };
    return codec_dispatch(&request) ? request.result.status : -1;
#else
    return codec_reset_direct(codec);
#endif
}

int esp_lc3_codec_encode20(esp_lc3_codec_t *codec, const int16_t pcm[320], uint8_t packet[48])
{
#if defined(ESP_LC3_SINGLE_OWNER)
    lc3_request_t request = {
        .op = LC3_OP_ENCODE20, .args.encode = { .codec = codec, .pcm = pcm, .packet = packet },
    };
    return codec_dispatch(&request) ? request.result.status : -1;
#else
    return codec_encode20_direct(codec, pcm, packet);
#endif
}

int esp_lc3_codec_decode20(esp_lc3_codec_t *codec, const uint8_t packet[48], bool plc,
                           int16_t pcm[320])
{
#if defined(ESP_LC3_SINGLE_OWNER)
    lc3_request_t request = {
        .op = LC3_OP_DECODE20,
        .args.decode = { .codec = codec, .packet = packet, .plc = plc, .pcm = pcm },
    };
    return codec_dispatch(&request) ? request.result.status : -1;
#else
    return codec_decode20_direct(codec, packet, plc, pcm);
#endif
}
