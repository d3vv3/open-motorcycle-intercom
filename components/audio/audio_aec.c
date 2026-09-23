#include "audio_aec.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "audio";
static bool s_failure_logged;

static void process_chunk(void *context, const int16_t *mic, const int16_t *ref, int16_t *out)
{
    audio_aec_t *aec = context;
    aec_process(aec->handle, (int16_t *)mic, (int16_t *)ref, out);
}

static void free_buffers(audio_aec_t *aec)
{
    heap_caps_free(aec->mic_fifo);
    heap_caps_free(aec->ref_fifo);
    heap_caps_free(aec->out_fifo);
    aec->mic_fifo = NULL;
    aec->ref_fifo = NULL;
    aec->out_fifo = NULL;
}

bool audio_aec_init(audio_aec_t *aec, bool enabled)
{
    if (aec == NULL) {
        return false;
    }
    audio_aec_deinit(aec);
    if (!enabled) {
        return false;
    }

    aec_config_t config = {
        .mic_num = 1,
        .ref_num = 1,
        .out_num = 1,
        .filter_length = 4,
        .sample_rate = 16000,
        .caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        .mode = AEC_MODE_VOIP_LOW_COST,
        .nlp_level = AEC_NLP_LEVEL_AGGR,
    };
    aec->handle = aec_create_from_config(&config);
    if (aec->handle == NULL) {
        goto failed;
    }
    int chunk_size = aec_get_chunksize(aec->handle);
    if (chunk_size <= 0 || (size_t)chunk_size > AUDIO_AEC_MAX_CHUNK_SIZE) {
        goto failed_handle;
    }

    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    aec->mic_fifo = heap_caps_aligned_alloc(16, AUDIO_AEC_FIFO_CAPACITY * sizeof(int16_t), caps);
    aec->ref_fifo = heap_caps_aligned_alloc(16, AUDIO_AEC_FIFO_CAPACITY * sizeof(int16_t), caps);
    aec->out_fifo = heap_caps_aligned_alloc(16, AUDIO_AEC_FIFO_CAPACITY * sizeof(int16_t), caps);
    if (aec->mic_fifo == NULL || aec->ref_fifo == NULL || aec->out_fifo == NULL ||
        !audio_reblock_init(&aec->reblock, aec->mic_fifo, aec->ref_fifo, aec->out_fifo,
                            AUDIO_AEC_FIFO_CAPACITY, (size_t)chunk_size)) {
        goto failed_handle;
    }
    aec->available = true;
    aec->active = true;
    return true;

failed_handle:
    aec_destroy(aec->handle);
    aec->handle = NULL;
    free_buffers(aec);
failed:
    if (!s_failure_logged) {
        ESP_LOGW(TAG, "ESP-SR AEC unavailable; using microphone noise suppression only");
        s_failure_logged = true;
    }
    return false;
}

void audio_aec_deinit(audio_aec_t *aec)
{
    if (aec == NULL) {
        return;
    }
    if (aec->handle != NULL) {
        aec_destroy(aec->handle);
    }
    free_buffers(aec);
    memset(aec, 0, sizeof(*aec));
}

bool audio_aec_process(audio_aec_t *aec, int16_t *mic, const int16_t *far_ref, size_t count)
{
    if (aec == NULL || !aec->active || mic == NULL || far_ref == NULL) {
        return false;
    }
    (void)audio_reblock_process_frame(&aec->reblock, mic, far_ref, mic, count, process_chunk,
                                      aec);
    return true;
}
