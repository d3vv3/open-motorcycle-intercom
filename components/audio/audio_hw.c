/**
 * @file audio_hw.c
 * @brief ADC, I2S, and Opus codec resource management.
 */

#include "audio_internal.h"

#include "esp_log.h"
#include "hal/adc_types.h"
#include "soc/soc.h"

static const char *TAG = "audio";

static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata, void *user_data)
{
    (void)handle;
    (void)edata;
    (void)user_data;
    BaseType_t must_yield = pdFALSE;
    TaskHandle_t task = atomic_load_explicit(&g_audio.adc_notify_task, memory_order_relaxed);
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &must_yield);
    }
    return must_yield == pdTRUE;
}

esp_err_t audio_hw_adc_init(const audio_config_t *config)
{
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = ADC_CONV_FRAME_SIZE * 2,
        .conv_frame_size = ADC_CONV_FRAME_SIZE,
    };
    esp_err_t ret = adc_continuous_new_handle(&handle_config, &g_audio.adc_handle);
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
    ret = adc_continuous_config(g_audio.adc_handle, &config_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %s", esp_err_to_name(ret));
        adc_continuous_deinit(g_audio.adc_handle);
        g_audio.adc_handle = NULL;
        return ret;
    }

    adc_continuous_evt_cbs_t callbacks = {.on_conv_done = adc_conv_done_cb};
    ret = adc_continuous_register_event_callbacks(g_audio.adc_handle, &callbacks, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ADC callbacks: %s", esp_err_to_name(ret));
        adc_continuous_deinit(g_audio.adc_handle);
        g_audio.adc_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "ADC continuous mode initialized at %lu Hz", config->sample_rate);
    return ESP_OK;
}

void audio_hw_adc_deinit(void)
{
    if (g_audio.adc_handle != NULL) {
        adc_continuous_deinit(g_audio.adc_handle);
        g_audio.adc_handle = NULL;
    }
}

esp_err_t audio_hw_i2s_init(const audio_config_t *config)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    channel_config.dma_frame_num = I2S_DMA_BUFFER_SIZE;
    channel_config.auto_clear_after_cb = true;

    esp_err_t ret = i2s_new_channel(&channel_config, &g_audio.tx_chan, NULL);
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
    ret = i2s_channel_init_std_mode(g_audio.tx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S TX: %s", esp_err_to_name(ret));
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "I2S TX initialized on BCLK=%d WS=%d DOUT=%d",
             config->i2s_pins.bclk_gpio, config->i2s_pins.ws_gpio,
             config->i2s_pins.dout_gpio);
    return ESP_OK;
}

void audio_hw_i2s_deinit(void)
{
    if (g_audio.tx_chan != NULL) {
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
    }
}

esp_err_t audio_hw_opus_init(const audio_config_t *config)
{
    int error = OPUS_OK;
    g_audio.opus_encoder =
        opus_encoder_create(config->sample_rate, config->channels, OPUS_APPLICATION_VOIP, &error);
    if (error != OPUS_OK || g_audio.opus_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(error));
        return ESP_FAIL;
    }
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_BITRATE(config->opus_bitrate));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_PACKET_LOSS_PERC(OPUS_EXPECTED_LOSS_PERC));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_COMPLEXITY(5));
    opus_encoder_ctl(g_audio.opus_encoder, OPUS_SET_DTX(1));

    g_audio.loopback_decoder =
        opus_decoder_create(config->sample_rate, config->channels, &error);
    if (error != OPUS_OK || g_audio.loopback_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to create loopback decoder: %s", opus_strerror(error));
        opus_encoder_destroy(g_audio.opus_encoder);
        g_audio.opus_encoder = NULL;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        g_audio.rx_sources[i].decoder =
            opus_decoder_create(config->sample_rate, config->channels, &error);
        if (error != OPUS_OK || g_audio.rx_sources[i].decoder == NULL) {
            ESP_LOGE(TAG, "Failed to create source decoder %zu: %s", i, opus_strerror(error));
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "Opus encoder and %u independent decoders initialized",
             AUDIO_MAX_RX_SOURCES + 1);
    return ESP_OK;
}

void audio_hw_opus_deinit(void)
{
    if (g_audio.opus_encoder != NULL) {
        opus_encoder_destroy(g_audio.opus_encoder);
        g_audio.opus_encoder = NULL;
    }
    if (g_audio.loopback_decoder != NULL) {
        opus_decoder_destroy(g_audio.loopback_decoder);
        g_audio.loopback_decoder = NULL;
    }
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        if (g_audio.rx_sources[i].decoder != NULL) {
            opus_decoder_destroy(g_audio.rx_sources[i].decoder);
            g_audio.rx_sources[i].decoder = NULL;
        }
    }
}
