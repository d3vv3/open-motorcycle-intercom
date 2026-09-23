/**
 * @file audio_hw.c
 * @brief ES8311, I2C, I2S, and Opus codec resource management.
 */

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "audio_internal.h"

static const char *TAG = "audio";

esp_err_t audio_hw_i2s_init(const audio_config_t *config)
{
    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    channel_config.dma_frame_num = I2S_DMA_BUFFER_SIZE;
    channel_config.auto_clear_after_cb = true;

    esp_err_t ret = i2s_new_channel(&channel_config, &g_audio.tx_chan, &g_audio.rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t standard_config = {
        /* Physical codec clock domain: voice processing remains 16 kHz. */
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_HW_SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = (gpio_num_t)config->i2s_pins.mclk_gpio,
                .bclk = (gpio_num_t)config->i2s_pins.bclk_gpio,
                .ws = (gpio_num_t)config->i2s_pins.ws_gpio,
                .dout = (gpio_num_t)config->i2s_pins.dout_gpio,
                .din = (gpio_num_t)config->i2s_pins.din_gpio,
                .invert_flags = {0},
            },
    };
    standard_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ret = i2s_channel_init_std_mode(g_audio.tx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S TX: %s", esp_err_to_name(ret));
        audio_hw_i2s_deinit();
        return ret;
    }
    ret = i2s_channel_init_std_mode(g_audio.rx_chan, &standard_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S RX: %s", esp_err_to_name(ret));
        audio_hw_i2s_deinit();
        return ret;
    }

    /* Mirror the I2S0 master WS to the nRF sync input; GPIO55 remains the
     * normal codec WS output managed by the I2S driver. */
    ret = omi_board_nrf_i2s_ws_sync_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect nRF I2S WS sync: %s", esp_err_to_name(ret));
        audio_hw_i2s_deinit();
        return ret;
    }
    g_audio.ws_sync_connected = true;
    ESP_LOGI(TAG, "I2S TX initialized on BCLK=%d WS=%d DOUT=%d", config->i2s_pins.bclk_gpio,
             config->i2s_pins.ws_gpio, config->i2s_pins.dout_gpio);
    return ESP_OK;
}

void audio_hw_i2s_deinit(void)
{
    if (g_audio.ws_sync_connected) {
        esp_err_t ret = omi_board_nrf_i2s_ws_sync_disconnect();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to disconnect nRF I2S WS sync: %s", esp_err_to_name(ret));
        }
        g_audio.ws_sync_connected = false;
    }
    if (g_audio.rx_chan != NULL) {
        i2s_del_channel(g_audio.rx_chan);
        g_audio.rx_chan = NULL;
    }
    if (g_audio.tx_chan != NULL) {
        i2s_del_channel(g_audio.tx_chan);
        g_audio.tx_chan = NULL;
    }
}

esp_err_t audio_hw_codec_init(const audio_config_t *config)
{
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OMI_BOARD_GPIO_AUDIO_I2C_SDA,
        .scl_io_num = OMI_BOARD_GPIO_AUDIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&i2c_cfg, &g_audio.i2c_bus);
    if (ret != ESP_OK) {
        return ret;
    }

    audio_codec_i2c_cfg_t ctrl_cfg = {
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = g_audio.i2c_bus,
    };
    g_audio.ctrl_if = audio_codec_new_i2c_ctrl(&ctrl_cfg);
    if (g_audio.ctrl_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    g_audio.gpio_if = audio_codec_new_gpio();
    if (g_audio.gpio_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    audio_codec_i2s_cfg_t tx_data_cfg = {
        .port = I2S_NUM_0,
        .tx_handle = g_audio.tx_chan,
    };
    g_audio.tx_data_if = audio_codec_new_i2s_data(&tx_data_cfg);
    if (g_audio.tx_data_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    audio_codec_i2s_cfg_t rx_data_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = g_audio.rx_chan,
    };
    g_audio.rx_data_if = audio_codec_new_i2s_data(&rx_data_cfg);
    if (g_audio.rx_data_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = g_audio.ctrl_if,
        .gpio_if = g_audio.gpio_if,
        .sys_cfg = {.is_master = false, .no_mclk = false},
        .adc_cfg = {.digital_mic = false, .label = "FC"},
        .pa_cfg = {.pa_pin = OMI_BOARD_GPIO_AUDIO_AMP_EN, .pa_active_low = false},
    };
    g_audio.codec_if = es8311_codec_new(&es8311_cfg);
    if (g_audio.codec_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    esp_codec_dev_cfg_t play_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = g_audio.codec_if,
        .data_if = g_audio.tx_data_if,
    };
    g_audio.play_dev = esp_codec_dev_new(&play_cfg);
    if (g_audio.play_dev == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    esp_codec_dev_cfg_t record_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = g_audio.codec_if,
        .data_if = g_audio.rx_data_if,
    };
    g_audio.record_dev = esp_codec_dev_new(&record_cfg);
    if (g_audio.record_dev == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    return ESP_OK;

fail:
    audio_hw_codec_deinit();
    audio_hw_i2c_deinit();
    return ret;
}

esp_err_t audio_hw_codec_start(const audio_config_t *config)
{
    int ret;
    if (g_audio.record_open || g_audio.play_open) {
        audio_hw_codec_stop();
    }
    esp_codec_dev_sample_info_t sample_info = {
        /* Both ES8311 directions share the 48 kHz physical I2S clock. */
        .sample_rate = AUDIO_HW_SAMPLE_RATE,
        .channel = config->channels,
        .channel_mask = 1,
        .bits_per_sample = config->bits_per_sample,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };
    ret = esp_codec_dev_open(g_audio.record_dev, &sample_info);
    if (ret != ESP_CODEC_DEV_OK) {
        audio_hw_codec_stop();
        return ret;
    }
    g_audio.record_open = true;
    ret = esp_codec_dev_open(g_audio.play_dev, &sample_info);
    if (ret != ESP_CODEC_DEV_OK) {
        audio_hw_codec_stop();
        return ret;
    }
    g_audio.play_open = true;
    ret = esp_codec_dev_set_out_vol(g_audio.play_dev, 60);
    if (ret != ESP_CODEC_DEV_OK) {
        audio_hw_codec_stop();
        return ret;
    }
    ret = esp_codec_dev_set_in_gain(g_audio.record_dev, 30.0f);
    if (ret != ESP_CODEC_DEV_OK) {
        audio_hw_codec_stop();
        return ret;
    }
    return ESP_OK;
}

void audio_hw_codec_stop(void)
{
    if (g_audio.record_dev != NULL && g_audio.record_open) {
        esp_codec_dev_close(g_audio.record_dev);
        g_audio.record_open = false;
    }
    if (g_audio.play_dev != NULL && g_audio.play_open) {
        esp_codec_dev_close(g_audio.play_dev);
        g_audio.play_open = false;
    }
}

void audio_hw_codec_deinit(void)
{
    audio_hw_codec_stop();
    if (g_audio.record_dev != NULL) {
        esp_codec_dev_delete(g_audio.record_dev);
        g_audio.record_dev = NULL;
    }
    if (g_audio.play_dev != NULL) {
        esp_codec_dev_delete(g_audio.play_dev);
        g_audio.play_dev = NULL;
    }
    if (g_audio.codec_if != NULL) {
        audio_codec_delete_codec_if(g_audio.codec_if);
        g_audio.codec_if = NULL;
    }
    if (g_audio.rx_data_if != NULL) {
        audio_codec_delete_data_if(g_audio.rx_data_if);
        g_audio.rx_data_if = NULL;
    }
    if (g_audio.tx_data_if != NULL) {
        audio_codec_delete_data_if(g_audio.tx_data_if);
        g_audio.tx_data_if = NULL;
    }
    if (g_audio.ctrl_if != NULL) {
        audio_codec_delete_ctrl_if(g_audio.ctrl_if);
        g_audio.ctrl_if = NULL;
    }
    if (g_audio.gpio_if != NULL) {
        audio_codec_delete_gpio_if(g_audio.gpio_if);
        g_audio.gpio_if = NULL;
    }
}

void audio_hw_i2c_deinit(void)
{
    if (g_audio.i2c_bus != NULL) {
        i2c_del_master_bus(g_audio.i2c_bus);
        g_audio.i2c_bus = NULL;
    }
}

static void free_opus_decoder(OpusDecoder **decoder)
{
    if (*decoder != NULL) {
        heap_caps_free(*decoder);
        *decoder = NULL;
    }
}

esp_err_t audio_hw_opus_init(const audio_config_t *config)
{
    const size_t decoder_count = AUDIO_MAX_RX_SOURCES + 1u;
    const int decoder_size = opus_decoder_get_size(config->channels);
    int error = OPUS_OK;

    audio_hw_opus_deinit();
    if (decoder_size <= 0) {
        ESP_LOGE(TAG, "Invalid Opus decoder size for %u channels", config->channels);
        return ESP_ERR_INVALID_ARG;
    }

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

    g_audio.loopback_decoder = heap_caps_malloc((size_t)decoder_size,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (g_audio.loopback_decoder == NULL) {
        ESP_LOGE(TAG, "Failed to allocate loopback decoder state (%d bytes)", decoder_size);
        audio_hw_opus_deinit();
        return ESP_ERR_NO_MEM;
    }
    error = opus_decoder_init(g_audio.loopback_decoder, config->sample_rate, config->channels);
    if (error != OPUS_OK) {
        ESP_LOGE(TAG, "Failed to initialize loopback decoder: %s", opus_strerror(error));
        audio_hw_opus_deinit();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        g_audio.rx_sources[i].decoder = heap_caps_malloc(
            (size_t)decoder_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (g_audio.rx_sources[i].decoder == NULL) {
            ESP_LOGE(TAG, "Failed to allocate source decoder %zu state (%d bytes)", i,
                     decoder_size);
            audio_hw_opus_deinit();
            return ESP_ERR_NO_MEM;
        }
        error = opus_decoder_init(g_audio.rx_sources[i].decoder, config->sample_rate,
                                   config->channels);
        if (error != OPUS_OK) {
            ESP_LOGE(TAG, "Failed to initialize source decoder %zu: %s", i,
                     opus_strerror(error));
            audio_hw_opus_deinit();
            return ESP_FAIL;
        }
    }
    const size_t total_decoder_bytes = decoder_count * (size_t)decoder_size;
    ESP_LOGI(TAG, "Opus decoders=%u bytes_each=%u PSRAM_total=%u internal_free=%u largest=%u",
             (unsigned)decoder_count, (unsigned)decoder_size, (unsigned)total_decoder_bytes,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return ESP_OK;
}

void audio_hw_opus_deinit(void)
{
    if (g_audio.opus_encoder != NULL) {
        opus_encoder_destroy(g_audio.opus_encoder);
        g_audio.opus_encoder = NULL;
    }
    free_opus_decoder(&g_audio.loopback_decoder);
    for (size_t i = 0; i < AUDIO_MAX_RX_SOURCES; ++i) {
        free_opus_decoder(&g_audio.rx_sources[i].decoder);
    }
}
