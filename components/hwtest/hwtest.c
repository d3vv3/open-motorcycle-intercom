/**
 * @file hwtest.c
 * @brief Hardware validation tests for OMI
 *
 * Simple tests to validate speaker (I2S) and microphone (ADC) hardware.
 */

#include "hwtest.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2s_std.h"

static const char *TAG = "hwtest";

/* Pin definitions (match wiring.md) */
#define SPEAKER_BCLK_GPIO 4
#define SPEAKER_WS_GPIO   5
#define SPEAKER_DOUT_GPIO 7
#define MIC_ADC_CHANNEL   ADC_CHANNEL_0 /* GPIO1 */

/* Audio parameters */
#define SAMPLE_RATE    16000
#define TONE_FREQ_HZ   1000
#define TONE_AMPLITUDE 16000 /* ~50% of int16_t max */

/* ============================================================================
 * Speaker Test - Generate 1kHz tone
 * ============================================================================ */

esp_err_t hwtest_speaker(void)
{
    ESP_LOGI(TAG, "=== SPEAKER TEST ===");
    ESP_LOGI(TAG, "Playing 1kHz tone for 2 seconds...");
    ESP_LOGI(TAG, "I2S pins: BCLK=%d, WS=%d, DOUT=%d", SPEAKER_BCLK_GPIO, SPEAKER_WS_GPIO,
             SPEAKER_DOUT_GPIO);

    esp_err_t ret;
    i2s_chan_handle_t tx_chan = NULL;

    /* Create I2S TX channel */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 256;

    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure I2S standard mode */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = SPEAKER_BCLK_GPIO,
                .ws = SPEAKER_WS_GPIO,
                .dout = SPEAKER_DOUT_GPIO,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
            },
    };

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S: %s", esp_err_to_name(ret));
        i2s_del_channel(tx_chan);
        return ret;
    }

    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S: %s", esp_err_to_name(ret));
        i2s_del_channel(tx_chan);
        return ret;
    }

    ESP_LOGI(TAG, "I2S initialized, generating tone...");

    /* Generate and play sine wave */
    const int samples_per_cycle = SAMPLE_RATE / TONE_FREQ_HZ;
    const int total_samples = SAMPLE_RATE * 2; /* 2 seconds */
    int16_t buffer[256];
    size_t bytes_written;

    for (int i = 0; i < total_samples; i += 256) {
        for (int j = 0; j < 256 && (i + j) < total_samples; j++) {
            float phase = 2.0f * M_PI * (float)((i + j) % samples_per_cycle) / samples_per_cycle;
            buffer[j] = (int16_t)(TONE_AMPLITUDE * sinf(phase));
        }

        i2s_channel_write(tx_chan, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
    }

    ESP_LOGI(TAG, "Tone complete, cleaning up...");

    /* Cleanup */
    i2s_channel_disable(tx_chan);
    i2s_del_channel(tx_chan);

    ESP_LOGI(TAG, "=== SPEAKER TEST DONE ===");
    ESP_LOGI(TAG, "Did you hear a 1kHz tone? If yes, speaker is working!");

    return ESP_OK;
}

/* ============================================================================
 * Microphone Test - Read ADC levels
 * ============================================================================ */

esp_err_t hwtest_mic(void)
{
    ESP_LOGI(TAG, "=== MICROPHONE TEST ===");
    ESP_LOGI(TAG, "Reading ADC levels for 5 seconds...");
    ESP_LOGI(TAG, "ADC channel: %d (GPIO1)", MIC_ADC_CHANNEL);
    ESP_LOGI(TAG, "Speak into the mic to see level changes!");

    esp_err_t ret;
    adc_oneshot_unit_handle_t adc_handle;

    /* Initialize ADC */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    ret = adc_oneshot_new_unit(&init_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    ret = adc_oneshot_config_channel(adc_handle, MIC_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc_handle);
        return ret;
    }

    ESP_LOGI(TAG, "ADC initialized, reading samples...");
    ESP_LOGI(TAG, "");

    /* Read ADC for 5 seconds */
    int64_t start_time = esp_timer_get_time();
    int64_t end_time = start_time + (5 * 1000000); /* 5 seconds */

    int min_val = 4095, max_val = 0;
    int sample_count = 0;
    int64_t sum = 0;

    while (esp_timer_get_time() < end_time) {
        /* Take 100 samples quickly */
        int batch_min = 4095, batch_max = 0;
        int64_t batch_sum = 0;

        for (int i = 0; i < 100; i++) {
            int raw;
            ret = adc_oneshot_read(adc_handle, MIC_ADC_CHANNEL, &raw);
            if (ret == ESP_OK) {
                batch_sum += raw;
                if (raw < batch_min)
                    batch_min = raw;
                if (raw > batch_max)
                    batch_max = raw;
                sample_count++;
            }
        }

        int batch_avg = batch_sum / 100;
        int batch_range = batch_max - batch_min;

        /* Update overall stats */
        sum += batch_sum;
        if (batch_min < min_val)
            min_val = batch_min;
        if (batch_max > max_val)
            max_val = batch_max;

        /* Visual level meter */
        int level = batch_range / 50; /* Scale to 0-80 roughly */
        if (level > 40)
            level = 40;

        char meter[42];
        memset(meter, ' ', 41);
        meter[41] = '\0';
        for (int i = 0; i < level; i++) {
            meter[i] = '#';
        }

        ESP_LOGI(TAG, "ADC: avg=%4d range=%4d [%s]", batch_avg, batch_range, meter);

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Final stats */
    int avg_val = (sample_count > 0) ? (sum / sample_count) : 0;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== MICROPHONE TEST DONE ===");
    ESP_LOGI(TAG, "Samples: %d", sample_count);
    ESP_LOGI(TAG, "Min: %d, Max: %d, Avg: %d", min_val, max_val, avg_val);
    ESP_LOGI(TAG, "Range: %d (should vary when speaking)", max_val - min_val);
    ESP_LOGI(TAG, "");

    if (max_val - min_val < 50) {
        ESP_LOGW(TAG, "Low variance detected - check mic wiring!");
        ESP_LOGW(TAG, "Expected: Bias ~2048, Range >100 when speaking");
    } else if (avg_val < 1500 || avg_val > 2500) {
        ESP_LOGW(TAG, "DC bias seems off (expected ~2048) - check bias resistors");
    } else {
        ESP_LOGI(TAG, "Microphone appears to be working!");
    }

    adc_oneshot_del_unit(adc_handle);

    return ESP_OK;
}

/* ============================================================================
 * Loopback Test - Mic to Speaker passthrough
 * ============================================================================ */

esp_err_t hwtest_loopback(uint32_t duration_sec)
{
    ESP_LOGI(TAG, "=== LOOPBACK TEST ===");
    ESP_LOGI(TAG, "Mic -> Speaker passthrough for %lu seconds", duration_sec);
    ESP_LOGI(TAG, "(duration=0 means run until reset)");

    esp_err_t ret;
    i2s_chan_handle_t tx_chan = NULL;
    adc_oneshot_unit_handle_t adc_handle = NULL;

    /* Initialize ADC for mic */
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {.unit_id = ADC_UNIT_1};
    ret = adc_oneshot_new_unit(&adc_init_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc_handle, MIC_ADC_CHANNEL, &adc_chan_cfg);

    /* Initialize I2S for speaker */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 256;

    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc_handle);
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = SPEAKER_BCLK_GPIO,
                .ws = SPEAKER_WS_GPIO,
                .dout = SPEAKER_DOUT_GPIO,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
            },
    };

    i2s_channel_init_std_mode(tx_chan, &std_cfg);
    i2s_channel_enable(tx_chan);

    ESP_LOGI(TAG, "Loopback running - speak into mic!");

    int64_t start_time = esp_timer_get_time();
    int64_t end_time =
        (duration_sec > 0) ? start_time + ((int64_t)duration_sec * 1000000) : INT64_MAX;

    int16_t mono_buffer[64];
    int16_t stereo_buffer[128]; /* Stereo output for I2S */
    size_t bytes_written;

    while (esp_timer_get_time() < end_time) {
        /* Read samples from ADC */
        for (int i = 0; i < 64; i++) {
            int raw;
            adc_oneshot_read(adc_handle, MIC_ADC_CHANNEL, &raw);
            /* Convert 12-bit unsigned to 16-bit signed, centered - gain (8x) */
            mono_buffer[i] = (int16_t)((raw - 2048) * 8);
        }

        /* Convert to stereo */
        for (int i = 0; i < 64; i++) {
            stereo_buffer[i * 2] = mono_buffer[i];     /* Left */
            stereo_buffer[i * 2 + 1] = mono_buffer[i]; /* Right */
        }

        /* Write to speaker */
        i2s_channel_write(tx_chan, stereo_buffer, sizeof(stereo_buffer), &bytes_written,
                          portMAX_DELAY);
    }

    /* Cleanup */
    i2s_channel_disable(tx_chan);
    i2s_del_channel(tx_chan);
    adc_oneshot_del_unit(adc_handle);

    ESP_LOGI(TAG, "=== LOOPBACK TEST DONE ===");

    return ESP_OK;
}
