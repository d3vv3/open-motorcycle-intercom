#include "omi_board_pins.h"

#include "esp_log.h"
#include "led_strip.h"
#include "mesh_channel.h"

static const char *TAG = "board_indicator";
static led_strip_handle_t indicator;

esp_err_t board_set_channel_indicator(uint8_t channel)
{
    if (!mesh_channel_valid(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!indicator) {
        const led_strip_config_t strip_config = {
            .strip_gpio_num = OMI_BOARD_GPIO_RGB,
            .max_leds = 1,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags.invert_out = false,
        };
        const led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10000000,
            .flags.with_dma = false,
        };
        led_strip_handle_t strip = NULL;
        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RGB indicator init failed: %s", esp_err_to_name(err));
            if (strip) {
                esp_err_t cleanup_err = led_strip_del(strip);
                if (cleanup_err != ESP_OK) {
                    ESP_LOGE(TAG, "RGB indicator cleanup failed: %s", esp_err_to_name(cleanup_err));
                }
            }
            return err;
        }
        indicator = strip;
    }

    const uint32_t red = channel == 2 ? 16 : 0;
    const uint32_t green = channel == 1 ? 16 : 0;
    const uint32_t blue = channel == 3 ? 16 : 0;
    esp_err_t err = led_strip_set_pixel(indicator, 0, red, green, blue);
    if (err == ESP_OK) {
        err = led_strip_refresh(indicator);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB indicator write failed: %s", esp_err_to_name(err));
    }
    return err;
}
