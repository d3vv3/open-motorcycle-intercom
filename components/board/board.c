/**
 * @file board.c
 * @brief Function CoreBoard-1 board-specific GPIO routing.
 */

#include "omi_board_pins.h"

#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"

esp_err_t omi_board_nrf_i2s_ws_sync_connect(void)
{
    esp_rom_gpio_pad_select_gpio(OMI_BOARD_GPIO_NRF_I2S_WS_SYNC);
    esp_rom_gpio_connect_out_signal(OMI_BOARD_GPIO_NRF_I2S_WS_SYNC,
                                    I2S0_O_WS_PAD_OUT_IDX, false, false);
    return ESP_OK;
}

esp_err_t omi_board_nrf_i2s_ws_sync_disconnect(void)
{
    return gpio_reset_pin(OMI_BOARD_GPIO_NRF_I2S_WS_SYNC);
}
