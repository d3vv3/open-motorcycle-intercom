/**
 * @file omi_board_pins.h
 * @brief ESP32-S31 Function CoreBoard-1 GPIO map.
 */

#ifndef OMI_BOARD_PINS_H
#define OMI_BOARD_PINS_H

#include "sdkconfig.h"
#include "esp_err.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S31)
#error "omi_board_pins.h requires the ESP32-S31 target"
#endif

/* Onboard audio codec: codec is the I2S clock/data peer. */
#define OMI_BOARD_GPIO_AUDIO_I2C_SCL  50
#define OMI_BOARD_GPIO_AUDIO_I2C_SDA  51
#define OMI_BOARD_GPIO_AUDIO_I2S_MCLK 52
#define OMI_BOARD_GPIO_AUDIO_I2S_BCLK 53
#define OMI_BOARD_GPIO_AUDIO_I2S_DIN  54 /* codec microphone data -> ESP32 */
#define OMI_BOARD_GPIO_AUDIO_I2S_WS   55
#define OMI_BOARD_GPIO_AUDIO_I2S_DOUT 56 /* ESP32 -> codec playback data */
#define OMI_BOARD_GPIO_AUDIO_AMP_EN   57

/* Onboard controls. */
#define OMI_BOARD_GPIO_BOOT_BUTTON 61
#define OMI_BOARD_GPIO_RGB         60

/* nRF bridge on J2. */
#define OMI_BOARD_GPIO_NRF_ACK          42 /* nRF -> ESP32 ACK */
#define OMI_BOARD_GPIO_NRF_SPI_MISO     43 /* ESP32 slave output -> nRF master input */
#define OMI_BOARD_GPIO_NRF_SPI_MOSI     44 /* nRF master output -> ESP32 slave input */
#define OMI_BOARD_GPIO_NRF_SPI_SCLK     45 /* nRF master clock -> ESP32 slave */
#define OMI_BOARD_GPIO_NRF_SPI_CS       46 /* nRF master chip select -> ESP32 slave */
#define OMI_BOARD_GPIO_NRF_I2S_WS_SYNC  47 /* ESP32 I2S WS output -> nRF sync input */

/* Reserved for UART0; these pins are intentionally not assigned to consumers. */
#define OMI_BOARD_GPIO_UART0_RX 58
#define OMI_BOARD_GPIO_UART0_TX 59

/** Connect I2S0 master WS to the nRF bridge sync input GPIO. */
esp_err_t omi_board_nrf_i2s_ws_sync_connect(void);

/** Disconnect the nRF bridge sync input and reset it to a neutral input. */
esp_err_t omi_board_nrf_i2s_ws_sync_disconnect(void);

#endif /* OMI_BOARD_PINS_H */
