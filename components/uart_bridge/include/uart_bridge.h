/**
 * @file uart_bridge.h
 * @brief SPI Bridge to nRF52840 Mesh Radio
 *
 * Handles communication between ESP32-S3 (audio processor, SPI slave) and
 * nRF52840 (ESB mesh radio, SPI master) via SPI.
 *
 * The file retains the "uart_bridge" naming for API compatibility with
 * existing callers (main.c, mesh_protocol.c).
 */

#ifndef OMI_UART_BRIDGE_H
#define OMI_UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* SPI slave pins (directly wired to nRF52840 SPI master) */
#define BRIDGE_SPI_MISO_PIN 9  /* ESP32 GPIO9  -> nRF MOSI (ESP sends) */
#define BRIDGE_SPI_MOSI_PIN 10 /* ESP32 GPIO10 <- nRF MOSI (ESP receives) */
#define BRIDGE_SPI_SCLK_PIN 11 /* ESP32 GPIO11 <- nRF SCK */
#define BRIDGE_SPI_CS_PIN   12 /* ESP32 GPIO12 <- nRF CS */

#define BRIDGE_SPI_MODE     0 /* CPOL=0, CPHA=0 */
#define BRIDGE_SPI_MAX_XFER 256
#define BRIDGE_SPI_DMA_CHAN SPI_DMA_CH_AUTO
#define BRIDGE_SPI_HOST     SPI2_HOST

/* Packet types matching nRF52840 bridge protocol (unchanged) */
#define BRIDGE_PKT_AUDIO      0x01
#define BRIDGE_PKT_STATUS     0x02
#define BRIDGE_PKT_MESH_EVENT 0x03
#define BRIDGE_PKT_CONTROL    0x04

/* Mesh events received from nRF52840 */
typedef enum {
    BRIDGE_EVENT_MESH_READY = 0x01,
    BRIDGE_EVENT_PEER_JOINED = 0x02,
    BRIDGE_EVENT_PEER_LEFT = 0x03,
    BRIDGE_EVENT_BECAME_COORDINATOR = 0x04,
    BRIDGE_EVENT_SYNC_LOST = 0x05,
} uart_bridge_event_t;

/* Status info from nRF52840 */
typedef struct {
    uint8_t node_id;
    uint8_t slot_index;
    uint8_t peer_count;
    bool is_coordinator;
    int8_t rssi;
} uart_bridge_status_t;

/* Callback for received audio packets */
typedef void (*uart_bridge_audio_cb_t)(uint8_t src_id, const uint8_t *data, uint16_t len,
                                       int64_t timestamp_us);

/* Callback for mesh events */
typedef void (*uart_bridge_event_cb_t)(uart_bridge_event_t event, const uint8_t *data,
                                       uint16_t len);

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Initialize SPI bridge to nRF52840
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_init(void);

/**
 * @brief Deinitialize SPI bridge
 */
void uart_bridge_deinit(void);

/**
 * @brief Send audio packet to mesh via nRF52840
 * @param data Opus encoded audio data
 * @param len Length of data
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_send_audio(const uint8_t *data, uint16_t len);

/**
 * @brief Register callback for received audio
 * @param cb Callback function
 */
void uart_bridge_set_audio_callback(uart_bridge_audio_cb_t cb);

/**
 * @brief Register callback for mesh events
 * @param cb Callback function
 */
void uart_bridge_set_event_callback(uart_bridge_event_cb_t cb);

/**
 * @brief Get current mesh status
 * @param status Output status structure
 * @return ESP_OK if status available
 */
esp_err_t uart_bridge_get_status(uart_bridge_status_t *status);

/**
 * @brief Check if nRF52840 is connected
 * @return true if connected, false otherwise
 */
bool uart_bridge_is_connected(void);

/**
 * @brief Probe for nRF52840 connection
 *
 * Queues a PING command. The nRF52840 SPI master polls periodically;
 * when it clocks out the PING it replies with a STATUS packet.
 *
 * @param timeout_ms Timeout in milliseconds
 * @return true if detected, false otherwise
 */
bool uart_bridge_probe(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* OMI_UART_BRIDGE_H */
