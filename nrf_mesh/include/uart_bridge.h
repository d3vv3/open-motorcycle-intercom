/**
 * @file uart_bridge.h
 * @brief SPI Bridge to ESP32-S3 (nRF52840 = SPI Master)
 *
 * "uart_bridge" naming retained for API compatibility with
 * existing callers (main.c, mesh_protocol.c).
 */

#ifndef OMI_UART_BRIDGE_H
#define OMI_UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

/* Packet types (unchanged from UART protocol) */
#define UART_PKT_AUDIO   0x01 /* Audio Data (Bidirectional) */
#define UART_PKT_STATUS  0x02 /* Mesh status -> ESP32 */
#define UART_PKT_EVENT   0x03 /* Mesh event -> ESP32 */
#define UART_PKT_COMMAND 0x04 /* Command ESP32 -> nRF */

/* Packet sync byte */
#define UART_SYNC_BYTE 0xAA

/* SPI master poll interval (ms) */
#define BRIDGE_SPI_POLL_INTERVAL_MS 5

/* Max transfer size - must match ESP32 slave */
#define BRIDGE_SPI_MAX_XFER 256

/**
 * @brief Initialize SPI bridge (master mode)
 * @return 0 on success
 */
int uart_bridge_init(void);

/**
 * @brief Send audio packet to ESP32
 * @param src_id Source node ID
 * @param data Opus audio data
 * @param len Data length
 * @return 0 on success
 */
int uart_bridge_send_audio(uint8_t src_id, const uint8_t *data, uint8_t len);

/**
 * @brief Send mesh event to ESP32
 * @param event_type Event type
 * @param data Optional event data
 * @param len Data length
 * @return 0 on success
 */
int uart_bridge_send_event(uint8_t event_type, const uint8_t *data, uint8_t len);

/**
 * @brief Send mesh status to ESP32
 * @param role Current role
 * @param peer_count Number of peers
 * @param node_id Local node ID
 */
int uart_bridge_send_status(uint8_t role, uint8_t peer_count, uint8_t node_id);

/**
 * @brief Process incoming SPI data (call from main loop)
 *
 * Performs one full-duplex SPI transaction: sends any queued TX packet
 * while simultaneously receiving from the ESP32 slave.
 */
void uart_bridge_process(void);

/**
 * @brief Check if SPI bridge is initialized
 * @return true if initialized
 */
bool uart_bridge_is_initialized(void);

#endif /* OMI_UART_BRIDGE_H */
