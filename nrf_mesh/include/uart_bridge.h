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

#include "bridge_protocol_defs.h"

#define UART_PKT_AUDIO    BRIDGE_PKT_AUDIO
#define UART_PKT_AUDIO_V2 BRIDGE_PKT_AUDIO_V2
#define UART_PKT_STATUS   BRIDGE_PKT_STATUS
#define UART_PKT_EVENT    BRIDGE_PKT_MESH_EVENT
#define UART_PKT_COMMAND  BRIDGE_PKT_CONTROL
#define UART_PKT_LOG      BRIDGE_PKT_LOG

/* Packet sync byte */
#define UART_SYNC_BYTE 0xAA

/* SPI master poll interval (ms) */
#define BRIDGE_SPI_POLL_INTERVAL_MS 5

/* Max transfer size - must match ESP32 slave */
#define BRIDGE_SPI_MAX_XFER 256

/* Hardware ACK line: nRF XIAO D10 (P1.15) -> ESP32 XIAO D1 (GPIO2) */
#define BRIDGE_ACK_PORT 1
#define BRIDGE_ACK_PIN  15

/**
 * @brief Initialize SPI bridge (master mode)
 * @return 0 on success
 */
int uart_bridge_init(void);

/**
 * @brief Send a legacy RTT probe to ESP32
 * @param src_id Source node ID
 * @param data RTT probe data
 * @param len Data length
 * @return 0 on success
 */
int uart_bridge_send_audio(uint8_t src_id, const uint8_t *data, uint8_t len);

/**
 * @brief Send an intact V2 audio bundle to ESP32 with its source ID prefix
 */
int uart_bridge_send_audio_v2(uint8_t src_id, const uint8_t *data, uint8_t len);

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
int uart_bridge_send_status(uint8_t state, uint8_t role, uint8_t peer_count, uint8_t node_id,
                            int8_t slot_index, uint8_t coordinator_id);

int uart_bridge_send_command_ack(uint8_t command, uint8_t generation, int result);

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

/**
 * @brief Send debug log message to ESP32
 * @param msg Log message string
 * @param len Length of message (max 250 bytes)
 * @return 0 on success
 */
int uart_bridge_send_log(const char *msg, uint8_t len);

#endif /* OMI_UART_BRIDGE_H */
