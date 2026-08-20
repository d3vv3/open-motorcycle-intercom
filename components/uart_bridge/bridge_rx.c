/**
 * @file bridge_rx.c
 * @brief Validation and dispatch of frames received from the nRF master.
 */

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "bridge_internal.h"

static const char *TAG = "spi_bridge";

static void handle_rx_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    switch (type) {
    case BRIDGE_PKT_AUDIO:
    case BRIDGE_PKT_AUDIO_V2:
        if (g_bridge.audio_cb && len > 1) {
            /* nRF prefixes both audio versions with src_id; preserve the bundle itself. */
            uint8_t src_id = payload[0];
            g_bridge.audio_rx_count++;
            g_bridge.audio_cb(src_id, payload + 1, len - 1, esp_timer_get_time(),
                              type == BRIDGE_PKT_AUDIO_V2);
        }
        break;

    case BRIDGE_PKT_STATUS:
        if (len == sizeof(bridge_status_payload_t)) {
            bridge_status_payload_t status;
            memcpy(&status, payload, sizeof(status));
            bridge_status_apply_v2(&status);
        } else if (len == 3) {
            bridge_status_apply_legacy(payload);
        } else {
            ESP_LOGW(TAG, "Ignoring bridge status with invalid length %u", len);
        }
        break;

    case BRIDGE_PKT_MESH_EVENT:
        if (len > 0 && payload[0] == BRIDGE_EVENT_COMMAND_ACK &&
            len >= 1 + sizeof(bridge_command_ack_payload_t)) {
            bridge_command_ack_payload_t ack;
            memcpy(&ack, payload + 1, sizeof(ack));
            bridge_commands_note_ack(&ack);
        }
        if (g_bridge.event_cb && len > 0) {
            uart_bridge_event_t event = (uart_bridge_event_t)payload[0];
            g_bridge.event_cb(event, payload + 1, len - 1);
        }
        break;

    case BRIDGE_PKT_LOG:
        if (len > 0) {
            /* Forward the nRF log line, null-terminated */
            char log_msg[129];
            uint16_t copy_len = (len < sizeof(log_msg) - 1) ? len : sizeof(log_msg) - 1;
            memcpy(log_msg, payload, copy_len);
            log_msg[copy_len] = '\0';
            ESP_LOGI("nRF52", "%s", log_msg);
        }
        break;

    default:
        ESP_LOGD(TAG, "Unknown packet type: 0x%02X", type);
        break;
    }
}

/**
 * @brief Parse a received SPI buffer for a framed packet.
 *
 * A buffer starting with 0x00 is an idle frame from the master and is
 * silently ignored.
 */
void bridge_rx_parse(const uint8_t *buf, size_t len)
{
    bridge_frame_view_t frame;
    bridge_frame_decode_result_t result =
        bridge_frame_decode(buf, len, BRIDGE_SPI_MAX_PAYLOAD, &frame);

    if (result == BRIDGE_FRAME_IDLE) {
        return;
    }
    if (result == BRIDGE_FRAME_TRUNCATED) {
        g_bridge.rx_trunc++;
        if (len >= BRIDGE_FRAME_OVERHEAD) {
            ESP_LOGW(TAG, "Truncated packet: need %u, got %u", buf[1] + 3u, (unsigned)len);
        }
        return;
    }
    if (result == BRIDGE_FRAME_BAD_SYNC) {
        g_bridge.rx_bad_sync++;
        return;
    }
    if (result == BRIDGE_FRAME_BAD_LENGTH) {
        g_bridge.rx_bad_len++;
        ESP_LOGW(TAG, "Bad packet length: %u", len >= 2u ? buf[1] : 0u);
        return;
    }
    if (result == BRIDGE_FRAME_BAD_CRC) {
        uint8_t pkt_len = buf[1];
        uint8_t rx_crc = buf[2u + pkt_len];
        uint8_t calc_crc = bridge_frame_crc8(buf + 1u, (size_t)pkt_len + 1u);

        g_bridge.rx_crc_fail++;
        if ((g_bridge.rx_crc_fail % 50) == 1) {
            ESP_LOGW(TAG, "CRC mismatch: rx=0x%02X calc=0x%02X fail=%lu", rx_crc, calc_crc,
                     g_bridge.rx_crc_fail);
        }
        return;
    }

    if (!g_bridge.rx_seq_init) {
        g_bridge.rx_seq_init = true;
    } else if (frame.seq != (uint8_t)(g_bridge.rx_expected + 1)) {
        g_bridge.rx_seq_gaps++;
    }
    g_bridge.rx_expected = frame.seq;

    handle_rx_packet(frame.type, frame.payload, (uint16_t)frame.payload_len);
}
