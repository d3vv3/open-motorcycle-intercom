/**
 * @file uart_bridge.c
 * @brief SPI Bridge to ESP32-S3 (nRF52840 = SPI Master)
 *
 * Protocol: [SYNC:0xAA][LEN:1B][TYPE:1B][PAYLOAD:0-128B]
 * Each call to uart_bridge_process() performs one full-duplex SPI
 * transaction at BRIDGE_SPI_MAX_XFER bytes.  The master sends its
 * queued packet (or an idle frame of zeros) and simultaneously
 * receives whatever the slave had prepared.
 *
 * "uart_bridge" naming retained for API compatibility.
 */

#include "uart_bridge.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mesh_protocol.h"

LOG_MODULE_REGISTER(uart_bridge, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SPI_MAX_PAYLOAD 128

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/* SPI bus device from devicetree */
#define SPI_BUS_NODE DT_NODELABEL(spi1)

static const struct device *s_spi_dev;
static struct spi_config s_spi_cfg;

/* Manual CS control — hardware CS is too fast for ESP32 SPI slave to detect */
#define CS_PORT_NODE DT_NODELABEL(gpio1)
#define CS_PIN       14
static const struct device *s_cs_port;

static uint8_t s_tx_buf[BRIDGE_SPI_MAX_XFER];
static uint8_t s_rx_buf[BRIDGE_SPI_MAX_XFER];

static bool s_initialized = false;

/* Pending TX packet length (0 = idle) */
static uint16_t s_tx_pending_len = 0;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Handle a fully received packet from the ESP32 slave.
 */
static void handle_rx_packet(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type) {
    case UART_PKT_AUDIO: {
        static int32_t last_log = 0;
        static uint32_t audio_pkt_count = 0;
        audio_pkt_count++;
        int32_t now = k_uptime_get();
        if (now - last_log > 5000) {
            LOG_INF("Audio: %u pkts received (last len=%d)", audio_pkt_count, len);
            last_log = now;
        }
        mesh_protocol_send_audio(payload, len);
        break;
    }

    case UART_PKT_COMMAND:
        if (len > 0) {
            uint8_t cmd_id = payload[0];
            LOG_INF("Received command: 0x%02X", cmd_id);
            if (cmd_id == 0x01) { /* Enable Mesh */
                /* Stop first if already running, then start fresh */
                if (mesh_protocol_get_state() != MESH_STATE_IDLE) {
                    LOG_INF("Mesh already running, restarting...");
                    mesh_protocol_stop();
                    k_sleep(K_MSEC(100)); /* Brief delay for clean shutdown */
                }
                mesh_protocol_start();
            } else if (cmd_id == 0x02) { /* Disable Mesh */
                mesh_protocol_stop();
            } else if (cmd_id == 0x03) { /* Ping / Get Status */
                printk("[uart_bridge] PING received, sending status\n");
                uart_bridge_send_status(mesh_protocol_get_role(), 0, 0);
            }
        }
        break;

    default:
        LOG_DBG("Unknown packet type from ESP32: 0x%02X", type);
        break;
    }
}

/**
 * @brief Parse the RX buffer after an SPI transaction.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    if (len < 3) {
        return;
    }

    /* Idle frame (no sync byte) - master sent nothing meaningful */
    if (buf[0] != UART_SYNC_BYTE) {
        return;
    }

    uint8_t pkt_len = buf[1];
    if (pkt_len < 1 || pkt_len > SPI_MAX_PAYLOAD + 1) {
        LOG_WRN("Bad packet length: %u", pkt_len);
        return;
    }

    if ((size_t)(pkt_len + 2) > len) {
        LOG_WRN("Truncated packet: need %u, got %u", pkt_len + 2, (unsigned int)len);
        return;
    }

    uint8_t pkt_type = buf[2];
    const uint8_t *payload = &buf[3];
    uint8_t payload_len = pkt_len - 1; /* -1 for type byte */

    handle_rx_packet(pkt_type, payload, payload_len);
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int uart_bridge_init(void)
{
    if (s_initialized) {
        return -EALREADY;
    }

    /* Get the SPI bus device from devicetree */
    s_spi_dev = DEVICE_DT_GET(SPI_BUS_NODE);
    if (!device_is_ready(s_spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* Configure manual CS GPIO (hardware CS is too fast for ESP32 slave) */
    s_cs_port = DEVICE_DT_GET(CS_PORT_NODE);
    if (!device_is_ready(s_cs_port)) {
        LOG_ERR("CS GPIO port not ready");
        return -ENODEV;
    }
    int cs_ret = gpio_pin_configure(s_cs_port, CS_PIN, GPIO_OUTPUT_HIGH);
    if (cs_ret) {
        LOG_ERR("CS pin config failed: %d", cs_ret);
        return cs_ret;
    }

    /* SPI master config: 4 MHz, CPOL=0/CPHA=0, MSB first, 8-bit words
     * cs field left as zero — no hardware CS, we control it manually */
    s_spi_cfg.frequency = 4000000U;
    s_spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    s_spi_cfg.slave = 0;

    s_initialized = true;
    printk("[uart_bridge] SPI bridge initialized (master mode, 4MHz)\n");
    LOG_INF("SPI bridge initialized");

    /* Send a test STATUS so ESP32 can detect us during probe */
    uart_bridge_send_status(mesh_protocol_get_role(), mesh_protocol_get_peer_count(),
                            mesh_protocol_get_node_id());

    return 0;
}

int uart_bridge_send_audio(uint8_t src_id, const uint8_t *data, uint8_t len)
{
    if (!s_initialized || data == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > SPI_MAX_PAYLOAD - 1) { /* -1 for src_id */
        return -EMSGSIZE;
    }

    /* Build packet: [SYNC][LEN][TYPE][SRC_ID][DATA...] */
    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = UART_SYNC_BYTE;
    s_tx_buf[1] = len + 2; /* type + src_id + audio data */
    s_tx_buf[2] = UART_PKT_AUDIO;
    s_tx_buf[3] = src_id;
    memcpy(&s_tx_buf[4], data, len);

    s_tx_pending_len = 4 + len;

    LOG_DBG("Queued audio packet: src=%d, len=%d", src_id, len);
    return 0;
}

int uart_bridge_send_event(uint8_t event_type, const uint8_t *data, uint8_t len)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = UART_SYNC_BYTE;
    s_tx_buf[1] = len + 2; /* type + event_type + data */
    s_tx_buf[2] = UART_PKT_EVENT;
    s_tx_buf[3] = event_type;

    if (len > 0 && data != NULL) {
        memcpy(&s_tx_buf[4], data, len);
    }

    s_tx_pending_len = 4 + len;

    LOG_DBG("Queued event: type=%d, len=%d", event_type, len);
    return 0;
}

int uart_bridge_send_status(uint8_t role, uint8_t peer_count, uint8_t node_id)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = UART_SYNC_BYTE;
    s_tx_buf[1] = 4; /* type + role + peer_count + node_id */
    s_tx_buf[2] = UART_PKT_STATUS;
    s_tx_buf[3] = role;
    s_tx_buf[4] = peer_count;
    s_tx_buf[5] = node_id;

    s_tx_pending_len = 6;

    LOG_DBG("Queued status: role=%d, peers=%d, id=%d", role, peer_count, node_id);
    return 0;
}

void uart_bridge_process(void)
{
    if (!s_initialized) {
        return;
    }

    static uint32_t txn_count = 0;

    /* If nothing queued to send, re-send STATUS so ESP32 can always detect us */
    if (s_tx_pending_len == 0) {
        uart_bridge_send_status(mesh_protocol_get_role(), mesh_protocol_get_peer_count(),
                                mesh_protocol_get_node_id());
    }

    memset(s_rx_buf, 0, BRIDGE_SPI_MAX_XFER);

    /* Set up SPI buffer descriptors */
    struct spi_buf tx_spi_buf = {
        .buf = s_tx_buf,
        .len = BRIDGE_SPI_MAX_XFER,
    };
    struct spi_buf_set tx_set = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };

    struct spi_buf rx_spi_buf = {
        .buf = s_rx_buf,
        .len = BRIDGE_SPI_MAX_XFER,
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_spi_buf,
        .count = 1,
    };

    /* Manual CS: assert, delay, transceive, delay, deassert.
     * Hardware CS is too fast for the ESP32 SPI slave to detect. */
    gpio_pin_set(s_cs_port, CS_PIN, 0); /* CS active low */
    k_busy_wait(100);                   /* 100µs setup time for ESP32 */

    int ret = spi_transceive(s_spi_dev, &s_spi_cfg, &tx_set, &rx_set);

    k_busy_wait(50);                    /* Hold CS a bit after transfer */
    gpio_pin_set(s_cs_port, CS_PIN, 1); /* Deassert CS */
    txn_count++;

    if (ret) {
        if (txn_count <= 10) {
            printk("[spi_bridge] txn #%u FAILED: %d\n", txn_count, ret);
        }
        return;
    }

    /* Debug: log first 3 transactions for boot verification */
    if (txn_count <= 3) {
        printk("[spi_bridge] txn #%u OK, rx: %02X %02X %02X %02X\n", txn_count, s_rx_buf[0],
               s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
    }

    /* TX was clocked out, clear pending */
    s_tx_pending_len = 0;

    /* Parse what the slave sent back */
    parse_rx(s_rx_buf, BRIDGE_SPI_MAX_XFER);
}

bool uart_bridge_is_initialized(void)
{
    return s_initialized;
}

int uart_bridge_send_log(const char *msg, uint8_t len)
{
    if (!s_initialized || msg == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > SPI_MAX_PAYLOAD) {
        len = SPI_MAX_PAYLOAD; /* Truncate if too long */
    }

    /* Build packet: [SYNC][LEN][TYPE][MSG...] */
    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = UART_SYNC_BYTE;
    s_tx_buf[1] = len + 1; /* type + msg */
    s_tx_buf[2] = UART_PKT_LOG;
    memcpy(&s_tx_buf[3], msg, len);

    s_tx_pending_len = 3 + len;

    return 0;
}
