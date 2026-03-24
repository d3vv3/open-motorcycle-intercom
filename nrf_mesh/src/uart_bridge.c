/**
 * @file uart_bridge.c
 * @brief SPI Bridge to ESP32-S3 (nRF52840 = SPI Master)
 *
 * Protocol: [SYNC:0xAA][LEN:1B][SEQ:1B][TYPE:1B][PAYLOAD:0-128B][CRC8:1B]
 * LEN covers: [SEQ][TYPE][PAYLOAD]
 * CRC8 covers: [LEN][SEQ][TYPE][PAYLOAD]
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
#define TX_AUDIO_QUEUE_SIZE 16

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

/* Outbound TX queues to ESP32 (audio prioritized over control).
 * Audio: lock-free SPSC ring (producer = mesh RX callback, consumer = SPI poll).
 * Control: mutex-protected FIFO (low frequency path). */
struct tx_entry {
    uint8_t buf[BRIDGE_SPI_MAX_XFER];
    uint16_t len;
};

static struct tx_entry s_audio_q[TX_AUDIO_QUEUE_SIZE];
static volatile uint8_t s_audio_head = 0;
static volatile uint8_t s_audio_tail = 0;

/* Control packet FIFO (replaces single-slot buffer to prevent event loss) */
#define TX_CTRL_QUEUE_SIZE 4
static struct tx_entry s_ctrl_q[TX_CTRL_QUEUE_SIZE];
static uint8_t s_ctrl_head = 0;
static uint8_t s_ctrl_tail = 0;

static struct k_mutex s_tx_lock;   /* Protects control FIFO only */
static uint32_t s_audio_q_overwrite = 0;

/* SPI poll interval jitter diagnostics */
static int64_t s_last_poll_us = 0;
static uint32_t s_poll_dt_count = 0;
static uint64_t s_poll_dt_sum_us = 0;
static uint32_t s_poll_dt_min_us = 0;
static uint32_t s_poll_dt_max_us = 0;
static uint8_t s_bridge_tx_seq = 0;
static uint8_t s_bridge_rx_expected = 0;
static bool s_bridge_rx_seq_init = false;
static uint32_t s_bridge_rx_seq_gap = 0;
static uint32_t s_bridge_rx_crc_fail = 0;

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

static uint8_t crc8_compute(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Parse the RX buffer after an SPI transaction.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    if (len < 5) {
        return;
    }

    /* Idle frame (no sync byte) - master sent nothing meaningful */
    if (buf[0] != UART_SYNC_BYTE) {
        return;
    }

    uint8_t pkt_len = buf[1];
    if (pkt_len < 2 || pkt_len > SPI_MAX_PAYLOAD + 2) {
        LOG_WRN("Bad packet length: %u", pkt_len);
        return;
    }

    if ((size_t)(pkt_len + 3) > len) {
        LOG_WRN("Truncated packet: need %u, got %u", pkt_len + 3, (unsigned int)len);
        return;
    }

    uint8_t rx_crc = buf[2 + pkt_len];
    uint8_t calc_crc = crc8_compute(&buf[1], (size_t)(pkt_len + 1));
    if (rx_crc != calc_crc) {
        s_bridge_rx_crc_fail++;
        if ((s_bridge_rx_crc_fail % 50) == 1) {
            LOG_WRN("CRC mismatch rx=0x%02X calc=0x%02X fail=%u", rx_crc, calc_crc,
                    s_bridge_rx_crc_fail);
        }
        return;
    }

    uint8_t seq = buf[2];
    if (!s_bridge_rx_seq_init) {
        s_bridge_rx_expected = seq;
        s_bridge_rx_seq_init = true;
    } else {
        uint8_t expected_next = (uint8_t)(s_bridge_rx_expected + 1);
        if (seq != expected_next) {
            s_bridge_rx_seq_gap++;
        }
    }
    s_bridge_rx_expected = seq;

    uint8_t pkt_type = buf[3];
    const uint8_t *payload = &buf[4];
    uint8_t payload_len = pkt_len - 2; /* -2 for seq + type bytes */

    handle_rx_packet(pkt_type, payload, payload_len);
}

static uint16_t build_packet(uint8_t *dst, uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t wire_len = (uint8_t)(2 + len);
    dst[0] = UART_SYNC_BYTE;
    dst[1] = wire_len;
    dst[2] = s_bridge_tx_seq++;
    dst[3] = type;
    if (len > 0 && payload != NULL) {
        memcpy(&dst[4], payload, len);
    }
    dst[2 + wire_len] = crc8_compute(&dst[1], (size_t)(wire_len + 1));
    return (uint16_t)(3 + wire_len);
}

static void queue_control_packet(const uint8_t *buf, uint16_t len)
{
    k_mutex_lock(&s_tx_lock, K_FOREVER);
    uint8_t next_head = (s_ctrl_head + 1) % TX_CTRL_QUEUE_SIZE;
    if (next_head == s_ctrl_tail) {
        /* Full: drop oldest to make room for the new event */
        s_ctrl_tail = (s_ctrl_tail + 1) % TX_CTRL_QUEUE_SIZE;
    }
    struct tx_entry *e = &s_ctrl_q[s_ctrl_head];
    memcpy(e->buf, buf, len);
    e->len = len;
    s_ctrl_head = next_head;
    k_mutex_unlock(&s_tx_lock);
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
    k_mutex_init(&s_tx_lock);
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

    /* Lock-free SPSC enqueue (single producer = mesh RX callback) */
    uint8_t cur_head = s_audio_head;
    uint8_t next_head = (cur_head + 1) % TX_AUDIO_QUEUE_SIZE;
    if (next_head == s_audio_tail) {
        /* Full: drop oldest to keep low latency */
        s_audio_tail = (s_audio_tail + 1) % TX_AUDIO_QUEUE_SIZE;
        s_audio_q_overwrite++;
        if ((s_audio_q_overwrite % 200) == 0) {
            LOG_WRN("Audio SPI TX queue overwrite count=%u", s_audio_q_overwrite);
        }
    }

    struct tx_entry *e = &s_audio_q[cur_head];
    memset(e->buf, 0, BRIDGE_SPI_MAX_XFER);
    uint8_t payload[SPI_MAX_PAYLOAD] = {0};
    payload[0] = src_id;
    memcpy(&payload[1], data, len);
    e->len = build_packet(e->buf, UART_PKT_AUDIO, payload, (uint8_t)(len + 1));

    /* Memory barrier: ensure entry data is visible before consumer sees new head */
    __DMB();
    s_audio_head = next_head;

    LOG_DBG("Queued audio packet: src=%d, len=%d", src_id, len);
    return 0;
}

int uart_bridge_send_event(uint8_t event_type, const uint8_t *data, uint8_t len)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    uint8_t pkt[BRIDGE_SPI_MAX_XFER] = {0};
    uint8_t payload[SPI_MAX_PAYLOAD] = {0};
    payload[0] = event_type;
    if (len > 0 && data != NULL) {
        memcpy(&payload[1], data, len);
    }

    uint16_t pkt_len = build_packet(pkt, UART_PKT_EVENT, payload, (uint8_t)(len + 1));
    queue_control_packet(pkt, pkt_len);

    LOG_DBG("Queued event: type=%d, len=%d", event_type, len);
    return 0;
}

int uart_bridge_send_status(uint8_t role, uint8_t peer_count, uint8_t node_id)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    uint8_t pkt[BRIDGE_SPI_MAX_XFER] = {0};
    uint8_t payload[3] = {role, peer_count, node_id};
    uint16_t pkt_len = build_packet(pkt, UART_PKT_STATUS, payload, sizeof(payload));
    queue_control_packet(pkt, pkt_len);

    LOG_DBG("Queued status: role=%d, peers=%d, id=%d", role, peer_count, node_id);
    return 0;
}

void uart_bridge_process(void)
{
    if (!s_initialized) {
        return;
    }

    static uint32_t txn_count = 0;

    int64_t now_us = k_uptime_get() * 1000;
    if (s_last_poll_us != 0 && now_us > s_last_poll_us) {
        uint32_t dt = (uint32_t)(now_us - s_last_poll_us);
        if (s_poll_dt_count == 0) {
            s_poll_dt_min_us = dt;
            s_poll_dt_max_us = dt;
        } else {
            if (dt < s_poll_dt_min_us) {
                s_poll_dt_min_us = dt;
            }
            if (dt > s_poll_dt_max_us) {
                s_poll_dt_max_us = dt;
            }
        }
        s_poll_dt_sum_us += dt;
        s_poll_dt_count++;
    }
    s_last_poll_us = now_us;

    static uint32_t status_keepalive = 0;

    /* Periodic keepalive status when no control packet is pending */
    if ((status_keepalive++ % 50) == 0) {
        uart_bridge_send_status(mesh_protocol_get_role(), mesh_protocol_get_peer_count(),
                                mesh_protocol_get_node_id());
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);

    /* Select packet to transmit: audio first (lock-free), then control (mutex) */
    uint8_t ah = s_audio_head;
    __DMB();  /* Ensure we see entry data written before head was updated */
    uint8_t at = s_audio_tail;
    if (ah != at) {
        struct tx_entry *e = &s_audio_q[at];
        memcpy(s_tx_buf, e->buf, e->len);
        s_audio_tail = (at + 1) % TX_AUDIO_QUEUE_SIZE;
    } else {
        /* Control FIFO — low frequency, use mutex */
        k_mutex_lock(&s_tx_lock, K_FOREVER);
        if (s_ctrl_head != s_ctrl_tail) {
            struct tx_entry *e = &s_ctrl_q[s_ctrl_tail];
            memcpy(s_tx_buf, e->buf, e->len);
            s_ctrl_tail = (s_ctrl_tail + 1) % TX_CTRL_QUEUE_SIZE;
        }
        k_mutex_unlock(&s_tx_lock);
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

    /* Parse what the slave sent back */
    parse_rx(s_rx_buf, BRIDGE_SPI_MAX_XFER);

    if ((txn_count % 2500) == 0 && s_poll_dt_count > 0) {
        uint32_t avg = (uint32_t)(s_poll_dt_sum_us / s_poll_dt_count);
        printk("[spi_bridge] poll_us min/avg/max=%u/%u/%u q_over=%u seq_gap=%u crc_fail=%u\n",
               s_poll_dt_min_us, avg, s_poll_dt_max_us, s_audio_q_overwrite, s_bridge_rx_seq_gap,
               s_bridge_rx_crc_fail);
    }

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

    uint8_t pkt[BRIDGE_SPI_MAX_XFER] = {0};
    uint16_t pkt_len = build_packet(pkt, UART_PKT_LOG, (const uint8_t *)msg, len);
    queue_control_packet(pkt, pkt_len);

    return 0;
}
