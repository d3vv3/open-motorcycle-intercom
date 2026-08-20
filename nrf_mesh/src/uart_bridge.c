/**
 * @file uart_bridge.c
 * @brief SPI Bridge to ESP32-S3 (nRF52840 = SPI Master)
 *
 * Protocol: [SYNC:0xAA][LEN:1B][SEQ:1B][TYPE:1B][PAYLOAD:0-208B][CRC8:1B]
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

#include "audio_bundle.h"
#include "bridge_frame.h"
#include "mesh_protocol.h"

LOG_MODULE_REGISTER(uart_bridge, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SPI_MAX_PAYLOAD     208
#define TX_AUDIO_QUEUE_SIZE 16

_Static_assert(MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 1 <= SPI_MAX_PAYLOAD,
               "V2 bridge payload exceeds SPI capacity");
_Static_assert(MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 1 + 5 <= BRIDGE_SPI_MAX_XFER,
               "maximum V2 bridge frame exceeds fixed SPI transfer");
_Static_assert(SPI_MAX_PAYLOAD <= BRIDGE_FRAME_MAX_PAYLOAD,
               "SPI bridge payload exceeds frame codec capacity");

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
static const struct device *s_ack_port;

static uint8_t s_tx_buf[BRIDGE_SPI_MAX_XFER];
static uint8_t s_rx_buf[BRIDGE_SPI_MAX_XFER];

static bool s_initialized = false;

/* Outbound TX queues to ESP32 (audio prioritized over control). Both queues are
 * protected because the audio producer may advance the tail when dropping the
 * oldest frame while the SPI consumer advances the same tail. */
struct tx_entry {
    uint8_t buf[BRIDGE_SPI_MAX_XFER];
    uint16_t len;
    uint8_t type;
};

static struct tx_entry s_audio_q[TX_AUDIO_QUEUE_SIZE];
static uint8_t s_audio_head = 0;
static uint8_t s_audio_tail = 0;

/* Control events can outpace an SPI poll; this FIFO prevents overwriting an unread event. */
/* aislop-ignore-next-line ai-slop/trivial-comment -- C preprocessor directive */
#define TX_CTRL_QUEUE_SIZE 4
static struct tx_entry s_ctrl_q[TX_CTRL_QUEUE_SIZE];
static uint8_t s_ctrl_head = 0;
static uint8_t s_ctrl_tail = 0;

static struct k_mutex s_tx_lock;
static uint32_t s_audio_q_overwrite = 0;
static uint32_t s_control_q_overwrite = 0;
static uint32_t s_audio_ingress_ok = 0;
static uint32_t s_audio_ingress_reject = 0;
static uint32_t s_audio_ingress_duplicate = 0;
static uint32_t s_spi_tx_audio = 0;
static uint32_t s_spi_tx_control = 0;
static uint32_t s_spi_tx_idle = 0;
static uint32_t s_spi_tx_fail = 0;
static uint32_t s_status_enqueue = 0;
static uint32_t s_status_tx = 0;
static uint32_t s_status_drop = 0;

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
static bool s_last_audio_seq_valid = false;
static uint8_t s_last_audio_seq = 0;
static uint32_t s_ack_pulse_count = 0;

static inline void pulse_ack_line(void)
{
    if (s_ack_port == NULL) {
        return;
    }

    gpio_pin_set(s_ack_port, BRIDGE_ACK_PIN, 1);
    k_busy_wait(20);
    gpio_pin_set(s_ack_port, BRIDGE_ACK_PIN, 0);
    s_ack_pulse_count++;
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Handle a fully received packet from the ESP32 slave.
 */
static int handle_rx_packet(uint8_t type, const uint8_t *payload, uint8_t len)
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
        if (len >= 1) {
            return mesh_protocol_send_audio(payload + 1, (uint8_t)(len - 1), payload[0]);
        }
        return -EINVAL;
    }

    case UART_PKT_AUDIO_V2:
        return mesh_protocol_send_audio_v2(payload, len);

    case UART_PKT_COMMAND:
        if (len > 0) {
            uint8_t cmd_id = payload[0];
            uint8_t generation = len >= sizeof(bridge_command_payload_t) ? payload[1] : 0;
            LOG_INF("Received command: 0x%02X", cmd_id);
            if (cmd_id == BRIDGE_COMMAND_MESH_START) {
                mesh_protocol_request_start(generation);
            } else if (cmd_id == BRIDGE_COMMAND_MESH_STOP) {
                mesh_protocol_request_stop(generation);
            } else if (cmd_id == BRIDGE_COMMAND_STATUS) {
                printk("[uart_bridge] PING received, sending status\n");
                mesh_protocol_request_status();
            }
        }
        return 0;

    default:
        LOG_DBG("Unknown packet type from ESP32: 0x%02X", type);
        return -ENOTSUP;
    }
    return -ENOTSUP;
}

/**
 * @brief Parse the RX buffer after an SPI transaction.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    bridge_frame_view_t frame;
    bridge_frame_decode_result_t result = bridge_frame_decode(buf, len, SPI_MAX_PAYLOAD, &frame);

    if (result == BRIDGE_FRAME_IDLE || result == BRIDGE_FRAME_BAD_SYNC ||
        (result == BRIDGE_FRAME_TRUNCATED && len < BRIDGE_FRAME_OVERHEAD)) {
        return;
    }
    if (result == BRIDGE_FRAME_BAD_LENGTH) {
        LOG_WRN("Bad packet length: %u", len >= 2u ? buf[1] : 0u);
        return;
    }
    if (result == BRIDGE_FRAME_TRUNCATED) {
        LOG_WRN("Truncated packet: need %u, got %u", buf[1] + 3u, (unsigned int)len);
        return;
    }
    if (result == BRIDGE_FRAME_BAD_CRC) {
        uint8_t pkt_len = buf[1];
        uint8_t rx_crc = buf[2u + pkt_len];
        uint8_t calc_crc = bridge_frame_crc8(buf + 1u, (size_t)pkt_len + 1u);

        s_bridge_rx_crc_fail++;
        if ((s_bridge_rx_crc_fail % 50) == 1) {
            LOG_WRN("CRC mismatch rx=0x%02X calc=0x%02X fail=%u", rx_crc, calc_crc,
                    s_bridge_rx_crc_fail);
        }
        return;
    }

    uint8_t seq = frame.seq;
    if (!s_bridge_rx_seq_init) {
        s_bridge_rx_expected = seq;
        s_bridge_rx_seq_init = true;
    } else {
        uint8_t expected_next = (uint8_t)(s_bridge_rx_expected + 1);
        if (seq == expected_next) {
            s_bridge_rx_expected = seq;
        } else if (seq != s_bridge_rx_expected) {
            s_bridge_rx_seq_gap++;
            s_bridge_rx_expected = seq;
        }
    }

    if (frame.type == UART_PKT_AUDIO || frame.type == UART_PKT_AUDIO_V2) {
        bool duplicate = s_last_audio_seq_valid && (seq == s_last_audio_seq);
        if (duplicate) {
            s_audio_ingress_duplicate++;
            pulse_ack_line();
        } else if (handle_rx_packet(frame.type, frame.payload, (uint8_t)frame.payload_len) == 0) {
            s_audio_ingress_ok++;
            s_last_audio_seq = seq;
            s_last_audio_seq_valid = true;
            pulse_ack_line();
        } else {
            s_audio_ingress_reject++;
        }
        return;
    }

    (void)handle_rx_packet(frame.type, frame.payload, (uint8_t)frame.payload_len);
}

static uint16_t build_packet(uint8_t *dst, uint8_t type, const uint8_t *payload, uint8_t len)
{
    size_t encoded_len = bridge_frame_encode(dst, BRIDGE_SPI_MAX_XFER, SPI_MAX_PAYLOAD,
                                             s_bridge_tx_seq, type, payload, len);

    if (encoded_len != 0u) {
        s_bridge_tx_seq++;
    }
    return (uint16_t)encoded_len;
}

static void queue_control_packet(const uint8_t *buf, uint16_t len, uint8_t type)
{
    k_mutex_lock(&s_tx_lock, K_FOREVER);
    uint8_t next_head = (s_ctrl_head + 1) % TX_CTRL_QUEUE_SIZE;
    if (next_head == s_ctrl_tail) {
        /* Full: drop oldest to make room for the new event */
        if (s_ctrl_q[s_ctrl_tail].type == UART_PKT_STATUS) {
            s_status_drop++;
        }
        s_control_q_overwrite++;
        s_ctrl_tail = (s_ctrl_tail + 1) % TX_CTRL_QUEUE_SIZE;
    }
    struct tx_entry *e = &s_ctrl_q[s_ctrl_head];
    memcpy(e->buf, buf, len);
    e->len = len;
    e->type = type;
    s_ctrl_head = next_head;
    if (type == UART_PKT_STATUS) {
        s_status_enqueue++;
    }
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

    s_ack_port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!device_is_ready(s_ack_port)) {
        LOG_ERR("ACK GPIO port not ready");
        return -ENODEV;
    }

    int ack_ret = gpio_pin_configure(s_ack_port, BRIDGE_ACK_PIN, GPIO_OUTPUT_INACTIVE);
    if (ack_ret) {
        LOG_ERR("ACK pin config failed: %d", ack_ret);
        return ack_ret;
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
    uart_bridge_send_status(MESH_STATE_IDLE, MESH_ROLE_NONE, 0, 0, -1, 0);

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

    k_mutex_lock(&s_tx_lock, K_FOREVER);
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
    e->type = UART_PKT_AUDIO;

    s_audio_head = next_head;
    k_mutex_unlock(&s_tx_lock);

    LOG_DBG("Queued audio packet: src=%d, len=%d", src_id, len);
    return 0;
}

int uart_bridge_send_audio_v2(uint8_t src_id, const uint8_t *data, uint8_t len)
{
    audio_bundle_view_t bundle;

    if (!s_initialized || !audio_bundle_parse(data, len, &bundle)) {
        return -EINVAL;
    }
    if ((uint16_t)len + 1U > SPI_MAX_PAYLOAD) {
        return -EMSGSIZE;
    }

    k_mutex_lock(&s_tx_lock, K_FOREVER);
    uint8_t cur_head = s_audio_head;
    uint8_t next_head = (uint8_t)((cur_head + 1) % TX_AUDIO_QUEUE_SIZE);
    if (next_head == s_audio_tail) {
        s_audio_tail = (uint8_t)((s_audio_tail + 1) % TX_AUDIO_QUEUE_SIZE);
        s_audio_q_overwrite++;
    }

    struct tx_entry *e = &s_audio_q[cur_head];
    memset(e->buf, 0, sizeof(e->buf));
    uint8_t payload[MESH_AUDIO_V2_MAX_BUNDLE_SIZE + 1];
    payload[0] = src_id;
    memcpy(payload + 1, data, len);
    e->len = build_packet(e->buf, UART_PKT_AUDIO_V2, payload, (uint8_t)(len + 1));
    e->type = UART_PKT_AUDIO_V2;
    s_audio_head = next_head;
    k_mutex_unlock(&s_tx_lock);
    return 0;
}

int uart_bridge_send_event(uint8_t event_type, const uint8_t *data, uint8_t len)
{
    if (!s_initialized) {
        return -EINVAL;
    }
    if (len > SPI_MAX_PAYLOAD - 1 || (len > 0 && data == NULL)) {
        return -EMSGSIZE;
    }

    uint8_t pkt[BRIDGE_SPI_MAX_XFER] = {0};
    uint8_t payload[SPI_MAX_PAYLOAD] = {0};
    payload[0] = event_type;
    if (len > 0 && data != NULL) {
        memcpy(&payload[1], data, len);
    }

    uint16_t pkt_len = build_packet(pkt, UART_PKT_EVENT, payload, (uint8_t)(len + 1));
    queue_control_packet(pkt, pkt_len, UART_PKT_EVENT);

    LOG_DBG("Queued event: type=%d, len=%d", event_type, len);
    return 0;
}

int uart_bridge_send_status(uint8_t state, uint8_t role, uint8_t peer_count, uint8_t node_id,
                            int8_t slot_index, uint8_t coordinator_id)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    uint8_t pkt[BRIDGE_SPI_MAX_XFER] = {0};
    bridge_status_payload_t payload = {
        .role = role,
        .peer_count = peer_count,
        .node_id = node_id,
        .version = BRIDGE_PROTOCOL_VERSION,
        .mesh_state = state,
        .slot_index = slot_index,
        .coordinator_id = coordinator_id,
        .marker = BRIDGE_STATUS_V2_MARKER,
    };
    uint16_t pkt_len =
        build_packet(pkt, UART_PKT_STATUS, (const uint8_t *)&payload, sizeof(payload));
    queue_control_packet(pkt, pkt_len, UART_PKT_STATUS);

    LOG_DBG("Queued status: state=%d role=%d peers=%d id=%d", state, role, peer_count, node_id);
    return 0;
}

int uart_bridge_send_command_ack(uint8_t command, uint8_t generation, int result)
{
    bridge_command_ack_payload_t ack = {
        .command = command,
        .generation = generation,
        .result = result == 0 ? 0 : -1,
    };
    return uart_bridge_send_event(BRIDGE_EVENT_COMMAND_ACK, (const uint8_t *)&ack, sizeof(ack));
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

    /* Publish bridge status at roughly 1 Hz. */
    if (++status_keepalive >= 200) {
        status_keepalive = 0;
        mesh_protocol_request_status();
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    uint8_t tx_kind = 0;
    uint8_t tx_index = 0;
    uint8_t tx_type = 0;

    /* Peek one packet. Advance its queue only after a successful transfer. */
    k_mutex_lock(&s_tx_lock, K_FOREVER);
    if (s_ctrl_head != s_ctrl_tail) {
        tx_index = s_ctrl_tail;
        struct tx_entry *e = &s_ctrl_q[s_ctrl_tail];
        memcpy(s_tx_buf, e->buf, e->len);
        tx_kind = 2;
        tx_type = e->type;
    } else if (s_audio_head != s_audio_tail) {
        tx_index = s_audio_tail;
        struct tx_entry *e = &s_audio_q[s_audio_tail];
        memcpy(s_tx_buf, e->buf, e->len);
        tx_kind = 1;
    }
    k_mutex_unlock(&s_tx_lock);

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
        s_spi_tx_fail++;
        if (txn_count <= 10) {
            printk("[spi_bridge] txn #%u FAILED: %d\n", txn_count, ret);
        }
        return;
    }

    k_mutex_lock(&s_tx_lock, K_FOREVER);
    if (tx_kind == 1 && s_audio_tail == tx_index) {
        s_audio_tail = (s_audio_tail + 1) % TX_AUDIO_QUEUE_SIZE;
        s_spi_tx_audio++;
    } else if (tx_kind == 2 && s_ctrl_tail == tx_index) {
        s_ctrl_tail = (s_ctrl_tail + 1) % TX_CTRL_QUEUE_SIZE;
        s_spi_tx_control++;
        if (tx_type == UART_PKT_STATUS) {
            s_status_tx++;
        }
    } else if (tx_kind == 0) {
        s_spi_tx_idle++;
    }
    k_mutex_unlock(&s_tx_lock);

    /* Debug: log first 3 transactions for boot verification */
    if (txn_count <= 3) {
        printk("[spi_bridge] txn #%u OK, rx: %02X %02X %02X %02X\n", txn_count, s_rx_buf[0],
               s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
    }

    /* Parse what the slave sent back */
    parse_rx(s_rx_buf, BRIDGE_SPI_MAX_XFER);

    if ((txn_count % 2500) == 0 && s_poll_dt_count > 0) {
        uint32_t avg = (uint32_t)(s_poll_dt_sum_us / s_poll_dt_count);
        printk("[spi_bridge] poll_us min/avg/max=%u/%u/%u q_over=%u seq_gap=%u crc_fail=%u "
               "ack_pulse=%u\n",
               s_poll_dt_min_us, avg, s_poll_dt_max_us, s_audio_q_overwrite, s_bridge_rx_seq_gap,
               s_bridge_rx_crc_fail, s_ack_pulse_count);
        printk("PIPE v=1 dev=nrf stage=spi ingress_ok=%u ingress_reject=%u ingress_dup=%u "
               "rx_seq_gap=%u rx_crc_drop=%u ack=%u audio_q_drop=%u control_q_drop=%u tx_audio=%u "
               "tx_control=%u tx_idle=%u tx_fail=%u poll_max_us=%u status_enqueue=%u status_tx=%u "
               "status_drop=%u\n",
               s_audio_ingress_ok, s_audio_ingress_reject, s_audio_ingress_duplicate,
               s_bridge_rx_seq_gap, s_bridge_rx_crc_fail, s_ack_pulse_count, s_audio_q_overwrite,
               s_control_q_overwrite, s_spi_tx_audio, s_spi_tx_control, s_spi_tx_idle,
               s_spi_tx_fail, s_poll_dt_max_us, s_status_enqueue, s_status_tx, s_status_drop);
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
    queue_control_packet(pkt, pkt_len, UART_PKT_LOG);

    return 0;
}
