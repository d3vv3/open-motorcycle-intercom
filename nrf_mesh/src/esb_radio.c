/**
 * @file esb_radio.c
 * @brief ESB Radio Driver Implementation
 */

#include "esb_radio.h"

#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(esb_radio, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define ESB_MAX_PAYLOAD_LEN 252
#define ESB_ADDR_LEN        5

/* Broadcast address for mesh discovery */
static const uint8_t broadcast_addr[ESB_ADDR_LEN] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static esb_rx_callback_t s_rx_callback = NULL;
static uint8_t s_local_addr[ESB_ADDR_LEN];
static struct esb_payload s_tx_payload;
static struct esb_payload s_rx_payload;
static bool s_initialized = false;

/* ============================================================================
 * ESB Event Handler
 * ============================================================================ */

static void on_esb_event(struct esb_evt const *event)
{
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("TX success");
        break;

    case ESB_EVENT_TX_FAILED:
        LOG_WRN("TX failed");
        break;

    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&s_rx_payload) == 0) {
            LOG_DBG("RX: %d bytes from pipe %d", s_rx_payload.length, s_rx_payload.pipe);

            if (s_rx_callback && s_rx_payload.length > 0) {
                /* Note: ESB doesn't provide source address directly,
                 * it's embedded in our mesh protocol header */
                s_rx_callback(s_rx_payload.data, s_rx_payload.length, NULL, s_rx_payload.rssi);
            }
        }
        break;
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int esb_radio_init(uint8_t channel)
{
    if (s_initialized) {
        LOG_WRN("ESB already initialized");
        return -EALREADY;
    }

    LOG_INF("Initializing ESB radio on channel %d", channel);

    /* Generate local address from device ID */
    /* Use last 5 bytes of device ID as ESB address */
    uint32_t dev_id[2];
    dev_id[0] = NRF_FICR->DEVICEID[0];
    dev_id[1] = NRF_FICR->DEVICEID[1];

    s_local_addr[0] = (dev_id[0] >> 0) & 0xFF;
    s_local_addr[1] = (dev_id[0] >> 8) & 0xFF;
    s_local_addr[2] = (dev_id[0] >> 16) & 0xFF;
    s_local_addr[3] = (dev_id[0] >> 24) & 0xFF;
    s_local_addr[4] = (dev_id[1] >> 0) & 0xFF;

    LOG_INF("Local ESB addr: %02X:%02X:%02X:%02X:%02X", s_local_addr[0], s_local_addr[1],
            s_local_addr[2], s_local_addr[3], s_local_addr[4]);

    /* ESB configuration */
    struct esb_config config = ESB_DEFAULT_CONFIG;
    config.protocol = ESB_PROTOCOL_ESB_DPL; /* Dynamic payload length */
    config.mode = ESB_MODE_PTX;             /* Start as PTX, switch as needed */
    config.event_handler = on_esb_event;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.crc = ESB_CRC_16BIT;
    config.tx_output_power = ESB_TX_POWER_4DBM;
    config.retransmit_delay = 600;
    config.retransmit_count = 3;
    config.tx_mode = ESB_TXMODE_AUTO;
    config.payload_length = ESB_MAX_PAYLOAD_LEN;
    config.selective_auto_ack = false;

    int ret = esb_init(&config);
    if (ret) {
        LOG_ERR("ESB init failed: %d", ret);
        return ret;
    }

    /* Set RF channel */
    ret = esb_set_rf_channel(channel);
    if (ret) {
        LOG_ERR("Set channel failed: %d", ret);
        return ret;
    }

    /* Set up pipes:
     * Pipe 0: Broadcast (for discovery, SYNC)
     * Pipe 1: Our local address (for unicast) */
    ret = esb_set_base_address_0(broadcast_addr);
    if (ret) {
        LOG_ERR("Set base addr 0 failed: %d", ret);
        return ret;
    }

    ret = esb_set_base_address_1(s_local_addr);
    if (ret) {
        LOG_ERR("Set base addr 1 failed: %d", ret);
        return ret;
    }

    s_initialized = true;
    LOG_INF("ESB radio initialized");

    return 0;
}

void esb_radio_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    esb_disable();
    s_initialized = false;
    LOG_INF("ESB radio deinitialized");
}

void esb_radio_set_rx_callback(esb_rx_callback_t cb)
{
    s_rx_callback = cb;
}

int esb_radio_send(const uint8_t *data, uint8_t len)
{
    return esb_radio_send_to(broadcast_addr, data, len);
}

int esb_radio_send_to(const uint8_t *addr, const uint8_t *data, uint8_t len)
{
    if (!s_initialized || data == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }

    /* Set destination address */
    int ret = esb_set_base_address_0(addr);
    if (ret) {
        LOG_ERR("Set TX addr failed: %d", ret);
        return ret;
    }

    /* Prepare payload */
    s_tx_payload.pipe = 0;
    s_tx_payload.length = len;
    memcpy(s_tx_payload.data, data, len);

    /* Send */
    ret = esb_write_payload(&s_tx_payload);
    if (ret) {
        LOG_ERR("TX write failed: %d", ret);
        return ret;
    }

    return 0;
}

int esb_radio_start_rx(void)
{
    if (!s_initialized) {
        return -EINVAL;
    }

    int ret = esb_start_rx();
    if (ret) {
        LOG_ERR("Start RX failed: %d", ret);
        return ret;
    }

    LOG_DBG("RX started");
    return 0;
}

void esb_radio_stop_rx(void)
{
    if (!s_initialized) {
        return;
    }

    esb_stop_rx();
    LOG_DBG("RX stopped");
}

void esb_radio_get_address(uint8_t *addr)
{
    if (addr) {
        memcpy(addr, s_local_addr, ESB_ADDR_LEN);
    }
}
