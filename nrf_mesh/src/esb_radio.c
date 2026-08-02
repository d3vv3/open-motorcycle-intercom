/**
 * @file esb_radio.c
 * @brief ESB Radio Driver Implementation
 */

#include "esb_radio.h"

#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(esb_radio, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define ESB_MAX_PAYLOAD_LEN CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_ADDR_LEN        5
#define TX_DONE_TIMEOUT_US  1450

/* RF robustness tuning */
#ifndef ESB_BITRATE_250KBPS
#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
#define ESB_BITRATE_250KBPS ((enum esb_bitrate)RADIO_MODE_MODE_Nrf_250Kbit)
#else
#define ESB_BITRATE_250KBPS ESB_BITRATE_1MBPS
#endif
#endif

#define OMI_ESB_BITRATE      ESB_BITRATE_2MBPS
#define OMI_ESB_TX_POWER_DBM 8

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
static bool s_tx_in_progress = false; /* Prevent re-entry during TX */
static bool s_rx_active = false;      /* Track if RX mode is running */
static bool s_tx_recovery_pending = false;
static bool s_recovery_restart_rx = false;

/* ESB TX/RX timing diagnostics */
static uint32_t s_tx_count = 0;
static uint32_t s_tx_timing_count = 0;
static uint32_t s_tx_timeout_count = 0;
static uint32_t s_tx_busy_count = 0;
static uint32_t s_tx_write_fail_count = 0;
static atomic_t s_tx_failed_event_count;
static uint64_t s_tx_wait_sum_us = 0;
static uint32_t s_tx_wait_max_us = 0;
static uint64_t s_rx_pause_sum_us = 0;
static uint32_t s_rx_pause_max_us = 0;
static atomic_t s_rx_no_callback_count;
static atomic_t s_rx_flush_drop_count;
static uint32_t s_rx_restart_fail_count = 0;

/* Semaphore signaled when TX completes (success or fail) */
static K_SEM_DEFINE(s_tx_done_sem, 0, 1);

/* ============================================================================
 * ESB Event Handler
 * ============================================================================ */

static volatile int s_last_tx_status = 0; /* 0=Success, -EIO=Failed */

static void on_esb_event(struct esb_evt const *event)
{
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        s_last_tx_status = 0;
        k_sem_give(&s_tx_done_sem);
        break;

    case ESB_EVENT_TX_FAILED:
        atomic_inc(&s_tx_failed_event_count);
        s_last_tx_status = -EIO;
        k_sem_give(&s_tx_done_sem);
        break;

    case ESB_EVENT_RX_RECEIVED: {
        int rx_count = 0;
        while (rx_count < 8 && esb_read_rx_payload(&s_rx_payload) == 0) {
            rx_count++;
            if (s_rx_callback && s_rx_payload.length > 0) {
                s_rx_callback(s_rx_payload.data, s_rx_payload.length, NULL, s_rx_payload.rssi);
            } else if (!s_rx_callback) {
                atomic_inc(&s_rx_no_callback_count);
            }
        }
        /* Flush anything remaining to prevent FIFO buildup */
        if (rx_count >= 8) {
            while (esb_read_rx_payload(&s_rx_payload) == 0) {
                atomic_inc(&s_rx_flush_drop_count);
            }
            esb_flush_rx();
        }
        break;
    }
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
    config.bitrate = OMI_ESB_BITRATE;
    config.crc = ESB_CRC_16BIT;
    config.tx_output_power = OMI_ESB_TX_POWER_DBM;
    config.retransmit_delay = 500;
    config.retransmit_count = 3;
    config.tx_mode = ESB_TXMODE_AUTO;
    config.payload_length = ESB_MAX_PAYLOAD_LEN;
    config.selective_auto_ack = true; /* Required for noack flag to work */

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
    LOG_INF("ESB radio initialized (bitrate=2Mbps)");

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
    int64_t tx_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t rx_pause_start_us = 0;

    if (!s_initialized || data == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }

    if (s_tx_recovery_pending) {
        if (!esb_is_idle()) {
            s_tx_busy_count++;
            return -EBUSY;
        }
        k_sem_reset(&s_tx_done_sem);
        s_tx_recovery_pending = false;
        if (s_recovery_restart_rx) {
            if (esb_start_rx() == 0) {
                s_rx_active = true;
                s_recovery_restart_rx = false;
            } else {
                s_rx_restart_fail_count++;
                s_tx_recovery_pending = true;
                return -EIO;
            }
        }
    }

    /* Prevent re-entry - if already transmitting, drop this packet */
    if (s_tx_in_progress) {
        s_tx_busy_count++;
        LOG_WRN("TX busy, dropping packet");
        return -EBUSY;
    }
    s_tx_in_progress = true;

    /* Only stop RX if it was actually active */
    bool was_rx_active = s_rx_active;
    if (was_rx_active) {
        rx_pause_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
        esb_stop_rx();
        s_rx_active = false;
    }

    /* Flush any pending TX to prevent FIFO overflow */
    esb_flush_tx();

    /* Set destination address */
    int ret = esb_set_base_address_0(addr);
    if (ret) {
        LOG_ERR("Set TX addr failed: %d", ret);
        s_tx_in_progress = false;
        if (was_rx_active) {
            if (esb_start_rx() == 0) {
                s_rx_active = true;
            } else {
                s_rx_restart_fail_count++;
            }
        }
        return ret;
    }

    /* Prepare payload */
    s_tx_payload.pipe = 0;
    s_tx_payload.length = len;
    s_tx_payload.noack = true; /* Don't wait for ACK on broadcast */
    memcpy(s_tx_payload.data, data, len);

    /* Reset semaphore before TX */
    k_sem_reset(&s_tx_done_sem);

    /* Send */
    ret = esb_write_payload(&s_tx_payload);
    if (ret) {
        s_tx_write_fail_count++;
        LOG_ERR("TX write failed: %d", ret);
        s_tx_in_progress = false;
        if (was_rx_active) {
            if (esb_start_rx() == 0) {
                s_rx_active = true;
            } else {
                s_rx_restart_fail_count++;
            }
        }
        return ret;
    }

    /* Keep a lost completion event from occupying more than one TDMA slot. */
    if (k_sem_take(&s_tx_done_sem, K_USEC(TX_DONE_TIMEOUT_US)) != 0) {
        LOG_ERR("TX timed out");
        s_tx_timeout_count++;
        s_tx_timing_count++;
        uint32_t timeout_wait_us =
            (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - tx_start_us);
        s_tx_wait_sum_us += timeout_wait_us;
        if (timeout_wait_us > s_tx_wait_max_us) {
            s_tx_wait_max_us = timeout_wait_us;
        }
        esb_flush_tx();
        for (int wait = 0; wait < 100 && !esb_is_idle(); wait++) {
            k_busy_wait(10);
        }
        k_sem_reset(&s_tx_done_sem);
        bool radio_idle = esb_is_idle();
        s_tx_recovery_pending = !radio_idle;
        s_tx_in_progress = false;
        if (was_rx_active) {
            if (!radio_idle) {
                s_recovery_restart_rx = true;
            } else if (esb_start_rx() == 0) {
                s_rx_active = true;
            } else {
                s_rx_restart_fail_count++;
            }

            if (rx_pause_start_us > 0) {
                uint32_t pause_us =
                    (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - rx_pause_start_us);
                s_rx_pause_sum_us += pause_us;
                if (pause_us > s_rx_pause_max_us) {
                    s_rx_pause_max_us = pause_us;
                }
            }
        }
        return -ETIMEDOUT;
    }

    /* Busy-wait for ESB to reach IDLE (should be immediate after sem) */
    int idle_wait = 0;
    while (!esb_is_idle() && idle_wait < 100) {
        k_busy_wait(10);
        idle_wait++;
    }

    s_tx_in_progress = false;

    /* Resume RX mode only if it was active before */
    if (was_rx_active) {
        int rx_ret = esb_start_rx();
        if (rx_ret == 0) {
            s_rx_active = true;

            if (rx_pause_start_us > 0) {
                uint32_t pause_us =
                    (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - rx_pause_start_us);
                s_rx_pause_sum_us += pause_us;
                if (pause_us > s_rx_pause_max_us) {
                    s_rx_pause_max_us = pause_us;
                }
            }
        } else {
            s_rx_restart_fail_count++;
            LOG_ERR("Failed to resume RX after TX: %d", rx_ret);
            s_rx_active = false;
        }
    }

    if (s_last_tx_status == 0) {
        s_tx_count++;
    }
    s_tx_timing_count++;
    uint32_t tx_wait_us = (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - tx_start_us);
    s_tx_wait_sum_us += tx_wait_us;
    if (tx_wait_us > s_tx_wait_max_us) {
        s_tx_wait_max_us = tx_wait_us;
    }

    return s_last_tx_status;
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

    s_rx_active = true;
    LOG_DBG("RX started");
    return 0;
}

void esb_radio_stop_rx(void)
{
    if (!s_initialized) {
        return;
    }

    esb_stop_rx();
    s_rx_active = false;
    LOG_DBG("RX stopped");
}

void esb_radio_get_address(uint8_t *addr)
{
    if (addr) {
        memcpy(addr, s_local_addr, ESB_ADDR_LEN);
    }
}

void esb_radio_get_timing_stats(esb_radio_timing_stats_t *stats)
{
    if (!stats) {
        return;
    }

    stats->tx_count = s_tx_count;
    stats->tx_timeout_count = s_tx_timeout_count;
    stats->tx_busy_count = s_tx_busy_count;
    stats->tx_write_fail_count = s_tx_write_fail_count;
    stats->tx_failed_event_count = (uint32_t)atomic_get(&s_tx_failed_event_count);
    stats->tx_wait_us_max = s_tx_wait_max_us;
    stats->tx_wait_us_avg =
        s_tx_timing_count ? (uint32_t)(s_tx_wait_sum_us / s_tx_timing_count) : 0;
    stats->rx_pause_us_max = s_rx_pause_max_us;
    stats->rx_pause_us_avg =
        s_tx_timing_count ? (uint32_t)(s_rx_pause_sum_us / s_tx_timing_count) : 0;
    stats->rx_no_callback_count = (uint32_t)atomic_get(&s_rx_no_callback_count);
    stats->rx_flush_drop_count = (uint32_t)atomic_get(&s_rx_flush_drop_count);
    stats->rx_restart_fail_count = s_rx_restart_fail_count;
}
