/**
 * @file esb_radio.c
 * @brief ESB Radio Driver Implementation
 */

#include "esb_radio.h"

#include <esb.h>
#include <nrfx_clock.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/onoff.h>

LOG_MODULE_REGISTER(esb_radio, LOG_LEVEL_INF);

/* ============================================================================
 * Constants
 * ============================================================================ */

#define ESB_MAX_PAYLOAD_LEN CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_ADDR_LEN        5
#define TX_DONE_TIMEOUT_US  1450
#define TX_RECOVERY_TIMEOUT_MS 20

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
static bool s_faulted = false;
static bool s_tx_in_progress = false; /* Prevent re-entry during TX */
static bool s_rx_active = false;      /* Track if RX mode is running */
static bool s_tx_recovery_pending = false;
static int64_t s_recovery_deadline_ms;
static bool s_rx_requested = false;
static enum esb_mode s_mode;
static struct esb_config s_config;
static uint8_t s_channel;
static K_MUTEX_DEFINE(s_radio_mutex);

/* The radio needs the external 32 MHz crystal (HFXO). The ESB library does not
 * request it; without this request the radio runs from the internal RC clock
 * and is off-frequency whenever nothing else (such as USB) holds HFXO on. */
static struct onoff_manager *s_hfclk_mgr = NULL;
static struct onoff_client s_hfclk_cli;
static bool s_hfclk_requested = false;

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
static void hfclk_stop(void);
static void recovery_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_recovery_work, recovery_work_handler);

static void fault_radio_locked(const char *reason)
{
    LOG_ERR("ESB radio faulted until reboot: %s", reason);
    s_faulted = true;
    s_tx_recovery_pending = false;
    s_rx_active = false;
    s_rx_requested = false;
    if (s_initialized) {
        /* esb_disable is the SDK's immediate-stop path; never reinit a busy TX. */
        esb_disable();
        s_initialized = false;
    }
    k_sem_reset(&s_tx_done_sem);
    hfclk_stop();
}

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

/* All role changes run under s_radio_mutex in thread context. esb_init resets
 * FIFOs and PIDs; packets already passed to the application callback survive. */
static int switch_mode_locked(enum esb_mode mode, bool force)
{
    if (s_mode == mode && !force) {
        return 0;
    }
    if (!esb_is_idle()) {
        return -EBUSY;
    }

    struct esb_payload dropped;

    /* RX is stopped; exclude the ESB event ISR's FIFO reader for this drain. */
    unsigned int key = irq_lock();
    for (int i = 0; i < CONFIG_ESB_RX_FIFO_SIZE; i++) {
        if (esb_read_rx_payload(&dropped) != 0) {
            break;
        }
        atomic_inc(&s_rx_flush_drop_count);
    }
    irq_unlock(key);
    esb_disable();
    s_initialized = false;
    s_rx_active = false;

    s_config.mode = mode;
    int ret = esb_init(&s_config);

    if (ret) {
        /* SDK init can fail after allocating resources without public rollback. */
        LOG_ERR("ESB reinit failed: %d; radio faulted until reboot", ret);
        s_faulted = true;
        hfclk_stop();
        return ret;
    }
    s_initialized = true;
    s_mode = mode;

    ret = esb_set_rf_channel(s_channel);
    if (!ret) {
        ret = esb_set_base_address_0(broadcast_addr);
    }
    if (!ret) {
        ret = esb_set_base_address_1(s_local_addr);
    }
    if (ret) {
        LOG_ERR("ESB role address/channel setup failed: %d", ret);
        esb_disable();
        s_initialized = false;
        hfclk_stop();
    }
    return ret;
}

static int restart_rx_after_tx_locked(void)
{
    int ret = switch_mode_locked(ESB_MODE_PRX, false);

    if (ret == 0) {
        ret = esb_start_rx();
    }

    if (ret == 0) {
        s_rx_active = true;
    } else {
        s_rx_restart_fail_count++;
    }
    return ret;
}

static void recovery_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    k_mutex_lock(&s_radio_mutex, K_FOREVER);
    if (s_initialized && s_tx_recovery_pending) {
        if (!esb_is_idle()) {
            if (k_uptime_get() >= s_recovery_deadline_ms) {
                fault_radio_locked("TX did not become idle");
            } else {
                k_work_reschedule(&s_recovery_work, K_MSEC(1));
            }
        } else {
            /* Even if already PTX, reset SDK flags/FIFOs and pending old events
             * before clearing the semaphore or accepting another payload. */
            int ret = switch_mode_locked(s_rx_requested ? ESB_MODE_PRX : ESB_MODE_PTX,
                                         true);
            if (!ret && s_rx_requested) {
                ret = esb_start_rx();
                if (!ret) {
                    s_rx_active = true;
                } else {
                    s_rx_restart_fail_count++;
                }
            }
            if (ret) {
                fault_radio_locked("TX recovery reinit/RX restart failed");
            } else {
                k_sem_reset(&s_tx_done_sem);
                s_tx_recovery_pending = false;
            }
        }
    }
    k_mutex_unlock(&s_radio_mutex);
}

static void account_tx_wait(int64_t tx_start_us)
{
    s_tx_timing_count++;
    uint32_t tx_wait_us = (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - tx_start_us);

    s_tx_wait_sum_us += tx_wait_us;
    if (tx_wait_us > s_tx_wait_max_us) {
        s_tx_wait_max_us = tx_wait_us;
    }
}

static void account_rx_pause(int64_t rx_pause_start_us)
{
    if (rx_pause_start_us > 0) {
        uint32_t pause_us = (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - rx_pause_start_us);
        s_rx_pause_sum_us += pause_us;
        if (pause_us > s_rx_pause_max_us) {
            s_rx_pause_max_us = pause_us;
        }
    }
}

static void recover_tx_timeout(int64_t tx_start_us, int64_t rx_pause_start_us)
{
    LOG_ERR("TX timed out");
    s_tx_timeout_count++;
    account_tx_wait(tx_start_us);
    for (int wait = 0; wait < 100 && !esb_is_idle(); wait++) {
        k_busy_wait(10);
    }
    s_tx_recovery_pending = true;
    s_recovery_deadline_ms = k_uptime_get() + TX_RECOVERY_TIMEOUT_MS;
    k_work_reschedule(&s_recovery_work, K_NO_WAIT);
    s_tx_in_progress = false;
    if (s_rx_requested) {
        account_rx_pause(rx_pause_start_us);
    }
}

/* UF2/USB bootloaders can hand over with HFXO already running. nrfx then
 * triggers HFCLKSTART on a running crystal, no HFCLKSTARTED event follows and
 * the clock manager stays in "turning on" forever. Stop it before the clock
 * driver initializes so the first request starts from a clean state. */
static int hfclk_release_bootloader_state(void)
{
    nrf_clock_hfclk_t src = NRF_CLOCK_HFCLK_LOW_ACCURACY;

    if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &src) &&
        src == NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
        nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK);
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    }
    return 0;
}
SYS_INIT(hfclk_release_bootloader_state, PRE_KERNEL_1, 0);

static int hfclk_start(void)
{
    if (s_hfclk_requested) {
        return 0;
    }

    s_hfclk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (s_hfclk_mgr == NULL) {
        LOG_ERR("HF clock manager unavailable");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&s_hfclk_cli.notify);
    int ret = onoff_request(s_hfclk_mgr, &s_hfclk_cli);
    if (ret < 0) {
        LOG_ERR("HF clock request failed: %d", ret);
        return ret;
    }
    int ret_req = ret;
    s_hfclk_requested = true;

    int res = 0;
    int64_t deadline = k_uptime_get() + 100;
    while ((ret = sys_notify_fetch_result(&s_hfclk_cli.notify, &res)) == -EAGAIN) {
        if (k_uptime_get() > deadline) {
            nrf_clock_hfclk_t src = NRF_CLOCK_HFCLK_LOW_ACCURACY;
            bool running = nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &src);
            if (running && src == NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
                LOG_WRN("HF clock manager stuck (state %d) but HFXO is running; continuing",
                        ret_req);
                return 0;
            }
            LOG_ERR("HF clock start timed out (state=%d running=%d src=%d)",
                    ret_req, (int)running, (int)src);
            return -ETIMEDOUT;
        }
        k_busy_wait(50);
    }
    if (ret < 0 || res < 0) {
        LOG_ERR("HF clock could not be started: %d/%d", ret, res);
        return ret < 0 ? ret : res;
    }

    LOG_INF("HFXO running for radio");
    return 0;
}

static void hfclk_stop(void)
{
    if (s_hfclk_requested && s_hfclk_mgr != NULL) {
        (void)onoff_cancel_or_release(s_hfclk_mgr, &s_hfclk_cli);
    }
    s_hfclk_requested = false;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int esb_radio_init(uint8_t channel)
{
    k_mutex_lock(&s_radio_mutex, K_FOREVER);
    if (s_faulted) {
        k_mutex_unlock(&s_radio_mutex);
        return -EIO;
    }
    if (s_initialized) {
        LOG_WRN("ESB already initialized");
        k_mutex_unlock(&s_radio_mutex);
        return -EALREADY;
    }

    LOG_INF("Initializing ESB radio on channel %d", channel);

    int clk_ret = hfclk_start();
    if (clk_ret) {
        hfclk_stop();
        k_mutex_unlock(&s_radio_mutex);
        return clk_ret;
    }

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
    s_config = (struct esb_config)ESB_DEFAULT_CONFIG;
    s_config.protocol = ESB_PROTOCOL_ESB_DPL; /* Dynamic payload length */
    s_config.mode = ESB_MODE_PTX; /* Reinitialize as PRX when reception starts */
    s_config.event_handler = on_esb_event;
    s_config.bitrate = OMI_ESB_BITRATE;
    s_config.crc = ESB_CRC_16BIT;
    s_config.tx_output_power = OMI_ESB_TX_POWER_DBM;
    s_config.retransmit_delay = 500;
    s_config.retransmit_count = 3;
    s_config.tx_mode = ESB_TXMODE_AUTO;
    s_config.payload_length = ESB_MAX_PAYLOAD_LEN;
    s_config.selective_auto_ack = true; /* Required for noack flag to work */
    s_channel = channel;

    int ret = esb_init(&s_config);
    if (ret) {
        /* SDK partial init may retain resources; do not retry until reboot. */
        LOG_ERR("ESB init failed: %d; radio faulted until reboot", ret);
        s_faulted = true;
        hfclk_stop();
        k_mutex_unlock(&s_radio_mutex);
        return ret;
    }
    s_initialized = true;
    s_mode = ESB_MODE_PTX;

    /* Set RF channel */
    ret = esb_set_rf_channel(channel);
    if (ret) {
        LOG_ERR("Set channel failed: %d", ret);
        goto init_cleanup;
    }

    /* Set up pipes:
     * Pipe 0: Broadcast (for discovery, SYNC)
     * Pipe 1: Our local address (for unicast) */
    ret = esb_set_base_address_0(broadcast_addr);
    if (ret) {
        LOG_ERR("Set base addr 0 failed: %d", ret);
        goto init_cleanup;
    }

    ret = esb_set_base_address_1(s_local_addr);
    if (ret) {
        LOG_ERR("Set base addr 1 failed: %d", ret);
        goto init_cleanup;
    }

    s_rx_active = false;
    s_rx_requested = false;
    s_tx_recovery_pending = false;
    LOG_INF("ESB radio initialized (bitrate=2Mbps)");
    k_mutex_unlock(&s_radio_mutex);
    return 0;

init_cleanup:
    esb_disable();
    s_initialized = false;
    hfclk_stop();
    k_mutex_unlock(&s_radio_mutex);
    return ret;
}

void esb_radio_deinit(void)
{
    k_mutex_lock(&s_radio_mutex, K_FOREVER);
    if (!s_initialized) {
        k_mutex_unlock(&s_radio_mutex);
        return;
    }

    if (s_rx_active) {
        int ret = esb_stop_rx();
        if (ret) {
            LOG_ERR("Cannot deinit while RX is active: %d", ret);
            k_mutex_unlock(&s_radio_mutex);
            return;
        }
        s_rx_active = false;
    }
    if (!esb_is_idle()) {
        LOG_ERR("Cannot deinit while ESB is busy");
        k_mutex_unlock(&s_radio_mutex);
        return;
    }
    esb_disable();
    hfclk_stop();
    s_initialized = false;
    s_rx_requested = false;
    s_tx_recovery_pending = false;
    (void)k_work_cancel_delayable(&s_recovery_work);
    LOG_INF("ESB radio deinitialized");
    k_mutex_unlock(&s_radio_mutex);
}

void esb_radio_set_rx_callback(esb_rx_callback_t cb)
{
    unsigned int key = irq_lock();
    s_rx_callback = cb;
    irq_unlock(key);
}

int esb_radio_send(const uint8_t *data, uint8_t len)
{
    return esb_radio_send_to(broadcast_addr, data, len);
}

int esb_radio_send_to(const uint8_t *addr, const uint8_t *data, uint8_t len)
{
    int64_t tx_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t rx_pause_start_us = 0;
    int ret;

    if (addr == NULL || data == NULL || len == 0) {
        return -EINVAL;
    }

    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }
    if (k_mutex_lock(&s_radio_mutex, K_NO_WAIT) != 0) {
        return -EBUSY;
    }
    if (s_faulted || !s_initialized) {
        ret = s_faulted ? -EIO : -EINVAL;
        goto out;
    }

    if (s_tx_recovery_pending) {
        s_tx_busy_count++;
        ret = -EBUSY;
        goto out;
    }

    /* Prevent re-entry - if already transmitting, drop this packet */
    if (s_tx_in_progress) {
        s_tx_busy_count++;
        LOG_WRN("TX busy, dropping packet");
        ret = -EBUSY;
        goto out;
    }
    s_tx_in_progress = true;

    /* Only stop RX if it was actually active */
    bool was_rx_active = s_rx_active;
    if (was_rx_active) {
        rx_pause_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
        ret = esb_stop_rx();
        if (ret) {
            goto tx_done;
        }
        s_rx_active = false;
    }
    if (!esb_is_idle()) {
        ret = -EBUSY;
        goto tx_done;
    }

    ret = switch_mode_locked(ESB_MODE_PTX, false);
    if (ret) {
        goto tx_done;
    }

    /* Flush any pending TX to prevent FIFO overflow */
    ret = esb_flush_tx();
    if (ret) {
        goto tx_done;
    }

    /* Set destination address */
    ret = esb_set_base_address_0(addr);
    if (ret) {
        LOG_ERR("Set TX addr failed: %d", ret);
        goto tx_done;
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
        goto tx_done;
    }

    /* Keep a lost completion event from occupying more than one TDMA slot. */
    if (k_sem_take(&s_tx_done_sem, K_USEC(TX_DONE_TIMEOUT_US)) != 0) {
        recover_tx_timeout(tx_start_us, rx_pause_start_us);
        ret = -ETIMEDOUT;
        goto out;
    }

    /* Busy-wait for ESB to reach IDLE (should be immediate after sem) */
    int idle_wait = 0;
    while (!esb_is_idle() && idle_wait < 100) {
        k_busy_wait(10);
        idle_wait++;
    }

    if (!esb_is_idle()) {
        s_tx_recovery_pending = true;
        s_recovery_deadline_ms = k_uptime_get() + TX_RECOVERY_TIMEOUT_MS;
        k_work_reschedule(&s_recovery_work, K_MSEC(1));
        ret = -EBUSY;
        goto tx_done;
    }
    ret = s_last_tx_status;
    if (ret == 0) {
        s_tx_count++;
    }
    account_tx_wait(tx_start_us);

tx_done:
    s_tx_in_progress = false;
    if (s_rx_requested && s_initialized && !s_tx_recovery_pending && esb_is_idle() &&
        !s_rx_active) {
        int rx_ret = restart_rx_after_tx_locked();
        if (rx_ret) {
            fault_radio_locked("RX restart after TX failed");
            if (ret == 0) {
                ret = rx_ret;
            }
        }
    }
    if (was_rx_active) {
        account_rx_pause(rx_pause_start_us);
    }
out:
    k_mutex_unlock(&s_radio_mutex);
    return ret;
}

int esb_radio_start_rx(void)
{
    k_mutex_lock(&s_radio_mutex, K_FOREVER);
    if (s_faulted) {
        k_mutex_unlock(&s_radio_mutex);
        return -EIO;
    }
    if (!s_initialized) {
        k_mutex_unlock(&s_radio_mutex);
        return -EINVAL;
    }

    if (s_rx_active) {
        k_mutex_unlock(&s_radio_mutex);
        return 0;
    }
    if (!esb_is_idle()) {
        k_mutex_unlock(&s_radio_mutex);
        return -EBUSY;
    }
    if (s_tx_recovery_pending) {
        k_mutex_unlock(&s_radio_mutex);
        return -EBUSY;
    }
    int ret = switch_mode_locked(ESB_MODE_PRX, false);
    if (!ret) {
        ret = esb_start_rx();
    }
    if (ret) {
        LOG_ERR("Start RX failed: %d", ret);
        k_mutex_unlock(&s_radio_mutex);
        return ret;
    }

    s_rx_active = true;
    s_rx_requested = true;
    LOG_DBG("RX started");
    k_mutex_unlock(&s_radio_mutex);
    return 0;
}

void esb_radio_stop_rx(void)
{
    k_mutex_lock(&s_radio_mutex, K_FOREVER);
    if (!s_initialized) {
        k_mutex_unlock(&s_radio_mutex);
        return;
    }

    s_rx_requested = false;
    if (s_rx_active) {
        int ret = esb_stop_rx();
        if (ret) {
            LOG_ERR("Stop RX failed: %d", ret);
        } else {
            s_rx_active = false;
        }
    }
    LOG_DBG("RX stopped");
    k_mutex_unlock(&s_radio_mutex);
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
