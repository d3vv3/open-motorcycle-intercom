#include "rtt_probe.h"

#include <string.h>

#include "esp_timer.h"

#include "rtt_probe_defs.h"
#include "uart_bridge.h"

#define RTT_PROBE_INTERVAL_MS 2000
#define RTT_TIMEOUT_US        5000000

static uint8_t s_probe_id = 0;
static uint8_t s_diag_tx_seq = 0;
static uint32_t s_sent = 0;
static uint32_t s_recv = 0;
static uint32_t s_lost = 0;
static uint32_t s_rtt_ms_avg = 0;
static uint32_t s_rtt_ms_max = 0;
static uint32_t s_jitter_ms_avg = 0;
static uint32_t s_jitter_ms_max = 0;

static int64_t s_prev_rtt_ms = -1;
static uint64_t s_rtt_sum_ms = 0;
static uint64_t s_jitter_sum_ms = 0;

static bool s_pending[256] = {0};
static int64_t s_send_ts_us[256] = {0};
static int64_t s_last_probe_ms = 0;

static void send_probe(void)
{
    uint8_t id = ++s_probe_id;
    uint8_t pkt[RTT_PKT_LEN + 1] = {
        0, ++s_diag_tx_seq, RTT_MAGIC0, RTT_MAGIC1, RTT_MAGIC2, RTT_MAGIC3, RTT_TYPE_REQ, id,
    };

    if (uart_bridge_send_audio(pkt, sizeof(pkt)) == ESP_OK) {
        s_pending[id] = true;
        s_send_ts_us[id] = esp_timer_get_time();
        s_sent++;
    }
}

static void housekeeping(void)
{
    int64_t now_us = esp_timer_get_time();

    for (int i = 0; i < 256; i++) {
        if (s_pending[i] && s_send_ts_us[i] > 0 && (now_us - s_send_ts_us[i]) > RTT_TIMEOUT_US) {
            s_pending[i] = false;
            s_lost++;
        }
    }
}

void rtt_probe_init(void)
{
    s_probe_id = 0;
    s_diag_tx_seq = 0;
    s_sent = 0;
    s_recv = 0;
    s_lost = 0;
    s_rtt_ms_avg = 0;
    s_rtt_ms_max = 0;
    s_jitter_ms_avg = 0;
    s_jitter_ms_max = 0;
    s_prev_rtt_ms = -1;
    s_rtt_sum_ms = 0;
    s_jitter_sum_ms = 0;
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_send_ts_us, 0, sizeof(s_send_ts_us));
    s_last_probe_ms = 0;
}

void rtt_probe_tick(int64_t now_ms, bool mesh_active, bool bridge_connected)
{
    if (mesh_active && bridge_connected && (now_ms - s_last_probe_ms) >= RTT_PROBE_INTERVAL_MS) {
        send_probe();
        s_last_probe_ms = now_ms;
    }

    housekeeping();
}

bool rtt_probe_handle_packet(uint8_t src_id, const uint8_t *data, uint16_t len)
{
    (void)src_id;

    if (!data || len != RTT_PKT_LEN) {
        return false;
    }

    if (data[1] != RTT_MAGIC0 || data[2] != RTT_MAGIC1 || data[3] != RTT_MAGIC2 ||
        data[4] != RTT_MAGIC3) {
        return false;
    }

    uint8_t type = data[5];
    uint8_t id = data[6];

    if (type == RTT_TYPE_REQ) {
        uint8_t rsp[RTT_PKT_LEN + 1] = {
            0, ++s_diag_tx_seq, RTT_MAGIC0, RTT_MAGIC1, RTT_MAGIC2, RTT_MAGIC3, RTT_TYPE_RSP, id,
        };
        (void)uart_bridge_send_audio(rsp, sizeof(rsp));
        return true;
    }

    if (type == RTT_TYPE_RSP) {
        if (!s_pending[id]) {
            return true;
        }

        int64_t now_us = esp_timer_get_time();
        int64_t start_us = s_send_ts_us[id];
        s_pending[id] = false;

        if (start_us > 0 && now_us > start_us) {
            uint32_t rtt_ms = (uint32_t)((now_us - start_us) / 1000);

            s_recv++;
            s_rtt_sum_ms += rtt_ms;
            s_rtt_ms_avg = (uint32_t)(s_rtt_sum_ms / s_recv);
            if (rtt_ms > s_rtt_ms_max) {
                s_rtt_ms_max = rtt_ms;
            }

            if (s_prev_rtt_ms >= 0) {
                uint32_t jitter = (rtt_ms > (uint32_t)s_prev_rtt_ms)
                                      ? (rtt_ms - (uint32_t)s_prev_rtt_ms)
                                      : ((uint32_t)s_prev_rtt_ms - rtt_ms);
                s_jitter_sum_ms += jitter;
                s_jitter_ms_avg = (uint32_t)(s_jitter_sum_ms / s_recv);
                if (jitter > s_jitter_ms_max) {
                    s_jitter_ms_max = jitter;
                }
            }
            s_prev_rtt_ms = rtt_ms;
        }
        return true;
    }

    return false;
}

void rtt_probe_get_stats(rtt_probe_stats_t *stats)
{
    if (!stats) {
        return;
    }

    stats->sent = s_sent;
    stats->recv = s_recv;
    stats->lost = s_lost;
    stats->rtt_ms_avg = s_rtt_ms_avg;
    stats->rtt_ms_max = s_rtt_ms_max;
    stats->jitter_ms_avg = s_jitter_ms_avg;
    stats->jitter_ms_max = s_jitter_ms_max;
}
