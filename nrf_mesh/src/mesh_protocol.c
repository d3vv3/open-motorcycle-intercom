/**
 * @file mesh_protocol.c
 * @brief Mesh Protocol State Machine
 *
 * Ported from ESP32 mesh.c - handles join/leave, coordinator election, peer tracking
 */

#include "mesh_protocol.h"

#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "esb_radio.h"
#include "tdma.h"
#include "uart_bridge.h"
#include "ws_sync.h"

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

/* ============================================================================
 * Debug Log Forwarding to ESP32
 * ============================================================================ */

/* Helper to send log to ESP32 via SPI bridge */
static void mesh_log(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintk(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0 && uart_bridge_is_initialized()) {
        uart_bridge_send_log(buf, (uint8_t)len);
    }
}

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SCAN_TIMEOUT_MS    3000
#define JOIN_TIMEOUT_MS    5000
#define JOIN_RETRY_MS      500
#define JOIN_RETRY_COUNT   10
#define STATUS_INTERVAL_MS 1000

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static mesh_state_t s_state = MESH_STATE_IDLE;
static mesh_role_t s_role = MESH_ROLE_NONE;
static uint8_t s_node_id = 0;
static int8_t s_slot_index = -1;
static uint8_t s_coordinator_id = 0;

static uint8_t s_local_addr[5];
static mesh_peer_info_t s_peers[MESH_MAX_NODES];
static uint8_t s_peer_count = 0;

static uint8_t s_tx_seq = 0;

/* Check for lost coordinator */
static uint32_t s_last_sync_time = 0;
#define SYNC_TIMEOUT_MS 5000 /* 5 seconds timeout to allow for some packet loss */

/* Work queue items */
static struct k_work_delayable s_scan_work;
static struct k_work_delayable s_join_work;
static struct k_work_delayable s_status_work;
static struct k_work s_rx_work; /* For deferred RX processing */

static int s_join_attempts = 0;

/* RX ring buffer for deferred processing (replaces single-packet buffer) */
#define RX_RING_SIZE 8
struct rx_ring_entry {
    uint8_t data[256];
    uint8_t len;
    int8_t rssi;
};
static struct rx_ring_entry s_rx_ring[RX_RING_SIZE];
static volatile uint8_t s_rx_ring_head = 0; /* ISR writes here */
static volatile uint8_t s_rx_ring_tail = 0; /* Work reads here */

/* Packet statistics */
static uint32_t s_stat_tx_count = 0;
static uint32_t s_stat_tx_fail = 0;
static uint32_t s_stat_tx_underflow = 0;

/* TX queue depth for ESP->nRF audio handoff */
#define TX_AUDIO_RING_SIZE 16

static uint32_t s_stat_rx_count = 0;
static uint32_t s_stat_rx_drop = 0;
static uint32_t s_stat_audio_fwd = 0;
static uint32_t s_stat_tx_overwrite = 0;  /* Audio overwritten before TDMA sent */
static uint32_t s_stat_spi_audio_in = 0;  /* Audio packets received from ESP32 SPI */
static uint32_t s_last_audio_in_time = 0; /* Timestamp of last audio packet from ESP32 */
static uint8_t s_tx_queue_depth_dbg = 0;  /* Current TX queue depth for diagnostics */
static uint32_t s_under_prev = 0;
static uint32_t s_under_delta_last = 0;
static uint32_t s_status_log_decim = 0;
static uint32_t s_under_log_next = 64;

/* End-to-end sequence diagnostics.
 * Sequence is injected by ESP in first 2 bytes of Opus payload and forwarded
 * unchanged over nRF mesh and back to ESP. */
struct e2e_src_state {
    bool init;
    uint16_t last_seq;
};

static struct e2e_src_state s_e2e_spi_in_src[256];
static struct e2e_src_state s_e2e_rf_rx_src[256];
static uint32_t s_e2e_spi_in_frames = 0;
static uint32_t s_e2e_spi_in_gap_evt = 0;
static uint32_t s_e2e_spi_in_gap_fr = 0;
static uint32_t s_e2e_spi_in_reset_evt = 0;
static uint32_t s_e2e_rf_tx_frames = 0;
static uint32_t s_e2e_rf_rx_frames = 0;
static uint32_t s_e2e_rf_rx_gap_evt = 0;
static uint32_t s_e2e_rf_rx_gap_fr = 0;
static uint32_t s_e2e_rf_rx_reset_evt = 0;
static uint32_t s_e2e_spi_out_frames = 0;

#define E2E_MAX_FORWARD_GAP 32
/* Skip-send queue buffering strategy.
 *
 * Problem: PI frame-stretch matched send rate to arrival rate but never built
 * queue depth — any jitter caused immediate underflow.
 *
 * New approach: hold back (skip sending) when queue is below a low-water mark
 * until it fills to a high-water target, then resume normal 1-per-slot sending.
 * This is prefill + hysteresis.
 */
#define QUEUE_LO          2    /* Resume skip when queue drops below this */
#define QUEUE_HI          3    /* Resume send when queue reaches this */

static bool     s_skip_mode = true;     /* Start in skip (prefill) mode */
static uint32_t s_skip_count = 0;       /* Frames skipped for logging */
static uint32_t s_auto_ticks = 0;       /* Status log counter */

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void scan_work_handler(struct k_work *work);
static void join_work_handler(struct k_work *work);
static void status_work_handler(struct k_work *work);
static void rx_work_handler(struct k_work *work);
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi);
static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi);
static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter);

static void note_underflow(const char *reason)
{
    if (s_stat_tx_underflow >= s_under_log_next) {
        int32_t tts = tdma_get_time_to_slot_us();
        printk("[UFLOW] r=%u id=%u sl=%d under=%u d1s=%u q=%u tts=%d reason=%s\n", s_role,
               s_node_id, s_slot_index, s_stat_tx_underflow, s_under_delta_last,
               s_tx_queue_depth_dbg, tts, reason);
        s_under_log_next += 64;
    }
}

/* ============================================================================
 * Packet Sending
 * ============================================================================ */

static int send_packet(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    uint8_t buf[sizeof(mesh_header_t) + 128];
    mesh_header_t *hdr = (mesh_header_t *)buf;

    hdr->version = MESH_PROTOCOL_VERSION;
    hdr->type = type;
    hdr->src_id = s_node_id;
    hdr->seq = s_tx_seq++;
    hdr->reserved0 = 0;
    hdr->flags = 0;
    hdr->payload_len = len;

    if (payload && len > 0) {
        memcpy(buf + sizeof(mesh_header_t), payload, len);
    }

    return esb_radio_send(buf, sizeof(mesh_header_t) + len);
}

static int send_join_request(void)
{
    mesh_join_payload_t payload = {
        .capabilities = 0x01, /* Has audio */
        .reserved = 0,
    };
    LOG_INF("Sending JOIN request");
    return send_packet(MESH_PKT_JOIN, &payload, sizeof(payload));
}

static int send_sync(void)
{
    mesh_sync_payload_t payload = {
        .frame_counter = tdma_get_frame_counter(),
        .drift_ppm = 0,
    };
    return send_packet(MESH_PKT_SYNC, &payload, sizeof(payload));
}

static int send_keepalive(void)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 255,
        .reserved = 0,
    };
    return send_packet(MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
}

static int send_slot_map(void)
{
    mesh_slot_map_payload_t payload = {0};
    payload.slot_count = MESH_MAX_NODES;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active && s_peers[i].slot_index >= 0 && s_peers[i].slot_index < MESH_MAX_NODES) {
            payload.slot_ids[s_peers[i].slot_index] = s_peers[i].node_id;
        }
    }

    return send_packet(MESH_PKT_SLOT_MAP, &payload, sizeof(payload));
}

static int send_status_packet(void)
{
    mesh_status_payload_t payload = {
        .battery_pct = 255,
        .rssi_dbm = 127,
        .peer_count = s_peer_count,
        .fw_version = MESH_PROTOCOL_VERSION,
        .temperature_c = 127,
        .reserved = 0,
    };
    return send_packet(MESH_PKT_STATUS, &payload, sizeof(payload));
}

static int send_join_ack(uint8_t assigned_id, uint8_t slot_index)
{
    mesh_join_ack_payload_t payload = {
        .assigned_id = assigned_id,
        .slot_index = slot_index,
        .coordinator_id = s_node_id,
    };
    LOG_INF("Sending JOIN_ACK: id=%d, slot=%d", assigned_id, slot_index);
    return send_packet(MESH_PKT_JOIN_ACK, &payload, sizeof(payload));
}

/* ============================================================================
 * Packet Handling (deferred to thread context)
 * ============================================================================ */

/* ISR-safe callback - copies data to ring buffer and submits work */
static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi)
{
    ARG_UNUSED(src_addr);

    /* NOTE: Don't printk here - ISR context, can deadlock with USB/UART */

    uint8_t next_head = (s_rx_ring_head + 1) % RX_RING_SIZE;
    if (next_head == s_rx_ring_tail) {
        /* Ring full - drop packet */
        s_stat_rx_drop++;
        return;
    }

    if (len > sizeof(s_rx_ring[0].data)) {
        return;
    }

    struct rx_ring_entry *entry = &s_rx_ring[s_rx_ring_head];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->rssi = rssi;
    s_rx_ring_head = next_head;

    /* Submit work to process in thread context */
    k_work_submit(&s_rx_work);
}

/* Work handler - drains all pending packets from ring buffer */
static void rx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    while (s_rx_ring_tail != s_rx_ring_head) {
        struct rx_ring_entry *entry = &s_rx_ring[s_rx_ring_tail];
        process_rx_packet(entry->data, entry->len, entry->rssi);
        s_rx_ring_tail = (s_rx_ring_tail + 1) % RX_RING_SIZE;
    }
}

/* Actual packet processing - safe to call kernel functions here */
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi)
{
    if (len < sizeof(mesh_header_t)) {
        return;
    }

    const mesh_header_t *hdr = (const mesh_header_t *)data;
    const uint8_t *payload = data + sizeof(mesh_header_t);

    if (hdr->version != MESH_PROTOCOL_VERSION) {
        return;
    }

    LOG_DBG("RX type=0x%02X src=%d seq=%d (RSSI=%d)", hdr->type, hdr->src_id, hdr->seq, rssi);

    s_stat_rx_count++;

    switch (hdr->type) {
    case MESH_PKT_AUDIO:
        if (s_state == MESH_STATE_ACTIVE && hdr->payload_len > 4) {
            /* Forward audio to ESP32 via UART */
            const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)payload;
            uint8_t audio_len = hdr->payload_len - 4; /* Subtract header */

            if (audio_len >= 2) {
                uint16_t e2e_seq = ((uint16_t)audio->data[0] << 8) | audio->data[1];
                struct e2e_src_state *st = &s_e2e_rf_rx_src[hdr->src_id];
                if (!st->init) {
                    st->init = true;
                } else {
                    uint16_t expected = (uint16_t)(st->last_seq + 1);
                    if (e2e_seq != expected) {
                        int16_t signed_delta = (int16_t)(e2e_seq - expected);
                        if (signed_delta > 0 && signed_delta <= E2E_MAX_FORWARD_GAP) {
                            s_e2e_rf_rx_gap_evt++;
                            s_e2e_rf_rx_gap_fr += (uint16_t)signed_delta;
                        } else {
                            s_e2e_rf_rx_reset_evt++;
                        }
                    }
                }
                st->last_seq = e2e_seq;
                s_e2e_rf_rx_frames++;
            }

            uart_bridge_send_audio(hdr->src_id, audio->data, audio_len);
            s_stat_audio_fwd++;
            s_e2e_spi_out_frames++;
        }
        break;

    case MESH_PKT_SYNC:
        if (s_state == MESH_STATE_SCANNING) {
            /* Found existing mesh */
            LOG_INF("Found mesh, coordinator=%d", hdr->src_id);
            s_coordinator_id = hdr->src_id;
            s_state = MESH_STATE_JOINING;
            s_join_attempts = 0;
            k_work_cancel_delayable(&s_scan_work);
            k_work_schedule(&s_join_work, K_NO_WAIT);
        } else if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT) {
            /* Sync to coordinator timing */
            const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)payload;
            tdma_sync(sync->frame_counter, sync->drift_ppm);
            s_last_sync_time = k_uptime_get_32();
        }
        break;

    case MESH_PKT_JOIN:
        if (s_role == MESH_ROLE_COORDINATOR) {
            /* Deduplication: if we recently assigned a slot and the new peer
             * hasn't confirmed (sent KEEPALIVE), assume this is a retry from
             * the same joiner and resend the previous JOIN_ACK. */
            static uint8_t s_last_join_id = 0;
            static int8_t s_last_join_slot = -1;
            static int64_t s_last_join_time = 0;

            int64_t now = k_uptime_get();
            if (s_last_join_id != 0 && (now - s_last_join_time) < 10000) {
                /* Check if the last assigned peer ever sent anything back */
                bool last_peer_confirmed = false;
                for (int i = 0; i < MESH_MAX_NODES; i++) {
                    if (s_peers[i].active && s_peers[i].node_id == s_last_join_id) {
                        /* If last_seen hasn't changed from assignment time,
                         * it's the same joiner retrying */
                        if ((now - s_peers[i].last_seen_ms) < 10000) {
                            last_peer_confirmed = false;
                        }
                        break;
                    }
                }

                if (!last_peer_confirmed) {
                    LOG_INF("JOIN retry, resending ACK: id=%d slot=%d", s_last_join_id,
                            s_last_join_slot);
                    send_join_ack(s_last_join_id, s_last_join_slot);
                    send_slot_map();
                    break;
                }
            }

            /* Find next available slot and ID for new node */
            uint8_t new_id = 0;
            int8_t new_slot = -1;

            /* Find first free slot (start at 1, slot 0 is coordinator) */
            for (int i = 1; i < MESH_MAX_NODES; i++) {
                bool slot_used = false;
                for (int j = 0; j < MESH_MAX_NODES; j++) {
                    if (s_peers[j].active && s_peers[j].slot_index == i) {
                        slot_used = true;
                        break;
                    }
                }
                if (!slot_used) {
                    new_slot = i;
                    new_id = i + 1; /* ID = slot + 1 */
                    break;
                }
            }

            if (new_slot < 0) {
                LOG_WRN("No free slots for new node");
                break;
            }

            LOG_INF("Received JOIN, assigning id=%d slot=%d", new_id, new_slot);

            /* Add new peer to our list */
            for (int i = 0; i < MESH_MAX_NODES; i++) {
                if (!s_peers[i].active) {
                    s_peers[i].node_id = new_id;
                    s_peers[i].slot_index = new_slot;
                    s_peers[i].active = true;
                    s_peers[i].last_seen_ms = k_uptime_get();
                    s_peer_count++;
                    break;
                }
            }

            /* Track this assignment for dedup */
            s_last_join_id = new_id;
            s_last_join_slot = new_slot;
            s_last_join_time = k_uptime_get();

            /* Send JOIN_ACK to the new node */
            send_join_ack(new_id, new_slot);
            send_slot_map();

            /* Notify ESP32 of new peer joining */
            uart_bridge_send_event(0x02, NULL, 0); /* BRIDGE_EVENT_PEER_JOINED */
        }
        break;

    case MESH_PKT_JOIN_ACK:
        if (s_state == MESH_STATE_JOINING) {
            const mesh_join_ack_payload_t *ack = (const mesh_join_ack_payload_t *)payload;
            s_node_id = ack->assigned_id;
            s_slot_index = ack->slot_index;
            s_coordinator_id = ack->coordinator_id;

            LOG_INF("JOIN_ACK: node_id=%d, slot=%d", s_node_id, s_slot_index);

            s_state = MESH_STATE_ACTIVE;
            s_role = MESH_ROLE_PARTICIPANT;

            k_work_cancel_delayable(&s_join_work);
            tdma_start(s_slot_index);

            /* Start periodic status updates */
            s_last_sync_time = k_uptime_get_32();
            k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));

            /* Notify ESP32 that we joined the mesh */
            uart_bridge_send_event(0x02, NULL, 0); /* BRIDGE_EVENT_PEER_JOINED */
        }
        break;

    case MESH_PKT_KEEPALIVE:
        /* Update peer last-seen */
        for (int i = 0; i < MESH_MAX_NODES; i++) {
            if (s_peers[i].node_id == hdr->src_id) {
                s_peers[i].last_seen_ms = k_uptime_get();
                break;
            }
        }
        break;

    case MESH_PKT_STATUS:
        if (hdr->payload_len >= sizeof(mesh_status_payload_t)) {
            const mesh_status_payload_t *status = (const mesh_status_payload_t *)payload;
            for (int i = 0; i < MESH_MAX_NODES; i++) {
                if (s_peers[i].active && s_peers[i].node_id == hdr->src_id) {
                    s_peers[i].battery_pct = status->battery_pct;
                    s_peers[i].rssi_dbm = rssi;
                    s_peers[i].peer_count = status->peer_count;
                    s_peers[i].fw_version = status->fw_version;
                    s_peers[i].temperature_c = status->temperature_c;
                    s_peers[i].last_seen_ms = k_uptime_get();
                    break;
                }
            }
        }
        break;

    case MESH_PKT_SLOT_MAP:
        if (hdr->payload_len >= sizeof(uint8_t)) {
            const mesh_slot_map_payload_t *slot_map = (const mesh_slot_map_payload_t *)payload;
            uint8_t slot_count = slot_map->slot_count;
            if (slot_count > MESH_MAX_NODES) {
                slot_count = MESH_MAX_NODES;
            }

            if (hdr->payload_len < (uint16_t)(1 + slot_count)) {
                break;
            }

            for (uint8_t slot = 0; slot < slot_count; slot++) {
                uint8_t node_id = slot_map->slot_ids[slot];
                if (node_id == 0) {
                    continue;
                }

                if (node_id == s_node_id) {
                    s_slot_index = (int8_t)slot;
                }

                for (int i = 0; i < MESH_MAX_NODES; i++) {
                    if (s_peers[i].active && s_peers[i].node_id == node_id) {
                        s_peers[i].slot_index = (int8_t)slot;
                        break;
                    }
                }
            }
        }
        break;

    case MESH_PKT_LEAVE:
        for (int i = 0; i < MESH_MAX_NODES; i++) {
            if (s_peers[i].active && s_peers[i].node_id == hdr->src_id) {
                s_peers[i].active = false;
                if (s_peer_count > 0) {
                    s_peer_count--;
                }
                LOG_INF("Peer %u left, remaining peers: %u", hdr->src_id, s_peer_count);

                /* Notify ESP32 so it can play a disconnect tone */
                uart_bridge_send_event(0x03, NULL, 0); /* BRIDGE_EVENT_PEER_LEFT */

                break;
            }
        }
        if (s_role == MESH_ROLE_COORDINATOR) {
            send_slot_map();
        }
        break;

    default:
        break;
    }
}

/* ============================================================================
 * Work Handlers
 * ============================================================================ */

static void scan_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (s_state != MESH_STATE_SCANNING) {
        return;
    }

    /* Timeout - become coordinator */
    LOG_INF("No mesh found, becoming coordinator");

    s_role = MESH_ROLE_COORDINATOR;
    s_node_id = 1;
    s_slot_index = 0;
    s_coordinator_id = 1;
    s_state = MESH_STATE_ACTIVE;

    /* Add self to peer list */
    s_peers[0].node_id = 1;
    esb_radio_get_address(s_peers[0].esb_addr);
    s_peers[0].slot_index = 0;
    s_peers[0].active = true;
    s_peers[0].last_seen_ms = k_uptime_get();
    s_peer_count = 1;

    /* RX is already running from scanning phase */

    /* Start TDMA */
    tdma_start(s_slot_index);

    /* Start periodic status updates */
    k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));

    LOG_INF("ACTIVE as coordinator, node_id=%d, slot=%d", s_node_id, s_slot_index);
}

static void join_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (s_state != MESH_STATE_JOINING) {
        return;
    }

    if (s_join_attempts < JOIN_RETRY_COUNT) {
        send_join_request();
        s_join_attempts++;
        LOG_INF("JOIN attempt %d/%d", s_join_attempts, JOIN_RETRY_COUNT);
        mesh_log("MESH: JOIN attempt %d/%d", s_join_attempts, JOIN_RETRY_COUNT);
        k_work_schedule(&s_join_work, K_MSEC(JOIN_RETRY_MS));
    } else {
        /* Give up and become coordinator */
        LOG_WRN("JOIN timeout, becoming coordinator");
        mesh_log("MESH: JOIN timeout, becoming coord");
        s_state = MESH_STATE_SCANNING;
        k_work_schedule(&s_scan_work, K_NO_WAIT);
    }
}

static void status_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (s_state != MESH_STATE_ACTIVE) {
        return;
    }

    /* Send status to ESP32 */
    uart_bridge_send_status(s_role, s_peer_count, s_node_id);

    /* Send mesh status and keepalive over RF */
    send_status_packet();
    send_keepalive();

    /* Log packet stats */
    esb_radio_timing_stats_t esb_stats = {0};
    esb_radio_get_timing_stats(&esb_stats);

    bool log_now = ((s_status_log_decim++ % 2) == 0);
    if (log_now) {
        printk("[MESH] r=%u id=%u sl=%d tx=%u(err=%u) rx=%u drop=%u fwd=%u | spi_in=%u overwr=%u under=%u q=%u\n",
               s_role, s_node_id, s_slot_index, s_stat_tx_count, s_stat_tx_fail, s_stat_rx_count,
               s_stat_rx_drop, s_stat_audio_fwd, s_stat_spi_audio_in, s_stat_tx_overwrite,
               s_stat_tx_underflow, s_tx_queue_depth_dbg);
        printk("[ESB_TIM] tx=%u to=%u txwait=%u/%u rxpause=%u/%u us\n", esb_stats.tx_count,
               esb_stats.tx_timeout_count, esb_stats.tx_wait_us_avg, esb_stats.tx_wait_us_max,
               esb_stats.rx_pause_us_avg, esb_stats.rx_pause_us_max);
    }

    s_under_delta_last = s_stat_tx_underflow - s_under_prev;
    s_under_prev = s_stat_tx_underflow;

    s_auto_ticks++;

    if (log_now) {
        uint32_t ws_edges = 0;
        int32_t ws_last_corr = 0, ws_cum_drift = 0;
        ws_sync_get_diag(&ws_edges, &ws_last_corr, &ws_cum_drift);
        printk("[ATUNE] r=%u id=%u q=%u under_d=%u skip=%u/%u ws_e=%u ws_c=%d ws_d=%d\n",
               s_role, s_node_id, s_tx_queue_depth_dbg, s_under_delta_last,
               s_skip_count, s_auto_ticks,
               ws_edges, ws_last_corr, ws_cum_drift);
        printk("[E2E_NRF] id=%u spi_in=%u spi_gap=%u/%u spi_reset=%u rf_tx=%u rf_rx=%u rf_gap=%u/%u rf_reset=%u spi_out=%u\n",
               s_node_id,
               s_e2e_spi_in_frames, s_e2e_spi_in_gap_evt, s_e2e_spi_in_gap_fr, s_e2e_spi_in_reset_evt,
               s_e2e_rf_tx_frames, s_e2e_rf_rx_frames, s_e2e_rf_rx_gap_evt, s_e2e_rf_rx_gap_fr,
               s_e2e_rf_rx_reset_evt,
               s_e2e_spi_out_frames);
    }

    /* Coordinator sends SYNC on every status update for discovery */
    if (s_role == MESH_ROLE_COORDINATOR) {
        static uint8_t sync_decim = 0;
        if ((sync_decim++ % 2) == 0) {
            send_sync();
        }
    } else if (s_role == MESH_ROLE_PARTICIPANT) {
        /* Check for coordinator timeout */
        if (k_uptime_get_32() - s_last_sync_time > SYNC_TIMEOUT_MS) {
            LOG_WRN("Coordinator lost (timeout), rescanning...");
            mesh_log("MESH: Coordinator lost, rescanning");

            /* Notify ESP32 that coordinator sync was lost.
             * Do NOT emit PEER_LEFT here: this path can be transient (role
             * transition/rescan) and audio may continue, which caused false
             * disconnect tones on ESP. */
            uart_bridge_send_event(0x05, NULL, 0); /* BRIDGE_EVENT_SYNC_LOST */

            /* Stop TDMA */
            tdma_stop();

            /* Return to scanning */
            s_state = MESH_STATE_SCANNING;
            s_role = MESH_ROLE_NONE; // Reset role
            s_node_id = 0;
            s_slot_index = -1;

            k_work_schedule(&s_scan_work, K_NO_WAIT);
            return; /* Don't reschedule status work */
        }
    }

    /* Reschedule */
    k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
}

struct tx_audio_entry {
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD];
    uint8_t len;
};

static struct tx_audio_entry s_tx_audio_ring[TX_AUDIO_RING_SIZE];
static volatile uint8_t s_tx_head = 0;
static volatile uint8_t s_tx_tail = 0;

static uint8_t tx_queue_depth(void)
{
    if (s_tx_head >= s_tx_tail) {
        return s_tx_head - s_tx_tail;
    }
    return TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
}

static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter)
{
    /* Called by TDMA when it's our slot - send audio if queued */
    ARG_UNUSED(slot_index);
    ARG_UNUSED(frame_counter);

    uint32_t now = k_uptime_get_32();
    bool has_audio_source = (now - s_last_audio_in_time) < 120;

    uint8_t depth = tx_queue_depth();
    s_tx_queue_depth_dbg = depth;

    /* Skip-send hysteresis: stay in skip mode until queue reaches
     * QUEUE_HI, then send until it drops below QUEUE_LO.
     * This rate-matches nRF output (~43/s from ESP) to TDMA slots,
     * preventing TX ring underflows (under_d=0). */
    if (s_skip_mode) {
        if (depth >= QUEUE_HI) {
            s_skip_mode = false;
        } else {
            return;
        }
    } else if (depth < QUEUE_LO) {
        s_skip_mode = true;
        return;
    }

    if (s_state == MESH_STATE_ACTIVE && s_tx_head != s_tx_tail) {
        /* Peek at tail */
        struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_tail];

        mesh_audio_payload_t payload = {
            .codec = 1, /* Opus */
            .frame_ms = 20,
            .stream_id = s_node_id,
            .reserved = 0,
        };

        if (entry->len > MESH_MAX_AUDIO_PAYLOAD) {
            entry->len = MESH_MAX_AUDIO_PAYLOAD;
        }

        memcpy(payload.data, entry->data, entry->len);

        /* Send packet */
        int ret = send_packet(MESH_PKT_AUDIO, &payload, 4 + entry->len);

        if (ret == 0) {
            s_stat_tx_count++;
            s_e2e_rf_tx_frames++;
        } else {
            s_stat_tx_fail++;
        }

        /* always consume */
        s_tx_tail = (s_tx_tail + 1) % TX_AUDIO_RING_SIZE;

        /* Update queue depth after consuming */
        depth = tx_queue_depth();
        s_tx_queue_depth_dbg = depth;

        if (depth == 0) {
            s_stat_tx_underflow++;
            note_underflow("drain0");
        }

    } else {
        if (has_audio_source && depth == 0) {
            s_stat_tx_underflow++;
            note_underflow("empty");
        }
    }
}

int mesh_protocol_send_audio(const uint8_t *data, uint8_t len)
{
    if (len > MESH_MAX_AUDIO_PAYLOAD) {
        return -EMSGSIZE;
    }

    if (len >= 2) {
        uint16_t e2e_seq = ((uint16_t)data[0] << 8) | data[1];
        struct e2e_src_state *st = &s_e2e_spi_in_src[s_node_id];
        if (!st->init) {
            st->init = true;
        } else {
            uint16_t expected = (uint16_t)(st->last_seq + 1);
            if (e2e_seq != expected) {
                int16_t signed_delta = (int16_t)(e2e_seq - expected);
                if (signed_delta > 0 && signed_delta <= E2E_MAX_FORWARD_GAP) {
                    s_e2e_spi_in_gap_evt++;
                    s_e2e_spi_in_gap_fr += (uint16_t)signed_delta;
                } else {
                    s_e2e_spi_in_reset_evt++;
                }
            }
        }
        st->last_seq = e2e_seq;
        s_e2e_spi_in_frames++;
    }

    s_stat_spi_audio_in++;
    s_last_audio_in_time = k_uptime_get_32();

    uint8_t next_head = (s_tx_head + 1) % TX_AUDIO_RING_SIZE;

    if (next_head == s_tx_tail) {
        /* Buffer full: drop oldest and keep newest to bound latency growth. */
        s_tx_tail = (s_tx_tail + 1) % TX_AUDIO_RING_SIZE;
        s_stat_tx_overwrite++;
    }

    struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_head];
    memcpy(entry->data, data, len);
    entry->len = len;

    s_tx_head = next_head;

    if (s_tx_head >= s_tx_tail) {
        s_tx_queue_depth_dbg = s_tx_head - s_tx_tail;
    } else {
        s_tx_queue_depth_dbg = TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
    }

    return 0;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

int mesh_protocol_init(void)
{
    LOG_INF("Initializing mesh protocol");

    /* Initialize work items */
    k_work_init_delayable(&s_scan_work, scan_work_handler);
    k_work_init_delayable(&s_join_work, join_work_handler);
    k_work_init_delayable(&s_status_work, status_work_handler);
    k_work_init(&s_rx_work, rx_work_handler);

    /* Set callbacks */
    esb_radio_set_rx_callback(esb_rx_callback);
    tdma_set_slot_callback(slot_tx_handler);

    /* Get local address */
    esb_radio_get_address(s_local_addr);

    s_state = MESH_STATE_IDLE;
    return 0;
}

int mesh_protocol_start(void)
{
    if (s_state != MESH_STATE_IDLE) {
        return -EALREADY;
    }

    LOG_INF("Starting mesh protocol");
    printk("[MESH] mesh_protocol_start called\n");

    /* Start RX so we can hear SYNC broadcasts from an existing coordinator */
    esb_radio_start_rx();

    /* Enter scanning state */
    s_state = MESH_STATE_SCANNING;
    printk("[MESH] Scanning for existing mesh (%dms timeout)...\n", SCAN_TIMEOUT_MS);
    k_work_schedule(&s_scan_work, K_MSEC(SCAN_TIMEOUT_MS));

    printk("[MESH] mesh_protocol_start complete\n");
    return 0;
}

void mesh_protocol_stop(void)
{
    LOG_INF("Stopping mesh protocol");

    k_work_cancel_delayable(&s_scan_work);
    k_work_cancel_delayable(&s_join_work);
    k_work_cancel_delayable(&s_status_work);

    if (s_state == MESH_STATE_ACTIVE && s_node_id != 0) {
        send_packet(MESH_PKT_LEAVE, NULL, 0);
        /* Notify ESP32 that we are leaving the mesh */
        uart_bridge_send_event(0x03, NULL, 0); /* BRIDGE_EVENT_PEER_LEFT */
    }

    tdma_stop();
    esb_radio_stop_rx();

    s_state = MESH_STATE_IDLE;
    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
}

mesh_state_t mesh_protocol_get_state(void)
{
    return s_state;
}

mesh_role_t mesh_protocol_get_role(void)
{
    return s_role;
}

uint8_t mesh_protocol_get_node_id(void)
{
    return s_node_id;
}

uint8_t mesh_protocol_get_peer_count(void)
{
    return s_peer_count;
}
