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

#define SCAN_TIMEOUT_MS    2000
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
#define TX_AUDIO_RING_SIZE 8

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
    hdr->hop = 2;
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
            uart_bridge_send_audio(hdr->src_id, audio->data, audio_len);
            s_stat_audio_fwd++;
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

    /* Log packet stats */
    esb_radio_timing_stats_t esb_stats = {0};
    esb_radio_get_timing_stats(&esb_stats);

    if ((s_status_log_decim++ % 2) == 0) {
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

    /* Coordinator sends SYNC on every status update for discovery */
    if (s_role == MESH_ROLE_COORDINATOR) {
        send_sync();
    } else if (s_role == MESH_ROLE_PARTICIPANT) {
        /* Check for coordinator timeout */
        if (k_uptime_get_32() - s_last_sync_time > SYNC_TIMEOUT_MS) {
            LOG_WRN("Coordinator lost (timeout), rescanning...");
            mesh_log("MESH: Coordinator lost, rescanning");

            /* Notify ESP32 that we lost connection */
            uart_bridge_send_event(0x03, NULL, 0); /* BRIDGE_EVENT_PEER_LEFT */

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

static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter)
{
    /* Called by TDMA when it's our slot - send audio if queued */
    ARG_UNUSED(slot_index);
    ARG_UNUSED(frame_counter);

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
        } else {
            s_stat_tx_fail++;
        }

        /* always consume */
        s_tx_tail = (s_tx_tail + 1) % TX_AUDIO_RING_SIZE;

        /* Flow control: check buffer level AFTER consuming.
         * Use proportional braking to match ESP32 audio production rate.
         */
        uint8_t buf_count;
        if (s_tx_head >= s_tx_tail) {
            buf_count = s_tx_head - s_tx_tail;
        } else {
            buf_count = TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
        }
        s_tx_queue_depth_dbg = buf_count;

        if (buf_count == 0) {
            /* Just drained the last packet - slow down next slot a bit. */
            s_stat_tx_underflow++;
            tdma_tune_timing((s_under_delta_last > 100) ? 600 : 1200);
        } else if (buf_count == 1) {
            tdma_tune_timing((s_under_delta_last > 100) ? 300 : 600);
        } else if (buf_count < 4) {
            tdma_tune_timing(100);
        }

    } else {
        uint32_t now = k_uptime_get_32();
        bool has_audio_source = (now - s_last_audio_in_time) < 100;

        if (has_audio_source) {
            /* Calculate number of packets in buffer */
            uint8_t count;
            if (s_tx_head >= s_tx_tail) {
                count = s_tx_head - s_tx_tail;
            } else {
                count = TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
            }
            s_tx_queue_depth_dbg = count;

            if (count == 0) {
                s_stat_tx_underflow++;
                tdma_tune_timing((s_under_delta_last > 100) ? 600 : 1200);
            } else if (count == 1) {
                tdma_tune_timing((s_under_delta_last > 100) ? 300 : 600);
            } else if (count < 4) {
                tdma_tune_timing(100);
            }
        }
    }
}

int mesh_protocol_send_audio(const uint8_t *data, uint8_t len)
{
    if (len > MESH_MAX_AUDIO_PAYLOAD) {
        return -EMSGSIZE;
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
