/**
 * @file mesh_protocol.c
 * @brief Mesh Protocol State Machine
 *
 * Ported from ESP32 mesh.c - handles join/leave, coordinator election, peer tracking
 */

#include "mesh_protocol.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "esb_radio.h"
#include "tdma.h"
#include "uart_bridge.h"

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

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

/* Work queue items */
static struct k_work_delayable s_scan_work;
static struct k_work_delayable s_join_work;
static struct k_work_delayable s_status_work;

static int s_join_attempts = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void scan_work_handler(struct k_work *work);
static void join_work_handler(struct k_work *work);
static void status_work_handler(struct k_work *work);
static void handle_rx_packet(const uint8_t *data, uint8_t len, const uint8_t *src_addr,
                             int8_t rssi);
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

/* ============================================================================
 * Packet Handling
 * ============================================================================ */

static void handle_rx_packet(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi)
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

    switch (hdr->type) {
    case MESH_PKT_AUDIO:
        if (s_state == MESH_STATE_ACTIVE && hdr->payload_len > 4) {
            /* Forward audio to ESP32 via UART */
            const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)payload;
            uint8_t audio_len = hdr->payload_len - 4; /* Subtract header */
            uart_bridge_send_audio(hdr->src_id, audio->data, audio_len);
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
        }
        break;

    case MESH_PKT_JOIN:
        if (s_role == MESH_ROLE_COORDINATOR) {
            /* Assign ID and slot to new node */
            /* TODO: Implement coordinator logic */
            LOG_INF("Received JOIN from new node");
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
            k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
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
        k_work_schedule(&s_join_work, K_MSEC(JOIN_RETRY_MS));
    } else {
        /* Give up and become coordinator */
        LOG_WRN("JOIN timeout, becoming coordinator");
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

    /* Coordinator sends SYNC periodically */
    if (s_role == MESH_ROLE_COORDINATOR) {
        if ((tdma_get_frame_counter() % MESH_SYNC_INTERVAL_FRAMES) == 0) {
            send_sync();
        }
    }

    /* Reschedule */
    k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
}

static uint8_t s_tx_audio_buf[MESH_MAX_AUDIO_PAYLOAD];
static uint8_t s_tx_audio_len = 0;
static bool s_tx_audio_ready = false;

static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter)
{
    /* Called by TDMA when it's our slot - send audio if queued */
    ARG_UNUSED(slot_index);
    ARG_UNUSED(frame_counter);

    if (s_state == MESH_STATE_ACTIVE && s_tx_audio_ready) {
        mesh_audio_payload_t payload = {
            .codec = 1, /* Opus */
            .frame_ms = 20,
            .stream_id = s_node_id,
            .reserved = 0,
        };

        if (s_tx_audio_len > MESH_MAX_AUDIO_PAYLOAD) {
            s_tx_audio_len = MESH_MAX_AUDIO_PAYLOAD;
        }

        memcpy(payload.data, s_tx_audio_buf, s_tx_audio_len);

        /* Calculate total payload length: header (4) + audio data */
        send_packet(MESH_PKT_AUDIO, &payload, 4 + s_tx_audio_len);

        s_tx_audio_ready = false;
    }
}

int mesh_protocol_send_audio(const uint8_t *data, uint8_t len)
{
    if (len > MESH_MAX_AUDIO_PAYLOAD) {
        return -EMSGSIZE;
    }

    /* Overwrite valid buffer (newest audio is best) */
    memcpy(s_tx_audio_buf, data, len);
    s_tx_audio_len = len;
    s_tx_audio_ready = true;

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

    /* Set callbacks */
    esb_radio_set_rx_callback(handle_rx_packet);
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

    /* Start RX */
    esb_radio_start_rx();

    /* Enter scanning state */
    s_state = MESH_STATE_SCANNING;
    k_work_schedule(&s_scan_work, K_MSEC(SCAN_TIMEOUT_MS));

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
