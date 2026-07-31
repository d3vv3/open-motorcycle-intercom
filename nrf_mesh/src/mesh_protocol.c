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
#include <zephyr/sys/atomic.h>

#include "esb_radio.h"
#include "rtt_probe_defs.h"
#include "tdma.h"
#include "uart_bridge.h"
#include "ws_sync.h"

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

static bool is_rtt_probe_payload(const uint8_t *data, uint8_t len)
{
    return len == RTT_PKT_LEN && data[1] == RTT_MAGIC0 && data[2] == RTT_MAGIC1 &&
           data[3] == RTT_MAGIC2 && data[4] == RTT_MAGIC3 &&
           (data[5] == RTT_TYPE_REQ || data[5] == RTT_TYPE_RSP);
}

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
#define ACTIVE_SPEAKER_TIMEOUT_MS 1500
#define SEEN_RING_SIZE     32
#define RELAY_RING_SIZE    16
#define CONTROL_RING_SIZE  32
#define TX_AUDIO_RING_SIZE 16
#define NRF_SYNC_RX_LATENCY_US 300
#define MESH_PACKET_PAYLOAD_MAX 128

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

/* The system workqueue owns all mesh protocol state. SPI commands are
 * coalesced atomically, audio is copied through a message queue, and timer or
 * radio ISR paths only submit work or publish into protected rings. */
static uint8_t s_active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
static uint8_t s_relay_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
static int64_t s_active_speaker_deadline_ms[MESH_MAX_NODES + 1] = {0};
static uint8_t s_heard_bitmap = 0;
static uint8_t s_relay_bitmap = 0;

/* Check for lost coordinator */
static uint32_t s_last_sync_time = 0;
#define SYNC_TIMEOUT_MS 5000 /* 5 seconds timeout to allow for some packet loss */

/* Work queue items */
static struct k_work_delayable s_scan_work;
static struct k_work_delayable s_join_work;
static struct k_work_delayable s_status_work;
static struct k_work s_rx_work; /* For deferred RX processing */
static struct k_work s_command_work;
static struct k_work s_audio_ingress_work;

struct audio_ingress_entry {
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD];
    uint8_t len;
    uint8_t audio_flags;
};

struct tx_audio_entry {
    uint8_t data[MESH_MAX_AUDIO_PAYLOAD];
    uint8_t len;
    uint8_t audio_flags;
};

K_MSGQ_DEFINE(s_audio_ingress_queue, sizeof(struct audio_ingress_entry), 16, 4);
K_MUTEX_DEFINE(s_audio_ingress_lock);
static bool s_audio_ingress_enabled = false;
static uint32_t s_stat_ingress_purge_drop = 0;
static struct tx_audio_entry s_tx_audio_ring[TX_AUDIO_RING_SIZE];
static uint8_t s_tx_head = 0;
static uint8_t s_tx_tail = 0;
static uint32_t s_stat_tx_purge_drop = 0;
static atomic_t s_requested_enabled;
static atomic_t s_control_pending;
static atomic_t s_status_pending;

static int s_join_attempts = 0;

/* RX ring buffer for deferred processing (replaces single-packet buffer) */
#define RX_RING_SIZE 8
struct rx_ring_entry {
    uint8_t data[256];
    uint8_t len;
    int8_t rssi;
    int64_t timestamp_us;
};
static struct rx_ring_entry s_rx_ring[RX_RING_SIZE];
static uint8_t s_rx_ring_head = 0;
static uint8_t s_rx_ring_tail = 0;
static struct k_spinlock s_rx_ring_lock;

/* Packet statistics */
static uint32_t s_stat_tx_count = 0;
static uint32_t s_stat_tx_fail = 0;
static uint32_t s_stat_tx_underflow = 0;

static uint32_t s_stat_rx_count = 0;
static atomic_t s_stat_rx_drop;
static uint32_t s_stat_audio_fwd = 0;
static atomic_t s_stat_tx_overwrite;
static uint32_t s_stat_spi_audio_in = 0;  /* Audio packets received from ESP32 SPI */
static uint32_t s_stat_ingress_inactive_drop = 0;
static atomic_t s_stat_ingress_msgq_drop;
static uint32_t s_stat_tx_ring_drop = 0;
static uint32_t s_stat_relay_ring_drop = 0;
static uint32_t s_stat_control_ring_drop = 0;
static uint32_t s_stat_rf_audio_try = 0;
static uint32_t s_stat_rf_audio_ok = 0;
static uint32_t s_stat_rf_audio_fail = 0;
static uint32_t s_stat_rf_rx_audio_ok = 0;
static uint32_t s_stat_rf_rx_malformed = 0;
static uint32_t s_stat_rf_rx_version_drop = 0;
static uint32_t s_stat_rf_rx_self_drop = 0;
static uint32_t s_stat_rf_rx_duplicate_drop = 0;
static uint32_t s_stat_rf_rx_inactive_drop = 0;
static uint32_t s_stat_spi_out_ok = 0;
static uint32_t s_stat_spi_out_drop = 0;
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

struct seen_entry {
    uint8_t type;
    uint8_t src_id;
    uint8_t seq;
    bool valid;
};

static struct seen_entry s_seen_ring[SEEN_RING_SIZE];
static uint8_t s_seen_head = 0;

struct relay_entry {
    uint8_t data[256];
    uint8_t len;
};

static struct relay_entry s_relay_ring[RELAY_RING_SIZE];
static uint8_t s_relay_head = 0;
static uint8_t s_relay_tail = 0;
static struct relay_entry s_control_ring[CONTROL_RING_SIZE];
static uint8_t s_control_head = 0;
static uint8_t s_control_tail = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void scan_work_handler(struct k_work *work);
static void join_work_handler(struct k_work *work);
static void status_work_handler(struct k_work *work);
static void rx_work_handler(struct k_work *work);
static void command_work_handler(struct k_work *work);
static void audio_ingress_work_handler(struct k_work *work);
static int process_audio_ingress(const uint8_t *data, uint8_t len, uint8_t audio_flags);
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi,
                              int64_t timestamp_us);
static void control_tx_handler(uint32_t frame_counter);
static void set_audio_ingress_enabled(bool enabled, bool purge);
static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi);
static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter);
static int send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len, uint8_t ttl,
                          uint8_t flags, uint8_t src_id, uint8_t seq);
static uint8_t node_bit(uint8_t node_id);
static bool seen_packet(uint8_t type, uint8_t src_id, uint8_t seq);
static void remember_packet(uint8_t type, uint8_t src_id, uint8_t seq);
static bool enqueue_relay_packet(const uint8_t *data, uint8_t len, uint8_t ttl, uint8_t flags);
static bool relay_queue_empty(void);

static void set_audio_ingress_enabled(bool enabled, bool purge)
{
    k_mutex_lock(&s_audio_ingress_lock, K_FOREVER);
    s_audio_ingress_enabled = enabled;
    if (purge) {
        s_stat_ingress_purge_drop += k_msgq_num_used_get(&s_audio_ingress_queue);
        k_msgq_purge(&s_audio_ingress_queue);
    }
    k_mutex_unlock(&s_audio_ingress_lock);
}

static void purge_tx_audio_ring(void)
{
    uint8_t depth = s_tx_head >= s_tx_tail
                        ? (uint8_t)(s_tx_head - s_tx_tail)
                        : (uint8_t)(TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head);
    s_stat_tx_purge_drop += depth;
    s_tx_head = 0;
    s_tx_tail = 0;
}
static bool relay_permitted_for_source(uint8_t src_id, uint8_t flags);
static void note_audio_activity(uint8_t src_id, uint8_t audio_flags);
static bool is_speaker_granted(uint8_t node_id);
static void update_speaker_grants(void);
static uint8_t compute_relay_mask(uint8_t speaker_id);
static void apply_speaker_grant(const mesh_speaker_grant_payload_t *grant);
static void clear_speaker_grants(void);
static int send_speaker_grant(void);
static int send_speaker_release(const uint8_t *speaker_ids, uint8_t speaker_count);
static void check_peer_timeouts(void);

static void update_peer_last_seen(uint8_t node_id, int8_t rssi)
{
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active && s_peers[i].node_id == node_id) {
            s_peers[i].last_seen_ms = k_uptime_get();
            s_peers[i].rssi_dbm = rssi;
            break;
        }
    }
}

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

static uint8_t node_bit(uint8_t node_id)
{
    if (node_id == 0 || node_id > MESH_MAX_NODES) {
        return 0;
    }
    return (uint8_t)(1U << (node_id - 1));
}

static bool seen_packet(uint8_t type, uint8_t src_id, uint8_t seq)
{
    for (int i = 0; i < SEEN_RING_SIZE; i++) {
        if (s_seen_ring[i].valid && s_seen_ring[i].type == type && s_seen_ring[i].src_id == src_id &&
            s_seen_ring[i].seq == seq) {
            return true;
        }
    }
    return false;
}

static void remember_packet(uint8_t type, uint8_t src_id, uint8_t seq)
{
    s_seen_ring[s_seen_head].type = type;
    s_seen_ring[s_seen_head].src_id = src_id;
    s_seen_ring[s_seen_head].seq = seq;
    s_seen_ring[s_seen_head].valid = true;
    s_seen_head = (uint8_t)((s_seen_head + 1) % SEEN_RING_SIZE);
}

static bool relay_queue_empty(void)
{
    return s_relay_head == s_relay_tail;
}

static bool is_speaker_granted(uint8_t node_id)
{
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (s_active_speaker_ids[i] == node_id) {
            return true;
        }
    }
    return false;
}

static void clear_speaker_grants(void)
{
    memset(s_active_speaker_ids, 0, sizeof(s_active_speaker_ids));
    memset(s_relay_masks, 0, sizeof(s_relay_masks));
}

static bool relay_permitted_for_source(uint8_t src_id, uint8_t flags)
{
    /* Only relay audio that carries a speaker grant.  Ungrated audio is
     * restricted to 1-hop — this prevents un-authorised flooding and
     * ensures the coordinator controls relay bandwidth. */
    if ((flags & MESH_FLAG_SPEAKER_GRANTED) == 0) {
        return false;
    }

    uint8_t self_bit = node_bit(s_node_id);
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (s_active_speaker_ids[i] == src_id) {
            return (s_relay_masks[i] & self_bit) != 0;
        }
    }

    return false;
}

static void note_audio_activity(uint8_t src_id, uint8_t audio_flags)
{
    if ((audio_flags & MESH_AUDIO_FLAG_ACTIVE) == 0 || src_id == 0 || src_id > MESH_MAX_NODES) {
        return;
    }

    s_heard_bitmap |= node_bit(src_id);
    s_active_speaker_deadline_ms[src_id] = k_uptime_get() + ACTIVE_SPEAKER_TIMEOUT_MS;
}

static uint8_t compute_relay_mask(uint8_t speaker_id)
{
    uint8_t speaker_bit = node_bit(speaker_id);
    uint8_t members = 0;
    uint8_t heard = 0;
    uint8_t member_count = 0;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].active || s_peers[i].node_id == 0 || s_peers[i].node_id == speaker_id) {
            continue;
        }

        uint8_t peer_bit = node_bit(s_peers[i].node_id);
        members |= peer_bit;
        member_count++;

        uint8_t heard_bitmap = s_peers[i].node_id == s_node_id
                                   ? s_heard_bitmap
                                   : s_peers[i].heard_bitmap;
        if ((heard_bitmap & speaker_bit) != 0) {
            heard |= peer_bit;
        }
    }

    if (member_count <= 1 || (members & (uint8_t)~heard) == 0) {
        return 0;
    }

    return heard != 0 ? heard : members;
}

static void apply_speaker_grant(const mesh_speaker_grant_payload_t *grant)
{
    clear_speaker_grants();

    uint8_t count = grant->speaker_count;
    if (count > MESH_MAX_ACTIVE_SPEAKERS) {
        count = MESH_MAX_ACTIVE_SPEAKERS;
    }

    for (uint8_t i = 0; i < count; i++) {
        s_active_speaker_ids[i] = grant->speaker_ids[i];
        s_relay_masks[i] = grant->relay_masks[i];
    }
}

static void update_speaker_grants(void)
{
    if (s_role != MESH_ROLE_COORDINATOR) {
        return;
    }

    uint8_t previous[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t selected[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t released[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    int idx = 0;
    uint8_t release_count = 0;
    int64_t now = k_uptime_get();

    memcpy(previous, s_active_speaker_ids, sizeof(previous));

    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES && idx < MESH_MAX_ACTIVE_SPEAKERS; node_id++) {
        if (s_active_speaker_deadline_ms[node_id] > now) {
            selected[idx] = node_id;
            relay_masks[idx] = compute_relay_mask(node_id);
            idx++;
        }
    }

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (previous[i] == 0) {
            continue;
        }

        bool still_selected = false;
        for (int j = 0; j < MESH_MAX_ACTIVE_SPEAKERS; j++) {
            if (selected[j] == previous[i]) {
                still_selected = true;
                break;
            }
        }

        if (!still_selected && release_count < MESH_MAX_ACTIVE_SPEAKERS) {
            released[release_count++] = previous[i];
        }
    }

    if (memcmp(selected, s_active_speaker_ids, sizeof(selected)) != 0 ||
        memcmp(relay_masks, s_relay_masks, sizeof(relay_masks)) != 0) {
        if (release_count > 0) {
            (void)send_speaker_release(released, release_count);
        }
        memcpy(s_active_speaker_ids, selected, sizeof(selected));
        memcpy(s_relay_masks, relay_masks, sizeof(relay_masks));
        (void)send_speaker_grant();
    }
}

static bool enqueue_relay_packet(const uint8_t *data, uint8_t len, uint8_t ttl, uint8_t flags)
{
    if (ttl == 0 || len < sizeof(mesh_header_t)) {
        return false;
    }

    uint8_t next_head = (uint8_t)((s_relay_head + 1) % RELAY_RING_SIZE);
    if (next_head == s_relay_tail) {
        s_stat_relay_ring_drop++;
        s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
    }

    struct relay_entry *entry = &s_relay_ring[s_relay_head];
    memcpy(entry->data, data, len);
    entry->len = len;

    mesh_header_t *hdr = (mesh_header_t *)entry->data;
    hdr->ttl = ttl;
    hdr->flags = flags;

    __DMB();  /* Ensure data is written before head becomes visible to consumer */
    s_relay_head = next_head;
    return true;
}

static int send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len, uint8_t ttl,
                          uint8_t flags, uint8_t src_id, uint8_t seq)
{
    if (len > MESH_PACKET_PAYLOAD_MAX) {
        return -EMSGSIZE;
    }
    uint8_t buf[sizeof(mesh_header_t) + 128];
    mesh_header_t *hdr = (mesh_header_t *)buf;

    hdr->version = MESH_PROTOCOL_VERSION;
    hdr->type = type;
    hdr->src_id = src_id;
    hdr->seq = seq;
    hdr->ttl = ttl;
    hdr->flags = flags;
    hdr->payload_len = len;

    if (payload && len > 0) {
        memcpy(buf + sizeof(mesh_header_t), payload, len);
    }

    return esb_radio_send(buf, sizeof(mesh_header_t) + len);
}

static int send_packet(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    return send_packet_ex(type, payload, len, 0, 0, s_node_id, s_tx_seq++);
}

static int queue_control_packet(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    if (len > MESH_PACKET_PAYLOAD_MAX) {
        return -EMSGSIZE;
    }

    uint8_t next_head = (uint8_t)((s_control_head + 1) % CONTROL_RING_SIZE);
    if (next_head == s_control_tail) {
        s_stat_control_ring_drop++;
        return -ENOBUFS;
    }

    struct relay_entry *entry = &s_control_ring[s_control_head];
    mesh_header_t *hdr = (mesh_header_t *)entry->data;
    hdr->version = MESH_PROTOCOL_VERSION;
    hdr->type = type;
    hdr->src_id = s_node_id;
    hdr->seq = s_tx_seq++;
    hdr->ttl = 0;
    hdr->flags = 0;
    hdr->payload_len = len;
    if (payload != NULL && len > 0) {
        memcpy(entry->data + sizeof(*hdr), payload, len);
    }
    entry->len = (uint8_t)(sizeof(*hdr) + len);
    s_control_head = next_head;
    return 0;
}

static int send_join_request(void)
{
    mesh_join_v2_payload_t payload = {
        .capabilities = 0x01, /* Has audio */
        .reserved = 0,
    };
    memcpy(payload.requester_addr, s_local_addr, sizeof(payload.requester_addr));
    LOG_INF("Sending JOIN request");
    return send_packet(MESH_PKT_JOIN_V2, &payload, sizeof(payload));
}

static int send_sync(void)
{
    mesh_sync_payload_t payload = {
        .frame_counter = tdma_get_frame_counter(),
        .drift_ppm = 0,
    };
    memcpy(payload.coordinator_addr, s_local_addr, 5);
    return send_packet(MESH_PKT_SYNC, &payload, sizeof(payload));
}

static int send_keepalive(void)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 255,
        .reserved = 0,
    };
    return queue_control_packet(MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
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

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (s_active_speaker_ids[i] != 0) {
            payload.active_speaker_count++;
        }
    }
    memcpy(payload.active_speaker_ids, s_active_speaker_ids, sizeof(payload.active_speaker_ids));
    memcpy(payload.relay_masks, s_relay_masks, sizeof(payload.relay_masks));

    return queue_control_packet(MESH_PKT_SLOT_MAP, &payload, sizeof(payload));
}

static int send_status_packet(void)
{
    mesh_status_payload_t payload = {
        .battery_pct = 255,
        .rssi_dbm = 127,
        .peer_count = s_peer_count,
        .fw_version = MESH_PROTOCOL_VERSION,
        .temperature_c = 127,
        .heard_bitmap = s_heard_bitmap,
        .relay_bitmap = s_relay_bitmap,
        .active_speakers = 0,
    };

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (s_active_speaker_ids[i] != 0) {
            payload.active_speakers++;
        }
    }

    return queue_control_packet(MESH_PKT_STATUS, &payload, sizeof(payload));
}

static int send_speaker_grant(void)
{
    mesh_speaker_grant_payload_t payload = {0};
    memcpy(payload.speaker_ids, s_active_speaker_ids, sizeof(payload.speaker_ids));
    memcpy(payload.relay_masks, s_relay_masks, sizeof(payload.relay_masks));

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (s_active_speaker_ids[i] != 0) {
            payload.speaker_count++;
        }
    }

    return queue_control_packet(MESH_PKT_SPEAKER_GRANT, &payload, sizeof(payload));
}

static int send_speaker_release(const uint8_t *speaker_ids, uint8_t speaker_count)
{
    mesh_speaker_release_payload_t payload = {0};

    if (speaker_count > MESH_MAX_ACTIVE_SPEAKERS) {
        speaker_count = MESH_MAX_ACTIVE_SPEAKERS;
    }

    payload.speaker_count = speaker_count;
    if (speaker_ids != NULL && speaker_count > 0) {
        memcpy(payload.speaker_ids, speaker_ids, speaker_count);
    }

    return queue_control_packet(MESH_PKT_SPEAKER_RELEASE, &payload, sizeof(payload));
}

static int send_join_ack(uint8_t assigned_id, uint8_t slot_index, const uint8_t target_addr[5])
{
    mesh_join_ack_v2_payload_t payload = {
        .assigned_id = assigned_id,
        .slot_index = slot_index,
        .coordinator_id = s_node_id,
    };
    memcpy(payload.target_addr, target_addr, sizeof(payload.target_addr));
    LOG_INF("Sending JOIN_ACK: id=%d, slot=%d", assigned_id, slot_index);
    return queue_control_packet(MESH_PKT_JOIN_ACK_V2, &payload, sizeof(payload));
}

/* ============================================================================
 * Packet Handling (deferred to thread context)
 * ============================================================================ */

/* ISR-safe callback - copies data to ring buffer and submits work */
static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi)
{
    ARG_UNUSED(src_addr);

    /* NOTE: Don't printk here - ISR context, can deadlock with USB/UART */

    k_spinlock_key_t key = k_spin_lock(&s_rx_ring_lock);
    uint8_t next_head = (s_rx_ring_head + 1) % RX_RING_SIZE;
    if (next_head == s_rx_ring_tail) {
        atomic_inc(&s_stat_rx_drop);
        k_spin_unlock(&s_rx_ring_lock, key);
        return;
    }

    struct rx_ring_entry *entry = &s_rx_ring[s_rx_ring_head];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->rssi = rssi;
    entry->timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
    s_rx_ring_head = next_head;
    k_spin_unlock(&s_rx_ring_lock, key);

    /* Submit work to process in thread context */
    k_work_submit(&s_rx_work);
}

/* Work handler - drains all pending packets from ring buffer */
static void rx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    while (true) {
        struct rx_ring_entry entry;
        k_spinlock_key_t key = k_spin_lock(&s_rx_ring_lock);
        if (s_rx_ring_tail == s_rx_ring_head) {
            k_spin_unlock(&s_rx_ring_lock, key);
            break;
        }
        entry = s_rx_ring[s_rx_ring_tail];
        s_rx_ring_tail = (s_rx_ring_tail + 1) % RX_RING_SIZE;
        k_spin_unlock(&s_rx_ring_lock, key);
        process_rx_packet(entry.data, entry.len, entry.rssi, entry.timestamp_us);
    }
}

/* Actual packet processing - safe to call kernel functions here */
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi,
                              int64_t timestamp_us)
{
    if (len < sizeof(mesh_header_t)) {
        s_stat_rf_rx_malformed++;
        return;
    }

    const mesh_header_t *hdr = (const mesh_header_t *)data;
    const uint8_t *payload = data + sizeof(mesh_header_t);

    if (hdr->payload_len > (uint16_t)(len - sizeof(mesh_header_t))) {
        s_stat_rf_rx_malformed++;
        return;
    }

    if (hdr->version != MESH_PROTOCOL_VERSION) {
        s_stat_rf_rx_version_drop++;
        return;
    }

    LOG_DBG("RX type=0x%02X src=%d seq=%d (RSSI=%d)", hdr->type, hdr->src_id, hdr->seq, rssi);

    s_stat_rx_count++;

    switch (hdr->type) {
    case MESH_PKT_AUDIO:
        if (s_state == MESH_STATE_ACTIVE && hdr->payload_len > 4) {
            const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)payload;
            uint8_t audio_len = hdr->payload_len - 4; /* Subtract header */
            uint8_t bridge_buf[MESH_MAX_AUDIO_PAYLOAD + 1];

            if (hdr->src_id == s_node_id) {
                s_stat_rf_rx_self_drop++;
                break;
            }
            if (audio_len > MESH_MAX_AUDIO_PAYLOAD) {
                s_stat_rf_rx_malformed++;
                break;
            }

            if (seen_packet(hdr->type, hdr->src_id, hdr->seq)) {
                s_stat_rf_rx_duplicate_drop++;
                break;
            }

            bool is_diagnostic = is_rtt_probe_payload(audio->data, audio_len);

            remember_packet(hdr->type, hdr->src_id, hdr->seq);
            update_peer_last_seen(hdr->src_id, rssi);
            if (!is_diagnostic) {
                note_audio_activity(hdr->src_id, audio->audio_flags);
            }

            if (!is_diagnostic && s_role == MESH_ROLE_COORDINATOR) {
                update_speaker_grants();
            }

            if (!is_diagnostic && audio_len >= 2) {
                uint16_t e2e_seq = ((uint16_t)audio->data[0] << 8) | audio->data[1];
                struct e2e_src_state *st = &s_e2e_rf_rx_src[hdr->src_id];
                if (!st->init) {
                    st->init = true;
                } else {
                    uint16_t expected = (uint16_t)(st->last_seq + 1);
                    if (e2e_seq != expected) {
                        int16_t signed_delta = (int16_t)(e2e_seq - expected);
                        if (signed_delta > 0) {
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

            bridge_buf[0] = audio->audio_flags;
            memcpy(&bridge_buf[1], audio->data, audio_len);
            s_stat_rf_rx_audio_ok++;
            if (uart_bridge_send_audio(hdr->src_id, bridge_buf, (uint8_t)(audio_len + 1)) == 0) {
                s_stat_audio_fwd++;
                if (!is_diagnostic) {
                    s_e2e_spi_out_frames++;
                }
                s_stat_spi_out_ok++;
            } else {
                s_stat_spi_out_drop++;
            }

            if (hdr->ttl > 0 && (hdr->flags & MESH_FLAG_RELAY_REQUEST) != 0 &&
                relay_permitted_for_source(hdr->src_id, hdr->flags)) {
                (void)enqueue_relay_packet(data, (uint8_t)(sizeof(mesh_header_t) + hdr->payload_len),
                                           (uint8_t)(hdr->ttl - 1),
                                           (uint8_t)(hdr->flags | MESH_FLAG_RELAYED));
            }
        } else if (s_state != MESH_STATE_ACTIVE) {
            s_stat_rf_rx_inactive_drop++;
        } else {
            s_stat_rf_rx_malformed++;
        }
        break;

    case MESH_PKT_SYNC:
        if (hdr->payload_len != sizeof(mesh_sync_payload_t) || hdr->src_id == 0 ||
            hdr->src_id > MESH_MAX_NODES) {
            break;
        }
        if (s_state == MESH_STATE_SCANNING) {
            /* Found existing mesh */
            LOG_INF("Found mesh, coordinator=%d", hdr->src_id);
            s_coordinator_id = hdr->src_id;
            s_state = MESH_STATE_JOINING;
            s_join_attempts = 0;
            k_work_cancel_delayable(&s_scan_work);
            k_work_schedule(&s_join_work, K_NO_WAIT);
        } else if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT) {
            if (hdr->src_id != s_coordinator_id) {
                break;
            }
            /* Sync to coordinator timing */
            const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)payload;
            int64_t frame_start_us = timestamp_us - (MESH_MAX_NODES * MESH_SLOT_MS * 1000) -
                                     NRF_SYNC_RX_LATENCY_US;
            tdma_sync(sync->frame_counter, sync->drift_ppm, frame_start_us);
            s_last_sync_time = k_uptime_get_32();
        } else if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_COORDINATOR) {
            /* Dual-coordinator conflict: lower MAC address wins */
            const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)payload;
            int cmp = memcmp(sync->coordinator_addr, s_local_addr, 5);
            if (cmp < 0) {
                /* Other coordinator has lower MAC — we demote */
                LOG_WRN("Dual coordinator detected, other MAC is lower - demoting to scan");
                mesh_log("MESH: Dual coordinator, demoting (lower MAC wins)");

                set_audio_ingress_enabled(false, false);
                tdma_stop();
                s_state = MESH_STATE_SCANNING;
                s_role = MESH_ROLE_NONE;
                s_node_id = 0;
                s_slot_index = -1;
                s_peer_count = 0;
                memset(s_peers, 0, sizeof(s_peers));
                memset(s_seen_ring, 0, sizeof(s_seen_ring));
                memset(s_relay_ring, 0, sizeof(s_relay_ring));
                memset(s_control_ring, 0, sizeof(s_control_ring));
                memset(s_active_speaker_deadline_ms, 0,
                       sizeof(s_active_speaker_deadline_ms));
                clear_speaker_grants();
                s_seen_head = 0;
                s_relay_head = 0;
                s_relay_tail = 0;
                s_control_head = 0;
                s_control_tail = 0;
                purge_tx_audio_ring();
                set_audio_ingress_enabled(false, true);

                k_work_cancel_delayable(&s_status_work);
                k_work_schedule(&s_scan_work, K_NO_WAIT);
            } else if (cmp > 0) {
                LOG_INF("Dual coordinator detected, we have lower MAC — staying coordinator");
            }
            /* cmp == 0: same node (shouldn't happen), ignore */
        }
        break;

    case MESH_PKT_JOIN_V2:
        if (s_role == MESH_ROLE_COORDINATOR && hdr->src_id == 0 &&
            hdr->payload_len == sizeof(mesh_join_v2_payload_t)) {
            const mesh_join_v2_payload_t *join = (const mesh_join_v2_payload_t *)payload;
            uint8_t assigned_id = 0;
            int8_t assigned_slot = -1;

            for (int i = 0; i < MESH_MAX_NODES; i++) {
                if (s_peers[i].active &&
                    memcmp(s_peers[i].esb_addr, join->requester_addr,
                           sizeof(join->requester_addr)) == 0) {
                    assigned_id = s_peers[i].node_id;
                    assigned_slot = s_peers[i].slot_index;
                    s_peers[i].last_seen_ms = k_uptime_get();
                    break;
                }
            }

            if (assigned_id == 0) {
                for (int slot = 1; slot < MESH_MAX_NODES; slot++) {
                    bool slot_used = false;
                    for (int i = 0; i < MESH_MAX_NODES; i++) {
                        if (s_peers[i].active && s_peers[i].slot_index == slot) {
                            slot_used = true;
                            break;
                        }
                    }
                    if (!slot_used) {
                        assigned_slot = slot;
                        assigned_id = (uint8_t)(slot + 1);
                        break;
                    }
                }

                for (int i = 0; assigned_id != 0 && i < MESH_MAX_NODES; i++) {
                    if (!s_peers[i].active) {
                        s_peers[i].node_id = assigned_id;
                        s_peers[i].slot_index = assigned_slot;
                        memcpy(s_peers[i].esb_addr, join->requester_addr,
                               sizeof(s_peers[i].esb_addr));
                        s_peers[i].last_seen_ms = k_uptime_get();
                        s_peers[i].active = true;
                        s_peer_count++;
                        uart_bridge_send_event(0x02, NULL, 0);
                        break;
                    }
                }
            }

            if (assigned_id == 0) {
                LOG_WRN("No free slots for new node");
                break;
            }

            send_join_ack(assigned_id, (uint8_t)assigned_slot, join->requester_addr);
            send_slot_map();
        }
        break;

    case MESH_PKT_JOIN_ACK_V2:
        if (s_state == MESH_STATE_JOINING &&
            hdr->payload_len == sizeof(mesh_join_ack_v2_payload_t)) {
            const mesh_join_ack_v2_payload_t *ack = (const mesh_join_ack_v2_payload_t *)payload;
            if (memcmp(ack->target_addr, s_local_addr, sizeof(ack->target_addr)) != 0 ||
                ack->assigned_id == 0 || ack->assigned_id > MESH_MAX_NODES ||
                ack->slot_index == 0 || ack->slot_index >= MESH_MAX_NODES ||
                ack->coordinator_id == 0 || ack->coordinator_id > MESH_MAX_NODES ||
                ack->assigned_id == ack->coordinator_id || hdr->src_id != ack->coordinator_id ||
                ack->coordinator_id != s_coordinator_id) {
                break;
            }

            s_node_id = ack->assigned_id;
            s_slot_index = ack->slot_index;
            s_coordinator_id = ack->coordinator_id;
            purge_tx_audio_ring();
            set_audio_ingress_enabled(false, true);
            LOG_INF("JOIN_ACK: node_id=%d, slot=%d", s_node_id, s_slot_index);
            s_state = MESH_STATE_ACTIVE;
            s_role = MESH_ROLE_PARTICIPANT;
            set_audio_ingress_enabled(true, false);
            k_work_cancel_delayable(&s_join_work);
            tdma_start(s_slot_index, false);
            s_last_sync_time = k_uptime_get_32();
            k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
            uart_bridge_send_event(0x02, NULL, 0);
        }
        break;

    case MESH_PKT_JOIN:
    case MESH_PKT_JOIN_ACK:
        /* Legacy nRF JOIN packets have no requester identity and cannot be
         * safely deduplicated or targeted. */
        break;

    case MESH_PKT_KEEPALIVE:
        update_peer_last_seen(hdr->src_id, rssi);
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
                    s_peers[i].heard_bitmap = status->heard_bitmap;
                    s_peers[i].relay_bitmap = status->relay_bitmap;
                    s_peers[i].last_seen_ms = k_uptime_get();
                    break;
                }
            }
        }
        break;

    case MESH_PKT_SLOT_MAP:
        if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT &&
            hdr->src_id == s_coordinator_id &&
            hdr->payload_len == sizeof(mesh_slot_map_payload_t)) {
            const mesh_slot_map_payload_t *slot_map = (const mesh_slot_map_payload_t *)payload;
            uint8_t slot_count = slot_map->slot_count;
            if (slot_count == 0 || slot_count > MESH_MAX_NODES ||
                slot_map->slot_ids[0] != s_coordinator_id ||
                slot_map->active_speaker_count > MESH_MAX_ACTIVE_SPEAKERS) {
                break;
            }

            uint16_t seen_ids = 0;
            int8_t assigned_slot = -1;
            bool valid_map = true;
            for (uint8_t slot = 0; slot < slot_count; slot++) {
                uint8_t node_id = slot_map->slot_ids[slot];
                if (node_id == 0) {
                    continue;
                }
                if (node_id > MESH_MAX_NODES) {
                    valid_map = false;
                    break;
                }
                uint16_t bit = (uint16_t)(1U << (node_id - 1));
                if ((seen_ids & bit) != 0) {
                    valid_map = false;
                    break;
                }
                seen_ids |= bit;
                if (node_id == s_node_id) {
                    assigned_slot = (int8_t)slot;
                }
            }
            for (uint8_t i = 0; valid_map && i < slot_map->active_speaker_count; i++) {
                if (slot_map->active_speaker_ids[i] == 0 ||
                    slot_map->active_speaker_ids[i] > MESH_MAX_NODES) {
                    valid_map = false;
                }
            }
            if (!valid_map || assigned_slot <= 0) {
                break;
            }

            for (uint8_t slot = 0; slot < slot_count; slot++) {
                uint8_t node_id = slot_map->slot_ids[slot];
                if (node_id == 0) {
                    continue;
                }

                for (int i = 0; i < MESH_MAX_NODES; i++) {
                    if (s_peers[i].active && s_peers[i].node_id == node_id) {
                        s_peers[i].slot_index = (int8_t)slot;
                        break;
                    }
                }
            }
            s_slot_index = assigned_slot;
            tdma_set_slot_index(assigned_slot);

            clear_speaker_grants();
            for (uint8_t i = 0; i < slot_map->active_speaker_count && i < MESH_MAX_ACTIVE_SPEAKERS;
                 i++) {
                s_active_speaker_ids[i] = slot_map->active_speaker_ids[i];
                s_relay_masks[i] = slot_map->relay_masks[i];
            }
        }
        break;

    case MESH_PKT_SPEAKER_GRANT:
        if (hdr->src_id == s_coordinator_id &&
            hdr->payload_len == sizeof(mesh_speaker_grant_payload_t)) {
            apply_speaker_grant((const mesh_speaker_grant_payload_t *)payload);
        }
        break;

    case MESH_PKT_SPEAKER_RELEASE:
        if (hdr->src_id == s_coordinator_id &&
            hdr->payload_len == sizeof(mesh_speaker_release_payload_t)) {
            const mesh_speaker_release_payload_t *release =
                (const mesh_speaker_release_payload_t *)payload;

            for (uint8_t rel = 0; rel < release->speaker_count && rel < MESH_MAX_ACTIVE_SPEAKERS;
                 rel++) {
                for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
                    if (s_active_speaker_ids[i] == release->speaker_ids[rel]) {
                        s_active_speaker_ids[i] = 0;
                        s_relay_masks[i] = 0;
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
    set_audio_ingress_enabled(true, false);

    /* Add self to peer list */
    s_peers[0].node_id = 1;
    esb_radio_get_address(s_peers[0].esb_addr);
    s_peers[0].slot_index = 0;
    s_peers[0].active = true;
    s_peers[0].last_seen_ms = k_uptime_get();
    s_peer_count = 1;

    /* RX is already running from scanning phase */

    /* Start TDMA */
    tdma_start(s_slot_index, true);

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

/* Drop peers that have gone silent past MESH_NODE_TIMEOUT_MS.  Liveness is
 * refreshed by AUDIO / KEEPALIVE / STATUS / JOIN packets (see
 * update_peer_last_seen()), all of which flow independently of voice activity,
 * so silence-suppressed nodes stay alive as long as their 1 Hz keepalive lands.
 * Mirrors the ESP-NOW check_peer_timeouts() in components/mesh/mesh.c. */
static void check_peer_timeouts(void)
{
    int64_t now = k_uptime_get();
    bool topology_changed = false;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].active) {
            continue;
        }
        /* Never time ourselves out. */
        if (s_peers[i].node_id == s_node_id) {
            continue;
        }
        /* Coordinator liveness is owned by the SYNC_TIMEOUT_MS path below,
         * which drives re-election/rescan.  Skip it here to avoid duplicate
         * teardown and spurious PEER_LEFT events during role transitions. */
        if (s_peers[i].node_id == s_coordinator_id) {
            continue;
        }

        if ((now - s_peers[i].last_seen_ms) > MESH_NODE_TIMEOUT_MS) {
            uint8_t timed_out_id = s_peers[i].node_id;
            s_peers[i].active = false;
            if (s_peer_count > 0) {
                s_peer_count--;
            }
            topology_changed = true;
            LOG_WRN("Peer %u timed out (silent %lld ms), remaining peers: %u",
                    timed_out_id, (long long)(now - s_peers[i].last_seen_ms),
                    s_peer_count);

            /* Notify ESP32 so it can play a disconnect tone. */
            uart_bridge_send_event(0x03, NULL, 0); /* BRIDGE_EVENT_PEER_LEFT */
        }
    }

    /* Re-publish the slot map so survivors learn the freed slot. */
    if (topology_changed && s_role == MESH_ROLE_COORDINATOR) {
        send_slot_map();
    }
}

static void status_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (s_state != MESH_STATE_ACTIVE) {
        return;
    }

    /* Reap silent peers once per status tick (1 Hz). */
    check_peer_timeouts();

    /* Send status to ESP32 */
    uart_bridge_send_status(s_role, s_peer_count, s_node_id);

    /* Send mesh status and keepalive over RF */
    if (s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
    }
    int status_ret = send_status_packet();
    send_keepalive();
    if (status_ret == 0) {
        s_heard_bitmap = 0;
        s_relay_bitmap = 0;
    }

    /* Log packet stats */
    esb_radio_timing_stats_t esb_stats = {0};
    esb_radio_get_timing_stats(&esb_stats);
    tdma_stats_t tdma_stats = {0};
    tdma_get_stats(&tdma_stats);

    bool log_now = ((s_status_log_decim++ % 2) == 0);
    if (log_now) {
        printk("[MESH] r=%u id=%u sl=%d tx=%u(err=%u) rx=%u drop=%u fwd=%u | spi_in=%u overwr=%u under=%u q=%u\n",
               s_role, s_node_id, s_slot_index, s_stat_tx_count, s_stat_tx_fail, s_stat_rx_count,
               (uint32_t)atomic_get(&s_stat_rx_drop), s_stat_audio_fwd, s_stat_spi_audio_in,
               (uint32_t)atomic_get(&s_stat_tx_overwrite),
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
        printk("PIPE v=1 dev=nrf stage=mesh node=%u ingress_ok=%u ingress_inactive_drop=%u ingress_q_drop=%u ingress_purge_drop=%u tx_ring_drop=%u tx_purge_drop=%u prefill_skip=%u rf_tx_try=%u rf_tx_ok=%u rf_tx_fail=%u rf_rx_ok=%u rf_rx_ring_drop=%u rf_rx_malformed=%u rf_rx_version_drop=%u rf_rx_self_drop=%u rf_rx_dup_drop=%u rf_rx_inactive_drop=%u relay_q_drop=%u control_q_drop=%u spi_out_ok=%u spi_out_drop=%u q_depth=%u\n",
               s_node_id, s_stat_spi_audio_in, s_stat_ingress_inactive_drop,
               (uint32_t)atomic_get(&s_stat_ingress_msgq_drop), s_stat_ingress_purge_drop,
               s_stat_tx_ring_drop, s_stat_tx_purge_drop, s_skip_count,
               s_stat_rf_audio_try, s_stat_rf_audio_ok,
               s_stat_rf_audio_fail, s_stat_rf_rx_audio_ok,
               (uint32_t)atomic_get(&s_stat_rx_drop), s_stat_rf_rx_malformed,
               s_stat_rf_rx_version_drop, s_stat_rf_rx_self_drop,
               s_stat_rf_rx_duplicate_drop, s_stat_rf_rx_inactive_drop,
               s_stat_relay_ring_drop, s_stat_control_ring_drop, s_stat_spi_out_ok,
               s_stat_spi_out_drop, s_tx_queue_depth_dbg);
        printk("PIPE v=1 dev=nrf stage=tdma node=%u slot_due=%u slot_submit_drop=%u slot_late_drop=%u control_due=%u control_submit_drop=%u control_late_drop=%u\n",
               s_node_id, tdma_stats.slot_due, tdma_stats.slot_submit_drop,
               tdma_stats.slot_late_drop, tdma_stats.control_due,
               tdma_stats.control_submit_drop, tdma_stats.control_late_drop);
        printk("PIPE v=1 dev=nrf stage=rf node=%u tx_ok=%u tx_timeout=%u tx_busy=%u tx_write_drop=%u tx_event_fail=%u rx_no_callback=%u rx_flush_drop=%u rx_restart_drop=%u tx_wait_max_us=%u rx_pause_max_us=%u\n",
               s_node_id, esb_stats.tx_count, esb_stats.tx_timeout_count,
               esb_stats.tx_busy_count, esb_stats.tx_write_fail_count,
               esb_stats.tx_failed_event_count, esb_stats.rx_no_callback_count,
               esb_stats.rx_flush_drop_count, esb_stats.rx_restart_fail_count,
               esb_stats.tx_wait_us_max, esb_stats.rx_pause_us_max);
    }

    if (s_role == MESH_ROLE_PARTICIPANT) {
        /* Check for coordinator timeout */
        if (k_uptime_get_32() - s_last_sync_time > SYNC_TIMEOUT_MS) {
            LOG_WRN("Coordinator lost (timeout), rescanning...");
            mesh_log("MESH: Coordinator lost, rescanning");
            set_audio_ingress_enabled(false, false);

            /* Notify ESP32 that coordinator sync was lost.
             * Do NOT emit PEER_LEFT here: this path can be transient (role
             * transition/rescan) and audio may continue, which caused false
             * disconnect tones on ESP. */
            uart_bridge_send_event(0x05, NULL, 0); /* BRIDGE_EVENT_SYNC_LOST */

            /* Stop TDMA */
            tdma_stop();

            memset(s_peers, 0, sizeof(s_peers));
            memset(s_seen_ring, 0, sizeof(s_seen_ring));
            memset(s_relay_ring, 0, sizeof(s_relay_ring));
            memset(s_control_ring, 0, sizeof(s_control_ring));
            memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
            clear_speaker_grants();
            s_heard_bitmap = 0;
            s_relay_bitmap = 0;
            s_peer_count = 0;
            s_seen_head = 0;
            s_relay_head = 0;
            s_relay_tail = 0;
            s_control_head = 0;
            s_control_tail = 0;
            purge_tx_audio_ring();
            set_audio_ingress_enabled(false, true);

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
            s_skip_count++;
            return;
        }
    } else if (depth < QUEUE_LO) {
        s_skip_mode = true;
        s_skip_count++;
        return;
    }

    if (s_state == MESH_STATE_ACTIVE && s_tx_head != s_tx_tail) {
        /* Peek at tail */
        struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_tail];
        bool is_diagnostic = is_rtt_probe_payload(entry->data, entry->len);
        uint8_t tx_flags = MESH_FLAG_RELAY_REQUEST;

        mesh_audio_payload_t payload = {
            .codec = 1, /* Opus */
            .frame_ms = 20,
            .stream_id = s_node_id,
            .audio_flags = entry->audio_flags,
        };

        if (is_speaker_granted(s_node_id)) {
            tx_flags |= MESH_FLAG_SPEAKER_GRANTED;
        }

        if (entry->len > MESH_MAX_AUDIO_PAYLOAD) {
            entry->len = MESH_MAX_AUDIO_PAYLOAD;
        }

        memcpy(payload.data, entry->data, entry->len);
        if (!is_diagnostic) {
            note_audio_activity(s_node_id, entry->audio_flags);
        }

        /* Send packet */
        s_stat_rf_audio_try++;
        int ret = send_packet_ex(MESH_PKT_AUDIO, &payload, 4 + entry->len, MESH_AUDIO_TTL_DEFAULT,
                                 tx_flags, s_node_id, s_tx_seq++);

        if (ret == 0) {
            s_stat_tx_count++;
            if (!is_diagnostic) {
                s_e2e_rf_tx_frames++;
            }
            s_stat_rf_audio_ok++;
        } else {
            s_stat_tx_fail++;
            s_stat_rf_audio_fail++;
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

    } else if (!relay_queue_empty()) {
        __DMB();  /* Ensure we see data written by producer before reading entry */
        struct relay_entry *entry = &s_relay_ring[s_relay_tail];
        const mesh_header_t *relay_hdr = (const mesh_header_t *)entry->data;
        int ret = esb_radio_send(entry->data, entry->len);

        if (ret == 0) {
            s_stat_tx_count++;
            s_relay_bitmap |= node_bit(relay_hdr->src_id);
        } else {
            s_stat_tx_fail++;
        }

        s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
    } else {
        if (has_audio_source && depth == 0) {
            s_stat_tx_underflow++;
            note_underflow("empty");
        }
    }
}

static void control_tx_handler(uint32_t frame_counter)
{
    if ((frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0) {
        if (s_role == MESH_ROLE_COORDINATOR) {
            send_sync();
        }
        return;
    }

    if ((frame_counter % MESH_MAX_NODES) != (uint32_t)s_slot_index) {
        return;
    }

    if (s_control_tail != s_control_head) {
        struct relay_entry *entry = &s_control_ring[s_control_tail];
        int ret = esb_radio_send(entry->data, entry->len);
        if (ret == 0) {
            s_control_tail = (uint8_t)((s_control_tail + 1) % CONTROL_RING_SIZE);
        } else {
            s_stat_tx_fail++;
        }
    }
}

static int process_audio_ingress(const uint8_t *data, uint8_t len, uint8_t audio_flags)
{
    if (s_state != MESH_STATE_ACTIVE) {
        s_stat_ingress_inactive_drop++;
        return -EAGAIN;
    }
    if (len > MESH_MAX_AUDIO_PAYLOAD) {
        return -EMSGSIZE;
    }

    bool is_diagnostic = is_rtt_probe_payload(data, len);

    if (!is_diagnostic && len >= 2) {
        uint16_t e2e_seq = ((uint16_t)data[0] << 8) | data[1];
        struct e2e_src_state *st = &s_e2e_spi_in_src[s_node_id];
        if (!st->init) {
            st->init = true;
        } else {
            uint16_t expected = (uint16_t)(st->last_seq + 1);
            if (e2e_seq != expected) {
                int16_t signed_delta = (int16_t)(e2e_seq - expected);
                if (signed_delta > 0) {
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
    if (!is_diagnostic) {
        note_audio_activity(s_node_id, audio_flags);
    }

    if (!is_diagnostic && s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
    }

    uint8_t next_head = (s_tx_head + 1) % TX_AUDIO_RING_SIZE;

    if (next_head == s_tx_tail) {
        /* Buffer full: drop oldest and keep newest to bound latency growth. */
        s_stat_tx_ring_drop++;
        s_tx_tail = (s_tx_tail + 1) % TX_AUDIO_RING_SIZE;
    }

    struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_head];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->audio_flags = audio_flags;

    s_tx_head = next_head;

    if (s_tx_head >= s_tx_tail) {
        s_tx_queue_depth_dbg = s_tx_head - s_tx_tail;
    } else {
        s_tx_queue_depth_dbg = TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
    }

    return 0;
}

int mesh_protocol_send_audio(const uint8_t *data, uint8_t len, uint8_t audio_flags)
{
    if (data == NULL || len == 0 || len > MESH_MAX_AUDIO_PAYLOAD) {
        return -EINVAL;
    }

    k_mutex_lock(&s_audio_ingress_lock, K_FOREVER);
    if (!s_audio_ingress_enabled) {
        k_mutex_unlock(&s_audio_ingress_lock);
        return -EAGAIN;
    }

    struct audio_ingress_entry entry = {
        .len = len,
        .audio_flags = audio_flags,
    };
    memcpy(entry.data, data, len);

    if (k_msgq_put(&s_audio_ingress_queue, &entry, K_NO_WAIT) != 0) {
        struct audio_ingress_entry dropped;
        if (k_msgq_get(&s_audio_ingress_queue, &dropped, K_NO_WAIT) == 0) {
            atomic_inc(&s_stat_ingress_msgq_drop);
            atomic_inc(&s_stat_tx_overwrite);
        }
        if (k_msgq_put(&s_audio_ingress_queue, &entry, K_NO_WAIT) != 0) {
            atomic_inc(&s_stat_ingress_msgq_drop);
            k_mutex_unlock(&s_audio_ingress_lock);
            return -ENOBUFS;
        }
    }

    k_work_submit(&s_audio_ingress_work);
    k_mutex_unlock(&s_audio_ingress_lock);
    return 0;
}

static void audio_ingress_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct audio_ingress_entry entry;

    while (k_msgq_get(&s_audio_ingress_queue, &entry, K_NO_WAIT) == 0) {
        (void)process_audio_ingress(entry.data, entry.len, entry.audio_flags);
    }
}

static void command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    do {
        if (atomic_set(&s_control_pending, 0) != 0) {
            bool enable = atomic_get(&s_requested_enabled) != 0;
            if (enable && s_state == MESH_STATE_IDLE) {
                (void)mesh_protocol_start();
            } else if (!enable && s_state != MESH_STATE_IDLE) {
                mesh_protocol_stop();
            }
        }
        if (atomic_set(&s_status_pending, 0) != 0) {
            uart_bridge_send_status(s_role, s_peer_count, s_node_id);
        }
    } while (atomic_get(&s_control_pending) != 0 || atomic_get(&s_status_pending) != 0);
}

void mesh_protocol_request_start(void)
{
    atomic_set(&s_requested_enabled, 1);
    atomic_set(&s_control_pending, 1);
    k_work_submit(&s_command_work);
}

void mesh_protocol_request_stop(void)
{
    atomic_set(&s_requested_enabled, 0);
    atomic_set(&s_control_pending, 1);
    k_work_submit(&s_command_work);
}

void mesh_protocol_request_status(void)
{
    atomic_set(&s_status_pending, 1);
    k_work_submit(&s_command_work);
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
    k_work_init(&s_command_work, command_work_handler);
    k_work_init(&s_audio_ingress_work, audio_ingress_work_handler);

    /* Set callbacks */
    esb_radio_set_rx_callback(esb_rx_callback);
    tdma_set_slot_callback(slot_tx_handler);
    tdma_set_control_callback(control_tx_handler);

    /* Get local address */
    esb_radio_get_address(s_local_addr);

    s_state = MESH_STATE_IDLE;
    set_audio_ingress_enabled(false, false);
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

    set_audio_ingress_enabled(false, false);
    k_work_cancel_delayable(&s_scan_work);
    k_work_cancel_delayable(&s_join_work);
    k_work_cancel_delayable(&s_status_work);

    if (s_state == MESH_STATE_ACTIVE && s_node_id != 0) {
        send_packet(MESH_PKT_LEAVE, NULL, 0);
        /* Notify ESP32 that we are leaving the mesh */
        uart_bridge_send_event(0x03, NULL, 0); /* BRIDGE_EVENT_PEER_LEFT */
    }

    s_state = MESH_STATE_IDLE;
    tdma_stop();
    esb_radio_stop_rx();
    k_work_cancel(&s_rx_work);
    k_work_cancel(&s_audio_ingress_work);
    k_spinlock_key_t rx_key = k_spin_lock(&s_rx_ring_lock);
    s_rx_ring_head = 0;
    s_rx_ring_tail = 0;
    k_spin_unlock(&s_rx_ring_lock, rx_key);

    memset(s_peers, 0, sizeof(s_peers));
    memset(s_seen_ring, 0, sizeof(s_seen_ring));
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    memset(s_control_ring, 0, sizeof(s_control_ring));
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    clear_speaker_grants();
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    s_peer_count = 0;
    s_seen_head = 0;
    s_relay_head = 0;
    s_relay_tail = 0;
    s_control_head = 0;
    s_control_tail = 0;
    purge_tx_audio_ring();
    set_audio_ingress_enabled(false, true);
    s_skip_mode = true;
    s_skip_count = 0;

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
