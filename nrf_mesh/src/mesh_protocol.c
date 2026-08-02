/**
 * @file mesh_protocol.c
 * @brief Mesh Protocol State Machine
 *
 * Ported from ESP32 mesh.c - handles join/leave, coordinator election, peer tracking
 */

#include "mesh_protocol.h"
#include "audio_bundle.h"
#include "mesh_core.h"

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
        uint8_t send_len = (uint8_t)MIN(len, (int)sizeof(buf) - 1);
        uart_bridge_send_log(buf, send_len);
    }
}

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SCAN_TIMEOUT_MS    3000
#define SCAN_BACKOFF_MAX_MS 500
#define JOIN_TIMEOUT_MS    5000
#define JOIN_RETRY_MS      500
#define JOIN_RETRY_COUNT   10
#define STATUS_INTERVAL_MS 1000
#define ACTIVE_SPEAKER_TIMEOUT_MS 1500
#define RELAY_RING_SIZE    16
#define CONTROL_RING_SIZE  32
#define TX_AUDIO_RING_SIZE 16
#define NRF_SYNC_RX_LATENCY_US 300
#define MESH_PACKET_PAYLOAD_MAX MESH_AUDIO_V2_MAX_BUNDLE_SIZE
#define MESH_PACKET_OUTER_MAX MESH_AUDIO_V2_MAX_PACKET_SIZE
#define ESB_NORMAL_RAMP_US 129U
#define ESB_1MBPS_OVERHEAD_BYTES 10U
#define AUDIO_TX_MARGIN_US 100U

/* Protocol v2 is a deliberate fail-closed RF migration: v1 peers are rejected. */
_Static_assert(MESH_PACKET_PAYLOAD_MAX == 200, "mesh payload capacity changed");
_Static_assert(MESH_PACKET_OUTER_MAX == 208, "mesh packet capacity changed");
_Static_assert(MESH_PACKET_OUTER_MAX <= UINT8_MAX, "ESB packet length no longer fits uint8_t");

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static mesh_state_t s_state = MESH_STATE_IDLE;
static mesh_role_t s_role = MESH_ROLE_NONE;
static uint8_t s_node_id = 0;
static int8_t s_slot_index = -1;
static uint8_t s_coordinator_id = 0;
static bool s_participant_membership_known = false;

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
    uint8_t data[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    uint8_t len;
    uint8_t audio_flags;
    uint8_t packet_type;
};

struct tx_audio_entry {
    uint8_t data[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    uint8_t len;
    uint8_t audio_flags;
    uint8_t packet_type;
};

K_MSGQ_DEFINE(s_audio_ingress_queue, sizeof(struct audio_ingress_entry), 16, 4);
K_MUTEX_DEFINE(s_audio_ingress_lock);
static bool s_audio_ingress_enabled = false;
static uint32_t s_stat_ingress_purge_drop = 0;
static struct tx_audio_entry s_tx_audio_ring[TX_AUDIO_RING_SIZE];
static uint8_t s_tx_head = 0;
static uint8_t s_tx_tail = 0;
static bool s_relay_contention_turn = false;
static bool s_local_deferred_pending = false;
static uint16_t s_local_deferred_seq = 0;
static uint32_t s_stat_tx_purge_drop = 0;
static atomic_t s_requested_enabled;
static atomic_t s_control_pending;
static atomic_t s_status_pending;
static atomic_t s_requested_command;
static atomic_t s_requested_generation;

static int s_join_attempts = 0;

static uint32_t scan_timeout_ms(void)
{
    uint16_t address_suffix = ((uint16_t)s_local_addr[3] << 8) | s_local_addr[4];
    return SCAN_TIMEOUT_MS + (address_suffix % (SCAN_BACKOFF_MAX_MS + 1));
}

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
static uint32_t s_stat_bundle_tx = 0;
static uint32_t s_stat_bundle_rx = 0;
static uint32_t s_stat_bundle_bad = 0;
static uint32_t s_stat_prev1_forwarded = 0;
static uint32_t s_stat_prev2_forwarded = 0;
static uint32_t s_stat_prev1_stripped = 0;
static uint32_t s_stat_prev2_stripped = 0;
static uint32_t s_stat_bundle_late_drop = 0;
static uint32_t s_stat_bundle_max_bytes = 0;
static uint32_t s_stat_local_deferred_recovery = 0;
static uint32_t s_last_audio_in_time = 0; /* Timestamp of last audio packet from ESP32 */
static uint8_t s_tx_queue_depth_dbg = 0;  /* Current TX queue depth for diagnostics */
static uint32_t s_under_prev = 0;
static uint32_t s_under_delta_last = 0;
static uint32_t s_status_log_decim = 0;
static uint32_t s_under_log_next = 64;

/* End-to-end sequence diagnostics.
 * Sequence is injected by ESP in first 2 bytes of Opus payload and forwarded
 * unchanged over nRF mesh and back to ESP. */
static mesh_core_seq16_t s_e2e_spi_in_src[256];
struct rf_seq16_tracker {
    uint16_t last;
    bool initialized;
};
static struct rf_seq16_tracker s_e2e_rf_rx_src[256];
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

static uint32_t s_skip_count = 0;
static uint32_t s_auto_ticks = 0;       /* Status log counter */

static mesh_core_dedupe_t s_dedupe;

static void reset_rf_e2e_tracker(uint8_t node_id)
{
    if (!mesh_core_node_id_valid(node_id)) {
        return;
    }
    memset(&s_e2e_rf_rx_src[node_id], 0, sizeof(s_e2e_rf_rx_src[node_id]));
}

static void reset_all_rf_e2e_trackers(void)
{
    memset(s_e2e_rf_rx_src, 0, sizeof(s_e2e_rf_rx_src));
}

static void track_rf_e2e_sequence(uint8_t src_id, uint16_t sequence)
{
    if (!mesh_core_node_id_valid(src_id)) {
        return;
    }
    struct rf_seq16_tracker *tracker = &s_e2e_rf_rx_src[src_id];

    if (!tracker->initialized) {
        tracker->last = sequence;
        tracker->initialized = true;
    } else {
        int16_t delta = (int16_t)(uint16_t)(sequence - tracker->last);
        if (delta > 0) {
            if (delta > 1) {
                s_e2e_rf_rx_gap_evt++;
                s_e2e_rf_rx_gap_fr += (uint16_t)(delta - 1);
            }
            tracker->last = sequence;
        } else {
            /* Duplicate and late/reordered packets never rewind the baseline. */
            s_e2e_rf_rx_reset_evt++;
        }
    }
    s_e2e_rf_rx_frames++;
}

struct relay_entry {
    uint8_t data[MESH_PACKET_OUTER_MAX];
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
static int process_audio_ingress(const uint8_t *data, uint8_t len, uint8_t audio_flags,
                                 uint8_t packet_type);
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi,
                              int64_t timestamp_us);
static void control_tx_handler(uint32_t frame_counter);
static void set_audio_ingress_enabled(bool enabled, bool purge);
static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi);
static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter);
static int send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len, uint8_t ttl,
                          uint8_t flags, uint8_t src_id, uint8_t seq);
static int send_slot_map(void);
static bool enqueue_relay_packet(const uint8_t *data, uint8_t len, uint8_t ttl, uint8_t flags);
static bool relay_queue_empty(void);

static uint8_t bridge_peer_count(void)
{
    if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT &&
        !s_participant_membership_known) {
        return BRIDGE_PEER_COUNT_UNKNOWN;
    }
    return s_peer_count;
}

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
    s_relay_contention_turn = false;
    s_local_deferred_pending = false;
    s_local_deferred_seq = 0;
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
            if (!s_peers[i].announced && s_role == MESH_ROLE_COORDINATOR) {
                reset_rf_e2e_tracker(node_id);
                s_peers[i].announced = true;
                s_peer_count++;
                uint8_t joined_id = node_id;
                uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id,
                                        s_slot_index, s_coordinator_id);
                uart_bridge_send_event(BRIDGE_EVENT_PEER_JOINED, &joined_id,
                                       sizeof(joined_id));
                send_slot_map();
            }
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

    uint8_t self_bit = mesh_core_node_bit(s_node_id);
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

    s_heard_bitmap |= mesh_core_node_bit(src_id);
    s_active_speaker_deadline_ms[src_id] = k_uptime_get() + ACTIVE_SPEAKER_TIMEOUT_MS;
}

static uint8_t compute_relay_mask(uint8_t speaker_id)
{
    mesh_core_peer_snapshot_t peers[MESH_MAX_NODES];

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        peers[i] = (mesh_core_peer_snapshot_t){
            .node_id = s_peers[i].node_id,
            .heard_bitmap = s_peers[i].heard_bitmap,
            .active = s_peers[i].active,
        };
    }
    return mesh_core_relay_mask(speaker_id, s_node_id, s_heard_bitmap, peers, MESH_MAX_NODES);
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
    if (ttl == 0 || len < sizeof(mesh_header_t) || len > MESH_PACKET_OUTER_MAX) {
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
    uint8_t buf[MESH_PACKET_OUTER_MAX];
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
        /* WS phase slew is not a persistent frequency estimate. */
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
        if (s_peers[i].active && s_peers[i].announced && s_peers[i].slot_index >= 0 &&
            s_peers[i].slot_index < MESH_MAX_NODES) {
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

    if (hdr->payload_len != (uint16_t)(len - sizeof(mesh_header_t))) {
        s_stat_rf_rx_malformed++;
        return;
    }
    if (hdr->payload_len > MESH_PACKET_PAYLOAD_MAX) {
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

            if (!mesh_core_dedupe_accept(&s_dedupe, hdr->type, hdr->src_id, hdr->seq)) {
                s_stat_rf_rx_duplicate_drop++;
                break;
            }

            bool is_diagnostic = is_rtt_probe_payload(audio->data, audio_len);
            if (!is_diagnostic) {
                s_stat_rf_rx_malformed++;
                break;
            }

            update_peer_last_seen(hdr->src_id, rssi);
            if (!is_diagnostic) {
                note_audio_activity(hdr->src_id, audio->audio_flags);
            }

            if (!is_diagnostic && s_role == MESH_ROLE_COORDINATOR) {
                update_speaker_grants();
            }

            if (!is_diagnostic && audio_len >= 2) {
                uint16_t e2e_seq = ((uint16_t)audio->data[0] << 8) | audio->data[1];
                track_rf_e2e_sequence(hdr->src_id, e2e_seq);
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

    case MESH_PKT_AUDIO_V2:
        if (s_state == MESH_STATE_ACTIVE) {
            audio_bundle_view_t bundle;

            if (!mesh_core_node_id_valid(hdr->src_id)) {
                s_stat_rf_rx_malformed++;
                break;
            }
            if (hdr->src_id == s_node_id) {
                s_stat_rf_rx_self_drop++;
                break;
            }
            if (!audio_bundle_parse(payload, hdr->payload_len, &bundle)) {
                s_stat_bundle_bad++;
                s_stat_rf_rx_malformed++;
                break;
            }
            if (!mesh_core_dedupe_accept(&s_dedupe, hdr->type, hdr->src_id, hdr->seq)) {
                s_stat_rf_rx_duplicate_drop++;
                break;
            }

            s_stat_bundle_rx++;
            s_stat_bundle_max_bytes = MAX(s_stat_bundle_max_bytes, hdr->payload_len);
            update_peer_last_seen(hdr->src_id, rssi);
            note_audio_activity(hdr->src_id,
                                bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
            if (s_role == MESH_ROLE_COORDINATOR) {
                update_speaker_grants();
            }

            track_rf_e2e_sequence(hdr->src_id, bundle.current_seq);
            s_stat_rf_rx_audio_ok++;

            if (uart_bridge_send_audio_v2(hdr->src_id, payload,
                                          (uint8_t)hdr->payload_len) == 0) {
                s_stat_audio_fwd++;
                s_e2e_spi_out_frames++;
                s_stat_spi_out_ok++;
            } else {
                s_stat_spi_out_drop++;
            }

            if (hdr->ttl > 0 && (hdr->flags & MESH_FLAG_RELAY_REQUEST) != 0 &&
                relay_permitted_for_source(hdr->src_id, hdr->flags)) {
                (void)enqueue_relay_packet(data,
                                           (uint8_t)(sizeof(mesh_header_t) + hdr->payload_len),
                                           (uint8_t)(hdr->ttl - 1),
                                           (uint8_t)(hdr->flags | MESH_FLAG_RELAYED));
            }
        } else {
            s_stat_rf_rx_inactive_drop++;
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
            int cmp = mesh_core_address_compare(sync->coordinator_addr, s_local_addr,
                                                sizeof(s_local_addr));
            if (cmp < 0) {
                /* Other coordinator has lower MAC — we demote */
                LOG_WRN("Dual coordinator detected, joining lower-address coordinator");
                mesh_log("MESH: Dual coordinator, joining lower-address winner");

                set_audio_ingress_enabled(false, false);
                tdma_stop();
                s_state = MESH_STATE_JOINING;
                s_role = MESH_ROLE_NONE;
                s_node_id = 0;
                s_slot_index = -1;
                s_coordinator_id = hdr->src_id;
                s_join_attempts = 0;
                s_peer_count = 0;
                memset(s_peers, 0, sizeof(s_peers));
                mesh_core_dedupe_reset(&s_dedupe);
                reset_all_rf_e2e_trackers();
                memset(s_relay_ring, 0, sizeof(s_relay_ring));
                memset(s_control_ring, 0, sizeof(s_control_ring));
                memset(s_active_speaker_deadline_ms, 0,
                       sizeof(s_active_speaker_deadline_ms));
                clear_speaker_grants();
                s_relay_head = 0;
                s_relay_tail = 0;
                s_control_head = 0;
                s_control_tail = 0;
                purge_tx_audio_ring();
                set_audio_ingress_enabled(false, true);

                k_work_cancel_delayable(&s_status_work);
                uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                        s_coordinator_id);
                k_work_schedule(&s_join_work, K_NO_WAIT);
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
                uint8_t occupied = mesh_core_node_bit(s_node_id);
                for (int i = 0; i < MESH_MAX_NODES; i++) {
                    if (s_peers[i].active) {
                        occupied |= mesh_core_node_bit(s_peers[i].node_id);
                    }
                }
                assigned_id = mesh_core_first_free_node_id(occupied);
                assigned_slot = mesh_core_slot_for_node_id(assigned_id);

                for (int i = 0; assigned_id != 0 && i < MESH_MAX_NODES; i++) {
                    if (!s_peers[i].active) {
                        s_peers[i].node_id = assigned_id;
                        s_peers[i].slot_index = assigned_slot;
                        memcpy(s_peers[i].esb_addr, join->requester_addr,
                               sizeof(s_peers[i].esb_addr));
                        s_peers[i].last_seen_ms = k_uptime_get();
                        s_peers[i].active = true;
                        s_peers[i].announced = false;
                        break;
                    }
                }
            }

            if (assigned_id == 0) {
                LOG_WRN("No free slots for new node");
                break;
            }

            reset_rf_e2e_tracker(assigned_id);
            send_join_ack(assigned_id, (uint8_t)assigned_slot, join->requester_addr);
            send_slot_map();
        }
        break;

    case MESH_PKT_JOIN_ACK_V2:
        if (s_state == MESH_STATE_JOINING &&
            hdr->payload_len == sizeof(mesh_join_ack_v2_payload_t)) {
            const mesh_join_ack_v2_payload_t *ack = (const mesh_join_ack_v2_payload_t *)payload;
            mesh_join_ack_payload_t assignment = {
                .assigned_id = ack->assigned_id,
                .slot_index = ack->slot_index,
                .coordinator_id = ack->coordinator_id,
            };
            if (memcmp(ack->target_addr, s_local_addr, sizeof(ack->target_addr)) != 0 ||
                !mesh_core_join_assignment_valid(&assignment, hdr->src_id, s_coordinator_id,
                                                 MESH_MAX_NODES)) {
                break;
            }

            s_node_id = ack->assigned_id;
            s_slot_index = ack->slot_index;
            s_coordinator_id = ack->coordinator_id;
            reset_all_rf_e2e_trackers();
            purge_tx_audio_ring();
            set_audio_ingress_enabled(false, true);
            LOG_INF("JOIN_ACK: node_id=%d, slot=%d", s_node_id, s_slot_index);
            s_state = MESH_STATE_ACTIVE;
            s_role = MESH_ROLE_PARTICIPANT;
            s_peer_count = 1;
            s_participant_membership_known = false;
            set_audio_ingress_enabled(true, false);
            k_work_cancel_delayable(&s_join_work);
            tdma_start(s_slot_index, false);
            s_last_sync_time = k_uptime_get_32();
            k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
            uart_bridge_send_status(s_state, s_role, BRIDGE_PEER_COUNT_UNKNOWN, s_node_id,
                                    s_slot_index, s_coordinator_id);
            uart_bridge_send_event(BRIDGE_EVENT_MESH_READY, NULL, 0);
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
            update_peer_last_seen(hdr->src_id, rssi);
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
            mesh_core_slot_map_result_t parsed;
            if (!mesh_core_slot_map_valid(slot_map, s_node_id, s_coordinator_id, &parsed)) {
                break;
            }

            for (uint8_t slot = 0; slot < slot_map->slot_count; slot++) {
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
            s_slot_index = parsed.local_slot;
            s_peer_count = (uint8_t)(parsed.member_count - 1U);
            s_participant_membership_known = true;
            tdma_set_slot_index(parsed.local_slot);
            uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                    s_coordinator_id);

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
        if (hdr->src_id == s_node_id) {
            LOG_WRN("Ignoring LEAVE with local node ID %u", hdr->src_id);
            break;
        }
        for (int i = 0; i < MESH_MAX_NODES; i++) {
            bool identity_matches = hdr->payload_len == 0;
            if (hdr->payload_len == sizeof(mesh_leave_v2_payload_t)) {
                const mesh_leave_v2_payload_t *leave =
                    (const mesh_leave_v2_payload_t *)payload;
                identity_matches = memcmp(s_peers[i].esb_addr, leave->sender_addr,
                                          sizeof(leave->sender_addr)) == 0;
            }
            if (s_peers[i].active && s_peers[i].node_id == hdr->src_id && identity_matches) {
                bool announced = s_peers[i].announced;
                s_peers[i].active = false;
                mesh_core_dedupe_purge_node(&s_dedupe, hdr->src_id);
                reset_rf_e2e_tracker(hdr->src_id);
                if (announced && s_peer_count > 0) {
                    s_peer_count--;
                }
                LOG_INF("Peer %u left, remaining peers: %u", hdr->src_id, s_peer_count);

                if (announced) {
                    uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id,
                                            s_slot_index, s_coordinator_id);
                    uint8_t departed_id = hdr->src_id;
                    uart_bridge_send_event(BRIDGE_EVENT_PEER_LEFT, &departed_id,
                                           sizeof(departed_id));
                }

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
    reset_all_rf_e2e_trackers();
    set_audio_ingress_enabled(true, false);

    /* Add self to peer list */
    s_peers[0].node_id = 1;
    esb_radio_get_address(s_peers[0].esb_addr);
    s_peers[0].slot_index = 0;
    s_peers[0].active = true;
    s_peers[0].announced = true;
    s_peers[0].last_seen_ms = k_uptime_get();
    s_peer_count = 0;

    /* RX is already running from scanning phase */

    /* Start TDMA */
    tdma_start(s_slot_index, true);

    /* Start periodic status updates */
    k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));

    LOG_INF("ACTIVE as coordinator, node_id=%d, slot=%d", s_node_id, s_slot_index);
    uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                            s_coordinator_id);
    uart_bridge_send_event(BRIDGE_EVENT_BECAME_COORDINATOR, NULL, 0);
    uart_bridge_send_event(BRIDGE_EVENT_MESH_READY, NULL, 0);
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
        uint32_t delay_ms = scan_timeout_ms();
        LOG_WRN("JOIN timeout, rescanning for %u ms", delay_ms);
        mesh_log("MESH: JOIN timeout, rescanning for %u ms", delay_ms);
        s_state = MESH_STATE_SCANNING;
        s_role = MESH_ROLE_NONE;
        s_node_id = 0;
        s_slot_index = -1;
        s_coordinator_id = 0;
        reset_all_rf_e2e_trackers();
        uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                s_coordinator_id);
        k_work_schedule(&s_scan_work, K_MSEC(delay_ms));
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
            bool announced = s_peers[i].announced;
            s_peers[i].active = false;
            mesh_core_dedupe_purge_node(&s_dedupe, timed_out_id);
            reset_rf_e2e_tracker(timed_out_id);
            if (announced && s_peer_count > 0) {
                s_peer_count--;
            }
            topology_changed = true;
            LOG_WRN("Peer %u timed out (silent %lld ms), remaining peers: %u",
                    timed_out_id, (long long)(now - s_peers[i].last_seen_ms),
                    s_peer_count);

            /* Notify ESP32 so it can play a disconnect tone. */
            if (announced) {
                uart_bridge_send_event(BRIDGE_EVENT_PEER_LEFT, &timed_out_id,
                                       sizeof(timed_out_id));
            }
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
        ws_sync_diag_t ws_diag = {0};
        ws_sync_get_diag(&ws_diag);
        printk("[ATUNE] r=%u id=%u q=%u under_d=%u skip=%u/%u ws_e=%u ws_n=%u ws_ok=%u ws_no=%u ws_rej=%u ws_delta=%u ws_c=%d ws_d=%d td_req=%u td_app=%u td_sum=%lld td_pend=%d td_last=%d td_cmd=%u td_meas=%u td_jit=%d td_jit_max=%u\n",
               s_role, s_node_id, s_tx_queue_depth_dbg, s_under_delta_last,
               s_skip_count, s_auto_ticks,
               ws_diag.total_edges, ws_diag.sample_count, ws_diag.valid_count,
               ws_diag.no_signal_count, ws_diag.rejected_count,
               ws_diag.last_delta_edges, ws_diag.last_correction_us,
               ws_diag.cumulative_drift_us, tdma_stats.tune_request_count,
               tdma_stats.correction_apply_count,
               (long long)tdma_stats.correction_applied_us,
               tdma_stats.correction_pending_us, tdma_stats.last_correction_us,
               tdma_stats.commanded_period_us, tdma_stats.measured_interval_us,
               tdma_stats.callback_jitter_us, tdma_stats.callback_jitter_max_us);
        printk("[E2E_NRF] id=%u spi_in=%u spi_gap=%u/%u spi_reset=%u rf_tx=%u rf_rx=%u rf_gap=%u/%u rf_reset=%u spi_out=%u\n",
               s_node_id,
               s_e2e_spi_in_frames, s_e2e_spi_in_gap_evt, s_e2e_spi_in_gap_fr, s_e2e_spi_in_reset_evt,
               s_e2e_rf_tx_frames, s_e2e_rf_rx_frames, s_e2e_rf_rx_gap_evt, s_e2e_rf_rx_gap_fr,
               s_e2e_rf_rx_reset_evt,
                s_e2e_spi_out_frames);
        printk("PIPE v=1 dev=nrf stage=mesh node=%u ingress_ok=%u ingress_inactive_drop=%u ingress_q_drop=%u ingress_purge_drop=%u tx_ring_drop=%u tx_purge_drop=%u prefill_skip=%u rf_tx_try=%u rf_tx_ok=%u rf_tx_fail=%u rf_rx_ok=%u rf_rx_ring_drop=%u rf_rx_malformed=%u rf_rx_version_drop=%u rf_rx_self_drop=%u rf_rx_dup_drop=%u rf_rx_inactive_drop=%u relay_q_drop=%u control_q_drop=%u spi_out_ok=%u spi_out_drop=%u q_depth=%u bundle_tx=%u bundle_rx=%u bundle_bad=%u prev1_forwarded=%u prev2_forwarded=%u prev1_stripped=%u prev2_stripped=%u bundle_late_drop=%u bundle_max_bytes=%u local_deferred_recovery=%u tx_duration_max_us=%u\n",
               s_node_id, s_stat_spi_audio_in, s_stat_ingress_inactive_drop,
               (uint32_t)atomic_get(&s_stat_ingress_msgq_drop), s_stat_ingress_purge_drop,
               s_stat_tx_ring_drop, s_stat_tx_purge_drop, s_skip_count,
               s_stat_rf_audio_try, s_stat_rf_audio_ok,
               s_stat_rf_audio_fail, s_stat_rf_rx_audio_ok,
               (uint32_t)atomic_get(&s_stat_rx_drop), s_stat_rf_rx_malformed,
               s_stat_rf_rx_version_drop, s_stat_rf_rx_self_drop,
               s_stat_rf_rx_duplicate_drop, s_stat_rf_rx_inactive_drop,
               s_stat_relay_ring_drop, s_stat_control_ring_drop, s_stat_spi_out_ok,
                s_stat_spi_out_drop, s_tx_queue_depth_dbg, s_stat_bundle_tx,
                 s_stat_bundle_rx, s_stat_bundle_bad, s_stat_prev1_forwarded,
                 s_stat_prev2_forwarded, s_stat_prev1_stripped,
                 s_stat_prev2_stripped, s_stat_bundle_late_drop,
                 s_stat_bundle_max_bytes, s_stat_local_deferred_recovery,
                 esb_stats.tx_wait_us_max);
        printk("PIPE v=1 dev=nrf stage=tdma node=%u slot_due=%u slot_submit_drop=%u slot_late_drop=%u control_due=%u control_submit_drop=%u control_late_drop=%u discipline_due=%u discipline_submit_drop=%u discipline_capture_drop=%u tune_req=%u tune_clamp=%u correction_apply=%u correction_applied_us=%lld correction_pending_us=%d last_correction_us=%d commanded_period_us=%u measured_interval_us=%u callback_jitter_us=%d callback_jitter_max_us=%u skipped_frames=%u sync_acquire=%u sync_reacquire=%u sync_history_miss=%u sync_frame_diff=%d sync_phase_us=%d\n",
               s_node_id, tdma_stats.slot_due, tdma_stats.slot_submit_drop,
               tdma_stats.slot_late_drop, tdma_stats.control_due,
               tdma_stats.control_submit_drop, tdma_stats.control_late_drop,
               tdma_stats.discipline_due, tdma_stats.discipline_submit_drop,
               tdma_stats.discipline_capture_drop, tdma_stats.tune_request_count,
               tdma_stats.tune_clamp_count, tdma_stats.correction_apply_count,
               (long long)tdma_stats.correction_applied_us,
               tdma_stats.correction_pending_us, tdma_stats.last_correction_us,
               tdma_stats.commanded_period_us, tdma_stats.measured_interval_us,
               tdma_stats.callback_jitter_us, tdma_stats.callback_jitter_max_us,
               tdma_stats.skipped_frame_count, tdma_stats.sync_acquire_count,
               tdma_stats.sync_reacquire_count, tdma_stats.sync_history_miss_count,
               tdma_stats.sync_frame_diff, tdma_stats.sync_phase_correction_us);
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
            mesh_core_dedupe_reset(&s_dedupe);
            reset_all_rf_e2e_trackers();
            memset(s_relay_ring, 0, sizeof(s_relay_ring));
            memset(s_control_ring, 0, sizeof(s_control_ring));
            memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
            clear_speaker_grants();
            s_heard_bitmap = 0;
            s_relay_bitmap = 0;
            s_peer_count = 0;
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
            s_coordinator_id = 0;

            uint32_t delay_ms = scan_timeout_ms();
            uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                    s_coordinator_id);
            k_work_schedule(&s_scan_work, K_MSEC(delay_ms));
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

static uint32_t estimate_esb_tx_us(uint8_t outer_packet_bytes)
{
    return ESB_NORMAL_RAMP_US +
           8U * ((uint32_t)outer_packet_bytes + ESB_1MBPS_OVERHEAD_BYTES);
}

static void consume_local_audio_entries(uint8_t count, const char *underflow_reason)
{
    s_tx_tail = (uint8_t)((s_tx_tail + count) % TX_AUDIO_RING_SIZE);
    s_tx_queue_depth_dbg = tx_queue_depth();
    if (s_tx_queue_depth_dbg == 0) {
        s_stat_tx_underflow++;
        note_underflow(underflow_reason);
    }
}

static bool local_tail_is_active_v2(uint16_t *current_seq)
{
    audio_bundle_view_t bundle;
    const struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_tail];

    if (entry->packet_type != MESH_PKT_AUDIO_V2 ||
        !audio_bundle_parse(entry->data, entry->len, &bundle) ||
        (bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) == 0u) {
        return false;
    }
    *current_seq = bundle.current_seq;
    return true;
}

static bool deferred_tail_has_proven_successor(void)
{
    audio_bundle_view_t tail_bundle;
    audio_bundle_view_t successor_bundle;
    uint8_t successor_index;
    const struct tx_audio_entry *tail;
    const struct tx_audio_entry *successor;

    if (!s_local_deferred_pending || tx_queue_depth() < 2) {
        return false;
    }

    successor_index = (uint8_t)((s_tx_tail + 1) % TX_AUDIO_RING_SIZE);
    tail = &s_tx_audio_ring[s_tx_tail];
    successor = &s_tx_audio_ring[successor_index];
    if (tail->packet_type != MESH_PKT_AUDIO_V2 ||
        successor->packet_type != MESH_PKT_AUDIO_V2 ||
        !audio_bundle_parse(tail->data, tail->len, &tail_bundle) ||
        !audio_bundle_parse(successor->data, successor->len, &successor_bundle) ||
        tail_bundle.current_seq != s_local_deferred_seq ||
        successor_bundle.current_seq != (uint16_t)(tail_bundle.current_seq + 1u) ||
        (successor_bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_PRESENT) == 0u ||
        successor_bundle.previous1_len != tail_bundle.current_len ||
        ((successor_bundle.flags & AUDIO_BUNDLE_FLAG_PREVIOUS1_ACTIVE) != 0u) !=
            ((tail_bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE) != 0u)) {
        return false;
    }

    return memcmp(successor_bundle.previous1_data, tail_bundle.current_data,
                  tail_bundle.current_len) == 0;
}

static void transmit_relay_entry(void)
{
    __DMB();
    uint8_t packet[MESH_PACKET_OUTER_MAX];
    const struct relay_entry *queued = &s_relay_ring[s_relay_tail];
    uint8_t packet_len = queued->len;
    int ret = -EINVAL;
    bool is_v2 = packet_len > 1u && queued->data[1] == MESH_PKT_AUDIO_V2;
    bool prev1_forwarded = false;
    bool prev2_forwarded = false;

    if (packet_len >= sizeof(mesh_header_t) && packet_len <= sizeof(packet)) {
        memcpy(packet, queued->data, packet_len);
        mesh_header_t *hdr = (mesh_header_t *)packet;

        if (!is_v2) {
            ret = esb_radio_send(packet, packet_len);
        } else if (hdr->payload_len != (uint16_t)(packet_len - sizeof(*hdr))) {
            s_stat_bundle_bad++;
        } else {
            uint8_t *payload = packet + sizeof(*hdr);
            size_t bundle_len = hdr->payload_len;
            audio_bundle_view_t bundle;

            if (!audio_bundle_parse(payload, bundle_len, &bundle)) {
                s_stat_bundle_bad++;
            } else {
                prev1_forwarded = bundle.previous1_len != 0u;
                prev2_forwarded = bundle.previous2_len != 0u;
                uint32_t remaining_us = tdma_get_current_slot_remaining_us();
                uint32_t required_us = estimate_esb_tx_us(packet_len) + AUDIO_TX_MARGIN_US;
                bool bundle_valid = true;

                while (required_us > remaining_us &&
                       (prev2_forwarded || prev1_forwarded)) {
                    bool stripping_prev2 = prev2_forwarded;
                    if (audio_bundle_strip_oldest(payload, &bundle_len)) {
                        hdr->payload_len = (uint16_t)bundle_len;
                        packet_len = (uint8_t)(sizeof(*hdr) + bundle_len);
                        if (stripping_prev2) {
                            s_stat_prev2_stripped++;
                            prev2_forwarded = false;
                        } else {
                            s_stat_prev1_stripped++;
                            prev1_forwarded = false;
                        }
                        required_us = estimate_esb_tx_us(packet_len) + AUDIO_TX_MARGIN_US;
                    } else {
                        s_stat_bundle_bad++;
                        bundle_valid = false;
                        break;
                    }
                }

                if (bundle_valid && required_us > remaining_us) {
                    s_stat_bundle_late_drop++;
                    ret = -ETIME;
                } else if (bundle_valid) {
                    ret = esb_radio_send(packet, packet_len);
                }
            }
        }
    } else if (is_v2) {
        s_stat_bundle_bad++;
    }

    if (ret == 0) {
        const mesh_header_t *sent_hdr = (const mesh_header_t *)packet;
        s_stat_tx_count++;
        s_relay_bitmap |= mesh_core_node_bit(sent_hdr->src_id);
        if (is_v2) {
            s_stat_bundle_tx++;
            if (prev1_forwarded) {
                s_stat_prev1_forwarded++;
            }
            if (prev2_forwarded) {
                s_stat_prev2_forwarded++;
            }
        }
    } else if (ret != -ETIME) {
        s_stat_tx_fail++;
    }

    s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
}

enum local_tx_outcome {
    LOCAL_TX_SUCCESS,
    LOCAL_TX_FAILED,
    LOCAL_TX_FALLBACK_ORIGINAL,
};

static enum local_tx_outcome transmit_local_entry(const struct tx_audio_entry *entry,
                                                   uint8_t tx_flags,
                                                   bool retain_prev1)
{
    int ret = -EINVAL;
    bool count_e2e = false;

    if (entry->packet_type == MESH_PKT_AUDIO_V2) {
        uint8_t bundle_data[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
        size_t bundle_len = entry->len;
        audio_bundle_view_t bundle;
        bool prev1_forwarded;
        bool prev2_forwarded;
        bool prev1_stripped = false;
        bool prev2_stripped = false;

        memcpy(bundle_data, entry->data, bundle_len);
        if (!audio_bundle_parse(bundle_data, bundle_len, &bundle)) {
            s_stat_bundle_bad++;
        } else {
            prev1_forwarded = bundle.previous1_len != 0u;
            prev2_forwarded = bundle.previous2_len != 0u;
            uint32_t remaining_us = tdma_get_current_slot_remaining_us();
            uint32_t required_us =
                estimate_esb_tx_us((uint8_t)(sizeof(mesh_header_t) + bundle_len)) +
                AUDIO_TX_MARGIN_US;
            bool bundle_valid = true;

            while (required_us > remaining_us &&
                   (prev2_forwarded || prev1_forwarded)) {
                bool stripping_prev2 = prev2_forwarded;
                if (!stripping_prev2 && retain_prev1) {
                    return LOCAL_TX_FALLBACK_ORIGINAL;
                }
                if (!audio_bundle_strip_oldest(bundle_data, &bundle_len)) {
                    s_stat_bundle_bad++;
                    bundle_valid = false;
                    break;
                }
                if (stripping_prev2) {
                    prev2_forwarded = false;
                    prev2_stripped = true;
                } else {
                    prev1_forwarded = false;
                    prev1_stripped = true;
                }
                required_us =
                    estimate_esb_tx_us((uint8_t)(sizeof(mesh_header_t) + bundle_len)) +
                    AUDIO_TX_MARGIN_US;
            }

            if (prev2_stripped) {
                s_stat_prev2_stripped++;
            }
            if (prev1_stripped) {
                s_stat_prev1_stripped++;
            }
            note_audio_activity(s_node_id,
                                bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
            if (bundle_valid && required_us > remaining_us) {
                s_stat_bundle_late_drop++;
                ret = -ETIME;
            } else if (bundle_valid) {
                s_stat_rf_audio_try++;
                ret = send_packet_ex(MESH_PKT_AUDIO_V2, bundle_data, (uint16_t)bundle_len,
                                     MESH_AUDIO_TTL_DEFAULT, tx_flags, s_node_id,
                                     s_tx_seq++);
            }
            if (ret == 0) {
                s_stat_bundle_tx++;
                s_stat_bundle_max_bytes = MAX(s_stat_bundle_max_bytes, bundle_len);
                if (prev1_forwarded) {
                    s_stat_prev1_forwarded++;
                }
                if (prev2_forwarded) {
                    s_stat_prev2_forwarded++;
                }
                count_e2e = true;
            }
        }
    } else if (retain_prev1) {
        return LOCAL_TX_FALLBACK_ORIGINAL;
    } else {
        bool is_diagnostic = is_rtt_probe_payload(entry->data, entry->len);
        mesh_audio_payload_t payload = {
            .codec = MESH_AUDIO_V2_CODEC_OPUS,
            .frame_ms = MESH_FRAME_MS,
            .stream_id = s_node_id,
            .audio_flags = entry->audio_flags,
        };

        memcpy(payload.data, entry->data, entry->len);
        if (!is_diagnostic) {
            note_audio_activity(s_node_id, entry->audio_flags);
        }
        s_stat_rf_audio_try++;
        ret = send_packet_ex(MESH_PKT_AUDIO, &payload, 4 + entry->len,
                             MESH_AUDIO_TTL_DEFAULT, tx_flags, s_node_id, s_tx_seq++);
        count_e2e = !is_diagnostic;
    }

    if (ret == 0) {
        s_stat_tx_count++;
        if (count_e2e) {
            s_e2e_rf_tx_frames++;
        }
        s_stat_rf_audio_ok++;
        return LOCAL_TX_SUCCESS;
    }
    if (ret != -ETIME) {
        s_stat_tx_fail++;
        s_stat_rf_audio_fail++;
    }
    return LOCAL_TX_FAILED;
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

    bool local_pending = s_state == MESH_STATE_ACTIVE && s_tx_head != s_tx_tail;
    bool relay_pending = !relay_queue_empty();
    uint16_t deferred_seq = 0;
    bool defer_local = local_pending && relay_pending && s_relay_contention_turn &&
                       local_tail_is_active_v2(&deferred_seq);

    if (defer_local) {
        /* Leave the local tail intact until the next local turn can prove that
         * its immediate successor carries this exact current frame as prev1. */
        s_local_deferred_pending = true;
        s_local_deferred_seq = deferred_seq;
        s_relay_contention_turn = false;
        transmit_relay_entry();
    } else if (local_pending) {
        bool proven_successor = deferred_tail_has_proven_successor();
        s_local_deferred_pending = false;
        s_local_deferred_seq = 0;
        uint8_t tx_flags = MESH_FLAG_RELAY_REQUEST;
        if (is_speaker_granted(s_node_id)) {
            tx_flags |= MESH_FLAG_SPEAKER_GRANTED;
        }

        if (proven_successor) {
            uint8_t successor_index = (uint8_t)((s_tx_tail + 1) % TX_AUDIO_RING_SIZE);
            enum local_tx_outcome successor_result =
                transmit_local_entry(&s_tx_audio_ring[successor_index], tx_flags, true);
            if (successor_result == LOCAL_TX_SUCCESS) {
                consume_local_audio_entries(2, "recover0");
                s_stat_local_deferred_recovery++;
                if (relay_pending) {
                    s_relay_contention_turn = true;
                }
                return;
            }
            if (successor_result == LOCAL_TX_FAILED) {
                /* Transaction failed: preserve both entries and service the
                 * original tail on the next local-preferred slot. */
                s_relay_contention_turn = false;
                return;
            }
        }

        (void)transmit_local_entry(&s_tx_audio_ring[s_tx_tail], tx_flags, false);
        if (relay_pending && !s_relay_contention_turn) {
            s_relay_contention_turn = true;
        }
        consume_local_audio_entries(1, "drain0");
    } else if (relay_pending) {
        transmit_relay_entry();
        s_relay_contention_turn = false;
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

static int process_audio_ingress(const uint8_t *data, uint8_t len, uint8_t audio_flags,
                                 uint8_t packet_type)
{
    audio_bundle_view_t bundle;
    bool is_v2 = packet_type == MESH_PKT_AUDIO_V2;

    if (s_state != MESH_STATE_ACTIVE) {
        s_stat_ingress_inactive_drop++;
        return -EAGAIN;
    }
    if ((!is_v2 && len > MESH_MAX_AUDIO_PAYLOAD) ||
        (is_v2 && !audio_bundle_parse(data, len, &bundle))) {
        if (is_v2) {
            s_stat_bundle_bad++;
        }
        return -EMSGSIZE;
    }

    bool is_diagnostic = !is_v2 && is_rtt_probe_payload(data, len);
    uint16_t e2e_seq = 0;
    bool has_e2e_seq = false;
    if (is_v2) {
        audio_flags = bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE;
        e2e_seq = bundle.current_seq;
        has_e2e_seq = true;
        s_stat_bundle_max_bytes = MAX(s_stat_bundle_max_bytes, len);
    } else if (!is_diagnostic && len >= 2) {
        e2e_seq = ((uint16_t)data[0] << 8) | data[1];
        has_e2e_seq = true;
    }

    if (has_e2e_seq) {
        mesh_core_seq_result_t sequence =
            mesh_core_seq16_accept(&s_e2e_spi_in_src[s_node_id], e2e_seq);
        if (sequence.classification == MESH_CORE_SEQ_GAP) {
            s_e2e_spi_in_gap_evt++;
            s_e2e_spi_in_gap_fr += sequence.gap;
        } else if (sequence.classification == MESH_CORE_SEQ_OLD_RESET) {
            s_e2e_spi_in_reset_evt++;
        }
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
        s_local_deferred_pending = false;
        s_local_deferred_seq = 0;
    }

    struct tx_audio_entry *entry = &s_tx_audio_ring[s_tx_head];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->audio_flags = audio_flags;
    entry->packet_type = packet_type;

    s_tx_head = next_head;

    if (s_tx_head >= s_tx_tail) {
        s_tx_queue_depth_dbg = s_tx_head - s_tx_tail;
    } else {
        s_tx_queue_depth_dbg = TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
    }

    return 0;
}

static int queue_audio_ingress(const uint8_t *data, uint8_t len, uint8_t audio_flags,
                               uint8_t packet_type)
{
    k_mutex_lock(&s_audio_ingress_lock, K_FOREVER);
    if (!s_audio_ingress_enabled) {
        k_mutex_unlock(&s_audio_ingress_lock);
        return -EAGAIN;
    }

    struct audio_ingress_entry entry = {
        .len = len,
        .audio_flags = audio_flags,
        .packet_type = packet_type,
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

int mesh_protocol_send_audio(const uint8_t *data, uint8_t len, uint8_t audio_flags)
{
    if (data == NULL || len == 0 || len > MESH_MAX_AUDIO_PAYLOAD ||
        !is_rtt_probe_payload(data, len)) {
        return -EINVAL;
    }
    return queue_audio_ingress(data, len, audio_flags, MESH_PKT_AUDIO);
}

int mesh_protocol_send_audio_v2(const uint8_t *data, uint8_t len)
{
    audio_bundle_view_t bundle;

    if (!audio_bundle_parse(data, len, &bundle)) {
        s_stat_bundle_bad++;
        return -EINVAL;
    }
    return queue_audio_ingress(data, len,
                               bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE,
                               MESH_PKT_AUDIO_V2);
}

static void audio_ingress_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    struct audio_ingress_entry entry;

    while (k_msgq_get(&s_audio_ingress_queue, &entry, K_NO_WAIT) == 0) {
        (void)process_audio_ingress(entry.data, entry.len, entry.audio_flags,
                                    entry.packet_type);
    }
}

static void command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    do {
        if (atomic_set(&s_control_pending, 0) != 0) {
            bool enable = atomic_get(&s_requested_enabled) != 0;
            uint8_t command = (uint8_t)atomic_get(&s_requested_command);
            uint8_t generation = (uint8_t)atomic_get(&s_requested_generation);
            int result = 0;
            if (enable && s_state == MESH_STATE_IDLE) {
                result = mesh_protocol_start();
            } else if (!enable && s_state != MESH_STATE_IDLE) {
                mesh_protocol_stop();
            }
            uart_bridge_send_command_ack(command, generation, result);
            uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                    s_coordinator_id);
        }
        if (atomic_set(&s_status_pending, 0) != 0) {
            uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                                    s_coordinator_id);
        }
    } while (atomic_get(&s_control_pending) != 0 || atomic_get(&s_status_pending) != 0);
}

void mesh_protocol_request_start(uint8_t generation)
{
    atomic_set(&s_requested_enabled, 1);
    atomic_set(&s_requested_command, BRIDGE_COMMAND_MESH_START);
    atomic_set(&s_requested_generation, generation);
    atomic_set(&s_control_pending, 1);
    k_work_submit(&s_command_work);
}

void mesh_protocol_request_stop(uint8_t generation)
{
    atomic_set(&s_requested_enabled, 0);
    atomic_set(&s_requested_command, BRIDGE_COMMAND_MESH_STOP);
    atomic_set(&s_requested_generation, generation);
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
    reset_all_rf_e2e_trackers();
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
    reset_all_rf_e2e_trackers();

    /* Start RX so we can hear SYNC broadcasts from an existing coordinator */
    esb_radio_start_rx();

    /* Enter scanning state */
    s_state = MESH_STATE_SCANNING;
    uint32_t delay_ms = scan_timeout_ms();
    printk("[MESH] Scanning for existing mesh (%ums timeout)...\n", delay_ms);
    k_work_schedule(&s_scan_work, K_MSEC(delay_ms));

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
        mesh_leave_v2_payload_t leave;
        memcpy(leave.sender_addr, s_local_addr, sizeof(leave.sender_addr));
        send_packet(MESH_PKT_LEAVE, &leave, sizeof(leave));
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
    mesh_core_dedupe_reset(&s_dedupe);
    reset_all_rf_e2e_trackers();
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    memset(s_control_ring, 0, sizeof(s_control_ring));
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    clear_speaker_grants();
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    s_peer_count = 0;
    s_relay_head = 0;
    s_relay_tail = 0;
    s_control_head = 0;
    s_control_tail = 0;
    purge_tx_audio_ring();
    set_audio_ingress_enabled(false, true);
    s_skip_count = 0;

    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = 0;
    uart_bridge_send_status(s_state, s_role, bridge_peer_count(), s_node_id, s_slot_index,
                            s_coordinator_id);
    uart_bridge_send_event(BRIDGE_EVENT_MESH_STOPPED, NULL, 0);
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
