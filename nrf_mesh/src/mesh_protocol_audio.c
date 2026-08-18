/**
 * @file mesh_protocol_audio.c
 * @brief Mesh Protocol Audio Data Plane
 */

#include "mesh_protocol_internal.h"
#include "mesh_tx_metrics.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "esb_radio.h"
#include "rtt_probe_defs.h"
#include "tdma.h"
#include "uart_bridge.h"

LOG_MODULE_DECLARE(mesh);

K_MSGQ_DEFINE(s_audio_ingress_queue, sizeof(struct audio_ingress_entry), 16, 4);
K_MUTEX_DEFINE(s_audio_ingress_lock);
static struct k_work s_audio_ingress_work;

#define s_context (*mesh_protocol_context_get())
#define s_state s_context.state
#define s_role s_context.role
#define s_node_id s_context.node_id
#define s_slot_index s_context.slot_index
#define s_tx_seq s_context.tx_seq
#define s_peers s_context.peers
#define s_active_speaker_ids s_context.active_speaker_ids
#define s_relay_masks s_context.relay_masks
#define s_active_speaker_deadline_ms s_context.active_speaker_deadline_ms
#define s_speaker_active_since_ms s_context.speaker_active_since_ms
#define s_heard_bitmap s_context.heard_bitmap
#define s_relay_bitmap s_context.relay_bitmap
#define s_audio_ingress_enabled s_context.audio_ingress_enabled
#define s_tx_audio_ring s_context.tx_audio_ring
#define s_tx_head s_context.tx_head
#define s_tx_tail s_context.tx_tail
#define s_relay_contention_turn s_context.relay_contention_turn
#define s_local_deferred_pending s_context.local_deferred_pending
#define s_local_deferred_seq s_context.local_deferred_seq
#define s_relay_ring s_context.relay_ring
#define s_relay_head s_context.relay_head
#define s_relay_tail s_context.relay_tail
#define s_dedupe s_context.dedupe
#define s_stat_ingress_purge_drop s_context.stat_ingress_purge_drop
#define s_stat_tx_purge_drop s_context.stat_tx_purge_drop
#define s_stat_tx_count s_context.stat_tx_count
#define s_stat_tx_fail s_context.stat_tx_fail
#define s_stat_tx_starvation s_context.stat_tx_starvation
#define s_stat_tx_queue_drain s_context.stat_tx_queue_drain
#define s_stat_audio_fwd s_context.stat_audio_fwd
#define s_stat_tx_overwrite s_context.stat_tx_overwrite
#define s_stat_spi_audio_in s_context.stat_spi_audio_in
#define s_stat_ingress_inactive_drop s_context.stat_ingress_inactive_drop
#define s_stat_ingress_msgq_drop s_context.stat_ingress_msgq_drop
#define s_stat_tx_ring_drop s_context.stat_tx_ring_drop
#define s_stat_relay_ring_drop s_context.stat_relay_ring_drop
#define s_stat_rf_audio_try s_context.stat_rf_audio_try
#define s_stat_rf_audio_ok s_context.stat_rf_audio_ok
#define s_stat_rf_audio_fail s_context.stat_rf_audio_fail
#define s_stat_rf_rx_audio_ok s_context.stat_rf_rx_audio_ok
#define s_stat_rf_rx_malformed s_context.stat_rf_rx_malformed
#define s_stat_rf_rx_version_drop s_context.stat_rf_rx_version_drop
#define s_stat_rf_rx_self_drop s_context.stat_rf_rx_self_drop
#define s_stat_rf_rx_duplicate_drop s_context.stat_rf_rx_duplicate_drop
#define s_stat_rf_rx_inactive_drop s_context.stat_rf_rx_inactive_drop
#define s_stat_spi_out_ok s_context.stat_spi_out_ok
#define s_stat_spi_out_drop s_context.stat_spi_out_drop
#define s_stat_bundle_tx s_context.stat_bundle_tx
#define s_stat_bundle_rx s_context.stat_bundle_rx
#define s_stat_bundle_bad s_context.stat_bundle_bad
#define s_stat_prev1_forwarded s_context.stat_prev1_forwarded
#define s_stat_prev2_forwarded s_context.stat_prev2_forwarded
#define s_stat_prev1_stripped s_context.stat_prev1_stripped
#define s_stat_prev2_stripped s_context.stat_prev2_stripped
#define s_stat_bundle_late_drop s_context.stat_bundle_late_drop
#define s_stat_bundle_max_bytes s_context.stat_bundle_max_bytes
#define s_stat_local_deferred_recovery s_context.stat_local_deferred_recovery
#define s_last_audio_in_time s_context.last_audio_in_time
#define s_tx_queue_depth_dbg s_context.tx_queue_depth_dbg
#define s_e2e_spi_in_src s_context.e2e_spi_in_src
#define s_e2e_rf_rx_src s_context.e2e_rf_rx_src
#define s_e2e_spi_in_frames s_context.e2e_spi_in_frames
#define s_e2e_spi_in_gap_evt s_context.e2e_spi_in_gap_evt
#define s_e2e_spi_in_gap_fr s_context.e2e_spi_in_gap_fr
#define s_e2e_spi_in_reset_evt s_context.e2e_spi_in_reset_evt
#define s_e2e_rf_tx_frames s_context.e2e_rf_tx_frames
#define s_e2e_rf_rx_frames s_context.e2e_rf_rx_frames
#define s_e2e_rf_rx_gap_evt s_context.e2e_rf_rx_gap_evt
#define s_e2e_rf_rx_gap_fr s_context.e2e_rf_rx_gap_fr
#define s_e2e_rf_rx_reset_evt s_context.e2e_rf_rx_reset_evt
#define s_e2e_spi_out_frames s_context.e2e_spi_out_frames

static void audio_ingress_work_handler(struct k_work *work);
static void slot_tx_handler(uint8_t slot_index, uint32_t frame_counter);

void mesh_protocol_audio_init(void)
{
    k_work_init(&s_audio_ingress_work, audio_ingress_work_handler);
    tdma_set_slot_callback(slot_tx_handler);
}

void mesh_protocol_audio_cancel_work(void)
{
    k_work_cancel(&s_audio_ingress_work);
}

static bool is_rtt_probe_payload(const uint8_t *data, uint8_t len)
{
    return len == RTT_PKT_LEN && data[1] == RTT_MAGIC0 && data[2] == RTT_MAGIC1 &&
           data[3] == RTT_MAGIC2 && data[4] == RTT_MAGIC3 &&
           (data[5] == RTT_TYPE_REQ || data[5] == RTT_TYPE_RSP);
}

void mesh_protocol_audio_reset_rf_e2e_tracker(uint8_t node_id)
{
    if (!mesh_core_node_id_valid(node_id)) {
        return;
    }
    memset(&s_e2e_rf_rx_src[node_id], 0, sizeof(s_e2e_rf_rx_src[node_id]));
}

void mesh_protocol_audio_reset_all_rf_e2e_trackers(void)
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

void mesh_protocol_audio_set_ingress_enabled(bool enabled, bool purge)
{
    k_mutex_lock(&s_audio_ingress_lock, K_FOREVER);
    s_audio_ingress_enabled = enabled;
    if (purge) {
        s_stat_ingress_purge_drop += k_msgq_num_used_get(&s_audio_ingress_queue);
        k_msgq_purge(&s_audio_ingress_queue);
    }
    k_mutex_unlock(&s_audio_ingress_lock);
}

void mesh_protocol_audio_purge_tx_ring(void)
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
void mesh_protocol_audio_update_speaker_grants(void);
static uint8_t compute_relay_mask(uint8_t speaker_id);
void mesh_protocol_audio_apply_speaker_grant(const mesh_speaker_grant_payload_t *grant);
static void clear_speaker_grants(void);
static int send_speaker_grant(void);
static int send_speaker_release(const uint8_t *speaker_ids, uint8_t speaker_count);
static void note_tx_starvation(void)
{
    if (mesh_tx_metrics_should_log_starvation(s_stat_tx_starvation)) {
        int32_t tts = tdma_get_time_to_slot_us();
        printk("[TXSTARVE] total=%u r=%u id=%u sl=%d q=%u tts=%d\n",
               s_stat_tx_starvation, s_role, s_node_id, s_slot_index,
               s_tx_queue_depth_dbg, tts);
    }
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

    int64_t now = k_uptime_get();

    s_heard_bitmap |= mesh_core_node_bit(src_id);
    if (s_active_speaker_deadline_ms[src_id] <= now) {
        s_speaker_active_since_ms[src_id] = now;
    }
    s_active_speaker_deadline_ms[src_id] = now + ACTIVE_SPEAKER_TIMEOUT_MS;
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

void mesh_protocol_audio_apply_speaker_grant(const mesh_speaker_grant_payload_t *grant)
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

void mesh_protocol_audio_update_speaker_grants(void)
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

    int64_t active_since[MESH_MAX_NODES + 1] = {0};
    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES; node_id++) {
        if (s_active_speaker_deadline_ms[node_id] > now) {
            active_since[node_id] = s_speaker_active_since_ms[node_id] > 0
                                        ? s_speaker_active_since_ms[node_id]
                                        : 1;
        }
    }
    idx = (int)mesh_core_select_speakers(previous, MESH_MAX_ACTIVE_SPEAKERS,
                                         active_since, selected);
    for (int i = 0; i < idx; i++) {
        relay_masks[i] = compute_relay_mask(selected[i]);
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

    return mesh_protocol_tx_queue_control(&s_context, MESH_PKT_SPEAKER_GRANT, &payload,
                                          sizeof(payload));
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

    return mesh_protocol_tx_queue_control(&s_context, MESH_PKT_SPEAKER_RELEASE, &payload,
                                          sizeof(payload));
}

void mesh_protocol_audio_clear_relay_ring(void)
{
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    s_relay_head = 0;
    s_relay_tail = 0;
}

void mesh_protocol_audio_clear_speaker_activity(void)
{
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    memset(s_speaker_active_since_ms, 0, sizeof(s_speaker_active_since_ms));
}

void mesh_protocol_audio_clear_speaker_grants(void)
{
    clear_speaker_grants();
}

void mesh_protocol_audio_clear_heard_relay_bitmaps(void)
{
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
}

void mesh_protocol_audio_apply_slot_map_speakers(const mesh_slot_map_payload_t *slot_map)
{
    clear_speaker_grants();
    for (uint8_t i = 0; i < slot_map->active_speaker_count && i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        s_active_speaker_ids[i] = slot_map->active_speaker_ids[i];
        s_relay_masks[i] = slot_map->relay_masks[i];
    }
}

void mesh_protocol_audio_apply_speaker_release(const mesh_speaker_release_payload_t *release)
{
    for (uint8_t rel = 0; rel < release->speaker_count && rel < MESH_MAX_ACTIVE_SPEAKERS; rel++) {
        for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
            if (s_active_speaker_ids[i] == release->speaker_ids[rel]) {
                s_active_speaker_ids[i] = 0;
                s_relay_masks[i] = 0;
            }
        }
    }
}

bool mesh_protocol_audio_process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi)
{
    const mesh_header_t *hdr = (const mesh_header_t *)data;
    const uint8_t *payload = data + sizeof(mesh_header_t);
    if (hdr->type != MESH_PKT_AUDIO && hdr->type != MESH_PKT_AUDIO_V2) {
        return false;
    }
    if (hdr->type == MESH_PKT_AUDIO) {
        if (s_state != MESH_STATE_ACTIVE || hdr->payload_len <= 4) {
            if (s_state != MESH_STATE_ACTIVE) s_stat_rf_rx_inactive_drop++;
            else s_stat_rf_rx_malformed++;
            return true;
        }
        const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)payload;
        uint8_t audio_len = hdr->payload_len - 4;
        uint8_t bridge_buf[MESH_MAX_AUDIO_PAYLOAD + 1];
        if (hdr->src_id == s_node_id) { s_stat_rf_rx_self_drop++; return true; }
        if (audio_len > MESH_MAX_AUDIO_PAYLOAD) { s_stat_rf_rx_malformed++; return true; }
        if (!mesh_core_dedupe_accept(&s_dedupe, hdr->type, hdr->src_id, hdr->seq)) {
            s_stat_rf_rx_duplicate_drop++; return true;
        }
        bool is_diagnostic = is_rtt_probe_payload(audio->data, audio_len);
        if (!is_diagnostic) { s_stat_rf_rx_malformed++; return true; }
        mesh_protocol_update_peer_last_seen(hdr->src_id, rssi);
        bridge_buf[0] = audio->audio_flags;
        memcpy(&bridge_buf[1], audio->data, audio_len);
        s_stat_rf_rx_audio_ok++;
        if (uart_bridge_send_audio(hdr->src_id, bridge_buf, (uint8_t)(audio_len + 1)) == 0) {
            s_stat_audio_fwd++; s_stat_spi_out_ok++;
        } else s_stat_spi_out_drop++;
    } else {
        audio_bundle_view_t bundle;
        if (s_state != MESH_STATE_ACTIVE) { s_stat_rf_rx_inactive_drop++; return true; }
        if (!mesh_core_node_id_valid(hdr->src_id)) { s_stat_rf_rx_malformed++; return true; }
        if (hdr->src_id == s_node_id) { s_stat_rf_rx_self_drop++; return true; }
        if (!audio_bundle_parse(payload, hdr->payload_len, &bundle)) {
            s_stat_bundle_bad++; s_stat_rf_rx_malformed++; return true;
        }
        if (!mesh_core_dedupe_accept(&s_dedupe, hdr->type, hdr->src_id, hdr->seq)) {
            s_stat_rf_rx_duplicate_drop++; return true;
        }
        s_stat_bundle_rx++;
        s_stat_bundle_max_bytes = MAX(s_stat_bundle_max_bytes, hdr->payload_len);
        mesh_protocol_update_peer_last_seen(hdr->src_id, rssi);
        note_audio_activity(hdr->src_id, bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
        if (s_role == MESH_ROLE_COORDINATOR) mesh_protocol_audio_update_speaker_grants();
        track_rf_e2e_sequence(hdr->src_id, bundle.current_seq);
        s_stat_rf_rx_audio_ok++;
        if (uart_bridge_send_audio_v2(hdr->src_id, payload, (uint8_t)hdr->payload_len) == 0) {
            s_stat_audio_fwd++; s_e2e_spi_out_frames++; s_stat_spi_out_ok++;
        } else s_stat_spi_out_drop++;
    }
    if (hdr->ttl > 0 && (hdr->flags & MESH_FLAG_RELAY_REQUEST) != 0 &&
        relay_permitted_for_source(hdr->src_id, hdr->flags)) {
        (void)enqueue_relay_packet(data, (uint8_t)(sizeof(mesh_header_t) + hdr->payload_len),
                                   (uint8_t)(hdr->ttl - 1),
                                   (uint8_t)(hdr->flags | MESH_FLAG_RELAYED));
    }
    return true;
}

static uint8_t tx_queue_depth(void)
{
    if (s_tx_head >= s_tx_tail) {
        return s_tx_head - s_tx_tail;
    }
    return TX_AUDIO_RING_SIZE - s_tx_tail + s_tx_head;
}

static void consume_local_audio_entries(uint8_t count, bool local_consumed_successfully)
{
    s_tx_tail = (uint8_t)((s_tx_tail + count) % TX_AUDIO_RING_SIZE);
    s_tx_queue_depth_dbg = tx_queue_depth();
    mesh_tx_metrics_input_t input = {
        .local_consumed = true,
        .local_consumed_successfully = local_consumed_successfully,
        .local_queue_empty_after_consume = s_tx_queue_depth_dbg == 0,
    };
    mesh_tx_metrics_result_t result = mesh_tx_metrics_classify(&input);
    if (result.queue_drained) {
        s_stat_tx_queue_drain++;
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
        !audio_bundle_parse(successor->data, successor->len, &successor_bundle)) {
        return false;
    }

    return mesh_core_successor_carries_prev1(s_local_deferred_seq, &tail_bundle,
                                             &successor_bundle);
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
                bool bundle_valid = true;

                /* Candidate outer lengths: as-is, after stripping prev2,
                 * then prev1.  audio_bundle_strip_oldest() removes exactly
                 * the stripped frame's payload bytes, so the lengths are
                 * known without mutating the packet. */
                uint32_t candidates[3];
                size_t candidate_count = 0;
                uint32_t candidate_len = packet_len;
                candidates[candidate_count++] = candidate_len;
                if (prev2_forwarded) {
                    candidate_len -= (uint32_t)bundle.previous2_len;
                    candidates[candidate_count++] = candidate_len;
                }
                if (prev1_forwarded) {
                    candidate_len -= (uint32_t)bundle.previous1_len;
                    candidates[candidate_count++] = candidate_len;
                }

                int strips = mesh_core_fit_airtime(remaining_us, AUDIO_TX_MARGIN_US,
                                                   ESB_NORMAL_RAMP_US,
                                                   ESB_2MBPS_US_PER_BYTE,
                                                   ESB_2MBPS_OVERHEAD_BYTES, candidates,
                                                   candidate_count);
                /* A late drop (-1) still strips every predecessor first so
                 * the strip counters describe what the relay attempted. */
                size_t strips_needed =
                    strips >= 0 ? (size_t)strips : candidate_count - 1u;

                for (size_t i = 0; i < strips_needed; i++) {
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
                    } else {
                        s_stat_bundle_bad++;
                        bundle_valid = false;
                        break;
                    }
                }

                if (bundle_valid && strips < 0) {
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
            bool bundle_valid = true;

            /* Candidate outer lengths: as-is, after stripping prev2, then
             * prev1 (see transmit_relay_entry for the length model). */
            uint32_t candidates[3];
            size_t candidate_count = 0;
            uint32_t candidate_len = (uint32_t)(sizeof(mesh_header_t) + bundle_len);
            candidates[candidate_count++] = candidate_len;
            if (prev2_forwarded) {
                candidate_len -= (uint32_t)bundle.previous2_len;
                candidates[candidate_count++] = candidate_len;
            }
            if (prev1_forwarded) {
                candidate_len -= (uint32_t)bundle.previous1_len;
                candidates[candidate_count++] = candidate_len;
            }

            int strips = mesh_core_fit_airtime(remaining_us, AUDIO_TX_MARGIN_US,
                                               ESB_NORMAL_RAMP_US, ESB_2MBPS_US_PER_BYTE,
                                               ESB_2MBPS_OVERHEAD_BYTES, candidates,
                                               candidate_count);
            /* A late drop (-1) still walks every strip step; with
             * retain_prev1 that walk reaches the prev1 step and falls back
             * before any counter is recorded. */
            size_t strips_needed = strips >= 0 ? (size_t)strips : candidate_count - 1u;

            for (size_t i = 0; i < strips_needed; i++) {
                bool stripping_prev2 = prev2_forwarded;
                if (!stripping_prev2 && retain_prev1) {
                    /* NOTE: prev1 is the deferred frame's delivery proof.
                     * Fall back before touching strip counters or
                     * note_audio_activity(); the caller services the deferred
                     * tail on its next local-preferred slot instead. */
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
            }

            if (prev2_stripped) {
                s_stat_prev2_stripped++;
            }
            if (prev1_stripped) {
                s_stat_prev1_stripped++;
            }
            note_audio_activity(s_node_id,
                                bundle.flags & AUDIO_BUNDLE_FLAG_CURRENT_ACTIVE);
            if (bundle_valid && strips < 0) {
                s_stat_bundle_late_drop++;
                ret = -ETIME;
            } else if (bundle_valid) {
                s_stat_rf_audio_try++;
                ret = mesh_protocol_tx_send_packet_ex(MESH_PKT_AUDIO_V2, bundle_data,
                                                       (uint16_t)bundle_len,
                                                       MESH_AUDIO_TTL_DEFAULT, tx_flags,
                                                       s_node_id, s_tx_seq++);
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
        ret = mesh_protocol_tx_send_packet_ex(MESH_PKT_AUDIO, &payload, 4 + entry->len,
                                               MESH_AUDIO_TTL_DEFAULT, tx_flags, s_node_id,
                                               s_tx_seq++);
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
    bool has_audio_source = s_stat_spi_audio_in != 0 &&
                            (now - s_last_audio_in_time) < 120;

    uint8_t depth = tx_queue_depth();
    s_tx_queue_depth_dbg = depth;

    bool local_pending = s_state == MESH_STATE_ACTIVE && s_tx_head != s_tx_tail;
    bool relay_pending = !relay_queue_empty();
    uint16_t deferred_seq = 0;
    /* local_tail_is_active_v2() parses the tail entry, so keep the original
     * short-circuit: only inspect the tail once contention actually holds. */
    bool defer_local = false;
    if (local_pending && relay_pending && s_relay_contention_turn) {
        defer_local = mesh_core_defer_local_tail(local_pending, relay_pending,
                                                 s_relay_contention_turn,
                                                 local_tail_is_active_v2(&deferred_seq));
    }

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
                consume_local_audio_entries(2, true);
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

        enum local_tx_outcome local_result =
            transmit_local_entry(&s_tx_audio_ring[s_tx_tail], tx_flags, false);
        if (relay_pending && !s_relay_contention_turn) {
            s_relay_contention_turn = true;
        }
        consume_local_audio_entries(1, local_result == LOCAL_TX_SUCCESS);
    } else if (relay_pending) {
        transmit_relay_entry();
        s_relay_contention_turn = false;
    } else {
        mesh_tx_metrics_input_t input = {
            .scheduled_local_slot = true,
            .relay_pending = relay_pending,
            .ingress_recent = has_audio_source,
        };
        mesh_tx_metrics_result_t result = mesh_tx_metrics_classify(&input);
        if (result.slot_starved) {
            s_stat_tx_starvation++;
            note_tx_starvation();
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
        mesh_protocol_audio_update_speaker_grants();
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
