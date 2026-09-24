#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "mesh_internal.h"
#include "power.h"

#ifdef MESH_S31_PAIR_UNICAST
static bool pair_unicast_dest(uint8_t remote_mac[6])
{
    if (s_peer_mutex == NULL || xSemaphoreTake(s_peer_mutex, 0) != pdTRUE) return false;

    bool eligible = false;
    if (s_state == MESH_STATE_ACTIVE &&
        (s_role == MESH_ROLE_COORDINATOR || s_role == MESH_ROLE_PARTICIPANT) &&
        s_node_id != 0 && s_node_id <= MESH_MAX_NODES) {
        /* Match mesh_get_node_count() under the same peer-table lock. */
        uint8_t total = s_peer_count;
        if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT && s_node_id != 0) {
            total++;
        }
        if (total == 2) {
            unsigned remotes = 0;
            bool valid_remote = false;
            for (int i = 0; i < MESH_MAX_NODES; i++) {
                const mesh_peer_info_t *peer = &s_peers[i].info;
                if (!peer->active ||
                    memcmp(peer->mac_addr, s_local_mac, sizeof(peer->mac_addr)) == 0) continue;
                remotes++;
                if (remotes == 1) {
                    valid_remote = peer->node_id != 0 && peer->node_id <= MESH_MAX_NODES;
                    memcpy(remote_mac, peer->mac_addr, sizeof(peer->mac_addr));
                }
            }
            eligible = remotes == 1 && valid_remote;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    return eligible && esp_now_is_peer_exist(remote_mac);
}
#endif

bool relay_queue_empty(void)
{
    return s_relay_head == s_relay_tail;
}

bool enqueue_relay_packet(const uint8_t *data, uint16_t len, uint8_t ttl, uint8_t flags)
{
    if (data == NULL || len < sizeof(mesh_header_t) || ttl == 0) {
        return false;
    }

    if (len > sizeof(s_relay_ring[0].data)) {
        return false;
    }

    uint8_t next_head = (uint8_t)((s_relay_head + 1) % RELAY_RING_SIZE);
    if (next_head == s_relay_tail) {
        STATS_INC(relay_overwrite);
        s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
    }

    relay_entry_t *entry = &s_relay_ring[s_relay_head];
    memcpy(entry->data, data, len);
    entry->len = len;

    mesh_header_t *header = (mesh_header_t *)entry->data;
    header->ttl = ttl;
    header->flags = flags;

    s_relay_head = next_head;
    return true;
}

void clear_speaker_state(void)
{
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_ids, 0, sizeof(s_active_speaker_ids));
    memset(s_relay_masks, 0, sizeof(s_relay_masks));
    taskEXIT_CRITICAL(&s_speaker_mux);
}

/* NOTE: Keep speaker-state critical sections to plain array copies; never send,
 * log, or take s_peer_mutex while holding s_speaker_mux. */
void speaker_state_get(uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                       uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS])
{
    taskENTER_CRITICAL(&s_speaker_mux);
    if (ids) {
        memcpy(ids, s_active_speaker_ids, sizeof(s_active_speaker_ids));
    }
    if (masks) {
        memcpy(masks, s_relay_masks, sizeof(s_relay_masks));
    }
    taskEXIT_CRITICAL(&s_speaker_mux);
}

void speaker_state_set(const uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS],
                       const uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS])
{
    taskENTER_CRITICAL(&s_speaker_mux);
    memcpy(s_active_speaker_ids, ids, sizeof(s_active_speaker_ids));
    memcpy(s_relay_masks, masks, sizeof(s_relay_masks));
    taskEXIT_CRITICAL(&s_speaker_mux);
}

void status_bitmaps_snapshot_and_clear(uint8_t *heard, uint8_t *relayed)
{
    taskENTER_CRITICAL(&s_speaker_mux);
    *heard = s_heard_bitmap;
    *relayed = s_relay_bitmap;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);
}

void note_audio_activity(uint8_t src_id, uint8_t audio_flags)
{
    if ((audio_flags & MESH_AUDIO_FLAG_ACTIVE) == 0 || src_id == 0 || src_id > MESH_MAX_NODES) {
        return;
    }

    uint8_t bit = mesh_core_node_bit(src_id);
    int64_t deadline = (esp_timer_get_time() / 1000) + ACTIVE_SPEAKER_TIMEOUT_MS;
    taskENTER_CRITICAL(&s_speaker_mux);
    s_heard_bitmap |= bit;
    s_active_speaker_deadline_ms[src_id] = deadline;
    taskEXIT_CRITICAL(&s_speaker_mux);
}

uint8_t compute_relay_mask(uint8_t speaker_id)
{
    mesh_core_peer_snapshot_t peers[MESH_MAX_NODES];
    uint8_t local_heard;

    taskENTER_CRITICAL(&s_speaker_mux);
    local_heard = s_heard_bitmap;
    taskEXIT_CRITICAL(&s_speaker_mux);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        peers[i] = (mesh_core_peer_snapshot_t){
            .node_id = s_peers[i].info.node_id,
            .heard_bitmap = s_peers[i].info.heard_bitmap,
            .active = s_peers[i].info.active,
        };
    }
    xSemaphoreGive(s_peer_mutex);
    return mesh_core_relay_mask(speaker_id, s_node_id, local_heard, peers, MESH_MAX_NODES);
}

void send_speaker_release_for(uint8_t speaker_id)
{
    if (speaker_id == 0) {
        return;
    }

    mesh_speaker_release_payload_t payload = {
        .speaker_count = 1,
        .speaker_ids = {speaker_id, 0},
    };

    (void)mesh_send_control_packet(MESH_PKT_SPEAKER_RELEASE, &payload, sizeof(payload));
}

void update_speaker_grants(void)
{
    if (s_role != MESH_ROLE_COORDINATOR) {
        return;
    }

    uint8_t previous[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t prev_masks[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t selected[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    int64_t deadlines[MESH_MAX_NODES + 1];
    size_t idx = 0;
    int64_t now_ms = esp_timer_get_time() / 1000;

    /* Atomically snapshot the shared state, then compute outside the lock
     * (compute_relay_mask takes s_peer_mutex and must not run under the spinlock). */
    taskENTER_CRITICAL(&s_speaker_mux);
    memcpy(previous, s_active_speaker_ids, sizeof(previous));
    memcpy(prev_masks, s_relay_masks, sizeof(prev_masks));
    memcpy(deadlines, s_active_speaker_deadline_ms, sizeof(deadlines));
    taskEXIT_CRITICAL(&s_speaker_mux);

    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES && idx < MESH_MAX_ACTIVE_SPEAKERS;
         node_id++) {
        if (deadlines[node_id] > now_ms) {
            selected[idx] = node_id;
            relay_masks[idx] = compute_relay_mask(node_id);
            idx++;
        }
    }

    if (memcmp(previous, selected, sizeof(previous)) == 0 &&
        memcmp(prev_masks, relay_masks, sizeof(prev_masks)) == 0) {
        return;
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

        if (!still_selected) {
            send_speaker_release_for(previous[i]);
        }
    }

    speaker_state_set(selected, relay_masks);

    mesh_speaker_grant_payload_t payload = {0};
    memcpy(payload.speaker_ids, selected, sizeof(payload.speaker_ids));
    memcpy(payload.relay_masks, relay_masks, sizeof(payload.relay_masks));
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (selected[i] != 0) {
            payload.speaker_count++;
        }
    }

    (void)mesh_send_control_packet(MESH_PKT_SPEAKER_GRANT, &payload, sizeof(payload));
}
void clear_transient_mesh_state(void)
{
    clear_speaker_state();
    taskENTER_CRITICAL(&s_speaker_mux);
    memset(s_active_speaker_deadline_ms, 0, sizeof(s_active_speaker_deadline_ms));
    mesh_core_dedupe_reset(&s_dedupe);
    memset(s_relay_ring, 0, sizeof(s_relay_ring));
    s_relay_head = 0;
    s_relay_tail = 0;
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    taskEXIT_CRITICAL(&s_speaker_mux);

    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);
    uint32_t purged = s_jitter_buffer.count;
    STATS_ADD(jitter_purge, purged);
    STATS_SET(jitter_depth, 0);
    mesh_jitter_reset(&s_jitter_buffer);
    xSemaphoreGive(s_jitter_mutex);
    STATS_ADD(tx_purge, uxQueueMessagesWaiting(s_tx_queue));
    xQueueReset(s_tx_queue);
    reset_control_queue();
}
void handle_audio_packet(const mesh_rx_item_t *rx)
{
    STATS_INC(rx_audio_seen);
    if (rx->header.payload_len < 4) {
        STATS_INC(rx_audio_invalid);
        return;
    }

    const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)rx->payload;
    uint16_t opus_len = rx->header.payload_len - 4;

#ifdef MESH_S31_LC3_WIRE
    const uint8_t local_codec = MESH_AUDIO_CODEC_LC3;
#else
    const uint8_t local_codec = MESH_AUDIO_CODEC_OPUS;
#endif
    if (!mesh_audio_wire_payload_valid(audio->codec, audio->frame_ms, opus_len, local_codec)) {
        STATS_INC(rx_audio_invalid);
        return;
    }

    if (rx->header.src_id == s_node_id) {
        STATS_INC(rx_audio_self);
        return;
    }

    if (!mesh_core_dedupe_accept(&s_dedupe, rx->header.type, rx->header.src_id, rx->header.seq)) {
        STATS_INC(rx_audio_dup);
        return;
    }

    note_audio_activity(rx->header.src_id, audio->audio_flags);
    if (s_role == MESH_ROLE_COORDINATOR) {
        update_speaker_grants();
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;

            mesh_core_seq_result_t sequence =
                mesh_core_seq8_accept(&s_peers[i].rx_seq, rx->header.seq);
            if (sequence.classification == MESH_CORE_SEQ_GAP) {
                s_peers[i].packets_lost += sequence.gap;
                STATS_ADD(audio_frames_lost, sequence.gap);
            }
            s_peers[i].packets_received++;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    jitter_buffer_insert(audio->data, opus_len, rx->header.src_id, rx->header.seq,
                         audio->audio_flags, rx->timestamp_us);

    if (rx->header.ttl > 0 && (rx->header.flags & MESH_FLAG_RELAY_REQUEST) != 0) {
        bool relay_allowed = false;

        if ((rx->header.flags & MESH_FLAG_SPEAKER_GRANTED) != 0) {
            /* Only coordinator-granted audio follows the speaker relay mask. */
            uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS];
            uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS];
            speaker_state_get(ids, masks);
            for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
                if (ids[i] == rx->header.src_id &&
                    (masks[i] & mesh_core_node_bit(s_node_id)) != 0) {
                    relay_allowed = true;
                    break;
                }
            }
        }

        if (relay_allowed) {
            enqueue_relay_packet((const uint8_t *)&rx->header,
                                 (uint16_t)(sizeof(mesh_header_t) + rx->header.payload_len),
                                 (uint8_t)(rx->header.ttl - 1),
                                 (uint8_t)(rx->header.flags | MESH_FLAG_RELAYED));
        }
    }

    STATS_INC(audio_frames_rx);
}
mesh_tx_slot_send_result_t send_audio_in_slot(int64_t deadline_us)
{
    mesh_tx_item_t tx_item;

    if (!wait_for_tx_idle(0)) {
        mesh_note_audio_tx_busy();
        STATS_INC(tx_busy_count);
        return MESH_TX_SLOT_BUSY;
    }

    if (xQueuePeek(s_tx_queue, &tx_item, 0) != pdTRUE) {
        if (!relay_queue_empty()) {
            relay_entry_t *entry = &s_relay_ring[s_relay_tail];
            mesh_header_t *relay_header = (mesh_header_t *)entry->data;
            esp_err_t relay_ret = tracked_esp_now_send(&(tracked_esp_now_send_request_t){
                .dest_mac = s_broadcast_mac,
                .data = entry->data,
                .len = entry->len,
                .type = ((const mesh_header_t *)entry->data)->type,
                .heard_bitmap = 0,
                .relay_bitmap = 0,
                .sequence = NULL,
                .audio_origin = false,
                .deadline_us = deadline_us,
            });

            if (relay_ret == ESP_OK) {
                taskENTER_CRITICAL(&s_speaker_mux);
                s_relay_bitmap |= mesh_core_node_bit(relay_header->src_id);
                taskEXIT_CRITICAL(&s_speaker_mux);
                s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
            }
            return relay_ret == ESP_OK ? MESH_TX_SLOT_SUBMITTED : MESH_TX_SLOT_ERROR;
        }

        return MESH_TX_SLOT_EMPTY;
    }

    uint8_t buffer[sizeof(mesh_header_t) + sizeof(mesh_audio_payload_t)];
    mesh_header_t *header = (mesh_header_t *)buffer;
    mesh_audio_payload_t *audio = (mesh_audio_payload_t *)(buffer + sizeof(mesh_header_t));

    header->version = MESH_PROTOCOL_VERSION;
    header->type = MESH_PKT_AUDIO;
    header->src_id = s_node_id;
    header->seq = 0;
    header->ttl = MESH_AUDIO_TTL_DEFAULT;
    header->flags = MESH_FLAG_RELAY_REQUEST;
    header->payload_len = 4 + tx_item.len;

    uint8_t slot_ids[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(slot_ids, NULL);
    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (slot_ids[i] == s_node_id) {
            header->flags |= MESH_FLAG_SPEAKER_GRANTED;
            break;
        }
    }

#ifdef MESH_S31_LC3_WIRE
    audio->codec = MESH_AUDIO_CODEC_LC3;
#else
    audio->codec = MESH_AUDIO_CODEC_OPUS;
#endif
    audio->frame_ms = MESH_FRAME_MS;
    audio->stream_id = s_node_id;
    audio->audio_flags = tx_item.audio_flags;
    memcpy(audio->data, tx_item.data, tx_item.len);

    const uint8_t *dest_mac = s_broadcast_mac;
#ifdef MESH_S31_PAIR_UNICAST
    uint8_t remote_mac[6];
    if (pair_unicast_dest(remote_mac)) dest_mac = remote_mac;
#endif
    int64_t send_start_us = esp_timer_get_time();
    esp_err_t ret = tracked_esp_now_send(&(tracked_esp_now_send_request_t){
        .dest_mac = dest_mac,
        .data = buffer,
        .len = sizeof(mesh_header_t) + 4 + tx_item.len,
        .type = MESH_PKT_AUDIO,
        .heard_bitmap = 0,
        .relay_bitmap = 0,
        .sequence = &s_audio_tx_seq,
        .audio_origin = true,
        .deadline_us = deadline_us,
    });

    if (ret == ESP_OK) {
        if (dest_mac == s_broadcast_mac) STATS_INC(tx_broadcast_submit_ok);
        else STATS_INC(tx_unicast_submit_ok);
        mesh_timing_record(&s_stats.tx_queue_age, send_start_us - tx_item.timestamp_us);
        (void)xQueueReceive(s_tx_queue, &tx_item, 0);
        note_audio_activity(s_node_id, tx_item.audio_flags);
        if (s_role == MESH_ROLE_COORDINATOR) {
            update_speaker_grants();
        }
    }

    return ret == ESP_OK ? MESH_TX_SLOT_SUBMITTED : MESH_TX_SLOT_ERROR;
}
void jitter_buffer_insert(const uint8_t *data, uint16_t len, uint8_t src_id, uint8_t seq,
                          uint8_t audio_flags, int64_t timestamp_us)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);
    int64_t now_us = esp_timer_get_time();
    if (mesh_jitter_push(&s_jitter_buffer, data, len, src_id, seq, audio_flags,
                         timestamp_us, now_us)) {
        STATS_INC(jitter_overruns);
    }
    STATS_SET(jitter_depth, s_jitter_buffer.count);
    xSemaphoreGive(s_jitter_mutex);
}

bool jitter_buffer_pop(uint8_t *data, uint16_t *len, uint8_t *src_id, uint8_t *audio_flags,
                       int64_t *timestamp_us)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);
    int64_t now_us = esp_timer_get_time();
    mesh_jitter_entry_t entry;
    mesh_jitter_pop_result_t result = mesh_jitter_pop(&s_jitter_buffer, now_us, &entry);
    taskENTER_CRITICAL(&s_stats_mux);
    s_stats.audio_frames_late += result.expired;
    s_stats.jitter_expired_with_pending_count += result.expired_with_pending;
    if (result.expired_age_us_max > s_stats.jitter_expired_age_us_max)
        s_stats.jitter_expired_age_us_max = result.expired_age_us_max;
    if (result.delivered) s_stats.jitter_pop++;
    else s_stats.jitter_underruns++;
    s_stats.jitter_depth = s_jitter_buffer.count;
    taskEXIT_CRITICAL(&s_stats_mux);
    if (result.delivered) {
        memcpy(data, entry.data, entry.len);
        *len = entry.len;
        *src_id = entry.src_id;
        *audio_flags = entry.audio_flags;
        *timestamp_us = entry.timestamp_us;
        mesh_timing_record(&s_stats.jitter_deliver_age, now_us - entry.enqueued_us);
    }
    xSemaphoreGive(s_jitter_mutex);
    return result.delivered;
}
