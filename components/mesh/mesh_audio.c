#include "mesh_internal.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "power.h"

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

    (void)send_packet(MESH_PKT_SPEAKER_RELEASE, &payload, sizeof(payload));
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

    for (uint8_t node_id = 1; node_id <= MESH_MAX_NODES && idx < MESH_MAX_ACTIVE_SPEAKERS; node_id++) {
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

    (void)send_packet(MESH_PKT_SPEAKER_GRANT, &payload, sizeof(payload));
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
    memset(s_jitter_buffer, 0, sizeof(s_jitter_buffer));
    s_jitter_read_idx = 0;
    s_jitter_write_idx = 0;
    xSemaphoreGive(s_jitter_mutex);
    xQueueReset(s_tx_queue);
    reset_control_queue();
}
void handle_audio_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < 4) {
        return;
    }

    const mesh_audio_payload_t *audio = (const mesh_audio_payload_t *)rx->payload;
    uint16_t opus_len = rx->header.payload_len - 4;

    if (opus_len > MESH_MAX_OPUS_BYTES) {
        return;
    }

    if (rx->header.src_id == s_node_id) {
        return;
    }

    if (!mesh_core_dedupe_accept(&s_dedupe, rx->header.type, rx->header.src_id,
                                 rx->header.seq)) {
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
esp_err_t send_audio_in_slot(void)
{
    mesh_tx_item_t tx_item;

    if (!wait_for_tx_idle(0)) {
        STATS_INC(slot_misses);
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueuePeek(s_tx_queue, &tx_item, 0) != pdTRUE) {
        if (!relay_queue_empty()) {
            relay_entry_t *entry = &s_relay_ring[s_relay_tail];
            mesh_header_t *relay_header = (mesh_header_t *)entry->data;
            esp_err_t relay_ret = tracked_esp_now_send(
                s_broadcast_mac, entry->data, entry->len,
                ((const mesh_header_t *)entry->data)->type, 0, 0, NULL, false);

            if (relay_ret == ESP_OK) {
                taskENTER_CRITICAL(&s_speaker_mux);
                s_relay_bitmap |= mesh_core_node_bit(relay_header->src_id);
                taskEXIT_CRITICAL(&s_speaker_mux);
                s_relay_tail = (uint8_t)((s_relay_tail + 1) % RELAY_RING_SIZE);
            } else {
                STATS_INC(slot_misses);
            }
            return relay_ret;
        }

        return ESP_OK;
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

    audio->codec = 0x01; /* Opus */
    audio->frame_ms = MESH_FRAME_MS;
    audio->stream_id = s_node_id;
    audio->audio_flags = tx_item.audio_flags;
    memcpy(audio->data, tx_item.data, tx_item.len);

    esp_err_t ret = tracked_esp_now_send(s_broadcast_mac, buffer,
                                         sizeof(mesh_header_t) + 4 + tx_item.len,
                                         MESH_PKT_AUDIO, 0, 0, &s_audio_tx_seq, true);

    if (ret == ESP_OK) {
        (void)xQueueReceive(s_tx_queue, &tx_item, 0);
        note_audio_activity(s_node_id, tx_item.audio_flags);
        if (s_role == MESH_ROLE_COORDINATOR) {
            update_speaker_grants();
        }
    } else {
        STATS_INC(slot_misses);
    }

    return ret;
}
void jitter_buffer_insert(const uint8_t *data, uint16_t len, uint8_t src_id, uint8_t seq,
                          uint8_t audio_flags, int64_t timestamp_us)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);

    jitter_entry_t *entry = &s_jitter_buffer[s_jitter_write_idx];

    if (entry->valid) {
        STATS_INC(jitter_overruns);
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    entry->src_id = src_id;
    entry->seq = seq;
    entry->audio_flags = audio_flags;
    entry->timestamp_us = timestamp_us;
    entry->enqueued_us = esp_timer_get_time();
    entry->valid = true;

    s_jitter_write_idx = (s_jitter_write_idx + 1) % MESH_JITTER_BUFFER_DEPTH;

    uint8_t depth = 0;
    for (int i = 0; i < MESH_JITTER_BUFFER_DEPTH; i++) {
        if (s_jitter_buffer[i].valid)
            depth++;
    }
    STATS_SET(jitter_depth, depth);

    xSemaphoreGive(s_jitter_mutex);
}

bool jitter_buffer_pop(uint8_t *data, uint16_t *len, uint8_t *src_id, uint8_t *audio_flags,
                       int64_t *timestamp_us)
{
    xSemaphoreTake(s_jitter_mutex, portMAX_DELAY);

    jitter_entry_t *entry = &s_jitter_buffer[s_jitter_read_idx];

    if (!entry->valid) {
        xSemaphoreGive(s_jitter_mutex);
        STATS_INC(jitter_underruns);
        return false;
    }

    int64_t age_us = esp_timer_get_time() - entry->enqueued_us;
    if (age_us > (MESH_FRAME_US * 3)) {
        STATS_INC(audio_frames_late);
        entry->valid = false;
        s_jitter_read_idx = (s_jitter_read_idx + 1) % MESH_JITTER_BUFFER_DEPTH;
        xSemaphoreGive(s_jitter_mutex);
        return false;
    }

    memcpy(data, entry->data, entry->len);
    *len = entry->len;
    *src_id = entry->src_id;
    *audio_flags = entry->audio_flags;
    *timestamp_us = entry->timestamp_us;

    entry->valid = false;
    s_jitter_read_idx = (s_jitter_read_idx + 1) % MESH_JITTER_BUFFER_DEPTH;

    xSemaphoreGive(s_jitter_mutex);
    return true;
}
