#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "mesh_internal.h"
#include "power.h"

typedef struct {
    int64_t scan_start;
    int64_t last_join_attempt;
    int64_t join_deadline;
    int64_t last_keepalive;
    int64_t last_status;
    int64_t last_topology;
    int64_t last_beacon;
    int join_attempts;
    bool saw_lower_mac;
    mesh_state_t prev_state;
} mesh_task_state_t;

static void mesh_task_update_state_tracking(mesh_task_state_t *task, int64_t now)
{
    if (s_state == MESH_STATE_SCANNING && task->prev_state != MESH_STATE_SCANNING) {
        ESP_LOGI(TAG, "Entering SCANNING state, resetting scan variables");
        task->scan_start = now;
        task->saw_lower_mac = false;
        task->last_beacon = 0;
        s_contention_next_tx_us = now + (esp_random() % (CONTENTION_JITTER_MS + 1)) * 1000;
    }
    if (s_state == MESH_STATE_JOINING && task->prev_state != MESH_STATE_JOINING) {
        task->join_attempts = 0;
        task->last_join_attempt = 0;
        task->join_deadline = now + (MESH_JOIN_TIMEOUT_MS * 1000);
    }
    task->prev_state = s_state;
}

static void mesh_task_scan(mesh_task_state_t *task, int64_t now)
{
    mesh_rx_item_t rx;
    if (xQueueReceive(s_rx_queue, &rx, pdMS_TO_TICKS(50)) == pdTRUE) {
        ESP_LOGI(TAG, "SCAN: RX packet type=0x%02X from " MACSTR " (RSSI=%d)", rx.header.type,
                 MAC2STR(rx.src_mac), rx.rssi);

        handle_packet(&rx);
        if (s_state != MESH_STATE_SCANNING) {
            return;
        }

        if (rx.header.type == MESH_PKT_SYNC_V3 &&
            rx.header.payload_len == sizeof(mesh_sync_v3_payload_t) && rx.header.src_id > 0 &&
            rx.header.src_id <= MESH_MAX_NODES) {
            mesh_sync_v3_payload_t sync;
            memcpy(&sync, rx.payload, sizeof(sync));
            if ((rx.header.flags & ~MESH_FLAG_RELAYED) ||
                rx.header.ttl != ((rx.header.flags & MESH_FLAG_RELAYED) ? 1 : 2) ||
                sync.relay_depth != !!(rx.header.flags & MESH_FLAG_RELAYED) ||
                !mesh_adaptive_identity_valid(&sync.leader) || sync.leader.address_len != 6 ||
                sync.member_count < 1 || sync.member_count > MESH_MAX_NODES ||
                sync.leader_id != rx.header.src_id || !sync.term || sync.phase_us >= MESH_FRAME_US ||
                (!(rx.header.flags & MESH_FLAG_RELAYED) &&
                 memcmp(sync.leader.address, rx.src_mac, 6))) return;
            ESP_LOGI(TAG, "Found existing mesh, coordinator=%d", rx.header.src_id);
            s_coordinator_id = rx.header.src_id;
            memcpy(s_coordinator_mac, sync.leader.address, sizeof(s_coordinator_mac));
            s_mesh.discovered_term = sync.term;
            if (rx.header.flags & MESH_FLAG_RELAYED) {
                memcpy(s_mesh.discovery_relay_mac, rx.src_mac, 6);
                s_mesh.discovery_relay_ms = (uint32_t)(rx.timestamp_us / 1000);
            } else {
                s_mesh.discovery_relay_ms = 0;
            }
            set_state(MESH_STATE_JOINING);
            task->join_attempts = 0;
            task->last_join_attempt = 0;
            return;
        } else if (rx.header.type == MESH_PKT_JOIN_V3 && rx.header.src_id == 0 &&
                   rx.header.payload_len == sizeof(mesh_join_v3_payload_t) &&
                   rx.header.flags == 0 && rx.header.ttl == 2) {
            mesh_join_v3_payload_t join;
            memcpy(&join, rx.payload, sizeof(join));
            if (join.origin.address_len != 6 ||
                memcmp(join.origin.address, rx.src_mac, 6)) return;
            /* Lower MAC wins coordinator election. */
            if (mesh_core_address_compare(rx.src_mac, s_local_mac, sizeof(s_local_mac)) < 0) {
                ESP_LOGI(TAG, "SCAN: Saw node with lower MAC, deferring coordinator role");
                task->saw_lower_mac = true;
                task->scan_start = now;
            } else {
                ESP_LOGI(TAG, "SCAN: Saw node with higher MAC, we may become coordinator");
            }
        }
    }

    if ((now - task->last_beacon) > (300 * 1000)) {
        if (send_join_request() == ESP_OK) {
            task->last_beacon = now;
            ESP_LOGD(TAG, "SCAN: Sent beacon (JOIN request)");
        }
    }

    int64_t timeout_us = MESH_SCAN_TIMEOUT_MS * 1000;
    if (task->saw_lower_mac) {
        timeout_us += 1000 * 1000;
    }

    if ((now - task->scan_start) > timeout_us) {
        ESP_LOGI(TAG, "No mesh found, becoming coordinator (saw_lower_mac=%d)",
                 task->saw_lower_mac);
        s_role = MESH_ROLE_COORDINATOR;
        s_node_id = 1;
        s_slot_index = 0;
        s_coordinator_id = 1;
        memcpy(s_coordinator_mac, s_local_mac, 6);

        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        s_peers[0].info.node_id = 1;
        memcpy(s_peers[0].info.mac_addr, s_local_mac, 6);
        s_peers[0].info.slot_index = 0;
        s_peers[0].info.active = true;
        s_peers[0].info.last_seen_ms = now / 1000;
        s_peer_count = 1;
        xSemaphoreGive(s_peer_mutex);

        mesh_adaptive_local_init();
        s_mesh.slot_map = (mesh_slot_map_payload_t){.slot_count = 1,
                                                     .slot_ids = {1}};

        advance_tdma_generation();
        s_frame_start_us = esp_timer_get_time();
        arm_frame_timer(MESH_FRAME_US);

        set_state(MESH_STATE_ACTIVE);
        ESP_LOGI(TAG, "ACTIVE as coordinator, node_id=%d, slot=%d", s_node_id, s_slot_index);
    }
}

static void mesh_task_join(mesh_task_state_t *task, int64_t now)
{
    mesh_rx_item_t rx;
    while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
        handle_packet(&rx);
        if (s_state != MESH_STATE_JOINING) {
            break;
        }
    }
    if (s_state != MESH_STATE_JOINING) {
        return;
    }

    if (now >= task->join_deadline) {
        ESP_LOGW(TAG, "JOIN deadline expired, returning to election scan");
        clear_transient_mesh_state();
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        memset(s_peers, 0, sizeof(s_peers));
        s_peer_count = 0;
        xSemaphoreGive(s_peer_mutex);
        s_role = MESH_ROLE_NONE;
        s_node_id = 0;
        s_slot_index = -1;
        s_coordinator_id = 0;
        memset(s_coordinator_mac, 0, sizeof(s_coordinator_mac));
        set_state(MESH_STATE_SCANNING);
        return;
    }

    if ((now - task->last_join_attempt) > (MESH_JOIN_RETRY_MS * 1000)) {
        task->last_join_attempt = now;
        if (send_join_request() == ESP_OK) {
            task->join_attempts++;
            STATS_INC(join_attempts);
            ESP_LOGI(TAG, "JOIN request #%d sent", task->join_attempts);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

static void mesh_task_drain_active_rx_and_deliver_audio(void)
{
    mesh_rx_item_t rx;
    int processed = 0;
    while (xQueueReceive(s_rx_queue, &rx, 0) == pdTRUE) {
        int64_t handle_start_us = esp_timer_get_time();
        mesh_timing_record(&s_stats.rx_dequeue_age, handle_start_us - rx.timestamp_us);
        handle_packet(&rx);
        mesh_timing_record(&s_stats.rx_handle, esp_timer_get_time() - handle_start_us);
        if (s_state != MESH_STATE_ACTIVE) break;
        processed++;
        if (processed >= 32) {
            break;
        }
    }

    if (processed > 8) {
        ESP_LOGD(TAG, "ACTIVE: processed %d packets (high load)", processed);
    }

    if (s_state == MESH_STATE_ACTIVE && s_audio_cb) {
        uint8_t audio_data[MESH_MAX_OPUS_BYTES];
        uint16_t audio_len;
        uint8_t src_id;
        uint8_t audio_flags;
        int64_t timestamp_us;

        while (jitter_buffer_pop(audio_data, &audio_len, &src_id, &audio_flags, &timestamp_us)) {
            STATS_INC(rx_deliver);
            int64_t cb_start_us = esp_timer_get_time();
            s_audio_cb(audio_data, audio_len, src_id, audio_flags, timestamp_us);
            int64_t cb_us = esp_timer_get_time() - cb_start_us;
            mesh_timing_record(&s_stats.rx_audio_callback, cb_us);
            if (cb_us > 20000) STATS_INC(rx_audio_callback_over_20ms_count);
        }
    }
}

void mesh_rejoin_after_eviction(const mesh_rx_item_t *rx,
                                const mesh_membership_v3_payload_t *snapshot)
{
    ESP_LOGW(TAG, "Authoritative membership removed local node %u; rejoining leader %u",
             s_node_id, snapshot->leader_id);
    xSemaphoreTake(s_frame_timer_mutex, portMAX_DELAY);
    advance_tdma_generation();
    esp_timer_stop(s_frame_timer);
    esp_timer_stop(s_slot_timer);
    esp_timer_stop(s_control_timer);
    xSemaphoreGive(s_frame_timer_mutex);
    drain_slot_signal();
    clear_transient_mesh_state();
    s_mesh.slot_map = (mesh_slot_map_payload_t){0};

    int8_t leader_slot = -1;
    for (uint8_t slot = 0; slot < snapshot->slot_map.slot_count; slot++)
        if (snapshot->slot_map.slot_ids[slot] == snapshot->leader_id) leader_slot = slot;
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    memset(s_peers, 0, sizeof(s_peers));
    s_peers[0].info.node_id = snapshot->leader_id;
    memcpy(s_peers[0].info.mac_addr, snapshot->identities[snapshot->leader_id - 1].address, 6);
    s_peers[0].info.slot_index = leader_slot;
    s_peers[0].info.last_seen_ms = rx->timestamp_us / 1000;
    s_peers[0].info.active = true;
    s_peer_count = 1;
    xSemaphoreGive(s_peer_mutex);

    s_node_id = 0;
    s_slot_index = -1;
    s_role = MESH_ROLE_PARTICIPANT;
    s_coordinator_id = snapshot->leader_id;
    memcpy(s_coordinator_mac, snapshot->identities[snapshot->leader_id - 1].address, 6);
    s_mesh.discovered_term = snapshot->term;
    s_mesh.upstream_sync_us = 0;
    s_mesh.forward_sync_last_tx_ms = 0;
    if (rx->header.flags & MESH_FLAG_RELAYED) {
        memcpy(s_mesh.discovery_relay_mac, rx->src_mac, 6);
        s_mesh.discovery_relay_ms = (uint32_t)(rx->timestamp_us / 1000);
    } else s_mesh.discovery_relay_ms = 0;
    memset(&s_mesh.adaptive, 0, sizeof(s_mesh.adaptive));
    memset(s_mesh.neighbors, 0, sizeof(s_mesh.neighbors));
    set_state(MESH_STATE_JOINING);
}

static void mesh_task_run_active_maintenance(mesh_task_state_t *task, int64_t now)
{
    if ((now - task->last_keepalive) > (MESH_KEEPALIVE_INTERVAL_MS * 1000)) {
        send_keepalive();
        task->last_keepalive = now;
    }

    if ((now - task->last_status) > (MESH_STATUS_INTERVAL_MS * 1000)) {
        if (s_role == MESH_ROLE_COORDINATOR) {
            update_speaker_grants();
        }
        send_status();
        task->last_status = now;
    }

    if ((now - task->last_topology) >= MESH_ADAPTIVE_REPORT_MS * 1000LL) {
        mesh_send_topology();
        task->last_topology = now;
    }

    mesh_request_local_speaker((uint32_t)(now / 1000));

    if (s_role == MESH_ROLE_COORDINATOR &&
        (uint32_t)((now / 1000) - s_mesh.membership_publish_ms) >= MESH_ADAPTIVE_REPORT_MS)
        mesh_send_membership();

    mesh_adaptive_maintenance((uint32_t)(now / 1000));

    check_peer_timeouts();
}

static void mesh_task_dispatch_timer_event(void)
{
    QueueSetMemberHandle_t ready = xQueueSelectFromSet(s_timer_queue_set, pdMS_TO_TICKS(5));
    if (ready == s_slot_semaphore && xSemaphoreTake(s_slot_semaphore, 0) == pdTRUE) {
        service_tx_slot();
    } else if (ready == s_control_semaphore && xSemaphoreTake(s_control_semaphore, 0) == pdTRUE) {
        service_control_window();
    } else if (ready == s_frame_event_queue) {
        frame_event_t event;
        if (xQueueReceive(s_frame_event_queue, &event, 0) == pdTRUE) {
            service_frame_boundary(&event);
        }
    }
}

void mesh_task(void *arg)
{
    ESP_LOGI(TAG, "Mesh task started");

    /* The creator has committed startup before releasing this gate. Stop also
     * wakes us so a cancelled startup can exit without emitting SCANNING. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    taskENTER_CRITICAL(&s_transport_mux);
    bool announce_scanning = !s_stopping && s_state == MESH_STATE_SCANNING;
    taskEXIT_CRITICAL(&s_transport_mux);
    if (announce_scanning) {
        if (s_state_cb) s_state_cb(MESH_STATE_IDLE, MESH_STATE_SCANNING);
        ESP_LOGI(TAG, "State: %d -> %d", MESH_STATE_IDLE, MESH_STATE_SCANNING);
    }

    mesh_task_state_t task = {
        .scan_start = esp_timer_get_time(),
        .prev_state = s_state,
    };
    s_contention_next_tx_us = task.scan_start + (esp_random() % (CONTENTION_JITTER_MS + 1)) * 1000;
    int64_t last_active_loop_us = 0;

    while (1) {
        taskENTER_CRITICAL(&s_transport_mux);
        bool stopping = s_stopping;
        taskEXIT_CRITICAL(&s_transport_mux);
        if (stopping) {
            break;
        }

        int64_t now = esp_timer_get_time();
        mesh_task_update_state_tracking(&task, now);
        if (s_state == MESH_STATE_ACTIVE) {
            if (last_active_loop_us != 0) {
                int64_t gap_us = now - last_active_loop_us;
                mesh_timing_record(&s_stats.rx_loop_gap, gap_us);
                if (gap_us > 5000) STATS_INC(rx_loop_gap_over_5ms_count);
                if (gap_us > 20000) STATS_INC(rx_loop_gap_over_20ms_count);
                if (gap_us > 60000) STATS_INC(rx_loop_gap_over_60ms_count);
            }
            last_active_loop_us = now;
        } else {
            last_active_loop_us = 0;
        }

        switch (s_state) {
        case MESH_STATE_SCANNING:
            mesh_task_scan(&task, now);
            break;

        case MESH_STATE_JOINING:
            mesh_task_join(&task, now);
            break;

        case MESH_STATE_ACTIVE:
            mesh_task_drain_active_rx_and_deliver_audio();
            if (s_state == MESH_STATE_ACTIVE) {
                mesh_task_run_active_maintenance(&task, now);
                mesh_task_dispatch_timer_event();
            }
            if (s_state != MESH_STATE_ACTIVE) last_active_loop_us = 0;
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }

    s_mesh_task = NULL;
    xSemaphoreGive(s_task_stopped_semaphore);
    vTaskDelete(NULL);
}

static bool handle_speaker_grant_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len != sizeof(mesh_speaker_grant_payload_t)) return false;

    const mesh_speaker_grant_payload_t *grant = (const mesh_speaker_grant_payload_t *)rx->payload;
    if (grant->speaker_count > MESH_MAX_ACTIVE_SPEAKERS) return false;
    uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (i >= grant->speaker_count) {
            if (grant->speaker_ids[i] || grant->relay_masks[i]) return false;
            continue;
        }
        uint8_t id = grant->speaker_ids[i];
        if (!(s_mesh.adaptive.members & mesh_core_node_bit(id)) ||
            (i && id == grant->speaker_ids[0]) ||
            (grant->relay_masks[i] & (uint8_t)~s_mesh.adaptive.members)) return false;
        ids[i] = grant->speaker_ids[i];
        masks[i] = grant->relay_masks[i];
    }
    speaker_state_set(ids, masks);
    return true;
}

static bool handle_speaker_release_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len != sizeof(mesh_speaker_release_payload_t)) return false;

    const mesh_speaker_release_payload_t *release =
        (const mesh_speaker_release_payload_t *)rx->payload;
    if (release->speaker_count > MESH_MAX_ACTIVE_SPEAKERS) return false;
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (i >= release->speaker_count) {
            if (release->speaker_ids[i]) return false;
        } else if (!mesh_core_node_bit(release->speaker_ids[i]) ||
                   (i && release->speaker_ids[i] == release->speaker_ids[0])) return false;
    }
    uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(ids, masks);
    for (uint8_t rel = 0; rel < release->speaker_count && rel < MESH_MAX_ACTIVE_SPEAKERS; rel++) {
        for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
            if (ids[i] == release->speaker_ids[rel]) {
                ids[i] = 0;
                masks[i] = 0;
            }
        }
    }
    if (!ids[0] && ids[1]) {
        ids[0] = ids[1];
        masks[0] = masks[1];
        ids[1] = 0;
        masks[1] = 0;
    }
    speaker_state_set(ids, masks);
    return true;
}

static void handle_leave_packet(const mesh_rx_item_t *rx)
{
    ESP_LOGI(TAG, "Node %d leaving mesh", rx->header.src_id);
    if (rx->header.src_id == s_node_id ||
        memcmp(rx->src_mac, s_local_mac, sizeof(s_local_mac)) == 0) {
        ESP_LOGW(TAG, "Ignoring LEAVE with local identity: node_id=%u, MAC=" MACSTR,
                 rx->header.src_id, MAC2STR(rx->src_mac));
        return;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    bool peer_removed = false;
    mesh_peer_info_t removed_peer = {0};
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.active && s_peers[i].info.node_id == rx->header.src_id &&
            memcmp(s_peers[i].info.mac_addr, rx->src_mac, sizeof(s_peers[i].info.mac_addr)) == 0) {
            removed_peer = s_peers[i].info;
            s_peers[i].info.active = false;
            if (s_peer_count > 0) {
                s_peer_count--;
            }
            peer_removed = true;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    if (peer_removed) {
        mesh_core_dedupe_purge_node(&s_dedupe, rx->header.src_id);
        if (s_peer_cb) {
            s_peer_cb(&removed_peer, false);
        }
    } else {
        ESP_LOGW(TAG, "Ignoring stale/unknown LEAVE: node_id=%u, MAC=" MACSTR, rx->header.src_id,
                 MAC2STR(rx->src_mac));
    }
    if (peer_removed && s_role == MESH_ROLE_COORDINATOR) {
        mesh_adaptive_members_update();
        taskENTER_CRITICAL(&s_speaker_mux);
        s_active_speaker_deadline_ms[rx->header.src_id] = 0;
        taskEXIT_CRITICAL(&s_speaker_mux);
        update_speaker_grants();
        send_slot_map();
        mesh_send_membership();
    }
}

void handle_packet(const mesh_rx_item_t *rx)
{
    ESP_LOGD(TAG, "handle_packet: type=0x%02X from src_id=%d", rx->header.type, rx->header.src_id);

    if (mesh_handle_adaptive_control(rx)) return;

    switch (rx->header.type) {
    case MESH_PKT_AUDIO:
        handle_audio_packet(rx);
        break;

    case MESH_PKT_KEEPALIVE:
        if (mesh_adaptive_bound_sender(rx) &&
            rx->header.payload_len == sizeof(mesh_keepalive_payload_t)) {
            handle_keepalive_packet(rx);
            mesh_refresh_peer_liveness(rx->header.src_id, rx->timestamp_us);
        }
        break;

    case MESH_PKT_SLOT_MAP:
        handle_slot_map_packet(rx);
        break;

    case MESH_PKT_STATUS:
        if (mesh_adaptive_bound_sender(rx) &&
            rx->header.payload_len == sizeof(mesh_status_payload_t)) {
            handle_status_packet(rx);
            mesh_refresh_peer_liveness(rx->header.src_id, rx->timestamp_us);
        }
        break;

    case MESH_PKT_SPEAKER_GRANT:
        if (handle_speaker_grant_packet(rx))
            mesh_refresh_peer_liveness(rx->header.src_id, rx->timestamp_us);
        break;

    case MESH_PKT_SPEAKER_RELEASE:
        if (handle_speaker_release_packet(rx))
            mesh_refresh_peer_liveness(rx->header.src_id, rx->timestamp_us);
        break;

    case MESH_PKT_LEAVE:
        if (mesh_adaptive_bound_sender(rx)) handle_leave_packet(rx);
        break;

    default:
        ESP_LOGD(TAG, "Unknown packet type: 0x%02X", rx->header.type);
        break;
    }
}
void handle_join_packet(const mesh_rx_item_t *rx)
{
    if (s_role != MESH_ROLE_COORDINATOR) {
        ESP_LOGD(TAG, "Ignoring JOIN (not coordinator, role=%d)", s_role);
        return;
    }

    mesh_join_v3_payload_t join;
    memcpy(&join, rx->payload, sizeof(join));
    const uint8_t *origin_mac = join.origin.address;
    ESP_LOGI(TAG, "JOIN request from " MACSTR, MAC2STR(origin_mac));

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    bool already_joined = false;
    uint8_t existing_id = 0;
    int8_t existing_slot = -1;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (memcmp(s_peers[i].info.mac_addr, origin_mac, 6) == 0 && s_peers[i].info.active) {
            already_joined = true;
            existing_id = s_peers[i].info.node_id;
            existing_slot = s_peers[i].info.slot_index;
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            mesh_core_seq8_reset(&s_peers[i].rx_seq);
            s_peers[i].packets_received = 0;
            s_peers[i].packets_lost = 0;
            break;
        }
    }

    if (already_joined) {
        xSemaphoreGive(s_peer_mutex);
        mesh_core_dedupe_purge_node(&s_dedupe, existing_id);
        send_join_ack(existing_id, existing_slot, origin_mac);
        mesh_send_membership();
        return;
    }

    uint8_t occupied = mesh_core_node_bit(s_node_id);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.active) {
            occupied |= mesh_core_node_bit(s_peers[i].info.node_id);
        }
    }
    uint8_t new_id = mesh_core_first_free_node_id(occupied);
    int8_t new_slot = -1;
    bool used[MESH_MAX_NODES] = {0};
    if (s_slot_index >= 0) used[s_slot_index] = true;
    for (int i = 0; i < MESH_MAX_NODES; i++)
        if (s_peers[i].info.active && s_peers[i].info.slot_index >= 0)
            used[s_peers[i].info.slot_index] = true;
    for (int slot = 0; slot < MESH_MAX_NODES; slot++)
        if (!used[slot]) { new_slot = slot; break; }

    if (new_id == 0 || new_slot < 0) {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGW(TAG, "Cannot assign node - mesh full");
        return;
    }

    bool peer_added = false;
    mesh_peer_info_t added_peer = {0};
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].info.active) {
            s_peers[i].info.node_id = new_id;
            memcpy(s_peers[i].info.mac_addr, origin_mac, 6);
            s_peers[i].info.slot_index = new_slot;
            s_peers[i].info.active = true;
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            mesh_core_seq8_reset(&s_peers[i].rx_seq);
            s_peers[i].packets_received = 0;
            s_peers[i].packets_lost = 0;
            s_peer_count++;
            added_peer = s_peers[i].info;
            peer_added = true;

            ESP_LOGI(TAG, "Assigned node_id=%d, slot=%d to " MACSTR, new_id, new_slot,
                      MAC2STR(origin_mac));

            break;
        }
    }

    xSemaphoreGive(s_peer_mutex);

    if (peer_added) {
        if (s_peer_cb) {
            s_peer_cb(&added_peer, true);
        }
    }

    mesh_adaptive_members_update();
    send_join_ack(new_id, new_slot, origin_mac);
    send_slot_map();
    mesh_send_membership();
}

void handle_join_ack_packet(const mesh_rx_item_t *rx)
{
    if (s_state != MESH_STATE_JOINING && s_state != MESH_STATE_SCANNING) {
        return;
    }

    if (rx->header.payload_len != sizeof(mesh_join_ack_v3_payload_t)) {
        return;
    }

    mesh_join_ack_v3_payload_t ack;
    memcpy(&ack, rx->payload, sizeof(ack));
    uint8_t expected_coordinator = s_state == MESH_STATE_JOINING ? s_coordinator_id : 0;
    if (ack.assigned_id < 1 || ack.assigned_id > MESH_MAX_NODES ||
        ack.slot_index >= MESH_VOICE_SLOTS || ack.coordinator_id != rx->header.src_id ||
        (expected_coordinator && ack.coordinator_id != expected_coordinator) ||
        ack.target.address_len != 6 || memcmp(ack.target.address, s_local_mac, 6)) {
        return;
    }

    ESP_LOGI(TAG, "JOIN_ACK received: node_id=%d, slot=%d, coordinator=%d", ack.assigned_id,
              ack.slot_index, ack.coordinator_id);

    s_node_id = ack.assigned_id;
    s_slot_index = ack.slot_index;
    s_coordinator_id = ack.coordinator_id;
    s_role = MESH_ROLE_PARTICIPANT;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    s_peers[0].info.node_id = ack.coordinator_id;
    memcpy(s_peers[0].info.mac_addr, s_coordinator_mac, 6);
    s_peers[0].info.slot_index = 0;
    s_peers[0].info.active = true;
    s_peers[0].info.last_seen_ms = rx->timestamp_us / 1000;
    s_peer_count = 1;
    xSemaphoreGive(s_peer_mutex);

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_coordinator_mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    mesh_adaptive_init(&s_mesh.adaptive, s_node_id, s_coordinator_id, ack.term,
                       mesh_core_node_bit(s_node_id) | mesh_core_node_bit(s_coordinator_id),
                       (uint32_t)(rx->timestamp_us / 1000));
    memset(s_mesh.neighbors, 0, sizeof(s_mesh.neighbors));
    s_mesh.report_seq = 0;
    s_mesh.report_wire_seq = 0;
    s_mesh.forward_sync_last_tx_ms = 0;
    mesh_wire_identity_t self = mesh_local_identity();
    mesh_wire_identity_t leader = {.address_len = 6};
    memcpy(leader.address, s_coordinator_mac, 6);
    (void)mesh_adaptive_set_member(&s_mesh.adaptive, s_node_id, &self);
    (void)mesh_adaptive_set_member(&s_mesh.adaptive, s_coordinator_id, &leader);

    STATS_INC(join_successes);
    set_state(MESH_STATE_ACTIVE);

    ESP_LOGI(TAG, "ACTIVE as participant, node_id=%d, slot=%d", s_node_id, s_slot_index);
}

void handle_sync_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len != sizeof(mesh_sync_v3_payload_t) || rx->header.src_id == 0 ||
        rx->header.src_id > MESH_MAX_NODES) {
        return;
    }

    mesh_sync_v3_payload_t packet;
    memcpy(&packet, rx->payload, sizeof(packet));
    const mesh_sync_v3_payload_t *sync = &packet;

    if (s_role == MESH_ROLE_PARTICIPANT && s_state == MESH_STATE_ACTIVE &&
        rx->header.src_id != s_coordinator_id) {
        return;
    }
    if (s_role != MESH_ROLE_COORDINATOR &&
        memcmp(sync->leader.address, s_coordinator_mac, 6) != 0) {
        return;
    }

    STATS_INC(sync_received);

    /* Cold-start merge only: an established multi-member mesh never yields here. */
    if (s_role == MESH_ROLE_COORDINATOR &&
        memcmp(sync->leader.address, s_local_mac, 6) != 0) {
        mesh_wire_identity_t local = mesh_local_identity();
        if (s_mesh.adaptive.term != 1 || s_mesh.adaptive.prepared ||
            s_mesh.adaptive.transition_done || sync->term != 1 ||
            !mesh_adaptive_cold_join(&local,
                (uint8_t)__builtin_popcount((unsigned)s_mesh.adaptive.members),
                &sync->leader, sync->member_count)) {
            ESP_LOGW(TAG, "Ignoring foreign cold SYNC: local members=0x%02x remote count=%u",
                     s_mesh.adaptive.members, sync->member_count);
            return;
        }
        ESP_LOGW(TAG, "Joining established/lower-identity leader %u (%u members)",
                 rx->header.src_id, sync->member_count);
        s_mesh.discovered_term = sync->term;
        s_mesh.discovery_relay_ms = 0;
        demote_to_participant(sync->leader.address, rx->header.src_id);
        return;
    }

    if (s_role == MESH_ROLE_PARTICIPANT) {
        taskENTER_CRITICAL(&s_tdma_mux);
        int64_t local_frame_start_us = s_frame_start_us;
        uint32_t local_frame_counter = s_frame_counter;
        taskEXIT_CRITICAL(&s_tdma_mux);

        int32_t frame_delta = (int32_t)(sync->frame_counter - local_frame_counter);
        int64_t expected_frame_start = local_frame_start_us + (MESH_FRAME_US * frame_delta);
        int64_t actual_frame_start = rx->timestamp_us - sync->phase_us -
                                     ESPNOW_SYNC_RX_LATENCY_US;
        int32_t drift_us = (int32_t)(actual_frame_start - expected_frame_start);

        /* Bound each correction to 500 us. */
        if (drift_us > 500)
            drift_us = 500;
        if (drift_us < -500)
            drift_us = -500;

        advance_tdma_generation();
        esp_timer_stop(s_slot_timer);
        esp_timer_stop(s_control_timer);
        drain_slot_signal();
        taskENTER_CRITICAL(&s_tdma_mux);
        s_frame_start_us = actual_frame_start;
        s_frame_counter = sync->frame_counter;
        taskEXIT_CRITICAL(&s_tdma_mux);
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = (now_us - actual_frame_start) % MESH_FRAME_US;
        if (elapsed_us < 0) {
            elapsed_us += MESH_FRAME_US;
        }
        int64_t next_frame_us = MESH_FRAME_US - elapsed_us;
        esp_timer_stop(s_frame_timer);
        arm_frame_timer(next_frame_us);
        STATS_SET(clock_drift_us, drift_us);

        if (abs(drift_us) > 100) {
            STATS_INC(sync_errors);
            ESP_LOGD(TAG, "SYNC drift: %d us", drift_us);
        }
    }

    s_mesh.upstream_sync_us = rx->timestamp_us;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

void handle_keepalive_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < sizeof(mesh_keepalive_payload_t)) {
        return;
    }

    const mesh_keepalive_payload_t *ka = (const mesh_keepalive_payload_t *)rx->payload;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            s_peers[i].info.battery_pct = ka->battery_pct;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}

void handle_slot_map_packet(const mesh_rx_item_t *rx)
{
    if (s_state != MESH_STATE_ACTIVE || s_role != MESH_ROLE_PARTICIPANT ||
        rx->header.src_id != s_coordinator_id ||
        rx->header.payload_len != sizeof(mesh_slot_map_payload_t)) {
        return;
    }

    const mesh_slot_map_payload_t *slot_map = (const mesh_slot_map_payload_t *)rx->payload;
    uint8_t members = 0;
    int8_t local_slot = -1;
    if (slot_map->slot_count == 0 || slot_map->slot_count > MESH_MAX_NODES ||
        slot_map->active_speaker_count > MESH_MAX_ACTIVE_SPEAKERS) return;
    for (uint8_t slot = 0; slot < MESH_MAX_NODES; slot++) {
        uint8_t id = slot_map->slot_ids[slot];
        if (slot >= slot_map->slot_count) {
            if (id) return;
            continue;
        }
        if (!id) continue;
        uint8_t mask = mesh_core_node_bit(id);
        if (!mask || (members & mask)) return;
        members |= mask;
        if (id == s_node_id) local_slot = slot;
    }
    if (!slot_map->slot_ids[slot_map->slot_count - 1] || local_slot < 0 ||
        members != s_mesh.adaptive.members ||
        !(members & mesh_core_node_bit(s_coordinator_id))) {
        return;
    }
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        uint8_t id = slot_map->active_speaker_ids[i];
        if (i < slot_map->active_speaker_count && !(members & mesh_core_node_bit(id))) return;
        if (i && i < slot_map->active_speaker_count &&
            id == slot_map->active_speaker_ids[0]) return;
        if (i >= slot_map->active_speaker_count && (id || slot_map->relay_masks[i])) return;
        if (slot_map->relay_masks[i] & (uint8_t)~members) return;
    }

    s_mesh.slot_map = *slot_map;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (uint8_t slot = 0; slot < slot_map->slot_count; slot++) {
        uint8_t node_id = slot_map->slot_ids[slot];
        if (node_id == 0) {
            continue;
        }

        for (int i = 0; i < MESH_MAX_NODES; i++) {
            if (s_peers[i].info.active && s_peers[i].info.node_id == node_id) {
                s_peers[i].info.slot_index = (int8_t)slot;
                break;
            }
        }
    }
    s_slot_index = local_slot;
    s_peer_count = 0;
    for (int i = 0; i < MESH_MAX_NODES; i++)
        if (s_peers[i].info.active && s_peers[i].info.node_id != s_node_id) s_peer_count++;

    uint8_t sm_ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t sm_masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    for (uint8_t i = 0; i < slot_map->active_speaker_count && i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        sm_ids[i] = slot_map->active_speaker_ids[i];
        sm_masks[i] = slot_map->relay_masks[i];
    }
    speaker_state_set(sm_ids, sm_masks);
    xSemaphoreGive(s_peer_mutex);
    mesh_refresh_peer_liveness(rx->header.src_id, rx->timestamp_us);
}

void handle_status_packet(const mesh_rx_item_t *rx)
{
    if (rx->header.payload_len < sizeof(mesh_status_payload_t)) {
        return;
    }

    const mesh_status_payload_t *status = (const mesh_status_payload_t *)rx->payload;

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].info.node_id == rx->header.src_id) {
            s_peers[i].info.last_seen_ms = rx->timestamp_us / 1000;
            if (!(rx->header.flags & MESH_FLAG_RELAYED)) s_peers[i].info.rssi_dbm = rx->rssi;
            s_peers[i].info.heard_bitmap = status->heard_bitmap;
            s_peers[i].info.relay_bitmap = status->relay_bitmap;
            s_peers[i].info.battery_pct = status->battery_pct;
            s_peers[i].info.peer_count = status->peer_count;
            s_peers[i].info.fw_version = status->fw_version;
            s_peers[i].info.temperature_c = status->temperature_c;
            break;
        }
    }
    xSemaphoreGive(s_peer_mutex);
}
void set_state(mesh_state_t new_state)
{
    mesh_state_t old_state = s_state;
    s_state = new_state;

    if (s_state_cb) {
        s_state_cb(old_state, new_state);
    }

    ESP_LOGI(TAG, "State: %d -> %d", old_state, new_state);
}

void check_peer_timeouts(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    mesh_peer_info_t timed_out[MESH_MAX_NODES];
    int timed_out_count = 0;
    bool coordinator_lost = false;

    /* NOTE: Only mutate the peer table while holding its mutex. Every operation that
     * can send, take another lock, or invoke application code happens below. */
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].info.active || s_peers[i].info.node_id == s_node_id) {
            continue;
        }

        int64_t age = now_ms - s_peers[i].info.last_seen_ms;
        if (age <= MESH_NODE_TIMEOUT_MS) {
            continue;
        }

        ESP_LOGW(TAG, "Node %d timeout (last seen %lld ms ago)", s_peers[i].info.node_id, age);
        timed_out[timed_out_count++] = s_peers[i].info;
        coordinator_lost = coordinator_lost || (s_role == MESH_ROLE_PARTICIPANT &&
                                                s_peers[i].info.node_id == s_coordinator_id);
        s_peers[i].info.active = false;
        if (s_peer_count > 0) {
            s_peer_count--;
        }
        STATS_INC(node_timeouts);
    }

    xSemaphoreGive(s_peer_mutex);

    for (int t = 0; t < timed_out_count; t++) {
        uint8_t timed_out_id = timed_out[t].node_id;
        mesh_core_dedupe_purge_node(&s_dedupe, timed_out_id);
        bool was_speaker = false;
        taskENTER_CRITICAL(&s_speaker_mux);
        s_active_speaker_deadline_ms[timed_out_id] = 0;
        for (int speaker = 0; speaker < MESH_MAX_ACTIVE_SPEAKERS; speaker++) {
            if (s_active_speaker_ids[speaker] == timed_out_id) {
                s_active_speaker_ids[speaker] = 0;
                s_relay_masks[speaker] = 0;
                was_speaker = true;
            }
        }
        taskEXIT_CRITICAL(&s_speaker_mux);

        if (was_speaker && s_role == MESH_ROLE_COORDINATOR) {
            send_speaker_release_for(timed_out_id);
        }
        if (s_peer_cb) {
            s_peer_cb(&timed_out[t], false);
        }
    }

    if (timed_out_count > 0 && s_role == MESH_ROLE_COORDINATOR) {
        mesh_adaptive_members_update();
        update_speaker_grants();
        send_slot_map();
        mesh_send_membership();
    }

    if (coordinator_lost) {
        ESP_LOGI(TAG, "Coordinator lost, returning to SCANNING");
        advance_tdma_generation();
        esp_timer_stop(s_slot_timer);
        esp_timer_stop(s_frame_timer);
        esp_timer_stop(s_control_timer);
        drain_slot_signal();
        /* RX callbacks remain enabled here; the existing reset can race a new enqueue. */
        drain_rx_queue_for_reset();
        xQueueReset(s_rx_queue);
        clear_transient_mesh_state();
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        memset(s_peers, 0, sizeof(s_peers));
        s_peer_count = 0;
        xSemaphoreGive(s_peer_mutex);
        s_role = MESH_ROLE_NONE;
        s_node_id = 0;
        s_slot_index = -1;
        s_coordinator_id = 0;
        memset(s_coordinator_mac, 0, sizeof(s_coordinator_mac));
        set_state(MESH_STATE_SCANNING);
    }
}
void demote_to_participant(const uint8_t *coordinator_mac, uint8_t coordinator_id)
{
    ESP_LOGW(TAG, "Demoting from coordinator to participant (new coord=%d)", coordinator_id);

    advance_tdma_generation();
    esp_timer_stop(s_slot_timer);
    esp_timer_stop(s_frame_timer);
    esp_timer_stop(s_control_timer);
    drain_slot_signal();
    clear_transient_mesh_state();

    uint32_t cleared = drain_rx_queue_for_reset();
    if (cleared > 0) {
        ESP_LOGI(TAG, "Cleared %lu packets from RX queue during demotion", (unsigned long)cleared);
    }

    s_role = MESH_ROLE_PARTICIPANT;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = coordinator_id;
    memcpy(s_coordinator_mac, coordinator_mac, sizeof(s_coordinator_mac));

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        s_peers[i].info.active = false;
    }
    s_peer_count = 0;

    s_peers[0].info.node_id = coordinator_id;
    memcpy(s_peers[0].info.mac_addr, coordinator_mac, 6);
    s_peers[0].info.slot_index = 0;
    s_peers[0].info.active = true;
    s_peers[0].info.last_seen_ms = esp_timer_get_time() / 1000;
    s_peer_count = 1;

    xSemaphoreGive(s_peer_mutex);

    set_state(MESH_STATE_JOINING);

    ESP_LOGI(TAG, "Now in JOINING state, will request node_id from coordinator %d", coordinator_id);
}
