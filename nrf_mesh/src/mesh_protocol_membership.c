/**
 * @file mesh_protocol_membership.c
 * @brief Mesh Protocol Membership State Machine
 */

#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "esb_radio.h"
#include "mesh_membership.h"
#include "mesh_protocol_internal.h"
#include "tdma.h"
#include "uart_bridge.h"

LOG_MODULE_DECLARE(mesh);

#define C                              mesh_protocol_context_get()
#define s_state                        C->state
#define s_role                         C->role
#define s_node_id                      C->node_id
#define s_slot_index                   C->slot_index
#define s_coordinator_id               C->coordinator_id
#define s_participant_membership_known C->participant_membership_known
#define s_local_addr                   C->local_addr
#define s_peers                        C->peers
#define s_peer_count                   C->peer_count
#define s_last_sync_time               C->last_sync_time
#define s_join_attempts                C->join_attempts
#define s_dedupe                       C->dedupe
#define s_control_ring                 C->control_ring
#define s_control_head                 C->control_head
#define s_control_tail                 C->control_tail

static struct k_work_delayable *s_scan_work;
static struct k_work_delayable *s_join_work;
static struct k_work_delayable *s_status_work;

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

void mesh_protocol_membership_bind_work(struct k_work_delayable *scan_work,
                                        struct k_work_delayable *join_work,
                                        struct k_work_delayable *status_work)
{
    s_scan_work = scan_work;
    s_join_work = join_work;
    s_status_work = status_work;
}

uint32_t mesh_protocol_membership_scan_timeout_ms(void)
{
    uint16_t address_suffix = ((uint16_t)s_local_addr[3] << 8) | s_local_addr[4];
    return SCAN_TIMEOUT_MS + (address_suffix % (SCAN_BACKOFF_MAX_MS + 1));
}

uint8_t mesh_protocol_membership_bridge_peer_count(void)
{
    if (s_state == MESH_STATE_ACTIVE && s_role == MESH_ROLE_PARTICIPANT &&
        !s_participant_membership_known) {
        return BRIDGE_PEER_COUNT_UNKNOWN;
    }
    return s_peer_count;
}

static mesh_membership_snapshot_t membership_snapshot(void)
{
    mesh_membership_snapshot_t snapshot = {
        .state = s_state,
        .role = s_role,
        .node_id = s_node_id,
        .slot_index = s_slot_index,
        .coordinator_id = s_coordinator_id,
        .peer_count = s_peer_count,
        .participant_membership_known = s_participant_membership_known,
        .address_len = sizeof(s_local_addr),
    };
    memcpy(snapshot.local_address, s_local_addr, sizeof(s_local_addr));
    return snapshot;
}

static void apply_membership_snapshot(const mesh_membership_snapshot_t *snapshot)
{
    s_state = snapshot->state;
    s_role = snapshot->role;
    s_node_id = snapshot->node_id;
    s_slot_index = snapshot->slot_index;
    s_coordinator_id = snapshot->coordinator_id;
    s_peer_count = snapshot->peer_count;
    s_participant_membership_known = snapshot->participant_membership_known;
}

static void reset_session_data(bool clear_heard_relay_bitmaps)
{
    memset(s_peers, 0, sizeof(C->peers));
    mesh_core_dedupe_reset(&s_dedupe);
    mesh_protocol_audio_reset_all_rf_e2e_trackers();
    mesh_protocol_audio_clear_relay_ring();
    memset(s_control_ring, 0, sizeof(C->control_ring));
    mesh_protocol_audio_clear_speaker_activity();
    mesh_protocol_audio_clear_speaker_grants();
    if (clear_heard_relay_bitmaps) {
        mesh_protocol_audio_clear_heard_relay_bitmaps();
    }
    s_peer_count = 0;
    s_control_head = 0;
    s_control_tail = 0;
    mesh_protocol_audio_purge_tx_ring();
    mesh_protocol_audio_set_ingress_enabled(false, true);
}

void mesh_protocol_membership_reset_session_data(void)
{
    reset_session_data(true);
}

void mesh_protocol_update_peer_last_seen(uint8_t node_id, int8_t rssi)
{
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active && s_peers[i].node_id == node_id) {
            s_peers[i].last_seen_ms = k_uptime_get();
            s_peers[i].rssi_dbm = rssi;
            if (!s_peers[i].announced && s_role == MESH_ROLE_COORDINATOR) {
                mesh_protocol_audio_reset_rf_e2e_tracker(node_id);
                s_peers[i].announced = true;
                s_peer_count++;
                uint8_t joined_id = node_id;
                uart_bridge_send_status(s_state, s_role,
                                        mesh_protocol_membership_bridge_peer_count(), s_node_id,
                                        s_slot_index, s_coordinator_id);
                uart_bridge_send_event(BRIDGE_EVENT_PEER_JOINED, &joined_id, sizeof(joined_id));
                mesh_protocol_tx_send_slot_map(C);
            }
            break;
        }
    }
}

static void process_sync(const mesh_header_t *hdr, const uint8_t *payload, int64_t timestamp_us)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_SYNC,
        .sender_id = hdr->src_id,
        .payload_valid = hdr->payload_len == sizeof(mesh_sync_payload_t),
    };
    if (event.payload_valid) {
        const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)payload;
        event.data.sync.address_len = sizeof(sync->coordinator_addr);
        memcpy(event.data.sync.coordinator_address, sync->coordinator_addr,
               sizeof(sync->coordinator_addr));
    }
    mesh_membership_snapshot_t current = membership_snapshot();
    mesh_membership_result_t transition = mesh_membership_reduce(&current, &event);

    if (transition.action == MESH_MEMBERSHIP_ACTION_DISCOVER_COORDINATOR) {
        LOG_INF("Found mesh, coordinator=%d", hdr->src_id);
        apply_membership_snapshot(&transition.next);
        s_join_attempts = 0;
        k_work_cancel_delayable(s_scan_work);
        k_work_schedule(s_join_work, K_NO_WAIT);
    } else if (transition.action == MESH_MEMBERSHIP_ACTION_ACCEPT_SYNC) {
        const mesh_sync_payload_t *sync = (const mesh_sync_payload_t *)payload;
        int64_t frame_start_us =
            timestamp_us - (MESH_MAX_NODES * MESH_SLOT_MS * 1000) - NRF_SYNC_RX_LATENCY_US;
        tdma_sync(sync->frame_counter, sync->drift_ppm, frame_start_us);
        s_last_sync_time = k_uptime_get_32();
    } else if (transition.action == MESH_MEMBERSHIP_ACTION_DEMOTE_COORDINATOR) {
        LOG_WRN("Dual coordinator detected, joining lower-address coordinator");
        mesh_log("MESH: Dual coordinator, joining lower-address winner");
        mesh_protocol_audio_set_ingress_enabled(false, false);
        tdma_stop();
        apply_membership_snapshot(&transition.next);
        s_join_attempts = 0;
        reset_session_data(false);
        k_work_cancel_delayable(s_status_work);
        uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                s_node_id, s_slot_index, s_coordinator_id);
        k_work_schedule(s_join_work, K_NO_WAIT);
    } else if ((transition.effects & MESH_MEMBERSHIP_EFFECT_REPORT_LOCAL_WIN) != 0U) {
        LOG_INF("Dual coordinator detected, we have lower MAC - staying coordinator");
    }
}

static void process_join(const mesh_header_t *hdr, const uint8_t *payload)
{
    if (s_role != MESH_ROLE_COORDINATOR || hdr->src_id != 0 ||
        hdr->payload_len != sizeof(mesh_join_v2_payload_t)) {
        return;
    }
    const mesh_join_v2_payload_t *join = (const mesh_join_v2_payload_t *)payload;
    uint8_t assigned_id = 0;
    int8_t assigned_slot = -1;
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active &&
            memcmp(s_peers[i].esb_addr, join->requester_addr, sizeof(join->requester_addr)) == 0) {
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
                memcpy(s_peers[i].esb_addr, join->requester_addr, sizeof(s_peers[i].esb_addr));
                s_peers[i].last_seen_ms = k_uptime_get();
                s_peers[i].active = true;
                s_peers[i].announced = false;
                break;
            }
        }
    }
    if (assigned_id == 0) {
        LOG_WRN("No free slots for new node");
        return;
    }
    mesh_protocol_audio_reset_rf_e2e_tracker(assigned_id);
    mesh_protocol_tx_send_join_ack(C, assigned_id, (uint8_t)assigned_slot, join->requester_addr);
    mesh_protocol_tx_send_slot_map(C);
}

static void process_join_ack(const mesh_header_t *hdr, const uint8_t *payload)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_JOIN_ACK,
        .sender_id = hdr->src_id,
        .payload_valid = hdr->payload_len == sizeof(mesh_join_ack_v2_payload_t),
    };
    if (event.payload_valid) {
        const mesh_join_ack_v2_payload_t *ack = (const mesh_join_ack_v2_payload_t *)payload;
        event.data.join_ack.assigned_id = ack->assigned_id;
        event.data.join_ack.slot_index = ack->slot_index;
        event.data.join_ack.coordinator_id = ack->coordinator_id;
        event.data.join_ack.address_len = sizeof(ack->target_addr);
        memcpy(event.data.join_ack.target_address, ack->target_addr, sizeof(ack->target_addr));
    }
    mesh_membership_snapshot_t current = membership_snapshot();
    mesh_membership_result_t transition = mesh_membership_reduce(&current, &event);
    if (transition.action == MESH_MEMBERSHIP_ACTION_ACTIVATE_PARTICIPANT) {
        apply_membership_snapshot(&transition.next);
        mesh_protocol_audio_reset_all_rf_e2e_trackers();
        mesh_protocol_audio_purge_tx_ring();
        mesh_protocol_audio_set_ingress_enabled(false, true);
        LOG_INF("JOIN_ACK: node_id=%d, slot=%d", s_node_id, s_slot_index);
        mesh_protocol_audio_set_ingress_enabled(true, false);
        k_work_cancel_delayable(s_join_work);
        tdma_start(s_slot_index, false);
        s_last_sync_time = k_uptime_get_32();
        k_work_schedule(s_status_work, K_MSEC(STATUS_INTERVAL_MS));
        uart_bridge_send_status(s_state, s_role, BRIDGE_PEER_COUNT_UNKNOWN, s_node_id, s_slot_index,
                                s_coordinator_id);
        uart_bridge_send_event(BRIDGE_EVENT_MESH_READY, NULL, 0);
    }
}

static void process_status(const mesh_header_t *hdr, const uint8_t *payload, int8_t rssi)
{
    if (hdr->payload_len < sizeof(mesh_status_payload_t)) {
        return;
    }
    const mesh_status_payload_t *status = (const mesh_status_payload_t *)payload;
    mesh_protocol_update_peer_last_seen(hdr->src_id, rssi);
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

static void process_slot_map(const mesh_header_t *hdr, const uint8_t *payload)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_SLOT_MAP,
        .sender_id = hdr->src_id,
        .payload_valid = hdr->payload_len == sizeof(mesh_slot_map_payload_t),
    };
    if (event.payload_valid) {
        memcpy(&event.data.slot_map, payload, sizeof(event.data.slot_map));
    }
    mesh_membership_snapshot_t current = membership_snapshot();
    mesh_membership_result_t transition = mesh_membership_reduce(&current, &event);
    if (transition.action == MESH_MEMBERSHIP_ACTION_APPLY_SLOT_MAP) {
        const mesh_slot_map_payload_t *slot_map = &event.data.slot_map;
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
        apply_membership_snapshot(&transition.next);
        tdma_set_slot_index(s_slot_index);
        uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                s_node_id, s_slot_index, s_coordinator_id);
        mesh_protocol_audio_apply_slot_map_speakers(slot_map);
    }
}

static void process_leave(const mesh_header_t *hdr, const uint8_t *payload)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_LEAVE,
        .sender_id = hdr->src_id,
        .payload_valid =
            hdr->payload_len == 0U || hdr->payload_len == sizeof(mesh_leave_v2_payload_t),
    };
    if (hdr->payload_len == 0U) {
        event.data.leave.identity = MESH_MEMBERSHIP_LEAVE_LEGACY;
    } else if (hdr->payload_len == sizeof(mesh_leave_v2_payload_t)) {
        const mesh_leave_v2_payload_t *leave = (const mesh_leave_v2_payload_t *)payload;
        event.data.leave.identity = MESH_MEMBERSHIP_LEAVE_ADDRESS;
        event.data.leave.address_len = sizeof(leave->sender_addr);
        memcpy(event.data.leave.sender_address, leave->sender_addr, sizeof(leave->sender_addr));
    } else {
        event.data.leave.identity = MESH_MEMBERSHIP_LEAVE_INVALID;
    }
    int peer_index = -1;
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active && s_peers[i].node_id == hdr->src_id) {
            peer_index = i;
            event.data.leave.peer.present = true;
            event.data.leave.peer.active = true;
            event.data.leave.peer.announced = s_peers[i].announced;
            event.data.leave.peer.node_id = s_peers[i].node_id;
            event.data.leave.peer.address_len = sizeof(s_peers[i].esb_addr);
            memcpy(event.data.leave.peer.address, s_peers[i].esb_addr, sizeof(s_peers[i].esb_addr));
            break;
        }
    }
    mesh_membership_snapshot_t current = membership_snapshot();
    mesh_membership_result_t transition = mesh_membership_reduce(&current, &event);
    if (hdr->src_id == s_node_id) {
        LOG_WRN("Ignoring LEAVE with local node ID %u", hdr->src_id);
        return;
    }
    if (transition.action == MESH_MEMBERSHIP_ACTION_REMOVE_PEER && peer_index >= 0) {
        s_peers[peer_index].active = false;
        apply_membership_snapshot(&transition.next);
        mesh_core_dedupe_purge_node(&s_dedupe, transition.affected_node_id);
        mesh_protocol_audio_reset_rf_e2e_tracker(transition.affected_node_id);
        LOG_INF("Peer %u left, remaining peers: %u", transition.affected_node_id, s_peer_count);
        if ((transition.effects & MESH_MEMBERSHIP_EFFECT_REPORT_PEER_LEFT) != 0U) {
            uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                    s_node_id, s_slot_index, s_coordinator_id);
            uart_bridge_send_event(BRIDGE_EVENT_PEER_LEFT, &transition.affected_node_id,
                                   sizeof(transition.affected_node_id));
        }
    }
    if ((transition.effects & MESH_MEMBERSHIP_EFFECT_PUBLISH_SLOT_MAP) != 0U) {
        mesh_protocol_tx_send_slot_map(C);
    }
}

bool mesh_protocol_membership_process_rx_packet(const mesh_header_t *hdr, const uint8_t *payload,
                                                int8_t rssi, int64_t timestamp_us)
{
    switch (hdr->type) {
    case MESH_PKT_SYNC:
        process_sync(hdr, payload, timestamp_us);
        return true;
    case MESH_PKT_JOIN_V2:
        process_join(hdr, payload);
        return true;
    case MESH_PKT_JOIN_ACK_V2:
        process_join_ack(hdr, payload);
        return true;
    case MESH_PKT_JOIN:
    case MESH_PKT_JOIN_ACK:
        return true;
    case MESH_PKT_KEEPALIVE:
        mesh_protocol_update_peer_last_seen(hdr->src_id, rssi);
        return true;
    case MESH_PKT_STATUS:
        process_status(hdr, payload, rssi);
        return true;
    case MESH_PKT_SLOT_MAP:
        process_slot_map(hdr, payload);
        return true;
    case MESH_PKT_LEAVE:
        process_leave(hdr, payload);
        return true;
    default:
        return false;
    }
}

void mesh_protocol_membership_scan_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (s_state != MESH_STATE_SCANNING) {
        return;
    }
    LOG_INF("No mesh found, becoming coordinator");
    s_role = MESH_ROLE_COORDINATOR;
    s_node_id = 1;
    s_slot_index = 0;
    s_coordinator_id = 1;
    s_state = MESH_STATE_ACTIVE;
    mesh_protocol_audio_reset_all_rf_e2e_trackers();
    mesh_protocol_audio_set_ingress_enabled(true, false);
    s_peers[0].node_id = 1;
    esb_radio_get_address(s_peers[0].esb_addr);
    s_peers[0].slot_index = 0;
    s_peers[0].active = true;
    s_peers[0].announced = true;
    s_peers[0].last_seen_ms = k_uptime_get();
    s_peer_count = 0;
    tdma_start(s_slot_index, true);
    k_work_schedule(s_status_work, K_MSEC(STATUS_INTERVAL_MS));
    LOG_INF("ACTIVE as coordinator, node_id=%d, slot=%d", s_node_id, s_slot_index);
    uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                            s_node_id, s_slot_index, s_coordinator_id);
    uart_bridge_send_event(BRIDGE_EVENT_BECAME_COORDINATOR, NULL, 0);
    uart_bridge_send_event(BRIDGE_EVENT_MESH_READY, NULL, 0);
}

void mesh_protocol_membership_join_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (s_state != MESH_STATE_JOINING) {
        return;
    }
    if (s_join_attempts < JOIN_RETRY_COUNT) {
        mesh_protocol_tx_send_join_request(C);
        s_join_attempts++;
        LOG_INF("JOIN attempt %d/%d", s_join_attempts, JOIN_RETRY_COUNT);
        mesh_log("MESH: JOIN attempt %d/%d", s_join_attempts, JOIN_RETRY_COUNT);
        k_work_schedule(s_join_work, K_MSEC(JOIN_RETRY_MS));
    } else {
        uint32_t delay_ms = mesh_protocol_membership_scan_timeout_ms();
        LOG_WRN("JOIN timeout, rescanning for %u ms", delay_ms);
        mesh_log("MESH: JOIN timeout, rescanning for %u ms", delay_ms);
        s_state = MESH_STATE_SCANNING;
        s_role = MESH_ROLE_NONE;
        s_node_id = 0;
        s_slot_index = -1;
        s_coordinator_id = 0;
        mesh_protocol_audio_reset_all_rf_e2e_trackers();
        uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                s_node_id, s_slot_index, s_coordinator_id);
        k_work_schedule(s_scan_work, K_MSEC(delay_ms));
    }
}

void mesh_protocol_membership_check_peer_timeouts(void)
{
    int64_t now = k_uptime_get();
    bool topology_changed = false;
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (!s_peers[i].active || s_peers[i].node_id == s_node_id ||
            s_peers[i].node_id == s_coordinator_id) {
            continue;
        }
        if ((now - s_peers[i].last_seen_ms) > MESH_NODE_TIMEOUT_MS) {
            uint8_t timed_out_id = s_peers[i].node_id;
            bool announced = s_peers[i].announced;
            s_peers[i].active = false;
            mesh_core_dedupe_purge_node(&s_dedupe, timed_out_id);
            mesh_protocol_audio_reset_rf_e2e_tracker(timed_out_id);
            if (announced && s_peer_count > 0) {
                s_peer_count--;
            }
            topology_changed = true;
            LOG_WRN("Peer %u timed out (silent %lld ms), remaining peers: %u", timed_out_id,
                    (long long)(now - s_peers[i].last_seen_ms), s_peer_count);
            if (announced) {
                uart_bridge_send_event(BRIDGE_EVENT_PEER_LEFT, &timed_out_id, sizeof(timed_out_id));
            }
        }
    }
    if (topology_changed && s_role == MESH_ROLE_COORDINATOR) {
        mesh_protocol_tx_send_slot_map(C);
    }
}

bool mesh_protocol_membership_handle_coordinator_timeout(void)
{
    if (s_role != MESH_ROLE_PARTICIPANT ||
        k_uptime_get_32() - s_last_sync_time <= SYNC_TIMEOUT_MS) {
        return false;
    }
    LOG_WRN("Coordinator lost (timeout), rescanning...");
    mesh_log("MESH: Coordinator lost, rescanning");
    mesh_protocol_audio_set_ingress_enabled(false, false);
    /* SYNC_LOST is deliberately not PEER_LEFT: rescans are often transient and
     * audio can continue, so the ESP suppresses disconnect tones for it. */
    uart_bridge_send_event(BRIDGE_EVENT_SYNC_LOST, NULL, 0);
    tdma_stop();
    reset_session_data(true);
    s_state = MESH_STATE_SCANNING;
    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = 0;
    uint32_t delay_ms = mesh_protocol_membership_scan_timeout_ms();
    uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                            s_node_id, s_slot_index, s_coordinator_id);
    k_work_schedule(s_scan_work, K_MSEC(delay_ms));
    return true;
}
