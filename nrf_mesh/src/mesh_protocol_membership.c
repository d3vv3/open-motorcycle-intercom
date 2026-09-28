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
        .term = C->term,
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
    C->term = snapshot->term;
    s_peer_count = snapshot->peer_count;
    s_participant_membership_known = snapshot->participant_membership_known;
}

static void reset_session_data(bool clear_heard_relay_bitmaps)
{
    mesh_protocol_adaptive_reset();
    memset(s_peers, 0, sizeof(C->peers));
    mesh_core_dedupe_reset(&s_dedupe);
    mesh_core_dedupe_reset(&C->control_presence_dedupe);
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

void mesh_protocol_note_peer_presence(uint8_t node_id)
{
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active && s_peers[i].node_id == node_id) {
            s_peers[i].last_seen_ms = k_uptime_get();
            return;
        }
    }
}

void mesh_protocol_membership_discover(uint8_t leader_id)
{
    if (s_state != MESH_STATE_SCANNING || !mesh_core_node_id_valid(leader_id)) return;
    s_state = MESH_STATE_JOINING;
    s_coordinator_id = leader_id;
    s_join_attempts = 0;
    k_work_cancel_delayable(s_scan_work);
    k_work_schedule(s_join_work, K_NO_WAIT);
}

void mesh_protocol_membership_demote(uint8_t leader_id, uint32_t term,
                                     mesh_wire_identity_t identity)
{
    if (s_state != MESH_STATE_ACTIVE || s_role != MESH_ROLE_COORDINATOR) return;
    mesh_protocol_audio_set_ingress_enabled(false, false);
    tdma_stop();
    reset_session_data(false);
    k_work_cancel_delayable(s_status_work);
    s_role = MESH_ROLE_NONE;
    s_state = MESH_STATE_JOINING;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = leader_id;
    C->term = term;
    C->leader_identity = identity;
    s_join_attempts = 0;
    uart_bridge_send_status(s_state, s_role, 0, 0, -1, s_coordinator_id);
    k_work_schedule(s_join_work, K_NO_WAIT);
}

void mesh_protocol_membership_rejoin(uint8_t leader_id, uint32_t term,
                                     mesh_wire_identity_t identity)
{
    if (s_state != MESH_STATE_ACTIVE || s_role != MESH_ROLE_PARTICIPANT) return;
    mesh_protocol_audio_set_ingress_enabled(false, false);
    tdma_stop();
    reset_session_data(true);
    k_work_cancel_delayable(s_status_work);
    s_role = MESH_ROLE_NONE;
    s_state = MESH_STATE_JOINING;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = leader_id;
    s_participant_membership_known = false;
    C->term = term;
    C->leader_identity = identity;
    s_join_attempts = 0;
    uart_bridge_send_event(BRIDGE_EVENT_SYNC_LOST, NULL, 0);
    uart_bridge_send_status(s_state, s_role, BRIDGE_PEER_COUNT_UNKNOWN, 0, -1,
                            s_coordinator_id);
    k_work_schedule(s_join_work, K_NO_WAIT);
}

static void process_join(const mesh_header_t *hdr, const uint8_t *payload)
{
    if (s_role != MESH_ROLE_COORDINATOR || hdr->src_id != 0 ||
        hdr->payload_len != sizeof(mesh_join_v3_payload_t)) {
        return;
    }
    const mesh_join_v3_payload_t *join = (const void *)payload;
    if ((join->capabilities & MESH_CAP_LC3) == 0u || join->origin.address_len != 5 ||
        memcmp(&join->origin, &C->leader_identity, sizeof(join->origin)) == 0 ||
        memcmp(&join->target, &C->leader_identity, sizeof(join->target))) {
        return;
    }
    uint8_t assigned_id = 0;
    int8_t assigned_slot = -1;
    bool new_member = false;
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_peers[i].active &&
            memcmp(s_peers[i].esb_addr, join->origin.address, 5) == 0) {
            assigned_id = s_peers[i].node_id;
            assigned_slot = s_peers[i].slot_index;
            if (!mesh_protocol_adaptive_control_sequence_accept(assigned_id, MESH_PKT_JOIN_V3,
                                                                hdr->seq)) return;
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
        uint8_t occupied_slots = (uint8_t)(1U << s_slot_index);
        for (int i = 0; i < MESH_MAX_NODES; i++) if (s_peers[i].active &&
            s_peers[i].slot_index >= 0 && s_peers[i].slot_index < MESH_MAX_NODES)
            occupied_slots |= (uint8_t)(1U << s_peers[i].slot_index);
        for (int slot = 0; slot < MESH_MAX_NODES; slot++)
            if (!(occupied_slots & (1U << slot))) { assigned_slot = (int8_t)slot; break; }
        if (assigned_slot < 0) return;
        if (assigned_id && !mesh_protocol_adaptive_bind_member(assigned_id, &join->origin))
            return;
        for (int i = 0; assigned_id != 0 && i < MESH_MAX_NODES; i++) {
            if (!s_peers[i].active) {
                s_peers[i].node_id = assigned_id;
                s_peers[i].slot_index = assigned_slot;
                memcpy(s_peers[i].esb_addr, join->origin.address, sizeof(s_peers[i].esb_addr));
                s_peers[i].last_seen_ms = k_uptime_get();
                s_peers[i].active = true;
                s_peers[i].announced = true;
                s_peer_count++;
                new_member = true;
                break;
            }
        }
    }
    if (assigned_id == 0) {
        LOG_WRN("No free slots for new node");
        return;
    }
    if (!mesh_protocol_adaptive_bind_member(assigned_id, &join->origin)) return;
    if (new_member) (void)mesh_protocol_adaptive_control_sequence_accept(assigned_id,
                                                                          MESH_PKT_JOIN_V3,
                                                                          hdr->seq);
    mesh_protocol_audio_reset_rf_e2e_tracker(assigned_id);
    if (new_member) {
        uart_bridge_send_status(s_state, s_role, s_peer_count, s_node_id, s_slot_index,
                                s_coordinator_id);
        uart_bridge_send_event(BRIDGE_EVENT_PEER_JOINED, &assigned_id, sizeof(assigned_id));
    }
    mesh_protocol_adaptive_members();
    mesh_protocol_tx_send_join_ack(C, assigned_id, (uint8_t)assigned_slot, join->origin.address);
    mesh_protocol_tx_send_slot_map(C);
}

static void process_join_ack(const mesh_header_t *hdr, const uint8_t *payload)
{
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_JOIN_ACK,
        .sender_id = hdr->src_id,
        .payload_valid = hdr->payload_len == sizeof(mesh_join_ack_v3_payload_t),
    };
    if (event.payload_valid) {
        const mesh_join_ack_v3_payload_t *ack = (const void *)payload;
        if (ack->term != C->term || ack->target.address_len != 5) return;
        event.data.join_ack.assigned_id = ack->assigned_id;
        event.data.join_ack.slot_index = ack->slot_index;
        event.data.join_ack.coordinator_id = ack->coordinator_id;
        event.data.join_ack.address_len = ack->target.address_len;
        memcpy(event.data.join_ack.target_address, ack->target.address, 5);
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
        mesh_protocol_adaptive_members();
        for (int i = 0; i < MESH_MAX_NODES; i++) if (!s_peers[i].active) {
            s_peers[i].node_id = s_coordinator_id;
            s_peers[i].slot_index = 0;
            memcpy(s_peers[i].esb_addr, C->leader_identity.address, 5);
            s_peers[i].active = true;
            s_peers[i].announced = true;
            s_peers[i].last_seen_ms = k_uptime_get();
            break;
        }
        s_last_sync_time = k_uptime_get_32();
        k_work_schedule(s_status_work, K_MSEC(STATUS_INTERVAL_MS));
        uart_bridge_send_status(s_state, s_role, BRIDGE_PEER_COUNT_UNKNOWN, s_node_id, s_slot_index,
                                s_coordinator_id);
        uart_bridge_send_event(BRIDGE_EVENT_MESH_READY, NULL, 0);
    }
}

static void process_leave(const mesh_header_t *hdr, const uint8_t *payload)
{
    if (s_role != MESH_ROLE_COORDINATOR ||
        hdr->payload_len != sizeof(mesh_leave_v2_payload_t)) return;
    mesh_membership_event_t event = {
        .type = MESH_MEMBERSHIP_EVENT_LEAVE,
        .sender_id = hdr->src_id,
        .payload_valid = true,
    };
    const mesh_leave_v2_payload_t *leave = (const void *)payload;
    event.data.leave.identity = MESH_MEMBERSHIP_LEAVE_ADDRESS;
    event.data.leave.address_len = sizeof(leave->sender_addr);
    memcpy(event.data.leave.sender_address, leave->sender_addr, sizeof(leave->sender_addr));
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
        (void)mesh_protocol_adaptive_bind_member(transition.affected_node_id, NULL);
        apply_membership_snapshot(&transition.next);
        mesh_protocol_audio_update_speaker_grants();
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
    ARG_UNUSED(rssi);
    ARG_UNUSED(timestamp_us);
    switch (hdr->type) {
    case MESH_PKT_SYNC_V3:
        /* SYNC_V3 is disciplined by the adaptive receiver with an RF timestamp. */
        return true;
    case MESH_PKT_JOIN_V3:
        process_join(hdr, payload);
        return true;
    case MESH_PKT_JOIN_ACK_V3:
        process_join_ack(hdr, payload);
        return true;
    case MESH_PKT_SYNC:
    case MESH_PKT_JOIN_V2:
    case MESH_PKT_JOIN_ACK_V2:
        return true;
    case MESH_PKT_JOIN:
    case MESH_PKT_JOIN_ACK:
        return true;
    case MESH_PKT_KEEPALIVE:
    case MESH_PKT_STATUS:
        /* Version 5 uses bound topology/requests for presence, not legacy telemetry. */
        return true;
    case MESH_PKT_SLOT_MAP:
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
    C->term = 1;
    C->leader_identity = mesh_protocol_local_identity();
    mesh_protocol_adaptive_members();
    tdma_start(s_slot_index, true);
    tdma_set_clock_source(true);
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
        if (!s_peers[i].active || s_role != MESH_ROLE_COORDINATOR ||
            s_peers[i].node_id == s_node_id ||
            s_peers[i].node_id == s_coordinator_id) {
            continue;
        }
        if ((now - s_peers[i].last_seen_ms) > MESH_NODE_TIMEOUT_MS) {
            uint8_t timed_out_id = s_peers[i].node_id;
            bool announced = s_peers[i].announced;
            s_peers[i].active = false;
            (void)mesh_protocol_adaptive_bind_member(timed_out_id, NULL);
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
        mesh_protocol_audio_update_speaker_grants();
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
