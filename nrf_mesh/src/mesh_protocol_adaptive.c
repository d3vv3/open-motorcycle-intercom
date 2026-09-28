#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mesh_membership.h"
#include "mesh_protocol_internal.h"
#include "tdma.h"
#include "uart_bridge.h"

LOG_MODULE_DECLARE(mesh);

#define C mesh_protocol_context_get()

static uint8_t bit(uint8_t id) { return mesh_core_node_bit(id); }

bool mesh_protocol_adaptive_control_sequence_accept(uint8_t id, mesh_pkt_type_t type,
                                                     uint8_t seq)
{
    if (!bit(id)) return false;
    uint8_t *seen = type == MESH_PKT_JOIN_V3 ? &C->join_seq_seen : &C->ack_seq_seen;
    uint8_t *last = type == MESH_PKT_JOIN_V3 ? C->join_last_seq : C->ack_last_seq;
    uint8_t delta = (uint8_t)(seq - last[id - 1U]);
    if ((*seen & bit(id)) && (!delta || delta >= 128U)) return false;
    last[id - 1U] = seq;
    *seen |= bit(id);
    return true;
}

static void clear_direct(uint8_t id)
{
    if (!bit(id)) return;
    C->direct_seq_valid[id - 1] = false;
    C->direct_quality[id - 1] = 0;
    C->direct_seen[id - 1] = 0;
}

bool mesh_protocol_adaptive_bind_member(uint8_t id, const mesh_wire_identity_t *identity)
{
    if (!bit(id)) return false;
    mesh_adaptive_t *a = &C->adaptive;
    bool changed = (a->members & bit(id)) == 0 ||
        (!identity ? true :
         memcmp(&a->nodes[id - 1].identity, identity, sizeof(*identity)) != 0);
    if (!mesh_adaptive_set_member(a, id, identity)) return false;
    if (changed) {
        clear_direct(id);
        mesh_core_dedupe_purge_node(&C->dedupe, id);
        mesh_core_dedupe_purge_node(&C->control_presence_dedupe, id);
        C->join_seq_seen &= (uint8_t)~bit(id);
        C->ack_seq_seen &= (uint8_t)~bit(id);
        mesh_protocol_audio_reset_rf_e2e_tracker(id);
        C->active_speaker_deadline_ms[id] = 0;
        C->speaker_active_since_ms[id] = 0;
        for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
            if (C->active_speaker_ids[i] != id) continue;
            mesh_speaker_release_payload_t release = {.speaker_count = 1,
                .speaker_ids = {id}};
            mesh_protocol_audio_apply_speaker_release(&release);
            if (C->state == MESH_STATE_ACTIVE && C->role == MESH_ROLE_COORDINATOR)
                (void)mesh_protocol_tx_queue_control(C, MESH_PKT_SPEAKER_RELEASE,
                                                     &release, sizeof(release));
            break;
        }
    }
    return true;
}

mesh_wire_identity_t mesh_protocol_local_identity(void)
{
    mesh_wire_identity_t id = {.address_len = 5};
    memcpy(id.address, C->local_addr, 5);
    return id;
}

void mesh_protocol_adaptive_reset(void)
{
    memset(&C->adaptive, 0, sizeof(C->adaptive));
    mesh_core_dedupe_reset(&C->control_presence_dedupe);
    C->join_seq_seen = C->ack_seq_seen = 0;
    memset(C->direct_seq_valid, 0, sizeof(C->direct_seq_valid));
    memset(C->direct_quality, 0, sizeof(C->direct_quality));
    memset(C->direct_seen, 0, sizeof(C->direct_seen));
    C->graph_report_seq = 0;
    C->air_report_seq = 0;
    C->local_request_seq = 0;
    C->last_local_active_ms = 0;
    C->last_request_ms = 0;
    C->local_request_active = false;
    C->term = 0;
    C->leader_identity = (mesh_wire_identity_t){0};
    C->priority_pending = false;
    C->authoritative_members = 0;
    C->convergence_until_ms = 0;
    C->grant_hold_until_ms = 0;
    C->last_membership_tx = 0;
    C->last_sync_log = 0;
}

void mesh_protocol_adaptive_members(void)
{
    if (C->state != MESH_STATE_ACTIVE || !bit(C->node_id) || !bit(C->coordinator_id)) return;
    mesh_adaptive_t *a = &C->adaptive;
    if (!a->local_id || a->local_id != C->node_id) {
        mesh_adaptive_init(a, C->node_id, C->coordinator_id, C->term,
                           bit(C->node_id) | bit(C->coordinator_id), k_uptime_get_32());
        mesh_wire_identity_t own = mesh_protocol_local_identity();
        (void)mesh_protocol_adaptive_bind_member(C->node_id, &own);
        if (C->node_id != C->coordinator_id &&
            mesh_adaptive_identity_valid(&C->leader_identity))
            (void)mesh_protocol_adaptive_bind_member(C->coordinator_id, &C->leader_identity);
    }
}

static bool identity_for(uint8_t src, const mesh_wire_identity_t *identity)
{
    if (!bit(src) || !mesh_adaptive_identity_valid(identity)) return false;
    mesh_adaptive_member_t *member = &C->adaptive.nodes[src - 1];
    if (mesh_adaptive_identity_valid(&member->identity))
        return memcmp(&member->identity, identity, sizeof(*identity)) == 0;
    return false;
}

void mesh_protocol_adaptive_observe(const mesh_header_t *h,
                                    const mesh_topology_payload_t *report, int8_t rssi)
{
    if (h->type != MESH_PKT_TOPOLOGY || (h->flags & MESH_FLAG_RELAYED) ||
        !bit(h->src_id) || h->src_id == C->node_id ||
        !(C->adaptive.members & bit(h->src_id)) || rssi >= 0 || rssi < -127) return;
    uint8_t i = h->src_id - 1;
    uint32_t now = k_uptime_get_32();
    /* The packet sequence includes SYNC and audio; topology has its own 1 Hz counter. */
    uint32_t seq = report->report_seq;
    uint32_t delta = seq - C->direct_seq[i];
    if (C->direct_seq_valid[i] && (!delta || delta >= UINT32_C(0x80000000))) return;
    uint8_t sample = C->direct_seq_valid[i] && now - C->direct_seen[i] < 3000 ?
                     (uint8_t)(100U / delta) : 100;
    C->direct_quality[i] = C->direct_seq_valid[i] ?
        (uint8_t)((3U * C->direct_quality[i] + sample + 2U) / 4U) : sample;
    C->direct_rssi[i] = C->direct_seq_valid[i] ?
        (int8_t)((3 * (int)C->direct_rssi[i] + rssi) / 4) : rssi;
    C->direct_seq[i] = seq;
    C->direct_seq_valid[i] = true;
    C->direct_seen[i] = now;
}

static void copy_slot_members(const mesh_slot_map_payload_t *map)
{
    uint8_t members = C->adaptive.members;
    for (int i = 0; i < 8; i++) if (C->peers[i].active &&
        !(members & bit(C->peers[i].node_id))) C->peers[i].active = false;
    for (uint8_t slot = 0; slot < map->slot_count && slot < 8; slot++) {
        uint8_t id = map->slot_ids[slot];
        if (!bit(id)) continue;
        if (id == C->node_id) continue;
        bool found = false;
        for (int i = 0; i < 8; i++) if (C->peers[i].active && C->peers[i].node_id == id) {
            C->peers[i].slot_index = slot;
            const mesh_wire_identity_t *identity = &C->adaptive.nodes[id - 1].identity;
            if (identity->address_len == 5)
                memcpy(C->peers[i].esb_addr, identity->address, 5);
            found = true;
            break;
        }
        if (!found) for (int i = 0; i < 8; i++) if (!C->peers[i].active) {
            C->peers[i].node_id = id;
            C->peers[i].slot_index = slot;
            C->peers[i].active = true;
            C->peers[i].announced = true;
            const mesh_wire_identity_t *identity = &C->adaptive.nodes[id - 1].identity;
            if (identity->address_len == 5)
                memcpy(C->peers[i].esb_addr, identity->address, 5);
            C->peers[i].last_seen_ms = k_uptime_get();
            break;
        }
    }
    C->peer_count = (uint8_t)(__builtin_popcount((unsigned)members) - 1U);
    C->authoritative_members = members;
}

static void prune_grants_for_members(uint8_t members)
{
    mesh_speaker_release_payload_t release = {0};
    bool changed = false;
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        uint8_t speaker = C->active_speaker_ids[i];
        if (speaker && !(members & bit(speaker))) {
            release.speaker_ids[release.speaker_count++] = speaker;
            C->active_speaker_ids[i] = 0;
            C->relay_masks[i] = 0;
            changed = true;
        } else if (speaker) {
            uint8_t allowed = members & (uint8_t)~bit(speaker);
            uint8_t mask = C->relay_masks[i] & allowed;
            if (mask != C->relay_masks[i]) {
                C->relay_masks[i] = mask;
                changed = true;
            }
        } else if (C->relay_masks[i]) {
            C->relay_masks[i] = 0;
            changed = true;
        }
    }
    if (release.speaker_count)
        (void)mesh_protocol_tx_queue_control(C, MESH_PKT_SPEAKER_RELEASE,
                                             &release, sizeof(release));
    if (changed) {
        mesh_speaker_grant_payload_t grant = {0};
        for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
            if (!C->active_speaker_ids[i]) continue;
            uint8_t index = grant.speaker_count++;
            grant.speaker_ids[index] = C->active_speaker_ids[i];
            grant.relay_masks[index] = C->relay_masks[i];
        }
        (void)mesh_protocol_tx_queue_control(C, MESH_PKT_SPEAKER_GRANT,
                                             &grant, sizeof(grant));
    }
}

int mesh_protocol_adaptive_publish_membership(void)
{
    if (C->role != MESH_ROLE_COORDINATOR || C->state != MESH_STATE_ACTIVE) return -EAGAIN;
    mesh_protocol_adaptive_members();
    prune_grants_for_members(C->adaptive.members);
    mesh_membership_v3_payload_t snapshot = {.term = C->term, .leader_id = C->node_id,
        .slot_map = mesh_protocol_tx_slot_snapshot(C)};
    for (uint8_t id = 1; id <= 8; id++) if (C->adaptive.members & bit(id))
        snapshot.identities[id - 1] = C->adaptive.nodes[id - 1].identity;
    uint8_t members;
    if (!mesh_adaptive_membership_valid(&snapshot, &members) ||
        members != C->adaptive.members) return -EINVAL;
    int ret = mesh_protocol_tx_queue_control(C, MESH_PKT_MEMBERSHIP_V3,
                                              &snapshot, sizeof(snapshot));
    if (!ret) C->last_membership_tx = k_uptime_get_32();
    return ret;
}

bool mesh_protocol_adaptive_converging(void)
{
    return C->role == MESH_ROLE_COORDINATOR && C->adaptive.transition_done &&
           (int32_t)(C->convergence_until_ms - k_uptime_get_32()) >= 0;
}

void mesh_protocol_adaptive_local_voice(bool active)
{
    if (C->state != MESH_STATE_ACTIVE || !bit(C->node_id)) return;
    uint32_t now = k_uptime_get_32();
    uint32_t refresh_ms = 400U;
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++)
        if (C->active_speaker_ids[i] == C->node_id) refresh_ms = 1000U;
    if (C->role != MESH_ROLE_PARTICIPANT ||
        (active == C->local_request_active &&
         (!active || now - C->last_request_ms < refresh_ms))) return;
    mesh_speaker_request_payload_t request = {.term = C->term,
        .request_seq = ++C->local_request_seq,
        .frame_counter = tdma_get_frame_counter(), .active = active ? 1U : 0U};
    if (mesh_protocol_tx_queue_control(C, MESH_PKT_SPEAKER_REQUEST, &request,
                                       sizeof(request)) == 0) {
        C->local_request_active = active;
        C->last_request_ms = now;
    }
}

static void apply_transition(void)
{
    mesh_adaptive_t *a = &C->adaptive;
    uint8_t old = C->coordinator_id;
    mesh_membership_snapshot_t snapshot = {.state = C->state, .role = C->role,
        .node_id = C->node_id, .slot_index = C->slot_index,
        .coordinator_id = old, .term = C->term};
    if (!mesh_membership_apply_handover(&snapshot, old, a->leader_id, a->term)) return;
    C->role = snapshot.role;
    C->term = snapshot.term;
    C->coordinator_id = snapshot.coordinator_id;
    C->join_seq_seen = C->ack_seq_seen = 0;
    C->leader_identity = a->nodes[a->leader_id - 1].identity;
    C->local_request_active = false;
    C->last_request_ms = 0;
    a->queue_head = a->queue_count = 0;
    C->priority_pending = false;
    C->control_tail = C->control_head; /* old-term leader control is no longer authoritative */
    /* The handover map freezes membership/slots, not later speaker grants. */
    copy_slot_members(&a->pending.slot_map);
    tdma_set_clock_source(C->role == MESH_ROLE_COORDINATOR);
    if (C->role == MESH_ROLE_COORDINATOR) {
        C->convergence_until_ms = k_uptime_get_32() + MESH_ADAPTIVE_RECOVERY_GRACE_MS;
        C->grant_hold_until_ms = k_uptime_get_32() + MESH_ADAPTIVE_EXPIRE_MS;
        (void)mesh_protocol_adaptive_publish_membership();
    }
    C->last_sync_time = k_uptime_get_32();
    LOG_INF("Handover apply leader=%u term=%u frame=%u", a->leader_id, a->term,
            a->pending.switch_frame);
    uart_bridge_send_status(C->state, C->role, C->peer_count, C->node_id,
                            C->slot_index, C->coordinator_id);
}

void mesh_protocol_adaptive_frame_tick(uint32_t frame)
{
    if (C->state != MESH_STATE_ACTIVE || !C->adaptive.local_id) return;
    mesh_adaptive_t *a = &C->adaptive;
    uint32_t now = k_uptime_get_32();
    bool timed_out = a->prepared && !a->committed && C->role == MESH_ROLE_COORDINATOR &&
                      (int32_t)(now - a->deadline_ms) > 0;
    bool was_prepared = a->prepared;
    mesh_handover_payload_t cancelled = a->pending;
    if (mesh_adaptive_tick(a, frame, now)) apply_transition();
    if (C->state == MESH_STATE_ACTIVE) {
        if (C->role == MESH_ROLE_PARTICIPANT && C->local_request_active &&
            now - C->last_local_active_ms > 120U)
            mesh_protocol_adaptive_local_voice(false);
        else if (C->role == MESH_ROLE_PARTICIPANT && C->local_request_active &&
                 now - C->last_request_ms >=
                     (C->active_speaker_ids[0] == C->node_id ||
                      C->active_speaker_ids[1] == C->node_id ? 1000U : 400U))
            mesh_protocol_adaptive_local_voice(true);
        if (C->role == MESH_ROLE_COORDINATOR)
            mesh_protocol_audio_expire_speakers(now);
    }
    if (was_prepared && !a->prepared && !a->transition_done && !timed_out &&
        C->priority_pending) {
        mesh_header_t *pending = (void *)C->priority_control.data;
        if (pending->type >= MESH_PKT_HANDOVER_PREPARE &&
            pending->type <= MESH_PKT_HANDOVER_CANCEL) C->priority_pending = false;
    }
    if (timed_out) {
        if (C->priority_pending) {
            mesh_header_t *pending = (void *)C->priority_control.data;
            if (pending->type == MESH_PKT_HANDOVER_PREPARE) C->priority_pending = false;
        }
        if (mesh_protocol_tx_queue_priority(MESH_PKT_HANDOVER_CANCEL,
                                            &cancelled, sizeof(cancelled)) != 0)
            (void)mesh_protocol_tx_queue_control(C, MESH_PKT_HANDOVER_CANCEL,
                                                 &cancelled, sizeof(cancelled));
        LOG_WRN("Handover cancel candidate=%u", cancelled.candidate_id);
    }
    if (C->role != MESH_ROLE_COORDINATOR || !a->prepared || a->transition_done) return;
    if (!a->committed && mesh_adaptive_commit(a, &a->pending, now))
        LOG_INF("Handover commit candidate=%u frame=%u", a->pending.candidate_id,
                a->pending.switch_frame);
    uint32_t *last = a->committed ? &C->last_commit_tx : &C->last_prepare_tx;
    if (now - *last >= 120) {
        if (mesh_protocol_tx_queue_priority(a->committed ? MESH_PKT_HANDOVER_COMMIT :
                MESH_PKT_HANDOVER_PREPARE, &a->pending, sizeof(a->pending)) == 0) *last = now;
    }
}

void mesh_protocol_adaptive_report_tick(void)
{
    if (C->state != MESH_STATE_ACTIVE || !C->adaptive.local_id) return;
    uint32_t now = k_uptime_get_32();
    mesh_protocol_adaptive_members();
    if (C->role == MESH_ROLE_COORDINATOR &&
        (C->last_membership_tx == 0 || now - C->last_membership_tx >= MESH_ADAPTIVE_REPORT_MS))
        (void)mesh_protocol_adaptive_publish_membership();
    mesh_topology_payload_t report = {.report_seq = ++C->graph_report_seq,
        .origin = mesh_protocol_local_identity(), .leader_id = C->coordinator_id, .term = C->term};
    report.slot_map = mesh_protocol_tx_slot_snapshot(C);
    for (uint8_t i = 0; i < 8; i++) {
        if (i + 1 == C->node_id || !(C->adaptive.members & bit(i + 1)) ||
            !C->direct_seq_valid[i] || now - C->direct_seen[i] > MESH_ADAPTIVE_EXPIRE_MS)
            continue;
        report.direct_mask |= bit(i + 1);
        report.rssi_dbm[i] = C->direct_rssi[i];
        report.quality[i] = C->direct_quality[i];
    }
    bool accepted = mesh_adaptive_report(&C->adaptive, C->node_id, &report, false, now);
    int queued = accepted ? mesh_protocol_tx_queue_control(C, MESH_PKT_TOPOLOGY,
                                                            &report, sizeof(report)) : -EINVAL;
    LOG_INF("Topology id=%u edges=0x%02x term=%u valid=%u queue=%d tx=%u rx_direct=%u "
            "rx_relay=%u", C->node_id, report.direct_mask, C->term, accepted, queued,
            C->stat_topology_tx_ok, C->stat_topology_rx_direct, C->stat_topology_rx_relayed);
    for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        if (!(report.direct_mask & bit(i + 1U))) continue;
        LOG_INF("Topology direct peer=%u rssi=%d q=%u age_ms=%u", i + 1U,
                report.rssi_dbm[i], report.quality[i], now - C->direct_seen[i]);
    }
    if (C->role == MESH_ROLE_COORDINATOR && !C->adaptive.prepared) {
        uint8_t candidate = mesh_adaptive_candidate(&C->adaptive, now);
        if (candidate) {
            mesh_handover_payload_t p;
            mesh_slot_map_payload_t slots = mesh_protocol_tx_slot_snapshot(C);
            if (mesh_adaptive_prepare(&C->adaptive, candidate, tdma_get_frame_counter(),
                                      &slots, now, &p)) {
                LOG_INF("Handover prepare candidate=%u frame=%u", candidate, p.switch_frame);
                C->last_prepare_tx = now - 120;
            }
        }
    }
}

static bool control_size(uint8_t type, uint16_t len)
{
    switch (type) {
    case MESH_PKT_SYNC_V3: return len == sizeof(mesh_sync_v3_payload_t);
    case MESH_PKT_JOIN_V3: return len == sizeof(mesh_join_v3_payload_t);
    case MESH_PKT_JOIN_ACK_V3: return len == sizeof(mesh_join_ack_v3_payload_t);
    case MESH_PKT_TOPOLOGY: return len == sizeof(mesh_topology_payload_t);
    case MESH_PKT_MEMBERSHIP_V3: return len == sizeof(mesh_membership_v3_payload_t);
    case MESH_PKT_SPEAKER_REQUEST: return len == sizeof(mesh_speaker_request_payload_t);
    case MESH_PKT_SLOT_MAP: return len == sizeof(mesh_slot_map_payload_t);
    case MESH_PKT_SPEAKER_GRANT: return len == sizeof(mesh_speaker_grant_payload_t);
    case MESH_PKT_SPEAKER_RELEASE: return len == sizeof(mesh_speaker_release_payload_t);
    case MESH_PKT_HANDOVER_PREPARE: case MESH_PKT_HANDOVER_COMMIT:
    case MESH_PKT_HANDOVER_CANCEL: return len == sizeof(mesh_handover_payload_t);
    case MESH_PKT_HANDOVER_ACK: return len == sizeof(mesh_handover_ack_payload_t);
    default: return false;
    }
}

bool mesh_protocol_adaptive_control_rx(const mesh_header_t *h, const uint8_t *payload,
                                       int8_t rssi, int64_t timestamp_us)
{
    if (!control_size(h->type, h->payload_len)) return false;
    uint32_t now = k_uptime_get_32();
    uint32_t received_ms = (uint32_t)(timestamp_us / 1000);
    mesh_adaptive_t *a = &C->adaptive;
    bool relayed = (h->flags & MESH_FLAG_RELAYED) != 0;
    if (h->flags != (relayed ? MESH_FLAG_RELAYED : 0)) return true;
    if ((relayed && (h->ttl != 1 || (h->type == MESH_PKT_SYNC_V3 &&
         ((const mesh_sync_v3_payload_t *)payload)->relay_depth != 1))) ||
        (!relayed && h->ttl != 2)) return true;
    const mesh_wire_identity_t *origin = NULL;
    uint32_t term = C->term;
    if (h->type == MESH_PKT_JOIN_V3) {
        const mesh_join_v3_payload_t *p = (const void *)payload;
        origin = &p->origin;
        if (!mesh_adaptive_identity_valid(origin) ||
            (C->state != MESH_STATE_ACTIVE && C->state != MESH_STATE_JOINING) ||
            !mesh_adaptive_identity_valid(&p->target) ||
            memcmp(&p->target, &C->leader_identity, sizeof(p->target)) != 0) return true;
    } else if (h->type == MESH_PKT_SYNC_V3) {
        const mesh_sync_v3_payload_t *p = (const void *)payload;
        origin = &p->leader;
        term = p->term;
        if (!mesh_adaptive_identity_valid(origin) || p->leader_id != h->src_id ||
            p->member_count < 1U || p->member_count > MESH_MAX_NODES ||
            p->phase_us >= 20000 || p->relay_depth != (relayed ? 1 : 0) ||
            now - received_ms > MESH_ADAPTIVE_SYNC_MAX_AGE_MS) return true;
    } else if (h->type == MESH_PKT_TOPOLOGY) {
        const mesh_topology_payload_t *p = (const void *)payload;
        origin = &p->origin;
        term = p->term;
    } else if (h->type == MESH_PKT_JOIN_ACK_V3) {
        const mesh_join_ack_v3_payload_t *p = (const void *)payload;
        term = p->term;
        if (!mesh_adaptive_identity_valid(&p->target) || p->coordinator_id != h->src_id ||
            !bit(p->assigned_id) || p->assigned_id == h->src_id ||
            p->slot_index >= MESH_MAX_NODES) return true;
    } else if (h->type == MESH_PKT_MEMBERSHIP_V3) {
        const mesh_membership_v3_payload_t *p = (const void *)payload;
        term = p->term;
        if (!mesh_adaptive_membership_valid(p, NULL)) return true;
    } else if (h->type == MESH_PKT_SPEAKER_REQUEST) {
        term = ((const mesh_speaker_request_payload_t *)payload)->term;
    } else if (h->type >= MESH_PKT_HANDOVER_PREPARE && h->type <= MESH_PKT_HANDOVER_CANCEL) {
        term = h->type == MESH_PKT_HANDOVER_ACK ?
            ((const mesh_handover_ack_payload_t *)payload)->next_term :
            ((const mesh_handover_payload_t *)payload)->next_term;
    }
    if (h->type == MESH_PKT_SYNC_V3 && C->state == MESH_STATE_SCANNING) {
        const mesh_sync_v3_payload_t *p = (const void *)payload;
        if (p->term != 0 && C->term != 0 && p->term < C->term) return true;
        if (!bit(p->leader_id) || memcmp(p->leader.address, C->local_addr, 5) == 0) return true;
        C->leader_identity = p->leader;
        C->term = p->term;
        mesh_protocol_membership_discover(p->leader_id);
        LOG_INF("SYNC discover leader=%u relay=%u term=%u", p->leader_id, relayed, p->term);
        return true;
    }
    if (C->state == MESH_STATE_JOINING && h->type == MESH_PKT_SYNC_V3) {
        const mesh_sync_v3_payload_t *p = (const void *)payload;
        if (p->leader_id != C->coordinator_id || p->term != C->term ||
            memcmp(&p->leader, &C->leader_identity, sizeof(p->leader))) return true;
    }
    if (C->state == MESH_STATE_ACTIVE && h->type == MESH_PKT_SYNC_V3 && a->prepared &&
        mesh_adaptive_reconcile_sync(a, h->src_id, (const void *)payload, now)) {
        apply_transition();
        LOG_INF("SYNC reconciled source=%u term=%u", h->src_id, C->term);
    }
    if (C->state == MESH_STATE_ACTIVE && C->role == MESH_ROLE_COORDINATOR &&
        h->type == MESH_PKT_SYNC_V3 &&
        !identity_for(C->node_id, origin)) {
        const mesh_sync_v3_payload_t *p = (const void *)payload;
        bool cold_start_win = C->term == 1 && p->term == 1 && !a->prepared &&
            !a->transition_done;
        mesh_wire_identity_t local = mesh_protocol_local_identity();
        uint8_t local_count = (uint8_t)__builtin_popcount((unsigned)a->members);
        cold_start_win = cold_start_win &&
            mesh_adaptive_cold_join(&local, local_count, &p->leader, p->member_count);
        if (cold_start_win) {
            mesh_protocol_membership_demote(p->leader_id, p->term, p->leader);
            LOG_INF("SYNC elected source=%u term=%u", p->leader_id, p->term);
        }
        return true;
    }
    if (C->state == MESH_STATE_ACTIVE) {
        mesh_protocol_adaptive_members();
        bool auth = mesh_adaptive_control_authorized(a, h->type, h->src_id);
        if (!auth || (h->type != MESH_PKT_JOIN_V3 && h->type != MESH_PKT_TOPOLOGY &&
                      h->type != MESH_PKT_SYNC_V3 && !identity_for(h->src_id,
                          &a->nodes[h->src_id - 1].identity))) return true;
        if (h->type == MESH_PKT_TOPOLOGY &&
            !mesh_adaptive_identity_valid(origin)) return true;
        if (h->type == MESH_PKT_TOPOLOGY && !identity_for(h->src_id, origin)) return true;
        if (h->type == MESH_PKT_SYNC_V3 &&
            (!identity_for(h->src_id, origin) || term != C->term)) return true;
        if (h->type == MESH_PKT_JOIN_ACK_V3 && term != C->term) return true;
        if (h->type == MESH_PKT_MEMBERSHIP_V3 && term != C->term) return true;
        if (h->type != MESH_PKT_JOIN_V3 && h->type != MESH_PKT_JOIN_ACK_V3 &&
            h->type != MESH_PKT_MEMBERSHIP_V3 &&
            h->type != MESH_PKT_SYNC_V3 && h->type != MESH_PKT_TOPOLOGY &&
            term != C->term && !(term == C->term + 1 &&
            h->type >= MESH_PKT_HANDOVER_PREPARE &&
            h->type <= MESH_PKT_HANDOVER_CANCEL)) return true;
    } else if (h->type != MESH_PKT_SYNC_V3 && h->type != MESH_PKT_JOIN_ACK_V3 &&
               h->type != MESH_PKT_JOIN_V3) return true;
    /* Membership handlers own JOIN/ACK and maps. Forward only accepted control. */
    if (h->type == MESH_PKT_TOPOLOGY) {
        const mesh_topology_payload_t *p = (const void *)payload;
        if (!mesh_adaptive_report_valid(a, h->src_id, p)) return true;
        if (!relayed) {
            C->stat_topology_rx_direct++;
            mesh_protocol_adaptive_observe(h, p, rssi);
            for (int i = 0; i < 8; i++) if (C->peers[i].active &&
                C->peers[i].node_id == h->src_id) {
                C->peers[i].rssi_dbm = rssi;
                break;
            }
        }
        if (!mesh_adaptive_report(a, h->src_id, p, relayed, now)) return true;
        if (relayed) C->stat_topology_rx_relayed++;
    } else if (h->type == MESH_PKT_SPEAKER_REQUEST) {
        const mesh_speaker_request_payload_t *request = (const void *)payload;
        if (!mesh_adaptive_speaker_request(a, h->src_id, request,
                                           tdma_get_frame_counter(), now)) return true;
        if (C->role == MESH_ROLE_COORDINATOR) {
            uint8_t id = h->src_id;
            if (request->active) {
                if (C->active_speaker_deadline_ms[id] <= now)
                    C->speaker_active_since_ms[id] = now;
                C->active_speaker_deadline_ms[id] = now + ACTIVE_SPEAKER_TIMEOUT_MS;
            } else {
                C->active_speaker_deadline_ms[id] = 0;
            }
            mesh_protocol_audio_update_speaker_grants();
        }
    } else if (h->type == MESH_PKT_MEMBERSHIP_V3) {
        const mesh_membership_v3_payload_t *p = (const void *)payload;
        uint8_t old_members = a->members;
        uint8_t old_authoritative = C->authoritative_members;
        mesh_wire_identity_t old_ids[MESH_MAX_NODES];
        for (uint8_t i = 0; i < 8; i++) old_ids[i] = a->nodes[i].identity;
        if (!mesh_adaptive_apply_membership(a, h->src_id, p)) {
            if (mesh_adaptive_membership_removes_local(a, h->src_id, p)) {
                mesh_wire_identity_t leader = a->nodes[a->leader_id - 1U].identity;
                LOG_WRN("Membership removed local id=%u leader=%u term=%u; rejoining",
                        C->node_id, a->leader_id, a->term);
                mesh_protocol_membership_rejoin(a->leader_id, a->term, leader);
            }
            return true;
        }
        for (uint8_t i = 0; i < 8; i++) if ((old_members & bit(i + 1)) !=
            (a->members & bit(i + 1)) ||
            memcmp(&old_ids[i], &a->nodes[i].identity, sizeof(old_ids[i])) != 0) {
            clear_direct(i + 1);
            mesh_core_dedupe_purge_node(&C->dedupe, i + 1);
            mesh_core_dedupe_purge_node(&C->control_presence_dedupe, i + 1);
            C->join_seq_seen &= (uint8_t)~bit(i + 1);
            C->ack_seq_seen &= (uint8_t)~bit(i + 1);
            mesh_protocol_audio_reset_rf_e2e_tracker(i + 1);
            C->active_speaker_deadline_ms[i + 1] = 0;
            C->speaker_active_since_ms[i + 1] = 0;
        }
        copy_slot_members(&p->slot_map);
        C->participant_membership_known = true;
        for (uint8_t slot = 0; slot < p->slot_map.slot_count; slot++) {
            if (p->slot_map.slot_ids[slot] == C->node_id && C->slot_index != (int8_t)slot) {
                C->slot_index = (int8_t)slot;
                tdma_set_slot_index(C->slot_index);
                break;
            }
        }
        mesh_protocol_audio_apply_slot_map_speakers(&p->slot_map);
        if (old_authoritative != a->members) {
            LOG_INF("Membership leader=%u mask=0x%02x term=%u", h->src_id, a->members, a->term);
            uart_bridge_send_status(C->state, C->role, C->peer_count, C->node_id,
                                    C->slot_index, C->coordinator_id);
        }
    } else if (h->type == MESH_PKT_JOIN_ACK_V3 && C->state == MESH_STATE_ACTIVE) {
        const mesh_join_ack_v3_payload_t *ack = (const void *)payload;
        if (!identity_for(h->src_id, &C->leader_identity) || ack->term != a->term)
            return true;
        /* ACK may precede or lag a map; only the authoritative map binds peers. */
    } else if (h->type == MESH_PKT_HANDOVER_PREPARE) {
        const mesh_handover_payload_t *p = (const void *)payload;
        if (!mesh_adaptive_receive_prepare(a, h->src_id, p, tdma_get_frame_counter(), now)) return true;
        mesh_handover_ack_payload_t ack = {.old_leader_id = p->old_leader_id,
            .candidate_id = p->candidate_id, .next_term = p->next_term,
            .switch_frame = p->switch_frame, .acknowledger_id = C->node_id};
        (void)mesh_protocol_tx_queue_priority(MESH_PKT_HANDOVER_ACK, &ack, sizeof(ack));
    } else if (h->type == MESH_PKT_HANDOVER_ACK) {
        if (!mesh_adaptive_ack(a, h->src_id, (const void *)payload, now)) return true;
        if (!mesh_protocol_adaptive_control_sequence_accept(h->src_id, MESH_PKT_HANDOVER_ACK,
                                                             h->seq)) return true;
    } else if (h->type == MESH_PKT_HANDOVER_COMMIT) {
        if (!mesh_adaptive_receive_commit(a, h->src_id, (const void *)payload, now)) return true;
    } else if (h->type == MESH_PKT_HANDOVER_CANCEL) {
        if (!mesh_adaptive_cancel(a, h->src_id, (const void *)payload)) return true;
    } else if (h->type == MESH_PKT_SYNC_V3) {
        const mesh_sync_v3_payload_t *p = (const void *)payload;
        if (C->state == MESH_STATE_ACTIVE && C->role == MESH_ROLE_PARTICIPANT) {
            int64_t base = timestamp_us - p->phase_us - NRF_SYNC_RX_LATENCY_US;
            tdma_sync(p->frame_counter, 0, base);
            C->last_sync_time = received_ms;
        }
    } else if (h->type == MESH_PKT_JOIN_V3 || h->type == MESH_PKT_JOIN_ACK_V3) {
        mesh_protocol_membership_process_rx_packet(h, payload, rssi, timestamp_us);
    } else if (h->type == MESH_PKT_SLOT_MAP) {
        mesh_membership_v3_payload_t check = {.term = a->term, .leader_id = a->leader_id,
            .slot_map = *(const mesh_slot_map_payload_t *)payload};
        for (uint8_t i = 0; i < 8; i++) if (a->members & bit(i + 1))
            check.identities[i] = a->nodes[i].identity;
        uint8_t members;
        if (!mesh_adaptive_membership_valid(&check, &members) || members != a->members) return true;
        mesh_protocol_audio_apply_slot_map_speakers(&check.slot_map);
    } else if (h->type == MESH_PKT_SPEAKER_GRANT)
        mesh_protocol_audio_apply_speaker_grant((const void *)payload);
    else if (h->type == MESH_PKT_SPEAKER_RELEASE)
        mesh_protocol_audio_apply_speaker_release((const void *)payload);

    bool pending_sync = false;
    if (h->type == MESH_PKT_SYNC_V3) {
        for (uint8_t i = 0; i < a->queue_count; i++)
            if (a->queue[(a->queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE].header.type ==
                MESH_PKT_SYNC_V3) { pending_sync = true; break; }
    }
    bool forward_sync = h->type != MESH_PKT_SYNC_V3 ||
        (!pending_sync && now - C->last_forward_sync >= 550 && a->queue_count < 2);
    if (C->state == MESH_STATE_ACTIVE && bit(h->src_id) &&
        h->src_id != C->node_id && (a->members & bit(h->src_id)) &&
        mesh_core_dedupe_accept(&C->control_presence_dedupe, h->type, h->src_id, h->seq))
        mesh_protocol_note_peer_presence(h->src_id);

    if (C->state == MESH_STATE_ACTIVE && h->src_id != C->node_id && forward_sync &&
        mesh_adaptive_forward_enqueue(a, h, payload, h->payload_len, origin, term,
                                       C->last_sync_time && now - C->last_sync_time <= 600, now)) {
        if (h->type == MESH_PKT_SYNC_V3) {
            C->last_forward_sync = now;
            if (now - C->last_sync_log >= 2000) {
                C->last_sync_log = now;
                LOG_INF("SYNC forward source=%u", h->src_id);
            }
        }
    }
    return true;
}
