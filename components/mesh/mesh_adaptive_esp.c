#include <string.h>

#include "esp_log.h"
#include "mesh_internal.h"

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static uint8_t bit(uint8_t id) { return mesh_core_node_bit(id); }

mesh_wire_identity_t mesh_local_identity(void)
{
    mesh_wire_identity_t id = {.address_len = 6};
    memcpy(id.address, s_local_mac, 6);
    return id;
}

void mesh_adaptive_local_init(void)
{
    memset(s_mesh.neighbors, 0, sizeof(s_mesh.neighbors));
    s_mesh.report_seq = 0;
    s_mesh.report_wire_seq = 0;
    mesh_adaptive_init(&s_mesh.adaptive, s_node_id, s_coordinator_id, 1,
                       bit(s_node_id) | bit(s_coordinator_id), now_ms());
    mesh_wire_identity_t self = mesh_local_identity();
    (void)mesh_adaptive_set_member(&s_mesh.adaptive, s_node_id, &self);
    s_mesh.upstream_sync_us = s_role == MESH_ROLE_COORDINATOR ? esp_timer_get_time() : 0;
}

void mesh_adaptive_members_update(void)
{
    if (s_role != MESH_ROLE_COORDINATOR || !s_node_id || !s_mesh.adaptive.local_id) return;
    mesh_wire_identity_t identities[MESH_MAX_NODES] = {0};
    identities[s_node_id - 1] = mesh_local_identity();
    uint8_t members = bit(s_node_id);
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        const mesh_peer_info_t *p = &s_peers[i].info;
        if (p->active && bit(p->node_id)) {
            members |= bit(p->node_id);
            identities[p->node_id - 1].address_len = 6;
            memcpy(identities[p->node_id - 1].address, p->mac_addr, 6);
        }
    }
    xSemaphoreGive(s_peer_mutex);
    if (!mesh_adaptive_update_members(&s_mesh.adaptive, members)) return;
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        if (!(members & bit(id))) {
            memset(&s_mesh.neighbors[id - 1], 0, sizeof(s_mesh.neighbors[0]));
            continue;
        }
        if (memcmp(&s_mesh.adaptive.nodes[id - 1].identity,
                   &identities[id - 1], sizeof(identities[0])))
            memset(&s_mesh.neighbors[id - 1], 0, sizeof(s_mesh.neighbors[0]));
        (void)mesh_adaptive_set_member(&s_mesh.adaptive, id, &identities[id - 1]);
    }
}

void mesh_observe_direct(const mesh_rx_item_t *rx)
{
    uint8_t id = rx->header.src_id;
    if (!bit(id) || id == s_node_id || (rx->header.flags & MESH_FLAG_RELAYED)) return;
    mesh_adaptive_member_t *n = &s_mesh.adaptive.nodes[id - 1];
    if (!mesh_adaptive_identity_valid(&n->identity) || n->identity.address_len != 6 ||
        memcmp(n->identity.address, rx->src_mac, 6)) return;
    typeof(s_mesh.neighbors[0]) *obs = &s_mesh.neighbors[id - 1];
    bool first = !obs->direct_seen_ms;
    obs->seen_ms = (uint32_t)(rx->timestamp_us / 1000);
    obs->direct_seen_ms = obs->seen_ms;
    int8_t rssi = rx->rssi < -127 ? -127 : rx->rssi > 0 ? 0 : rx->rssi;
    obs->rssi = first ? rssi : (int8_t)((3 * (int)obs->rssi + rssi) / 4);
    if (rx->header.type == MESH_PKT_AUDIO) {
        mesh_core_seq_result_t seq = mesh_core_seq8_accept(&obs->seq, rx->header.seq);
        if (seq.classification == MESH_CORE_SEQ_GAP) obs->lost += seq.gap;
        if (seq.classification != MESH_CORE_SEQ_DUPLICATE) obs->received++;
    }
}

void mesh_observe_direct_report(const mesh_rx_item_t *rx, const mesh_topology_payload_t *report)
{
    if (rx->header.flags & MESH_FLAG_RELAYED) return;
    mesh_observe_direct(rx);
    uint8_t id = rx->header.src_id;
    typeof(s_mesh.neighbors[0]) *obs = &s_mesh.neighbors[id - 1];
    uint32_t delta = report->report_seq - obs->report_seq;
    if (obs->report_seen && (!delta || delta >= UINT32_C(0x80000000))) return;
    uint8_t sample = obs->report_seen ? (uint8_t)(100U / delta) : 100U;
    obs->pdr_quality = obs->pdr_seen ?
        (uint8_t)((3U * obs->pdr_quality + sample + 2U) / 4U) : sample;
    obs->pdr_seen = true;
    obs->report_seen = true;
    obs->report_seq = report->report_seq;
    obs->report_ms = (uint32_t)(rx->timestamp_us / 1000);
}

void mesh_send_membership(void)
{
    if (s_role != MESH_ROLE_COORDINATOR || s_state != MESH_STATE_ACTIVE) return;
    mesh_membership_v3_payload_t snapshot = {.term = s_mesh.adaptive.term,
        .leader_id = s_node_id, .slot_map = s_mesh.slot_map};
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++)
        if (s_mesh.adaptive.members & bit(id))
            snapshot.identities[id - 1] = s_mesh.adaptive.nodes[id - 1].identity;
    uint8_t members;
    if (mesh_adaptive_membership_valid(&snapshot, &members) &&
        members == s_mesh.adaptive.members) {
        snapshot.revision = mesh_adaptive_next_membership_revision(&s_mesh.adaptive);
        if (mesh_send_control_packet(MESH_PKT_MEMBERSHIP_V3, &snapshot, sizeof(snapshot)) == ESP_OK) {
            s_mesh.membership_publish_ms = now_ms();
            ESP_LOGD(TAG, "MEMBERSHIP term=%lu members=0x%02x", (unsigned long)snapshot.term, members);
        }
    }
}

void mesh_send_topology(void)
{
    if (s_state != MESH_STATE_ACTIVE || !s_mesh.adaptive.local_id) return;
    uint32_t now = now_ms();
    mesh_topology_payload_t r = {.origin = mesh_local_identity(),
        .report_seq = s_mesh.report_seq++, .leader_id = s_mesh.adaptive.leader_id,
        .term = s_mesh.adaptive.term, .slot_map = s_mesh.slot_map};
    for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        if (!(s_mesh.adaptive.members & bit(i + 1)) ||
            (uint32_t)(now - s_mesh.neighbors[i].direct_seen_ms) > MESH_ADAPTIVE_EXPIRE_MS ||
            !s_mesh.neighbors[i].direct_seen_ms || !s_mesh.neighbors[i].report_seen ||
            (uint32_t)(now - s_mesh.neighbors[i].report_ms) > MESH_ADAPTIVE_EXPIRE_MS) continue;
        r.direct_mask |= bit(i + 1);
        r.rssi_dbm[i] = s_mesh.neighbors[i].rssi;
        r.quality[i] = s_mesh.neighbors[i].pdr_quality;
    }
    if (!mesh_adaptive_report(&s_mesh.adaptive, s_node_id, &r, false, now)) return;
    ESP_LOGD(TAG, "TOPOLOGY node=%u direct=0x%02x term=%lu", s_node_id, r.direct_mask,
             (unsigned long)r.term);
    (void)mesh_send_control_packet(MESH_PKT_TOPOLOGY, &r, sizeof(r));
}

void mesh_request_local_speaker(uint32_t now)
{
    if (s_state != MESH_STATE_ACTIVE || !s_mesh.adaptive.local_id) return;
    bool active;
    taskENTER_CRITICAL(&s_speaker_mux);
    if (s_mesh.local_voice_active && (int64_t)now >= s_mesh.local_voice_deadline_ms)
        s_mesh.local_voice_active = false;
    active = s_mesh.local_voice_active;
    taskEXIT_CRITICAL(&s_speaker_mux);

    bool new_term = s_mesh.local_request_term != s_mesh.adaptive.term;
    if (!active && !s_mesh.local_request_announced) {
        s_mesh.local_request_term = s_mesh.adaptive.term;
        return;
    }
    uint8_t granted[MESH_MAX_ACTIVE_SPEAKERS];
    speaker_state_get(granted, NULL);
    bool has_grant = false;
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++)
        if (granted[i] == s_node_id) has_grant = true;
    uint32_t interval = has_grant ? MESH_ADAPTIVE_REPORT_MS : MESH_SPEAKER_REQUEST_INTERVAL_MS;
    if ((!active && !s_mesh.local_request_announced && !new_term) ||
        (active == s_mesh.local_request_announced && !new_term &&
         (!active || (uint32_t)(now - s_mesh.local_request_last_ms) <
             interval))) return;

    mesh_speaker_request_payload_t request = {.term = s_mesh.adaptive.term,
        .request_seq = ++s_mesh.local_request_seq, .frame_counter = mesh_get_frame_counter(),
        .active = active ? 1 : 0};
    if (mesh_send_control_packet(MESH_PKT_SPEAKER_REQUEST, &request, sizeof(request)) != ESP_OK)
        return;
    s_mesh.local_request_announced = active;
    s_mesh.local_request_last_ms = now;
    s_mesh.local_request_term = request.term;
    if (!active) s_mesh.speaker_transition_grants &= (uint8_t)~bit(s_node_id);
    if (mesh_adaptive_speaker_request(&s_mesh.adaptive, s_node_id, &request,
                                      request.frame_counter, now) &&
        s_role == MESH_ROLE_COORDINATOR) update_speaker_grants();
}

static bool known_sender(const mesh_rx_item_t *rx)
{
    uint8_t id = rx->header.src_id;
    if (!(s_mesh.adaptive.members & bit(id)) || id == s_node_id) return false;
    const mesh_wire_identity_t *origin = &s_mesh.adaptive.nodes[id - 1].identity;
    if (!mesh_adaptive_identity_valid(origin) || origin->address_len != 6) return false;
    if (!(rx->header.flags & MESH_FLAG_RELAYED))
        return !memcmp(origin->address, rx->src_mac, 6) && rx->header.ttl == 2;
    if (rx->header.ttl != 1) return false;
    for (uint8_t i = 1; i <= MESH_MAX_NODES; i++) {
        if (i == id || i == s_node_id || !(s_mesh.adaptive.members & bit(i))) continue;
        const mesh_wire_identity_t *via = &s_mesh.adaptive.nodes[i - 1].identity;
        if (via->address_len == 6 && !memcmp(via->address, rx->src_mac, 6) &&
            (uint32_t)(now_ms() - s_mesh.neighbors[i - 1].seen_ms) <= MESH_ADAPTIVE_EXPIRE_MS)
            return true;
    }
    return false;
}

bool mesh_adaptive_bound_sender(const mesh_rx_item_t *rx)
{
    return known_sender(rx);
}

void mesh_refresh_peer_liveness(uint8_t src_id, int64_t received_us)
{
    if (!(s_mesh.adaptive.members & bit(src_id)) || src_id == s_node_id) return;
    const mesh_wire_identity_t *identity = &s_mesh.adaptive.nodes[src_id - 1].identity;
    if (identity->address_len != 6) return;
    mesh_peer_info_t joined = {0};
    bool added = false, found = false;
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        mesh_peer_info_t *peer = &s_peers[i].info;
        if (peer->active && peer->node_id == src_id) {
            if (memcmp(peer->mac_addr, identity->address, 6)) {
                memcpy(peer->mac_addr, identity->address, 6);
                mesh_core_seq8_reset(&s_peers[i].rx_seq);
            }
            int64_t received_ms = received_us / 1000;
            if (peer->last_seen_ms < received_ms) peer->last_seen_ms = received_ms;
            found = true;
            break;
        }
    }
    if (!found) for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        mesh_peer_info_t *peer = &s_peers[i].info;
        if (peer->active) continue;
        s_peers[i] = (peer_tracking_t){0};
        peer->active = true;
        peer->node_id = src_id;
        memcpy(peer->mac_addr, identity->address, 6);
        peer->slot_index = -1;
        for (uint8_t slot = 0; slot < s_mesh.slot_map.slot_count; slot++)
            if (s_mesh.slot_map.slot_ids[slot] == src_id) peer->slot_index = slot;
        peer->last_seen_ms = received_us / 1000;
        s_peer_count++;
        joined = *peer;
        added = true;
        break;
    }
    xSemaphoreGive(s_peer_mutex);
    if (added && s_peer_cb) s_peer_cb(&joined, true);
}

static bool known_relay_mac(const uint8_t mac[6], const uint8_t origin[6])
{
    uint32_t now = now_ms();
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        if (id == s_node_id || !(s_mesh.adaptive.members & bit(id))) continue;
        const mesh_wire_identity_t *neighbor = &s_mesh.adaptive.nodes[id - 1].identity;
        if (neighbor->address_len == 6 && !memcmp(neighbor->address, mac, 6) &&
            memcmp(mac, origin, 6) && s_mesh.neighbors[id - 1].seen_ms &&
            (uint32_t)(now - s_mesh.neighbors[id - 1].seen_ms) <= MESH_ADAPTIVE_EXPIRE_MS)
            return true;
    }
    return false;
}

static uint16_t expected_size(uint8_t type)
{
    switch (type) {
    case MESH_PKT_JOIN_V3: return sizeof(mesh_join_v3_payload_t);
    case MESH_PKT_JOIN_ACK_V3: return sizeof(mesh_join_ack_v3_payload_t);
    case MESH_PKT_SYNC_V3: return sizeof(mesh_sync_v3_payload_t);
    case MESH_PKT_TOPOLOGY: return sizeof(mesh_topology_payload_t);
    case MESH_PKT_MEMBERSHIP_V3: return sizeof(mesh_membership_v3_payload_t);
    case MESH_PKT_SPEAKER_REQUEST: return sizeof(mesh_speaker_request_payload_t);
    case MESH_PKT_SLOT_MAP: return sizeof(mesh_slot_map_payload_t);
    case MESH_PKT_SPEAKER_GRANT: return sizeof(mesh_speaker_grant_payload_t);
    case MESH_PKT_SPEAKER_RELEASE: return sizeof(mesh_speaker_release_payload_t);
    case MESH_PKT_HANDOVER_ACK: return sizeof(mesh_handover_ack_payload_t);
    case MESH_PKT_HANDOVER_PREPARE: case MESH_PKT_HANDOVER_COMMIT:
    case MESH_PKT_HANDOVER_CANCEL: return sizeof(mesh_handover_payload_t);
    default: return 0;
    }
}

static void forward_control(const mesh_rx_item_t *rx, const mesh_wire_identity_t *origin)
{
    if (s_state != MESH_STATE_ACTIVE || !s_mesh.adaptive.local_id ||
        (rx->header.flags & MESH_FLAG_RELAYED)) return;
    uint32_t now = now_ms();
    if (rx->header.type == MESH_PKT_SYNC_V3) {
        for (uint8_t i = 0; i < s_mesh.adaptive.queue_count; i++) {
            mesh_adaptive_forward_packet_t *pending = &s_mesh.adaptive.queue[
                (s_mesh.adaptive.queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE];
            if (pending->header.type != MESH_PKT_SYNC_V3 ||
                pending->header.src_id != rx->header.src_id) continue;
            mesh_sync_v3_payload_t older, newer;
            memcpy(&older, pending->payload, sizeof(older));
            memcpy(&newer, rx->payload, sizeof(newer));
            if (older.term != newer.term || older.leader_id != newer.leader_id) continue;
            pending->header = rx->header;
            pending->header.flags |= MESH_FLAG_RELAYED;
            pending->header.ttl = 1;
            memcpy(pending->payload, &newer, sizeof(newer));
            pending->enqueued_ms = now;
            return;
        }
        if (mesh_coalesce_forward_sync(rx)) return;
        if (s_mesh.forward_sync_last_tx_ms &&
            (uint32_t)(now - s_mesh.forward_sync_last_tx_ms) < MESH_FORWARD_SYNC_INTERVAL_MS)
            return;
    }
    uint32_t age = s_mesh.upstream_sync_us > 0 ?
        (uint32_t)((esp_timer_get_time() - s_mesh.upstream_sync_us) / 1000) : UINT32_MAX;
    if (mesh_adaptive_forward_enqueue(&s_mesh.adaptive, &rx->header, rx->payload,
           rx->header.payload_len, origin, s_mesh.adaptive.term,
           s_role == MESH_ROLE_COORDINATOR || age <= MESH_ADAPTIVE_SYNC_MAX_AGE_MS, now))
        ESP_LOGD(TAG, "forward queued type=0x%02x src=%u", rx->header.type, rx->header.src_id);
}

static void apply_membership_peers(const mesh_membership_v3_payload_t *snapshot, uint8_t members)
{
    mesh_peer_info_t added[MESH_MAX_NODES], removed[MESH_MAX_NODES];
    uint8_t added_count = 0, removed_count = 0;
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        uint8_t id = s_peers[i].info.node_id;
        if (s_peers[i].info.active && id != s_node_id && !(members & bit(id))) {
            removed[removed_count++] = s_peers[i].info;
            memset(&s_mesh.neighbors[id - 1], 0, sizeof(s_mesh.neighbors[0]));
            mesh_core_dedupe_purge_node(&s_dedupe, id);
            s_peers[i].info.active = false;
            if (s_peer_count) s_peer_count--;
        }
    }
    for (uint8_t slot = 0; slot < snapshot->slot_map.slot_count; slot++) {
        uint8_t id = snapshot->slot_map.slot_ids[slot];
        if (!id) continue;
        if (id == s_node_id) {
            s_slot_index = slot;
            for (uint8_t i = 0; i < MESH_MAX_NODES; i++)
                if (s_peers[i].info.active && s_peers[i].info.node_id == id)
                    s_peers[i].info.slot_index = slot;
            continue;
        }
        peer_tracking_t *peer = NULL;
        bool existing = false;
        for (uint8_t i = 0; i < MESH_MAX_NODES; i++)
            if (s_peers[i].info.active && s_peers[i].info.node_id == id) {
                peer = &s_peers[i]; existing = true; break;
            }
        if (!peer) for (uint8_t i = 0; i < MESH_MAX_NODES; i++)
            if (!s_peers[i].info.active) {
                peer = &s_peers[i];
                *peer = (peer_tracking_t){0};
                peer->info.active = true;
                peer->info.node_id = id;
                s_peer_count++;
                added[added_count++] = peer->info;
                break;
            }
        if (!peer) continue;
        if (existing &&
            memcmp(peer->info.mac_addr, snapshot->identities[id - 1].address, 6)) {
            memset(&s_mesh.neighbors[id - 1], 0, sizeof(s_mesh.neighbors[0]));
            mesh_core_dedupe_purge_node(&s_dedupe, id);
            mesh_core_seq8_reset(&peer->rx_seq);
        }
        peer->info.slot_index = slot;
        peer->info.last_seen_ms = now_ms();
        memcpy(peer->info.mac_addr, snapshot->identities[id - 1].address, 6);
    }
    xSemaphoreGive(s_peer_mutex);
    if (s_peer_cb) {
        for (uint8_t i = 0; i < removed_count; i++) s_peer_cb(&removed[i], false);
        for (uint8_t i = 0; i < added_count; i++) {
            mesh_peer_info_t peer;
            if (mesh_get_peer_info(added[i].node_id, &peer) == ESP_OK)
                s_peer_cb(&peer, true);
        }
    }
    s_mesh.slot_map = snapshot->slot_map;
    uint8_t ids[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    uint8_t masks[MESH_MAX_ACTIVE_SPEAKERS] = {0};
    memcpy(ids, snapshot->slot_map.active_speaker_ids, sizeof(ids));
    memcpy(masks, snapshot->slot_map.relay_masks, sizeof(masks));
    speaker_state_set(ids, masks);
}

bool mesh_handle_adaptive_control(const mesh_rx_item_t *rx)
{
    uint8_t type = rx->header.type, src = rx->header.src_id;
    uint16_t size = expected_size(type);
    if (!size) return false;
    if (rx->header.payload_len != size || (rx->header.flags & ~MESH_FLAG_RELAYED) ||
        (rx->header.flags & MESH_FLAG_RELAYED ? rx->header.ttl != 1 : rx->header.ttl != 2))
        return true;
    if (type == MESH_PKT_JOIN_V3) {
        mesh_join_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (src != 0 || !mesh_adaptive_identity_valid(&p.origin) || p.origin.address_len != 6 ||
            (rx->header.flags & MESH_FLAG_RELAYED ?
             !known_relay_mac(rx->src_mac, p.origin.address) :
             memcmp(rx->src_mac, p.origin.address, 6))) return true;
        if (s_state == MESH_STATE_ACTIVE &&
            (p.target.address_len != 6 ||
             memcmp(&p.target, &s_mesh.adaptive.nodes[s_coordinator_id - 1].identity,
                    sizeof(p.target)))) return true;
        if (p.target.address_len && (!mesh_adaptive_identity_valid(&p.target) ||
            (s_role == MESH_ROLE_COORDINATOR && memcmp(p.target.address, s_local_mac, 6)))) return true;
        forward_control(rx, &p.origin);
        if (s_role == MESH_ROLE_COORDINATOR) handle_join_packet(rx);
        if (rx->header.flags & MESH_FLAG_RELAYED)
            for (uint8_t id = 1; id <= MESH_MAX_NODES; id++)
                if ((s_mesh.adaptive.members & bit(id)) && id != s_node_id &&
                    s_mesh.adaptive.nodes[id - 1].identity.address_len == 6 &&
                    !memcmp(s_mesh.adaptive.nodes[id - 1].identity.address, rx->src_mac, 6)) {
                    mesh_refresh_peer_liveness(id, rx->timestamp_us);
                    break;
                }
        return true;
    }
    if (type == MESH_PKT_SYNC_V3 && (s_state == MESH_STATE_SCANNING ||
        s_state == MESH_STATE_JOINING)) {
        mesh_sync_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.leader_id == src && p.term && p.phase_us < MESH_FRAME_US &&
            p.member_count >= 1 && p.member_count <= MESH_MAX_NODES &&
            mesh_adaptive_identity_valid(&p.leader) && p.leader.address_len == 6 &&
            (!(rx->header.flags & MESH_FLAG_RELAYED) ||
             p.relay_depth == 1) &&
             ((rx->header.flags & MESH_FLAG_RELAYED) || !memcmp(p.leader.address, rx->src_mac, 6))) {
            if (s_state == MESH_STATE_JOINING && (rx->header.flags & MESH_FLAG_RELAYED) &&
                src == s_coordinator_id && !memcmp(p.leader.address, s_coordinator_mac, 6) &&
                s_mesh.discovery_relay_ms && !memcmp(rx->src_mac, s_mesh.discovery_relay_mac, 6))
                s_mesh.discovery_relay_ms = now_ms();
            handle_sync_packet(rx);
        }
        return true;
    }
    if (type == MESH_PKT_JOIN_ACK_V3 && s_state == MESH_STATE_JOINING) {
        mesh_join_ack_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (src == s_coordinator_id && p.coordinator_id == src && p.term &&
            p.term == s_mesh.discovered_term &&
            p.target.address_len == 6 && !memcmp(p.target.address, s_local_mac, 6) &&
            (rx->header.flags & MESH_FLAG_RELAYED ?
             s_mesh.discovery_relay_ms &&
             (uint32_t)(now_ms() - s_mesh.discovery_relay_ms) <= MESH_ADAPTIVE_EXPIRE_MS &&
             !memcmp(rx->src_mac, s_mesh.discovery_relay_mac, 6) :
             !memcmp(rx->src_mac, s_coordinator_mac, 6))) handle_join_ack_packet(rx);
        return true;
    }
    if (type == MESH_PKT_SYNC_V3 && s_role == MESH_ROLE_COORDINATOR &&
        s_mesh.adaptive.term == 1 &&
        !s_mesh.adaptive.transition_done && !s_mesh.adaptive.prepared &&
        !(rx->header.flags & MESH_FLAG_RELAYED)) {
        mesh_sync_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.leader_id == src && p.term == 1 && p.leader.address_len == 6 &&
            p.member_count >= 1 && p.member_count <= MESH_MAX_NODES &&
            memcmp(p.leader.address, s_local_mac, 6) &&
            p.phase_us < MESH_FRAME_US && !memcmp(p.leader.address, rx->src_mac, 6))
            handle_sync_packet(rx);
        return true;
    }
    if (s_state != MESH_STATE_ACTIVE) return true;
    if (type == MESH_PKT_MEMBERSHIP_V3) {
        mesh_membership_v3_payload_t p;
        uint8_t members;
        memcpy(&p, rx->payload, sizeof(p));
        if (!mesh_adaptive_membership_valid(&p, &members)) return true;
        int relay_id = 0;
        bool via_known = known_sender(rx);
        if (via_known && (rx->header.flags & MESH_FLAG_RELAYED)) {
            bool mapped_relay = false;
            for (uint8_t id = 1; id <= MESH_MAX_NODES; id++)
                if (id != src && id != s_node_id && (members & bit(id)) &&
                    p.identities[id - 1].address_len == 6 &&
                    !memcmp(p.identities[id - 1].address, rx->src_mac, 6))
                    mapped_relay = true;
            if (!mapped_relay) return true;
        }
        if (!via_known && (rx->header.flags & MESH_FLAG_RELAYED) &&
            src == s_mesh.adaptive.leader_id && s_mesh.discovery_relay_ms &&
            (uint32_t)(now_ms() - s_mesh.discovery_relay_ms) <= MESH_ADAPTIVE_EXPIRE_MS &&
            !memcmp(rx->src_mac, s_mesh.discovery_relay_mac, 6)) {
            for (uint8_t id = 1; id <= MESH_MAX_NODES; id++)
                if (id != src && id != s_node_id && (members & bit(id)) &&
                    p.identities[id - 1].address_len == 6 &&
                    !memcmp(p.identities[id - 1].address, rx->src_mac, 6))
                    relay_id = id;
        }
        if (!via_known && !relay_id) return true;
        if (mesh_adaptive_membership_removes_local(&s_mesh.adaptive, src, &p)) {
            mesh_rejoin_after_eviction(rx, &p);
            return true;
        }
        if (!mesh_adaptive_apply_membership(&s_mesh.adaptive, src, &p)) return true;
        if (relay_id) s_mesh.neighbors[relay_id - 1].seen_ms = now_ms();
        apply_membership_peers(&p, members);
        forward_control(rx, &s_mesh.adaptive.nodes[src - 1].identity);
        return true;
    }
    if (!known_sender(rx)) return true;
    if (type == MESH_PKT_SYNC_V3 && s_mesh.adaptive.prepared &&
        src == s_mesh.adaptive.pending.candidate_id) {
        mesh_sync_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.leader_id == src && p.leader.address_len == 6 &&
            !memcmp(&p.leader, &s_mesh.adaptive.nodes[src - 1].identity, sizeof(p.leader)) &&
            p.relay_depth == !!(rx->header.flags & MESH_FLAG_RELAYED) &&
            mesh_adaptive_reconcile_sync(&s_mesh.adaptive, src, &p, now_ms())) {
            mesh_adaptive_apply_transition();
            handle_sync_packet(rx);
            mesh_refresh_peer_liveness(src, rx->timestamp_us);
        }
        return true;
    }
    if (!mesh_adaptive_control_authorized(&s_mesh.adaptive, type, src))
        return true;
    if (type == MESH_PKT_TOPOLOGY) {
        mesh_topology_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.origin.address_len != 6 ||
            !mesh_adaptive_report_valid(&s_mesh.adaptive, src, &p)) return true;
        mesh_observe_direct_report(rx, &p);
        if (!mesh_adaptive_report(&s_mesh.adaptive, src, &p,
                                   (rx->header.flags & MESH_FLAG_RELAYED) != 0, now_ms())) return true;
        mesh_refresh_peer_liveness(src, rx->timestamp_us);
    } else if (type == MESH_PKT_SPEAKER_REQUEST) {
        mesh_speaker_request_payload_t request;
        memcpy(&request, rx->payload, sizeof(request));
        uint32_t now = now_ms();
        if (!mesh_adaptive_speaker_request(&s_mesh.adaptive, src, &request,
                                           mesh_get_frame_counter(), now)) return true;
        mesh_refresh_peer_liveness(src, rx->timestamp_us);
        if (s_role == MESH_ROLE_COORDINATOR) {
            taskENTER_CRITICAL(&s_speaker_mux);
            if (request.active) {
                if (s_active_speaker_deadline_ms[src] <= now ||
                    !s_active_speaker_since_ms[src]) s_active_speaker_since_ms[src] = now;
                s_active_speaker_deadline_ms[src] = now + MESH_ADAPTIVE_REQUEST_ACTIVE_MS;
            } else {
                s_active_speaker_deadline_ms[src] = 0;
                s_active_speaker_since_ms[src] = 0;
                s_mesh.speaker_transition_grants &= (uint8_t)~bit(src);
            }
            taskEXIT_CRITICAL(&s_speaker_mux);
            update_speaker_grants();
        }
    } else if (type == MESH_PKT_SYNC_V3) {
        mesh_sync_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.leader_id != src || p.leader.address_len != 6 ||
            p.member_count < 1 || p.member_count > MESH_MAX_NODES ||
            memcmp(&p.leader, &s_mesh.adaptive.nodes[src - 1].identity, sizeof(p.leader)) ||
            p.phase_us >= MESH_FRAME_US || p.relay_depth != !!(rx->header.flags & MESH_FLAG_RELAYED) ||
            p.term != s_mesh.adaptive.term) return true;
        if (s_mesh.adaptive.transition_done && s_coordinator_id != s_mesh.adaptive.leader_id)
            mesh_adaptive_apply_transition();
        handle_sync_packet(rx);
        mesh_refresh_peer_liveness(src, rx->timestamp_us);
    } else if (type == MESH_PKT_SLOT_MAP) handle_slot_map_packet(rx);
    else if (type == MESH_PKT_SPEAKER_GRANT || type == MESH_PKT_SPEAKER_RELEASE) {
        /* Applied by the common packet dispatcher after authorization. */
    } else if (type == MESH_PKT_JOIN_ACK_V3) {
        mesh_join_ack_v3_payload_t p;
        memcpy(&p, rx->payload, sizeof(p));
        if (p.coordinator_id != src || p.term != s_mesh.adaptive.term ||
            !mesh_adaptive_identity_valid(&p.target) || p.target.address_len != 6 ||
            !bit(p.assigned_id) || p.assigned_id == src || p.slot_index >= MESH_MAX_NODES ||
            (p.assigned_id == s_node_id && memcmp(p.target.address, s_local_mac, 6))) return true;
        mesh_refresh_peer_liveness(src, rx->timestamp_us);
    } else {
        uint32_t frame = mesh_get_frame_counter();
        mesh_handover_payload_t p;
        if (type == MESH_PKT_HANDOVER_ACK) {
            mesh_handover_ack_payload_t ack;
            memcpy(&ack, rx->payload, sizeof(ack));
            if (!mesh_adaptive_ack(&s_mesh.adaptive, src, &ack, now_ms())) return true;
        } else {
            memcpy(&p, rx->payload, sizeof(p));
            bool valid = type == MESH_PKT_HANDOVER_PREPARE ?
                mesh_adaptive_receive_prepare(&s_mesh.adaptive, src, &p, frame, now_ms()) :
                type == MESH_PKT_HANDOVER_COMMIT ?
                mesh_adaptive_receive_commit(&s_mesh.adaptive, src, &p, now_ms()) :
                mesh_adaptive_cancel(&s_mesh.adaptive, src, &p);
            if (!valid) return true;
            if (type == MESH_PKT_HANDOVER_PREPARE) {
                mesh_handover_ack_payload_t ack = {.old_leader_id = p.old_leader_id,
                    .candidate_id = p.candidate_id, .next_term = p.next_term,
                    .switch_frame = p.switch_frame, .acknowledger_id = s_node_id};
                (void)mesh_send_control_packet(MESH_PKT_HANDOVER_ACK, &ack, sizeof(ack));
            }
            ESP_LOGI(TAG, "handover type=0x%02x leader=%u candidate=%u frame=%lu", type,
                     p.old_leader_id, p.candidate_id, (unsigned long)p.switch_frame);
        }
        mesh_refresh_peer_liveness(src, rx->timestamp_us);
    }
    forward_control(rx, &s_mesh.adaptive.nodes[src - 1].identity);
    return type != MESH_PKT_SPEAKER_GRANT && type != MESH_PKT_SPEAKER_RELEASE;
}

void mesh_adaptive_apply_transition(void)
{
    uint8_t new_id = s_mesh.adaptive.leader_id;
    const mesh_wire_identity_t *id = &s_mesh.adaptive.nodes[new_id - 1].identity;
    if (!mesh_adaptive_identity_valid(id) || id->address_len != 6) return;
    s_coordinator_id = new_id;
    memcpy(s_coordinator_mac, id->address, 6);
    s_role = new_id == s_node_id ? MESH_ROLE_COORDINATOR : MESH_ROLE_PARTICIPANT;
    s_mesh.adaptive.queue_head = s_mesh.adaptive.queue_count = 0;
    s_mesh.local_request_term = 0; /* Re-advertise current VOX state in the new term. */
    if (s_role == MESH_ROLE_COORDINATOR) {
        mesh_speaker_grants_snapshot();
        s_mesh.membership_publish_ms = 0;
    }
    ESP_LOGI(TAG, "handover switched leader=%u term=%lu local=%u", new_id,
             (unsigned long)s_mesh.adaptive.term, s_node_id);
}

void mesh_adaptive_frame_tick(uint32_t frame, uint32_t now)
{
    if (mesh_adaptive_tick(&s_mesh.adaptive, frame, now)) mesh_adaptive_apply_transition();
}

void mesh_adaptive_maintenance(uint32_t now)
{
    if (!s_mesh.adaptive.local_id) return;
    mesh_adaptive_expire(&s_mesh.adaptive, now);
    if (s_role == MESH_ROLE_COORDINATOR && !s_mesh.adaptive.prepared) {
        uint8_t candidate = mesh_adaptive_candidate(&s_mesh.adaptive, now);
        mesh_handover_payload_t p;
        if (candidate && mesh_adaptive_prepare(&s_mesh.adaptive, candidate,
                mesh_get_frame_counter(), &s_mesh.slot_map, now, &p)) {
            s_mesh.handover_retry_ms = now;
            (void)mesh_send_control_packet(MESH_PKT_HANDOVER_PREPARE, &p, sizeof(p));
            ESP_LOGI(TAG, "handover prepare candidate=%u frame=%lu", candidate,
                     (unsigned long)p.switch_frame);
        }
    }
    if (s_role == MESH_ROLE_COORDINATOR && s_mesh.adaptive.prepared &&
        !s_mesh.adaptive.committed) {
        mesh_handover_payload_t p;
        if (mesh_adaptive_commit(&s_mesh.adaptive, &p, now)) {
            s_mesh.handover_commit_retry_ms = now;
            (void)mesh_send_control_packet(MESH_PKT_HANDOVER_COMMIT, &p, sizeof(p));
        } else if ((uint32_t)(now - s_mesh.handover_retry_ms) >= 180 &&
                   (int32_t)(s_mesh.adaptive.deadline_ms - now) > 0) {
            s_mesh.handover_retry_ms = now;
            (void)mesh_send_control_packet(MESH_PKT_HANDOVER_PREPARE,
                                           &s_mesh.adaptive.pending, sizeof(s_mesh.adaptive.pending));
        }
    }
    if (s_role == MESH_ROLE_COORDINATOR && s_mesh.adaptive.committed &&
        !s_mesh.adaptive.transition_done &&
        (int32_t)(s_mesh.adaptive.recovery_deadline_ms - now) >= 0 &&
        (uint32_t)(now - s_mesh.handover_commit_retry_ms) >= 180) {
        s_mesh.handover_commit_retry_ms = now;
        (void)mesh_send_control_packet(MESH_PKT_HANDOVER_COMMIT,
                                       &s_mesh.adaptive.pending, sizeof(s_mesh.adaptive.pending));
    }
    mesh_adaptive_forward_packet_t packet;
    if (s_mesh.adaptive.queue_count) {
        uint8_t best = 0;
        uint8_t priority = 0;
        for (uint8_t i = 0; i < s_mesh.adaptive.queue_count; i++) {
            uint8_t type = s_mesh.adaptive.queue[
                (s_mesh.adaptive.queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE].header.type;
            uint8_t p = type == MESH_PKT_JOIN_V3 || type == MESH_PKT_JOIN_ACK_V3 ||
                type == MESH_PKT_MEMBERSHIP_V3 || type == MESH_PKT_SLOT_MAP ||
                (type >= MESH_PKT_HANDOVER_PREPARE && type <= MESH_PKT_HANDOVER_CANCEL) ? 2 :
                (type == MESH_PKT_SYNC_V3 || type == MESH_PKT_SPEAKER_REQUEST) ? 1 : 0;
            if (p > priority) { priority = p; best = i; }
        }
        if (best) {
            uint8_t index = (s_mesh.adaptive.queue_head + best) % MESH_ADAPTIVE_FORWARD_QUEUE;
            mesh_adaptive_forward_packet_t tmp = s_mesh.adaptive.queue[s_mesh.adaptive.queue_head];
            s_mesh.adaptive.queue[s_mesh.adaptive.queue_head] = s_mesh.adaptive.queue[index];
            s_mesh.adaptive.queue[index] = tmp;
        }
        uint32_t age = s_role == MESH_ROLE_COORDINATOR ? 0 :
            s_mesh.upstream_sync_us > 0 ?
            (uint32_t)((esp_timer_get_time() - s_mesh.upstream_sync_us) / 1000) : UINT32_MAX;
        /* SYNC is re-stamped again at the actual radio submission. */
        if (mesh_adaptive_forward_dequeue(&s_mesh.adaptive, &packet,
                mesh_get_frame_counter(), 0, age))
            (void)enqueue_forward_packet(&packet);
    }
}
