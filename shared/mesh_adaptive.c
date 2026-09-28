#include "mesh_adaptive.h"

#include <string.h>

static uint8_t bit(uint8_t id) { return id >= 1 && id <= 8 ? (uint8_t)(1U << (id - 1U)) : 0; }
static bool after(uint32_t a, uint32_t b) { return (uint32_t)(a - b) < UINT32_C(0x80000000); }
static bool serial_newer(uint32_t value, uint32_t previous)
{
    uint32_t delta = value - previous;
    return delta != 0U && delta < UINT32_C(0x80000000);
}
static bool fresh(uint32_t now, uint32_t then, uint32_t limit)
{
    return (uint32_t)(now - then) <= limit;
}
static uint8_t count(uint8_t mask)
{
    uint8_t n = 0;
    while (mask) { n += mask & 1U; mask >>= 1; }
    return n;
}
static bool slots_valid(const mesh_slot_map_payload_t *m, uint8_t members);
bool mesh_adaptive_identity_valid(const mesh_wire_identity_t *id)
{
    if (!id || id->address_len < 1 || id->address_len > 6) return false;
    bool zero = true, ff = true;
    for (uint8_t i = 0; i < 6; i++) {
        if (i >= id->address_len && id->address[i]) return false;
        if (i < id->address_len) { zero &= id->address[i] == 0; ff &= id->address[i] == 255; }
    }
    return !zero && !ff;
}
bool mesh_adaptive_cold_join(const mesh_wire_identity_t *local_identity,
                             uint8_t local_count,
                             const mesh_wire_identity_t *remote_identity,
                             uint8_t remote_count)
{
    if (!mesh_adaptive_identity_valid(local_identity) ||
        !mesh_adaptive_identity_valid(remote_identity) ||
        local_identity->address_len != remote_identity->address_len ||
        local_count != 1U || remote_count < 1U || remote_count > MESH_MAX_NODES) return false;
    int comparison = memcmp(local_identity->address, remote_identity->address,
                            local_identity->address_len);
    return comparison != 0 && (remote_count > 1U || comparison > 0);
}
bool mesh_adaptive_control_authorized(const mesh_adaptive_t *s, uint8_t type, uint8_t source)
{
    if (!s) return false;
    if (type == MESH_PKT_JOIN_V3) return source == 0 || (s->members & bit(source)) != 0;
    if (!(s->members & bit(source))) return false;
    if (type == MESH_PKT_TOPOLOGY || type == MESH_PKT_HANDOVER_ACK ||
        type == MESH_PKT_SPEAKER_REQUEST) return true;
    switch (type) {
    case MESH_PKT_JOIN_ACK_V3: case MESH_PKT_SYNC_V3: case MESH_PKT_SLOT_MAP:
    case MESH_PKT_MEMBERSHIP_V3:
    case MESH_PKT_SPEAKER_GRANT: case MESH_PKT_SPEAKER_RELEASE:
    case MESH_PKT_HANDOVER_PREPARE: case MESH_PKT_HANDOVER_COMMIT:
    case MESH_PKT_HANDOVER_CANCEL: return source == s->leader_id;
    default: return false;
    }
}
void mesh_adaptive_init(mesh_adaptive_t *s, uint8_t local_id, uint8_t leader_id,
                        uint32_t term, uint8_t members, uint32_t now_ms)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    if (!bit(local_id) || !bit(leader_id) || !(members & bit(local_id)) ||
        !(members & bit(leader_id))) return;
    s->local_id = local_id; s->leader_id = leader_id; s->term = term;
    s->members = members; s->leader_since_ms = now_ms;
    s->nodes[local_id - 1].present = true;
    s->nodes[local_id - 1].last_seen_ms = now_ms;
}
static void reset_member(mesh_adaptive_t *s, uint8_t id)
{
    memset(&s->nodes[id - 1U], 0, sizeof(s->nodes[0]));
    s->request_seq[id - 1U] = 0;
    s->request_deadline_ms[id - 1U] = 0;
    s->request_seq_seen[id - 1U] = false;
    s->request_active[id - 1U] = false;
    s->edge_live[id - 1U] = 0;
    for (uint8_t i = 0; i < MESH_MAX_NODES; i++) {
        s->edge_live[i] &= (uint8_t)~bit(id);
        s->nodes[i].direct_mask &= (uint8_t)~bit(id);
        s->nodes[i].quality[id - 1U] = 0;
    }
    memset(s->cache, 0, sizeof(s->cache));
    s->queue_head = s->queue_count = 0;
    s->prepared = s->committed = s->transition_done = false;
    s->better_id = 0;
}
bool mesh_adaptive_set_member(mesh_adaptive_t *s, uint8_t id,
                              const mesh_wire_identity_t *identity)
{
    if (!s || !bit(id) || (identity && !mesh_adaptive_identity_valid(identity)) ||
        (!identity && (id == s->local_id || id == s->leader_id))) return false;
    if (identity) {
        for (uint8_t other = 1; other <= MESH_MAX_NODES; other++)
            if (other != id && (s->members & bit(other)) &&
                s->nodes[other - 1U].identity.address_len &&
                memcmp(&s->nodes[other - 1U].identity, identity, sizeof(*identity)) == 0)
                return false;
    }
    mesh_adaptive_member_t *node = &s->nodes[id - 1U];
    if (identity && (s->members & bit(id)) && node->identity.address_len &&
        memcmp(&node->identity, identity, sizeof(*identity)) == 0) return true;
    reset_member(s, id);
    if (identity) {
        s->members |= bit(id);
        node->present = true;
        node->identity = *identity;
    } else s->members &= (uint8_t)~bit(id);
    return true;
}
bool mesh_adaptive_update_members(mesh_adaptive_t *s, uint8_t members)
{
    if (!s || !(members & bit(s->local_id)) || !(members & bit(s->leader_id))) return false;
    uint8_t changed = s->members ^ members;
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++)
        if (changed & bit(id)) reset_member(s, id);
    s->members = members;
    return true;
}
bool mesh_adaptive_report_valid(const mesh_adaptive_t *s, uint8_t source,
                                const mesh_topology_payload_t *r)
{
    if (!s || !r || !(s->members & bit(source)) || !mesh_adaptive_identity_valid(&r->origin) ||
        !bit(r->leader_id) || r->term != s->term || r->leader_id != s->leader_id ||
        (r->direct_mask & bit(source)) ||
        (r->slot_map.slot_count && !slots_valid(&r->slot_map, s->members)) ||
        (!r->slot_map.slot_count && r->slot_map.active_speaker_count)) return false;
    for (uint8_t i = 0; i < 8; i++) {
        if (r->direct_mask & bit(i + 1U)) {
            if (!(s->members & bit(i + 1U)) || r->quality[i] > 100 ||
                r->rssi_dbm[i] > 0 || r->rssi_dbm[i] < -127) return false;
        }
    }
    const mesh_adaptive_member_t *n = &s->nodes[source - 1U];
    if (!n->present || !mesh_adaptive_identity_valid(&n->identity) ||
        memcmp(&n->identity, &r->origin, sizeof(r->origin)) != 0) return false;
    return true;
}
bool mesh_adaptive_report(mesh_adaptive_t *s, uint8_t source, const mesh_topology_payload_t *r,
                           bool relayed, uint32_t now_ms)
{
    if (!mesh_adaptive_report_valid(s, source, r)) return false;
    mesh_adaptive_member_t *n = &s->nodes[source - 1U];
    if (n->seq_seen) {
        uint32_t delta = r->report_seq - n->report_seq;
        if (!delta || delta >= UINT32_C(0x80000000)) return false;
        uint8_t sample = (uint8_t)(100U / delta);
        n->loss_quality = (uint8_t)((3U * n->loss_quality + sample + 2U) / 4U);
    } else n->loss_quality = 100;
    bool first = !n->report_seen;
    n->seq_seen = true; n->report_seen = true;
    n->report_seq = r->report_seq; n->report_ms = now_ms;
    n->direct_mask = r->direct_mask; n->leader_id = r->leader_id; n->term = r->term;
    if (!relayed) n->last_seen_ms = now_ms;
    for (uint8_t i = 0; i < 8; i++) {
        if (r->direct_mask & bit(i + 1U)) {
            n->rssi[i] = (first || n->quality[i] == 0) ? r->rssi_dbm[i] :
                         (int8_t)((3 * (int)n->rssi[i] + r->rssi_dbm[i]) / 4);
            n->quality[i] = (first || n->quality[i] == 0) ? r->quality[i] :
                (uint8_t)((3U * n->quality[i] + r->quality[i] + 2U) / 4U);
        } else n->quality[i] = 0;
    }
    return true;
}
bool mesh_adaptive_speaker_request(mesh_adaptive_t *s, uint8_t source,
                                   const mesh_speaker_request_payload_t *request,
                                   uint32_t current_frame, uint32_t now_ms)
{
    if (!s || !request || !bit(source) || !(s->members & bit(source)) ||
        !mesh_adaptive_identity_valid(&s->nodes[source - 1U].identity) ||
        request->term != s->term || request->active > 1U ||
        (uint32_t)(current_frame - request->frame_counter) >
            MESH_ADAPTIVE_REQUEST_MAX_AGE_FRAMES) return false;
    uint8_t index = source - 1U;
    if (s->request_seq_seen[index] &&
        !serial_newer(request->request_seq, s->request_seq[index])) return false;
    s->request_seq_seen[index] = true;
    s->request_seq[index] = request->request_seq;
    s->request_active[index] = request->active != 0U;
    s->request_deadline_ms[index] = now_ms + MESH_ADAPTIVE_REQUEST_ACTIVE_MS;
    return true;
}
bool mesh_adaptive_speaker_request_active(const mesh_adaptive_t *s, uint8_t source,
                                          uint32_t now_ms)
{
    return s && bit(source) && (s->members & bit(source)) &&
        s->request_active[source - 1U] &&
        after(s->request_deadline_ms[source - 1U], now_ms);
}
void mesh_adaptive_expire(mesh_adaptive_t *s, uint32_t now_ms)
{
    if (!s) return;
    for (uint8_t i = 0; i < 8; i++) {
        mesh_adaptive_member_t *n = &s->nodes[i];
        if (n->report_seen && !fresh(now_ms, n->report_ms, MESH_ADAPTIVE_EXPIRE_MS)) {
            n->report_seen = false; n->direct_mask = 0; s->edge_live[i] = 0;
        }
    }
}
uint8_t mesh_adaptive_direct_graph(mesh_adaptive_t *s, uint8_t id, uint32_t now_ms)
{
    if (!s || !(s->members & bit(id))) return 0;
    mesh_adaptive_expire(s, now_ms);
    mesh_adaptive_member_t *a = &s->nodes[id - 1U];
    if (!a->report_seen) return 0;
    uint8_t edges = 0;
    for (uint8_t j = 1; j <= 8; j++) {
        mesh_adaptive_member_t *b = &s->nodes[j - 1U];
        uint8_t edge = bit(j);
        if (!(s->members & edge) || !(a->direct_mask & edge) || !b->report_seen ||
            !(b->direct_mask & bit(id))) { s->edge_live[id - 1U] &= (uint8_t)~edge; continue; }
        uint8_t q = a->quality[j - 1U] < b->quality[id - 1U] ? a->quality[j - 1U] : b->quality[id - 1U];
        if (q >= ((s->edge_live[id - 1U] & edge) ? 45U : 65U)) {
            edges |= edge; s->edge_live[id - 1U] |= edge;
        } else s->edge_live[id - 1U] &= (uint8_t)~edge;
    }
    return edges;
}
int mesh_adaptive_route(mesh_adaptive_t *s, uint8_t speaker, uint32_t now_ms, uint8_t *relay_mask)
{
    if (!relay_mask) return -1;
    *relay_mask = 0;
    if (!s || !(s->members & bit(speaker))) return -1;
    uint8_t first = mesh_adaptive_direct_graph(s, speaker, now_ms);
    uint8_t missing = s->members & (uint8_t)~(first | bit(speaker));
    if (!missing) return 0;
    uint8_t best = 0; int best_count = 9;
    for (unsigned mask = 1; mask <= 255; mask++) {
        if (((uint8_t)mask & (uint8_t)~first) || count((uint8_t)mask) >= best_count) continue;
        uint8_t covered = 0;
        for (uint8_t id = 1; id <= 8; id++)
            if (mask & bit(id)) covered |= mesh_adaptive_direct_graph(s, id, now_ms);
        if ((covered & missing) == missing) { best = (uint8_t)mask; best_count = count(best); }
    }
    if (best_count == 9) return -1;
    *relay_mask = best;
    return best_count;
}
static bool universal(mesh_adaptive_t *s, uint8_t id, uint32_t now)
{
    return (mesh_adaptive_direct_graph(s, id, now) | bit(id)) == s->members;
}
static int weakest(mesh_adaptive_t *s, uint8_t id, uint32_t now)
{
    uint8_t relays;
    if (mesh_adaptive_route(s, id, now, &relays) < 0) return -1000;
    int min = 1000;
    for (uint8_t peer = 1; peer <= 8; peer++) {
        if (!(s->members & bit(peer)) || peer == id ||
            !(mesh_adaptive_direct_graph(s, id, now) & bit(peer))) continue;
        mesh_adaptive_member_t *a = &s->nodes[id - 1], *b = &s->nodes[peer - 1];
        int r = a->rssi[peer - 1] < b->rssi[id - 1] ? a->rssi[peer - 1] : b->rssi[id - 1];
        int q = a->quality[peer - 1] < b->quality[id - 1] ? a->quality[peer - 1] : b->quality[id - 1];
        int score = r + q / 4;
        if (score < min) min = score;
    }
    return min - (universal(s, id, now) ? 0 : 12);
}
uint8_t mesh_adaptive_candidate(mesh_adaptive_t *s, uint32_t now_ms)
{
    if (!s || s->local_id != s->leader_id || s->prepared ||
        !after(now_ms, s->leader_since_ms + MESH_ADAPTIVE_HOLD_MS)) return 0;
    int leader_score = weakest(s, s->leader_id, now_ms);
    /* A fragmented leader report must not trigger a switch. */
    if (leader_score == -1000) { s->better_id = 0; return 0; }
    uint8_t best = 0; int best_score = leader_score + 7;
    for (uint8_t id = 1; id <= 8; id++) {
        if (!(s->members & bit(id)) || id == s->leader_id) continue;
        if (!universal(s, id, now_ms)) continue;
        int score = weakest(s, id, now_ms);
        if (score > best_score) { best = id; best_score = score; }
    }
    if (best != s->better_id) { s->better_id = best; s->better_since_ms = now_ms; }
    return best && after(now_ms, s->better_since_ms + MESH_ADAPTIVE_BETTER_MS) ? best : 0;
}
static bool slots_valid(const mesh_slot_map_payload_t *m, uint8_t members)
{
    if (!m || !members || !m->slot_count || m->slot_count > 8 ||
        m->slot_ids[m->slot_count - 1U] == 0U ||
        m->active_speaker_count > MESH_MAX_ACTIVE_SPEAKERS) return false;
    uint8_t found = 0, speakers = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t v = m->slot_ids[i], b = bit(v);
        if (i >= m->slot_count && v) return false;
        if (v && (!(members & b) || (found & b))) return false;
        found |= b;
    }
    for (uint8_t i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        uint8_t v = m->active_speaker_ids[i], b = bit(v);
        if (i >= m->active_speaker_count && (v || m->relay_masks[i])) return false;
        if (i < m->active_speaker_count && (!(members & b) || (speakers & b) ||
            (m->relay_masks[i] & (uint8_t)~members))) return false;
        speakers |= b;
    }
    return found == members;
}
bool mesh_adaptive_membership_valid(const mesh_membership_v3_payload_t *snapshot,
                                    uint8_t *members)
{
    if (!snapshot || !snapshot->term || !bit(snapshot->leader_id)) return false;
    uint8_t mask = 0;
    for (uint8_t slot = 0; slot < MESH_MAX_NODES; slot++) {
        uint8_t id = snapshot->slot_map.slot_ids[slot];
        if (id && !bit(id)) return false;
        mask |= bit(id);
    }
    if (!(mask & bit(snapshot->leader_id)) || !slots_valid(&snapshot->slot_map, mask)) return false;
    const mesh_wire_identity_t empty = {0};
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        const mesh_wire_identity_t *identity = &snapshot->identities[id - 1U];
        if (!(mask & bit(id))) {
            if (memcmp(identity, &empty, sizeof(empty)) != 0) return false;
            continue;
        }
        if (!mesh_adaptive_identity_valid(identity)) return false;
        for (uint8_t earlier = 1; earlier < id; earlier++)
            if ((mask & bit(earlier)) &&
                memcmp(identity, &snapshot->identities[earlier - 1U], sizeof(*identity)) == 0)
                return false;
    }
    if (members) *members = mask;
    return true;
}
bool mesh_adaptive_apply_membership(mesh_adaptive_t *s, uint8_t source,
                                    const mesh_membership_v3_payload_t *snapshot)
{
    uint8_t members;
    if (!s || !mesh_adaptive_control_authorized(s, MESH_PKT_MEMBERSHIP_V3, source) ||
        !mesh_adaptive_membership_valid(snapshot, &members) ||
        snapshot->term != s->term || snapshot->leader_id != s->leader_id ||
        (s->membership_revision_seen &&
         !serial_newer(snapshot->revision, s->membership_revision)) ||
        !(members & bit(s->local_id))) return false;
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        if (id != s->local_id && id != s->leader_id) continue;
        const mesh_wire_identity_t *bound = &s->nodes[id - 1U].identity;
        if (bound->address_len &&
            memcmp(bound, &snapshot->identities[id - 1U], sizeof(*bound)) != 0) return false;
    }
    /* No operation below can fail. Validate every byte before the first reset. */
    for (uint8_t id = 1; id <= MESH_MAX_NODES; id++) {
        const mesh_wire_identity_t *identity = &snapshot->identities[id - 1U];
        if ((s->members & bit(id)) == (members & bit(id)) &&
            (!(members & bit(id)) ||
             memcmp(&s->nodes[id - 1U].identity, identity, sizeof(*identity)) == 0)) continue;
        reset_member(s, id);
        if (members & bit(id)) {
            s->nodes[id - 1U].identity = *identity;
            s->nodes[id - 1U].present = true;
        }
    }
    s->members = members;
    s->membership_revision = snapshot->revision;
    s->membership_revision_seen = true;
    return true;
}
bool mesh_adaptive_membership_removes_local(const mesh_adaptive_t *s, uint8_t source,
                                            const mesh_membership_v3_payload_t *snapshot)
{
    uint8_t members;
    if (!s || !bit(s->local_id) || !(s->members & bit(s->local_id)) ||
        !mesh_adaptive_control_authorized(s, MESH_PKT_MEMBERSHIP_V3, source) ||
        !mesh_adaptive_membership_valid(snapshot, &members) ||
        snapshot->term != s->term || snapshot->leader_id != s->leader_id ||
        (s->membership_revision_seen &&
         !serial_newer(snapshot->revision, s->membership_revision)) ||
        (members & bit(s->local_id))) return false;
    const mesh_wire_identity_t *leader = &s->nodes[s->leader_id - 1U].identity;
    return mesh_adaptive_identity_valid(leader) &&
        memcmp(leader, &snapshot->identities[s->leader_id - 1U], sizeof(*leader)) == 0;
}
uint32_t mesh_adaptive_next_membership_revision(mesh_adaptive_t *s)
{
    if (!s || !s->term || s->local_id != s->leader_id) return 0;
    return ++s->publish_revision;
}
static bool proposal_matches(const mesh_handover_payload_t *a, const mesh_handover_payload_t *b)
{
    return memcmp(a, b, sizeof(*a)) == 0;
}
static void reset_term_reports(mesh_adaptive_t *s)
{
    s->membership_revision_seen = false;
    s->membership_revision = s->publish_revision = 0;
    memset(s->request_seq, 0, sizeof(s->request_seq));
    memset(s->request_seq_seen, 0, sizeof(s->request_seq_seen));
    memset(s->request_active, 0, sizeof(s->request_active));
    memset(s->request_deadline_ms, 0, sizeof(s->request_deadline_ms));
    memset(s->edge_live, 0, sizeof(s->edge_live));
    for (uint8_t id = 0; id < MESH_MAX_NODES; id++) {
        s->nodes[id].report_seen = false;
        s->nodes[id].seq_seen = false;
        s->nodes[id].direct_mask = 0;
        memset(s->nodes[id].quality, 0, sizeof(s->nodes[id].quality));
    }
}
bool mesh_adaptive_prepare(mesh_adaptive_t *s, uint8_t candidate, uint32_t frame,
                           const mesh_slot_map_payload_t *slots, uint32_t now,
                           mesh_handover_payload_t *out)
{
    if (!s || !out || s->prepared || s->local_id != s->leader_id || !s->term ||
        s->term == UINT32_MAX || candidate != mesh_adaptive_candidate(s, now) ||
        !slots_valid(slots, s->members)) return false;
    s->pending = (mesh_handover_payload_t){.old_leader_id = s->leader_id,
        .candidate_id = candidate, .next_term = s->term + 1U,
        .switch_frame = frame + MESH_ADAPTIVE_SWITCH_LEAD_FRAMES,
        .members = s->members, .slot_map = *slots};
    s->ack_mask = bit(s->local_id); s->prepared = true;
    s->deadline_ms = now + MESH_ADAPTIVE_ACK_DEADLINE_MS;
    s->recovery_deadline_ms = now + MESH_ADAPTIVE_SWITCH_LEAD_FRAMES * MESH_FRAME_MS +
                              MESH_ADAPTIVE_RECOVERY_GRACE_MS;
    s->committed = false; s->transition_done = false;
    *out = s->pending; return true;
}
bool mesh_adaptive_receive_prepare(mesh_adaptive_t *s, uint8_t sender,
                                   const mesh_handover_payload_t *p, uint32_t frame, uint32_t now)
{
    if (!s || !p || sender != s->leader_id || sender == s->local_id ||
        p->old_leader_id != sender || !(s->members & bit(p->candidate_id)) ||
        p->candidate_id == sender || p->members != s->members || !s->term ||
        s->term == UINT32_MAX || p->next_term != s->term + 1U) return false;
    if (s->prepared) return proposal_matches(&s->pending, p);
    if (
        (uint32_t)(p->switch_frame - frame) < MESH_ADAPTIVE_PREPARE_MIN_FRAMES ||
        (uint32_t)(p->switch_frame - frame) > MESH_ADAPTIVE_SWITCH_LEAD_FRAMES ||
        !slots_valid(&p->slot_map, s->members) ||
        (s->local_id == p->candidate_id
             ? !universal(s, p->candidate_id, now)
             : (!mesh_adaptive_identity_valid(&s->nodes[s->local_id - 1U].identity) ||
                !mesh_adaptive_identity_valid(&s->nodes[p->candidate_id - 1U].identity) ||
                !(mesh_adaptive_direct_graph(s, s->local_id, now) & bit(p->candidate_id)))))
        return false;
    s->pending = *p; s->prepared = true; s->committed = false;
    s->deadline_ms = now + MESH_ADAPTIVE_ACK_DEADLINE_MS;
    s->recovery_deadline_ms = now +
        (uint32_t)(p->switch_frame - frame) * MESH_FRAME_MS + MESH_ADAPTIVE_RECOVERY_GRACE_MS;
    s->transition_done = false;
    return true;
}
bool mesh_adaptive_ack(mesh_adaptive_t *s, uint8_t sender,
                       const mesh_handover_ack_payload_t *ack, uint32_t now)
{
    if (!s || !ack || !s->prepared || s->committed || !bit(s->local_id) ||
        !(s->members & bit(s->local_id)) || !after(s->deadline_ms, now) ||
        !(s->members & bit(sender)) || sender != ack->acknowledger_id ||
        s->pending.old_leader_id != s->leader_id || s->term == UINT32_MAX ||
        ack->old_leader_id != s->leader_id ||
        ack->candidate_id != s->pending.candidate_id ||
        ack->next_term != s->term + 1U || ack->next_term != s->pending.next_term ||
        ack->switch_frame != s->pending.switch_frame) return false;
    if (s->local_id == s->leader_id) s->ack_mask |= bit(sender);
    return true;
}
bool mesh_adaptive_commit(mesh_adaptive_t *s, mesh_handover_payload_t *out, uint32_t now)
{
    if (!s || !out || !s->prepared || s->local_id != s->leader_id ||
        s->ack_mask != s->members || !after(s->recovery_deadline_ms, now) ||
        (!s->committed && !after(s->deadline_ms, now))) return false;
    s->committed = true; *out = s->pending; return true;
}
bool mesh_adaptive_receive_commit(mesh_adaptive_t *s, uint8_t sender,
                                  const mesh_handover_payload_t *p, uint32_t now)
{
    if (!s || !p || sender != s->leader_id || !s->prepared ||
        !proposal_matches(&s->pending, p) || !after(s->recovery_deadline_ms, now)) return false;
    s->committed = true; return true;
}
bool mesh_adaptive_cancel(mesh_adaptive_t *s, uint8_t sender,
                          const mesh_handover_payload_t *p)
{
    if (!s || !p || !s->prepared || s->committed || sender != s->leader_id ||
        !proposal_matches(&s->pending, p)) return false;
    s->prepared = false; s->better_id = 0; return true;
}
bool mesh_adaptive_tick(mesh_adaptive_t *s, uint32_t frame, uint32_t now)
{
    if (!s || !s->prepared) return false;
    if (!s->committed && s->local_id == s->leader_id && after(now, s->deadline_ms)) {
        s->prepared = false; s->better_id = 0; return false;
    }
    if (!s->transition_done && after(now, s->recovery_deadline_ms)) {
        s->prepared = false; s->better_id = 0; return false;
    }
    if (!s->committed || s->transition_done || s->local_id != s->pending.candidate_id ||
        !after(frame, s->pending.switch_frame)) return false;
    s->leader_id = s->pending.candidate_id; s->term = s->pending.next_term;
    s->leader_since_ms = now; s->transition_done = true; s->better_id = 0;
    s->prepared = false; reset_term_reports(s);
    return true;
}
bool mesh_adaptive_reconcile_sync(mesh_adaptive_t *s, uint8_t sender,
                                   const mesh_sync_v3_payload_t *sync, uint32_t now)
{
    if (!s || !sync || !s->prepared || s->transition_done ||
        s->local_id == s->pending.candidate_id || !after(s->recovery_deadline_ms, now) ||
        sender != s->pending.candidate_id || sync->leader_id != sender ||
        sync->member_count < 1U || sync->member_count > MESH_MAX_NODES ||
        sync->term != s->pending.next_term || sync->phase_us >= 20000U ||
        !mesh_adaptive_identity_valid(&sync->leader) ||
        memcmp(&sync->leader, &s->nodes[sender - 1U].identity, sizeof(sync->leader)) ||
        !after(sync->frame_counter, s->pending.switch_frame)) return false;
    s->committed = true;
    s->leader_id = sender; s->term = sync->term;
    s->leader_since_ms = now; s->transition_done = true; s->better_id = 0;
    s->prepared = false; reset_term_reports(s);
    return true;
}
bool mesh_adaptive_stamp_sync(mesh_sync_v3_payload_t *sync, bool relay,
                              uint32_t frame, uint16_t phase, uint32_t age)
{
    if (!sync || !mesh_adaptive_identity_valid(&sync->leader) || !bit(sync->leader_id) ||
        sync->member_count < 1U || sync->member_count > MESH_MAX_NODES ||
        !sync->term || phase >= 20000U ||
        (relay && (sync->relay_depth != 0 || age > MESH_ADAPTIVE_SYNC_MAX_AGE_MS))) return false;
    sync->frame_counter = frame; sync->phase_us = phase; sync->relay_depth = relay ? 1U : 0U;
    return true;
}
static bool forward_type(uint8_t type)
{
    switch (type) {
    case MESH_PKT_JOIN_V3: case MESH_PKT_JOIN_ACK_V3: case MESH_PKT_TOPOLOGY:
    case MESH_PKT_SLOT_MAP: case MESH_PKT_SPEAKER_GRANT: case MESH_PKT_SPEAKER_RELEASE:
    case MESH_PKT_HANDOVER_PREPARE: case MESH_PKT_HANDOVER_ACK:
    case MESH_PKT_HANDOVER_COMMIT: case MESH_PKT_HANDOVER_CANCEL:
    case MESH_PKT_MEMBERSHIP_V3: case MESH_PKT_SPEAKER_REQUEST:
    case MESH_PKT_SYNC_V3: return true;
    default: return false;
    }
}
static uint8_t priority_type(uint8_t type)
{
    if (type == MESH_PKT_JOIN_V3 || type == MESH_PKT_JOIN_ACK_V3 ||
        type == MESH_PKT_MEMBERSHIP_V3 || type == MESH_PKT_SLOT_MAP ||
        (type >= MESH_PKT_HANDOVER_PREPARE && type <= MESH_PKT_HANDOVER_CANCEL)) return 2;
    return type == MESH_PKT_SPEAKER_REQUEST ? 1 : 0;
}
static bool forward_length_valid(uint8_t type, uint16_t length)
{
    switch (type) {
    case MESH_PKT_JOIN_V3: return length == sizeof(mesh_join_v3_payload_t);
    case MESH_PKT_JOIN_ACK_V3: return length == sizeof(mesh_join_ack_v3_payload_t);
    case MESH_PKT_TOPOLOGY: return length == sizeof(mesh_topology_payload_t);
    case MESH_PKT_SPEAKER_REQUEST: return length == sizeof(mesh_speaker_request_payload_t);
    case MESH_PKT_MEMBERSHIP_V3: return length == sizeof(mesh_membership_v3_payload_t);
    case MESH_PKT_SLOT_MAP: return length == sizeof(mesh_slot_map_payload_t);
    case MESH_PKT_SPEAKER_GRANT: return length == sizeof(mesh_speaker_grant_payload_t);
    case MESH_PKT_SPEAKER_RELEASE: return length == sizeof(mesh_speaker_release_payload_t);
    case MESH_PKT_SYNC_V3: return length == sizeof(mesh_sync_v3_payload_t);
    case MESH_PKT_HANDOVER_ACK: return length == sizeof(mesh_handover_ack_payload_t);
    case MESH_PKT_HANDOVER_PREPARE: case MESH_PKT_HANDOVER_COMMIT:
    case MESH_PKT_HANDOVER_CANCEL: return length == sizeof(mesh_handover_payload_t);
    default: return false;
    }
}
static void drop_queued(mesh_adaptive_t *s, uint8_t offset)
{
    for (uint8_t i = offset; i + 1U < s->queue_count; i++)
        s->queue[(s->queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE] =
            s->queue[(s->queue_head + i + 1U) % MESH_ADAPTIVE_FORWARD_QUEUE];
    s->queue_count--;
}
bool mesh_adaptive_forward_enqueue(mesh_adaptive_t *s, const mesh_header_t *h,
                                   const uint8_t *payload, uint16_t len,
                                   const mesh_wire_identity_t *origin, uint32_t term,
                                   bool joined_synced, uint32_t now)
{
    if (!s || !h || !payload || !joined_synced || !bit(s->local_id) ||
        h->version != MESH_PROTOCOL_VERSION || !forward_type(h->type) ||
        !forward_length_valid(h->type, len) ||
        (h->flags & MESH_FLAG_RELAYED) || h->ttl != 2 || h->payload_len != len ||
        len > sizeof(s->queue[0].payload) ||
        (h->src_id == 0 && (h->type != MESH_PKT_JOIN_V3 ||
         !mesh_adaptive_identity_valid(origin))) ||
        (h->src_id != 0 && (!(s->members & bit(h->src_id)) && h->type != MESH_PKT_JOIN_V3)))
        return false;
    if (h->type == MESH_PKT_JOIN_V3 && (len != sizeof(mesh_join_v3_payload_t) ||
        !origin || memcmp(origin, payload, sizeof(*origin)) != 0)) return false;
    if ((s->local_id == s->leader_id &&
         (h->type == MESH_PKT_JOIN_V3 || h->type == MESH_PKT_SPEAKER_REQUEST ||
          h->type == MESH_PKT_HANDOVER_ACK || h->type == MESH_PKT_TOPOLOGY)) ||
        (h->src_id == s->leader_id &&
         (h->type == MESH_PKT_SPEAKER_REQUEST || h->type == MESH_PKT_TOPOLOGY)))
        return false;
    if (h->type == MESH_PKT_JOIN_ACK_V3) {
        mesh_join_ack_v3_payload_t ack;
        memcpy(&ack, payload, sizeof(ack));
        const mesh_wire_identity_t *local = &s->nodes[s->local_id - 1U].identity;
        if (mesh_adaptive_identity_valid(local) &&
            memcmp(&ack.target, local, sizeof(ack.target)) == 0) return false;
    }
    if (h->type == MESH_PKT_MEMBERSHIP_V3) {
        mesh_membership_v3_payload_t snapshot;
        memcpy(&snapshot, payload, sizeof(snapshot));
        if (!mesh_adaptive_control_authorized(s, h->type, h->src_id) ||
            snapshot.term != s->term || snapshot.leader_id != s->leader_id ||
            term != snapshot.term || !mesh_adaptive_membership_valid(&snapshot, NULL)) return false;
    }
    if (h->type == MESH_PKT_SPEAKER_REQUEST) {
        mesh_speaker_request_payload_t request;
        memcpy(&request, payload, sizeof(request));
        if (!mesh_adaptive_control_authorized(s, h->type, h->src_id) ||
            !mesh_adaptive_identity_valid(&s->nodes[h->src_id - 1U].identity) ||
            request.term != s->term || term != request.term || request.active > 1U)
            return false;
    }
    if (h->type == MESH_PKT_SYNC_V3) {
        mesh_sync_v3_payload_t sync;
        memcpy(&sync, payload, sizeof(sync));
        if (!mesh_adaptive_identity_valid(&sync.leader) || !bit(sync.leader_id) ||
            sync.member_count < 1U || sync.member_count > MESH_MAX_NODES ||
            !sync.term || sync.phase_us >= MESH_FRAME_MS * 1000U || sync.relay_depth != 0U)
            return false;
    }
    for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_CACHE; i++) {
        mesh_adaptive_forward_key_t *k = &s->cache[i];
        if (k->used && fresh(now, k->seen_ms, 1000U) && k->type == h->type &&
            k->src_id == h->src_id && k->seq == h->seq && k->term == term &&
            (h->src_id != 0 || (origin && memcmp(&k->origin, origin, sizeof(*origin)) == 0)))
            return false;
    }
    if (s->queue_count == MESH_ADAPTIVE_FORWARD_QUEUE) {
        bool replaced = false;
        if (priority_type(h->type)) {
            for (uint8_t i = 0; i < s->queue_count; i++) {
                mesh_adaptive_forward_packet_t *queued =
                    &s->queue[(s->queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE];
                if (priority_type(queued->header.type) < priority_type(h->type) &&
                    !fresh(now, queued->enqueued_ms, 1000U)) {
                    drop_queued(s, i); replaced = true; break;
                }
            }
        }
        if (!replaced) return false;
    }
    mesh_adaptive_forward_key_t *slot = NULL;
    uint32_t oldest_age = 0;
    for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_CACHE; i++)
        if (!s->cache[i].used || !fresh(now, s->cache[i].seen_ms, 1000U)) { slot = &s->cache[i]; break; }
    if (!slot) {
        for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_CACHE; i++) {
            uint32_t age = now - s->cache[i].seen_ms;
            if (!slot || age > oldest_age) { slot = &s->cache[i]; oldest_age = age; }
        }
    }
    *slot = (mesh_adaptive_forward_key_t){.type = h->type, .src_id = h->src_id,
        .seq = h->seq, .term = term, .seen_ms = now, .used = true};
    if (origin) slot->origin = *origin;
    mesh_adaptive_forward_packet_t *packet = &s->queue[(s->queue_head + s->queue_count) % MESH_ADAPTIVE_FORWARD_QUEUE];
    packet->header = *h; packet->header.ttl = 1;
    packet->header.flags |= MESH_FLAG_RELAYED;
    packet->payload_len = len; packet->enqueued_ms = now;
    memcpy(packet->payload, payload, len);
    s->queue_count++;
    return true;
}
bool mesh_adaptive_forward_dequeue(mesh_adaptive_t *s, mesh_adaptive_forward_packet_t *out,
                                   uint32_t frame, uint16_t phase_us,
                                   uint32_t upstream_age_ms)
{
    if (!s || !out || !s->queue_count) return false;
    *out = s->queue[s->queue_head];
    s->queue_head = (uint8_t)((s->queue_head + 1U) % MESH_ADAPTIVE_FORWARD_QUEUE);
    s->queue_count--;
    if (out->header.type == MESH_PKT_SYNC_V3) {
        mesh_sync_v3_payload_t sync;
        memcpy(&sync, out->payload, sizeof(sync));
        if (!mesh_adaptive_stamp_sync(&sync, true, frame, phase_us, upstream_age_ms)) return false;
        memcpy(out->payload, &sync, sizeof(sync));
    }
    return true;
}
