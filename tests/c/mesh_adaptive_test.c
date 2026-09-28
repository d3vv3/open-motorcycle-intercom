#include "shared/mesh_adaptive.h"
#include "shared/mesh_membership.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static mesh_topology_payload_t report(uint8_t id, uint8_t mask, uint32_t seq)
{
    mesh_topology_payload_t r = {0};
    r.origin.address_len = 6;
    r.origin.address[0] = id;
    r.report_seq = seq;
    r.direct_mask = mask;
    r.leader_id = 1;
    r.term = 1;
    for (uint8_t i = 0; i < 8; i++) {
        r.rssi_dbm[i] = -55;
        r.quality[i] = 90;
    }
    return r;
}

static void init_bound(mesh_adaptive_t *s, uint8_t local, uint8_t members)
{
    mesh_adaptive_init(s, local, 1, 1, members, 0);
    for (uint8_t id = 1; id <= 8; id++) {
        if (!(members & (uint8_t)(1U << (id - 1U)))) continue;
        mesh_wire_identity_t identity = report(id, 0, 0).origin;
        assert(mesh_adaptive_set_member(s, id, &identity));
    }
}

static void chain(mesh_adaptive_t *s, uint32_t now, uint32_t seq)
{
    mesh_topology_payload_t a = report(1, 2, seq);
    mesh_topology_payload_t b = report(2, 1 | 4, seq);
    mesh_topology_payload_t c = report(3, 2, seq);
    assert(mesh_adaptive_report(s, 1, &a, false, now));
    assert(mesh_adaptive_report(s, 2, &b, false, now));
    assert(mesh_adaptive_report(s, 3, &c, true, now));
}

static mesh_slot_map_payload_t slots(void)
{
    mesh_slot_map_payload_t m = {.slot_count = 3, .slot_ids = {1, 2, 3},
                                  .active_speaker_count = 1, .active_speaker_ids = {3},
                                  .relay_masks = {2}};
    return m;
}

static mesh_membership_v3_payload_t membership_snapshot(void)
{
    mesh_membership_v3_payload_t snapshot = {.term = 1, .revision = 1, .leader_id = 1,
                                              .slot_map = slots()};
    for (uint8_t id = 1; id <= 3; id++)
        snapshot.identities[id - 1U] = report(id, 0, 0).origin;
    return snapshot;
}

static void test_cold_join_policy(void)
{
    mesh_wire_identity_t a = {.address_len = 6, .address = {0x20}};
    mesh_wire_identity_t d = {.address_len = 6, .address = {0x10}};
    mesh_wire_identity_t higher = {.address_len = 6, .address = {0x30}};
    assert(mesh_adaptive_cold_join(&a, 1, &d, 1));
    assert(!mesh_adaptive_cold_join(&d, 1, &a, 1));
    assert(!mesh_adaptive_cold_join(&a, 3, &d, 1)); /* established A keeps IDs */
    assert(mesh_adaptive_cold_join(&d, 1, &a, 3)); /* D joins despite higher A MAC */
    assert(mesh_adaptive_cold_join(&higher, 1, &a, 1));
    assert(!mesh_adaptive_cold_join(&a, 1, &a, 3));
    assert(!mesh_adaptive_cold_join(&a, 0, &d, 2));
    assert(!mesh_adaptive_cold_join(&a, 9, &d, 2));
    assert(!mesh_adaptive_cold_join(&d, 1, &a, 0));
    assert(!mesh_adaptive_cold_join(&d, 1, &a, 9));
    d.address_len = 5;
    assert(!mesh_adaptive_cold_join(&a, 1, &d, 2));
    d.address_len = 7;
    assert(!mesh_adaptive_cold_join(&a, 1, &d, 2));
}

static void test_authoritative_membership(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 1, 7);
    chain(&s, 100, 1);
    mesh_membership_v3_payload_t snapshot = membership_snapshot();
    uint8_t members = 0;
    assert(sizeof(snapshot) == 79);
    assert(mesh_adaptive_membership_valid(&snapshot, &members) && members == 7);
    s.prepared = true;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    mesh_adaptive_t duplicate_before = s;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &duplicate_before, sizeof(s)) == 0);
    snapshot.revision = 2;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(s.prepared && s.nodes[1].report_seen); /* newer identical snapshot is inert */
    mesh_adaptive_t before = s;
    snapshot.identities[2] = snapshot.identities[1];
    assert(!mesh_adaptive_membership_valid(&snapshot, NULL));
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    snapshot.term = 2;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    snapshot.leader_id = 2;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    snapshot.slot_map.slot_ids[2] = 0;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    snapshot.identities[3].address_len = 6;
    snapshot.identities[3].address[0] = 4; /* absent ID must be all zero */
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    snapshot.identities[0].address[0] = 40;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    snapshot = membership_snapshot();
    snapshot.revision = 3;
    assert(!mesh_adaptive_apply_membership(&s, 2, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);

    /* Authoritative ID replacement resets only changed bindings and edges. */
    snapshot.revision = 3;
    snapshot.identities[2].address[0] = 44;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(s.members == 7 && !s.prepared);
    assert(s.nodes[0].report_seen && s.nodes[1].report_seen && !s.nodes[2].report_seen);
    mesh_topology_payload_t old = report(3, 2, 2);
    assert(!mesh_adaptive_report(&s, 3, &old, true, 200));
    old.origin = snapshot.identities[2];
    assert(mesh_adaptive_report(&s, 3, &old, true, 200));

    snapshot.revision = 4;
    snapshot.slot_map = (mesh_slot_map_payload_t){.slot_count = 2, .slot_ids = {1, 2}};
    memset(&snapshot.identities[2], 0, sizeof(snapshot.identities[2]));
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(s.members == 3 && !s.nodes[2].present);
    snapshot = membership_snapshot(); /* old revision cannot re-add removed ID */
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(s.members == 3);

    init_bound(&s, 1, 7);
    snapshot = membership_snapshot();
    snapshot.revision = UINT32_MAX - 1U;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    snapshot.revision = UINT32_MAX;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    snapshot.revision = 0;
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    snapshot.revision = UINT32_MAX;
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(mesh_adaptive_next_membership_revision(&s) == 1U);
    assert(mesh_adaptive_next_membership_revision(&s) == 2U);

    snapshot = membership_snapshot();
    snapshot.slot_map = (mesh_slot_map_payload_t){.slot_count = 3, .slot_ids = {1, 0, 3}};
    memset(&snapshot.identities[1], 0, sizeof(snapshot.identities[1]));
    assert(mesh_adaptive_membership_valid(&snapshot, &members) && members == 5);
    snapshot.slot_map.slot_count = 4; /* trailing empty slot is noncanonical */
    assert(!mesh_adaptive_membership_valid(&snapshot, NULL));
}

static void test_authoritative_local_eviction(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 2, 7);
    mesh_membership_v3_payload_t snapshot = membership_snapshot();
    assert(mesh_adaptive_apply_membership(&s, 1, &snapshot));
    mesh_adaptive_t before = s;
    snapshot.revision = 2;
    snapshot.slot_map.slot_ids[1] = 0; /* preserve 3 in slot 2 */
    memset(&snapshot.identities[1], 0, sizeof(snapshot.identities[1]));
    assert(!mesh_adaptive_membership_valid(&snapshot, NULL)); /* relay mask still names 2 */
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.slot_map.relay_masks[0] = 0;
    assert(mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    assert(!mesh_adaptive_apply_membership(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);

    snapshot.revision = 1;
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.revision = 0;
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.revision = 2;
    assert(!mesh_adaptive_membership_removes_local(&s, 3, &snapshot));
    snapshot.term = 2;
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.term = 1;
    snapshot.leader_id = 3;
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.leader_id = 1;
    snapshot.identities[0].address[0] = 9;
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.identities[0].address[0] = 1;
    snapshot.identities[2] = snapshot.identities[0];
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    snapshot.identities[2] = report(3, 0, 0).origin;
    snapshot.slot_map.slot_count = 4; /* noncanonical trailing empty slot */
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
    assert(memcmp(&s, &before, sizeof(s)) == 0);

    /* A source ID without a trusted leader binding cannot evict anyone. */
    snapshot.slot_map.slot_count = 3;
    s.nodes[0].identity = (mesh_wire_identity_t){0};
    assert(!mesh_adaptive_membership_removes_local(&s, 1, &snapshot));
}

static void test_routes_and_election(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 1, 7);
    chain(&s, 100, UINT32_MAX - 1U);
    uint8_t mask = 0;
    assert(mesh_adaptive_route(&s, 1, 100, &mask) == 1 && mask == 2);
    assert(mesh_adaptive_direct_graph(&s, 1, 100) == 2);
    assert(s.nodes[1].loss_quality == 100);
    chain(&s, 1100, 0); /* uint32 wrap and one lost beacon */
    assert(s.nodes[1].loss_quality < 100);
    mesh_topology_payload_t old = report(2, 1 | 4, UINT32_MAX - 1U);
    assert(!mesh_adaptive_report(&s, 2, &old, false, 1101));
    assert(mesh_adaptive_candidate(&s, 1100) == 0);
    chain(&s, 10100, 1);
    assert(mesh_adaptive_candidate(&s, 10100) == 0);
    chain(&s, 13101, 2);
    assert(mesh_adaptive_candidate(&s, 13101) == 2);

    mesh_handover_payload_t proposal;
    mesh_slot_map_payload_t m = slots();
    assert(mesh_adaptive_prepare(&s, 2, 0xfffffff0U, &m, 13101, &proposal));
    assert(proposal.switch_frame == 144 && proposal.slot_map.slot_ids[0] == 1);
    assert(!mesh_adaptive_commit(&s, &proposal, 13102));
    mesh_handover_ack_payload_t ack = {.old_leader_id = 1, .candidate_id = 2,
                                       .next_term = 2, .switch_frame = 144, .acknowledger_id = 2};
    assert(mesh_adaptive_ack(&s, 2, &ack, 13102));
    assert(!mesh_adaptive_commit(&s, &proposal, 13102));
    ack.acknowledger_id = 3;
    assert(mesh_adaptive_ack(&s, 3, &ack, 13103));
    assert(mesh_adaptive_commit(&s, &proposal, 13104));
    assert(!mesh_adaptive_tick(&s, 143, 16000));
    assert(!mesh_adaptive_tick(&s, 144, 16400)); /* old leader waits for candidate SYNC */
    mesh_sync_v3_payload_t sync = {.leader_id = 2, .member_count = 3,
        .term = 2, .frame_counter = 144,
        .phase_us = 500, .leader = s.nodes[1].identity};
    assert(mesh_adaptive_reconcile_sync(&s, 2, &sync, 16420));
    assert(s.local_id == 1 && s.leader_id == 2 && s.term == 2);
    mesh_membership_v3_payload_t new_term = membership_snapshot();
    new_term.term = 2; new_term.leader_id = 2; new_term.revision = 1;
    assert(mesh_adaptive_apply_membership(&s, 2, &new_term));
    assert(s.membership_revision_seen && s.membership_revision == 1);
    assert(!mesh_adaptive_tick(&s, 45, 14320));

    mesh_membership_snapshot_t member = {.state = MESH_STATE_ACTIVE,
        .role = MESH_ROLE_COORDINATOR, .node_id = 1, .coordinator_id = 1, .slot_index = 0};
    assert(mesh_membership_apply_handover(&member, 1, 2, 2));
    assert(member.role == MESH_ROLE_PARTICIPANT && member.slot_index == 0 && member.node_id == 1);
    assert(!mesh_membership_apply_handover(&member, 2, 1, 2));
}

static void test_partition_and_silence(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 1, 15);
    chain(&s, 100, 1);
    mesh_topology_payload_t d = report(4, 4, 1);
    mesh_topology_payload_t c = report(3, 2 | 8, 2);
    assert(mesh_adaptive_report(&s, 3, &c, true, 200));
    assert(mesh_adaptive_report(&s, 4, &d, true, 200));
    uint8_t mask;
    assert(mesh_adaptive_route(&s, 1, 200, &mask) == -1);
    assert(mesh_adaptive_candidate(&s, 14000) == 0);
    /* A forwarded report advertises edges but cannot invent local direct RSSI. */
    assert(s.nodes[2].last_seen_ms == 0);
    assert(mesh_adaptive_report_valid(&s, 3, &c));
    assert(!mesh_adaptive_report(&s, 3, &c, false, 201)); /* graph dedupe only */
    init_bound(&s, 1, 7);
    chain(&s, 100, 1);
    assert(mesh_adaptive_direct_graph(&s, 1, 100) == 2);
    mesh_topology_payload_t malformed = report(2, 1 | 4, 2);
    malformed.origin.address_len = 7;
    assert(!mesh_adaptive_report(&s, 2, &malformed, false, 1000));
    malformed = report(2, 4, 2);
    assert(mesh_adaptive_report(&s, 2, &malformed, false, 1100));
    assert(mesh_adaptive_direct_graph(&s, 1, 1100) == 0);
    assert(!mesh_adaptive_report(&s, 2, &malformed, false, 1101));
    malformed.direct_mask = 1 | 4;
    malformed.quality[0] = 101;
    assert(!mesh_adaptive_report(&s, 2, &malformed, false, 1102));
    for (uint32_t t = 2100; t < 10000; t += 1000) chain(&s, t, (uint8_t)(t / 1000 + 2));
    assert(mesh_adaptive_route(&s, 1, 9100, &mask) == 1);
    assert(mesh_adaptive_direct_graph(&s, 1, 14000) == 0);
    /* Age does not make a previous sequence acceptable. */
    malformed = report(2, 1 | 4, 11);
    assert(!mesh_adaptive_report(&s, 2, &malformed, false, 14001));
    malformed = report(2, 1 | 4, 200);
    assert(mesh_adaptive_report_valid(&s, 2, &malformed));
    assert(mesh_adaptive_report(&s, 2, &malformed, false, 14002));
    assert(s.nodes[1].report_seq == 200);
    malformed.origin.address[0] = 4;
    assert(!mesh_adaptive_report_valid(&s, 2, &malformed));
    malformed.origin.address[0] = 2;
    malformed.quality[0] = 101;
    assert(!mesh_adaptive_report_valid(&s, 2, &malformed));
}

static void test_pregrant_request_relay(void)
{
    mesh_adaptive_t relay, coordinator;
    init_bound(&relay, 2, 7);
    init_bound(&coordinator, 1, 7);
    mesh_speaker_request_payload_t request = {.term = 1, .request_seq = UINT32_MAX,
        .frame_counter = 0, .active = 1};
    mesh_header_t h = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_SPEAKER_REQUEST,
                       .src_id = 3, .seq = 7, .ttl = 2, .payload_len = sizeof(request)};
    mesh_adaptive_forward_packet_t forwarded;
    assert(mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&request,
                                         sizeof(request), NULL, 1, true, 0));
    assert(mesh_adaptive_forward_dequeue(&relay, &forwarded, 0, 0, 0));
    assert(forwarded.header.src_id == 3 && forwarded.header.ttl == 1 &&
           (forwarded.header.flags & MESH_FLAG_RELAYED));
    mesh_topology_payload_t periodic = report(2, 0, 1);
    mesh_header_t beacon = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_TOPOLOGY,
        .src_id = 2, .ttl = 2, .payload_len = sizeof(periodic)};
    for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_QUEUE; i++) {
        beacon.seq = i;
        assert(mesh_adaptive_forward_enqueue(&relay, &beacon, (const uint8_t *)&periodic,
                                             sizeof(periodic), NULL, 1, true, 0));
    }
    h.seq = 8;
    assert(mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&request,
                                         sizeof(request), NULL, 1, true, 1001));
    assert(mesh_adaptive_forward_dequeue(&relay, &forwarded, 0, 0, 0));
    assert(forwarded.header.type == MESH_PKT_TOPOLOGY && forwarded.header.seq == 1);
    while (relay.queue_count)
        assert(mesh_adaptive_forward_dequeue(&relay, &forwarded, 0, 0, 0));
    mesh_slot_map_payload_t critical = slots();
    mesh_header_t map_header = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_SLOT_MAP,
        .src_id = 1, .ttl = 2, .payload_len = sizeof(critical)};
    for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_QUEUE; i++) {
        map_header.seq = (uint8_t)(20U + i);
        assert(mesh_adaptive_forward_enqueue(&relay, &map_header, (const uint8_t *)&critical,
                                             sizeof(critical), NULL, 1, true, 2000));
    }
    h.seq = 9;
    assert(!mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&request,
                                          sizeof(request), NULL, 1, true, 3001));
    assert(mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 0));
    assert(mesh_adaptive_speaker_request_active(&coordinator, 3, 1000));
    assert(!mesh_adaptive_speaker_request(&coordinator, 3, &request, 1, 20));
    assert(!mesh_adaptive_speaker_request_active(&coordinator, 3, 1501));
    request.request_seq = 0; /* true uint32 wrap */
    request.frame_counter = UINT32_MAX;
    assert(mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2000));
    assert(mesh_adaptive_speaker_request_active(&coordinator, 3, 2000));
    request.request_seq = 1;
    request.frame_counter = 0;
    assert(!mesh_adaptive_speaker_request(&coordinator, 3, &request, 51, 2020));
    assert(!mesh_adaptive_speaker_request(&coordinator, 3, &request, UINT32_MAX, 2020));
    request.term = 2;
    assert(!mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2020));
    request.term = 1;
    assert(!mesh_adaptive_speaker_request(&coordinator, 4, &request, 0, 2020));
    request.active = 2;
    assert(!mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2020));
    request.active = 0;
    assert(mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2020));
    assert(!mesh_adaptive_speaker_request_active(&coordinator, 3, 2020));
    request.active = 1; request.request_seq = 2;
    assert(mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2030));
    mesh_wire_identity_t replacement = report(3, 0, 0).origin;
    replacement.address[0] = 43;
    assert(mesh_adaptive_set_member(&coordinator, 3, &replacement));
    assert(!mesh_adaptive_speaker_request_active(&coordinator, 3, 2031));
    assert(mesh_adaptive_speaker_request(&coordinator, 3, &request, 0, 2031));
}

static void test_report_loss_is_not_link_quality(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 1, 3);
    mesh_topology_payload_t a = report(1, 2, 1);
    mesh_topology_payload_t b = report(2, 1, 1);
    assert(mesh_adaptive_report(&s, 1, &a, false, 100));
    assert(mesh_adaptive_report(&s, 2, &b, false, 100));
    for (uint32_t i = 1; i <= 4; i++) {
        a.report_seq++;
        b.report_seq += 2; /* sustained missing direct report beacons */
        assert(mesh_adaptive_report(&s, 1, &a, false, 100 + i * 1000));
        assert(mesh_adaptive_report(&s, 2, &b, false, 100 + i * 1000));
    }
    assert(s.nodes[1].loss_quality < s.nodes[0].loss_quality);
    assert(mesh_adaptive_direct_graph(&s, 1, 4100) == 2);
    b.report_seq++;
    b.quality[0] = 0; /* direct RF estimator, not beacon loss, governs edge */
    assert(mesh_adaptive_report(&s, 2, &b, false, 4200));
    assert(s.nodes[1].loss_quality < 100);
    assert(mesh_adaptive_direct_graph(&s, 1, 4200) == 2); /* hysteresis */
    for (uint32_t i = 1; i <= 4; i++) {
        b.report_seq++;
        assert(mesh_adaptive_report(&s, 2, &b, false, 4200 + i * 100));
    }
    assert(mesh_adaptive_direct_graph(&s, 1, 4600) == 0);
}

static void test_binding_and_replacement(void)
{
    mesh_adaptive_t s;
    mesh_adaptive_init(&s, 1, 1, 1, 7, 0);
    mesh_topology_payload_t b = report(2, 1 | 4, 1);
    assert(!mesh_adaptive_report(&s, 2, &b, false, 100));
    init_bound(&s, 1, 7);
    assert(mesh_adaptive_report(&s, 2, &b, false, 100));
    mesh_wire_identity_t other = b.origin;
    other.address[0] = 42;
    b.origin = other;
    assert(!mesh_adaptive_report(&s, 2, &b, false, 101));
    assert(!mesh_adaptive_set_member(&s, 1, &s.nodes[1].identity));
    assert(mesh_adaptive_set_member(&s, 2, &other));
    assert(mesh_adaptive_report(&s, 2, &b, false, 102));
    assert(s.nodes[1].loss_quality == 100);
    assert(mesh_adaptive_set_member(&s, 2, NULL));
    assert(!mesh_adaptive_report(&s, 2, &b, false, 103));
    assert(mesh_adaptive_update_members(&s, 7));
    assert(!mesh_adaptive_report(&s, 2, &b, false, 104));
    assert(mesh_adaptive_set_member(&s, 2, &other));
    assert(mesh_adaptive_report(&s, 2, &b, false, 105));
}

static void test_follower_recovery(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 3, 7);
    chain(&s, 100, 1);
    mesh_handover_payload_t p = {.old_leader_id = 1, .candidate_id = 2,
        .next_term = 2, .switch_frame = 160, .members = 7, .slot_map = slots()};
    assert(!mesh_adaptive_receive_prepare(&s, 2, &p, 0, 100));
    assert(mesh_adaptive_receive_prepare(&s, 1, &p, 0, 100));
    assert(!mesh_adaptive_cancel(&s, 2, &p));
    mesh_handover_payload_t conflict = p;
    conflict.switch_frame++;
    assert(!mesh_adaptive_receive_commit(&s, 1, &conflict, 120));
    assert(!mesh_adaptive_tick(&s, 160, 3300));
    mesh_sync_v3_payload_t sync = {.leader_id = 2, .member_count = 3,
        .term = 2, .frame_counter = 160,
        .phase_us = 500, .leader = s.nodes[1].identity};
    sync.member_count = 0;
    assert(!mesh_adaptive_reconcile_sync(&s, 2, &sync, 3300));
    sync.member_count = 9;
    assert(!mesh_adaptive_reconcile_sync(&s, 2, &sync, 3300));
    sync.member_count = 3;
    assert(!mesh_adaptive_reconcile_sync(&s, 1, &sync, 3300));
    assert(mesh_adaptive_reconcile_sync(&s, 2, &sync, 3300));
    assert(s.leader_id == 2 && s.term == 2);

    init_bound(&s, 3, 7);
    chain(&s, 100, 1);
    assert(mesh_adaptive_receive_prepare(&s, 1, &p, 0, 100));
    assert(!mesh_adaptive_tick(&s, 160, 3300));
    assert(s.leader_id == 1);
    assert(mesh_adaptive_receive_commit(&s, 1, &p, 3500));
    assert(!mesh_adaptive_tick(&s, 160, 3500));
    assert(mesh_adaptive_reconcile_sync(&s, 2, &sync, 3501));

    init_bound(&s, 3, 7);
    chain(&s, 100, 1);
    assert(mesh_adaptive_receive_prepare(&s, 1, &p, 0, 100));
    assert(!mesh_adaptive_receive_prepare(&s, 1, &conflict, 0, 200));
    assert(mesh_adaptive_cancel(&s, 1, &p));
    assert(!mesh_adaptive_reconcile_sync(&s, 2, &sync, 3400));

    init_bound(&s, 3, 5);
    mesh_topology_payload_t first = report(1, 4, 1);
    mesh_topology_payload_t third = report(3, 1, 1);
    assert(mesh_adaptive_report(&s, 1, &first, false, 100));
    assert(mesh_adaptive_report(&s, 3, &third, false, 100));
    mesh_handover_payload_t sparse = {.old_leader_id = 1, .candidate_id = 3,
        .next_term = 2, .switch_frame = 160, .members = 5,
        .slot_map = {.slot_count = 3, .slot_ids = {1, 0, 3}}};
    assert(mesh_adaptive_receive_prepare(&s, 1, &sparse, 0, 100));
    assert(s.pending.slot_map.slot_ids[1] == 0);
}

static void test_lost_candidate_commit(void)
{
    mesh_adaptive_t old, candidate;
    init_bound(&old, 1, 7);
    init_bound(&candidate, 2, 7);
    chain(&old, 10100, 1);
    chain(&candidate, 10100, 1);
    assert(mesh_adaptive_candidate(&old, 10100) == 0);
    chain(&old, 13101, 2);
    chain(&candidate, 13101, 2);
    mesh_handover_payload_t p;
    mesh_slot_map_payload_t m = slots();
    assert(mesh_adaptive_prepare(&old, 2, 1000, &m, 13101, &p));
    assert(mesh_adaptive_receive_prepare(&candidate, 1, &p, 1001, 13121));
    mesh_handover_ack_payload_t ack = {.old_leader_id = 1, .candidate_id = 2,
        .next_term = 2, .switch_frame = 1160, .acknowledger_id = 2};
    assert(mesh_adaptive_ack(&old, 2, &ack, 13200));
    ack.acknowledger_id = 3;
    assert(mesh_adaptive_ack(&old, 3, &ack, 13200));
    assert(mesh_adaptive_commit(&old, &p, 13201));
    /* Candidate missed COMMIT: old coordinator remains, including at boundary. */
    assert(!mesh_adaptive_tick(&candidate, 1160, 16321));
    assert(!mesh_adaptive_tick(&old, 1160, 16321));
    assert(old.leader_id == 1 && candidate.leader_id == 1);
    assert(mesh_adaptive_commit(&old, &p, 16400)); /* retry through grace */
    assert(mesh_adaptive_receive_commit(&candidate, 1, &p, 16401));
    mesh_speaker_request_payload_t active = {.term = 1, .request_seq = 1,
        .frame_counter = 1161, .active = 1};
    assert(mesh_adaptive_speaker_request(&candidate, 3, &active, 1161, 16401));
    assert(mesh_adaptive_tick(&candidate, 1161, 16420));
    assert(!mesh_adaptive_speaker_request_active(&candidate, 3, 16420));
    mesh_sync_v3_payload_t sync = {.leader_id = 2, .member_count = 3,
        .term = 2, .frame_counter = 1161,
        .phase_us = 400, .leader = candidate.nodes[1].identity};
    assert(mesh_adaptive_reconcile_sync(&old, 2, &sync, 16430));
    assert(old.leader_id == 2 && candidate.leader_id == 2);
    assert(!old.membership_revision_seen && !candidate.membership_revision_seen);
    assert(mesh_adaptive_next_membership_revision(&candidate) == 1);
    sync.term = 1;
    assert(!mesh_adaptive_reconcile_sync(&old, 2, &sync, 16440));
}

static void test_ack_relay_to_leader(void)
{
    mesh_adaptive_t a, b, c, c_asymmetric, c_stale;
    init_bound(&a, 1, 7);
    init_bound(&b, 2, 7);
    init_bound(&c, 3, 7);
    init_bound(&c_asymmetric, 3, 7);
    init_bound(&c_stale, 3, 7);
    chain(&a, 10100, 1); chain(&b, 10100, 1);
    mesh_topology_payload_t b_row = report(2, 1 | 4, 1);
    mesh_topology_payload_t c_row = report(3, 2, 1);
    assert(mesh_adaptive_report(&c, 2, &b_row, false, 10100));
    assert(mesh_adaptive_report(&c, 3, &c_row, false, 10100));
    assert(mesh_adaptive_candidate(&a, 10100) == 0);
    chain(&a, 13101, 2); chain(&b, 13101, 2);
    b_row.report_seq = 2; c_row.report_seq = 2;
    assert(mesh_adaptive_report(&c, 2, &b_row, false, 13101));
    assert(mesh_adaptive_report(&c, 3, &c_row, false, 13101));
    assert(!c.nodes[0].report_seen); /* C never received A's topology row. */
    mesh_slot_map_payload_t map = slots();
    mesh_handover_payload_t proposal;
    assert(mesh_adaptive_prepare(&a, 2, 1000, &map, 13101, &proposal));
    assert(mesh_adaptive_receive_prepare(&b, 1, &proposal, 1001, 13121));
    assert(mesh_adaptive_receive_prepare(&c, 1, &proposal, 1001, 13121));
    assert((mesh_adaptive_direct_graph(&c, 2, 13121) & 1U) == 0U);
    assert((mesh_adaptive_direct_graph(&c, 3, 13121) & 2U) != 0U);
    b_row.direct_mask = 1; /* B does not advertise hearing C: no reverse edge. */
    assert(mesh_adaptive_report(&c_asymmetric, 2, &b_row, false, 13101));
    assert(mesh_adaptive_report(&c_asymmetric, 3, &c_row, false, 13101));
    assert(!mesh_adaptive_receive_prepare(&c_asymmetric, 1, &proposal, 1001, 13121));
    assert(!c_asymmetric.prepared);
    b_row.direct_mask = 1 | 4;
    assert(mesh_adaptive_report(&c_stale, 2, &b_row, false, 9000));
    assert(mesh_adaptive_report(&c_stale, 3, &c_row, false, 13101));
    assert(!mesh_adaptive_receive_prepare(&c_stale, 1, &proposal, 1001, 13121));
    mesh_handover_ack_payload_t ack = {.old_leader_id = 1, .candidate_id = 2,
        .next_term = 2, .switch_frame = proposal.switch_frame, .acknowledger_id = 3};
    mesh_header_t header = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_HANDOVER_ACK,
        .src_id = 3, .seq = 9, .ttl = 2, .payload_len = sizeof(ack)};
    mesh_handover_ack_payload_t invalid = ack;
    invalid.next_term = 3;
    bool accepted = mesh_adaptive_ack(&b, 3, &invalid, 13150);
    assert(!accepted);
    if (accepted)
        assert(mesh_adaptive_forward_enqueue(&b, &header, (const uint8_t *)&invalid,
                                             sizeof(invalid), NULL, 1, true, 13150));
    assert(b.queue_count == 0 && b.ack_mask == 0);
    assert(!mesh_adaptive_ack(&b, 2, &ack, 13150));
    assert(mesh_adaptive_ack(&c, 3, &ack, 13150));
    assert(mesh_adaptive_ack(&b, 3, &ack, 13160));
    assert(b.ack_mask == 0 && !mesh_adaptive_commit(&b, &proposal, 13160));
    assert(mesh_adaptive_forward_enqueue(&b, &header, (const uint8_t *)&ack,
                                         sizeof(ack), NULL, 1, true, 13160));
    mesh_adaptive_forward_packet_t forwarded;
    assert(mesh_adaptive_forward_dequeue(&b, &forwarded, 1002, 0, 0));
    assert(forwarded.header.src_id == 3 && forwarded.header.ttl == 1 &&
           (forwarded.header.flags & MESH_FLAG_RELAYED));
    mesh_handover_ack_payload_t received;
    memcpy(&received, forwarded.payload, sizeof(received));
    assert(mesh_adaptive_ack(&a, forwarded.header.src_id, &received, 13180));
    assert(a.ack_mask == (1 | 4));
    assert(!mesh_adaptive_commit(&a, &proposal, 13180));
    ack.acknowledger_id = 2;
    assert(mesh_adaptive_ack(&a, 2, &ack, 13200));
    assert(a.ack_mask == 7 && mesh_adaptive_commit(&a, &proposal, 13200));
    assert(b.ack_mask == 0 && !mesh_adaptive_commit(&b, &proposal, 13200));
}

static void test_local_control_termination(void)
{
    mesh_adaptive_t leader, relay;
    init_bound(&leader, 1, 7);
    init_bound(&relay, 2, 7);
    mesh_join_v3_payload_t join = {.origin = { .address_len = 6, .address = {9} },
        .target = { .address_len = 6, .address = {1} }};
    mesh_header_t h = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_JOIN_V3,
        .src_id = 0, .seq = 1, .ttl = 2, .payload_len = sizeof(join)};
    assert(!mesh_adaptive_forward_enqueue(&leader, &h, (const uint8_t *)&join,
                                          sizeof(join), &join.origin, 1, true, 100));
    assert(mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&join,
                                         sizeof(join), &join.origin, 1, true, 100));
    mesh_topology_payload_t topology = report(3, 0, 1);
    h.type = MESH_PKT_TOPOLOGY; h.src_id = 3; h.seq = 2;
    h.payload_len = sizeof(topology);
    assert(!mesh_adaptive_forward_enqueue(&leader, &h, (const uint8_t *)&topology,
                                          sizeof(topology), NULL, 1, true, 100));
    assert(mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&topology,
                                         sizeof(topology), NULL, 1, true, 100));
    h.src_id = 1; h.seq = 3; topology = report(1, 0, 1);
    assert(!mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&topology,
                                          sizeof(topology), NULL, 1, true, 100));
    mesh_speaker_request_payload_t request = {.term = 1, .request_seq = 1,
        .frame_counter = 0, .active = 1};
    h.type = MESH_PKT_SPEAKER_REQUEST; h.src_id = 3; h.seq = 4;
    h.payload_len = sizeof(request);
    assert(!mesh_adaptive_forward_enqueue(&leader, &h, (const uint8_t *)&request,
                                          sizeof(request), NULL, 1, true, 100));
    assert(mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&request,
                                         sizeof(request), NULL, 1, true, 100));
    h.src_id = 1; h.seq = 5;
    assert(!mesh_adaptive_forward_enqueue(&relay, &h, (const uint8_t *)&request,
                                          sizeof(request), NULL, 1, true, 100));
    mesh_handover_ack_payload_t ack = {.old_leader_id = 1, .candidate_id = 2,
        .next_term = 2, .switch_frame = 160, .acknowledger_id = 3};
    h.type = MESH_PKT_HANDOVER_ACK; h.src_id = 3; h.seq = 6;
    h.payload_len = sizeof(ack);
    assert(!mesh_adaptive_forward_enqueue(&leader, &h, (const uint8_t *)&ack,
                                          sizeof(ack), NULL, 1, true, 100));
    assert(leader.queue_count == 0 && relay.queue_count == 3);
}

static void test_forwarding_and_clock(void)
{
    mesh_adaptive_t s;
    init_bound(&s, 2, 3);
    assert(mesh_adaptive_control_authorized(&s, MESH_PKT_JOIN_V3, 0));
    assert(mesh_adaptive_control_authorized(&s, MESH_PKT_TOPOLOGY, 2));
    assert(!mesh_adaptive_control_authorized(&s, MESH_PKT_SLOT_MAP, 2));
    assert(mesh_adaptive_control_authorized(&s, MESH_PKT_SLOT_MAP, 1));
    mesh_header_t h = {.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_JOIN_V3,
                       .src_id = 0, .seq = 9, .ttl = 2,
                       .payload_len = sizeof(mesh_join_v3_payload_t)};
    mesh_join_v3_payload_t join = {0};
    join.origin.address_len = 6;
    join.origin.address[0] = 10;
    join.target.address_len = 6;
    join.target.address[0] = 1;
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&join, sizeof(join),
                                         &join.origin, 1, true, 0));
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&join, sizeof(join),
                                          &join.origin, 1, true, 1));
    join.origin.address[0] = 11;
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&join, sizeof(join),
                                         &join.origin, 1, true, 2));
    mesh_adaptive_forward_packet_t out;
    assert(mesh_adaptive_forward_dequeue(&s, &out, 0, 0, 0));
    assert(out.header.ttl == 1 && (out.header.flags & MESH_FLAG_RELAYED));
    h = out.header;
    assert(!mesh_adaptive_forward_enqueue(&s, &h, out.payload, out.payload_len,
                                          &join.origin, 1, true, 3));
    h.type = MESH_PKT_AUDIO;
    assert(!mesh_adaptive_forward_enqueue(&s, &h, out.payload, out.payload_len,
                                          &join.origin, 1, true, 4));
    mesh_sync_v3_payload_t sync = {.leader_id = 1, .member_count = 2,
        .term = 1, .leader = join.target};
    sync.member_count = 0;
    assert(!mesh_adaptive_stamp_sync(&sync, false, 1, 0, 0));
    sync.member_count = 9;
    assert(!mesh_adaptive_stamp_sync(&sync, false, 1, 0, 0));
    sync.member_count = 2;
    assert(mesh_adaptive_stamp_sync(&sync, false, UINT32_MAX, 19999, 0));
    assert(mesh_adaptive_stamp_sync(&sync, true, 0, 0, 599));
    assert(sync.frame_counter == 0 && sync.phase_us == 0 && sync.relay_depth == 1);
    sync.relay_depth = 0;
    assert(!mesh_adaptive_stamp_sync(&sync, true, 1, 0, 601));
    h = (mesh_header_t){.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_SYNC_V3,
                         .src_id = 1, .seq = 10, .ttl = 2, .payload_len = sizeof(sync)};
    sync.member_count = 0;
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                          NULL, 1, true, 10));
    sync.member_count = 9;
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                          NULL, 1, true, 10));
    sync.member_count = 2;
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                          NULL, 1, true, 10));
    assert(mesh_adaptive_forward_dequeue(&s, &out, 0, 0, 0));
    assert(!mesh_adaptive_forward_dequeue(&s, &out, 1, 100, 601));
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                         NULL, 1, true, 1011));
    assert(mesh_adaptive_forward_dequeue(&s, &out, 0, 19, 400));
    memcpy(&sync, out.payload, sizeof(sync));
    assert(sync.frame_counter == 0 && sync.phase_us == 19 &&
           sync.relay_depth == 1 && sync.member_count == 2);

    /* Cache eviction does not stall sustained traffic after queue drain. */
    sync.relay_depth = 0;
    for (uint8_t i = 0; i < 40; i++) {
        h.seq = (uint8_t)(20U + i);
        assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                             NULL, 1, true, 1100));
        assert(mesh_adaptive_forward_dequeue(&s, &out, 10, 0, 0));
    }
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&sync, sizeof(sync),
                                          NULL, 1, true, 1101));

    mesh_topology_payload_t periodic = report(2, 0, 1);
    h = (mesh_header_t){.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_TOPOLOGY,
                         .src_id = 2, .ttl = 2, .payload_len = sizeof(periodic)};
    for (uint8_t i = 0; i < MESH_ADAPTIVE_FORWARD_QUEUE; i++) {
        h.seq = i;
        assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&periodic, sizeof(periodic),
                                             NULL, 1, true, 1200));
    }
    join.origin.address[0] = 12;
    h.type = MESH_PKT_JOIN_V3;
    h.src_id = 0; h.seq = 200; h.payload_len = sizeof(join);
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&join, sizeof(join),
                                         &join.origin, 1, true, 2201));
    assert(mesh_adaptive_forward_dequeue(&s, &out, 11, 0, 0));
    assert(out.header.type == MESH_PKT_TOPOLOGY && out.header.seq == 1);

    init_bound(&s, 2, 3);
    mesh_membership_v3_payload_t identity_map = membership_snapshot();
    identity_map.slot_map = (mesh_slot_map_payload_t){.slot_count = 2, .slot_ids = {1, 2}};
    memset(&identity_map.identities[2], 0, sizeof(identity_map.identities[2]));
    h = (mesh_header_t){.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_MEMBERSHIP_V3,
                        .src_id = 2, .seq = 1, .ttl = 2, .payload_len = sizeof(identity_map)};
    assert(!mesh_adaptive_control_authorized(&s, h.type, 2));
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&identity_map,
                                          sizeof(identity_map), NULL, 1, true, 100));
    h.src_id = 1;
    assert(mesh_adaptive_control_authorized(&s, h.type, 1));
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&identity_map,
                                         sizeof(identity_map), NULL, 1, true, 101));
    assert(mesh_adaptive_forward_dequeue(&s, &out, 10, 0, 0));
    assert(out.header.type == MESH_PKT_MEMBERSHIP_V3 && out.header.ttl == 1);
    assert(!mesh_adaptive_forward_enqueue(&s, &out.header, out.payload, out.payload_len,
                                           NULL, 1, true, 102));

    mesh_join_ack_v3_payload_t targeted = {.target = s.nodes[1].identity,
        .assigned_id = 2, .coordinator_id = 1, .term = 1};
    h = (mesh_header_t){.version = MESH_PROTOCOL_VERSION, .type = MESH_PKT_JOIN_ACK_V3,
        .src_id = 1, .seq = 2, .ttl = 2, .payload_len = sizeof(targeted)};
    assert(!mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&targeted,
                                          sizeof(targeted), NULL, 1, true, 200));
    targeted.target = report(3, 0, 0).origin;
    assert(mesh_adaptive_forward_enqueue(&s, &h, (const uint8_t *)&targeted,
                                         sizeof(targeted), NULL, 1, true, 201));
}

int main(void)
{
    test_cold_join_policy(); test_authoritative_membership(); test_authoritative_local_eviction();
    test_routes_and_election(); test_partition_and_silence();
    test_pregrant_request_relay(); test_report_loss_is_not_link_quality();
    test_binding_and_replacement(); test_follower_recovery();
    test_lost_candidate_commit(); test_ack_relay_to_leader();
    test_local_control_termination(); test_forwarding_and_clock();
    puts("mesh adaptive tests passed");
    return 0;
}
