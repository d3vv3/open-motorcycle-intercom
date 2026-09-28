/**
 * @file mesh_protocol_tx.c
 * @brief Mesh Protocol Control Transmission
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "esb_radio.h"
#include "mesh_protocol_internal.h"
#include "tdma.h"

LOG_MODULE_DECLARE(mesh);

static int send_sync(mesh_protocol_context_t *context)
{
    uint32_t frame;
    uint16_t phase;
    if (!tdma_clock_snapshot(&frame, &phase)) return -EAGAIN;
    mesh_sync_v3_payload_t sync = {.leader = mesh_protocol_local_identity(),
        .leader_id = context->node_id, .term = context->term,
        .member_count = (uint8_t)__builtin_popcount((unsigned)context->adaptive.members)};
    if (!mesh_adaptive_stamp_sync(&sync, false, frame, phase, 0)) return -EINVAL;
    int ret = mesh_protocol_tx_send_packet_ex(MESH_PKT_SYNC_V3, &sync, sizeof(sync), 2, 0,
                                               context->node_id, context->tx_seq++);
    if (ret == 0) context->last_sync_time = k_uptime_get_32();
    return ret;
}

int mesh_protocol_tx_send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len,
                                    uint8_t ttl, uint8_t flags, uint8_t src_id, uint8_t seq)
{
    audio_bundle_view_t bundle;
    if (type == MESH_PKT_AUDIO ||
        (type == MESH_PKT_AUDIO_V2 && !audio_bundle_parse(payload, len, &bundle))) {
        return -EINVAL;
    }
    if (len > MESH_PACKET_PAYLOAD_MAX) {
        return -EMSGSIZE;
    }

    uint8_t buf[MESH_PACKET_OUTER_MAX];
    mesh_header_t *hdr = (mesh_header_t *)buf;

    hdr->version = MESH_PROTOCOL_VERSION;
    hdr->type = type;
    hdr->src_id = src_id;
    hdr->seq = seq;
    hdr->ttl = ttl;
    hdr->flags = flags;
    hdr->talk_channel = mesh_protocol_context_get()->talk_channel;
    hdr->payload_len = len;

    if (payload && len > 0) {
        memcpy(buf + sizeof(mesh_header_t), payload, len);
    }

    return esb_radio_send(buf, sizeof(mesh_header_t) + len);
}

int mesh_protocol_tx_send_packet(mesh_protocol_context_t *context, mesh_pkt_type_t type,
                                 const void *payload, uint16_t len)
{
    return mesh_protocol_tx_send_packet_ex(type, payload, len, 0, 0, context->node_id,
                                           context->tx_seq++);
}

int mesh_protocol_tx_queue_control(mesh_protocol_context_t *context, mesh_pkt_type_t type,
                                   const void *payload, uint16_t len)
{
    if (type == MESH_PKT_AUDIO || type == MESH_PKT_AUDIO_V2) {
        return -EINVAL;
    }
    if (len > MESH_PACKET_PAYLOAD_MAX) {
        return -EMSGSIZE;
    }

    /* Coalescing does not create a new publication or RF sequence. */
    if (type == MESH_PKT_TOPOLOGY || type == MESH_PKT_MEMBERSHIP_V3 ||
        type == MESH_PKT_SPEAKER_REQUEST || type == MESH_PKT_SPEAKER_GRANT) {
        for (uint8_t i = context->control_tail; i != context->control_head;
             i = (uint8_t)((i + 1U) % CONTROL_RING_SIZE)) {
            struct relay_entry *queued = &context->control_ring[i];
            mesh_header_t *qh = (void *)queued->data;
            if (qh->type != type) continue;
            uint32_t revision = 0;
            if (type == MESH_PKT_MEMBERSHIP_V3) {
                const mesh_membership_v3_payload_t *old =
                    (const void *)(queued->data + sizeof(*qh));
                revision = old->revision;
            }
            memcpy(queued->data + sizeof(*qh), payload, len);
            if (type == MESH_PKT_MEMBERSHIP_V3) {
                mesh_membership_v3_payload_t *snapshot =
                    (void *)(queued->data + sizeof(*qh));
                snapshot->revision = revision;
            }
            qh->payload_len = len;
            queued->len = (uint8_t)(sizeof(*qh) + len);
            return 0;
        }
    }

    uint8_t next_head = (uint8_t)((context->control_head + 1) % CONTROL_RING_SIZE);
    if (next_head == context->control_tail) {
        context->stat_control_ring_drop++;
        return -ENOBUFS;
    }

    struct relay_entry *entry = &context->control_ring[context->control_head];
    mesh_header_t *hdr = (mesh_header_t *)entry->data;
    hdr->version = MESH_PROTOCOL_VERSION;
    hdr->type = type;
    hdr->src_id = context->node_id;
    hdr->seq = context->tx_seq++;
    hdr->ttl = 2;
    hdr->flags = 0;
    hdr->talk_channel = context->talk_channel;
    hdr->payload_len = len;
    if (payload != NULL && len > 0) {
        memcpy(entry->data + sizeof(*hdr), payload, len);
    }
    if (type == MESH_PKT_MEMBERSHIP_V3) {
        mesh_membership_v3_payload_t *snapshot =
            (void *)(entry->data + sizeof(*hdr));
        snapshot->revision = mesh_adaptive_next_membership_revision(&context->adaptive);
    }
    entry->len = (uint8_t)(sizeof(*hdr) + len);
    context->control_head = next_head;
    return 0;
}

int mesh_protocol_tx_queue_priority(mesh_pkt_type_t type, const void *payload, uint16_t len)
{
    mesh_protocol_context_t *c = mesh_protocol_context_get();
    if (len > MESH_PACKET_PAYLOAD_MAX || c->priority_pending) return -ENOBUFS;
    mesh_header_t h = {.version = MESH_PROTOCOL_VERSION, .type = type, .src_id = c->node_id,
        .seq = c->tx_seq++, .ttl = 2, .talk_channel = c->talk_channel, .payload_len = len};
    memcpy(c->priority_control.data, &h, sizeof(h));
    memcpy(c->priority_control.data + sizeof(h), payload, len);
    c->priority_control.len = (uint8_t)(sizeof(h) + len);
    c->priority_pending = true;
    return 0;
}

int mesh_protocol_tx_send_join_request(mesh_protocol_context_t *context)
{
    mesh_join_v3_payload_t payload = {.origin = mesh_protocol_local_identity(),
        .capabilities = MESH_CAP_LC3, .target = context->leader_identity};
    LOG_INF("Sending JOIN request");
    return mesh_protocol_tx_send_packet_ex(MESH_PKT_JOIN_V3, &payload, sizeof(payload),
                                            2, 0, 0, context->tx_seq++);
}

int mesh_protocol_tx_send_keepalive(mesh_protocol_context_t *context)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 255,
        .reserved = 0,
    };
    return mesh_protocol_tx_queue_control(context, MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
}

mesh_slot_map_payload_t mesh_protocol_tx_slot_snapshot(mesh_protocol_context_t *context)
{
    mesh_slot_map_payload_t payload = {0};
    uint8_t highest = 0;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (context->peers[i].active && context->peers[i].announced &&
            context->peers[i].slot_index >= 0 && context->peers[i].slot_index < MESH_MAX_NODES) {
            payload.slot_ids[context->peers[i].slot_index] = context->peers[i].node_id;
            highest = MAX(highest, (uint8_t)(context->peers[i].slot_index + 1));
        }
    }
    if (context->slot_index >= 0 && context->slot_index < MESH_MAX_NODES) {
        payload.slot_ids[context->slot_index] = context->node_id;
        highest = MAX(highest, (uint8_t)(context->slot_index + 1));
    }
    payload.slot_count = highest;

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (context->active_speaker_ids[i] != 0) {
            uint8_t n = payload.active_speaker_count;
            payload.active_speaker_ids[n] = context->active_speaker_ids[i];
            payload.relay_masks[n] = context->relay_masks[i];
            payload.active_speaker_count++;
        }
    }

    return payload;
}

int mesh_protocol_tx_send_slot_map(mesh_protocol_context_t *context)
{
    ARG_UNUSED(context);
    return mesh_protocol_adaptive_publish_membership();
}

int mesh_protocol_tx_send_status_packet(mesh_protocol_context_t *context)
{
    mesh_status_payload_t payload = {
        .battery_pct = 255,
        .rssi_dbm = 127,
        .peer_count = context->peer_count,
        .fw_version = MESH_PROTOCOL_VERSION,
        .temperature_c = 127,
        .heard_bitmap = context->heard_bitmap,
        .relay_bitmap = context->relay_bitmap,
        .active_speakers = 0,
    };

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (context->active_speaker_ids[i] != 0) {
            payload.active_speakers++;
        }
    }

    return mesh_protocol_tx_queue_control(context, MESH_PKT_STATUS, &payload, sizeof(payload));
}

int mesh_protocol_tx_send_join_ack(mesh_protocol_context_t *context, uint8_t assigned_id,
                                   uint8_t slot_index, const uint8_t target_addr[5])
{
    mesh_join_ack_v3_payload_t payload = {
        .target = {.address_len = 5}, .capabilities = MESH_CAP_LC3,
        .assigned_id = assigned_id,
        .slot_index = slot_index,
        .coordinator_id = context->node_id,
        .term = context->term,
    };
    memcpy(payload.target.address, target_addr, 5);
    LOG_INF("Sending JOIN_ACK: id=%d, slot=%d", assigned_id, slot_index);
    return mesh_protocol_tx_queue_priority(MESH_PKT_JOIN_ACK_V3, &payload, sizeof(payload));
}

void mesh_protocol_tx_control_handler(uint32_t frame_counter)
{
    mesh_protocol_context_t *context = mesh_protocol_context_get();
    mesh_protocol_adaptive_frame_tick(frame_counter);

    if ((frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0) {
        if (context->role == MESH_ROLE_COORDINATOR) {
            send_sync(context);
        }
        return;
    }

    if (context->state != MESH_STATE_ACTIVE || context->slot_index < 0 ||
        (frame_counter % MESH_MAX_NODES) != (uint32_t)context->slot_index) {
        return;
    }

    if (mesh_protocol_adaptive_converging() &&
        ((frame_counter / MESH_MAX_NODES) % 2U) == 0) {
        (void)send_sync(context);
        return;
    }

    if (context->priority_pending) {
        if (esb_radio_send(context->priority_control.data, context->priority_control.len) == 0)
            context->priority_pending = false;
        return;
    }

    /* One own packet for every forwarded packet, with forwarding ahead of periodic reports. */
    if (context->adaptive.queue_count &&
        (context->control_turn++ % 2 == 0 || context->control_tail == context->control_head)) {
        mesh_adaptive_t *a = &context->adaptive;
        uint8_t best = 0;
        int best_rank = 0;
        for (uint8_t i = 0; i < a->queue_count; i++) {
            uint8_t pos = (uint8_t)((a->queue_head + i) % MESH_ADAPTIVE_FORWARD_QUEUE);
            uint8_t type = a->queue[pos].header.type;
            int rank = (type == MESH_PKT_HANDOVER_ACK || type == MESH_PKT_HANDOVER_COMMIT ||
                        type == MESH_PKT_MEMBERSHIP_V3 || type == MESH_PKT_JOIN_ACK_V3) ? 4 :
                       (type == MESH_PKT_HANDOVER_PREPARE || type == MESH_PKT_JOIN_V3 ||
                        type == MESH_PKT_HANDOVER_CANCEL) ? 3 :
                       type == MESH_PKT_SPEAKER_REQUEST ? 2 :
                       type == MESH_PKT_TOPOLOGY ? 1 : 0;
            if (rank > best_rank) { best_rank = rank; best = i; }
        }
        if (best) {
            mesh_adaptive_forward_packet_t urgent =
                a->queue[(a->queue_head + best) % MESH_ADAPTIVE_FORWARD_QUEUE];
            for (uint8_t j = best; j > 0; j--)
                a->queue[(a->queue_head + j) % MESH_ADAPTIVE_FORWARD_QUEUE] =
                    a->queue[(a->queue_head + j - 1U) % MESH_ADAPTIVE_FORWARD_QUEUE];
            a->queue[a->queue_head] = urgent;
        }
        mesh_adaptive_forward_packet_t packet;
        uint32_t frame;
        uint16_t phase;
        if (tdma_clock_snapshot(&frame, &phase) &&
            mesh_adaptive_forward_dequeue(&context->adaptive, &packet, frame, phase,
                k_uptime_get_32() - context->last_sync_time)) {
            uint8_t raw[MESH_CONTROL_MAX_PACKET_SIZE];
            memcpy(raw, &packet.header, sizeof(packet.header));
            memcpy(raw + sizeof(packet.header), packet.payload, packet.payload_len);
            if (esb_radio_send(raw, sizeof(packet.header) + packet.payload_len) != 0)
                context->stat_tx_fail++;
        }
        return;
    }

    if (context->control_tail != context->control_head) {
        uint8_t best = context->control_tail;
        int best_rank = -1;
        for (uint8_t i = context->control_tail; i != context->control_head;
             i = (uint8_t)((i + 1U) % CONTROL_RING_SIZE)) {
            uint8_t type = ((const mesh_header_t *)context->control_ring[i].data)->type;
            int rank = type == MESH_PKT_MEMBERSHIP_V3 || type == MESH_PKT_SLOT_MAP ? 3 :
                       type == MESH_PKT_SPEAKER_REQUEST || type == MESH_PKT_SPEAKER_GRANT ||
                       type == MESH_PKT_SPEAKER_RELEASE ? 2 : 0;
            if (rank > best_rank) { best_rank = rank; best = i; }
        }
        if (best != context->control_tail) {
            struct relay_entry selected = context->control_ring[best];
            for (uint8_t i = best; i != context->control_tail;
                 i = (uint8_t)((i + CONTROL_RING_SIZE - 1U) % CONTROL_RING_SIZE))
                context->control_ring[i] =
                    context->control_ring[(i + CONTROL_RING_SIZE - 1U) % CONTROL_RING_SIZE];
            context->control_ring[context->control_tail] = selected;
        }
        struct relay_entry *entry = &context->control_ring[context->control_tail];
        mesh_header_t *hdr = (void *)entry->data;
        if (hdr->type == MESH_PKT_TOPOLOGY) {
            mesh_topology_payload_t *report = (void *)(entry->data + sizeof(*hdr));
            /* Assign only at the actual RF attempt, never on workqueue coalescing. */
            report->report_seq = context->air_report_seq + 1U;
        } else if (hdr->type == MESH_PKT_SPEAKER_REQUEST) {
            mesh_speaker_request_payload_t *request =
                (void *)(entry->data + sizeof(*hdr));
            request->frame_counter = tdma_get_frame_counter();
        }
        int ret = esb_radio_send(entry->data, entry->len);
        if (ret == 0) {
            if (hdr->type == MESH_PKT_TOPOLOGY) {
                context->air_report_seq++;
                context->stat_topology_tx_ok++;
            }
            context->control_tail = (uint8_t)((context->control_tail + 1) % CONTROL_RING_SIZE);
        } else {
            context->stat_tx_fail++;
            /* Retry gets a new dedupe key; its topology RF sequence is reused. */
            hdr->seq = context->tx_seq++;
        }
    }
}
