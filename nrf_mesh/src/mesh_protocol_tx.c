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
    mesh_sync_payload_t payload = {
        .frame_counter = tdma_get_frame_counter(),
        /* WS phase slew is not a persistent frequency estimate. */
        .drift_ppm = 0,
    };
    memcpy(payload.coordinator_addr, context->local_addr, 5);
    return mesh_protocol_tx_send_packet(context, MESH_PKT_SYNC, &payload, sizeof(payload));
}

int mesh_protocol_tx_send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len,
                                    uint8_t ttl, uint8_t flags, uint8_t src_id, uint8_t seq)
{
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
    if (len > MESH_PACKET_PAYLOAD_MAX) {
        return -EMSGSIZE;
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
    hdr->ttl = 0;
    hdr->flags = 0;
    hdr->payload_len = len;
    if (payload != NULL && len > 0) {
        memcpy(entry->data + sizeof(*hdr), payload, len);
    }
    entry->len = (uint8_t)(sizeof(*hdr) + len);
    context->control_head = next_head;
    return 0;
}

int mesh_protocol_tx_send_join_request(mesh_protocol_context_t *context)
{
    mesh_join_v2_payload_t payload = {
        .capabilities = 0x01, /* Has audio */
        .reserved = 0,
    };
    memcpy(payload.requester_addr, context->local_addr, sizeof(payload.requester_addr));
    LOG_INF("Sending JOIN request");
    return mesh_protocol_tx_send_packet(context, MESH_PKT_JOIN_V2, &payload, sizeof(payload));
}

int mesh_protocol_tx_send_keepalive(mesh_protocol_context_t *context)
{
    mesh_keepalive_payload_t payload = {
        .battery_pct = 255,
        .reserved = 0,
    };
    return mesh_protocol_tx_queue_control(context, MESH_PKT_KEEPALIVE, &payload, sizeof(payload));
}

int mesh_protocol_tx_send_slot_map(mesh_protocol_context_t *context)
{
    mesh_slot_map_payload_t payload = {0};
    payload.slot_count = MESH_MAX_NODES;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (context->peers[i].active && context->peers[i].announced &&
            context->peers[i].slot_index >= 0 && context->peers[i].slot_index < MESH_MAX_NODES) {
            payload.slot_ids[context->peers[i].slot_index] = context->peers[i].node_id;
        }
    }

    for (int i = 0; i < MESH_MAX_ACTIVE_SPEAKERS; i++) {
        if (context->active_speaker_ids[i] != 0) {
            payload.active_speaker_count++;
        }
    }
    memcpy(payload.active_speaker_ids, context->active_speaker_ids,
           sizeof(payload.active_speaker_ids));
    memcpy(payload.relay_masks, context->relay_masks, sizeof(payload.relay_masks));

    return mesh_protocol_tx_queue_control(context, MESH_PKT_SLOT_MAP, &payload, sizeof(payload));
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
    mesh_join_ack_v2_payload_t payload = {
        .assigned_id = assigned_id,
        .slot_index = slot_index,
        .coordinator_id = context->node_id,
    };
    memcpy(payload.target_addr, target_addr, sizeof(payload.target_addr));
    LOG_INF("Sending JOIN_ACK: id=%d, slot=%d", assigned_id, slot_index);
    return mesh_protocol_tx_queue_control(context, MESH_PKT_JOIN_ACK_V2, &payload, sizeof(payload));
}

void mesh_protocol_tx_control_handler(uint32_t frame_counter)
{
    mesh_protocol_context_t *context = mesh_protocol_context_get();

    if ((frame_counter % MESH_SYNC_INTERVAL_FRAMES) == 0) {
        if (context->role == MESH_ROLE_COORDINATOR) {
            send_sync(context);
        }
        return;
    }

    if ((frame_counter % MESH_MAX_NODES) != (uint32_t)context->slot_index) {
        return;
    }

    if (context->control_tail != context->control_head) {
        struct relay_entry *entry = &context->control_ring[context->control_tail];
        int ret = esb_radio_send(entry->data, entry->len);
        if (ret == 0) {
            context->control_tail = (uint8_t)((context->control_tail + 1) % CONTROL_RING_SIZE);
        } else {
            context->stat_tx_fail++;
        }
    }
}
