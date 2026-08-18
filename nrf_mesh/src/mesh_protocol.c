/**
 * @file mesh_protocol.c
 * @brief Mesh protocol facade: shared context singleton and public accessors
 *
 * Module ownership: mesh_protocol_rx.c (ESB receive handoff),
 * mesh_protocol_membership.c (join/election/peers), mesh_protocol_audio.c
 * (audio ingress/relay/slot TX), mesh_protocol_tx.c (control/packet egress),
 * mesh_protocol_runtime.c (lifecycle, command coalescing, status work).
 *
 * NOTE: The system workqueue owns all mesh protocol state. SPI commands are
 * coalesced atomically, audio is copied through a message queue, and timer or
 * radio ISR paths only submit work or publish into protected rings.
 */

#include "mesh_protocol.h"
#include "mesh_protocol_internal.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

/* Protocol v2 is a deliberate fail-closed RF migration: v1 peers are rejected. */
_Static_assert(MESH_PACKET_PAYLOAD_MAX == 200, "mesh payload capacity changed");
_Static_assert(MESH_PACKET_OUTER_MAX == 208, "mesh packet capacity changed");
_Static_assert(MESH_PACKET_OUTER_MAX <= UINT8_MAX, "ESB packet length no longer fits uint8_t");

static mesh_protocol_context_t s_context = {
    .state = MESH_STATE_IDLE,
    .role = MESH_ROLE_NONE,
    .slot_index = -1,
};

mesh_protocol_context_t *mesh_protocol_context_get(void)
{
    return &s_context;
}

mesh_state_t mesh_protocol_get_state(void)
{
    return s_context.state;
}

mesh_role_t mesh_protocol_get_role(void)
{
    return s_context.role;
}

uint8_t mesh_protocol_get_node_id(void)
{
    return s_context.node_id;
}

uint8_t mesh_protocol_get_peer_count(void)
{
    return s_context.peer_count;
}
