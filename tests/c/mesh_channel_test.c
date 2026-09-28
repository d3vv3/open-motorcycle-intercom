#include "shared/bridge_protocol_defs.h"
#include "shared/mesh_protocol_defs.h"
#include "main/transport_nrf_reconcile.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_rf_ingress(void)
{
    assert(MESH_CHANNEL_DEFAULT == 1);
    assert(sizeof(mesh_header_t) == 9);
    assert(offsetof(mesh_header_t, talk_channel) == 6);

    const uint8_t types[] = {
        MESH_PKT_AUDIO, MESH_PKT_AUDIO_V2, MESH_PKT_JOIN, MESH_PKT_JOIN_V2,
        MESH_PKT_JOIN_ACK, MESH_PKT_JOIN_ACK_V2, MESH_PKT_LEAVE, MESH_PKT_SYNC,
        MESH_PKT_SLOT_MAP, MESH_PKT_STATUS, MESH_PKT_KEEPALIVE,
        MESH_PKT_SPEAKER_GRANT, MESH_PKT_SPEAKER_RELEASE,
    };
    for (size_t i = 0; i < sizeof(types); ++i) {
        for (uint8_t local = 1; local <= MESH_CHANNEL_COUNT; ++local) {
            mesh_header_t header = {
                .version = MESH_PROTOCOL_VERSION,
                .type = types[i],
                .talk_channel = local,
            };
            assert(mesh_header_accepts_channel(&header, local));
            for (uint8_t other = 1; other <= MESH_CHANNEL_COUNT; ++other) {
                if (other != local) {
                    header.talk_channel = other;
                    assert(!mesh_header_accepts_channel(&header, local));
                }
            }
            header.talk_channel = 0;
            assert(!mesh_header_accepts_channel(&header, local));
            header.talk_channel = 4;
            assert(!mesh_header_accepts_channel(&header, local));
            header.talk_channel = local;
            header.version = 3;
            assert(!mesh_header_accepts_channel(&header, local));
        }
    }
    mesh_header_t header = {.version = MESH_PROTOCOL_VERSION, .talk_channel = 1};
    assert(!mesh_header_accepts_channel(&header, 0));
    assert(!mesh_header_accepts_channel(&header, 4));
    assert(!mesh_header_accepts_channel(NULL, 1));
}

static void test_bridge_commands(void)
{
    assert(sizeof(bridge_command_payload_t) == 3);
    assert(!bridge_audio_supports_lc3(BRIDGE_PROTOCOL_VERSION_V3,
                                      MESH_AUDIO_V2_CODEC_LC3, MESH_FRAME_MS));
    for (uint8_t group = 1; group <= MESH_CHANNEL_COUNT; ++group) {
        bridge_command_payload_t start = {
            .command = BRIDGE_COMMAND_MESH_START, .generation = 73, .talk_channel = group,
        };
        uint8_t wire[3];
        memcpy(wire, &start, sizeof(wire));
        assert(wire[0] == BRIDGE_COMMAND_MESH_START && wire[1] == 73 && wire[2] == group);
        assert(bridge_mesh_command_valid(&start, sizeof(start)));
        start.talk_channel = 0;
        assert(!bridge_mesh_command_valid(&start, sizeof(start)));
        start.talk_channel = 4;
        assert(!bridge_mesh_command_valid(&start, sizeof(start)));
    }
    bridge_command_payload_t stop = {.command = BRIDGE_COMMAND_MESH_STOP, .talk_channel = 0};
    assert(bridge_mesh_command_valid(&stop, sizeof(stop)));
    assert(!bridge_mesh_command_valid(&stop, 2));
    assert(!bridge_mesh_command_valid(&stop, 4));
    assert(!bridge_mesh_command_valid(NULL, sizeof(stop)));
    stop.command = BRIDGE_COMMAND_STATUS;
    assert(!bridge_mesh_command_valid(&stop, sizeof(stop)));

    uint32_t first = bridge_mesh_request_pack(BRIDGE_COMMAND_MESH_START, 255, 2);
    bridge_command_payload_t pending = bridge_mesh_request_unpack(first);
    assert(pending.command == BRIDGE_COMMAND_MESH_START);
    assert(pending.generation == 255 && pending.talk_channel == 2);
    pending = bridge_mesh_request_unpack(bridge_mesh_request_pack(BRIDGE_COMMAND_MESH_STOP, 0, 0));
    assert(pending.command == BRIDGE_COMMAND_MESH_STOP);
    assert(pending.generation == 0 && pending.talk_channel == 0);
}

static void test_rf_mapping_and_recovery(void)
{
    assert(mesh_channel_espnow_rf(1) == 1 && mesh_channel_esb_rf(1) == 40);
    assert(mesh_channel_espnow_rf(2) == 6 && mesh_channel_esb_rf(2) == 20);
    assert(mesh_channel_espnow_rf(3) == 11 && mesh_channel_esb_rf(3) == 60);
    assert(mesh_channel_espnow_rf(0) == 0 && mesh_channel_esb_rf(0) == 0);
    assert(mesh_channel_espnow_rf(4) == 0 && mesh_channel_esb_rf(4) == 0);

    assert(nrf_mesh_reconcile_action(true, false, BRIDGE_MESH_STATE_ACTIVE) == NRF_MESH_STOP);
    assert(nrf_mesh_reconcile_action(true, false, BRIDGE_MESH_STATE_SCANNING) == NRF_MESH_STOP);
    assert(nrf_mesh_reconcile_action(true, false, BRIDGE_MESH_STATE_IDLE) == NRF_MESH_START);
    assert(nrf_mesh_reconcile_action(true, true, BRIDGE_MESH_STATE_ACTIVE) == NRF_MESH_NO_COMMAND);
    assert(nrf_mesh_reconcile_action(true, true, BRIDGE_MESH_STATE_SCANNING) == NRF_MESH_NO_COMMAND);
    assert(nrf_mesh_reconcile_action(false, true, BRIDGE_MESH_STATE_ACTIVE) == NRF_MESH_STOP);
    assert(nrf_mesh_reconcile_action(false, false, BRIDGE_MESH_STATE_IDLE) == NRF_MESH_NO_COMMAND);
    assert(nrf_mesh_reconcile_interval_ms(0) == 2000);
    assert(nrf_mesh_reconcile_interval_ms(2) == 2000);
    assert(nrf_mesh_reconcile_interval_ms(3) == 10000);
    assert(nrf_mesh_reconcile_interval_ms(UINT8_MAX) == 10000);
}

int main(void)
{
    test_rf_ingress();
    test_bridge_commands();
    test_rf_mapping_and_recovery();
    puts("mesh channel tests passed");
    return 0;
}
