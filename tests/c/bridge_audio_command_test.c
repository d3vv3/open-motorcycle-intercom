#include "shared/bridge_protocol_defs.h"
#include "nrf_mesh/include/mesh_audio_delivery.h"

#include <assert.h>
#include <stdio.h>

static void test_commands(void)
{
    bridge_command_payload_t command = {BRIDGE_COMMAND_MESH_START, 21, MESH_CHANNEL_DEFAULT};
    assert(bridge_mesh_command_valid(&command, sizeof(command)));
    command.talk_channel = 0;
    assert(!bridge_mesh_command_valid(&command, sizeof(command)));
    command.command = BRIDGE_COMMAND_MESH_STOP;
    assert(bridge_mesh_command_valid(&command, sizeof(command)));

    const uint8_t audio_commands[] = {BRIDGE_COMMAND_AUDIO_PAUSE, BRIDGE_COMMAND_AUDIO_RESUME};
    for (size_t i = 0; i < sizeof(audio_commands); i++) {
        command.command = audio_commands[i];
        command.generation = 255;
        command.talk_channel = 255; /* ignored, but still required on the wire */
        assert(bridge_mesh_command_valid(&command, sizeof(command)));
        assert(!bridge_mesh_command_valid(&command, sizeof(command) - 1));
        assert(!bridge_mesh_command_valid(&command, sizeof(command) + 1));
        bridge_command_payload_t decoded = bridge_mesh_request_unpack(
            bridge_mesh_request_pack(command.command, command.generation, command.talk_channel));
        assert(decoded.command == command.command && decoded.generation == command.generation);
        assert(decoded.talk_channel == command.talk_channel);
    }
    command.command = BRIDGE_COMMAND_STATUS;
    assert(!bridge_mesh_command_valid(&command, sizeof(command)));
    command.command = 0xff;
    assert(!bridge_mesh_command_valid(&command, sizeof(command)));
    assert(!bridge_mesh_command_valid(NULL, sizeof(command)));
}

static void test_local_audio_boundary(void)
{
    uint32_t epoch = 0;
    int enabled = 1;
    uint32_t older_arrival = epoch;
    assert(bridge_audio_epoch_accept(older_arrival, epoch, enabled));

    /* PAUSE first closes admission, then invalidates waiting producers. */
    enabled = 0;
    assert(!bridge_audio_epoch_accept(older_arrival, epoch, enabled));
    epoch++;
    assert(!bridge_audio_epoch_accept(older_arrival, epoch, enabled));
    /* Duplicate PAUSE cannot reopen admission. */
    epoch++;
    assert(!bridge_audio_epoch_accept(older_arrival, epoch, enabled));

    /* RESUME flushes again before reopening; a pre-pause waiting frame is stale. */
    epoch++;
    enabled = 1;
    assert(!bridge_audio_epoch_accept(older_arrival, epoch, enabled));
    assert(bridge_audio_epoch_accept(epoch, epoch, enabled));
    uint32_t before_duplicate_resume = epoch;
    epoch++;
    assert(!bridge_audio_epoch_accept(before_duplicate_resume, epoch, enabled));
    assert(bridge_audio_epoch_accept(epoch, epoch, enabled));
}

static void test_rf_receipt_cutoff(void)
{
    /* The stale frame stays in the RF RX ring until after resume, but its
     * ISR timestamp precedes the new playback boundary. */
    const int64_t stale_received_us = 1000;
    const int64_t resume_cutoff_us = 3000;
    assert(mesh_audio_delivery_allowed(true, stale_received_us, 0));
    assert(!mesh_audio_delivery_allowed(false, 2000, 2000));
    assert(!mesh_audio_delivery_allowed(true, stale_received_us, resume_cutoff_us));
    assert(mesh_audio_delivery_allowed(true, resume_cutoff_us, resume_cutoff_us));
    assert(mesh_audio_delivery_allowed(true, 3001, resume_cutoff_us));
}

int main(void)
{
    test_commands();
    test_local_audio_boundary();
    test_rf_receipt_cutoff();
    puts("bridge_audio_command_test: PASS");
    return 0;
}
