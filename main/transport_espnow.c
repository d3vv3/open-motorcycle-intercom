/**
 * @file transport_espnow.c
 * @brief ESP-NOW transport glue (fallback when no nRF52840 is detected).
 */

#include "transport_espnow.h"

#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "audio.h"
#include "mesh.h"
#include "power.h"

static const char *TAG = "omi";

void transport_espnow_send_audio(const uint8_t *data, uint16_t len, bool active)
{
    if (mesh_get_state() != MESH_STATE_ACTIVE) {
        return;
    }

    uint8_t audio_flags = active ? MESH_AUDIO_FLAG_ACTIVE : 0;
    esp_err_t ret = mesh_send_audio(data, len, audio_flags);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to queue audio for TX: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback from mesh subsystem when audio frame is received
 */
static void mesh_audio_callback(const uint8_t *data, uint16_t len, uint8_t src_id,
                                uint8_t audio_flags, int64_t timestamp_us)
{
    audio_frame_t frame;

    if (len > sizeof(frame.data)) {
        ESP_LOGW(TAG, "Audio frame too large: %u bytes", len);
        return;
    }

    memcpy(frame.data, data, len);
    frame.len = len;
    frame.timestamp_ms = timestamp_us / 1000;
    frame.active = (audio_flags & MESH_AUDIO_FLAG_ACTIVE) != 0;
    /* ESP-NOW carries no end-to-end audio sequence, so playout cannot detect holes. */
    frame.seq = 0;
    frame.has_seq = false;

    esp_err_t ret = audio_put_rx_frame(&frame, src_id);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to queue RX audio: %s", esp_err_to_name(ret));
    }
}

static void mesh_state_callback(mesh_state_t old_state, mesh_state_t new_state)
{
    const char *state_names[] = {"IDLE", "SCANNING", "JOINING", "ACTIVE"};

    ESP_LOGI(TAG, "Mesh state: %s -> %s", state_names[old_state], state_names[new_state]);

    if (new_state == MESH_STATE_ACTIVE) {
        mesh_role_t role = mesh_get_role();
        uint8_t node_id = mesh_get_node_id();
        int8_t slot = mesh_get_slot();

        power_set_state(POWER_STATE_MESH_IDLE);

        ESP_LOGI(TAG, "=== Mesh Active ===");
        ESP_LOGI(TAG, "  Role: %s", role == MESH_ROLE_COORDINATOR ? "COORDINATOR" : "PARTICIPANT");
        ESP_LOGI(TAG, "  Node ID: %u", node_id);
        ESP_LOGI(TAG, "  Slot: %d", slot);
        ESP_LOGI(TAG, "");
    }
}

static void mesh_peer_callback(const mesh_peer_info_t *peer, bool joined)
{
    if (joined) {
        ESP_LOGI(TAG, "Peer JOINED: node_id=%u, slot=%d, MAC=" MACSTR, peer->node_id,
                 peer->slot_index, MAC2STR(peer->mac_addr));
        (void)audio_play_notification(AUDIO_NOTIFY_PEER_JOIN);
    } else {
        ESP_LOGI(TAG, "Peer LEFT: node_id=%u", peer->node_id);
        (void)audio_play_notification(AUDIO_NOTIFY_PEER_LEAVE);
    }
}

esp_err_t transport_espnow_init(void)
{
    esp_err_t ret = mesh_init();
    if (ret != ESP_OK) {
        return ret;
    }

    mesh_register_audio_callback(mesh_audio_callback);
    mesh_register_state_callback(mesh_state_callback);
    mesh_register_peer_callback(mesh_peer_callback);
    return ESP_OK;
}
