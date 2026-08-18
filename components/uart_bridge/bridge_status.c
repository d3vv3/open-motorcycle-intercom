/**
 * @file bridge_status.c
 * @brief Registry of the last nRF mesh status and its freshness.
 *
 * The nRF reports mesh status with every poll cycle. A status older than
 * BRIDGE_STATUS_STALE_TIMEOUT_US no longer counts as a connection; readers
 * then see the bridge as disconnected and queued audio gets discarded.
 */

#include "bridge_internal.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "spi_bridge";

bool bridge_status_is_fresh_locked(int64_t now_us)
{
    return g_bridge.connected && g_bridge.status.received_at_us > 0 &&
           now_us >= g_bridge.status.received_at_us &&
           now_us - g_bridge.status.received_at_us <= BRIDGE_STATUS_STALE_TIMEOUT_US;
}

static void update_status_age_locked(int64_t now_us)
{
    if (g_bridge.status.received_at_us <= 0 || now_us < g_bridge.status.received_at_us) {
        g_bridge.status_age_current_ms = 0;
        return;
    }

    int64_t age_ms = (now_us - g_bridge.status.received_at_us) / 1000;
    g_bridge.status_age_current_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
    if (g_bridge.status_age_current_ms > g_bridge.status_age_max_ms) {
        g_bridge.status_age_max_ms = g_bridge.status_age_current_ms;
    }
}

void bridge_status_log_telemetry(int64_t now_us)
{
    static int64_t last_log_us = 0;
    if (now_us - last_log_us <= 5000000) {
        return;
    }

    uint32_t valid_rx;
    uint32_t expiration_count;
    uint32_t age_ms;
    uint32_t max_age_ms;
    uint32_t generation;
    uint8_t state;
    uint32_t expired_generation;
    uint8_t expired_state;
    uint32_t gate_stale;
    uint32_t gate_inactive;
    uint32_t gate_disconnected;
    uint32_t gate_invalid_node;

    portENTER_CRITICAL(&g_bridge_status_lock);
    update_status_age_locked(now_us);
    valid_rx = g_bridge.status_rx_valid;
    expiration_count = g_bridge.status_expiration_count;
    age_ms = g_bridge.status_age_current_ms;
    max_age_ms = g_bridge.status_age_max_ms;
    generation = g_bridge.status.generation;
    state = g_bridge.status.mesh_state;
    expired_generation = g_bridge.status_expired_generation;
    expired_state = g_bridge.status_expired_state;
    gate_stale = g_bridge.gate_stale;
    gate_inactive = g_bridge.gate_inactive;
    gate_disconnected = g_bridge.gate_disconnected;
    gate_invalid_node = g_bridge.gate_invalid_node;
    portEXIT_CRITICAL(&g_bridge_status_lock);

    ESP_LOGI(TAG,
             "PIPE v=1 dev=esp stage=bridge_status valid_rx=%lu expire=%lu age_ms=%lu max_age_ms=%lu gen=%lu state=%u exp_gen=%lu exp_state=%u gate_stale=%lu gate_inactive=%lu gate_disconnected=%lu gate_invalid_node=%lu",
             valid_rx, expiration_count, age_ms, max_age_ms, generation, state,
             expired_generation, expired_state, gate_stale, gate_inactive, gate_disconnected,
             gate_invalid_node);
    last_log_us = now_us;
}

/**
 * @brief Store a validated status and notify the status callback.
 *
 * @return true when a change-relevant field differs from the previous
 *         status (only evaluated for v2 statuses).
 */
static bool commit_status(const uart_bridge_status_t *incoming, bool detect_change)
{
    int64_t received_at_us = esp_timer_get_time();

    portENTER_CRITICAL(&g_bridge_status_lock);
    bool continuity_lost = g_bridge.status.received_at_us > 0 &&
                           received_at_us >= g_bridge.status.received_at_us &&
                           received_at_us - g_bridge.status.received_at_us >
                               BRIDGE_STATUS_STALE_TIMEOUT_US;
    if (continuity_lost && !g_bridge.status_expired) {
        update_status_age_locked(received_at_us);
        g_bridge.status_expiration_count++;
        g_bridge.status_expired_generation = g_bridge.status.generation;
        g_bridge.status_expired_state = g_bridge.status.mesh_state;
    }

    bool changed = detect_change &&
                   ((!g_bridge.connected) ||
                    (g_bridge.status.mesh_state != incoming->mesh_state) ||
                    (g_bridge.status.role != incoming->role) ||
                    (g_bridge.status.peer_count != incoming->peer_count) ||
                    (g_bridge.status.node_id != incoming->node_id));

    uint32_t generation = g_bridge.status.generation + 1;
    g_bridge.status = *incoming;
    g_bridge.status.generation = generation;
    g_bridge.status.received_at_us = received_at_us;
    g_bridge.status.continuity_lost = continuity_lost;
    g_bridge.connected = true;
    g_bridge.status_expired = false;
    g_bridge.status_rx_valid++;
    g_bridge.status_age_current_ms = 0;
    uart_bridge_status_t published_status = g_bridge.status;
    portEXIT_CRITICAL(&g_bridge_status_lock);

    if (g_bridge.status_cb) {
        g_bridge.status_cb(&published_status);
    }
    return changed;
}

void bridge_status_apply_v2(const bridge_status_payload_t *payload)
{
    if (payload->version != BRIDGE_PROTOCOL_VERSION ||
        payload->marker != BRIDGE_STATUS_V2_MARKER ||
        payload->mesh_state > BRIDGE_MESH_STATE_ACTIVE) {
        ESP_LOGW(TAG, "Ignoring invalid bridge v2 status (version=%u marker=0x%02X)",
                 payload->version, payload->marker);
        return;
    }

    uart_bridge_status_t incoming = {
        .mesh_state = payload->mesh_state,
        .role = payload->role,
        .node_id = payload->node_id,
        .slot_index = (uint8_t)payload->slot_index,
        .coordinator_id = payload->coordinator_id,
        .peer_count = payload->peer_count,
        .is_coordinator = payload->role == 1,
        .has_mesh_state = true,
        .protocol_version = payload->version,
    };

    bool changed = commit_status(&incoming, true);
    if (changed) {
        ESP_LOGI(TAG, "Status: state=%u role=%u node=%u slot=%d coord=%u peers=%u",
                 payload->mesh_state, payload->role, payload->node_id, payload->slot_index,
                 payload->coordinator_id, payload->peer_count);
    }
    if (payload->mesh_state != BRIDGE_MESH_STATE_ACTIVE || payload->node_id == 0) {
        bridge_tx_discard_pending_audio();
    }
}

/* Legacy 3-byte status: [role] [peer_count] [node_id], no mesh state field. */
void bridge_status_apply_legacy(const uint8_t payload[3])
{
    uart_bridge_status_t incoming = {
        .mesh_state = payload[0] != 0 && payload[2] != 0 ? BRIDGE_MESH_STATE_ACTIVE
                                                         : BRIDGE_MESH_STATE_IDLE,
        .role = payload[0],
        .node_id = payload[2],
        .slot_index = UINT8_MAX,
        .peer_count = payload[1],
        .is_coordinator = payload[0] == 1,
        .has_mesh_state = false,
        .protocol_version = 1,
    };

    commit_status(&incoming, false);

    if (payload[0] == 0 || payload[2] == 0) {
        bridge_tx_discard_pending_audio();
    }
}

esp_err_t uart_bridge_get_status(uart_bridge_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool expired = false;
    uint32_t expired_generation = 0;
    uint8_t expired_state = BRIDGE_MESH_STATE_IDLE;
    uint32_t expired_age_ms = 0;
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&g_bridge_status_lock);
    update_status_age_locked(now_us);
    if (!bridge_status_is_fresh_locked(now_us)) {
        expired = g_bridge.connected;
        expired_generation = g_bridge.status.generation;
        expired_state = g_bridge.status.mesh_state;
        expired_age_ms = g_bridge.status_age_current_ms;
        if (expired) {
            g_bridge.status_expiration_count++;
            g_bridge.status_expired_generation = expired_generation;
            g_bridge.status_expired_state = expired_state;
            g_bridge.status_expired = true;
        }
        g_bridge.connected = false;
        portEXIT_CRITICAL(&g_bridge_status_lock);
        if (expired) {
            ESP_LOGW(TAG, "Status expired: age_ms=%lu generation=%lu state=%u", expired_age_ms,
                     expired_generation, expired_state);
            bridge_tx_discard_for_expired_status(expired_generation);
        }
        bridge_status_log_telemetry(now_us);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(status, &g_bridge.status, sizeof(uart_bridge_status_t));
    portEXIT_CRITICAL(&g_bridge_status_lock);
    bridge_status_log_telemetry(now_us);
    return ESP_OK;
}

bool uart_bridge_is_connected(void)
{
    uart_bridge_status_t status;
    return uart_bridge_get_status(&status) == ESP_OK;
}

bool uart_bridge_is_mesh_ready(void)
{
    uart_bridge_status_t status;
    esp_err_t err = uart_bridge_get_status(&status);

    portENTER_CRITICAL(&g_bridge_status_lock);
    if (err != ESP_OK) {
        if (g_bridge.status_expired) {
            g_bridge.gate_stale++;
        } else {
            g_bridge.gate_disconnected++;
        }
    } else if (status.mesh_state != BRIDGE_MESH_STATE_ACTIVE) {
        g_bridge.gate_inactive++;
    } else if (status.node_id == 0) {
        g_bridge.gate_invalid_node++;
    }
    portEXIT_CRITICAL(&g_bridge_status_lock);

    return err == ESP_OK && status.mesh_state == BRIDGE_MESH_STATE_ACTIVE && status.node_id != 0;
}
