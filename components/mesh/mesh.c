/**
 * @file mesh.c
 * @brief OMI Mesh Networking Implementation (Stub)
 *
 * Phase 0: Stub implementation for build validation
 * Phase 2: ESP-NOW based TDMA mesh
 */

#include "mesh.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"

static const char *TAG = "mesh";

static bool s_initialized = false;
static mesh_config_t s_config = MESH_CONFIG_DEFAULT();
static mesh_stats_t s_stats = {0};
static mesh_role_t s_role = MESH_ROLE_NONE;
static int8_t s_slot = -1;
static mesh_audio_cb_t s_audio_cb = NULL;

static TaskHandle_t s_tx_task = NULL;
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_config.channel, WIFI_SECOND_CHAN_NONE));
}

static void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    s_stats.packets_rx++;

    // Log the received packet
    ESP_LOGI(TAG, "RX from " MACSTR " len=%d: %.*s", MAC2STR(esp_now_info->src_addr), len, len,
             (char *)data);
}

static void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_stats.packets_tx++;
    } else {
        s_stats.packets_dropped++;
    }
}

static void tx_task(void *arg)
{
    char payload[32];
    uint32_t count = 0;

    while (1) {
        snprintf(payload, sizeof(payload), "OMI-PING-%" PRIu32, count++);

        esp_err_t ret = esp_now_send(s_broadcast_mac, (uint8_t *)payload, strlen(payload));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Send failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t mesh_init(void)
{
    return mesh_init_with_config(NULL);
}

esp_err_t mesh_init_with_config(const mesh_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config != NULL) {
        s_config = *config;
    }

    ESP_LOGI(TAG, "Initializing mesh subsystem (Phase 2 Preview)");

    // 1. Initialize WiFi
    wifi_init();

    // 2. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));

    // 3. Add Broadcast Peer
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = s_config.channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // 4. Start TX Task
    xTaskCreate(tx_task, "mesh_tx", 2048, NULL, 4, &s_tx_task);

    s_initialized = true;
    return ESP_OK;
}

esp_err_t mesh_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing mesh subsystem");

    // TODO: Cleanup ESP-NOW and WiFi

    s_role = MESH_ROLE_NONE;
    s_slot = -1;
    s_initialized = false;
    return ESP_OK;
}

esp_err_t mesh_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting mesh networking");

    // TODO: Start TDMA scheduler
    // TODO: Begin join process or become master

    s_role = MESH_ROLE_MASTER; // Stub: assume master for now
    s_slot = 0;

    return ESP_OK;
}

esp_err_t mesh_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping mesh networking");

    // TODO: Send LEAVE packet
    // TODO: Stop TDMA scheduler

    s_role = MESH_ROLE_NONE;
    s_slot = -1;

    return ESP_OK;
}

mesh_role_t mesh_get_role(void)
{
    return s_role;
}

int8_t mesh_get_slot(void)
{
    return s_slot;
}

uint8_t mesh_get_node_count(void)
{
    // TODO: Return actual connected node count
    return (s_role != MESH_ROLE_NONE) ? 1 : 0;
}

esp_err_t mesh_send_audio(const uint8_t *data, uint16_t len)
{
    if (!s_initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_role == MESH_ROLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    // TODO: Queue frame for next TDMA slot
    s_stats.packets_tx++;

    return ESP_OK;
}

esp_err_t mesh_register_audio_callback(mesh_audio_cb_t cb)
{
    s_audio_cb = cb;
    return ESP_OK;
}

esp_err_t mesh_get_stats(mesh_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *stats = s_stats;
    return ESP_OK;
}
