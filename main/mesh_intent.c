/**
 * @file mesh_intent.c
 * @brief Persisted user intent for mesh networking (NVS-backed).
 */

#include "mesh_intent.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "omi";

#define MESH_NVS_NAMESPACE "omi"
#define MESH_NVS_ENABLED_KEY "mesh_enabled"

static _Atomic bool s_enabled = false;

esp_err_t mesh_intent_load(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MESH_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        s_enabled = false;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t enabled = 0;
    ret = nvs_get_u8(handle, MESH_NVS_ENABLED_KEY, &enabled);
    nvs_close(handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        s_enabled = false;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }
    if (enabled > 1) {
        ESP_LOGW(TAG, "Invalid persisted mesh intent %u; defaulting to disabled", enabled);
        enabled = 0;
    }
    s_enabled = enabled != 0;
    return ESP_OK;
}

esp_err_t mesh_intent_persist(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MESH_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_u8(handle, MESH_NVS_ENABLED_KEY, enabled ? 1 : 0);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

void mesh_intent_set(bool enabled)
{
    atomic_store(&s_enabled, enabled);
}

bool mesh_intent_enabled(void)
{
    return atomic_load(&s_enabled);
}
