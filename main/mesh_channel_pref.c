#include "mesh_channel_pref.h"

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "omi";
static uint8_t s_selected = MESH_CHANNEL_DEFAULT;

esp_err_t mesh_channel_pref_load(uint8_t fallback)
{
    s_selected = mesh_channel_restore(0, fallback);
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("omi", NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (ret != ESP_OK) return ret;
    uint8_t stored = 0;
    ret = nvs_get_u8(handle, "mesh_channel", &stored);
    nvs_close(handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (ret != ESP_OK) return ret;
    if (!mesh_channel_valid(stored)) {
        ESP_LOGW(TAG, "Invalid stored talk channel %u; using fallback %u", stored, s_selected);
        return ESP_OK;
    }
    s_selected = stored;
    return ESP_OK;
}

esp_err_t mesh_channel_pref_persist(uint8_t channel)
{
    if (!mesh_channel_valid(channel)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("omi", NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_u8(handle, "mesh_channel", channel);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret == ESP_OK) s_selected = channel;
    return ret;
}

uint8_t mesh_channel_pref_selected(void)
{
    return s_selected;
}
