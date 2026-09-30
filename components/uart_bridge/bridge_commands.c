/**
 * @file bridge_commands.c
 * @brief Probe and mesh command exchange with the nRF.
 *
 * Commands are generation-tagged and retried until the nRF acknowledges
 * application. Status alone cannot attest which talk group was selected.
 */

#include "esp_log.h"
#include "esp_timer.h"

#include "bridge_internal.h"

static const char *TAG = "spi_bridge";

void bridge_commands_note_ack(const bridge_command_ack_payload_t *ack)
{
    g_bridge.command_ack_command = ack->command;
    g_bridge.command_ack_generation = ack->generation;
    g_bridge.command_ack_result = ack->result;
    __atomic_store_n(&g_bridge.command_ack_received, true, __ATOMIC_RELEASE);
}

bool uart_bridge_probe(uint32_t timeout_ms)
{
    if (!g_bridge.initialized) {
        return false;
    }

    const int MAX_PROBE_ATTEMPTS = 3;
    uart_bridge_status_t initial_status;
    uint32_t initial_generation =
        uart_bridge_get_status(&initial_status) == ESP_OK ? initial_status.generation : 0;

    for (int attempt = 1; attempt <= MAX_PROBE_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing for nRF52840 (attempt %d/%d)...", attempt, MAX_PROBE_ATTEMPTS);

        /* Bare one-byte status request (legacy ping form, no generation).
         * The master picks it up on its next poll and answers with STATUS. */
        uint8_t ping_payload[] = {BRIDGE_COMMAND_STATUS};
        bridge_tx_queue_control(BRIDGE_PKT_CONTROL, ping_payload, sizeof(ping_payload));

        int64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) < ((int64_t)timeout_ms * 1000)) {
            uart_bridge_status_t status;
            if (uart_bridge_get_status(&status) == ESP_OK &&
                status.generation != initial_generation) {
                ESP_LOGI(TAG, "nRF52840 probe response received!");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGW(TAG, "Probe attempt %d timed out", attempt);

        /* Tear down SPI and reinit to reset DMA state. This handles the case
         * where the ESP32 rebooted while the nRF52 was mid-transaction,
         * leaving the SPI peripheral desynced. */
        if (attempt < MAX_PROBE_ATTEMPTS) {
            ESP_LOGI(TAG, "Reinitializing SPI bus...");
            uart_bridge_deinit();
            vTaskDelay(pdMS_TO_TICKS(100)); /* Let bus settle */
            esp_err_t err = uart_bridge_init();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "SPI reinit failed: %s", esp_err_to_name(err));
                return false;
            }
        }
    }

    ESP_LOGW(TAG, "nRF52840 probe timed out after %d attempts", MAX_PROBE_ATTEMPTS);
    return false;
}

static esp_err_t send_mesh_command(bridge_command_t command, uint8_t talk_channel)
{
    if (command == BRIDGE_COMMAND_MESH_START && !mesh_channel_valid(talk_channel)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_bridge.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (__atomic_exchange_n(&g_bridge.command_in_progress, true, __ATOMIC_ACQUIRE)) {
        return ESP_ERR_INVALID_STATE;
    }

    bridge_command_payload_t payload = {
        .command = command,
        .generation = ++g_bridge.command_generation,
        .talk_channel = talk_channel,
    };
    esp_err_t result = ESP_ERR_TIMEOUT;
    __atomic_store_n(&g_bridge.command_ack_received, false, __ATOMIC_RELEASE);

    for (int attempt = 1; attempt <= BRIDGE_COMMAND_MAX_ATTEMPTS; attempt++) {
        result =
            bridge_tx_queue_control(BRIDGE_PKT_CONTROL, (const uint8_t *)&payload, sizeof(payload));
        if (result != ESP_OK) {
            break;
        }

        int64_t deadline_us = esp_timer_get_time() + BRIDGE_COMMAND_ACK_TIMEOUT_MS * 1000LL;
        while (esp_timer_get_time() < deadline_us) {
            if (__atomic_load_n(&g_bridge.command_ack_received, __ATOMIC_ACQUIRE) &&
                g_bridge.command_ack_command == command &&
                g_bridge.command_ack_generation == payload.generation) {
                result = g_bridge.command_ack_result == 0 ? ESP_OK : ESP_FAIL;
                goto done;
            }

            vTaskDelay(pdMS_TO_TICKS(5));
        }
        ESP_LOGW(TAG, "Command 0x%02X generation %u ACK timeout (%d/%d)", command,
                 payload.generation, attempt, BRIDGE_COMMAND_MAX_ATTEMPTS);
    }

done:
    __atomic_store_n(&g_bridge.command_in_progress, false, __ATOMIC_RELEASE);
    return result;
}

esp_err_t uart_bridge_mesh_enable(uint8_t talk_channel)
{
    ESP_LOGI(TAG, "Requesting mesh enable from nRF52840");
    return send_mesh_command(BRIDGE_COMMAND_MESH_START, talk_channel);
}

esp_err_t uart_bridge_mesh_disable(void)
{
    ESP_LOGI(TAG, "Requesting mesh disable from nRF52840");
    return send_mesh_command(BRIDGE_COMMAND_MESH_STOP, MESH_CHANNEL_DEFAULT);
}

esp_err_t uart_bridge_mesh_audio_pause(void)
{
    return send_mesh_command(BRIDGE_COMMAND_AUDIO_PAUSE, 0);
}

esp_err_t uart_bridge_mesh_audio_resume(void)
{
    return send_mesh_command(BRIDGE_COMMAND_AUDIO_RESUME, 0);
}
