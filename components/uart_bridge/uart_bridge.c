/**
 * @file uart_bridge.c
 * @brief SPI Bridge to nRF52840 Mesh Radio (ESP32-S3 = SPI Slave)
 *
 * The nRF52840 is the SPI master and polls the ESP32 periodically.
 * Each SPI transaction is full-duplex: while the master clocks out a
 * packet to the slave, the slave simultaneously sends its queued packet
 * (or a zero-filled idle frame) to the master.
 *
 * Packet wire format (same as old UART protocol, fits in one SPI xfer):
 *   [0xAA] [LEN] [TYPE] [PAYLOAD...]
 * Padded with zeros to BRIDGE_SPI_MAX_XFER bytes.
 *
 * "uart_bridge" naming retained for API compatibility.
 */

#include "uart_bridge.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

static const char *TAG = "spi_bridge";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SYNC_BYTE          0xAA
#define MAX_PACKET_LEN     256
#define RX_TASK_STACK_SIZE 4096
#define RX_TASK_PRIORITY   5
#define RX_TASK_CORE       0

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static TaskHandle_t s_rx_task = NULL;

static uart_bridge_audio_cb_t s_audio_cb = NULL;
static uart_bridge_event_cb_t s_event_cb = NULL;

static uart_bridge_status_t s_status = {0};
static bool s_connected = false;

/*
 * TX double-buffer: s_tx_buf is the staging area where the app writes
 * new packets.  s_tx_dma_buf is what the SPI DMA actually reads from.
 * prepare_tx_buf() copies staging -> DMA so the audio callback can
 * safely write to s_tx_buf while a DMA transfer is in flight.
 */
static WORD_ALIGNED_ATTR uint8_t s_tx_buf[BRIDGE_SPI_MAX_XFER];     /* Staging buffer */
static WORD_ALIGNED_ATTR uint8_t s_tx_dma_buf[BRIDGE_SPI_MAX_XFER]; /* DMA buffer */
static WORD_ALIGNED_ATTR uint8_t s_rx_buf[BRIDGE_SPI_MAX_XFER];

/* Mutex protecting s_tx_buf between the app thread and the SPI task */
static SemaphoreHandle_t s_tx_mutex;

/* Pending TX packet length (0 = idle, send zeros) */
static volatile uint16_t s_tx_pending_len = 0;

/* Pipeline diagnostics */
static uint32_t s_audio_tx_queued = 0;    /* Audio frames queued for SPI TX */
static uint32_t s_audio_tx_overwrite = 0; /* Audio frames that overwrote unsent ones */
static uint32_t s_audio_rx_count = 0;     /* Audio frames received from nRF */

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void handle_rx_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    switch (type) {
    case BRIDGE_PKT_AUDIO:
        if (s_audio_cb && len > 1) {
            uint8_t src_id = payload[0];
            s_audio_rx_count++;
            s_audio_cb(src_id, payload + 1, len - 1, esp_timer_get_time());
        }
        break;

    case BRIDGE_PKT_STATUS:
        if (len >= 3) {
            /* Check for changes before updating */
            bool changed = (!s_connected) || (s_status.is_coordinator != (payload[0] != 0)) ||
                           (s_status.peer_count != payload[1]) || (s_status.node_id != payload[2]);

            memset(&s_status, 0, sizeof(s_status));
            s_status.is_coordinator = (payload[0] != 0);
            s_status.peer_count = payload[1];
            s_status.node_id = payload[2];
            s_connected = true;

            if (changed) {
                ESP_LOGI(TAG, "Status: node=%d peers=%d coordinator=%d", s_status.node_id,
                         s_status.peer_count, s_status.is_coordinator);
            }
        }
        break;

    case BRIDGE_PKT_MESH_EVENT:
        if (s_event_cb && len > 0) {
            uart_bridge_event_t event = (uart_bridge_event_t)payload[0];
            s_event_cb(event, payload + 1, len - 1);
        }
        break;

    case BRIDGE_PKT_LOG:
        if (len > 0) {
            /* Print nRF52 log with prefix, ensure null termination */
            char log_msg[129];
            uint16_t copy_len = (len < sizeof(log_msg) - 1) ? len : sizeof(log_msg) - 1;
            memcpy(log_msg, payload, copy_len);
            log_msg[copy_len] = '\0';
            ESP_LOGI("nRF52", "%s", log_msg);
        }
        break;

    default:
        ESP_LOGD(TAG, "Unknown packet type: 0x%02X", type);
        break;
    }
}

/**
 * @brief Parse a received SPI buffer for a framed packet.
 *
 * The first byte must be the sync byte.  If the buffer starts with
 * 0x00 it was an idle frame from the master and is silently ignored.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    if (len < 3) {
        return;
    }

    /* Idle frame from master (all zeros or no sync byte) */
    if (buf[0] != SYNC_BYTE) {
        return;
    }

    uint8_t pkt_len = buf[1];
    if (pkt_len < 1 || pkt_len > MAX_PACKET_LEN - 2) {
        ESP_LOGW(TAG, "Bad packet length: %u", pkt_len);
        return;
    }

    /* Ensure we actually received enough bytes */
    if ((size_t)(pkt_len + 2) > len) {
        ESP_LOGW(TAG, "Truncated packet: need %u, got %u", pkt_len + 2, (unsigned)len);
        return;
    }

    uint8_t pkt_type = buf[2];
    const uint8_t *payload = &buf[3];
    uint16_t payload_len = pkt_len - 1; /* -1 for type byte */

    handle_rx_packet(pkt_type, payload, payload_len);
}

/**
 * @brief Prepare the DMA TX buffer for the next SPI transaction.
 *
 * Copies from staging buffer (s_tx_buf) to DMA buffer (s_tx_dma_buf)
 * and clears the pending flag so the staging buffer can accept new data.
 */
static void prepare_tx_buf(void)
{
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_tx_pending_len > 0) {
            /* Copy staging -> DMA and clear pending */
            memcpy(s_tx_dma_buf, s_tx_buf, BRIDGE_SPI_MAX_XFER);
            s_tx_pending_len = 0;
        } else {
            /* Nothing pending, send idle frame */
            memset(s_tx_dma_buf, 0, BRIDGE_SPI_MAX_XFER);
        }
        xSemaphoreGive(s_tx_mutex);
    } else {
        /* Couldn't lock - send idle */
        memset(s_tx_dma_buf, 0, BRIDGE_SPI_MAX_XFER);
    }
}

/**
 * @brief SPI slave RX task
 *
 * Continuously queues SPI slave transactions.  Each transaction is
 * full-duplex: we send s_tx_buf and receive into s_rx_buf simultaneously.
 */
static void spi_slave_task(void *arg)
{
    ESP_LOGI(TAG, "SPI slave task started on core %d", xPortGetCoreID());

    spi_slave_transaction_t txn;
    memset(&txn, 0, sizeof(txn));

    uint32_t txn_count = 0;

    while (1) {
        /* Prepare outgoing data */
        prepare_tx_buf();

        memset(s_rx_buf, 0, BRIDGE_SPI_MAX_XFER);
        txn.length = BRIDGE_SPI_MAX_XFER * 8; /* length in bits */
        txn.tx_buffer = s_tx_dma_buf;
        txn.rx_buffer = s_rx_buf;

        /* Block until the master performs a transaction */
        esp_err_t err = spi_slave_transmit(BRIDGE_SPI_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        txn_count++;

        /* Parse what the master sent us */
        size_t rx_bytes = txn.trans_len / 8;

        /* Debug: log first 3 transactions for boot verification */
        if (txn_count <= 3) {
            ESP_LOGI(TAG, "SPI txn #%lu: %u bytes, rx[0..3]: %02X %02X %02X %02X", txn_count,
                     (unsigned)rx_bytes, s_rx_buf[0], s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
        }

        if (rx_bytes > 0) {
            parse_rx(s_rx_buf, rx_bytes);
        }
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t uart_bridge_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }

    /* SPI slave bus configuration */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BRIDGE_SPI_MOSI_PIN,
        .miso_io_num = BRIDGE_SPI_MISO_PIN,
        .sclk_io_num = BRIDGE_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BRIDGE_SPI_MAX_XFER,
    };

    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = BRIDGE_SPI_CS_PIN,
        .flags = 0,
        .queue_size = 2,
        .mode = BRIDGE_SPI_MODE,
    };

    esp_err_t err =
        spi_slave_initialize(BRIDGE_SPI_HOST, &bus_cfg, &slave_cfg, BRIDGE_SPI_DMA_CHAN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_tx_mutex);
        return err;
    }

    /* Pull-ups on input lines — required for reliable CS/SCK detection.
     * Must be set AFTER spi_slave_initialize so the SPI driver owns the pins first. */
    gpio_set_pull_mode(BRIDGE_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_SCLK_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_CS_PIN, GPIO_PULLUP_ONLY);

    /* Start SPI slave task on core 0 so audio stays on core 1 */
    BaseType_t ret = xTaskCreatePinnedToCore(spi_slave_task, "spi_bridge_rx", RX_TASK_STACK_SIZE,
                                             NULL, RX_TASK_PRIORITY, &s_rx_task, RX_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI task");
        spi_slave_free(BRIDGE_SPI_HOST);
        vSemaphoreDelete(s_tx_mutex);
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "SPI bridge initialized (MOSI=%d, MISO=%d, SCK=%d, CS=%d)", BRIDGE_SPI_MOSI_PIN,
             BRIDGE_SPI_MISO_PIN, BRIDGE_SPI_SCLK_PIN, BRIDGE_SPI_CS_PIN);

    return ESP_OK;
}

void uart_bridge_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_rx_task) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }

    spi_slave_free(BRIDGE_SPI_HOST);

    if (s_tx_mutex) {
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
    }

    s_initialized = false;
    s_connected = false;

    ESP_LOGI(TAG, "SPI bridge deinitialized");
}

/**
 * @brief Queue a framed packet into the TX buffer for the next SPI transaction.
 */
static esp_err_t queue_tx_packet(uint8_t type, const uint8_t *payload, uint16_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t total = 3 + len; /* SYNC + LEN + TYPE + PAYLOAD */
    if (total > BRIDGE_SPI_MAX_XFER) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = SYNC_BYTE;
    s_tx_buf[1] = (uint8_t)(len + 1); /* LEN = type byte + payload */
    s_tx_buf[2] = type;
    if (len > 0 && payload != NULL) {
        memcpy(&s_tx_buf[3], payload, len);
    }
    s_tx_pending_len = total;

    xSemaphoreGive(s_tx_mutex);
    return ESP_OK;
}

esp_err_t uart_bridge_send_audio(const uint8_t *data, uint16_t len)
{
    /* Track overwrites: if previous audio wasn't sent yet */
    if (s_tx_pending_len > 0) {
        s_audio_tx_overwrite++;
    }
    s_audio_tx_queued++;

    /* Log stats periodically */
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 5000000) { /* Every 5s */
        ESP_LOGI(TAG, "Audio pipe: tx_queued=%lu tx_overwr=%lu rx_from_nrf=%lu", s_audio_tx_queued,
                 s_audio_tx_overwrite, s_audio_rx_count);
        last_log = now;
    }

    return queue_tx_packet(BRIDGE_PKT_AUDIO, data, len);
}

void uart_bridge_set_audio_callback(uart_bridge_audio_cb_t cb)
{
    s_audio_cb = cb;
}

void uart_bridge_set_event_callback(uart_bridge_event_cb_t cb)
{
    s_event_cb = cb;
}

esp_err_t uart_bridge_get_status(uart_bridge_status_t *status)
{
    if (!s_connected) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(status, &s_status, sizeof(uart_bridge_status_t));
    return ESP_OK;
}

bool uart_bridge_is_connected(void)
{
    return s_connected;
}

bool uart_bridge_probe(uint32_t timeout_ms)
{
    if (!s_initialized) {
        return false;
    }

    if (s_connected) {
        return true;
    }

    const int MAX_PROBE_ATTEMPTS = 3;

    for (int attempt = 1; attempt <= MAX_PROBE_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing for nRF52840 (attempt %d/%d)...", attempt, MAX_PROBE_ATTEMPTS);

        /* Queue a PING command for the master to pick up on its next poll */
        uint8_t ping_payload[] = {0x03}; /* CMD_ID = PING */
        queue_tx_packet(BRIDGE_PKT_CONTROL, ping_payload, sizeof(ping_payload));

        /* Wait for the master to respond with a STATUS packet */
        int64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) < ((int64_t)timeout_ms * 1000)) {
            if (s_connected) {
                ESP_LOGI(TAG, "nRF52840 probe response received!");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGW(TAG, "Probe attempt %d timed out", attempt);

        /* If not the last attempt, tear down SPI and reinit to reset DMA state.
         * This handles the case where the ESP32 rebooted while the nRF52
         * was mid-transaction, leaving the SPI peripheral desynced. */
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

esp_err_t uart_bridge_mesh_enable(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending mesh enable command to nRF52840");
    uint8_t cmd_payload[] = {0x01}; /* CMD_ID = Enable Mesh */
    return queue_tx_packet(BRIDGE_PKT_CONTROL, cmd_payload, sizeof(cmd_payload));
}

esp_err_t uart_bridge_mesh_disable(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending mesh disable command to nRF52840");
    uint8_t cmd_payload[] = {0x02}; /* CMD_ID = Disable Mesh */
    return queue_tx_packet(BRIDGE_PKT_CONTROL, cmd_payload, sizeof(cmd_payload));
}
