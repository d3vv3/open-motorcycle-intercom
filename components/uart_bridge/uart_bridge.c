/**
 * @file uart_bridge.c
 * @brief SPI bridge facade: slave hardware lifecycle and transfer task.
 *
 * The nRF52840 is the SPI master and polls the ESP32 periodically. Each
 * SPI transaction is full-duplex: while the master clocks out a packet to
 * the slave, the slave simultaneously sends its queued packet (or a
 * zero-filled idle frame).
 *
 * Wire format (shared/bridge_frame.h):
 *   [0xAA] [LEN] [SEQ] [TYPE] [PAYLOAD...] [CRC8]
 * padded with zeros to BRIDGE_SPI_MAX_XFER bytes.
 *
 * "uart_bridge" naming retained for API compatibility.
 */

#include <string.h>

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "bridge_internal.h"

static const char *TAG = "spi_bridge";

_Static_assert(BRIDGE_SPI_MAX_PAYLOAD <= BRIDGE_FRAME_MAX_PAYLOAD,
               "SPI bridge payload exceeds frame codec capacity");

bridge_context_t g_bridge;
portMUX_TYPE g_bridge_status_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_bridge_ack_lock = portMUX_INITIALIZER_UNLOCKED;

/*
 * TX double-buffer: g_bridge_tx_staging is where packet construction
 * happens. g_bridge_tx_dma is what the SPI DMA actually reads from.
 * bridge_tx_prepare_dma() copies staging -> DMA so producers can safely
 * write while a DMA transfer is in flight.
 */
WORD_ALIGNED_ATTR uint8_t g_bridge_tx_staging[BRIDGE_SPI_MAX_XFER];
WORD_ALIGNED_ATTR uint8_t g_bridge_tx_dma[BRIDGE_SPI_MAX_XFER];
WORD_ALIGNED_ATTR uint8_t g_bridge_rx_dma[BRIDGE_SPI_MAX_XFER];

/**
 * @brief SPI slave transfer task.
 *
 * Continuously queues full-duplex SPI slave transactions and parses
 * whatever the master clocked out.
 */
static void spi_slave_task(void *arg)
{
    ESP_LOGI(TAG, "SPI slave task started on core %d", xPortGetCoreID());

    spi_slave_transaction_t txn;
    memset(&txn, 0, sizeof(txn));

    uint32_t txn_count = 0;

    while (1) {
        bridge_tx_prepare_dma();

        memset(g_bridge_rx_dma, 0, BRIDGE_SPI_MAX_XFER);
        txn.length = BRIDGE_SPI_MAX_XFER * 8; /* length in bits */
        txn.tx_buffer = g_bridge_tx_dma;
        txn.rx_buffer = g_bridge_rx_dma;

        /* Block until the master performs a transaction */
        esp_err_t err = spi_slave_transmit(BRIDGE_SPI_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        txn_count++;

        size_t rx_bytes = txn.trans_len / 8;

        /* Log the first transactions for boot verification */
        if (txn_count <= 3) {
            ESP_LOGI(TAG, "SPI txn #%lu: %u bytes, rx[0..3]: %02X %02X %02X %02X", txn_count,
                     (unsigned)rx_bytes, g_bridge_rx_dma[0], g_bridge_rx_dma[1], g_bridge_rx_dma[2],
                     g_bridge_rx_dma[3]);
        }

        if (rx_bytes > 0) {
            bridge_rx_parse(g_bridge_rx_dma, rx_bytes);
        }
    }
}

static void destroy_sync_resources(void)
{
    if (g_bridge.tx_mutex) {
        vSemaphoreDelete(g_bridge.tx_mutex);
        g_bridge.tx_mutex = NULL;
    }
    if (g_bridge.audio_slots_free) {
        vSemaphoreDelete(g_bridge.audio_slots_free);
        g_bridge.audio_slots_free = NULL;
    }
    if (g_bridge.audio_slots_used) {
        vSemaphoreDelete(g_bridge.audio_slots_used);
        g_bridge.audio_slots_used = NULL;
    }
}

esp_err_t uart_bridge_init(void)
{
    if (g_bridge.initialized) {
        return ESP_OK;
    }

    g_bridge.tx_mutex = xSemaphoreCreateMutex();
    if (g_bridge.tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }

    g_bridge.audio_slots_free =
        xSemaphoreCreateCounting(BRIDGE_TX_AUDIO_QUEUE_SIZE - 1, BRIDGE_TX_AUDIO_QUEUE_SIZE - 1);
    g_bridge.audio_slots_used = xSemaphoreCreateCounting(BRIDGE_TX_AUDIO_QUEUE_SIZE - 1, 0);
    if (g_bridge.audio_slots_free == NULL || g_bridge.audio_slots_used == NULL) {
        ESP_LOGE(TAG, "Failed to create audio queue semaphores");
        destroy_sync_resources();
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t ack_gpio_cfg = {
        .pin_bit_mask = (1ULL << BRIDGE_ACK_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&ack_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ACK GPIO: %s", esp_err_to_name(err));
        destroy_sync_resources();
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        destroy_sync_resources();
        return err;
    }

    err = gpio_isr_handler_add(BRIDGE_ACK_GPIO_PIN, bridge_tx_ack_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ACK GPIO ISR: %s", esp_err_to_name(err));
        destroy_sync_resources();
        return err;
    }

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

    err = spi_slave_initialize(BRIDGE_SPI_HOST, &bus_cfg, &slave_cfg, BRIDGE_SPI_DMA_CHAN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(err));
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        destroy_sync_resources();
        return err;
    }

    /* Pull-ups on input lines - required for reliable CS/SCK detection.
     * Must be set AFTER spi_slave_initialize so the SPI driver owns the pins first. */
    gpio_set_pull_mode(BRIDGE_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_SCLK_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BRIDGE_SPI_CS_PIN, GPIO_PULLUP_ONLY);

    /* SPI slave task runs on core 0 so audio stays on core 1 */
    BaseType_t ret =
        xTaskCreatePinnedToCore(spi_slave_task, "spi_bridge_rx", BRIDGE_RX_TASK_STACK_SIZE, NULL,
                                BRIDGE_RX_TASK_PRIORITY, &g_bridge.rx_task, BRIDGE_RX_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI task");
        spi_slave_free(BRIDGE_SPI_HOST);
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        destroy_sync_resources();
        return ESP_FAIL;
    }

    g_bridge.initialized = true;
    g_bridge.audio_head = 0;
    g_bridge.audio_tail = 0;
    g_bridge.waiting_ack = false;
    g_bridge.inflight_valid = false;
    g_bridge.command_in_progress = false;
    g_bridge.command_ack_received = false;
    ESP_LOGI(TAG, "SPI bridge initialized (MOSI=%d, MISO=%d, SCK=%d, CS=%d)", BRIDGE_SPI_MOSI_PIN,
             BRIDGE_SPI_MISO_PIN, BRIDGE_SPI_SCLK_PIN, BRIDGE_SPI_CS_PIN);

    return ESP_OK;
}

void uart_bridge_deinit(void)
{
    if (!g_bridge.initialized) {
        return;
    }

    if (g_bridge.rx_task) {
        vTaskDelete(g_bridge.rx_task);
        g_bridge.rx_task = NULL;
    }

    gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
    spi_slave_free(BRIDGE_SPI_HOST);

    destroy_sync_resources();

    g_bridge.initialized = false;
    portENTER_CRITICAL(&g_bridge_status_lock);
    g_bridge.connected = false;
    memset(&g_bridge.status, 0, sizeof(g_bridge.status));
    g_bridge.status_expired = false;
    g_bridge.status_age_current_ms = 0;
    portEXIT_CRITICAL(&g_bridge_status_lock);
    g_bridge.waiting_ack = false;
    g_bridge.inflight_valid = false;
    g_bridge.command_in_progress = false;
    g_bridge.command_ack_received = false;

    ESP_LOGI(TAG, "SPI bridge deinitialized");
}

void uart_bridge_set_audio_callback(uart_bridge_audio_cb_t cb)
{
    g_bridge.audio_cb = cb;
}

void uart_bridge_set_event_callback(uart_bridge_event_cb_t cb)
{
    g_bridge.event_cb = cb;
}

void uart_bridge_set_status_callback(uart_bridge_status_cb_t cb)
{
    g_bridge.status_cb = cb;
}
