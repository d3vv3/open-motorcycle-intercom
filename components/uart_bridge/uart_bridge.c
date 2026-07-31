/**
 * @file uart_bridge.c
 * @brief SPI Bridge to nRF52840 Mesh Radio (ESP32-S3 = SPI Slave)
 *
 * The nRF52840 is the SPI master and polls the ESP32 periodically.
 * Each SPI transaction is full-duplex: while the master clocks out a
 * packet to the slave, the slave simultaneously sends its queued packet
 * (or a zero-filled idle frame) to the master.
 *
 * Packet wire format:
 *   [0xAA] [LEN] [SEQ] [TYPE] [PAYLOAD...] [CRC8]
 * LEN covers: [SEQ] [TYPE] [PAYLOAD...]
 * CRC8 is calculated over: [LEN] [SEQ] [TYPE] [PAYLOAD...]
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
#define TX_AUDIO_QUEUE_SIZE 16
#define TX_AUDIO_MAX_AGE_US 120000

typedef struct {
    uint8_t buf[BRIDGE_SPI_MAX_XFER];
    uint16_t len;
    int64_t queued_at_us;
} tx_entry_t;

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

/* Mutex protecting packet construction and the multi-producer audio ring head. */
static SemaphoreHandle_t s_tx_mutex;

/* TX audio ring. Producers share s_tx_mutex; the SPI task is the sole consumer. */
static tx_entry_t s_audio_q[TX_AUDIO_QUEUE_SIZE];
static volatile uint8_t s_audio_head = 0;
static volatile uint8_t s_audio_tail = 0;
static SemaphoreHandle_t s_audio_slots_free;
static SemaphoreHandle_t s_audio_slots_used;

/* Control slot: protected by s_tx_mutex (low-frequency path) */
static tx_entry_t s_ctrl_pending;
static volatile bool s_ctrl_pending_valid = false;

/* Pipeline diagnostics */
static uint32_t s_audio_tx_queued = 0;    /* Audio frames queued for SPI TX */
static uint32_t s_audio_tx_overwrite = 0; /* Audio frames that overwrote unsent ones */
static uint32_t s_audio_tx_enqueue_timeout = 0;
static uint32_t s_audio_tx_invalid_size = 0;
static uint32_t s_audio_tx_stale_drop = 0;
static uint32_t s_audio_tx_wait_count = 0;
static uint64_t s_audio_tx_wait_sum_us = 0;
static uint32_t s_audio_tx_wait_max_us = 0;
static uint32_t s_audio_rx_count = 0;     /* Audio frames received from nRF */
static uint8_t s_bridge_tx_seq = 0;
static uint8_t s_bridge_rx_expected = 0;
static bool s_bridge_rx_seq_init = false;
static uint32_t s_bridge_rx_seq_gaps = 0;
static uint32_t s_bridge_rx_crc_fail = 0;
static uint32_t s_bridge_rx_bad_sync = 0;
static uint32_t s_bridge_rx_bad_len = 0;
static uint32_t s_bridge_rx_trunc = 0;
static uint8_t s_audio_tx_waiting_ack_seq = 0;
static volatile bool s_audio_tx_waiting_ack = false;
static tx_entry_t s_audio_tx_inflight;
static volatile bool s_audio_tx_inflight_valid = false;
static volatile uint32_t s_ack_irq_count = 0;
static volatile uint32_t s_ack_release_count = 0;
static volatile uint32_t s_ack_spurious_count = 0;
static int64_t s_ack_wait_start_us = 0;          /* Timestamp when waiting_ack was set */
static volatile uint32_t s_ack_timeout_count = 0; /* Forced releases due to timeout */
#define ACK_TIMEOUT_US  50000  /* 50 ms — force-release if ACK not received */

static void IRAM_ATTR ack_gpio_isr_handler(void *arg)
{
    BaseType_t woke = pdFALSE;

    (void)arg;
    s_ack_irq_count++;

    if (s_audio_tx_waiting_ack) {
        s_audio_tx_waiting_ack = false;
        s_audio_tx_inflight_valid = false;
        s_ack_release_count++;
        xSemaphoreGiveFromISR(s_audio_slots_free, &woke);
    } else {
        s_ack_spurious_count++;
    }

    if (woke == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static uint8_t audio_queue_depth(void)
{
    return s_audio_slots_used ? (uint8_t)uxSemaphoreGetCount(s_audio_slots_used) : 0;
}

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

static uint8_t crc8_compute(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Parse a received SPI buffer for a framed packet.
 *
 * The first byte must be the sync byte.  If the buffer starts with
 * 0x00 it was an idle frame from the master and is silently ignored.
 */
static void parse_rx(const uint8_t *buf, size_t len)
{
    if (len < 5) {
        s_bridge_rx_trunc++;
        return;
    }

    if (buf[0] == 0) {
        return;
    }
    if (buf[0] != SYNC_BYTE) {
        s_bridge_rx_bad_sync++;
        return;
    }

    uint8_t pkt_len = buf[1];
    if (pkt_len < 2 || pkt_len > MAX_PACKET_LEN - 3) {
        s_bridge_rx_bad_len++;
        ESP_LOGW(TAG, "Bad packet length: %u", pkt_len);
        return;
    }

    /* Ensure we actually received enough bytes */
    if ((size_t)(pkt_len + 3) > len) {
        s_bridge_rx_trunc++;
        ESP_LOGW(TAG, "Truncated packet: need %u, got %u", pkt_len + 3, (unsigned)len);
        return;
    }

    uint8_t rx_crc = buf[2 + pkt_len];
    uint8_t calc_crc = crc8_compute(&buf[1], (size_t)(pkt_len + 1));
    if (rx_crc != calc_crc) {
        s_bridge_rx_crc_fail++;
        if ((s_bridge_rx_crc_fail % 50) == 1) {
            ESP_LOGW(TAG, "CRC mismatch: rx=0x%02X calc=0x%02X fail=%lu", rx_crc, calc_crc,
                     s_bridge_rx_crc_fail);
        }
        return;
    }

    uint8_t seq = buf[2];
    if (!s_bridge_rx_seq_init) {
        s_bridge_rx_expected = seq;
        s_bridge_rx_seq_init = true;
    } else {
        uint8_t expected_next = (uint8_t)(s_bridge_rx_expected + 1);
        if (seq != expected_next) {
            s_bridge_rx_seq_gaps++;
        }
    }
    s_bridge_rx_expected = seq;

    uint8_t pkt_type = buf[3];
    const uint8_t *payload = &buf[4];
    uint16_t payload_len = pkt_len - 2; /* -2 for seq + type bytes */

    handle_rx_packet(pkt_type, payload, payload_len);
}

/**
 * @brief Prepare the DMA TX buffer for the next SPI transaction.
 *
 * Audio dequeue is lock-free (SPSC ring).  Only the control slot
 * needs the mutex, and it's the low-frequency path.
 */
static void prepare_tx_buf(void)
{
    /* ---- ACK timeout recovery ----
     * If the nRF ACK pulse was missed (noise, reboot, glitch), force-release
     * the inflight frame so the pipeline doesn't deadlock permanently. */
    if (s_audio_tx_waiting_ack) {
        int64_t now_us = esp_timer_get_time();
        if ((now_us - s_ack_wait_start_us) > ACK_TIMEOUT_US) {
            /* Force-release: give back the slot, clear inflight */
            portDISABLE_INTERRUPTS();
            s_audio_tx_waiting_ack = false;
            s_audio_tx_inflight_valid = false;
            portENABLE_INTERRUPTS();
            xSemaphoreGive(s_audio_slots_free);
            s_ack_timeout_count++;
        }
    }

    /* Fast path: audio queue with GPIO ACK-driven stop-and-wait. */
    if (s_audio_tx_waiting_ack && s_audio_tx_inflight_valid) {
        memcpy(s_tx_dma_buf, s_audio_tx_inflight.buf, BRIDGE_SPI_MAX_XFER);
        return;
    }

    while (xSemaphoreTake(s_audio_slots_used, 0) == pdTRUE) {
        uint8_t t = s_audio_tail;
        __sync_synchronize();
        tx_entry_t *e = &s_audio_q[t];
        s_audio_tail = (uint8_t)((t + 1) % TX_AUDIO_QUEUE_SIZE);

        int64_t now_us = esp_timer_get_time();
        uint32_t wait_us = e->queued_at_us > 0 && now_us > e->queued_at_us
                               ? (uint32_t)(now_us - e->queued_at_us)
                               : 0;
        if (wait_us > TX_AUDIO_MAX_AGE_US) {
            s_audio_tx_stale_drop++;
            xSemaphoreGive(s_audio_slots_free);
            continue;
        }

        s_audio_tx_wait_count++;
        s_audio_tx_wait_sum_us += wait_us;
        if (wait_us > s_audio_tx_wait_max_us) {
            s_audio_tx_wait_max_us = wait_us;
        }

        memcpy(s_tx_dma_buf, e->buf, BRIDGE_SPI_MAX_XFER);

        memcpy(s_audio_tx_inflight.buf, e->buf, BRIDGE_SPI_MAX_XFER);
        s_audio_tx_inflight.len = e->len;

        /* Critical section: set inflight + waiting_ack atomically so the
         * ISR cannot see a half-updated state and count a valid ACK as
         * spurious (which would leak a semaphore slot permanently). */
        portDISABLE_INTERRUPTS();
        s_audio_tx_inflight_valid = true;
        s_audio_tx_waiting_ack = true;
        portENABLE_INTERRUPTS();
        s_audio_tx_waiting_ack_seq = e->buf[2];
        s_ack_wait_start_us = esp_timer_get_time();
        return;
    }

    /* Slow path: control packet (needs mutex, but very rare) */
    if (s_ctrl_pending_valid) {
        if (xSemaphoreTake(s_tx_mutex, 0) == pdTRUE) {
            if (s_ctrl_pending_valid) {
                memcpy(s_tx_dma_buf, s_ctrl_pending.buf, BRIDGE_SPI_MAX_XFER);
                s_ctrl_pending_valid = false;
            } else {
                memset(s_tx_dma_buf, 0, BRIDGE_SPI_MAX_XFER);
            }
            xSemaphoreGive(s_tx_mutex);
            return;
        }
    }

    /* Nothing pending, send idle frame */
    memset(s_tx_dma_buf, 0, BRIDGE_SPI_MAX_XFER);
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

    s_audio_slots_free = xSemaphoreCreateCounting(TX_AUDIO_QUEUE_SIZE - 1, TX_AUDIO_QUEUE_SIZE - 1);
    s_audio_slots_used = xSemaphoreCreateCounting(TX_AUDIO_QUEUE_SIZE - 1, 0);
    if (s_audio_slots_free == NULL || s_audio_slots_used == NULL) {
        ESP_LOGE(TAG, "Failed to create audio queue semaphores");
        if (s_audio_slots_free) {
            vSemaphoreDelete(s_audio_slots_free);
            s_audio_slots_free = NULL;
        }
        if (s_audio_slots_used) {
            vSemaphoreDelete(s_audio_slots_used);
            s_audio_slots_used = NULL;
        }
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
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
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
    }

    err = gpio_isr_handler_add(BRIDGE_ACK_GPIO_PIN, ack_gpio_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ACK GPIO ISR: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        vSemaphoreDelete(s_tx_mutex);
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        s_tx_mutex = NULL;
        return err;
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

    err = spi_slave_initialize(BRIDGE_SPI_HOST, &bus_cfg, &slave_cfg, BRIDGE_SPI_DMA_CHAN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(err));
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        vSemaphoreDelete(s_tx_mutex);
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        s_tx_mutex = NULL;
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
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
        gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
        vSemaphoreDelete(s_tx_mutex);
        vSemaphoreDelete(s_audio_slots_free);
        vSemaphoreDelete(s_audio_slots_used);
        s_tx_mutex = NULL;
        s_audio_slots_free = NULL;
        s_audio_slots_used = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    s_audio_head = 0;
    s_audio_tail = 0;
    s_audio_tx_waiting_ack = false;
    s_audio_tx_waiting_ack_seq = 0;
    s_audio_tx_inflight_valid = false;
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

    gpio_isr_handler_remove(BRIDGE_ACK_GPIO_PIN);
    spi_slave_free(BRIDGE_SPI_HOST);

    if (s_tx_mutex) {
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
    }

    if (s_audio_slots_free) {
        vSemaphoreDelete(s_audio_slots_free);
        s_audio_slots_free = NULL;
    }
    if (s_audio_slots_used) {
        vSemaphoreDelete(s_audio_slots_used);
        s_audio_slots_used = NULL;
    }

    s_initialized = false;
    s_connected = false;
    s_audio_tx_waiting_ack = false;
    s_audio_tx_inflight_valid = false;

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

    uint16_t wire_len = (uint16_t)(2 + len);   /* SEQ + TYPE + PAYLOAD */
    uint16_t total = (uint16_t)(3 + wire_len); /* SYNC + LEN + BODY + CRC */
    if (total > BRIDGE_SPI_MAX_XFER) {
        s_audio_tx_invalid_size++;
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(s_tx_buf, 0, BRIDGE_SPI_MAX_XFER);
    s_tx_buf[0] = SYNC_BYTE;
    s_tx_buf[1] = (uint8_t)wire_len;
    s_tx_buf[2] = s_bridge_tx_seq++;
    s_tx_buf[3] = type;
    if (len > 0 && payload != NULL) {
        memcpy(&s_tx_buf[4], payload, len);
    }
    s_tx_buf[2 + wire_len] = crc8_compute(&s_tx_buf[1], (size_t)(wire_len + 1));
    memcpy(s_ctrl_pending.buf, s_tx_buf, BRIDGE_SPI_MAX_XFER);
    s_ctrl_pending.len = total;
    s_ctrl_pending_valid = true;

    xSemaphoreGive(s_tx_mutex);
    return ESP_OK;
}

esp_err_t uart_bridge_send_audio(const uint8_t *data, uint16_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t wire_len = (uint16_t)(2 + len);   /* SEQ + TYPE + PAYLOAD */
    uint16_t total = (uint16_t)(3 + wire_len); /* SYNC + LEN + BODY + CRC */
    if (total > BRIDGE_SPI_MAX_XFER) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(s_audio_slots_free, pdMS_TO_TICKS(20)) != pdTRUE) {
        s_audio_tx_enqueue_timeout++;
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);

    uint8_t cur_head = s_audio_head;
    uint8_t next_head = (uint8_t)((cur_head + 1) % TX_AUDIO_QUEUE_SIZE);

    tx_entry_t *entry = &s_audio_q[cur_head];
    memset(entry->buf, 0, BRIDGE_SPI_MAX_XFER);
    entry->buf[0] = SYNC_BYTE;
    entry->buf[1] = (uint8_t)wire_len;
    entry->buf[2] = s_bridge_tx_seq++;
    entry->buf[3] = BRIDGE_PKT_AUDIO;
    if (len > 0 && data != NULL) {
        memcpy(&entry->buf[4], data, len);
    }
    entry->buf[2 + wire_len] = crc8_compute(&entry->buf[1], (size_t)(wire_len + 1));
    entry->len = total;
    entry->queued_at_us = esp_timer_get_time();

    /* Memory barrier: ensure all writes to entry are visible before
     * the consumer sees the new head. On ESP32 (Xtensa) this is implicit
     * because volatile reads/writes are sequentially consistent within
     * a single core, but the consumer runs on a different core. */
    __sync_synchronize();
    s_audio_head = next_head;
    xSemaphoreGive(s_audio_slots_used);
    s_audio_tx_queued++;
    xSemaphoreGive(s_tx_mutex);

    /* Log stats periodically — OUTSIDE any critical section */
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 5000000) { /* Every 5s */
        ESP_LOGI(TAG,
                 "Audio pipe: tx_queued=%lu tx_overwr=%lu rx_from_nrf=%lu tx_q=%u ctrl_pending=%d bad_sync=%lu bad_len=%lu trunc=%lu crc_fail=%lu seq_gap=%lu ack_irq=%lu ack_rel=%lu ack_spur=%lu ack_to=%lu waiting=%d",
                 s_audio_tx_queued, s_audio_tx_overwrite, s_audio_rx_count,
                 audio_queue_depth(), s_ctrl_pending_valid ? 1 : 0,
                 s_bridge_rx_bad_sync, s_bridge_rx_bad_len, s_bridge_rx_trunc,
                 s_bridge_rx_crc_fail, s_bridge_rx_seq_gaps, s_ack_irq_count,
                 s_ack_release_count, s_ack_spurious_count, s_ack_timeout_count,
                 s_audio_tx_waiting_ack ? 1 : 0);
        ESP_LOGI(TAG,
                 "PIPE v=1 dev=esp stage=spi tx_q_ok=%lu tx_q_timeout=%lu tx_size_drop=%lu tx_stale_drop=%lu tx_wait_avg_us=%lu tx_wait_max_us=%lu rx_ok=%lu crc_drop=%lu sync_drop=%lu len_drop=%lu trunc_drop=%lu seq_gap=%lu ack_timeout=%lu q_depth=%u",
                 s_audio_tx_queued, s_audio_tx_enqueue_timeout, s_audio_tx_invalid_size,
                 s_audio_tx_stale_drop,
                 s_audio_tx_wait_count ? (uint32_t)(s_audio_tx_wait_sum_us / s_audio_tx_wait_count) : 0,
                 s_audio_tx_wait_max_us,
                 s_audio_rx_count, s_bridge_rx_crc_fail, s_bridge_rx_bad_sync,
                 s_bridge_rx_bad_len, s_bridge_rx_trunc, s_bridge_rx_seq_gaps,
                 s_ack_timeout_count, audio_queue_depth());
        last_log = now;
    }

    return ESP_OK;
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
