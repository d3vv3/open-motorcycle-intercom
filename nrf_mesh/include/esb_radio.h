/**
 * @file esb_radio.h
 * @brief ESB Radio Driver for OMI Mesh
 */

#ifndef OMI_ESB_RADIO_H
#define OMI_ESB_RADIO_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Callback for received ESB packets
 */
typedef void (*esb_rx_callback_t)(const uint8_t *data, uint8_t len, const uint8_t *src_addr,
                                  int8_t rssi);

typedef struct {
    uint32_t tx_count;
    uint32_t tx_timeout_count;
    uint32_t tx_busy_count;
    uint32_t tx_write_fail_count;
    uint32_t tx_failed_event_count;
    uint32_t tx_wait_us_avg;
    uint32_t tx_wait_us_max;
    uint32_t rx_pause_us_avg;
    uint32_t rx_pause_us_max;
    uint32_t rx_no_callback_count;
    uint32_t rx_flush_drop_count;
    uint32_t rx_restart_fail_count;
} esb_radio_timing_stats_t;

/**
 * @brief Initialize ESB radio
 * @param channel RF channel (0-100)
 * @return 0 on success, negative error code on failure
 */
int esb_radio_init(uint8_t channel);

/**
 * @brief Deinitialize ESB radio
 */
void esb_radio_deinit(void);

/**
 * @brief Set RX callback
 */
void esb_radio_set_rx_callback(esb_rx_callback_t cb);

/**
 * @brief Send packet via ESB (broadcast)
 * @param data Packet data
 * @param len Data length (max 252 bytes)
 * @return 0 on success
 */
int esb_radio_send(const uint8_t *data, uint8_t len);

/**
 * @brief Send packet to specific address
 */
int esb_radio_send_to(const uint8_t *addr, const uint8_t *data, uint8_t len);

/**
 * @brief Start RX mode
 */
int esb_radio_start_rx(void);

/**
 * @brief Stop RX mode
 */
void esb_radio_stop_rx(void);

/**
 * @brief Get local ESB address
 */
void esb_radio_get_address(uint8_t *addr);

/**
 * @brief Get ESB TX/RX timing statistics
 */
void esb_radio_get_timing_stats(esb_radio_timing_stats_t *stats);

#endif /* OMI_ESB_RADIO_H */
