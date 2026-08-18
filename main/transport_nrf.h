/**
 * @file transport_nrf.h
 * @brief nRF52840 transport controller (ESB via SPI bridge).
 *
 * Owns the bridge callbacks, membership/notification tracking, mesh state
 * reconciliation, and the redundant-bundle TX path.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief Create synchronization resources. Call once before any bridge callback can fire. */
esp_err_t transport_nrf_init(void);

/** @brief Register the bridge callbacks after a successful probe. */
void transport_nrf_attach(void);

/** @brief Encode and enqueue one encoded frame as a redundant bundle over SPI. */
void transport_nrf_send_audio(const uint8_t *data, uint16_t len, bool active,
                              int64_t timestamp_us);

/** @brief Periodic maintenance: reconcile mesh state and play queued tones. */
void transport_nrf_tick(int64_t now_ms);

/** @brief Apply a runtime mesh-intent transition (tracking and cache resets). */
void transport_nrf_set_user_enabled(bool enabled);

/** @brief Restart the reconciliation attempt budget. */
void transport_nrf_reset_reconciliation(void);

/** @brief Suppress a queued "mesh enabled" tone (toggle rollback paths). */
void transport_nrf_cancel_enable_notification(void);

/** @brief Forget peer-count tracking and queued tones. */
void transport_nrf_reset_membership_tracking(void);

/** @brief Drop cached predecessor audio (transport fallback or disable). */
void transport_nrf_reset_tx_cache(void);

/** @brief Node ID reported by the bridge, or 0 when unknown. */
uint8_t transport_nrf_node_id(void);
