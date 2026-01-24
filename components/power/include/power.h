/**
 * @file power.h
 * @brief OMI Power Management Interface - Phase 3
 *
 * Power state machine and radio duty cycling for battery optimization.
 * See docs/power.md for power budget and state definitions.
 */

#ifndef OMI_POWER_H
#define OMI_POWER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Power State Definitions (from power.md)
 * ============================================================================ */

/**
 * @brief System power states
 *
 * State transitions:
 *   DEEP_SLEEP -> STANDBY -> MESH_IDLE <-> ACTIVE_VOICE
 *
 * Power targets:
 *   DEEP_SLEEP:   <10 µA (RTC only)
 *   STANDBY:      3-10 mW (beacon listening)
 *   MESH_IDLE:    30-50 mW (TDMA sync, VOX monitoring)
 *   ACTIVE_VOICE: 120-180 mW (Opus + Radio TX)
 */
typedef enum {
    POWER_STATE_DEEP_SLEEP,   /**< Lowest power, RTC wake only */
    POWER_STATE_STANDBY,      /**< Powered on, not in mesh */
    POWER_STATE_MESH_IDLE,    /**< Mesh active, no voice */
    POWER_STATE_ACTIVE_VOICE, /**< Voice transmission active */
} power_state_t;

/**
 * @brief Power subsystem configuration
 */
typedef struct {
    uint32_t deep_sleep_timeout_sec; /**< Seconds of inactivity before deep sleep */
    uint32_t standby_beacon_ms;      /**< Beacon interval in standby (ms) */
    bool enable_radio_duty_cycle;    /**< Enable slot-based radio gating */
    bool enable_light_sleep;         /**< Enable light sleep between frames */
} power_config_t;

/**
 * @brief Default power configuration
 *
 * Note: Radio duty cycling is disabled by default because toggling
 * WiFi power save every 20ms frame adds more overhead than it saves.
 * WiFi PS_MIN_MODEM is still used for power savings.
 */
#define POWER_CONFIG_DEFAULT()                                                                     \
    {                                                                                              \
        .deep_sleep_timeout_sec = 300, /* 5 minutes */                                             \
        .standby_beacon_ms = 1000,                                                                 \
        .enable_radio_duty_cycle = false, /* Disabled: per-frame toggling adds overhead */         \
        .enable_light_sleep = true,                                                                \
    }

/**
 * @brief Power statistics
 */
typedef struct {
    uint32_t state_changes;       /**< Total state transitions */
    uint32_t voice_activations;   /**< Times voice was activated */
    uint32_t radio_duty_cycles;   /**< Radio on/off cycles */
    uint32_t light_sleep_count;   /**< Light sleep entries */
    uint64_t time_in_state_ms[4]; /**< Time spent in each state */
    int64_t last_activity_ms;     /**< Last activity timestamp */
} power_stats_t;

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Initialize power management subsystem
 * @return ESP_OK on success
 */
esp_err_t power_init(void);

/**
 * @brief Initialize with custom configuration
 * @param config Power configuration
 * @return ESP_OK on success
 */
esp_err_t power_init_with_config(const power_config_t *config);

/**
 * @brief Deinitialize power management
 * @return ESP_OK on success
 */
esp_err_t power_deinit(void);

/**
 * @brief Request state transition
 *
 * The power manager may delay or deny transitions based on
 * current system state (e.g., cannot enter deep sleep during TX).
 *
 * @param state Target power state
 * @return ESP_OK if transition accepted
 */
esp_err_t power_set_state(power_state_t state);

/**
 * @brief Get current power state
 * @return Current power state
 */
power_state_t power_get_state(void);

/**
 * @brief Get power state name as string
 * @param state Power state
 * @return State name string
 */
const char *power_state_str(power_state_t state);

/* ============================================================================
 * Voice Activity Notifications (called by audio subsystem)
 * ============================================================================ */

/**
 * @brief Notify power manager that voice transmission is starting
 *
 * Called by audio subsystem when VOX triggers.
 * Transitions to ACTIVE_VOICE state.
 */
void power_notify_voice_start(void);

/**
 * @brief Notify power manager that voice transmission has ended
 *
 * Called by audio subsystem when VOX releases.
 * Transitions back to MESH_IDLE state.
 */
void power_notify_voice_end(void);

/* ============================================================================
 * Radio Control (called by mesh subsystem)
 * ============================================================================ */

/**
 * @brief Enable radio for TX/RX window
 *
 * Called by mesh at start of TDMA slot.
 * Wakes radio from power save if needed.
 */
void power_radio_slot_start(void);

/**
 * @brief Disable radio after TX/RX window
 *
 * Called by mesh at end of TDMA slot.
 * Puts radio into power save mode.
 */
void power_radio_slot_end(void);

/**
 * @brief Check if radio is currently enabled
 * @return true if radio is active
 */
bool power_is_radio_active(void);

/* ============================================================================
 * Activity Tracking
 * ============================================================================ */

/**
 * @brief Reset inactivity timer
 *
 * Called when any activity occurs (voice, mesh packet, etc.)
 * to prevent deep sleep timeout.
 */
void power_activity_ping(void);

/**
 * @brief Get time since last activity in milliseconds
 * @return Milliseconds since last activity
 */
uint32_t power_get_idle_time_ms(void);

/* ============================================================================
 * Statistics
 * ============================================================================ */

/**
 * @brief Get power statistics
 * @param stats Output statistics structure
 * @return ESP_OK on success
 */
esp_err_t power_get_stats(power_stats_t *stats);

/**
 * @brief Reset power statistics
 * @return ESP_OK on success
 */
esp_err_t power_reset_stats(void);

/* ============================================================================
 * Deep Sleep Control
 * ============================================================================ */

/**
 * @brief Enter deep sleep mode
 *
 * Device will wake after configured timeout via RTC timer.
 * This function does not return.
 *
 * @param wake_after_sec Seconds to sleep before RTC wake (0 = indefinite)
 */
void power_enter_deep_sleep(uint32_t wake_after_sec);

/**
 * @brief Check if waking from deep sleep
 * @return true if this boot was a deep sleep wake
 */
bool power_is_deep_sleep_wake(void);

#ifdef __cplusplus
}
#endif

#endif /* OMI_POWER_H */
