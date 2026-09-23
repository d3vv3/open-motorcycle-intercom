/**
 * @file phone_audio.h
 * @brief Bluetooth Classic phone audio sink and media-control interface.
 */
#ifndef OMI_PHONE_AUDIO_H
#define OMI_PHONE_AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "phone_audio_call_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialized;
    bool terminal;
    bool a2dp_connected;
    /** Remote A2DP STARTED/SUSPENDED state, independent of local route application. */
    bool media_streaming;
    /** Monotonic time of the most recent remote A2DP state transition, in ms. */
    int64_t media_transition_ms;
    bool a2dp_audio_active;
    bool avrcp_connected;
    bool discoverable;
    bool pairing_window_open;
    bool volume_control_limited;
    uint32_t sample_rate;
    uint8_t channels;
    phone_audio_call_state_t call;
} phone_audio_state_t;

typedef struct {
    uint32_t incoming_frames;
    uint32_t incoming_drops;
    uint32_t outgoing_callbacks;
    uint32_t padded_samples;
    uint32_t ready_notifications;
    uint32_t pending_timeouts;
    uint32_t expected_samples;
    bool slc_connected;
    bool sco_active;
    bool outgoing_ready_pending;
} phone_audio_stats_t;

/** Initialize the terminal Bluetooth Classic phone-audio service. */
esp_err_t phone_audio_init(void);
/** Permanently tear down Bluetooth for this boot; it cannot be initialized again. */
esp_err_t phone_audio_deinit(void);
/** Copy a callback-safe snapshot; media_streaming is remote stream state, not applied routing. */
esp_err_t phone_audio_get_state(phone_audio_state_t *state);
/**
 * Destructively forget every Bluetooth Classic phone bond for replacement-phone use.
 * Fails while a phone is connected and does not erase unrelated NVS data.
 */
esp_err_t phone_audio_forget_all_bonds(void);
/** Change general discoverability while retaining connectability. */
/* Application UI must call this API to open replacement-phone pairing. */
esp_err_t phone_audio_set_discoverable(bool discoverable);
/** Emit the low-priority HFP audio statistics snapshot. */
void phone_audio_log_stats(void);

esp_err_t phone_audio_play(void);
esp_err_t phone_audio_pause(void);
esp_err_t phone_audio_stop(void);
esp_err_t phone_audio_next(void);
esp_err_t phone_audio_previous(void);

esp_err_t phone_audio_get_call_state(phone_audio_call_state_t *state);
esp_err_t phone_audio_answer_call(void);
esp_err_t phone_audio_reject_call(void);
esp_err_t phone_audio_dial(const char *number);
esp_err_t phone_audio_redial(void);
esp_err_t phone_audio_connect_audio(void);
esp_err_t phone_audio_disconnect_audio(void);

#ifdef __cplusplus
}
#endif

#endif /* OMI_PHONE_AUDIO_H */
