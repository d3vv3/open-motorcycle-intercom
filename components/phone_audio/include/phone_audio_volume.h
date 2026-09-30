#ifndef OMI_PHONE_AUDIO_VOLUME_H
#define OMI_PHONE_AUDIO_VOLUME_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { PHONE_VOLUME_MEDIA, PHONE_VOLUME_CALL } phone_volume_profile_t;
typedef enum { PHONE_VOLUME_RN_NONE, PHONE_VOLUME_RN_INTERIM, PHONE_VOLUME_RN_ACTIVE } phone_volume_rn_t;

typedef struct {
    uint8_t percent;
    uint8_t media_wire;
    uint8_t call_wire;
    bool media_known;
    bool call_known;
    phone_volume_profile_t profile;
    phone_volume_rn_t rn;
    bool rn_changed;
    bool call_pending;
    uint32_t revision;
    uint32_t session;
    uint32_t registration;
    uint32_t call_revision;
    uint32_t media_revision;
    bool media_local;
} phone_volume_policy_t;

typedef struct {
    uint32_t session;
    uint32_t peer_epoch;
    uint32_t registration;
    uint32_t revision;
    uint8_t wire;
    phone_volume_profile_t profile;
    phone_volume_rn_t rn;
} phone_volume_tx_t;

uint8_t phone_volume_to_wire(uint8_t percent, phone_volume_profile_t profile);
uint8_t phone_volume_from_wire(uint8_t wire, phone_volume_profile_t profile);
void phone_volume_init(phone_volume_policy_t *policy, uint8_t fallback);
void phone_volume_disconnect(phone_volume_policy_t *policy);
void phone_volume_invalidate_session(phone_volume_policy_t *policy);
void phone_volume_profile(phone_volume_policy_t *policy, phone_volume_profile_t profile);
bool phone_volume_local(phone_volume_policy_t *policy, uint8_t percent);
bool phone_volume_remote(phone_volume_policy_t *policy, phone_volume_profile_t profile, uint8_t wire);
void phone_volume_register(phone_volume_policy_t *policy);
phone_volume_tx_t phone_volume_rn_tx(const phone_volume_policy_t *policy, uint32_t peer_epoch);
phone_volume_tx_t phone_volume_call_tx(const phone_volume_policy_t *policy, uint32_t peer_epoch);
bool phone_volume_rn_eligible(const phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                              uint32_t peer_epoch);
bool phone_volume_call_eligible(const phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                                uint32_t peer_epoch);
void phone_volume_rn_result(phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                            uint32_t peer_epoch, bool ok);
void phone_volume_call_result(phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                              uint32_t peer_epoch, bool ok);

#endif
