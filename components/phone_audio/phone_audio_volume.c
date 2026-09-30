#include "phone_audio_volume.h"

static uint8_t max_wire(phone_volume_profile_t profile)
{
    return profile == PHONE_VOLUME_MEDIA ? 127u : 15u;
}

uint8_t phone_volume_to_wire(uint8_t percent, phone_volume_profile_t profile)
{
    return (uint8_t)((percent * (unsigned)max_wire(profile) + 50u) / 100u);
}

uint8_t phone_volume_from_wire(uint8_t wire, phone_volume_profile_t profile)
{
    return (uint8_t)((wire * 100u + max_wire(profile) / 2u) / max_wire(profile));
}

void phone_volume_init(phone_volume_policy_t *policy, uint8_t fallback)
{
    *policy = (phone_volume_policy_t){0};
    policy->percent = fallback;
    policy->media_wire = phone_volume_to_wire(fallback, PHONE_VOLUME_MEDIA);
    policy->call_wire = phone_volume_to_wire(fallback, PHONE_VOLUME_CALL);
}

void phone_volume_disconnect(phone_volume_policy_t *policy)
{
    if (policy->profile == PHONE_VOLUME_CALL) {
        policy->percent = phone_volume_from_wire(policy->media_wire, PHONE_VOLUME_MEDIA);
    } else {
        policy->media_wire = phone_volume_to_wire(policy->percent, PHONE_VOLUME_MEDIA);
    }
    policy->call_wire = phone_volume_to_wire(policy->percent, PHONE_VOLUME_CALL);
    policy->session++;
    policy->revision++;
    policy->registration++;
    policy->media_known = false;
    policy->call_known = false;
    policy->rn = PHONE_VOLUME_RN_NONE;
    policy->rn_changed = false;
    policy->call_pending = false;
    policy->media_revision++;
    policy->call_revision++;
    policy->media_local = false;
    policy->profile = PHONE_VOLUME_MEDIA;
}

void phone_volume_invalidate_session(phone_volume_policy_t *policy)
{
    policy->session++;
    policy->registration++;
    policy->media_revision++;
    policy->call_revision++;
    policy->rn = PHONE_VOLUME_RN_NONE;
    policy->rn_changed = false;
    policy->call_pending = false;
    policy->media_local = false;
}

void phone_volume_profile(phone_volume_policy_t *policy, phone_volume_profile_t profile)
{
    if (policy->profile == profile) return;
    if (policy->profile == PHONE_VOLUME_CALL && policy->call_pending) {
        policy->call_pending = false;
        policy->call_revision++;
    }
    policy->profile = profile;
    policy->percent = profile == PHONE_VOLUME_MEDIA
        ? phone_volume_from_wire(policy->media_wire, profile)
        : (policy->call_known ? phone_volume_from_wire(policy->call_wire, profile) : policy->percent);
    policy->revision++;
}

bool phone_volume_local(phone_volume_policy_t *policy, uint8_t percent)
{
    if (percent > 100u) return false;
    phone_volume_profile_t profile = policy->profile;
    uint8_t wire = phone_volume_to_wire(percent, profile);
    uint8_t *cached = profile == PHONE_VOLUME_MEDIA ? &policy->media_wire : &policy->call_wire;
    bool changed = *cached != wire ||
                   !(profile == PHONE_VOLUME_MEDIA ? policy->media_known : policy->call_known);
    *cached = wire;
    if (profile == PHONE_VOLUME_MEDIA) {
        policy->media_known = true;
        if (changed) {
            policy->media_revision++;
            policy->media_local = true;
        }
        if (changed && policy->rn == PHONE_VOLUME_RN_ACTIVE) policy->rn_changed = true;
    } else {
        policy->call_known = true;
        if (changed) {
            policy->call_revision++;
            policy->call_pending = true;
        }
    }
    policy->percent = phone_volume_from_wire(wire, profile);
    policy->revision++;
    return true;
}

bool phone_volume_remote(phone_volume_policy_t *policy, phone_volume_profile_t profile, uint8_t wire)
{
    if (wire > max_wire(profile)) return false;
    if (profile == PHONE_VOLUME_MEDIA) {
        policy->media_wire = wire;
        policy->media_known = true;
        policy->rn_changed = false;
        policy->media_local = false;
        policy->media_revision++;
    } else {
        policy->call_wire = wire;
        policy->call_known = true;
        policy->call_pending = false;
        policy->call_revision++;
    }
    if (policy->profile == profile) {
        policy->percent = phone_volume_from_wire(wire, profile);
        policy->revision++;
    }
    return true;
}

void phone_volume_register(phone_volume_policy_t *policy)
{
    policy->registration++;
    policy->rn = PHONE_VOLUME_RN_INTERIM;
    policy->rn_changed = false;
}

phone_volume_tx_t phone_volume_rn_tx(const phone_volume_policy_t *policy, uint32_t peer_epoch)
{
    return (phone_volume_tx_t){.session = policy->session, .peer_epoch = peer_epoch,
        .registration = policy->registration, .revision = policy->media_revision,
        .wire = policy->media_wire, .profile = PHONE_VOLUME_MEDIA, .rn = policy->rn};
}

phone_volume_tx_t phone_volume_call_tx(const phone_volume_policy_t *policy, uint32_t peer_epoch)
{
    return (phone_volume_tx_t){.session = policy->session, .peer_epoch = peer_epoch,
        .revision = policy->call_revision, .wire = policy->call_wire,
        .profile = PHONE_VOLUME_CALL};
}

bool phone_volume_rn_eligible(const phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                              uint32_t peer_epoch)
{
    if (tx->profile != PHONE_VOLUME_MEDIA || tx->peer_epoch != peer_epoch ||
        policy->session != tx->session || policy->registration != tx->registration ||
        policy->rn != tx->rn || policy->media_revision != tx->revision ||
        policy->media_wire != tx->wire) return false;
    if (tx->rn == PHONE_VOLUME_RN_INTERIM) return true;
    return tx->rn == PHONE_VOLUME_RN_ACTIVE && policy->profile == PHONE_VOLUME_MEDIA &&
           policy->rn_changed && policy->media_local;
}

bool phone_volume_call_eligible(const phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                                uint32_t peer_epoch)
{
    return tx->profile == PHONE_VOLUME_CALL && tx->peer_epoch == peer_epoch &&
           policy->session == tx->session && policy->profile == PHONE_VOLUME_CALL &&
           policy->call_pending && policy->call_revision == tx->revision &&
           policy->call_wire == tx->wire;
}

void phone_volume_rn_result(phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                            uint32_t peer_epoch, bool ok)
{
    if (!ok || tx->peer_epoch != peer_epoch || policy->session != tx->session ||
        policy->registration != tx->registration || policy->rn != tx->rn) return;
    if (tx->rn == PHONE_VOLUME_RN_INTERIM) {
        policy->rn = PHONE_VOLUME_RN_ACTIVE;
        policy->rn_changed = policy->media_local && policy->media_revision != tx->revision &&
                             policy->media_wire != tx->wire;
    } else if (tx->rn == PHONE_VOLUME_RN_ACTIVE) {
        /* A queued CHANGED consumes this registration, even when a newer local level exists. */
        policy->rn = PHONE_VOLUME_RN_NONE;
        policy->rn_changed = false;
    }
}

void phone_volume_call_result(phone_volume_policy_t *policy, const phone_volume_tx_t *tx,
                              uint32_t peer_epoch, bool ok)
{
    if (ok && tx->peer_epoch == peer_epoch && policy->session == tx->session &&
        policy->call_revision == tx->revision && policy->call_wire == tx->wire)
        policy->call_pending = false;
}
