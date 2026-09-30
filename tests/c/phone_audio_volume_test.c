#include <assert.h>
#include <stdint.h>

#include "phone_audio_volume.h"
#include "channel_control.h"

static void test_mapping_and_profiles(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    assert(phone_volume_to_wire(0, PHONE_VOLUME_MEDIA) == 0);
    assert(phone_volume_to_wire(100, PHONE_VOLUME_MEDIA) == 127);
    assert(phone_volume_to_wire(100, PHONE_VOLUME_CALL) == 15);
    for (unsigned i = 0; i <= 127; ++i) {
        uint8_t roundtrip = phone_volume_to_wire(phone_volume_from_wire(i, PHONE_VOLUME_MEDIA),
                                                  PHONE_VOLUME_MEDIA);
        assert(roundtrip >= i - (i != 0) && roundtrip <= i + 1);
    }
    for (unsigned i = 0; i <= 15; ++i)
        assert(phone_volume_to_wire(phone_volume_from_wire(i, PHONE_VOLUME_CALL), PHONE_VOLUME_CALL) == i);
    assert(!phone_volume_local(&p, 101));
    assert(!phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 128));
    assert(!phone_volume_remote(&p, PHONE_VOLUME_CALL, 16));
    assert(p.percent == 60);

    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 70);
    uint8_t music = p.percent;
    phone_volume_remote(&p, PHONE_VOLUME_CALL, 3);
    assert(p.percent == music && !p.call_pending && !p.rn_changed);
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(p.percent == 20);
    phone_volume_local(&p, 98);
    assert(p.percent == 100 && mesh_volume_at_limit(20, p.percent, 1));
    assert(p.call_wire == 15 && p.call_pending);
    phone_volume_remote(&p, PHONE_VOLUME_CALL, 6);
    assert(p.percent == 40 && !p.call_pending);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 45);
    assert(p.percent == 40);
    phone_volume_profile(&p, PHONE_VOLUME_MEDIA);
    assert(p.percent == phone_volume_from_wire(45, PHONE_VOLUME_MEDIA));
    phone_volume_disconnect(&p);
    assert(!p.media_known && !p.call_known && !p.call_pending);
    assert(p.media_wire == phone_volume_to_wire(p.percent, PHONE_VOLUME_MEDIA));
    assert(p.call_wire == phone_volume_to_wire(p.percent, PHONE_VOLUME_CALL));
}

static void test_interim_origin(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    phone_volume_register(&p);
    phone_volume_local(&p, 0);
    phone_volume_tx_t tx = phone_volume_rn_tx(&p, 7);
    assert(tx.wire == 0 && phone_volume_rn_eligible(&p, &tx, 7));
    phone_volume_rn_result(&p, &tx, 7, false);
    assert(p.rn == PHONE_VOLUME_RN_INTERIM);
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(p.rn == PHONE_VOLUME_RN_ACTIVE && !p.rn_changed);

    phone_volume_register(&p);
    tx = phone_volume_rn_tx(&p, 7);
    phone_volume_local(&p, 100);
    assert(!phone_volume_rn_eligible(&p, &tx, 7));
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(p.rn_changed);

    phone_volume_register(&p);
    tx = phone_volume_rn_tx(&p, 7);
    phone_volume_local(&p, 20);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 40);
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(p.rn == PHONE_VOLUME_RN_ACTIVE && !p.rn_changed);

    phone_volume_register(&p);
    tx = phone_volume_rn_tx(&p, 7);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 55);
    phone_volume_local(&p, 70);
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(p.rn_changed);

    phone_volume_register(&p);
    tx = phone_volume_rn_tx(&p, 7);
    phone_volume_local(&p, 30);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 35);
    phone_volume_local(&p, 90);
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(p.rn_changed);

    phone_volume_register(&p);
    tx = phone_volume_rn_tx(&p, 7);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, tx.wire);
    phone_volume_rn_result(&p, &tx, 7, true);
    assert(!p.rn_changed);
}

static void test_changed_one_shot_and_stale_tokens(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    phone_volume_register(&p);
    phone_volume_tx_t tx = phone_volume_rn_tx(&p, 8);
    phone_volume_rn_result(&p, &tx, 8, true);
    phone_volume_local(&p, 30);
    tx = phone_volume_rn_tx(&p, 8);
    assert(phone_volume_rn_eligible(&p, &tx, 8));
    phone_volume_local(&p, 40);
    assert(!phone_volume_rn_eligible(&p, &tx, 8));
    phone_volume_rn_result(&p, &tx, 8, false);
    assert(p.rn_changed);
    tx = phone_volume_rn_tx(&p, 8);
    assert(phone_volume_rn_eligible(&p, &tx, 8));
    phone_volume_local(&p, 50); /* SDK queued CHANGED for 40 before the next local tap. */
    phone_volume_rn_result(&p, &tx, 8, true);
    assert(p.rn == PHONE_VOLUME_RN_NONE && !p.rn_changed);
    assert(p.media_wire == phone_volume_to_wire(50, PHONE_VOLUME_MEDIA));
    assert(!phone_volume_rn_eligible(&p, &tx, 8));
    phone_volume_register(&p);
    assert(phone_volume_rn_tx(&p, 8).wire == phone_volume_to_wire(50, PHONE_VOLUME_MEDIA));
    phone_volume_rn_result(&p, &tx, 8, true);
    assert(p.rn == PHONE_VOLUME_RN_INTERIM);

    phone_volume_tx_t interim = phone_volume_rn_tx(&p, 8);
    phone_volume_rn_result(&p, &interim, 8, true);
    phone_volume_local(&p, 60);
    tx = phone_volume_rn_tx(&p, 8);
    phone_volume_register(&p); /* Re-registration races the previous CHANGED completion. */
    phone_volume_rn_result(&p, &tx, 8, true);
    assert(p.rn == PHONE_VOLUME_RN_INTERIM);

    tx = phone_volume_rn_tx(&p, 8);
    assert(!phone_volume_rn_eligible(&p, &tx, 9));
    phone_volume_rn_result(&p, &tx, 9, true);
    assert(p.rn == PHONE_VOLUME_RN_INTERIM);
    phone_volume_disconnect(&p);
    phone_volume_register(&p);
    phone_volume_rn_result(&p, &tx, 8, true);
    assert(p.rn == PHONE_VOLUME_RN_INTERIM);
}

static void test_call_transactions(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    phone_volume_local(&p, 98);
    assert(p.percent == 100 && mesh_volume_at_limit(60, p.percent, 1));
    phone_volume_tx_t tx = phone_volume_call_tx(&p, 12);
    assert(phone_volume_call_eligible(&p, &tx, 12));
    phone_volume_remote(&p, PHONE_VOLUME_CALL, 6);
    assert(!phone_volume_call_eligible(&p, &tx, 12));
    phone_volume_call_result(&p, &tx, 12, true);
    assert(p.percent == 40 && !p.call_pending);

    phone_volume_local(&p, 95);
    tx = phone_volume_call_tx(&p, 12);
    phone_volume_remote(&p, PHONE_VOLUME_MEDIA, 30);
    assert(p.call_revision == tx.revision);
    assert(phone_volume_call_eligible(&p, &tx, 12));
    phone_volume_call_result(&p, &tx, 12, false);
    assert(p.call_pending);
    phone_volume_local(&p, 85);
    assert(!phone_volume_call_eligible(&p, &tx, 12));
    tx = phone_volume_call_tx(&p, 12);
    assert(!phone_volume_call_eligible(&p, &tx, 13));
    phone_volume_call_result(&p, &tx, 13, true);
    assert(p.call_pending);
    phone_volume_call_result(&p, &tx, 12, true);
    assert(!p.call_pending);
    phone_volume_local(&p, 100);
    tx = phone_volume_call_tx(&p, 12);
    phone_volume_profile(&p, PHONE_VOLUME_MEDIA);
    assert(!p.call_pending && !phone_volume_call_eligible(&p, &tx, 12));
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(p.percent == 100 && !p.call_pending);
    phone_volume_local(&p, 80);
    tx = phone_volume_call_tx(&p, 12);
    phone_volume_disconnect(&p);
    assert(!phone_volume_call_eligible(&p, &tx, 12));
    phone_volume_call_result(&p, &tx, 12, true);
    assert(!p.call_pending);
}

static void test_media_baseline_after_call(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    assert(phone_volume_local(&p, 30));
    uint8_t media = p.percent;
    uint8_t media_wire = p.media_wire;
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(phone_volume_local(&p, 80));
    phone_volume_disconnect(&p);
    assert(p.profile == PHONE_VOLUME_MEDIA && p.percent == media);
    assert(p.media_wire == media_wire && !p.media_known);

    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(phone_volume_local(&p, 80));
    phone_volume_profile(&p, PHONE_VOLUME_MEDIA);
    assert(p.percent == media && p.media_wire == media_wire);

    phone_volume_init(&p, 30);
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(phone_volume_local(&p, 80));
    phone_volume_disconnect(&p);
    assert(p.percent == phone_volume_from_wire(phone_volume_to_wire(30, PHONE_VOLUME_MEDIA),
                                                PHONE_VOLUME_MEDIA));
    assert(p.media_wire == phone_volume_to_wire(30, PHONE_VOLUME_MEDIA));

    assert(phone_volume_local(&p, 42));
    media = p.percent;
    phone_volume_disconnect(&p);
    assert(p.percent == media && p.media_wire == phone_volume_to_wire(media, PHONE_VOLUME_MEDIA));
}

static void test_a2dp_loss_during_call(void)
{
    phone_volume_policy_t p;
    phone_volume_init(&p, 60);
    assert(phone_volume_local(&p, 30));
    uint8_t media_wire = p.media_wire;
    phone_volume_profile(&p, PHONE_VOLUME_CALL);
    assert(phone_volume_local(&p, 80));
    phone_volume_register(&p);
    phone_volume_tx_t media_tx = phone_volume_rn_tx(&p, 4);
    phone_volume_tx_t call_tx = phone_volume_call_tx(&p, 8);
    uint32_t previous_session = p.session;
    uint32_t previous_registration = p.registration;
    uint8_t call_wire = p.call_wire;

    phone_volume_invalidate_session(&p);
    assert(p.session == previous_session + 1 && p.registration == previous_registration + 1);
    assert(p.profile == PHONE_VOLUME_CALL && p.percent == 80);
    assert(p.media_known && p.call_known && p.media_wire == media_wire && p.call_wire == call_wire);
    assert(!p.call_pending && p.rn == PHONE_VOLUME_RN_NONE);
    assert(!phone_volume_call_eligible(&p, &call_tx, 8));
    phone_volume_rn_result(&p, &media_tx, 4, true);
    phone_volume_call_result(&p, &call_tx, 8, true);
    assert(!p.call_pending && p.rn == PHONE_VOLUME_RN_NONE);

    assert(phone_volume_local(&p, 87));
    assert(p.call_pending && p.media_wire == media_wire && p.call_wire != call_wire);
    phone_volume_disconnect(&p);
    assert(p.profile == PHONE_VOLUME_MEDIA && p.percent == 30 && p.media_wire == media_wire);
    assert(!p.call_pending);
}

int main(void)
{
    test_mapping_and_profiles();
    test_interim_origin();
    test_changed_one_shot_and_stale_tokens();
    test_call_transactions();
    test_media_baseline_after_call();
    test_a2dp_loss_during_call();
    return 0;
}
