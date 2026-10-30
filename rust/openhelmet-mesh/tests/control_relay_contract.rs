use openhelmet_mesh::control_relay::{self as relay, Decision::*};

#[test]
fn admission_order_and_wrapping_cadence() {
    assert_eq!(relay::receive(false, 0, true), Inactive);
    assert_eq!(relay::receive(true, 0, true), LocalMissing);
    assert_eq!(relay::receive(true, 1, true), AlreadyRelayed);
    assert_eq!(relay::receive(true, 1, false), Send);
    for last in [1u32, 1234, u32::MAX - 100] {
        assert_eq!(relay::cadence(true, last.wrapping_add(549), last), Cadence);
        assert_eq!(relay::cadence(true, last.wrapping_add(550), last), Send);
        assert_eq!(relay::cadence(false, last, last), Send);
    }
    assert_eq!(relay::cadence(true, 0, 0), Send);
    assert_eq!(relay::upstream(false, 600), Send);
    assert_eq!(relay::upstream(false, 601), UpstreamStale);
    assert_eq!(relay::upstream(true, u32::MAX), Send);
}

#[test]
fn current_authority_then_strict_signed_modular_age() {
    assert_eq!(relay::authority(true, 7, 2, 7, 2), Send);
    assert_eq!(relay::authority(true, 7, 2, 8, 2), AuthorityChanged);
    assert_eq!(relay::authority(true, 7, 2, 7, 3), AuthorityChanged);
    assert_eq!(relay::authority(false, 7, 2, 8, 3), Send);
    for (sync, lifecycle, limit) in [
        (true, true, 600_000i64),
        (false, true, 2_400_000),
        (false, false, 1_000_000),
    ] {
        for queued in [0i64, i64::MAX - 100, i64::MIN + 100] {
            for delta in [-1, 0, limit, limit + 1] {
                assert_eq!(
                    relay::queued_age(true, sync, lifecycle, queued.wrapping_add(delta), queued),
                    if delta > limit { Expired } else { Send }
                );
                assert_eq!(
                    relay::queued_age(false, sync, lifecycle, queued.wrapping_add(delta), queued),
                    Send
                );
            }
        }
    }
}
