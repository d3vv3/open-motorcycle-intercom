use openhelmet_mesh::maintenance::{Cadence, Stage};
#[test]
fn signed_boundaries_sampled_attempts_and_no_catchup() {
    let mut c = Cadence::default();
    for now in [-1, 499_999, 500_000] {
        assert!(!c.poll(Stage::Keepalive, now));
    }
    assert!(c.poll(Stage::Keepalive, 500_001));
    assert!(!c.poll(Stage::Keepalive, 1_000_001));
    assert!(c.poll(Stage::Keepalive, 9_000_000));
    assert!(!c.poll(Stage::Keepalive, 9_000_000));
    assert!(!c.poll(Stage::Status, 1_000_000));
    assert!(c.poll(Stage::Status, 1_000_001));
    assert!(!c.poll(Stage::Status, 2_000_001));
}
#[test]
fn membership_success_only_fresh_time_and_wrap() {
    let mut c = Cadence::default();
    assert!(!c.poll(Stage::Membership, 999_999));
    assert!(c.poll(Stage::Membership, 1_000_000));
    assert!(c.poll(Stage::Membership, 1_000_000)); // failed send
    c.membership_success(1500);
    assert!(!c.poll(Stage::Membership, 2_499_999));
    assert!(c.poll(Stage::Membership, 2_500_000));
    c.membership_success(u32::MAX - 499);
    assert!(!c.poll(Stage::Membership, 499_000));
    assert!(c.poll(Stage::Membership, 500_000));
    c.reset_membership();
    assert!(c.poll(Stage::Membership, 1_000_000));
}
