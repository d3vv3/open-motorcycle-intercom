use openhelmet_mesh::{adaptive::EXPIRE_MS, core::*};

#[test]
fn legacy_recovers_before_half_window_after_2560_to_3000ms_outage() {
    for outage in (2560..3000).step_by(20) {
        let mut d = Dedupe::default();
        assert!(audio_replay_accept(Some(&mut d), 2, 0, true, 0));
        let seq = (outage / 20) as u16;
        assert!(audio_replay_accept(Some(&mut d), 2, seq, true, outage));
        assert!(!audio_replay_accept(Some(&mut d), 2, seq, true, outage + 1));
    }
}
#[test]
fn audio_replay_survives_tuple_eviction_and_other_sources() {
    let mut d = Dedupe::default();
    assert!(audio_replay_accept(Some(&mut d), 2, 0, false, 0));
    for seq in 1..=64 {
        assert!(audio_replay_accept(Some(&mut d), 2, seq, false, seq as u32));
        for src in 3..=8 {
            assert!(audio_replay_accept(
                Some(&mut d),
                src,
                seq,
                false,
                seq as u32
            ));
        }
        dedupe_accept(Some(&mut d), 13, 2, seq as u8);
    }
    assert!(!audio_replay_accept(Some(&mut d), 2, 0, false, 65));
    assert!(!audio_replay_accept(Some(&mut d), 2, 64, false, 65));
    // A relay-first copy uses the origin, not the transmitting neighbor.
    let mut receiver = Dedupe::default();
    assert!(audio_replay_accept(Some(&mut receiver), 2, 64, false, 65));
    assert!(!audio_replay_accept(Some(&mut receiver), 2, 64, false, 66));
}

#[test]
fn restart_requires_reset_or_existing_expiry_and_replays_do_not_extend_it() {
    for (legacy, initial) in [(false, 1000), (true, 100)] {
        let expiry = if legacy { 2500 } else { EXPIRE_MS };
        let mut d = Dedupe::default();
        let start = u32::MAX - 100;
        assert!(audio_replay_accept(Some(&mut d), 2, initial, legacy, start));
        for age in [1, expiry - 1, expiry] {
            assert!(!audio_replay_accept(
                Some(&mut d),
                2,
                0,
                legacy,
                start.wrapping_add(age)
            ));
        }
        assert!(audio_replay_accept(
            Some(&mut d),
            2,
            0,
            legacy,
            start.wrapping_add(expiry + 1)
        ));
        assert!(audio_replay_accept(
            Some(&mut d),
            2,
            1,
            legacy,
            start.wrapping_add(expiry + 2)
        ));
        dedupe_purge_node(Some(&mut d), 2);
        assert!(audio_replay_accept(
            Some(&mut d),
            2,
            0,
            legacy,
            start.wrapping_add(expiry + 3)
        ));
        dedupe_reset(Some(&mut d));
        assert!(audio_replay_accept(Some(&mut d), 2, 0, legacy, 0));
    }
    let mut d = Dedupe::default();
    assert!(audio_replay_accept(Some(&mut d), 2, 40000, false, 0));
    assert!(audio_replay_accept(Some(&mut d), 2, 0, false, 1));
}

#[test]
fn old_observations_do_not_rewind_sequence_or_fabricate_loss() {
    let mut byte = Seq8::default();
    let mut wide = Seq16::default();
    for (seq, class) in [
        (100, SeqClass::First),
        (99, SeqClass::OldReset),
        (101, SeqClass::InOrder),
    ] {
        assert_eq!(
            seq8_accept(Some(&mut byte), seq as u8),
            SeqResult {
                classification: class,
                gap: 0
            }
        );
        assert_eq!(
            seq16_accept(Some(&mut wide), seq),
            SeqResult {
                classification: class,
                gap: 0
            }
        );
    }
    seq8_reset(Some(&mut byte));
    seq16_reset(Some(&mut wide));
    assert_eq!(
        seq8_accept(Some(&mut byte), 0).classification,
        SeqClass::First
    );
    assert_eq!(
        seq16_accept(Some(&mut wide), 0).classification,
        SeqClass::First
    );
}
