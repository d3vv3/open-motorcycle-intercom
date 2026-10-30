use openhelmet_mesh::{adaptive as a, speaker::Speakers};
#[test]
fn producer_lease_exact_expiry_refresh_clear_and_full_width() {
    let mut voice = openhelmet_mesh::speaker::LocalVoice::new();
    assert!(!voice.sample(0));
    let now = i64::from(u32::MAX) + 100;
    voice.offer(true, now);
    assert!(voice.sample(now + 1499));
    // Refresh even when the corresponding queue offer will fail.
    voice.offer(true, now + 1499);
    assert!(voice.sample(now + 2998));
    assert!(!voice.sample(now + 2999));
    assert!(!voice.sample(now)); // expiry is sticky
    voice.offer(true, now);
    voice.offer(false, now + 1);
    assert!(!voice.sample(now + 1));
    voice.offer(true, now);
    voice.clear();
    assert!(!voice.sample(now));
}
#[test]
fn producer_map_preserves_sparse_slots_rejects_removed_and_uses_owned_speakers() {
    let mut a = setup();
    let mut speakers = Speakers::default();
    let mut layout = openhelmet_mesh::core::SlotMap {
        slot_count: 6,
        slot_ids: [1, 0, 3, 0, 2, 4, 0, 0],
        // Copied projection speaker fields are not authority.
        active_speaker_count: 2,
        active_speaker_ids: [8, 7],
        relay_masks: [255, 255],
    };
    assert!(speakers.grant(&a, 1, [3, 0], [9, 0]));
    let map = speakers.produce_map(&a, &layout).unwrap();
    assert_eq!(map.slot_ids, [1, 0, 3, 0, 2, 4, 0, 0]);
    assert_eq!(map.slot_count, 6);
    assert_eq!(map.active_speaker_ids, [3, 0]);
    assert_eq!(map.relay_masks, [9, 0]);
    a::init(Some(&mut a), 1, 1, 8, 7, 0); // removed ID 4
    assert!(speakers.produce_map(&a, &layout).is_none());
    layout.slot_ids[5] = 0;
    layout.slot_count = 5;
    assert_eq!(
        speakers.produce_map(&a, &layout).unwrap().relay_masks,
        [1, 0]
    );
    layout.slot_ids[1] = 3; // duplicate
    assert!(speakers.produce_map(&a, &layout).is_none());
    layout.slot_ids[1] = 9; // out of domain, must not masquerade as empty
    assert!(speakers.produce_map(&a, &layout).is_none());
}
fn setup() -> a::Adaptive {
    let mut a = a::Adaptive::default();
    a::init(Some(&mut a), 1, 1, 7, 15, 0);
    for id in 1..=4 {
        a::set_member(
            Some(&mut a),
            id,
            Some(&a::Identity {
                address_len: 6,
                address: [id; 6],
            }),
        );
    }
    a
}
fn request(s: &mut Speakers, a: &mut a::Adaptive, id: u8, active: u8, seq: u32, now: u32) {
    assert!(a::speaker_request(
        Some(a),
        id,
        Some(&a::SpeakerRequest {
            term: 7,
            request_seq: seq,
            frame_counter: 1,
            active
        }),
        1,
        now
    ));
    s.request(id, active != 0, i64::from(now), true);
}
#[test]
fn sticky_compaction_expiry_audio_bridge_and_inactive() {
    let mut a = setup();
    let mut s = Speakers::default();
    request(&mut s, &mut a, 2, 1, 1, 10);
    request(&mut s, &mut a, 3, 1, 1, 11);
    assert_eq!(s.arbitrate(&mut a, 12).ids, [2, 3]);
    request(&mut s, &mut a, 1, 1, 1, 13);
    assert!(!s.arbitrate(&mut a, 14).changed);
    request(&mut s, &mut a, 2, 0, 2, 15);
    let e = s.arbitrate(&mut a, 16);
    assert_eq!(e.ids, [3, 1]);
    assert_eq!(e.removed, [2, 0]);
    s.audio(3, 1500);
    assert_eq!(s.arbitrate(&mut a, 1511).ids, [3, 1]);
    assert_eq!(s.arbitrate(&mut a, 1513).ids, [3, 1]); // Adaptive request equality is inclusive.
    assert_eq!(s.arbitrate(&mut a, 1514).ids, [3, 0]);
    assert_eq!(s.arbitrate(&mut a, 3000).ids, [0, 0]);
}
#[test]
fn oldest_then_id_tie_and_ungranted_audio_cannot_bootstrap() {
    let mut a = setup();
    let mut s = Speakers::default();
    request(&mut s, &mut a, 4, 1, 1, 20);
    request(&mut s, &mut a, 3, 1, 1, 10);
    request(&mut s, &mut a, 2, 1, 1, 10);
    assert_eq!(s.arbitrate(&mut a, 21).ids, [2, 3]);
    s.audio(4, 1600);
    assert_eq!(s.arbitrate(&mut a, 1521).ids, [0, 0]);
    assert!(s.grant(&a, 1, [4, 0], [0, 0]));
    assert_eq!(s.arbitrate(&mut a, 1601).ids, [4, 0]);
    request(&mut s, &mut a, 4, 0, 2, 1602);
    assert_eq!(s.arbitrate(&mut a, 1603).ids, [0, 0]);
}
#[test]
fn validation_atomic_release_compaction_and_mask_only_change() {
    let mut a = setup();
    let mut s = Speakers::default();
    assert!(s.grant(&a, 2, [2, 3], [1, 2]));
    assert!(!s.grant(&a, 1, [2, 3], [1, 0]));
    assert!(!s.release(2, [2, 2]));
    assert_eq!(s.projection().ids, [2, 3]);
    assert!(s.release(1, [2, 0]));
    assert_eq!(s.projection().ids, [3, 0]);
    assert_eq!(s.projection().masks, [2, 0]);
    assert!(!s.grant(&a, 1, [2, 0], [128, 0]));
    request(&mut s, &mut a, 3, 1, 1, 10);
    assert!(s.arbitrate(&mut a, 11).changed);
    assert!(!s.arbitrate(&mut a, 12).changed);
}
#[test]
fn grace_inclusive_zero_sentinel_and_snapshot_chronology() {
    let mut a = setup();
    let mut s = Speakers::default();
    request(&mut s, &mut a, 2, 1, 1, 0);
    assert_eq!(s.arbitrate(&mut a, 0).ids, [0, 0]);
    assert_eq!(s.arbitrate(&mut a, 1).ids, [2, 0]);
    assert!(s.grant(&a, 2, [3, 2], [0, 0]));
    s.handover(100);
    assert_eq!(s.arbitrate(&mut a, 1600).ids, [3, 2]);
    assert_eq!(s.arbitrate(&mut a, 1601).ids, [0, 0]);
    s.audio(4, 1602);
    assert_eq!(s.arbitrate(&mut a, 1603).ids, [0, 0]);
    s.clear_member(3);
    assert_eq!(s.projection().ids, [0, 0]);
}
