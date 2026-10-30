use openhelmet_mesh::{
    adaptive::{self, Adaptive},
    core::SlotMap,
};

#[test]
fn participant_map_uses_current_members_and_canonical_adaptive_slots() {
    let mut owner = Adaptive::default();
    adaptive::init(Some(&mut owner), 2, 1, 7, 7, 0);
    let map = SlotMap {
        slot_count: 4,
        slot_ids: [2, 0, 3, 1, 0, 0, 0, 0],
        active_speaker_count: 2,
        active_speaker_ids: [2, 3],
        relay_masks: [5, 3],
    };
    assert_eq!(adaptive::participant_map(&owner, true, 1, &map), Some(0));
    assert_eq!(adaptive::participant_map(&owner, false, 1, &map), None);
    assert_eq!(adaptive::participant_map(&owner, true, 3, &map), None);
    for index in 0..14 {
        let mut bad = map;
        match index {
            0 => bad.slot_count = 0,
            1 => bad.slot_count = 9,
            2 => bad.slot_ids[1] = 2,
            3 => bad.slot_ids[2] = 9,
            4 => bad.slot_ids[0] = 0,
            5 => bad.slot_ids[3] = 0,
            6 => bad.slot_ids[1] = 4,
            7 => bad.slot_ids[7] = 4,
            8 => bad.active_speaker_count = 3,
            9 => bad.active_speaker_ids[0] = 4,
            10 => bad.active_speaker_ids[1] = 2,
            11 => bad.active_speaker_count = 1,
            12 => bad.relay_masks[0] = 8,
            _ => {
                bad.active_speaker_count = 1;
                bad.active_speaker_ids[1] = 0;
            }
        }
        assert_eq!(
            adaptive::participant_map(&owner, true, 1, &bad),
            None,
            "boundary {index}"
        );
    }
    let missing_member = SlotMap {
        slot_count: 2,
        slot_ids: [2, 1, 0, 0, 0, 0, 0, 0],
        ..Default::default()
    };
    assert_eq!(
        adaptive::participant_map(&owner, true, 1, &missing_member),
        None
    );
    adaptive::init(Some(&mut owner), 2, 1, 7, 3, 0);
    assert_eq!(adaptive::participant_map(&owner, true, 1, &map), None);
    adaptive::init(Some(&mut owner), 1, 1, 7, 7, 0);
    assert_eq!(adaptive::participant_map(&owner, true, 1, &map), None);
}
