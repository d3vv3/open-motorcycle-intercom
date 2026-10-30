use openhelmet_mesh::{
    adaptive::{self as a, Adaptive, Identity},
    admission::{self, Input, Transaction},
    core::SlotMap,
};
#[test]
fn copied_projection_conflicts_and_commit_revalidation() {
    use openhelmet_mesh::peer_evidence::Row;
    let (mut s, input) = setup();
    let local = Row {
        active: 1,
        id: 3,
        mac: [3; 6],
        slot: 2,
        ..Default::default()
    };
    let mut good = input;
    good.peers[5] = local;
    assert_eq!(admission::plan(&s, good).unwrap().row, 0);
    for case in 0..7 {
        let mut bad = good;
        match case {
            0 => bad.peers[1] = local,
            1 => bad.peers[5].mac = [4; 6],
            2 => bad.peers[5].id = 1,
            3 => bad.peers[5].slot = 0,
            4 => bad.local_slot = -1,
            5 => {
                bad.peers[1] = Row {
                    active: 1,
                    id: 1,
                    mac: [4; 6],
                    slot: 0,
                    ..Default::default()
                }
            }
            _ => bad.peers[5].active = 2,
        }
        let mut t = Transaction::default();
        t.prepare(&s, good, 44).unwrap();
        let before = s.snapshot();
        assert!(admission::plan(&s, bad).is_none(), "case {case}");
        assert!(t.commit(&mut s, bad, 44).is_none());
        assert_eq!(s.snapshot(), before);
    }
}
fn identity(n: u8) -> Identity {
    Identity {
        address_len: 6,
        address: [n; 6],
    }
}
fn setup() -> (Adaptive, Input) {
    let mut s = Adaptive::default();
    a::init(Some(&mut s), 3, 3, 7, 4, 0);
    assert!(a::set_member(Some(&mut s), 3, Some(&identity(3))));
    (
        s,
        Input {
            generation: 9,
            active: true,
            source: 0,
            local_slot: 2,
            peers: Default::default(),
            origin: identity(4),
            target: identity(3),
            map: SlotMap {
                slot_count: 3,
                slot_ids: [0, 0, 3, 0, 0, 0, 0, 0],
                ..Default::default()
            },
        },
    )
}
#[test]
fn prepare_is_pure_commit_once_rejoin_preserves_history_and_lowest_holes() {
    let (mut s, input) = setup();
    let before = s.snapshot();
    let mut t = Transaction::default();
    let p = t.prepare(&s, input, 1).unwrap();
    assert_eq!((p.kind, p.id, p.slot), (2, 1, 0));
    assert_eq!(s.snapshot(), before);
    assert!(t.commit(&mut s, input, 1).is_some());
    assert!(t.commit(&mut s, input, 1).is_none());
    let mut input = input;
    input.map = p.map;
    let request = a::SpeakerRequest {
        term: 7,
        request_seq: 1,
        frame_counter: 20,
        active: 1,
    };
    assert!(a::speaker_request(Some(&mut s), 1, Some(&request), 20, 50));
    let header = a::Header {
        version: 5,
        packet_type: 9,
        src_id: 1,
        seq: 1,
        ttl: 2,
        payload_len: 5,
        ..Default::default()
    };
    assert!(a::forward_enqueue(
        Some(&mut s),
        Some(&header),
        Some(&[7, 0, 0, 0, 0]),
        None,
        7,
        true,
        50
    ));
    let before = s.snapshot();
    let p = t.prepare(&s, input, 2).unwrap();
    assert_eq!((p.kind, p.id, p.slot), (1, 1, 0));
    assert!(t.commit(&mut s, input, 2).is_some());
    assert_eq!(s.snapshot(), before);
    let mut invalid = input;
    invalid.source = 9;
    assert!(t.prepare(&s, invalid, 3).is_none());
    assert_eq!(s.snapshot(), before);
    t.prepare(&s, input, 4).unwrap();
    t.cancel();
    assert!(t.commit(&mut s, input, 4).is_none());
    assert_eq!(s.snapshot(), before);
}
#[test]
fn invalid_context_and_maps_are_noops() {
    let (s, input) = setup();
    let mut bad = Vec::new();
    let mut i = input;
    i.active = false;
    bad.push(i);
    for source in [1, 8, 9, 255] {
        let mut i = input;
        i.source = source;
        bad.push(i);
    }
    for origin in [
        Identity::default(),
        identity(255),
        identity(3),
        Identity {
            address_len: 5,
            address: [4; 6],
        },
    ] {
        let mut i = input;
        i.origin = origin;
        bad.push(i);
    }
    let mut i = input;
    i.target = identity(7);
    bad.push(i);
    for count in [0, 9, 255] {
        let mut i = input;
        i.map.slot_count = count;
        bad.push(i);
    }
    let mut i = input;
    i.map.slot_ids[0] = 3;
    bad.push(i);
    let mut i = input;
    i.map.slot_ids[2] = 0;
    bad.push(i);
    let mut i = input;
    i.map.slot_ids[7] = 1;
    bad.push(i);
    let mut i = input;
    i.map.slot_ids[0] = 9;
    bad.push(i);
    for i in bad {
        assert!(admission::plan(&s, i).is_none());
    }
}
#[test]
fn collisions_cancel_generation_and_changed_term_reject_without_rollback() {
    let (mut s, input) = setup();
    let mut t = Transaction::default();
    t.prepare(&s, input, 4).unwrap();
    let mut changed = input;
    changed.generation += 1;
    assert!(t.commit(&mut s, changed, 4).is_none());
    t.prepare(&s, input, 5).unwrap();
    t.cancel();
    assert!(t.commit(&mut s, input, 5).is_none());
    t.prepare(&s, input, 6).unwrap();
    assert!(a::set_member(Some(&mut s), 1, Some(&identity(8))));
    let before = s.snapshot();
    assert!(t.commit(&mut s, input, 6).is_none());
    assert_eq!(s.snapshot(), before);
    for (local, leader, term, members) in [(3, 3, 8, 4), (3, 1, 7, 5), (1, 1, 7, 1)] {
        let (mut s, input) = setup();
        t.prepare(&s, input, 7).unwrap();
        a::init(Some(&mut s), local, leader, term, members, 0);
        assert!(a::set_member(Some(&mut s), local, Some(&identity(local))));
        let before = s.snapshot();
        assert!(t.commit(&mut s, input, 7).is_none());
        assert_eq!(s.snapshot(), before);
    }
    for (local, leader, term, members) in [(3, 3, 0, 4), (3, 1, 7, 5), (0, 0, 0, 0)] {
        let (mut s, input) = setup();
        a::init(Some(&mut s), local, leader, term, members, 0);
        assert!(admission::plan(&s, input).is_none());
    }
}
#[test]
fn sparse_bindings_allocate_lowest_identity_and_slot_independently() {
    let (mut s, mut input) = setup();
    assert!(a::set_member(Some(&mut s), 1, Some(&identity(1))));
    assert!(a::set_member(Some(&mut s), 6, Some(&identity(6))));
    input.map = SlotMap {
        slot_count: 7,
        slot_ids: [6, 0, 3, 0, 0, 0, 1, 0],
        ..Default::default()
    };
    for (row, (id, slot)) in [(6, 0), (1, 6)].into_iter().enumerate() {
        input.peers[row] = openhelmet_mesh::peer_evidence::Row {
            active: 1,
            id,
            mac: [id; 6],
            slot,
            ..Default::default()
        };
    }
    let assignment = admission::plan(&s, input).unwrap();
    assert_eq!((assignment.id, assignment.slot), (2, 1));
    assert_eq!(assignment.map.slot_ids, [6, 2, 3, 0, 0, 0, 1, 0]);
}
#[test]
fn full_mesh_still_rejoins_and_rejects_new_identity() {
    let (mut s, mut input) = setup();
    for id in 1..=8 {
        assert!(a::set_member(Some(&mut s), id, Some(&identity(id))));
    }
    input.map = SlotMap {
        slot_count: 8,
        slot_ids: [1, 2, 3, 4, 5, 6, 7, 8],
        ..Default::default()
    };
    input.local_slot = 2;
    for id in 1..=8 {
        input.peers[usize::from(id - 1)] = openhelmet_mesh::peer_evidence::Row {
            active: 1,
            id,
            mac: [id; 6],
            slot: id as i8 - 1,
            ..Default::default()
        };
    }
    input.origin = identity(4);
    assert_eq!(admission::plan(&s, input).unwrap().kind, 1);
    input.origin = identity(9);
    assert!(admission::plan(&s, input).is_none());
}
