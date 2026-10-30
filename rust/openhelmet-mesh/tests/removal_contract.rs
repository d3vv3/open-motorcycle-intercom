use openhelmet_mesh::{
    adaptive::{self as a, Adaptive, Identity},
    core::SlotMap,
    removal::{self, Input, Transaction},
};
#[test]
fn copied_rows_reject_alias_duplicates_and_fresh_commit_observation() {
    let (mut s, input) = setup();
    assert_eq!(removal::plan(&s, input).unwrap().row, 4);
    for case in 0..5 {
        let mut bad = input;
        match case {
            0 => bad.peers[0] = bad.peers[4],
            1 => bad.peers[4].mac = [8; 6],
            2 => bad.peers[4].id = 6,
            3 => bad.peers[4].seen = 101i64.to_le_bytes(),
            _ => bad.peers[4].active = 0,
        }
        let mut t = Transaction::default();
        t.prepare(&s, input, 19).unwrap();
        let before = s.snapshot();
        assert!(removal::plan(&s, bad).is_none(), "case {case}");
        assert!(t.commit(&mut s, bad, 19).is_none());
        assert_eq!(s.snapshot(), before);
    }
    let mut leave = input;
    leave.reason = 1;
    leave.peers = Default::default();
    assert_eq!(removal::plan(&s, leave).unwrap().row, 255);
}
fn identity(id: u8) -> Identity {
    Identity {
        address_len: 6,
        address: [id; 6],
    }
}
fn setup() -> (Adaptive, Input) {
    let mut s = Adaptive::default();
    a::init(Some(&mut s), 3, 3, 7, 4, 0);
    for id in [1, 3, 6] {
        assert!(a::set_member(Some(&mut s), id, Some(&identity(id))));
    }
    (
        s,
        Input {
            generation: 9,
            active: true,
            reason: 2,
            id: 1,
            identity: identity(1),
            peers: core::array::from_fn(|n| {
                if n == 4 {
                    openhelmet_mesh::peer_evidence::Row {
                        active: 1,
                        id: 1,
                        mac: [1; 6],
                        slot: 6,
                        seen: 100i64.to_le_bytes(),
                    }
                } else {
                    Default::default()
                }
            }),
            last_seen_ms: 100,
            now_ms: 3101,
            map: SlotMap {
                slot_count: 7,
                slot_ids: [6, 0, 3, 0, 0, 0, 1, 0],
                active_speaker_count: 2,
                active_speaker_ids: [1, 6],
                relay_masks: [32, 1],
            },
        },
    )
}
#[test]
fn authority_time_and_projection_fail_closed() {
    let (s, i) = setup();
    let mut bad = Vec::new();
    for now in [0, 99, 100, 3100] {
        let mut b = i;
        b.now_ms = now;
        bad.push(b);
    }
    let mut b = i;
    b.peers = Default::default();
    bad.push(b);
    let mut b = i;
    b.active = false;
    bad.push(b);
    let mut b = i;
    b.identity = identity(8);
    bad.push(b);
    for id in [0, 3, 8, 9, 255] {
        let mut b = i;
        b.id = id;
        b.identity = identity(id);
        bad.push(b);
    }
    let mut b = i;
    b.map.slot_ids[1] = 6;
    bad.push(b);
    for b in bad {
        assert!(removal::plan(&s, b).is_none(), "{b:?}");
    }
    for (local, leader, term) in [(3, 1, 7), (3, 3, 0)] {
        let mut s = s.clone();
        a::init(Some(&mut s), local, leader, term, 37, 0);
        assert!(removal::plan(&s, i).is_none());
    }
    let mut leave = i;
    leave.reason = 1;
    leave.peers = Default::default();
    assert!(removal::plan(&s, leave).is_some());
}
#[test]
fn commit_once_preserves_sparse_survivors_cleans_request_and_relay() {
    let (mut s, i) = setup();
    let req = a::SpeakerRequest {
        term: 7,
        request_seq: 1,
        frame_counter: 10,
        active: 1,
    };
    assert!(a::speaker_request(Some(&mut s), 1, Some(&req), 10, 100));
    let mut t = Transaction::default();
    let before = s.snapshot();
    let p = t.prepare(&s, i, 1).unwrap();
    assert_eq!(s.snapshot(), before);
    assert_eq!(p.map.slot_ids, [6, 0, 3, 0, 0, 0, 0, 0]);
    assert_eq!(p.map.slot_count, 3);
    assert_eq!(p.map.active_speaker_ids, [6, 0]);
    assert_eq!(p.map.relay_masks, [0, 0]);
    assert!(t.commit(&mut s, i, 1).is_some());
    assert_eq!(s.session().2, 36);
    assert_eq!(s.member_identity(1), Identity::default());
    assert_eq!(s.request(1), (false, 0));
    assert!(t.commit(&mut s, i, 1).is_none());
}
#[test]
fn stale_completion_refreshed_evidence_map_and_binding_never_remove() {
    for n in 0..8 {
        let (mut s, i) = setup();
        let mut t = Transaction::default();
        t.prepare(&s, i, 17).unwrap();
        let mut b = i;
        match n {
            0 => b.generation += 1,
            1 => b.last_seen_ms += 1,
            2 => b.map.slot_ids.swap(0, 2),
            3 => b.now_ms -= 1,
            4 => t.cancel(),
            5 => {
                a::set_member(Some(&mut s), 1, Some(&identity(8)));
            }
            6 => {
                a::init(Some(&mut s), 3, 3, 8, 37, 0);
            }
            _ => b.active = false,
        }
        let before = s.snapshot();
        assert!(t.commit(&mut s, b, 17).is_none());
        assert_eq!(s.snapshot(), before);
    }
}
