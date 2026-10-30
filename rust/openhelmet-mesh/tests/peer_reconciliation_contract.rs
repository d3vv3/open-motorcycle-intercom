use openhelmet_mesh::peer_evidence::{Row, Rows, reconcile};

fn row(id: u8, mac: u8, slot: i8, seen: i64) -> Row {
    Row {
        active: 1,
        id,
        mac: [mac; 6],
        slot,
        seen: seen.to_le_bytes(),
    }
}

#[test]
fn detached_membership_preserves_survivors_and_selects_coherent_reuse() {
    let mut peers = Rows::default();
    peers[0] = row(2, 2, 1, 987);
    peers[3] = row(3, 3, 2, 654);
    peers[7] = row(4, 4, 3, 321);
    let mut identities = [[0; 6]; 8];
    identities[1] = [9; 6];
    identities[2] = [3; 6];
    identities[4] = [5; 6];
    let plan = reconcile(&peers, 1, 23, &identities, &[1, 2, 3, 0, 5], 0);
    assert_eq!(plan.valid, 1);
    assert_eq!(plan.operations[0], 4);
    assert_eq!(plan.rows[0].mac, [9; 6]);
    assert_eq!(plan.rows[0].last_seen(), 987);
    assert_eq!(plan.rows[3], peers[3]);
    assert_eq!(plan.operations[3], 1);
    assert_eq!(plan.cleanup, 2 | 16);
    assert_eq!(plan.removed, 128);
    assert_eq!(plan.added, 2);
    assert_eq!(plan.rows[1].id, 5);
    assert_eq!(plan.rows[1].slot, 4);
    // Input is copied and never mutated or retained.
    assert_eq!(peers[0].mac, [2; 6]);
}

#[test]
fn liveness_only_touches_bound_target_and_has_missing_map_fallback() {
    let mut peers = Rows::default();
    peers[3] = row(3, 3, 5, 12345);
    let mut identities = [[0; 6]; 8];
    identities[1] = [2; 6];
    let plan = reconcile(&peers, 1, 7, &identities, &[1, 0, 3], 2);
    assert_eq!(plan.valid, 1);
    assert_eq!(plan.rows[3], peers[3]);
    assert_eq!(plan.operations[3], 0);
    assert_eq!(plan.added, 1);
    assert_eq!(plan.cleanup, 2);
    assert_eq!(plan.rows[0].id, 2);
    assert_eq!(plan.rows[0].slot, -1);
    for target in [1, 4, 9, 255] {
        assert_eq!(
            reconcile(&peers, 1, 7, &identities, &[1, 2, 3], target).valid,
            0
        );
    }
    peers[0] = row(2, 2, 1, 23456);
    let unchanged = reconcile(&peers, 1, 7, &identities, &[1, 2, 3], 2);
    assert_eq!(unchanged.rows[0], peers[0]);
    assert_eq!(unchanged.cleanup, 0);
    assert_eq!(unchanged.added, 0);
    peers[1] = peers[0];
    assert_eq!(reconcile(&peers, 1, 7, &identities, &[1, 2, 3], 2).valid, 0);
}
