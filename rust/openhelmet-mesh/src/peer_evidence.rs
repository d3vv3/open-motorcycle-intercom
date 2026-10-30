//! Call-local transport projection evidence. Row positions are preserved; no
//! snapshot is retained by the authoritative membership owner.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Row {
    pub active: u8,
    pub id: u8,
    pub mac: [u8; 6],
    pub slot: i8,
    pub seen: [u8; 8],
}
impl Row {
    pub fn last_seen(self) -> i64 {
        i64::from_le_bytes(self.seen)
    }
}
pub type Rows = [Row; 8];
pub const ABSENT: u8 = 255;

/// Detached projection operations: 0 untouched, 1 retain, 2 remove,
/// 3 insert, 4 rebind. Masks refer to rows, never node IDs.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Reconciliation {
    pub rows: Rows,
    pub operations: [u8; 8],
    pub cleanup: u8,
    pub added: u8,
    pub removed: u8,
    pub valid: u8,
}

/// Derive transport effects from the existing authority and call-local evidence.
/// `target == 0` reconciles a complete membership; otherwise only liveness for
/// that bound remote member. No telemetry or owner state is retained here.
pub fn reconcile(
    peers: &Rows,
    local: u8,
    members: u8,
    identities: &[[u8; 6]; 8],
    slots: &[u8],
    target: u8,
) -> Reconciliation {
    let mut out = Reconciliation {
        rows: *peers,
        ..Default::default()
    };
    if target > 8 || (target != 0 && (target == local || members & (1 << (target - 1)) == 0)) {
        return out;
    }
    // Duplicate IDs cannot safely produce detached member-removal effects:
    // fail closed rather than scrub evidence belonging to a surviving row.
    let mut retained = 0u8;
    for (n, peer) in peers.iter().enumerate() {
        if peer.active == 0 || (target != 0 && peer.id != target) {
            continue;
        }
        let bit = if (1..=8).contains(&peer.id) {
            1 << (peer.id - 1)
        } else {
            0
        };
        if retained & bit != 0 {
            return Reconciliation::default();
        }
        if bit == 0 || (peer.id != local && members & bit == 0) {
            out.operations[n] = 2;
            out.rows[n].active = 0;
            out.removed |= 1 << n;
        } else {
            retained |= bit;
            out.operations[n] = 1;
        }
    }
    for (slot, &id) in slots.iter().enumerate() {
        if !(1..=8).contains(&id) || (target != 0 && id != target) {
            continue;
        }
        let bit = 1 << (id - 1);
        if members & bit == 0 {
            continue;
        }
        let existing = out.rows.iter().position(|r| r.active != 0 && r.id == id);
        if id == local {
            if let Some(n) = existing {
                out.rows[n].slot = slot as i8;
            }
            continue;
        }
        let Some(n) = existing.or_else(|| out.rows.iter().position(|r| r.active == 0)) else {
            return Reconciliation::default();
        };
        let rebind = existing.is_some() && out.rows[n].mac != identities[usize::from(id - 1)];
        if existing.is_none() || rebind {
            out.cleanup |= bit;
            out.operations[n] = if rebind { 4 } else { 3 };
            if existing.is_none() {
                out.added |= 1 << n;
            }
        }
        out.rows[n].active = 1;
        out.rows[n].id = id;
        out.rows[n].mac = identities[usize::from(id - 1)];
        out.rows[n].slot = slot as i8;
    }
    // A liveness refresh can precede a projected slot map.
    if target != 0 && !slots.contains(&target) {
        let mut fallback = [0; 8];
        fallback[0] = target;
        let mut result = reconcile(peers, local, members, identities, &fallback[..1], target);
        for (n, row) in result.rows.iter_mut().enumerate() {
            if row.active != 0 && row.id == target {
                row.slot = if result.operations[n] == 3 {
                    -1
                } else {
                    peers[n].slot
                };
            }
        }
        return result;
    }
    out.valid = 1;
    out
}
const _: () = {
    assert!(core::mem::size_of::<Row>() == 17);
    assert!(core::mem::align_of::<Row>() == 1);
};
