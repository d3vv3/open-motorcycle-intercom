//! Stateless participant projection departure and already-authorized JOINING destinations.
//! No membership mutation, recovery transaction, or retained destination lives here.
use crate::adaptive::{self, Adaptive, Identity, Membership};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Destination {
    pub leader: Identity,
    pub leader_id: u8,
    pub slot: i8,
    pub term: u32,
}
#[allow(clippy::too_many_arguments)]
pub fn departure(
    s: &Adaptive,
    active: bool,
    local: u8,
    leader: u8,
    source: u8,
    identity: Identity,
    local_identity: Identity,
    projection: bool,
) -> Option<u8> {
    let (bound_local, bound_leader, _, term) = s.session();
    (active
        && local != leader
        && local == bound_local
        && leader == bound_leader
        && term != 0
        && source != local
        && identity != local_identity
        && identity.address_len == 6
        && source != 0
        && s.member_identity(source) == identity
        && projection)
        .then_some(source)
}
pub fn eviction(s: &Adaptive, source: u8, m: &Membership) -> Option<Destination> {
    if !adaptive::membership_removes_local(Some(s), source, Some(m)) {
        return None;
    }
    let slot = m.slot_map.slot_ids[..usize::from(m.slot_map.slot_count)]
        .iter()
        .position(|id| *id == m.leader_id)? as i8;
    Some(Destination {
        leader: m.identities[usize::from(m.leader_id - 1)],
        leader_id: m.leader_id,
        slot,
        term: m.term,
    })
}
/// Retirement barrier: begin, invalidate, timers, RX wait/finish, queue drain.
/// Invalidate establishes a new expected token before any blocking primitive.
pub fn retire_effects(mut effect: impl FnMut(u32) -> bool) -> bool {
    if !effect(30) {
        return false;
    }
    effect(31);
    if !effect(0) {
        return false;
    }
    effect(32);
    if !effect(0) {
        return false;
    }
    if !effect(33) {
        return false;
    }
    if !effect(0) {
        return false;
    }
    effect(34);
    effect(0)
}

/// Destination state/episode initialization occurs while ingress is closed.
/// The episode effect establishes its own token immediately; never adopt one
/// after a wait or application callback. Notification is strictly last.
pub fn reopen_effects(mut effect: impl FnMut(u32) -> bool) {
    if !effect(0) || !effect(40) || !effect(0) {
        return;
    }
    effect(41);
    if !effect(0) || !effect(42) {
        return;
    }
    if effect(0) {
        effect(43);
    }
}

/// A retired destination: cleanup may block, so projection and reopening must
/// retain the token returned by this transaction's retirement, not resample it.
/// Op 50 alone may replace it, with the token of its own invalidation captured
/// before its blocking cleanup, after rejecting any intervening reset.
pub fn destination_effects(mut effect: impl FnMut(u32) -> bool) {
    if !effect(0) || !effect(49) || !effect(0) {
        return;
    }
    effect(50);
    if !effect(0) || !effect(51) || !effect(0) {
        return;
    }
    reopen_effects(effect);
}

/// Cold election / accepted ACK activation. C supplies only detached row,
/// owner-init, SDK and timer primitives. Owner init must return its token before
/// any blocking SDK/timer operation. Notification is the final operation.
pub fn activation_effects(mut effect: impl FnMut(u32) -> bool) {
    if !effect(0) || !effect(60) || !effect(0) || !effect(61) || !effect(0) {
        return;
    }
    effect(62);
    if !effect(0) {
        return;
    }
    effect(63);
    if effect(0) {
        effect(64);
    }
}

/// Entry bookkeeping uses copied state queries, not another episode owner.
/// Preserve begin-before-cancel and the caller's single time sample.
pub fn tracking_effects(mut effect: impl FnMut(u32) -> bool) {
    let scanning = effect(70);
    let was_scanning = effect(71);
    let joining = effect(72);
    let was_joining = effect(73);
    if scanning && !was_scanning {
        effect(74);
    }
    if joining && !was_joining {
        effect(75);
    }
    if !scanning && was_scanning {
        effect(76);
    }
    if !joining && was_joining {
        effect(77);
    }
    effect(78);
}

/// Synchronous ACTIVE maintenance continuation. Each effect is detached from the
/// owner, and every continuation is admitted afresh (including between grants
/// and STATUS). Operation zero checks lifecycle and the original session.
pub fn maintenance_effects(mut effect: impl FnMut(u32) -> bool) {
    if !effect(0) {
        return;
    }
    if effect(10) {
        effect(11);
    }
    if !effect(0) {
        return;
    }
    if effect(12) {
        if effect(13) {
            effect(14);
            if !effect(0) {
                return;
            }
        }
        effect(15);
    }
    for op in 16..=20 {
        if !effect(0) {
            return;
        }
        effect(op);
    }
}

/// Cold yield authorization remains at the existing live-membership boundary.
/// Its executor historically projects the coordinator at slot zero.
pub fn cold_destination(
    accepted: bool,
    leader: Identity,
    leader_id: u8,
    term: u32,
) -> Option<Destination> {
    (accepted && leader.address_len == 6 && (1..=8).contains(&leader_id) && term == 1).then_some(
        Destination {
            leader,
            leader_id,
            slot: 0,
            term,
        },
    )
}
