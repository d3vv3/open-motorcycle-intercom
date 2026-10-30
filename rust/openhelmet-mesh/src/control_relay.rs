//! Stateless control-relay decisions. Queues and lifecycle remain with their
//! existing owners; callers supply only observations, never retained snapshots.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Decision {
    Send = 0,
    Inactive = 1,
    LocalMissing = 2,
    AlreadyRelayed = 3,
    Cadence = 4,
    UpstreamStale = 5,
    AuthorityChanged = 6,
    Expired = 7,
}

pub fn receive(active: bool, local: u8, relayed: bool) -> Decision {
    if !active {
        Decision::Inactive
    } else if local == 0 {
        Decision::LocalMissing
    } else if relayed {
        Decision::AlreadyRelayed
    } else {
        Decision::Send
    }
}

pub fn cadence(sync: bool, now: u32, last: u32) -> Decision {
    if sync && last != 0 && now.wrapping_sub(last) < 550 {
        Decision::Cadence
    } else {
        Decision::Send
    }
}

pub fn upstream(coordinator: bool, age_ms: u32) -> Decision {
    if coordinator || age_ms <= 600 {
        Decision::Send
    } else {
        Decision::UpstreamStale
    }
}

pub fn authority(
    relayed_sync: bool,
    term: u32,
    leader: u8,
    current_term: u32,
    current_leader: u8,
) -> Decision {
    if relayed_sync && (term != current_term || leader != current_leader) {
        Decision::AuthorityChanged
    } else {
        Decision::Send
    }
}

pub fn queued_age(relayed: bool, sync: bool, lifecycle: bool, now: i64, enqueued: i64) -> Decision {
    let limit = if sync {
        600_000
    } else if lifecycle {
        2_400_000
    } else {
        1_000_000
    };
    if relayed && now.wrapping_sub(enqueued) > limit {
        Decision::Expired
    } else {
        Decision::Send
    }
}
