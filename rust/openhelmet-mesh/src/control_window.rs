//! Control-window transaction sequencing. Durable owners and platform effects stay separate.
//! Clock distances use the signed modular half-range, including both endpoints.
pub const REJECT: u8 = 0;
pub const SYNC: u8 = 1;
pub const QUEUED: u8 = 2;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Context {
    pub control_generation: u32,
    pub tdma_generation: u32,
    pub lifecycle: u32,
    pub session: u32,
    pub frame: u32,
    pub term: u32,
    pub local: u8,
    pub coordinator: u8,
    pub owner_local: u8,
    pub leader: u8,
    pub role: u8,
    pub slot: i8,
    pub active: bool,
    pub stopping: bool,
}
pub fn window_open(start: i64, deadline: i64, now: i64) -> bool {
    now.wrapping_sub(start) >= 0 && deadline.wrapping_sub(now) >= 0
}
#[allow(clippy::too_many_arguments)]
pub fn grace(
    coordinator: bool,
    transitioned: bool,
    local: u8,
    leader: u8,
    now: u32,
    deadline: u32,
    frame: u32,
    voices: u8,
) -> bool {
    coordinator
        && transitioned
        && local == leader
        && (now.wrapping_sub(deadline) as i32) <= 0
        && (frame / u32::from(voices)).is_multiple_of(2)
}
pub fn select(
    frame: u32,
    coordinator: bool,
    slot: i8,
    voices: u8,
    interval: u32,
    grace: bool,
) -> u8 {
    if !crate::frame::owns_control(frame, coordinator, slot, voices, interval) {
        REJECT
    } else if (coordinator && frame.is_multiple_of(interval)) || grace {
        SYNC
    } else {
        QUEUED
    }
}
pub fn continuation(original: &Context, current: &Context) -> bool {
    original == current
        && current.local == current.owner_local
        && current.coordinator == current.leader
        && current.active
        && !current.stopping
        && original.control_generation == original.tdma_generation
}

/// Operation-specific DTO contract is mirrored in mesh_control_window.h.
pub mod operation {
    pub const SNAPSHOT: u32 = 0;
    pub const CLOCK: u32 = 1;
    pub const FRAME: u32 = 2;
    pub const ROLE: u32 = 3;
    pub const SLOT: u32 = 4;
    pub const GRACE: u32 = 5;
    pub const IDLE: u32 = 6;
    pub const DEQUEUE: u32 = 7;
    pub const AUTHORITY: u32 = 8;
    pub const LOCAL: u32 = 9;
    pub const SYNC: u32 = 10;
    pub const SEND: u32 = 11;
    pub const RESTORE: u32 = 12;
    pub const STAT: u32 = 13;
}
#[repr(C)]
#[derive(Default)]
pub struct Snapshot {
    pub identity: Context,
    pub start: i64,
    pub deadline: i64,
}
#[repr(C)]
#[derive(Default)]
pub struct Grace {
    pub deadline: u32,
    pub local: u8,
    pub leader: u8,
    pub transitioned: u8,
}
#[repr(C)]
#[derive(Default)]
pub struct Authority {
    pub term: u32,
    pub leader: u8,
}
#[repr(C)]
pub struct Submission {
    pub item: *mut crate::control_queue::Item,
    pub deadline: i64,
    pub credit: u16,
    pub sequence: u8,
}

const _: () = {
    use core::mem::{offset_of, size_of};
    assert!(size_of::<Snapshot>() == 48);
    assert!(size_of::<Context>() == 32);
    assert!(offset_of!(Context, role) == 28);
    assert!(size_of::<Grace>() == 8);
    assert!(size_of::<Authority>() == 8);
    assert!(offset_of!(Submission, deadline) == 8);
    assert!(offset_of!(Submission, credit) == 16);
    assert!(offset_of!(Submission, sequence) == 18);
};

/// Synchronous mesh-task transaction; effects receive initialized detached DTOs,
/// never a durable owner. The callback must honor operation-specific pointer
/// contracts, cannot retain pointers, and cannot recursively service a window.
/// A wait may change live identity: continuation deliberately re-observes it.
pub fn service(
    mut effect: impl FnMut(u32, i64, *mut core::ffi::c_void) -> i64,
    voices: u8,
    interval: u32,
) {
    use operation::*;
    let null = core::ptr::null_mut();
    if voices == 0 || interval == 0 {
        return;
    }
    let mut original = Snapshot::default();
    if effect(SNAPSHOT, 0, (&raw mut original).cast()) != 1 {
        return;
    }
    let now = effect(CLOCK, 0, null);
    let id = &original.identity;
    if id.control_generation != id.tdma_generation
        || !id.active
        || id.stopping
        || !window_open(original.start, original.deadline, now)
    {
        return;
    }
    let Ok(frame) = u32::try_from(effect(FRAME, 0, null)) else {
        return;
    };
    let Ok(role) = u8::try_from(effect(ROLE, 0, null)) else {
        return;
    };
    let Ok(slot) = i8::try_from(effect(SLOT, 0, null)) else {
        return;
    };
    if role > 2 || !crate::frame::owns_control(frame, role == 1, slot, voices, interval) {
        return;
    }
    original.identity.frame = frame;
    let Ok(role) = u8::try_from(effect(ROLE, 0, null)) else {
        return;
    };
    if role > 2 {
        return;
    }
    let mut candidate = false;
    if role == 1 {
        let mut g = Grace::default();
        if effect(GRACE, 0, (&raw mut g).cast()) != 1 || g.transitioned > 1 {
            return;
        }
        if g.transitioned == 1 && g.local == g.leader {
            candidate = grace(
                true,
                true,
                g.local,
                g.leader,
                (effect(CLOCK, 0, null) / 1000) as u32,
                g.deadline,
                frame,
                voices,
            );
        }
    }
    let Ok(role) = u8::try_from(effect(ROLE, 0, null)) else {
        return;
    };
    let Ok(slot) = i8::try_from(effect(SLOT, 0, null)) else {
        return;
    };
    if role > 2 {
        return;
    }
    let selection = select(frame, role == 1, slot, voices, interval, candidate);
    let idle = effect(IDLE, 0, null);
    if !(0..=1).contains(&idle) {
        return;
    }
    if idle == 0 {
        let remaining = original.deadline.wrapping_sub(effect(CLOCK, 0, null));
        let ms = if remaining > 0 {
            remaining / 1000 + i64::from(remaining % 1000 != 0)
        } else {
            0
        };
        let waited = effect(IDLE, ms, null);
        if waited != 1 || effect(CLOCK, 0, null).wrapping_sub(original.deadline) > 0 {
            if selection == self::SYNC {
                effect(STAT, 0, null);
            }
            return;
        }
    }
    let mut current = Snapshot::default();
    if effect(SNAPSHOT, 0, (&raw mut current).cast()) != 1 {
        return;
    }
    current.identity.frame = original.identity.frame;
    let id = &current.identity;
    if id.role > 2
        || (id.role == 1) != (id.owner_local == id.leader)
        || !continuation(&original.identity, id)
    {
        return;
    }
    if effect(FRAME, 0, null) != i64::from(frame) {
        return;
    }
    let Ok(role) = u8::try_from(effect(ROLE, 0, null)) else {
        return;
    };
    let Ok(slot) = i8::try_from(effect(SLOT, 0, null)) else {
        return;
    };
    if role > 2
        || !crate::frame::owns_control(frame, role == 1, slot, voices, interval)
        || effect(CLOCK, 0, null).wrapping_sub(original.deadline) > 0
    {
        return;
    }
    if selection == self::SYNC {
        if effect(SYNC, original.deadline, null) != 0 {
            effect(STAT, 0, null);
        }
        return;
    }
    if effect(IDLE, 0, null) != 1 {
        return;
    }
    let mut item = crate::control_queue::Item::default();
    if effect(DEQUEUE, 0, (&raw mut item).cast()) != 1 {
        return;
    }
    // Callback success does not establish packet bounds. Never parse or submit
    // outside the actual fixed DTO allocation, even for a faulty executor.
    if usize::from(item.len) < 9 || usize::from(item.len) > item.data.len() {
        effect(STAT, 0, null);
        return;
    }
    let relayed = item.data[5] & 2 != 0;
    if relayed {
        let sync = item.kind == 0x11;
        if sync {
            // SYNC_V3 wire payload: identity(7), leader, count, LE term.
            if item.len != 29 || u16::from_le_bytes([item.data[7], item.data[8]]) != 20 {
                effect(STAT, 0, null);
                return;
            }
            let mut authority = Authority::default();
            if effect(AUTHORITY, 0, (&raw mut authority).cast()) != 1 {
                return;
            }
            let term = u32::from_le_bytes(item.data[18..22].try_into().unwrap());
            if crate::control_relay::authority(
                true,
                term,
                item.data[16],
                authority.term,
                authority.leader,
            ) != crate::control_relay::Decision::Send
            {
                effect(STAT, 0, null);
                return;
            }
        }
        if crate::control_relay::queued_age(
            true,
            sync,
            item.priority >= 4,
            effect(CLOCK, 0, null),
            item.enqueued_us,
        ) != crate::control_relay::Decision::Send
        {
            effect(STAT, 0, null);
            return;
        }
    }
    let Ok(local) = u8::try_from(effect(LOCAL, 0, null)) else {
        return;
    };
    let credit = crate::status::local_credit(
        item.kind,
        item.len,
        item.data[2],
        local,
        item.data[5],
        item.data[14],
        item.data[15],
    );
    let mut submission = Submission {
        item: &raw mut item,
        deadline: original.deadline,
        credit,
        sequence: u8::from(!relayed),
    };
    if effect(SEND, 0, (&raw mut submission).cast()) != 0 {
        effect(RESTORE, 0, (&raw mut item).cast());
        effect(STAT, 0, null);
        effect(STAT, 1, null);
    }
}
