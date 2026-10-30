//! Pure transitions for the Rust-owned durable transport queue. No platform calls.
pub const CAPACITY: usize = 12;
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Item {
    pub data: [u8; 209],
    pub dest_mac: [u8; 6],
    pub len: u16,
    pub order: u32,
    pub enqueued_us: i64,
    pub kind: u8,
    pub priority: u8,
}
impl Default for Item {
    fn default() -> Self {
        Self {
            data: [0; 209],
            dest_mac: [0; 6],
            len: 0,
            order: 0,
            enqueued_us: 0,
            kind: 0,
            priority: 0,
        }
    }
}
#[derive(Default)]
pub struct Queue {
    pub items: [Item; CAPACITY],
    pub count: u8,
    pub next_order: u32,
}
impl Queue {
    pub const fn new() -> Self {
        const EMPTY: Item = Item {
            data: [0; 209],
            dest_mac: [0; 6],
            len: 0,
            order: 0,
            enqueued_us: 0,
            kind: 0,
            priority: 0,
        };
        Self {
            items: [EMPTY; CAPACITY],
            count: 0,
            next_order: 0,
        }
    }
}
#[repr(C)]
#[derive(Default, Debug, PartialEq, Eq)]
pub struct Outcome {
    pub accepted: u8,
    pub replaced: u8,
    pub have_evicted: u8,
    pub drops: u8,
    pub local_heard: u8,
    pub local_relay: u8,
}
pub fn priority(t: u8) -> u8 {
    match t {
        0x11 => 6,
        0x0f | 0x10 | 0x12..=0x15 => 5,
        6 | 0x16 | 4 => 4,
        0x17 => 3,
        9 | 10 | 14 => 2,
        _ => 1,
    }
}
fn replaceable(t: u8) -> bool {
    matches!(t, 7 | 8 | 6 | 9 | 14 | 0x16 | 0x17)
}
fn local_status(i: &Item, local: u8) -> bool {
    i.kind == 7 && i.data[1] == 7 && i.data[2] == local && i.data[5] & 2 == 0 && i.len == 17
}
pub fn enqueue(q: &mut Queue, item: Item, forward: bool, local: u8) -> Outcome {
    let mut out = Outcome::default();
    enqueue_into(q, &item, forward, local, &mut out);
    out
}
/// Borrow disjoint input and output only for this transition. Copy the incoming
/// packet directly into canonical storage, then update its queue-owned fields.
pub fn enqueue_into(q: &mut Queue, item: &Item, forward: bool, local: u8, out: &mut Outcome) {
    *out = Outcome::default();
    if q.count as usize > CAPACITY
        || item.len < 9
        || item.len > 209
        || (!forward && item.kind == 0x11)
    {
        return;
    }
    let incoming_priority = if forward && item.kind == 0x11 {
        2
    } else {
        priority(item.kind)
    };
    for n in 0..q.count as usize {
        let old = &q.items[n];
        let replace = if forward {
            item.kind == 0x17
                && old.data[1] == 0x17
                && old.data[5] & 2 != 0
                && old.data[2] == item.data[2]
        } else {
            replaceable(item.kind)
                && old.kind == item.kind
                && old.dest_mac == item.dest_mac
                && (item.kind != 0x17 || (old.data[5] & 2 == 0 && old.data[2] == local))
        };
        if !replace {
            continue;
        }
        let union = local_status(item, local) && local_status(old, local);
        let heard = if union { old.data[14] } else { 0 };
        let relay = if union { old.data[15] } else { 0 };
        let order = old.order;
        let age = if forward {
            item.enqueued_us
        } else {
            old.enqueued_us
        };
        let slot = &mut q.items[n];
        *slot = *item;
        slot.data[14] |= heard;
        slot.data[15] |= relay;
        slot.order = order;
        slot.enqueued_us = age;
        slot.priority = incoming_priority;
        out.accepted = 1;
        out.replaced = 1;
        return;
    }
    if q.count as usize == CAPACITY {
        let mut victim = 0;
        for n in 1..CAPACITY {
            if q.items[n].priority < q.items[victim].priority
                || (!forward
                    && q.items[n].priority == q.items[victim].priority
                    && q.items[n].order < q.items[victim].order)
            {
                victim = n;
            }
        }
        let p = q.items[victim].priority;
        out.drops = 1;
        if p > incoming_priority || (p == incoming_priority && (forward || incoming_priority == 1))
        {
            return;
        }
        let evicted = &q.items[victim];
        // Match C restoration exactly: unlike replacement union, the wire type
        // byte is not part of this predicate. Zero credit still is an eviction.
        if evicted.kind == 7
            && evicted.len == 17
            && evicted.data[2] == local
            && evicted.data[5] & 2 == 0
        {
            out.local_heard = evicted.data[14];
            out.local_relay = evicted.data[15];
        }
        out.have_evicted = 1;
        q.count -= 1;
        q.items[victim] = q.items[q.count as usize];
    }
    let slot = &mut q.items[q.count as usize];
    *slot = *item;
    slot.order = q.next_order;
    slot.priority = incoming_priority;
    q.next_order = q.next_order.wrapping_add(1);
    q.count += 1;
    out.accepted = 1;
}
pub fn dequeue(q: &mut Queue, now: i64, last_sync: u32) -> Option<Item> {
    if q.count == 0 || q.count as usize > CAPACITY {
        return None;
    }
    let mut selected = 0;
    for n in 1..q.count as usize {
        let a = &q.items[n];
        let b = &q.items[selected];
        let overdue = now.wrapping_sub(a.enqueued_us) >= 500_000;
        let old_overdue = now.wrapping_sub(b.enqueued_us) >= 500_000;
        if a.priority > b.priority
            || (a.priority == b.priority
                && ((overdue && !old_overdue) || (overdue == old_overdue && a.order < b.order)))
        {
            selected = n;
        }
    }
    let elapsed = if last_sync == 0 {
        u32::MAX
    } else {
        ((now / 1000) as u32).wrapping_sub(last_sync)
    };
    if elapsed >= 550 {
        for n in 0..q.count as usize {
            let a = &q.items[n];
            if a.data[1] == 0x11
                && a.data[5] & 2 != 0
                && (q.items[selected].priority < 4 || elapsed >= 600)
            {
                selected = n;
                break;
            }
        }
    }
    let item = q.items[selected];
    q.count -= 1;
    q.items[selected] = q.items[q.count as usize];
    Some(item)
}
pub fn reset(q: &mut Queue) {
    // Clear in place: do not materialize a second 12-packet array on the stack.
    for i in &mut q.items {
        i.data.fill(0);
        i.dest_mac.fill(0);
        i.len = 0;
        i.order = 0;
        i.enqueued_us = 0;
        i.kind = 0;
        i.priority = 0;
    }
    q.count = 0;
}
pub fn coalesce_sync(q: &mut Queue, incoming: &Item) -> bool {
    if q.count as usize > CAPACITY || incoming.len != 29 || incoming.data[1] != 0x11 {
        return false;
    }
    for a in &mut q.items[..q.count as usize] {
        if a.data[1] != 0x11
            || a.data[5] & 2 == 0
            || a.data[2] != incoming.data[2]
            || a.len != 29
            || a.data[18..22] != incoming.data[18..22]
            || a.data[16] != incoming.data[16]
        {
            continue;
        }
        a.data[..29].copy_from_slice(&incoming.data[..29]);
        a.data[4] = 1;
        a.data[5] |= 2;
        a.enqueued_us = incoming.enqueued_us;
        return true;
    }
    false
}
fn references(i: &Item, id: u8, mac: [u8; 6], local: u8, coordinator: u8) -> bool {
    let d = &i.data;
    let p = &d[9..];
    let bit = if (1..=8).contains(&id) {
        1 << (id - 1)
    } else {
        0
    };
    if d[2] == id || i.dest_mac == mac {
        return true;
    }
    let map = match d[1] {
        6 => Some(0),
        0x16 => Some(9),
        14 => Some(33),
        0x12 | 0x14 | 0x15 => return p[10] & bit != 0,
        0x13 => return true,
        0x11 => return coordinator != 0 && d[2] == local,
        0x10 => return p[8] == id || p[1..7] == mac,
        10 => return p[1..3].contains(&id),
        9 => return p[1..3].contains(&id) || p[3..5].iter().any(|m| m & bit != 0),
        _ => None,
    };
    if let Some(offset) = map {
        let m = &p[offset..];
        return m[1..9].contains(&id)
            || m[10..12].contains(&id)
            || m[12..14].iter().any(|v| v & bit != 0);
    }
    false
}
pub fn remove_member(
    q: &mut Queue,
    id: u8,
    mac: [u8; 6],
    local: u8,
    coordinator: u8,
    peers: u8,
    speakers: u8,
) {
    if q.count as usize > CAPACITY {
        return;
    }
    let bit = if (1..=8).contains(&id) {
        1 << (id - 1)
    } else {
        0
    };
    let mut kept = 0;
    for n in 0..q.count as usize {
        let mut i = q.items[n];
        if local_status(&i, local) {
            i.data[14] &= !bit;
            i.data[15] &= !bit;
            i.data[11] = peers;
            i.data[16] = speakers;
        }
        if references(&i, id, mac, local, coordinator) {
            continue;
        }
        q.items[kept] = i;
        kept += 1;
    }
    q.count = kept as u8;
}
pub fn before_reuse(q: &mut Queue, id: u8) {
    if q.count as usize > CAPACITY {
        return;
    }
    let mut kept = 0;
    for n in 0..q.count as usize {
        let i = q.items[n];
        if i.data[1] == 10 && i.data[10..12].contains(&id) {
            continue;
        }
        q.items[kept] = i;
        kept += 1;
    }
    q.count = kept as u8;
}
const _: () = {
    assert!(core::mem::size_of::<Item>() == 240);
    assert!(core::mem::align_of::<Item>() == 8);
    assert!(core::mem::offset_of!(Item, len) == 216);
    assert!(core::mem::offset_of!(Item, order) == 220);
    assert!(core::mem::offset_of!(Item, enqueued_us) == 224);
    assert!(core::mem::offset_of!(Item, kind) == 232);
    assert!(core::mem::offset_of!(Item, dest_mac) == 209);
    assert!(core::mem::offset_of!(Item, priority) == 233);
    assert!(core::mem::size_of::<Outcome>() == 6);
    assert!(core::mem::align_of::<Outcome>() == 1);
    assert!(core::mem::offset_of!(Outcome, accepted) == 0);
    assert!(core::mem::offset_of!(Outcome, local_heard) == 4);
    assert!(core::mem::offset_of!(Outcome, local_relay) == 5);
};
