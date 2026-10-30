//! Bounded policy over canonical C storage; callers serialize ephemeral borrows.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Entry {
    pub data: [u8; 64],
    pub len: u16,
    pub src_id: u8,
    pub seq: u8,
    pub audio_flags: u8,
    pub timestamp_us: i64,
    pub enqueued_us: i64,
}
impl Default for Entry {
    fn default() -> Self {
        Self {
            data: [0; 64],
            len: 0,
            src_id: 0,
            seq: 0,
            audio_flags: 0,
            timestamp_us: 0,
            enqueued_us: 0,
        }
    }
}
#[repr(C)]
#[derive(Default)]
pub struct Queue {
    pub entries: [Entry; 4],
    pub read_idx: u8,
    pub write_idx: u8,
    pub count: u8,
}
#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct PopResult {
    pub delivered: u8,
    pub expired: u8,
    pub expired_with_pending: u8,
    pub expired_age_us_max: u32,
}
const _: () = {
    assert!(size_of::<Entry>() == 88 && align_of::<Entry>() == 8);
    assert!(core::mem::offset_of!(Entry, len) == 64);
    assert!(core::mem::offset_of!(Entry, src_id) == 66);
    assert!(core::mem::offset_of!(Entry, seq) == 67);
    assert!(core::mem::offset_of!(Entry, audio_flags) == 68);
    assert!(core::mem::offset_of!(Entry, timestamp_us) == 72);
    assert!(core::mem::offset_of!(Entry, enqueued_us) == 80);
    assert!(size_of::<Queue>() == 360 && align_of::<Queue>() == 8);
    assert!(core::mem::offset_of!(Queue, read_idx) == 352);
    assert!(core::mem::offset_of!(Queue, write_idx) == 353);
    assert!(core::mem::offset_of!(Queue, count) == 354);
    assert!(size_of::<PopResult>() == 8);
};
pub fn valid(q: &Queue) -> bool {
    q.read_idx < 4
        && q.write_idx < 4
        && q.count <= 4
        && q.write_idx == (q.read_idx + q.count) % 4
        && (0..q.count).all(|n| (1..=64).contains(&q.entries[((q.read_idx + n) % 4) as usize].len))
}
pub fn push(
    q: &mut Queue,
    data: &[u8],
    src: u8,
    seq: u8,
    flags: u8,
    timestamp: i64,
    now: i64,
) -> u8 {
    if !valid(q) || data.is_empty() || data.len() > 64 {
        return 0;
    }
    let status = if q.count == 4 {
        q.read_idx = (q.read_idx + 1) % 4;
        2
    } else {
        q.count += 1;
        1
    };
    let e = &mut q.entries[q.write_idx as usize];
    e.data[..data.len()].copy_from_slice(data);
    e.len = data.len() as u16;
    e.src_id = src;
    e.seq = seq;
    e.audio_flags = flags;
    e.timestamp_us = timestamp;
    e.enqueued_us = now;
    q.write_idx = (q.write_idx + 1) % 4;
    status
}
pub fn pop(q: &mut Queue, now: i64, out: &mut Entry, result: &mut PopResult) -> u8 {
    if !valid(q) {
        return 0;
    }
    let mut r = PopResult::default();
    while q.count != 0 {
        let e = &q.entries[q.read_idx as usize];
        let age = now.wrapping_sub(e.enqueued_us);
        if age > 60_000 {
            r.expired += 1;
            r.expired_age_us_max = r
                .expired_age_us_max
                .max((age as u64).min(u32::MAX as u64) as u32);
            if (1..q.count).any(|n| {
                now.wrapping_sub(q.entries[((q.read_idx + n) % 4) as usize].enqueued_us) <= 60_000
            }) {
                r.expired_with_pending += 1;
            }
        } else {
            *out = *e;
            r.delivered = 1;
        }
        q.read_idx = (q.read_idx + 1) % 4;
        q.count -= 1;
        if r.delivered != 0 {
            break;
        }
    }
    *result = r;
    if r.delivered != 0 { 2 } else { 1 }
}
pub fn remove(q: &mut Queue, id: u8) -> u8 {
    if !valid(q) {
        return 255;
    }
    let mut kept = 0;
    let old = q.count;
    for n in 0..old {
        let read = ((q.read_idx + n) % 4) as usize;
        if q.entries[read].src_id == id {
            continue;
        }
        let write = ((q.read_idx + kept) % 4) as usize;
        if read != write {
            q.entries[write] = q.entries[read];
        }
        kept += 1;
    }
    q.count = kept;
    q.write_idx = (q.read_idx + kept) % 4;
    old - kept
}
