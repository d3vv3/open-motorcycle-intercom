//! Speaker authority owned by the mesh-task owner. Requests remain in Adaptive.
use crate::{adaptive as a, core};
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct Effect {
    pub changed: bool,
    pub ids: [u8; 2],
    pub masks: [u8; 2],
    pub removed: [u8; 2],
}
// Byte storage deliberately keeps the live owner aligned to four on all hosts.
#[derive(Default)]
pub struct Speakers {
    ids: [u8; 2],
    masks: [u8; 2],
    deadline: [[u8; 8]; 9],
    since: [[u8; 8]; 9],
    transition: u8,
    grace: u32,
}
/// Producer mailbox, separate from the task-confined Adaptive/Speakers owner.
/// Every access, including reset and expiry sampling, requires speaker_mux.
#[derive(Default)]
pub struct LocalVoice {
    active: bool,
    deadline_ms: i64,
}
impl LocalVoice {
    pub const fn new() -> Self {
        Self {
            active: false,
            deadline_ms: 0,
        }
    }
    pub fn offer(&mut self, active: bool, now_ms: i64) {
        self.active = active;
        self.deadline_ms = if active { now_ms + 1500 } else { 0 };
    }
    pub fn clear(&mut self) {
        *self = Self::new();
    }
    pub fn sample(&mut self, now_ms: i64) -> bool {
        if self.active && now_ms >= self.deadline_ms {
            self.active = false;
        }
        self.active
    }
}
impl Speakers {
    /// Preserve the committed sparse layout; never reconstruct it from peers.
    /// Stale membership/layout is rejected rather than silently remapped.
    pub fn produce_map(&self, a: &a::Adaptive, layout: &core::SlotMap) -> Option<core::SlotMap> {
        let members = a.session().2;
        let mut map = core::SlotMap {
            slot_count: layout.slot_count,
            slot_ids: layout.slot_ids,
            ..Default::default()
        };
        if map.slot_ids.iter().any(|&id| id > 8) || !a::slots_valid(&map, members) {
            return None;
        }
        for i in 0..2 {
            if members & core::node_bit(self.ids[i]) != 0 {
                let n = usize::from(map.active_speaker_count);
                map.active_speaker_ids[n] = self.ids[i];
                map.relay_masks[n] = self.masks[i] & members;
                map.active_speaker_count += 1;
            }
        }
        Some(map)
    }
    pub fn projection(&self) -> Effect {
        Effect {
            ids: self.ids,
            masks: self.masks,
            ..Effect::default()
        }
    }
    pub fn clear_member(&mut self, id: u8) {
        if core::node_bit(id) == 0 {
            return;
        }
        self.deadline[usize::from(id)] = [0; 8];
        self.since[usize::from(id)] = [0; 8];
        self.transition &= !core::node_bit(id);
        if self.transition == 0 {
            self.grace = 0;
        }
        for i in 0..2 {
            self.masks[i] &= !core::node_bit(id);
            if self.ids[i] == id {
                self.ids[i] = 0;
                self.masks[i] = 0;
            }
        }
        self.compact();
    }
    fn compact(&mut self) {
        if self.ids[0] == 0 && self.ids[1] != 0 {
            self.ids.swap(0, 1);
            self.masks.swap(0, 1);
        }
    }
    pub fn audio(&mut self, id: u8, now: i64) {
        if core::node_bit(id) == 0 {
            return;
        }
        let i = usize::from(id);
        if i64::from_le_bytes(self.deadline[i]) <= now {
            self.since[i] = now.to_le_bytes();
        }
        self.deadline[i] = (now + 1500).to_le_bytes();
    }
    pub fn request(&mut self, id: u8, active: bool, now: i64, coordinator: bool) {
        if core::node_bit(id) == 0 {
            return;
        }
        if !active {
            self.transition &= !core::node_bit(id);
        }
        if !coordinator {
            return;
        }
        let i = usize::from(id);
        if active {
            if i64::from_le_bytes(self.deadline[i]) <= now || i64::from_le_bytes(self.since[i]) == 0
            {
                self.since[i] = now.to_le_bytes();
            }
            self.deadline[i] = (now + 1500).to_le_bytes();
        } else {
            self.deadline[i] = [0; 8];
            self.since[i] = [0; 8];
        }
    }
    pub fn handover(&mut self, now: u32) {
        self.transition = self
            .ids
            .iter()
            .fold(0, |mask, id| mask | core::node_bit(*id));
        self.grace = now.wrapping_add(1500);
    }
    pub fn grant(&mut self, a: &a::Adaptive, count: u8, ids: [u8; 2], masks: [u8; 2]) -> bool {
        if count > 2 {
            return false;
        }
        let members = a.session().2;
        for i in 0..2 {
            if i >= usize::from(count) {
                if ids[i] != 0 || masks[i] != 0 {
                    return false;
                }
            } else if members & core::node_bit(ids[i]) == 0
                || (i == 1 && ids[0] == ids[1])
                || masks[i] & !members != 0
            {
                return false;
            }
        }
        self.ids = ids;
        self.masks = masks;
        true
    }
    pub fn release(&mut self, count: u8, ids: [u8; 2]) -> bool {
        if count > 2 {
            return false;
        }
        for i in 0..2 {
            if i >= usize::from(count) {
                if ids[i] != 0 {
                    return false;
                }
            } else if core::node_bit(ids[i]) == 0 || (i == 1 && ids[0] == ids[1]) {
                return false;
            }
        }
        for id in &ids[..usize::from(count)] {
            for i in 0..2 {
                if self.ids[i] == *id {
                    self.ids[i] = 0;
                    self.masks[i] = 0;
                }
            }
        }
        self.compact();
        true
    }
    pub fn arbitrate(&mut self, a: &mut a::Adaptive, now: i64) -> Effect {
        let mut since = self.since.map(i64::from_le_bytes);
        let members = a.session().2;
        for id in 1..=8 {
            let i = usize::from(id);
            if members & core::node_bit(id) == 0 {
                since[i] = 0;
                continue;
            }
            let eligible = a::speaker_request_active(Some(a), id, now as u32)
                || (a.request(id).0
                    && i64::from_le_bytes(self.deadline[i]) > now
                    && self.ids.contains(&id))
                || (self.transition & core::node_bit(id) != 0
                    && self.grace.wrapping_sub(now as u32) as i32 >= 0);
            if eligible {
                if since[i] == 0 {
                    since[i] = now;
                }
            } else {
                since[i] = 0;
            }
        }
        let mut ids = [0; 2];
        core::select_speakers(Some(&self.ids), Some(&since), &mut ids);
        let mut masks = [0; 2];
        for i in 0..2 {
            if ids[i] != 0 {
                a::route(Some(a), ids[i], now as u32, Some(&mut masks[i]));
                masks[i] &= members;
            }
        }
        let mut e = Effect {
            changed: ids != self.ids || masks != self.masks,
            ids,
            masks,
            removed: [0; 2],
        };
        let mut n = 0;
        for id in self.ids {
            if id != 0 && !ids.contains(&id) {
                e.removed[n] = id;
                n += 1;
            }
        }
        self.ids = ids;
        self.masks = masks;
        e
    }
}
