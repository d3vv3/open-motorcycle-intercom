//! Synchronous frame transaction. Platform critical sections protect only copied
//! scalar state. No durable owner borrow may span an effect.
use crate::{frame, tx_slot::Slot};
use core::ffi::c_void;
pub const GENERATION: u32 = 0;
pub const ACTIVE: u32 = 1;
pub const CLOCK: u32 = 2;
pub const TIMING: u32 = 3;
pub const STOPPING: u32 = 4;
pub const ENTER: u32 = 5;
pub const EXIT: u32 = 6;
pub const READ: u32 = 7;
pub const WRITE: u32 = 8;
pub const ARM_WRITE: u32 = 9;
pub const ARM: u32 = 10;
pub const LOG: u32 = 11;
pub const TICK: u32 = 12;
pub const COORDINATOR: u32 = 13;
pub const INDEX: u32 = 14;
pub const CONTROL: u32 = 15;
pub const CONTROL_ARM: u32 = 16;
pub const CONTROL_SIGNAL: u32 = 17;
pub const SLOT_ARM: u32 = 18;
pub const SLOT_SIGNAL: u32 = 19;
pub const STAT: u32 = 20;
#[repr(C)]
#[derive(Default)]
pub struct State {
    pub start: i64,
    pub counter: u32,
}
#[repr(C)]
pub struct Arm {
    pub boundary: i64,
    pub generation: u32,
}
#[repr(C)]
pub struct Tick {
    pub counter: u32,
    pub now_ms: u32,
}
#[repr(C)]
pub struct Control {
    pub start: i64,
    pub deadline: i64,
    pub generation: u32,
}
#[derive(Clone, Copy)]
pub struct Config {
    pub voices: u8,
    pub interval: u32,
    pub frame: i64,
    pub slot: i64,
    pub control: i64,
    pub guard: i64,
}
impl Config {
    pub fn valid(self) -> bool {
        self.voices > 0
            && self.interval > 0
            && self.frame > 0
            && self.slot > 0
            && self.control > 0
            && self.guard >= 0
    }
}
pub enum SlotAction {
    Rollover(u32, i64),
    Install(i8, u32, i64, i64),
    Complete(u32),
}
impl SlotAction {
    pub fn apply(self, slot: &mut Slot) -> u8 {
        match self {
            Self::Rollover(g, now) => slot.rollover(g, now),
            Self::Install(i, g, s, d) => {
                slot.install(i, g, s, d);
                0
            }
            Self::Complete(g) => {
                slot.complete(g);
                0
            }
        }
    }
}
type Effect<'a> = dyn FnMut(u32, i64, *mut c_void) -> i64 + 'a;
fn scalar(e: &mut Effect<'_>, op: u32, v: i64) -> i64 {
    e(op, v, core::ptr::null_mut())
}
fn dto<T>(e: &mut Effect<'_>, op: u32, value: &mut T) {
    e(op, 0, core::ptr::from_mut(value).cast());
}
// Only scalar generation/read/write effects occur between ENTER/EXIT. There
// are no returns, fallible conversions, owner accesses or clocks in this region.
fn publish_arm(e: &mut Effect<'_>, generation: u32, boundary: i64, stopping: bool) -> bool {
    scalar(e, ENTER, 0);
    let current = scalar(e, GENERATION, 0);
    let valid = !stopping && current == i64::from(generation);
    if valid {
        dto(
            e,
            ARM_WRITE,
            &mut Arm {
                boundary,
                generation,
            },
        );
    }
    scalar(e, EXIT, 0);
    valid
}
pub fn service(
    e: &mut Effect<'_>,
    mut slot: impl FnMut(SlotAction) -> u8,
    start: i64,
    generation: u32,
    c: Config,
) {
    if !c.valid() {
        return;
    }
    scalar(e, ENTER, 0);
    let valid = scalar(e, GENERATION, 0) == i64::from(generation);
    scalar(e, EXIT, 0);
    if !valid || scalar(e, ACTIVE, 0) == 0 {
        return;
    }
    let now = scalar(e, CLOCK, 0);
    scalar(e, TIMING, now.wrapping_sub(start));
    let stopping = scalar(e, STOPPING, 0) != 0;
    if !publish_arm(e, generation, start.wrapping_add(c.frame), stopping) {
        return;
    }
    let delay = start
        .wrapping_add(c.frame)
        .wrapping_sub(scalar(e, CLOCK, 0))
        .max(1);
    let result = scalar(e, ARM, delay);
    // Platform executor normalizes ESP_ERR_INVALID_STATE to -1; all other
    // nonzero errors retain their raw value for logging.
    if result != 0 {
        if result != -1 {
            scalar(e, LOG, result);
        }
        return;
    }
    let mut state = State::default();
    scalar(e, ENTER, 0);
    dto(e, READ, &mut state);
    state.counter = frame::advance(state.start, state.counter, start, c.frame);
    state.start = start;
    dto(e, WRITE, &mut state);
    scalar(e, EXIT, 0);
    let now_ms = (scalar(e, CLOCK, 0) / 1000) as u32;
    dto(
        e,
        TICK,
        &mut Tick {
            counter: state.counter,
            now_ms,
        },
    );
    let coordinator = scalar(e, COORDINATOR, 0) != 0;
    let index = scalar(e, INDEX, 0) as i8;
    let p = frame::timing(
        start,
        state.counter,
        coordinator,
        index,
        c.voices,
        c.interval,
        c.slot,
        c.control,
        c.guard,
    );
    if p.owns_control {
        dto(
            e,
            CONTROL,
            &mut Control {
                start: p.control_start,
                deadline: p.control_deadline,
                generation,
            },
        );
        let delay = p.control_start.wrapping_sub(scalar(e, CLOCK, 0));
        if delay > 0 {
            scalar(e, CONTROL_ARM, delay);
        } else {
            scalar(e, CONTROL_SIGNAL, 0);
        }
    }
    if p.has_slot && scalar(e, ACTIVE, 0) != 0 {
        let stopping = scalar(e, STOPPING, 0) != 0;
        scalar(e, ENTER, 0);
        let current = scalar(e, GENERATION, 0) as u32;
        scalar(e, EXIT, 0);
        let missed = if stopping {
            0
        } else {
            slot(SlotAction::Rollover(current, scalar(e, CLOCK, 0)))
        };
        if missed != 0 {
            scalar(e, STAT, 0);
            if missed == 2 {
                scalar(e, STAT, 1);
            }
        }
        let index = scalar(e, INDEX, 0) as i8;
        slot(SlotAction::Install(
            index,
            generation,
            p.slot_start,
            p.slot_deadline,
        ));
        let delay = p.slot_start.wrapping_sub(scalar(e, CLOCK, 0));
        if delay <= 0 {
            scalar(e, SLOT_SIGNAL, 0);
        } else if scalar(e, SLOT_ARM, delay) == 0 {
            slot(SlotAction::Complete(generation));
            scalar(e, STAT, 0);
        }
    }
}
