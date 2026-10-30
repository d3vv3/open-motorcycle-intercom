//! Task-side fallback dispatch. Adaptive controls are consumed before this seam.
//! Binding comes from the existing Owner; semantic handlers still commit work.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Dispatch {
    Unknown = 0,
    Rejected = 1,
    Audio = 2,
    SlotMap = 3,
    Grant = 4,
    Release = 5,
    Status = 6,
    Keepalive = 7,
    Leave = 8,
}

pub fn dispatch(kind: u8, size: u16, bound: bool) -> Dispatch {
    match kind {
        0x01 => Dispatch::Audio,
        0x06 => Dispatch::SlotMap,
        0x09 => Dispatch::Grant,
        0x0a => Dispatch::Release,
        0x07 if bound && size == 8 => Dispatch::Status,
        0x08 if bound && size == 2 => Dispatch::Keepalive,
        0x04 if bound => Dispatch::Leave,
        0x04 | 0x07 | 0x08 => Dispatch::Rejected,
        _ => Dispatch::Unknown,
    }
}
