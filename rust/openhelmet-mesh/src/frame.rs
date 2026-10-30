//! Stateless TDMA due-boundary arithmetic. Times must be within one signed
//! modular half-range; the ambiguous half-range itself is not due.
/// Recover the latest currently due boundary, unlike the deliberately checked
/// signed arithmetic contract of `core::recover_frame_boundary`.
pub fn due_boundary(expected: i64, now: i64, frame_us: i64) -> Option<i64> {
    let elapsed = now.wrapping_sub(expected);
    if elapsed < 0 || frame_us <= 0 {
        return None;
    }
    // Both positive operands and the rounded product are <= i64::MAX.
    let advance = (elapsed / frame_us) * frame_us;
    Some(expected.wrapping_add(advance))
}

/// At least one frame per accepted event; quotient is truncated before the
/// explicit u32 counter wrap, including large gaps and nonpositive deltas.
pub fn advance(previous: i64, counter: u32, start: i64, frame_us: i64) -> u32 {
    let delta = start.wrapping_sub(previous);
    let elapsed = if delta > frame_us {
        (delta / frame_us) as u32
    } else {
        1
    };
    counter.wrapping_add(elapsed)
}

pub fn owns_control(
    counter: u32,
    coordinator: bool,
    slot: i8,
    voice_slots: u8,
    sync_interval: u32,
) -> bool {
    if counter.is_multiple_of(sync_interval) {
        coordinator
    } else {
        slot == (counter % u32::from(voice_slots)) as i8
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    pub control_start: i64,
    pub control_deadline: i64,
    pub slot_start: i64,
    pub slot_deadline: i64,
    pub owns_control: bool,
    pub has_slot: bool,
}

#[allow(clippy::too_many_arguments)]
pub fn timing(
    start: i64,
    counter: u32,
    coordinator: bool,
    slot: i8,
    voice_slots: u8,
    sync_interval: u32,
    slot_us: i64,
    control_us: i64,
    guard_us: i64,
) -> Timing {
    let control_start = start.wrapping_add(i64::from(voice_slots).wrapping_mul(slot_us));
    let slot_start = start.wrapping_add(i64::from(slot).wrapping_mul(slot_us));
    Timing {
        control_start,
        control_deadline: control_start
            .wrapping_add(control_us)
            .wrapping_sub(guard_us),
        slot_start,
        slot_deadline: slot_start.wrapping_add(slot_us).wrapping_sub(guard_us),
        owns_control: owns_control(counter, coordinator, slot, voice_slots, sync_interval),
        has_slot: slot >= 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn handwritten_advancement_and_timing() {
        for (start, expected) in [
            (99999, 8),
            (100000, 8),
            (120000, 8),
            (139999, 8),
            (140000, 9),
            (160001, 10),
        ] {
            assert_eq!(advance(100000, 7, start, 20000), expected);
        }
        assert_eq!(advance(100000, u32::MAX, 120000, 20000), 0);
        assert_eq!(advance(0, 7, 85899345920000, 20000), 7);
        assert_eq!(advance(i64::MAX - 9999, 5, i64::MIN + 30000, 20000), 7);
        let p = timing(120000, 11, false, 3, 8, 10, 2000, 4000, 500);
        assert_eq!(
            p,
            Timing {
                control_start: 136000,
                control_deadline: 139500,
                slot_start: 126000,
                slot_deadline: 127500,
                owns_control: true,
                has_slot: true
            }
        );
        assert!(!timing(0, 10, false, 2, 8, 10, 2000, 4000, 500).owns_control);
        let p = timing(0, 10, true, -1, 8, 10, 2000, 4000, 500);
        assert!(p.owns_control && !p.has_slot);
        let p = timing(i64::MAX - 999, 11, false, 3, 8, 10, 2000, 4000, 500);
        assert_eq!(
            (
                p.control_start,
                p.control_deadline,
                p.slot_start,
                p.slot_deadline
            ),
            (
                i64::MIN + 15000,
                i64::MIN + 18500,
                i64::MIN + 5000,
                i64::MIN + 6500
            )
        );
    }
    #[test]
    fn literal_due_wrap_and_half_range() {
        assert_eq!(due_boundary(120000, 119999, 20000), None);
        assert_eq!(due_boundary(120000, 120000, 20000), Some(120000));
        assert_eq!(due_boundary(120000, 165000, 20000), Some(160000));
        assert_eq!(
            due_boundary(i64::MAX - 9999, i64::MIN + 30000, 20000),
            Some(i64::MIN + 30000)
        );
        assert_eq!(due_boundary(0, i64::MIN, 20000), None);
        assert_eq!(due_boundary(0, i64::MAX, 20000), Some(9223372036854760000));
        assert_eq!(due_boundary(0, 0, 0), None);
    }
}
