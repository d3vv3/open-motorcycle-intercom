//! Pure helpers shared by MPSL diagnostic code.
//!
//! This module intentionally has no `nrf-mpsl` or PAC dependency so protocol
//! rules used by the MPSL diagnostics can be covered by host-side tests.

use crate::header::EsbHeader;
use crate::mpsl_schedule::{
    SCHEDULE_COUNTER_HINT_PAYLOAD_LEN, SCHEDULE_HINT_PAYLOAD_OFFSET, ScheduleHint,
    encode_schedule_hint,
};

pub(crate) const NUM_PIPES: usize = 8;

pub(crate) const fn advance_pid(pid: u8) -> u8 {
    pid.wrapping_add(1) & 0x03
}

pub(crate) fn next_pipe_in_mask(current: u8, mask: u8) -> Option<u8> {
    if mask == 0 {
        return None;
    }

    let mut next = current;
    for _ in 0..NUM_PIPES {
        next = (next + 1) % NUM_PIPES as u8;
        if mask & (1 << next) != 0 {
            return Some(next);
        }
    }

    None
}

pub(crate) fn write_counter_packet(buf: &mut [u8; 256], pid: u8, counter: u32) {
    let header = unsafe { &mut *(buf.as_mut_ptr().cast::<EsbHeader>()) };
    header.length = 4;
    header.pid_no_ack = 0;
    header.set_pid(pid);
    header.set_no_ack(false);

    let p = EsbHeader::PAYLOAD_OFFSET;
    buf[p..p + 4].copy_from_slice(&counter.to_le_bytes());
}

pub(crate) fn write_counter_payload_packet(
    buf: &mut [u8; 256],
    pid: u8,
    counter: u32,
    payload: &[u8],
) -> bool {
    let total_len = 4 + payload.len();
    if total_len > EsbHeader::MAX_PAYLOAD as usize
        || EsbHeader::PAYLOAD_OFFSET + total_len > buf.len()
    {
        return false;
    }

    write_counter_packet(buf, pid, counter);

    let p = EsbHeader::PAYLOAD_OFFSET;
    buf[p + 4..p + total_len].copy_from_slice(payload);

    let header = unsafe { &mut *(buf.as_mut_ptr().cast::<EsbHeader>()) };
    header.length = total_len as u8;
    true
}

pub(crate) fn write_counter_schedule_packet(
    buf: &mut [u8; 256],
    pid: u8,
    counter: u32,
    hint: ScheduleHint,
) {
    let header = unsafe { &mut *(buf.as_mut_ptr().cast::<EsbHeader>()) };
    header.length = SCHEDULE_COUNTER_HINT_PAYLOAD_LEN as u8;
    header.pid_no_ack = 0;
    header.set_pid(pid);
    header.set_no_ack(false);

    let p = EsbHeader::PAYLOAD_OFFSET;
    buf[p..p + 4].copy_from_slice(&counter.to_le_bytes());
    let _ = encode_schedule_hint(
        &mut buf[p + SCHEDULE_HINT_PAYLOAD_OFFSET
            ..p + SCHEDULE_HINT_PAYLOAD_OFFSET + crate::mpsl_schedule::SCHEDULE_HINT_ENCODED_LEN],
        hint,
    );
}

pub(crate) fn write_counter_schedule_extension_packet(
    buf: &mut [u8; 256],
    pid: u8,
    counter: u32,
    hint: ScheduleHint,
    extension: &[u8],
) {
    write_counter_schedule_packet(buf, pid, counter, hint);

    let extension_offset = EsbHeader::PAYLOAD_OFFSET + SCHEDULE_COUNTER_HINT_PAYLOAD_LEN;
    if !extension.is_empty() && extension_offset + extension.len() <= buf.len() {
        buf[extension_offset..extension_offset + extension.len()].copy_from_slice(extension);

        let header = unsafe { &mut *(buf.as_mut_ptr().cast::<EsbHeader>()) };
        header.length = (SCHEDULE_COUNTER_HINT_PAYLOAD_LEN + extension.len()) as u8;
    }
}

pub(crate) fn read_counter_payload(buf: &[u8]) -> Option<u32> {
    let dma = EsbHeader::DMA_OFFSET;
    let payload = EsbHeader::PAYLOAD_OFFSET;
    if buf.get(dma).copied()? < 4 || buf.len() < payload + 4 {
        return None;
    }

    Some(u32::from_le_bytes([
        buf[payload],
        buf[payload + 1],
        buf[payload + 2],
        buf[payload + 3],
    ]))
}

pub(crate) fn delta_per_pipe(
    current: &[u32; NUM_PIPES],
    previous: &[u32; NUM_PIPES],
) -> [u32; NUM_PIPES] {
    let mut out = [0u32; NUM_PIPES];
    for i in 0..NUM_PIPES {
        out[i] = current[i].saturating_sub(previous[i]);
    }
    out
}

pub(crate) fn spin_until(mut condition: impl FnMut() -> bool, limit: u32) -> bool {
    let mut spins = 0;
    let mut done = condition();
    while !done && spins < limit {
        spins += 1;
        done = condition();
    }
    done
}

#[cfg(test)]
mod tests {
    use super::{
        NUM_PIPES, advance_pid, delta_per_pipe, next_pipe_in_mask, read_counter_payload,
        spin_until, write_counter_packet, write_counter_payload_packet,
        write_counter_schedule_extension_packet, write_counter_schedule_packet,
    };
    use crate::header::EsbHeader;
    use crate::mpsl_schedule::{
        SCHEDULE_COUNTER_HINT_PAYLOAD_LEN, ScheduleHint, decode_counter_payload_schedule_hint,
    };

    #[test]
    fn pid_advances_in_two_bit_space() {
        assert_eq!(advance_pid(0), 1);
        assert_eq!(advance_pid(1), 2);
        assert_eq!(advance_pid(2), 3);
        assert_eq!(advance_pid(3), 0);
        assert_eq!(advance_pid(7), 0);
    }

    #[test]
    fn pipe_mask_round_robin_wraps_and_rejects_empty_mask() {
        assert_eq!(next_pipe_in_mask(0, 0), None);
        assert_eq!(next_pipe_in_mask(0, 0b0000_0110), Some(1));
        assert_eq!(next_pipe_in_mask(1, 0b0000_0110), Some(2));
        assert_eq!(next_pipe_in_mask(2, 0b0000_0110), Some(1));
        assert_eq!(next_pipe_in_mask(7, 0b0000_0010), Some(1));
    }

    #[test]
    fn counter_packet_uses_header_helpers_and_little_endian_payload() {
        let mut buf = [0u8; 256];

        write_counter_packet(&mut buf, 2, 0x4433_2211);

        let header = unsafe { &*(buf.as_ptr().cast::<EsbHeader>()) };
        assert_eq!(header.length, 4);
        assert_eq!(header.pid(), 2);
        assert!(!header.no_ack());
        assert_eq!(read_counter_payload(&buf), Some(0x4433_2211));
    }

    #[test]
    fn counter_payload_packet_keeps_counter_prefix_and_app_payload() {
        let mut buf = [0u8; 256];

        assert!(write_counter_payload_packet(
            &mut buf,
            3,
            0x0403_0201,
            &[0xAA, 0xBB, 0xCC],
        ));

        let header = unsafe { &*(buf.as_ptr().cast::<EsbHeader>()) };
        assert_eq!(header.length, 7);
        assert_eq!(header.pid(), 3);
        assert!(!header.no_ack());
        assert_eq!(read_counter_payload(&buf), Some(0x0403_0201));
        assert_eq!(
            &buf[EsbHeader::PAYLOAD_OFFSET + 4..EsbHeader::PAYLOAD_OFFSET + 7],
            &[0xAA, 0xBB, 0xCC]
        );
    }

    #[test]
    fn counter_payload_packet_rejects_oversized_payload() {
        let mut buf = [0u8; 256];
        let payload = [0x55; EsbHeader::MAX_PAYLOAD as usize - 3];

        assert!(!write_counter_payload_packet(&mut buf, 0, 1, &payload));
    }

    #[test]
    fn counter_payload_rejects_short_buffers_or_short_payloads() {
        let mut buf = [0u8; 8];
        buf[EsbHeader::DMA_OFFSET] = 3;
        assert_eq!(read_counter_payload(&buf), None);
        assert_eq!(
            read_counter_payload(&buf[..EsbHeader::PAYLOAD_OFFSET + 3]),
            None
        );
    }

    #[test]
    fn scheduled_counter_packet_keeps_legacy_counter_prefix() {
        let mut buf = [0u8; 256];
        let hint = ScheduleHint::new(0, 9, 1500, 5000, 4500);

        write_counter_schedule_packet(&mut buf, 1, 0x4433_2211, hint);

        let header = unsafe { &*(buf.as_ptr().cast::<EsbHeader>()) };
        assert_eq!(header.length, SCHEDULE_COUNTER_HINT_PAYLOAD_LEN as u8);
        assert_eq!(header.pid(), 1);
        assert_eq!(read_counter_payload(&buf), Some(0x4433_2211));

        let payload = &buf[EsbHeader::PAYLOAD_OFFSET
            ..EsbHeader::PAYLOAD_OFFSET + SCHEDULE_COUNTER_HINT_PAYLOAD_LEN];
        assert_eq!(decode_counter_payload_schedule_hint(payload), Ok(hint));
    }

    #[test]
    fn scheduled_counter_packet_can_append_ack_extension() {
        let mut buf = [0u8; 256];
        let hint = ScheduleHint::new(0, 9, 1500, 5000, 4500);
        let extension = [0x54, 0x41, 2, 7];

        write_counter_schedule_extension_packet(&mut buf, 1, 0x4433_2211, hint, &extension);

        let header = unsafe { &*(buf.as_ptr().cast::<EsbHeader>()) };
        assert_eq!(
            header.length,
            (SCHEDULE_COUNTER_HINT_PAYLOAD_LEN + extension.len()) as u8
        );
        assert_eq!(read_counter_payload(&buf), Some(0x4433_2211));

        let payload = EsbHeader::PAYLOAD_OFFSET;
        let ack_start = payload + SCHEDULE_COUNTER_HINT_PAYLOAD_LEN;
        assert_eq!(&buf[ack_start..ack_start + extension.len()], extension);
    }

    #[test]
    fn per_pipe_delta_saturates() {
        let current = [10, 0, 5, 1, 2, 3, 4, 5];
        let previous = [4, 1, 10, 1, 0, 10, 4, 6];
        assert_eq!(
            delta_per_pipe(&current, &previous),
            [6, 0, 0, 0, 2, 0, 0, 0]
        );
        assert_eq!(
            delta_per_pipe(&[0; NUM_PIPES], &[1; NUM_PIPES]),
            [0; NUM_PIPES]
        );
    }

    #[test]
    fn spin_until_returns_false_at_limit_and_true_when_condition_flips() {
        assert!(!spin_until(|| false, 3));

        let mut polls = 0;
        assert!(spin_until(
            || {
                polls += 1;
                polls == 3
            },
            5
        ));
    }
}
