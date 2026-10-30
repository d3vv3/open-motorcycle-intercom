//! Codec replacing the former `shared/bridge_frame.c`:
//! AA, length, sequence, type, payload, CRC8.

pub const SYNC_BYTE: u8 = 0xAA;
pub const MAX_PAYLOAD: usize = 253;
pub const OVERHEAD: usize = 5;
pub const MAX_SIZE: usize = MAX_PAYLOAD + OVERHEAD;
/// Current nRF SPI implementation limit, not the generic codec limit.
pub const NRF_SPI_MAX_PAYLOAD: usize = 208;
/// Current ESP SPI implementation limit, not the generic codec limit.
pub const ESP_SPI_MAX_PAYLOAD: usize = 251;

/// CRC8 with polynomial 0x07, initial value zero, no reflection or final XOR.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Encode into caller-owned storage, returning the frame length or zero.
/// Invalid maximum, payload length, or capacity leaves all of `dst` unchanged.
/// Bytes after the returned frame length are also left unchanged.
pub fn encode(
    dst: &mut [u8],
    max_payload: usize,
    seq: u8,
    frame_type: u8,
    payload: &[u8],
) -> usize {
    if max_payload > MAX_PAYLOAD || payload.len() > max_payload {
        return 0;
    }
    // The validated payload bound makes the addition and wire length fit.
    let frame_len = payload.len() + OVERHEAD;
    if dst.len() < frame_len {
        return 0;
    }
    dst[0] = SYNC_BYTE;
    dst[1] = (payload.len() + 2) as u8;
    dst[2] = seq;
    dst[3] = frame_type;
    dst[4..frame_len - 1].copy_from_slice(payload);
    dst[frame_len - 1] = crc8(&dst[1..frame_len - 1]);
    frame_len
}

/// Non-success results matching the original C decode result names.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeError {
    Idle,
    Truncated,
    BadSync,
    BadLength,
    BadCrc,
}

/// A successful view borrows only the declared payload, excluding CRC/padding.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameView<'a> {
    pub payload: &'a [u8],
    pub seq: u8,
    pub frame_type: u8,
}

/// Decode one frame; trailing bytes are ignored, including nonzero padding.
/// On failure no partial view is exposed. An empty slice represents no input;
/// C null pointers are deliberately not part of this safe API.
pub fn decode(frame: &[u8], max_payload: usize) -> Result<FrameView<'_>, DecodeError> {
    if max_payload > MAX_PAYLOAD {
        return Err(DecodeError::BadLength);
    }
    let Some(&sync) = frame.first() else {
        return Err(DecodeError::Truncated);
    };
    if sync == 0 {
        return Err(DecodeError::Idle);
    }
    if frame.len() < OVERHEAD {
        return Err(DecodeError::Truncated);
    }
    if sync != SYNC_BYTE {
        return Err(DecodeError::BadSync);
    }
    if frame[1] < 2 {
        return Err(DecodeError::BadLength);
    }
    let payload_len = usize::from(frame[1]) - 2;
    if payload_len > max_payload {
        return Err(DecodeError::BadLength);
    }
    let declared_len = payload_len + OVERHEAD;
    if frame.len() < declared_len {
        return Err(DecodeError::Truncated);
    }
    if frame[declared_len - 1] != crc8(&frame[1..declared_len - 1]) {
        return Err(DecodeError::BadCrc);
    }
    Ok(FrameView {
        payload: &frame[4..declared_len - 1],
        seq: frame[2],
        frame_type: frame[3],
    })
}
