//! Canonical LC3 redundancy bundle encoding, parsing and in-place stripping.

pub const HEADER_SIZE: usize = 8;
pub const MAX_SIZE: usize = 200;
pub const FRAME_BYTES: usize = 48;
pub const CURRENT_ACTIVE: u8 = 0x01;
pub const PREVIOUS1_PRESENT: u8 = 0x02;
pub const PREVIOUS1_ACTIVE: u8 = 0x04;
pub const PREVIOUS2_PRESENT: u8 = 0x08;
pub const PREVIOUS2_ACTIVE: u8 = 0x10;
pub const RELAYED: u8 = 0x20;
pub const FLAG_MASK: u8 = 0x3f;

/// Empty predecessor slices represent absent frames. Sequence predecessors
/// are `current_seq.wrapping_sub(1)` and `current_seq.wrapping_sub(2)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BundleView<'a> {
    pub previous1_data: &'a [u8],
    pub previous2_data: &'a [u8],
    pub current_data: &'a [u8],
    pub current_seq: u16,
    pub stream_id: u8,
    pub flags: u8,
    pub codec: u8,
}

fn fields_valid(bundle: &BundleView<'_>) -> bool {
    let p1 = bundle.flags & PREVIOUS1_PRESENT != 0;
    let p2 = bundle.flags & PREVIOUS2_PRESENT != 0;
    bundle.codec == 2
        && bundle.current_data.len() == FRAME_BYTES
        && bundle.flags & !FLAG_MASK == 0
        && if p1 {
            bundle.previous1_data.len() == FRAME_BYTES
        } else {
            bundle.previous1_data.is_empty() && bundle.flags & PREVIOUS1_ACTIVE == 0
        }
        && if p2 {
            p1 && bundle.previous2_data.len() == FRAME_BYTES
        } else {
            bundle.previous2_data.is_empty() && bundle.flags & PREVIOUS2_ACTIVE == 0
        }
}

/// Returns zero without mutating output on invalid fields or capacity.
/// Source slices and output cannot alias; in-place stripping is supported below.
pub fn audio_bundle_encode(bundle: &BundleView<'_>, output: &mut [u8]) -> usize {
    if !fields_valid(bundle) {
        return 0;
    }
    let len = HEADER_SIZE
        + bundle.previous2_data.len()
        + bundle.previous1_data.len()
        + bundle.current_data.len();
    if output.len() < len {
        return 0;
    }
    let seq = bundle.current_seq.to_be_bytes();
    output[..HEADER_SIZE].copy_from_slice(&[
        bundle.codec,
        20,
        bundle.stream_id,
        bundle.flags,
        seq[0],
        seq[1],
        FRAME_BYTES as u8,
        bundle.previous1_data.len() as u8,
    ]);
    let mut offset = HEADER_SIZE;
    for frame in [
        bundle.previous2_data,
        bundle.previous1_data,
        bundle.current_data,
    ] {
        output[offset..offset + frame.len()].copy_from_slice(frame);
        offset += frame.len();
    }
    len
}

/// Parse an exact bundle, borrowing frame data in previous2/previous1/current order.
pub fn audio_bundle_parse(data: &[u8]) -> Option<BundleView<'_>> {
    if !(HEADER_SIZE..=MAX_SIZE).contains(&data.len())
        || data[0] != 2
        || data[1] != 20
        || data[3] & !FLAG_MASK != 0
    {
        return None;
    }
    let current_len = usize::from(data[6]);
    let previous1_len = usize::from(data[7]);
    let frame_len = data.len() - HEADER_SIZE;
    if current_len != FRAME_BYTES || previous1_len > 64 || current_len + previous1_len > frame_len {
        return None;
    }
    let previous2_len = frame_len - current_len - previous1_len;
    let p1_offset = HEADER_SIZE + previous2_len;
    let current_offset = p1_offset + previous1_len;
    let bundle = BundleView {
        previous2_data: &data[HEADER_SIZE..p1_offset],
        previous1_data: &data[p1_offset..current_offset],
        current_data: &data[current_offset..],
        current_seq: u16::from_be_bytes([data[4], data[5]]),
        stream_id: data[2],
        flags: data[3],
        codec: data[0],
    };
    fields_valid(&bundle).then_some(bundle)
}

/// Remove the oldest predecessor in place. Invalid input or no predecessor
/// leaves both storage and length unchanged, including trailing storage bytes.
pub fn audio_bundle_strip_oldest(data: &mut [u8], data_len: &mut usize) -> bool {
    let Some(input) = data.get(..*data_len) else {
        return false;
    };
    let Some(bundle) = audio_bundle_parse(input) else {
        return false;
    };
    let p2_len = bundle.previous2_data.len();
    let p1_len = bundle.previous1_data.len();
    if p2_len != 0 {
        data.copy_within(HEADER_SIZE + p2_len..*data_len, HEADER_SIZE);
        data[3] &= !(PREVIOUS2_PRESENT | PREVIOUS2_ACTIVE);
        *data_len -= p2_len;
        true
    } else if p1_len != 0 {
        data.copy_within(HEADER_SIZE + p1_len..*data_len, HEADER_SIZE);
        data[3] &= !(PREVIOUS1_PRESENT | PREVIOUS1_ACTIVE);
        data[7] = 0;
        *data_len = HEADER_SIZE + FRAME_BYTES;
        true
    } else {
        false
    }
}
