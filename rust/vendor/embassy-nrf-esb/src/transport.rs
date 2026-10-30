//! Optional framing helpers for higher-level split protocols.
//!
//! The ESB core transmits arbitrary payload bytes and does not interpret this
//! format. Applications may use these helpers in a board-level transport
//! adapter for routing, duplicate suppression, and application acknowledgements.

use crate::error::Error;
use crate::header::EsbHeader;

/// Current ESB transport framing version.
pub const TRANSPORT_VERSION: u8 = 1;

/// Header size in bytes.
pub const TRANSPORT_HEADER_LEN: usize = 5;
/// Maximum higher-level payload length that can fit in one framed ESB packet.
pub const MAX_TRANSPORT_PAYLOAD_LEN: usize = EsbHeader::MAX_PAYLOAD as usize - TRANSPORT_HEADER_LEN;

/// Frame carries an application-level acknowledgement.
pub const FLAG_ACK: u8 = 0x01;
/// Frame carries a retransmitted application message.
pub const FLAG_RETRANSMIT: u8 = 0x02;
/// Flags currently defined by transport version 1.
pub const FLAGS_V1_MASK: u8 = FLAG_ACK | FLAG_RETRANSMIT;

/// Transport acknowledgement marker carried inside an ESB ACK payload.
pub const TRANSPORT_ACK_MAGIC: u16 = 0x4154; // "TA", little-endian on wire.
/// Encoded transport acknowledgement length in bytes.
pub const TRANSPORT_ACK_LEN: usize = 4;

/// Return the ESB payload length required to carry a framed higher-level
/// payload of `payload_len` bytes.
pub const fn required_esb_payload_len(payload_len: usize) -> usize {
    TRANSPORT_HEADER_LEN + payload_len
}

/// Return whether an ESB `payload_length` can carry a framed higher-level
/// payload of `payload_len` bytes.
pub const fn fits_esb_payload(esb_payload_len: u8, payload_len: usize) -> bool {
    payload_len <= MAX_TRANSPORT_PAYLOAD_LEN
        && esb_payload_len <= EsbHeader::MAX_PAYLOAD
        && required_esb_payload_len(payload_len) <= esb_payload_len as usize
}

/// Validate that an ESB `payload_length` can carry a framed higher-level
/// payload of `payload_len` bytes.
///
/// This is intentionally separate from `EsbConfig::validate()`: the base ESB
/// config cannot know which higher-level transport, if any, will wrap the
/// payload. Split-protocol adapters should call this during their own board or
/// transport configuration validation.
pub const fn validate_payload_length(esb_payload_len: u8, payload_len: usize) -> Result<(), Error> {
    if fits_esb_payload(esb_payload_len, payload_len) {
        Ok(())
    } else {
        Err(Error::InvalidParam)
    }
}

/// Header used by higher-level split protocols carried over ESB.
///
/// Layout:
///
/// ```text
/// byte 0: protocol version
/// byte 1: device id
/// byte 2: sequence number
/// byte 3: flags
/// byte 4: payload length
/// byte 5..: serialized higher-level payload
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransportHeader {
    /// Framing version. Must match [`TRANSPORT_VERSION`].
    pub version: u8,
    /// Application-level device id, which an application may map through a
    /// static binding table.
    pub device_id: u8,
    /// Application-level sequence number for duplicate suppression.
    pub sequence: u8,
    /// Transport flags reserved for future use.
    pub flags: u8,
    /// Serialized payload length after the header.
    pub payload_len: u8,
}

impl TransportHeader {
    /// Create a v1 header.
    pub fn new(device_id: u8, sequence: u8, flags: u8, payload_len: usize) -> Result<Self, Error> {
        if payload_len > MAX_TRANSPORT_PAYLOAD_LEN || flags & !FLAGS_V1_MASK != 0 {
            return Err(Error::InvalidParam);
        }

        Ok(Self {
            version: TRANSPORT_VERSION,
            device_id,
            sequence,
            flags,
            payload_len: payload_len as u8,
        })
    }

    /// Encode the header into the beginning of `out`.
    pub fn encode_into(&self, out: &mut [u8]) -> Result<usize, Error> {
        if self.version != TRANSPORT_VERSION || out.len() < TRANSPORT_HEADER_LEN {
            return Err(Error::InvalidParam);
        }

        out[0] = self.version;
        out[1] = self.device_id;
        out[2] = self.sequence;
        out[3] = self.flags;
        out[4] = self.payload_len;
        Ok(TRANSPORT_HEADER_LEN)
    }

    /// Decode a header from a full frame buffer.
    pub fn decode(frame: &[u8]) -> Result<Self, Error> {
        if frame.len() < TRANSPORT_HEADER_LEN {
            return Err(Error::InvalidParam);
        }

        let header = Self {
            version: frame[0],
            device_id: frame[1],
            sequence: frame[2],
            flags: frame[3],
            payload_len: frame[4],
        };

        if header.version != TRANSPORT_VERSION {
            return Err(Error::InvalidParam);
        }

        if header.flags & !FLAGS_V1_MASK != 0 {
            return Err(Error::InvalidParam);
        }

        let total_len = TRANSPORT_HEADER_LEN + header.payload_len as usize;
        if frame.len() < total_len {
            return Err(Error::InvalidParam);
        }

        Ok(header)
    }
}

/// Encode a complete frame into `out`.
pub fn encode_frame(
    device_id: u8,
    sequence: u8,
    flags: u8,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, Error> {
    let header = TransportHeader::new(device_id, sequence, flags, payload.len())?;
    let total_len = TRANSPORT_HEADER_LEN + payload.len();
    if out.len() < total_len {
        return Err(Error::InvalidParam);
    }

    header.encode_into(out)?;
    out[TRANSPORT_HEADER_LEN..total_len].copy_from_slice(payload);
    Ok(total_len)
}

/// Decode a complete frame into `(header, payload)`.
pub fn decode_frame(frame: &[u8]) -> Result<(TransportHeader, &[u8]), Error> {
    let header = TransportHeader::decode(frame)?;
    let start = TRANSPORT_HEADER_LEN;
    let end = start + header.payload_len as usize;
    Ok((header, &frame[start..end]))
}

/// Application-level acknowledgement for one accepted transport frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransportAck {
    pub device_id: u8,
    pub sequence: u8,
}

impl TransportAck {
    pub const fn new(device_id: u8, sequence: u8) -> Self {
        Self {
            device_id,
            sequence,
        }
    }

    pub fn matches(self, device_id: u8, sequence: u8) -> bool {
        self.device_id == device_id && self.sequence == sequence
    }
}

/// Encode a transport acknowledgement into `out`.
pub fn encode_transport_ack(ack: TransportAck, out: &mut [u8]) -> Result<(), Error> {
    if out.len() < TRANSPORT_ACK_LEN {
        return Err(Error::InvalidParam);
    }

    out[0..2].copy_from_slice(&TRANSPORT_ACK_MAGIC.to_le_bytes());
    out[2] = ack.device_id;
    out[3] = ack.sequence;
    Ok(())
}

/// Decode a transport acknowledgement from `buf`.
pub fn decode_transport_ack(buf: &[u8]) -> Result<TransportAck, Error> {
    if buf.len() < TRANSPORT_ACK_LEN {
        return Err(Error::InvalidParam);
    }

    let magic = u16::from_le_bytes([buf[0], buf[1]]);
    if magic != TRANSPORT_ACK_MAGIC {
        return Err(Error::InvalidParam);
    }

    Ok(TransportAck::new(buf[2], buf[3]))
}

/// Decode and validate a frame received on `pipe`.
///
/// This helper implements a common split-device receive flow:
///
/// 1. Decode the transport header.
/// 2. Check the static `pipe -> device_id` binding.
/// 3. Drop duplicate `device_id + sequence` messages.
///
/// Returns `Ok(Some(...))` for a new accepted frame, `Ok(None)` for a duplicate,
/// and `Err(Error::InvalidParam)` for malformed frames, binding mismatches, or
/// device ids outside the sequence tracker.
pub fn accept_bound_frame<'a, const PIPES: usize, const DEVICES: usize>(
    bindings: &StaticBindingTable<PIPES>,
    tracker: &mut SequenceTracker<DEVICES>,
    pipe: u8,
    frame: &'a [u8],
) -> Result<Option<(TransportHeader, &'a [u8])>, Error> {
    let (header, payload) = decode_frame(frame)?;
    if !bindings.accepts(pipe, header.device_id)? {
        return Err(Error::InvalidParam);
    }

    if !tracker.accept(header.device_id, header.sequence)? {
        return Ok(None);
    }

    Ok(Some((header, payload)))
}

/// Per-device sequence tracker for duplicate suppression above ESB PID/CRC.
///
/// ESB PID detects radio-level repeats on a pipe. Applications that can retry
/// above ESB still need deduplication keyed by `device_id + sequence`.
#[derive(Debug, Clone, Copy)]
pub struct SequenceTracker<const N: usize> {
    valid: [bool; N],
    last: [u8; N],
}

impl<const N: usize> SequenceTracker<N> {
    /// Create an empty tracker.
    pub const fn new() -> Self {
        Self {
            valid: [false; N],
            last: [0; N],
        }
    }

    /// Return `Ok(true)` for a new sequence, `Ok(false)` for a duplicate.
    pub fn accept(&mut self, device_id: u8, sequence: u8) -> Result<bool, Error> {
        let index = device_id as usize;
        if index >= N {
            return Err(Error::InvalidParam);
        }

        if self.valid[index] && self.last[index] == sequence {
            return Ok(false);
        }

        self.valid[index] = true;
        self.last[index] = sequence;
        Ok(true)
    }

    /// Clear one device entry.
    pub fn reset_device(&mut self, device_id: u8) -> Result<(), Error> {
        let index = device_id as usize;
        if index >= N {
            return Err(Error::InvalidParam);
        }

        self.valid[index] = false;
        self.last[index] = 0;
        Ok(())
    }
}

impl<const N: usize> Default for SequenceTracker<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Static binding table for validating `pipe -> device_id` routing.
///
/// Each entry maps a local ESB pipe to the application-level device id that is
/// allowed to send frames on that pipe. This is intentionally small and generic;
/// pairing, persistence, and policy belong in the higher-level firmware.
#[derive(Debug, Clone, Copy)]
pub struct StaticBindingTable<const N: usize> {
    pipe_to_device: [Option<u8>; N],
}

impl<const N: usize> StaticBindingTable<N> {
    /// Create an empty binding table.
    pub const fn new() -> Self {
        Self {
            pipe_to_device: [None; N],
        }
    }

    /// Create a binding table from pipe-indexed entries.
    pub const fn from_pipe_entries(pipe_to_device: [Option<u8>; N]) -> Self {
        Self { pipe_to_device }
    }

    /// Bind one pipe to one device id.
    pub fn bind(&mut self, pipe: u8, device_id: u8) -> Result<(), Error> {
        let index = pipe as usize;
        if index >= N {
            return Err(Error::InvalidParam);
        }

        self.pipe_to_device[index] = Some(device_id);
        Ok(())
    }

    /// Clear one pipe binding.
    pub fn unbind(&mut self, pipe: u8) -> Result<(), Error> {
        let index = pipe as usize;
        if index >= N {
            return Err(Error::InvalidParam);
        }

        self.pipe_to_device[index] = None;
        Ok(())
    }

    /// Return the device bound to `pipe`, if any.
    pub fn device_for_pipe(&self, pipe: u8) -> Result<Option<u8>, Error> {
        let index = pipe as usize;
        if index >= N {
            return Err(Error::InvalidParam);
        }

        Ok(self.pipe_to_device[index])
    }

    /// Check whether a frame from `device_id` is allowed on `pipe`.
    pub fn accepts(&self, pipe: u8, device_id: u8) -> Result<bool, Error> {
        Ok(self.device_for_pipe(pipe)? == Some(device_id))
    }
}

impl<const N: usize> Default for StaticBindingTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        FLAG_ACK, FLAG_RETRANSMIT, FLAGS_V1_MASK, MAX_TRANSPORT_PAYLOAD_LEN, SequenceTracker,
        StaticBindingTable, TRANSPORT_ACK_LEN, TRANSPORT_HEADER_LEN, TRANSPORT_VERSION,
        TransportAck, TransportHeader, accept_bound_frame, decode_frame, decode_transport_ack,
        encode_frame, encode_transport_ack, fits_esb_payload, required_esb_payload_len,
        validate_payload_length,
    };
    use crate::error::Error;

    #[test]
    fn frame_round_trips_header_and_payload() {
        let payload = [1, 2, 3, 4];
        let mut frame = [0u8; 16];

        let flags = FLAG_ACK | FLAG_RETRANSMIT;
        let len = encode_frame(2, 7, flags, &payload, &mut frame).unwrap();
        assert_eq!(len, TRANSPORT_HEADER_LEN + payload.len());

        let (header, decoded_payload) = decode_frame(&frame[..len]).unwrap();
        assert_eq!(
            header,
            TransportHeader {
                version: TRANSPORT_VERSION,
                device_id: 2,
                sequence: 7,
                flags,
                payload_len: 4,
            }
        );
        assert_eq!(decoded_payload, payload);
    }

    #[test]
    fn transport_ack_round_trips() {
        let mut buf = [0u8; TRANSPORT_ACK_LEN];
        let ack = TransportAck::new(3, 9);

        encode_transport_ack(ack, &mut buf).unwrap();

        assert_eq!(decode_transport_ack(&buf), Ok(ack));
        assert_eq!(decode_transport_ack(&buf[..3]), Err(Error::InvalidParam));
        buf[0] = 0;
        assert_eq!(decode_transport_ack(&buf), Err(Error::InvalidParam));
    }

    #[test]
    fn rejects_truncated_or_wrong_version_frames() {
        assert_eq!(decode_frame(&[]), Err(Error::InvalidParam));
        assert_eq!(
            decode_frame(&[TRANSPORT_VERSION, 1, 2, 3, 4]),
            Err(Error::InvalidParam)
        );
        assert_eq!(decode_frame(&[0xff, 1, 2, 3, 0]), Err(Error::InvalidParam));
        assert_eq!(
            decode_frame(&[TRANSPORT_VERSION, 1, 2, 0x80, 0]),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn decode_frame_ignores_trailing_bytes_after_declared_payload() {
        let frame = [TRANSPORT_VERSION, 1, 2, 0, 1, 0xAA, 0xBB, 0xCC];

        let (header, payload) = decode_frame(&frame).unwrap();
        assert_eq!(header.device_id, 1);
        assert_eq!(header.sequence, 2);
        assert_eq!(header.payload_len, 1);
        assert_eq!(payload, &[0xAA]);
    }

    #[test]
    fn encode_frame_requires_output_capacity() {
        let mut frame = [0u8; TRANSPORT_HEADER_LEN];
        assert_eq!(
            encode_frame(0, 0, 0, &[1], &mut frame),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn required_payload_length_includes_transport_header() {
        assert_eq!(required_esb_payload_len(0), TRANSPORT_HEADER_LEN);
        assert_eq!(required_esb_payload_len(32), TRANSPORT_HEADER_LEN + 32);
        assert!(fits_esb_payload(37, 32));
        assert!(!fits_esb_payload(36, 32));
        assert_eq!(validate_payload_length(37, 32), Ok(()));
        assert_eq!(validate_payload_length(36, 32), Err(Error::InvalidParam));
    }

    #[test]
    fn framed_payload_cannot_exceed_single_esb_packet_capacity() {
        assert_eq!(MAX_TRANSPORT_PAYLOAD_LEN, 247);
        assert!(TransportHeader::new(0, 0, 0, MAX_TRANSPORT_PAYLOAD_LEN).is_ok());
        assert_eq!(
            TransportHeader::new(0, 0, 0, MAX_TRANSPORT_PAYLOAD_LEN + 1),
            Err(Error::InvalidParam)
        );

        let payload = [0u8; MAX_TRANSPORT_PAYLOAD_LEN + 1];
        let mut frame = [0u8; TRANSPORT_HEADER_LEN + MAX_TRANSPORT_PAYLOAD_LEN + 1];
        assert_eq!(
            encode_frame(0, 0, 0, &payload, &mut frame),
            Err(Error::InvalidParam)
        );
        assert!(fits_esb_payload(252, MAX_TRANSPORT_PAYLOAD_LEN));
        assert!(!fits_esb_payload(252, MAX_TRANSPORT_PAYLOAD_LEN + 1));
    }

    #[test]
    fn sequence_tracker_accepts_new_and_rejects_duplicates_per_device() {
        let mut tracker = SequenceTracker::<2>::new();

        assert_eq!(tracker.accept(0, 1), Ok(true));
        assert_eq!(tracker.accept(0, 1), Ok(false));
        assert_eq!(tracker.accept(0, 2), Ok(true));
        assert_eq!(tracker.accept(1, 1), Ok(true));
        assert_eq!(tracker.accept(2, 1), Err(Error::InvalidParam));

        tracker.reset_device(0).unwrap();
        assert_eq!(tracker.accept(0, 2), Ok(true));
    }

    #[test]
    fn static_binding_table_validates_pipe_to_device_mapping() {
        let mut bindings = StaticBindingTable::<4>::new();

        assert_eq!(bindings.accepts(1, 7), Ok(false));
        bindings.bind(1, 7).unwrap();

        assert_eq!(bindings.device_for_pipe(1), Ok(Some(7)));
        assert_eq!(bindings.accepts(1, 7), Ok(true));
        assert_eq!(bindings.accepts(1, 8), Ok(false));
        assert_eq!(bindings.accepts(2, 7), Ok(false));
        assert_eq!(bindings.bind(4, 1), Err(Error::InvalidParam));

        bindings.unbind(1).unwrap();
        assert_eq!(bindings.device_for_pipe(1), Ok(None));
    }

    #[test]
    fn static_binding_table_can_be_const_initialized() {
        const BINDINGS: StaticBindingTable<3> =
            StaticBindingTable::from_pipe_entries([Some(10), None, Some(12)]);

        assert_eq!(BINDINGS.accepts(0, 10), Ok(true));
        assert_eq!(BINDINGS.accepts(2, 12), Ok(true));
        assert_eq!(BINDINGS.accepts(1, 10), Ok(false));
    }

    #[test]
    fn accept_bound_frame_validates_binding_and_drops_duplicates() {
        let bindings = StaticBindingTable::<4>::from_pipe_entries([None, Some(7), None, None]);
        let mut tracker = SequenceTracker::<8>::new();
        let mut frame = [0u8; 16];
        let len = encode_frame(7, 42, 0, &[0xAA, 0xBB], &mut frame).unwrap();

        let accepted = accept_bound_frame(&bindings, &mut tracker, 1, &frame[..len])
            .unwrap()
            .expect("new frame");
        assert_eq!(accepted.0.device_id, 7);
        assert_eq!(accepted.0.sequence, 42);
        assert_eq!(accepted.1, &[0xAA, 0xBB]);

        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 1, &frame[..len]),
            Ok(None)
        );
        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 2, &frame[..len]),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn accept_bound_frame_rejects_unbound_or_out_of_range_routes() {
        let bindings = StaticBindingTable::<2>::from_pipe_entries([Some(0), None]);
        let mut tracker = SequenceTracker::<1>::new();
        let mut frame = [0u8; 16];

        let len = encode_frame(0, 1, 0, &[0x10], &mut frame).unwrap();
        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 1, &frame[..len]),
            Err(Error::InvalidParam)
        );
        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 2, &frame[..len]),
            Err(Error::InvalidParam)
        );

        let len = encode_frame(1, 1, 0, &[0x10], &mut frame).unwrap();
        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 0, &frame[..len]),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn transport_ack_matches_only_when_device_and_sequence_agree() {
        let ack = TransportAck::new(7, 42);

        assert!(ack.matches(7, 42));
        assert!(!ack.matches(8, 42));
        assert!(!ack.matches(7, 43));
        assert!(!ack.matches(0, 42));
        assert!(!ack.matches(7, 0));
    }

    #[test]
    fn transport_ack_matches_at_u8_boundaries() {
        let lo = TransportAck::new(0, 0);
        assert!(lo.matches(0, 0));
        assert!(!lo.matches(255, 0));
        assert!(!lo.matches(0, 255));

        let hi = TransportAck::new(255, 255);
        assert!(hi.matches(255, 255));
        assert!(!hi.matches(0, 255));
        assert!(!hi.matches(255, 0));
    }

    #[test]
    fn encode_decode_transport_ack_round_trips_typical_and_boundary_values() {
        for (device_id, sequence) in [(0u8, 0u8), (1, 1), (3, 9), (254, 255), (255, 255), (255, 0)]
        {
            let mut buf = [0u8; TRANSPORT_ACK_LEN];
            let ack = TransportAck::new(device_id, sequence);

            encode_transport_ack(ack, &mut buf).unwrap();

            // Wire layout: magic (LE) + device_id + sequence.
            assert_eq!(&buf[0..2], &[0x54, 0x41]);
            assert_eq!(buf[2], device_id);
            assert_eq!(buf[3], sequence);

            assert_eq!(decode_transport_ack(&buf), Ok(ack));
        }
    }

    #[test]
    fn decode_transport_ack_rejects_undersized_buffer() {
        // Every strict prefix shorter than TRANSPORT_ACK_LEN must be rejected.
        let mut buf = [0u8; TRANSPORT_ACK_LEN];
        encode_transport_ack(TransportAck::new(1, 2), &mut buf).unwrap();

        for prefix in 0..TRANSPORT_ACK_LEN {
            assert_eq!(
                decode_transport_ack(&buf[..prefix]),
                Err(Error::InvalidParam),
                "prefix of {prefix} bytes should be rejected"
            );
        }
    }

    #[test]
    fn decode_transport_ack_rejects_wrong_magic() {
        let mut buf = [0u8; TRANSPORT_ACK_LEN];
        encode_transport_ack(TransportAck::new(1, 2), &mut buf).unwrap();

        // Flip each byte of the magic; decode must reject all of them.
        buf[0] = !buf[0];
        assert_eq!(decode_transport_ack(&buf), Err(Error::InvalidParam));

        encode_transport_ack(TransportAck::new(1, 2), &mut buf).unwrap();
        buf[1] = !buf[1];
        assert_eq!(decode_transport_ack(&buf), Err(Error::InvalidParam));

        // And a fully bogus magic.
        buf[0] = 0xff;
        buf[1] = 0xff;
        assert_eq!(decode_transport_ack(&buf), Err(Error::InvalidParam));
    }

    #[test]
    fn decode_transport_ack_ignores_trailing_bytes() {
        // decode_transport_ack only consumes the first TRANSPORT_ACK_LEN bytes;
        // trailing bytes (e.g. embedded inside a longer ESB ACK payload) are
        // left for the caller to interpret.
        let mut buf = [0u8; TRANSPORT_ACK_LEN + 3];
        encode_transport_ack(TransportAck::new(4, 5), &mut buf).unwrap();
        buf[TRANSPORT_ACK_LEN] = 0xde;
        buf[TRANSPORT_ACK_LEN + 1] = 0xad;
        buf[TRANSPORT_ACK_LEN + 2] = 0xbe;

        assert_eq!(decode_transport_ack(&buf), Ok(TransportAck::new(4, 5)));
    }

    #[test]
    fn encode_transport_ack_accepts_larger_buffer_without_touching_trailing_bytes() {
        let mut buf = [0xffu8; TRANSPORT_ACK_LEN + 2];
        encode_transport_ack(TransportAck::new(2, 8), &mut buf).unwrap();

        assert_eq!(&buf[0..2], &[0x54, 0x41]);
        assert_eq!(buf[2], 2);
        assert_eq!(buf[3], 8);
        // Trailing bytes outside the encoded range stay untouched.
        assert_eq!(&buf[TRANSPORT_ACK_LEN..], &[0xff, 0xff]);
    }

    #[test]
    fn encode_transport_ack_rejects_undersized_output() {
        let mut buf = [0u8; TRANSPORT_ACK_LEN];
        encode_transport_ack(TransportAck::new(1, 1), &mut buf).unwrap();

        for cap in 0..TRANSPORT_ACK_LEN {
            assert_eq!(
                encode_transport_ack(TransportAck::new(1, 1), &mut buf[..cap]),
                Err(Error::InvalidParam),
                "capacity of {cap} bytes should be rejected"
            );
        }
    }

    #[test]
    fn v1_flags_are_disjoint_and_combine_into_mask() {
        assert_eq!(FLAG_ACK, 0x01);
        assert_eq!(FLAG_RETRANSMIT, 0x02);
        assert_eq!(FLAGS_V1_MASK, FLAG_ACK | FLAG_RETRANSMIT);
        assert_eq!(FLAGS_V1_MASK, 0x03);

        // No shared bits: the two flags can be set independently and together.
        assert_eq!(FLAG_ACK & FLAG_RETRANSMIT, 0);
        assert_eq!(FLAG_ACK | FLAG_RETRANSMIT, FLAGS_V1_MASK);
    }

    #[test]
    fn transport_header_accepts_both_flags_set_together() {
        let both = TransportHeader::new(1, 1, FLAG_ACK | FLAG_RETRANSMIT, 0).unwrap();
        assert_eq!(both.flags, FLAG_ACK | FLAG_RETRANSMIT);

        // Each flag on its own is also valid.
        assert!(TransportHeader::new(1, 1, FLAG_ACK, 0).is_ok());
        assert!(TransportHeader::new(1, 1, FLAG_RETRANSMIT, 0).is_ok());
        assert!(TransportHeader::new(1, 1, 0, 0).is_ok());
    }

    #[test]
    fn transport_header_rejects_flags_outside_v1_mask() {
        // Every bit not in FLAGS_V1_MASK must be rejected at construction.
        for bit in 0u8..8 {
            let flag = 1u8 << bit;
            if flag & FLAGS_V1_MASK != 0 {
                continue;
            }
            assert_eq!(
                TransportHeader::new(0, 0, flag, 0),
                Err(Error::InvalidParam),
                "flag 0x{flag:02x} should be rejected"
            );
        }

        // A combined flag with any reserved bit set is also rejected.
        assert_eq!(
            TransportHeader::new(0, 0, FLAG_ACK | 0x80, 0),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn retransmit_flag_survives_full_frame_round_trip() {
        let payload = [0x10, 0x20, 0x30];
        let mut frame = [0u8; 16];
        let len = encode_frame(5, 99, FLAG_RETRANSMIT, &payload, &mut frame).unwrap();

        let (header, decoded_payload) = decode_frame(&frame[..len]).unwrap();
        assert_eq!(header.flags, FLAG_RETRANSMIT);
        assert_eq!(header.device_id, 5);
        assert_eq!(header.sequence, 99);
        assert_eq!(decoded_payload, payload);
    }

    #[test]
    fn ack_and_retransmit_flags_round_trip_together_through_frame() {
        let payload = [0xAB];
        let mut frame = [0u8; 16];
        let flags = FLAG_ACK | FLAG_RETRANSMIT;
        let len = encode_frame(6, 200, flags, &payload, &mut frame).unwrap();

        let (header, decoded_payload) = decode_frame(&frame[..len]).unwrap();
        assert_eq!(header.flags, flags);
        assert!(header.flags & FLAG_ACK != 0);
        assert!(header.flags & FLAG_RETRANSMIT != 0);
        assert_eq!(decoded_payload, payload);
    }

    #[test]
    fn accept_bound_frame_accepts_retransmitted_frame_as_new_after_reset() {
        // accept_bound_frame does not itself interpret FLAG_RETRANSMIT; it only
        // does binding + sequence dedup. A retransmitted sequence number is
        // dropped as a duplicate until the tracker entry is reset.
        let bindings = StaticBindingTable::<2>::from_pipe_entries([Some(3), None]);
        let mut tracker = SequenceTracker::<8>::new();
        let mut frame = [0u8; 16];
        let len = encode_frame(3, 17, FLAG_RETRANSMIT, &[0x01], &mut frame).unwrap();

        // First arrival: accepted.
        let accepted = accept_bound_frame(&bindings, &mut tracker, 0, &frame[..len])
            .unwrap()
            .expect("first copy of retransmitted frame should be accepted");
        assert_eq!(accepted.0.flags, FLAG_RETRANSMIT);

        // Immediate repeat (same device+sequence): dropped as duplicate.
        assert_eq!(
            accept_bound_frame(&bindings, &mut tracker, 0, &frame[..len]),
            Ok(None)
        );

        // After resetting that device's slot, the same sequence is new again.
        tracker.reset_device(3).unwrap();
        let accepted_again = accept_bound_frame(&bindings, &mut tracker, 0, &frame[..len])
            .unwrap()
            .expect("frame should be accepted after device reset");
        assert_eq!(accepted_again.0.flags, FLAG_RETRANSMIT);
        assert_eq!(accepted_again.0.sequence, 17);
    }
}
