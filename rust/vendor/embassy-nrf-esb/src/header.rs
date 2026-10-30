//! ESB packet header layout for DMA.
//!
//! The hardware DMA header is 2 bytes (length + PID/NO_ACK).
//! The software header adds 2 bytes (RSSI + pipe) before the DMA header,
//! giving a 4-byte struct.
//!
//! # On-air S1 field layout (3 bits)
//!
//! The nRF RADIO S1 field is 3 bits wide (PCNF0.S1LEN=3). The lower 3 bits
//! of `pid_no_ack` map to the S1 field:
//! - Bit 0: ACK flag (1 = transmitter requests ACK, Nordic ACK = !noack)
//! - Bits 2:1: PID (2-bit packet identifier for duplicate detection)

/// ESB on-air packet header as stored in the DMA buffer.
///
/// Layout (do not reorder — hardware-dependent):
/// ```text
/// byte 0: rssi       [SW] — filled after RX, not transmitted
/// byte 1: pipe       [SW] — pipe number, not transmitted
/// byte 2: length     [HW] — RADIO PCNF0.LFLEN field
/// byte 3: pid_no_ack [HW] — RADIO PCNF0.S1LEN field (3 bits used)
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EsbHeader {
    /// RSSI value (software-only, filled after receive).
    pub rssi: u8,
    /// Pipe number (software-only, identifies which pipe received).
    pub pipe: u8,
    /// Payload length (hardware DMA field).
    pub length: u8,
    /// PID (bits 2:1) and ACK flag (bit 0). Historical field name retained.
    pub pid_no_ack: u8,
}

impl EsbHeader {
    /// Offset from struct start to the hardware DMA fields (length, pid_no_ack).
    pub const DMA_OFFSET: usize = 2;

    /// Offset from struct start to the payload data (after full 4-byte header).
    pub const PAYLOAD_OFFSET: usize = 4;

    /// Maximum ESB payload length.
    pub const MAX_PAYLOAD: u8 = 252;

    /// Extract PID (2-bit packet identifier, bits 2:1).
    pub fn pid(&self) -> u8 {
        (self.pid_no_ack >> 1) & 0x03
    }

    /// Set PID value (only lower 2 bits used, stored in bits 2:1).
    pub fn set_pid(&mut self, pid: u8) {
        self.pid_no_ack = (self.pid_no_ack & !0x06) | ((pid & 0x03) << 1);
    }

    /// Check if no ACK is requested (on-air ACK bit 0 is clear).
    pub fn no_ack(&self) -> bool {
        self.pid_no_ack & 0x01 == 0
    }

    /// Set semantic NO_ACK, encoding its inverse in on-air bit 0.
    pub fn set_no_ack(&mut self, no_ack: bool) {
        if no_ack {
            self.pid_no_ack &= !0x01;
        } else {
            self.pid_no_ack |= 0x01;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::EsbHeader;

    #[test]
    fn pid_uses_bits_one_and_two_only() {
        let mut header = EsbHeader::default();

        for pid in 0u8..=7 {
            header.pid_no_ack = 0b1111_1001;
            header.set_pid(pid);

            assert_eq!(header.pid(), pid & 0x03);
            assert_eq!(header.pid_no_ack & 0x01, 0x01);
            assert_eq!(header.pid_no_ack & 0b1111_1000, 0b1111_1000);
        }
    }

    #[test]
    fn no_ack_uses_bit_zero_only() {
        let mut header = EsbHeader {
            pid_no_ack: 0b1010_0110,
            ..Default::default()
        };

        header.set_no_ack(true);
        assert!(header.no_ack());
        assert_eq!(header.pid_no_ack, 0b1010_0110);

        header.set_no_ack(false);
        assert!(!header.no_ack());
        assert_eq!(header.pid_no_ack, 0b1010_0111);
    }

    #[test]
    fn nordic_dma_header_golden_vectors() {
        // Literal wire values, independent of the encoder's arithmetic.
        let vectors = [(0, 0, 1), (1, 2, 3), (2, 4, 5), (3, 6, 7)];
        for (pid, no_ack_byte, ack_byte) in vectors {
            for length in 0..=EsbHeader::MAX_PAYLOAD {
                for (no_ack, wire_byte) in [(true, no_ack_byte), (false, ack_byte)] {
                    let mut header = EsbHeader {
                        length,
                        ..Default::default()
                    };
                    header.set_no_ack(no_ack);
                    header.set_pid(pid);
                    let bytes = unsafe {
                        core::slice::from_raw_parts((&header as *const EsbHeader).cast::<u8>(), 4)
                    };
                    assert_eq!(&bytes[EsbHeader::DMA_OFFSET..], &[length, wire_byte]);
                    let decoded = EsbHeader {
                        length,
                        pid_no_ack: wire_byte,
                        ..Default::default()
                    };
                    assert_eq!(decoded.pid(), pid);
                    assert_eq!(decoded.no_ack(), no_ack);
                }
            }
        }
    }

    #[test]
    fn dma_and_payload_offsets_match_layout() {
        assert_eq!(EsbHeader::DMA_OFFSET, 2);
        assert_eq!(EsbHeader::PAYLOAD_OFFSET, 4);
        assert_eq!(core::mem::size_of::<EsbHeader>(), 4);
        assert_eq!(core::mem::align_of::<EsbHeader>(), 1);
    }
}
