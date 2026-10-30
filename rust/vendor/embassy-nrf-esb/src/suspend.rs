//! ESB suspend/resume saved state types.

use crate::addresses::MAX_PIPES;

/// Saved ESB protocol state that survives across suspend/resume cycles.
///
/// Stored by the caller (MPSL timeslot handler or BLE/ESB switch code)
/// between `suspend()` and `restore()` calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EsbSavedState {
    /// Per-pipe PID counters (2-bit values, for duplicate detection).
    pub pid: [u8; MAX_PIPES],
    /// Per-pipe last-received CRC (for duplicate detection on PRX side).
    pub last_crc: [u16; MAX_PIPES],
    /// Whether the duplicate-detection entry for each pipe is initialized.
    pub last_valid: [bool; MAX_PIPES],
    /// Active TX pipe at time of suspend (PTX only; ignored for PRX).
    pub tx_pipe: u8,
    /// Retransmit attempt counter at time of suspend (PTX only).
    pub attempts: u8,
    /// Protocol state at time of suspend.
    pub protocol_state: SavedProtocolState,
}

impl Default for EsbSavedState {
    fn default() -> Self {
        Self {
            pid: [0; MAX_PIPES],
            last_crc: [0; MAX_PIPES],
            last_valid: [false; MAX_PIPES],
            tx_pipe: 0,
            attempts: 0,
            protocol_state: SavedProtocolState::Idle,
        }
    }
}

/// Simplified protocol state for save/restore.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SavedProtocolState {
    /// Was idle — clean suspend.
    Idle,
    /// Was mid-transaction when forced to suspend.
    /// Packet was dropped; attempt count preserved for diagnostics.
    ForcedIdle { dropped_attempt: u8 },
}

const _: () = {
    // Compile-time trait assertions.
    const fn _assert_traits()
    where
        EsbSavedState: Copy + Eq + Default,
        SavedProtocolState: Copy + Eq,
    {
    }
};
