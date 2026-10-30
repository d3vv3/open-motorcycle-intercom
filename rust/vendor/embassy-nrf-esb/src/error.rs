//! ESB error types.

/// ESB operation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// TX buffer full — no space to queue another packet.
    TxFull,
    /// RX buffer empty — no packet available.
    RxEmpty,
    /// Invalid parameter (e.g. payload length > 252).
    InvalidParam,
    /// Maximum retransmits reached (PTX).
    MaxRetransmit,
    /// Radio not initialized or in wrong state.
    NotReady,
    /// Address configuration error.
    InvalidAddress,
    /// Packet pool exhausted — no free buffers for DMA.
    OutOfMemory,
    /// Cannot suspend — state machine is mid-transaction.
    Busy,
    /// RADIO stop needed a power-cycle; no physical success may be inferred.
    RadioFault,
    /// MPSL raw API returned an error.
    Mpsl,
}

const _: () = {
    const fn _assert_traits()
    where
        Error: Copy + Eq,
    {
    }
};
