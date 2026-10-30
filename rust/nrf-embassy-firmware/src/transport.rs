//! Detached, bounded producer storage, not a live firmware actor integration.
//! A zero publication result means the owned record is already in this mailbox;
//! only then may a Bridge consumer ACK. The future sole protocol actor selects
//! `Runtime::use_external_ingress` and consumes popped records directly, BEFORE
//! work/slot callbacks. Never enqueue them into Runtime's second 16-entry ring.

use nrf_embassy::protocol_audio::{
    IngressAdmissionGate, IngressDeltaAccounting, IngressToken, PreparedIngress,
};
use nrf_embassy::protocol_tx::EAGAIN;
use nrf_embassy::runtime::{Command, CommandKind, EALREADY, EINVAL};

pub const MAILBOX_RING: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportError {
    WrongContext,
    NotInitialized,
    AlreadyInitialized,
}

/// Cortex-M vector 0 alone is Thread mode (1 is Reset, not Thread mode).
pub fn thread_mode(vector: u16) -> Result<(), TransportError> {
    if vector == 0 {
        Ok(())
    } else {
        Err(TransportError::WrongContext)
    }
}

/// Production access ordering, shared with the ARM facade. `access` must return
/// an OWNED result after releasing its loan/CS. Notification may reenter access.
/// The context check precedes even an initialization lookup or critical section.
pub fn thread_access<T>(
    vector: u16,
    access: impl FnOnce() -> (T, bool),
    notify: impl FnOnce(),
) -> Result<T, TransportError> {
    thread_mode(vector)?;
    let (owned, wake) = access();
    if wake {
        notify();
    }
    Ok(owned)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PendingAccounting {
    pub ingress: IngressDeltaAccounting,
    pub bundle_bad: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PendingRequests {
    pub command: Option<Command>,
    pub status: bool,
}

/// Read-only copy; never accepted as a restore/update capability.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SharedInboxSnapshot {
    pub initialized: bool,
    pub gate: IngressAdmissionGate,
    pub queued: usize,
    pub accounting: PendingAccounting,
    pub requests: PendingRequests,
}

/// All mutable storage is private. A serialized fixed-array drop/put cannot
/// fail its second put: `failed_enqueue` is always zero. This is NOT a proof of
/// kernel msgq concurrency or real MCU mutex failure behavior.
pub struct TransportState {
    initialized: bool,
    gate: IngressAdmissionGate,
    data: [Option<PreparedIngress>; MAILBOX_RING],
    read: usize,
    write: usize,
    count: usize,
    pending: PendingAccounting,
    requests: PendingRequests,
}

impl Default for TransportState {
    fn default() -> Self {
        Self::inactive()
    }
}

impl TransportState {
    pub const fn inactive() -> Self {
        Self {
            initialized: false,
            gate: IngressAdmissionGate {
                enabled: false,
                epoch: 0,
            },
            data: [None; MAILBOX_RING],
            read: 0,
            write: 0,
            count: 0,
            pending: PendingAccounting {
                ingress: IngressDeltaAccounting {
                    overwritten: 0,
                    purged: 0,
                    failed_enqueue: 0,
                },
                bundle_bad: 0,
            },
            requests: PendingRequests {
                command: None,
                status: false,
            },
        }
    }

    /// Called once by the actor, with its actual combined core gate. Default
    /// ReceiverOnly/not-integrated ownership must pass a disabled gate.
    pub fn initialize_owned_gate(&mut self, gate: IngressAdmissionGate) -> Result<(), i32> {
        if self.initialized {
            return Err(EALREADY);
        }
        self.gate = gate;
        self.initialized = true;
        Ok(())
    }

    pub fn snapshot(&self) -> SharedInboxSnapshot {
        SharedInboxSnapshot {
            initialized: self.initialized,
            gate: self.gate,
            queued: self.count,
            accounting: self.pending,
            requests: self.requests,
        }
    }

    /// Capture epoch before validation/transfer; deliberately does not test active.
    pub fn capture(&self) -> Result<IngressToken, TransportError> {
        if !self.initialized {
            return Err(TransportError::NotInitialized);
        }
        Ok(self.gate.capture())
    }

    /// Returns EINVAL for malformed input even with closed/stale admission.
    /// Validation errors contribute only a scalar delta, never queue occupancy.
    pub fn publish(&mut self, prepared: PreparedIngress) -> i32 {
        let request = match prepared.request() {
            Ok(request) => request,
            Err(_) => {
                self.pending.bundle_bad = self.pending.bundle_bad.wrapping_add(1);
                return EINVAL;
            }
        };
        if !self.initialized || !self.gate.enabled || request.captured_epoch() != self.gate.epoch {
            return EAGAIN;
        }
        if self.count == MAILBOX_RING {
            let _ = self.pop_owned();
            self.pending.ingress.overwritten = self.pending.ingress.overwritten.wrapping_add(1);
        }
        self.data[self.write] = Some(prepared);
        self.write = (self.write + 1) % MAILBOX_RING;
        self.count += 1;
        0
    }

    pub fn pop_owned(&mut self) -> Option<PreparedIngress> {
        if self.count == 0 {
            return None;
        }
        // Copy, not take: source purge/pop leaves stale backing packets intact.
        let record = self.data[self.read];
        self.read = (self.read + 1) % MAILBOX_RING;
        self.count -= 1;
        record
    }

    /// Atomic under the facade mutex. Closing admission WITHOUT purge preserves
    /// published records for local-enabled, same-epoch actor consumption.
    /// Caller contract: every local boundary epoch change MUST pass purge=true,
    /// synchronously after pause/resume and before any cooperative SPI await.
    /// This does not infer purge from flag equality or enforce actor scheduling.
    pub fn publish_gate(&mut self, gate: IngressAdmissionGate, purge: bool) {
        if purge {
            self.pending.ingress.purged =
                self.pending.ingress.purged.wrapping_add(self.count as u32);
            self.read = 0;
            self.write = 0;
            self.count = 0;
        }
        self.gate = gate;
    }

    pub fn take_accounting(&mut self) -> PendingAccounting {
        core::mem::take(&mut self.pending)
    }

    /// Last writer, including wrapped generations; no freshness filter or FIFO.
    pub fn publish_command(&mut self, command: Command) -> i32 {
        if command.kind == CommandKind::Start && !(1..=3).contains(&command.talk_channel) {
            return EINVAL;
        }
        if !self.initialized {
            return EAGAIN;
        }
        self.requests.command = Some(command);
        0
    }

    pub fn request_status(&mut self) -> i32 {
        if !self.initialized {
            return EAGAIN;
        }
        self.requests.status = true;
        0
    }

    pub fn take_requests(&mut self) -> PendingRequests {
        core::mem::take(&mut self.requests)
    }
}

#[cfg(test)]
#[path = "transport_tests.rs"]
mod tests;
