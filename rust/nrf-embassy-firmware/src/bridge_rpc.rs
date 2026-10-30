//! One outgoing mutation rendezvous, not an outbox or encoded packet queue.
//! Completion means actual Bridge mutation, never physical transfer or ACK.
#![forbid(unsafe_code)]

use nrf_embassy::bridge::{Bridge, EINVAL, EMSGSIZE, MAX_PAYLOAD};
use nrf_embassy::runtime::Membership;
use openhelmet_wire::audio_bundle::audio_bundle_parse;

/// Private length and storage prevent construction of unchecked payload slices.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Payload {
    bytes: [u8; MAX_PAYLOAD],
    len: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventRequest {
    event: u8,
    data: Payload,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogRequest(Payload);
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AudioRequest {
    source: u8,
    data: Payload,
}

impl Payload {
    fn copy(data: &[u8]) -> Self {
        let mut bytes = [0; MAX_PAYLOAD];
        bytes[..data.len()].copy_from_slice(data);
        Self {
            bytes,
            len: data.len() as u8,
        }
    }
    fn data(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(clippy::large_enum_variant)] // Fixed owned storage, deliberately no heap indirection.
pub enum Request {
    Status(Membership),
    Ack {
        command: u8,
        generation: u8,
        result: i32,
    },
    Event(EventRequest),
    Log(LogRequest),
    AudioBundle(AudioRequest),
    DiscardAudio,
    Delivery(bool),
}

impl Request {
    #[cfg(feature = "slot-entry-trace")]
    pub fn kind(&self) -> RequestKind {
        match self {
            Self::Status(_) => RequestKind::Status,
            Self::Ack { .. } => RequestKind::Ack,
            Self::Event(_) => RequestKind::Event,
            Self::Log(_) => RequestKind::Log,
            Self::AudioBundle(_) => RequestKind::AudioBundle,
            Self::DiscardAudio => RequestKind::DiscardAudio,
            Self::Delivery(_) => RequestKind::Delivery,
        }
    }
    pub fn event(event: u8, data: &[u8]) -> Result<Self, i32> {
        if data.len() > MAX_PAYLOAD - 1 {
            return Err(EMSGSIZE);
        }
        Ok(Self::Event(EventRequest {
            event,
            data: Payload::copy(data),
        }))
    }
    pub fn log(data: &[u8]) -> Result<Self, i32> {
        if data.is_empty() {
            return Err(EINVAL);
        }
        // Legacy logs truncate, rather than reject, oversized input.
        Ok(Self::Log(LogRequest(Payload::copy(
            &data[..data.len().min(MAX_PAYLOAD)],
        ))))
    }
    pub fn audio_bundle(source: u8, data: &[u8]) -> Result<Self, i32> {
        // Exactly the Bridge ordering: malformed (including >200) is EINVAL.
        if audio_bundle_parse(data).is_none() {
            return Err(EINVAL);
        }
        if data.len() + 1 > MAX_PAYLOAD {
            return Err(EMSGSIZE);
        }
        Ok(Self::AudioBundle(AudioRequest {
            source,
            data: Payload::copy(data),
        }))
    }
    /// Synchronous, exclusive owner access. A borrowed Prepared must have ended;
    /// a feature-gated detached transfer permits mutation through its coordinator.
    pub fn apply(self, bridge: &mut Bridge) -> i32 {
        let result = match self {
            Self::Status(m) => bridge.send_status(
                m.state as u8,
                m.role as u8,
                m.peer_count,
                m.node_id,
                m.slot_index,
                m.coordinator_id,
            ),
            Self::Ack {
                command,
                generation,
                result,
            } => bridge.send_command_ack(command, generation, result),
            Self::Event(event) => bridge.send_event(event.event, event.data.data()),
            Self::Log(data) => bridge.send_log(data.0.data()),
            Self::AudioBundle(audio) => bridge.send_audio_v2(audio.source, audio.data.data()),
            Self::DiscardAudio => {
                bridge.discard_pending_audio();
                Ok(())
            }
            Self::Delivery(enabled) => {
                bridge.set_audio_delivery_enabled(enabled);
                Ok(())
            }
        };
        result.err().unwrap_or(0)
    }
}

/// Trace marker values; zero is reserved for no in-flight request.
#[cfg(feature = "slot-entry-trace")]
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum RequestKind {
    Status = 1,
    Ack = 2,
    Event = 3,
    Log = 4,
    AudioBundle = 5,
    DiscardAudio = 6,
    Delivery = 7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Busy,
    Stale,
    Exhausted,
    NotInitialized,
    AlreadyInitialized,
    WrongContext,
}

/// Opaque identity; no raw constructor, wrapping increment or restore API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ticket(u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Requested,
    Claimed,
    Completed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub initialized: bool,
    pub exhausted: bool,
    pub phase: Phase,
    pub abandoned: bool,
}

#[allow(clippy::large_enum_variant)] // Exactly one request, never a second queue.
enum Slot {
    Idle,
    Requested(Ticket, Request),
    Claimed(Ticket, bool),
    Completed(Ticket, i32),
}

pub struct State {
    initialized: bool,
    last: u64,
    exhausted: bool,
    slot: Slot,
}

impl State {
    pub const fn inactive() -> Self {
        Self {
            initialized: false,
            last: 0,
            exhausted: false,
            slot: Slot::Idle,
        }
    }
    pub fn initialize(&mut self) -> Result<(), Error> {
        if self.initialized {
            return Err(Error::AlreadyInitialized);
        }
        self.initialized = true;
        Ok(())
    }
    pub fn snapshot(&self) -> Snapshot {
        let (phase, abandoned) = match self.slot {
            Slot::Idle => (Phase::Idle, false),
            Slot::Requested(..) => (Phase::Requested, false),
            Slot::Claimed(_, abandoned) => (Phase::Claimed, abandoned),
            Slot::Completed(..) => (Phase::Completed, false),
        };
        Snapshot {
            initialized: self.initialized,
            exhausted: self.exhausted,
            phase,
            abandoned,
        }
    }
    pub fn reserve(&mut self, request: Request) -> Result<Ticket, Error> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }
        if self.exhausted {
            return Err(Error::Exhausted);
        }
        if !matches!(self.slot, Slot::Idle) {
            return Err(Error::Busy);
        }
        let Some(next) = self.last.checked_add(1) else {
            self.exhausted = true;
            return Err(Error::Exhausted);
        };
        self.last = next;
        let ticket = Ticket(next);
        self.slot = Slot::Requested(ticket, request);
        Ok(ticket)
    }
    /// Moves the only pending request out; repeated claims cannot copy it again.
    pub fn claim(&mut self) -> Option<(Ticket, Request)> {
        if let Slot::Requested(ticket, request) = self.slot {
            self.slot = Slot::Claimed(ticket, false);
            Some((ticket, request))
        } else {
            None
        }
    }
    /// Returns whether an actual result became available for the waiting client.
    pub fn complete(&mut self, ticket: Ticket, result: i32) -> Result<bool, Error> {
        match self.slot {
            Slot::Claimed(current, abandoned) if current == ticket => {
                self.slot = if abandoned {
                    Slot::Idle
                } else {
                    Slot::Completed(ticket, result)
                };
                Ok(!abandoned)
            }
            _ => Err(Error::Stale),
        }
    }
    pub fn take_result(&mut self, ticket: Ticket) -> Result<Option<i32>, Error> {
        match self.slot {
            Slot::Completed(current, result) if current == ticket => {
                self.slot = Slot::Idle;
                Ok(Some(result))
            }
            Slot::Requested(current, _) | Slot::Claimed(current, false) if current == ticket => {
                Ok(None)
            }
            _ => Err(Error::Stale),
        }
    }
    pub fn cancel(&mut self, ticket: Ticket) -> Result<(), Error> {
        match self.slot {
            Slot::Requested(current, _) | Slot::Completed(current, _) if current == ticket => {
                self.slot = Slot::Idle
            }
            Slot::Claimed(current, _) if current == ticket => {
                self.slot = Slot::Claimed(ticket, true)
            }
            _ => return Err(Error::Stale),
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "bridge_rpc_tests.rs"]
mod tests;
