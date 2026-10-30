//! ARM-private single-core endpoints; no Runtime/Bridge loan crosses a task.

use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;
use cortex_m::peripheral::scb::VectActive;
use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use nrf_embassy::protocol_audio::{IngressAdmissionGate, IngressToken, PreparedIngress};
use nrf_embassy::runtime::Command;
use nrf_embassy_firmware::transport::{
    PendingAccounting, PendingRequests, SharedInboxSnapshot, TransportError, TransportState,
    thread_access, thread_mode,
};

static INBOX: Mutex<RefCell<TransportState>> = Mutex::new(RefCell::new(TransportState::inactive()));
static WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn vector() -> u16 {
    // Only ThreadMode maps to 0. Every exception/IRQ, including Reset, rejects
    // before any PRIMASK operation or RefCell access, even before initialization.
    match cortex_m::peripheral::SCB::vect_active() {
        VectActive::ThreadMode => 0,
        _ => 1,
    }
}

fn access<T>(op: impl FnOnce(&mut TransportState) -> (T, bool)) -> Result<T, TransportError> {
    thread_access(
        vector(),
        || {
            critical_section::with(|cs| {
                let mut state = INBOX.borrow(cs).borrow_mut();
                op(&mut state)
            })
        },
        || WAKE.signal(()),
    )
}

/// Opaque copied producer capability: never contains Runtime, Bridge or pointers.
#[derive(Clone, Copy)]
pub(crate) struct Producer {
    _private: (),
}

impl Producer {
    pub(crate) fn publish_gate(
        self,
        gate: IngressAdmissionGate,
        purge: bool,
    ) -> Result<(), TransportError> {
        access(|state| {
            state.publish_gate(gate, purge);
            ((), false)
        })
    }
    pub(crate) fn capture(self) -> Result<IngressToken, TransportError> {
        access(|state| (state.capture(), false))?
    }
    pub(crate) fn publish(self, prepared: PreparedIngress) -> Result<i32, TransportError> {
        access(|state| {
            if !state.snapshot().initialized {
                return (Err(TransportError::NotInitialized), false);
            }
            let rc = state.publish(prepared);
            (Ok(rc), rc != nrf_embassy::protocol_tx::EAGAIN)
        })?
    }
    pub(crate) fn command(self, command: Command) -> Result<i32, TransportError> {
        access(|state| {
            if !state.snapshot().initialized {
                return (Err(TransportError::NotInitialized), false);
            }
            let rc = state.publish_command(command);
            (Ok(rc), rc == 0)
        })?
    }
    pub(crate) fn request_status(self) -> Result<i32, TransportError> {
        access(|state| {
            if !state.snapshot().initialized {
                return (Err(TransportError::NotInitialized), false);
            }
            let rc = state.request_status();
            (Ok(rc), rc == 0)
        })?
    }
    #[allow(dead_code)] // Copied diagnostic query, not an additional actor lease.
    pub(crate) fn snapshot(self) -> Result<SharedInboxSnapshot, TransportError> {
        access(|state| (state.snapshot(), false))
    }
}

/// Unique actor lease, not Copy/Clone. No steal or public raw endpoint getter.
pub(crate) struct Actor {
    _private: (),
}

impl Actor {
    pub(crate) fn initialize_owned_gate(
        gate: IngressAdmissionGate,
    ) -> Result<(Self, Producer), TransportError> {
        access(|state| {
            (
                state
                    .initialize_owned_gate(gate)
                    .map_err(|_| TransportError::AlreadyInitialized),
                false,
            )
        })??;
        Ok((Self { _private: () }, Producer { _private: () }))
    }
    /// Actor calls synchronously after core pause/resume, before any SPI await.
    pub(crate) fn publish_gate(
        &mut self,
        gate: IngressAdmissionGate,
        purge: bool,
    ) -> Result<(), TransportError> {
        access(|state| {
            state.publish_gate(gate, purge);
            ((), false)
        })
    }
    pub(crate) fn pop_owned(&mut self) -> Result<Option<PreparedIngress>, TransportError> {
        access(|state| (state.pop_owned(), false))
    }
    pub(crate) fn take_accounting(&mut self) -> Result<PendingAccounting, TransportError> {
        access(|state| (state.take_accounting(), false))
    }
    pub(crate) fn take_requests(&mut self) -> Result<PendingRequests, TransportError> {
        access(|state| (state.take_requests(), false))
    }
    pub(crate) async fn wait(&mut self) -> Result<(), TransportError> {
        thread_mode(vector())?;
        // Signal's mutex/waker is outside every INBOX loan and CS.
        let mut wait = pin!(WAKE.wait());
        poll_fn(|cx| {
            // Recheck on EVERY poll, before Signal can enter its own mutex.
            if let Err(error) = thread_mode(vector()) {
                return Poll::Ready(Err(error));
            }
            wait.as_mut().poll(cx).map(Ok)
        })
        .await
    }
}
