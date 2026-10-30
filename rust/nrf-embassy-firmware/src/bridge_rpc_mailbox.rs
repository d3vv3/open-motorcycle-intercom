//! ARM-private live actor/SPI rendezvous; never entered from IRQ context.

use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;
use cortex_m::peripheral::scb::VectActive;
use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(not(feature = "nonblocking-bridge-delivery"))]
use nrf_embassy::bridge::Bridge;
use nrf_embassy_firmware::bridge_rpc::{Error, Request, State, Ticket};
use nrf_embassy_firmware::transport::{thread_access, thread_mode};

static SLOT: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::inactive()));
static REQUEST_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static RESULT_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static BRIDGE_INITIALIZED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "nonblocking-bridge-delivery")]
static DELIVERY: nrf_embassy_firmware::bridge_delivery::Coordinator =
    nrf_embassy_firmware::bridge_delivery::Coordinator::new();

pub(crate) fn bridge_initialized() -> bool {
    BRIDGE_INITIALIZED.load(Ordering::Acquire)
}

pub(crate) fn snapshot() -> Result<nrf_embassy_firmware::bridge_rpc::Snapshot, Error> {
    access(|state| (state.snapshot(), false), || {})
}

fn vector() -> u16 {
    match cortex_m::peripheral::SCB::vect_active() {
        VectActive::ThreadMode => 0,
        _ => 1,
    }
}

fn access<T>(op: impl FnOnce(&mut State) -> (T, bool), notify: impl FnOnce()) -> Result<T, Error> {
    thread_access(
        vector(),
        || {
            critical_section::with(|cs| {
                let mut state = SLOT.borrow(cs).borrow_mut();
                op(&mut state)
            })
        },
        notify,
    )
    .map_err(|_| Error::WrongContext)
}

async fn wait(signal: &Signal<CriticalSectionRawMutex, ()>) -> Result<(), Error> {
    thread_mode(vector()).map_err(|_| Error::WrongContext)?;
    let mut future = pin!(signal.wait());
    poll_fn(|cx| {
        if thread_mode(vector()).is_err() {
            return Poll::Ready(Err(Error::WrongContext));
        }
        future.as_mut().poll(cx).map(Ok)
    })
    .await
}

/// Unique single initialization lease. Neither endpoint is Copy or Clone.
pub(crate) struct Actor {
    _private: (),
}
pub(crate) struct Server {
    _private: (),
}

impl Actor {
    #[cfg(feature = "nonblocking-bridge-delivery")]
    pub(crate) fn apply_rx(&mut self, request: Result<Request, i32>) -> i32 {
        thread_mode(vector()).expect("RX Bridge thread context");
        DELIVERY.apply(request)
    }
    pub(crate) fn initialize() -> Result<(Self, Server), Error> {
        access(|state| (state.initialize(), false), || {})??;
        Ok((Self { _private: () }, Server { _private: () }))
    }

    /// Submission alone returns no i32. The unique mutable actor loan lasts
    /// through completion/Drop; Signals are private and have only one waiter.
    pub(crate) async fn call(&mut self, request: Request) -> Result<i32, Error> {
        let ticket = access(
            |state| {
                let result = state.reserve(request);
                let wake = result.is_ok();
                (result, wake)
            },
            || REQUEST_WAKE.signal(()),
        )??;
        let mut pending = Pending {
            ticket,
            active: true,
        };
        loop {
            if let Some(result) = access(|state| (state.take_result(ticket), false), || {})?? {
                pending.active = false;
                return Ok(result);
            }
            wait(&RESULT_WAKE).await?;
        }
    }
}

struct Pending {
    ticket: Ticket,
    active: bool,
}
impl Drop for Pending {
    fn drop(&mut self) {
        // Moving a polled call into IRQ context violates the thread-only contract.
        // Fail-stop rather than enter a CS there or silently leak a reservation.
        let canceled = access(
            |state| {
                if self.active {
                    (state.cancel(self.ticket), false)
                } else {
                    (Ok(()), false)
                }
            },
            || {},
        );
        assert!(matches!(canceled, Ok(Ok(()))));
    }
}

impl Server {
    #[cfg(feature = "nonblocking-bridge-delivery")]
    pub(crate) fn coordinator(
        &self,
    ) -> &'static nrf_embassy_firmware::bridge_delivery::Coordinator {
        &DELIVERY
    }
    pub(crate) fn mark_bridge_initialized(&mut self) {
        #[cfg(feature = "nonblocking-bridge-delivery")]
        DELIVERY.initialize();
        assert!(
            !BRIDGE_INITIALIZED.swap(true, Ordering::Release),
            "duplicate Bridge owner"
        );
    }
    /// Caller must end Prepared BEFORE entering this method/claim. Exclusive
    /// &mut Bridge proves there is no outstanding prepared borrow. No await and
    /// no Bridge reference in SLOT; apply is entirely outside the loan and CS.
    #[cfg(not(feature = "nonblocking-bridge-delivery"))]
    pub(crate) fn execute(&mut self, bridge: &mut Bridge) -> Result<bool, Error> {
        let Some((ticket, request)) = access(|state| (state.claim(), false), || {})? else {
            return Ok(false);
        };
        let result = request.apply(bridge);
        access(
            |state| {
                let completion = state.complete(ticket, result);
                let wake = matches!(completion, Ok(true));
                (completion, wake)
            },
            || RESULT_WAKE.signal(()),
        )??;
        Ok(true)
    }
    #[cfg(feature = "nonblocking-bridge-delivery")]
    pub(crate) fn execute(&mut self) -> Result<bool, Error> {
        let Some((ticket, request)) = access(|state| (state.claim(), false), || {})? else {
            return Ok(false);
        };
        // Owned request and ticket only: no SLOT loan or CS spans Bridge work.
        let result = DELIVERY.apply(Ok(request));
        access(
            |state| {
                let completion = state.complete(ticket, result);
                let wake = matches!(completion, Ok(true));
                (completion, wake)
            },
            || RESULT_WAKE.signal(()),
        )??;
        Ok(true)
    }
    /// One server waiter; a wake is only a hint, never a result or a claim.
    pub(crate) async fn wait(&mut self) -> Result<(), Error> {
        wait(&REQUEST_WAKE).await
    }
}
