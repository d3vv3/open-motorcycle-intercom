//! Shared Bridge owner. All loans end inside synchronous closures.
//! Default storage masks IRQs. The opt-in ARM owner requires single-core,
//! cooperative thread-mode execution; IRQ handlers must never enter Bridge.
//! Host tests retain safe CS storage and do not execute ARM ownership checks.
use crate::bridge_rpc::{Error, Request, State};
#[cfg(feature = "rx-blocking-trace")]
use crate::rx_blocking_trace::{self as trace, Operation};
use core::cell::RefCell;
#[cfg(not(all(target_arch = "arm", feature = "unmasked-bridge-owner")))]
use critical_section::Mutex;
#[cfg(all(target_arch = "arm", feature = "unmasked-bridge-owner"))]
use embassy_sync::blocking_mutex::ThreadModeMutex as Mutex;
use nrf_embassy::bridge::{Bridge, Consumer, Snapshot, TRANSFER_SIZE};

/// Common routing for runtime, membership, audio and adaptive-status outputs.
pub fn direct_rx(rx_outputs: bool, request: &Result<Request, i32>) -> bool {
    rx_outputs
        && matches!(
            request,
            Ok(Request::AudioBundle(_) | Request::Status(_) | Request::Event(_))
        )
}

pub struct Coordinator {
    bridge: Mutex<RefCell<Option<Bridge>>>,
}

impl Default for Coordinator {
    fn default() -> Self {
        Self::new()
    }
}

impl Coordinator {
    pub const fn new() -> Self {
        Self {
            bridge: Mutex::new(RefCell::new(None)),
        }
    }

    pub fn initialize(&self) {
        self.with_cell(|cell| {
            let mut bridge = cell.borrow_mut();
            assert!(bridge.is_none(), "duplicate Bridge owner");
            *bridge = Some(Bridge::new());
        });
    }

    /// Also check entries that can return without borrowing storage, and lease
    /// access/Drop even after finish. ThreadModeMutex checks storage and its Drop.
    fn check_context(&self) {
        #[cfg(all(target_arch = "arm", feature = "unmasked-bridge-owner"))]
        assert!(
            matches!(
                cortex_m::peripheral::SCB::vect_active(),
                cortex_m::peripheral::scb::VectActive::ThreadMode
            ),
            "Bridge owner requires thread context"
        );
    }

    fn with_cell<T>(&self, op: impl FnOnce(&RefCell<Option<Bridge>>) -> T) -> T {
        self.check_context();
        #[cfg(all(target_arch = "arm", feature = "unmasked-bridge-owner"))]
        {
            self.bridge.lock(op)
        }
        #[cfg(not(all(target_arch = "arm", feature = "unmasked-bridge-owner")))]
        critical_section::with(|cs| op(self.bridge.borrow(cs)))
    }

    fn with<T>(
        &self,
        #[cfg(feature = "rx-blocking-trace")] kind: Operation,
        op: impl FnOnce(&mut Bridge) -> T,
    ) -> T {
        self.with_cell(|cell| {
            #[cfg(feature = "rx-blocking-trace")]
            let start = trace::now();
            let result = op(cell.borrow_mut().as_mut().expect("Bridge initialized"));
            // The temporary Bridge RefMut above has ended before the clock call.
            #[cfg(feature = "rx-blocking-trace")]
            {
                let end = trace::now();
                trace::record(kind, start, end);
            }
            result
        })
    }

    /// Actual Request::apply result, including validation and delivery errno.
    pub fn apply(&self, request: Result<Request, i32>) -> i32 {
        self.check_context();
        match request {
            Ok(request) => self.with(
                #[cfg(feature = "rx-blocking-trace")]
                match request {
                    Request::AudioBundle(_) => Operation::ApplyAudio,
                    Request::Status(_) => Operation::ApplyStatus,
                    Request::Event(_) => Operation::ApplyEvent,
                    Request::Ack { .. } => Operation::ApplyAck,
                    Request::Log(_) => Operation::ApplyLog,
                    Request::DiscardAudio => Operation::Discard,
                    Request::Delivery(_) => Operation::Delivery,
                },
                |bridge| request.apply(bridge),
            ),
            Err(error) => error,
        }
    }

    /// Synchronous State service for host/actor composition. The live mailbox
    /// separately claims and completes under SLOT access, applying outside it.
    pub fn execute(&self, state: &mut State) -> Result<bool, Error> {
        self.check_context();
        let Some((ticket, request)) = state.claim() else {
            return Ok(false);
        };
        let result = self.apply(Ok(request));
        state.complete(ticket, result)
    }

    pub fn snapshot(&self) -> Snapshot {
        self.with(
            #[cfg(feature = "rx-blocking-trace")]
            Operation::Snapshot,
            |bridge| bridge.snapshot(),
        )
    }

    pub fn begin(&self) -> Option<Transfer<'_>> {
        self.with(
            #[cfg(feature = "rx-blocking-trace")]
            Operation::Begin,
            Bridge::begin_detached,
        )
        .map(|bytes| Transfer {
            owner: self,
            bytes,
            active: true,
        })
    }
}

/// Only owned DMA bytes and an owner handle cross awaits. No CS token, RefCell
/// loan or Bridge reference escapes. Drop releases the one active lease without
/// reporting completion or resurrecting overwritten entries.
pub struct Transfer<'a> {
    owner: &'a Coordinator,
    bytes: [u8; TRANSFER_SIZE],
    active: bool,
}

impl Transfer<'_> {
    pub fn bytes(&self) -> &[u8; TRANSFER_SIZE] {
        self.owner.check_context();
        &self.bytes
    }
    pub fn bytes_mut(&mut self) -> &mut [u8; TRANSFER_SIZE] {
        self.owner.check_context();
        &mut self.bytes
    }
    pub fn finish(mut self, ok: bool, rx: &[u8; TRANSFER_SIZE], consumer: &mut impl Consumer) {
        self.owner.with(
            #[cfg(feature = "rx-blocking-trace")]
            Operation::Finish,
            |bridge| bridge.finish_detached(ok, rx, consumer),
        );
        self.active = false;
    }
}

impl Drop for Transfer<'_> {
    fn drop(&mut self) {
        self.owner.check_context();
        if self.active {
            self.owner.with(
                #[cfg(feature = "rx-blocking-trace")]
                Operation::Cancel,
                Bridge::cancel_detached,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::bridge_rpc::Phase;
    use openhelmet_wire::audio_bundle::{BundleView, audio_bundle_encode};

    #[test]
    fn scoped_bridge_loan_rejects_reentry_and_releases_after_unwind() {
        let owner = coordinator();
        for initialize in [false, true] {
            let rejected = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                owner.with(
                    #[cfg(feature = "rx-blocking-trace")]
                    Operation::Snapshot,
                    |_| {
                        if initialize {
                            owner.initialize();
                        } else {
                            owner.snapshot();
                        }
                    },
                );
            }));
            let panic = rejected.expect_err("nested Bridge access must reject its RefCell loan");
            let message = panic
                .downcast_ref::<std::string::String>()
                .map(|s| s.as_str())
                .or_else(|| panic.downcast_ref::<&str>().copied())
                .unwrap();
            assert!(message.contains("already borrowed"), "{message}");
            // The original Bridge and its loan survive rejected nested access.
            assert_eq!(owner.apply(Ok(Request::Delivery(false))), 0);
            assert!(!owner.snapshot().delivery_enabled);
        }
    }

    #[test]
    fn mailbox_owned_claim_apply_complete_and_cancellation_boundaries() {
        // Exercise the real State, Coordinator and notification ordering helper.
        // Host RefCell models SLOT loans; ARM wiring is checked separately below.
        for cancel_at in [None, Some(0), Some(1), Some(2), Some(3)] {
            let owner = coordinator();
            let mut state = State::inactive();
            state.initialize().unwrap();
            let slot = RefCell::new(state);
            let ticket = slot.borrow_mut().reserve(Request::Delivery(false)).unwrap();
            if cancel_at == Some(0) {
                slot.borrow_mut().cancel(ticket).unwrap();
            }
            let claimed = crate::transport::thread_access(
                0,
                || (slot.borrow_mut().claim(), false),
                || panic!("claim must not notify"),
            )
            .unwrap();
            let Some((claimed_ticket, request)) = claimed else {
                assert_eq!(cancel_at, Some(0));
                assert!(owner.snapshot().delivery_enabled);
                assert_eq!(slot.borrow().snapshot().phase, Phase::Idle);
                continue;
            };
            assert_eq!(claimed_ticket, ticket);
            assert_eq!(slot.borrow().snapshot().phase, Phase::Claimed);
            assert_eq!(slot.borrow_mut().take_result(ticket), Ok(None));
            if cancel_at == Some(1) {
                slot.borrow_mut().cancel(ticket).unwrap();
            }
            // Claim returned owned values and released the SLOT loan. A new
            // access is possible before apply; end it before Bridge mutation.
            {
                let mut state = slot.borrow_mut();
                assert!(state.claim().is_none());
                assert_eq!(state.reserve(Request::DiscardAudio), Err(Error::Busy));
            }
            let result = owner.apply(Ok(request));
            assert_eq!(result, 0);
            assert!(!owner.snapshot().delivery_enabled);
            if cancel_at == Some(2) {
                slot.borrow_mut().cancel(ticket).unwrap();
            }
            let notified = core::cell::Cell::new(false);
            let completed = crate::transport::thread_access(
                0,
                || {
                    let mut state = slot.borrow_mut();
                    let completion = state.complete(ticket, result);
                    (completion, matches!(completion, Ok(true)))
                },
                || {
                    // Notification runs after completion and the SLOT loan ends.
                    assert_eq!(slot.borrow_mut().snapshot().phase, Phase::Completed);
                    notified.set(true);
                },
            )
            .unwrap()
            .unwrap();
            let abandoned = matches!(cancel_at, Some(1) | Some(2));
            assert_eq!(completed, !abandoned);
            assert_eq!(notified.get(), !abandoned);
            if cancel_at == Some(3) {
                slot.borrow_mut().cancel(ticket).unwrap();
            }
            assert_eq!(
                slot.borrow_mut().take_result(ticket),
                if abandoned || cancel_at == Some(3) {
                    Err(Error::Stale)
                } else {
                    Ok(Some(0))
                }
            );
            assert_eq!(slot.borrow().snapshot().phase, Phase::Idle);
            let next = slot.borrow_mut().reserve(Request::Delivery(true)).unwrap();
            assert_ne!(next, ticket);
            assert_eq!(slot.borrow_mut().complete(ticket, 0), Err(Error::Stale));
        }
    }

    #[test]
    fn live_mailbox_applies_between_separate_slot_accesses() {
        // Secondary source witness: the ARM facade is not executed by host tests.
        let source = include_str!("bridge_rpc_mailbox.rs");
        let execute = source
            .split_once("pub(crate) fn execute(&mut self) -> Result<bool, Error> {")
            .unwrap()
            .1
            .split_once("/// One server waiter")
            .unwrap()
            .0;
        let claim = execute
            .find("access(|state| (state.claim(), false), || {})?")
            .unwrap();
        let apply = execute
            .find("let result = DELIVERY.apply(Ok(request));")
            .unwrap();
        let complete_access = execute[apply..].find("access(").unwrap() + apply;
        let complete = execute.find("state.complete(ticket, result)").unwrap();
        let notify = execute.find("|| RESULT_WAKE.signal(())").unwrap();
        assert!(
            claim < apply
                && apply < complete_access
                && complete_access < complete
                && complete < notify
        );
        assert!(!execute.contains("DELIVERY.execute(state)"));
        assert_eq!(execute.matches("access(").count(), 2);
        assert!(!execute.contains(".await"));
    }

    #[cfg(feature = "rx-blocking-trace")]
    #[test]
    fn nested_callbacks_and_clock_calls_hold_no_statistics_loan() {
        std::thread_local! {
            static TICK: core::cell::Cell<i64> = const { core::cell::Cell::new(0) };
        }
        // Clock may use independent diagnostic state; it never reenters Bridge.
        fn clock() -> Option<i64> {
            trace::record(Operation::Snapshot, Some(0), Some(3));
            Some(TICK.with(|tick| {
                let now = tick.get();
                tick.set(now + 10);
                now
            }))
        }
        trace::install_clock(clock);
        let outer = coordinator();
        let inner = coordinator();
        let result = outer.with(Operation::Finish, |_| {
            // Model an existing finish callback entering a distinct owner.
            inner.apply(Ok(Request::Delivery(false)))
        });
        assert_eq!(result, 0);
        let captured = trace::capture(0);
        // Inner 10us and outer 30us overlap, and belong to distinct operations.
        // Other parallel tests may add observations, so assert lower bounds.
        assert!(captured.bridge[Operation::Delivery as usize].bins[1] >= 1);
        assert!(captured.bridge[Operation::Finish as usize].bins[2] >= 1);
        assert!(!inner.snapshot().delivery_enabled);
    }

    struct Sink;
    impl Consumer for Sink {
        fn accept_audio_v2(&mut self, _: &[u8]) -> i32 {
            0
        }
        fn request_start(&mut self, _: u8, _: u8) {}
        fn request_stop(&mut self, _: u8) {}
        fn request_pause(&mut self, _: u8) {}
        fn request_resume(&mut self, _: u8) {}
        fn request_status(&mut self) {}
        fn pulse_ack(&mut self) {}
    }
    fn audio(source: u8) -> Request {
        let mut bytes = [0; 56];
        assert_eq!(
            audio_bundle_encode(
                &BundleView {
                    current_data: &[42; 48],
                    previous1_data: &[],
                    previous2_data: &[],
                    current_seq: 7,
                    stream_id: 1,
                    flags: 1,
                    codec: 2,
                },
                &mut bytes
            ),
            56
        );
        Request::audio_bundle(source, &bytes).unwrap()
    }
    fn coordinator() -> Coordinator {
        let owner = Coordinator::new();
        owner.initialize();
        owner
    }
    fn service(owner: &Coordinator, rpc: &mut State, request: Request) -> i32 {
        let ticket = rpc.reserve(request).unwrap();
        assert!(owner.execute(rpc).unwrap());
        rpc.take_result(ticket).unwrap().unwrap()
    }

    #[test]
    fn overflowing_coordinator_bytes_equal_existing_borrowed_bridge_oracle() {
        let owner = coordinator();
        let mut direct = Bridge::new();
        // Overflow both real rings, then compare the complete encoded transfers
        // against the original borrowed API, including failed-transfer retries.
        for source in 1..=32 {
            for request in [audio(source), Request::event(2, &[source]).unwrap()] {
                assert_eq!(owner.apply(Ok(request)), request.apply(&mut direct));
                assert_eq!(owner.snapshot(), direct.snapshot());
            }
        }
        assert!(owner.snapshot().audio_overwrite > 0);
        assert!(owner.snapshot().control_overwrite > 0);
        for attempt in 0..28 {
            let actual = owner.begin().unwrap();
            let expected = direct.prepare();
            assert_eq!(actual.bytes(), expected.bytes());
            let ok = attempt % 3 != 0;
            actual.finish(ok, &[0; TRANSFER_SIZE], &mut Sink);
            expected.finish(ok, &[0; TRANSFER_SIZE], &mut Sink);
            assert_eq!(owner.snapshot(), direct.snapshot());
        }
    }

    #[test]
    fn peer_bursts_equal_direct_ring_capacity_overflow_and_bytes() {
        let owner = coordinator();
        let mut direct = Bridge::new();
        for peer in 1..=7 {
            for request in [
                Request::Status(Default::default()),
                Request::event(2, &[peer]).unwrap(),
                audio(peer),
            ] {
                assert_eq!(owner.apply(Ok(request)), request.apply(&mut direct));
                assert_eq!(owner.snapshot(), direct.snapshot());
            }
        }
        assert_eq!(owner.snapshot().audio_overwrite, 0);
        assert_eq!(owner.snapshot().control_overwrite, 12);
        for _ in 0..11 {
            let transfer = owner.begin().unwrap();
            let prepared = direct.prepare();
            assert_eq!(transfer.bytes(), prepared.bytes());
            transfer.finish(true, &[0; TRANSFER_SIZE], &mut Sink);
            prepared.finish(true, &[0; TRANSFER_SIZE], &mut Sink);
            assert_eq!(owner.snapshot(), direct.snapshot());
        }
    }

    #[test]
    fn actual_server_orders_delivery_and_discard_with_direct_admission() {
        let owner = coordinator();
        let mut rpc = State::inactive();
        rpc.initialize().unwrap();
        let selected = owner.begin().unwrap();
        let ticket = rpc.reserve(Request::Delivery(false)).unwrap();
        for source in 1..=7 {
            assert_eq!(owner.apply(Ok(audio(source))), 0);
        }
        assert_eq!(owner.snapshot().audio_head, 7);
        assert_eq!(rpc.take_result(ticket), Ok(None));
        assert!(owner.execute(&mut rpc).unwrap());
        assert_eq!(rpc.take_result(ticket), Ok(Some(0)));
        assert_eq!(owner.apply(Ok(audio(8))), nrf_embassy::bridge::EAGAIN);
        assert_eq!(
            owner.apply(Request::audio_bundle(8, &[0])),
            nrf_embassy::bridge::EINVAL
        );
        assert_eq!(service(&owner, &mut rpc, Request::Delivery(true)), 0);
        assert_eq!(owner.apply(Ok(audio(9))), 0);
        selected.finish(true, &[0; TRANSFER_SIZE], &mut Sink);
        assert_eq!(service(&owner, &mut rpc, Request::DiscardAudio), 0);
        assert_eq!(owner.snapshot().audio_head, owner.snapshot().audio_tail);
        assert_eq!(owner.apply(Ok(audio(10))), 0);
        let transfer = owner.begin().unwrap();
        assert_eq!(
            openhelmet_wire::bridge::decode(transfer.bytes(), 208)
                .unwrap()
                .payload[0],
            10
        );
        drop(transfer);
    }

    #[test]
    fn drop_and_failure_after_overflow_do_not_resurrect_or_count_success() {
        for fail in [false, true] {
            let owner = coordinator();
            let selected = owner.begin().unwrap();
            assert!(owner.begin().is_none());
            for value in 0..16 {
                owner.apply(Request::event(2, &[value]));
            }
            let before = owner.snapshot();
            if fail {
                selected.finish(false, &[0; TRANSFER_SIZE], &mut Sink);
            } else {
                drop(selected);
            }
            let after = owner.snapshot();
            assert_eq!(after.control_tail, before.control_tail);
            assert_eq!(after.tx_control, 0);
            assert_eq!(after.tx_fail, u32::from(fail));
            assert_eq!(after.attempts, u32::from(fail));
            let next = owner.begin().unwrap();
            assert_eq!(
                openhelmet_wire::bridge::decode(next.bytes(), 208)
                    .unwrap()
                    .payload,
                &[2, 13]
            );
        }
    }

    #[test]
    fn rx_routing_includes_adaptive_status_and_excludes_server_commands_logs() {
        for request in [
            Request::Status(Default::default()),
            Request::event(1, &[]).unwrap(),
            audio(1),
        ] {
            assert!(direct_rx(true, &Ok(request)));
            assert!(!direct_rx(false, &Ok(request)));
        }
        for request in [
            Request::DiscardAudio,
            Request::Delivery(false),
            Request::log(b"log").unwrap(),
            Request::Ack {
                command: 1,
                generation: 1,
                result: 0,
            },
        ] {
            assert!(!direct_rx(true, &Ok(request)));
        }
    }
}
