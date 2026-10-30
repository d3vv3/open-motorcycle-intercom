//! Sole protocol owner. Handler futures run to completion, never raced with
//! commands or shutdown: cancellation would expose partially committed state.

use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;
use critical_section::Mutex;
use embassy_time::{Duration, with_timeout};
use nrf_embassy::platform::Platform;
use nrf_embassy::protocol_runtime::FullProtocolPlatform;
use nrf_embassy::runtime::{Event, Runtime, Timer};
use nrf_embassy_firmware::actor_policy::{
    Capability, TxWindow, due_timing_allowed, early_timing_allowed,
};
use nrf_embassy_firmware::diagnostics::{ACTOR_PERIOD_US, ActorSnapshot};
#[cfg(feature = "slot-entry-trace")]
use nrf_embassy_firmware::slot_entry_trace::{Phase, phase};

use crate::{
    protocol_backend::Backend,
    timing_backend::{self, ActorCallback},
    transport_mailbox,
};

static CAPABILITY: AtomicBool = AtomicBool::new(false);
static LATEST: Mutex<RefCell<Option<ActorSnapshot>>> = Mutex::new(RefCell::new(None));

pub(crate) fn latest() -> Option<ActorSnapshot> {
    critical_section::with(|cs| *LATEST.borrow(cs).borrow())
}

fn publish(
    runtime: &Runtime,
    backend: &Backend,
    capability: &Capability,
    sequence: u32,
    clock_us: u64,
) {
    let snapshot = ActorSnapshot {
        sequence,
        clock_us,
        runtime: runtime.snapshot(),
        reported_peers: runtime.membership_bridge_peer_count(),
        capability: capability.snapshot(),
        admission: runtime.ingress_admission_gate(),
        radio: backend.radio.snapshot(),
        #[cfg(feature = "slot-entry-trace")]
        slot_entry: timing_backend::slot_entry_snapshot(),
        #[cfg(feature = "pipeline-trace")]
        audio_diagnostic: nrf_embassy_firmware::diagnostics::AudioDiagnostic::capture(runtime),
        #[cfg(feature = "sync-trace")]
        tx_diagnostics: backend.tx_diagnostics,
    };
    critical_section::with(|cs| *LATEST.borrow(cs).borrow_mut() = Some(snapshot));
    // No notification is needed: the independent SPI publisher samples at 1 Hz.
}

pub(crate) fn capable() -> bool {
    CAPABILITY.load(Ordering::Acquire)
}

fn drain_ingress(
    runtime: &mut Runtime,
    backend: &mut Backend,
    inbox: &mut transport_mailbox::Actor,
) {
    // Exact one sixteen-entry inbox, directly consumed, never re-enqueued.
    for _ in 0..16 {
        let Some(prepared) = inbox.pop_owned().expect("actor inbox pop") else {
            break;
        };
        let _ = runtime.consume_prepared_ingress(&prepared, &mut FullProtocolPlatform(backend));
    }
    let accounting = inbox.take_accounting().expect("actor take accounting");
    runtime
        .account_external_ingress(accounting.ingress)
        .expect("external ingress accounting");
    runtime
        .account_external_ingress_validation(accounting.bundle_bad)
        .expect("external validation accounting");
}

/// One existing bounded timing batch, with all owner borrows retained through
/// each handler's completion. Slot ingress is refreshed immediately before TX.
async fn service_timing(
    runtime: &mut Runtime,
    backend: &mut Backend,
    inbox: &mut transport_mailbox::Actor,
) {
    #[cfg(feature = "slot-entry-trace")]
    phase(Phase::ServiceTiming);
    let callbacks = backend.timing.drain(&mut backend.tdma);
    let generation = backend.tdma.snapshot().generation;
    for callback in callbacks.into_iter().flatten() {
        let current = backend.tdma.snapshot();
        if !current.running || current.generation != generation {
            continue; // A prior awaited handler may have stopped/restarted TDMA.
        }
        let physical_now = backend.now_us();
        let Some(timing_now) = backend.timing.timing_at_physical(physical_now) else {
            continue;
        };
        let Some(window) = TxWindow::capture(current, callback, timing_now) else {
            continue; // Reject expired callbacks before any core work/output.
        };
        #[cfg(feature = "slot-entry-trace")]
        let mapping = backend.timing.clock_mapping();
        backend.tx_window = Some(window);
        match callback {
            ActorCallback::Slot { slot, frame } => {
                drain_ingress(runtime, backend, inbox);
                runtime
                    .audio_slot_tx_handler_async(slot, frame, &mut FullProtocolPlatform(backend))
                    .await;
            }
            ActorCallback::Control { frame } => {
                runtime
                    .tx_control_handler_async(frame, &mut FullProtocolPlatform(backend))
                    .await;
            }
        }
        backend.tx_window = None;
        #[cfg(feature = "slot-entry-trace")]
        if let (ActorCallback::Slot { slot, .. }, TxWindow::Slot(metadata)) = (callback, window) {
            use nrf_embassy_firmware::timing_clock::TimingTime;
            // Preserve the admitted entry sample and mapping across the handler;
            // conversion and histogram work must not delay its TX launch.
            if let Ok(physical) = mapping.convert(
                TimingTime::new(metadata.start_us),
                TimingTime::new(metadata.deadline_us),
            ) {
                timing_backend::record_slot_entry(
                    slot,
                    physical_now as u32,
                    physical.start().value() as u32,
                );
            }
        }
    }
}

/// Bounded-latency slot entry: a due alarm is serviced at the next phase
/// boundary instead of after the whole pass. Only called after command work
/// has executed, so a queued STOP has already invalidated the TDMA generation.
async fn service_due_timing(
    runtime: &mut Runtime,
    backend: &mut Backend,
    inbox: &mut transport_mailbox::Actor,
) {
    if due_timing_allowed(false, timing_backend::alarm_pending()) {
        service_timing(runtime, backend, inbox).await;
    }
}

async fn wait(inbox: &mut transport_mailbox::Actor) {
    let mut transport = pin!(inbox.wait());
    let mut timing = pin!(timing_backend::wait());
    let _ = with_timeout(
        Duration::from_micros(200),
        poll_fn(|cx| {
            if let Poll::Ready(result) = transport.as_mut().poll(cx) {
                result.expect("actor wait context");
                return Poll::Ready(());
            }
            timing.as_mut().poll(cx)
        }),
    )
    .await;
}

#[embassy_executor::task]
pub(crate) async fn task(mut backend: Backend, mut inbox: transport_mailbox::Actor) {
    let mut runtime = Runtime::default();
    let mut capability = Capability::default();
    // Spawn ordering alone is not a first-poll guarantee. SPI must have its real
    // Bridge before init can render/query software logging or issue any output.
    while !crate::bridge_rpc_mailbox::bridge_initialized() {
        backend.healthy();
        embassy_time::Timer::after_micros(200).await;
    }
    backend.tdma.init(&mut backend.timing);
    assert_eq!(
        runtime.init(&mut FullProtocolPlatform(&mut backend)),
        0,
        "strict hardware init"
    );
    capability.strict_initialized();
    runtime
        .use_external_ingress()
        .expect("single external ingress mode");
    capability.external_ingress();
    inbox
        .publish_gate(runtime.ingress_admission_gate(), true)
        .expect("initial published gate");
    capability.gate_published();
    assert!(capability.ready());
    CAPABILITY.store(true, Ordering::Release);
    let mut sequence = 1u32;
    let mut last_publish = backend.now_us() as u64;
    publish(&runtime, &backend, &capability, sequence, last_publish);
    // S31 must see software capability before it can send START. This is not an
    // RF qualification claim, and initial pre-init STATUS is masked by SPI.
    runtime.request_status(&mut FullProtocolPlatform(&mut backend));

    loop {
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::Commands);
        backend.healthy(); // Also runs stopped, far more often than once/second.
        let requests = inbox.take_requests().expect("actor requests");
        let command_present = requests.command.is_some();
        if let Some(command) = requests.command {
            runtime.handle(
                Event::Command(command),
                &mut FullProtocolPlatform(&mut backend),
            );
        }
        if requests.status {
            runtime.request_status(&mut FullProtocolPlatform(&mut backend));
        }
        // Command events only queue work (including STOP). Skip early TX for
        // every consumed command so its work executes first. With experimental
        // opt-in, due TX precedes background awaits; late service always runs.
        if early_timing_allowed(cfg!(feature = "early-timing-service"), command_present) {
            service_timing(&mut runtime, &mut backend, &mut inbox).await;
        }
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::CommandWork);
        runtime
            .command_work_handler_async(&mut FullProtocolPlatform(&mut backend))
            .await;
        service_due_timing(&mut runtime, &mut backend, &mut inbox).await;

        // One deadline per timer per pass; Runtime validates fresh tokens again.
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::Timers);
        for timer in [Timer::Scan, Timer::Join, Timer::Status] {
            if let Some(token) = runtime.timer(timer)
                && token.deadline_us <= backend.now_us()
            {
                runtime
                    .handle_async(Event::Timer(token), &mut FullProtocolPlatform(&mut backend))
                    .await;
            }
        }

        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::RadioService);
        backend.radio.service();
        // Pool bound eight; no unbounded RX starvation of TDMA/commands.
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::RxDequeue);
        for _ in 0..8 {
            if let Some(packet) = backend.radio.dequeue().expect("RF dequeue") {
                runtime
                    .rx_publish(
                        &packet.bytes[..packet.len],
                        packet.rssi,
                        packet.stamp,
                        &mut FullProtocolPlatform(&mut backend),
                    )
                    .expect("owned RX publication");
            }
        }
        service_due_timing(&mut runtime, &mut backend, &mut inbox).await;
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::RxWork);
        if backend.rf_work {
            backend.rf_work = false;
            #[cfg(feature = "nonblocking-bridge-delivery")]
            {
                backend.rx_outputs = true;
            }
            runtime
                .rx_work_handler_async(&mut FullProtocolPlatform(&mut backend))
                .await
                .expect("RF protocol arithmetic");
            #[cfg(feature = "nonblocking-bridge-delivery")]
            {
                backend.rx_outputs = false;
            }
        }

        service_due_timing(&mut runtime, &mut backend, &mut inbox).await;
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::Ingress);
        drain_ingress(&mut runtime, &mut backend, &mut inbox);
        if backend.ingress_work {
            backend.ingress_work = false;
            runtime.audio_ingress_work_handler(&mut FullProtocolPlatform(&mut backend));
        }

        service_timing(&mut runtime, &mut backend, &mut inbox).await;
        let now = backend.now_us() as u64;
        if now - last_publish >= ACTOR_PERIOD_US {
            sequence = sequence.wrapping_add(1);
            last_publish = now;
            publish(&runtime, &backend, &capability, sequence, now);
        }
        #[cfg(feature = "slot-entry-trace")]
        phase(Phase::Wait);
        wait(&mut inbox).await;
    }
}
