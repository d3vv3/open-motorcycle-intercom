use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_nrf::usb::{Driver, vbus_detect::HardwareVbusDetect};
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::{Builder, Config, UsbDevice};
use nrf_embassy_firmware::actor_policy::diagnostic_format_allowed;
use nrf_embassy_firmware::diagnostics::{
    DiagnosticLine, DiagnosticSnapshot, IDENTITY, packets, serial_number,
};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::sync_trace::{
    TraceLine, TransportCounters, WriterPreference, WriterSource,
};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::tx_failure_trace::{
    Line as FailureLine, Loss as FailureLoss, TransportCounters as FailureTransportCounters,
};
use static_cell::StaticCell;

use crate::Irqs;
#[cfg(feature = "sync-trace")]
use crate::sync_trace_runtime::Publication;
use crate::timing_backend;

type UsbDriver = Driver<'static, HardwareVbusDetect>;
static SNAPSHOTS: Channel<CriticalSectionRawMutex, DiagnosticSnapshot, 8> = Channel::new();
static DROPS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sync-trace")]
static TRACES: Channel<CriticalSectionRawMutex, Publication, 2> = Channel::new();
#[cfg(feature = "sync-trace")]
static TRACE_TRANSPORT: critical_section::Mutex<core::cell::RefCell<TransportCounters>> =
    critical_section::Mutex::new(core::cell::RefCell::new(TransportCounters {
        queue_rejected: 0,
        coalesced: 0,
        format_failed: 0,
        usb_interrupted: 0,
        counter_saturated: false,
    }));
#[cfg(feature = "sync-trace")]
static FAILURE_TRANSPORT: critical_section::Mutex<core::cell::RefCell<FailureTransportCounters>> =
    critical_section::Mutex::new(core::cell::RefCell::new(FailureTransportCounters::new()));
static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State<'static>> = StaticCell::new();
static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();

pub fn start(spawner: Spawner, usbd: Peri<'static, peripherals::USBD>) {
    let ficr = embassy_nrf::pac::FICR;
    let serial = SERIAL.init(serial_number(
        ficr.deviceid(0).read(),
        ficr.deviceid(1).read(),
    ));
    // The helper produces exactly sixteen ASCII hex bytes, with no terminator.
    let serial = core::str::from_utf8(serial).expect("hex serial");
    let driver = Driver::new(usbd, Irqs, HardwareVbusDetect::new(Irqs));
    // Reuse the legacy development ID, not a new production USB allocation.
    let mut config = Config::new(0x2FE3, 0x0001);
    config.manufacturer = Some("OpenHelmet");
    config.product = Some("OpenHelmet bridge integration");
    config.serial_number = Some(serial);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUFFER.init([0; 64]),
    );
    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
    let device = builder.build();
    let (sender, receiver) = cdc.split();
    spawner.spawn(device_task(device).expect("USB device task"));
    spawner.spawn(writer_task(sender).expect("USB writer task"));
    spawner.spawn(reader_task(receiver).expect("USB reader task"));
}

/// The SPI owner never awaits USB, connection state or channel capacity.
pub fn publish(mut snapshot: DiagnosticSnapshot) {
    #[cfg(feature = "rx-blocking-trace")]
    {
        use nrf_embassy_firmware::rx_blocking_trace;
        static SEQUENCE: AtomicU32 = AtomicU32::new(0);
        let mut trace = rx_blocking_trace::capture(SEQUENCE.fetch_add(1, Ordering::Relaxed));
        trace.radio = crate::radio_receiver::rx_diagnostics();
        trace.radio_clock_us = rx_blocking_trace::now();
        snapshot.rx_blocking = trace;
    }
    snapshot.actor = crate::protocol_actor::latest();
    snapshot.software_capability = crate::protocol_actor::capable();
    snapshot.rpc = crate::bridge_rpc_mailbox::snapshot().ok();
    snapshot.drops = DROPS.load(Ordering::Relaxed);
    if SNAPSHOTS.try_send(snapshot).is_err() {
        DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(feature = "sync-trace")]
fn trace_loss(update: impl FnOnce(&mut TransportCounters)) {
    critical_section::with(|cs| update(&mut TRACE_TRANSPORT.borrow(cs).borrow_mut()));
}

#[cfg(feature = "sync-trace")]
pub(crate) fn publish_trace(publication: Publication) {
    let failures = publication.failures();
    if TRACES.try_send(publication).is_err() {
        trace_loss(TransportCounters::reject_queue);
        failure_loss(
            FailureLoss::Queue,
            failures.batch.map_or(0, |batch| batch.len()),
            true,
        );
    }
}

#[cfg(feature = "sync-trace")]
fn failure_loss(cause: FailureLoss, remaining: usize, health_pending: bool) {
    critical_section::with(|cs| {
        FAILURE_TRANSPORT
            .borrow(cs)
            .borrow_mut()
            .lose(cause, remaining, health_pending);
    });
}

#[cfg(feature = "sync-trace")]
// Owned bounded values keep selection allocation-free on the firmware target.
#[allow(clippy::large_enum_variant)]
enum WriterInput {
    Diagnostic(DiagnosticSnapshot),
    Trace(Publication),
}

#[cfg(feature = "sync-trace")]
async fn writer_input(
    latest: &mut Option<DiagnosticSnapshot>,
    preference: &mut WriterPreference,
) -> WriterInput {
    use core::future::{Future, poll_fn};
    use core::pin::pin;
    use core::task::Poll;

    let mut diagnostic = pin!(SNAPSHOTS.receive());
    let mut trace = pin!(TRACES.receive());
    poll_fn(|cx| {
        preference
            .choose(|source| match source {
                WriterSource::Diagnostic => {
                    if let Some(snapshot) = latest.take() {
                        return Some(WriterInput::Diagnostic(snapshot));
                    }
                    match diagnostic.as_mut().poll(cx) {
                        Poll::Ready(snapshot) => Some(WriterInput::Diagnostic(snapshot)),
                        Poll::Pending => None,
                    }
                }
                WriterSource::Trace => match trace.as_mut().poll(cx) {
                    Poll::Ready(publication) => Some(WriterInput::Trace(publication)),
                    Poll::Pending => None,
                },
            })
            .map_or(Poll::Pending, Poll::Ready)
    })
    .await
}

#[cfg(feature = "sync-trace")]
async fn send_trace(
    sender: &mut Sender<'static, UsbDriver>,
    publication: Publication,
) -> Result<(), ()> {
    let failures = publication.failures();
    let mut remaining = failures.batch.map_or(0, |batch| batch.len());
    // SYNC counters retain their original meaning. If its prefix fails, the
    // failure suffix is also discarded, explicitly accounted, and never retried.
    if let Err(cause) = send_sync_trace(sender, publication).await {
        failure_loss(cause, remaining, true);
        return Err(());
    }
    let transport = critical_section::with(|cs| *FAILURE_TRANSPORT.borrow(cs).borrow());
    let health = FailureLine::health(failures, transport).map_err(|_| {
        failure_loss(FailureLoss::Format, remaining, true);
    })?;
    if send(sender, health.as_bytes()).await.is_err() {
        failure_loss(FailureLoss::Usb, remaining, true);
        return Err(());
    }
    if let Some(batch) = failures.batch {
        for record in batch.records() {
            let line = FailureLine::record(batch.id, *record).map_err(|_| {
                failure_loss(FailureLoss::Format, remaining, false);
            })?;
            if send(sender, line.as_bytes()).await.is_err() {
                // Includes the possibly partially submitted current row.
                failure_loss(FailureLoss::Usb, remaining, false);
                return Err(());
            }
            remaining -= 1;
        }
    }
    Ok(())
}

#[cfg(feature = "sync-trace")]
async fn send_sync_trace(
    sender: &mut Sender<'static, UsbDriver>,
    publication: Publication,
) -> Result<(), FailureLoss> {
    let transport = critical_section::with(|cs| *TRACE_TRANSPORT.borrow(cs).borrow());
    let header = match &publication {
        Publication::Batch(captured) => {
            TraceLine::header(&captured.batch, captured.tracker, transport)
        }
        Publication::Health { trace, tracker, .. } => {
            TraceLine::health(*trace, *tracker, transport)
        }
    };
    // Formatting is outside CS. Abort the whole batch on any failure, including
    // a row failure after its header or earlier rows were already submitted.
    let header = header.map_err(|_| {
        trace_loss(TransportCounters::fail_format);
        FailureLoss::Format
    })?;
    if send(sender, header.as_bytes()).await.is_err() {
        trace_loss(TransportCounters::interrupt_usb);
        return Err(FailureLoss::Usb);
    }
    if let Publication::Batch(captured) = publication {
        for record in captured.batch.records() {
            let line = TraceLine::record(captured.batch.id, *record).map_err(|_| {
                trace_loss(TransportCounters::fail_format);
                FailureLoss::Format
            })?;
            if send(sender, line.as_bytes()).await.is_err() {
                trace_loss(TransportCounters::interrupt_usb);
                return Err(FailureLoss::Usb);
            }
        }
    }
    // Successful endpoint writes do not establish host receipt/acknowledgement.
    Ok(())
}

#[embassy_executor::task]
async fn device_task(mut device: UsbDevice<'static, UsbDriver>) {
    // Keep the device/bus/control future alive across all endpoint errors.
    device.run().await;
}

async fn send(sender: &mut Sender<'static, UsbDriver>, bytes: &[u8]) -> Result<(), ()> {
    for packet in packets(bytes) {
        if !matches!(
            with_timeout(Duration::from_secs(1), sender.write_packet(packet)).await,
            Ok(Ok(()))
        ) {
            return Err(());
        }
    }
    // Every formatted line follows a send; hold the next formatting unit until
    // the timing actor has a quiet window. Bounded so a stuck alarm cannot
    // silence diagnostics (worst case one frame).
    for _ in 0..80 {
        if diagnostic_format_allowed(
            timing_backend::alarm_pending(),
            timing_backend::next_alarm_in_us(),
        ) {
            break;
        }
        Timer::after_micros(250).await;
    }
    Ok(())
}

fn drain_latest(mut latest: Option<DiagnosticSnapshot>) -> Option<DiagnosticSnapshot> {
    while let Ok(snapshot) = SNAPSHOTS.try_receive() {
        latest = Some(snapshot);
    }
    latest
}

#[cfg(feature = "sync-trace")]
async fn send_tx_classes(
    sender: &mut Sender<'static, UsbDriver>,
    actor: nrf_embassy_firmware::diagnostics::ActorSnapshot,
) -> Result<(), ()> {
    // One actor copy supplies every row across USB awaits. The extra bounded
    // 1 Hz traffic and snapshot copies are not a CPU/RF timing qualification.
    for class in nrf_embassy_firmware::tx_diagnostics::WireClass::ALL {
        let line = DiagnosticLine::tx_class_status(actor, class).map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
        send(sender, line.as_bytes()).await?;
        let line = DiagnosticLine::tx_admission_status(actor, class).map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
        send(sender, line.as_bytes()).await?;
        if let Some(line) = DiagnosticLine::tx_window_status(actor, class).map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })? {
            send(sender, line.as_bytes()).await?;
        }
    }
    Ok(())
}

#[cfg(feature = "pipeline-trace")]
async fn send_pipeline(
    sender: &mut Sender<'static, UsbDriver>,
    snapshot: DiagnosticSnapshot,
) -> Result<(), ()> {
    for row in nrf_embassy_firmware::diagnostics::PipelineRow::ALL {
        // Includes PIPE_SLOT_ENTRY from the same owned actor publication; no live loans.
        let line = DiagnosticLine::pipeline(snapshot, row).map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
        send(sender, line.as_bytes()).await.map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
    }
    Ok(())
}

#[cfg(feature = "rx-blocking-trace")]
async fn send_rx_blocking(
    sender: &mut Sender<'static, UsbDriver>,
    snapshot: DiagnosticSnapshot,
) -> Result<(), ()> {
    use nrf_embassy_firmware::rx_blocking_trace::Operation;
    for op in Operation::ALL {
        let line = DiagnosticLine::bridge_cs(snapshot.rx_blocking, op).map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
        send(sender, line.as_bytes()).await.map_err(|_| {
            DROPS.fetch_add(1, Ordering::Relaxed);
        })?;
    }
    let line = DiagnosticLine::radio_rx(snapshot.rx_blocking).map_err(|_| {
        DROPS.fetch_add(1, Ordering::Relaxed);
    })?;
    send(sender, line.as_bytes()).await.map_err(|_| {
        DROPS.fetch_add(1, Ordering::Relaxed);
    })
}

#[embassy_executor::task]
async fn writer_task(mut sender: Sender<'static, UsbDriver>) {
    #[cfg(feature = "sync-trace")]
    let mut preference = WriterPreference::new();
    loop {
        // POWER removal does not necessarily wake the endpoint's waker.
        if with_timeout(Duration::from_secs(1), sender.wait_connection())
            .await
            .is_err()
        {
            continue;
        }
        let mut latest = drain_latest(None);
        if send(&mut sender, IDENTITY).await.is_ok() {
            loop {
                #[cfg(feature = "sync-trace")]
                match writer_input(&mut latest, &mut preference).await {
                    WriterInput::Diagnostic(snapshot) => latest = Some(snapshot),
                    WriterInput::Trace(batch) => {
                        if send_trace(&mut sender, batch).await.is_err() {
                            break;
                        }
                        continue;
                    }
                }
                #[cfg(not(feature = "sync-trace"))]
                if latest.is_none() {
                    latest = Some(SNAPSHOTS.receive().await);
                }
                // Discard historical backlog on reconnect and after slow writes.
                latest = drain_latest(latest);
                let mut snapshot = latest.take().expect("snapshot available");
                snapshot.drops = DROPS.load(Ordering::Relaxed);
                // Repeat executing-image metadata so missing enumeration's first
                // write does not lose identity. SPI publishes at bounded 1 Hz.
                if send(&mut sender, IDENTITY).await.is_err() {
                    break;
                }
                let Ok(line) = DiagnosticLine::from_snapshot(snapshot) else {
                    DROPS.fetch_add(1, Ordering::Relaxed);
                    continue;
                };
                if send(&mut sender, line.as_bytes()).await.is_err() {
                    break;
                }
                let Ok(line) = DiagnosticLine::actor_status(snapshot) else {
                    DROPS.fetch_add(1, Ordering::Relaxed);
                    continue;
                };
                if send(&mut sender, line.as_bytes()).await.is_err() {
                    break;
                }
                let Ok(line) = DiagnosticLine::radio_status(snapshot) else {
                    DROPS.fetch_add(1, Ordering::Relaxed);
                    continue;
                };
                if send(&mut sender, line.as_bytes()).await.is_err() {
                    break;
                }
                #[cfg(feature = "sync-trace")]
                if let Some(actor) = snapshot.actor
                    && send_tx_classes(&mut sender, actor).await.is_err()
                {
                    break;
                }
                #[cfg(feature = "pipeline-trace")]
                if send_pipeline(&mut sender, snapshot).await.is_err() {
                    break;
                }
                #[cfg(feature = "reset-diagnostics")]
                {
                    let Ok(line) =
                        DiagnosticLine::reset(crate::reset_diagnostics_runtime::snapshot())
                    else {
                        DROPS.fetch_add(1, Ordering::Relaxed);
                        continue;
                    };
                    if send(&mut sender, line.as_bytes()).await.is_err() {
                        DROPS.fetch_add(1, Ordering::Relaxed);
                        break;
                    }
                }
                #[cfg(feature = "rx-blocking-trace")]
                if send_rx_blocking(&mut sender, snapshot).await.is_err() {
                    break;
                }
            }
        }
        // Also yield if the disabled endpoint produces an immediate error.
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn reader_task(mut receiver: Receiver<'static, UsbDriver>) {
    let mut buffer = [0; 64];
    loop {
        if with_timeout(Duration::from_secs(1), receiver.wait_connection())
            .await
            .is_err()
        {
            continue;
        }
        match with_timeout(Duration::from_secs(1), receiver.read_packet(&mut buffer)).await {
            Ok(Ok(_)) | Err(_) => {}
            Ok(Err(_)) => Timer::after_millis(100).await,
        }
        // All OUT bytes are discarded: no RF commands or runtime-state injection.
    }
}
