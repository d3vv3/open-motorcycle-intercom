//! Private static facade; the sole protocol actor owns its lifecycle and TX calls.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use critical_section::Mutex;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::interrupt::typelevel::{Binding, Handler, Interrupt, RADIO, TIMER2, TIMER4};
use embassy_nrf::{Peri, peripherals};
use embassy_nrf_esb::addresses::EsbAddresses;
use embassy_nrf_esb::config::{Bitrate, Channel, CrcConfig, EsbConfig, TxPower};
use embassy_nrf_esb::error::Error;
use embassy_nrf_esb::payload::PacketPool;
use embassy_nrf_esb::{Esb, TxFault, TxOutcome};
use nrf_embassy::protocol_tx::{RadioSend, RadioSendResult, SubmissionDisposition};
use nrf_embassy::runtime::{EBUSY, EINVAL, ENOTSUP};
use nrf_embassy_firmware::actor_policy::{RfLifecycle, TxCounters, TxTerminal};
use nrf_embassy_firmware::diagnostics::RadioSnapshot;
use nrf_embassy_firmware::receiver::{
    ReceiverMetadata, ReceiverRecord, ReceiverSnapshot, device_identity,
};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::sync_trace::{SendRecord, SendResult};
#[cfg(feature = "sync-trace")]
use nrf_embassy_firmware::tx_diagnostics::{Stage, Stats, Terminal, WireClass};
use static_cell::StaticCell;

use crate::shared_clock::SharedClock;

type Facade = Esb<peripherals::TIMER2, 8, 256>;
static POOL: PacketPool<8, 256> = PacketPool::new();
static FACADE: StaticCell<Facade> = StaticCell::new();
static DRIVER: Mutex<RefCell<Option<&'static Facade>>> = Mutex::new(RefCell::new(None));
static LATEST: Mutex<RefCell<ReceiverSnapshot>> = Mutex::new(RefCell::new(ReceiverSnapshot {
    irq_count: 0,
    received_packets: 0,
    bytes: 0,
    missing_metadata: 0,
    missing_clock: 0,
    clock_errors: 0,
    last: None,
}));
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
static CLOCK_ERRORS: AtomicU32 = AtomicU32::new(0);

fn driver() -> Option<&'static Facade> {
    // End both the RefCell loan and CS before calling the facade/clock hook.
    critical_section::with(|cs| *DRIVER.borrow(cs).borrow())
}

#[cfg(feature = "rx-blocking-trace")]
pub(crate) fn rx_diagnostics() -> Option<[u32; 5]> {
    let c = driver()?.rx_diagnostics().ok()?;
    Some([
        c.bad_crc,
        c.duplicate,
        c.start_alloc_failed,
        c.post_delivery_starved,
        c.queue_rejected,
    ])
}

pub struct RadioHandler;
impl Handler<RADIO> for RadioHandler {
    unsafe fn on_interrupt() {
        IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
        if let Some(driver) = driver() {
            // SAFETY: installed static facade; equal P2 RADIO/TIMER2 never nest.
            unsafe { driver.on_radio_interrupt() };
        }
    }
}

pub struct TimerHandler;
impl Handler<TIMER2> for TimerHandler {
    unsafe fn on_interrupt() {
        if let Some(driver) = driver() {
            // SAFETY: only the bound TIMER2 vector, same activation contract.
            unsafe { driver.on_timer_interrupt() };
        }
    }
}

pub struct DeadlineHandler;
impl Handler<TIMER4> for DeadlineHandler {
    unsafe fn on_interrupt() {
        if let Some(driver) = driver() {
            // SAFETY: exact static facade, equal P2; registry loan already ended.
            unsafe { driver.on_deadline_interrupt() };
        }
    }
}

fn rx_clock() -> Option<i64> {
    let result = SharedClock::read_static();
    if result.is_err() {
        CLOCK_ERRORS.fetch_add(1, Ordering::Relaxed);
    }
    result.ok()
}

/// Root retains only immutable static driver/pool references and wire identity.
pub(crate) struct RadioOwner {
    driver: &'static Facade,
    _pool: &'static PacketPool<8, 256>,
    identity: [u8; 5],
    addresses: EsbAddresses,
    channel: u8,
    last_epoch: u64,
    lifecycle: RfLifecycle,
    tx: TxCounters,
}

pub(crate) fn initialize(
    radio: Peri<'static, peripherals::RADIO>,
    timer: Peri<'static, peripherals::TIMER2>,
    deadline_resources: crate::deadline_guard::Resources,
    _binding: impl Binding<RADIO, RadioHandler>
    + Binding<TIMER2, TimerHandler>
    + Binding<TIMER4, DeadlineHandler>,
) -> RadioOwner {
    RADIO::disable();
    TIMER2::disable();
    let deadline = crate::deadline_guard::initialize(&radio, deadline_resources);
    crate::clock_or_halt(SharedClock::read_static());
    let ficr = embassy_nrf::pac::FICR;
    let identity = device_identity(ficr.deviceid(0).read(), ficr.deviceid(1).read());
    let addresses = EsbAddresses::new(
        [0xe7; 4],
        [identity[0], identity[1], identity[2], identity[3]],
        [0xe7, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8],
        2, // RXADDRESS mask 0x03; identity[4] does not replace Nordic prefix C2.
    )
    .expect("two passive RX pipes");
    let config = EsbConfig {
        bitrate: Bitrate::Mbps2,
        receive_auto_ack: false,
        channel: Channel(40),
        crc: CrcConfig {
            length: 2,
            init: 0xffff,
            poly: 0x1021,
        },
        payload_length: 252,
        tx_power: TxPower::Pos8dBm,
        ..EsbConfig::default()
    };
    let driver: &'static Facade = FACADE.init(
        Esb::new_with_rx_clock_and_deadline(
            timer, radio, &POOL, &config, &addresses, rx_clock, deadline,
        )
        .expect("passive RX constructor"),
    );
    critical_section::with(|cs| *DRIVER.borrow(cs).borrow_mut() = Some(driver));
    RADIO::set_priority(Priority::P2);
    TIMER2::set_priority(Priority::P2);
    crate::deadline_guard::enable_irq();
    // SAFETY: unique real tokens consumed once, immutable facade/pool static,
    // all three vectors bound to this facade at nonnesting P2, TIMER0 at P3.
    // Shared clock initialized; hook accesses only independent CS clock state.
    // Embassy thread executor wakers defer polling. No NMI/fault facade access,
    // competing raw RADIO owner or FEM/MPSL exists here. Only the actor can send;
    // TIMER4's provider is bound to this facade's existing PPI3/4 routes.
    unsafe { driver.activate_stopped() }.expect("activate stopped actor radio");
    let mut lifecycle = RfLifecycle::default();
    lifecycle.stop_or_configure(0, 40);
    RadioOwner {
        driver,
        _pool: &POOL,
        identity,
        addresses,
        channel: 40,
        last_epoch: 0,
        lifecycle,
        tx: TxCounters::default(),
    }
}

pub(crate) fn latest() -> ReceiverSnapshot {
    critical_section::with(|cs| {
        let mut snapshot = *LATEST.borrow(cs).borrow();
        snapshot.irq_count = IRQ_COUNT.load(Ordering::Relaxed);
        snapshot.clock_errors = CLOCK_ERRORS.load(Ordering::Relaxed);
        snapshot
    })
}

pub(crate) struct RfPacket {
    pub bytes: [u8; 252],
    pub len: usize,
    pub rssi: i8,
    pub stamp: i64,
}

fn now() -> u64 {
    crate::clock_or_halt(SharedClock::read_static()) as u64
}

pub(crate) fn error_errno(error: Error) -> i32 {
    match error {
        Error::TxFull | Error::RxEmpty | Error::OutOfMemory => -11,
        Error::InvalidParam | Error::InvalidAddress => EINVAL,
        Error::MaxRetransmit => -110,
        Error::NotReady => ENOTSUP,
        Error::Busy => EBUSY,
        Error::RadioFault | Error::Mpsl => -5,
    }
}

fn fault_errno(fault: TxFault) -> i32 {
    match fault {
        TxFault::WrongContext => -1,
        TxFault::Busy => EBUSY,
        TxFault::NotReady => ENOTSUP,
        TxFault::TimedOut | TxFault::Deadline => -110,
        TxFault::Cancelled => -125,
        TxFault::StaleEpoch => -116,
        TxFault::RxEpochOverflow => -75,
        TxFault::ReceiverUnavailable => -11,
        TxFault::DeadlineGuardFault | TxFault::PowerCycled | TxFault::ClockRegressed => -5,
    }
}

impl RadioOwner {
    pub(crate) fn snapshot(&self) -> RadioSnapshot {
        let (wanted, min_epoch, _) = self.lifecycle.snapshot();
        RadioSnapshot {
            wanted,
            min_epoch,
            current_session: self
                .driver
                .receiver_epoch()
                .expect("thread RF epoch query")
                .map(|epoch| epoch.value()),
            channel: self.channel,
            tx: self.tx,
        }
    }
    pub(crate) fn identity(&self) -> [u8; 5] {
        self.identity
    }

    fn observe_epoch(&mut self) {
        if let Some(epoch) = self.driver.receiver_epoch().expect("thread RF epoch query") {
            self.last_epoch = self.last_epoch.max(epoch.value());
        }
    }

    pub(crate) fn stop(&mut self) -> i32 {
        self.observe_epoch();
        self.lifecycle
            .stop_or_configure(self.last_epoch, self.channel);
        self.driver.recv_stop().map_or_else(fault_errno, |_| 0)
    }

    pub(crate) fn configure(&mut self, channel: u8) -> i32 {
        let rc = self.stop();
        if rc != 0 {
            return rc;
        }
        self.lifecycle.stop_or_configure(self.last_epoch, channel);
        match self
            .driver
            .configure_idle(Channel(channel), &self.addresses)
        {
            Ok(()) => {
                self.channel = channel;
                0
            }
            Err(error) => error_errno(error),
        }
    }

    pub(crate) fn start(&mut self) -> i32 {
        // recv_start is idempotent when already receiving. Explicit START must
        // create a new lifecycle instead of retaining an older session epoch.
        let stopped = self.stop();
        if stopped != 0 {
            return stopped;
        }
        match self.driver.recv_start() {
            Ok(epoch) => {
                self.last_epoch = epoch.value();
                self.lifecycle.started(epoch.value());
                0
            }
            Err(error) => {
                // Vendor retains requested reception on allocation failure.
                self.driver.recv_stop().expect("withdraw failed RX desire");
                fault_errno(error)
            }
        }
    }

    pub(crate) async fn send_classified(
        &mut self,
        packet: RadioSend<'_>,
        deadline: Option<u64>,
        #[cfg(feature = "sync-trace")] mut trace: Option<&mut SendRecord>,
        #[cfg(feature = "sync-trace")] diagnostics: &mut Stats,
        #[cfg(feature = "sync-trace")] class: WireClass,
    ) -> RadioSendResult {
        if packet.pipe != 0 || packet.address != [0xe7; 5] {
            #[cfg(feature = "sync-trace")]
            diagnostics.record(class, Stage::ValidationRejected);
            #[cfg(feature = "sync-trace")]
            if let Some(record) = trace.as_deref_mut() {
                record.result = SendResult::ValidationRejected;
            }
            return RadioSendResult {
                code: EINVAL,
                disposition: SubmissionDisposition::NotSubmitted,
            };
        }
        let now_us = now();
        #[cfg(feature = "sync-trace")]
        if let Some(record) = trace.as_deref_mut() {
            record.submit_us = Some(now_us as i64);
        }
        let result = match deadline {
            Some(until) => {
                if let Err(result) =
                    nrf_embassy_firmware::actor_policy::bounded_air_deadline_classified(
                        now_us as i64,
                        until as i64,
                        packet.bytes.len(),
                    )
                {
                    #[cfg(feature = "slot-entry-trace")]
                    crate::timing_backend::record_budget_reject(
                        // Same airtime model as evaluate_budget.
                        (now_us as i64 + 129 + 4 * (packet.bytes.len() as i64 + 11) + 100
                            - until as i64)
                            .clamp(0, i64::from(u32::MAX)) as u32,
                        packet.bytes.len(),
                    );
                    #[cfg(feature = "sync-trace")]
                    diagnostics.record(class, Stage::BudgetRejected);
                    #[cfg(feature = "sync-trace")]
                    if let Some(record) = trace.as_deref_mut() {
                        record.result = SendResult::DeadlineRejected;
                    }
                    return result;
                }
                self.driver
                    .begin_send_no_ack_to_until(0, packet.bytes, now_us, until)
            }
            None => self.driver.begin_send_no_ack_to(0, packet.bytes, now_us),
        };
        let ticket = match result {
            Ok(ticket) => ticket,
            Err(error) => {
                #[cfg(feature = "sync-trace")]
                diagnostics.record(class, Stage::SubmissionRejected);
                #[cfg(feature = "sync-trace")]
                if let Some(record) = trace.as_deref_mut() {
                    record.result = SendResult::SubmissionRejected;
                }
                // A failed launch may already have touched hardware, even
                // though the driver did not return a ticket.
                return RadioSendResult {
                    code: error_errno(error),
                    disposition: SubmissionDisposition::Unknown,
                };
            }
        };
        #[cfg(feature = "sync-trace")]
        diagnostics.record(class, Stage::AcceptedTickets);
        #[cfg(feature = "sync-trace")]
        let epoch = ticket.epoch();
        #[cfg(feature = "sync-trace")]
        if let Some(record) = trace.as_deref_mut() {
            record.accepted_ticket = true;
        }
        #[cfg(feature = "sync-trace")]
        let mut last_poll_sample = None;
        #[cfg(feature = "sync-trace")]
        let report = ticket
            .wait_with_clock(|| {
                let sample = now();
                last_poll_sample = Some(sample);
                sample
            })
            .await;
        #[cfg(not(feature = "sync-trace"))]
        let report = ticket.wait(now).await;
        #[cfg(feature = "sync-trace")]
        {
            use embassy_nrf_esb::{TxCompletionOrigin, TxEndCaptureReason};
            use nrf_embassy_firmware::tx_failure_trace::{
                CompletionOrigin, EndCapture, EndCaptureReason, Observations, Record,
                Terminal as RawTerminal,
            };
            let terminal = match report.outcome {
                TxOutcome::PhysicalComplete => RawTerminal::PhysicalComplete,
                TxOutcome::Pending => RawTerminal::Pending,
                TxOutcome::Fault(fault) => match fault {
                    TxFault::WrongContext => RawTerminal::WrongContext,
                    TxFault::Busy => RawTerminal::Busy,
                    TxFault::NotReady => RawTerminal::NotReady,
                    TxFault::TimedOut => RawTerminal::TimedOut,
                    TxFault::Deadline => RawTerminal::Deadline,
                    TxFault::DeadlineGuardFault => RawTerminal::DeadlineGuardFault,
                    TxFault::Cancelled => RawTerminal::Cancelled,
                    TxFault::PowerCycled => RawTerminal::PowerCycled,
                    TxFault::ClockRegressed => RawTerminal::ClockRegressed,
                    TxFault::ReceiverUnavailable => RawTerminal::ReceiverUnavailable,
                    TxFault::StaleEpoch => RawTerminal::StaleEpoch,
                    TxFault::RxEpochOverflow => RawTerminal::RxEpochOverflow,
                },
            };
            let observations = report.observations.map(|o| Observations {
                completion: o.completion.map(|origin| match origin {
                    TxCompletionOrigin::RadioIrq => CompletionOrigin::RadioIrq,
                    TxCompletionOrigin::TaskService => CompletionOrigin::TaskService,
                }),
                provider_expiry_observed: o.provider_expiry_observed,
                end_capture: o.end_capture.map(|capture| EndCapture {
                    epoch: capture.epoch.value(),
                    epoch_match: capture.epoch == epoch,
                    anchor_raw: capture.anchor_raw,
                    cutoff_raw: capture.cutoff_raw,
                    sampled_cc: capture.sampled_cc,
                    observation_raw: capture.observation_raw,
                    elapsed_us: capture.elapsed_us,
                    horizon_us: capture.horizon_us,
                    reason: match capture.reason {
                        TxEndCaptureReason::Missing => EndCaptureReason::Missing,
                        TxEndCaptureReason::Unstable => EndCaptureReason::Unstable,
                        TxEndCaptureReason::NoCompletionEvidence => {
                            EndCaptureReason::NoCompletionEvidence
                        }
                        TxEndCaptureReason::OutsideEpochRange => {
                            EndCaptureReason::OutsideEpochRange
                        }
                        TxEndCaptureReason::ObservationOutsideHorizon => {
                            EndCaptureReason::ObservationOutsideHorizon
                        }
                        TxEndCaptureReason::NotArmed => EndCaptureReason::NotArmed,
                        TxEndCaptureReason::Candidate => EndCaptureReason::Candidate,
                    },
                    cutoff_delta_us: capture.cutoff_delta_us,
                }),
            });
            // Wait and all facade loans have ended. Parse outside recorder CS;
            // capture performs only a bounded copy, without clock/log/await work.
            if terminal != RawTerminal::PhysicalComplete
                && let Some(record) = Record::failure(
                    packet.bytes,
                    (epoch.value(), report.epoch.value(), epoch == report.epoch),
                    now_us,
                    deadline,
                    last_poll_sample,
                    (terminal, observations),
                )
            {
                crate::sync_trace_runtime::observe_failure(record);
            }
        }
        #[cfg(feature = "sync-trace")]
        diagnostics.record_terminal(
            class,
            match report.outcome {
                TxOutcome::PhysicalComplete => Terminal::PhysicalComplete,
                TxOutcome::Fault(TxFault::Deadline) => Terminal::Deadline,
                TxOutcome::Fault(TxFault::TimedOut) => Terminal::TimedOut,
                TxOutcome::Fault(TxFault::Cancelled) => Terminal::Cancelled,
                TxOutcome::Pending => Terminal::Pending,
                TxOutcome::Fault(_) => Terminal::OtherFault,
            },
        );
        #[cfg(feature = "sync-trace")]
        if let Some(record) = trace {
            let terminal_return_us = SharedClock::read_static().ok();
            let result = match report.outcome {
                TxOutcome::PhysicalComplete => SendResult::PhysicalComplete,
                TxOutcome::Pending => SendResult::UnexpectedPending,
                TxOutcome::Fault(TxFault::Deadline) => SendResult::DeadlineRejected,
                TxOutcome::Fault(TxFault::TimedOut) => SendResult::TimedOut,
                TxOutcome::Fault(_) => SendResult::OtherFault,
            };
            record.finish_terminal(result, epoch == report.epoch, terminal_return_us);
        }
        let (terminal, rc) = match report.outcome {
            TxOutcome::PhysicalComplete => (TxTerminal::PhysicalComplete, 0),
            TxOutcome::Pending => (TxTerminal::Pending, -5),
            TxOutcome::Fault(fault) => (
                if matches!(fault, TxFault::Deadline | TxFault::TimedOut) {
                    TxTerminal::Deadline
                } else {
                    TxTerminal::Fault
                },
                fault_errno(fault),
            ),
        };
        self.tx.record_send_result(terminal, rc)
    }

    pub(crate) fn service(&mut self) {
        let _ = self.driver.service(now());
        self.observe_epoch(); // Does NOT advance the actor lifecycle floor.
    }

    pub(crate) fn dequeue(&mut self) -> Result<Option<RfPacket>, i32> {
        let Some(packet) = self.driver.try_receive().map_err(error_errno)? else {
            return Ok(None);
        };
        let mut bytes = [0u8; 252];
        let len = packet.payload().len();
        bytes[..len].copy_from_slice(packet.payload());
        let rssi = (-i16::from(packet.rssi())).max(i16::from(i8::MIN)) as i8;
        let metadata = packet.metadata();
        drop(packet);
        critical_section::with(|cs| {
            LATEST.borrow(cs).borrow_mut().record(ReceiverRecord {
                bytes: len as u32,
                metadata: metadata.map(|metadata| ReceiverMetadata {
                    channel: metadata.rf_channel,
                    epoch: metadata.receiver_epoch.value(),
                    timestamp_us: metadata.timestamp_us,
                }),
            });
        });
        let Some(metadata) = metadata else {
            return Ok(None);
        };
        if !self.lifecycle.accepts(
            len,
            metadata.rf_channel,
            metadata.receiver_epoch.value(),
            metadata.timestamp_us,
        ) {
            return Ok(None);
        }
        Ok(Some(RfPacket {
            bytes,
            len,
            rssi,
            stamp: metadata.timestamp_us.expect("validated RX stamp"),
        }))
    }
}
