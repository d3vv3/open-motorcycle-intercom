#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::config::{Debug, HfclkSource, LfclkSource};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::gpiote::{OutputChannel, OutputChannelPolarity};
use embassy_nrf::ppi::Ppi;
use embassy_nrf::timer::{Frequency as TimerFrequency, Timer as HardwareTimer};
use embassy_nrf::{bind_interrupts, peripherals, spim, usb};
use embassy_time::{Duration, Ticker, Timer, with_timeout};
#[cfg(not(feature = "nonblocking-bridge-delivery"))]
use nrf_embassy::bridge::Bridge;
use nrf_embassy::bridge::{Consumer, TRANSFER_SIZE};
use nrf_embassy::protocol_audio::IngressToken;
use nrf_embassy::runtime::{Command, CommandKind};
use nrf_embassy_firmware::cancel::{AbortOperations, GuardedFuture};
use nrf_embassy_firmware::clock::ClockFault;
use nrf_embassy_firmware::diagnostics::DiagnosticSnapshot;
#[cfg(not(feature = "reset-diagnostics"))]
use panic_halt as _;
use shared_clock::SharedClock;

mod bridge_rpc_mailbox;
mod deadline_guard;
mod protocol_actor;
mod protocol_backend;
mod radio_receiver;
#[cfg(feature = "reset-diagnostics")]
mod reset_diagnostics_runtime;
mod shared_clock;
#[cfg(feature = "sync-trace")]
mod sync_trace_runtime;
mod timing_backend;
mod transport_mailbox;
mod usb_diagnostics;

bind_interrupts!(struct Irqs {
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    TIMER0 => timing_backend::AlarmHandler;
    RADIO => radio_receiver::RadioHandler;
    TIMER2 => radio_receiver::TimerHandler;
    TIMER4 => radio_receiver::DeadlineHandler;
});

// No SPIM1 register/task accessor or DerefMut escapes this owner. In particular,
// no external PPI route may target SPIM1 START while it owns TWISPI1.
struct PrivateCancelSafeSpim<'d> {
    spi: spim::Spim<'d>,
    mmio: Spim1Mmio,
}

impl<'d> PrivateCancelSafeSpim<'d> {
    fn new(
        peripheral: embassy_nrf::Peri<'d, peripherals::TWISPI1>,
        sck: embassy_nrf::Peri<'d, peripherals::P1_13>,
        miso: embassy_nrf::Peri<'d, peripherals::P1_12>,
        mosi: embassy_nrf::Peri<'d, peripherals::P1_11>,
        config: spim::Config,
    ) -> Self {
        Self {
            spi: spim::Spim::new(peripheral, Irqs, sck, miso, mosi, config),
            mmio: Spim1Mmio,
        }
    }

    fn transfer_from_ram<'a>(
        &'a mut self,
        rx: &'a mut [u8],
        tx: &'a [u8],
    ) -> impl core::future::Future<Output = Result<(), spim::Error>> + 'a {
        // Split borrows keep MMIO available to PinnedDrop while the HAL child
        // still retains the SPI and RAM borrows. Do not wrap a copying transfer.
        GuardedFuture::new(self.spi.transfer_from_ram(rx, tx), &mut self.mmio)
    }
}

struct Spim1Mmio;

impl AbortOperations for Spim1Mmio {
    fn abort(&mut self) {
        const STOP_SPIN_LIMIT: usize = 1_000_000;
        // Raw PAC access is confined to the exclusively owned SPIM1 wrapper.
        // The IRQ handler only observes END and disables its interrupt; neither
        // it nor any configured PPI route starts a new transfer during abort.
        let r = embassy_nrf::pac::SPIM1;
        r.intenclr().write(|w| w.set_end(true));
        r.events_stopped().write_value(0);
        let _ = r.events_stopped().read();
        r.tasks_stop().write_value(1);
        for _ in 0..STOP_SPIN_LIMIT {
            if r.events_stopped().read() != 0 {
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                cortex_m::asm::dsb();
                r.events_end().write_value(0);
                let _ = r.events_end().read();
                return;
            }
            core::hint::spin_loop();
        }
        // No disable/reset is assumed to quiesce DMA. panic-halt never unwinds:
        // the pinned child and its TX/RX storage remain alive on this path.
        cortex_m::interrupt::disable();
        panic!("SPIM1 STOPPED not observed; retaining DMA buffers");
    }
}

#[used]
#[unsafe(link_section = ".integration_metadata")]
static INTEGRATION_METADATA: [u8; include_bytes!(concat!(
    env!("OUT_DIR"),
    "/integration-metadata.txt"
))
.len()] = *include_bytes!(concat!(env!("OUT_DIR"), "/integration-metadata.txt"));

struct AckPulse<'d> {
    timer: HardwareTimer<'d>,
    _output: OutputChannel<'d>,
    _set: Ppi<'d, peripherals::PPI_CH0, 1, 1>,
    _clear: Ppi<'d, peripherals::PPI_CH1, 1, 1>,
}

impl AckPulse<'_> {
    fn fire(&mut self) {
        self.timer.stop();
        self.timer.clear();
        self.timer.cc(0).clear_events();
        self.timer.cc(1).clear_events();
        // Both edges are hardware events, not a CPU SET followed by START.
        self.timer.start();
    }
}

struct MailboxConsumer<'a, 'd> {
    producer: transport_mailbox::Producer,
    token: IngressToken,
    ack: &'a mut AckPulse<'d>,
}

impl Consumer for MailboxConsumer<'_, '_> {
    fn accept_audio_v2(&mut self, payload: &[u8]) -> i32 {
        self.producer
            .publish(self.token.validate(payload))
            .expect("SPI thread publication")
    }

    fn request_start(&mut self, generation: u8, group: u8) {
        self.command(CommandKind::Start, generation, group);
    }

    fn request_stop(&mut self, generation: u8) {
        self.command(CommandKind::Stop, generation, 0);
    }

    fn request_pause(&mut self, generation: u8) {
        self.command(CommandKind::Pause, generation, 0);
    }

    fn request_resume(&mut self, generation: u8) {
        self.command(CommandKind::Resume, generation, 0);
    }

    fn request_status(&mut self) {
        assert_eq!(
            self.producer
                .request_status()
                .expect("SPI status publication"),
            0
        );
    }

    fn pulse_ack(&mut self) {
        self.ack.fire();
    }
}

impl MailboxConsumer<'_, '_> {
    fn command(&mut self, kind: CommandKind, generation: u8, talk_channel: u8) {
        assert_eq!(
            self.producer
                .command(Command {
                    kind,
                    generation,
                    talk_channel
                })
                .expect("SPI command publication"),
            0
        );
    }
}

struct SelectedCs<'a, 'd>(&'a mut Output<'d>);

impl<'a, 'd> SelectedCs<'a, 'd> {
    fn new(cs: &'a mut Output<'d>) -> Self {
        cs.set_low();
        Self(cs)
    }
}

impl Drop for SelectedCs<'_, '_> {
    fn drop(&mut self) {
        self.0.set_high();
    }
}

fn clock_or_halt(result: Result<i64, ClockFault>) -> i64 {
    match result {
        Ok(now) => now,
        Err(_) => {
            // panic-halt cannot unwind: any live DMA child/buffers stay alive.
            cortex_m::interrupt::disable();
            panic!("shared TIMER0 clock fault");
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.lfclk_source = LfclkSource::InternalRC;
    config.debug = Debug::NotConfigured;
    config.dcdc.reg0_voltage = None;
    let p = embassy_nrf::init(config);
    #[cfg(feature = "reset-diagnostics")]
    reset_diagnostics_runtime::capture_boot();
    let (timing, clock) =
        timing_backend::initialize(p.TIMER0, p.TIMER1, p.GPIOTE_CH1, p.PPI_CH2, p.P0_02);
    #[cfg(feature = "rx-blocking-trace")]
    nrf_embassy_firmware::rx_blocking_trace::install_clock(|| SharedClock::read_static().ok());
    let receiver = radio_receiver::initialize(
        p.RADIO,
        p.TIMER2,
        deadline_guard::Resources {
            timer: p.TIMER4,
            ch3: p.PPI_CH3,
            ch4: p.PPI_CH4,
            group0: p.PPI_GROUP0,
            #[cfg(feature = "sync-trace")]
            ch5: p.PPI_CH5,
        },
        Irqs,
    );
    usb_diagnostics::start(spawner, p.USBD);

    let (inbox, producer) = transport_mailbox::Actor::initialize_owned_gate(
        nrf_embassy::protocol_audio::IngressAdmissionGate {
            enabled: false,
            epoch: 0,
        },
    )
    .expect("unique ingress lease");
    let (rpc, server) = bridge_rpc_mailbox::Actor::initialize().expect("unique RPC lease");

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M4;
    spi_config.mode = spim::MODE_0;
    let spi = PrivateCancelSafeSpim::new(p.TWISPI1, p.P1_13, p.P1_12, p.P1_11, spi_config);
    let cs = Output::new(p.P1_14, Level::High, OutputDrive::Standard);
    let red = Output::new(p.P0_26, Level::High, OutputDrive::Standard);

    let timer = HardwareTimer::new(p.TIMER3);
    timer.set_frequency(TimerFrequency::F1MHz);
    timer.cc(0).write(1);
    timer.cc(1).write(21);
    timer.cc(1).short_compare_stop();
    timer.cc(1).short_compare_clear();
    timer.cc(0).clear_events();
    timer.cc(1).clear_events();
    let output = OutputChannel::new(
        p.GPIOTE_CH0,
        p.P1_15,
        Level::Low,
        OutputDrive::Standard,
        OutputChannelPolarity::Toggle,
    );
    let mut set = Ppi::new_one_to_one(p.PPI_CH0, timer.cc(0).event_compare(), output.task_set());
    let mut clear = Ppi::new_one_to_one(p.PPI_CH1, timer.cc(1).event_compare(), output.task_clr());
    set.enable();
    clear.enable();
    let ack = AckPulse {
        timer,
        _output: output,
        _set: set,
        _clear: clear,
    };

    spawner.spawn(spi_task(spi, cs, red, ack, producer, server, clock).expect("SPI owner task"));
    spawner.spawn(
        protocol_actor::task(
            protocol_backend::Backend {
                tdma: nrf_embassy::tdma::Tdma::new(),
                timing,
                radio: receiver,
                rpc,
                producer,
                ingress_work: false,
                rf_work: false,
                #[cfg(feature = "nonblocking-bridge-delivery")]
                rx_outputs: false,
                tx_window: None,
                #[cfg(feature = "sync-trace")]
                sync_trace: nrf_embassy_firmware::sync_trace::Tracker::new(),
                #[cfg(feature = "sync-trace")]
                tx_diagnostics: nrf_embassy_firmware::tx_diagnostics::Stats::default(),
            },
            inbox,
        )
        .expect("live unqualified protocol actor"),
    );
}

#[embassy_executor::task]
async fn spi_task(
    mut spi: PrivateCancelSafeSpim<'static>,
    mut cs: Output<'static>,
    mut red: Output<'static>,
    mut ack: AckPulse<'static>,
    producer: transport_mailbox::Producer,
    mut server: bridge_rpc_mailbox::Server,
    clock: SharedClock,
) {
    #[cfg(not(feature = "nonblocking-bridge-delivery"))]
    let mut bridge = Bridge::new();
    server.mark_bridge_initialized();
    #[cfg(feature = "nonblocking-bridge-delivery")]
    let bridge = server.coordinator();
    let mut ticker = Ticker::every(Duration::from_millis(2));
    let mut heartbeat = clock_or_halt(clock.now_us());
    let mut polls = 0u32;
    let mut spi_ok = 0u32;
    let mut spi_fail = 0u32;
    loop {
        let poll_due = {
            use core::future::{Future, poll_fn};
            use core::pin::pin;
            use core::task::Poll;
            let mut next = pin!(ticker.next());
            let mut request = pin!(server.wait());
            poll_fn(|cx| {
                // Poll deadline first so an RPC stream cannot starve transfers.
                if next.as_mut().poll(cx).is_ready() {
                    return Poll::Ready(true);
                }
                if let Poll::Ready(result) = request.as_mut().poll(cx) {
                    result.expect("SPI RPC waiter context");
                    return Poll::Ready(false);
                }
                Poll::Pending
            })
            .await
        };
        #[cfg(not(feature = "nonblocking-bridge-delivery"))]
        server.execute(&mut bridge).expect("SPI RPC before prepare");
        #[cfg(feature = "nonblocking-bridge-delivery")]
        server.execute().expect("SPI RPC before begin");
        if !poll_due {
            continue;
        }
        // Captured before Prepared/DMA, never recaptured during consumption.
        let token = producer.capture().expect("SPI capture ingress gate");
        #[cfg(not(feature = "nonblocking-bridge-delivery"))]
        let prepared = bridge.prepare();
        #[cfg(feature = "nonblocking-bridge-delivery")]
        let mut prepared = bridge.begin().expect("unique SPI transfer lease");
        #[cfg(not(feature = "nonblocking-bridge-delivery"))]
        let tx = nrf_embassy_firmware::actor_policy::status_tx(
            prepared.bytes(),
            protocol_actor::capable(),
        );
        #[cfg(not(feature = "nonblocking-bridge-delivery"))]
        let tx = &tx;
        #[cfg(feature = "nonblocking-bridge-delivery")]
        if !protocol_actor::capable() {
            *prepared.bytes_mut() = nrf_embassy_firmware::unsupported_status_tx(prepared.bytes());
        }
        #[cfg(feature = "nonblocking-bridge-delivery")]
        let tx = prepared.bytes();
        let mut rx = [0; TRANSFER_SIZE];
        let selected = SelectedCs::new(&mut cs);
        Timer::after_micros(100).await;
        // Timeout drops the inner guard before RAM reuse or CS-high. Both
        // timeout and HAL errors finish as failures after the coded hold delay.
        let ok = matches!(
            with_timeout(
                Duration::from_micros(1500),
                spi.transfer_from_ram(&mut rx, tx),
            )
            .await,
            Ok(Ok(()))
        );
        Timer::after_micros(50).await;
        drop(selected);
        prepared.finish(
            ok,
            &rx,
            &mut MailboxConsumer {
                producer,
                token,
                ack: &mut ack,
            },
        );
        #[cfg(not(feature = "nonblocking-bridge-delivery"))]
        server
            .execute(&mut bridge)
            .expect("SPI RPC after prepared finish");
        #[cfg(feature = "nonblocking-bridge-delivery")]
        server.execute().expect("SPI RPC after detached finish");
        polls = polls.wrapping_add(1);
        if ok {
            spi_ok = spi_ok.wrapping_add(1);
        } else {
            spi_fail = spi_fail.wrapping_add(1);
        }
        let now = clock_or_halt(clock.now_us());
        if now - heartbeat >= 1_000_000 {
            red.toggle();
            heartbeat = now;
            // Prepared is consumed and RPC execution has returned; no bridge
            // borrow or asynchronous operation spans this publication capture.
            #[cfg(feature = "pipeline-trace")]
            let bridge_snapshot = bridge.snapshot();
            #[cfg(feature = "pipeline-trace")]
            let bridge_clock_us = clock_or_halt(clock.now_us()) as u64;
            let timing = timing_backend::latest();
            #[cfg(feature = "sync-trace")]
            usb_diagnostics::publish_trace(sync_trace_runtime::drain());
            usb_diagnostics::publish(DiagnosticSnapshot {
                clock_us: now as u64,
                polls,
                spi_ok,
                spi_fail,
                tdma_clock_us: timing.clock_us,
                tdma_frame: timing.frame,
                tdma_jitter_max_us: timing.jitter_max_us,
                tdma_skipped: timing.skipped,
                tdma_correction_pending_us: timing.correction_pending_us,
                tdma_fault: timing.fault,
                ws_frame: timing.ws_frame,
                ws_count: timing.ws_count,
                ws: timing.ws,
                drops: 0,
                receiver: radio_receiver::latest(),
                ingress: producer.snapshot().ok(),
                #[cfg(feature = "pipeline-trace")]
                bridge: bridge_snapshot,
                #[cfg(feature = "pipeline-trace")]
                bridge_clock_us,
                ..DiagnosticSnapshot::default()
            });
        }
    }
}
