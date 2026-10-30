//! ARM-private shared TIMER0 clock. Inherited single-core PRIMASK contract:
//! no external PPI/user may capture CC3, clear, stop or reconfigure TIMER0.
//! With slot-entry-trace, the TIMER0 IRQ also captures CC3 under the same
//! critical section, reading only the low 32 bits without mutating wrap state.

use core::cell::RefCell;
use cortex_m::peripheral::scb::{Exception, VectActive};
use critical_section::{CriticalSection, Mutex};
use embassy_nrf::{Peri, pac, peripherals};
use nrf_embassy_firmware::clock::{ClockContext, ClockFault, SharedClockState, context_gate};

static CLOCK: Mutex<RefCell<SharedClockState>> =
    Mutex::new(RefCell::new(SharedClockState::inactive()));

fn context() -> ClockContext {
    match cortex_m::peripheral::SCB::vect_active() {
        VectActive::ThreadMode => ClockContext::Thread,
        VectActive::Exception(Exception::NonMaskableInt) => ClockContext::Nmi,
        VectActive::Exception(Exception::HardFault) => ClockContext::HardFault,
        _ => ClockContext::MaskableException,
    }
}

/// No register capability. Copies remain valid only while the owner is alive.
#[derive(Clone, Copy)]
pub(crate) struct SharedClock {
    _private: (),
}

impl SharedClock {
    /// Static function-pointer entry: the state rejects inactive access before MMIO.
    pub(crate) fn read_static() -> Result<i64, ClockFault> {
        Self { _private: () }.now_us()
    }

    pub(crate) fn now_us(self) -> Result<i64, ClockFault> {
        // NMI/HardFault rejection must precede both PRIMASK guard and RefCell.
        context_gate(context(), || critical_section::with(|cs| self.read(cs)))
    }

    pub(crate) fn read(self, cs: CriticalSection<'_>) -> Result<i64, ClockFault> {
        context_gate(context(), || {
            CLOCK.borrow(cs).borrow_mut().capture_and_extend(|| {
                pac::TIMER0.tasks_capture(3).write_value(1);
                pac::TIMER0.cc(3).read()
            })
        })
    }
}

pub(crate) struct PrivateClockLease {
    clock: SharedClock,
}

impl PrivateClockLease {
    /// Called only by the unique TIMER0 owner while its IRQ is masked. Retains
    /// no borrowed owner: the real token stays in TimingPlatform, never cloned.
    pub(crate) fn initialize(
        cs: CriticalSection<'_>,
        _timer: &Peri<'static, peripherals::TIMER0>,
    ) -> Self {
        let r = pac::TIMER0;
        r.tasks_stop().write_value(1);
        r.mode()
            .write(|w| w.set_mode(pac::timer::vals::Mode::TIMER));
        r.bitmode()
            .write(|w| w.set_bitmode(pac::timer::vals::Bitmode::_32BIT));
        r.prescaler().write(|w| w.set_prescaler(4));
        r.shorts().write_value(pac::timer::regs::Shorts(0));
        r.tasks_clear().write_value(1);
        for index in 0..4 {
            r.cc(index).write_value(0);
        }
        r.tasks_start().write_value(1);
        CLOCK.borrow(cs).borrow_mut().activate();
        Self {
            clock: SharedClock { _private: () },
        }
    }

    pub(crate) fn clock(&self) -> SharedClock {
        self.clock
    }

    pub(crate) fn deactivate(&self, cs: CriticalSection<'_>) {
        CLOCK.borrow(cs).borrow_mut().deactivate();
    }
}
