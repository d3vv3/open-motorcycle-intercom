//! ESB timer abstraction over nRF TIMER peripheral.
//!
//! Provides microsecond-precision timing for ESB protocol operations:
//! - CC\[0\]: retransmit timeout (absolute, clear + start)
//! - CC\[1\]: ACK timeout (relative, capture + add)
//!
//! Uses Embassy generic pattern for timer selection — no feature gates.
//! TIMER0 is excluded (reserved for MPSL).
//!
//! Portions adapted from [esb-ng](https://github.com/jamesmunns/esb)
//! (MIT OR Apache-2.0). See `NOTICE.md` for attribution.
//!
//! Ref: esb-ng `src/peripherals.rs` lines 500–684.
//! Ref: nRF52840 PS §6.24 TIMER.

use embassy_nrf::PeripheralType;

use crate::pac::timer::vals::Bitmode;
use crate::pac::timer::{Timer, regs};

mod sealed {
    pub(crate) struct Access(());
    pub(super) const ACCESS: Access = Access(());

    pub(crate) trait Sealed {
        const COMPARE_COUNT: usize;
        fn regs(access: Access) -> super::Timer;
        fn interrupt(access: Access) -> crate::pac::Interrupt;
    }
}

/// Timer peripheral trait — Embassy generic pattern (A1).
///
/// Implemented for `embassy_nrf::peripherals::TIMER1` through `TIMER4`.
/// TIMER0 is NOT available - owned by MPSL.
/// This is a marker bound only; peripheral access stays internal. A private
/// capability also prevents external calls through generic supertrait lookup.
///
/// ```compile_fail
/// fn raw_timer<T: embassy_nrf_esb::timer::TimerInstance>() {
///     let _ = T::regs();
/// }
/// ```
#[allow(private_bounds)]
pub trait TimerInstance: sealed::Sealed + PeripheralType + 'static {}

pub(crate) fn timer_regs<T: TimerInstance>() -> Timer {
    <T as sealed::Sealed>::regs(sealed::ACCESS)
}

pub(crate) fn timer_interrupt<T: TimerInstance>() -> crate::pac::Interrupt {
    <T as sealed::Sealed>::interrupt(sealed::ACCESS)
}

/// ESB timer driver.
///
/// Consumes the Embassy peripheral token at construction, preventing
/// other code from accessing the same TIMER instance. ISR accesses
/// the hardware via `timer_regs::<T>()` (Copy, no `&'static mut` needed).
pub(crate) struct EsbTimer<T: TimerInstance> {
    _timer: embassy_nrf::Peri<'static, T>,
}

// ---- Macro: implement TimerInstance for Embassy peripheral types ----
// Ref: embassy-nrf `src/timer.rs` lines 33–52.

macro_rules! impl_timer_instance {
    ($type:ident, $pac_type:ident, $irq:ident, $compare_count:expr) => {
        impl sealed::Sealed for embassy_nrf::peripherals::$type {
            const COMPARE_COUNT: usize = $compare_count;
            #[inline]
            fn regs(_access: sealed::Access) -> Timer {
                unsafe { Timer::from_ptr(crate::pac::$pac_type.as_ptr()) }
            }
            #[inline]
            fn interrupt(_access: sealed::Access) -> crate::pac::Interrupt {
                crate::pac::Interrupt::$irq
            }
        }
        impl TimerInstance for embassy_nrf::peripherals::$type {}
    };
}

// TIMER0 intentionally excluded — reserved for MPSL (PS §6.24).
impl_timer_instance!(TIMER1, TIMER1, TIMER1, 4);
impl_timer_instance!(TIMER2, TIMER2, TIMER2, 4);
impl_timer_instance!(TIMER3, TIMER3, TIMER3, 6);
impl_timer_instance!(TIMER4, TIMER4, TIMER4, 6);

#[allow(dead_code)]
impl<T: TimerInstance> EsbTimer<T> {
    /// Create a new ESB timer, consuming the Embassy peripheral token.
    ///
    /// Initializes the timer to 32-bit mode, 1 MHz (prescaler = 4).
    /// Ref: esb-ng `peripherals.rs` lines 592–602.
    pub(crate) fn new(_timer: embassy_nrf::Peri<'static, T>) -> Self {
        let t = timer_regs::<T>();

        // Disable all timer interrupts (esb-ng line 594).
        t.intenclr().write_value(regs::Int(0xFFFF_FFFF));

        // Stop any running counter (esb-ng line 595).
        t.tasks_stop().write_value(1);

        // 32-bit timer mode (esb-ng line 598).
        t.bitmode().write(|w| w.set_bitmode(Bitmode::_32BIT));

        // Prescaler = 4 → 16 MHz / 2^4 = 1 MHz (µs resolution)
        // (esb-ng line 601).
        t.prescaler().write(|w| w.set_prescaler(4));

        Self { _timer }
    }

    /// Get PAC Timer handle for direct register access.
    ///
    /// `pub(crate)` — not exposed outside this crate.
    #[inline]
    pub(crate) fn regs(&self) -> Timer {
        timer_regs::<T>()
    }

    // ---- CC[0]: Retransmit timeout ----

    /// Arm the retransmit timer.
    ///
    /// Sets CC[0] to `micros`, clears and restarts the counter from 0.
    /// Enables the COMPARE[0] interrupt.
    ///
    /// The timer value is absolute (count from 0 to micros).
    /// Ref: esb-ng `peripherals.rs` lines 608–616.
    #[inline]
    pub(crate) fn arm_retransmit(&self, micros: u16) {
        let t = timer_regs::<T>();

        t.cc(0).write_value(micros as u32);
        t.events_compare(0).write_value(0);
        t.intenset().write(|w| w.set_compare(0, true));

        // Clear counter and start counting from 0 (esb-ng lines 614–615).
        t.tasks_clear().write_value(1);
        t.tasks_start().write_value(1);
    }

    /// Disarm the retransmit timer and stop the counter.
    ///
    /// Ref: esb-ng `peripherals.rs` lines 619–627.
    #[inline]
    pub(crate) fn disarm_retransmit(&self) {
        let t = timer_regs::<T>();

        t.intenclr().write(|w| w.set_compare(0, true));
        t.events_compare(0).write_value(0);
        t.tasks_stop().write_value(1);
    }

    /// Check if the retransmit compare event has fired.
    ///
    /// Ref: esb-ng `peripherals.rs` lines 630–634.
    #[inline]
    pub(crate) fn is_retransmit_fired(&self) -> bool {
        timer_regs::<T>().events_compare(0).read() == 1
    }

    // ---- CC[1]: ACK timeout ----

    /// Arm the ACK timeout timer.
    ///
    /// Captures the current counter value into CC[1], then adds `micros`
    /// to set a relative timeout. Enables the COMPARE[1] interrupt.
    ///
    /// Unlike retransmit (absolute), ACK timeout is relative to the
    /// current counter position — no clear/start.
    /// Ref: esb-ng `peripherals.rs` lines 637–647.
    #[inline]
    pub(crate) fn arm_ack_timeout(&self, micros: u16) {
        let t = timer_regs::<T>();

        // Capture current counter into CC[1] (esb-ng line 639).
        t.tasks_capture(1).write_value(1);
        let current = t.cc(1).read();

        // Set CC[1] = current + timeout (esb-ng line 643).
        t.cc(1).write_value(current + micros as u32);
        t.events_compare(1).write_value(0);
        t.intenset().write(|w| w.set_compare(1, true));
    }

    /// Disarm the ACK timeout timer.
    ///
    /// Ref: esb-ng `peripherals.rs` lines 650–656.
    #[inline]
    pub(crate) fn disarm_ack_timeout(&self) {
        let t = timer_regs::<T>();

        t.intenclr().write(|w| w.set_compare(1, true));
        t.events_compare(1).write_value(0);
    }

    /// Check if the ACK timeout compare event has fired.
    ///
    /// Ref: esb-ng `peripherals.rs` lines 659–663.
    #[inline]
    pub(crate) fn is_ack_timeout_fired(&self) -> bool {
        timer_regs::<T>().events_compare(1).read() == 1
    }

    // ---- General ----

    /// Stop the timer counter (esb-ng line 668).
    #[inline]
    pub(crate) fn stop(&self) {
        timer_regs::<T>().tasks_stop().write_value(1);
    }

    pub(crate) fn clear_all_events(&self) {
        let t = timer_regs::<T>();
        t.tasks_stop().write_value(1);
        t.intenclr().write_value(regs::Int(u32::MAX));
        t.shorts().write_value(regs::Shorts(0));
        for index in 0..<T as sealed::Sealed>::COMPARE_COUNT {
            t.events_compare(index).write_value(0);
        }
    }

    /// Read the current timer counter value via CC[2] capture.
    ///
    /// Uses CC[2] to avoid interfering with CC[0]/CC[1] which are
    /// reserved for retransmit and ACK timeout respectively.
    #[inline]
    pub(crate) fn now(&self) -> u32 {
        let t = timer_regs::<T>();
        t.tasks_capture(2).write_value(1);
        t.cc(2).read()
    }
}
