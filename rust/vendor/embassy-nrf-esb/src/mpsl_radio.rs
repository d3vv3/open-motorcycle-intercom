//! RADIO handoff helpers used by MPSL timeslot diagnostics.

use core::sync::atomic::{Ordering, compiler_fence};

use embassy_nrf::pac;

use crate::mpsl_common::spin_until;

const RADIO_DISABLE_SPIN_LIMIT: u32 = 100_000;

/// Result of asking RADIO to reach DISABLED within a bounded spin window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioDisableResult {
    Disabled,
    TimedOut,
}

impl RadioDisableResult {
    pub const fn timed_out(self) -> bool {
        matches!(self, Self::TimedOut)
    }
}

/// Recovery policy used when handing RADIO back at the end of a timeslot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioRecoveryPolicy {
    /// Force a RADIO power cycle after the bounded disable attempt. This is the
    /// conservative diagnostic policy before handing the peripheral back to SDC.
    ForceResetAfterBoundedDisable,
}

/// Result of quiescing RADIO before MPSL hands control back to other protocols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioQuiesceResult {
    Disabled,
    TimedOutForcedReset,
}

impl RadioQuiesceResult {
    pub const fn timed_out(self) -> bool {
        matches!(self, Self::TimedOutForcedReset)
    }
}

pub(crate) fn disable_radio_bounded(clear_rxen: bool, clear_txen: bool) -> RadioDisableResult {
    let r = pac::RADIO;
    r.shorts().modify(|w| {
        w.set_ready_start(false);
        w.set_end_disable(false);
        if clear_rxen {
            w.set_disabled_rxen(false);
        }
        if clear_txen {
            w.set_disabled_txen(false);
        }
    });
    r.intenclr().write(|w| w.set_disabled(true));
    r.events_disabled().write_value(0);
    r.tasks_disable().write_value(1);
    let disabled = spin_until(|| r.events_disabled().read() != 0, RADIO_DISABLE_SPIN_LIMIT);
    r.events_disabled().write_value(0);
    compiler_fence(Ordering::Acquire);

    if disabled {
        RadioDisableResult::Disabled
    } else {
        RadioDisableResult::TimedOut
    }
}

pub(crate) fn quiesce_radio_before_timeslot_end(policy: RadioRecoveryPolicy) -> RadioQuiesceResult {
    let r = pac::RADIO;
    let disabled = disable_radio_bounded(true, true);

    r.events_address().write_value(0);
    r.events_payload().write_value(0);
    r.events_end().write_value(0);
    r.events_ready().write_value(0);
    r.events_disabled().write_value(0);

    match policy {
        RadioRecoveryPolicy::ForceResetAfterBoundedDisable => {
            // Reset residual RADIO state programmed by the ESB slot before MPSL
            // hands the peripheral back to SDC.
            r.power().write(|w| w.set_power(false));
            r.power().write(|w| w.set_power(true));
        }
    }

    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::RADIO);
    compiler_fence(Ordering::Acquire);

    if disabled.timed_out() {
        RadioQuiesceResult::TimedOutForcedReset
    } else {
        RadioQuiesceResult::Disabled
    }
}
