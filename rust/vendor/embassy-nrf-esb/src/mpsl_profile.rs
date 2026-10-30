//! Named MPSL coexistence profiles and their derived diagnostic configs.

use crate::error::Error;
use crate::mpsl_radio::RadioRecoveryPolicy;

/// These are starting points for hardware runs, not product guarantees. The
/// explicit config structs can be edited after choosing a profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoexistenceProfile {
    /// Narrow pipe-1 diagnostic used to classify ACK misses.
    DiagnosticPipe1,
    /// Same narrow pipe-1 diagnostic with a longer ACK wait window.
    DiagnosticPipe1RelaxedAck,
    /// Same narrow pipe-1 diagnostic with one in-slot retry after ACK timeout.
    DiagnosticPipe1Retry1,
    /// Same narrow pipe-1 diagnostic with a longer PTX slot and ACK window.
    DiagnosticPipe1LongSlot,
    /// Same narrow pipe-1 diagnostic with PRX-hint phase-locked PTX scheduling.
    DiagnosticPipe1ScheduledGate,
    DiagnosticPipe2ScheduledGate,
    DiagnosticPipe5ScheduledGate,
    /// Same narrow pipe-1 diagnostic with an 8 ms PRX receive window.
    DiagnosticPipe1Prx8ms,
    /// Same narrow pipe-1 diagnostic with a 12 ms PRX receive window.
    DiagnosticPipe1Prx12ms,
    /// Nordic official EXTEND mode: short initial slot + dynamic extension.
    NordicExtend,
    /// 11 ms PRX receive window requested every 12 ms, paired with PTX alignment.
    DiagnosticPipe1Prx11msPaced12ms,
    /// Same narrow pipe-1 diagnostic with a 20 ms PRX receive window.
    DiagnosticPipe1Prx20ms,
    /// Advertising-visible BLE coexistence smoke profile.
    AdvertisingCoexistence,
    /// Active BLE connection profile assuming relaxed connection parameters.
    ConnectedRelaxedBle,
    /// Low-density keyboard traffic starting point for RMK-style smoke tests.
    RmkKeyboardLowLatency,
}

/// BLE-side assumption attached to a coexistence profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BleCoexistenceHint {
    DiagnosticOnly,
    AdvertisingVisible,
    ConnectedRelaxed,
    RmkKeyboardLowLatency,
}

/// Timeslot request policy shared by PRX/PTX configs in a profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimeslotRequestConfig {
    /// Timeout used for earliest timeslot requests.
    pub timeout_us: u32,
    /// Whether BLOCKED/CANCELLED recovery should retry at high priority.
    pub retry_blocked_at_high_priority: bool,
}

impl TimeslotRequestConfig {
    pub const fn diagnostic_default() -> Self {
        Self {
            timeout_us: 1_000_000,
            retry_blocked_at_high_priority: true,
        }
    }

    pub fn validate(self) -> Result<(), Error> {
        if self.timeout_us == 0 {
            return Err(Error::InvalidParam);
        }

        Ok(())
    }
}

/// Configuration for a long-lived PRX timeslot session.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PrxSlotConfig {
    /// Timeslot duration per PRX window.
    pub slot_length_us: u32,
    /// TIMER0 compare value within the slot. Must be less than `slot_length_us`.
    pub in_slot_match_us: u32,
    /// Number of slots per report.
    pub report_every: u32,
    /// Enabled ESB RX pipe mask.
    pub enabled_pipes: u8,
    /// Timeslot request policy.
    pub request: TimeslotRequestConfig,
    /// RADIO recovery policy before handing the peripheral back to MPSL/SDC.
    pub recovery: RadioRecoveryPolicy,
    /// Optional pacing for chained PRX windows.
    pub schedule: PrxScheduleConfig,
    /// Enable Nordic-style dynamic timeslot extension (EXTEND mode).
    pub extend_mode: bool,
    /// Extension window length in microseconds (added per extend).
    pub extend_length_us: u32,
    /// Maximum number of extensions per timeslot before requesting a new one.
    pub max_extends: u32,
}

/// PRX pacing policy for long-lived chained sessions.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PrxScheduleConfig {
    /// Default NORMAL request distance from the previous timeslot start.
    ///
    /// Use 0 to mean `slot_length_us`, i.e. continuous chaining.
    pub normal_distance_us: u32,
    /// Insert a longer request distance after this many granted windows.
    ///
    /// Use 0 to disable planned gaps.
    pub gap_after_windows: u32,
    /// NORMAL request distance used for the planned gap.
    pub gap_distance_us: u32,
}

impl PrxScheduleConfig {
    pub const fn continuous() -> Self {
        Self {
            normal_distance_us: 0,
            gap_after_windows: 0,
            gap_distance_us: 0,
        }
    }

    pub const fn planned_gap(gap_after_windows: u32, gap_distance_us: u32) -> Self {
        Self {
            normal_distance_us: 0,
            gap_after_windows,
            gap_distance_us,
        }
    }

    pub const fn paced(normal_distance_us: u32) -> Self {
        Self {
            normal_distance_us,
            gap_after_windows: 0,
            gap_distance_us: 0,
        }
    }

    pub fn validate(self, slot_length_us: u32) -> Result<(), Error> {
        if self.normal_distance_us != 0 && self.normal_distance_us < slot_length_us {
            return Err(Error::InvalidParam);
        }
        if self.gap_after_windows == 0 {
            if self.gap_distance_us != 0 {
                return Err(Error::InvalidParam);
            }
        } else if self.gap_distance_us < slot_length_us {
            return Err(Error::InvalidParam);
        }

        Ok(())
    }
}

impl PrxSlotConfig {
    pub const fn for_profile(profile: CoexistenceProfile) -> Self {
        CoexistenceProfileConfig::for_profile(profile).prx
    }

    pub fn validate(self) -> Result<(), Error> {
        if self.slot_length_us == 0
            || self.in_slot_match_us == 0
            || self.in_slot_match_us >= self.slot_length_us
            || self.report_every == 0
            || self.enabled_pipes == 0
        {
            return Err(Error::InvalidParam);
        }

        self.request.validate()?;
        self.schedule.validate(self.slot_length_us)?;
        Ok(())
    }
}

/// Configuration for a long-lived PTX poll session.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtxPollConfig {
    /// Timeslot duration per poll.
    pub slot_length_us: u32,
    /// TIMER0 compare value within the slot. Must be less than `slot_length_us`.
    pub in_slot_match_us: u32,
    /// Bitmask of pipes to poll.
    pub pipe_mask: u8,
    /// Number of slots per report.
    pub report_every: u32,
    /// ACK wait window after TX completion.
    pub ack_timeout_us: u32,
    /// In-slot retries after ACK timeout. Use 0 to keep misses visible.
    pub max_retries: u8,
    /// Timeslot request policy.
    pub request: TimeslotRequestConfig,
    /// Optional PTX scheduling policy derived from PRX ACK hints.
    pub schedule_gate: PtxScheduleGateConfig,
}

/// PTX scheduling mode derived from PRX schedule hints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PtxScheduleMode {
    /// Always request the next PTX timeslot as soon as MPSL can provide it.
    Disabled,
    /// Diagnostic-only policy that skips a fixed number of PTX slots after a hint.
    FixedSkipAfterHint,
    /// Lock subsequent PTX timeslots to the hinted PRX period using MPSL NORMAL requests.
    PhaseLocked,
}

/// PTX pacing policy derived from PRX schedule hints.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtxScheduleGateConfig {
    /// Scheduling strategy.
    pub mode: PtxScheduleMode,
    /// Number of PTX timeslots to skip after receiving a valid hint in fixed-skip mode.
    pub skip_after_hint_slots: u8,
    /// Consecutive TX misses tolerated before phase-locked mode falls back to scanning.
    pub lock_miss_limit: u8,
    /// Extra offset from the estimated PRX window start before transmitting.
    pub lock_tx_offset_us: u32,
}

impl PtxScheduleGateConfig {
    pub const fn disabled() -> Self {
        Self {
            mode: PtxScheduleMode::Disabled,
            skip_after_hint_slots: 0,
            lock_miss_limit: 0,
            lock_tx_offset_us: 0,
        }
    }

    pub const fn conservative_pipe1() -> Self {
        Self::phase_locked_pipe1()
    }

    pub const fn fixed_skip_pipe1() -> Self {
        Self {
            mode: PtxScheduleMode::FixedSkipAfterHint,
            skip_after_hint_slots: 2,
            lock_miss_limit: 0,
            lock_tx_offset_us: 0,
        }
    }

    pub const fn phase_locked_pipe1() -> Self {
        Self {
            mode: PtxScheduleMode::PhaseLocked,
            skip_after_hint_slots: 0,
            lock_miss_limit: 4,
            lock_tx_offset_us: 250,
        }
    }

    pub fn validate(self) -> Result<(), Error> {
        match self.mode {
            PtxScheduleMode::Disabled => {}
            PtxScheduleMode::FixedSkipAfterHint => {
                if self.skip_after_hint_slots == 0 {
                    return Err(Error::InvalidParam);
                }
            }
            PtxScheduleMode::PhaseLocked => {
                if self.lock_miss_limit == 0 {
                    return Err(Error::InvalidParam);
                }
            }
        }

        Ok(())
    }
}

impl PtxPollConfig {
    /// Build a poll config from a named coexistence profile.
    pub const fn for_profile(profile: CoexistenceProfile) -> Self {
        CoexistenceProfileConfig::for_profile(profile).ptx
    }

    /// Build a diagnostic poll config with no in-slot retries.
    pub const fn diagnostic(
        slot_length_us: u32,
        in_slot_match_us: u32,
        pipe_mask: u8,
        report_every: u32,
    ) -> Self {
        Self {
            slot_length_us,
            in_slot_match_us,
            pipe_mask,
            report_every,
            ack_timeout_us: 400,
            max_retries: 0,
            request: TimeslotRequestConfig::diagnostic_default(),
            schedule_gate: PtxScheduleGateConfig::disabled(),
        }
    }

    pub const fn pipe_count(self) -> u32 {
        self.pipe_mask.count_ones()
    }

    pub fn validate(self) -> Result<(), Error> {
        if self.slot_length_us == 0
            || self.in_slot_match_us == 0
            || self.in_slot_match_us >= self.slot_length_us
            || self.pipe_mask == 0
            || self.report_every == 0
            || self.ack_timeout_us == 0
        {
            return Err(Error::InvalidParam);
        }

        self.request.validate()?;
        self.schedule_gate.validate()?;
        Ok(())
    }
}

/// Configuration for an event-driven PTX session.
///
/// Unlike `PtxPollConfig`, this does not auto-chain timeslots.
/// Each `send()` call requests one timeslot on demand.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PtxEventConfig {
    /// Timeslot duration per send attempt.
    pub slot_length_us: u32,
    /// TIMER0 compare value within the slot.
    pub in_slot_match_us: u32,
    /// Target ESB pipe for transmissions.
    pub pipe: u8,
    /// ACK wait window after TX completion.
    pub ack_timeout_us: u32,
    /// In-slot retries after ACK timeout.
    pub max_retries: u8,
    /// Timeslot request policy.
    pub request: TimeslotRequestConfig,
}

impl PtxEventConfig {
    pub const fn for_profile(profile: CoexistenceProfile) -> Self {
        CoexistenceProfileConfig::for_profile(profile).ptx_event
    }

    pub fn validate(self) -> Result<(), Error> {
        if self.slot_length_us == 0
            || self.in_slot_match_us == 0
            || self.in_slot_match_us >= self.slot_length_us
            || self.pipe >= 8
            || self.ack_timeout_us == 0
        {
            return Err(Error::InvalidParam);
        }

        self.request.validate()?;
        Ok(())
    }
}

/// Single source for the PRX/PTX/MPSL assumptions attached to a named profile.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoexistenceProfileConfig {
    pub prx: PrxSlotConfig,
    pub ptx: PtxPollConfig,
    pub ptx_event: PtxEventConfig,
    pub ble_hint: BleCoexistenceHint,
}

impl CoexistenceProfileConfig {
    pub const fn for_profile(profile: CoexistenceProfile) -> Self {
        let request = TimeslotRequestConfig::diagnostic_default();
        let recovery = RadioRecoveryPolicy::ForceResetAfterBoundedDisable;
        let prx_schedule = PrxScheduleConfig::continuous();

        let ptx_event = PtxEventConfig {
            slot_length_us: 1500,
            in_slot_match_us: 1300,
            pipe: 1,
            ack_timeout_us: 400,
            max_retries: 0,
            request,
        };

        match profile {
            CoexistenceProfile::DiagnosticPipe1 => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1RelaxedAck => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 600,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event: PtxEventConfig {
                    ack_timeout_us: 600,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1Retry1 => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 1,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event: PtxEventConfig {
                    max_retries: 1,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1LongSlot => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 600,
                    max_retries: 1,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event: PtxEventConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    ack_timeout_us: 600,
                    max_retries: 1,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1ScheduledGate => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::phase_locked_pipe1(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe2ScheduledGate => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x06,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x06,
                    report_every: 2,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::phase_locked_pipe1(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe5ScheduledGate => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x3E,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 2500,
                    in_slot_match_us: 2300,
                    pipe_mask: 0x3E,
                    report_every: 5,
                    ack_timeout_us: 400,
                    max_retries: 1,
                    request,
                    schedule_gate: PtxScheduleGateConfig::phase_locked_pipe1(),
                },
                ptx_event: PtxEventConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    ack_timeout_us: 600,
                    max_retries: 1,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1Prx8ms => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 8000,
                    in_slot_match_us: 7600,
                    report_every: 12,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1Prx12ms => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 11_000,
                    in_slot_match_us: 10_500,
                    report_every: 8,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::NordicExtend => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 530,
                    report_every: 8,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: true,
                    extend_length_us: 530,
                    max_extends: 200,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1Prx11msPaced12ms => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 11_000,
                    in_slot_match_us: 10_500,
                    report_every: 8,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: PrxScheduleConfig::paced(12_000),
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::DiagnosticPipe1Prx20ms => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 20_000,
                    in_slot_match_us: 19_000,
                    report_every: 5,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::DiagnosticOnly,
            },
            CoexistenceProfile::AdvertisingCoexistence => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 12_000,
                    in_slot_match_us: 11_500,
                    report_every: 50,
                    enabled_pipes: 0x06,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    pipe_mask: 0x06,
                    report_every: 2,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event: PtxEventConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::AdvertisingVisible,
            },
            CoexistenceProfile::ConnectedRelaxedBle => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 5000,
                    in_slot_match_us: 4500,
                    report_every: 20,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 1500,
                    in_slot_match_us: 1300,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 0,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event,
                ble_hint: BleCoexistenceHint::ConnectedRelaxed,
            },
            CoexistenceProfile::RmkKeyboardLowLatency => Self {
                prx: PrxSlotConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    report_every: 4,
                    enabled_pipes: 0x02,
                    request,
                    recovery,
                    schedule: prx_schedule,
                    extend_mode: false,
                    extend_length_us: 0,
                    max_extends: 0,
                },
                ptx: PtxPollConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    pipe_mask: 0x02,
                    report_every: 1,
                    ack_timeout_us: 400,
                    max_retries: 1,
                    request,
                    schedule_gate: PtxScheduleGateConfig::disabled(),
                },
                ptx_event: PtxEventConfig {
                    slot_length_us: 3000,
                    in_slot_match_us: 2800,
                    max_retries: 1,
                    ..ptx_event
                },
                ble_hint: BleCoexistenceHint::RmkKeyboardLowLatency,
            },
        }
    }

    pub fn validate(self) -> Result<(), Error> {
        self.prx.validate()?;
        self.ptx.validate()?;
        Ok(())
    }
}
