//! Bounded, allocation-free diagnostics in the shared TIMER0 microsecond domain.

use crate::receiver::ReceiverSnapshot;
use core::fmt::{self, Write};
use nrf_embassy::ws::{SampleResult, WsSampler, WsStats};

pub const PACKET_SIZE: usize = 64;
// RF counters and optional 64-bit records exceed the previous 512-byte bound.
pub const LINE_SIZE: usize = 896;
const GENERATED_IDENTITY: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/usb-identity.txt"));
const COMPLETION_POLICY_FIELD: &[u8] = if cfg!(feature = "guarded-completion-retention") {
    b" completion_policy=RetainGuardedCompletion\n"
} else {
    b" completion_policy=StrictSoftwareDeadline\n"
};
const IDENTITY_SIZE: usize = GENERATED_IDENTITY.len() - 1 + COMPLETION_POLICY_FIELD.len();
const _: () = assert!(IDENTITY_SIZE <= 512);
/// Executing-image policy is static and independent of trace and TDMA features.
/// The existing USB writer sends these same bounded bytes on each identity row.
pub const IDENTITY: &[u8] = &configured_identity();

const fn configured_identity() -> [u8; IDENTITY_SIZE] {
    assert!(GENERATED_IDENTITY[GENERATED_IDENTITY.len() - 1] == b'\n');
    let mut bytes = [0; IDENTITY_SIZE];
    let mut index = 0;
    while index < GENERATED_IDENTITY.len() - 1 {
        bytes[index] = GENERATED_IDENTITY[index];
        index += 1;
    }
    let mut field_index = 0;
    while field_index < COMPLETION_POLICY_FIELD.len() {
        bytes[index] = COMPLETION_POLICY_FIELD[field_index];
        index += 1;
        field_index += 1;
    }
    bytes
}
pub const ACTOR_PERIOD_US: u64 = 1_000_000;

#[cfg(feature = "pipeline-trace")]
#[derive(Clone, Copy, Debug, Default)]
pub struct AudioDiagnostic {
    pub counters: nrf_embassy::protocol_audio::AudioCounters,
    pub local_tx_depth: u8,
    pub ingress_depth: u8,
}

#[cfg(feature = "pipeline-trace")]
impl AudioDiagnostic {
    pub fn capture(runtime: &nrf_embassy::runtime::Runtime) -> Self {
        let audio = runtime.audio_state();
        Self {
            counters: audio.counters(),
            local_tx_depth: audio.tx_depth(),
            ingress_depth: audio.ingress_used(),
        }
    }
}

#[cfg(feature = "pipeline-trace")]
#[derive(Clone, Copy, Debug)]
pub enum PipelineRow {
    AudioIn,
    AudioTx,
    AudioRx,
    Bridge,
    #[cfg(feature = "slot-entry-trace")]
    SlotEntry,
    #[cfg(feature = "slot-entry-trace")]
    FrameEntry,
}

#[cfg(feature = "pipeline-trace")]
impl PipelineRow {
    pub const ALL: [Self; 6] = [
        Self::AudioIn,
        Self::AudioTx,
        Self::AudioRx,
        Self::Bridge,
        Self::SlotEntry,
        Self::FrameEntry,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::AudioIn => "PIPE_AUDIO_IN",
            Self::AudioTx => "PIPE_AUDIO_TX",
            Self::AudioRx => "PIPE_AUDIO_RX",
            Self::Bridge => "PIPE_BRIDGE",
            #[cfg(feature = "slot-entry-trace")]
            Self::SlotEntry => "PIPE_SLOT_ENTRY",
            #[cfg(feature = "slot-entry-trace")]
            Self::FrameEntry => "PIPE_FRAME_ENTRY",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RadioSnapshot {
    pub wanted: bool,
    pub min_epoch: u64,
    pub current_session: Option<u64>,
    pub channel: u8,
    pub tx: crate::actor_policy::TxCounters,
}

#[derive(Clone, Copy, Debug)]
pub struct ActorSnapshot {
    pub sequence: u32,
    pub clock_us: u64,
    pub runtime: nrf_embassy::runtime::Snapshot,
    pub reported_peers: u8,
    pub capability: (bool, bool, bool),
    pub admission: nrf_embassy::protocol_audio::IngressAdmissionGate,
    pub radio: RadioSnapshot,
    #[cfg(feature = "slot-entry-trace")]
    pub slot_entry: crate::slot_entry_trace::Stats,
    #[cfg(feature = "pipeline-trace")]
    pub audio_diagnostic: AudioDiagnostic,
    #[cfg(feature = "sync-trace")]
    pub tx_diagnostics: crate::tx_diagnostics::Stats,
}

impl ActorSnapshot {
    pub fn age_us(self, now: u64) -> Option<u64> {
        now.checked_sub(self.clock_us)
    }
    pub fn stale(self, now: u64) -> bool {
        self.age_us(now).is_none_or(|age| age > ACTOR_PERIOD_US)
    }
}

/// DEVICEID[0], then DEVICEID[1], each as eight uppercase hex digits.
pub fn serial_number(deviceid0: u32, deviceid1: u32) -> [u8; 16] {
    let mut serial = [0; 16];
    for (word, dst) in [deviceid0, deviceid1].into_iter().zip(serial.chunks_mut(8)) {
        for (index, digit) in dst.iter_mut().enumerate() {
            *digit = b"0123456789ABCDEF"[((word >> (28 - index * 4)) & 15) as usize];
        }
    }
    serial
}

#[derive(Default)]
pub struct FrameScheduler {
    last: Option<u32>,
}

impl FrameScheduler {
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// Called immediately before capture, with no intervening await.
    pub fn new_frame(&mut self, clock_us: u64) -> Option<u32> {
        let frame = (clock_us / 20_000) as u32;
        if self.last == Some(frame) {
            return None;
        }
        self.last = Some(frame);
        Some(frame)
    }
}

/// Does not apply correction to a timer, radio deadline or runtime state.
pub fn sample_diagnostic(sampler: &mut WsSampler, count: u32, frame: u32) -> SampleResult {
    sampler.sample(count, frame)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TimingSnapshot {
    pub clock_us: u64,
    pub frame: u32,
    pub jitter_max_us: u32,
    pub skipped: u32,
    pub correction_pending_us: i32,
    /// 0 healthy; 1 shared clock fault, 2 deadline range, 3 epoch exhaustion, 4 core error.
    pub fault: u32,
    pub ws_frame: u32,
    pub ws_count: u32,
    pub ws: WsStats,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DiagnosticSnapshot {
    /// Same shared TIMER0 epoch as TDMA; snapshot times can differ by age.
    pub clock_us: u64,
    pub polls: u32,
    pub spi_ok: u32,
    pub spi_fail: u32,
    pub tdma_clock_us: u64,
    pub tdma_frame: u32,
    pub tdma_jitter_max_us: u32,
    pub tdma_skipped: u32,
    pub tdma_correction_pending_us: i32,
    pub tdma_fault: u32,
    pub ws_frame: u32,
    pub ws_count: u32,
    pub ws: WsStats,
    pub drops: u32,
    pub receiver: ReceiverSnapshot,
    pub actor: Option<ActorSnapshot>,
    pub software_capability: bool,
    pub ingress: Option<crate::transport::SharedInboxSnapshot>,
    pub rpc: Option<crate::bridge_rpc::Snapshot>,
    #[cfg(feature = "rx-blocking-trace")]
    pub rx_blocking: crate::rx_blocking_trace::Publication,
    #[cfg(feature = "pipeline-trace")]
    pub bridge: nrf_embassy::bridge::Snapshot,
    #[cfg(feature = "pipeline-trace")]
    pub bridge_clock_us: u64,
}

struct Optional<T>(Option<T>);

#[cfg(feature = "sync-trace")]
struct OptionalScalar<T>(Option<T>);

#[cfg(feature = "sync-trace")]
impl<T: fmt::Display> fmt::Display for OptionalScalar<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(value) => value.fmt(f),
            None => f.write_str("NA"),
        }
    }
}

impl<T: fmt::Display> fmt::Display for Optional<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(value) => value.fmt(f),
            None => f.write_str("none"),
        }
    }
}

pub struct DiagnosticLine {
    bytes: [u8; LINE_SIZE],
    len: usize,
}

impl DiagnosticLine {
    #[cfg(feature = "reset-diagnostics")]
    pub fn reset(s: crate::reset_diagnostics::Snapshot) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        let bc = s.breadcrumb.unwrap_or_default();
        let kind = match bc.kind {
            1 => "panic",
            2 => "hardfault",
            _ => "none",
        };
        writeln!(
            line,
            "PIPE_RESET v=1 resetreas=0x{:08x} boots={} bc_valid={} bc_kind={} bc_line={} bc_col={} bc_pc=0x{:08x} bc_lr=0x{:08x} bc_boot={}",
            s.resetreas,
            s.boots,
            u8::from(s.breadcrumb.is_some()),
            kind,
            bc.line,
            bc.column,
            bc.pc,
            bc.lr,
            bc.boot
        )?;
        Ok(line)
    }

    #[cfg(feature = "rx-blocking-trace")]
    pub fn bridge_cs(
        s: crate::rx_blocking_trace::Publication,
        op: crate::rx_blocking_trace::Operation,
    ) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        write!(
            line,
            "{} v=1 seq={} clock_us={} available={} op={}",
            if cfg!(feature = "unmasked-bridge-owner") {
                "PIPE_BRIDGE_OP"
            } else {
                "PIPE_BRIDGE_CS"
            },
            s.sequence,
            Optional(s.clock_us),
            s.bridge_available,
            op as usize
        )?;
        #[cfg(feature = "unmasked-bridge-owner")]
        write!(line, " irq_masked=0")?;
        if s.bridge_available {
            let h = s.bridge[op as usize];
            write!(line, " hist=")?;
            for (i, count) in h.bins.into_iter().enumerate() {
                if i != 0 {
                    write!(line, ",")?;
                }
                write!(line, "{count}")?;
            }
            write!(
                line,
                " count={} max_us={} errors={}",
                h.count, h.max_us, h.errors
            )?;
        }
        writeln!(line)?;
        Ok(line)
    }

    #[cfg(feature = "rx-blocking-trace")]
    pub fn radio_rx(s: crate::rx_blocking_trace::Publication) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        write!(
            line,
            "PIPE_RADIO_RX v=1 seq={} clock_us={} radio_clock_us={} available={}",
            s.sequence,
            Optional(s.clock_us),
            Optional(s.radio_clock_us),
            s.radio.is_some()
        )?;
        if let Some(
            [
                bad_crc,
                duplicate,
                start_alloc_failed,
                post_delivery_starved,
                queue_rejected,
            ],
        ) = s.radio
        {
            write!(
                line,
                " bad_crc={bad_crc} duplicate={duplicate} start_alloc_failed={start_alloc_failed} post_delivery_starved={post_delivery_starved} queue_rejected={queue_rejected}"
            )?;
        }
        writeln!(line)?;
        Ok(line)
    }

    /// Owned publications only: the USB writer never loans actor/bridge state.
    #[cfg(feature = "pipeline-trace")]
    pub fn pipeline(s: DiagnosticSnapshot, row: PipelineRow) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        write!(
            line,
            "{} v=1 spi_poll_id={} spi_clock_us={}",
            row.name(),
            s.polls,
            s.clock_us
        )?;
        if matches!(row, PipelineRow::Bridge) {
            let b = s.bridge;
            writeln!(
                line,
                " bridge_clock_us={} audio_head={} audio_tail={} control_head={} control_tail={} delivery_enabled={} tx_seq={} rx_seq={} rx_seq_initialized={} last_audio_seq={} last_audio_seq_valid={} audio_overwrite={} control_overwrite={} ingress_ok={} ingress_reject={} ingress_duplicate={} tx_audio={} tx_control={} tx_idle={} tx_fail={} status_enqueue={} status_tx={} status_drop={} rx_seq_gap={} rx_crc_fail={} ack_pulses={} attempts={} keepalive_phase={}",
                s.bridge_clock_us,
                b.audio_head,
                b.audio_tail,
                b.control_head,
                b.control_tail,
                b.delivery_enabled,
                b.tx_seq,
                b.rx_seq,
                b.rx_seq_initialized,
                b.last_audio_seq,
                b.last_audio_seq_valid,
                b.audio_overwrite,
                b.control_overwrite,
                b.ingress_ok,
                b.ingress_reject,
                b.ingress_duplicate,
                b.tx_audio,
                b.tx_control,
                b.tx_idle,
                b.tx_fail,
                b.status_enqueue,
                b.status_tx,
                b.status_drop,
                b.rx_seq_gap,
                b.rx_crc_fail,
                b.ack_pulses,
                b.attempts,
                b.keepalive
            )?;
            return Ok(line);
        }
        let Some(a) = s.actor else {
            writeln!(line, " actor=unavailable")?;
            return Ok(line);
        };
        write!(
            line,
            " actor_seq={} actor_clock_us={} actor_age_us={} actor_stale={}",
            a.sequence,
            a.clock_us,
            Optional(a.age_us(s.clock_us)),
            a.stale(s.clock_us)
        )?;
        let c = a.audio_diagnostic.counters;
        match row {
            PipelineRow::AudioIn => writeln!(
                line,
                " ingress_depth={} admission={} admission_epoch={} local_audio={} local_epoch={} role={} stored_peers={} ingress_purge_drop={} spi_audio_in={} ingress_inactive_drop={} ingress_msgq_drop={} spi_in_frames={} spi_in_gap_evt={} spi_in_gap_fr={} spi_in_reset_evt={}",
                a.audio_diagnostic.ingress_depth,
                a.admission.enabled,
                a.admission.epoch,
                a.runtime.audio.enabled,
                a.runtime.audio.epoch,
                a.runtime.membership.role as u8,
                a.runtime.membership.peer_count,
                c.ingress_purge_drop,
                c.spi_audio_in,
                c.ingress_inactive_drop,
                c.ingress_msgq_drop,
                c.spi_in_frames,
                c.spi_in_gap_evt,
                c.spi_in_gap_fr,
                c.spi_in_reset_evt
            )?,
            PipelineRow::AudioTx => {
                write!(
                    line,
                    " local_tx_depth={} relay_depth=unavailable tx_purge_drop={} tx_count={} tx_starvation={} tx_queue_drain={} audio_fwd={} tx_overwrite={} tx_ring_drop={} relay_ring_drop={} rf_audio_try={} rf_audio_ok={} rf_audio_fail={} bundle_tx={} prev1_stripped={} prev2_stripped={} bundle_late_drop={} bundle_max_bytes={} local_deferred_recovery={} rf_tx_frames={}",
                    a.audio_diagnostic.local_tx_depth,
                    c.tx_purge_drop,
                    c.tx_count,
                    c.tx_starvation,
                    c.tx_queue_drain,
                    c.audio_fwd,
                    c.tx_overwrite,
                    c.tx_ring_drop,
                    c.relay_ring_drop,
                    c.rf_audio_try,
                    c.rf_audio_ok,
                    c.rf_audio_fail,
                    c.bundle_tx,
                    c.prev1_stripped,
                    c.prev2_stripped,
                    c.bundle_late_drop,
                    c.bundle_max_bytes,
                    c.local_deferred_recovery,
                    c.rf_tx_frames
                )?;
                #[cfg(feature = "pipeline-trace")]
                write!(
                    line,
                    " local_prefit_no_time={} local_prefit_consumed={} local_entries_consumed={}",
                    c.local_prefit_no_time, c.local_prefit_consumed, c.local_entries_consumed
                )?;
                #[cfg(feature = "same-slot-audio-refit")]
                write!(
                    line,
                    " same_slot_refit_attempt={} same_slot_refit_complete={}",
                    c.same_slot_refit_attempt, c.same_slot_refit_complete
                )?;
                writeln!(line)?;
            }
            PipelineRow::AudioRx => writeln!(
                line,
                " rf_rx_audio_ok={} rf_rx_malformed={} rf_rx_version_drop={} rf_rx_self_drop={} rf_rx_duplicate_drop={} rf_rx_inactive_drop={} spi_out_ok={} spi_out_drop={} bundle_rx={} bundle_bad={} prev1_forwarded={} prev2_forwarded={} rf_rx_frames={} rf_rx_gap_evt={} rf_rx_gap_fr={} rf_rx_reset_evt={} spi_out_frames={}",
                c.rf_rx_audio_ok,
                c.rf_rx_malformed,
                c.rf_rx_version_drop,
                c.rf_rx_self_drop,
                c.rf_rx_duplicate_drop,
                c.rf_rx_inactive_drop,
                c.spi_out_ok,
                c.spi_out_drop,
                c.bundle_rx,
                c.bundle_bad,
                c.prev1_forwarded,
                c.prev2_forwarded,
                c.rf_rx_frames,
                c.rf_rx_gap_evt,
                c.rf_rx_gap_fr,
                c.rf_rx_reset_evt,
                c.spi_out_frames
            )?,
            PipelineRow::Bridge => unreachable!(),
            #[cfg(feature = "slot-entry-trace")]
            PipelineRow::SlotEntry => {
                use crate::slot_entry_trace::Csv;
                let t = a.slot_entry;
                writeln!(
                    line,
                    " slot_phase={} frame_phase={} slot_rpc={} frame_rpc={} entry0={} entryn={} irq_service={}",
                    Csv(&t.slot_alarm_phase),
                    Csv(&t.frame_alarm_phase),
                    Csv(&t.slot_alarm_rpc),
                    Csv(&t.frame_alarm_rpc),
                    t.entry0,
                    t.entryn,
                    t.irq_service
                )?;
            }
            #[cfg(feature = "slot-entry-trace")]
            PipelineRow::FrameEntry => {
                let t = a.slot_entry;
                use crate::slot_entry_trace::Csv;
                writeln!(
                    line,
                    " frame_dispatch={} frame_drain={} late_phase={} late_rpc={} budget_short={} budget_len={}",
                    t.frame_dispatch,
                    t.frame_drain,
                    Csv(&t.late_alarm_phase),
                    Csv(&t.late_alarm_rpc),
                    t.budget_short,
                    Csv(&t.budget_reject_len)
                )?;
            }
        }
        Ok(line)
    }

    pub fn from_snapshot(s: DiagnosticSnapshot) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        writeln!(
            line,
            "clock_us={} polls={} spi_ok={} spi_fail={} tdma_clock_us={} tdma_frame={} jitterMax={} skipped={} correctionPending={} fault={} ws_frame={} ws_count={} edges={} sample={} valid={} reject={} noSignal={} delta={} corr={} drift={} drop={} RF_IRQ={} receivedPackets={} bytes={} missingMetadata={} missingClock={} clockErrors={} lastChannel={} lastEpoch={} lastTimestamp={} rx_age={} radio=actor-owned-unqualified RFqualified=false",
            s.clock_us,
            s.polls,
            s.spi_ok,
            s.spi_fail,
            s.tdma_clock_us,
            s.tdma_frame,
            s.tdma_jitter_max_us,
            s.tdma_skipped,
            s.tdma_correction_pending_us,
            s.tdma_fault,
            s.ws_frame,
            s.ws_count,
            s.ws.total_edges,
            s.ws.sample_count,
            s.ws.valid_count,
            s.ws.rejected_count,
            s.ws.no_signal_count,
            s.ws.last_delta_edges,
            s.ws.last_correction_us,
            s.ws.cumulative_drift_us,
            s.drops,
            s.receiver.irq_count,
            s.receiver.received_packets,
            s.receiver.bytes,
            s.receiver.missing_metadata,
            s.receiver.missing_clock,
            s.receiver.clock_errors,
            Optional(s.receiver.last.map(|last| last.channel)),
            Optional(s.receiver.last.map(|last| last.epoch)),
            Optional(s.receiver.last.and_then(|last| last.timestamp_us)),
            Optional(s.receiver.age_us(s.clock_us)),
        )?;
        Ok(line)
    }

    pub fn actor_status(s: DiagnosticSnapshot) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        write!(
            line,
            "software_capability={} RFqualified=false ",
            u8::from(s.software_capability)
        )?;
        if let Some(a) = s.actor {
            let m = a.runtime.membership;
            writeln!(
                line,
                "actor=initialized actor_seq={} actor_clock_us={} actor_age_us={} actor_stale={} state={} role={} node={} slot={} coordinator={} stored_peers={} reported_peers={} group={} strict_init={} external_ingress={} gate_published={} local_audio={} local_epoch={} admission={} admission_epoch={} mv3_applied={} mv3_removed_local={} mv3_rej_source={} mv3_rej_term={} mv3_rej_revision={} mv3_rej_identity={} mv3_rej_other={}",
                a.sequence,
                a.clock_us,
                Optional(a.age_us(s.clock_us)),
                a.stale(s.clock_us),
                m.state as u8,
                m.role as u8,
                m.node_id,
                m.slot_index,
                m.coordinator_id,
                m.peer_count,
                a.reported_peers,
                a.runtime.talk_channel,
                a.capability.0,
                a.capability.1,
                a.capability.2,
                a.runtime.audio.enabled,
                a.runtime.audio.epoch,
                a.admission.enabled,
                a.admission.epoch,
                a.runtime.membership_rx.applied,
                a.runtime.membership_rx.removed_local,
                a.runtime.membership_rx.source,
                a.runtime.membership_rx.term,
                a.runtime.membership_rx.revision,
                a.runtime.membership_rx.identity,
                a.runtime.membership_rx.other
            )?;
        } else {
            writeln!(line, "actor=unavailable")?;
        }
        Ok(line)
    }

    pub fn radio_status(s: DiagnosticSnapshot) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        let r = s.actor.map(|a| a.radio);
        writeln!(
            line,
            "RFwanted={} minEpoch={} currentSession={} channel={} TXattempted={} PhysicalComplete={} TXdeadline={} TXfault={} ingress_admitted=none ingress_overwritten_delta={} ingress_purged_delta={} ingress_invalid_delta={} ingress_queued={} published_admission={} published_epoch={} rpc_phase={} rpc_initialized={} rpc_exhausted={} rpc_abandoned={}",
            Optional(r.map(|r| r.wanted)),
            Optional(r.map(|r| r.min_epoch)),
            Optional(r.and_then(|r| r.current_session)),
            Optional(r.map(|r| r.channel)),
            Optional(r.map(|r| r.tx.attempted)),
            Optional(r.map(|r| r.tx.physical_complete)),
            Optional(r.map(|r| r.tx.deadline)),
            Optional(r.map(|r| r.tx.fault)),
            Optional(s.ingress.map(|i| i.accounting.ingress.overwritten)),
            Optional(s.ingress.map(|i| i.accounting.ingress.purged)),
            Optional(s.ingress.map(|i| i.accounting.bundle_bad)),
            Optional(s.ingress.map(|i| i.queued)),
            Optional(s.ingress.map(|i| i.gate.enabled)),
            Optional(s.ingress.map(|i| i.gate.epoch)),
            Optional(s.rpc.map(|r| match r.phase {
                crate::bridge_rpc::Phase::Idle => "idle",
                crate::bridge_rpc::Phase::Requested => "requested",
                crate::bridge_rpc::Phase::Claimed => "claimed",
                crate::bridge_rpc::Phase::Completed => "completed",
            })),
            Optional(s.rpc.map(|r| r.initialized)),
            Optional(s.rpc.map(|r| r.exhausted)),
            Optional(s.rpc.map(|r| r.abandoned))
        )?;
        Ok(line)
    }

    #[cfg(feature = "sync-trace")]
    pub fn tx_class_status(
        a: ActorSnapshot,
        class: crate::tx_diagnostics::WireClass,
    ) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        let row = a.tx_diagnostics.rows[class as usize];
        writeln!(
            line,
            "TXclass={} actor_seq={} actor_clock_us={} counter_saturated={} requests={} backend_window_rejected={} validation_rejected={} budget_rejected={} submission_rejected={} accepted_tickets={} complete={} guard_deadline={} software_timed_out={} cancelled={} other={}",
            class.name(),
            a.sequence,
            a.clock_us,
            a.tx_diagnostics.counter_saturated,
            row.requests,
            row.backend_window_rejected,
            row.validation_rejected,
            row.budget_rejected,
            row.submission_rejected,
            row.accepted_tickets,
            row.complete,
            row.guard_deadline,
            row.software_timed_out,
            row.cancelled,
            row.other,
        )?;
        Ok(line)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    #[cfg(feature = "sync-trace")]
    pub fn tx_admission_status(
        a: ActorSnapshot,
        class: crate::tx_diagnostics::WireClass,
    ) -> Result<Self, fmt::Error> {
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        write!(
            line,
            "TXADMIT={} actor_seq={} actor_clock_us={} counter_saturated={} backend_window_rejected={}",
            class.name(),
            a.sequence,
            a.clock_us,
            a.tx_diagnostics.counter_saturated,
            a.tx_diagnostics.rows[class as usize].backend_window_rejected,
        )?;
        // r0..r33 use AdmissionRejection::index; Display on TXWINDOW is the legend.
        for (index, count) in a.tx_diagnostics.admission_reasons[class as usize]
            .iter()
            .enumerate()
        {
            write!(line, " r{index}={count}")?;
        }
        writeln!(line)?;
        Ok(line)
    }

    #[cfg(feature = "sync-trace")]
    pub fn tx_window_status(
        a: ActorSnapshot,
        class: crate::tx_diagnostics::WireClass,
    ) -> Result<Option<Self>, fmt::Error> {
        let Some(w) = a.tx_diagnostics.last_admission_rejection[class as usize] else {
            return Ok(None);
        };
        let mut line = Self {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        // actor_seq describes publication; ordinal/physical_now describe rejection.
        writeln!(
            line,
            "TXWINDOW={} actor_seq={} ordinal={} r={} reason={} kind={:?} len={} profile={:?} anchor_p={} anchor_t={} physical_now={} timing_now={} window_start={} window_end={} budget={} saved_generation={} current_generation={} snapshot_generation={} saved_frame={} current_work_frame={} snapshot_frame={} saved_start={} saved_deadline={} current_start={} current_deadline={} frame_start={} running={} synchronized={} slot={}",
            class.name(),
            a.sequence,
            w.ordinal,
            w.reason.index(),
            w.reason,
            w.kind,
            w.packet_len,
            w.mapping.profile(),
            w.mapping.physical_anchor().value(),
            w.mapping.timing_anchor().value(),
            w.physical_now,
            OptionalScalar(w.timing_now),
            OptionalScalar(w.physical_window_start),
            OptionalScalar(w.physical_window_deadline),
            OptionalScalar(w.budget_us),
            w.saved.generation,
            w.current.generation,
            w.snapshot.generation,
            w.saved.frame,
            w.current.frame,
            w.snapshot.frame,
            w.saved.start_us,
            w.saved.deadline_us,
            w.current.start_us,
            w.current.deadline_us,
            w.snapshot.frame_start,
            w.snapshot.running,
            w.snapshot.synchronized,
            w.snapshot.slot_index,
        )?;
        Ok(Some(line))
    }
}

impl Write for DiagnosticLine {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if !text.is_ascii() || text.len() > LINE_SIZE - self.len {
            return Err(fmt::Error);
        }
        let end = self.len + text.len();
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// Packet slices followed by a ZLP when the final data packet is full-sized.
pub fn packets(bytes: &[u8]) -> impl Iterator<Item = &[u8]> {
    bytes.chunks(PACKET_SIZE).chain(
        core::iter::once(&bytes[..0]).take(usize::from(bytes.len().is_multiple_of(PACKET_SIZE))),
    )
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[cfg(feature = "rx-blocking-trace")]
    #[test]
    fn rx_blocking_distinct_fields_unavailable_and_maximum_rows() {
        use crate::rx_blocking_trace::{Histogram, Operation, Publication};
        let mut s = Publication {
            sequence: u32::MAX,
            clock_us: Some(i64::MAX),
            radio_clock_us: Some(17),
            bridge_available: true,
            bridge: core::array::from_fn(|i| Histogram {
                bins: core::array::from_fn(|b| (100 * i + b) as u32),
                count: 81,
                max_us: 82,
                errors: 83,
            }),
            radio: Some([11, 12, 13, 14, 15]),
        };
        for op in Operation::ALL {
            let line = DiagnosticLine::bridge_cs(s, op).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(
                text.starts_with(if cfg!(feature = "unmasked-bridge-owner") {
                    "PIPE_BRIDGE_OP v=1 "
                } else {
                    "PIPE_BRIDGE_CS v=1 "
                })
            );
            assert_eq!(
                text.contains(" irq_masked=0"),
                cfg!(feature = "unmasked-bridge-owner")
            );
            assert!(text.contains(&std::format!(
                "op={}{} hist={}",
                op as usize,
                if cfg!(feature = "unmasked-bridge-owner") {
                    " irq_masked=0"
                } else {
                    ""
                },
                op as usize * 100
            )));
            assert!(text.ends_with(" count=81 max_us=82 errors=83\n"));
        }
        let line = DiagnosticLine::radio_rx(s).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("radio_clock_us=17 available=true bad_crc=11 duplicate=12 start_alloc_failed=13 post_delivery_starved=14 queue_rejected=15"));
        s.radio = Some([u32::MAX; 5]);
        s.radio_clock_us = Some(i64::MAX);
        s.bridge.fill(Histogram {
            bins: [u32::MAX; 8],
            count: u32::MAX,
            max_us: u64::MAX,
            errors: u32::MAX,
        });
        for op in Operation::ALL {
            assert!(DiagnosticLine::bridge_cs(s, op).unwrap().as_bytes().len() <= 896);
        }
        assert!(DiagnosticLine::radio_rx(s).unwrap().as_bytes().len() <= 896);
        s.bridge_available = false;
        s.radio = None;
        s.clock_us = None;
        for line in [
            DiagnosticLine::bridge_cs(s, Operation::Begin).unwrap(),
            DiagnosticLine::radio_rx(s).unwrap(),
        ] {
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("clock_us=none") && text.contains("available=false"));
            assert!(!text.contains("hist=") && !text.contains("bad_crc="));
        }
    }

    #[test]
    fn unmasked_bridge_metadata_is_present_only_for_opt_in() {
        let metadata = include_str!(concat!(env!("OUT_DIR"), "/integration-metadata.txt"));
        assert_eq!(
            metadata
                .lines()
                .filter(|row| *row == "unmasked_bridge_owner=UnmaskedBridgeOwnerExperimental")
                .count(),
            usize::from(cfg!(feature = "unmasked-bridge-owner"))
        );
        assert_eq!(
            metadata.contains("unmasked_bridge_owner="),
            cfg!(feature = "unmasked-bridge-owner")
        );
    }

    #[test]
    fn nonblocking_bridge_metadata_is_present_only_for_opt_in() {
        let metadata = include_str!(concat!(env!("OUT_DIR"), "/integration-metadata.txt"));
        assert_eq!(
            metadata
                .lines()
                .filter(|row| *row
                    == "nonblocking_bridge_delivery=NonblockingBridgeDeliveryExperimental")
                .count(),
            usize::from(cfg!(feature = "nonblocking-bridge-delivery"))
        );
        assert_eq!(
            metadata.contains("nonblocking_bridge_delivery="),
            cfg!(feature = "nonblocking-bridge-delivery")
        );
    }

    #[test]
    fn same_slot_refit_metadata_is_present_only_for_opt_in() {
        let metadata = include_str!(concat!(env!("OUT_DIR"), "/integration-metadata.txt"));
        assert_eq!(
            metadata
                .lines()
                .filter(|row| *row == "same_slot_audio_refit=SameSlotAudioRefitExperimental")
                .count(),
            usize::from(cfg!(feature = "same-slot-audio-refit"))
        );
        assert_eq!(
            metadata.contains("same_slot_audio_refit="),
            cfg!(feature = "same-slot-audio-refit")
        );
    }

    #[cfg(feature = "pipeline-trace")]
    fn pipeline_fixture(maximum: bool) -> DiagnosticSnapshot {
        use nrf_embassy::protocol_audio::AudioCounters;
        let value = |n| if maximum { u32::MAX } else { n };
        let counters = AudioCounters {
            ingress_purge_drop: value(1),
            tx_purge_drop: value(2),
            tx_count: value(3),
            tx_starvation: value(4),
            tx_queue_drain: value(5),
            audio_fwd: value(6),
            tx_overwrite: value(7),
            spi_audio_in: value(8),
            ingress_inactive_drop: value(9),
            ingress_msgq_drop: value(10),
            tx_ring_drop: value(11),
            relay_ring_drop: value(12),
            rf_audio_try: value(13),
            rf_audio_ok: value(14),
            rf_audio_fail: value(15),
            rf_rx_audio_ok: value(16),
            rf_rx_malformed: value(17),
            rf_rx_version_drop: value(18),
            rf_rx_self_drop: value(19),
            rf_rx_duplicate_drop: value(20),
            rf_rx_inactive_drop: value(21),
            spi_out_ok: value(22),
            spi_out_drop: value(23),
            bundle_tx: value(24),
            bundle_rx: value(25),
            bundle_bad: value(26),
            prev1_forwarded: value(27),
            prev2_forwarded: value(28),
            prev1_stripped: value(29),
            prev2_stripped: value(30),
            bundle_late_drop: value(31),
            bundle_max_bytes: value(32),
            local_deferred_recovery: value(33),
            spi_in_frames: value(34),
            spi_in_gap_evt: value(35),
            spi_in_gap_fr: value(36),
            spi_in_reset_evt: value(37),
            rf_tx_frames: value(38),
            rf_rx_frames: value(39),
            rf_rx_gap_evt: value(40),
            rf_rx_gap_fr: value(41),
            rf_rx_reset_evt: value(42),
            spi_out_frames: value(43),
            local_prefit_no_time: value(46),
            local_prefit_consumed: value(47),
            local_entries_consumed: value(48),
            #[cfg(feature = "same-slot-audio-refit")]
            same_slot_refit_attempt: value(44),
            #[cfg(feature = "same-slot-audio-refit")]
            same_slot_refit_complete: value(45),
        };
        let mut runtime = nrf_embassy::runtime::Runtime::default().snapshot();
        runtime.audio.epoch = u32::MAX;
        runtime.membership.role = nrf_embassy::runtime::Role::Participant;
        runtime.membership.peer_count = u8::MAX;
        let actor = ActorSnapshot {
            sequence: u32::MAX,
            clock_us: 7,
            runtime,
            reported_peers: u8::MAX,
            capability: (false, false, false),
            admission: nrf_embassy::protocol_audio::IngressAdmissionGate {
                enabled: true,
                epoch: u32::MAX,
            },
            radio: RadioSnapshot {
                wanted: false,
                min_epoch: 0,
                current_session: None,
                channel: 0,
                tx: crate::actor_policy::TxCounters::default(),
            },
            tx_diagnostics: crate::tx_diagnostics::Stats::default(),
            slot_entry: {
                let h = |start| crate::slot_entry_trace::Histogram {
                    bins: core::array::from_fn(|i| value(start + i as u32)),
                    max: value(start + 7),
                    count: value(start + 8),
                    sum: value(start + 9),
                };
                crate::slot_entry_trace::Stats {
                    slot_alarm_phase: core::array::from_fn(|i| value(201 + i as u32)),
                    frame_alarm_phase: core::array::from_fn(|i| value(210 + i as u32)),
                    slot_alarm_rpc: core::array::from_fn(|i| value(219 + i as u32)),
                    frame_alarm_rpc: core::array::from_fn(|i| value(224 + i as u32)),
                    entry0: h(229),
                    entryn: h(239),
                    irq_service: h(249),
                    frame_dispatch: h(259),
                    frame_drain: h(269),
                    late_alarm_phase: core::array::from_fn(|i| value(279 + i as u32)),
                    late_alarm_rpc: core::array::from_fn(|i| value(288 + i as u32)),
                    budget_short: h(293),
                    budget_reject_len: core::array::from_fn(|i| value(303 + i as u32)),
                }
            },
            audio_diagnostic: AudioDiagnostic {
                counters,
                local_tx_depth: u8::MAX,
                ingress_depth: u8::MAX,
            },
        };
        DiagnosticSnapshot {
            clock_us: u64::MAX,
            polls: u32::MAX,
            actor: Some(actor),
            bridge_clock_us: u64::MAX,
            bridge: nrf_embassy::bridge::Snapshot {
                audio_head: u8::MAX,
                audio_tail: u8::MAX,
                control_head: u8::MAX,
                control_tail: u8::MAX,
                delivery_enabled: false,
                tx_seq: u8::MAX,
                rx_seq: u8::MAX,
                rx_seq_initialized: false,
                last_audio_seq: u8::MAX,
                last_audio_seq_valid: false,
                audio_overwrite: value(101),
                control_overwrite: value(102),
                ingress_ok: value(103),
                ingress_reject: value(104),
                ingress_duplicate: value(105),
                tx_audio: value(106),
                tx_control: value(107),
                tx_idle: value(108),
                tx_fail: value(109),
                status_enqueue: value(110),
                status_tx: value(111),
                status_drop: value(112),
                rx_seq_gap: value(113),
                rx_crc_fail: value(114),
                ack_pulses: value(115),
                attempts: value(116),
                keepalive: value(117),
            },
            ..DiagnosticSnapshot::default()
        }
    }

    #[cfg(feature = "pipeline-trace")]
    #[test]
    fn pipeline_production_rows_map_every_audio_and_bridge_field_once() {
        // Distinct source sentinels detect swapped fields and accidental reuse.
        // Expectations are independent of the formatter's grouping/order.
        let expected_audio = [
            "ingress_purge_drop=1",
            "tx_purge_drop=2",
            "tx_count=3",
            "tx_starvation=4",
            "tx_queue_drain=5",
            "audio_fwd=6",
            "tx_overwrite=7",
            "spi_audio_in=8",
            "ingress_inactive_drop=9",
            "ingress_msgq_drop=10",
            "tx_ring_drop=11",
            "relay_ring_drop=12",
            "rf_audio_try=13",
            "rf_audio_ok=14",
            "rf_audio_fail=15",
            "rf_rx_audio_ok=16",
            "rf_rx_malformed=17",
            "rf_rx_version_drop=18",
            "rf_rx_self_drop=19",
            "rf_rx_duplicate_drop=20",
            "rf_rx_inactive_drop=21",
            "spi_out_ok=22",
            "spi_out_drop=23",
            "bundle_tx=24",
            "bundle_rx=25",
            "bundle_bad=26",
            "prev1_forwarded=27",
            "prev2_forwarded=28",
            "prev1_stripped=29",
            "prev2_stripped=30",
            "bundle_late_drop=31",
            "bundle_max_bytes=32",
            "local_deferred_recovery=33",
            "spi_in_frames=34",
            "spi_in_gap_evt=35",
            "spi_in_gap_fr=36",
            "spi_in_reset_evt=37",
            "rf_tx_frames=38",
            "rf_rx_frames=39",
            "rf_rx_gap_evt=40",
            "rf_rx_gap_fr=41",
            "rf_rx_reset_evt=42",
            "spi_out_frames=43",
            "local_prefit_no_time=46",
            "local_prefit_consumed=47",
            "local_entries_consumed=48",
            #[cfg(feature = "same-slot-audio-refit")]
            "same_slot_refit_attempt=44",
            #[cfg(feature = "same-slot-audio-refit")]
            "same_slot_refit_complete=45",
        ];
        let expected_bridge = [
            "audio_overwrite=101",
            "control_overwrite=102",
            "ingress_ok=103",
            "ingress_reject=104",
            "ingress_duplicate=105",
            "tx_audio=106",
            "tx_control=107",
            "tx_idle=108",
            "tx_fail=109",
            "status_enqueue=110",
            "status_tx=111",
            "status_drop=112",
            "rx_seq_gap=113",
            "rx_crc_fail=114",
            "ack_pulses=115",
            "attempts=116",
            "keepalive_phase=117",
        ];
        assert_eq!(
            expected_audio.len(),
            46 + if cfg!(feature = "same-slot-audio-refit") {
                2
            } else {
                0
            }
        );
        let snapshot = pipeline_fixture(false);
        let mut audio = std::string::String::new();
        for row in [
            PipelineRow::AudioIn,
            PipelineRow::AudioTx,
            PipelineRow::AudioRx,
        ] {
            let line = DiagnosticLine::pipeline(snapshot, row).unwrap();
            if matches!(row, PipelineRow::AudioTx) {
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert_eq!(
                    text.split_whitespace()
                        .filter(|field| field.contains('='))
                        .count(),
                    30 + if cfg!(feature = "same-slot-audio-refit") {
                        2
                    } else {
                        0
                    }
                );
                for key in [
                    "local_prefit_no_time=46",
                    "local_prefit_consumed=47",
                    "local_entries_consumed=48",
                ] {
                    assert!(text.split_whitespace().any(|field| field == key));
                }
            }
            audio.push_str(core::str::from_utf8(line.as_bytes()).unwrap());
        }
        for expected in expected_audio {
            assert_eq!(
                audio
                    .split_whitespace()
                    .filter(|field| *field == expected)
                    .count(),
                1,
                "{expected}"
            );
            let key = expected.split_once('=').unwrap().0;
            assert_eq!(
                audio
                    .split_whitespace()
                    .filter(|field| field
                        .split_once('=')
                        .is_some_and(|(actual, _)| actual == key))
                    .count(),
                1,
                "duplicate {key}"
            );
        }
        let bridge = DiagnosticLine::pipeline(snapshot, PipelineRow::Bridge).unwrap();
        let bridge = core::str::from_utf8(bridge.as_bytes()).unwrap();
        for expected in expected_bridge {
            assert_eq!(
                bridge
                    .split_whitespace()
                    .filter(|field| *field == expected)
                    .count(),
                1,
                "{expected}"
            );
            let key = expected.split_once('=').unwrap().0;
            assert_eq!(
                bridge
                    .split_whitespace()
                    .filter(|field| field
                        .split_once('=')
                        .is_some_and(|(actual, _)| actual == key))
                    .count(),
                1,
                "duplicate {key}"
            );
        }
        // Ten cursor/flag fields are published without inventing another total.
        for field in [
            "audio_head=255",
            "audio_tail=255",
            "control_head=255",
            "control_tail=255",
            "delivery_enabled=false",
            "tx_seq=255",
            "rx_seq=255",
            "rx_seq_initialized=false",
            "last_audio_seq=255",
            "last_audio_seq_valid=false",
        ] {
            assert!(bridge.split_whitespace().any(|actual| actual == field));
        }
        assert!(audio.contains("bundle_max_bytes=32"));
        assert!(audio.contains("local_tx_depth=255 relay_depth=unavailable"));
        assert!(audio.contains("ingress_depth=255 admission=true admission_epoch=4294967295"));
        assert!(audio.contains("local_epoch=4294967295 role=2 stored_peers=255"));
        #[cfg(not(feature = "same-slot-audio-refit"))]
        assert!(!audio.contains("same_slot_refit_"));
    }

    #[cfg(feature = "pipeline-trace")]
    #[test]
    fn slot_entry_keys_csv_maxima_and_row_count() {
        assert_eq!(PipelineRow::ALL.len(), 6);
        let frame =
            DiagnosticLine::pipeline(pipeline_fixture(false), PipelineRow::FrameEntry).unwrap();
        let frame = core::str::from_utf8(frame.as_bytes()).unwrap();
        assert!(frame.starts_with("PIPE_FRAME_ENTRY v=1 "));
        assert!(frame.ends_with(
            " frame_dispatch=259,260,261,262,263,264,265,266,267,268 frame_drain=269,270,271,272,273,274,275,276,277,278 late_phase=279,280,281,282,283,284,285,286,287 late_rpc=288,289,290,291,292 budget_short=293,294,295,296,297,298,299,300,301,302 budget_len=303,304,305\n"
        ));
        let distinct =
            DiagnosticLine::pipeline(pipeline_fixture(false), PipelineRow::SlotEntry).unwrap();
        let distinct = core::str::from_utf8(distinct.as_bytes()).unwrap();
        assert_eq!(
            distinct.split_whitespace().collect::<std::vec::Vec<_>>(),
            [
                "PIPE_SLOT_ENTRY",
                "v=1",
                "spi_poll_id=4294967295",
                "spi_clock_us=18446744073709551615",
                "actor_seq=4294967295",
                "actor_clock_us=7",
                "actor_age_us=18446744073709551608",
                "actor_stale=true",
                "slot_phase=201,202,203,204,205,206,207,208,209",
                "frame_phase=210,211,212,213,214,215,216,217,218",
                "slot_rpc=219,220,221,222,223",
                "frame_rpc=224,225,226,227,228",
                "entry0=229,230,231,232,233,234,235,236,237,238",
                "entryn=239,240,241,242,243,244,245,246,247,248",
                "irq_service=249,250,251,252,253,254,255,256,257,258",
            ]
        );
        let mut s = pipeline_fixture(true);
        s.actor.as_mut().unwrap().clock_us = 0;
        let line = DiagnosticLine::pipeline(s, PipelineRow::SlotEntry).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        let fields: std::vec::Vec<_> = text.split_whitespace().collect();
        assert_eq!(fields[0], "PIPE_SLOT_ENTRY");
        let keys: std::vec::Vec<_> = fields[1..]
            .iter()
            .map(|f| f.split_once('=').unwrap().0)
            .collect();
        assert_eq!(
            keys,
            [
                "v",
                "spi_poll_id",
                "spi_clock_us",
                "actor_seq",
                "actor_clock_us",
                "actor_age_us",
                "actor_stale",
                "slot_phase",
                "frame_phase",
                "slot_rpc",
                "frame_rpc",
                "entry0",
                "entryn",
                "irq_service"
            ]
        );
        for (field, count) in fields[8..].iter().zip([9, 9, 5, 5, 10, 10, 10]) {
            let values: std::vec::Vec<_> = field.split_once('=').unwrap().1.split(',').collect();
            assert_eq!(values.len(), count);
            assert!(values.iter().all(|v| *v == "4294967295"));
        }
        // Header/scalar bound + CSV keys/separators + 58 u32s + 51 commas + LF.
        const WORST_CASE: usize = 185 + 75 + 58 * 10 + 51 + 1;
        assert_eq!(WORST_CASE, 892);
        assert!(text.len() <= WORST_CASE && WORST_CASE <= LINE_SIZE);
        DiagnosticLine::pipeline(s, PipelineRow::FrameEntry).expect("max frame row fits");
        s.actor = None;
        let line = DiagnosticLine::pipeline(s, PipelineRow::SlotEntry).unwrap();
        assert!(
            core::str::from_utf8(line.as_bytes())
                .unwrap()
                .ends_with(" actor=unavailable\n")
        );
    }

    #[cfg(feature = "pipeline-trace")]
    #[test]
    fn pipeline_maxima_fit_with_independent_clocks_and_stale_or_missing_actor() {
        let mut snapshot = pipeline_fixture(true);
        for actor_clock in [0, u64::MAX] {
            snapshot.actor.as_mut().unwrap().clock_us = actor_clock;
            for row in PipelineRow::ALL {
                let line = DiagnosticLine::pipeline(snapshot, row).unwrap();
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert!(text.is_ascii() && text.len() <= 896 && text.ends_with('\n'));
                assert!(
                    text.contains("v=1 spi_poll_id=4294967295 spi_clock_us=18446744073709551615")
                );
                if matches!(row, PipelineRow::Bridge) {
                    assert!(text.contains("bridge_clock_us=18446744073709551615"));
                    assert!(!text.contains("actor_seq="));
                    assert_eq!(
                        text.split_whitespace()
                            .filter(|field| field.ends_with("=4294967295"))
                            .count(),
                        18
                    );
                } else {
                    let expected_maxima = match row {
                        PipelineRow::AudioIn => 12, // 8 counters + poll/actor IDs + 2 epochs.
                        PipelineRow::AudioTx => {
                            // pipeline-trace forwards the three local accounting counters.
                            23 + 2 * usize::from(cfg!(feature = "same-slot-audio-refit"))
                        }
                        PipelineRow::AudioRx => 19, // 17 counters + poll/actor IDs.
                        PipelineRow::SlotEntry | PipelineRow::FrameEntry => 2, // poll/actor IDs; counters are CSV fields.
                        PipelineRow::Bridge => unreachable!(),
                    };
                    assert_eq!(
                        text.split_whitespace()
                            .filter(|field| field.ends_with("=4294967295"))
                            .count(),
                        expected_maxima
                    );
                    assert!(text.contains("actor_seq=4294967295"));
                    assert!(text.contains(&std::format!("actor_clock_us={actor_clock}")));
                    assert!(text.contains(if actor_clock == 0 {
                        "actor_age_us=18446744073709551615 actor_stale=true"
                    } else {
                        "actor_age_us=0 actor_stale=false"
                    }));
                }
            }
        }
        snapshot.clock_us = 1;
        let line = DiagnosticLine::pipeline(snapshot, PipelineRow::AudioIn).unwrap();
        assert!(
            core::str::from_utf8(line.as_bytes())
                .unwrap()
                .contains("actor_age_us=none actor_stale=true")
        );
        snapshot.actor = None;
        for row in [
            PipelineRow::AudioIn,
            PipelineRow::AudioTx,
            PipelineRow::AudioRx,
        ] {
            let line = DiagnosticLine::pipeline(snapshot, row).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.ends_with(" spi_clock_us=1 actor=unavailable\n"));
            assert!(!text.contains("actor_seq="));
            assert!(!text.contains("=0"));
        }
        let line = DiagnosticLine::pipeline(snapshot, PipelineRow::Bridge).unwrap();
        assert!(
            core::str::from_utf8(line.as_bytes())
                .unwrap()
                .contains("attempts=4294967295")
        );
    }

    #[cfg(feature = "pipeline-trace")]
    #[test]
    fn pipeline_capture_owns_existing_getters_and_ascii_rejection_preserves_row() {
        let mut runtime = nrf_embassy::runtime::Runtime::default();
        runtime.use_external_ingress().unwrap();
        runtime.account_external_ingress_validation(37).unwrap();
        let captured = AudioDiagnostic::capture(&runtime);
        assert_eq!(captured.counters, runtime.audio_state().counters());
        assert_eq!(captured.local_tx_depth, runtime.audio_state().tx_depth());
        assert_eq!(captured.ingress_depth, runtime.audio_state().ingress_used());
        runtime.account_external_ingress_validation(5).unwrap();
        assert_eq!(runtime.audio_state().counters().bundle_bad, 42);
        assert_eq!(captured.counters.bundle_bad, 37);
        let mut line =
            DiagnosticLine::pipeline(pipeline_fixture(false), PipelineRow::AudioTx).unwrap();
        let before = std::vec::Vec::from(line.as_bytes());
        assert!(line.write_str("\u{00e9}").is_err());
        assert_eq!(line.as_bytes(), before.as_slice());
        assert!(
            line.write_str(core::str::from_utf8(&[b'x'; LINE_SIZE]).unwrap())
                .is_err()
        );
        assert_eq!(line.as_bytes(), before.as_slice());
    }

    #[test]
    fn backend_admission_uses_one_ordered_sample_and_preserves_observer_boundary() {
        let source = include_str!("protocol_backend.rs");
        let admission = source
            .split_once("let deadline = if let Some(window) = self.tx_window {")
            .unwrap()
            .1
            .split_once("None // Source JOIN/LEAVE")
            .unwrap()
            .0;
        for operation in [
            "self.tdma.snapshot()",
            "self.now_us()",
            "self.timing.clock_mapping()",
            "window.evaluate_for_clock(",
            ".record_backend_window_rejection(class, witness)",
            "sync_trace_runtime::observe(",
            "return RadioSendResult {",
        ] {
            assert_eq!(admission.matches(operation).count(), 1);
        }
        let snapshot = admission.find("self.tdma.snapshot()").unwrap();
        let clock = admission.find("self.now_us()").unwrap();
        let mapping = admission.find("self.timing.clock_mapping()").unwrap();
        let evaluation = admission.find("window.evaluate_for_clock(").unwrap();
        let record = admission.find(".record_backend_window_rejection(").unwrap();
        let observe = admission.find("sync_trace_runtime::observe(").unwrap();
        let reject = admission.find("return RadioSendResult {").unwrap();
        let rejection = admission[reject..].split_once("};").unwrap().0;
        assert_eq!(
            rejection
                .split_whitespace()
                .collect::<std::string::String>(),
            "returnRadioSendResult{code:-11,disposition:SubmissionDisposition::NotSubmitted,"
        );
        assert!(snapshot < clock && clock < mapping && mapping < evaluation);
        assert!(evaluation < record && record < observe && observe < reject);
        assert!(!admission.contains("Stage::BackendWindowRejected"));
        let probe = source.split_once("fn tx_window_valid(&self)").unwrap().1;
        let probe = probe.split_once("fn apply(").unwrap().0;
        assert!(probe.contains("self.tx_deadline(window, 0).is_some()"));
        assert!(!probe.contains("record"));
    }

    #[test]
    fn radio_driver_errors_remain_unknown_and_terminal_receiver_unavailable_keeps_errno() {
        // The hardware owner is ARM-only. Check its actual conversion boundaries,
        // without fabricating a driver launch or a physical completion report.
        let source = include_str!("radio_receiver.rs");
        let submission = source
            .split_once("let ticket = match result {")
            .unwrap()
            .1
            .split_once("diagnostics.record(class, Stage::AcceptedTickets)")
            .unwrap()
            .0;
        assert_eq!(submission.matches("return RadioSendResult {").count(), 1);
        let rejection = submission
            .split_once("return RadioSendResult {")
            .unwrap()
            .1
            .split_once("};")
            .unwrap()
            .0;
        assert_eq!(
            rejection
                .split_whitespace()
                .collect::<std::string::String>(),
            "code:error_errno(error),disposition:SubmissionDisposition::Unknown,"
        );
        let errors = source
            .split_once("pub(crate) fn error_errno(error: Error) -> i32 {")
            .unwrap()
            .1
            .split_once("fn fault_errno(")
            .unwrap()
            .0;
        assert!(errors.contains("Error::TxFull | Error::RxEmpty | Error::OutOfMemory => -11,"));
        let faults = source
            .split_once("fn fault_errno(fault: TxFault) -> i32 {")
            .unwrap()
            .1
            .split_once("impl RadioOwner {")
            .unwrap()
            .0;
        assert!(faults.contains("TxFault::ReceiverUnavailable => -11,"));
        let terminal = source
            .split_once("let (terminal, rc) = match report.outcome {")
            .unwrap()
            .1;
        let terminal = terminal.split_once("pub(crate) fn service(").unwrap().0;
        assert!(terminal.contains("fault_errno(fault)"));
        assert!(terminal.contains("self.tx.record_send_result(terminal, rc)"));
    }

    #[test]
    fn executing_identity_renders_current_feature_policy() {
        let identity = core::str::from_utf8(IDENTITY).unwrap();
        let generated = core::str::from_utf8(GENERATED_IDENTITY).unwrap();
        assert!(identity.starts_with(generated.strip_suffix('\n').unwrap()));
        let (selected, other) = if cfg!(feature = "guarded-completion-retention") {
            ("RetainGuardedCompletion", "StrictSoftwareDeadline")
        } else {
            ("StrictSoftwareDeadline", "RetainGuardedCompletion")
        };
        let field = identity.split_once(" completion_policy=").unwrap().1;
        assert_eq!(field.strip_suffix('\n').unwrap(), selected);
        assert!(!identity.contains(other));
        assert!(identity.is_ascii() && identity.len() <= 512);
    }

    #[test]
    fn generated_identity_matches_executing_metadata_not_image_checksum() {
        let identity = core::str::from_utf8(IDENTITY).unwrap();
        let metadata = include_str!(concat!(env!("OUT_DIR"), "/integration-metadata.txt"));
        let revision = metadata
            .lines()
            .find_map(|line| line.strip_prefix("source_sha="))
            .unwrap();
        assert!(identity.contains(revision));
        assert!(identity.contains(concat!("version=", env!("CARGO_PKG_VERSION"))));
        assert!(metadata.contains("backend=live-strict-full-protocol-hardware-unqualified"));
        assert!(identity.contains("backend=live-strict-full-protocol-hardware-unqualified"));
        assert!(identity.contains("source_state=unqualified-working-tree"));
        assert!(identity.contains("bridge=6 mesh=5 RFqualified=false"));
        assert!(!identity.contains("not-integrated"));
        assert!(!identity.contains("receiver-only"));
        assert!(!identity.contains("checksum"));
        assert!(identity.is_ascii() && identity.len() <= 512);
    }

    #[test]
    fn initial_actor_and_unavailable_counters_are_not_fabricated_readiness() {
        let snapshot = DiagnosticSnapshot::default();
        let line = DiagnosticLine::actor_status(snapshot).unwrap();
        assert_eq!(
            core::str::from_utf8(line.as_bytes()).unwrap(),
            "software_capability=0 RFqualified=false actor=unavailable\n"
        );
        let line = DiagnosticLine::radio_status(snapshot).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("RFwanted=none minEpoch=none currentSession=none channel=none"));
        assert!(text.contains("TXattempted=none PhysicalComplete=none"));
        assert!(text.contains("ingress_admitted=none"));
        assert!(text.contains("rpc_phase=none"));
    }

    #[test]
    fn owned_status_rows_fit_maxima_and_age_marks_await_stalls() {
        use nrf_embassy::runtime::{Membership, Role, State};
        let mut runtime = nrf_embassy::runtime::Runtime::default().snapshot();
        runtime.membership = Membership {
            state: State::Active,
            role: Role::Participant,
            peer_count: u8::MAX,
            membership_known: false,
            node_id: u8::MAX,
            slot_index: i8::MIN,
            coordinator_id: u8::MAX,
        };
        runtime.talk_channel = u8::MAX;
        runtime.audio.epoch = u32::MAX;
        runtime.membership_rx = nrf_embassy::protocol_membership::MembershipRx {
            applied: u32::MAX,
            removed_local: u32::MAX,
            source: u32::MAX,
            term: u32::MAX,
            revision: u32::MAX,
            identity: u32::MAX,
            other: u32::MAX,
        };
        let actor = ActorSnapshot {
            sequence: u32::MAX,
            clock_us: u64::MAX,
            runtime,
            reported_peers: u8::MAX,
            capability: (false, false, false),
            admission: nrf_embassy::protocol_audio::IngressAdmissionGate {
                enabled: false,
                epoch: u32::MAX,
            },
            radio: RadioSnapshot {
                wanted: false,
                min_epoch: u64::MAX,
                current_session: Some(u64::MAX),
                channel: u8::MAX,
                tx: crate::actor_policy::TxCounters {
                    attempted: u32::MAX,
                    physical_complete: u32::MAX,
                    deadline: u32::MAX,
                    fault: u32::MAX,
                },
            },
            #[cfg(feature = "sync-trace")]
            tx_diagnostics: crate::tx_diagnostics::Stats::default(),
            #[cfg(feature = "pipeline-trace")]
            audio_diagnostic: AudioDiagnostic::default(),
            #[cfg(feature = "slot-entry-trace")]
            slot_entry: crate::slot_entry_trace::Stats::ZERO,
        };
        #[cfg(feature = "sync-trace")]
        {
            use crate::tx_diagnostics::{Row, Stats, WireClass};
            let row = Row {
                requests: u32::MAX,
                backend_window_rejected: u32::MAX,
                validation_rejected: u32::MAX,
                budget_rejected: u32::MAX,
                submission_rejected: u32::MAX,
                accepted_tickets: u32::MAX,
                complete: u32::MAX,
                guard_deadline: u32::MAX,
                software_timed_out: u32::MAX,
                cancelled: u32::MAX,
                other: u32::MAX,
            };
            let captured = ActorSnapshot {
                tx_diagnostics: Stats {
                    rows: [row; 6],
                    counter_saturated: true,
                    ..Stats::default()
                },
                ..actor
            };
            for class in WireClass::ALL {
                let line = DiagnosticLine::tx_class_status(captured, class).unwrap();
                let text = core::str::from_utf8(line.as_bytes()).unwrap();
                assert!(text.is_ascii() && text.len() <= LINE_SIZE);
                assert!(text.starts_with("TXclass="));
                assert!(text.contains(class.name()));
                assert!(text.contains("actor_seq=4294967295"));
                assert!(text.contains("actor_clock_us=18446744073709551615"));
                assert!(text.contains("counter_saturated=true"));
                assert_eq!(text.matches("=4294967295").count(), 12);
                assert!(text.ends_with("other=4294967295\n"));
            }
            for class in WireClass::ALL {
                assert!(
                    DiagnosticLine::tx_window_status(captured, class)
                        .unwrap()
                        .is_none()
                );
                let zero = DiagnosticLine::tx_admission_status(captured, class).unwrap();
                let zero = core::str::from_utf8(zero.as_bytes()).unwrap();
                assert!(zero.contains(" r0=0 "));
                assert!(zero.ends_with(" r33=0\n"));
                for reason in crate::tx_diagnostics::all_reasons() {
                    let mut captured = captured;
                    captured.tx_diagnostics.admission_reasons =
                        [[u32::MAX; crate::actor_policy::AdmissionRejection::COUNT]; 6];
                    let mut witness = crate::tx_diagnostics::test_witness(reason);
                    witness.ordinal = u32::MAX;
                    captured.tx_diagnostics.last_admission_rejection[class as usize] =
                        Some(witness);
                    let counts = DiagnosticLine::tx_admission_status(captured, class).unwrap();
                    let counts = core::str::from_utf8(counts.as_bytes()).unwrap();
                    assert!(counts.is_ascii() && counts.len() <= LINE_SIZE);
                    for index in 0..crate::actor_policy::AdmissionRejection::COUNT {
                        assert!(counts.contains(&std::format!(" r{index}=4294967295")));
                    }
                    assert!(!counts.contains(" r34="));
                    for absent in [false, true] {
                        let mut witness = witness;
                        if absent {
                            witness.kind = crate::tx_diagnostics::WindowKind::Slot;
                            witness.snapshot.running = true;
                            witness.snapshot.synchronized = true;
                            witness.timing_now = None;
                            witness.physical_window_start = None;
                            witness.physical_window_deadline = None;
                            witness.budget_us = None;
                        }
                        captured.tx_diagnostics.last_admission_rejection[class as usize] =
                            Some(witness);
                        let line = DiagnosticLine::tx_window_status(captured, class)
                            .unwrap()
                            .unwrap();
                        let text = core::str::from_utf8(line.as_bytes()).unwrap();
                        assert!(text.is_ascii() && text.len() <= LINE_SIZE);
                        assert!(text.starts_with("TXWINDOW="));
                        assert!(text.contains("actor_seq=4294967295 ordinal=4294967295"));
                        assert!(
                            text.contains(&std::format!("r={} reason={reason}", reason.index()))
                        );
                        assert!(text.contains("profile=LegacyNrfRtc10000"));
                        assert!(text.contains(
                            "anchor_p=18446744073709551615 anchor_t=-9223372036854775808"
                        ));
                        assert!(text.contains("physical_now=-9223372036854775808"));
                        assert!(text.contains("saved_frame=4294967295 current_work_frame=4294967295 snapshot_frame=4294967295"));
                        if absent {
                            assert!(text.contains("kind=Slot"));
                            assert!(text.contains("running=true synchronized=true slot=-128"));
                            assert!(
                                text.contains(
                                    "timing_now=NA window_start=NA window_end=NA budget=NA"
                                )
                            );
                        } else {
                            assert!(text.contains("kind=Control"));
                            assert!(text.contains("running=false synchronized=false slot=-128"));
                            assert!(text.contains("timing_now=-9223372036854775808 window_start=18446744073709551615 window_end=18446744073709551615 budget=-9223372036854775808"));
                        }
                    }
                }
                let mut zero_witness = crate::tx_diagnostics::test_witness(
                    crate::actor_policy::AdmissionRejection::BudgetTooShort,
                );
                zero_witness.physical_now = 0;
                zero_witness.timing_now = Some(0);
                zero_witness.physical_window_start = Some(0);
                zero_witness.physical_window_deadline = Some(0);
                zero_witness.budget_us = Some(0);
                let mut zero_actor = captured;
                zero_actor.tx_diagnostics.last_admission_rejection[class as usize] =
                    Some(zero_witness);
                let zero = DiagnosticLine::tx_window_status(zero_actor, class)
                    .unwrap()
                    .unwrap();
                let zero = core::str::from_utf8(zero.as_bytes()).unwrap();
                assert!(
                    zero.contains(
                        "physical_now=0 timing_now=0 window_start=0 window_end=0 budget=0"
                    )
                );
            }
        }
        let mut inbox = crate::transport::TransportState::inactive().snapshot();
        inbox.queued = usize::MAX;
        inbox.gate.epoch = u32::MAX;
        inbox.accounting.ingress.overwritten = u32::MAX;
        inbox.accounting.ingress.purged = u32::MAX;
        inbox.accounting.bundle_bad = u32::MAX;
        let s = DiagnosticSnapshot {
            clock_us: u64::MAX,
            actor: Some(actor),
            software_capability: true,
            ingress: Some(inbox),
            rpc: Some(crate::bridge_rpc::Snapshot {
                initialized: false,
                exhausted: false,
                phase: crate::bridge_rpc::Phase::Completed,
                abandoned: false,
            }),
            ..DiagnosticSnapshot::default()
        };
        for snapshot in [
            s,
            DiagnosticSnapshot {
                actor: Some(ActorSnapshot {
                    clock_us: 0,
                    ..actor
                }),
                ..s
            },
        ] {
            for line in [
                DiagnosticLine::actor_status(snapshot).unwrap(),
                DiagnosticLine::radio_status(snapshot).unwrap(),
            ] {
                assert!(line.as_bytes().is_ascii());
                assert!(line.as_bytes().len() <= LINE_SIZE);
            }
            let line = DiagnosticLine::actor_status(snapshot).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.contains("slot=-128"));
            assert!(text.contains(" mv3_applied=4294967295 mv3_removed_local=4294967295 mv3_rej_source=4294967295 mv3_rej_term=4294967295 mv3_rej_revision=4294967295 mv3_rej_identity=4294967295 mv3_rej_other=4294967295\n"));
        }
        assert!(!actor.stale(u64::MAX));
        assert!(actor.stale(0)); // Regressed time is not fresh.
        let old = ActorSnapshot {
            clock_us: 0,
            ..actor
        };
        assert!(!old.stale(ACTOR_PERIOD_US));
        assert!(old.stale(ACTOR_PERIOD_US + 1));
        assert_eq!(old.sequence, actor.sequence); // Copies are not live loans.
    }

    #[test]
    fn serial_fixed_width_word_order() {
        assert_eq!(&serial_number(0, 0), b"0000000000000000");
        assert_eq!(&serial_number(0x0123abcd, 0x89ef4567), b"0123ABCD89EF4567");
        assert_eq!(&serial_number(u32::MAX, u32::MAX), b"FFFFFFFFFFFFFFFF");
    }

    #[test]
    fn worst_case_ascii_line_fits_and_signed_values_survive() {
        for (signed, stamp) in [(i32::MIN, i64::MIN), (i32::MAX, i64::MAX), (i32::MIN, 0)] {
            let s = DiagnosticSnapshot {
                clock_us: u64::MAX,
                polls: u32::MAX,
                spi_ok: u32::MAX,
                spi_fail: u32::MAX,
                tdma_clock_us: u64::MAX,
                tdma_frame: u32::MAX,
                tdma_jitter_max_us: u32::MAX,
                tdma_skipped: u32::MAX,
                tdma_correction_pending_us: signed,
                tdma_fault: u32::MAX,
                ws_frame: u32::MAX,
                ws_count: u32::MAX,
                drops: u32::MAX,
                receiver: ReceiverSnapshot {
                    irq_count: u32::MAX,
                    received_packets: u32::MAX,
                    bytes: u32::MAX,
                    missing_metadata: u32::MAX,
                    missing_clock: u32::MAX,
                    clock_errors: u32::MAX,
                    last: Some(crate::receiver::ReceiverMetadata {
                        channel: u8::MAX,
                        epoch: u64::MAX,
                        timestamp_us: Some(stamp),
                    }),
                },
                ws: WsStats {
                    total_edges: u32::MAX,
                    sample_count: u32::MAX,
                    valid_count: u32::MAX,
                    no_signal_count: u32::MAX,
                    rejected_count: u32::MAX,
                    last_delta_edges: u32::MAX,
                    last_correction_us: signed,
                    cumulative_drift_us: signed,
                },
                ..DiagnosticSnapshot::default()
            };
            let line = DiagnosticLine::from_snapshot(s).unwrap();
            let text = core::str::from_utf8(line.as_bytes()).unwrap();
            assert!(text.is_ascii());
            assert!(text.len() <= LINE_SIZE);
            assert!(
                text.len() > 512,
                "RF extension requires the explicit new bound"
            );
            assert!(text.contains("lastEpoch=18446744073709551615"));
            assert!(text.contains(match stamp {
                i64::MIN => "lastTimestamp=-9223372036854775808",
                i64::MAX => "lastTimestamp=9223372036854775807",
                _ => "lastTimestamp=0 rx_age=18446744073709551615",
            }));
            assert!(text.ends_with("radio=actor-owned-unqualified RFqualified=false\n"));
            assert!(text.contains(if signed < 0 {
                "correctionPending=-2147483648"
            } else {
                "correctionPending=2147483647"
            }));
            assert!(text.contains(if signed < 0 {
                "corr=-2147483648 drift=-2147483648"
            } else {
                "corr=2147483647 drift=2147483647"
            }));
        }
        let mut line = DiagnosticLine {
            bytes: [0; LINE_SIZE],
            len: 0,
        };
        assert!(
            line.write_str(core::str::from_utf8(&[b'x'; LINE_SIZE]).unwrap())
                .is_ok()
        );
        assert!(line.write_str("x").is_err());
        assert_eq!(line.as_bytes().len(), LINE_SIZE);
    }

    #[test]
    fn absent_rx_record_is_ascii_none_not_a_fake_zero_or_readiness() {
        let line = DiagnosticLine::from_snapshot(DiagnosticSnapshot::default()).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("lastChannel=none lastEpoch=none lastTimestamp=none rx_age=none"));
        assert!(text.contains("radio=actor-owned-unqualified RFqualified=false"));
        assert!(IDENTITY.is_ascii());
        assert!(
            core::str::from_utf8(IDENTITY)
                .unwrap()
                .contains("RFqualified=false")
        );
        let mut snapshot = DiagnosticSnapshot::default();
        snapshot.receiver.last = Some(crate::receiver::ReceiverMetadata {
            channel: 40,
            epoch: 1,
            timestamp_us: Some(i64::MIN),
        });
        let line = DiagnosticLine::from_snapshot(snapshot).unwrap();
        let text = core::str::from_utf8(line.as_bytes()).unwrap();
        assert!(text.contains("lastTimestamp=-9223372036854775808 rx_age=none"));
    }

    #[test]
    fn packets_are_bounded_and_terminate_full_packets() {
        let bytes = [b'x'; LINE_SIZE];
        for len in [
            0,
            1,
            63,
            64,
            65,
            128,
            383,
            384,
            511,
            512,
            LINE_SIZE - 1,
            LINE_SIZE,
        ] {
            let mut total = 0;
            let mut last = None;
            let mut count = 0;
            for packet in packets(&bytes[..len]) {
                assert!(packet.len() <= PACKET_SIZE);
                total += packet.len();
                last = Some(packet.len());
                count += 1;
            }
            assert_eq!(total, len);
            assert_eq!(last, Some(len % PACKET_SIZE));
            assert_eq!(count, len / PACKET_SIZE + 1);
        }
    }

    #[test]
    fn repeated_ticks_skip_elapsed_frames_and_wrap_are_real_clock_frames() {
        let mut scheduler = FrameScheduler::new();
        let mut sampler = WsSampler::new();
        sampler.start();
        assert_eq!(scheduler.new_frame(0), Some(0));
        assert!(!sample_diagnostic(&mut sampler, 0, 0).valid);
        for time in [0, 2_000, 18_000, 19_999] {
            assert_eq!(scheduler.new_frame(time), None);
        }
        let frame = scheduler.new_frame(60_000).unwrap();
        assert_eq!(frame, 3);
        assert!(sample_diagnostic(&mut sampler, 2_880, frame).valid);
        assert_eq!(sampler.stats().sample_count, 2);
        assert_eq!(scheduler.new_frame(60_000), None);
        assert_eq!(
            scheduler.new_frame(u64::from(u32::MAX) * 20_000),
            Some(u32::MAX)
        );
        assert_eq!(
            scheduler.new_frame((u64::from(u32::MAX) + 1) * 20_000),
            Some(0)
        );
        assert_eq!(
            scheduler.new_frame((u64::from(u32::MAX) + 1) * 20_000 + 1),
            None
        );
    }

    #[test]
    fn sampler_count_and_frame_wrap_and_invalid_stats_remain_diagnostic() {
        let mut sampler = WsSampler::new();
        sampler.start();
        let baseline = u32::MAX - 100;
        assert!(!sample_diagnostic(&mut sampler, baseline, u32::MAX).valid);
        let count = baseline.wrapping_add(920);
        let result = sample_diagnostic(&mut sampler, count, 0);
        assert!(result.valid);
        assert_eq!(result.correction_us, 208);
        assert_eq!(sampler.stats().total_edges, count);
        assert_eq!(sampler.stats().last_delta_edges, 920);
        assert_eq!(sampler.stats().valid_count, 1);
        assert_eq!(sampler.stats().cumulative_drift_us, 833);
        assert_eq!(
            sample_diagnostic(&mut sampler, count, 1),
            SampleResult::default()
        );
        assert_eq!(sampler.stats().no_signal_count, 1);
        assert_eq!(
            sample_diagnostic(&mut sampler, count + 1, 2),
            SampleResult::default()
        );
        assert_eq!(sampler.stats().rejected_count, 1);
        sampler.stop();
        let stats = sampler.stats();
        assert_eq!(
            sample_diagnostic(&mut sampler, count + 960, 3),
            SampleResult::default()
        );
        assert_eq!(sampler.stats(), stats);
    }
}
