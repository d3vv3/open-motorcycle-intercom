//! Serialized port of all 41 functions in `mesh_protocol_audio.c`.
//! Tokens model producer capture/admission policy, not IRQ or mutex qualification.
use crate::platform::{
    accepts_protocol_algorithms, AudioBoundary, Effect, Integration, Platform, Reply,
};
use crate::protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply};
use crate::protocol_membership::{Error, MembershipEffect, MembershipPlatform, ProtocolState};
use crate::protocol_tx::{ClockSnapshot, CombinedFullTxPlatform, RadioSend, EAGAIN};
use crate::runtime::{RelayEntry, Role, Runtime, State, EBUSY, EINVAL, ENOTSUP};
use openhelmet_mesh::{adaptive as a, core as c};
use openhelmet_wire::{audio_bundle as b, mesh as w};

const ETIME: i32 = -62;
const FRAME_MS: u32 = 20;
pub const CURRENT_ONLY_AIRTIME_US: u32 = 129 + (11 + 9 + 8 + 48) * 4 + 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AudioEntry {
    pub data: [u8; 200],
    pub len: u8,
    pub audio_flags: u8,
    pub packet_type: u8,
}
impl Default for AudioEntry {
    fn default() -> Self {
        Self {
            data: [0; 200],
            len: 0,
            audio_flags: 0,
            packet_type: 0,
        }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RfSeq16 {
    pub last: u16,
    pub initialized: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AudioCounters {
    pub ingress_purge_drop: u32,
    pub tx_purge_drop: u32,
    pub tx_count: u32,
    pub tx_starvation: u32,
    pub tx_queue_drain: u32,
    pub audio_fwd: u32,
    pub tx_overwrite: u32,
    pub spi_audio_in: u32,
    pub ingress_inactive_drop: u32,
    pub ingress_msgq_drop: u32,
    pub tx_ring_drop: u32,
    pub relay_ring_drop: u32,
    pub rf_audio_try: u32,
    pub rf_audio_ok: u32,
    pub rf_audio_fail: u32,
    pub rf_rx_audio_ok: u32,
    pub rf_rx_malformed: u32,
    pub rf_rx_version_drop: u32,
    pub rf_rx_self_drop: u32,
    pub rf_rx_duplicate_drop: u32,
    pub rf_rx_inactive_drop: u32,
    pub spi_out_ok: u32,
    pub spi_out_drop: u32,
    pub bundle_tx: u32,
    pub bundle_rx: u32,
    pub bundle_bad: u32,
    pub prev1_forwarded: u32,
    pub prev2_forwarded: u32,
    pub prev1_stripped: u32,
    pub prev2_stripped: u32,
    pub bundle_late_drop: u32,
    pub bundle_max_bytes: u32,
    pub local_deferred_recovery: u32,
    pub spi_in_frames: u32,
    pub spi_in_gap_evt: u32,
    pub spi_in_gap_fr: u32,
    pub spi_in_reset_evt: u32,
    pub rf_tx_frames: u32,
    pub rf_rx_frames: u32,
    pub rf_rx_gap_evt: u32,
    pub rf_rx_gap_fr: u32,
    pub rf_rx_reset_evt: u32,
    pub spi_out_frames: u32,
    #[cfg(feature = "audio-accounting-trace")]
    pub local_prefit_no_time: u32,
    #[cfg(feature = "audio-accounting-trace")]
    pub local_prefit_consumed: u32,
    #[cfg(feature = "audio-accounting-trace")]
    pub local_entries_consumed: u32,
    #[cfg(feature = "same-slot-audio-refit")]
    pub same_slot_refit_attempt: u32,
    #[cfg(feature = "same-slot-audio-refit")]
    pub same_slot_refit_complete: u32,
}
fn inc(x: &mut u32) {
    *x = x.wrapping_add(1);
}

#[derive(PartialEq, Eq)]
pub struct AudioState {
    // Rust-only queue ownership guard, excluded from the source C projection.
    external_ingress: bool,
    pub(crate) boundary: AudioBoundary,
    pub(crate) ingress_enabled: bool,
    pub(crate) ingress: [AudioEntry; 16],
    pub(crate) ingress_read: u8,
    pub(crate) ingress_write: u8,
    pub(crate) ingress_used: u8,
    pub(crate) work_pending: bool,
    pub(crate) initialized: bool,
    pub(crate) tx: [AudioEntry; 16],
    pub(crate) tx_head: u8,
    pub(crate) tx_tail: u8,
    pub(crate) relay_contention_turn: bool,
    pub(crate) deferred_pending: bool,
    pub(crate) deferred_seq: u16,
    pub(crate) spi_seq: [c::Seq16; 256],
    pub(crate) rf_seq: [RfSeq16; 256],
    pub(crate) counters: AudioCounters,
    pub(crate) last_audio_in_time: u32,
    pub(crate) phase_hist: [u32; 10],
    pub(crate) tx_depth_dbg: u8,
}
impl Default for AudioState {
    fn default() -> Self {
        Self {
            external_ingress: false,
            boundary: AudioBoundary::default(),
            ingress_enabled: false,
            ingress: [AudioEntry::default(); 16],
            ingress_read: 0,
            ingress_write: 0,
            ingress_used: 0,
            work_pending: false,
            initialized: false,
            tx: [AudioEntry::default(); 16],
            tx_head: 0,
            tx_tail: 0,
            relay_contention_turn: false,
            deferred_pending: false,
            deferred_seq: 0,
            spi_seq: [c::Seq16::default(); 256],
            rf_seq: [RfSeq16::default(); 256],
            counters: AudioCounters::default(),
            last_audio_in_time: 0,
            phase_hist: [0; 10],
            tx_depth_dbg: 0,
        }
    }
}
impl AudioState {
    pub fn boundary(&self) -> AudioBoundary {
        self.boundary
    }
    pub fn counters(&self) -> AudioCounters {
        self.counters
    }
    pub fn ingress_storage(&self) -> &[AudioEntry; 16] {
        &self.ingress
    }
    pub fn tx_storage(&self) -> &[AudioEntry; 16] {
        &self.tx
    }
    pub fn ingress_used(&self) -> u8 {
        self.ingress_used
    }
    pub fn tx_depth(&self) -> u8 {
        (self.tx_head + 16 - self.tx_tail) % 16
    }
    fn purge_ingress(&mut self) {
        self.counters.ingress_purge_drop = self
            .counters
            .ingress_purge_drop
            .wrapping_add(u32::from(self.ingress_used));
        self.ingress_read = 0;
        self.ingress_write = 0;
        self.ingress_used = 0;
    }
}

/// Copy this token before validating/copying the producer buffer. A future
/// hardware producer reads the published gate, then sends the request to owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IngressToken {
    epoch: u32,
}
impl IngressToken {
    pub fn from_gate(gate: AudioBoundary) -> Self {
        Self { epoch: gate.epoch }
    }
    pub fn epoch(self) -> u32 {
        self.epoch
    }
    pub fn prepare(self, data: &[u8]) -> Result<IngressRequest, i32> {
        let bundle = b::audio_bundle_parse(data).ok_or(EINVAL)?;
        let mut entry = AudioEntry {
            len: data.len() as u8,
            audio_flags: bundle.flags & b::CURRENT_ACTIVE,
            packet_type: 13,
            ..AudioEntry::default()
        };
        entry.data[..data.len()].copy_from_slice(data);
        Ok(IngressRequest {
            captured_epoch: self.epoch,
            entry,
        })
    }
    /// Publish either outcome to the actor so validation counters remain owned.
    pub fn validate(self, data: &[u8]) -> PreparedIngress {
        PreparedIngress {
            result: self.prepare(data).map_err(|_| self),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PreparedIngress {
    result: Result<IngressRequest, IngressToken>,
}
impl PreparedIngress {
    /// Borrow the validated message without copying it or replacing its epoch.
    pub fn request(&self) -> Result<&IngressRequest, IngressToken> {
        self.result.as_ref().map_err(|token| *token)
    }
}

/// Producer admission only: Scanning is deliberately not part of this gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IngressAdmissionGate {
    pub enabled: bool,
    pub epoch: u32,
}
impl IngressAdmissionGate {
    /// Capture before validation/copying or an awaited transfer.
    pub fn capture(self) -> IngressToken {
        IngressToken { epoch: self.epoch }
    }
}

/// Completed external mailbox operations, applied once by the actor.
/// A pop followed by a failed put counts both an overwrite and a failed enqueue.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IngressDeltaAccounting {
    pub overwritten: u32,
    pub purged: u32,
    pub failed_enqueue: u32,
}
/// Owned immutable message. Admission never replaces its captured epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IngressRequest {
    captured_epoch: u32,
    entry: AudioEntry,
}
impl IngressRequest {
    pub fn captured_epoch(&self) -> u32 {
        self.captured_epoch
    }
    pub fn entry(&self) -> &AudioEntry {
        &self.entry
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AudioEffect<'a> {
    InitWork,
    SetSlotCallback,
    CancelWork,
    SubmitWork,
    NowUs,
    SlotRemainingUs,
    TimeToSlotUs,
    BridgeDeliver {
        source: u8,
        bundle: &'a [u8],
    },
    Starvation {
        total: u32,
        role: Role,
        node: u8,
        slot: i8,
        depth: u8,
        ingress: u8,
        time_to_slot: i32,
    },
}
/// IO-only extension. Algorithm ownership is explicitly selected by the adapter.
#[allow(async_fn_in_trait)]
pub trait TypedAudioPlatform: AdaptivePlatform {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64;
    async fn audio_io_async(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_io(effect)
    }
}
pub use self::TypedAudioPlatform as AudioPlatform;

fn rejects_nonstrict_hardware(p: &impl AdaptivePlatform) -> bool {
    p.integration() == Integration::HardwareUnqualified && !p.owned_protocol_algorithms()
}

pub struct FullAudioPlatform<'a, P: AudioPlatform>(pub &'a mut P);
impl<P: AudioPlatform> Platform for FullAudioPlatform<'_, P> {
    fn ingress_gate_changed(&mut self, gate: IngressAdmissionGate, purge: bool) {
        self.0.ingress_gate_changed(gate, purge);
    }
    fn prepare_ingress_boundary(&mut self, gate: IngressAdmissionGate) {
        self.0.prepare_ingress_boundary(gate);
    }
    fn tx_window_valid(&self) -> bool {
        self.0.tx_window_valid()
    }
    fn owned_protocol_algorithms(&self) -> bool {
        self.0.owned_protocol_algorithms()
    }
    fn integration(&self) -> Integration {
        self.0.integration()
    }
    fn now_us(&self) -> i64 {
        self.0.now_us()
    }
    async fn apply_async(&mut self, e: Effect) -> Reply {
        self.0.apply_async(e).await
    }
    fn apply(&mut self, e: Effect) -> Reply {
        self.0.apply(e)
    }
    fn dispatch_audio(&mut self, e: Effect, r: &mut Runtime) -> Option<Reply> {
        if self.integration() == Integration::HardwareUnqualified
            && !self.owned_protocol_algorithms()
            && matches!(
                e,
                Effect::AudioInit
                    | Effect::AudioCancel
                    | Effect::ResetRfTrackers
                    | Effect::Ingress { .. }
                    | Effect::PurgeTx
                    | Effect::ClearGrants
                    | Effect::ClearBitmaps
                    | Effect::LocalAudio(_)
                    | Effect::UpdateGrants
            )
        {
            return Some(Reply::Value(ENOTSUP));
        }
        let reply = match e {
            Effect::AudioInit => {
                r.audio_init(self);
                Reply::None
            }
            Effect::AudioCancel => {
                r.audio_cancel_work(self);
                Reply::None
            }
            Effect::ResetRfTrackers => {
                r.audio_reset_all_rf_trackers();
                Reply::None
            }
            Effect::Ingress { enabled, purge } => {
                r.audio_set_ingress_enabled(enabled, purge);
                self.ingress_gate_changed(r.ingress_admission_gate(), purge);
                Reply::None
            }
            Effect::PurgeTx => {
                r.audio_purge_tx_ring();
                Reply::None
            }
            Effect::ClearGrants => {
                r.audio_clear_speaker_grants();
                Reply::None
            }
            Effect::ClearBitmaps => {
                r.audio_clear_bitmaps();
                Reply::None
            }
            Effect::LocalAudio(enabled) => {
                r.audio_set_local_enabled(enabled, self);
                Reply::Audio(r.audio_state.boundary)
            }
            Effect::UpdateGrants => {
                r.audio_update_speaker_grants(self);
                Reply::None
            }
            _ => return None,
        };
        Some(reply)
    }
}
impl<P: AudioPlatform> MembershipPlatform for FullAudioPlatform<'_, P> {
    async fn membership_output_async(
        &mut self,
        output: crate::protocol_membership::MembershipOutput,
    ) -> bool {
        self.0.membership_output_async(output).await
    }
    async fn dispatch_membership_async(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        if e == MembershipEffect::JoinRequest {
            CombinedFullTxPlatform(self)
                .dispatch_membership_async(e, r)
                .await
        } else {
            self.dispatch_membership(e, r)
        }
    }
    fn uptime_ms(&mut self) -> i64 {
        self.0.uptime_ms()
    }
    fn membership_effect(&mut self, e: MembershipEffect, s: &mut ProtocolState) -> bool {
        self.0.membership_effect(e, s)
    }
    fn dispatch_membership(&mut self, e: MembershipEffect, r: &mut Runtime) -> bool {
        if r.audio_membership_effect(e, self) {
            true
        } else {
            CombinedFullTxPlatform(self).dispatch_membership(e, r)
        }
    }
}
impl<P: AudioPlatform> AdaptivePlatform for FullAudioPlatform<'_, P> {
    fn tdma_rx_timestamp(&mut self, physical_stamp: i64) -> Option<i64> {
        self.0.tdma_rx_timestamp(physical_stamp)
    }
    async fn adaptive_status_async(&mut self, status: crate::runtime::Membership) -> AdaptiveReply {
        self.0.adaptive_status_async(status).await
    }
    fn adaptive_effect(&mut self, e: AdaptiveEffect, io: AdaptiveIo<'_>) -> AdaptiveReply {
        self.0.adaptive_effect(e, io)
    }
    fn owned_adaptive_wrappers(&self) -> bool {
        true
    }
    fn owned_tx_wrappers(&self) -> bool {
        true
    }
    fn owned_audio_wrappers(&self) -> bool {
        true
    }
    fn audio_effect(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io(e)
    }
    async fn audio_effect_async(&mut self, e: AudioEffect<'_>) -> i64 {
        self.0.audio_io_async(e).await
    }
    fn clock_snapshot(&mut self) -> Option<ClockSnapshot> {
        self.0.clock_snapshot()
    }
    fn radio_send(&mut self, packet: RadioSend<'_>) -> i32 {
        self.0.radio_send(packet)
    }
    async fn radio_send_async(&mut self, packet: RadioSend<'_>) -> i32 {
        self.0.radio_send_async(packet).await
    }
    fn radio_send_classified(
        &mut self,
        packet: RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified(packet)
    }
    async fn radio_send_classified_async(
        &mut self,
        packet: RadioSend<'_>,
    ) -> crate::protocol_tx::RadioSendResult {
        self.0.radio_send_classified_async(packet).await
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LocalOutcome {
    Success,
    Failed,
    #[cfg(feature = "audio-accounting-trace")]
    NoTimeBeforeAttempt,
    FallbackOriginal,
}
impl LocalOutcome {
    fn failed(self) -> bool {
        match self {
            Self::Failed => true,
            #[cfg(feature = "audio-accounting-trace")]
            Self::NoTimeBeforeAttempt => true,
            _ => false,
        }
    }
}

impl Runtime {
    pub fn audio_state(&self) -> &AudioState {
        &self.audio_state
    }
    pub fn audio_capture_ingress(&self) -> IngressToken {
        IngressToken::from_gate(self.audio_state.boundary)
    }
    pub fn ingress_admission_gate(&self) -> IngressAdmissionGate {
        IngressAdmissionGate {
            enabled: self.audio_state.ingress_enabled && self.audio_state.boundary.enabled,
            epoch: self.audio_state.boundary.epoch,
        }
    }
    /// Select a single external 16-entry mailbox after Runtime initialization.
    /// Busy local storage is never purged or otherwise changed by this call.
    pub fn use_external_ingress(&mut self) -> Result<(), i32> {
        if self.audio_state.ingress_used != 0 {
            return Err(EBUSY);
        }
        self.audio_state.external_ingress = true;
        Ok(())
    }
    pub(crate) fn reset_ingress_mode(&mut self) {
        self.audio_state.external_ingress = false;
    }
    /// Apply completed producer/purge deltas, not anticipated queue operations.
    /// No processing counters or work/log effects are synthesized here.
    pub fn account_external_ingress(&mut self, delta: IngressDeltaAccounting) -> Result<(), i32> {
        if !self.audio_state.external_ingress {
            return Err(ENOTSUP);
        }
        let counters = &mut self.audio_state.counters;
        counters.ingress_msgq_drop = counters
            .ingress_msgq_drop
            .wrapping_add(delta.overwritten)
            .wrapping_add(delta.failed_enqueue);
        counters.tx_overwrite = counters.tx_overwrite.wrapping_add(delta.overwritten);
        counters.ingress_purge_drop = counters.ingress_purge_drop.wrapping_add(delta.purged);
        Ok(())
    }
    /// Apply the scalar count of malformed bundles rejected by the external
    /// producer. The caller takes/resets its pending count exactly once.
    pub fn account_external_ingress_validation(&mut self, count: u32) -> Result<(), i32> {
        if !self.audio_state.external_ingress {
            return Err(ENOTSUP);
        }
        self.audio_state.counters.bundle_bad =
            self.audio_state.counters.bundle_bad.wrapping_add(count);
        Ok(())
    }
    /// Process one already-dequeued external message. This is not producer ACK:
    /// ACK acknowledges completed mailbox publication, not future actor acceptance.
    /// The caller owns capacity, serialized drop-oldest publication and purge.
    /// Valid requests are trusted as already admitted; this immutable validation
    /// type cannot prove publication. Closing admission without purge preserves
    /// accepted messages. Processing rechecks epoch, ACTIVE state and local enable.
    pub fn consume_prepared_ingress(
        &mut self,
        prepared: &PreparedIngress,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if !self.audio_state.external_ingress
            || !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return ENOTSUP;
        }
        let request = match prepared.request() {
            Ok(request) => request,
            Err(_) => {
                inc(&mut self.audio_state.counters.bundle_bad);
                return EINVAL;
            }
        };
        if request.captured_epoch != self.audio_state.boundary.epoch {
            return EAGAIN;
        }
        self.process_audio_ingress(&request.entry, p)
    }
    pub fn audio_init(&mut self, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        self.reset_ingress_mode();
        self.audio_state.initialized = true;
        self.audio_state.work_pending = false;
        p.audio_effect(AudioEffect::InitWork);
        p.audio_effect(AudioEffect::SetSlotCallback);
    }
    pub fn audio_cancel_work(&mut self, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        self.audio_state.work_pending = false;
        p.audio_effect(AudioEffect::CancelWork);
    }
    pub fn audio_reset_rf_tracker(&mut self, id: u8) {
        if c::node_id_valid(id) {
            self.audio_state.rf_seq[usize::from(id)] = RfSeq16::default();
        }
    }
    pub fn audio_reset_all_rf_trackers(&mut self) {
        self.audio_state.rf_seq.fill(RfSeq16::default());
    }
    fn track_rf_e2e_sequence(&mut self, id: u8, seq: u16) {
        if !c::node_id_valid(id) {
            return;
        }
        let s = &mut self.audio_state;
        let tracker = &mut s.rf_seq[usize::from(id)];
        if !tracker.initialized {
            tracker.last = seq;
            tracker.initialized = true;
        } else {
            let delta = seq.wrapping_sub(tracker.last) as i16;
            if delta > 0 {
                if delta > 1 {
                    inc(&mut s.counters.rf_rx_gap_evt);
                    s.counters.rf_rx_gap_fr =
                        s.counters.rf_rx_gap_fr.wrapping_add((delta - 1) as u32);
                }
                tracker.last = seq;
            } else {
                inc(&mut s.counters.rf_rx_reset_evt);
            }
        }
        inc(&mut s.counters.rf_rx_frames);
    }
    pub fn audio_set_ingress_enabled(&mut self, enabled: bool, purge: bool) {
        self.audio_state.ingress_enabled = enabled;
        if purge {
            self.audio_state.purge_ingress();
        }
    }
    pub fn audio_set_local_enabled(&mut self, enabled: bool, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        if enabled == self.audio_state.boundary.enabled {
            // The S31 re-sends Resume whenever the bridge status returns; a
            // resume while already enabled must not purge live audio or bump
            // the epoch. Republish the unchanged gate: the async preflight
            // closed external ingress and only a publication reopens it.
            p.ingress_gate_changed(self.ingress_admission_gate(), false);
            return;
        }
        if !enabled {
            self.audio_state.boundary.enabled = false;
            self.audio_state.boundary.receipt_cutoff_us = p.audio_effect(AudioEffect::NowUs);
        }
        self.audio_state.boundary.epoch = self.audio_state.boundary.epoch.wrapping_add(1);
        self.audio_state.purge_ingress();
        self.audio_purge_tx_ring();
        self.audio_state.tx_depth_dbg = 0;
        self.audio_state.last_audio_in_time = 0;
        let id = self.membership.node_id;
        if c::node_id_valid(id) {
            self.session.speaker_deadlines_ms[usize::from(id)] = 0;
            self.session.speaker_active_since_ms[usize::from(id)] = 0;
            self.session.heard_bitmap &= !c::node_bit(id);
        }
        if !enabled {
            self.adaptive_local_voice(false, p);
            if self.membership.role == Role::Coordinator {
                self.audio_update_speaker_grants(p);
            }
        } else {
            self.audio_state.boundary.receipt_cutoff_us = p.audio_effect(AudioEffect::NowUs);
            self.audio_state.boundary.enabled = true;
        }
        p.ingress_gate_changed(self.ingress_admission_gate(), true);
    }
    pub fn audio_purge_tx_ring(&mut self) {
        let depth = self.audio_state.tx_depth();
        let s = &mut self.audio_state;
        s.counters.tx_purge_drop = s.counters.tx_purge_drop.wrapping_add(u32::from(depth));
        s.tx_head = 0;
        s.tx_tail = 0;
        s.relay_contention_turn = false;
        s.deferred_pending = false;
        s.deferred_seq = 0;
    }
    fn relay_queue_empty(&self) -> bool {
        self.session.relay_head == self.session.relay_tail
    }
    fn is_speaker_granted(&self, id: u8) -> bool {
        self.session.active_speaker_ids.contains(&id)
    }
    pub fn audio_clear_speaker_grants(&mut self) {
        self.session.active_speaker_ids.fill(0);
        self.session.relay_masks.fill(0);
    }
    fn relay_permitted_for_source(&self, id: u8, flags: u8) -> bool {
        flags & 4 != 0
            && self
                .session
                .active_speaker_ids
                .iter()
                .zip(self.session.relay_masks)
                .any(|(&source, mask)| {
                    source == id && mask & c::node_bit(self.membership.node_id) != 0
                })
    }
    fn note_audio_activity(&mut self, id: u8, flags: u8, p: &mut impl AdaptivePlatform) {
        if flags & 1 == 0 || !c::node_id_valid(id) {
            return;
        }
        let now = p.uptime_ms();
        let i = usize::from(id);
        self.session.heard_bitmap |= c::node_bit(id);
        if self.session.speaker_deadlines_ms[i] <= now {
            self.session.speaker_active_since_ms[i] = now;
        }
        self.session.speaker_deadlines_ms[i] =
            now.checked_add(1500).expect("C-defined audio deadline");
    }
    pub fn audio_apply_speaker_grant(&mut self, count: u8, ids: [u8; 2], masks: [u8; 2]) {
        self.audio_clear_speaker_grants();
        let count = usize::from(count.min(2));
        self.session.active_speaker_ids[..count].copy_from_slice(&ids[..count]);
        self.session.relay_masks[..count].copy_from_slice(&masks[..count]);
    }
    pub fn audio_update_speaker_grants(&mut self, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        if self.membership.role != Role::Coordinator {
            return;
        }
        let previous = self.session.active_speaker_ids;
        let now = p.uptime_ms();
        let mut since = [0; 9];
        for (id, active_since) in since.iter_mut().enumerate().skip(1) {
            if self.session.speaker_deadlines_ms[id] > now
                || a::speaker_request_active(Some(&self.protocol.adaptive), id as u8, now as u32)
            {
                *active_since = self.session.speaker_active_since_ms[id].max(1);
            }
        }
        let mut selected = [0; 2];
        let count = c::select_speakers(Some(&previous), Some(&since), &mut selected);
        let mut masks = [0; 2];
        for (i, mask) in masks.iter_mut().enumerate().take(count) {
            if a::route(
                Some(&mut self.protocol.adaptive),
                selected[i],
                now as u32,
                Some(&mut *mask),
            ) < 0
                && self.protocol.grant_hold_until_ms != 0
                && (self.protocol.grant_hold_until_ms.wrapping_sub(now as u32) as i32) > 0
            {
                for (j, &id) in previous.iter().enumerate() {
                    if id == selected[i] {
                        *mask = self.session.relay_masks[j];
                    }
                }
            }
        }
        let mut released = [0; 2];
        let mut release_count = 0;
        for id in previous {
            if id != 0 && !selected.contains(&id) {
                released[release_count] = id;
                release_count += 1;
            }
        }
        if selected != previous || masks != self.session.relay_masks {
            if release_count != 0 {
                self.send_speaker_release(Some(&released), release_count as u8);
            }
            self.session.active_speaker_ids = selected;
            self.session.relay_masks = masks;
            self.send_speaker_grant();
        }
    }
    pub fn audio_expire_speakers(&mut self, now: u32, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        if self.membership.role != Role::Coordinator {
            return;
        }
        for id in self.session.active_speaker_ids {
            if id != 0
                && self.session.speaker_deadlines_ms[usize::from(id)] <= i64::from(now)
                && !a::speaker_request_active(Some(&self.protocol.adaptive), id, now)
            {
                self.audio_update_speaker_grants(p);
                return;
            }
        }
    }
    fn enqueue_relay_packet(&mut self, data: &[u8], ttl: u8, flags: u8, received_ms: u32) -> bool {
        if ttl == 0 || !(9..=209).contains(&data.len()) || data[1] != 13 {
            return false;
        }
        let next = (self.session.relay_head + 1) % 16;
        if next == self.session.relay_tail {
            inc(&mut self.audio_state.counters.relay_ring_drop);
            self.session.relay_tail = (self.session.relay_tail + 1) % 16;
        }
        let entry = &mut self.session.relay_ring[usize::from(self.session.relay_head)];
        entry.data[..data.len()].copy_from_slice(data);
        entry.len = data.len() as u8;
        entry.queued_ms = received_ms;
        entry.data[4] = ttl;
        entry.data[5] = flags;
        self.session.relay_head = next;
        true
    }
    fn send_speaker_grant(&mut self) -> i32 {
        let grant = w::SpeakerGrant {
            speaker_count: self
                .session
                .active_speaker_ids
                .iter()
                .filter(|&&id| id != 0)
                .count() as u8,
            speaker_ids: self.session.active_speaker_ids,
            relay_masks: self.session.relay_masks,
        };
        let mut data = [0; w::SpeakerGrant::SIZE];
        grant.encode(&mut data).expect("grant layout");
        self.tx_queue_control(9, &data)
    }
    fn send_speaker_release(&mut self, ids: Option<&[u8; 2]>, count: u8) -> i32 {
        let count = count.min(2);
        let mut release = w::SpeakerRelease {
            speaker_count: count,
            speaker_ids: [0; 2],
        };
        if let Some(ids) = ids {
            release.speaker_ids[..usize::from(count)].copy_from_slice(&ids[..usize::from(count)]);
        }
        let mut data = [0; w::SpeakerRelease::SIZE];
        release.encode(&mut data).expect("release layout");
        self.tx_queue_control(10, &data)
    }
    pub fn audio_clear_relay_ring(&mut self) {
        self.session.relay_ring.fill(RelayEntry::default());
        self.session.relay_head = 0;
        self.session.relay_tail = 0;
    }
    pub fn audio_clear_speaker_activity(&mut self) {
        self.session.speaker_deadlines_ms.fill(0);
        self.session.speaker_active_since_ms.fill(0);
    }
    pub fn audio_clear_bitmaps(&mut self) {
        self.session.heard_bitmap = 0;
        self.session.relay_bitmap = 0;
    }
    pub fn audio_apply_slot_map_speakers(&mut self, map: &c::SlotMap) {
        self.audio_apply_speaker_grant(
            map.active_speaker_count,
            map.active_speaker_ids,
            map.relay_masks,
        );
    }
    pub fn audio_apply_speaker_release(&mut self, count: u8, ids: [u8; 2]) {
        for id in &ids[..usize::from(count.min(2))] {
            for i in 0..2 {
                if self.session.active_speaker_ids[i] == *id {
                    self.session.active_speaker_ids[i] = 0;
                    self.session.relay_masks[i] = 0;
                }
            }
        }
    }
    /// Direct audio RX, after outer RX has supplied readable header/backing bytes.
    /// Deliberately does not invent version/channel/outer-length validation here.
    pub fn audio_process_rx_packet(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        crate::corefuture::ready(self.audio_process_rx_packet_impl::<false>(
            data,
            rssi,
            received_us,
            p,
        ))
    }
    /// Await presence outputs before actual BridgeDeliver IO. Cancellation keeps
    /// prior RX state; no delivery result, relay enqueue, rollback or replay is synthesized.
    pub async fn audio_process_rx_packet_async(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        self.audio_process_rx_packet_impl::<true>(data, rssi, received_us, p)
            .await
    }
    pub(crate) async fn audio_process_rx_packet_impl<const ASYNC_DELIVER: bool>(
        &mut self,
        data: &[u8],
        rssi: i8,
        received_us: i64,
        p: &mut impl AdaptivePlatform,
    ) -> Result<bool, Error> {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return Err(Error::NotIntegrated);
        }
        let raw = data.get(..9).ok_or(Error::ShortPayload)?;
        let t = raw[1];
        if t == 1 {
            inc(&mut self.audio_state.counters.rf_rx_malformed);
            return Ok(true);
        }
        if t != 13 {
            return Ok(false);
        }
        if self.membership.state != State::Active {
            inc(&mut self.audio_state.counters.rf_rx_inactive_drop);
            return Ok(true);
        }
        let id = raw[2];
        let flags = raw[5];
        let ttl = raw[4];
        if !c::node_id_valid(id) {
            inc(&mut self.audio_state.counters.rf_rx_malformed);
            return Ok(true);
        }
        let adaptive = self.protocol.adaptive.snapshot();
        if adaptive.members & c::node_bit(id) == 0
            || !a::identity_valid(Some(&adaptive.nodes[usize::from(id - 1)].identity))
            || ttl != if flags & 2 != 0 { 1 } else { 2 }
        {
            inc(&mut self.audio_state.counters.rf_rx_malformed);
            return Ok(true);
        }
        if id == self.membership.node_id {
            inc(&mut self.audio_state.counters.rf_rx_self_drop);
            return Ok(true);
        }
        let len = usize::from(u16::from_le_bytes([raw[7], raw[8]]));
        let payload = data.get(9..9 + len).ok_or(Error::ShortPayload)?;
        let Some(bundle) = b::audio_bundle_parse(payload) else {
            inc(&mut self.audio_state.counters.bundle_bad);
            inc(&mut self.audio_state.counters.rf_rx_malformed);
            return Ok(true);
        };
        if !c::audio_replay_accept(
            Some(&mut self.dedupe),
            id,
            bundle.current_seq,
            false,
            p.uptime_ms() as u32,
        ) {
            inc(&mut self.audio_state.counters.rf_rx_duplicate_drop);
            return Ok(true);
        }
        inc(&mut self.audio_state.counters.bundle_rx);
        self.audio_state.counters.bundle_max_bytes =
            self.audio_state.counters.bundle_max_bytes.max(len as u32);
        if flags & 2 == 0 {
            self.update_peer_last_seen_impl::<ASYNC_DELIVER>(id, rssi, p)
                .await;
        } else {
            self.note_peer_presence(id, p);
        }
        self.note_audio_activity(id, bundle.flags & b::CURRENT_ACTIVE, p);
        if self.membership.role == Role::Coordinator {
            self.audio_update_speaker_grants(p);
        }
        self.track_rf_e2e_sequence(id, bundle.current_seq);
        inc(&mut self.audio_state.counters.rf_rx_audio_ok);
        let gate = self.audio_state.boundary;
        if gate.enabled && received_us >= gate.receipt_cutoff_us {
            let mut delivery = [0; 200];
            delivery[..len].copy_from_slice(payload);
            delivery[3] = (delivery[3] & !b::RELAYED) | if flags & 2 != 0 { b::RELAYED } else { 0 };
            let effect = AudioEffect::BridgeDeliver {
                source: id,
                bundle: &delivery[..len],
            };
            let result = if ASYNC_DELIVER {
                p.audio_effect_async(effect).await
            } else {
                p.audio_effect(effect)
            };
            if result == 0 {
                inc(&mut self.audio_state.counters.audio_fwd);
                inc(&mut self.audio_state.counters.spi_out_frames);
                inc(&mut self.audio_state.counters.spi_out_ok);
            } else {
                inc(&mut self.audio_state.counters.spi_out_drop);
            }
        }
        if ttl == 2
            && flags & 2 == 0
            && flags & 1 != 0
            && self.relay_permitted_for_source(id, flags)
            && (p.uptime_ms() as u32).wrapping_sub((received_us / 1000) as u32) <= FRAME_MS
        {
            self.enqueue_relay_packet(
                &data[..9 + len],
                ttl - 1,
                flags | 2,
                (received_us / 1000) as u32,
            );
        }
        Ok(true)
    }
    fn consume_local_audio_entries(&mut self, count: u8, success: bool) {
        #[cfg(feature = "audio-accounting-trace")]
        {
            // Observe entries present before the existing tail advance; do not
            // change its policy even for test probes requesting past empty.
            let actual = count.min(self.audio_state.tx_depth());
            self.audio_state.counters.local_entries_consumed = self
                .audio_state
                .counters
                .local_entries_consumed
                .wrapping_add(u32::from(actual));
        }
        self.audio_state.tx_tail = (self.audio_state.tx_tail + count) % 16;
        self.audio_state.tx_depth_dbg = self.audio_state.tx_depth();
        if success && self.audio_state.tx_depth_dbg == 0 {
            inc(&mut self.audio_state.counters.tx_queue_drain);
        }
    }
    fn consume_ordinary_local_entry(&mut self, outcome: LocalOutcome) {
        #[cfg(feature = "audio-accounting-trace")]
        if outcome == LocalOutcome::NoTimeBeforeAttempt {
            let actual = 1.min(self.audio_state.tx_depth());
            self.audio_state.counters.local_prefit_consumed = self
                .audio_state
                .counters
                .local_prefit_consumed
                .wrapping_add(u32::from(actual));
        }
        self.consume_local_audio_entries(1, outcome == LocalOutcome::Success);
    }
    fn local_tail_is_active_v2(&self) -> Option<u16> {
        let entry = &self.audio_state.tx[usize::from(self.audio_state.tx_tail)];
        if entry.packet_type != 13 {
            return None;
        }
        let bundle = b::audio_bundle_parse(&entry.data[..usize::from(entry.len)])?;
        (bundle.flags & b::CURRENT_ACTIVE != 0).then_some(bundle.current_seq)
    }
    fn deferred_tail_has_proven_successor(&self) -> bool {
        let s = &self.audio_state;
        if !s.deferred_pending || s.tx_depth() < 2 {
            return false;
        }
        let tail = &s.tx[usize::from(s.tx_tail)];
        let next = &s.tx[usize::from((s.tx_tail + 1) % 16)];
        if tail.packet_type != 13 || next.packet_type != 13 {
            return false;
        }
        let tail_bundle =
            b::audio_bundle_parse(&tail.data[..usize::from(tail.len)]).map(core_audio_bundle_view);
        let successor_bundle =
            b::audio_bundle_parse(&next.data[..usize::from(next.len)]).map(core_audio_bundle_view);
        c::successor_carries_prev1(
            s.deferred_seq,
            tail_bundle.as_ref(),
            successor_bundle.as_ref(),
        )
    }
    async fn transmit_relay_audio_v2_impl<const ASYNC_SEND: bool>(
        &mut self,
        packet: &mut [u8; 209],
        len: &mut usize,
        prev1: &mut bool,
        prev2: &mut bool,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        let payload_len = usize::from(u16::from_le_bytes([packet[7], packet[8]]));
        if payload_len != *len - 9 {
            inc(&mut self.audio_state.counters.bundle_bad);
            return EINVAL;
        }
        let Some(bundle) = b::audio_bundle_parse(&packet[9..*len]) else {
            inc(&mut self.audio_state.counters.bundle_bad);
            return EINVAL;
        };
        *prev1 = !bundle.previous1_data.is_empty();
        *prev2 = !bundle.previous2_data.is_empty();
        let remaining = p.audio_effect(AudioEffect::SlotRemainingUs) as u32;
        if remaining < reserve.wrapping_add(CURRENT_ONLY_AIRTIME_US) {
            return ETIME;
        }
        let (strips, count) = fit_candidates(*len, *prev1, *prev2, remaining.wrapping_sub(reserve));
        let mut bundle_len = payload_len;
        for _ in 0..strips.unwrap_or(count - 1) {
            let oldest_is_prev2 = *prev2;
            if !b::audio_bundle_strip_oldest(&mut packet[9..], &mut bundle_len) {
                inc(&mut self.audio_state.counters.bundle_bad);
                return EINVAL;
            }
            packet[7..9].copy_from_slice(&(bundle_len as u16).to_le_bytes());
            *len = 9 + bundle_len;
            if oldest_is_prev2 {
                inc(&mut self.audio_state.counters.prev2_stripped);
                *prev2 = false;
            } else {
                inc(&mut self.audio_state.counters.prev1_stripped);
                *prev1 = false;
            }
        }
        if strips.is_none() {
            inc(&mut self.audio_state.counters.bundle_late_drop);
            return ETIME;
        }
        crate::corefuture::radio_send::<ASYNC_SEND>(
            p,
            RadioSend {
                bytes: &packet[..*len],
                pipe: 0,
                address: [0xe7; 5],
            },
        )
        .await
    }
    async fn transmit_relay_entry_impl<const ASYNC_SEND: bool>(
        &mut self,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> bool {
        let queued = self.session.relay_ring[usize::from(self.session.relay_tail)];
        let mut packet = [0; 209];
        let mut len = usize::from(queued.len);
        let mut prev1 = false;
        let mut prev2 = false;
        let ret = if (9..=209).contains(&len) && queued.data[1] == 13 {
            packet[..len].copy_from_slice(&queued.data[..len]);
            self.transmit_relay_audio_v2_impl::<ASYNC_SEND>(
                &mut packet,
                &mut len,
                &mut prev1,
                &mut prev2,
                reserve,
                p,
            )
            .await
        } else {
            inc(&mut self.audio_state.counters.bundle_bad);
            EINVAL
        };
        if ret == 0 {
            inc(&mut self.audio_state.counters.tx_count);
            self.session.relay_bitmap |= c::node_bit(packet[2]);
            inc(&mut self.audio_state.counters.bundle_tx);
            if prev1 {
                inc(&mut self.audio_state.counters.prev1_forwarded);
            }
            if prev2 {
                inc(&mut self.audio_state.counters.prev2_forwarded);
            }
        } else if ret != ETIME {
            inc(&mut self.protocol.stat_tx_fail);
        }
        self.session.relay_tail = (self.session.relay_tail + 1) % 16;
        ret == 0
    }
    async fn transmit_local_entry_impl<const ASYNC_SEND: bool>(
        &mut self,
        entry: &AudioEntry,
        flags: u8,
        retain_prev1: bool,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> LocalOutcome {
        let mut ret = EINVAL;
        let mut count_e2e = false;
        if entry.packet_type == 13 {
            let mut data = entry.data;
            let mut len = usize::from(entry.len);
            data[3] &= !b::RELAYED;
            if let Some(bundle) = b::audio_bundle_parse(&data[..len]) {
                let mut prev1 = !bundle.previous1_data.is_empty();
                let mut prev2 = !bundle.previous2_data.is_empty();
                let active = bundle.flags & b::CURRENT_ACTIVE;
                let remaining = p.audio_effect(AudioEffect::SlotRemainingUs) as u32;
                if remaining < reserve.wrapping_add(CURRENT_ONLY_AIRTIME_US) {
                    #[cfg(feature = "audio-accounting-trace")]
                    {
                        inc(&mut self.audio_state.counters.local_prefit_no_time);
                        return LocalOutcome::NoTimeBeforeAttempt;
                    }
                    #[cfg(not(feature = "audio-accounting-trace"))]
                    return LocalOutcome::Failed;
                }
                let (strips, count) =
                    fit_candidates(9 + len, prev1, prev2, remaining.wrapping_sub(reserve));
                let mut stripped1 = false;
                let mut stripped2 = false;
                let mut valid = true;
                for _ in 0..strips.unwrap_or(count - 1) {
                    let oldest_is_prev2 = prev2;
                    if !oldest_is_prev2 && retain_prev1 {
                        return LocalOutcome::FallbackOriginal;
                    }
                    if !b::audio_bundle_strip_oldest(&mut data, &mut len) {
                        inc(&mut self.audio_state.counters.bundle_bad);
                        valid = false;
                        break;
                    }
                    if oldest_is_prev2 {
                        prev2 = false;
                        stripped2 = true;
                    } else {
                        prev1 = false;
                        stripped1 = true;
                    }
                }
                if stripped2 {
                    inc(&mut self.audio_state.counters.prev2_stripped);
                }
                if stripped1 {
                    inc(&mut self.audio_state.counters.prev1_stripped);
                }
                self.note_audio_activity(self.membership.node_id, active, p);
                if valid && strips.is_none() {
                    inc(&mut self.audio_state.counters.bundle_late_drop);
                    ret = ETIME;
                } else if valid {
                    inc(&mut self.audio_state.counters.rf_audio_try);
                    let seq = self.protocol.tx_seq;
                    self.protocol.tx_seq = seq.wrapping_add(1);
                    let result = self
                        .tx_send_packet_ex_classified_impl::<ASYNC_SEND>(
                            13,
                            &data[..len],
                            2,
                            flags,
                            self.membership.node_id,
                            seq,
                            p,
                        )
                        .await;
                    ret = result.code;
                    #[cfg(feature = "same-slot-audio-refit")]
                    if !retain_prev1
                        && result.code == EAGAIN
                        && result.disposition
                            == crate::protocol_tx::SubmissionDisposition::NotSubmitted
                        && b::audio_bundle_parse(&data[..len]).is_some_and(|bundle| {
                            !bundle.previous1_data.is_empty() || !bundle.previous2_data.is_empty()
                        })
                        && p.audio_effect(AudioEffect::SlotRemainingUs)
                            >= i64::from(reserve) + i64::from(CURRENT_ONLY_AIRTIME_US)
                    {
                        // No radio start was possible. Refit only this local copy,
                        // using the same sequence and the still-live handler window.
                        // The unchanged packet helper/backend recheck final admission.
                        while prev2 || prev1 {
                            if !b::audio_bundle_strip_oldest(&mut data, &mut len) {
                                valid = false;
                                break;
                            }
                            if prev2 {
                                prev2 = false;
                                inc(&mut self.audio_state.counters.prev2_stripped);
                            } else {
                                prev1 = false;
                                inc(&mut self.audio_state.counters.prev1_stripped);
                            }
                        }
                        if valid {
                            inc(&mut self.audio_state.counters.same_slot_refit_attempt);
                            ret = self
                                .tx_send_packet_ex_classified_impl::<ASYNC_SEND>(
                                    13,
                                    &data[..len],
                                    2,
                                    flags,
                                    self.membership.node_id,
                                    seq,
                                    p,
                                )
                                .await
                                .code;
                            if ret == 0 {
                                inc(&mut self.audio_state.counters.same_slot_refit_complete);
                            }
                        }
                    }
                }
                if ret == 0 {
                    inc(&mut self.audio_state.counters.bundle_tx);
                    self.audio_state.counters.bundle_max_bytes =
                        self.audio_state.counters.bundle_max_bytes.max(len as u32);
                    if prev1 {
                        inc(&mut self.audio_state.counters.prev1_forwarded);
                    }
                    if prev2 {
                        inc(&mut self.audio_state.counters.prev2_forwarded);
                    }
                    count_e2e = true;
                }
            } else {
                inc(&mut self.audio_state.counters.bundle_bad);
            }
        } else {
            inc(&mut self.audio_state.counters.bundle_bad);
        }
        if ret == 0 {
            inc(&mut self.audio_state.counters.tx_count);
            if count_e2e {
                inc(&mut self.audio_state.counters.rf_tx_frames);
            }
            inc(&mut self.audio_state.counters.rf_audio_ok);
            LocalOutcome::Success
        } else {
            if ret != ETIME {
                inc(&mut self.protocol.stat_tx_fail);
                inc(&mut self.audio_state.counters.rf_audio_fail);
            }
            LocalOutcome::Failed
        }
    }
    fn note_tx_starvation(&mut self, p: &mut impl AdaptivePlatform) {
        let total = self.audio_state.counters.tx_starvation;
        if total != 0 && total % 64 == 1 {
            let time_to_slot = p.audio_effect(AudioEffect::TimeToSlotUs) as i32;
            p.audio_effect(AudioEffect::Starvation {
                total,
                role: self.membership.role,
                node: self.membership.node_id,
                slot: self.membership.slot_index,
                depth: self.audio_state.tx_depth_dbg,
                ingress: self.audio_state.ingress_used,
                time_to_slot,
            });
        }
    }
    pub fn audio_slot_tx_handler(&mut self, _slot: u8, _frame: u32, p: &mut impl AdaptivePlatform) {
        crate::corefuture::ready(self.audio_slot_tx_handler_impl::<false>(_slot, _frame, p))
    }
    pub async fn audio_slot_tx_handler_async(
        &mut self,
        slot: u8,
        frame: u32,
        p: &mut impl AdaptivePlatform,
    ) {
        self.audio_slot_tx_handler_impl::<true>(slot, frame, p)
            .await
    }
    async fn audio_slot_tx_handler_impl<const ASYNC_SEND: bool>(
        &mut self,
        _slot: u8,
        _frame: u32,
        p: &mut impl AdaptivePlatform,
    ) {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return;
        }
        self.audio_drain_ingress(p);
        let now = p.uptime_ms() as u32;
        while !self.relay_queue_empty()
            && now.wrapping_sub(
                self.session.relay_ring[usize::from(self.session.relay_tail)].queued_ms,
            ) > FRAME_MS
        {
            inc(&mut self.audio_state.counters.relay_ring_drop);
            self.session.relay_tail = (self.session.relay_tail + 1) % 16;
        }
        let recent = self.audio_state.counters.spi_audio_in != 0
            && now.wrapping_sub(self.audio_state.last_audio_in_time) < 120;
        self.audio_state.tx_depth_dbg = self.audio_state.tx_depth();
        let local = self.membership.state == State::Active
            && self.audio_state.tx_head != self.audio_state.tx_tail;
        let relay = !self.relay_queue_empty();
        let flags = 1 | if self.is_speaker_granted(self.membership.node_id) {
            4
        } else {
            0
        };
        if local
            && relay
            && !self.audio_state.deferred_pending
            && p.audio_effect(AudioEffect::SlotRemainingUs) as u32 >= 2 * CURRENT_ONLY_AIRTIME_US
        {
            let relay_first = self.audio_state.relay_contention_turn;
            self.audio_state.relay_contention_turn = !relay_first;
            if relay_first {
                self.transmit_relay_entry_impl::<ASYNC_SEND>(CURRENT_ONLY_AIRTIME_US, p)
                    .await;
                if p.audio_effect(AudioEffect::SlotRemainingUs) as u32 >= CURRENT_ONLY_AIRTIME_US {
                    let entry = self.audio_state.tx[usize::from(self.audio_state.tx_tail)];
                    let outcome = self
                        .transmit_local_entry_impl::<ASYNC_SEND>(&entry, flags, false, 0, p)
                        .await;
                    self.consume_ordinary_local_entry(outcome);
                }
            } else {
                let entry = self.audio_state.tx[usize::from(self.audio_state.tx_tail)];
                let outcome = self
                    .transmit_local_entry_impl::<ASYNC_SEND>(
                        &entry,
                        flags,
                        false,
                        CURRENT_ONLY_AIRTIME_US,
                        p,
                    )
                    .await;
                self.consume_ordinary_local_entry(outcome);
                if p.audio_effect(AudioEffect::SlotRemainingUs) as u32 >= CURRENT_ONLY_AIRTIME_US {
                    self.transmit_relay_entry_impl::<ASYNC_SEND>(0, p).await;
                }
            }
            return;
        }
        let deferred = if local && relay && self.audio_state.relay_contention_turn {
            self.local_tail_is_active_v2()
        } else {
            None
        };
        if c::defer_local_tail(
            local,
            relay,
            self.audio_state.relay_contention_turn,
            deferred.is_some(),
        ) {
            self.audio_state.deferred_pending = true;
            self.audio_state.deferred_seq = deferred.unwrap_or(0);
            self.audio_state.relay_contention_turn = false;
            self.transmit_relay_entry_impl::<ASYNC_SEND>(0, p).await;
        } else if local {
            let proven = self.deferred_tail_has_proven_successor();
            self.audio_state.deferred_pending = false;
            self.audio_state.deferred_seq = 0;
            if proven {
                let entry = self.audio_state.tx[usize::from((self.audio_state.tx_tail + 1) % 16)];
                let outcome = self
                    .transmit_local_entry_impl::<ASYNC_SEND>(&entry, flags, true, 0, p)
                    .await;
                if outcome == LocalOutcome::Success {
                    self.consume_local_audio_entries(2, true);
                    inc(&mut self.audio_state.counters.local_deferred_recovery);
                    if relay {
                        self.audio_state.relay_contention_turn = true;
                    }
                    return;
                }
                if outcome.failed() {
                    self.audio_state.relay_contention_turn = false;
                    return;
                }
            }
            let entry = self.audio_state.tx[usize::from(self.audio_state.tx_tail)];
            let outcome = self
                .transmit_local_entry_impl::<ASYNC_SEND>(&entry, flags, false, 0, p)
                .await;
            if relay && !self.audio_state.relay_contention_turn {
                self.audio_state.relay_contention_turn = true;
            }
            self.consume_ordinary_local_entry(outcome);
        } else if relay {
            self.transmit_relay_entry_impl::<ASYNC_SEND>(0, p).await;
            self.audio_state.relay_contention_turn = false;
        } else if recent {
            inc(&mut self.audio_state.counters.tx_starvation);
            self.note_tx_starvation(p);
        }
    }
    fn process_audio_ingress(&mut self, entry: &AudioEntry, p: &mut impl AdaptivePlatform) -> i32 {
        if self.membership.state != State::Active || !self.audio_state.boundary.enabled {
            inc(&mut self.audio_state.counters.ingress_inactive_drop);
            return EAGAIN;
        }
        let data = &entry.data[..usize::from(entry.len)];
        let bundle = if entry.packet_type == 13 {
            b::audio_bundle_parse(data)
        } else {
            None
        };
        let Some(bundle) = bundle else {
            inc(&mut self.audio_state.counters.bundle_bad);
            return EINVAL;
        };
        let flags = bundle.flags & b::CURRENT_ACTIVE;
        self.audio_state.counters.bundle_max_bytes = self
            .audio_state
            .counters
            .bundle_max_bytes
            .max(u32::from(entry.len));
        let seq = c::seq16_accept(
            Some(&mut self.audio_state.spi_seq[usize::from(self.membership.node_id)]),
            bundle.current_seq,
        );
        if seq.classification == c::SeqClass::Gap {
            inc(&mut self.audio_state.counters.spi_in_gap_evt);
            self.audio_state.counters.spi_in_gap_fr = self
                .audio_state
                .counters
                .spi_in_gap_fr
                .wrapping_add(u32::from(seq.gap));
        } else if seq.classification == c::SeqClass::OldReset {
            inc(&mut self.audio_state.counters.spi_in_reset_evt);
        }
        inc(&mut self.audio_state.counters.spi_in_frames);
        inc(&mut self.audio_state.counters.spi_audio_in);
        self.audio_state.last_audio_in_time = p.uptime_ms() as u32;
        self.note_audio_activity(self.membership.node_id, flags, p);
        if flags != 0 {
            self.protocol.last_local_active_ms = p.uptime_ms() as u32;
        }
        self.adaptive_local_voice(flags != 0, p);
        if self.membership.slot_index >= 0 {
            let tts = p.audio_effect(AudioEffect::TimeToSlotUs) as i32;
            if tts >= 0 {
                inc(&mut self.audio_state.phase_hist[(tts as usize / 2000).min(9)]);
            }
        }
        if self.membership.role == Role::Coordinator {
            self.audio_update_speaker_grants(p);
        }
        let next = (self.audio_state.tx_head + 1) % 16;
        if next == self.audio_state.tx_tail {
            inc(&mut self.audio_state.counters.tx_ring_drop);
            self.audio_state.tx_tail = (self.audio_state.tx_tail + 1) % 16;
            self.audio_state.deferred_pending = false;
            self.audio_state.deferred_seq = 0;
        }
        let target = &mut self.audio_state.tx[usize::from(self.audio_state.tx_head)];
        target.data[..data.len()].copy_from_slice(data);
        target.len = entry.len;
        target.audio_flags = flags;
        target.packet_type = entry.packet_type;
        self.audio_state.tx_head = next;
        self.audio_state.tx_depth_dbg = self.audio_state.tx_depth();
        0
    }
    pub fn audio_admit_ingress(
        &mut self,
        request: IngressRequest,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if self.audio_state.external_ingress
            || !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || !p.owned_audio_wrappers()
        {
            return ENOTSUP;
        }
        let s = &mut self.audio_state;
        if !s.ingress_enabled || !s.boundary.enabled || request.captured_epoch != s.boundary.epoch {
            return EAGAIN;
        }
        if s.ingress_used == 16 {
            s.ingress_read = (s.ingress_read + 1) % 16;
            s.ingress_used -= 1;
            inc(&mut s.counters.ingress_msgq_drop);
            inc(&mut s.counters.tx_overwrite);
        }
        s.ingress[usize::from(s.ingress_write)] = request.entry;
        s.ingress_write = (s.ingress_write + 1) % 16;
        s.ingress_used += 1;
        s.work_pending = true;
        p.audio_effect(AudioEffect::SubmitWork);
        0
    }
    pub fn audio_send_legacy(&mut self, _data: &[u8], _flags: u8) -> i32 {
        ENOTSUP
    }
    pub fn audio_send_v2(&mut self, data: &[u8], p: &mut impl AdaptivePlatform) -> i32 {
        if rejects_nonstrict_hardware(p) {
            return ENOTSUP;
        }
        let token = self.audio_capture_ingress();
        self.audio_admit_prepared(token.validate(data), p)
    }
    pub fn audio_admit_prepared(
        &mut self,
        prepared: PreparedIngress,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if rejects_nonstrict_hardware(p) {
            return ENOTSUP;
        }
        match prepared.result {
            Ok(request) => self.audio_admit_ingress(request, p),
            Err(_) => {
                inc(&mut self.audio_state.counters.bundle_bad);
                EINVAL
            }
        }
    }
    pub fn audio_drain_ingress(&mut self, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        while self.audio_state.ingress_used != 0 {
            let entry = self.audio_state.ingress[usize::from(self.audio_state.ingress_read)];
            self.audio_state.ingress_read = (self.audio_state.ingress_read + 1) % 16;
            self.audio_state.ingress_used -= 1;
            self.process_audio_ingress(&entry, p);
        }
    }
    pub fn audio_ingress_work_handler(&mut self, p: &mut impl AdaptivePlatform) {
        if rejects_nonstrict_hardware(p) {
            return;
        }
        self.audio_state.work_pending = false;
        self.audio_drain_ingress(p);
    }
    pub(crate) fn audio_adaptive_effect(
        &mut self,
        e: AdaptiveEffect,
        p: &mut impl AdaptivePlatform,
    ) -> bool {
        match e {
            AdaptiveEffect::ResetTracker(id) => self.audio_reset_rf_tracker(id),
            AdaptiveEffect::ApplySlotMap(map) => self.audio_apply_slot_map_speakers(&map),
            AdaptiveEffect::ApplyGrant { count, ids, masks } => {
                self.audio_apply_speaker_grant(count, ids, masks)
            }
            AdaptiveEffect::ApplyRelease { count, ids } => {
                self.audio_apply_speaker_release(count, ids)
            }
            AdaptiveEffect::UpdateGrants => self.audio_update_speaker_grants(p),
            AdaptiveEffect::ExpireSpeakers(now) => self.audio_expire_speakers(now, p),
            _ => return false,
        }
        true
    }
    pub(crate) fn audio_membership_effect(
        &mut self,
        e: MembershipEffect,
        p: &mut impl AdaptivePlatform,
    ) -> bool {
        match e {
            MembershipEffect::ResetTracker(id) => self.audio_reset_rf_tracker(id),
            MembershipEffect::ResetTrackers => self.audio_reset_all_rf_trackers(),
            MembershipEffect::ClearRelay => self.audio_clear_relay_ring(),
            MembershipEffect::ClearActivity => self.audio_clear_speaker_activity(),
            MembershipEffect::ClearGrants => self.audio_clear_speaker_grants(),
            MembershipEffect::ClearBitmaps => self.audio_clear_bitmaps(),
            MembershipEffect::PurgeTx => self.audio_purge_tx_ring(),
            MembershipEffect::Ingress { enabled, purge } => {
                self.audio_set_ingress_enabled(enabled, purge);
                p.ingress_gate_changed(self.ingress_admission_gate(), purge);
            }
            MembershipEffect::UpdateGrants => self.audio_update_speaker_grants(p),
            _ => return false,
        }
        true
    }
}
fn core_audio_bundle_view<'a>(bundle: b::BundleView<'a>) -> c::AudioBundleView<'a> {
    c::AudioBundleView {
        previous1_data: bundle.previous1_data,
        previous2_data: bundle.previous2_data,
        current_data: bundle.current_data,
        current_seq: bundle.current_seq,
        stream_id: bundle.stream_id,
        flags: bundle.flags,
        codec: bundle.codec,
    }
}

fn fit_candidates(len: usize, prev1: bool, prev2: bool, remaining: u32) -> (Option<usize>, usize) {
    let mut candidates = [len as u32; 3];
    let mut count = 1;
    let mut len = len as u32;
    if prev2 {
        len -= 48;
        candidates[count] = len;
        count += 1;
    }
    if prev1 {
        len -= 48;
        candidates[count] = len;
        count += 1;
    }
    (
        c::fit_airtime(remaining, 100, 129, 4, 11, Some(&candidates[..count])),
        count,
    )
}

#[cfg(test)]
impl Runtime {
    pub(crate) fn audio_test_transmit_probe(
        &mut self,
        data: &[u8],
        packet_type: u8,
        kind: u8,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        crate::corefuture::ready(self.audio_test_transmit_probe_impl::<false>(
            data,
            packet_type,
            kind,
            reserve,
            p,
        ))
    }
    pub(crate) async fn audio_test_transmit_probe_async(
        &mut self,
        data: &[u8],
        packet_type: u8,
        kind: u8,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        self.audio_test_transmit_probe_impl::<true>(data, packet_type, kind, reserve, p)
            .await
    }
    async fn audio_test_transmit_probe_impl<const ASYNC_SEND: bool>(
        &mut self,
        data: &[u8],
        packet_type: u8,
        kind: u8,
        reserve: u32,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        if kind == 0 {
            let mut entry = AudioEntry {
                len: data.len() as u8,
                packet_type,
                ..AudioEntry::default()
            };
            entry.data[..data.len()].copy_from_slice(data);
            return match self
                .transmit_local_entry_impl::<ASYNC_SEND>(&entry, 1, false, reserve, p)
                .await
            {
                LocalOutcome::Success => 0,
                LocalOutcome::Failed => 1,
                #[cfg(feature = "audio-accounting-trace")]
                LocalOutcome::NoTimeBeforeAttempt => 1,
                LocalOutcome::FallbackOriginal => 2,
            };
        }
        let mut packet = [0; 209];
        let mut len = data.len();
        packet[..len].copy_from_slice(data);
        self.transmit_relay_audio_v2_impl::<ASYNC_SEND>(
            &mut packet,
            &mut len,
            &mut false,
            &mut false,
            reserve,
            p,
        )
        .await
    }
    pub(crate) fn audio_test_process(
        &mut self,
        entry: &AudioEntry,
        p: &mut impl AdaptivePlatform,
    ) -> i32 {
        self.process_audio_ingress(entry, p)
    }
    pub(crate) fn audio_test_helpers(
        &mut self,
        data: &[u8],
        id: u8,
        sequence: u16,
        p: &mut impl AdaptivePlatform,
    ) {
        self.track_rf_e2e_sequence(id, sequence);
        self.note_tx_starvation(p);
        let _ = self.relay_queue_empty();
        let _ = self.is_speaker_granted(id);
        let _ = self.relay_permitted_for_source(id, 0);
        self.note_audio_activity(id, 0, p);
        let _ = self.enqueue_relay_packet(data, 0, 0, 0);
        self.send_speaker_grant();
        self.send_speaker_release(None, 0);
        let _ = self.audio_state.tx_depth();
        self.consume_local_audio_entries(0, false);
        let _ = self.local_tail_is_active_v2();
        let _ = self.deferred_tail_has_proven_successor();
    }
}

#[cfg(test)]
#[path = "audio_tests.rs"]
mod tests;
