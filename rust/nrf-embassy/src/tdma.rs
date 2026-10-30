//! Serialized, portable counterpart of nrf_mesh/src/tdma.c, not a timer driver.
//!
//! TimingOverflow identifies unsupported timing arithmetic or a sync catch-up
//! increment that cannot fit u32, including some inputs with defined C arithmetic.
//! An error can follow partial state/effects: the owner must stop/discard the engine and
//! repair its clock before reuse. There is no invented rollback of platform calls.

pub const FRAME_US: i64 = 20_000;
pub const SLOT_US: i64 = 2_000;
pub const GUARD_US: i64 = 500;
/// Legacy CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 policy, not hardware resolution.
pub const LEGACY_CORRECTION_QUANTUM_US: i32 = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerKind {
    Frame,
    Slot,
    Control,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkKind {
    Slot,
    Control,
    Discipline,
}

pub trait Clock {
    fn now_us(&mut self) -> i64;
    /// Diagnostic clock only; must not consume the scripted timing-clock read.
    fn uptime_ms(&mut self) -> u32;
}
pub trait Timers {
    fn init_timer(&mut self, kind: TimerKind);
    fn arm_timer(&mut self, kind: TimerKind, delay_us: i64);
    /// `target_us` is absolute in the supplied TDMA clock's units; `delay_us`
    /// is the already checked/clamped legacy relative hint. The default ignores
    /// the target and forwards the hint exactly once, without reading a clock.
    ///
    /// An absolute override reads coherent physical and TDMA clock samples;
    /// the absolute target governs scheduling regardless of a legacy hint
    /// clamped to 100 or zero. If `target_us <= current_tdma_us`, including a
    /// target before the mapping anchor, queue one deferred expiration with a
    /// fresh adapter-owned operation epoch. Do not call back synchronously or
    /// add the clamped hint to a fresh clock sample. A due target need not be
    /// inverted through a mapping that cannot represent that past target.
    ///
    /// For a future target, map the absolute TDMA target to the physical clock.
    /// Conversion failure or an out-of-range target must suppress the arm through
    /// the adapter's bounded error path, without panicking, inventing a target,
    /// adding the hint, or rescaling the guard. Operation epoch allocation failure
    /// likewise uses the adapter's existing bounded fault path as a distinct fault.
    /// The adapter owns operation epochs and pending timer programs; this trait
    /// does not pass an epoch. After installing a new mapping profile, the caller
    /// separately retires old generations. This hook supplies no live clock
    /// mapping, automatic profile switch, physical guard change, or PHY deadlines.
    fn arm_timer_at(&mut self, kind: TimerKind, target_us: i64, delay_us: i64) {
        let _ = target_us;
        self.arm_timer(kind, delay_us);
    }
    fn cancel_timer(&mut self, kind: TimerKind);
}
pub trait WorkQueue {
    fn init_work(&mut self, kind: WorkKind);
    /// Exactly k_work_submit semantics: zero and negative values count as drops.
    fn submit_work(&mut self, kind: WorkKind) -> i32;
    fn cancel_work(&mut self, kind: WorkKind);
}
pub trait WsCapture {
    /// Called after the ordered timer arms, in the frame-state phase. The future
    /// IRQ adapter must serialize this capture with the paired frame publication.
    fn capture_ws(&mut self) -> Option<u32>;
    fn sample_ws(&mut self, frame: u32, absolute_edges: u32) -> Option<i32>;
}
pub trait Callbacks {
    fn slot_callback(&mut self, slot: u8, frame: u32);
    fn control_callback(&mut self, frame: u32);
}
/// One stable adapter, synchronous and non-reentrant; its caller serializes access.
pub trait TimingPlatform: Clock + Timers + WorkQueue + WsCapture + Callbacks {}
impl<T: Clock + Timers + WorkQueue + WsCapture + Callbacks> TimingPlatform for T {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidSlot,
    AlreadyRunning,
    TimingOverflow,
}
type Result<T> = core::result::Result<T, Error>;
fn add(a: i64, b: i64) -> Result<i64> {
    a.checked_add(b).ok_or(Error::TimingOverflow)
}
fn sub(a: i64, b: i64) -> Result<i64> {
    a.checked_sub(b).ok_or(Error::TimingOverflow)
}
fn pending(a: i64) -> i32 {
    a.clamp(-2000, 2000) as i32
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub slot_due: u32,
    pub slot_submit_drop: u32,
    pub slot_late_drop: u32,
    pub control_due: u32,
    pub control_submit_drop: u32,
    pub control_late_drop: u32,
    pub discipline_due: u32,
    pub discipline_submit_drop: u32,
    pub discipline_capture_drop: u32,
    pub tune_request_count: u32,
    pub tune_clamp_count: u32,
    pub correction_apply_count: u32,
    pub skipped_frame_count: u32,
    pub sync_acquire_count: u32,
    pub sync_reacquire_count: u32,
    pub sync_history_miss_count: u32,
    pub correction_applied_us: i64,
    pub correction_pending_us: i32,
    pub last_correction_us: i32,
    pub sync_frame_diff: i32,
    pub sync_phase_correction_us: i32,
    pub commanded_period_us: u32,
    pub measured_interval_us: u32,
    pub remote_rate_ppm: i16,
    pub remote_rate_valid: bool,
    pub callback_jitter_us: i32,
    pub callback_jitter_max_us: u32,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameBoundary {
    pub frame: u32,
    pub start_us: i64,
    pub valid: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RateSample {
    pub frame: u32,
    pub start_us: i64,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RemoteRateSnapshot {
    pub samples: [RateSample; 8],
    pub last: RateSample,
    pub count: u8,
    pub seen: bool,
    pub valid: bool,
    pub ppm: i16,
}
impl RemoteRateSnapshot {
    fn reset(&mut self) {
        self.count = 0;
        self.seen = false;
        self.valid = false;
        self.ppm = 0;
    }
    fn observe(&mut self, frame: u32, start_us: i64) -> Result<()> {
        if self.seen {
            let frames = frame.wrapping_sub(self.last.frame);
            if frames == 0 || frames >= 0x8000_0000 || start_us <= self.last.start_us {
                return Ok(());
            }
            if frames > 250 {
                self.reset();
            } else {
                let elapsed = (start_us as u64).wrapping_sub(self.last.start_us as u64);
                let expected = u64::from(frames) * 20_000;
                let tolerance = expected * 5000 / 1_000_000 + 300;
                if elapsed < expected - tolerance || elapsed > expected + tolerance {
                    return Ok(());
                }
            }
        }
        self.last = RateSample { frame, start_us };
        self.seen = true;
        if self.count != 0
            && frame.wrapping_sub(self.samples[usize::from(self.count - 1)].frame) < 10
        {
            return Ok(());
        }
        while self.count >= 2 && frame.wrapping_sub(self.samples[1].frame) >= 50 {
            self.shift();
        }
        if self.count == 8 {
            self.shift();
        }
        self.samples[usize::from(self.count)] = self.last;
        self.count += 1;
        let frames = frame.wrapping_sub(self.samples[0].frame);
        if frames >= 10 {
            let elapsed = (start_us as u64).wrapping_sub(self.samples[0].start_us as u64) as i64;
            let expected = i64::from(frames) * FRAME_US;
            let numerator = sub(elapsed, expected)?
                .checked_mul(1_000_000)
                .ok_or(Error::TimingOverflow)?;
            self.ppm = (numerator / expected).clamp(-5000, 5000) as i16;
            self.valid = true;
        }
        Ok(())
    }
    fn shift(&mut self) {
        for i in 1..usize::from(self.count) {
            self.samples[i - 1] = self.samples[i];
        }
        self.count -= 1;
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorkMetadata {
    pub generation: u32,
    pub frame: u32,
    pub start_us: i64,
    pub deadline_us: i64,
}
/// Detached semantic state. Modifying this value cannot modify the engine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub frame_counter: u32,
    pub frame_start_us: i64,
    pub pending_correction_us: i32,
    pub rate_whole_us: i32,
    pub rate_fraction: i32,
    pub rate_residual: i64,
    pub slot_index: i8,
    pub running: bool,
    pub synchronized: bool,
    pub local_clock_source: bool,
    pub timer_quiesced: bool,
    pub generation: u32,
    pub next_frame_deadline_us: i64,
    pub last_callback_us: i64,
    pub slot: WorkMetadata,
    pub control: WorkMetadata,
    pub stats: Stats,
    pub remote_rate: RemoteRateSnapshot,
    pub discipline_frame: u32,
    pub discipline_edge_count: u32,
    pub history: [FrameBoundary; 16],
    pub history_head: u8,
    pub slot_callback_enabled: bool,
    pub control_callback_enabled: bool,
    pub reacquire_diag_last_ms: u32,
    pub reacquire_diag_count: u32,
}
pub struct Tdma {
    state: Snapshot,
}
impl Default for Tdma {
    fn default() -> Self {
        Self::new()
    }
}
impl Tdma {
    pub fn new() -> Self {
        Self {
            state: Snapshot {
                frame_counter: 0,
                frame_start_us: 0,
                pending_correction_us: 0,
                rate_whole_us: 0,
                rate_fraction: 0,
                rate_residual: 0,
                slot_index: -1,
                running: false,
                synchronized: false,
                local_clock_source: false,
                timer_quiesced: false,
                generation: 0,
                next_frame_deadline_us: 0,
                last_callback_us: 0,
                slot: WorkMetadata::default(),
                control: WorkMetadata::default(),
                stats: Stats::default(),
                remote_rate: RemoteRateSnapshot::default(),
                discipline_frame: 0,
                discipline_edge_count: 0,
                history: [FrameBoundary::default(); 16],
                history_head: 0,
                slot_callback_enabled: false,
                control_callback_enabled: false,
                reacquire_diag_last_ms: 0,
                reacquire_diag_count: 0,
            },
        }
    }
    pub fn init(&mut self, p: &mut impl TimingPlatform) {
        for k in [TimerKind::Frame, TimerKind::Slot, TimerKind::Control] {
            p.init_timer(k);
        }
        for k in [WorkKind::Slot, WorkKind::Control, WorkKind::Discipline] {
            p.init_work(k);
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        self.state
    }
    pub fn stats(&self) -> Stats {
        self.state.stats
    }
    pub fn frame_counter(&self) -> u32 {
        self.state.frame_counter
    }
    pub fn set_slot_callback(&mut self, enabled: bool) {
        self.state.slot_callback_enabled = enabled;
    }
    pub fn set_control_callback(&mut self, enabled: bool) {
        self.state.control_callback_enabled = enabled;
    }
    pub fn set_callbacks(&mut self, slot: bool, control: bool) {
        self.set_slot_callback(slot);
        self.set_control_callback(control);
    }
    pub fn set_slot_index(&mut self, slot: i8) {
        if (0..8).contains(&slot) {
            self.state.slot_index = slot;
        }
    }
    fn invalidate_history(&mut self) {
        for h in &mut self.state.history {
            h.valid = false;
        }
        self.state.history_head = 0;
    }
    fn record_history(&mut self, frame: u32, start_us: i64) {
        self.state.history[usize::from(self.state.history_head)] = FrameBoundary {
            frame,
            start_us,
            valid: true,
        };
        self.state.history_head = (self.state.history_head + 1) % 16;
    }
    fn lookup_history(&self, frame: u32) -> Option<i64> {
        self.state
            .history
            .iter()
            .find(|h| h.valid && h.frame == frame)
            .map(|h| h.start_us)
    }
    fn set_rate(&mut self, ppm: i32) {
        let scaled = ppm.clamp(-5000, 5000) * FRAME_US as i32;
        self.state.rate_whole_us = scaled / 1_000_000;
        self.state.rate_fraction = scaled % 1_000_000;
    }
    fn reset_remote(&mut self) {
        self.state.remote_rate.reset();
        self.state.stats.remote_rate_ppm = 0;
        self.state.stats.remote_rate_valid = false;
    }
    pub fn start(
        &mut self,
        slot: i8,
        synchronized: bool,
        p: &mut impl TimingPlatform,
    ) -> Result<()> {
        if !(0..8).contains(&slot) {
            return Err(Error::InvalidSlot);
        }
        if self.state.running {
            return Err(Error::AlreadyRunning);
        }
        self.state.slot_index = slot;
        self.reset_remote();
        self.state.reacquire_diag_count = 0;
        self.state.reacquire_diag_last_ms = 0;
        self.state.frame_counter = 0;
        self.state.frame_start_us = p.now_us();
        self.invalidate_history();
        self.record_history(0, self.state.frame_start_us);
        self.state.next_frame_deadline_us = add(self.state.frame_start_us, FRAME_US)?;
        self.state.last_callback_us = 0;
        self.state.pending_correction_us = 0;
        self.set_rate(0);
        self.state.rate_residual = 0;
        self.state.stats.correction_pending_us = 0;
        self.state.stats.last_correction_us = 0;
        self.state.stats.commanded_period_us = FRAME_US as u32;
        self.state.stats.measured_interval_us = 0;
        self.state.stats.callback_jitter_us = 0;
        self.state.generation = self.state.generation.wrapping_add(1);
        self.state.running = true;
        self.state.synchronized = synchronized;
        self.state.local_clock_source = false;
        self.state.timer_quiesced = false;
        let delay = sub(self.state.next_frame_deadline_us, p.now_us())?
            .max(i64::from(LEGACY_CORRECTION_QUANTUM_US));
        p.arm_timer_at(TimerKind::Frame, self.state.next_frame_deadline_us, delay);
        Ok(())
    }
    fn cancel_all(p: &mut impl TimingPlatform) {
        for k in [TimerKind::Frame, TimerKind::Slot, TimerKind::Control] {
            p.cancel_timer(k);
        }
        for k in [WorkKind::Slot, WorkKind::Control, WorkKind::Discipline] {
            p.cancel_work(k);
        }
    }
    pub fn stop(&mut self, p: &mut impl TimingPlatform) {
        if !self.state.running {
            return;
        }
        self.state.running = false;
        self.reset_remote();
        self.state.synchronized = false;
        self.state.local_clock_source = false;
        self.state.timer_quiesced = true;
        self.state.generation = self.state.generation.wrapping_add(1);
        Self::cancel_all(p);
    }
    pub fn set_clock_source(&mut self, local: bool) {
        self.reset_remote();
        self.set_rate(0);
        self.state.rate_residual = 0;
        self.state.local_clock_source = local;
    }
    pub fn clock_snapshot(&self, p: &mut impl TimingPlatform) -> Result<Option<(u32, u16)>> {
        let elapsed = sub(p.now_us(), self.state.frame_start_us)?;
        Ok(
            (self.state.running && self.state.synchronized && (0..FRAME_US).contains(&elapsed))
                .then_some((self.state.frame_counter, elapsed as u16)),
        )
    }
    /// Phase is valid only in the same nominal frame window as clock_snapshot.
    pub fn frame_phase(&self, p: &mut impl TimingPlatform) -> Result<Option<u16>> {
        Ok(self.clock_snapshot(p)?.map(|(_, phase)| phase))
    }
    pub fn time_to_slot_us(&self, p: &mut impl TimingPlatform) -> Result<i32> {
        let s = &self.state;
        if s.slot_index < 0 || !s.running {
            return Ok(-1);
        }
        let now = p.now_us();
        let offset = i64::from(s.slot_index) * SLOT_US;
        let start = add(s.frame_start_us, offset)?;
        Ok(if now < start {
            sub(start, now)?
        } else {
            add(sub(s.next_frame_deadline_us, now)?.max(0), offset)?
        } as i32)
    }
    pub fn current_slot_remaining_us(&self, p: &mut impl TimingPlatform) -> Result<u32> {
        let now = p.now_us();
        let s = &self.state;
        if !s.running || s.slot_index < 0 || now < s.slot.start_us {
            return Ok(0);
        }
        Ok(sub(s.slot.deadline_us, now)?.clamp(0, i64::from(u32::MAX)) as u32)
    }
    pub fn tune_timing(&mut self, offset: i32) {
        if !self.state.running {
            return;
        }
        let requested = i64::from(self.state.pending_correction_us) + i64::from(offset);
        let bounded = pending(requested);
        let d = &mut self.state.stats;
        d.tune_request_count = d.tune_request_count.wrapping_add(1);
        if i64::from(bounded) != requested {
            d.tune_clamp_count = d.tune_clamp_count.wrapping_add(1);
        }
        self.state.pending_correction_us = bounded;
        d.correction_pending_us = bounded;
    }
    /// Dispatch expiry separately from work execution. No invented token guards.
    pub fn timer_expired(&mut self, kind: TimerKind, p: &mut impl TimingPlatform) -> Result<()> {
        match kind {
            TimerKind::Frame => self.frame_timer(p)?,
            TimerKind::Slot => {
                self.state.stats.slot_due = self.state.stats.slot_due.wrapping_add(1);
                if p.submit_work(WorkKind::Slot) <= 0 {
                    self.state.stats.slot_submit_drop =
                        self.state.stats.slot_submit_drop.wrapping_add(1);
                }
            }
            TimerKind::Control => {
                self.state.stats.control_due = self.state.stats.control_due.wrapping_add(1);
                if p.submit_work(WorkKind::Control) <= 0 {
                    self.state.stats.control_submit_drop =
                        self.state.stats.control_submit_drop.wrapping_add(1);
                }
            }
        }
        Ok(())
    }
    pub fn work_executed(&mut self, kind: WorkKind, p: &mut impl TimingPlatform) -> Result<()> {
        match kind {
            WorkKind::Discipline => {
                if self.state.running && self.state.local_clock_source {
                    if let Some(correction) = p.sample_ws(
                        self.state.discipline_frame,
                        self.state.discipline_edge_count,
                    ) {
                        self.tune_timing(correction);
                    }
                }
            }
            WorkKind::Slot | WorkKind::Control => {
                let now = p.now_us();
                let s = &mut self.state;
                let m = if kind == WorkKind::Slot {
                    s.slot
                } else {
                    s.control
                };
                let current = s.running
                    && m.generation == s.generation
                    && (kind == WorkKind::Slot || s.synchronized);
                if current && now > m.deadline_us {
                    let count = if kind == WorkKind::Slot {
                        &mut s.stats.slot_late_drop
                    } else {
                        &mut s.stats.control_late_drop
                    };
                    *count = count.wrapping_add(1);
                }
                if current && now >= m.start_us && now <= m.deadline_us {
                    if kind == WorkKind::Slot && s.slot_callback_enabled && s.slot_index >= 0 {
                        p.slot_callback(s.slot_index as u8, m.frame);
                    }
                    if kind == WorkKind::Control && s.control_callback_enabled {
                        p.control_callback(m.frame);
                    }
                }
            }
        }
        Ok(())
    }
    fn frame_timer(&mut self, p: &mut impl TimingPlatform) -> Result<()> {
        let now = p.now_us();
        if !self.state.running || self.state.timer_quiesced {
            return Ok(());
        }
        let scheduled = self.state.next_frame_deadline_us;
        let jitter = sub(now, scheduled)?;
        let absolute = jitter.checked_abs().ok_or(Error::TimingOverflow)? as u64;
        self.state.stats.callback_jitter_us =
            jitter.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
        self.state.stats.callback_jitter_max_us = self
            .state
            .stats
            .callback_jitter_max_us
            .max(absolute.min(u64::from(u32::MAX)) as u32);
        if self.state.last_callback_us != 0 {
            self.state.stats.measured_interval_us =
                sub(now, self.state.last_callback_us)?.clamp(0, i64::from(u32::MAX)) as u32;
        }
        self.state.last_callback_us = now;
        self.state.frame_counter = self.state.frame_counter.wrapping_add(1);
        self.record_history(self.state.frame_counter, scheduled);
        let mut start = scheduled;
        if add(scheduled, FRAME_US)? <= now {
            let skipped = (sub(now, scheduled)? / FRAME_US) as u32;
            start = add(start, i64::from(skipped) * FRAME_US)?;
            if skipped >= 16 {
                self.invalidate_history();
                self.state.frame_counter = self.state.frame_counter.wrapping_add(skipped);
                self.record_history(self.state.frame_counter, start);
            } else {
                for i in 0..skipped {
                    self.state.frame_counter = self.state.frame_counter.wrapping_add(1);
                    self.record_history(
                        self.state.frame_counter,
                        add(scheduled, i64::from(i + 1) * FRAME_US)?,
                    );
                }
            }
            self.state.stats.skipped_frame_count =
                self.state.stats.skipped_frame_count.wrapping_add(skipped);
        }
        let s = &mut self.state;
        let mut rate = s.rate_whole_us;
        s.rate_residual = add(s.rate_residual, i64::from(s.rate_fraction))?;
        if s.rate_residual >= 1_000_000 {
            rate += 1;
            s.rate_residual -= 1_000_000;
        } else if s.rate_residual <= -1_000_000 {
            rate -= 1;
            s.rate_residual += 1_000_000;
        }
        s.pending_correction_us = pending(i64::from(s.pending_correction_us) + i64::from(rate));
        let mut applied = if s.pending_correction_us >= LEGACY_CORRECTION_QUANTUM_US {
            LEGACY_CORRECTION_QUANTUM_US
        } else if s.pending_correction_us <= -LEGACY_CORRECTION_QUANTUM_US {
            -LEGACY_CORRECTION_QUANTUM_US
        } else {
            0
        };
        s.pending_correction_us -= applied;
        let nominal = add(start, FRAME_US)?;
        let mut next = add(nominal, i64::from(applied))?;
        if next <= now {
            s.pending_correction_us += applied;
            applied = 0;
            next = nominal;
        }
        s.stats.correction_pending_us = s.pending_correction_us;
        s.stats.last_correction_us = applied;
        s.stats.commanded_period_us = (FRAME_US + i64::from(applied)) as u32;
        s.stats.correction_applied_us = add(s.stats.correction_applied_us, i64::from(applied))?;
        if applied != 0 {
            s.stats.correction_apply_count = s.stats.correction_apply_count.wrapping_add(1);
        }
        s.frame_start_us = start;
        s.next_frame_deadline_us = next;
        let slot = if s.synchronized { s.slot_index } else { -1 };
        s.slot.generation = s.generation;
        s.slot.frame = s.frame_counter;
        if slot >= 0 {
            s.slot.start_us = add(start, i64::from(slot) * SLOT_US)?;
            s.slot.deadline_us = add(s.slot.start_us, SLOT_US - GUARD_US)?;
        }
        s.control = WorkMetadata {
            generation: s.generation,
            frame: s.frame_counter,
            start_us: add(start, 16_000)?,
            deadline_us: add(start, 17_500)?,
        };
        if slot >= 0 && now <= s.slot.deadline_us {
            p.arm_timer_at(
                TimerKind::Slot,
                s.slot.start_us,
                sub(s.slot.start_us, now)?.max(0),
            );
        } else if slot >= 0 {
            s.stats.slot_late_drop = s.stats.slot_late_drop.wrapping_add(1);
        }
        if s.synchronized && now <= s.control.deadline_us {
            p.arm_timer_at(
                TimerKind::Control,
                s.control.start_us,
                sub(s.control.start_us, now)?.max(0),
            );
        } else if s.synchronized {
            s.stats.control_late_drop = s.stats.control_late_drop.wrapping_add(1);
        }
        p.arm_timer_at(TimerKind::Frame, next, sub(next, now)?);
        if s.local_clock_source {
            s.stats.discipline_due = s.stats.discipline_due.wrapping_add(1);
            if let Some(edges) = p.capture_ws() {
                s.discipline_frame = s.frame_counter;
                s.discipline_edge_count = edges;
                if p.submit_work(WorkKind::Discipline) <= 0 {
                    s.stats.discipline_submit_drop = s.stats.discipline_submit_drop.wrapping_add(1);
                }
            } else {
                s.stats.discipline_capture_drop = s.stats.discipline_capture_drop.wrapping_add(1);
            }
        }
        Ok(())
    }
    fn log_reacquire(&mut self, p: &mut impl TimingPlatform) {
        let now = p.uptime_ms();
        let s = &mut self.state;
        let old = s.reacquire_diag_count;
        s.reacquire_diag_count = old.wrapping_add(1);
        if old >= 3 && now.wrapping_sub(s.reacquire_diag_last_ms) < 1000 {
            return;
        }
        s.reacquire_diag_last_ms = now;
    }
    fn advance_sync(frame: &mut u32, start: &mut i64, next: &mut i64, now: i64) -> Result<()> {
        if *next <= now {
            let elapsed_wide = add(sub(now, *next)? / FRAME_US, 1)?;
            let elapsed = u32::try_from(elapsed_wide).map_err(|_| Error::TimingOverflow)?;
            let advanced_start = add(*next, i64::from(elapsed - 1) * FRAME_US)?;
            let advanced_next = add(*next, i64::from(elapsed) * FRAME_US)?;
            if advanced_next <= now {
                return Err(Error::TimingOverflow);
            }
            *frame = frame.wrapping_add(elapsed);
            *start = advanced_start;
            *next = advanced_next;
        }
        Ok(())
    }
    fn acquire_sync(
        &mut self,
        mut frame: u32,
        ppm: i16,
        mut start: i64,
        reacquire: bool,
        p: &mut impl TimingPlatform,
    ) -> Result<()> {
        let now = p.now_us();
        let mut next = add(start, FRAME_US)?;
        Self::advance_sync(&mut frame, &mut start, &mut next, now)?;
        if !self.state.running {
            return Ok(());
        }
        self.state.timer_quiesced = true;
        self.state.synchronized = false;
        self.state.generation = self.state.generation.wrapping_add(1);
        let generation = self.state.generation;
        Self::cancel_all(p);
        if !self.state.running || self.state.generation != generation {
            return Ok(());
        }
        let now = p.now_us();
        Self::advance_sync(&mut frame, &mut start, &mut next, now)?;
        self.state.frame_counter = frame;
        self.state.frame_start_us = start;
        self.invalidate_history();
        self.record_history(frame, start);
        self.state.next_frame_deadline_us = next;
        self.state.last_callback_us = 0;
        self.state.pending_correction_us = 0;
        self.state.stats.correction_pending_us = 0;
        self.state.stats.last_correction_us = 0;
        self.state.stats.commanded_period_us = FRAME_US as u32;
        self.state.stats.sync_phase_correction_us = 0;
        if !reacquire {
            self.state.stats.sync_frame_diff = 0;
        }
        self.set_rate(i32::from(ppm));
        self.state.rate_residual = 0;
        self.state.synchronized = true;
        self.state.timer_quiesced = false;
        let count = if reacquire {
            &mut self.state.stats.sync_reacquire_count
        } else {
            &mut self.state.stats.sync_acquire_count
        };
        *count = count.wrapping_add(1);
        p.arm_timer_at(TimerKind::Frame, next, sub(next, now)?);
        Ok(())
    }
    pub fn sync(
        &mut self,
        frame: u32,
        drift_ppm: i16,
        start: i64,
        p: &mut impl TimingPlatform,
    ) -> Result<()> {
        if !self.state.running {
            return Ok(());
        }
        if !self.state.local_clock_source {
            self.state.remote_rate.observe(frame, start)?;
        }
        self.state.stats.remote_rate_ppm = self.state.remote_rate.ppm;
        self.state.stats.remote_rate_valid = self.state.remote_rate.valid;
        let ppm = if self.state.remote_rate.valid {
            self.state.remote_rate.ppm
        } else {
            drift_ppm
        };
        if !self.state.synchronized {
            return self.acquire_sync(frame, ppm, start, false, p);
        }
        let diff = frame.wrapping_sub(self.state.frame_counter) as i32;
        self.state.stats.sync_frame_diff = diff;
        if !(-4..=4).contains(&diff) {
            if let Some(local) = self.lookup_history(frame) {
                sub(start, local)?;
            }
            self.log_reacquire(p);
            return self.acquire_sync(frame, ppm, start, true, p);
        }
        let Some(local) = self.lookup_history(frame) else {
            self.state.stats.sync_history_miss_count =
                self.state.stats.sync_history_miss_count.wrapping_add(1);
            self.state.stats.sync_phase_correction_us = 0;
            return Ok(());
        };
        let phase = pending(sub(start, local)?);
        self.state.stats.sync_phase_correction_us = phase;
        if !(-500..=500).contains(&phase) {
            self.log_reacquire(p);
            return self.acquire_sync(frame, ppm, start, true, p);
        }
        self.state.pending_correction_us = phase;
        self.state.stats.correction_pending_us = phase;
        self.set_rate(i32::from(ppm));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{Error, Tdma, FRAME_US};

    #[test]
    fn advance_sync_bounded_max_and_rejected_wide_increment() {
        let max = i64::from(u32::MAX);
        let mut frame = 1;
        let mut next = -(max - 1) * FRAME_US;
        let mut start = next - FRAME_US;
        Tdma::advance_sync(&mut frame, &mut start, &mut next, 0).unwrap();
        assert_eq!((frame, start, next), (0, 0, FRAME_US));

        // The quotient fits u32::MAX, but the required +1 increment does not.
        // All timestamp arithmetic for this input is defined; narrowing would
        // otherwise reproduce C's wrapped catch-up rather than a future deadline.
        let mut frame = 1;
        let mut next = -max * FRAME_US;
        let mut start = next - FRAME_US;
        let before = (frame, start, next);
        assert_eq!(
            Tdma::advance_sync(&mut frame, &mut start, &mut next, 0),
            Err(Error::TimingOverflow)
        );
        assert_eq!((frame, start, next), before);
    }

    #[test]
    fn advance_sync_rejects_timestamp_overflow_without_committing_outputs() {
        let mut frame = 1;
        let mut next = i64::MAX - 100;
        let mut start = next - FRAME_US;
        let before = (frame, start, next);
        let now = next;
        assert_eq!(
            Tdma::advance_sync(&mut frame, &mut start, &mut next, now),
            Err(Error::TimingOverflow)
        );
        assert_eq!((frame, start, next), before);
    }
}
