use crate::platform::{
    accepts_protocol_algorithms, allows_fixture_injection, AudioBoundary, Effect, Integration,
    Platform, Reply,
};
use openhelmet_mesh::core::{dedupe_reset, Dedupe};

pub const EINVAL: i32 = -22;
pub const EBUSY: i32 = -16;
pub const EALREADY: i32 = -114;
pub const ENOTSUP: i32 = -95;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle,
    Scanning,
    Joining,
    Active,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Role {
    #[default]
    None,
    Coordinator,
    Participant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandKind {
    Start = 1,
    Stop = 2,
    Pause = 4,
    Resume = 5,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub kind: CommandKind,
    pub generation: u8,
    pub talk_channel: u8,
}

impl Command {
    pub fn ack_payload(self, result: i32) -> [u8; 3] {
        [
            self.kind as u8,
            self.generation,
            if result == 0 { 0 } else { 255 },
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Membership {
    pub state: State,
    pub role: Role,
    pub peer_count: u8,
    pub membership_known: bool,
    pub node_id: u8,
    pub slot_index: i8,
    pub coordinator_id: u8,
}

impl Default for Membership {
    fn default() -> Self {
        Self {
            state: State::Idle,
            role: Role::None,
            peer_count: 0,
            membership_known: false,
            node_id: 0,
            slot_index: -1,
            coordinator_id: 0,
        }
    }
}

impl Membership {
    pub fn bridge_peer_count(self) -> u8 {
        if self.state == State::Active && self.role == Role::Participant && !self.membership_known {
            255
        } else {
            self.peer_count
        }
    }
    /// Bridge v6 status (LC3, 20 ms); negotiation/transport remains external.
    pub fn status_v6(self) -> [u8; 10] {
        [
            self.role as u8,
            self.peer_count,
            self.node_id,
            6,
            self.state as u8,
            self.slot_index as u8,
            self.coordinator_id,
            0xa5,
            2,
            20,
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Timer {
    Scan,
    Join,
    Status,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimerToken {
    pub timer: Timer,
    pub generation: u64,
    pub deadline_us: i64,
}

/// External membership policy supplies owned mutations; it is not reduced here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerDirective {
    Cancel(Timer),
    Schedule(Timer, u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MembershipTransition {
    Unchanged,
    Apply {
        next: Membership,
        timers: [Option<TimerDirective>; 3],
    },
    /// Qualification is external; only the source-ordered lifecycle is applied.
    CoordinatorLost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Command(Command),
    RequestStatus,
    DispatchCommands,
    Timer(TimerToken),
    /// Only accepted from the simulated backend until membership is integrated.
    Membership(Membership),
    /// Harness-only snapshot injection, with no activation side effects.
    InjectMembership(Membership),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub membership: Membership,
    pub membership_rx: crate::protocol_membership::MembershipRx,
    pub talk_channel: u8,
    pub audio: AudioBoundary,
    pub auto_ticks: u32,
    pub status_log_decim: u32,
    pub session_empty: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Peer {
    pub(crate) node_id: u8,
    pub(crate) address: [u8; 5],
    pub(crate) slot_index: i8,
    pub(crate) last_seen_ms: i64,
    battery_pct: u8,
    pub(crate) rssi_dbm: i8,
    peer_count: u8,
    fw_version: u8,
    temperature_c: i8,
    heard_bitmap: u8,
    relay_bitmap: u8,
    pub(crate) active: bool,
    pub(crate) announced: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RelayEntry {
    pub data: [u8; 209],
    pub len: u8,
    pub queued_ms: u32,
}

impl Default for RelayEntry {
    fn default() -> Self {
        Self {
            data: [0; 209],
            len: 0,
            queued_ms: 0,
        }
    }
}

#[derive(Default, PartialEq, Eq)]
pub(crate) struct Session {
    pub(crate) peers: [Peer; 8],
    pub(crate) relay_ring: [RelayEntry; 16],
    pub(crate) control_ring: [RelayEntry; 32],
    pub(crate) speaker_deadlines_ms: [i64; 9],
    pub(crate) speaker_active_since_ms: [i64; 9],
    pub(crate) active_speaker_ids: [u8; 2],
    pub(crate) relay_masks: [u8; 2],
    pub(crate) relay_head: u8,
    pub(crate) relay_tail: u8,
    pub(crate) control_head: u8,
    pub(crate) control_tail: u8,
    pub(crate) heard_bitmap: u8,
    pub(crate) relay_bitmap: u8,
    skip_count: u32,
}

impl Session {
    fn is_empty(&self) -> bool {
        self.peers.iter().all(|p| *p == Peer::default())
            && self
                .relay_ring
                .iter()
                .chain(self.control_ring.iter())
                .all(|e| *e == RelayEntry::default())
            && self.speaker_deadlines_ms == [0; 9]
            && self.speaker_active_since_ms == [0; 9]
            && self.active_speaker_ids == [0; 2]
            && self.relay_masks == [0; 2]
            && [
                self.relay_head,
                self.relay_tail,
                self.control_head,
                self.control_tail,
                self.heard_bitmap,
                self.relay_bitmap,
            ] == [0; 6]
            && self.skip_count == 0
    }
}

/// Single owner; no ISR entry, mutexes, globals, allocator, or HAL.
pub struct Runtime {
    pub(crate) membership: Membership,
    pub(crate) talk_channel: u8,
    pub(crate) address: [u8; 5],
    pending: Option<Command>,
    status_pending: bool,
    // Last BridgeDelivery intent (bridge default is enabled); lets a repeated
    // Pause/Resume skip the purge/flush sequence it would otherwise replay.
    pub(crate) bridge_delivery_enabled: bool,
    // Rust-only strict capability guard, not original C context state.
    owned_protocol_initialized: bool,
    pub(crate) audio_state: crate::protocol_audio::AudioState,
    pub(crate) rx_state: crate::protocol_rx::RxState,
    pub(crate) timers: [Option<TimerToken>; 3],
    generations: [u64; 3],
    pub(crate) dedupe: Dedupe,
    pub(crate) session: Session,
    pub(crate) protocol: crate::protocol_membership::ProtocolState,
    auto_ticks: u32,
    status_log_decim: u32,
}

impl Default for Runtime {
    fn default() -> Self {
        Self {
            membership: Membership::default(),
            talk_channel: 1,
            address: [0; 5],
            pending: None,
            status_pending: false,
            bridge_delivery_enabled: true,
            owned_protocol_initialized: false,
            audio_state: crate::protocol_audio::AudioState::default(),
            rx_state: crate::protocol_rx::RxState::default(),
            timers: [None; 3],
            generations: [0; 3],
            dedupe: Dedupe::default(),
            session: Session::default(),
            protocol: crate::protocol_membership::ProtocolState::default(),
            auto_ticks: 0,
            status_log_decim: 0,
        }
    }
}

fn value(reply: Reply) -> i32 {
    if let Reply::Value(value) = reply {
        value
    } else {
        ENOTSUP
    }
}

impl Runtime {
    pub(crate) fn legacy_adaptive_status(
        membership: Membership,
        p: &mut impl crate::protocol_adaptive::AdaptivePlatform,
    ) -> crate::protocol_adaptive::AdaptiveReply {
        let mut scratch = Self {
            membership,
            ..Self::default()
        };
        scratch.ae(
            p,
            crate::protocol_adaptive::AdaptiveEffect::Status(membership),
        )
    }

    async fn output_impl<const ASYNC_SEND: bool>(p: &mut impl Platform, effect: Effect) -> Reply {
        if ASYNC_SEND {
            p.apply_async(effect).await
        } else {
            p.apply(effect)
        }
    }
    fn audio_effect(&mut self, e: Effect, p: &mut impl Platform) -> Reply {
        if p.owned_protocol_algorithms() {
            if let Some(reply) = p.dispatch_protocol(self, e) {
                return reply;
            }
            if crate::protocol_runtime::owned_effect(e) {
                return Reply::Value(ENOTSUP);
            }
        }
        if let Some(reply) = p.dispatch_rx(e, self) {
            reply
        } else {
            p.dispatch_audio(e, self).unwrap_or_else(|| p.apply(e))
        }
    }
    fn required_owned_effect(&mut self, e: Effect, p: &mut impl Platform) -> Result<(), i32> {
        let reply = self.audio_effect(e, p);
        match reply {
            Reply::Value(error) if p.owned_protocol_algorithms() && error < 0 => Err(error),
            _ => Ok(()),
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            membership: self.membership,
            membership_rx: self.protocol.membership_rx,
            talk_channel: self.talk_channel,
            audio: self.audio_state.boundary,
            auto_ticks: self.auto_ticks,
            status_log_decim: self.status_log_decim,
            session_empty: self.session.is_empty(),
        }
    }

    pub fn timer(&self, timer: Timer) -> Option<TimerToken> {
        self.timers[timer as usize]
    }

    pub fn init(&mut self, p: &mut impl Platform) -> i32 {
        self.owned_protocol_initialized = false;
        if p.integration() == Integration::HardwareUnqualified && !p.owned_protocol_algorithms() {
            return ENOTSUP;
        }
        self.reset_ingress_mode();
        for timer in [Timer::Scan, Timer::Join, Timer::Status] {
            if p.owned_protocol_algorithms() {
                self.timers[timer as usize] = None;
            }
            p.apply(Effect::InitTimer(timer));
        }
        self.audio_effect(Effect::InitCommand, p);
        for effect in [Effect::BindMembershipWork, Effect::AudioInit] {
            if let Err(error) = self.required_owned_effect(effect, p) {
                return error;
            }
        }
        self.audio_state.boundary.enabled = true;
        p.ingress_gate_changed(self.ingress_admission_gate(), false);
        if let Err(error) = self.required_owned_effect(Effect::RxInit, p) {
            return error;
        }
        p.apply(Effect::SetControlCallback);
        if let Reply::Address(address) = p.apply(Effect::GetAddress) {
            self.address = address;
        }
        if let Err(error) = self.required_owned_effect(Effect::AdaptiveReset, p) {
            return error;
        }
        self.membership.state = State::Idle;
        for effect in [
            Effect::AdaptiveReset,
            Effect::ResetRfTrackers,
            Effect::Ingress {
                enabled: false,
                purge: false,
            },
        ] {
            if let Err(error) = self.required_owned_effect(effect, p) {
                return error;
            }
        }
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            ENOTSUP
        } else {
            self.owned_protocol_initialized = p.owned_protocol_algorithms();
            0
        }
    }

    pub(crate) fn schedule(&mut self, timer: Timer, delay_ms: u32, p: &mut impl Platform) {
        let i = timer as usize;
        if self.timers[i].is_some() {
            p.apply(Effect::SchedulePending(timer, delay_ms));
            return;
        }
        // Exhaustion fails closed rather than reusing a stale generation.
        if let Some(generation) = self.generations[i].checked_add(1) {
            self.generations[i] = generation;
            let token = TimerToken {
                timer,
                generation,
                deadline_us: p.now_us().saturating_add(i64::from(delay_ms) * 1000),
            };
            self.timers[i] = Some(token);
            p.apply(Effect::Schedule(token));
        } else {
            self.timers[i] = None;
        }
    }

    pub fn start(&mut self, p: &mut impl Platform) -> i32 {
        if p.owned_protocol_algorithms() && !self.owned_protocol_initialized {
            return ENOTSUP;
        }
        if self.membership.state != State::Idle {
            return EALREADY;
        }
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
            return ENOTSUP;
        }
        if let Err(error) = self.required_owned_effect(Effect::ResetRfTrackers, p) {
            return error;
        }
        let ret = value(p.apply(Effect::StartRx));
        if ret != 0 {
            return ret;
        }
        self.membership.state = State::Scanning;
        let delay = value(self.audio_effect(Effect::ScanTimeout, p));
        if delay < 0 && p.owned_protocol_algorithms() {
            p.apply(Effect::StopRx);
            self.membership.state = State::Idle;
            if self.timers[Timer::Scan as usize].take().is_some() {
                p.apply(Effect::Cancel(Timer::Scan));
            }
            return delay;
        }
        self.schedule(Timer::Scan, delay.max(0) as u32, p);
        0
    }

    pub fn stop(&mut self, p: &mut impl Platform) {
        crate::corefuture::ready(self.stop_impl::<false>(p))
    }
    pub async fn stop_async(&mut self, p: &mut impl Platform) {
        self.stop_impl::<true>(p).await
    }
    async fn stop_impl<const ASYNC_SEND: bool>(&mut self, p: &mut impl Platform) {
        self.audio_effect(
            Effect::Ingress {
                enabled: false,
                purge: false,
            },
            p,
        );
        for timer in [Timer::Scan, Timer::Join, Timer::Status] {
            self.timers[timer as usize] = None;
            p.apply(Effect::Cancel(timer));
        }
        if self.membership.state == State::Active && self.membership.node_id != 0 {
            if ASYNC_SEND {
                self.physical_protocol_effect(Effect::Leave(self.address), p)
                    .await;
            } else {
                self.audio_effect(Effect::Leave(self.address), p);
            }
        }
        self.membership.state = State::Idle;
        for effect in [
            Effect::TdmaStop,
            Effect::StopRx,
            Effect::RxStop,
            Effect::AudioCancel,
        ] {
            self.audio_effect(effect, p);
        }
        self.session.peers.fill(Peer::default());
        dedupe_reset(Some(&mut self.dedupe));
        self.audio_effect(Effect::ResetRfTrackers, p);
        self.session.relay_ring.fill(RelayEntry::default());
        self.session.control_ring.fill(RelayEntry::default());
        self.session.speaker_deadlines_ms.fill(0);
        self.session.speaker_active_since_ms.fill(0);
        self.session.active_speaker_ids.fill(0);
        self.session.relay_masks.fill(0);
        self.audio_effect(Effect::ClearGrants, p);
        self.session.heard_bitmap = 0;
        self.session.relay_bitmap = 0;
        self.membership.peer_count = 0;
        self.session.relay_head = 0;
        self.session.relay_tail = 0;
        self.session.control_head = 0;
        self.session.control_tail = 0;
        self.audio_effect(Effect::PurgeTx, p);
        self.audio_effect(
            Effect::Ingress {
                enabled: false,
                purge: true,
            },
            p,
        );
        self.session.skip_count = 0;
        self.membership.role = Role::None;
        self.audio_effect(Effect::AdaptiveReset, p);
        self.membership.node_id = 0;
        self.membership.slot_index = -1;
        self.membership.coordinator_id = 0;
        self.send_status_impl::<ASYNC_SEND>(p).await;
        Self::output_impl::<ASYNC_SEND>(p, Effect::MeshStopped).await;
    }

    async fn send_status_impl<const ASYNC_SEND: bool>(&mut self, p: &mut impl Platform) {
        let mut status = self.membership;
        if p.owned_protocol_algorithms() {
            let count = value(self.audio_effect(Effect::PeerCount(self.membership), p));
            if count < 0 {
                return;
            }
            status.peer_count = count as u8;
        } else {
            p.apply(Effect::PeerCount(self.membership));
            status.peer_count = self.membership.bridge_peer_count();
        }
        Self::output_impl::<ASYNC_SEND>(p, Effect::Status(status)).await;
    }

    pub fn request_start(&mut self, generation: u8, talk_channel: u8, p: &mut impl Platform) {
        self.request(
            Command {
                kind: CommandKind::Start,
                generation,
                talk_channel,
            },
            p,
        );
    }
    pub fn request_stop(&mut self, generation: u8, p: &mut impl Platform) {
        self.request(
            Command {
                kind: CommandKind::Stop,
                generation,
                talk_channel: 0,
            },
            p,
        );
    }
    pub fn request_audio_pause(&mut self, generation: u8, p: &mut impl Platform) {
        self.request(
            Command {
                kind: CommandKind::Pause,
                generation,
                talk_channel: 0,
            },
            p,
        );
    }
    pub fn request_audio_resume(&mut self, generation: u8, p: &mut impl Platform) {
        self.request(
            Command {
                kind: CommandKind::Resume,
                generation,
                talk_channel: 0,
            },
            p,
        );
    }
    pub fn request_status(&mut self, p: &mut impl Platform) {
        self.status_pending = true;
        p.apply(Effect::SubmitCommand);
    }
    fn request(&mut self, command: Command, p: &mut impl Platform) {
        self.pending = Some(command);
        p.apply(Effect::SubmitCommand);
    }

    pub fn command_work_handler(&mut self, p: &mut impl Platform) {
        crate::corefuture::ready(self.command_work_handler_impl::<false>(p))
    }
    pub async fn command_work_handler_async(&mut self, p: &mut impl Platform) {
        self.command_work_handler_impl::<true>(p).await
    }
    async fn command_work_handler_impl<const ASYNC_SEND: bool>(&mut self, p: &mut impl Platform) {
        if let Some(command) = self.pending.take() {
            let mut result = 0;
            match command.kind {
                CommandKind::Start => {
                    if !(1..=3).contains(&command.talk_channel) {
                        result = EINVAL;
                    } else if !accepts_protocol_algorithms(
                        p.integration(),
                        p.owned_protocol_algorithms(),
                    ) || (p.owned_protocol_algorithms()
                        && !self.owned_protocol_initialized)
                    {
                        result = ENOTSUP;
                    } else if self.membership.state == State::Idle {
                        let rf = [40, 20, 60][usize::from(command.talk_channel - 1)];
                        result = value(p.apply(Effect::SetChannel(rf)));
                        if result == 0 {
                            self.talk_channel = command.talk_channel;
                            result = self.start(p);
                        }
                    } else if self.talk_channel != command.talk_channel {
                        result = EBUSY;
                    }
                }
                CommandKind::Stop => {
                    if self.membership.state != State::Idle {
                        self.stop_impl::<ASYNC_SEND>(p).await;
                    }
                    Self::output_impl::<ASYNC_SEND>(p, Effect::DiscardBridgeAudio).await;
                }
                CommandKind::Pause | CommandKind::Resume => {
                    if p.integration() == Integration::HardwareUnqualified
                        && !p.owned_protocol_algorithms()
                    {
                        result = ENOTSUP;
                    } else {
                        let enabled = command.kind == CommandKind::Resume;
                        // The S31 re-sends Resume whenever bridge status returns;
                        // replaying the boundary would purge ingress and flush
                        // queued bridge audio for nothing. Ack only. Owned Rust
                        // policy; the source C projection replays the boundary.
                        let redundant = p.owned_protocol_algorithms()
                            && enabled == self.audio_state.boundary.enabled
                            && enabled == self.bridge_delivery_enabled;
                        if !redundant {
                            if enabled {
                                if ASYNC_SEND {
                                    p.prepare_ingress_boundary(self.ingress_admission_gate());
                                }
                                Self::output_impl::<ASYNC_SEND>(p, Effect::BridgeDelivery(false))
                                    .await;
                            }
                            let reply = self.audio_effect(Effect::LocalAudio(enabled), p);
                            if let Reply::Audio(boundary) = reply {
                                self.audio_state.boundary = boundary;
                            } else {
                                result = ENOTSUP;
                            }
                            Self::output_impl::<ASYNC_SEND>(p, Effect::BridgeDelivery(enabled))
                                .await;
                            self.bridge_delivery_enabled = enabled;
                        }
                    }
                }
            }
            Self::output_impl::<ASYNC_SEND>(p, Effect::Ack { command, result }).await;
            self.send_status_impl::<ASYNC_SEND>(p).await;
        }
        if self.status_pending {
            self.status_pending = false;
            self.send_status_impl::<ASYNC_SEND>(p).await;
        }
    }

    pub fn status_work_handler(&mut self, p: &mut impl Platform) {
        crate::corefuture::ready(self.status_work_handler_impl::<false>(p))
    }
    pub async fn status_work_handler_async(&mut self, p: &mut impl Platform) {
        self.status_work_handler_impl::<true>(p).await
    }
    async fn status_work_handler_impl<const ASYNC_SEND: bool>(&mut self, p: &mut impl Platform) {
        if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms())
            || self.membership.state != State::Active
        {
            return;
        }
        if p.owned_protocol_algorithms()
            && value(if ASYNC_SEND {
                self.physical_protocol_effect(Effect::CheckPeerTimeouts, p)
                    .await
            } else {
                self.audio_effect(Effect::CheckPeerTimeouts, p)
            }) < 0
        {
            return;
        }
        if !p.owned_protocol_algorithms() {
            p.apply(Effect::CheckPeerTimeouts);
        }
        self.audio_effect(Effect::AdaptiveReport, p);
        if self.membership.role == Role::Coordinator {
            self.audio_effect(Effect::UpdateGrants, p);
        }
        self.audio_effect(Effect::ClearBitmaps, p);
        p.apply(Effect::RadioStats);
        p.apply(Effect::TdmaStats);
        let log_now = self.status_log_decim.is_multiple_of(2);
        self.status_log_decim = self.status_log_decim.wrapping_add(1);
        self.auto_ticks = self.auto_ticks.wrapping_add(1);
        if log_now {
            p.apply(Effect::WsDiag);
        }
        let reply = if ASYNC_SEND && p.owned_protocol_algorithms() {
            self.physical_protocol_effect(Effect::CoordinatorTimeout, p)
                .await
        } else {
            self.audio_effect(Effect::CoordinatorTimeout, p)
        };
        if p.owned_protocol_algorithms() {
            // The actual helper has already performed and scheduled the transition.
            if reply == Reply::Value(0) {
                self.schedule(Timer::Status, 1000, p);
            }
            return;
        }
        if let Reply::Membership(transition) = reply {
            if transition != MembershipTransition::Unchanged {
                self.apply_membership_impl::<ASYNC_SEND>(transition, p)
                    .await;
                return;
            }
        } else {
            return; // Missing typed membership integration fails closed.
        }
        self.schedule(Timer::Status, 1000, p);
    }

    pub fn handle(&mut self, event: Event, p: &mut impl Platform) -> i32 {
        crate::corefuture::ready(self.handle_impl::<false>(event, p))
    }
    pub async fn handle_async(&mut self, event: Event, p: &mut impl Platform) -> i32 {
        self.handle_impl::<true>(event, p).await
    }
    // Direct owned orchestration: radio and membership output helpers only.
    async fn physical_protocol_effect(&mut self, e: Effect, p: &mut impl Platform) -> Reply {
        if p.owned_protocol_algorithms() {
            if let Some(reply) = p.dispatch_protocol_async(self, e).await {
                return reply;
            }
            return Reply::Value(ENOTSUP);
        }
        self.audio_effect(e, p)
    }
    async fn handle_impl<const ASYNC_SEND: bool>(
        &mut self,
        event: Event,
        p: &mut impl Platform,
    ) -> i32 {
        match event {
            Event::Command(command) => self.request(command, p),
            Event::RequestStatus => self.request_status(p),
            Event::DispatchCommands => self.command_work_handler_impl::<ASYNC_SEND>(p).await,
            Event::Membership(membership) | Event::InjectMembership(membership) => {
                if !allows_fixture_injection(p.integration()) || p.owned_protocol_algorithms() {
                    return ENOTSUP;
                }
                self.membership = membership;
                if matches!(event, Event::Membership(_)) && membership.state == State::Active {
                    self.timers[Timer::Join as usize] = None;
                    p.apply(Effect::Cancel(Timer::Join));
                    self.schedule(Timer::Status, 1000, p);
                }
            }
            Event::Timer(token) => {
                if !accepts_protocol_algorithms(p.integration(), p.owned_protocol_algorithms()) {
                    return ENOTSUP;
                }
                if p.owned_protocol_algorithms() && token.timer != Timer::Status {
                    return value(if ASYNC_SEND {
                        self.physical_protocol_effect(Effect::OwnedMembershipTimer(token), p)
                            .await
                    } else {
                        self.audio_effect(Effect::OwnedMembershipTimer(token), p)
                    });
                }
                let i = token.timer as usize;
                if self.timers[i] != Some(token) || p.now_us() < token.deadline_us {
                    return 0;
                }
                self.timers[i] = None;
                if token.timer == Timer::Status {
                    self.status_work_handler_impl::<ASYNC_SEND>(p).await;
                } else if let Reply::Membership(transition) =
                    p.apply(Effect::MembershipTimer(token.timer))
                {
                    self.apply_membership_impl::<ASYNC_SEND>(transition, p)
                        .await;
                }
            }
        }
        0
    }

    /// Source packed atomic request, for serial oracle observation only.
    pub fn pending_request(&self) -> u32 {
        self.pending.map_or(0, |c| {
            u32::from(c.kind as u8)
                | (u32::from(c.generation) << 8)
                | (u32::from(c.talk_channel) << 16)
        })
    }

    pub fn status_pending(&self) -> bool {
        self.status_pending
    }

    async fn apply_membership_impl<const ASYNC_SEND: bool>(
        &mut self,
        transition: MembershipTransition,
        p: &mut impl Platform,
    ) {
        if !allows_fixture_injection(p.integration()) {
            return;
        }
        match transition {
            MembershipTransition::Unchanged => {}
            MembershipTransition::Apply { next, timers } => {
                self.membership = next;
                for directive in timers.into_iter().flatten() {
                    match directive {
                        TimerDirective::Cancel(timer) => {
                            self.timers[timer as usize] = None;
                            p.apply(Effect::Cancel(timer));
                        }
                        TimerDirective::Schedule(timer, delay) => self.schedule(timer, delay, p),
                    }
                }
            }
            MembershipTransition::CoordinatorLost => {
                self.audio_effect(
                    Effect::Ingress {
                        enabled: false,
                        purge: false,
                    },
                    p,
                );
                Self::output_impl::<ASYNC_SEND>(p, Effect::SyncLost).await;
                p.apply(Effect::TdmaStop);
                // External reset covers audio/adaptive-owned state; clear our projection too.
                p.apply(Effect::ResetMembershipSession);
                let skip_count = self.session.skip_count;
                self.session = Session::default();
                self.session.skip_count = skip_count;
                dedupe_reset(Some(&mut self.dedupe));
                self.membership = Membership {
                    state: State::Scanning,
                    membership_known: self.membership.membership_known,
                    ..Membership::default()
                };
                let delay = value(p.apply(Effect::ScanTimeout));
                self.send_status_impl::<ASYNC_SEND>(p).await;
                self.schedule(Timer::Scan, delay.max(0) as u32, p);
            }
        }
    }
}

#[cfg(test)]
#[path = "membership_tests.rs"]
mod membership_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::NotIntegrated;
    use openhelmet_mesh::core::dedupe_accept;

    #[test]
    fn stop_clears_owned_session_storage_and_dedupe() {
        let mut runtime = Runtime::default();
        runtime.session.peers[7].active = true;
        runtime.session.peers[7].address.fill(42);
        runtime.session.relay_ring[15].data.fill(13);
        runtime.session.relay_ring[15].queued_ms = 123;
        runtime.session.control_ring[31].len = 9;
        runtime.session.speaker_deadlines_ms.fill(321);
        runtime.session.speaker_active_since_ms.fill(123);
        runtime.session.relay_head = 15;
        runtime.session.relay_tail = 7;
        runtime.session.control_head = 31;
        runtime.session.control_tail = 8;
        runtime.session.heard_bitmap = 255;
        runtime.session.relay_bitmap = 127;
        runtime.session.skip_count = 55;
        dedupe_accept(Some(&mut runtime.dedupe), 1, 2, 3);
        assert!(!runtime.snapshot().session_empty);
        runtime.stop(&mut NotIntegrated);
        assert!(runtime.snapshot().session_empty);
        assert_eq!(runtime.dedupe, Dedupe::default());
    }
}
