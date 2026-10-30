use crate::runtime::{Command, Membership, MembershipTransition, Timer, TimerToken};

// Opt-in membership IO does not change the existing kernel simulation contract.
pub use crate::protocol_membership::{MembershipEffect, MembershipPlatform};

/// Algorithm eligibility is separate from physical integration qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Integration {
    NotIntegrated,
    ActorOracle,
    /// Explicit hardware opt-in requiring strict protocol ownership, not readiness.
    HardwareUnqualified,
}

pub const fn accepts_protocol_algorithms(integration: Integration, owned_protocol: bool) -> bool {
    match integration {
        Integration::ActorOracle => true,
        Integration::HardwareUnqualified => owned_protocol,
        Integration::NotIntegrated => false,
    }
}

pub const fn allows_fixture_injection(integration: Integration) -> bool {
    matches!(integration, Integration::ActorOracle)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AudioBoundary {
    pub enabled: bool,
    pub epoch: u32,
    pub receipt_cutoff_us: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Effect {
    InitTimer(Timer),
    InitCommand,
    BindMembershipWork,
    AudioInit,
    RxInit,
    SetControlCallback,
    GetAddress,
    AdaptiveReset,
    ResetRfTrackers,
    Ingress {
        enabled: bool,
        purge: bool,
    },
    SubmitCommand,
    SetChannel(u8),
    StartRx,
    ScanTimeout,
    Schedule(TimerToken),
    SchedulePending(Timer, u32),
    Cancel(Timer),
    Leave([u8; 5]),
    TdmaStop,
    StopRx,
    RxStop,
    AudioCancel,
    ClearGrants,
    PurgeTx,
    PeerCount(Membership),
    Status(Membership),
    MeshStopped,
    DiscardBridgeAudio,
    BridgeDelivery(bool),
    LocalAudio(bool),
    Ack {
        command: Command,
        result: i32,
    },
    CheckPeerTimeouts,
    AdaptiveReport,
    UpdateGrants,
    ClearBitmaps,
    RadioStats,
    TdmaStats,
    WsDiag,
    CoordinatorTimeout,
    MembershipTimer(Timer),
    /// Full protocol owner validates and consumes this token, not the old kernel.
    OwnedMembershipTimer(TimerToken),
    SyncLost,
    /// External reset_session_data(true), including its audio/adaptive effects.
    ResetMembershipSession,
}

/// Query results are owned values, never shared mutable context.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply {
    None,
    Value(i32),
    Address([u8; 5]),
    Audio(AudioBoundary),
    Membership(MembershipTransition),
}

/// Calls are ordered. Sync algorithms use apply; async output orchestration uses
/// apply_async, preserving ACK generation and the exact result before encoding.
/// The default kernel mock retains external audio replies. FullAudioPlatform
/// dispatches audio against Runtime's actual owner before backend IO borrows.
#[allow(async_fn_in_trait)]
pub trait Platform {
    fn integration(&self) -> Integration;
    fn now_us(&self) -> i64;
    fn apply(&mut self, effect: Effect) -> Reply;
    /// Async lifecycle preflight only. Close and purge external ingress without
    /// changing the owner's epoch; only a later owned gate publication may reopen.
    fn prepare_ingress_boundary(&mut self, _gate: crate::protocol_audio::IngressAdmissionGate) {}
    /// Async owner revalidation after output handover; default preserves replay.
    fn tx_window_valid(&self) -> bool {
        true
    }
    /// Synchronous external admission publication, before any subsequent output
    /// can yield. The endpoint must not borrow or reenter Runtime.
    fn ingress_gate_changed(
        &mut self,
        _gate: crate::protocol_audio::IngressAdmissionGate,
        _purge: bool,
    ) {
    }
    /// Completes the backend enqueue/mutation, not a physical SPI transfer.
    async fn apply_async(&mut self, effect: Effect) -> Reply {
        self.apply(effect)
    }
    /// Strict serial software composition; never a hardware readiness marker.
    fn owned_protocol_algorithms(&self) -> bool {
        false
    }
    /// Called before borrowing any backend algorithm IO. No reentrant producers.
    fn dispatch_protocol(
        &mut self,
        _runtime: &mut crate::runtime::Runtime,
        _effect: Effect,
    ) -> Option<Reply> {
        None
    }
    async fn dispatch_protocol_async(
        &mut self,
        runtime: &mut crate::runtime::Runtime,
        effect: Effect,
    ) -> Option<Reply> {
        self.dispatch_protocol(runtime, effect)
    }
    /// Default-off RX owner dispatch. This is not automatic kernel RX delivery.
    fn dispatch_rx(
        &mut self,
        _effect: Effect,
        _runtime: &mut crate::runtime::Runtime,
    ) -> Option<Reply> {
        None
    }
    /// Opt-in owner dispatch, before any mutable IO borrow. Old mocks use apply.
    fn dispatch_audio(
        &mut self,
        _effect: Effect,
        _runtime: &mut crate::runtime::Runtime,
    ) -> Option<Reply> {
        None
    }
}

/// Fail-closed placeholder, not a hardware implementation.
pub struct NotIntegrated;

impl Platform for NotIntegrated {
    fn integration(&self) -> Integration {
        Integration::NotIntegrated
    }
    fn now_us(&self) -> i64 {
        0
    }
    fn apply(&mut self, _: Effect) -> Reply {
        Reply::Value(crate::runtime::ENOTSUP)
    }
}
