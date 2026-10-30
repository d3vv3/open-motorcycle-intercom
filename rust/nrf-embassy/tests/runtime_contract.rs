use nrf_embassy::platform::{AudioBoundary, Effect, Integration, NotIntegrated, Platform, Reply};
use nrf_embassy::runtime::{
    CommandKind, Event, Membership, MembershipTransition, Role, Runtime, State, Timer,
    TimerDirective, ENOTSUP,
};
use std::fmt::Write as _;

struct Simulation {
    trace: String,
    now: i64,
    channel_result: i32,
    rx_result: i32,
    timeout: i32,
    scan_delay: i32,
    peers: u8,
    audio: AudioBoundary,
    membership_reply: MembershipTransition,
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            trace: String::new(),
            now: 0,
            channel_result: 0,
            rx_result: 0,
            timeout: 0,
            scan_delay: 3000,
            peers: 0,
            audio: AudioBoundary::default(),
            membership_reply: MembershipTransition::Unchanged,
        }
    }
}

impl Simulation {
    fn log(&mut self, id: u8, values: &[i64]) {
        write!(self.trace, "E {id}").unwrap();
        for value in values {
            write!(self.trace, " {value}").unwrap();
        }
        self.trace.push('\n');
    }
}

impl Platform for Simulation {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        let (id, args): (u8, Vec<i64>) = match effect {
            Effect::InitTimer(t) => (1, vec![t as i64]),
            Effect::InitCommand => (2, vec![]),
            Effect::BindMembershipWork => (3, vec![]),
            Effect::AudioInit => {
                self.audio.enabled = true;
                (4, vec![])
            }
            Effect::RxInit => (5, vec![]),
            Effect::SetControlCallback => (6, vec![]),
            Effect::GetAddress => {
                self.log(7, &[]);
                return Reply::Address([1, 2, 3, 4, 5]);
            }
            Effect::AdaptiveReset => (8, vec![]),
            Effect::ResetRfTrackers => (9, vec![]),
            Effect::Ingress { enabled, purge } => (10, vec![enabled as i64, purge as i64]),
            Effect::SubmitCommand => (12, vec![]),
            Effect::SetChannel(rf) => {
                self.log(13, &[i64::from(rf)]);
                return Reply::Value(self.channel_result);
            }
            Effect::StartRx => {
                self.log(14, &[]);
                return Reply::Value(self.rx_result);
            }
            Effect::ScanTimeout => {
                self.log(15, &[]);
                return Reply::Value(self.scan_delay);
            }
            Effect::Schedule(t) => (17, vec![t.timer as i64, (t.deadline_us - self.now) / 1000]),
            Effect::SchedulePending(t, delay) => (17, vec![t as i64, i64::from(delay)]),
            Effect::Cancel(t) => (18, vec![t as i64]),
            Effect::Leave(address) => (19, address.map(i64::from).to_vec()),
            Effect::TdmaStop => (20, vec![]),
            Effect::StopRx => (21, vec![]),
            Effect::RxStop => (22, vec![]),
            Effect::AudioCancel => (23, vec![]),
            Effect::ClearGrants => (24, vec![]),
            Effect::PurgeTx => {
                self.peers = 0;
                (25, vec![])
            }
            Effect::PeerCount(m) => {
                self.log(26, &[]);
                return Reply::Value(i32::from(m.bridge_peer_count()));
            }
            Effect::Status(m) => {
                let payload = m.status_v6();
                assert_eq!(
                    payload,
                    [
                        m.role as u8,
                        m.peer_count,
                        m.node_id,
                        6,
                        m.state as u8,
                        m.slot_index as u8,
                        m.coordinator_id,
                        165,
                        2,
                        20
                    ]
                );
                (
                    27,
                    vec![
                        m.state as i64,
                        m.role as i64,
                        i64::from(m.peer_count),
                        i64::from(m.node_id),
                        i64::from(m.slot_index),
                        i64::from(m.coordinator_id),
                        6,
                        165,
                        2,
                        20,
                    ],
                )
            }
            Effect::MeshStopped => (28, vec![]),
            Effect::DiscardBridgeAudio => (29, vec![]),
            Effect::BridgeDelivery(enabled) => (30, vec![enabled as i64]),
            Effect::LocalAudio(enabled) => {
                self.log(31, &[enabled as i64]);
                self.audio.enabled = enabled;
                self.audio.epoch = self.audio.epoch.wrapping_add(1);
                self.audio.receipt_cutoff_us = self.now;
                return Reply::Audio(self.audio);
            }
            Effect::Ack { command, result } => {
                let payload = command.ack_payload(result);
                assert_eq!(payload[0], command.kind as u8);
                assert_eq!(payload[1], command.generation);
                (
                    32,
                    vec![
                        command.kind as i64,
                        i64::from(command.generation),
                        i64::from(result),
                        i64::from(payload[2] as i8),
                    ],
                )
            }
            Effect::CheckPeerTimeouts => (33, vec![]),
            Effect::AdaptiveReport => (34, vec![]),
            Effect::UpdateGrants => (35, vec![]),
            Effect::ClearBitmaps => (36, vec![]),
            Effect::RadioStats => (37, vec![]),
            Effect::TdmaStats => (38, vec![]),
            Effect::WsDiag => (39, vec![]),
            Effect::CoordinatorTimeout => {
                self.log(40, &[]);
                return Reply::Membership(if self.timeout != 0 {
                    MembershipTransition::CoordinatorLost
                } else {
                    MembershipTransition::Unchanged
                });
            }
            Effect::MembershipTimer(t) => {
                self.log(41, &[t as i64]);
                return Reply::Membership(self.membership_reply);
            }
            Effect::OwnedMembershipTimer(_) => return Reply::Value(nrf_embassy::runtime::ENOTSUP),
            Effect::ResetMembershipSession => (42, vec![]),
            Effect::SyncLost => (43, vec![]),
        };
        self.log(id, &args);
        Reply::None
    }
}

fn state(value: i64) -> State {
    match value {
        0 => State::Idle,
        1 => State::Scanning,
        2 => State::Joining,
        3 => State::Active,
        _ => panic!("state"),
    }
}
fn role(value: i64) -> Role {
    match value {
        0 => Role::None,
        1 => Role::Coordinator,
        2 => Role::Participant,
        _ => panic!("role"),
    }
}
fn timer(value: i64) -> Timer {
    match value {
        0 => Timer::Scan,
        1 => Timer::Join,
        2 => Timer::Status,
        _ => panic!("timer"),
    }
}

fn replay(script: &str) -> String {
    let mut runtime = Runtime::default();
    let mut p = Simulation::default();
    for line in script.lines() {
        let mut parts = line.split_whitespace();
        let op = parts.next().unwrap();
        let args: Vec<i64> = parts.map(|s| s.parse().unwrap()).collect();
        match op {
            "I" => {
                let r = runtime.init(&mut p);
                writeln!(p.trace, "R {r}").unwrap();
            }
            "Q" => match args[0] {
                1 => runtime.request_start(args[1] as u8, args[2] as u8, &mut p),
                2 => runtime.request_stop(args[1] as u8, &mut p),
                4 => runtime.request_audio_pause(args[1] as u8, &mut p),
                5 => runtime.request_audio_resume(args[1] as u8, &mut p),
                _ => panic!("command"),
            },
            "H" => runtime.request_status(&mut p),
            "D" => runtime.command_work_handler(&mut p),
            "S" => {
                let r = runtime.start(&mut p);
                writeln!(p.trace, "R {r}").unwrap();
            }
            "X" => runtime.stop(&mut p),
            "T" => runtime.status_work_handler(&mut p),
            "M" | "A" => {
                p.peers = args[2] as u8;
                let membership = Membership {
                    state: state(args[0]),
                    role: role(args[1]),
                    peer_count: p.peers,
                    membership_known: false,
                    node_id: args[3] as u8,
                    slot_index: args[4] as i8,
                    coordinator_id: args[5] as u8,
                };
                assert_eq!(
                    runtime.handle(
                        if op == "A" {
                            Event::Membership(membership)
                        } else {
                            Event::InjectMembership(membership)
                        },
                        &mut p
                    ),
                    0
                );
            }
            "N" => {
                let mut membership = runtime.snapshot().membership;
                membership.membership_known = args[0] != 0;
                runtime.handle(Event::InjectMembership(membership), &mut p);
            }
            "K" => {
                p.now = args[0];
                p.channel_result = args[1] as i32;
                p.rx_result = args[2] as i32;
                p.timeout = args[3] as i32;
                p.scan_delay = args[4] as i32;
            }
            "W" => {
                p.now = args[2];
                if let Some(mut token) = runtime.timer(timer(args[0])) {
                    token.generation = args[1] as u64;
                    runtime.handle(Event::Timer(token), &mut p);
                }
            }
            _ => panic!("operation {op}"),
        }
        let s = runtime.snapshot();
        let m = s.membership;
        writeln!(
            p.trace,
            "P {} {} {} {} {} {} {} {} {} {} {} {}",
            m.state as u8,
            m.role as u8,
            m.peer_count,
            m.node_id,
            m.slot_index,
            m.coordinator_id,
            s.talk_channel,
            s.auto_ticks,
            s.status_log_decim,
            u8::from(s.audio.enabled),
            s.audio.epoch,
            s.audio.receipt_cutoff_us
        )
        .unwrap();
        writeln!(p.trace, "N {}", u8::from(m.membership_known)).unwrap();
        for timer in [Timer::Scan, Timer::Join, Timer::Status] {
            if let Some(token) = runtime.timer(timer) {
                writeln!(
                    p.trace,
                    "Z {} {} {}",
                    timer as u8, token.generation, token.deadline_us
                )
                .unwrap();
            } else {
                writeln!(p.trace, "Z {} -", timer as u8).unwrap();
            }
        }
    }
    p.trace
}

#[test]
fn scripted_scenarios_hold_coalescing_boundary_pending_and_loss_requirements() {
    let cases = [
        ("coalescing", "I\nQ 1 254 1\nQ 1 255 2\nH\nH\nD\nD\nQ 2 0 0\nD\n"),
        ("invalid-and-rf-failure", "I\nQ 1 1 0\nD\nQ 1 2 4\nD\nK 100 -5 0 0 3123\nQ 1 3 2\nD\nK 200 0 -19 0 3123\nQ 1 4 3\nD\nK 300 0 0 0 3123\nQ 1 5 1\nD\n"),
        ("channel-change-restart", "I\nQ 1 1 1\nD\nQ 1 2 1\nD\nQ 1 3 2\nD\nS\nQ 2 4 0\nD\nQ 1 5 2\nD\nQ 2 6 0\nD\nQ 2 7 0\nD\n"),
        ("lifecycle-idle-active", "I\nX\nS\nM 3 1 4 1 0 1\nX\nS\nM 3 2 255 2 1 1\nH\nD\nX\n"),
        ("pause-resume-boundaries", "I\nK 1000 0 0 0 3000\nQ 4 10 0\nD\nK 2000 0 0 0 3000\nQ 5 11 0\nD\nQ 1 12 1\nD\nM 3 1 2 1 0 1\nK 3000 0 0 0 3000\nQ 4 13 0\nD\nQ 2 14 0\nD\nQ 5 15 0\nD\n"),
        ("timer-guard-status", "I\nS\nW 0 1 2999999\nW 0 99 3000000\nW 0 1 3000000\nX\nS\nW 0 1 6000000\nW 0 2 6000000\nM 3 1 2 1 0 1\nT\nW 2 99 7000000\nW 2 1 6999999\nW 2 1 7000000\nM 3 2 255 2 1 1\nW 2 2 8000000\nK 9000000 0 0 1 3000\nW 2 3 9000000\nX\nW 2 3 10000000\nT\n"),
        ("pending-status-deadline", "I\nA 3 1 3 1 0 1\nK 100000 0 0 0 3000\nT\nK 200000 0 0 0 3000\nT\nW 2 1 1000000\n"),
        ("participant-peer-query", "I\nA 3 2 3 2 1 1\nH\nD\nN 1\nH\nD\n"),
        ("coordinator-loss-transition", "I\nA 3 2 3 2 1 1\nK 1000000 0 0 1 3123\nW 2 1 1000000\n"),
    ];
    let coalesced = replay(cases[0].1);
    assert!(!coalesced.contains("E 32 1 254"));
    assert!(coalesced.contains("E 32 1 255 0 0"));
    let boundary = replay(cases[4].1);
    assert!(boundary.contains("E 30 0\nE 31 1\nE 30 1\n"));
    assert!(boundary.contains("E 31 0\nE 30 0\n"));
    let pending = replay(cases[6].1);
    assert_eq!(pending.matches("Z 2 1 1000000\n").count(), 5);
    assert!(pending.contains("Z 2 2 2000000\n"));
    let peers = replay(cases[7].1);
    assert!(peers.contains("E 27 3 2 255 2 1 1"));
    assert!(peers.contains("E 27 3 2 3 2 1 1"));
    assert!(peers.contains("P 3 2 3 2 1 1"));
    let lost = replay(cases[8].1);
    assert!(lost.contains(
        "E 40\nE 10 0 0\nE 43\nE 20\nE 42\nE 15\nE 26\nE 27 1 0 0 0 -1 0 6 165 2 20\nE 17 0 3123\n"
    ));
    assert!(lost.contains("P 1 0 0 0 -1 0"));
    assert!(lost.contains("Z 0 1 4123000\nZ 1 -\nZ 2 -\n"));
}

#[test]
fn unsupported_backend_cannot_start_or_become_active() {
    let mut runtime = Runtime::default();
    let mut p = NotIntegrated;
    assert_eq!(runtime.init(&mut p), ENOTSUP);
    assert_eq!(runtime.start(&mut p), ENOTSUP);
    runtime.request_start(255, 1, &mut p);
    runtime.command_work_handler(&mut p);
    assert_eq!(runtime.snapshot().membership.state, State::Idle);
    assert_eq!(
        runtime.handle(
            Event::Membership(Membership {
                state: State::Active,
                ..Membership::default()
            }),
            &mut p
        ),
        ENOTSUP
    );
    assert_eq!(runtime.snapshot().membership.state, State::Idle);
    assert_eq!(
        runtime.handle(
            Event::InjectMembership(Membership {
                state: State::Active,
                ..Membership::default()
            }),
            &mut p
        ),
        ENOTSUP
    );
    assert_eq!(runtime.snapshot().membership.state, State::Idle);
}

#[test]
fn stale_actor_token_is_rejected_after_restart() {
    let mut runtime = Runtime::default();
    let mut p = Simulation::default();
    runtime.init(&mut p);
    assert_eq!(runtime.start(&mut p), 0);
    let old = runtime.timer(Timer::Scan).unwrap();
    runtime.stop(&mut p);
    assert_eq!(runtime.start(&mut p), 0);
    p.now = 4_000_000;
    p.trace.clear();
    runtime.handle(Event::Timer(old), &mut p);
    assert!(p.trace.is_empty());
    let new = runtime.timer(Timer::Scan).unwrap();
    assert_ne!(new.generation, old.generation);
    runtime.handle(Event::Timer(new), &mut p);
    assert_eq!(p.trace, "E 41 0\n");
}

#[test]
fn membership_activation_cancels_join_and_starts_event_driven_status() {
    let mut runtime = Runtime::default();
    let mut p = Simulation::default();
    runtime.init(&mut p);
    runtime.start(&mut p);
    p.membership_reply = MembershipTransition::Apply {
        next: Membership {
            state: State::Joining,
            ..Membership::default()
        },
        timers: [Some(TimerDirective::Schedule(Timer::Join, 500)), None, None],
    };
    p.now = 3_000_000;
    runtime.handle(Event::Timer(runtime.timer(Timer::Scan).unwrap()), &mut p);
    assert_eq!(runtime.snapshot().membership.state, State::Joining);
    let join = runtime.timer(Timer::Join).unwrap();
    let active = Membership {
        state: State::Active,
        role: Role::Participant,
        peer_count: 3,
        node_id: 2,
        slot_index: 1,
        coordinator_id: 1,
        membership_known: false,
    };
    p.trace.clear();
    runtime.handle(Event::Membership(active), &mut p);
    assert_eq!(runtime.timer(Timer::Join), None);
    let first = runtime.timer(Timer::Status).unwrap();
    assert_eq!(first.deadline_us, p.now + 1_000_000);
    assert_eq!(p.trace, "E 18 1\nE 17 2 1000\n");
    p.now = first.deadline_us;
    runtime.handle(Event::Timer(join), &mut p);
    assert_eq!(runtime.snapshot().auto_ticks, 0);
    runtime.handle(Event::Timer(first), &mut p);
    assert_eq!(runtime.snapshot().auto_ticks, 1);
    assert_eq!(
        runtime.timer(Timer::Status).unwrap().deadline_us,
        p.now + 1_000_000
    );
}

#[test]
fn membership_timer_reply_applies_owned_state_and_timer_directives() {
    let mut runtime = Runtime::default();
    let mut p = Simulation::default();
    runtime.init(&mut p);
    runtime.start(&mut p);
    let active = Membership {
        state: State::Active,
        role: Role::Coordinator,
        node_id: 1,
        slot_index: 0,
        coordinator_id: 1,
        ..Membership::default()
    };
    p.membership_reply = MembershipTransition::Apply {
        next: active,
        timers: [
            Some(TimerDirective::Cancel(Timer::Join)),
            Some(TimerDirective::Schedule(Timer::Status, 1000)),
            None,
        ],
    };
    p.now = 3_000_000;
    runtime.handle(Event::Timer(runtime.timer(Timer::Scan).unwrap()), &mut p);
    assert_eq!(runtime.snapshot().membership, active);
    assert_eq!(runtime.timer(Timer::Join), None);
    let status = runtime.timer(Timer::Status).unwrap();
    assert_eq!(status.deadline_us, 4_000_000);
    runtime.status_work_handler(&mut p);
    p.now += 100_000;
    runtime.status_work_handler(&mut p);
    assert_eq!(runtime.timer(Timer::Status), Some(status));
}

#[test]
fn response_preserves_exact_result_and_wire_generation() {
    let mut runtime = Runtime::default();
    let mut p = Simulation::default();
    runtime.init(&mut p);
    p.channel_result = -5;
    p.trace.clear();
    runtime.request_start(0, 2, &mut p);
    runtime.command_work_handler(&mut p);
    assert!(p.trace.contains("E 13 20\nE 32 1 0 -5 -1\n"));
    let command = nrf_embassy::runtime::Command {
        kind: CommandKind::Start,
        generation: 255,
        talk_channel: 3,
    };
    assert_eq!(command.ack_payload(-19), [1, 255, 255]);
}

#[test]
fn audio_boundary_is_an_external_owned_response() {
    struct BoundaryPlatform {
        simulation: Simulation,
        response: AudioBoundary,
    }
    impl Platform for BoundaryPlatform {
        fn integration(&self) -> Integration {
            Integration::ActorOracle
        }
        fn now_us(&self) -> i64 {
            self.simulation.now
        }
        fn apply(&mut self, effect: Effect) -> Reply {
            let reply = self.simulation.apply(effect);
            if matches!(effect, Effect::LocalAudio(_)) {
                Reply::Audio(self.response)
            } else {
                reply
            }
        }
    }
    let mut runtime = Runtime::default();
    let mut p = BoundaryPlatform {
        simulation: Simulation::default(),
        response: AudioBoundary {
            enabled: false,
            epoch: u32::MAX,
            receipt_cutoff_us: 900,
        },
    };
    runtime.init(&mut p);
    p.simulation.now = 100;
    runtime.request_audio_pause(1, &mut p);
    runtime.command_work_handler(&mut p);
    assert_eq!(runtime.snapshot().audio, p.response);
    p.response = AudioBoundary {
        enabled: true,
        epoch: 4,
        receipt_cutoff_us: 12_345,
    };
    p.simulation.now = 200;
    runtime.request_audio_resume(2, &mut p);
    runtime.command_work_handler(&mut p);
    assert_eq!(runtime.snapshot().audio, p.response);
    // Deliberately unrelated to the actor clock/previous epoch: the kernel
    // must not invent audio-module admission or receipt-cutoff algorithms.
}

#[test]
fn unsupported_start_ack_is_failure_without_rf_effects() {
    struct UnsupportedRecorder(Simulation);
    impl Platform for UnsupportedRecorder {
        fn integration(&self) -> Integration {
            Integration::NotIntegrated
        }
        fn now_us(&self) -> i64 {
            self.0.now
        }
        fn apply(&mut self, effect: Effect) -> Reply {
            self.0.apply(effect)
        }
    }
    let mut runtime = Runtime::default();
    let mut p = UnsupportedRecorder(Simulation::default());
    assert_eq!(runtime.init(&mut p), ENOTSUP);
    p.0.trace.clear();
    runtime.request_start(255, 3, &mut p);
    runtime.command_work_handler(&mut p);
    assert!(p.0.trace.contains("E 32 1 255 -95 -1\n"));
    assert!(!p.0.trace.contains("E 13"));
    assert!(!p.0.trace.contains("E 14"));
    assert!(p.0.trace.contains("E 27 0 0 0 0 -1 0 6 165 2 20\n"));
    assert_eq!(runtime.snapshot().membership.state, State::Idle);
}
