//! Serial protocol composition corpus with explicit lifecycle requirements.
use super::*;
use crate::protocol_audio::FullAudioPlatform;
use crate::protocol_membership::Error;
use crate::protocol_runtime::FullProtocolPlatform;

fn lifecycle(code: u8, now: i64) -> JointOp {
    JointOp::new(code, now)
}
fn request(code: u8, generation: u8, group: u8, now: i64) -> JointOp {
    lifecycle(code, now).arg(1, generation).arg(2, group)
}
fn tick(timer: Timer, now: i64) -> JointOp {
    lifecycle(111, now).arg(1, timer as u8)
}
fn rf_message(msg: Message<'_>, src: u8, seq: u8, now: i64) -> JointOp {
    let mut payload = [0; 200];
    let len = msg.encode(&mut payload).unwrap();
    rf_op(
        93,
        &outer(msg.packet_type(), src, seq, &payload[..len]),
        now * 1000,
        now,
    )
}
fn protocol_snapshot(p: &mut Joint, r: &Runtime) {
    p32(&mut p.inner.bytes, r.pending_request());
    p32(&mut p.inner.bytes, u32::from(r.status_pending()));
    p.inner.bytes.push(p.bridge_delivery as u8);
    p32(&mut p.inner.bytes, p.bridge_flushes);
    p32(&mut p.inner.bytes, p.bridge_gate_calls);
}
fn run_protocol(
    r: &mut Runtime,
    p: &mut Joint,
    token: &mut Option<IngressToken>,
    saved: &mut [Option<TimerToken>; 3],
    op: JointOp,
) {
    let x = op.0;
    if x[0] < 100 {
        if x[0] == 0 {
            p.full_protocol = false;
            saved.fill(None);
        }
        run_rx(r, p, token, op);
        if x[0] == 0 {
            // The preserved corpus seeds legacy guards, not source request words.
            r.pending = None;
            r.status_pending = false;
        }
    } else {
        configure(p, &x);
        let timeout_expected = x[0] == 109
            && r.membership.state == State::Active
            && r.membership.role == Role::Participant
            && (p.inner.now as u32).wrapping_sub(r.protocol.last_sync_time) > 5000;
        let effect_start = p.inner.bytes.len();
        p.channel_result = g32(&x[16..]) as i32;
        p.start_result = g32(&x[20..]) as i32;
        if x[0] == 100 {
            p.full_protocol = true;
        }
        assert!(p.full_protocol);
        let mut io = FullProtocolPlatform(p);
        let ret = match x[0] {
            100 => Some(r.init(&mut io)),
            101 => Some(r.start(&mut io)),
            102 => {
                r.stop(&mut io);
                None
            }
            103 => {
                r.request_start(x[1], x[2], &mut io);
                None
            }
            104 => {
                r.request_stop(x[1], &mut io);
                None
            }
            105 => {
                r.request_audio_pause(x[1], &mut io);
                None
            }
            106 => {
                r.request_audio_resume(x[1], &mut io);
                None
            }
            107 => {
                r.request_status(&mut io);
                None
            }
            108 => {
                r.command_work_handler(&mut io);
                None
            }
            109 => {
                r.status_work_handler(&mut io);
                None
            }
            110 => {
                saved[usize::from(x[1] % 3)] = r.timers[usize::from(x[1] % 3)];
                None
            }
            111 => {
                let i = usize::from(x[1] % 3);
                let token = if x[2] != 0 { saved[i] } else { r.timers[i] };
                if let Some(token) = token {
                    let stale_or_early =
                        r.timers[i] != Some(token) || io.now_us() < token.deadline_us;
                    let before_membership = r.membership;
                    let before_timers = r.timers;
                    let before_effects = io.0.inner.bytes.len();
                    assert_eq!(r.handle(Event::Timer(token), &mut io), 0);
                    if stale_or_early {
                        assert_eq!(
                            r.membership, before_membership,
                            "stale/early timer cannot change membership"
                        );
                        assert_eq!(
                            r.timers, before_timers,
                            "stale/early timer cannot reschedule"
                        );
                        assert_eq!(
                            io.0.inner.bytes.len(),
                            before_effects,
                            "stale/early timer cannot emit effects"
                        );
                    }
                }
                None
            }
            112 => {
                r.membership.membership_known = x[1] != 0;
                None
            }
            _ => panic!("unknown protocol operation {}", x[0]),
        };
        match x[0] {
            102 => {
                assert_eq!(r.membership.state, State::Idle);
                assert!(r.timers.iter().all(Option::is_none));
                assert_eq!(r.rx_state().cursors(), (0, 0));
                assert!(!r.rx_state().work_pending());
            }
            103..=106 => {
                let kind = match x[0] {
                    103 => CommandKind::Start,
                    104 => CommandKind::Stop,
                    105 => CommandKind::Pause,
                    _ => CommandKind::Resume,
                };
                assert_eq!(
                    r.pending,
                    Some(Command {
                        kind,
                        generation: x[1],
                        talk_channel: if x[0] == 103 { x[2] } else { 0 },
                    }),
                    "new request replaces pending command without losing generation"
                );
            }
            107 => assert!(r.status_pending),
            108 => assert!(r.pending.is_none(), "worker consumes pending command"),
            _ => {}
        }
        if timeout_expected {
            assert_eq!(r.membership.state, State::Scanning);
            assert_eq!(r.membership.role, Role::None);
            assert_eq!(r.membership.node_id, 0);
            assert!(r.timer(Timer::Scan).is_some());
            assert_eq!(
                io.0.inner.bytes[effect_start..]
                    .windows(4)
                    .filter(|record| *record == [b'E', 22, 5, 0])
                    .count(),
                1,
                "coordinator loss must emit SYNC_LOST exactly once"
            );
        }
        if let Some(ret) = ret {
            io.0.inner.e(52);
            p32(&mut io.0.inner.bytes, ret as u32);
        }
        snapshot_joint(r, io.0, false);
        audio_snapshot(&mut io.0.inner.bytes, r);
        rx_snapshot(&mut io.0.inner.bytes, r);
    }
    protocol_snapshot(p, r);
}

fn protocol_fixtures() -> Vec<JointOp> {
    let mut ops = rx_fixtures();
    // Prefix alone retains legacy semantics. All lifecycle cases opt into strict.
    ops.extend([
        lifecycle(0, 10000),
        lifecycle(100, 10000),
        lifecycle(100, 10000),
    ]);
    for (group, channel_error, rx_error) in [
        (0u8, 0i32, 0i32),
        (4, 0, 0),
        (255, 0, 0),
        (2, -5, 0),
        (3, 0, -7),
        (1, 0, -9),
        (2, 0, 0),
    ] {
        ops.push(request(103, group.wrapping_add(230), group, 10000));
        ops.push(
            lifecycle(108, 10000)
                .scalar(16, channel_error as u32)
                .scalar(20, rx_error as u32)
                .scalar(24, if rx_error != 0 { (-105i32) as u32 } else { 0 }),
        );
    }
    ops.extend([
        request(103, 255, 2, 10000),
        lifecycle(108, 10000),
        request(103, 0, 1, 10000),
        lifecycle(108, 10000),
        lifecycle(101, 10000),
    ]);
    // Coalesced last writer, additional status bit, stop scanning and stop idle.
    ops.extend([
        request(105, 1, 0, 10000),
        request(106, 2, 0, 10000),
        request(104, 3, 0, 10000),
        lifecycle(107, 10000),
        lifecycle(107, 10000),
        lifecycle(108, 10000),
        request(104, 4, 0, 10000),
        lifecycle(108, 10000),
    ]);
    for group in 1..=3 {
        ops.extend([
            request(103, group, group, 10000),
            lifecycle(108, 10000),
            request(104, group, 0, 10000),
            lifecycle(108, 10000),
        ]);
    }
    // Actual scan promotion via a token, early and duplicate deliveries.
    ops.extend([
        request(103, 8, 1, 10000),
        lifecycle(108, 10000),
        lifecycle(110, 10000).arg(1, 0),
        tick(Timer::Scan, 13000),
        tick(Timer::Scan, 14000),
        lifecycle(111, 14000).arg(1, 0).arg(2, 1),
    ]);
    // Real coordinator JOIN and presence, timeout threshold and eight report ticks.
    let own = wire::Identity {
        address_len: 5,
        address: [9, 8, 7, 6, 5, 0],
    };
    ops.push(rf_message(
        Message::JoinV3(wire::JoinV3 {
            origin: remote(2),
            target: own,
            capabilities: 2,
        }),
        0,
        1,
        14001,
    ));
    ops.push(rf_op(93, &outer(255, 2, 2, &[]), 14002000, 14002));
    // Test-domain scalar peer telemetry seed; the subsequent expiration is real.
    ops.push(JointOp::old(peer(1, 2, true, 14002)));
    for now in (15000..=22000).step_by(1000) {
        ops.push(tick(Timer::Status, now));
    }
    ops.push(lifecycle(109, 22000)); // pending deadline must remain unchanged
    ops.push(lifecycle(110, 22000).arg(1, Timer::Status as u8));
    // Active stop sends actual LEAVE with source sequence/address before quieting RF.
    ops.extend([
        request(104, 9, 0, 22000),
        lifecycle(108, 22000).tx_failure(false, -5),
        request(103, 10, 1, 22000),
        lifecycle(108, 22000),
    ]);
    for t in [Timer::Scan, Timer::Join, Timer::Status] {
        ops.push(lifecycle(111, 26000).arg(1, t as u8).arg(2, 1));
    }
    // RX-driven discovery and actual join retries (no Event::Membership injection).
    let sync = wire::SyncV3 {
        leader: remote(1),
        leader_id: 1,
        member_count: 2,
        term: 0,
        frame_counter: 1,
        phase_us: 100,
        relay_depth: 0,
    };
    ops.push(rf_message(Message::SyncV3(sync), 1, 1, 26000));
    for i in 0..=10 {
        ops.push(tick(Timer::Join, 26000 + i * 500));
    }
    ops.push(tick(Timer::Scan, 35000));
    ops.extend([
        request(104, 11, 0, 35001),
        lifecycle(108, 35001),
        request(103, 12, 1, 35002),
        lifecycle(108, 35002),
    ]);
    ops.push(rf_message(
        Message::SyncV3(wire::SyncV3 { term: 7, ..sync }),
        1,
        2,
        35003,
    ));
    // Save an actual pending JOIN, stop, then reject it against a new JOIN scope.
    ops.extend([
        lifecycle(110, 35003).arg(1, Timer::Join as u8),
        request(104, 15, 0, 35003),
        lifecycle(108, 35003),
        request(103, 16, 1, 35003),
        lifecycle(108, 35003),
    ]);
    ops.push(rf_message(
        Message::SyncV3(wire::SyncV3 { term: 7, ..sync }),
        1,
        3,
        35003,
    ));
    ops.push(lifecycle(111, 35003).arg(1, Timer::Join as u8).arg(2, 1));
    ops.push(tick(Timer::Join, 35003));
    ops.push(rf_message(
        Message::JoinAckV3(wire::JoinAckV3 {
            target: own,
            capabilities: 2,
            assigned_id: 2,
            slot_index: 1,
            coordinator_id: 1,
            term: 7,
        }),
        1,
        4,
        35004,
    ));
    ops.extend([
        lifecycle(107, 35004),
        lifecycle(108, 35004).scalar(24, (-105i32) as u32),
        lifecycle(112, 35004).arg(1, 1),
        lifecycle(107, 35004),
        lifecycle(108, 35004),
    ]);
    // Shared source audio epoch, bridge gates, delayed producer, RX receipt cutoff.
    ops.push(audio_op(66, &[], 35004));
    ops.extend([
        request(105, 13, 0, 35004),
        lifecycle(108, 35004),
        request(106, 14, 0, 35005),
        lifecycle(108, 35005),
    ]);
    ops.push(
        audio_op(67, &bundle(1, 2, true), 35005)
            .arg(1, 1)
            .arg(2, 13),
    );
    ops.push(audio_op(75, &[1, 3, 0, c::node_bit(2), 0], 35005));
    for (seq, receipt) in [(50, 35004999), (51, 35005000)] {
        let mut packet = outer(13, 3, seq, &bundle(u16::from(seq), 2, true));
        packet[5] = 5;
        ops.push(rf_op(93, &packet, receipt, 35005));
    }
    ops.push(audio_op(64, &bundle(2, 2, true), 35006));
    ops.push(audio_op(65, &[], 35006));
    ops.push(audio_op(64, &bundle(3, 2, true), 35006));
    // At exactly 5 seconds retain participant; > threshold invokes one real reset.
    ops.extend([
        lifecycle(109, 40003),
        lifecycle(109, 40004),
        lifecycle(109, 40005),
        tick(Timer::Status, 41000),
        tick(Timer::Scan, 45000),
        lifecycle(100, 45000),
        lifecycle(102, 45001),
    ]);
    ops
}

#[test]
fn strict_protocol_rejects_injected_membership_and_missing_algorithm_hooks() {
    struct MissingHook {
        generic_calls: usize,
    }
    impl Platform for MissingHook {
        fn integration(&self) -> Integration {
            Integration::ActorOracle
        }
        fn now_us(&self) -> i64 {
            0
        }
        fn owned_protocol_algorithms(&self) -> bool {
            true
        }
        fn apply(&mut self, _: Effect) -> Reply {
            self.generic_calls += 1;
            Reply::Value(0)
        }
    }
    let mut missing = MissingHook { generic_calls: 0 };
    let mut unsupported = Runtime::default();
    let token = TimerToken {
        timer: Timer::Scan,
        generation: 1,
        deadline_us: 0,
    };
    assert_eq!(
        unsupported.handle(Event::Timer(token), &mut missing),
        ENOTSUP
    );
    assert_eq!(missing.generic_calls, 0);
    assert_eq!(unsupported.init(&mut missing), ENOTSUP);
    assert!(!unsupported.owned_protocol_initialized);
    let calls = missing.generic_calls;
    assert_eq!(unsupported.start(&mut missing), ENOTSUP);
    assert_eq!(missing.generic_calls, calls);
    assert_eq!(unsupported.membership.state, State::Idle);
    assert!(unsupported.timers.iter().all(Option::is_none));
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    p.full_rx = true;
    p.full_protocol = true;
    let mut r = Runtime::default();
    let mut io = FullProtocolPlatform(&mut p);
    for event in [
        Event::Membership(Membership::default()),
        Event::InjectMembership(Membership::default()),
    ] {
        assert_eq!(r.handle(event, &mut io), ENOTSUP);
    }
    assert_eq!(io.apply(Effect::AdaptiveReset), Reply::Value(ENOTSUP));
    assert_eq!(
        io.apply(Effect::MembershipTimer(Timer::Scan)),
        Reply::Value(ENOTSUP)
    );
    assert_eq!(r.init(&mut io), 0);
    assert_eq!(r.start(&mut io), 0);
    let scan = r.timer(Timer::Scan).unwrap();
    io.0.inner.now = scan.deadline_us / 1000;
    assert_eq!(r.handle(Event::Timer(scan), &mut io), 0);
    assert_eq!(r.membership.state, State::Active);
    let seq = r.protocol.tx_seq;
    let frames = io.0.radio_frames.len();
    r.stop(&mut io);
    assert_eq!(r.protocol.tx_seq, seq.wrapping_add(1));
    assert_eq!(io.0.radio_frames.len(), frames + 1);
    assert_eq!(io.0.radio_frames.last().unwrap()[1], 4);
    assert_eq!(&io.0.radio_frames.last().unwrap()[9..], &[9, 8, 7, 6, 5]);
    assert_eq!(r.handle(Event::Timer(scan), &mut io), 0);
    assert_eq!(r.membership.state, State::Idle);
    assert!(r.timers.iter().all(Option::is_none));
}

#[test]
fn protocol_corpus_owned_lifecycle() {
    let ops = protocol_fixtures();
    let mut p = Joint::new();
    p.full_tx = true;
    p.full_audio = true;
    p.full_rx = true;
    let mut r = Runtime::default();
    let mut token = None;
    let mut saved = [None; 3];
    for op in &ops {
        run_protocol(&mut r, &mut p, &mut token, &mut saved, *op);
    }
    std::eprintln!("authored full-protocol corpus: {} operations", ops.len());
}

struct FaultedProtocol {
    backend: Joint,
    missing: Option<Effect>,
    failure: Option<i32>,
    fail_occurrence: usize,
    occurrences: usize,
    integration: Integration,
    calls: Vec<Effect>,
}
impl FaultedProtocol {
    fn new() -> Self {
        let mut backend = Joint::new();
        backend.full_tx = true;
        backend.full_audio = true;
        backend.full_rx = true;
        backend.full_protocol = true;
        Self {
            backend,
            missing: None,
            failure: None,
            fail_occurrence: 1,
            occurrences: 0,
            integration: Integration::ActorOracle,
            calls: Vec::new(),
        }
    }
    fn hardware() -> Self {
        let mut p = Self::new();
        p.integration = Integration::HardwareUnqualified;
        p.backend.integration = Integration::HardwareUnqualified;
        p
    }
}
impl Platform for FaultedProtocol {
    fn integration(&self) -> Integration {
        self.integration
    }
    fn now_us(&self) -> i64 {
        self.backend.now_us()
    }
    fn owned_protocol_algorithms(&self) -> bool {
        true
    }
    fn apply(&mut self, e: Effect) -> Reply {
        assert!(
            !crate::protocol_runtime::owned_effect(e),
            "owned effect reached generic IO"
        );
        self.calls.push(e);
        self.backend.apply(e)
    }
    fn dispatch_protocol(&mut self, r: &mut Runtime, e: Effect) -> Option<Reply> {
        if crate::protocol_runtime::owned_effect(e) {
            self.calls.push(e);
        }
        if self.missing == Some(e) {
            self.occurrences += 1;
            if self.occurrences == self.fail_occurrence {
                return self.failure.map(Reply::Value);
            }
        }
        FullProtocolPlatform(&mut self.backend).dispatch_protocol(r, e)
    }
}

#[test]
fn strict_init_checks_every_required_owned_reply_and_invalidates_previous_success() {
    for (effect, occurrence) in [
        (Effect::BindMembershipWork, 1),
        (Effect::AudioInit, 1),
        (Effect::RxInit, 1),
        (Effect::AdaptiveReset, 1),
        (Effect::AdaptiveReset, 2),
        (Effect::ResetRfTrackers, 1),
        (
            Effect::Ingress {
                enabled: false,
                purge: false,
            },
            1,
        ),
    ] {
        for failure in [None, Some(-5)] {
            let mut p = FaultedProtocol::new();
            let mut r = Runtime::default();
            assert_eq!(r.init(&mut p), 0);
            assert!(r.owned_protocol_initialized);
            p.missing = Some(effect);
            p.failure = failure;
            p.fail_occurrence = occurrence;
            assert_eq!(r.init(&mut p), failure.unwrap_or(ENOTSUP));
            assert!(!r.owned_protocol_initialized);
            p.calls.clear();
            assert_eq!(r.start(&mut p), ENOTSUP);
            assert!(p.calls.is_empty());
            assert_eq!(r.membership.state, State::Idle);
            assert!(r.timers.iter().all(Option::is_none));
            r.request_start(17, 2, &mut p);
            p.calls.clear();
            r.command_work_handler(&mut p);
            assert!(!p
                .calls
                .iter()
                .any(|e| matches!(e, Effect::SetChannel(_) | Effect::StartRx)));
            assert!(p.calls.contains(&Effect::Ack {
                command: Command {
                    kind: CommandKind::Start,
                    generation: 17,
                    talk_channel: 2
                },
                result: ENOTSUP,
            }));
        }
    }
    // Even successful hook replies cannot make a NotIntegrated backend ready.
    let mut p = FaultedProtocol::new();
    p.integration = Integration::NotIntegrated;
    let mut r = Runtime::default();
    assert_eq!(r.init(&mut p), ENOTSUP);
    assert!(!r.owned_protocol_initialized);
    p.calls.clear();
    assert_eq!(r.start(&mut p), ENOTSUP);
    assert!(p.calls.is_empty());
}

#[test]
fn strict_start_rolls_back_rx_when_scan_timeout_dispatch_disappears() {
    for failure in [None, Some(-5)] {
        let mut p = FaultedProtocol::new();
        let mut r = Runtime::default();
        assert_eq!(r.init(&mut p), 0);
        p.missing = Some(Effect::ScanTimeout);
        p.failure = failure;
        p.calls.clear();
        assert_eq!(r.start(&mut p), failure.unwrap_or(ENOTSUP));
        assert_eq!(
            p.calls,
            [
                Effect::ResetRfTrackers,
                Effect::StartRx,
                Effect::ScanTimeout,
                Effect::StopRx
            ]
        );
        assert_eq!(r.membership.state, State::Idle);
        assert!(r.timers.iter().all(Option::is_none));
    }
}

#[test]
fn successful_strict_init_and_start_preserve_lifecycle_effect_order() {
    let mut p = FaultedProtocol::new();
    let mut r = Runtime::default();
    assert_eq!(r.init(&mut p), 0);
    assert_eq!(
        p.calls,
        [
            Effect::InitTimer(Timer::Scan),
            Effect::InitTimer(Timer::Join),
            Effect::InitTimer(Timer::Status),
            Effect::InitCommand,
            Effect::BindMembershipWork,
            Effect::AudioInit,
            Effect::RxInit,
            Effect::SetControlCallback,
            Effect::GetAddress,
            Effect::AdaptiveReset,
            Effect::AdaptiveReset,
            Effect::ResetRfTrackers,
            Effect::Ingress {
                enabled: false,
                purge: false
            },
        ]
    );
    p.calls.clear();
    assert_eq!(r.start(&mut p), 0);
    let scan = r.timer(Timer::Scan).unwrap();
    assert_eq!(
        p.calls,
        [
            Effect::ResetRfTrackers,
            Effect::StartRx,
            Effect::ScanTimeout,
            Effect::Schedule(scan)
        ]
    );
    assert_eq!(r.membership.state, State::Scanning);
}

#[test]
fn integration_permissions_separate_algorithms_from_fixtures() {
    use crate::platform::{accepts_protocol_algorithms, allows_fixture_injection};
    for (integration, owned, expected) in [
        (Integration::ActorOracle, false, true),
        (Integration::ActorOracle, true, true),
        (Integration::HardwareUnqualified, false, false),
        (Integration::HardwareUnqualified, true, true),
        (Integration::NotIntegrated, false, false),
        (Integration::NotIntegrated, true, false),
    ] {
        assert_eq!(accepts_protocol_algorithms(integration, owned), expected);
        assert_eq!(
            allows_fixture_injection(integration),
            integration == Integration::ActorOracle
        );
    }
}

fn permission_state(r: &Runtime) -> Vec<u8> {
    let mut bytes = Vec::new();
    snapshot(&mut bytes, r);
    wrapper_snapshot(&mut bytes, r, &[(0, false); 256]);
    audio_snapshot(&mut bytes, r);
    rx_snapshot(&mut bytes, r);
    bytes.push(r.audio_state.initialized as u8);
    bytes
}

fn permission_seed() -> Runtime {
    let mut r = Runtime::default();
    r.membership.state = State::Active;
    r.membership.role = Role::Participant;
    r.membership.node_id = 2;
    r.membership.slot_index = 1;
    r.membership.coordinator_id = 1;
    r.address = [2; 5];
    openhelmet_mesh::adaptive::init(Some(&mut r.protocol.adaptive), 2, 1, 1, 7, 0);
    r.protocol.tx_seq = 255;
    r.protocol.control_turn = 255;
    r.protocol.air_report_seq = u32::MAX;
    r.audio_state.boundary.enabled = true;
    r.audio_state.boundary.epoch = 42;
    r.audio_state.boundary.receipt_cutoff_us = 123;
    r.audio_state.ingress_enabled = true;
    r.audio_state.work_pending = true;
    r.audio_state.initialized = true;
    r.audio_state.rf_seq[3] = crate::protocol_audio::RfSeq16 {
        last: 77,
        initialized: true,
    };
    r.session.active_speaker_ids = [2, 3];
    r.session.relay_masks = [1, 2];
    r.session.heard_bitmap = 7;
    r.session.relay_bitmap = 3;
    r.session.speaker_deadlines_ms[2] = 1500;
    r.session.speaker_active_since_ms[2] = 1;
    let payload = bundle(7, 2, true);
    let request = r.audio_capture_ingress().prepare(&payload).unwrap();
    r.audio_state.tx[0] = *request.entry();
    r.audio_state.tx_head = 1;
    r.audio_state.ingress[0] = *request.entry();
    r.audio_state.ingress_used = 1;
    r.audio_state.ingress_write = 1;
    let packet = outer(13, 3, 8, &payload);
    r.session.relay_ring[0].data[..packet.len()].copy_from_slice(&packet);
    r.session.relay_ring[0].len = packet.len() as u8;
    r.session.relay_head = 1;
    assert_eq!(r.tx_queue_control(14, &[0; wire::Topology::SIZE]), 0);
    assert_eq!(r.tx_queue_control(23, &[0; wire::SpeakerRequest::SIZE]), 0);
    assert_eq!(r.tx_queue_priority(16, &[1]), 0);
    let h = openhelmet_mesh::adaptive::Header {
        version: 5,
        packet_type: 19,
        src_id: 3,
        seq: 9,
        ttl: 2,
        flags: 0,
        talk_channel: 1,
        payload_len: wire::HandoverAck::SIZE as u16,
    };
    assert!(openhelmet_mesh::adaptive::forward_enqueue(
        Some(&mut r.protocol.adaptive),
        Some(&h),
        Some(&[0; wire::HandoverAck::SIZE]),
        None,
        1,
        true,
        0,
    ));
    r
}

fn reject_physical_routes(p: &mut impl AdaptivePlatform, asynchronous: bool) {
    use crate::corefuture::ready;
    for route in 0..6 {
        let mut r = permission_seed();
        if route != 0 {
            r.protocol.priority_pending = false;
        }
        if route == 2 {
            r.protocol.adaptive.clear_forward_queue();
        }
        if route == 3 {
            r.membership.role = Role::Coordinator;
        }
        if route == 4 {
            r.session.relay_tail = r.session.relay_head;
        }
        if route == 5 {
            r.audio_state.tx_tail = r.audio_state.tx_head;
            r.audio_state.ingress_used = 0;
        }
        let before = permission_state(&r);
        if asynchronous {
            assert_eq!(ready(r.tx_send_packet_async(1, &[0; 201], p)), ENOTSUP);
            assert_eq!(
                ready(r.tx_send_packet_ex_async(4, &[2; 5], 2, 0, 2, 7, p)),
                ENOTSUP
            );
            assert_eq!(ready(r.tx_send_packet_async(4, &[2; 5], p)), ENOTSUP);
            assert_eq!(ready(r.tx_send_join_request_async(p)), ENOTSUP);
            assert_eq!(ready(r.tx_send_sync_async(p)), ENOTSUP);
            ready(r.tx_control_handler_async(if route == 3 { 10 } else { 1 }, p));
            ready(r.audio_slot_tx_handler_async(1, 1, p));
        } else {
            assert_eq!(r.tx_send_packet(1, &[0; 201], p), ENOTSUP);
            assert_eq!(r.tx_send_packet_ex(4, &[2; 5], 2, 0, 2, 7, p), ENOTSUP);
            assert_eq!(r.tx_send_packet(4, &[2; 5], p), ENOTSUP);
            assert_eq!(r.tx_send_join_request(p), ENOTSUP);
            assert_eq!(r.tx_send_sync(p), ENOTSUP);
            r.tx_control_handler(if route == 3 { 10 } else { 1 }, p);
            r.audio_slot_tx_handler(1, 1, p);
        }
        assert_eq!(permission_state(&r), before, "physical route {route}");
    }
}

#[test]
fn direct_sync_physical_routes_reject_ineligible_platforms_without_mutation() {
    for integration in [Integration::NotIntegrated, Integration::HardwareUnqualified] {
        let mut p = FaultedProtocol::hardware().backend;
        p.integration = integration;
        reject_physical_routes(&mut p, false);
        reject_physical_routes(&mut CombinedMembershipPlatform(&mut p), false);
        reject_physical_routes(&mut CombinedFullTxPlatform(&mut p), false);
        reject_physical_routes(&mut FullAudioPlatform(&mut p), false);
        reject_physical_routes(&mut FullRxPlatform(&mut p), false);
        if integration == Integration::NotIntegrated {
            reject_physical_routes(&mut FullProtocolPlatform(&mut p), false);
        }
        assert!(p.inner.bytes.is_empty());
        assert!(p.radio_frames.is_empty());
    }
}

#[test]
fn direct_async_physical_routes_reject_ineligible_platforms_without_mutation() {
    for integration in [Integration::NotIntegrated, Integration::HardwareUnqualified] {
        let mut p = FaultedProtocol::hardware().backend;
        p.integration = integration;
        reject_physical_routes(&mut p, true);
        reject_physical_routes(&mut CombinedMembershipPlatform(&mut p), true);
        reject_physical_routes(&mut CombinedFullTxPlatform(&mut p), true);
        reject_physical_routes(&mut FullAudioPlatform(&mut p), true);
        reject_physical_routes(&mut FullRxPlatform(&mut p), true);
        if integration == Integration::NotIntegrated {
            reject_physical_routes(&mut FullProtocolPlatform(&mut p), true);
        }
        assert!(p.inner.bytes.is_empty());
        assert!(p.radio_frames.is_empty());
    }
}

#[test]
fn standalone_hardware_audio_dispatch_rejects_all_lifecycle_effects() {
    let mut p = FaultedProtocol::hardware().backend;
    let mut r = permission_seed();
    let before = permission_state(&r);
    for effect in [
        Effect::AudioInit,
        Effect::AudioCancel,
        Effect::ResetRfTrackers,
        Effect::Ingress {
            enabled: false,
            purge: true,
        },
        Effect::PurgeTx,
        Effect::ClearGrants,
        Effect::ClearBitmaps,
        Effect::LocalAudio(false),
        Effect::LocalAudio(true),
        Effect::UpdateGrants,
    ] {
        assert_eq!(
            FullAudioPlatform(&mut p).dispatch_audio(effect, &mut r),
            Some(Reply::Value(ENOTSUP))
        );
        assert_eq!(
            FullRxPlatform(&mut p).dispatch_audio(effect, &mut r),
            Some(Reply::Value(ENOTSUP))
        );
        assert_eq!(permission_state(&r), before);
    }
    assert_eq!(
        FullAudioPlatform(&mut p).dispatch_audio(Effect::SubmitCommand, &mut r),
        None
    );
    assert!(p.inner.bytes.is_empty());
    assert!(p.radio_frames.is_empty());
}

fn reject_audio_commands(p: &mut impl Platform, asynchronous: bool) {
    for kind in [CommandKind::Pause, CommandKind::Resume] {
        let mut r = permission_seed();
        let before = permission_state(&r);
        let command = Command {
            kind,
            generation: 17,
            talk_channel: 0,
        };
        assert_eq!(command.ack_payload(ENOTSUP), [kind as u8, 17, 255]);
        r.handle(Event::Command(command), p);
        r.request_status(p);
        if asynchronous {
            crate::corefuture::ready(r.command_work_handler_async(p));
        } else {
            r.command_work_handler(p);
        }
        assert_eq!(permission_state(&r), before);
        assert_eq!(r.pending_request(), 0);
        assert!(!r.status_pending());
    }
}

#[test]
fn standalone_hardware_pause_resume_failure_ack_without_audio_or_bridge_effects() {
    for asynchronous in [false, true] {
        for wrapper in 0..3 {
            let mut p = FaultedProtocol::hardware().backend;
            p.bridge_delivery = true;
            match wrapper {
                0 => reject_audio_commands(&mut p, asynchronous),
                1 => reject_audio_commands(&mut FullAudioPlatform(&mut p), asynchronous),
                _ => reject_audio_commands(&mut FullRxPlatform(&mut p), asynchronous),
            }
            let mut expected = Vec::new();
            for kind in [CommandKind::Pause, CommandKind::Resume] {
                expected.extend([b'E', 70, b'E', 70, b'E', 78, kind as u8, 17]);
                p32(&mut expected, ENOTSUP as u32);
                for _ in 0..2 {
                    expected.extend([
                        b'E',
                        21,
                        State::Active as u8,
                        Role::Participant as u8,
                        255,
                        2,
                        1,
                        1,
                    ]);
                    p32(&mut expected, 0);
                }
            }
            assert_eq!(p.inner.bytes, expected);
            assert!(p.radio_frames.is_empty());
            assert!(p.bridge_delivery);
            assert_eq!(p.bridge_gate_calls, 0);
            assert_eq!(p.bridge_flushes, 0);
        }
    }
}

#[test]
fn strict_hardware_and_oracle_physical_tx_and_audio_commands_mutate_real_state() {
    use crate::corefuture::ready;
    for integration in [Integration::HardwareUnqualified, Integration::ActorOracle] {
        for asynchronous in [false, true] {
            let mut p = FaultedProtocol::hardware().backend;
            p.integration = integration;
            p.clock_available = true;
            p.frame = 8;
            p.slot_remaining = 10000;
            let mut r = Runtime::default();
            let mut io = FullProtocolPlatform(&mut p);
            assert_eq!(r.init(&mut io), 0);
            assert_eq!(r.start(&mut io), 0);
            let scan = r.timer(Timer::Scan).unwrap();
            io.0.inner.now = scan.deadline_us / 1000;
            assert_eq!(r.handle(Event::Timer(scan), &mut io), 0);
            assert_eq!(r.membership.state, State::Active);
            let seq = r.protocol.tx_seq;
            let frames = io.0.radio_frames.len();
            if asynchronous {
                assert_eq!(
                    ready(r.tx_send_packet_ex_async(4, &[2; 5], 2, 0, 2, 7, &mut io)),
                    0
                );
                assert_eq!(ready(r.tx_send_packet_async(4, &[2; 5], &mut io)), 0);
                assert_eq!(ready(r.tx_send_join_request_async(&mut io)), 0);
                assert_eq!(ready(r.tx_send_sync_async(&mut io)), 0);
            } else {
                assert_eq!(r.tx_send_packet_ex(4, &[2; 5], 2, 0, 2, 7, &mut io), 0);
                assert_eq!(r.tx_send_packet(4, &[2; 5], &mut io), 0);
                assert_eq!(r.tx_send_join_request(&mut io), 0);
                assert_eq!(r.tx_send_sync(&mut io), 0);
            }
            assert_eq!(r.protocol.tx_seq, seq.wrapping_add(3));
            assert_eq!(io.0.radio_frames.len(), frames + 4);
            assert_eq!(r.protocol.last_sync_time, io.0.inner.now as u32);
            assert_eq!(r.tx_queue_priority(16, &[1]), 0);
            if asynchronous {
                ready(r.tx_control_handler_async(8, &mut io));
            } else {
                r.tx_control_handler(8, &mut io);
            }
            assert!(!r.protocol.priority_pending);
            assert_eq!(io.0.radio_frames.last().unwrap()[1], 16);
            assert_eq!(r.audio_send_v2(&bundle(7, 2, true), &mut io), 0);
            r.audio_ingress_work_handler(&mut io);
            assert_eq!(r.audio_state.tx_depth(), 1);
            if asynchronous {
                ready(r.audio_slot_tx_handler_async(0, 8, &mut io));
            } else {
                r.audio_slot_tx_handler(0, 8, &mut io);
            }
            assert_eq!(r.audio_state.tx_depth(), 0);
            assert_eq!(r.audio_state.counters.rf_audio_ok, 1);
            assert_eq!(io.0.radio_frames.last().unwrap()[1], 13);
            for kind in [CommandKind::Pause, CommandKind::Resume] {
                let epoch = r.audio_state.boundary.epoch;
                let command = Command {
                    kind,
                    generation: 17,
                    talk_channel: 0,
                };
                r.handle(Event::Command(command), &mut io);
                let trace_start = io.0.inner.bytes.len();
                if asynchronous {
                    ready(r.command_work_handler_async(&mut io));
                } else {
                    r.command_work_handler(&mut io);
                }
                assert_eq!(r.audio_state.boundary.epoch, epoch.wrapping_add(1));
                assert_eq!(r.audio_state.boundary.enabled, kind == CommandKind::Resume);
                assert_eq!(io.0.bridge_delivery, kind == CommandKind::Resume);
                let mut ack = vec![b'E', 78, kind as u8, 17];
                p32(&mut ack, 0);
                assert!(io.0.inner.bytes[trace_start..]
                    .windows(ack.len())
                    .any(|bytes| bytes == ack));
            }
            assert_eq!(io.0.bridge_gate_calls, 3);
        }
    }
}

#[test]
fn oracle_validation_order_and_notintegrated_legacy_audio_dispatch_are_preserved() {
    for asynchronous in [false, true] {
        let mut p = FaultedProtocol::new().backend;
        let mut r = Runtime::default();
        r.protocol.tx_seq = 255;
        let ret = if asynchronous {
            crate::corefuture::ready(r.tx_send_packet_async(1, &[0; 201], &mut p))
        } else {
            r.tx_send_packet(1, &[0; 201], &mut p)
        };
        assert_eq!(ret, crate::runtime::EINVAL);
        assert_eq!(r.protocol.tx_seq, 0);
        assert!(p.radio_frames.is_empty());
    }
    // P2 is hardware-only: the legacy NotIntegrated audio adapter is unchanged.
    let mut p = FaultedProtocol::new().backend;
    p.integration = Integration::NotIntegrated;
    let mut r = permission_seed();
    assert_eq!(
        FullAudioPlatform(&mut p).dispatch_audio(Effect::LocalAudio(false), &mut r),
        Some(Reply::Audio(r.audio_state.boundary))
    );
    assert!(!r.audio_state.boundary.enabled);
    assert_eq!(r.audio_state.boundary.epoch, 43);
    assert_eq!(r.audio_state.tx_depth(), 0);
    assert_eq!(r.audio_state.ingress_used(), 0);
    // LocalAudio(false) queries NowUs, which this backend does not trace.
    assert_eq!(r.audio_state.boundary.receipt_cutoff_us, p.inner.now * 1000);
    assert!(p.inner.bytes.is_empty());
    assert!(p.radio_frames.is_empty());
}

fn queued_audio_permission_seed() -> Runtime {
    let mut r = permission_seed();
    r.audio_state.ingress_used = 0;
    r.audio_state.ingress_write = 0;
    r.audio_state.work_pending = false;
    let mut oracle = FaultedProtocol::new().backend;
    assert_eq!(r.audio_send_v2(&bundle(8, 2, true), &mut oracle), 0);
    assert_eq!(oracle.inner.bytes, [b'E', 57]);
    assert_eq!(r.audio_state.ingress_used(), 1);
    assert!(r.audio_state.work_pending);
    r
}

fn reject_direct_audio_methods(p: &mut impl AdaptivePlatform) {
    let mut r = queued_audio_permission_seed();
    r.membership.role = Role::Coordinator;
    let before = permission_state(&r);
    r.audio_set_local_enabled(false, p);
    assert_eq!(permission_state(&r), before);
    r.audio_set_local_enabled(true, p);
    assert_eq!(permission_state(&r), before);
    r.audio_drain_ingress(p);
    assert_eq!(permission_state(&r), before);
    r.audio_ingress_work_handler(p);
    assert_eq!(permission_state(&r), before);
    r.audio_init(p);
    assert_eq!(permission_state(&r), before);
    r.audio_cancel_work(p);
    assert_eq!(permission_state(&r), before);
    r.audio_update_speaker_grants(p);
    assert_eq!(permission_state(&r), before);
    r.audio_expire_speakers(u32::MAX, p);
    assert_eq!(permission_state(&r), before);
    for data in [&[][..], &bundle(9, 2, true)[..]] {
        let prepared = r.audio_capture_ingress().validate(data);
        assert_eq!(r.audio_admit_prepared(prepared, p), ENOTSUP);
        assert_eq!(r.audio_send_v2(data, p), ENOTSUP);
        assert_eq!(permission_state(&r), before);
    }
    let request = r
        .audio_capture_ingress()
        .prepare(&bundle(9, 2, true))
        .unwrap();
    assert_eq!(r.audio_admit_ingress(request, p), ENOTSUP);
    assert_eq!(permission_state(&r), before);
    // Rejected init must not reset the low-level external ownership selection.
    let mut external = Runtime::default();
    external.use_external_ingress().unwrap();
    external.audio_init(p);
    assert_eq!(external.account_external_ingress_validation(0), Ok(()));
    let before = permission_state(&external);
    let invalid = external.audio_capture_ingress().validate(&[]);
    assert_eq!(external.consume_prepared_ingress(&invalid, p), ENOTSUP);
    assert_eq!(permission_state(&external), before);
}

#[test]
fn direct_audio_methods_reject_nonstrict_hardware_before_state_and_io() {
    let mut p = FaultedProtocol::hardware().backend;
    reject_direct_audio_methods(&mut p);
    reject_direct_audio_methods(&mut CombinedMembershipPlatform(&mut p));
    reject_direct_audio_methods(&mut CombinedFullTxPlatform(&mut p));
    reject_direct_audio_methods(&mut FullAudioPlatform(&mut p));
    reject_direct_audio_methods(&mut FullRxPlatform(&mut p));
    assert!(p.inner.bytes.is_empty());
    assert!(p.radio_frames.is_empty());
    assert_eq!(p.bridge_gate_calls, 0);
    assert_eq!(p.bridge_flushes, 0);
}

#[test]
fn direct_audio_drain_and_local_enable_preserve_strict_hardware_and_legacy_policy() {
    for integration in [
        Integration::HardwareUnqualified,
        Integration::ActorOracle,
        Integration::NotIntegrated,
    ] {
        for work_handler in [false, true] {
            let mut p = FaultedProtocol::new().backend;
            p.integration = integration;
            p.inner.now = 10;
            p.time_to_slot = 2000;
            let mut r = queued_audio_permission_seed();
            let mut io = FullProtocolPlatform(&mut p);
            if work_handler {
                r.audio_ingress_work_handler(&mut io);
            } else {
                r.audio_drain_ingress(&mut io);
            }
            assert_eq!(r.audio_state.ingress_used(), 0);
            assert_eq!(r.audio_state.work_pending, !work_handler);
            assert_eq!(r.audio_state.tx_depth(), 2);
            assert_eq!(r.audio_state.counters.spi_in_frames, 1);
            assert_eq!(r.audio_state.boundary.epoch, 42);
            assert!(io.0.inner.bytes.windows(2).any(|bytes| bytes == [b'E', 59]));
            r.audio_set_local_enabled(false, &mut io);
            assert_eq!(r.audio_state.boundary.epoch, 43);
            assert!(!r.audio_state.boundary.enabled);
            assert_eq!(r.audio_state.tx_depth(), 0);
            assert_eq!(r.audio_state.counters.tx_purge_drop, 2);
            assert_eq!(r.audio_state.boundary.receipt_cutoff_us, 10000);
            r.audio_set_local_enabled(true, &mut io);
            assert_eq!(r.audio_state.boundary.epoch, 44);
            assert!(r.audio_state.boundary.enabled);
            assert!(io.0.radio_frames.is_empty());
        }
    }
}

#[test]
fn direct_owned_oracle_audio_init_has_exact_backend_io() {
    let mut p = FaultedProtocol::new().backend;
    let mut r = Runtime::default();
    r.audio_init(&mut FullAudioPlatform(&mut p));
    assert!(r.audio_state.initialized);
    assert_eq!(p.inner.bytes, [b'E', 54, b'E', 55]);
    r.audio_cancel_work(&mut FullAudioPlatform(&mut p));
    assert_eq!(p.inner.bytes, [b'E', 54, b'E', 55, b'E', 56]);
    assert_eq!(
        r.audio_send_v2(&[], &mut FullAudioPlatform(&mut p)),
        crate::runtime::EINVAL
    );
    assert_eq!(r.audio_state.counters.bundle_bad, 1);
    assert_eq!(p.inner.bytes, [b'E', 54, b'E', 55, b'E', 56]);
}

fn reject_nonstrict_hardware(p: &mut impl AdaptivePlatform) {
    assert_eq!(p.integration(), Integration::HardwareUnqualified);
    assert!(!p.owned_protocol_algorithms());
    let mut r = Runtime::default();
    let before = r.snapshot();
    assert_eq!(r.init(p), ENOTSUP);
    assert_eq!(r.start(p), ENOTSUP);
    assert_eq!(
        r.membership_handle(Input::ResetSession, p),
        Err(Error::NotIntegrated)
    );
    assert_eq!(
        r.audio_process_rx_packet(&[], -40, 0, p),
        Err(Error::NotIntegrated)
    );
    let header = Header {
        version: 5,
        packet_type: 0,
        src_id: 0,
        seq: 0,
        ttl: 2,
        flags: 0,
        talk_channel: 1,
        payload_len: 0,
    };
    assert_eq!(
        r.adaptive_control_rx(header, &[], -40, 0, p),
        Err(Error::NotIntegrated)
    );
    let prepared = r
        .ingress_admission_gate()
        .capture()
        .validate(&bundle(1, 2, true));
    assert_eq!(
        r.audio_admit_ingress(*prepared.request().unwrap(), p),
        ENOTSUP
    );
    assert_eq!(r.snapshot(), before);
    assert_eq!(r.rx_state.cursors(), (0, 0));
    assert_eq!(r.audio_state.ingress_used(), 0);
    r.use_external_ingress().unwrap();
    assert_eq!(r.consume_prepared_ingress(&prepared, p), ENOTSUP);
    r.request_start(17, 2, p);
    r.command_work_handler(p);
    assert_eq!(r.snapshot(), before);
}

#[test]
fn hardware_standalone_wrappers_reject_before_algorithm_io_or_mutation() {
    let mut p = FaultedProtocol::hardware().backend;
    let mut r = Runtime::default();
    for wrapped in [false, true] {
        if wrapped {
            let mut io = FullRxPlatform(&mut p);
            assert_eq!(
                r.rx_publish(&[], -40, 0, &mut io),
                Err(Error::NotIntegrated)
            );
            assert_eq!(r.rx_work_handler(&mut io), Err(Error::NotIntegrated));
            assert_eq!(
                r.rx_process_packet(&[], -40, 0, &mut io),
                Err(Error::NotIntegrated)
            );
        } else {
            assert_eq!(r.rx_publish(&[], -40, 0, &mut p), Err(Error::NotIntegrated));
            assert_eq!(r.rx_work_handler(&mut p), Err(Error::NotIntegrated));
            assert_eq!(
                r.rx_process_packet(&[], -40, 0, &mut p),
                Err(Error::NotIntegrated)
            );
        }
    }
    assert!(p.inner.bytes.is_empty());
    assert_eq!(r.rx_state.cursors(), (0, 0));
    reject_nonstrict_hardware(&mut p);
    // Only command submission, ACK and status are allowed; no RF configuration.
    assert_eq!(p.inner.bytes.len(), 22);
    assert_eq!(&p.inner.bytes[..6], &[b'E', 70, b'E', 78, 1, 17]);
    assert_eq!(g32(&p.inner.bytes[6..]) as i32, ENOTSUP);
    assert!(p.radio_frames.is_empty());
    p.inner.bytes.clear();
    assert_eq!(
        Runtime::default().init(&mut FullRxPlatform(&mut p)),
        ENOTSUP
    );
    assert!(p.inner.bytes.is_empty());
    reject_nonstrict_hardware(&mut FullRxPlatform(&mut p));
    assert_eq!(p.inner.bytes.len(), 22);
    p.inner.bytes.clear();
    reject_nonstrict_hardware(&mut FullAudioPlatform(&mut p));
    assert_eq!(p.inner.bytes.len(), 22);
    p.inner.bytes.clear();
    reject_nonstrict_hardware(&mut CombinedFullTxPlatform(&mut p));
    assert_eq!(p.inner.bytes.len(), 22);
    p.inner.bytes.clear();
    reject_nonstrict_hardware(&mut CombinedMembershipPlatform(&mut p));
    assert_eq!(p.inner.bytes.len(), 22);
}

#[test]
fn strict_hardware_executes_real_lifecycle_membership_rx_and_audio() {
    let mut p = FaultedProtocol::hardware().backend;
    let mut r = Runtime::default();
    let mut io = FullProtocolPlatform(&mut p);
    assert_eq!(io.integration(), Integration::HardwareUnqualified);
    assert_eq!(r.start(&mut io), ENOTSUP);
    assert!(io.0.inner.bytes.is_empty());
    assert_eq!(r.init(&mut io), 0);
    assert!(r.owned_protocol_initialized);
    r.request_start(17, 2, &mut io);
    r.command_work_handler(&mut io);
    assert_eq!(r.talk_channel, 2);
    assert_eq!(r.membership.state, State::Scanning);
    let scan = r.timer(Timer::Scan).unwrap();
    io.0.inner.now = scan.deadline_us / 1000;
    assert_eq!(
        crate::corefuture::ready(r.handle_async(Event::Timer(scan), &mut io)),
        0
    );
    assert_eq!(r.membership.state, State::Active);
    assert_eq!(r.membership.role, Role::Coordinator);
    assert_eq!(
        r.membership_handle(Input::NotePresence(2), &mut io),
        Ok(false)
    );
    let mut packet = outer(13, 2, 1, &bundle(1, 2, true));
    packet[6] = 2;
    let now = io.now_us();
    assert_eq!(r.rx_publish(&packet, -40, now, &mut io), Ok(true));
    assert_eq!(
        crate::corefuture::ready(r.rx_work_handler_async(&mut io)),
        Ok(())
    );
    assert_eq!(r.rx_state.count(), 1);
    assert_eq!(r.rx_state.cursors().0, r.rx_state.cursors().1);
    assert_eq!(r.audio_send_v2(&bundle(2, 2, true), &mut io), 0);
    assert_eq!(r.audio_state.ingress_used(), 1);
    r.audio_ingress_work_handler(&mut io);
    r.use_external_ingress().unwrap();
    let prepared = r
        .ingress_admission_gate()
        .capture()
        .validate(&bundle(3, 2, true));
    assert_eq!(r.consume_prepared_ingress(&prepared, &mut io), 0);
    let ticks = r.auto_ticks;
    crate::corefuture::ready(r.status_work_handler_async(&mut io));
    assert_eq!(r.auto_ticks, ticks + 1);
    assert_eq!(
        FullRxPlatform(&mut io).integration(),
        Integration::HardwareUnqualified
    );
    assert!(FullAudioPlatform(&mut io).owned_protocol_algorithms());
    assert!(CombinedFullTxPlatform(&mut io).owned_protocol_algorithms());
    assert!(CombinedMembershipPlatform(&mut io).owned_protocol_algorithms());
}

#[test]
fn hardware_denies_direct_and_typed_membership_fixtures() {
    struct TypedFixture {
        owned: bool,
        reply: Reply,
    }
    impl Platform for TypedFixture {
        fn integration(&self) -> Integration {
            Integration::HardwareUnqualified
        }
        fn owned_protocol_algorithms(&self) -> bool {
            self.owned
        }
        fn now_us(&self) -> i64 {
            i64::MAX
        }
        fn apply(&mut self, _: Effect) -> Reply {
            self.reply
        }
        fn dispatch_protocol(&mut self, _: &mut Runtime, _: Effect) -> Option<Reply> {
            Some(self.reply)
        }
    }
    for owned in [false, true] {
        let mut p = FaultedProtocol::hardware().backend;
        let mut r = Runtime::default();
        let next = Membership {
            state: State::Active,
            role: Role::Coordinator,
            ..Membership::default()
        };
        let mut strict = FullProtocolPlatform(&mut p);
        assert_eq!(r.init(&mut strict), 0);
        assert_eq!(r.start(&mut strict), 0);
        let before = r.snapshot();
        let bytes = strict.0.inner.bytes.len();
        let timers = r.timers;
        for event in [Event::Membership(next), Event::InjectMembership(next)] {
            let result = if owned {
                r.handle(event, &mut strict)
            } else {
                r.handle(event, strict.0)
            };
            assert_eq!(result, ENOTSUP);
        }
        for transition in [
            MembershipTransition::Apply {
                next,
                timers: [Some(TimerDirective::Schedule(Timer::Status, 1)), None, None],
            },
            MembershipTransition::CoordinatorLost,
        ] {
            if owned {
                crate::corefuture::ready(r.apply_membership_impl::<false>(transition, &mut strict));
            } else {
                crate::corefuture::ready(r.apply_membership_impl::<false>(transition, strict.0));
            }
            let mut typed = TypedFixture {
                owned,
                reply: Reply::Membership(transition),
            };
            let scan = r.timer(Timer::Scan).unwrap();
            assert_eq!(r.handle(Event::Timer(scan), &mut typed), ENOTSUP);
        }
        assert_eq!(r.snapshot(), before);
        assert_eq!(r.timers, timers);
        assert_eq!(strict.0.inner.bytes.len(), bytes);
    }
}

#[test]
fn hardware_missing_or_failed_strict_hooks_keep_start_closed() {
    for effect in [
        Effect::BindMembershipWork,
        Effect::AudioInit,
        Effect::RxInit,
        Effect::AdaptiveReset,
        Effect::ResetRfTrackers,
        Effect::Ingress {
            enabled: false,
            purge: false,
        },
    ] {
        for failure in [None, Some(-5)] {
            let mut p = FaultedProtocol::hardware();
            let mut r = Runtime::default();
            assert_eq!(r.init(&mut p), 0);
            p.missing = Some(effect);
            p.failure = failure;
            assert_eq!(r.init(&mut p), failure.unwrap_or(ENOTSUP));
            assert!(!r.owned_protocol_initialized);
            p.calls.clear();
            assert_eq!(r.start(&mut p), ENOTSUP);
            assert!(p.calls.is_empty());
            r.request_start(17, 2, &mut p);
            p.calls.clear();
            r.command_work_handler(&mut p);
            assert!(!p
                .calls
                .iter()
                .any(|e| matches!(e, Effect::SetChannel(_) | Effect::StartRx)));
            assert_eq!(r.membership.state, State::Idle);
        }
    }
}
