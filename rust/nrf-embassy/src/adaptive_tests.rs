//! Joint membership+adaptive+TX corpus replay with Rust-side effect review. IO implementations below are explicitly
//! ALGORITHM MOCKS; successful queue replies do not claim actual RF sends.
use super::*;
use crate::protocol_adaptive::{
    AdaptiveEffect as A, AdaptiveIo, AdaptivePlatform, AdaptiveReply as R,
    CombinedMembershipPlatform,
};
use openhelmet_wire::mesh::Message;

struct Joint {
    integration: Integration,
    inner: Simulation,
    frame: u32,
    queue_result: i32,
    priority_result: i32,
    trackers: [(u16, bool); 256],
    grant_count: u8,
    grant_ids: [u8; 2],
    grant_masks: [u8; 2],
    effects: Vec<A>,
    full_tx: bool,
    clock_available: bool,
    clock_phase: u16,
    radio_frames: Vec<Vec<u8>>,
    full_audio: bool,
    full_rx: bool,
    full_protocol: bool,
    channel_result: i32,
    start_result: i32,
    bridge_delivery: bool,
    bridge_flushes: u32,
    bridge_gate_calls: u32,
    rx_pending: bool,
    rx_submit_result: i32,
    runtime_effects: Vec<Effect>,
    slot_remaining: u32,
    time_to_slot: i32,
    bridge_result: i32,
}
impl Joint {
    fn new() -> Self {
        Self {
            integration: Integration::ActorOracle,
            inner: Simulation {
                bytes: Vec::new(),
                now: 0,
                bind_mask: 255,
                bind_index: 0,
                sequence: true,
                logs: true,
            },
            frame: 0,
            queue_result: 0,
            priority_result: 0,
            trackers: [(0, false); 256],
            grant_count: 255,
            grant_ids: [0; 2],
            grant_masks: [0; 2],
            effects: Vec::new(),
            full_tx: false,
            clock_available: false,
            clock_phase: 0,
            radio_frames: Vec::new(),
            full_audio: false,
            full_rx: false,
            full_protocol: false,
            channel_result: 0,
            start_result: 0,
            bridge_delivery: false,
            bridge_flushes: 0,
            bridge_gate_calls: 0,
            rx_pending: false,
            rx_submit_result: 1,
            runtime_effects: Vec::new(),
            slot_remaining: 0,
            time_to_slot: 0,
            bridge_result: 0,
        }
    }
    fn write_entry(io: &mut AdaptiveIo<'_>, entry: &mut RelayEntry, t: u8, payload: &[u8]) {
        let h = Header {
            version: 5,
            packet_type: t,
            src_id: io.membership.node_id,
            seq: *io.tx_seq,
            ttl: 2,
            flags: 0,
            talk_channel: io.talk_channel,
            payload_len: payload.len() as u16,
        };
        *io.tx_seq = io.tx_seq.wrapping_add(1);
        wire::encode(&h, payload, &mut entry.data, 200).unwrap();
        entry.len = (9 + payload.len()) as u8;
    }
    fn queue(&mut self, mut io: AdaptiveIo<'_>, priority: bool, t: u8, payload: &[u8]) -> i32 {
        self.inner.e(40);
        self.inner.bytes.extend([priority as u8, t]);
        self.inner
            .bytes
            .extend((payload.len() as u16).to_le_bytes());
        self.inner.bytes.extend(payload);
        let mut result = if priority {
            self.priority_result
        } else {
            self.queue_result
        };
        if result == 0 && priority && *io.priority_pending {
            result = -105;
        }
        if result == 0 {
            if priority {
                // Work on the borrowed storage, retaining all stale suffix bytes.
                let mut entry = *io.priority_control;
                Self::write_entry(&mut io, &mut entry, t, payload);
                *io.priority_control = entry;
                *io.priority_pending = true;
            } else {
                let mut found = None;
                if matches!(t, 14 | 22 | 23 | 9) {
                    let mut i = *io.control_tail;
                    while i != *io.control_head {
                        if io.control_ring[usize::from(i)].data[1] == t {
                            found = Some(i);
                            break;
                        }
                        i = (i + 1) % 32;
                    }
                }
                if let Some(i) = found {
                    let entry = &mut io.control_ring[usize::from(i)];
                    let revision = if t == 22 { g32(&entry.data[13..]) } else { 0 };
                    entry.data[9..9 + payload.len()].copy_from_slice(payload);
                    entry.data[7..9].copy_from_slice(&(payload.len() as u16).to_le_bytes());
                    entry.len = (9 + payload.len()) as u8;
                    if t == 22 {
                        entry.data[13..17].copy_from_slice(&revision.to_le_bytes());
                    }
                } else {
                    let next = (*io.control_head + 1) % 32;
                    if next == *io.control_tail {
                        *io.control_drops = io.control_drops.wrapping_add(1);
                        result = -105;
                    } else {
                        let i = usize::from(*io.control_head);
                        let mut entry = io.control_ring[i];
                        Self::write_entry(&mut io, &mut entry, t, payload);
                        if t == 22 {
                            let revision = a::next_membership_revision(Some(io.adaptive));
                            entry.data[13..17].copy_from_slice(&revision.to_le_bytes());
                        }
                        io.control_ring[i] = entry;
                        *io.control_head = next;
                    }
                }
            }
        }
        p32(&mut self.inner.bytes, result as u32);
        result
    }
    fn update_grants(&mut self, speakers: &mut [u8; 2], masks: &mut [u8; 2]) {
        self.inner.e(13);
        self.inner.bytes.push(self.grant_count);
        self.inner.bytes.extend(self.grant_ids);
        self.inner.bytes.extend(self.grant_masks);
        if self.grant_count <= 2 {
            speakers.fill(0);
            masks.fill(0);
            let n = usize::from(self.grant_count);
            speakers[..n].copy_from_slice(&self.grant_ids[..n]);
            masks[..n].copy_from_slice(&self.grant_masks[..n]);
        }
    }
}
impl Platform for Joint {
    fn integration(&self) -> Integration {
        self.integration
    }
    fn now_us(&self) -> i64 {
        self.inner.now_us()
    }
    fn apply(&mut self, e: Effect) -> Reply {
        if self.full_protocol {
            match e {
                Effect::InitTimer(t) => {
                    self.inner.e(66);
                    self.inner.bytes.push(t as u8);
                }
                Effect::InitCommand => self.inner.e(69),
                Effect::SubmitCommand => self.inner.e(70),
                Effect::SetControlCallback => self.inner.e(71),
                Effect::SetChannel(channel) => {
                    self.inner.e(72);
                    self.inner.bytes.push(channel);
                    p32(&mut self.inner.bytes, self.channel_result as u32);
                    return Reply::Value(self.channel_result);
                }
                Effect::StartRx => {
                    self.inner.e(73);
                    p32(&mut self.inner.bytes, self.start_result as u32);
                    return Reply::Value(self.start_result);
                }
                Effect::StopRx => self.inner.e(74),
                Effect::TdmaStop => self.inner.e(17),
                Effect::RadioStats => self.inner.e(75),
                Effect::TdmaStats => self.inner.e(76),
                Effect::WsDiag => self.inner.e(77),
                Effect::Ack { command, result } => {
                    self.inner.e(78);
                    self.inner
                        .bytes
                        .extend([command.kind as u8, command.generation]);
                    p32(&mut self.inner.bytes, result as u32);
                }
                Effect::PeerCount(_) => {}
                Effect::Status(m) => {
                    self.inner.e(21);
                    self.inner.bytes.extend([
                        m.state as u8,
                        m.role as u8,
                        m.peer_count,
                        m.node_id,
                        m.slot_index as u8,
                        m.coordinator_id,
                    ]);
                    p32(&mut self.inner.bytes, self.bridge_result as u32);
                    return Reply::Value(self.bridge_result);
                }
                Effect::MeshStopped => {
                    self.inner.e(22);
                    self.inner.bytes.extend([6, 0]);
                }
                Effect::DiscardBridgeAudio => {
                    self.inner.e(79);
                    self.bridge_flushes += 1;
                }
                Effect::BridgeDelivery(enabled) => {
                    self.inner.e(80);
                    self.inner.bytes.push(enabled as u8);
                    self.bridge_delivery = enabled;
                    self.bridge_flushes += 1;
                    self.bridge_gate_calls += 1;
                }
                _ => return self.inner.apply(e),
            }
            return Reply::None;
        }
        if self.full_rx
            && matches!(
                e,
                Effect::InitTimer(_)
                    | Effect::InitCommand
                    | Effect::BindMembershipWork
                    | Effect::RxInit
                    | Effect::SetControlCallback
                    | Effect::AdaptiveReset
                    | Effect::TdmaStop
                    | Effect::StopRx
                    | Effect::RxStop
                    | Effect::PeerCount(_)
                    | Effect::Status(_)
                    | Effect::MeshStopped
            )
        {
            self.runtime_effects.push(e);
            return Reply::None;
        }
        self.inner.apply(e)
    }
}
impl MembershipPlatform for Joint {
    fn membership_effect(&mut self, e: M, s: &mut ProtocolState) -> bool {
        if let M::Status(m) = e {
            if self.full_protocol {
                self.apply(Effect::Status(m));
                return false;
            }
        }
        match e {
            M::AdaptiveReset
            | M::BindMember { .. }
            | M::ControlSequence { .. }
            | M::AdaptiveMembers { .. }
            | M::SlotMap
            | M::JoinAck { .. }
            | M::JoinRequest => panic!("combined wrapper escaped to standalone mock: {e:?}"),
            M::ResetTracker(id) if c::node_id_valid(id) => {
                self.trackers[usize::from(id)] = (0, false);
            }
            M::ResetTrackers => self.trackers.fill((0, false)),
            _ => {}
        }
        self.inner.membership_effect(e, s)
    }
}
impl AdaptivePlatform for Joint {
    fn owned_audio_wrappers(&self) -> bool {
        self.full_audio
    }
    fn audio_effect(&mut self, e: crate::protocol_audio::AudioEffect<'_>) -> i64 {
        use crate::protocol_audio::AudioEffect as E;
        assert!(self.full_audio);
        match e {
            E::InitWork => self.inner.e(54),
            E::SetSlotCallback => self.inner.e(55),
            E::CancelWork => self.inner.e(56),
            E::SubmitWork => self.inner.e(57),
            E::NowUs => return self.inner.now * 1000,
            E::SlotRemainingUs => {
                self.inner.e(58);
                p32(&mut self.inner.bytes, self.slot_remaining);
                return i64::from(self.slot_remaining);
            }
            E::TimeToSlotUs => {
                self.inner.e(59);
                p32(&mut self.inner.bytes, self.time_to_slot as u32);
                return i64::from(self.time_to_slot);
            }
            E::BridgeDeliver { source, bundle } => {
                self.inner.e(60);
                self.inner.bytes.extend([source, bundle.len() as u8]);
                self.inner.bytes.extend(bundle);
                p32(&mut self.inner.bytes, self.bridge_result as u32);
                return i64::from(self.bridge_result);
            }
            E::Starvation {
                total,
                role,
                node,
                slot,
                depth,
                ingress,
                time_to_slot,
            } => {
                let text = std::format!("[TXSTARVE] total={total} r={} id={node} sl={slot} q={depth} mq={ingress} tts={time_to_slot}\n", role as u8);
                self.inner.e(61);
                self.inner.bytes.extend((text.len() as u16).to_le_bytes());
                self.inner.bytes.extend(text.as_bytes());
            }
        }
        0
    }
    fn owned_tx_wrappers(&self) -> bool {
        self.full_tx
    }
    fn clock_snapshot(&mut self) -> Option<crate::protocol_tx::ClockSnapshot> {
        assert!(self.full_tx);
        self.inner.e(50);
        self.inner.bytes.push(self.clock_available as u8);
        if !self.clock_available {
            return None;
        }
        p32(&mut self.inner.bytes, self.frame);
        self.inner.bytes.extend(self.clock_phase.to_le_bytes());
        Some(crate::protocol_tx::ClockSnapshot {
            frame: self.frame,
            phase_us: self.clock_phase,
        })
    }
    fn radio_send(&mut self, packet: crate::protocol_tx::RadioSend<'_>) -> i32 {
        assert!(self.full_tx);
        assert_eq!(packet.pipe, 0);
        assert_eq!(packet.address, [0xe7; 5]);
        assert!(packet.bytes.len() <= 209);
        self.inner.e(51);
        self.inner.bytes.push(packet.pipe);
        self.inner.bytes.extend(packet.address);
        self.inner
            .bytes
            .extend((packet.bytes.len() as u16).to_le_bytes());
        self.inner.bytes.extend(packet.bytes);
        p32(&mut self.inner.bytes, self.queue_result as u32);
        self.radio_frames.push(packet.bytes.to_vec());
        if self.full_audio {
            self.slot_remaining = self
                .slot_remaining
                .saturating_sub(129 + (11 + packet.bytes.len() as u32) * 4);
        }
        self.queue_result
    }
    fn adaptive_effect(&mut self, e: A, io: AdaptiveIo<'_>) -> R {
        if self.full_tx {
            assert!(
                !matches!(
                    e,
                    A::Queue { .. } | A::SlotSnapshot | A::JoinRequest | A::JoinAck { .. }
                ),
                "owned TX escaped to mock algorithm: {e:?}"
            );
        }
        self.effects.push(e);
        match e {
            A::Queue {
                priority,
                packet_type,
                payload,
                len,
            } => {
                return R::Accepted(self.queue(
                    io,
                    priority,
                    packet_type,
                    &payload[..usize::from(len)],
                ))
            }
            A::JoinRequest => {
                self.inner.e(15);
                *io.tx_seq = io.tx_seq.wrapping_add(1);
                return R::Accepted(self.queue_result);
            }
            A::JoinAck { id, slot, address } => {
                self.inner.e(16);
                self.inner.bytes.extend([id, slot]);
                self.inner.bytes.extend(address);
                let mut target = wire::Identity {
                    address_len: 5,
                    address: [0; 6],
                };
                target.address[..5].copy_from_slice(&address);
                let ack = wire::JoinAckV3 {
                    target,
                    capabilities: 2,
                    assigned_id: id,
                    slot_index: slot,
                    coordinator_id: io.membership.node_id,
                    term: io.term,
                };
                let mut payload = [0; wire::JoinAckV3::SIZE];
                ack.encode(&mut payload).unwrap();
                return R::Accepted(self.queue(io, true, 16, &payload));
            }
            A::SlotSnapshot => {
                self.inner.e(41);
                let mut map = c::SlotMap::default();
                for peer in io.peers {
                    if peer.active && peer.announced && (0..8).contains(&peer.slot) {
                        map.slot_ids[peer.slot as usize] = peer.id;
                        map.slot_count = map.slot_count.max(peer.slot as u8 + 1);
                    }
                }
                if (0..8).contains(&io.membership.slot_index) {
                    map.slot_ids[io.membership.slot_index as usize] = io.membership.node_id;
                    map.slot_count = map.slot_count.max(io.membership.slot_index as u8 + 1);
                }
                for i in 0..2 {
                    if io.speakers[i] != 0 {
                        let n = usize::from(map.active_speaker_count);
                        map.active_speaker_count += 1;
                        map.active_speaker_ids[n] = io.speakers[i];
                        map.relay_masks[n] = io.masks[i];
                    }
                }
                slots(&mut self.inner.bytes, map);
                return R::SlotMap(map);
            }
            A::FrameCounter => {
                self.inner.e(42);
                p32(&mut self.inner.bytes, self.frame);
                return R::Frame(self.frame);
            }
            A::ResetTracker(id) => {
                self.inner.e(5);
                self.inner.bytes.push(id);
                if c::node_id_valid(id) {
                    self.trackers[usize::from(id)] = (0, false);
                }
            }
            A::ApplySlotMap(map) => {
                self.inner.e(45);
                slots(&mut self.inner.bytes, map);
                io.speakers.fill(0);
                io.masks.fill(0);
                let n = usize::from(map.active_speaker_count.min(2));
                io.speakers[..n].copy_from_slice(&map.active_speaker_ids[..n]);
                io.masks[..n].copy_from_slice(&map.relay_masks[..n]);
            }
            A::ApplyGrant { count, ids, masks } => {
                self.inner.e(46);
                self.inner.bytes.push(count);
                self.inner.bytes.extend(ids);
                self.inner.bytes.extend(masks);
                io.speakers.fill(0);
                io.masks.fill(0);
                let n = usize::from(count.min(2));
                io.speakers[..n].copy_from_slice(&ids[..n]);
                io.masks[..n].copy_from_slice(&masks[..n]);
            }
            A::ApplyRelease { count, ids } => {
                self.inner.e(47);
                self.inner.bytes.push(count);
                self.inner.bytes.extend(ids);
                for id in &ids[..usize::from(count.min(2))] {
                    for i in 0..2 {
                        if io.speakers[i] == *id {
                            io.speakers[i] = 0;
                            io.masks[i] = 0;
                        }
                    }
                }
            }
            A::UpdateGrants => self.update_grants(io.speakers, io.masks),
            A::ExpireSpeakers(now) => {
                self.inner.e(48);
                p32(&mut self.inner.bytes, now);
                if io.membership.role == Role::Coordinator {
                    for id in *io.speakers {
                        if id != 0
                            && io.deadlines_ms[usize::from(id)] <= i64::from(now)
                            && !a::speaker_request_active(Some(io.adaptive), id, now)
                        {
                            self.update_grants(io.speakers, io.masks);
                            break;
                        }
                    }
                }
            }
            A::ClockSource(local) => {
                self.inner.e(19);
                self.inner.bytes.push(local as u8);
            }
            A::SetSlot(slot) => {
                self.inner.e(43);
                self.inner.bytes.push(slot as u8);
            }
            A::Sync {
                frame,
                drift,
                base_us,
            } => {
                self.inner.e(44);
                p32(&mut self.inner.bytes, frame);
                self.inner.bytes.extend(drift.to_le_bytes());
                p64(&mut self.inner.bytes, base_us as u64);
            }
            A::Status(m) => {
                self.inner.e(21);
                self.inner.bytes.extend([
                    m.state as u8,
                    m.role as u8,
                    m.peer_count,
                    m.node_id,
                    m.slot_index as u8,
                    m.coordinator_id,
                ]);
            }
        }
        R::Applied
    }
}

fn wrapper_snapshot(v: &mut Vec<u8>, r: &Runtime, trackers: &[(u16, bool); 256]) {
    let s = &r.protocol;
    for n in s.direct_seq {
        p32(v, n);
    }
    v.extend(s.direct_seq_valid.map(u8::from));
    v.extend(s.direct_quality);
    v.extend(s.direct_rssi.map(|x| x as u8));
    for n in s.direct_seen {
        p32(v, n);
    }
    for n in [
        s.graph_report_seq,
        s.air_report_seq,
        s.local_request_seq,
        s.last_local_active_ms,
        s.last_request_ms,
    ] {
        p32(v, n);
    }
    v.push(s.local_request_active as u8);
    for n in [
        s.last_prepare_tx,
        s.last_commit_tx,
        s.last_forward_sync,
        s.last_sync_log,
        s.convergence_until_ms,
        s.grant_hold_until_ms,
        s.last_membership_tx,
    ] {
        p32(v, n);
    }
    v.push(s.authoritative_members);
    v.extend(s.join_last_seq);
    v.extend(s.ack_last_seq);
    v.extend([s.join_seq_seen, s.ack_seq_seen]);
    v.extend(r.session.active_speaker_ids);
    v.extend(r.session.relay_masks);
    v.extend(s.priority_control.data);
    v.push(s.priority_control.len);
    p32(v, s.priority_control.queued_ms);
    v.extend([s.priority_pending as u8, s.tx_seq, s.control_turn]);
    for n in [
        s.stat_control_ring_drop,
        s.stat_tx_fail,
        s.stat_topology_tx_ok,
        s.stat_topology_rx_direct,
        s.stat_topology_rx_relayed,
    ] {
        p32(v, n);
    }
    for (last, initialized) in trackers {
        v.extend(last.to_le_bytes());
        v.push(*initialized as u8);
    }
}

#[derive(Clone, Copy)]
struct JointOp([u8; 256], ReviewCheck);

#[derive(Clone, Copy)]
enum ReviewCheck {
    None,
    Unchanged {
        frame_query: bool,
    },
    GrantReply {
        count: u8,
        ids: [u8; 2],
        masks: [u8; 2],
    },
    Published {
        ids: [u8; 2],
        masks: [u8; 2],
        pruned: bool,
    },
    AppliedGrant {
        ids: [u8; 2],
        masks: [u8; 2],
    },
    DirectSequence(u32),
    LocalSequence(u32),
    MembershipRevision(u32),
    Authority {
        context_term: u32,
        context_leader: u8,
        shared_term: u32,
        shared_leader: u8,
        handled: bool,
        observation: AuthorityObservation,
    },
}

#[derive(Clone, Copy)]
enum AuthorityObservation {
    Unchanged,
    Sync,
    Report,
    SlotMap,
    LocalVoice,
    Publication,
}
impl JointOp {
    fn new(code: u8, now: i64) -> Self {
        let mut x = [0; 256];
        x[..80].copy_from_slice(&Op::new(code, now).0);
        x[220] = 255;
        x[255] = 1;
        Self(x, ReviewCheck::None)
    }
    fn old(op: Op) -> Self {
        let mut x = Self::new(op.0[0], 0);
        x.0[..80].copy_from_slice(&op.0);
        x
    }
    fn arg(mut self, i: usize, n: u8) -> Self {
        self.0[i] = n;
        self
    }
    fn expect(mut self, check: ReviewCheck) -> Self {
        self.1 = check;
        self
    }
    fn scalar(mut self, i: usize, n: u32) -> Self {
        self.0[i..i + 4].copy_from_slice(&n.to_le_bytes());
        self
    }
    fn grant_reply(mut self, count: u8, ids: [u8; 2], masks: [u8; 2]) -> Self {
        self.0[220] = count;
        self.0[221..223].copy_from_slice(&ids);
        self.0[223..225].copy_from_slice(&masks);
        self
    }
    fn frame(mut self, frame: u32) -> Self {
        self.0[240..244].copy_from_slice(&frame.to_le_bytes());
        self
    }
    fn identity(mut self, id: wire::Identity) -> Self {
        id.encode(&mut self.0[20..27]).unwrap();
        self
    }
    fn tx_failure(mut self, priority: bool, result: i32) -> Self {
        let i = if priority { 248 } else { 244 };
        self.0[i..i + 4].copy_from_slice(&result.to_le_bytes());
        self
    }
    fn packet(msg: wire::Message<'_>, src: u8, seq: u8, now: i64) -> Self {
        let mut op = Self::new(27, now);
        let len = msg.encode(&mut op.0[36..115]).unwrap();
        op.0[27..36].copy_from_slice(&[5, msg.packet_type(), src, seq, 2, 0, 1, len as u8, 0]);
        op.0[2] = (-60i8) as u8;
        op.0[232..240].copy_from_slice(&(now * 1000).to_le_bytes());
        op
    }
}
fn header(x: &[u8]) -> Header {
    Header {
        version: x[27],
        packet_type: x[28],
        src_id: x[29],
        seq: x[30],
        ttl: x[31],
        flags: x[32],
        talk_channel: x[33],
        payload_len: u16::from_le_bytes([x[34], x[35]]),
    }
}
fn run_joint(r: &mut Runtime, p: &mut Joint, op: JointOp) -> bool {
    let x = op.0;
    p.inner.now = g64(&x[8..]);
    p.inner.logs = x[255] != 0;
    p.frame = g32(&x[240..]);
    p.queue_result = g32(&x[244..]) as i32;
    p.priority_result = g32(&x[248..]) as i32;
    p.clock_phase = u16::from_le_bytes([x[252], x[253]]);
    p.clock_available = x[254] != 0;
    p.grant_count = x[220];
    p.grant_ids.copy_from_slice(&x[221..223]);
    p.grant_masks.copy_from_slice(&x[223..225]);
    // Harness initialization/projection seeds reuse the standalone seed code,
    // never its adaptive effect mocks. Remove its observation, then serialize
    // the joint observation once below.
    if matches!(x[0], 0..=3 | 18) {
        let start = p.inner.bytes.len();
        run_op(r, &mut p.inner, Op(x[..80].try_into().unwrap()));
        p.inner.bytes.truncate(start);
        if x[0] == 0 {
            p.trackers.fill((0, false));
            r.pending = Some(Command {
                kind: CommandKind::Stop,
                generation: 200,
                talk_channel: 3,
            });
            r.status_pending = true;
            r.audio_state.boundary = AudioBoundary {
                enabled: true,
                epoch: 999,
                receipt_cutoff_us: 777,
            };
            if p.full_audio {
                r.audio_state = crate::protocol_audio::AudioState::default();
            }
        }
        snapshot_joint(r, p, false);
        return false;
    }
    let mut combined = CombinedMembershipPlatform(p);
    let p = &mut combined;
    let id = wire::Identity::decode(&x[20..27]).unwrap();
    let mut result = false;
    match x[0] {
        4 => {
            r.membership_handle(Input::BindWork, p).unwrap();
        }
        5 => {
            r.membership_handle(Input::ResetSession, p).unwrap();
        }
        6 => {
            r.membership_handle(
                Input::UpdateLastSeen {
                    node: x[1],
                    rssi: x[2] as i8,
                },
                p,
            )
            .unwrap();
        }
        7 => {
            r.membership_handle(Input::NotePresence(x[1]), p).unwrap();
        }
        8 => {
            r.membership_handle(Input::Discover(x[1]), p).unwrap();
        }
        9 | 10 => {
            let id = IdentityValue {
                address_len: id.address_len,
                address: id.address,
            };
            let input = if x[0] == 9 {
                Input::Demote {
                    leader: x[1],
                    term: g32(&x[16..]),
                    identity: id,
                }
            } else {
                Input::Rejoin {
                    leader: x[1],
                    term: g32(&x[16..]),
                    identity: id,
                }
            };
            r.membership_handle(input, p).unwrap();
        }
        11 => {
            let h = header(&x);
            result = r
                .membership_handle(
                    Input::Packet {
                        header: h,
                        payload: &x[36..36 + usize::from(h.payload_len)],
                    },
                    p,
                )
                .unwrap();
        }
        12 => {
            r.membership_handle(Input::ScanWork, p).unwrap();
        }
        13 => {
            r.membership_handle(Input::JoinWork, p).unwrap();
        }
        14 => {
            r.membership_handle(Input::CheckPeerTimeouts, p).unwrap();
        }
        15 => result = r.membership_handle(Input::CoordinatorTimeout, p).unwrap(),
        16 => {
            if let Some(token) = r.timers[usize::from(x[1] % 2)] {
                r.membership_handle(Input::Timer(token), p).unwrap();
            }
        }
        17 => r.mesh_log(format_args!("fixture {}/{}", 255u8, -7i8), p),
        20 => r.adaptive_reset(),
        21 => r.adaptive_members_owned(p),
        22 => {
            let id = IdentityValue {
                address_len: id.address_len,
                address: id.address,
            };
            result = r.adaptive_bind_member(x[1], (x[2] != 0).then_some(&id), p);
        }
        23 => result = r.adaptive_control_sequence_accept(x[1], x[2], x[3]),
        24 => r.adaptive_local_voice(x[1] != 0, p),
        25 => r.adaptive_frame_tick(p.0.frame, p),
        26 => r.adaptive_report_tick(p),
        27 => {
            let h = header(&x);
            result = r
                .adaptive_control_rx(h, &x[36..115], x[2] as i8, g64(&x[232..]), p)
                .unwrap();
        }
        28 => result = r.adaptive_publish_membership(p) == 0,
        29 => result = r.adaptive_converging(p.0.inner.now as u32),
        30 => {
            let w = wire::Topology::decode(&x[36..36 + wire::Topology::SIZE]).unwrap();
            let report = a::Topology {
                report_seq: w.report_seq,
                origin: IdentityValue {
                    address_len: w.origin.address_len,
                    address: w.origin.address,
                },
                direct_mask: w.direct_mask,
                rssi_dbm: w.rssi_dbm,
                quality: w.quality,
                leader_id: w.leader_id,
                term: w.term,
                slot_map: c::SlotMap {
                    slot_count: w.slot_map.slot_count,
                    slot_ids: w.slot_map.slot_ids,
                    active_speaker_count: w.slot_map.active_speaker_count,
                    active_speaker_ids: w.slot_map.active_speaker_ids,
                    relay_masks: w.slot_map.relay_masks,
                },
            };
            result = a::report(
                Some(&mut r.protocol.adaptive),
                x[1],
                Some(&report),
                false,
                p.0.inner.now as u32,
            );
        }
        31 => {
            r.session.active_speaker_ids.copy_from_slice(&x[1..3]);
            r.session.relay_masks.copy_from_slice(&x[3..5]);
            r.protocol.last_local_active_ms = g32(&x[16..]);
            r.session.speaker_deadlines_ms[usize::from(x[5] % 9)] = g64(&x[41..]);
        }
        32 => {
            if p.0.full_audio {
                r.audio_state.rf_seq[usize::from(x[1])] = crate::protocol_audio::RfSeq16 {
                    last: 65530,
                    initialized: true,
                };
            } else {
                p.0.trackers[usize::from(x[1])] = (65530, true);
            }
        }
        33 => {
            if p.0.full_tx {
                r.tx_control_handler(p.0.frame, p);
                snapshot_joint(r, p.0, false);
                return false;
            }
            let mut i = r.session.control_tail;
            while i != r.session.control_head {
                if r.session.control_ring[usize::from(i)].data[1] == 14 {
                    let selected = r.session.control_ring[usize::from(i)];
                    let mut pos = i;
                    while pos != r.session.control_tail {
                        let previous = (pos + 31) % 32;
                        r.session.control_ring[usize::from(pos)] =
                            r.session.control_ring[usize::from(previous)];
                        pos = previous;
                    }
                    r.session.control_ring[usize::from(r.session.control_tail)] = selected;
                    let entry = &mut r.session.control_ring[usize::from(r.session.control_tail)];
                    let report_seq = r.protocol.air_report_seq.wrapping_add(1);
                    entry.data[9..13].copy_from_slice(&report_seq.to_le_bytes());
                    p.0.inner.e(49);
                    p.0.inner.bytes.push((x[1] != 0) as u8);
                    p32(&mut p.0.inner.bytes, report_seq);
                    if x[1] != 0 {
                        r.protocol.air_report_seq = report_seq;
                        r.protocol.stat_topology_tx_ok =
                            r.protocol.stat_topology_tx_ok.wrapping_add(1);
                        r.session.control_tail = (r.session.control_tail + 1) % 32;
                    } else {
                        r.protocol.stat_tx_fail = r.protocol.stat_tx_fail.wrapping_add(1);
                        entry.data[3] = r.protocol.tx_seq;
                        r.protocol.tx_seq = r.protocol.tx_seq.wrapping_add(1);
                    }
                    break;
                }
                i = (i + 1) % 32;
            }
        }
        34 => {
            // Context scalars only. Never normalize the existing shared state.
            r.protocol.term = g32(&x[16..]);
            r.membership.coordinator_id = x[1];
            r.protocol.leader_identity = IdentityValue {
                address_len: id.address_len,
                address: id.address,
            };
            r.protocol.local_request_seq = g32(&x[36..]);
        }
        35 => {
            // Defined shared API initialization, not private snapshot restoration.
            a::init(
                Some(&mut r.protocol.adaptive),
                x[1],
                x[2],
                g32(&x[16..]),
                x[3],
                p.0.inner.now as u32,
            );
            for node in 1..=3 {
                let wire = remote(node);
                let identity = IdentityValue {
                    address_len: wire.address_len,
                    address: wire.address,
                };
                assert!(a::set_member(
                    Some(&mut r.protocol.adaptive),
                    node,
                    Some(&identity)
                ));
            }
        }
        _ => panic!("unknown joint operation"),
    }
    snapshot_joint(r, p.0, result);
    result
}

#[path = "full_tx_tests.rs"]
mod full_tx_tests;
fn snapshot_joint(r: &mut Runtime, p: &mut Joint, result: bool) {
    p.inner.bytes.extend([b'R', result as u8]);
    snapshot(&mut p.inner.bytes, r);
    wrapper_snapshot(&mut p.inner.bytes, r, &p.trackers);
    if p.full_audio {
        p.inner.bytes.truncate(p.inner.bytes.len() - 256 * 3);
    }
    let s = r.membership_snapshot();
    r.apply_membership_snapshot(s);
}

fn setup(ops: &mut Vec<JointOp>, local_id: u8, now: i64) {
    ops.push(JointOp::new(0, now));
    let role = if local_id == 1 { 1 } else { 2 };
    ops.push(JointOp::old(
        seed(3, role, 2, now)
            .arg(3, local_id)
            .arg(4, local_id - 1)
            .addr([local_id, 2, 3, 4, 5]),
    ));
    ops.push(JointOp::new(21, now));
    for id in 1..=3 {
        ops.push(
            JointOp::new(22, now)
                .arg(1, id)
                .arg(2, 1)
                .identity(remote(id)),
        );
        ops.push(JointOp::old(peer(id - 1, id, true, now)));
    }
}
fn slot_map() -> wire::SlotMap {
    wire::SlotMap {
        slot_count: 3,
        slot_ids: [1, 2, 3, 0, 0, 0, 0, 0],
        active_speaker_count: 0,
        active_speaker_ids: [0; 2],
        relay_masks: [0; 2],
    }
}
fn topo(id: u8, seq: u32) -> wire::Topology {
    let mut rssi = [0; 8];
    let mut quality = [0; 8];
    for peer in 1..=3 {
        if peer != id {
            rssi[usize::from(peer - 1)] = if (id == 1 && peer == 3) || (id == 3 && peer == 1) {
                -100
            } else {
                -60
            };
            quality[usize::from(peer - 1)] = 100;
        }
    }
    wire::Topology {
        report_seq: seq,
        origin: remote(id),
        direct_mask: 7 & !c::node_bit(id),
        rssi_dbm: rssi,
        quality,
        leader_id: 1,
        term: 7,
        slot_map: slot_map(),
    }
}
fn map(revision: u32) -> wire::MembershipV3 {
    let mut ids = [wire::Identity {
        address_len: 0,
        address: [0; 6],
    }; 8];
    for id in 1..=3 {
        ids[usize::from(id - 1)] = remote(id);
    }
    wire::MembershipV3 {
        term: 7,
        revision,
        leader_id: 1,
        slot_map: slot_map(),
        identities: ids,
    }
}

fn joint_fixtures() -> Vec<JointOp> {
    let mut ops = vec![JointOp::new(0, 0), JointOp::new(4, 0), JointOp::new(17, 0)];
    // Membership scope: discovery/retries, scan, JOIN/ACK/LEAVE, announcement,
    // demote/rejoin, timeout, reset and both private snapshot conversion helpers.
    ops.push(JointOp::old(seed(1, 0, 0, 0)));
    ops.push(JointOp::old(Op::new(8, 0).arg(1, 1)));
    for i in 0..=10 {
        ops.push(JointOp::old(Op::new(16, i * 500).arg(1, 1)));
    }
    ops.push(JointOp::new(12, 5500));
    ops.push(JointOp::old(join(remote(2), 2, 1, 5501)));
    ops.push(JointOp::old(join(remote(2), 2, 2, 5502)));
    ops.push(JointOp::old(peer(2, 3, false, 5502)));
    ops.push(JointOp::old(
        Op::new(6, 5503).arg(1, 3).arg(2, (-60i8) as u8),
    ));
    let leave = wire::LeaveNrf {
        sender_addr: [2, 2, 3, 4, 5],
    };
    let mut payload = [0; 5];
    leave.encode(&mut payload).unwrap();
    ops.push(JointOp::old(Op::packet(4, 2, 3, &payload, 5504)));
    ops.push(JointOp::new(14, 9000));
    ops.push(JointOp::old(
        Op::new(9, 9001).arg(1, 1).term(7).identity(local()),
    ));
    ops.push(JointOp::old(ack(local(), 2, 1, 1, 7, 9002)));
    ops.push(JointOp::new(15, 15003));
    ops.push(JointOp::new(5, 15004));
    // Cold term-zero discovery is accepted by the wrapper (not stamp_sync).
    ops.push(JointOp::new(0, 0));
    ops.push(JointOp::old(seed(1, 0, 0, 0).addr([9, 2, 3, 4, 5])));
    let sync = wire::SyncV3 {
        leader: local(),
        leader_id: 1,
        member_count: 2,
        term: 0,
        frame_counter: 1,
        phase_us: 100,
        relay_depth: 0,
    };
    ops.push(JointOp::packet(Message::SyncV3(sync), 1, 1, 1));
    ops.push(JointOp::packet(Message::SyncV3(sync), 1, 2, 2));
    ops.push(JointOp::new(0, 40));
    ops.push(JointOp::old(seed(2, 0, 0, 40).addr([2, 2, 3, 4, 5])));
    ops.push(JointOp::packet(
        Message::JoinAckV3(wire::JoinAckV3 {
            target: remote(2),
            capabilities: 2,
            assigned_id: 2,
            slot_index: 1,
            coordinator_id: 1,
            term: 7,
        }),
        1,
        1,
        41,
    ));
    setup(&mut ops, 1, 50);
    ops.push(JointOp::packet(
        Message::JoinV3(wire::JoinV3 {
            origin: remote(4),
            capabilities: 2,
            target: local(),
        }),
        0,
        1,
        51,
    ));
    ops.push(JointOp::new(0, 60));
    ops.push(JointOp::old(seed(1, 0, 0, 60).addr([9, 2, 3, 4, 5])));
    ops.push(JointOp::new(12, 60));
    ops.push(JointOp::packet(
        Message::SyncV3(wire::SyncV3 { term: 1, ..sync }),
        1,
        1,
        61,
    ));
    setup(&mut ops, 2, 100);
    // Every legal adaptive type, wrong size, invalid flags/TTL, recognized
    // invalid fields and direct/relayed age/depth limits.
    let sync = wire::SyncV3 { term: 7, ..sync };
    let proposal = wire::HandoverProposal {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 260,
        members: 7,
        slot_map: slot_map(),
    };
    let ack = wire::HandoverAck {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 260,
        acknowledger_id: 3,
    };
    let request = wire::SpeakerRequest {
        term: 7,
        request_seq: 1,
        frame_counter: 0,
        active: 1,
    };
    let join = wire::JoinV3 {
        origin: remote(4),
        capabilities: 2,
        target: local(),
    };
    let join_ack = wire::JoinAckV3 {
        target: remote(4),
        capabilities: 2,
        assigned_id: 4,
        slot_index: 3,
        coordinator_id: 1,
        term: 7,
    };
    let controls = [
        Message::SyncV3(sync),
        Message::JoinV3(join),
        Message::JoinAckV3(join_ack),
        Message::Topology(topo(3, 1)),
        Message::MembershipV3(map(1)),
        Message::SpeakerRequest(request),
        Message::SlotMap(slot_map()),
        Message::SpeakerGrant(wire::SpeakerGrant {
            speaker_count: 2,
            speaker_ids: [1, 3],
            relay_masks: [6, 3],
        }),
        Message::SpeakerRelease(wire::SpeakerRelease {
            speaker_count: 1,
            speaker_ids: [3, 0],
        }),
        Message::HandoverPrepare(proposal),
        Message::HandoverAck(ack),
        Message::HandoverCommit(proposal),
        Message::HandoverCancel(proposal),
    ];
    for (i, msg) in controls.into_iter().enumerate() {
        let source = match msg {
            Message::JoinV3(_) => 0,
            Message::Topology(_) | Message::SpeakerRequest(_) | Message::HandoverAck(_) => 3,
            _ => 1,
        };
        let op = JointOp::packet(msg, source, i as u8, 101 + i as i64);
        ops.extend([op.arg(34, 0), op.arg(32, 4), op.arg(31, 0), op]);
    }
    for count in [0, 1, 8, 9] {
        ops.push(JointOp::packet(
            Message::SyncV3(wire::SyncV3 {
                member_count: count,
                ..sync
            }),
            1,
            count,
            200,
        ));
    }
    for phase in [19999, 20000, 65535] {
        ops.push(JointOp::packet(
            Message::SyncV3(wire::SyncV3 {
                phase_us: phase,
                ..sync
            }),
            1,
            8,
            200,
        ));
    }
    for depth in [0, 1, 2] {
        ops.push(
            JointOp::packet(
                Message::SyncV3(wire::SyncV3 {
                    relay_depth: depth,
                    ..sync
                }),
                1,
                depth,
                201,
            )
            .arg(31, 1)
            .arg(32, 2),
        );
    }
    for age in [600i64, 601] {
        let mut op = JointOp::packet(Message::SyncV3(sync), 1, 9, 1000);
        op.0[232..240].copy_from_slice(&((1000 - age) * 1000).to_le_bytes());
        ops.push(op);
    }
    // Binding reuse clears sequence/dedupe/link estimates and RF tracker; stale
    // direct_seq/rssi and nonchanged fields survive reset exactly as C writes.
    ops.push(JointOp::new(32, 1001).arg(1, 3));
    for seq in [255, 0, 0, 128, 1] {
        ops.push(JointOp::new(23, 1001).arg(1, 3).arg(2, 15).arg(3, seq));
    }
    ops.push(
        JointOp::new(22, 1002)
            .arg(1, 3)
            .arg(2, 1)
            .identity(remote(5)),
    );
    ops.push(
        JointOp::new(22, 1003)
            .arg(1, 3)
            .arg(2, 1)
            .identity(remote(3)),
    );
    // Quality/report sequence acceptance precedes presence dedupe; retransmits
    // with a fresh topology sequence still update reports and RF metrics.
    for seq in [2, 3, 3, 4] {
        ops.push(JointOp::packet(
            Message::Topology(topo(3, seq)),
            3,
            40,
            1004,
        ));
    }
    ops.push(JointOp::new(24, 1005).arg(1, 1).tx_failure(false, -105));
    ops.push(JointOp::new(24, 1006).arg(1, 1));
    ops.push(JointOp::new(25, 1127));
    // Authoritative local removal calls the actual owned rejoin/reset wrapper.
    let mut removal = map(2);
    removal.slot_map.slot_ids[1] = 0;
    removal.identities[1] = wire::Identity {
        address_len: 0,
        address: [0; 6],
    };
    ops.push(JointOp::packet(Message::MembershipV3(removal), 1, 41, 1130));
    // Pruning grant IDs/masks never clears surviving activity deadlines.
    setup(&mut ops, 1, 2000);
    let mut grant_seed = JointOp::new(31, 2000)
        .arg(1, 4)
        .arg(2, 2)
        .arg(3, 255)
        .arg(4, 255)
        .arg(5, 2);
    grant_seed.0[41..49].copy_from_slice(&99999i64.to_le_bytes());
    ops.push(grant_seed);
    ops.push(JointOp::new(28, 2001));
    ops.push(JointOp::new(28, 2002));
    ops.push(JointOp::new(26, 2003));
    ops.push(JointOp::new(26, 2004));
    ops.push(JointOp::new(33, 2005));
    ops.push(JointOp::new(33, 2006).arg(1, 1));
    setup(&mut ops, 1, 2100);
    ops.push(JointOp::new(3, 2100));
    ops.push(JointOp::new(18, 2100));
    ops.push(JointOp::new(32, 2100).arg(1, 255));
    ops.push(JointOp::new(22, 2101).arg(1, 2));
    ops.push(JointOp::new(5, 2102));
    // Actual handover handlers: prepare -> ACK quorum -> commit -> reconcile;
    // shared graph seeds are real report calls, not restored snapshots.
    for local_id in [1, 2] {
        setup(&mut ops, local_id, 0);
        for (now, seq) in [(10000, 1), (13000, 2)] {
            for id in 1..=3 {
                let mut op = JointOp::packet(Message::Topology(topo(id, seq)), id, seq as u8, now);
                op.0[0] = 30;
                op.0[1] = id;
                ops.push(op);
            }
            if local_id == 1 {
                ops.push(JointOp::new(26, now).frame(100));
            }
        }
        if local_id == 1 {
            for id in [2, 3] {
                ops.push(JointOp::packet(
                    Message::HandoverAck(wire::HandoverAck {
                        acknowledger_id: id,
                        ..ack
                    }),
                    id,
                    2,
                    13001,
                ));
            }
            ops.push(JointOp::new(25, 13002).frame(101));
            let next_sync = wire::SyncV3 {
                leader: remote(2),
                leader_id: 2,
                term: 8,
                frame_counter: 260,
                ..sync
            };
            ops.push(JointOp::packet(Message::SyncV3(next_sync), 2, 3, 13003));
        } else {
            ops.push(JointOp::packet(Message::HandoverPrepare(proposal), 1, 2, 13001).frame(100));
            ops.push(JointOp::packet(
                Message::HandoverCommit(proposal),
                1,
                3,
                13002,
            ));
            ops.push(JointOp::new(25, 13003).frame(260));
            for now in [13003, 15003, 15004] {
                ops.push(JointOp::new(29, now));
            }
        }
    }
    setup(&mut ops, 1, 0);
    for (now, seq) in [(10000, 1), (13000, 2)] {
        for id in 1..=3 {
            let mut op = JointOp::packet(Message::Topology(topo(id, seq)), id, seq as u8, now);
            op.0[0] = 30;
            op.0[1] = id;
            ops.push(op);
        }
        ops.push(JointOp::new(26, now).frame(100));
    }
    ops.push(JointOp::new(25, 13001).frame(101));
    ops.push(JointOp::new(25, 15401).frame(102).tx_failure(true, -105));
    // Prepared timeout and cancellation including fallback queue failure.
    setup(&mut ops, 3, 0);
    for id in 1..=3 {
        let mut op = JointOp::packet(Message::Topology(topo(id, 1)), id, 1, 10000);
        op.0[0] = 30;
        op.0[1] = id;
        ops.push(op);
    }
    ops.push(JointOp::packet(Message::HandoverPrepare(proposal), 1, 1, 10001).frame(100));
    ops.push(JointOp::packet(
        Message::HandoverCancel(proposal),
        1,
        2,
        10002,
    ));
    ops.push(JointOp::new(25, 16000).frame(500));
    // Unsigned ms wrap with intervals below 2^31, no arbitrary uptime clamp.
    setup(&mut ops, 2, i64::from(u32::MAX) - 50);
    ops.push(
        JointOp::new(24, i64::from(u32::MAX) - 49)
            .arg(1, 1)
            .frame(u32::MAX),
    );
    ops.push(JointOp::new(25, i64::from(u32::MAX) + 100).frame(0));
    ops.push(JointOp::new(20, i64::from(u32::MAX) + 101));
    review_fixtures(&mut ops);
    ops
}

fn sync_payload() -> wire::SyncV3 {
    wire::SyncV3 {
        leader: local(),
        leader_id: 1,
        member_count: 3,
        term: 7,
        frame_counter: 10,
        phase_us: 100,
        relay_depth: 0,
    }
}

fn rejected(ops: &mut Vec<JointOp>, msg: Message<'_>, source: u8, frame_query: bool) {
    // Each payload has its exact original packed length and otherwise-correct
    // flags/TTL/age. Isolate semantic rejection from dedupe and prior mutations.
    setup(ops, 2, 3000);
    ops.push(
        JointOp::packet(msg, source, 1, 3010)
            .frame(51)
            .expect(ReviewCheck::Unchanged { frame_query }),
    );
}

fn review_fixtures(ops: &mut Vec<JointOp>) {
    // The scripted selection reply is an audio ALGORITHM MOCK, but callback
    // mutation is synchronous on the real session and preserves selected rank.
    for count in 0..=2 {
        setup(ops, 1, 2500);
        ops.push(
            JointOp::new(31, 2500)
                .arg(1, 2)
                .arg(2, 3)
                .arg(3, 5)
                .arg(4, 3),
        );
        let ids = [3, 2];
        let masks = [3, 5];
        let request = wire::SpeakerRequest {
            term: 7,
            request_seq: 1,
            frame_counter: 10,
            active: 1,
        };
        ops.push(
            JointOp::packet(Message::SpeakerRequest(request), 3, 1, 2510)
                .frame(10)
                .grant_reply(count, ids, masks)
                .expect(ReviewCheck::GrantReply { count, ids, masks }),
        );
        let retained_ids = match count {
            0 => [0, 0],
            1 => [3, 0],
            _ => ids,
        };
        let retained_masks = match count {
            0 => [0, 0],
            1 => [3, 0],
            _ => masks,
        };
        ops.push(JointOp::new(28, 2511).expect(ReviewCheck::Published {
            ids: retained_ids,
            masks: retained_masks,
            pruned: false,
        }));
    }
    setup(ops, 1, 2600);
    ops.push(
        JointOp::packet(
            Message::SpeakerRequest(wire::SpeakerRequest {
                term: 7,
                request_seq: 1,
                frame_counter: 10,
                active: 1,
            }),
            3,
            1,
            2610,
        )
        .frame(10)
        .grant_reply(2, [4, 2], [255, 255])
        .expect(ReviewCheck::GrantReply {
            count: 2,
            ids: [4, 2],
            masks: [255, 255],
        }),
    );
    ops.push(JointOp::new(28, 2611).expect(ReviewCheck::Published {
        ids: [0, 2],
        masks: [0, 5],
        pruned: true,
    }));

    let bad_identities = [
        wire::Identity {
            address_len: 0,
            address: [0; 6],
        },
        wire::Identity {
            address_len: 7,
            address: remote(3).address,
        },
        wire::Identity {
            address_len: 255,
            address: remote(3).address,
        },
        wire::Identity {
            address_len: 5,
            address: [0; 6],
        },
        wire::Identity {
            address_len: 5,
            address: [255, 255, 255, 255, 255, 0],
        },
        wire::Identity {
            address_len: 5,
            address: [3, 2, 3, 4, 5, 1],
        },
    ];
    for id in bad_identities {
        rejected(
            ops,
            Message::SyncV3(wire::SyncV3 {
                leader: id,
                ..sync_payload()
            }),
            1,
            false,
        );
        rejected(
            ops,
            Message::Topology(wire::Topology {
                origin: id,
                ..topo(3, 1)
            }),
            3,
            false,
        );
        rejected(
            ops,
            Message::JoinV3(wire::JoinV3 {
                origin: id,
                capabilities: 2,
                target: local(),
            }),
            0,
            false,
        );
        rejected(
            ops,
            Message::JoinV3(wire::JoinV3 {
                origin: remote(4),
                capabilities: 2,
                target: id,
            }),
            0,
            false,
        );
        rejected(
            ops,
            Message::JoinAckV3(wire::JoinAckV3 {
                target: id,
                capabilities: 2,
                assigned_id: 4,
                slot_index: 3,
                coordinator_id: 1,
                term: 7,
            }),
            1,
            false,
        );
        let mut snapshot = map(1);
        snapshot.identities[2] = id;
        rejected(ops, Message::MembershipV3(snapshot), 1, false);
    }
    let mut duplicate_identity = map(1);
    duplicate_identity.identities[2] = local();
    rejected(ops, Message::MembershipV3(duplicate_identity), 1, false);
    let mut absent_identity = map(1);
    absent_identity.identities[3] = remote(4);
    rejected(ops, Message::MembershipV3(absent_identity), 1, false);

    let mut invalid_slots = Vec::new();
    let mut slots = slot_map();
    slots.slot_ids[2] = 2;
    invalid_slots.push(slots);
    for id in [9, 255] {
        let mut slots = slot_map();
        slots.slot_ids[2] = id;
        invalid_slots.push(slots);
    }
    let mut slots = slot_map();
    slots.slot_count = 0;
    invalid_slots.push(slots);
    let mut slots = slot_map();
    slots.slot_count = 4;
    invalid_slots.push(slots); // Empty last counted slot.
    let mut slots = slot_map();
    slots.slot_count = 2;
    invalid_slots.push(slots); // Nonzero trailing slot.
    let mut slots = slot_map();
    slots.active_speaker_count = 3;
    invalid_slots.push(slots);
    for id in [0, 4, 9, 255] {
        let mut slots = slot_map();
        slots.active_speaker_count = 1;
        slots.active_speaker_ids[0] = id;
        invalid_slots.push(slots);
    }
    let mut slots = slot_map();
    slots.active_speaker_count = 2;
    slots.active_speaker_ids = [3, 3];
    invalid_slots.push(slots);
    let mut slots = slot_map();
    slots.active_speaker_count = 1;
    slots.active_speaker_ids[0] = 3;
    slots.relay_masks[0] = 8;
    invalid_slots.push(slots);
    let mut slots = slot_map();
    slots.active_speaker_ids[1] = 2;
    invalid_slots.push(slots);
    let mut slots = slot_map();
    slots.relay_masks[1] = 1;
    invalid_slots.push(slots);
    for slots in invalid_slots {
        let mut snapshot = map(1);
        snapshot.slot_map = slots;
        rejected(ops, Message::MembershipV3(snapshot), 1, false);
        rejected(ops, Message::SlotMap(slots), 1, false);
        if slots.slot_count == 0 && slots.active_speaker_count == 0 {
            // Original report validation ignores the remaining slot bytes when
            // slot_count is zero; map validation still rejects the same bytes.
            setup(ops, 2, 3000);
            ops.push(
                JointOp::packet(
                    Message::Topology(wire::Topology {
                        slot_map: slots,
                        ..topo(3, 1)
                    }),
                    3,
                    1,
                    3010,
                )
                .expect(ReviewCheck::DirectSequence(1)),
            );
        } else {
            rejected(
                ops,
                Message::Topology(wire::Topology {
                    slot_map: slots,
                    ..topo(3, 1)
                }),
                3,
                false,
            );
        }
    }
    // Raw speaker grants are deliberately NOT slot-map validators: original
    // audio application clamps count, preserves duplicates/IDs/masks verbatim.
    for grant in [
        wire::SpeakerGrant {
            speaker_count: 3,
            speaker_ids: [3, 2],
            relay_masks: [3, 5],
        },
        wire::SpeakerGrant {
            speaker_count: 2,
            speaker_ids: [9, 2],
            relay_masks: [255, 5],
        },
        wire::SpeakerGrant {
            speaker_count: 2,
            speaker_ids: [3, 3],
            relay_masks: [3, 3],
        },
        wire::SpeakerGrant {
            speaker_count: 1,
            speaker_ids: [3, 2],
            relay_masks: [8, 255],
        },
    ] {
        setup(ops, 2, 3000);
        let ids = if grant.speaker_count == 1 {
            [grant.speaker_ids[0], 0]
        } else {
            grant.speaker_ids
        };
        let masks = if grant.speaker_count == 1 {
            [grant.relay_masks[0], 0]
        } else {
            grant.relay_masks
        };
        ops.push(
            JointOp::packet(Message::SpeakerGrant(grant), 1, 1, 3010)
                .expect(ReviewCheck::AppliedGrant { ids, masks }),
        );
        // Reset before a tick: id 9 must not reach the excluded C audio array UB.
    }
    for source in [0, 2, 3, 8, 9] {
        rejected(ops, Message::MembershipV3(map(1)), source, false);
        rejected(
            ops,
            Message::SpeakerGrant(wire::SpeakerGrant {
                speaker_count: 1,
                speaker_ids: [3, 0],
                relay_masks: [3, 0],
            }),
            source,
            false,
        );
    }
    rejected(ops, Message::Topology(topo(3, 1)), 8, false);
    rejected(
        ops,
        Message::Topology(wire::Topology {
            term: 6,
            ..topo(3, 1)
        }),
        3,
        false,
    );
    rejected(
        ops,
        Message::Topology(wire::Topology {
            leader_id: 3,
            ..topo(3, 1)
        }),
        3,
        false,
    );
    rejected(
        ops,
        Message::SyncV3(wire::SyncV3 {
            term: 6,
            ..sync_payload()
        }),
        1,
        false,
    );
    rejected(
        ops,
        Message::SyncV3(wire::SyncV3 {
            leader: remote(3),
            leader_id: 3,
            ..sync_payload()
        }),
        3,
        false,
    );
    let mut wrong_term = map(1);
    wrong_term.term = 6;
    rejected(ops, Message::MembershipV3(wrong_term), 1, false);
    let mut wrong_leader = map(1);
    wrong_leader.leader_id = 3;
    rejected(ops, Message::MembershipV3(wrong_leader), 1, false);
    for request in [
        wire::SpeakerRequest {
            term: 6,
            request_seq: 1,
            frame_counter: 51,
            active: 1,
        },
        wire::SpeakerRequest {
            term: 7,
            request_seq: 1,
            frame_counter: 0,
            active: 1,
        },
        wire::SpeakerRequest {
            term: 7,
            request_seq: 1,
            frame_counter: 51,
            active: 2,
        },
    ] {
        rejected(ops, Message::SpeakerRequest(request), 3, request.term == 7);
    }
    rejected(
        ops,
        Message::JoinAckV3(wire::JoinAckV3 {
            target: remote(4),
            capabilities: 2,
            assigned_id: 4,
            slot_index: 3,
            coordinator_id: 1,
            term: 6,
        }),
        1,
        false,
    );
    let proposal = wire::HandoverProposal {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 7,
        switch_frame: 100,
        members: 7,
        slot_map: slot_map(),
    };
    rejected(ops, Message::HandoverPrepare(proposal), 1, true);
    rejected(
        ops,
        Message::HandoverPrepare(wire::HandoverProposal {
            old_leader_id: 3,
            next_term: 8,
            ..proposal
        }),
        1,
        true,
    );

    setup(ops, 2, 4000);
    for (seq, now) in [(u32::MAX, 4010), (0, 4020)] {
        ops.push(
            JointOp::packet(Message::Topology(topo(3, seq)), 3, seq as u8, now)
                .expect(ReviewCheck::DirectSequence(seq)),
        );
    }
    ops.push(
        JointOp::new(34, 4030)
            .arg(1, 1)
            .identity(local())
            .scalar(16, 7)
            .scalar(36, u32::MAX),
    );
    ops.push(
        JointOp::new(24, 4040)
            .arg(1, 1)
            .frame(10)
            .expect(ReviewCheck::LocalSequence(0)),
    );
    for (revision, now) in [(u32::MAX, 4050), (0, 4060)] {
        ops.push(
            JointOp::packet(Message::MembershipV3(map(revision)), 1, revision as u8, now)
                .expect(ReviewCheck::MembershipRevision(revision)),
        );
    }
    // Accepted wire revision wrap does not normalize the separate outbound
    // publish_revision. Follow it with an actual adapter publication.
    setup(ops, 1, 4100);
    for (revision, now) in [(u32::MAX, 4110), (0, 4120)] {
        ops.push(
            JointOp::packet(Message::MembershipV3(map(revision)), 1, revision as u8, now)
                .expect(ReviewCheck::MembershipRevision(revision)),
        );
    }
    ops.push(JointOp::new(28, 4130).expect(ReviewCheck::Published {
        ids: [0; 2],
        masks: [0; 2],
        pruned: false,
    }));
    // Context authority and actual shared authority intentionally disagree.
    // Only init/set_member define shared state; no private Snapshot is forged.
    for (msg, source, observation) in [
        (
            Message::SyncV3(wire::SyncV3 {
                leader: remote(3),
                leader_id: 3,
                term: 9,
                ..sync_payload()
            }),
            3,
            AuthorityObservation::Unchanged,
        ),
        (
            Message::SyncV3(wire::SyncV3 {
                leader: remote(3),
                leader_id: 3,
                ..sync_payload()
            }),
            3,
            AuthorityObservation::Sync,
        ),
        (
            Message::SyncV3(sync_payload()),
            1,
            AuthorityObservation::Unchanged,
        ),
        (
            Message::Topology(wire::Topology {
                term: 9,
                leader_id: 3,
                ..topo(1, 1)
            }),
            1,
            AuthorityObservation::Report,
        ),
        (
            Message::SpeakerRequest(wire::SpeakerRequest {
                term: 9,
                request_seq: 1,
                frame_counter: 10,
                active: 1,
            }),
            1,
            AuthorityObservation::Unchanged,
        ),
        (
            Message::SpeakerRequest(wire::SpeakerRequest {
                term: 7,
                request_seq: 1,
                frame_counter: 10,
                active: 1,
            }),
            1,
            AuthorityObservation::Unchanged,
        ),
        (
            Message::SlotMap(slot_map()),
            3,
            AuthorityObservation::SlotMap,
        ),
    ] {
        setup(ops, 2, 5000);
        if matches!(observation, AuthorityObservation::SlotMap) {
            ops.push(
                JointOp::new(31, 5000)
                    .arg(1, 3)
                    .arg(2, 2)
                    .arg(3, 3)
                    .arg(4, 5),
            );
        }
        ops.push(
            JointOp::new(35, 5001)
                .arg(1, 2)
                .arg(2, 3)
                .arg(3, 7)
                .scalar(16, 9),
        );
        ops.push(
            JointOp::packet(msg, source, 1, 5010)
                .frame(10)
                .expect(ReviewCheck::Authority {
                    context_term: 7,
                    context_leader: 1,
                    shared_term: 9,
                    shared_leader: 3,
                    handled: true,
                    observation,
                }),
        );
    }
    for term in [7, 9] {
        setup(ops, 2, 5000);
        ops.push(
            JointOp::new(35, 5001)
                .arg(1, 2)
                .arg(2, 3)
                .arg(3, 7)
                .scalar(16, 9),
        );
        let mut snapshot = map(1);
        snapshot.term = term;
        snapshot.leader_id = 3;
        ops.push(
            JointOp::packet(Message::MembershipV3(snapshot), 3, 1, 5010).expect(
                ReviewCheck::Authority {
                    context_term: 7,
                    context_leader: 1,
                    shared_term: 9,
                    shared_leader: 3,
                    handled: true,
                    observation: AuthorityObservation::Unchanged,
                },
            ),
        );
    }
    setup(ops, 2, 5100);
    ops.push(
        JointOp::new(35, 5101)
            .arg(1, 2)
            .arg(2, 3)
            .arg(3, 7)
            .scalar(16, 9),
    );
    ops.push(
        JointOp::new(24, 5110)
            .arg(1, 1)
            .frame(10)
            .expect(ReviewCheck::Authority {
                context_term: 7,
                context_leader: 1,
                shared_term: 9,
                shared_leader: 3,
                handled: false,
                observation: AuthorityObservation::LocalVoice,
            }),
    );
    setup(ops, 1, 5200);
    ops.push(
        JointOp::new(35, 5201)
            .arg(1, 1)
            .arg(2, 3)
            .arg(3, 7)
            .scalar(16, 9),
    );
    ops.push(JointOp::new(28, 5210).expect(ReviewCheck::Authority {
        context_term: 7,
        context_leader: 1,
        shared_term: 9,
        shared_leader: 3,
        handled: true,
        observation: AuthorityObservation::Publication,
    }));
}

struct ReviewBefore {
    owned: Vec<u8>,
    effects_start: usize,
    deadlines: [i64; 9],
    active_since: [i64; 9],
    topology_direct: u32,
    publish_revision: u32,
}
impl ReviewBefore {
    fn capture(r: &Runtime, p: &Joint) -> Self {
        Self {
            owned: owned_state(r, p),
            effects_start: p.effects.len(),
            deadlines: r.session.speaker_deadlines_ms,
            active_since: r.session.speaker_active_since_ms,
            topology_direct: r.protocol.stat_topology_rx_direct,
            publish_revision: r.protocol.adaptive.snapshot().publish_revision,
        }
    }
}
fn owned_state(r: &Runtime, p: &Joint) -> Vec<u8> {
    let mut bytes = Vec::new();
    snapshot(&mut bytes, r);
    wrapper_snapshot(&mut bytes, r, &p.trackers);
    bytes
}
fn queued_payload(r: &Runtime, packet_type: u8) -> &[u8] {
    for offset in 0..32 {
        let i = (usize::from(r.session.control_tail) + offset) % 32;
        if i == usize::from(r.session.control_head) {
            break;
        }
        let entry = &r.session.control_ring[i];
        if entry.data[1] == packet_type {
            return &entry.data[9..usize::from(entry.len)];
        }
    }
    panic!("expected queued type {packet_type}");
}
fn publication_effect_types(effects: &[A]) -> Vec<u8> {
    effects
        .iter()
        .map(|e| match e {
            A::SlotSnapshot => 41,
            A::Queue {
                packet_type,
                priority: false,
                ..
            } => *packet_type,
            _ => panic!("unexpected publication boundary {e:?}"),
        })
        .collect()
}
fn check_review(op: JointOp, handled: bool, before: &ReviewBefore, r: &Runtime, p: &Joint) {
    let effects = &p.effects[before.effects_start..];
    match op.1 {
        ReviewCheck::None => {}
        ReviewCheck::Unchanged { frame_query } => {
            assert!(
                handled,
                "recognized malformed type {} must return true",
                op.0[28]
            );
            assert_eq!(
                owned_state(r, p),
                before.owned,
                "rejected payload mutated owned state"
            );
            let expected = if frame_query {
                &[A::FrameCounter][..]
            } else {
                &[][..]
            };
            assert_eq!(effects, expected, "rejection changed boundary order");
        }
        ReviewCheck::GrantReply { count, ids, masks } => {
            assert!(handled);
            assert_eq!(effects, &[A::FrameCounter, A::UpdateGrants]);
            let mut expected_ids = [0; 2];
            let mut expected_masks = [0; 2];
            let n = usize::from(count);
            expected_ids[..n].copy_from_slice(&ids[..n]);
            expected_masks[..n].copy_from_slice(&masks[..n]);
            assert_eq!(r.session.active_speaker_ids, expected_ids);
            assert_eq!(r.session.relay_masks, expected_masks);
            let mut deadlines = before.deadlines;
            let mut since = before.active_since;
            deadlines[3] = p.inner.now + 1500;
            since[3] = p.inner.now;
            assert_eq!(r.session.speaker_deadlines_ms, deadlines);
            assert_eq!(r.session.speaker_active_since_ms, since);
            let shared = r.protocol.adaptive.snapshot();
            assert!(shared.request_active[2]);
            assert_eq!(shared.request_seq[2], 1);
            assert_eq!(
                shared.request_deadline_ms[2],
                (p.inner.now as u32).wrapping_add(1500)
            );
        }
        ReviewCheck::Published { ids, masks, pruned } => {
            assert!(handled);
            assert_eq!(r.session.active_speaker_ids, ids);
            assert_eq!(r.session.relay_masks, masks);
            assert_eq!(r.session.speaker_deadlines_ms, before.deadlines);
            assert_eq!(r.session.speaker_active_since_ms, before.active_since);
            assert_eq!(
                publication_effect_types(effects),
                if pruned {
                    vec![10, 9, 41, 22]
                } else {
                    vec![41, 22]
                }
            );
            let publication = wire::MembershipV3::decode(queued_payload(r, 22)).unwrap();
            let mut expected = slot_map();
            for i in 0..2 {
                if ids[i] != 0 {
                    let n = usize::from(expected.active_speaker_count);
                    expected.active_speaker_count += 1;
                    expected.active_speaker_ids[n] = ids[i];
                    expected.relay_masks[n] = masks[i];
                }
            }
            assert_eq!(publication.slot_map, expected);
            assert_eq!(publication.term, 7);
            assert_eq!(publication.leader_id, 1);
            assert_eq!(publication.revision, 1);
        }
        ReviewCheck::AppliedGrant { ids, masks } => {
            assert!(handled);
            assert!(matches!(effects, [A::ApplyGrant { .. }]));
            assert_eq!(r.session.active_speaker_ids, ids);
            assert_eq!(r.session.relay_masks, masks);
            assert_eq!(r.session.speaker_deadlines_ms, before.deadlines);
            assert_eq!(r.session.speaker_active_since_ms, before.active_since);
        }
        ReviewCheck::DirectSequence(seq) => {
            assert!(handled);
            assert!(effects.is_empty());
            assert!(r.protocol.direct_seq_valid[2]);
            assert_eq!(r.protocol.direct_seq[2], seq);
            assert_eq!(r.protocol.direct_quality[2], 100);
            assert_eq!(r.protocol.direct_seen[2], p.inner.now as u32);
            assert_eq!(
                r.protocol.stat_topology_rx_direct,
                before.topology_direct.wrapping_add(1)
            );
            let shared = r.protocol.adaptive.snapshot();
            assert!(shared.nodes[2].seq_seen);
            assert_eq!(shared.nodes[2].report_seq, seq);
        }
        ReviewCheck::LocalSequence(seq) => {
            assert!(!handled);
            assert_eq!(r.protocol.local_request_seq, seq);
            assert!(matches!(
                effects,
                [
                    A::FrameCounter,
                    A::Queue {
                        packet_type: 23,
                        priority: false,
                        ..
                    }
                ]
            ));
            let request = wire::SpeakerRequest::decode(queued_payload(r, 23)).unwrap();
            assert_eq!(request.request_seq, seq);
            assert_eq!(request.term, r.protocol.term);
            assert!(r.protocol.local_request_active);
            assert_eq!(r.protocol.last_request_ms, p.inner.now as u32);
        }
        ReviewCheck::MembershipRevision(revision) => {
            assert!(handled);
            let shared = r.protocol.adaptive.snapshot();
            assert!(shared.membership_revision_seen);
            assert_eq!(shared.membership_revision, revision);
            assert_eq!(shared.publish_revision, before.publish_revision);
            assert!(r.membership.membership_known);
        }
        ReviewCheck::Authority {
            context_term,
            context_leader,
            shared_term,
            shared_leader,
            handled: expected,
            observation,
        } => {
            assert_eq!(handled, expected);
            let shared = r.protocol.adaptive.snapshot();
            assert_eq!(r.protocol.term, context_term);
            assert_eq!(r.membership.coordinator_id, context_leader);
            assert_eq!(r.protocol.leader_identity.address[0], context_leader);
            assert_eq!(shared.term, shared_term);
            assert_eq!(shared.leader_id, shared_leader);
            assert_eq!(
                shared.nodes[usize::from(shared_leader - 1)]
                    .identity
                    .address[0],
                shared_leader
            );
            assert_ne!(context_term, shared_term);
            assert_ne!(context_leader, shared_leader);
            match observation {
                AuthorityObservation::Unchanged => {
                    assert_eq!(owned_state(r, p), before.owned);
                    assert!(effects.is_empty() || effects == [A::FrameCounter]);
                }
                AuthorityObservation::Sync => {
                    assert!(matches!(
                        effects,
                        [A::Sync {
                            frame: 10,
                            drift: 0,
                            ..
                        }]
                    ));
                    assert_eq!(r.protocol.last_sync_time, p.inner.now as u32);
                }
                AuthorityObservation::Report => {
                    assert!(effects.is_empty());
                    assert_eq!(shared.nodes[0].term, shared_term);
                    assert_eq!(shared.nodes[0].leader_id, shared_leader);
                    assert_eq!(
                        r.protocol.stat_topology_rx_direct,
                        before.topology_direct.wrapping_add(1)
                    );
                }
                AuthorityObservation::SlotMap => {
                    assert!(matches!(effects, [A::ApplySlotMap(_)]));
                    assert_eq!(r.session.active_speaker_ids, [0; 2]);
                    assert_eq!(r.session.relay_masks, [0; 2]);
                }
                AuthorityObservation::LocalVoice => {
                    assert!(matches!(
                        effects,
                        [
                            A::FrameCounter,
                            A::Queue {
                                packet_type: 23,
                                priority: false,
                                ..
                            }
                        ]
                    ));
                    let request = wire::SpeakerRequest::decode(queued_payload(r, 23)).unwrap();
                    assert_eq!(request.term, context_term);
                    assert_ne!(request.term, shared_term);
                }
                AuthorityObservation::Publication => {
                    assert_eq!(publication_effect_types(effects), vec![41, 22]);
                    let publication = wire::MembershipV3::decode(queued_payload(r, 22)).unwrap();
                    assert_eq!(publication.term, context_term);
                    assert_eq!(publication.leader_id, r.membership.node_id);
                    // TX's shared revision helper sees a nonleader, unlike the context role.
                    assert_eq!(publication.revision, 0);
                    assert_eq!(shared.publish_revision, before.publish_revision);
                }
            }
        }
    }
}

#[test]
fn replacing_member_invalidates_peer_history_and_reset_clears_authority() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    let mut ops = Vec::new();
    setup(&mut ops, 1, 100);
    for op in ops {
        run_joint(&mut r, &mut p, op);
    }
    for id in [2u8, 3] {
        c::dedupe_accept(Some(&mut r.dedupe), 13, id, 9);
        c::dedupe_accept(Some(&mut r.protocol.control_presence_dedupe), 14, id, 9);
        r.protocol.direct_seq_valid[usize::from(id - 1)] = true;
        r.protocol.direct_quality[usize::from(id - 1)] = 77;
        r.protocol.direct_seen[usize::from(id - 1)] = 100;
        r.session.speaker_deadlines_ms[usize::from(id)] = 500;
        r.session.speaker_active_since_ms[usize::from(id)] = 100;
        p.trackers[usize::from(id)] = (99, true);
    }
    r.protocol.join_seq_seen = 6;
    r.protocol.ack_seq_seen = 6;
    let replacement = IdentityValue {
        address_len: 5,
        address: [9, 2, 3, 4, 5, 0],
    };
    assert!(r.adaptive_bind_member(2, Some(&replacement), &mut p));
    assert_eq!(
        r.protocol.adaptive.snapshot().nodes[1].identity,
        replacement
    );
    assert!(!c::dedupe_contains(Some(&r.dedupe), 13, 2, 9));
    assert!(!c::dedupe_contains(
        Some(&r.protocol.control_presence_dedupe),
        14,
        2,
        9
    ));
    assert!(c::dedupe_contains(Some(&r.dedupe), 13, 3, 9));
    assert!(c::dedupe_contains(
        Some(&r.protocol.control_presence_dedupe),
        14,
        3,
        9
    ));
    assert_eq!((r.protocol.join_seq_seen, r.protocol.ack_seq_seen), (4, 4));
    assert!(!r.protocol.direct_seq_valid[1]);
    assert_eq!(
        (r.protocol.direct_quality[1], r.protocol.direct_seen[1]),
        (0, 0)
    );
    assert_eq!(
        (
            r.session.speaker_deadlines_ms[2],
            r.session.speaker_active_since_ms[2]
        ),
        (0, 0)
    );
    assert_eq!(p.trackers[2], (0, false));
    assert_eq!(p.trackers[3], (99, true));
    assert_eq!(r.session.speaker_deadlines_ms[3], 500);
    r.adaptive_reset();
    assert_eq!(r.protocol.adaptive.snapshot(), a::Snapshot::default());
    assert_eq!(r.protocol.control_presence_dedupe, Dedupe::default());
    assert_eq!(
        (
            r.protocol.term,
            r.protocol.join_seq_seen,
            r.protocol.ack_seq_seen
        ),
        (0, 0, 0)
    );
    assert_eq!(r.protocol.leader_identity, IdentityValue::default());
    assert_eq!(r.protocol.direct_seq_valid, [false; 8]);
    assert_eq!(r.protocol.direct_quality, [0; 8]);
    assert_eq!(r.protocol.direct_seen, [0; 8]);
    assert!(!r.protocol.priority_pending);
    assert_eq!(r.protocol.authoritative_members, 0);
}

#[test]
fn committed_candidate_waits_for_switch_frame_before_becoming_coordinator() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    let mut ops = Vec::new();
    setup(&mut ops, 2, 0);
    for op in ops {
        run_joint(&mut r, &mut p, op);
    }
    for id in 1..=3 {
        let mut report = JointOp::packet(Message::Topology(topo(id, 1)), id, 1, 99);
        report.0[0] = 30;
        report.0[1] = id;
        assert!(run_joint(&mut r, &mut p, report));
    }
    let proposal = wire::HandoverProposal {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 8,
        switch_frame: 260,
        members: 7,
        slot_map: slot_map(),
    };
    run_joint(
        &mut r,
        &mut p,
        JointOp::packet(Message::HandoverPrepare(proposal), 1, 2, 100).frame(100),
    );
    assert!(r.protocol.adaptive.snapshot().prepared);
    assert_eq!(r.membership.role, Role::Participant);
    run_joint(
        &mut r,
        &mut p,
        JointOp::packet(Message::HandoverCommit(proposal), 1, 3, 101),
    );
    assert!(r.protocol.adaptive.snapshot().committed);
    run_joint(&mut r, &mut p, JointOp::new(25, 102).frame(259));
    assert_eq!(r.membership.role, Role::Participant);
    assert_eq!(r.protocol.term, 7);
    run_joint(&mut r, &mut p, JointOp::new(25, 103).frame(260));
    assert_eq!(r.membership.role, Role::Coordinator);
    assert_eq!(r.membership.coordinator_id, 2);
    assert_eq!(r.protocol.term, 8);
    assert_eq!(r.protocol.adaptive.snapshot().leader_id, 2);
}

#[test]
fn failed_membership_publication_does_not_advance_success_time_or_queue() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    let mut ops = Vec::new();
    setup(&mut ops, 1, 100);
    for op in ops {
        run_joint(&mut r, &mut p, op);
    }
    r.protocol.last_membership_tx = 50;
    let head = r.session.control_head;
    let tail = r.session.control_tail;
    assert!(!run_joint(
        &mut r,
        &mut p,
        JointOp::new(28, 200).tx_failure(false, -105)
    ));
    assert_eq!(r.protocol.last_membership_tx, 50);
    assert_eq!(
        (r.session.control_head, r.session.control_tail),
        (head, tail)
    );
    assert!(run_joint(&mut r, &mut p, JointOp::new(28, 201)));
    assert_eq!(r.protocol.last_membership_tx, 201);
    assert_ne!(r.session.control_head, head);
    assert_eq!(r.session.control_tail, tail);
    assert_eq!(queued_payload(&r, 22).len(), wire::MembershipV3::SIZE);
}

#[test]
fn adaptive_checked_errors_and_owned_wrapper_scope() {
    let mut r = Runtime::default();
    let mut p = Joint::new();
    let msg = Message::SyncV3(wire::SyncV3 {
        leader: local(),
        leader_id: 1,
        member_count: 1,
        term: 7,
        frame_counter: 1,
        phase_us: 19999,
        relay_depth: 0,
    });
    let op = JointOp::packet(msg, 1, 0, 1);
    let h = header(&op.0);
    assert_eq!(
        r.adaptive_control_rx(h, &op.0[36..115], -60, 1000, &mut p),
        Err(crate::protocol_membership::Error::NotIntegrated)
    );
    let mut p = CombinedMembershipPlatform(&mut p);
    assert_eq!(
        r.adaptive_control_rx(h, &[], -60, 1000, &mut p),
        Err(crate::protocol_membership::Error::ShortPayload)
    );
    // Header length mismatch precedes backing-storage validation, just like C.
    let bad = Header {
        payload_len: 0,
        ..h
    };
    assert_eq!(
        r.adaptive_control_rx(bad, &[], -60, 1000, &mut p),
        Ok(false)
    );
    assert_eq!(
        r.adaptive_control_rx(Header { flags: 4, ..h }, &[], -60, 1000, &mut p),
        Ok(true)
    );
    assert_eq!(
        r.adaptive_control_rx(Header { ttl: 0, ..h }, &[], -60, 1000, &mut p),
        Ok(true)
    );
    // Defined signed division of MIN by 1000 remains allowed; subtracting phase
    // is rejected only when reached by a valid participant SYNC and overflows.
    r.membership = Membership {
        state: State::Active,
        role: Role::Participant,
        node_id: 2,
        slot_index: 1,
        coordinator_id: 1,
        ..Membership::default()
    };
    r.address = [2, 2, 3, 4, 5];
    r.protocol.term = 7;
    r.protocol.leader_identity = IdentityValue {
        address_len: 5,
        address: local().address,
    };
    p.0.inner.now = i64::from((i64::MIN / 1000) as u32);
    r.adaptive_members_owned(&mut p);
    assert_eq!(
        r.adaptive_control_rx(h, &op.0[36..115], -60, i64::MIN, &mut p),
        Err(crate::protocol_membership::Error::UndefinedArithmetic)
    );
}

#[test]
fn joint_corpus_owned_state_and_ordered_io_review() {
    let ops = joint_fixtures();
    let mut p = Joint::new();
    let mut r = Runtime::default();
    for op in &ops {
        let before = ReviewBefore::capture(&r, &p);
        let handled = run_joint(&mut r, &mut p, *op);
        check_review(*op, handled, &before, &r, &p);
        assert_eq!(
            r.pending,
            Some(Command {
                kind: CommandKind::Stop,
                generation: 200,
                talk_channel: 3
            })
        );
        assert!(r.status_pending);
        assert_eq!(
            r.audio_state.boundary,
            AudioBoundary {
                enabled: true,
                epoch: 999,
                receipt_cutoff_us: 777
            }
        );
    }
    std::eprintln!(
        "joint inputs={} snapshots={}; TX/audio algorithm mocks",
        ops.len(),
        ops.len()
    );
}

#[test]
fn sync_member_count_mismatch_rejoins_only_after_three_report_periods() {
    use crate::protocol_adaptive::sync_member_mismatch;
    // Agreement clears any pending mismatch.
    assert_eq!(sync_member_mismatch(None, 10, false), (None, false));
    assert_eq!(sync_member_mismatch(Some(10), 5000, false), (None, false));
    // First disagreement only stamps the time.
    assert_eq!(sync_member_mismatch(None, 100, true), (Some(100), false));
    // A transient mismatch (coordinator's MembershipV3 still in flight) is tolerated.
    assert_eq!(
        sync_member_mismatch(Some(100), 3100, true),
        (Some(100), false)
    );
    // Persisting past 3 × REPORT_MS forces a rejoin and clears the stamp.
    assert_eq!(sync_member_mismatch(Some(100), 3101, true), (None, true));
    // Wrap-safe uptime arithmetic.
    assert_eq!(
        sync_member_mismatch(Some(u32::MAX - 10), 3000, true),
        (None, true)
    );
}

#[test]
fn membership_reject_counter_classifies_first_failing_check() {
    use crate::protocol_adaptive::membership_reject_counter;
    use crate::protocol_membership::MembershipRx;
    let ident = |b: u8| a::Identity {
        address: [b, 2, 3, 4, 5, 0],
        address_len: 5,
    };
    let mut s = a::Snapshot {
        local_id: 2,
        leader_id: 1,
        term: 7,
        members: 0b11,
        membership_revision: 5,
        membership_revision_seen: true,
        ..Default::default()
    };
    s.nodes[0].identity = ident(1);
    s.nodes[1].identity = ident(2);
    let mut m = a::Membership {
        term: 7,
        leader_id: 1,
        revision: 6,
        slot_map: c::SlotMap {
            slot_count: 2,
            slot_ids: [1, 2, 0, 0, 0, 0, 0, 0],
            ..Default::default()
        },
        identities: Default::default(),
    };
    m.identities[0] = ident(1);
    m.identities[1] = ident(2);
    let mut evicted = m;
    evicted.slot_map.slot_count = 1;
    evicted.slot_map.slot_ids[1] = 0;
    evicted.identities[1] = a::Identity::default();
    let mut wrong_leader = m;
    wrong_leader.identities[0] = ident(9);
    let mut c = MembershipRx::default();
    *membership_reject_counter(&mut c, &s, 1, &evicted, true) += 1;
    *membership_reject_counter(&mut c, &s, 2, &m, false) += 1;
    *membership_reject_counter(&mut c, &s, 1, &a::Membership { term: 8, ..m }, false) += 1;
    *membership_reject_counter(&mut c, &s, 1, &a::Membership { revision: 5, ..m }, false) += 1;
    *membership_reject_counter(&mut c, &s, 1, &evicted, false) += 1;
    *membership_reject_counter(&mut c, &s, 1, &wrong_leader, false) += 1;
    assert_eq!(
        c,
        MembershipRx {
            applied: 0,
            removed_local: 1,
            source: 1,
            term: 1,
            revision: 1,
            identity: 1,
            other: 1
        }
    );
}
