use super::*;
use crate::protocol_membership::{
    IdentityValue, Input, MembershipEffect as M, MembershipPlatform, ProtocolState,
};
use openhelmet_mesh::{adaptive as a, core as c};
use openhelmet_wire::mesh::{self as wire, Header};
use std::{vec, vec::Vec};

#[path = "adaptive_tests.rs"]
mod adaptive_tests;

fn p32(v: &mut Vec<u8>, n: u32) {
    v.extend(n.to_le_bytes());
}
fn p64(v: &mut Vec<u8>, n: u64) {
    v.extend(n.to_le_bytes());
}
fn ident(v: &mut Vec<u8>, n: IdentityValue) {
    v.push(n.address_len);
    v.extend(n.address);
}
fn slots(v: &mut Vec<u8>, s: c::SlotMap) {
    v.push(s.slot_count);
    v.extend(s.slot_ids);
    v.push(s.active_speaker_count);
    v.extend(s.active_speaker_ids);
    v.extend(s.relay_masks);
}
fn adaptive(v: &mut Vec<u8>, s: a::Snapshot) {
    v.extend([s.local_id, s.leader_id, s.members]);
    p32(v, s.term);
    p32(v, s.leader_since_ms);
    p32(v, s.better_since_ms);
    v.push(s.better_id);
    for n in s.nodes {
        v.extend([n.present as u8, n.report_seen as u8, n.seq_seen as u8]);
        ident(v, n.identity);
        p32(v, n.last_seen_ms);
        p32(v, n.report_ms);
        p32(v, n.report_seq);
        v.extend([n.loss_quality, n.direct_mask]);
        v.extend(n.rssi.map(|x| x as u8));
        v.extend(n.quality);
        v.push(n.leader_id);
        p32(v, n.term);
    }
    v.extend(s.edge_live);
    let h = s.pending;
    v.extend([h.old_leader_id, h.candidate_id]);
    p32(v, h.next_term);
    p32(v, h.switch_frame);
    v.push(h.members);
    slots(v, h.slot_map);
    v.push(s.ack_mask);
    p32(v, s.deadline_ms);
    p32(v, s.recovery_deadline_ms);
    p32(v, s.membership_revision);
    p32(v, s.publish_revision);
    v.push(s.membership_revision_seen as u8);
    for n in s.request_seq {
        p32(v, n);
    }
    for n in s.request_deadline_ms {
        p32(v, n);
    }
    v.extend(s.request_seq_seen.map(u8::from));
    v.extend(s.request_active.map(u8::from));
    v.extend([s.prepared as u8, s.committed as u8, s.transition_done as u8]);
    for k in s.cache {
        v.extend([k.packet_type, k.src_id, k.seq]);
        p32(v, k.term);
        p32(v, k.seen_ms);
        ident(v, k.origin);
        v.push(k.used as u8);
    }
    for q in s.queue {
        let h = q.header;
        v.extend([
            h.version,
            h.packet_type,
            h.src_id,
            h.seq,
            h.ttl,
            h.flags,
            h.talk_channel,
        ]);
        v.extend(h.payload_len.to_le_bytes());
        v.extend(q.payload);
        v.extend(q.payload_len.to_le_bytes());
        p32(v, q.enqueued_ms);
    }
    v.extend([s.queue_head, s.queue_count]);
}
fn dedupe(v: &mut Vec<u8>, d: &Dedupe) {
    let s = d.snapshot();
    for k in s.entries {
        v.extend([k.packet_type, k.src_id, k.seq, k.valid as u8]);
    }
    v.push(s.next);
}
fn snapshot(v: &mut Vec<u8>, r: &Runtime) {
    let m = r.membership;
    v.extend([
        b'S',
        m.state as u8,
        m.role as u8,
        m.peer_count,
        m.membership_known as u8,
        m.node_id,
        m.slot_index as u8,
        m.coordinator_id,
    ]);
    v.extend(r.address);
    p32(v, r.protocol.term);
    ident(v, r.protocol.leader_identity);
    p32(v, r.protocol.join_attempts as u32);
    p32(v, r.protocol.last_sync_time);
    for p in r.session.peers {
        v.push(p.node_id);
        v.extend(p.address);
        v.push(p.slot_index as u8);
        p64(v, p.last_seen_ms as u64);
        v.extend([
            p.battery_pct,
            p.rssi_dbm as u8,
            p.peer_count,
            p.fw_version,
            p.temperature_c as u8,
            p.heard_bitmap,
            p.relay_bitmap,
            p.active as u8,
            p.announced as u8,
        ]);
    }
    for e in r
        .session
        .relay_ring
        .iter()
        .chain(r.session.control_ring.iter())
    {
        v.extend(e.data);
        v.push(e.len);
        p32(v, e.queued_ms);
    }
    for n in r.session.speaker_deadlines_ms {
        p64(v, n as u64);
    }
    for n in r.session.speaker_active_since_ms {
        p64(v, n as u64);
    }
    v.extend([
        r.session.relay_head,
        r.session.relay_tail,
        r.session.control_head,
        r.session.control_tail,
        r.session.heard_bitmap,
        r.session.relay_bitmap,
    ]);
    p32(v, r.session.skip_count);
    dedupe(v, &r.dedupe);
    dedupe(v, &r.protocol.control_presence_dedupe);
    adaptive(v, r.protocol.adaptive.snapshot());
    for i in 0..3 {
        v.push(r.timers[i].is_some() as u8);
        p64(v, r.generations[i]);
        p64(v, r.timers[i].map(|t| t.deadline_us as u64).unwrap_or(0));
    }
    v.push(r.talk_channel);
    p32(v, r.auto_ticks);
    p32(v, r.status_log_decim);
    p32(v, r.membership_scan_timeout_ms());
    v.push(r.membership_bridge_peer_count());
}

struct Simulation {
    bytes: Vec<u8>,
    now: i64,
    bind_mask: u8,
    bind_index: u8,
    sequence: bool,
    logs: bool,
}
impl Simulation {
    fn e(&mut self, id: u8) {
        self.bytes.extend([b'E', id]);
    }
}
impl Platform for Simulation {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now * 1000
    }
    fn apply(&mut self, e: Effect) -> Reply {
        match e {
            Effect::Schedule(t) => {
                self.e(30);
                self.bytes.push(t.timer as u8);
                let now = self.now_us();
                p32(&mut self.bytes, ((t.deadline_us - now) / 1000) as u32);
            }
            Effect::SchedulePending(t, ms) => {
                self.e(30);
                self.bytes.push(t as u8);
                p32(&mut self.bytes, ms);
            }
            Effect::Cancel(t) => {
                self.e(31);
                self.bytes.push(t as u8);
            }
            Effect::GetAddress => {
                self.e(20);
                self.bytes.extend([9, 8, 7, 6, 5]);
                return Reply::Address([9, 8, 7, 6, 5]);
            }
            _ => panic!("unexpected runtime boundary {e:?}"),
        }
        Reply::None
    }
}
impl MembershipPlatform for Simulation {
    fn membership_effect(&mut self, e: M, s: &mut ProtocolState) -> bool {
        match e {
            M::BindWork => self.e(0),
            M::AdaptiveReset => {
                self.e(1);
                s.reset_boundary_model();
            }
            M::BindMember { id, identity } => {
                self.e(2);
                self.bytes.extend([id, identity.is_some() as u8]);
                if let Some(x) = identity {
                    ident(&mut self.bytes, x);
                }
                let scripted = self.bind_mask & (1 << (self.bind_index % 8)) != 0;
                self.bind_index += 1;
                let result =
                    scripted && a::set_member(Some(&mut s.adaptive), id, identity.as_ref());
                self.bytes.push(result as u8);
                return result;
            }
            M::ControlSequence {
                id,
                packet_type,
                seq,
            } => {
                self.e(3);
                self.bytes
                    .extend([id, packet_type, seq, self.sequence as u8]);
                return self.sequence;
            }
            M::AdaptiveMembers {
                state,
                local,
                leader,
                term,
            } => {
                self.e(4);
                self.bytes.extend([state as u8, local, leader]);
                p32(&mut self.bytes, term);
                if state == State::Active && s.adaptive.snapshot().local_id == 0 {
                    a::init(
                        Some(&mut s.adaptive),
                        local,
                        leader,
                        term,
                        c::node_bit(local) | c::node_bit(leader),
                        self.now as u32,
                    );
                }
            }
            M::ResetTracker(id) => {
                self.e(5);
                self.bytes.push(id);
            }
            M::ResetTrackers => self.e(6),
            M::ClearRelay => self.e(7),
            M::ClearActivity => self.e(8),
            M::ClearGrants => self.e(9),
            M::ClearBitmaps => self.e(10),
            M::PurgeTx => self.e(11),
            M::Ingress { enabled, purge } => {
                self.e(12);
                self.bytes.extend([enabled as u8, purge as u8]);
            }
            M::UpdateGrants => self.e(13),
            M::SlotMap => self.e(14),
            M::JoinRequest => self.e(15),
            M::JoinAck { id, slot, address } => {
                self.e(16);
                self.bytes.extend([id, slot]);
                self.bytes.extend(address);
            }
            M::TdmaStop => self.e(17),
            M::TdmaStart { slot, coordinator } => {
                self.e(18);
                self.bytes.extend([slot as u8, coordinator as u8]);
            }
            M::ClockSource(x) => {
                self.e(19);
                self.bytes.push(x as u8);
            }
            M::Status(m) => {
                self.e(21);
                self.bytes.extend([
                    m.state as u8,
                    m.role as u8,
                    m.peer_count,
                    m.node_id,
                    m.slot_index as u8,
                    m.coordinator_id,
                ]);
            }
            M::Event { id, node } => {
                self.e(22);
                self.bytes.extend([id, node.is_some() as u8]);
                if let Some(n) = node {
                    self.bytes.push(n);
                }
            }
            M::LogInitialized => {
                self.e(23);
                self.bytes.push(self.logs as u8);
                return self.logs;
            }
            M::Log { bytes, len } => {
                self.e(24);
                self.bytes.push(len);
                self.bytes.extend(&bytes[..usize::from(len)]);
            }
        }
        true
    }
}

#[derive(Clone, Copy)]
struct Op([u8; 80]);
impl Op {
    fn new(code: u8, now: i64) -> Self {
        let mut x = [0; 80];
        x[0] = code;
        x[8..16].copy_from_slice(&now.to_le_bytes());
        x[68] = 255;
        x[69] = 1;
        x[70] = 1;
        Self(x)
    }
    fn arg(mut self, n: usize, x: u8) -> Self {
        self.0[n] = x;
        self
    }
    fn term(mut self, term: u32) -> Self {
        self.0[16..20].copy_from_slice(&term.to_le_bytes());
        self
    }
    fn identity(mut self, id: wire::Identity) -> Self {
        id.encode(&mut self.0[20..27]).unwrap();
        self
    }
    fn addr(mut self, address: [u8; 5]) -> Self {
        self.0[36..41].copy_from_slice(&address);
        self
    }
    fn packet(kind: u8, source: u8, seq: u8, payload: &[u8], now: i64) -> Self {
        let mut x = Self::new(11, now);
        let h = Header {
            version: 5,
            packet_type: kind,
            src_id: source,
            seq,
            ttl: 2,
            flags: 0,
            talk_channel: 1,
            payload_len: payload.len() as u16,
        };
        wire::encode(&h, payload, &mut x.0[27..], 32).unwrap();
        x
    }
}
fn g32(x: &[u8]) -> u32 {
    u32::from_le_bytes(x[..4].try_into().unwrap())
}
fn g64(x: &[u8]) -> i64 {
    i64::from_le_bytes(x[..8].try_into().unwrap())
}
fn run_op(r: &mut Runtime, p: &mut Simulation, op: Op) {
    let x = op.0;
    p.now = g64(&x[8..]);
    p.bind_mask = x[68];
    p.bind_index = 0;
    p.sequence = x[69] != 0;
    p.logs = x[70] != 0;
    let w = wire::Identity::decode(&x[20..27]).unwrap();
    let id = IdentityValue {
        address_len: w.address_len,
        address: w.address,
    };
    let input = match x[0] {
        0 => {
            *r = Runtime::default();
            None
        }
        1 => {
            r.membership = Membership {
                state: [State::Idle, State::Scanning, State::Joining, State::Active][x[1] as usize],
                role: [Role::None, Role::Coordinator, Role::Participant][x[2] as usize],
                node_id: x[3],
                slot_index: x[4] as i8,
                coordinator_id: x[5],
                peer_count: x[6],
                membership_known: x[7] != 0,
            };
            r.address.copy_from_slice(&x[36..41]);
            r.protocol.term = g32(&x[16..]);
            r.protocol.leader_identity = id;
            r.protocol.join_attempts = g32(&x[41..]) as i32;
            r.protocol.last_sync_time = g32(&x[45..]);
            None
        }
        2 => {
            r.session.peers[usize::from(x[1] % 8)] = Peer {
                node_id: x[2],
                slot_index: x[3] as i8,
                active: x[4] != 0,
                announced: x[5] != 0,
                address: x[36..41].try_into().unwrap(),
                last_seen_ms: g64(&x[41..]),
                battery_pct: 77,
                rssi_dbm: -42,
                peer_count: 6,
                fw_version: 13,
                temperature_c: -7,
                heard_bitmap: 99,
                relay_bitmap: 33,
            };
            None
        }
        3 => {
            for (i, e) in r
                .session
                .relay_ring
                .iter_mut()
                .chain(r.session.control_ring.iter_mut())
                .enumerate()
            {
                e.data.fill(i as u8 + 1);
                e.len = i as u8 + 1;
                e.queued_ms = 500 + i as u32;
            }
            for i in 0..9 {
                r.session.speaker_deadlines_ms[i] = 123 + i as i64;
                r.session.speaker_active_since_ms[i] = 321 + i as i64;
            }
            r.session.relay_head = 7;
            r.session.relay_tail = 3;
            r.session.control_head = 23;
            r.session.control_tail = 11;
            r.session.heard_bitmap = 85;
            r.session.relay_bitmap = 170;
            r.session.skip_count = 1234;
            r.auto_ticks = 456;
            r.status_log_decim = 789;
            for i in 0..40 {
                c::dedupe_accept(Some(&mut r.dedupe), 13, i % 8 + 1, i);
                c::dedupe_accept(
                    Some(&mut r.protocol.control_presence_dedupe),
                    14,
                    i % 8 + 1,
                    i,
                );
            }
            None
        }
        4 => Some(Input::BindWork),
        5 => Some(Input::ResetSession),
        6 => Some(Input::UpdateLastSeen {
            node: x[1],
            rssi: x[2] as i8,
        }),
        7 => Some(Input::NotePresence(x[1])),
        8 => Some(Input::Discover(x[1])),
        9 => Some(Input::Demote {
            leader: x[1],
            term: g32(&x[16..]),
            identity: id,
        }),
        10 => Some(Input::Rejoin {
            leader: x[1],
            term: g32(&x[16..]),
            identity: id,
        }),
        11 => {
            let h = Header {
                version: x[27],
                packet_type: x[28],
                src_id: x[29],
                seq: x[30],
                ttl: x[31],
                flags: x[32],
                talk_channel: x[33],
                payload_len: u16::from_le_bytes([x[34], x[35]]),
            };
            Some(Input::Packet {
                header: h,
                payload: &x[36..36 + usize::from(h.payload_len)],
            })
        }
        12 => Some(Input::ScanWork),
        13 => Some(Input::JoinWork),
        14 => Some(Input::CheckPeerTimeouts),
        15 => Some(Input::CoordinatorTimeout),
        16 => r.timers[usize::from(x[1] % 2)].map(Input::Timer),
        17 => {
            r.mesh_log(format_args!("fixture {}/{}", 255u8, -7i8), p);
            None
        }
        18 => {
            let own = IdentityValue {
                address_len: 5,
                address: local().address,
            };
            let other = IdentityValue {
                address_len: 5,
                address: remote(2).address,
            };
            let adaptive = &mut r.protocol.adaptive;
            a::init(Some(&mut *adaptive), 1, 1, 7, 3, p.now as u32);
            assert!(a::set_member(Some(&mut *adaptive), 1, Some(&own)));
            assert!(a::set_member(Some(&mut *adaptive), 2, Some(&other)));
            let mut payload = [0; wire::JoinAckV3::SIZE];
            wire::JoinAckV3 {
                target: remote(3),
                capabilities: 2,
                assigned_id: 3,
                slot_index: 2,
                coordinator_id: 1,
                term: 7,
            }
            .encode(&mut payload)
            .unwrap();
            for seq in 10..18 {
                let header = a::Header {
                    version: 5,
                    packet_type: 16,
                    src_id: 1,
                    seq,
                    ttl: 2,
                    flags: 0,
                    talk_channel: 1,
                    payload_len: payload.len() as u16,
                };
                assert!(a::forward_enqueue(
                    Some(&mut *adaptive),
                    Some(&header),
                    Some(&payload),
                    Some(&own),
                    7,
                    true,
                    p.now as u32
                ));
            }
            None
        }
        _ => panic!("unknown fixture"),
    };
    let result = input.is_some_and(|i| r.membership_handle(i, p).unwrap());
    p.bytes.extend([b'R', result as u8]);
    snapshot(&mut p.bytes, r);
    let s = r.membership_snapshot();
    r.apply_membership_snapshot(s);
}

fn local() -> wire::Identity {
    wire::Identity {
        address_len: 5,
        address: [1, 2, 3, 4, 5, 0],
    }
}
fn remote(n: u8) -> wire::Identity {
    wire::Identity {
        address_len: 5,
        address: [n, 2, 3, 4, 5, 0],
    }
}
fn seed(state: u8, role: u8, count: u8, now: i64) -> Op {
    Op::new(1, now)
        .arg(1, state)
        .arg(2, role)
        .arg(3, if role == 2 { 2 } else { 1 })
        .arg(4, if role == 2 { 1 } else { 0 })
        .arg(5, 1)
        .arg(6, count)
        .term(7)
        .identity(local())
        .addr([1, 2, 3, 4, 5])
}
fn peer(index: u8, id: u8, announced: bool, now: i64) -> Op {
    Op::new(2, now)
        .arg(1, index)
        .arg(2, id)
        .arg(3, index)
        .arg(4, 1)
        .arg(5, announced as u8)
        .addr([id, 2, 3, 4, 5])
}
fn join(origin: wire::Identity, capabilities: u8, seq: u8, now: i64) -> Op {
    let mut payload = [0; wire::JoinV3::SIZE];
    wire::JoinV3 {
        origin,
        capabilities,
        target: local(),
    }
    .encode(&mut payload)
    .unwrap();
    Op::packet(15, 0, seq, &payload, now)
}
fn ack(target: wire::Identity, node: u8, slot: u8, leader: u8, term: u32, now: i64) -> Op {
    let mut payload = [0; wire::JoinAckV3::SIZE];
    wire::JoinAckV3 {
        target,
        capabilities: 0,
        assigned_id: node,
        slot_index: slot,
        coordinator_id: leader,
        term,
    }
    .encode(&mut payload)
    .unwrap();
    Op::packet(16, leader, 0, &payload, now)
}
fn fixtures() -> Vec<Op> {
    let mut ops = vec![
        Op::new(0, 0),
        Op::new(4, 0),
        Op::new(17, 0),
        Op::new(17, 0).arg(70, 0),
    ];
    // Entropy, invalid discovery and pending timer ownership, all ten retries.
    for suffix in [0u16, 500, 501, 65535] {
        ops.push(Op::new(0, 0));
        ops.push(seed(1, 0, 3, 0).addr([1, 2, 3, (suffix >> 8) as u8, suffix as u8]));
        ops.push(Op::new(8, 0).arg(1, 0));
        ops.push(Op::new(8, 0).arg(1, 9));
        ops.push(Op::new(8, 0).arg(1, 1));
        for i in 0..=10 {
            ops.push(Op::new(16, i * 500).arg(1, 1));
        }
        ops.push(Op::new(13, 6000));
        ops.push(Op::new(12, 6000));
    }
    // Real scan allocation preserves preexisting telemetry, not full peer reset.
    ops.extend([
        Op::new(0, 0),
        seed(1, 0, 0, 0),
        peer(0, 6, false, 0),
        Op::new(12, 100),
        Op::new(12, 100),
    ]);
    // JOIN validation, scripted wrapper sequence/bind outcomes, duplicate reuse.
    ops.extend([Op::new(0, 0), seed(3, 1, 0, 0), Op::new(3, 0)]);
    ops.extend([
        join(local(), 2, 1, 1),
        join(remote(2), 0, 1, 1),
        join(
            wire::Identity {
                address_len: 6,
                ..remote(2)
            },
            2,
            1,
            1,
        ),
        join(
            wire::Identity {
                address_len: 5,
                address: [0; 6],
            },
            2,
            1,
            1,
        ),
        join(
            wire::Identity {
                address_len: 5,
                address: [255; 6],
            },
            2,
            1,
            1,
        ),
        join(remote(2), 2, 1, 1).arg(29, 1),
        join(remote(2), 2, 1, 1).arg(68, 0),
        join(remote(2), 2, 1, 1).arg(68, 1),
        join(remote(2), 2, 1, 2),
        join(remote(2), 2, 1, 3).arg(69, 0),
        join(remote(2), 2, 2, 4),
        Op::new(6, 5).arg(1, 2).arg(2, 200),
        Op::new(7, 6).arg(1, 2),
    ]);
    for id in 3..=8 {
        ops.push(join(remote(id), 2, id, 10));
    }
    ops.push(join(remote(9), 2, 9, 10));
    // Full physical peer storage with duplicate IDs/slots is still C-defined.
    // C can ACK an allocation even when no inactive peer entry can be written.
    ops.extend([Op::new(0, 0), seed(3, 1, 7, 0)]);
    for i in 0..8 {
        ops.push(peer(i, 1, true, 0).arg(3, 0));
    }
    ops.push(join(remote(9), 2, 1, 1));
    // Unsigned promoted shifts at 8..31 are defined, then narrow to zero.
    for slot in [8, 31] {
        ops.extend([
            Op::new(0, 0),
            seed(3, 1, 0, 0).arg(4, slot),
            peer(0, 7, false, 0).arg(4, 0),
            join(remote(2), 2, 1, 1),
        ]);
    }
    // Identity equality includes all six stored bytes, not just the ESB prefix.
    ops.extend([
        Op::new(0, 0),
        seed(3, 1, 0, 0),
        join(
            wire::Identity {
                address: [2, 2, 3, 4, 5, 99],
                ..remote(2)
            },
            2,
            1,
            1,
        ),
        join(remote(2), 2, 1, 1).arg(50, 99),
        join(remote(2), 2, 1, 1).arg(27, 255),
    ]);
    for kind in 0..=24 {
        ops.push(Op::packet(kind, 0, 0, &[], 10));
    }
    // Announcement vs presence, strict peer timeout, inactive storage retention.
    ops.extend([
        Op::new(0, 0),
        seed(3, 1, 0, 0),
        peer(0, 1, true, 0),
        peer(1, 2, false, 0),
        peer(2, 3, true, 0),
        Op::new(3, 0),
        Op::new(7, 1).arg(1, 2),
        Op::new(6, 1).arg(1, 2).arg(2, 180),
        Op::new(6, 2).arg(1, 2).arg(2, 190),
        Op::new(14, 3000),
        Op::new(14, 3001),
        Op::new(14, 3002),
        Op::new(14, 3003),
    ]);
    // LEAVE identity, local/invalid IDs and announced/unannounced removal.
    for (id, announced) in [(0, false), (1, true), (2, false), (3, true), (9, true)] {
        ops.extend([
            Op::new(0, 0),
            seed(3, 1, 3, 0),
            peer(1, id, announced, 0),
            Op::new(3, 0),
            Op::packet(4, id, 0, &[99; 5], 1),
            Op::packet(4, id, 0, &[id, 2, 3, 4, 5], 2),
            Op::packet(4, id, 0, &[id, 2, 3, 4, 5], 3),
        ]);
    }
    // Seed nonzero adaptive cache/queue through shared APIs, remove a member
    // (logical queue clear preserves stale bytes), then full fake reset zeroes it.
    ops.extend([
        Op::new(0, 0),
        seed(3, 1, 1, 0),
        peer(1, 2, true, 0),
        Op::new(18, 0),
        Op::packet(4, 2, 0, &[2, 2, 3, 4, 5], 1),
        Op::new(5, 2),
    ]);
    // Target ACK, malformed IDs/slots/term/length, preserving inactive fields.
    ops.extend([
        Op::new(0, 0),
        seed(2, 0, 3, 0),
        peer(0, 7, false, 0).arg(4, 0),
        Op::new(3, 0),
    ]);
    ops.extend([
        ack(remote(2), 2, 1, 1, 7, 1),
        ack(local(), 2, 1, 1, 8, 1),
        ack(local(), 0, 1, 1, 7, 1),
        ack(local(), 2, 8, 1, 7, 1),
        ack(local(), 1, 0, 1, 7, 1),
        ack(local(), 2, 1, 9, 7, 1),
        ack(local(), 2, 1, 1, 7, 1).arg(29, 2),
        ack(
            wire::Identity {
                address_len: 6,
                ..local()
            },
            2,
            1,
            1,
            7,
            1,
        ),
        ack(local(), 2, 1, 1, 7, 2).arg(42, 99),
        ack(local(), 2, 1, 1, 7, 2),
        ack(local(), 2, 1, 1, 7, 3),
    ]);
    // Known/unknown bridge count and unsigned coordinator-time wrap.
    ops.push(seed(3, 2, 3, 0));
    ops.push(seed(3, 2, 3, 0).arg(7, 1));
    let mut wrapping = seed(3, 2, 3, 4_294_967_000);
    wrapping.0[45..49].copy_from_slice(&4_294_966_000u32.to_le_bytes());
    ops.extend([
        wrapping,
        Op::new(15, 4_294_971_000),
        Op::new(15, 4_294_971_001),
    ]);
    // Demote preserves membership-known/bitmaps; rejoin clears both. Reset order.
    for (role, code) in [(1, 9), (2, 10)] {
        ops.extend([
            Op::new(0, 0),
            seed(3, role, 4, 0).arg(7, 1),
            peer(5, 6, true, 0),
            Op::new(3, 0),
            Op::new(code, 1).arg(1, 8).term(42).identity(remote(8)),
            Op::new(code, 2).arg(1, 8).term(43).identity(remote(8)),
            Op::new(5, 3),
        ]);
    }
    // Deterministic serialized mix within defined signed timestamp/shift domain.
    let mut entropy = 0x5123_9abcu32;
    for i in 0..512 {
        entropy = entropy.wrapping_mul(1664525).wrapping_add(1013904223);
        let t = i64::from(i) * 37;
        match entropy % 8 {
            0 => ops.push(seed(3, 1, (entropy >> 8) as u8, t)),
            1 => ops.push(peer(
                (entropy >> 8) as u8 % 8,
                (entropy >> 16) as u8 % 10,
                entropy & 256 != 0,
                t,
            )),
            2 => ops.push(
                Op::new(6, t)
                    .arg(1, (entropy >> 8) as u8 % 10)
                    .arg(2, (entropy >> 16) as u8),
            ),
            3 => ops.push(Op::new(7, t).arg(1, (entropy >> 8) as u8 % 10)),
            4 => ops.push(Op::new(14, t)),
            5 => ops.push(
                join(remote((entropy >> 8) as u8), 2, entropy as u8, t)
                    .arg(69, (entropy & 1) as u8),
            ),
            6 => ops.push(Op::new(5, t)),
            _ => ops.push(Op::new(3, t)),
        }
    }
    ops
}

#[test]
fn discovery_retries_then_rescans_without_allocating_a_member() {
    let mut p = Simulation {
        bytes: Vec::new(),
        now: 0,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    let mut r = Runtime::default();
    run_op(&mut r, &mut p, seed(1, 0, 0, 0));
    let before = r.membership;
    for leader in [0, 9, 255] {
        run_op(&mut r, &mut p, Op::new(8, 0).arg(1, leader));
        assert_eq!(r.membership, before);
    }
    run_op(&mut r, &mut p, Op::new(8, 0).arg(1, 1));
    assert_eq!(r.membership.state, State::Joining);
    assert_eq!(r.protocol.join_attempts, 0);
    assert!(r.timers[Timer::Scan as usize].is_none());
    for attempt in 1..=10 {
        run_op(&mut r, &mut p, Op::new(13, i64::from(attempt) * 500));
        assert_eq!(r.protocol.join_attempts, attempt);
        assert_eq!(r.membership.state, State::Joining);
    }
    run_op(&mut r, &mut p, Op::new(13, 5500));
    assert_eq!(r.membership.state, State::Scanning);
    assert_eq!(r.membership.role, Role::None);
    assert_eq!(
        (
            r.membership.node_id,
            r.membership.slot_index,
            r.membership.coordinator_id
        ),
        (0, -1, 0)
    );
    assert_eq!(r.protocol.join_attempts, 10);
    assert!(r.timers[Timer::Scan as usize].is_some());
    assert!(r.session.peers.iter().all(|peer| !peer.active));
}

#[test]
fn ack_requires_target_and_term_and_retires_join_timer_on_activation() {
    let mut p = Simulation {
        bytes: Vec::new(),
        now: 0,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    let mut r = Runtime::default();
    run_op(&mut r, &mut p, seed(1, 0, 3, 0));
    run_op(&mut r, &mut p, Op::new(8, 0).arg(1, 1));
    let stale = r.timers[Timer::Join as usize].unwrap();
    let before = r.membership;
    for op in [ack(remote(9), 2, 1, 1, 7, 1), ack(local(), 2, 1, 1, 8, 1)] {
        run_op(&mut r, &mut p, op);
        assert_eq!(r.membership, before);
        assert_eq!(r.timers[Timer::Join as usize], Some(stale));
        assert!(r.session.peers.iter().all(|peer| !peer.active));
    }
    run_op(&mut r, &mut p, ack(local(), 2, 1, 1, 7, 2));
    assert_eq!(r.membership.state, State::Active);
    assert_eq!(r.membership.role, Role::Participant);
    assert_eq!(
        (
            r.membership.node_id,
            r.membership.slot_index,
            r.membership.coordinator_id
        ),
        (2, 1, 1)
    );
    assert!(r.timers[Timer::Join as usize].is_none());
    assert!(r.timers[Timer::Status as usize].is_some());
    let peer = r.session.peers.iter().find(|peer| peer.active).unwrap();
    assert_eq!(peer.node_id, 1);
    assert_eq!(peer.address, [1, 2, 3, 4, 5]);
    assert!(peer.announced);
    let before = r.membership;
    let effects = p.bytes.len();
    assert!(!r.membership_handle(Input::Timer(stale), &mut p).unwrap());
    assert_eq!(r.membership, before);
    assert_eq!(p.bytes.len(), effects);
}

#[test]
fn leave_requires_bound_peer_identity_and_removes_only_that_peer() {
    let mut p = Simulation {
        bytes: Vec::new(),
        now: 0,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    let mut r = Runtime::default();
    run_op(&mut r, &mut p, seed(3, 1, 2, 0));
    run_op(&mut r, &mut p, peer(0, 2, true, 0));
    run_op(&mut r, &mut p, peer(1, 3, true, 0));
    let other = r.session.peers[1];
    run_op(&mut r, &mut p, Op::packet(4, 2, 0, &[99; 5], 1));
    assert!(r.session.peers[0].active);
    assert_eq!(r.membership.peer_count, 2);
    run_op(&mut r, &mut p, Op::packet(4, 2, 1, &[2, 2, 3, 4, 5], 2));
    assert!(!r.session.peers[0].active);
    assert_eq!(r.membership.peer_count, 1);
    assert_eq!(r.session.peers[1], other);
    run_op(&mut r, &mut p, Op::packet(4, 2, 2, &[2, 2, 3, 4, 5], 3));
    assert_eq!(r.membership.peer_count, 1);
    assert_eq!(r.session.peers[1], other);
}

#[test]
// Historical 753-operation boundary fixtures intentionally retain scripted
// bind/sequence failures. They qualify standalone MembershipMockCompatibility,
// not the owned wrappers tested separately by adaptive_tests.
fn membership_corpus_never_touches_kernel_command_or_audio_ownership() {
    let ops = fixtures();
    let mut p = Simulation {
        bytes: Vec::new(),
        now: 0,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    let mut r = Runtime::default();
    for op in &ops {
        run_op(&mut r, &mut p, *op);
        // Membership never touches the existing kernel command/audio ownership.
        assert_eq!(r.pending, None);
        assert!(!r.status_pending);
        assert_eq!(r.audio_state.boundary, AudioBoundary::default());
    }
    std::println!(
        "membership: {} operations; owned kernel command/audio state untouched",
        ops.len()
    );
}

#[test]
fn membership_rejects_undefined_arithmetic_and_short_backing_storage_only() {
    let mut p = Simulation {
        bytes: Vec::new(),
        now: i64::MAX / 1000,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    let mut r = Runtime::default();
    r.membership.role = Role::Coordinator;
    r.session.peers[0].active = true;
    r.session.peers[0].node_id = 2;
    r.session.peers[0].last_seen_ms = i64::MIN;
    assert_eq!(
        r.membership_handle(Input::CheckPeerTimeouts, &mut p),
        Err(crate::protocol_membership::Error::UndefinedArithmetic)
    );
    let op = join(remote(2), 2, 0, 0);
    p.now = 0;
    let h = Header {
        version: 5,
        packet_type: 15,
        src_id: 0,
        seq: 0,
        ttl: 2,
        flags: 0,
        talk_channel: 1,
        payload_len: wire::JoinV3::SIZE as u16,
    };
    assert_eq!(
        r.membership_handle(
            Input::Packet {
                header: h,
                payload: &[]
            },
            &mut p
        ),
        Err(crate::protocol_membership::Error::ShortPayload)
    );
    r.membership.slot_index = -1;
    assert_eq!(
        r.membership_handle(
            Input::Packet {
                header: h,
                payload: &op.0[36..36 + wire::JoinV3::SIZE]
            },
            &mut p
        ),
        Ok(true)
    );
    // Target mismatch returned before the shift. Matching leader reaches C UB.
    r.protocol.leader_identity = IdentityValue {
        address_len: 5,
        address: local().address,
    };
    assert_eq!(
        r.membership_handle(
            Input::Packet {
                header: h,
                payload: &op.0[36..36 + wire::JoinV3::SIZE]
            },
            &mut p
        ),
        Err(crate::protocol_membership::Error::UndefinedArithmetic)
    );
}

#[test]
fn membership_reset_preserves_kernel_ownership_and_rejects_missing_backend() {
    let mut r = Runtime::default();
    assert_eq!(
        r.membership_handle(Input::ScanWork, &mut crate::platform::NotIntegrated),
        Err(crate::protocol_membership::Error::NotIntegrated)
    );
    let mut p = Simulation {
        bytes: Vec::new(),
        now: 0,
        bind_mask: 255,
        bind_index: 0,
        sequence: true,
        logs: true,
    };
    run_op(&mut r, &mut p, seed(3, 1, 4, 0).arg(7, 1));
    run_op(&mut r, &mut p, Op::new(3, 0));
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
    r.talk_channel = 3;
    r.protocol.join_attempts = 8;
    r.protocol.last_sync_time = 123;
    let membership = r.membership;
    let pending = r.pending;
    let audio = r.audio_state.boundary;
    r.membership_handle(Input::ResetSession, &mut p).unwrap();
    assert_eq!(r.pending, pending);
    assert!(r.status_pending);
    assert_eq!(r.audio_state.boundary, audio);
    assert_eq!(r.talk_channel, 3);
    assert_eq!((r.auto_ticks, r.status_log_decim), (456, 789));
    assert_eq!(
        (r.protocol.join_attempts, r.protocol.last_sync_time),
        (8, 123)
    );
    assert_eq!(r.session.skip_count, 1234);
    assert_eq!(
        r.membership,
        Membership {
            peer_count: 0,
            ..membership
        }
    );
    assert_eq!(r.dedupe, Dedupe::default());
    assert_eq!(r.protocol.control_presence_dedupe, Dedupe::default());
    assert!(r.session.peers.iter().all(|x| *x == Peer::default()));
    assert!(r
        .session
        .relay_ring
        .iter()
        .chain(r.session.control_ring.iter())
        .all(|x| *x == RelayEntry::default()));
}
