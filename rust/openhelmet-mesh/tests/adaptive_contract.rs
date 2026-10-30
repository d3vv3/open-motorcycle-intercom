//! Domain: API-produced states only; valid backing storage, no aliases or forged
//! indices. Candidate-zero PREPARE is exercised, but never reconciled with src=0
//! (a nodes[-1] index in the former C). Integers are little endian.
use openhelmet_mesh::{adaptive::*, core::SlotMap};

fn p32(v: &mut Vec<u8>, n: u32) {
    v.extend(n.to_le_bytes());
}
fn p16(v: &mut Vec<u8>, n: u16) {
    v.extend(n.to_le_bytes());
}
fn identity(v: &mut Vec<u8>, x: Identity) {
    v.push(x.address_len);
    v.extend(x.address);
}
fn slots(v: &mut Vec<u8>, x: SlotMap) {
    v.push(x.slot_count);
    v.extend(x.slot_ids);
    v.push(x.active_speaker_count);
    v.extend(x.active_speaker_ids);
    v.extend(x.relay_masks);
}
fn handover(v: &mut Vec<u8>, x: Handover) {
    v.extend([x.old_leader_id, x.candidate_id]);
    p32(v, x.next_term);
    p32(v, x.switch_frame);
    v.push(x.members);
    slots(v, x.slot_map);
}
fn sync(v: &mut Vec<u8>, x: Sync) {
    identity(v, x.leader);
    v.extend([x.leader_id, x.member_count]);
    p32(v, x.term);
    p32(v, x.frame_counter);
    p16(v, x.phase_us);
    v.push(x.relay_depth);
}
fn header(v: &mut Vec<u8>, x: Header) {
    v.extend([
        x.version,
        x.packet_type,
        x.src_id,
        x.seq,
        x.ttl,
        x.flags,
        x.talk_channel,
    ]);
    p16(v, x.payload_len);
}
fn packet(v: &mut Vec<u8>, x: ForwardPacket) {
    header(v, x.header);
    v.extend(x.payload);
    p16(v, x.payload_len);
    p32(v, x.enqueued_ms);
}
fn state(v: &mut Vec<u8>, s: Snapshot) {
    v.extend([s.local_id, s.leader_id, s.members]);
    p32(v, s.term);
    p32(v, s.leader_since_ms);
    p32(v, s.better_since_ms);
    v.push(s.better_id);
    for n in s.nodes {
        v.extend([
            u8::from(n.present),
            u8::from(n.report_seen),
            u8::from(n.seq_seen),
        ]);
        identity(v, n.identity);
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
    handover(v, s.pending);
    v.push(s.ack_mask);
    p32(v, s.deadline_ms);
    p32(v, s.recovery_deadline_ms);
    p32(v, s.membership_revision);
    p32(v, s.publish_revision);
    v.push(u8::from(s.membership_revision_seen));
    for x in s.request_seq {
        p32(v, x);
    }
    for x in s.request_deadline_ms {
        p32(v, x);
    }
    v.extend(s.request_seq_seen.map(u8::from));
    v.extend(s.request_active.map(u8::from));
    v.extend([
        u8::from(s.prepared),
        u8::from(s.committed),
        u8::from(s.transition_done),
    ]);
    for k in s.cache {
        v.extend([k.packet_type, k.src_id, k.seq]);
        p32(v, k.term);
        p32(v, k.seen_ms);
        identity(v, k.origin);
        v.push(u8::from(k.used));
    }
    for q in s.queue {
        packet(v, q);
    }
    v.extend([s.queue_head, s.queue_count]);
}
struct Decoder<'a>(&'a [u8]);
impl Decoder<'_> {
    fn byte(&mut self) -> u8 {
        let x = self.0[0];
        self.0 = &self.0[1..];
        x
    }
    fn array<const N: usize>(&mut self) -> [u8; N] {
        let x = self.0[..N].try_into().unwrap();
        self.0 = &self.0[N..];
        x
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.array())
    }
    fn id(&mut self) -> Identity {
        Identity {
            address_len: self.byte(),
            address: self.array(),
        }
    }
    fn slots(&mut self) -> SlotMap {
        SlotMap {
            slot_count: self.byte(),
            slot_ids: self.array(),
            active_speaker_count: self.byte(),
            active_speaker_ids: self.array(),
            relay_masks: self.array(),
        }
    }
    fn proposal(&mut self) -> Handover {
        Handover {
            old_leader_id: self.byte(),
            candidate_id: self.byte(),
            next_term: self.u32(),
            switch_frame: self.u32(),
            members: self.byte(),
            slot_map: self.slots(),
        }
    }
    fn sync(&mut self) -> Sync {
        Sync {
            leader: self.id(),
            leader_id: self.byte(),
            member_count: self.byte(),
            term: self.u32(),
            frame_counter: self.u32(),
            phase_us: u16::from_le_bytes(self.array()),
            relay_depth: self.byte(),
        }
    }
    fn topology(&mut self) -> Topology {
        Topology {
            report_seq: self.u32(),
            origin: self.id(),
            direct_mask: self.byte(),
            rssi_dbm: self.array::<8>().map(|x| x as i8),
            quality: self.array(),
            leader_id: self.byte(),
            term: self.u32(),
            slot_map: self.slots(),
        }
    }
    fn membership(&mut self) -> Membership {
        let mut x = Membership {
            term: self.u32(),
            revision: self.u32(),
            leader_id: self.byte(),
            slot_map: self.slots(),
            ..Membership::default()
        };
        for id in &mut x.identities {
            *id = self.id();
        }
        x
    }
    fn request(&mut self) -> SpeakerRequest {
        SpeakerRequest {
            term: self.u32(),
            request_seq: self.u32(),
            frame_counter: self.u32(),
            active: self.byte(),
        }
    }
    fn ack(&mut self) -> HandoverAck {
        HandoverAck {
            old_leader_id: self.byte(),
            candidate_id: self.byte(),
            next_term: self.u32(),
            switch_frame: self.u32(),
            acknowledger_id: self.byte(),
        }
    }
}
#[derive(Clone)]
struct Input {
    op: u8,
    source: u8,
    aux: u8,
    flags: u8,
    a: u32,
    b: u32,
    c: u32,
    d: u32,
    header: Header,
    payload: [u8; 200],
    len: u16,
    origin: Identity,
}
impl Input {
    fn new(op: u8) -> Self {
        Self {
            op,
            source: 1,
            aux: 1,
            flags: 15,
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            header: Header::default(),
            payload: [0; 200],
            len: 0,
            origin: Identity::default(),
        }
    }
    fn data(mut self, v: Vec<u8>) -> Self {
        self.len = v.len() as u16;
        self.payload[..v.len()].copy_from_slice(&v);
        self
    }
    fn encode(&self) -> Vec<u8> {
        let mut v = vec![self.op, self.source, self.aux, self.flags];
        for n in [self.a, self.b, self.c, self.d] {
            p32(&mut v, n);
        }
        header(&mut v, self.header);
        p16(&mut v, self.len);
        v.extend(self.payload);
        identity(&mut v, self.origin);
        v.extend([0; 2]);
        assert_eq!(v.len(), 240);
        v
    }
    fn run(&self, storage: &mut Adaptive) -> Vec<u8> {
        let s = (self.flags & 1 != 0).then_some(storage);
        let present = self.flags & 2 != 0;
        let output = self.flags & 4 != 0;
        let origin = (self.flags & 8 != 0).then_some(&self.origin);
        let mut mask = 0xa5;
        let mut out = Decoder(&[0xa5; 25]).proposal();
        let mut stamped = Decoder(&self.payload).sync();
        let mut popped = ForwardPacket::default();
        let mut dec = Decoder(&self.payload);
        let (a, b, c, d) = (self.a, self.b, self.c, self.d);
        let source = self.source;
        let result: u32 = match self.op {
            0 => {
                init(s, source, self.aux, a, b as u8, c);
                0
            }
            1 => u32::from(identity_valid(present.then_some(&dec.id()))),
            2 => u32::from(cold_join(
                present.then_some(&dec.id()),
                source,
                origin,
                self.aux,
            )),
            3 => u32::from(set_member(s, source, origin)),
            4 => u32::from(update_members(s, source)),
            5 => u32::from(membership_valid(
                present.then_some(&dec.membership()),
                output.then_some(&mut mask),
            )),
            6 => u32::from(apply_membership(
                s,
                source,
                present.then_some(&dec.membership()),
            )),
            7 => u32::from(membership_removes_local(
                s.as_deref(),
                source,
                present.then_some(&dec.membership()),
            )),
            8 => next_membership_revision(s),
            9 => u32::from(control_authorized(s.as_deref(), self.aux, source)),
            10 => u32::from(report(
                s,
                source,
                present.then_some(&dec.topology()),
                d != 0,
                a,
            )),
            11 => u32::from(report_valid(
                s.as_deref(),
                source,
                present.then_some(&dec.topology()),
            )),
            12 => u32::from(speaker_request(
                s,
                source,
                present.then_some(&dec.request()),
                b,
                a,
            )),
            13 => u32::from(speaker_request_active(s.as_deref(), source, a)),
            14 => {
                expire(s, a);
                0
            }
            15 => u32::from(direct_graph(s, source, a)),
            16 => route(s, source, a, output.then_some(&mut mask)) as u32,
            17 => u32::from(candidate(s, a)),
            18 => u32::from(prepare(
                s,
                source,
                b,
                present.then_some(&dec.slots()),
                a,
                output.then_some(&mut out),
            )),
            19 => u32::from(receive_prepare(
                s,
                source,
                present.then_some(&dec.proposal()),
                b,
                a,
            )),
            20 => u32::from(ack(s, source, present.then_some(&dec.ack()), a)),
            21 => u32::from(commit(s, output.then_some(&mut out), a)),
            22 => u32::from(receive_commit(
                s,
                source,
                present.then_some(&dec.proposal()),
                a,
            )),
            23 => u32::from(cancel(s, source, present.then_some(&dec.proposal()))),
            24 => u32::from(tick(s, b, a)),
            25 => u32::from(reconcile_sync(s, source, present.then_some(&dec.sync()), a)),
            26 => u32::from(stamp_sync(
                present.then_some(&mut stamped),
                d != 0,
                b,
                c as u16,
                a,
            )),
            27 => u32::from(forward_enqueue(
                s,
                output.then_some(&self.header),
                present.then_some(&self.payload[..usize::from(self.len)]),
                origin,
                b,
                d != 0,
                a,
            )),
            28 => u32::from(forward_dequeue(
                s,
                output.then_some(&mut popped),
                b,
                c as u16,
                a,
            )),
            _ => panic!("unknown op"),
        };
        let mut v = Vec::new();
        p32(&mut v, result);
        v.push(mask);
        handover(&mut v, out);
        sync(&mut v, stamped);
        packet(&mut v, popped);
        v
    }
}
fn id(n: u8) -> Identity {
    Identity {
        address_len: 6,
        address: [n, 0, 0, 0, 0, 0],
    }
}
fn map() -> SlotMap {
    SlotMap {
        slot_count: 3,
        slot_ids: [1, 2, 3, 0, 0, 0, 0, 0],
        active_speaker_count: 1,
        active_speaker_ids: [3, 0],
        relay_masks: [2, 0],
    }
}
fn report_data(x: Topology) -> Vec<u8> {
    let mut v = Vec::new();
    p32(&mut v, x.report_seq);
    identity(&mut v, x.origin);
    v.push(x.direct_mask);
    v.extend(x.rssi_dbm.map(|x| x as u8));
    v.extend(x.quality);
    v.push(x.leader_id);
    p32(&mut v, x.term);
    slots(&mut v, x.slot_map);
    v
}
fn member_data(x: Membership) -> Vec<u8> {
    let mut v = Vec::new();
    p32(&mut v, x.term);
    p32(&mut v, x.revision);
    v.push(x.leader_id);
    slots(&mut v, x.slot_map);
    for i in x.identities {
        identity(&mut v, i);
    }
    v
}
fn proposal_data(x: Handover) -> Vec<u8> {
    let mut v = Vec::new();
    handover(&mut v, x);
    v
}
fn sync_data(x: Sync) -> Vec<u8> {
    let mut v = Vec::new();
    sync(&mut v, x);
    v
}
fn request_data(x: SpeakerRequest) -> Vec<u8> {
    let mut v = Vec::new();
    p32(&mut v, x.term);
    p32(&mut v, x.request_seq);
    p32(&mut v, x.frame_counter);
    v.push(x.active);
    v
}
fn ack_data(x: HandoverAck) -> Vec<u8> {
    let mut v = vec![x.old_leader_id, x.candidate_id];
    p32(&mut v, x.next_term);
    p32(&mut v, x.switch_frame);
    v.push(x.acknowledger_id);
    v
}
struct Corpus {
    inputs: Vec<u8>,
    expected: Vec<u8>,
    s: Adaptive,
    count: usize,
    covered: [bool; 29],
}
impl Corpus {
    fn push(&mut self, x: Input) -> Vec<u8> {
        self.inputs.extend_from_slice(&x.encode());
        let mut v = x.run(&mut self.s);
        state(&mut v, self.s.snapshot());
        self.expected.extend_from_slice(&v);
        self.count += 1;
        self.covered[usize::from(x.op)] = true;
        v
    }
    fn bound(&mut self, local: u8, members: u8, term: u32, now: u32) {
        let mut x = Input::new(0);
        x.source = local;
        x.a = term;
        x.b = u32::from(members);
        x.c = now;
        self.push(x);
        for n in 1..=8 {
            if members & (1 << (n - 1)) != 0 {
                let mut x = Input::new(3);
                x.source = n;
                x.origin = id(n);
                self.push(x);
            }
        }
    }
    fn row(&mut self, n: u8, mask: u8, seq: u32, now: u32) {
        let s = self.s.snapshot();
        let r = Topology {
            report_seq: seq,
            origin: s.nodes[usize::from(n - 1)].identity,
            direct_mask: mask,
            rssi_dbm: [-55; 8],
            quality: [90; 8],
            leader_id: s.leader_id,
            term: s.term,
            ..Topology::default()
        };
        let mut x = Input::new(10).data(report_data(r));
        x.source = n;
        x.a = now;
        x.d = u32::from(n == 3);
        self.push(x);
    }
    fn chain(&mut self, seq: u32, now: u32) {
        self.row(1, 2, seq, now);
        self.row(2, 5, seq, now);
        self.row(3, 2, seq, now);
    }
    fn timed(&mut self, op: u8, source: u8, now: u32) -> Vec<u8> {
        let mut x = Input::new(op);
        x.source = source;
        x.a = now;
        self.push(x)
    }
    fn forwarding(
        &mut self,
        t: u8,
        source: u8,
        seq: u8,
        now: u32,
        data: Vec<u8>,
        origin: Option<Identity>,
    ) -> Vec<u8> {
        let mut x = Input::new(27).data(data);
        x.header = Header {
            version: 5,
            packet_type: t,
            src_id: source,
            seq,
            ttl: 2,
            talk_channel: 1,
            payload_len: x.len,
            ..Header::default()
        };
        x.a = now;
        x.b = self.s.snapshot().term;
        x.d = 1;
        if let Some(id) = origin {
            x.origin = id;
        } else {
            x.flags &= !8;
        }
        self.push(x)
    }
}
fn accepted(v: &[u8]) -> bool {
    u32::from_le_bytes(v[..4].try_into().unwrap()) != 0
}

fn fixtures(c: &mut Corpus) {
    // 1: cold_join_policy
    for (a, ac, b, bc, yes) in [
        (32, 1, 16, 1, true),
        (16, 1, 32, 1, false),
        (32, 3, 16, 1, false),
        (16, 1, 32, 3, true),
        (48, 1, 32, 1, true),
        (32, 1, 32, 3, false),
        (32, 0, 16, 2, false),
        (32, 9, 16, 2, false),
        (16, 1, 32, 0, false),
        (16, 1, 32, 9, false),
    ] {
        let mut v = Vec::new();
        identity(&mut v, id(a));
        let mut x = Input::new(2).data(v);
        x.source = ac;
        x.aux = bc;
        x.origin = id(b);
        assert_eq!(accepted(&c.push(x)), yes);
    }
    // 2: authoritative_membership, plus 3: authoritative_local_eviction
    let mut m = Membership {
        term: 1,
        revision: 1,
        leader_id: 1,
        slot_map: map(),
        ..Membership::default()
    };
    for n in 1..=3 {
        m.identities[n - 1] = id(n as u8);
    }
    c.bound(1, 7, 1, 0);
    c.chain(1, 100);
    assert!(accepted(&c.push(Input::new(5).data(member_data(m)))));
    assert!(accepted(&c.push(Input::new(6).data(member_data(m)))));
    assert!(!accepted(&c.push(Input::new(6).data(member_data(m)))));
    for field in 0..8 {
        let mut bad = m;
        bad.revision = 2;
        match field {
            0 => bad.identities[2] = bad.identities[1],
            1 => bad.term = 2,
            2 => bad.leader_id = 2,
            3 => bad.slot_map.slot_ids[2] = 0,
            4 => bad.identities[3] = id(4),
            5 => bad.identities[0] = id(40),
            6 => bad.slot_map.slot_count = 4,
            _ => bad.slot_map.relay_masks[0] = 128,
        }
        assert!(!accepted(&c.push(Input::new(6).data(member_data(bad)))));
    }
    let mut x = Input::new(12).data(request_data(SpeakerRequest {
        term: 1,
        request_seq: 1,
        frame_counter: 5,
        active: 1,
    }));
    x.source = 3;
    x.a = 100;
    x.b = 5;
    assert!(accepted(&c.push(x)));
    assert!(accepted(&c.timed(13, 3, 100)));
    m.revision = 3;
    m.identities[2] = id(44);
    assert!(accepted(&c.push(Input::new(6).data(member_data(m)))));
    // A changed membership binding resets this request, not the current term.
    let reset = c.s.snapshot();
    assert_eq!(reset.term, 1);
    assert_eq!(reset.request_seq[2], 0);
    assert_eq!(reset.request_deadline_ms[2], 0);
    assert!(!reset.request_seq_seen[2]);
    assert!(!reset.request_active[2]);
    assert!(!accepted(&c.timed(13, 3, 101)));
    c.bound(2, 7, 1, 0);
    m.identities[2] = id(3);
    m.revision = 1;
    c.push(Input::new(6).data(member_data(m)));
    m.revision = 2;
    m.slot_map.slot_ids[1] = 0;
    m.identities[1] = Identity::default();
    assert!(!accepted(&c.push(Input::new(7).data(member_data(m)))));
    m.slot_map.relay_masks[0] = 0;
    assert!(accepted(&c.push(Input::new(7).data(member_data(m)))));
    assert!(!accepted(&c.push(Input::new(6).data(member_data(m)))));
    // 4: routes_and_election
    c.bound(1, 7, 1, 0);
    c.chain(u32::MAX - 1, 100);
    let v = c.timed(16, 1, 100);
    assert_eq!(&v[..5], &[1, 0, 0, 0, 2]);
    c.chain(0, 1100);
    c.timed(17, 1, 1100);
    c.chain(1, 10100);
    assert!(!accepted(&c.timed(17, 1, 10100)));
    c.chain(2, 13101);
    assert_eq!(c.timed(17, 1, 13101)[0], 2);
    let mut v = Vec::new();
    slots(&mut v, map());
    let mut x = Input::new(18).data(v);
    x.source = 2;
    x.b = 0xfffffff0;
    x.a = 13101;
    assert!(accepted(&c.push(x)));
    let p = c.s.snapshot().pending;
    assert!(!accepted(&c.timed(21, 1, 13102)));
    for n in 2..=3 {
        let a = HandoverAck {
            old_leader_id: 1,
            candidate_id: 2,
            next_term: 2,
            switch_frame: p.switch_frame,
            acknowledger_id: n,
        };
        let mut x = Input::new(20).data(ack_data(a));
        x.source = n;
        x.a = 13103;
        assert!(accepted(&c.push(x)));
        if n == 2 {
            assert_eq!(c.s.snapshot().ack_mask, 3);
            let before = c.s.snapshot();
            assert!(!accepted(&c.timed(21, 1, 13103)));
            assert_eq!(c.s.snapshot(), before);
        }
    }
    assert!(accepted(&c.timed(21, 1, 13104)));
    let mut x = Input::new(24);
    x.b = p.switch_frame;
    x.a = 16400;
    assert!(!accepted(&c.push(x)));
    let sy = Sync {
        leader: id(2),
        leader_id: 2,
        member_count: 3,
        term: 2,
        frame_counter: p.switch_frame,
        phase_us: 500,
        relay_depth: 0,
    };
    let mut x = Input::new(25).data(sync_data(sy));
    x.source = 2;
    x.a = 16420;
    assert!(accepted(&c.push(x)));
    // 5: partition_and_silence
    c.bound(1, 15, 1, 0);
    c.chain(1, 100);
    c.row(3, 10, 2, 200);
    c.row(4, 4, 1, 200);
    assert_eq!(c.timed(16, 1, 200)[0], 255);
    c.timed(17, 1, 14000);
    assert_eq!(c.s.snapshot().nodes[2].last_seen_ms, 0);
    // 6: pregrant_request_relay
    c.bound(2, 7, 1, 0);
    let request = SpeakerRequest {
        term: 1,
        request_seq: u32::MAX,
        frame_counter: 0,
        active: 1,
    };
    assert!(accepted(&c.forwarding(
        23,
        3,
        7,
        0,
        request_data(request),
        None
    )));
    c.timed(28, 1, 0);
    for seq in 0..8 {
        c.forwarding(14, 2, seq, 0, vec![0; 47], None);
    }
    assert!(accepted(&c.forwarding(
        23,
        3,
        8,
        1001,
        request_data(request),
        None
    )));
    let v = c.timed(28, 1, 0);
    assert_eq!(v[50 + 3], 1);
    c.bound(1, 7, 1, 0);
    let mut x = Input::new(12).data(request_data(request));
    x.source = 3;
    c.push(x);
    assert!(accepted(&c.timed(13, 3, 1500)));
    assert!(!accepted(&c.timed(13, 3, 1501)));
    // 7: report_loss_is_not_link_quality
    c.bound(1, 3, 1, 0);
    for i in 0..5 {
        c.row(1, 2, 1 + i, 100 + i * 1000);
        c.row(2, 1, 1 + i * 2, 100 + i * 1000);
    }
    assert_eq!(c.timed(15, 1, 4100)[0], 2);
    assert!(c.s.snapshot().nodes[1].loss_quality < 100);
    for i in 0..5 {
        let r = Topology {
            origin: id(2),
            leader_id: 1,
            term: 1,
            report_seq: 10 + i,
            direct_mask: 1,
            rssi_dbm: [-55; 8],
            quality: [0; 8],
            ..Topology::default()
        };
        let mut x = Input::new(10).data(report_data(r));
        x.source = 2;
        x.a = 4200 + i * 100;
        c.push(x);
        c.timed(15, 1, 4200 + i * 100);
    }
    assert_eq!(c.timed(15, 1, 4600)[0], 0);
    // 8: binding_and_replacement
    c.bound(1, 7, 1, 0);
    c.chain(1, 100);
    let mut x = Input::new(12).data(request_data(SpeakerRequest {
        term: 1,
        request_seq: 1,
        frame_counter: 5,
        active: 1,
    }));
    x.source = 2;
    x.a = 100;
    x.b = 5;
    assert!(accepted(&c.push(x)));
    assert!(accepted(&c.timed(13, 2, 100)));
    let mut x = Input::new(3);
    x.source = 2;
    x.origin = id(42);
    c.push(x.clone());
    let reset = c.s.snapshot();
    assert_eq!(reset.term, 1);
    assert_eq!(reset.request_seq[1], 0);
    assert_eq!(reset.request_deadline_ms[1], 0);
    assert!(!reset.request_seq_seen[1]);
    assert!(!reset.request_active[1]);
    assert!(!accepted(&c.timed(13, 2, 101)));
    x.flags &= !8;
    c.push(x);
    let mut x = Input::new(4);
    x.source = 7;
    c.push(x);
    assert!(!c.s.snapshot().nodes[1].present);
    // 9: follower_recovery, 10: lost_candidate_commit, 11: ack_relay_to_leader
    let p = Handover {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 160,
        members: 7,
        slot_map: map(),
    };
    for local in [3, 2] {
        c.bound(local, 7, 1, 0);
        c.chain(1, 100);
        let mut x = Input::new(19).data(proposal_data(p));
        x.a = 100;
        assert!(accepted(&c.push(x.clone())));
        x.a = 200;
        assert!(accepted(&c.push(x)));
        assert_eq!(c.s.snapshot().deadline_ms, 2500);
        let a = HandoverAck {
            old_leader_id: 1,
            candidate_id: 2,
            next_term: 2,
            switch_frame: 160,
            acknowledger_id: 3,
        };
        let mut x = Input::new(20).data(ack_data(a));
        x.source = 3;
        x.a = 200;
        assert!(accepted(&c.push(x)));
        assert_eq!(c.s.snapshot().ack_mask, 0);
        assert!(accepted(&c.forwarding(19, 3, 9, 200, ack_data(a), None)));
        c.timed(28, 1, 0);
        let request_ms = if local == 2 { 3499 } else { 3299 };
        let mut x = Input::new(12).data(request_data(SpeakerRequest {
            term: 1,
            request_seq: 1,
            frame_counter: 160,
            active: 1,
        }));
        x.source = 3;
        x.a = request_ms;
        x.b = 160;
        assert!(accepted(&c.push(x)));
        assert!(accepted(&c.timed(13, 3, request_ms)));
        if local == 2 {
            let mut x = Input::new(22).data(proposal_data(p));
            x.a = 3500;
            assert!(accepted(&c.push(x)));
            assert_eq!(c.s.snapshot().term, 1);
            assert!(accepted(&c.timed(13, 3, 3500)));
            let mut x = Input::new(24);
            x.b = 160;
            x.a = 3501;
            assert!(accepted(&c.push(x)));
        } else {
            let sy = Sync {
                frame_counter: 160,
                ..sy
            };
            let mut x = Input::new(25).data(sync_data(sy));
            x.source = 2;
            x.a = 3300;
            assert!(accepted(&c.push(x)));
        }
        let reset = c.s.snapshot();
        assert_eq!(reset.term, 2);
        assert_eq!(reset.request_seq, [0; 8]);
        assert_eq!(reset.request_deadline_ms, [0; 8]);
        assert_eq!(reset.request_seq_seen, [false; 8]);
        assert_eq!(reset.request_active, [false; 8]);
        assert!(!accepted(&c.timed(13, 3, request_ms + 2)));
        // Only the term differs: sender remains bound, sequence is fresh and age is zero.
        assert!(reset.nodes[2].present);
        assert!(identity_valid(Some(&reset.nodes[2].identity)));
        let mut x = Input::new(12).data(request_data(SpeakerRequest {
            term: 1,
            request_seq: 2,
            frame_counter: 161,
            active: 1,
        }));
        x.source = 3;
        x.a = request_ms + 3;
        x.b = 161;
        let before = c.s.snapshot();
        assert!(!accepted(&c.push(x.clone())));
        assert_eq!(c.s.snapshot(), before);
        x.payload[..4].copy_from_slice(&2u32.to_le_bytes());
        assert!(accepted(&c.push(x)));
        assert!(accepted(&c.timed(13, 3, request_ms + 3)));
    }
    // 12: local_control_termination
    for local in [1, 2] {
        c.bound(local, 7, 1, 0);
        for t in [15, 14, 23, 19] {
            let data = match t {
                15 => {
                    let mut v = Vec::new();
                    identity(&mut v, id(9));
                    v.push(0);
                    identity(&mut v, id(1));
                    v
                }
                14 => vec![0; 47],
                23 => request_data(request),
                _ => vec![0; 11],
            };
            assert_eq!(
                accepted(&c.forwarding(t, if t == 15 { 0 } else { 3 }, t, 100, data, Some(id(9)))),
                local == 2
            );
        }
    }
    // 13: forwarding_and_clock
    c.bound(2, 3, 1, 0);
    let sy = Sync {
        leader: id(1),
        leader_id: 1,
        member_count: 2,
        term: 1,
        ..Sync::default()
    };
    for (age, yes) in [(599, true), (600, true), (601, false)] {
        let mut x = Input::new(26).data(sync_data(sy));
        x.d = 1;
        x.a = age;
        x.c = 19999;
        assert_eq!(accepted(&c.push(x)), yes);
    }
    for seq in 0..40 {
        assert!(accepted(&c.forwarding(
            17,
            1,
            seq,
            1100,
            sync_data(sy),
            None
        )));
        assert!(accepted(&c.timed(28, 1, 0)));
    }
    assert!(!accepted(&c.forwarding(
        17,
        1,
        39,
        1101,
        sync_data(sy),
        None
    )));
    assert!(accepted(&c.forwarding(
        17,
        1,
        40,
        1101,
        sync_data(sy),
        None
    )));
    assert!(!accepted(&c.timed(28, 1, 601)));
}

fn edges(c: &mut Corpus) {
    // Equal relay solutions retain numerically first mask; one relay beats two.
    c.bound(1, 15, 1, 0);
    for (n, mask) in [(1, 6), (2, 9), (3, 9), (4, 6)] {
        c.row(n, mask, 1, 0);
    }
    assert_eq!(&c.timed(16, 1, 0)[..5], &[1, 0, 0, 0, 2]);
    // Strict >7 improvement, and inclusive 10000/3000 hold boundaries.
    for (rssi, selected) in [(-60, false), (-59, true)] {
        c.bound(1, 7, 1, 0);
        for (seq, now) in [(1, 9999), (2, 10000), (3, 12999), (4, 13000)] {
            for (n, mask) in [(1, 2), (2, 5), (3, 2)] {
                let mut values = [-55; 8];
                if n == 2 {
                    values[2] = rssi;
                }
                if n == 3 {
                    values[1] = rssi;
                }
                let r = Topology {
                    origin: id(n),
                    leader_id: 1,
                    term: 1,
                    report_seq: seq,
                    direct_mask: mask,
                    rssi_dbm: values,
                    quality: [90; 8],
                    ..Topology::default()
                };
                let mut x = Input::new(10).data(report_data(r));
                x.source = n;
                x.a = now;
                c.push(x);
            }
            assert_eq!(accepted(&c.timed(17, 1, now)), selected && now == 13000);
        }
    }
    // Explicit nullable branches for every API, including missing output/model.
    for op in 0..29 {
        for flags in 0..16 {
            let mut x = Input::new(op);
            x.flags = flags;
            c.push(x);
        }
    }
    for term in [0, 1, 0x7fff_ffff, 0x8000_0000, u32::MAX - 1, u32::MAX] {
        c.bound(1, 7, term, u32::MAX - 1000);
        c.chain(u32::MAX, u32::MAX - 1000);
        for delta in [0, 1, 0x7fff_ffff, 0x8000_0000, 0x8000_0001, u32::MAX] {
            let r = Topology {
                origin: id(2),
                leader_id: 1,
                term,
                report_seq: u32::MAX.wrapping_add(delta),
                direct_mask: 5,
                rssi_dbm: [-57; 8],
                quality: [65; 8],
                ..Topology::default()
            };
            let mut x = Input::new(10).data(report_data(r));
            x.source = 2;
            x.a = u32::MAX - 999;
            c.push(x);
            let q = SpeakerRequest {
                term,
                request_seq: delta,
                frame_counter: u32::MAX,
                active: 1,
            };
            let mut x = Input::new(12).data(request_data(q));
            x.source = 2;
            x.b = 49;
            x.a = u32::MAX - 999;
            c.push(x);
        }
        for age in [2999, 3000, 3001, 0x8000_0000] {
            c.timed(15, 1, (u32::MAX - 1000).wrapping_add(age));
        }
        let mut v = Vec::new();
        slots(&mut v, map());
        let mut x = Input::new(18).data(v);
        x.source = 0;
        x.a = u32::MAX - 1000;
        x.b = u32::MAX - 159;
        c.push(x);
        // No reconcile with sender zero: undefined C index even though PREPARE itself is defined.
        let s = c.s.snapshot();
        if s.prepared {
            let a = HandoverAck {
                old_leader_id: 1,
                candidate_id: s.pending.candidate_id,
                next_term: s.pending.next_term,
                switch_frame: s.pending.switch_frame,
                acknowledger_id: 2,
            };
            let mut x = Input::new(20).data(ack_data(a));
            x.source = 2;
            x.a = s.deadline_ms;
            assert!(accepted(&c.push(x)));
            c.timed(24, 1, s.deadline_ms);
            assert!(!c.s.snapshot().prepared);
        }
    }
    // Validator separation: holes and leader relocation accepted, tails rejected.
    for field in 0..7 {
        let mut m = Membership {
            term: 1,
            revision: 1,
            leader_id: 1,
            slot_map: SlotMap {
                slot_count: 3,
                slot_ids: [3, 0, 1, 0, 0, 0, 0, 0],
                ..SlotMap::default()
            },
            ..Membership::default()
        };
        m.identities[0] = id(1);
        m.identities[2] = id(3);
        match field {
            0 => {}
            1 => m.slot_map.slot_count = 4,
            2 => m.slot_map.active_speaker_ids[1] = 1,
            3 => m.slot_map.relay_masks[1] = 1,
            4 => m.identities[1].address[5] = 1,
            5 => m.slot_map.slot_ids[3] = 9,
            _ => m.identities[2] = id(1),
        }
        assert_eq!(
            accepted(&c.push(Input::new(5).data(member_data(m)))),
            field == 0
        );
    }
    for n in 0..=255 {
        let mut v = Vec::new();
        let mut identity_value = id(1);
        identity_value.address_len = n;
        identity(&mut v, identity_value);
        c.push(Input::new(1).data(v));
    }
    for value in [
        Identity::default(),
        Identity {
            address_len: 6,
            address: [255; 6],
        },
        Identity {
            address_len: 1,
            address: [1, 1, 0, 0, 0, 0],
        },
    ] {
        let mut v = Vec::new();
        identity(&mut v, value);
        assert!(!accepted(&c.push(Input::new(1).data(v))));
    }
    c.bound(1, 7, 1, 0);
    for field in 0..8 {
        let mut r = Topology {
            origin: id(2),
            leader_id: 1,
            term: 1,
            report_seq: 1,
            direct_mask: 5,
            rssi_dbm: [-55; 8],
            quality: [90; 8],
            ..Topology::default()
        };
        match field {
            0 => r.origin.address_len = 7,
            1 => r.origin.address[0] = 4,
            2 => r.quality[0] = 101,
            3 => r.rssi_dbm[0] = -128,
            4 => r.rssi_dbm[0] = 1,
            5 => r.direct_mask |= 2,
            6 => r.direct_mask |= 8,
            _ => {
                r.rssi_dbm[1] = -128;
                r.quality[1] = 255;
            }
        }
        let mut x = Input::new(11).data(report_data(r));
        x.source = 2;
        assert_eq!(accepted(&c.push(x)), field == 7);
    }
    // Separate entry thresholds and signed truncation, independent of beacon loss.
    for q in [44, 45, 46, 64, 65, 66] {
        c.bound(1, 3, 1, 0);
        for n in 1..=2 {
            let r = Topology {
                origin: id(n),
                leader_id: 1,
                term: 1,
                report_seq: 1,
                direct_mask: if n == 1 { 2 } else { 1 },
                rssi_dbm: [-57; 8],
                quality: [q; 8],
                ..Topology::default()
            };
            let mut x = Input::new(10).data(report_data(r));
            x.source = n;
            c.push(x);
        }
        assert_eq!(c.timed(15, 1, 0)[0], if q >= 65 { 2 } else { 0 });
        c.row(1, 2, 2, 1);
        c.row(2, 1, 2, 1);
        c.timed(15, 1, 1);
        let r = Topology {
            origin: id(2),
            leader_id: 1,
            term: 1,
            report_seq: 3,
            direct_mask: 1,
            rssi_dbm: [-54; 8],
            quality: [q; 8],
            ..Topology::default()
        };
        let mut x = Input::new(10).data(report_data(r));
        x.source = 2;
        x.a = 2;
        c.push(x);
        c.timed(15, 1, 2);
    }
    // Admit at 65 first, then reach each exit boundary through filtered reports.
    // Samples target-2 converge exactly to target despite integer rounding.
    for target in [44, 45, 46] {
        c.bound(1, 3, 1, 0);
        for n in 1..=2 {
            let r = Topology {
                origin: id(n),
                leader_id: 1,
                term: 1,
                report_seq: 1,
                direct_mask: if n == 1 { 2 } else { 1 },
                rssi_dbm: [-55; 8],
                quality: [65; 8],
                ..Topology::default()
            };
            let mut x = Input::new(10).data(report_data(r));
            x.source = n;
            assert!(accepted(&c.push(x)));
        }
        assert_eq!(c.timed(15, 1, 0)[0], 2);
        assert_eq!(c.timed(15, 2, 0)[0], 1);
        let mut boundary_ms = 0;
        for seq in 2..=64 {
            for n in 1..=2 {
                let r = Topology {
                    origin: id(n),
                    leader_id: 1,
                    term: 1,
                    report_seq: seq,
                    direct_mask: if n == 1 { 2 } else { 1 },
                    rssi_dbm: [-55; 8],
                    quality: [target - 2; 8],
                    ..Topology::default()
                };
                let mut x = Input::new(10).data(report_data(r));
                x.source = n;
                x.a = seq;
                assert!(accepted(&c.push(x)));
            }
            let s = c.s.snapshot();
            if s.nodes[0].quality[1] == target && s.nodes[1].quality[0] == target {
                boundary_ms = seq;
                break;
            }
            assert!(s.nodes[0].quality[1] > target);
            assert!(s.nodes[1].quality[0] > target);
            assert_eq!(c.timed(15, 1, seq)[0], 2);
            assert_eq!(c.timed(15, 2, seq)[0], 1);
        }
        assert_ne!(boundary_ms, 0, "filtered quality must reach {target}");
        let s = c.s.snapshot();
        assert_eq!(s.nodes[0].quality[1], target);
        assert_eq!(s.nodes[1].quality[0], target);
        assert_eq!(s.edge_live[0] & 2, 2);
        assert_eq!(s.edge_live[1] & 1, 1);
        assert_eq!(
            c.timed(15, 1, boundary_ms)[0],
            if target >= 45 { 2 } else { 0 }
        );
        assert_eq!(
            c.timed(15, 2, boundary_ms)[0],
            if target >= 45 { 1 } else { 0 }
        );
        let result = c.timed(16, 1, boundary_ms);
        assert_eq!(
            u32::from_le_bytes(result[..4].try_into().unwrap()),
            if target >= 45 { 0 } else { u32::MAX }
        );
        assert_eq!(result[4], 0);
    }
    // First report after expiry discards smoothing, but not sequence ordering.
    c.bound(1, 7, 1, 0);
    c.chain(10, 0);
    c.timed(14, 1, 3001);
    c.chain(10, 3002);
    assert!(!c.s.snapshot().nodes[1].report_seen);
    c.chain(11, 3003);
    // Serial membership revisions wrap; exact half-range is not newer.
    c.bound(1, 7, 1, 0);
    for rev in [u32::MAX - 1, u32::MAX, 0, 0x8000_0000, 0x7fff_ffff] {
        let mut m = Membership {
            term: 1,
            revision: rev,
            leader_id: 1,
            slot_map: map(),
            ..Membership::default()
        };
        for n in 1..=3 {
            m.identities[n - 1] = id(n as u8);
        }
        c.push(Input::new(6).data(member_data(m)));
    }
    // Isolated fresh sequences: ordinary/wrapping future frames fail, older wrap succeeds.
    for (current_frame, request_frame, expected) in
        [(100, 101, false), (u32::MAX, 0, false), (0, u32::MAX, true)]
    {
        c.bound(2, 7, 1, 100);
        let before = c.s.snapshot();
        assert!(before.nodes[2].present);
        assert!(identity_valid(Some(&before.nodes[2].identity)));
        assert!(!before.request_seq_seen[2]);
        let mut x = Input::new(12).data(request_data(SpeakerRequest {
            term: 1,
            request_seq: 1,
            frame_counter: request_frame,
            active: 1,
        }));
        x.source = 3;
        x.a = 100;
        x.b = current_frame;
        assert_eq!(accepted(&c.push(x)), expected);
        if expected {
            assert_eq!(c.s.snapshot().request_seq[2], 1);
            assert!(accepted(&c.timed(13, 3, 100)));
        } else {
            assert_eq!(c.s.snapshot(), before);
        }
    }
    // Requests: frame age 50 included and 51 rejected; expiry is not a reset.
    c.bound(2, 7, 1, u32::MAX - 100);
    for (seq, frame, age, active) in [
        (u32::MAX, u32::MAX, 50, 1),
        (0, u32::MAX, 50, 1),
        (1, 0, 51, 1),
        (1, 1, 0, 2),
        (1, 0, 0, 0),
        (2, 0, 0, 1),
        (0x8000_0002, 0, 0, 1),
    ] {
        let q = SpeakerRequest {
            term: 1,
            request_seq: seq,
            frame_counter: frame,
            active,
        };
        let mut x = Input::new(12).data(request_data(q));
        x.source = 3;
        x.b = frame.wrapping_add(age);
        x.a = u32::MAX - 100;
        c.push(x);
    }
    assert!(accepted(&c.timed(13, 3, 1399)));
    assert!(!accepted(&c.timed(13, 3, 1400)));
    // Matching duplicates bypass both stale graph and frame lead, without deadline refresh.
    let p = Handover {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 160,
        members: 7,
        slot_map: map(),
    };
    c.bound(3, 7, 1, 0);
    c.chain(1, 0);
    let mut x = Input::new(19).data(proposal_data(p));
    assert!(accepted(&c.push(x.clone())));
    x.a = 3001;
    x.b = 200;
    assert!(accepted(&c.push(x)));
    assert_eq!(c.s.snapshot().deadline_ms, 2400);
    // Packed equality compares every proposal byte, including unused grants and holes.
    for i in 0..25 {
        let mut x = Input::new(22).data(proposal_data(p));
        x.payload[i] ^= 1;
        x.a = 1;
        assert!(!accepted(&c.push(x)));
    }
    for i in 0..25 {
        let mut x = Input::new(23).data(proposal_data(p));
        x.payload[i] ^= 1;
        assert!(!accepted(&c.push(x)));
    }
    let mut x = Input::new(23).data(proposal_data(p));
    assert!(accepted(&c.push(x.clone())));
    assert!(!accepted(&c.push(x.clone())));
    // Failed PREPARE can expire graph rows; no transaction/rollback is introduced.
    c.bound(3, 7, 1, 0);
    c.chain(1, 0);
    x = Input::new(19).data(proposal_data(p));
    x.a = 3001;
    assert!(!accepted(&c.push(x)));
    assert!(!c.s.snapshot().nodes[1].report_seen);
    // Candidate verifies universal graph; followers only require their own reverse edge.
    c.bound(3, 7, 1, 0);
    c.row(2, 5, 1, 0);
    c.row(3, 2, 1, 0);
    assert!(accepted(&c.push(Input::new(19).data(proposal_data(p)))));
    c.bound(2, 7, 1, 0);
    c.row(2, 5, 1, 0);
    c.row(3, 2, 1, 0);
    assert!(!accepted(&c.push(Input::new(19).data(proposal_data(p)))));
    // Exact min/max frame lead and deadline/recovery boundaries, including wrap.
    for lead in [19, 20, 160, 161] {
        c.bound(3, 7, 1, 0);
        c.chain(1, 0);
        let proposal = Handover {
            switch_frame: u32::MAX.wrapping_add(lead),
            ..p
        };
        let mut x = Input::new(19).data(proposal_data(proposal));
        x.b = u32::MAX;
        assert_eq!(accepted(&c.push(x)), (20..=160).contains(&lead));
    }
    for delta in [2399, 2400, 2401, 5199, 5200, 5201] {
        c.bound(2, 7, 1, 0);
        c.chain(1, 0);
        c.push(Input::new(19).data(proposal_data(p)));
        let mut x = Input::new(22).data(proposal_data(p));
        x.a = delta;
        c.push(x);
        let mut x = Input::new(24);
        x.a = delta;
        x.b = 160;
        c.push(x);
    }
    // FIFO and replacement: strictly lower priority AND stale >1000 required.
    for (queued_t, new_t, age, yes) in [
        (14, 23, 1000, false),
        (14, 23, 1001, true),
        (23, 23, 1001, false),
        (23, 6, 1001, true),
        (6, 23, 1001, false),
        (6, 6, 1001, false),
    ] {
        c.bound(2, 7, 1, 0);
        let old = match queued_t {
            14 => vec![0; 47],
            23 => request_data(SpeakerRequest {
                term: 1,
                active: 1,
                ..SpeakerRequest::default()
            }),
            _ => {
                let mut v = Vec::new();
                slots(&mut v, map());
                v
            }
        };
        for seq in 0..8 {
            assert!(accepted(&c.forwarding(
                queued_t,
                3,
                seq,
                0,
                old.clone(),
                None
            )));
        }
        let new = if new_t == 23 {
            request_data(SpeakerRequest {
                term: 1,
                active: 1,
                ..SpeakerRequest::default()
            })
        } else {
            let mut v = Vec::new();
            slots(&mut v, map());
            v
        };
        assert_eq!(accepted(&c.forwarding(new_t, 3, 9, age, new, None)), yes);
        for _ in 0..8 {
            c.timed(28, 1, 0);
        }
    }
    // Source-zero JOIN cache identity, exact cache freshness and local target stop.
    c.bound(2, 7, 1, 0);
    for (origin, now, yes) in [
        (id(9), 0, true),
        (id(9), 1000, false),
        (id(10), 1000, true),
        (id(9), 1001, true),
    ] {
        let mut v = Vec::new();
        identity(&mut v, origin);
        v.push(0);
        identity(&mut v, id(1));
        assert_eq!(accepted(&c.forwarding(15, 0, 1, now, v, Some(origin))), yes);
        c.timed(28, 1, 0);
    }
    let mut ack = Vec::new();
    identity(&mut ack, id(2));
    ack.extend([0, 2, 1, 1]);
    p32(&mut ack, 1);
    assert!(!accepted(&c.forwarding(16, 1, 1, 0, ack.clone(), None)));
    ack[1] = 3;
    assert!(accepted(&c.forwarding(16, 1, 1, 0, ack, None)));
}

fn random(c: &mut Corpus) {
    let mut rng = 0x92ab_7731u32;
    for epoch in 0..24 {
        c.bound(if epoch % 2 == 0 { 1 } else { 3 }, 7, 1, 0);
        let mut now = if epoch % 3 == 0 { u32::MAX - 20000 } else { 0 };
        for step in 0..512 {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            now = now.wrapping_add(rng % 101);
            let source = (rng % 10) as u8;
            let op = (rng.rotate_left(11) % 29) as u8;
            let mut x = Input::new(op);
            x.source = source;
            x.a = now;
            x.b = step;
            x.c = rng % 20002;
            x.d = rng & 1;
            let s = c.s.snapshot();
            let good_source = source.clamp(1, 8);
            let identity = s.nodes[usize::from(good_source - 1)].identity;
            match op {
                0 => {
                    x.source = if epoch % 2 == 0 { 1 } else { 3 };
                    x.aux = 1;
                    x.a = 1;
                    x.b = 7;
                    x.c = now;
                }
                1 | 2 => {
                    let mut v = Vec::new();
                    let mut value = identity;
                    if rng & 4 != 0 {
                        value.address_len = (rng >> 24) as u8;
                    }
                    crate::identity(&mut v, value);
                    x = x.data(v);
                    x.origin = id(9);
                }
                3 => {
                    x.origin = if rng & 4 == 0 {
                        identity
                    } else {
                        id(20 + source)
                    };
                    if rng & 8 == 0 {
                        x.flags &= !8;
                    }
                }
                4 => {
                    x.source = (rng >> 16) as u8;
                }
                5..=7 => {
                    let mut m = Membership {
                        term: s.term,
                        revision: rng,
                        leader_id: s.leader_id,
                        slot_map: map(),
                        ..Membership::default()
                    };
                    for n in 1..=3 {
                        m.identities[n - 1] = id(n as u8);
                    }
                    if rng & 8 != 0 {
                        m.slot_map.slot_ids[(rng % 8) as usize] = (rng >> 24) as u8;
                    }
                    x = x.data(member_data(m));
                }
                9 => {
                    x.aux = (rng % 25) as u8;
                }
                10 | 11 => {
                    let mut r = Topology {
                        origin: identity,
                        leader_id: s.leader_id,
                        term: s.term,
                        report_seq: rng,
                        direct_mask: (rng >> 8) as u8 & 7 & !(1 << (good_source - 1)),
                        rssi_dbm: [-57; 8],
                        quality: [(rng % 102) as u8; 8],
                        ..Topology::default()
                    };
                    if rng & 8 == 0 {
                        r.origin = id(90);
                    }
                    x = x.data(report_data(r));
                }
                12 => {
                    x = x.data(request_data(SpeakerRequest {
                        term: s.term,
                        request_seq: rng,
                        frame_counter: step.wrapping_sub(rng % 53),
                        active: (rng % 3) as u8,
                    }));
                }
                18 => {
                    x.source = if rng & 8 == 0 { 0 } else { 2 };
                    let mut v = Vec::new();
                    slots(&mut v, map());
                    x = x.data(v);
                }
                19 | 22 | 23 => {
                    let p = if s.prepared {
                        s.pending
                    } else {
                        Handover {
                            old_leader_id: 1,
                            candidate_id: 2,
                            next_term: 2,
                            switch_frame: step.wrapping_add(20 + rng % 141),
                            members: 7,
                            slot_map: map(),
                        }
                    };
                    x.source = 1;
                    x = x.data(proposal_data(p));
                }
                20 => {
                    x = x.data(ack_data(HandoverAck {
                        old_leader_id: s.leader_id,
                        candidate_id: s.pending.candidate_id,
                        next_term: s.pending.next_term,
                        switch_frame: s.pending.switch_frame,
                        acknowledger_id: source,
                    }));
                }
                25 | 26 => {
                    x.source = 2;
                    x = x.data(sync_data(Sync {
                        leader: id(2),
                        leader_id: 2,
                        member_count: (rng % 10) as u8,
                        term: 2,
                        frame_counter: step,
                        phase_us: (rng % 20002) as u16,
                        relay_depth: 0,
                    }));
                }
                27 => {
                    let t = [6, 9, 10, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23][(rng % 13) as usize];
                    let data = match t {
                        6 => {
                            let mut v = Vec::new();
                            slots(&mut v, map());
                            v
                        }
                        9 => vec![0; 5],
                        10 => vec![0; 3],
                        14 => vec![0; 47],
                        15 | 16 => vec![0; 15],
                        17 => sync_data(Sync {
                            leader: id(1),
                            leader_id: 1,
                            member_count: 3,
                            term: 1,
                            ..Sync::default()
                        }),
                        19 => vec![0; 11],
                        22 => vec![0; 79],
                        23 => request_data(SpeakerRequest {
                            term: s.term,
                            active: 1,
                            ..SpeakerRequest::default()
                        }),
                        _ => proposal_data(s.pending),
                    };
                    x = x.data(data);
                    x.header = Header {
                        version: 5,
                        packet_type: t,
                        src_id: source,
                        seq: (rng >> 24) as u8,
                        ttl: 2,
                        payload_len: x.len,
                        ..Header::default()
                    };
                    x.b = s.term;
                    x.d = 1;
                    x.flags &= !8;
                }
                _ => {}
            }
            // Private state is valid or zero-init; receive_prepare on zero-init short-circuits.
            // Never supply sender=0 to reconcile a candidate-zero proposal.
            c.push(x);
        }
    }
}

#[test]
fn all_29_apis_corpus_requirements_and_coverage() {
    let mut corpus = Corpus {
        inputs: Vec::new(),
        expected: Vec::new(),
        s: Adaptive::default(),
        count: 0,
        covered: [false; 29],
    };
    fixtures(&mut corpus);
    edges(&mut corpus);
    random(&mut corpus);
    assert!(corpus.covered.into_iter().all(|x| x));
    assert!(corpus.count < 20000);
    assert!(!corpus.expected.is_empty());
    eprintln!("adaptive corpus: {} requests", corpus.count);
}
