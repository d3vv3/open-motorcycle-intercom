use openhelmet_mesh::core::SlotMap;
use openhelmet_mesh::membership::*;

const STATES: [State; 4] = [State::Idle, State::Scanning, State::Joining, State::Active];
const ROLES: [Role; 3] = [Role::None, Role::Coordinator, Role::Participant];
const TERMS: [u32; 7] = [0, 1, 2, 0x7fff_ffff, 0x8000_0000, u32::MAX - 1, u32::MAX];

fn snapshot(state: State, role: Role) -> Snapshot {
    Snapshot {
        state,
        role,
        node_id: if role == Role::Coordinator { 1 } else { 3 },
        slot_index: if role == Role::Coordinator { 0 } else { 2 },
        coordinator_id: 1,
        term: 0,
        peer_count: 3,
        participant_membership_known: true,
        local_address: [0x10, 0x20, 0x30, 0x40, 0x50, 0xa5],
        address_len: 5,
    }
}

fn event(data: EventData) -> Event {
    Event {
        sender_id: 1,
        payload_valid: true,
        data,
    }
}

fn sync() -> Sync {
    Sync {
        coordinator_address: [0x20, 0x20, 0x30, 0x40, 0x50, 0x5a],
        address_len: 5,
        term: 0,
    }
}

fn ack() -> JoinAck {
    JoinAck {
        assigned_id: 3,
        slot_index: 2,
        coordinator_id: 1,
        target_address: [0x10, 0x20, 0x30, 0x40, 0x50, 0x5a],
        address_len: 5,
    }
}

fn map() -> SlotMap {
    SlotMap {
        slot_count: 3,
        slot_ids: [1, 3, 5, 0, 0, 0, 0, 0],
        active_speaker_count: 0,
        active_speaker_ids: [255, 255],
        relay_masks: [255, 255],
    }
}

fn leave() -> Leave {
    Leave {
        identity: LeaveIdentity::Address,
        sender_address: [1, 2, 3, 4, 5, 0xa5],
        address_len: 5,
        peer: Peer {
            present: true,
            active: true,
            announced: true,
            node_id: 4,
            address: [1, 2, 3, 4, 5, 0x5a],
            address_len: 5,
        },
    }
}

fn number(dst: &mut Vec<u8>, n: u32) {
    dst.extend_from_slice(&n.to_le_bytes());
}

fn serialize_snapshot(dst: &mut Vec<u8>, s: &Snapshot) {
    dst.extend_from_slice(&[
        s.state as u8,
        s.role as u8,
        s.node_id,
        s.slot_index as u8,
        s.coordinator_id,
    ]);
    number(dst, s.term);
    dst.extend_from_slice(&[s.peer_count, u8::from(s.participant_membership_known)]);
    dst.extend_from_slice(&s.local_address);
    dst.push(s.address_len);
}

fn serialize_event(dst: &mut Vec<u8>, e: &Event) {
    let tag = match e.data {
        EventData::Sync(_) => 0,
        EventData::JoinAck(_) => 1,
        EventData::SlotMap(_) => 2,
        EventData::Leave(_) => 3,
        EventData::Unknown(n) => n,
    };
    number(dst, tag);
    dst.extend_from_slice(&[e.sender_id, u8::from(e.payload_valid)]);
    match e.data {
        EventData::Sync(s) => {
            dst.extend_from_slice(&s.coordinator_address);
            dst.push(s.address_len);
            number(dst, s.term);
        }
        EventData::JoinAck(a) => {
            dst.extend_from_slice(&[a.assigned_id, a.slot_index, a.coordinator_id]);
            dst.extend_from_slice(&a.target_address);
            dst.push(a.address_len);
        }
        EventData::SlotMap(m) => {
            dst.push(m.slot_count);
            dst.extend_from_slice(&m.slot_ids);
            dst.push(m.active_speaker_count);
            dst.extend_from_slice(&m.active_speaker_ids);
            dst.extend_from_slice(&m.relay_masks);
        }
        EventData::Leave(l) => {
            number(
                dst,
                match l.identity {
                    LeaveIdentity::Invalid => 0,
                    LeaveIdentity::Legacy => 1,
                    LeaveIdentity::Address => 2,
                    LeaveIdentity::Unknown(n) => n,
                },
            );
            dst.extend_from_slice(&l.sender_address);
            dst.push(l.address_len);
            dst.extend_from_slice(&[
                u8::from(l.peer.present),
                u8::from(l.peer.active),
                u8::from(l.peer.announced),
                l.peer.node_id,
            ]);
            dst.extend_from_slice(&l.peer.address);
            dst.push(l.peer.address_len);
        }
        EventData::Unknown(_) => {}
    }
}

#[derive(Default)]
struct Corpus {
    input: Vec<u8>,
    expected: Vec<u8>,
    cases: Vec<(&'static str, usize)>,
    current: Snapshot,
}

impl Corpus {
    fn reset(&mut self, s: Snapshot) {
        self.input.push(0);
        serialize_snapshot(&mut self.input, &s);
        self.current = s;
    }
    fn step(
        &mut self,
        name: &'static str,
        e: Event,
        current_present: bool,
        event_present: bool,
    ) -> MembershipResult {
        self.input
            .extend_from_slice(&[1, u8::from(current_present), u8::from(event_present)]);
        serialize_event(&mut self.input, &e);
        let r = reduce(
            current_present.then_some(&self.current),
            event_present.then_some(&e),
        );
        if !current_present || !event_present {
            assert_eq!(r, MembershipResult::default(), "{name}: absent input");
        } else {
            assert_eq!(r.effects & !7, 0, "{name}: unknown effects");
            if !e.payload_valid || matches!(e.data, EventData::Unknown(_)) {
                assert_eq!(
                    r,
                    MembershipResult {
                        next: self.current,
                        ..MembershipResult::default()
                    },
                    "{name}: rejected payload must have no effects"
                );
            }
            if let EventData::Sync(sync) = e.data {
                let remote = sync
                    .coordinator_address
                    .get(..usize::from(sync.address_len));
                let local = self
                    .current
                    .local_address
                    .get(..usize::from(self.current.address_len));
                let valid = e.payload_valid
                    && (1..=8).contains(&e.sender_id)
                    && (1..=6).contains(&sync.address_len)
                    && sync.address_len == self.current.address_len
                    && remote.is_some_and(|bytes| {
                        bytes.iter().any(|b| *b != 0) && bytes.iter().any(|b| *b != 255)
                    })
                    && remote != local;
                let expected = if !valid {
                    Action::Ignore
                } else {
                    match (self.current.state, self.current.role) {
                        (State::Scanning, _) => Action::DiscoverCoordinator,
                        (State::Active, Role::Participant)
                            if e.sender_id == self.current.coordinator_id
                                && (self.current.term == 0 || self.current.term == sync.term) =>
                        {
                            Action::AcceptSync
                        }
                        (State::Active, Role::Coordinator)
                            if self.current.term == 0 && sync.term == 0 =>
                        {
                            if remote < local {
                                Action::DemoteCoordinator
                            } else {
                                Action::KeepCoordinator
                            }
                        }
                        _ => Action::Ignore,
                    }
                };
                assert_eq!(
                    r.action, expected,
                    "{name}: discovery, authority and election"
                );
            }
            if let EventData::JoinAck(ack) = e.data {
                let address_matches = (1..=6).contains(&ack.address_len)
                    && ack.address_len == self.current.address_len
                    && ack.target_address[..usize::from(ack.address_len)]
                        == self.current.local_address[..usize::from(ack.address_len)];
                let eligible = self.current.state == State::Joining
                    && e.payload_valid
                    && address_matches
                    && (1..=8).contains(&ack.assigned_id)
                    && ack.slot_index < 8
                    && (1..=8).contains(&ack.coordinator_id)
                    && ack.assigned_id != ack.coordinator_id
                    && e.sender_id == ack.coordinator_id
                    && (self.current.coordinator_id == 0
                        || self.current.coordinator_id == e.sender_id);
                assert_eq!(
                    r.action,
                    if eligible {
                        Action::ActivateParticipant
                    } else {
                        Action::Ignore
                    },
                    "{name}"
                );
            }
            if let EventData::Leave(leave) = e.data {
                let identity_matches = match leave.identity {
                    LeaveIdentity::Legacy => true,
                    LeaveIdentity::Address => {
                        (1..=6).contains(&leave.peer.address_len)
                            && leave.address_len == leave.peer.address_len
                            && leave.sender_address[..usize::from(leave.address_len)]
                                == leave.peer.address[..usize::from(leave.address_len)]
                    }
                    _ => false,
                };
                let eligible = e.payload_valid
                    && (1..=8).contains(&e.sender_id)
                    && e.sender_id != self.current.node_id
                    && leave.peer.present
                    && leave.peer.active
                    && leave.peer.node_id == e.sender_id
                    && identity_matches;
                assert_eq!(
                    r.action,
                    if eligible {
                        Action::RemovePeer
                    } else {
                        Action::Ignore
                    },
                    "{name}"
                );
            }
            if let EventData::SlotMap(map) = e.data {
                let eligible = (|| {
                    if !e.payload_valid
                        || self.current.state != State::Active
                        || self.current.role != Role::Participant
                        || e.sender_id != self.current.coordinator_id
                        || !(1..=8).contains(&self.current.node_id)
                        || !(1..=8).contains(&self.current.coordinator_id)
                    {
                        return false;
                    }
                    let Some(slots) = map.slot_ids.get(..usize::from(map.slot_count)) else {
                        return false;
                    };
                    let Some(speakers) = map
                        .active_speaker_ids
                        .get(..usize::from(map.active_speaker_count))
                    else {
                        return false;
                    };
                    let members: std::collections::BTreeSet<_> =
                        slots.iter().copied().filter(|id| *id != 0).collect();
                    !slots.is_empty()
                        && slots.last() != Some(&0)
                        && map.slot_ids[slots.len()..].iter().all(|id| *id == 0)
                        && members.iter().all(|id| (1..=8).contains(id))
                        && members.len() == slots.iter().filter(|id| **id != 0).count()
                        && members.contains(&self.current.node_id)
                        && members.contains(&self.current.coordinator_id)
                        && speakers.iter().all(|id| members.contains(id))
                        && speakers
                            .iter()
                            .collect::<std::collections::BTreeSet<_>>()
                            .len()
                            == speakers.len()
                })();
                assert_eq!(
                    r.action,
                    if eligible {
                        Action::ApplySlotMap
                    } else {
                        Action::Ignore
                    },
                    "{name}: membership map eligibility"
                );
            }
            match r.action {
                Action::Ignore => assert_eq!(
                    r,
                    MembershipResult {
                        next: self.current,
                        ..MembershipResult::default()
                    }
                ),
                Action::AcceptSync => {
                    assert_eq!(r.next, self.current);
                    assert_eq!((r.effects, r.affected_node_id), (0, 0));
                }
                Action::KeepCoordinator => {
                    assert_eq!(r.next, self.current);
                    assert_eq!(
                        (r.effects, r.affected_node_id),
                        (EFFECT_REPORT_LOCAL_WIN, 0)
                    );
                }
                Action::DiscoverCoordinator => {
                    let mut next = self.current;
                    next.state = State::Joining;
                    next.coordinator_id = e.sender_id;
                    assert_eq!(r.next, next);
                    assert_eq!((r.effects, r.affected_node_id), (0, 0));
                }
                Action::RemovePeer => {
                    let EventData::Leave(leave) = e.data else {
                        panic!("remove without leave")
                    };
                    let mut next = self.current;
                    next.peer_count = next
                        .peer_count
                        .saturating_sub(u8::from(leave.peer.announced));
                    assert_eq!(r.next, next);
                    assert_eq!(r.affected_node_id, e.sender_id);
                    assert_eq!(
                        r.effects,
                        if self.current.role == Role::Coordinator {
                            EFFECT_PUBLISH_SLOT_MAP
                        } else {
                            0
                        } | if leave.peer.announced {
                            EFFECT_REPORT_PEER_LEFT
                        } else {
                            0
                        }
                    );
                }
                Action::ActivateParticipant => {
                    let EventData::JoinAck(ack) = e.data else {
                        panic!("activation without ack")
                    };
                    let mut next = self.current;
                    next.state = State::Active;
                    next.role = Role::Participant;
                    next.node_id = ack.assigned_id;
                    next.slot_index = ack.slot_index as i8;
                    next.coordinator_id = ack.coordinator_id;
                    next.peer_count = 1;
                    next.participant_membership_known = false;
                    assert_eq!(r.next, next);
                    assert_eq!((r.effects, r.affected_node_id), (0, 0));
                }
                Action::DemoteCoordinator => {
                    let mut next = self.current;
                    next.state = State::Joining;
                    next.role = Role::None;
                    next.node_id = 0;
                    next.slot_index = -1;
                    next.coordinator_id = e.sender_id;
                    next.peer_count = 0;
                    next.participant_membership_known = false;
                    assert_eq!(r.next, next);
                    assert_eq!((r.effects, r.affected_node_id), (0, 0));
                }
                Action::ApplySlotMap => {
                    let EventData::SlotMap(map) = e.data else {
                        panic!("map action without map")
                    };
                    let slots = &map.slot_ids[..usize::from(map.slot_count)];
                    let mut next = self.current;
                    next.slot_index =
                        slots.iter().position(|id| *id == next.node_id).unwrap() as i8;
                    next.peer_count = slots.iter().filter(|id| **id != 0).count() as u8 - 1;
                    next.participant_membership_known = true;
                    assert_eq!(r.next, next);
                    assert_eq!((r.effects, r.affected_node_id), (0, 0));
                }
            }
        }
        serialize_snapshot(&mut self.expected, &r.next);
        self.expected.push(r.action as u8);
        number(&mut self.expected, r.effects);
        self.expected.push(r.affected_node_id);
        if current_present && event_present {
            self.current = r.next;
        }
        self.cases.push((name, self.expected.len()));
        r
    }
    fn reduce(&mut self, name: &'static str, s: Snapshot, e: Event) -> MembershipResult {
        self.reset(s);
        self.step(name, e, true, true)
    }
    fn handover(&mut self, present: bool, old: u8, new: u8, term: u32) -> bool {
        self.input
            .extend_from_slice(&[2, u8::from(present), old, new]);
        number(&mut self.input, term);
        let before = self.current;
        let ok = apply_handover(
            if present {
                Some(&mut self.current)
            } else {
                None
            },
            old,
            new,
            term,
        );
        let eligible = present
            && before.state == State::Active
            && (1..=8).contains(&before.node_id)
            && before.coordinator_id == old
            && (1..=8).contains(&new)
            && new != old
            && (0..8).contains(&before.slot_index)
            && term > before.term;
        assert_eq!(ok, eligible, "handover eligibility");
        if !ok {
            assert_eq!(
                self.current, before,
                "handover failure must preserve every field"
            );
        } else {
            let mut expected = before;
            expected.coordinator_id = new;
            expected.term = term;
            expected.role = if before.node_id == new {
                Role::Coordinator
            } else {
                Role::Participant
            };
            assert_eq!(
                self.current, expected,
                "handover changes only role/coordinator/term"
            );
        }
        self.expected.push(u8::from(ok));
        serialize_snapshot(&mut self.expected, &self.current);
        self.cases
            .push(("handover result and full post-state", self.expected.len()));
        ok
    }
}

fn sync_corpus(c: &mut Corpus) {
    for state in STATES {
        for role in ROLES {
            let base = snapshot(state, role);
            for sender in 0..=255 {
                for current_term in TERMS {
                    for remote_term in TERMS {
                        let mut s = base;
                        s.term = current_term;
                        for address in [[0; 6], [255; 6], base.local_address, [0x0f; 6], [0x80; 6]]
                        {
                            let e = Event {
                                sender_id: sender,
                                payload_valid: true,
                                data: EventData::Sync(Sync {
                                    coordinator_address: address,
                                    term: remote_term,
                                    ..sync()
                                }),
                            };
                            c.reduce("SYNC state/role/sender/term/address", s, e);
                        }
                    }
                }
            }
            for local_len in 0..=255 {
                for remote_len in [0, 1, 2, 3, 4, 5, 6, 7, 255] {
                    let mut s = base;
                    s.address_len = local_len;
                    let mut v = sync();
                    v.address_len = remote_len;
                    c.reduce("SYNC raw address lengths", s, event(EventData::Sync(v)));
                }
            }
            c.reduce(
                "SYNC invalid payload",
                base,
                Event {
                    payload_valid: false,
                    ..event(EventData::Sync(sync()))
                },
            );
        }
    }
    for len in 1..=6 {
        for index in 0..6 {
            for value in 0..=255 {
                let mut s = snapshot(State::Active, Role::Coordinator);
                s.address_len = len;
                let mut v = Sync {
                    coordinator_address: s.local_address,
                    address_len: len,
                    term: 0,
                };
                v.coordinator_address[index] = value;
                c.reduce(
                    "SYNC unsigned prefix election and ignored tails",
                    s,
                    event(EventData::Sync(v)),
                );
            }
        }
    }
}

fn join_map_corpus(c: &mut Corpus) {
    for state in STATES {
        for role in ROLES {
            let s = snapshot(state, role);
            for sender in 0..=255 {
                c.reduce(
                    "JOIN sender gating",
                    s,
                    Event {
                        sender_id: sender,
                        ..event(EventData::JoinAck(ack()))
                    },
                );
                c.reduce(
                    "SLOT_MAP sender gating",
                    s,
                    Event {
                        sender_id: sender,
                        ..event(EventData::SlotMap(map()))
                    },
                );
            }
            for valid in [false, true] {
                c.reduce(
                    "JOIN state/role/payload",
                    s,
                    Event {
                        payload_valid: valid,
                        ..event(EventData::JoinAck(ack()))
                    },
                );
                c.reduce(
                    "SLOT_MAP state/role/payload",
                    s,
                    Event {
                        payload_valid: valid,
                        ..event(EventData::SlotMap(map()))
                    },
                );
            }
        }
    }
    let joining = snapshot(State::Joining, Role::None);
    for assigned in [0, 1, 2, 3, 8, 9, 255] {
        for slot in 0..=255 {
            for coordinator in [0, 1, 3, 8, 9, 255] {
                for expected in [0, 1, 3, 255] {
                    let mut s = joining;
                    s.coordinator_id = expected;
                    let a = JoinAck {
                        assigned_id: assigned,
                        slot_index: slot,
                        coordinator_id: coordinator,
                        ..ack()
                    };
                    c.reduce(
                        "JOIN membership-specific validator",
                        s,
                        Event {
                            sender_id: coordinator,
                            ..event(EventData::JoinAck(a))
                        },
                    );
                }
            }
        }
    }
    for local_len in 0..=255 {
        for remote_len in 0..=255 {
            let mut s = joining;
            s.address_len = local_len;
            c.reduce(
                "JOIN raw lengths",
                s,
                event(EventData::JoinAck(JoinAck {
                    address_len: remote_len,
                    ..ack()
                })),
            );
        }
    }
    for len in 1..=6 {
        for index in 0..6 {
            for value in [0, 0x10, 0x20, 0x30, 0x40, 0x50, 0xa5, 255] {
                let mut s = joining;
                s.address_len = len;
                let mut a = ack();
                a.address_len = len;
                a.target_address = s.local_address;
                a.target_address[index] = value;
                c.reduce("JOIN prefix targeting", s, event(EventData::JoinAck(a)));
            }
        }
    }
    let participant = snapshot(State::Active, Role::Participant);
    for id in 1..=8 {
        let singleton = SlotMap {
            slot_count: 1,
            slot_ids: [id, 0, 0, 0, 0, 0, 0, 0],
            active_speaker_count: 1,
            active_speaker_ids: [id, 255],
            relay_masks: [255, 255],
        };
        let s = Snapshot {
            node_id: id,
            coordinator_id: id,
            ..participant
        };
        let r = c.reduce(
            "SLOT_MAP singleton local equals coordinator",
            s,
            Event {
                sender_id: id,
                ..event(EventData::SlotMap(singleton))
            },
        );
        assert_eq!(r.action, Action::ApplySlotMap);
        assert_eq!(r.next.peer_count, 0);
        assert_eq!(r.next.slot_index, 0);
    }
    let bases = [
        map(),
        SlotMap {
            slot_count: 5,
            slot_ids: [3, 0, 5, 0, 1, 0, 0, 0],
            active_speaker_count: 2,
            active_speaker_ids: [3, 5],
            relay_masks: [255, 255],
        },
    ];
    for base in bases {
        for value in 0..=255 {
            let mut s = participant;
            s.node_id = value;
            c.reduce("SLOT_MAP local ID", s, event(EventData::SlotMap(base)));
            s = participant;
            s.coordinator_id = value;
            c.reduce(
                "SLOT_MAP coordinator ID",
                s,
                Event {
                    sender_id: value,
                    ..event(EventData::SlotMap(base))
                },
            );
            c.reduce(
                "SLOT_MAP declared slot count",
                participant,
                event(EventData::SlotMap(SlotMap {
                    slot_count: value,
                    ..base
                })),
            );
            c.reduce(
                "SLOT_MAP declared speaker count",
                participant,
                event(EventData::SlotMap(SlotMap {
                    active_speaker_count: value,
                    ..base
                })),
            );
            for index in 0..8 {
                let mut m = base;
                m.slot_ids[index] = value;
                c.reduce(
                    "SLOT_MAP holes/duplicate/invalid/trailing IDs",
                    participant,
                    event(EventData::SlotMap(m)),
                );
            }
            for index in 0..2 {
                for count in 0..=2 {
                    let mut m = base;
                    m.active_speaker_count = count;
                    m.active_speaker_ids[index] = value;
                    c.reduce(
                        "SLOT_MAP declared/unused speakers",
                        participant,
                        event(EventData::SlotMap(m)),
                    );
                    m = base;
                    m.relay_masks[index] = value;
                    c.reduce(
                        "SLOT_MAP ignored relay masks",
                        participant,
                        event(EventData::SlotMap(m)),
                    );
                }
            }
        }
    }
}

fn leave_corpus(c: &mut Corpus) {
    for state in STATES {
        for role in ROLES {
            for count in [0, 1, 3, 255] {
                let mut s = snapshot(state, role);
                s.peer_count = count;
                for flags in 0u8..16 {
                    for identity in [
                        LeaveIdentity::Invalid,
                        LeaveIdentity::Legacy,
                        LeaveIdentity::Address,
                        LeaveIdentity::Unknown(255),
                    ] {
                        let mut l = leave();
                        l.identity = identity;
                        l.peer.present = flags & 1 != 0;
                        l.peer.active = flags & 2 != 0;
                        l.peer.announced = flags & 4 != 0;
                        for sender in 0..=255 {
                            l.peer.node_id = sender;
                            c.reduce(
                                "LEAVE all states/roles/peer flags/counts/senders/identities",
                                s,
                                Event {
                                    sender_id: sender,
                                    payload_valid: flags & 8 != 0,
                                    data: EventData::Leave(l),
                                },
                            );
                        }
                    }
                }
            }
        }
    }
    let s = snapshot(State::Idle, Role::Coordinator);
    for local in 0..=255 {
        c.reduce(
            "LEAVE local ID/self removal gate",
            Snapshot {
                node_id: local,
                ..s
            },
            Event {
                sender_id: 4,
                ..event(EventData::Leave(leave()))
            },
        );
    }
    for peer_len in 0..=255 {
        for sender_len in 0..=255 {
            for identity in [LeaveIdentity::Legacy, LeaveIdentity::Address] {
                let mut l = leave();
                l.identity = identity;
                l.peer.address_len = peer_len;
                l.address_len = sender_len;
                c.reduce(
                    "LEAVE raw lengths; legacy bypass",
                    s,
                    Event {
                        sender_id: 4,
                        ..event(EventData::Leave(l))
                    },
                );
            }
        }
    }
    for id in 0..=255 {
        let mut l = leave();
        l.peer.node_id = id;
        c.reduce(
            "LEAVE peer identity ID",
            s,
            Event {
                sender_id: 4,
                ..event(EventData::Leave(l))
            },
        );
    }
    for len in 1..=6 {
        for index in 0..6 {
            for value in 0..=255 {
                let mut l = leave();
                l.address_len = len;
                l.peer.address_len = len;
                l.sender_address = l.peer.address;
                l.sender_address[index] = value;
                c.reduce(
                    "LEAVE identity prefix and ignored tails",
                    s,
                    Event {
                        sender_id: 4,
                        ..event(EventData::Leave(l))
                    },
                );
            }
        }
    }
}

fn handover_corpus(c: &mut Corpus) {
    for state in STATES {
        for role in ROLES {
            for local in 0..=255 {
                for slot in [i8::MIN, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, i8::MAX] {
                    let mut s = snapshot(state, role);
                    s.node_id = local;
                    s.slot_index = slot;
                    c.reset(s);
                    c.handover(true, 1, 2, 1);
                }
            }
        }
    }
    for role in ROLES {
        for local in [1, 2, 3, 8] {
            for old in 0..=255 {
                for new in [0, 1, 2, 3, 8, 9, 255] {
                    for current_term in TERMS {
                        for next_term in TERMS {
                            let mut s = snapshot(State::Active, role);
                            s.node_id = local;
                            s.term = current_term;
                            c.reset(s);
                            c.handover(true, old, new, next_term);
                        }
                    }
                }
                // Matching invalid old leaders are intentionally not rejected by C.
                let mut s = snapshot(State::Active, role);
                s.node_id = local;
                s.coordinator_id = old;
                c.reset(s);
                c.handover(true, old, 2, 1);
            }
        }
    }
    for new in 0..=255 {
        let s = snapshot(State::Active, Role::None);
        c.reset(s);
        c.handover(true, 1, new, 1);
    }
    c.reset(snapshot(State::Active, Role::Coordinator));
    c.handover(false, 1, 2, 1);
}

fn sequences(c: &mut Corpus) {
    for role in ROLES {
        for state in STATES {
            c.reset(snapshot(state, role));
            for tag in [4, 255, 65535, 0x7fff_ffff] {
                c.step(
                    "unknown event preserves all fields",
                    event(EventData::Unknown(tag)),
                    true,
                    true,
                );
            }
            for current in [false, true] {
                for present in [false, true] {
                    let r = c.step(
                        "null arguments zero every result field",
                        event(EventData::Sync(sync())),
                        current,
                        present,
                    );
                    if !current || !present {
                        assert_eq!(r, MembershipResult::default());
                    }
                }
            }
        }
    }
    c.reset(Snapshot {
        state: State::Scanning,
        node_id: 0,
        slot_index: -1,
        coordinator_id: 0,
        ..snapshot(State::Scanning, Role::None)
    });
    assert_eq!(
        c.step("discovery", event(EventData::Sync(sync())), true, true)
            .action,
        Action::DiscoverCoordinator
    );
    assert_eq!(
        c.step(
            "join slot zero",
            event(EventData::JoinAck(JoinAck {
                slot_index: 0,
                ..ack()
            })),
            true,
            true
        )
        .action,
        Action::ActivateParticipant
    );
    let sparse = SlotMap {
        slot_count: 5,
        slot_ids: [3, 0, 5, 0, 1, 0, 0, 0],
        ..map()
    };
    assert_eq!(
        c.step(
            "map local zero coordinator elsewhere",
            event(EventData::SlotMap(sparse)),
            true,
            true
        )
        .next
        .slot_index,
        0
    );
    assert!(c.handover(true, 1, 3, 2));
    assert!(!c.handover(true, 3, 1, 2));
    assert_eq!(
        c.step(
            "post-handover election disabled",
            Event {
                sender_id: 2,
                ..event(EventData::Sync(sync()))
            },
            true,
            true
        )
        .action,
        Action::Ignore
    );
    let l = Event {
        sender_id: 4,
        ..event(EventData::Leave(leave()))
    };
    for _ in 0..4 {
        c.step(
            "announced leave reaches zero without underflow",
            l,
            true,
            true,
        );
    }
    let mut inactive = leave();
    inactive.peer.active = false;
    assert_eq!(
        c.step(
            "duplicate leave inactive peer",
            Event {
                data: EventData::Leave(inactive),
                ..l
            },
            true,
            true
        )
        .action,
        Action::Ignore
    );
    // Exact membership handover assertions from mesh_adaptive_test.c.
    c.reset(Snapshot {
        state: State::Active,
        role: Role::Coordinator,
        node_id: 1,
        coordinator_id: 1,
        slot_index: 0,
        local_address: [1, 2, 3, 4, 5, 0],
        address_len: 5,
        ..Snapshot::default()
    });
    assert!(c.handover(true, 1, 2, 2));
    assert_eq!(c.current.role, Role::Participant);
    assert_eq!(c.current.node_id, 1);
    assert_eq!(c.current.slot_index, 0);
    assert!(!c.handover(true, 2, 1, 2));
    let remote = sync();
    assert!((1..=6).contains(&remote.address_len));
    assert_eq!(c.current.address_len, remote.address_len);
    let len = usize::from(remote.address_len);
    let prefix = &remote.coordinator_address[..len];
    assert!(prefix.iter().any(|&byte| byte != 0));
    assert!(prefix.iter().any(|&byte| byte != u8::MAX));
    assert_ne!(prefix, &c.current.local_address[..len]);
    assert_eq!(c.current.state, State::Active);
    assert_eq!(c.current.role, Role::Participant);
    assert_eq!(c.current.coordinator_id, 2);
    assert_eq!(c.current.term, 2);
    for term in [0, 1, 2] {
        let received = Event {
            sender_id: 2,
            ..event(EventData::Sync(Sync { term, ..remote }))
        };
        assert!(received.payload_valid);
        assert!((1..=8).contains(&received.sender_id));
        assert_eq!(received.sender_id, c.current.coordinator_id);
        let zero_term = Snapshot {
            term: 0,
            ..c.current
        };
        assert_eq!(
            reduce(Some(&zero_term), Some(&received)).action,
            Action::AcceptSync,
            "the same validated SYNC is accepted with local term zero"
        );
        let matching_term = Snapshot { term, ..c.current };
        assert_eq!(
            reduce(Some(&matching_term), Some(&received)).action,
            Action::AcceptSync,
            "the same validated SYNC is accepted with a matching local term"
        );
    }
    assert_eq!(
        c.step(
            "participant nonzero term rejects legacy sync",
            Event {
                sender_id: 2,
                ..event(EventData::Sync(remote))
            },
            true,
            true
        )
        .action,
        Action::Ignore
    );
    assert_eq!(
        c.step(
            "participant after handover rejects old nonzero term",
            Event {
                sender_id: 2,
                ..event(EventData::Sync(Sync { term: 1, ..remote }))
            },
            true,
            true
        )
        .action,
        Action::Ignore
    );
    assert_eq!(
        c.step(
            "participant matching term",
            Event {
                sender_id: 2,
                ..event(EventData::Sync(Sync { term: 2, ..remote }))
            },
            true,
            true
        )
        .action,
        Action::AcceptSync
    );
    assert!(c.handover(true, 2, 1, 3));
}

#[test]
fn corpus_walk_holds_reducer_invariants() {
    let mut c = Corpus::default();
    sync_corpus(&mut c);
    join_map_corpus(&mut c);
    leave_corpus(&mut c);
    handover_corpus(&mut c);
    sequences(&mut c);
    eprintln!("membership corpus: {} cases", c.cases.len());
}

#[test]
fn explicit_validator_distinctions_and_effects() {
    let joining = snapshot(State::Joining, Role::Coordinator);
    let a = JoinAck {
        slot_index: 0,
        ..ack()
    };
    let r = reduce(Some(&joining), Some(&event(EventData::JoinAck(a))));
    assert_eq!(r.action, Action::ActivateParticipant);
    assert_eq!(r.next.slot_index, 0);
    assert!(!openhelmet_mesh::core::join_assignment_valid(
        Some(&openhelmet_mesh::core::JoinAssignment {
            assigned_id: a.assigned_id,
            slot_index: a.slot_index,
            coordinator_id: a.coordinator_id
        }),
        1,
        1,
        8
    ));
    let s = snapshot(State::Active, Role::Participant);
    let m = SlotMap {
        slot_count: 5,
        slot_ids: [3, 0, 5, 0, 1, 0, 0, 0],
        ..map()
    };
    assert_eq!(
        reduce(Some(&s), Some(&event(EventData::SlotMap(m)))).action,
        Action::ApplySlotMap
    );
    assert!(!openhelmet_mesh::core::slot_map_valid(Some(&m), 3, 1, None));
    let mut trailing = map();
    trailing.slot_ids[7] = 255;
    assert_eq!(
        reduce(Some(&s), Some(&event(EventData::SlotMap(trailing)))).action,
        Action::Ignore
    );
    assert!(openhelmet_mesh::core::slot_map_valid(
        Some(&trailing),
        3,
        1,
        None
    ));
    let mut empty_last = map();
    empty_last.slot_count = 4;
    assert_eq!(
        reduce(Some(&s), Some(&event(EventData::SlotMap(empty_last)))).action,
        Action::Ignore
    );
    assert!(openhelmet_mesh::core::slot_map_valid(
        Some(&empty_last),
        3,
        1,
        None
    ));
    for state in STATES {
        let s = Snapshot {
            peer_count: 0,
            ..snapshot(state, Role::Coordinator)
        };
        let r = reduce(
            Some(&s),
            Some(&Event {
                sender_id: 4,
                ..event(EventData::Leave(leave()))
            }),
        );
        assert_eq!(r.action, Action::RemovePeer);
        assert_eq!(r.next, s);
        assert_eq!(r.effects, EFFECT_REPORT_PEER_LEFT | EFFECT_PUBLISH_SLOT_MAP);
        assert_eq!(r.affected_node_id, 4);
    }
    let mut s = Snapshot {
        term: u32::MAX,
        ..snapshot(State::Active, Role::None)
    };
    let before = s;
    assert!(!apply_handover(Some(&mut s), 1, 2, 0));
    assert_eq!(s, before);
}
