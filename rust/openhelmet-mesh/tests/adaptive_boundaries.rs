use openhelmet_mesh::{adaptive::*, core::SlotMap};

// Literal native-test requirements, executed against the production Rust owner.
// No serialized actual-output corpus or legacy C oracle is involved.
fn membership_fixture(local: u8) -> (Adaptive, Membership) {
    let mut state = Adaptive::default();
    init(Some(&mut state), local, 1, 1, 7, 0);
    let mut membership = Membership {
        term: 1,
        revision: 1,
        leader_id: 1,
        slot_map: SlotMap {
            slot_count: 3,
            slot_ids: [1, 2, 3, 0, 0, 0, 0, 0],
            active_speaker_count: 1,
            active_speaker_ids: [3, 0],
            relay_masks: [2, 0],
        },
        ..Membership::default()
    };
    for id in 1..=3 {
        let identity = Identity {
            address_len: 6,
            address: [id, 0, 0, 0, 0, 0],
        };
        membership.identities[usize::from(id - 1)] = identity;
        assert!(set_member(Some(&mut state), id, Some(&identity)));
    }
    (state, membership)
}

fn topology_row(id: u8, mask: u8, seq: u32) -> Topology {
    Topology {
        report_seq: seq,
        origin: Identity {
            address_len: 6,
            address: [id, 0, 0, 0, 0, 0],
        },
        direct_mask: mask,
        rssi_dbm: [-55; 8],
        quality: [90; 8],
        leader_id: 1,
        term: 1,
        ..Topology::default()
    }
}

fn chain_reports(state: &mut Adaptive, now: u32, seq: u32) {
    for (id, mask) in [(1, 2), (2, 5), (3, 2)] {
        assert!(report(
            Some(state),
            id,
            Some(&topology_row(id, mask, seq)),
            id == 3,
            now
        ));
    }
}

#[test]
fn election_wrap_switch_reconciles_and_resets_new_term_membership_revision() {
    let (mut state, mut membership) = membership_fixture(1);
    membership.revision = 99;
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    assert_eq!(next_membership_revision(Some(&mut state)), 1);
    chain_reports(&mut state, 100, u32::MAX - 1);
    chain_reports(&mut state, 1100, 0);
    assert_eq!(candidate(Some(&mut state), 1100), 0);
    chain_reports(&mut state, 10100, 1);
    assert_eq!(candidate(Some(&mut state), 10100), 0);
    chain_reports(&mut state, 13101, 2);
    assert_eq!(candidate(Some(&mut state), 13101), 2);
    let mut proposal = Handover::default();
    assert!(prepare(
        Some(&mut state),
        2,
        0xfffffff0,
        Some(&membership.slot_map),
        13101,
        Some(&mut proposal)
    ));
    assert_eq!(
        proposal,
        Handover {
            old_leader_id: 1,
            candidate_id: 2,
            next_term: 2,
            switch_frame: 144,
            members: 7,
            slot_map: membership.slot_map,
        }
    );
    let mut committed = Handover::default();
    assert!(!commit(Some(&mut state), Some(&mut committed), 13102));
    let mut acknowledgement = HandoverAck {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 144,
        acknowledger_id: 2,
    };
    assert!(ack(Some(&mut state), 2, Some(&acknowledgement), 13102));
    assert!(!commit(Some(&mut state), Some(&mut committed), 13102));
    acknowledgement.acknowledger_id = 3;
    assert!(ack(Some(&mut state), 3, Some(&acknowledgement), 13103));
    assert!(commit(Some(&mut state), Some(&mut committed), 13104));
    assert_eq!(committed, proposal);
    let waiting = state.snapshot();
    assert!(!tick(Some(&mut state), 143, 16000));
    assert!(!tick(Some(&mut state), 144, 16400));
    assert_eq!(state.snapshot(), waiting);
    let sync = Sync {
        leader: membership.identities[1],
        leader_id: 2,
        member_count: 3,
        term: 2,
        frame_counter: 144,
        phase_us: 500,
        relay_depth: 0,
    };
    assert!(reconcile_sync(Some(&mut state), 2, Some(&sync), 16420));
    let transferred = state.snapshot();
    assert_eq!(
        (
            transferred.local_id,
            transferred.leader_id,
            transferred.term
        ),
        (1, 2, 2)
    );
    assert!(!transferred.membership_revision_seen);
    assert_eq!(
        (
            transferred.membership_revision,
            transferred.publish_revision
        ),
        (0, 0)
    );
    membership.term = 2;
    membership.leader_id = 2;
    membership.revision = 1;
    assert!(apply_membership(Some(&mut state), 2, Some(&membership)));
    assert!(state.snapshot().membership_revision_seen);
    assert_eq!(state.snapshot().membership_revision, 1);
    let before_old_frame = state.snapshot();
    assert!(!tick(Some(&mut state), 45, 14320));
    assert_eq!(state.snapshot(), before_old_frame);
}

#[test]
fn three_owner_ack_relay_preserves_bytes_and_only_old_leader_collects_quorum() {
    let (mut old, membership) = membership_fixture(1);
    let (mut candidate_owner, _) = membership_fixture(2);
    let (mut follower, _) = membership_fixture(3);
    for (now, seq) in [(10100, 1), (13101, 2)] {
        chain_reports(&mut old, now, seq);
        chain_reports(&mut candidate_owner, now, seq);
        for (id, mask) in [(2, 5), (3, 2)] {
            assert!(report(
                Some(&mut follower),
                id,
                Some(&topology_row(id, mask, seq)),
                false,
                now
            ));
        }
        assert_eq!(candidate(Some(&mut old), now), if seq == 1 { 0 } else { 2 });
    }
    assert!(!follower.snapshot().nodes[0].report_seen);
    let mut proposal = Handover::default();
    assert!(prepare(
        Some(&mut old),
        2,
        1000,
        Some(&membership.slot_map),
        13101,
        Some(&mut proposal)
    ));
    assert_eq!(proposal.switch_frame, 1160);
    assert!(receive_prepare(
        Some(&mut candidate_owner),
        1,
        Some(&proposal),
        1001,
        13121
    ));
    assert!(receive_prepare(
        Some(&mut follower),
        1,
        Some(&proposal),
        1001,
        13121
    ));
    assert_eq!(direct_graph(Some(&mut follower), 2, 13121) & 1, 0);
    assert_eq!(direct_graph(Some(&mut follower), 3, 13121) & 2, 2);
    // Independent asymmetric and stale reverse-edge controls, with bound IDs.
    for (reverse_mask, reverse_time) in [(1, 13101), (5, 9000)] {
        let (mut rejected, _) = membership_fixture(3);
        assert!(report(
            Some(&mut rejected),
            2,
            Some(&topology_row(2, reverse_mask, 2)),
            false,
            reverse_time
        ));
        assert!(report(
            Some(&mut rejected),
            3,
            Some(&topology_row(3, 2, 2)),
            false,
            13101
        ));
        assert!(!receive_prepare(
            Some(&mut rejected),
            1,
            Some(&proposal),
            1001,
            13121
        ));
        assert!(!rejected.snapshot().prepared);
    }
    let mut acknowledgement = HandoverAck {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 1160,
        acknowledger_id: 3,
    };
    let before_invalid = candidate_owner.snapshot();
    let invalid = HandoverAck {
        next_term: 3,
        ..acknowledgement
    };
    assert!(!ack(Some(&mut candidate_owner), 3, Some(&invalid), 13150));
    assert_eq!(candidate_owner.snapshot(), before_invalid);
    assert!(!ack(
        Some(&mut candidate_owner),
        2,
        Some(&acknowledgement),
        13150
    ));
    assert_eq!(candidate_owner.snapshot(), before_invalid);
    assert_eq!(candidate_owner.forward_queue_len(), 0);
    assert!(ack(Some(&mut follower), 3, Some(&acknowledgement), 13150));
    assert!(ack(
        Some(&mut candidate_owner),
        3,
        Some(&acknowledgement),
        13160
    ));
    assert_eq!(follower.snapshot().ack_mask, 0);
    assert_eq!(candidate_owner.snapshot().ack_mask, 0);
    let mut committed = Handover::default();
    assert!(!commit(
        Some(&mut candidate_owner),
        Some(&mut committed),
        13160
    ));
    // Literal wire contract: IDs, LE term2, LE frame1160, acknowledger3.
    let payload = [1, 2, 2, 0, 0, 0, 0x88, 4, 0, 0, 3];
    let header = Header {
        version: 5,
        packet_type: 19,
        src_id: 3,
        seq: 9,
        ttl: 2,
        flags: 0,
        talk_channel: 0,
        payload_len: 11,
    };
    assert!(forward_enqueue(
        Some(&mut candidate_owner),
        Some(&header),
        Some(&payload),
        None,
        1,
        true,
        13160
    ));
    assert_eq!(candidate_owner.forward_queue_len(), 1);
    let mut forwarded = ForwardPacket::default();
    assert!(forward_dequeue(
        Some(&mut candidate_owner),
        Some(&mut forwarded),
        1002,
        0,
        0
    ));
    assert_eq!(candidate_owner.forward_queue_len(), 0);
    assert_eq!(
        forwarded.header,
        Header {
            ttl: 1,
            flags: 2,
            ..header
        }
    );
    assert_eq!(forwarded.payload_len, 11);
    assert_eq!(&forwarded.payload[..11], &payload);
    let received = HandoverAck {
        old_leader_id: forwarded.payload[0],
        candidate_id: forwarded.payload[1],
        next_term: u32::from_le_bytes(forwarded.payload[2..6].try_into().unwrap()),
        switch_frame: u32::from_le_bytes(forwarded.payload[6..10].try_into().unwrap()),
        acknowledger_id: forwarded.payload[10],
    };
    assert_eq!(received, acknowledgement);
    assert_eq!(old.snapshot().ack_mask, 1);
    assert!(ack(
        Some(&mut old),
        forwarded.header.src_id,
        Some(&received),
        13180
    ));
    assert_eq!(old.snapshot().ack_mask, 5);
    assert!(!commit(Some(&mut old), Some(&mut committed), 13180));
    acknowledgement.acknowledger_id = 2;
    assert!(ack(Some(&mut old), 2, Some(&acknowledgement), 13200));
    assert_eq!(old.snapshot().ack_mask, 7);
    assert!(commit(Some(&mut old), Some(&mut committed), 13200));
    assert_eq!(committed, proposal);
    assert_eq!(candidate_owner.snapshot().ack_mask, 0);
    assert!(!commit(
        Some(&mut candidate_owner),
        Some(&mut committed),
        13200
    ));
    assert!(!commit(Some(&mut follower), Some(&mut committed), 13200));
}

#[test]
fn lost_candidate_commit_retains_both_owners_then_retries_through_grace() {
    let (mut old, mut membership) = membership_fixture(1);
    let (mut candidate_owner, _) = membership_fixture(2);
    membership.revision = 99;
    for owner in [&mut old, &mut candidate_owner] {
        assert!(apply_membership(Some(owner), 1, Some(&membership)));
        chain_reports(owner, 10100, 1);
        assert_eq!(candidate(Some(owner), 10100), 0);
        chain_reports(owner, 13101, 2);
    }
    assert_eq!(next_membership_revision(Some(&mut old)), 1);
    let mut proposal = Handover::default();
    assert!(prepare(
        Some(&mut old),
        2,
        1000,
        Some(&membership.slot_map),
        13101,
        Some(&mut proposal)
    ));
    assert_eq!(proposal.switch_frame, 1160);
    assert!(receive_prepare(
        Some(&mut candidate_owner),
        1,
        Some(&proposal),
        1001,
        13121
    ));
    for id in [2, 3] {
        let acknowledgement = HandoverAck {
            old_leader_id: 1,
            candidate_id: 2,
            next_term: 2,
            switch_frame: 1160,
            acknowledger_id: id,
        };
        assert!(ack(Some(&mut old), id, Some(&acknowledgement), 13200));
    }
    let mut committed = Handover::default();
    assert!(commit(Some(&mut old), Some(&mut committed), 13201));
    assert_eq!(committed, proposal);
    let old_waiting = old.snapshot();
    let candidate_waiting = candidate_owner.snapshot();
    assert!(old_waiting.committed);
    assert!(!candidate_waiting.committed);
    assert!(!tick(Some(&mut candidate_owner), 1160, 16321));
    assert!(!tick(Some(&mut old), 1160, 16321));
    assert_eq!(old.snapshot(), old_waiting);
    assert_eq!(candidate_owner.snapshot(), candidate_waiting);
    assert_eq!(
        (
            old.snapshot().leader_id,
            candidate_owner.snapshot().leader_id
        ),
        (1, 1)
    );
    // Retry is after the ACK deadline, but still inside recovery grace.
    assert!(16400 > old_waiting.deadline_ms);
    assert!(16400 < old_waiting.recovery_deadline_ms);
    assert!(commit(Some(&mut old), Some(&mut committed), 16400));
    assert_eq!(committed, proposal);
    assert!(receive_commit(
        Some(&mut candidate_owner),
        1,
        Some(&committed),
        16401
    ));
    let active = SpeakerRequest {
        term: 1,
        request_seq: 1,
        frame_counter: 1161,
        active: 1,
    };
    assert!(speaker_request(
        Some(&mut candidate_owner),
        3,
        Some(&active),
        1161,
        16401
    ));
    assert!(speaker_request_active(Some(&candidate_owner), 3, 16401));
    assert!(tick(Some(&mut candidate_owner), 1161, 16420));
    assert!(!speaker_request_active(Some(&candidate_owner), 3, 16420));
    let mut sync = Sync {
        leader: membership.identities[1],
        leader_id: 2,
        member_count: 3,
        term: 2,
        frame_counter: 1161,
        phase_us: 400,
        relay_depth: 0,
    };
    assert!(reconcile_sync(Some(&mut old), 2, Some(&sync), 16430));
    for (owner, local) in [(&old, 1), (&candidate_owner, 2)] {
        let snapshot = owner.snapshot();
        assert_eq!(
            (snapshot.local_id, snapshot.leader_id, snapshot.term),
            (local, 2, 2)
        );
        assert!(!snapshot.membership_revision_seen);
        assert_eq!(
            (snapshot.membership_revision, snapshot.publish_revision),
            (0, 0)
        );
    }
    assert_eq!(next_membership_revision(Some(&mut candidate_owner)), 1);
    sync.term = 1;
    let before_old_sync = old.snapshot();
    assert!(!reconcile_sync(Some(&mut old), 2, Some(&sync), 16440));
    assert_eq!(old.snapshot(), before_old_sync);
}

#[test]
fn authoritative_identical_revision_preserves_preparation_and_replacement_is_selective() {
    let (mut state, mut membership) = membership_fixture(2);
    for (id, mask) in [(1, 2), (2, 5), (3, 2)] {
        let topology = Topology {
            report_seq: 1,
            origin: membership.identities[usize::from(id - 1)],
            direct_mask: mask,
            rssi_dbm: [-55; 8],
            quality: [90; 8],
            leader_id: 1,
            term: 1,
            ..Topology::default()
        };
        assert!(report(Some(&mut state), id, Some(&topology), id == 3, 100));
    }
    // Materialize live reciprocal edges before testing selective invalidation.
    assert_eq!(direct_graph(Some(&mut state), 1, 100), 2);
    assert_eq!(direct_graph(Some(&mut state), 2, 100), 5);
    assert_eq!(direct_graph(Some(&mut state), 3, 100), 2);
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    let proposal = Handover {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 120,
        members: 7,
        slot_map: membership.slot_map,
    };
    assert!(receive_prepare(
        Some(&mut state),
        1,
        Some(&proposal),
        100,
        100
    ));
    let before = state.snapshot();
    assert!(before.prepared);
    assert!(before.nodes[..3].iter().all(|node| node.report_seen));
    assert_eq!(&before.edge_live[..3], &[2, 5, 2]);
    assert!(!apply_membership(Some(&mut state), 1, Some(&membership)));
    assert_eq!(state.snapshot(), before);
    membership.revision = 2;
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    let mut identical_expected = before;
    identical_expected.membership_revision = 2;
    assert_eq!(state.snapshot(), identical_expected);

    membership.revision = 3;
    membership.identities[2].address[0] = 44;
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    let replaced = state.snapshot();
    assert_eq!(
        (
            replaced.members,
            replaced.term,
            replaced.membership_revision
        ),
        (7, 1, 3)
    );
    assert!(!replaced.prepared);
    // Only incident edges/quality columns may be lost in surviving report rows.
    let mut survivor1 = identical_expected.nodes[0];
    survivor1.direct_mask &= !4;
    survivor1.quality[2] = 0;
    let mut survivor2 = identical_expected.nodes[1];
    survivor2.direct_mask &= !4;
    survivor2.quality[2] = 0;
    assert_eq!(replaced.nodes[0], survivor1);
    assert_eq!(replaced.nodes[1], survivor2);
    assert_eq!(
        replaced.nodes[2],
        Member {
            present: true,
            identity: membership.identities[2],
            ..Member::default()
        }
    );
    assert_eq!(&replaced.edge_live[..3], &[2, 1, 0]);
    // Behavioral controls: the old binding must fail without mutation, while
    // the new binding accepts the very same report and starts fresh at 100.
    let mut topology = Topology {
        report_seq: 2,
        origin: identical_expected.nodes[2].identity,
        direct_mask: 2,
        rssi_dbm: [-55; 8],
        quality: [90; 8],
        leader_id: 1,
        term: 1,
        ..Topology::default()
    };
    assert!(!report(Some(&mut state), 3, Some(&topology), true, 200));
    assert_eq!(state.snapshot(), replaced);
    topology.origin = membership.identities[2];
    assert!(report(Some(&mut state), 3, Some(&topology), true, 200));
    assert_eq!(state.snapshot().nodes[2].loss_quality, 100);

    membership.revision = 4;
    membership.slot_map = SlotMap {
        slot_count: 2,
        slot_ids: [1, 2, 0, 0, 0, 0, 0, 0],
        ..SlotMap::default()
    };
    membership.identities[2] = Identity::default();
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    let removed = state.snapshot();
    assert_eq!(removed.members, 3);
    assert_eq!(removed.nodes[2], Member::default());
    let (_, stale_readd) = membership_fixture(1);
    assert!(!apply_membership(Some(&mut state), 1, Some(&stale_readd)));
    assert_eq!(state.snapshot(), removed);
}

#[test]
fn membership_revisions_progress_max_minus_one_max_zero_and_reject_backwards() {
    let (mut state, mut membership) = membership_fixture(1);
    for revision in [u32::MAX - 1, u32::MAX, 0] {
        membership.revision = revision;
        assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
        assert!(state.snapshot().membership_revision_seen);
        assert_eq!(state.snapshot().membership_revision, revision);
        let before = state.snapshot();
        assert!(!apply_membership(Some(&mut state), 1, Some(&membership)));
        assert_eq!(state.snapshot(), before);
    }
    let before = state.snapshot();
    for revision in [u32::MAX, u32::MAX - 1, 0x8000_0000] {
        membership.revision = revision;
        assert!(!apply_membership(Some(&mut state), 1, Some(&membership)));
        assert_eq!(state.snapshot(), before);
    }
    // Publication is an independent local allocator, not the received revision.
    assert_eq!(next_membership_revision(Some(&mut state)), 1);
    assert_eq!(next_membership_revision(Some(&mut state)), 2);
}

#[test]
fn local_eviction_requires_new_trusted_canonical_authority_without_mutation() {
    let (mut state, membership) = membership_fixture(2);
    assert!(apply_membership(Some(&mut state), 1, Some(&membership)));
    let before = state.snapshot();
    let mut eviction = membership;
    eviction.revision = 2;
    eviction.slot_map.slot_ids[1] = 0;
    eviction.identities[1] = Identity::default();
    assert!(!membership_valid(Some(&eviction), None)); // residual relay to removed ID
    assert!(!membership_removes_local(Some(&state), 1, Some(&eviction)));
    assert!(!apply_membership(Some(&mut state), 1, Some(&eviction)));
    assert_eq!(state.snapshot(), before);
    eviction.slot_map.relay_masks[0] = 0;
    assert!(membership_valid(Some(&eviction), None));
    assert!(membership_removes_local(Some(&state), 1, Some(&eviction)));
    assert!(!apply_membership(Some(&mut state), 1, Some(&eviction)));
    assert_eq!(state.snapshot(), before);
    // Each independently mutated input starts from the positive control.
    for case in 0..10 {
        let mut bad = eviction;
        let mut source = 1;
        match case {
            0 => bad.revision = 1, // equal
            1 => bad.revision = 0, // old
            2 => source = 3,
            3 => bad.term = 2,
            4 => bad.leader_id = 3,
            5 => bad.identities[0].address[0] = 9, // leader binding mismatch
            6 => bad.identities[2] = bad.identities[0], // duplicate identity
            7 => bad.slot_map.slot_count = 4,      // noncanonical empty tail
            8 => {
                bad = membership;
                bad.revision = 2;
            } // does not remove local
            _ => bad.identities[3] = membership.identities[2], // absent identity tail
        }
        assert!(
            !membership_removes_local(Some(&state), source, Some(&bad)),
            "case {case}"
        );
        assert_eq!(state.snapshot(), before, "predicate case {case}");
        // A still-present local member is ordinary membership, not eviction.
        if case != 8 {
            assert!(
                !apply_membership(Some(&mut state), source, Some(&bad)),
                "case {case}"
            );
            assert_eq!(state.snapshot(), before, "apply case {case}");
        }
    }
    // Use an API-produced unbound owner, not native private-state surgery.
    let mut unbound = Adaptive::default();
    init(Some(&mut unbound), 2, 1, 1, 7, 0);
    let identity = membership.identities[1];
    assert!(set_member(Some(&mut unbound), 2, Some(&identity)));
    let unbound_before = unbound.snapshot();
    assert_eq!(unbound_before.nodes[0].identity, Identity::default());
    assert!(!membership_removes_local(
        Some(&unbound),
        1,
        Some(&eviction)
    ));
    assert_eq!(unbound.snapshot(), unbound_before);
    // Binding the trusted leader alone changes this rejection to acceptance.
    assert!(set_member(
        Some(&mut unbound),
        1,
        Some(&membership.identities[0])
    ));
    let bound_before = unbound.snapshot();
    assert!(membership_removes_local(Some(&unbound), 1, Some(&eviction)));
    assert!(!apply_membership(Some(&mut unbound), 1, Some(&eviction)));
    assert_eq!(unbound.snapshot(), bound_before);
    assert!(!membership_removes_local(None, 1, Some(&eviction)));
    assert!(!membership_removes_local(Some(&state), 1, None));
}

#[test]
fn partition_has_no_candidate_and_sustained_reports_restore_route() {
    let (mut state, _) = membership_fixture(1);
    assert!(update_members(Some(&mut state), 15));
    assert!(set_member(
        Some(&mut state),
        4,
        Some(&topology_row(4, 0, 0).origin)
    ));
    chain_reports(&mut state, 100, 1);
    for (id, mask) in [(3, 10), (4, 4)] {
        assert!(report(
            Some(&mut state),
            id,
            Some(&topology_row(id, mask, 2)),
            true,
            200
        ));
    }
    let mut mask = 255;
    assert_eq!(route(Some(&mut state), 1, 200, Some(&mut mask)), -1);
    assert_eq!(candidate(Some(&mut state), 14000), 0);
    assert_eq!(state.snapshot().nodes[2].last_seen_ms, 0);

    let (mut state, _) = membership_fixture(1);
    chain_reports(&mut state, 100, 1);
    assert_eq!(direct_graph(Some(&mut state), 1, 100), 2);
    assert!(report(
        Some(&mut state),
        2,
        Some(&topology_row(2, 4, 2)),
        false,
        1100
    ));
    assert_eq!(direct_graph(Some(&mut state), 1, 1100), 0);
    for now in (2100..10000).step_by(1000) {
        chain_reports(&mut state, now, now / 1000 + 2);
    }
    assert_eq!(route(Some(&mut state), 1, 9100, Some(&mut mask)), 1);
    assert_eq!(mask, 2);
    assert_eq!(direct_graph(Some(&mut state), 1, 14000), 0);
}

#[test]
fn pregrant_request_rejects_invalid_fields_without_consuming_sequence() {
    let (mut state, _) = membership_fixture(1);
    let initial = SpeakerRequest {
        term: 1,
        request_seq: u32::MAX,
        frame_counter: 0,
        active: 1,
    };
    assert!(speaker_request(Some(&mut state), 3, Some(&initial), 0, 0));
    assert!(speaker_request_active(Some(&state), 3, 1000));
    assert!(!speaker_request_active(Some(&state), 3, 1501));
    let wrapped = SpeakerRequest {
        request_seq: 0,
        frame_counter: u32::MAX,
        ..initial
    };
    assert!(speaker_request(
        Some(&mut state),
        3,
        Some(&wrapped),
        0,
        2000
    ));
    let next = SpeakerRequest {
        request_seq: 1,
        frame_counter: 0,
        ..initial
    };
    for (source, request, frame) in [
        (3, wrapped, 0), // duplicate
        (3, initial, 0), // older sequence
        (
            3,
            SpeakerRequest {
                request_seq: 0x8000_0000,
                ..next
            },
            0,
        ),
        (3, next, 51),
        (3, next, u32::MAX), // future request
        (3, SpeakerRequest { term: 2, ..next }, 0),
        (4, next, 0), // nonmember
        (3, SpeakerRequest { active: 2, ..next }, 0),
    ] {
        let before = state.snapshot();
        assert!(!speaker_request(
            Some(&mut state),
            source,
            Some(&request),
            frame,
            2020
        ));
        assert_eq!(state.snapshot(), before);
    }
    let inactive = SpeakerRequest { active: 0, ..next };
    assert!(speaker_request(
        Some(&mut state),
        3,
        Some(&inactive),
        0,
        2020
    ));
    assert!(!speaker_request_active(Some(&state), 3, 2020));
    assert!(speaker_request(
        Some(&mut state),
        3,
        Some(&SpeakerRequest {
            request_seq: 2,
            ..next
        }),
        0,
        2030
    ));
    assert!(speaker_request_active(Some(&state), 3, 2030));
}

#[test]
fn report_loss_comparison_and_first_low_sample_preserve_rf_hysteresis() {
    for local in [1, 2] {
        let (mut state, _) = membership_fixture(local);
        assert!(update_members(Some(&mut state), 3));
        for i in 0..=4 {
            for (id, mask, seq) in [(1, 2, 1 + i), (2, 1, 1 + 2 * i)] {
                assert!(report(
                    Some(&mut state),
                    id,
                    Some(&topology_row(id, mask, seq)),
                    false,
                    100 + i * 1000
                ));
            }
        }
        let nodes = state.snapshot().nodes;
        assert_eq!(nodes[0].loss_quality, 100);
        assert_eq!(nodes[1].loss_quality, 67);
        assert!(nodes[1].loss_quality < nodes[0].loss_quality);
        assert_eq!(direct_graph(Some(&mut state), 1, 4100), 2);
        let mut low = topology_row(2, 1, 10);
        low.quality[0] = 0;
        assert!(report(Some(&mut state), 2, Some(&low), false, 4200));
        assert!(state.snapshot().nodes[1].loss_quality < 100);
        assert_eq!(state.snapshot().nodes[1].quality[0], 68);
        assert_eq!(direct_graph(Some(&mut state), 1, 4200), 2);
        for i in 1..=4 {
            low.report_seq += 1;
            assert!(report(
                Some(&mut state),
                2,
                Some(&low),
                false,
                4200 + i * 100
            ));
        }
        assert_eq!(direct_graph(Some(&mut state), 1, 4600), 0);
    }
}

#[test]
fn topology_requires_binding_and_clear_readd_starts_fresh() {
    let mut state = Adaptive::default();
    init(Some(&mut state), 1, 1, 1, 7, 0);
    let mut row = topology_row(2, 5, 1);
    let before = state.snapshot();
    assert!(!report(Some(&mut state), 2, Some(&row), false, 100));
    assert_eq!(state.snapshot(), before);
    let (mut state, _) = membership_fixture(1);
    assert!(report(Some(&mut state), 2, Some(&row), false, 100));
    let old = row;
    row.origin.address[0] = 42;
    let before = state.snapshot();
    assert!(!report(Some(&mut state), 2, Some(&row), false, 101));
    assert!(!set_member(Some(&mut state), 1, Some(&old.origin)));
    assert_eq!(state.snapshot(), before);
    assert!(set_member(Some(&mut state), 2, Some(&row.origin)));
    assert!(!report(Some(&mut state), 2, Some(&old), false, 102));
    assert!(report(Some(&mut state), 2, Some(&row), false, 102));
    assert_eq!(state.snapshot().nodes[1].loss_quality, 100);
    assert!(set_member(Some(&mut state), 2, None));
    assert!(!report(Some(&mut state), 2, Some(&row), false, 103));
    assert!(update_members(Some(&mut state), 7));
    let before = state.snapshot();
    assert!(!report(Some(&mut state), 2, Some(&row), false, 104));
    assert_eq!(state.snapshot(), before);
    assert!(set_member(Some(&mut state), 2, Some(&row.origin)));
    assert!(report(Some(&mut state), 2, Some(&row), false, 105));
    assert_eq!(state.snapshot().nodes[1].loss_quality, 100);
}

#[test]
fn follower_rejects_malformed_sync_and_preserves_sparse_pending_slots() {
    let (mut state, membership) = membership_fixture(3);
    chain_reports(&mut state, 100, 1);
    let proposal = Handover {
        old_leader_id: 1,
        candidate_id: 2,
        next_term: 2,
        switch_frame: 160,
        members: 7,
        slot_map: membership.slot_map,
    };
    assert!(receive_prepare(
        Some(&mut state),
        1,
        Some(&proposal),
        0,
        100
    ));
    assert!(!tick(Some(&mut state), 160, 3300));
    let sync = Sync {
        leader: topology_row(2, 0, 0).origin,
        leader_id: 2,
        member_count: 3,
        term: 2,
        frame_counter: 160,
        phase_us: 500,
        relay_depth: 0,
    };
    for (source, member_count) in [(2, 0), (2, 9), (1, 3)] {
        let before = state.snapshot();
        assert!(!reconcile_sync(
            Some(&mut state),
            source,
            Some(&Sync {
                member_count,
                ..sync
            }),
            3300
        ));
        assert_eq!(state.snapshot(), before);
    }
    assert!(reconcile_sync(Some(&mut state), 2, Some(&sync), 3300));
    assert_eq!(state.session(), (3, 2, 7, 2));

    let (mut state, _) = membership_fixture(3);
    assert!(update_members(Some(&mut state), 5));
    for (id, mask) in [(1, 4), (3, 1)] {
        assert!(report(
            Some(&mut state),
            id,
            Some(&topology_row(id, mask, 1)),
            false,
            100
        ));
    }
    let sparse = Handover {
        candidate_id: 3,
        members: 5,
        slot_map: SlotMap {
            slot_count: 3,
            slot_ids: [1, 0, 3, 0, 0, 0, 0, 0],
            ..SlotMap::default()
        },
        ..proposal
    };
    assert!(receive_prepare(Some(&mut state), 1, Some(&sparse), 0, 100));
    assert_eq!(state.snapshot().pending, sparse);
    assert_eq!(
        state.snapshot().pending.slot_map.slot_ids,
        [1, 0, 3, 0, 0, 0, 0, 0]
    );
}

// Payloads below are independent wire literals, not serialized owner output.
fn enqueue_literal(
    state: &mut Adaptive,
    packet_type: u8,
    source: u8,
    seq: u8,
    payload: &[u8],
    origin: Option<&Identity>,
) -> bool {
    forward_enqueue(
        Some(state),
        Some(&Header {
            version: 5,
            packet_type,
            src_id: source,
            seq,
            ttl: 2,
            payload_len: payload.len() as u16,
            ..Header::default()
        }),
        Some(payload),
        origin,
        1,
        true,
        100,
    )
}

#[test]
fn local_control_termination_rejects_leader_origins_and_has_exact_queue_counts() {
    let (mut leader, _) = membership_fixture(1);
    let (mut relay, _) = membership_fixture(2);
    let join = [6, 9, 0, 0, 0, 0, 0, 0, 6, 1, 0, 0, 0, 0, 0];
    let origin = Identity {
        address_len: 6,
        address: [9, 0, 0, 0, 0, 0],
    };
    let topology = [0; 47];
    let request = [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1];
    for (packet_type, source, seq, payload, origin) in [
        (15, 0, 1, &join[..], Some(&origin)),
        (14, 3, 2, &topology[..], None),
        (23, 3, 4, &request[..], None),
    ] {
        assert!(!enqueue_literal(
            &mut leader,
            packet_type,
            source,
            seq,
            payload,
            origin
        ));
        assert!(enqueue_literal(
            &mut relay,
            packet_type,
            source,
            seq,
            payload,
            origin
        ));
    }
    for (packet_type, seq, payload) in [(14, 3, &topology[..]), (23, 5, &request[..])] {
        let before = relay.snapshot();
        assert!(!enqueue_literal(
            &mut relay,
            packet_type,
            1,
            seq,
            payload,
            None
        ));
        assert_eq!(relay.snapshot(), before);
    }
    let ack = [1, 2, 2, 0, 0, 0, 160, 0, 0, 0, 3];
    assert!(!enqueue_literal(&mut leader, 19, 3, 6, &ack, None));
    assert_eq!(leader.forward_queue_len(), 0);
    assert_eq!(relay.forward_queue_len(), 3);
}

#[test]
fn forwarded_sync_has_exact_clock_depth_count_and_admission_rejections_are_inert() {
    let (mut state, _) = membership_fixture(2);
    // identity, leader ID, member count, term, frame, phase, depth
    let sync = [6, 1, 0, 0, 0, 0, 0, 1, 2, 1, 0, 0, 0, 123, 0, 0, 0, 0, 0, 0];
    for count in [0, 9] {
        let mut bad = sync;
        bad[8] = count;
        let before = state.snapshot();
        assert!(!enqueue_literal(&mut state, 17, 1, 10, &bad, None));
        assert_eq!(state.snapshot(), before);
    }
    assert!(enqueue_literal(&mut state, 17, 1, 10, &sync, None));
    let mut out = ForwardPacket::default();
    assert!(forward_dequeue(
        Some(&mut state),
        Some(&mut out),
        0,
        19,
        400
    ));
    assert_eq!(
        out.header,
        Header {
            version: 5,
            packet_type: 17,
            src_id: 1,
            seq: 10,
            ttl: 1,
            flags: 2,
            payload_len: 20,
            ..Header::default()
        }
    );
    assert_eq!(out.payload_len, 20);
    assert_eq!(
        &out.payload[..20],
        &[6, 1, 0, 0, 0, 0, 0, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 19, 0, 1]
    );
    assert_eq!(state.forward_queue_len(), 0);
    for (ttl, flags) in [(0, 0), (1, 0), (3, 0), (2, 2)] {
        let header = Header {
            ttl,
            flags,
            seq: 11,
            ..out.header
        };
        let before = state.snapshot();
        assert!(!forward_enqueue(
            Some(&mut state),
            Some(&header),
            Some(&sync),
            None,
            1,
            true,
            101
        ));
        assert_eq!(state.snapshot(), before);
    }
    let before = state.snapshot();
    assert!(!forward_enqueue(
        Some(&mut state),
        Some(&out.header),
        Some(&out.payload[..20]),
        None,
        1,
        true,
        101
    ));
    assert_eq!(state.snapshot(), before);

    // Canonical two-member authoritative snapshot: header fields, 14-byte map,
    // then eight seven-byte identities. Unused identities and slot tail are zero.
    assert!(update_members(Some(&mut state), 3));
    let mut membership = [0; 79];
    membership[..12].copy_from_slice(&[1, 0, 0, 0, 1, 0, 0, 0, 1, 2, 1, 2]);
    membership[23..30].copy_from_slice(&[6, 1, 0, 0, 0, 0, 0]);
    membership[30..37].copy_from_slice(&[6, 2, 0, 0, 0, 0, 0]);
    let before = state.snapshot();
    assert!(!enqueue_literal(&mut state, 22, 2, 12, &membership, None));
    assert_eq!(state.snapshot(), before);
    assert!(enqueue_literal(&mut state, 22, 1, 12, &membership, None));
    assert!(forward_dequeue(Some(&mut state), Some(&mut out), 10, 0, 0));
    assert_eq!(out.header.packet_type, 22);
    assert_eq!(out.header.src_id, 1);
    assert_eq!(out.header.ttl, 1);
    assert_eq!(out.header.flags, 2);
    assert_eq!(out.payload_len, 79);
    assert_eq!(&out.payload[..79], &membership);
    assert!(!forward_enqueue(
        Some(&mut state),
        Some(&out.header),
        Some(&out.payload[..79]),
        None,
        1,
        true,
        102
    ));
    assert_eq!(state.forward_queue_len(), 0);
}

#[test]
fn identity_requires_bounded_nonreserved_address_and_zero_padding() {
    assert!(!identity_valid(None));
    for (length, address, valid) in [
        (0, [1, 0, 0, 0, 0, 0], false),
        (1, [1, 0, 0, 0, 0, 0], true),
        (6, [1, 2, 3, 4, 5, 6], true),
        (7, [1, 2, 3, 4, 5, 6], false),
        (255, [1, 2, 3, 4, 5, 6], false),
        (6, [0; 6], false),
        (6, [255; 6], false),
        (1, [255, 0, 0, 0, 0, 0], false),
        (1, [1, 2, 0, 0, 0, 0], false),
    ] {
        assert_eq!(
            identity_valid(Some(&Identity {
                address_len: length,
                address
            })),
            valid
        );
    }
}

#[test]
fn reinitialization_clears_old_session_and_only_leader_allocates_revisions() {
    let mut state = Adaptive::default();
    init(Some(&mut state), 1, 1, 7, 3, 123);
    assert_eq!(next_membership_revision(Some(&mut state)), 1);
    assert_eq!(next_membership_revision(Some(&mut state)), 2);
    init(Some(&mut state), 2, 1, 8, 3, 456);
    let snapshot = state.snapshot();
    assert_eq!(
        (
            snapshot.local_id,
            snapshot.leader_id,
            snapshot.term,
            snapshot.members
        ),
        (2, 1, 8, 3)
    );
    assert_eq!(snapshot.leader_since_ms, 456);
    assert_eq!(snapshot.publish_revision, 0);
    assert!(!snapshot.nodes[0].present);
    assert!(snapshot.nodes[1].present);
    assert_eq!(snapshot.nodes[1].last_seen_ms, 456);
    assert_eq!(next_membership_revision(Some(&mut state)), 0);
    assert_eq!(state.snapshot(), snapshot);
    init(Some(&mut state), 1, 1, 0, 1, 1);
    assert_eq!(next_membership_revision(Some(&mut state)), 0);
    for (local, leader, members) in [
        (0, 1, 3),
        (9, 1, 3),
        (1, 0, 3),
        (1, 9, 3),
        (1, 2, 1),
        (2, 1, 1),
    ] {
        init(Some(&mut state), 1, 1, 7, 3, 123);
        init(Some(&mut state), local, leader, 7, members, 999);
        assert_eq!(state.snapshot(), Snapshot::default());
    }
    assert_eq!(next_membership_revision(None), 0);
}

#[test]
fn control_authority_distinguishes_join_member_and_leader_packets() {
    let mut state = Adaptive::default();
    init(Some(&mut state), 2, 1, 7, 3, 0);
    for packet in 0..=255 {
        for source in [0, 1, 2, 3, 9, 255] {
            let allowed = match packet {
                15 => matches!(source, 0..=2),
                14 | 19 | 23 => matches!(source, 1 | 2),
                6 | 9 | 10 | 16 | 17 | 18 | 20 | 21 | 22 => source == 1,
                _ => false,
            };
            assert_eq!(
                control_authorized(Some(&state), packet, source),
                allowed,
                "packet={packet} source={source}"
            );
            assert!(!control_authorized(None, packet, source));
        }
    }
}

#[test]
fn sync_stamping_enforces_phase_and_single_fresh_relay_without_partial_writes() {
    let initial = Sync {
        leader: Identity {
            address_len: 1,
            address: [1, 0, 0, 0, 0, 0],
        },
        leader_id: 1,
        member_count: 2,
        term: 7,
        frame_counter: 123,
        phase_us: 456,
        relay_depth: 0,
    };
    for (relay, phase, age, accepted) in [
        (false, 19999, 999, true),
        (false, 20000, 0, false),
        (true, 19999, 600, true),
        (true, 0, 601, false),
    ] {
        let mut value = initial;
        assert_eq!(
            stamp_sync(Some(&mut value), relay, 999, phase, age),
            accepted
        );
        if accepted {
            assert_eq!(
                value,
                Sync {
                    frame_counter: 999,
                    phase_us: phase,
                    relay_depth: u8::from(relay),
                    ..initial
                }
            );
        } else {
            assert_eq!(value, initial);
        }
    }
    for bad in [
        Sync {
            leader_id: 0,
            ..initial
        },
        Sync {
            member_count: 0,
            ..initial
        },
        Sync {
            member_count: 9,
            ..initial
        },
        Sync { term: 0, ..initial },
        Sync {
            relay_depth: 1,
            ..initial
        },
        Sync {
            leader: Identity::default(),
            ..initial
        },
    ] {
        let mut value = bad;
        assert!(!stamp_sync(Some(&mut value), true, 999, 0, 0));
        assert_eq!(value, bad);
    }
    assert!(!stamp_sync(None, false, 0, 0, 0));
}
