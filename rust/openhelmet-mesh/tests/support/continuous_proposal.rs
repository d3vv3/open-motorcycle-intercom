use super::driver::*;
use openhelmet_mesh::control_admission::JoinContext;
fn identity(id: u8) -> CIdentity {
    CIdentity {
        address_len: 6,
        address: [id; 6],
    }
}
pub unsafe fn prepare(participant: *mut CAdaptive) {
    unsafe {
        let mut a = core::mem::MaybeUninit::<CAdaptive>::uninit();
        let leader = a.as_mut_ptr();
        for (s, local) in [(leader, 1), (participant, 3)] {
            mesh_adaptive_reset(s);
            mesh_adaptive_init(s, local, 1, 7, 7, 0);
            for id in 1..=3 {
                assert!(mesh_adaptive_set_member(s, id, &identity(id)));
            }
            mesh_adaptive_publish_active(s, true);
        }
        let map = CSlots {
            slot_count: 3,
            slot_ids: [1, 2, 3, 0, 0, 0, 0, 0],
            ..Default::default()
        };
        let membership = CMembership {
            term: 7,
            revision: 50000,
            leader_id: 1,
            slot_map: map,
            identities: [
                identity(1),
                identity(2),
                identity(3),
                CIdentity::default(),
                CIdentity::default(),
                CIdentity::default(),
                CIdentity::default(),
                CIdentity::default(),
            ],
        };
        let mut h = CHeader {
            packet_type: 0x16,
            src_id: 1,
            ttl: 2,
            payload_len: 79,
            ..Default::default()
        };
        let context = JoinContext {
            state: 1,
            ..Default::default()
        };
        assert_eq!(
            mesh_membership_receive(
                participant,
                &context,
                &h,
                &[1; 6],
                (&membership as *const CMembership).cast(),
                1,
                &Default::default()
            )
            .action,
            1
        );
        for now in (100..=14100).step_by(100) {
            for s in [leader, participant] {
                for id in 1..=3 {
                    // Candidate 2 has strong symmetric links; leader's link to 3 is weak.
                    let quality = match id {
                        1 => [0, 90, 30, 0, 0, 0, 0, 0],
                        2 => [90, 0, 90, 0, 0, 0, 0, 0],
                        _ => [30, 90, 0, 0, 0, 0, 0, 0],
                    };
                    let report = CTopology {
                        origin: identity(id),
                        report_seq: now,
                        direct_mask: 7 ^ (1 << (id - 1)),
                        leader_id: 1,
                        term: 7,
                        rssi_dbm: [-55; 8],
                        quality,
                        ..Default::default()
                    };
                    assert!(mesh_adaptive_report(s, id, &report, false, now));
                }
            }
            let candidate = mesh_adaptive_candidate(leader, now);
            if now == 14100 {
                assert_eq!(candidate, 2);
            } else if now <= 12900 {
                assert_eq!(candidate, 0);
            }
        }
        let mut proposal = CHandover::default();
        assert!(mesh_adaptive_prepare(
            leader,
            2,
            700,
            &map,
            14100,
            &mut proposal
        ));
        h = CHeader {
            packet_type: 0x12,
            src_id: 1,
            ttl: 2,
            payload_len: 25,
            ..Default::default()
        };
        let prepared = mesh_control_receive(
            participant,
            &h,
            &[1; 6],
            (&proposal as *const CHandover).cast(),
            700,
            14100,
        );
        assert_eq!(prepared.action, 4);
        h = CHeader {
            packet_type: 0x13,
            src_id: 3,
            ttl: 2,
            payload_len: 11,
            ..Default::default()
        };
        assert_eq!(
            mesh_control_receive(
                leader,
                &h,
                &[3; 6],
                (&prepared.ack as *const CAck).cast(),
                700,
                14101
            )
            .action,
            3
        );
        let ack = CAck {
            acknowledger_id: 2,
            ..prepared.ack
        };
        h.src_id = 2;
        assert_eq!(
            mesh_control_receive(
                leader,
                &h,
                &[2; 6],
                (&ack as *const CAck).cast(),
                700,
                14101
            )
            .action,
            3
        );
        let mut commit = CHandover::default();
        assert!(mesh_adaptive_commit(leader, &mut commit, 14102));
        h = CHeader {
            packet_type: 0x14,
            src_id: 1,
            ttl: 2,
            payload_len: 25,
            ..Default::default()
        };
        assert_eq!(
            mesh_control_receive(
                participant,
                &h,
                &[1; 6],
                (&commit as *const CHandover).cast(),
                700,
                14102
            )
            .action,
            3
        );
        // Keep proposal evidence fresh throughout the 160-frame switch window.
        for now in (14200..=17300).step_by(100) {
            for s in [leader, participant] {
                for id in 1..=3 {
                    let quality = match id {
                        1 => [0, 90, 30, 0, 0, 0, 0, 0],
                        2 => [90, 0, 90, 0, 0, 0, 0, 0],
                        _ => [30, 90, 0, 0, 0, 0, 0, 0],
                    };
                    let report = CTopology {
                        origin: identity(id),
                        report_seq: now,
                        direct_mask: 7 ^ (1 << (id - 1)),
                        leader_id: 1,
                        term: 7,
                        rssi_dbm: [-55; 8],
                        quality,
                        ..Default::default()
                    };
                    assert!(mesh_adaptive_report(s, id, &report, false, now));
                }
            }
        }
    }
}
