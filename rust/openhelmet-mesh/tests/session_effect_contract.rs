use openhelmet_mesh::{participant_session as session, removal};

#[test]
fn ordered_effects_and_stale_continuations() {
    let mut trace = Vec::new();
    assert!(session::retire_effects(|op| {
        trace.push(op);
        true
    }));
    assert_eq!(trace, [30, 31, 0, 32, 0, 33, 0, 34, 0]);
    for reject in [30, 33] {
        let mut trace = Vec::new();
        assert!(!session::retire_effects(|op| {
            trace.push(op);
            op != reject
        }));
        assert_eq!(trace.last(), Some(&reject));
        assert!(!trace.contains(&34));
    }
    for boundary in 1..=4 {
        let mut checks = 0;
        let mut trace = Vec::new();
        assert!(!session::retire_effects(|op| {
            trace.push(op);
            if op == 0 {
                checks += 1;
                checks != boundary
            } else {
                true
            }
        }));
        assert_eq!(trace.last(), Some(&0));
    }
    let mut trace = Vec::new();
    session::reopen_effects(|op| {
        trace.push(op);
        true
    });
    assert_eq!(trace, [0, 40, 0, 41, 0, 42, 0, 43]);
    for stale_after in [41, 42] {
        let mut stale = false;
        let mut trace = Vec::new();
        session::reopen_effects(|op| {
            trace.push(op);
            if op == stale_after {
                stale = true;
            }
            op != 0 || !stale
        });
        assert!(!trace.contains(&43));
    }
}

#[test]
fn destination_and_activation_guard_every_blocking_boundary_callback_last() {
    for activation in [false, true] {
        let expected: &[u32] = if activation {
            &[0, 60, 0, 61, 0, 62, 0, 63, 0, 64]
        } else {
            &[0, 49, 0, 50, 0, 51, 0, 0, 40, 0, 41, 0, 42, 0, 43]
        };
        for reset_at in
            core::iter::once(None).chain(expected.iter().copied().filter(|op| *op != 0).map(Some))
        {
            let mut trace = Vec::new();
            let mut stale = false;
            let mut effect = |op| {
                trace.push(op);
                if Some(op) == reset_at {
                    stale = true;
                }
                op != 0 || !stale
            };
            if activation {
                session::activation_effects(&mut effect);
            } else {
                session::destination_effects(&mut effect);
            }
            if let Some(op) = reset_at {
                let end = trace.iter().position(|value| *value == op).unwrap();
                assert!(trace[end + 1..].iter().all(|value| *value == 0));
            } else {
                assert_eq!(trace, expected);
            }
        }
    }
}

#[test]
fn entry_tracking_preserves_begin_cancel_order_and_single_episode() {
    for previous in 0..3 {
        for current in 0..3 {
            let mut trace = Vec::new();
            session::tracking_effects(|op| match op {
                70 => current == 0,
                71 => previous == 0,
                72 => current == 1,
                73 => previous == 1,
                _ => {
                    trace.push(op);
                    true
                }
            });
            let mut expected = Vec::new();
            if current == 0 && previous != 0 {
                expected.push(74);
            }
            if current == 1 && previous != 1 {
                expected.push(75);
            }
            if current != 0 && previous == 0 {
                expected.push(76);
            }
            if current != 1 && previous == 1 {
                expected.push(77);
            }
            expected.push(78);
            assert_eq!(trace, expected);
        }
    }
}

#[test]
fn removal_commit_cleanup_finishes_but_reentrant_publication_stops() {
    let mut trace = Vec::new();
    removal::committed_effects(|op| {
        trace.push(op);
        true
    });
    assert_eq!(trace, [1, 2, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8]);
    for reset_at in 1..=8 {
        let mut stale = false;
        let mut trace = Vec::new();
        removal::committed_effects(|op| {
            trace.push(op);
            if op == reset_at {
                stale = true;
            }
            op != 0 || !stale
        });
        assert_eq!(&trace[..3], [1, 2, 3]);
        if reset_at < 8 {
            assert!(!trace.contains(&8));
        }
        if (4..8).contains(&reset_at) {
            assert!(!trace.contains(&(reset_at + 1)));
        }
    }
}

#[test]
fn maintenance_admits_each_effect_and_never_continues_replaced_session() {
    let mut trace = Vec::new();
    session::maintenance_effects(|op| {
        trace.push(op);
        true
    });
    assert_eq!(
        trace,
        [
            0, 10, 11, 0, 12, 13, 14, 0, 15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20
        ]
    );
    for reset_at in [11, 14, 15, 16, 17, 18, 19, 20] {
        let mut stale = false;
        let mut trace = Vec::new();
        session::maintenance_effects(|op| {
            trace.push(op);
            if op == reset_at {
                stale = true;
            }
            op != 0 || !stale
        });
        let end = trace.iter().position(|op| *op == reset_at).unwrap();
        assert!(trace[end + 1..].iter().all(|op| *op == 0));
    }
}
