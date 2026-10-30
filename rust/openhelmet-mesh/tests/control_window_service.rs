//! Deterministic transaction traces independent of the C source extractor.
use openhelmet_mesh::{
    control_queue::Item,
    control_window::{self as cw, operation as op},
};

#[derive(Default)]
struct Run {
    trace: Vec<(u32, i64)>,
    busy: bool,
    timeout: bool,
    stale: bool,
    coordinator: bool,
    grace: bool,
    failed_send: bool,
    relayed: bool,
    sync: bool,
    authority_stale: bool,
    bad_len: bool,
    now: i64,
    deadline: i64,
    advance: i64,
    credit: Option<u16>,
    sequence: Option<u8>,
}
impl Run {
    fn execute(&mut self) {
        let mut waits = 0;
        let mut snapshots = 0;
        cw::service(
            |operation, value, dto| {
                self.trace.push((operation, value));
                match operation {
                    op::SNAPSHOT => {
                        let role = if self.coordinator { 1 } else { 2 };
                        let local = if self.coordinator { 1 } else { 2 };
                        // All pointers are initialized detached operation DTOs.
                        let s = unsafe { &mut *dto.cast::<cw::Snapshot>() };
                        s.start = 1000;
                        s.deadline = self.deadline;
                        s.identity = cw::Context {
                            control_generation: 7,
                            tdma_generation: 7,
                            lifecycle: 3 + u32::from(self.stale && snapshots > 0),
                            session: 9,
                            term: 1,
                            local,
                            coordinator: 1,
                            owner_local: local,
                            leader: 1,
                            role,
                            slot: 1,
                            active: true,
                            ..Default::default()
                        };
                        snapshots += 1;
                        1
                    }
                    op::CLOCK => self.now,
                    op::FRAME => {
                        if self.coordinator {
                            10
                        } else {
                            1
                        }
                    }
                    op::ROLE => {
                        if self.coordinator {
                            1
                        } else {
                            2
                        }
                    }
                    op::SLOT => 1,
                    op::GRACE => {
                        let g = unsafe { &mut *dto.cast::<cw::Grace>() };
                        *g = cw::Grace {
                            local: 1,
                            leader: 1,
                            transitioned: u8::from(self.grace),
                            deadline: 2,
                        };
                        1
                    }
                    op::IDLE => {
                        waits += 1;
                        if self.busy && waits == 1 {
                            0
                        } else if self.busy && waits == 2 {
                            self.now = self.now.wrapping_add(self.advance);
                            // Callback is free to execute another owner export: no
                            // queue/adaptive borrow spans a platform callback.
                            let mut queue = openhelmet_mesh::control_queue::Queue::new();
                            openhelmet_mesh::control_queue::reset(&mut queue);
                            i64::from(!self.timeout)
                        } else {
                            1
                        }
                    }
                    op::DEQUEUE => {
                        let item = unsafe { &mut *dto.cast::<Item>() };
                        item.kind = if self.sync { 0x11 } else { 7 };
                        item.len = if self.bad_len {
                            250
                        } else if self.sync {
                            29
                        } else {
                            17
                        };
                        item.data[2] = 2;
                        item.data[5] = if self.relayed { 2 } else { 0 };
                        item.data[14] = 2;
                        item.data[15] = 4;
                        item.data[7] = if self.sync { 20 } else { 8 };
                        item.data[16] = 1;
                        item.data[18] = 1;
                        item.enqueued_us = self.now - 1_000_001;
                        1
                    }
                    op::AUTHORITY => {
                        let a = unsafe { &mut *dto.cast::<cw::Authority>() };
                        *a = cw::Authority {
                            term: if self.authority_stale { 2 } else { 1 },
                            leader: 1,
                        };
                        1
                    }
                    op::LOCAL => 2,
                    op::SEND => {
                        let s = unsafe { &*dto.cast::<cw::Submission>() };
                        assert_eq!(s.deadline, self.deadline);
                        assert!(!s.item.is_null());
                        self.credit = Some(s.credit);
                        self.sequence = Some(s.sequence);
                        i64::from(self.failed_send)
                    }
                    op::SYNC => {
                        assert_eq!(value, self.deadline);
                        i64::from(self.failed_send)
                    }
                    op::RESTORE | op::STAT => 0,
                    _ => panic!("unexpected operation {operation}"),
                }
            },
            8,
            10,
        );
    }
    fn new() -> Self {
        Self {
            now: 1000,
            deadline: 2600,
            ..Default::default()
        }
    }
    fn ops(&self) -> Vec<u32> {
        self.trace.iter().map(|x| x.0).collect()
    }
}

#[test]
fn exact_queued_and_failed_status_traces() {
    let mut r = Run::new();
    r.execute();
    let base = vec![
        op::SNAPSHOT,
        op::CLOCK,
        op::FRAME,
        op::ROLE,
        op::SLOT,
        op::ROLE,
        op::ROLE,
        op::SLOT,
        op::IDLE,
        op::SNAPSHOT,
        op::FRAME,
        op::ROLE,
        op::SLOT,
        op::CLOCK,
        op::IDLE,
        op::DEQUEUE,
        op::LOCAL,
        op::SEND,
    ];
    assert_eq!(r.ops(), base);
    assert_eq!((r.credit, r.sequence), (Some(0x402), Some(1)));
    let mut r = Run {
        failed_send: true,
        ..Run::new()
    };
    r.execute();
    assert_eq!(
        r.ops(),
        [base, vec![op::RESTORE, op::STAT, op::STAT]].concat()
    );
    assert_eq!(
        &r.trace[r.trace.len() - 2..],
        &[(op::STAT, 0), (op::STAT, 1)]
    );
}

#[test]
fn busy_continuation_clocks_equality_and_budget() {
    for (advance, sent) in [(1600, true), (1601, false)] {
        let mut r = Run {
            busy: true,
            advance,
            ..Run::new()
        };
        r.execute();
        assert!(r.trace.contains(&(op::IDLE, 2)));
        assert_eq!(r.ops().contains(&op::SEND), sent);
    }
    let mut r = Run {
        busy: true,
        stale: true,
        ..Run::new()
    };
    r.execute();
    assert_eq!(r.ops().last(), Some(&op::SNAPSHOT));
    assert_eq!(r.ops().iter().filter(|&&x| x == op::CLOCK).count(), 3);
    assert!(!r.ops().contains(&op::DEQUEUE));
    let mut r = Run {
        busy: true,
        timeout: true,
        coordinator: true,
        ..Run::new()
    };
    r.execute();
    assert_eq!(
        &r.trace[r.trace.len() - 2..],
        &[(op::IDLE, 2), (op::STAT, 0)]
    );
    assert_eq!(r.ops().iter().filter(|&&x| x == op::CLOCK).count(), 2);
    for grace in [false, true] {
        let mut r = Run {
            coordinator: true,
            grace,
            ..Run::new()
        };
        r.execute();
        assert_eq!(
            r.ops().iter().filter(|&&x| x == op::CLOCK).count(),
            2 + usize::from(grace)
        );
        assert_eq!(r.ops().last(), Some(&op::SYNC));
        assert!(!r.ops().contains(&op::DEQUEUE));
    }
}

#[test]
fn destructive_dequeue_before_live_relay_checks_and_bad_lengths() {
    let mut r = Run {
        relayed: true,
        sync: true,
        authority_stale: true,
        ..Run::new()
    };
    r.execute();
    assert_eq!(
        &r.ops()[r.trace.len() - 3..],
        &[op::DEQUEUE, op::AUTHORITY, op::STAT]
    );
    let mut r = Run {
        relayed: true,
        ..Run::new()
    };
    r.execute();
    assert_eq!(
        &r.ops()[r.trace.len() - 3..],
        &[op::DEQUEUE, op::CLOCK, op::STAT]
    );
    for bad_len in [true, false] {
        let mut r = Run {
            bad_len,
            ..Run::new()
        };
        r.execute();
        assert_eq!(r.ops().contains(&op::SEND), !bad_len);
    }
    let mut r = Run {
        now: 999,
        ..Run::new()
    };
    r.execute();
    assert_eq!(r.ops(), vec![op::SNAPSHOT, op::CLOCK]);
}
