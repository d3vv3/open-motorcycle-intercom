use openhelmet_mesh::{
    audio_route::{self as p, Executor, Packet, effect::*},
    tx_slot::SendResult,
};
struct Fake {
    trace: Vec<u8>,
    local: bool,
    relay: bool,
    busy: bool,
    success: bool,
    gate: u8,
    malformed: bool,
    consumed: u8,
}
impl Fake {
    fn new() -> Self {
        Self {
            trace: vec![],
            local: true,
            relay: true,
            busy: false,
            success: true,
            gate: 0,
            malformed: false,
            consumed: 0,
        }
    }
}
impl Executor for Fake {
    fn effect(&mut self, op: u8, value: i64, packet: Option<&mut Packet>) -> i64 {
        self.trace.push(op);
        match op {
            IDLE => (!self.busy) as i64,
            ENABLED => {
                self.gate += 1;
                (self.gate < 4) as i64
            }
            LOCAL => {
                let packet = packet.unwrap();
                packet.len = if self.malformed { 65 } else { 2 };
                packet.data[12] = 1;
                packet.data[13..15].copy_from_slice(&[7, 8]);
                self.local as i64
            }
            META => 1 | (7 << 8) | (1 << 16),
            SPEAKERS => 1,
            SUBMIT => {
                let packet = packet.unwrap();
                assert_eq!(value, 99);
                assert_eq!(
                    &packet.data[..15],
                    &[5, 1, 1, 0, 2, 5, 7, 6, 0, 1, 20, 1, 1, 7, 8]
                );
                assert_eq!(packet.len, 15);
                self.success as i64
            }
            LOCAL_CONSUME => {
                self.consumed += 1;
                0
            }
            COORDINATOR => 1,
            _ => 0,
        }
    }
    fn expire(&mut self, _: i64) -> u8 {
        self.trace.push(100);
        0
    }
    fn pending(&self) -> bool {
        self.relay
    }
    fn peek(&self, _: &mut Packet) -> bool {
        false
    }
    fn complete(&mut self) {
        panic!("local must not consume relay")
    }
}
#[test]
fn exact_success_and_failure_trace() {
    let mut e = Fake::new();
    assert_eq!(p::service(&mut e, 99), SendResult::Submitted);
    assert_eq!(e.consumed, 1);
    assert_eq!(
        e.trace,
        [
            IDLE,
            CLOCK,
            100,
            DROPPED,
            ENABLED,
            LOCAL,
            META,
            WARN,
            ENABLED,
            SPEAKERS,
            PREPARE,
            ENABLED,
            SUBMIT,
            TURN,
            LOCAL_CONSUME,
            LOCAL_ACTIVITY,
            COORDINATOR,
            GRANTS
        ]
    );
    let mut e = Fake::new();
    e.success = false;
    assert_eq!(p::service(&mut e, 99), SendResult::Error);
    assert_eq!(e.consumed, 0);
    assert_eq!(&e.trace[e.trace.len() - 2..], &[SUBMIT, TURN]);
}
#[test]
fn busy_precedes_clock_expiry_and_queue() {
    let mut e = Fake::new();
    e.busy = true;
    assert_eq!(p::service(&mut e, 99), SendResult::Busy);
    assert_eq!(e.trace, [IDLE, BUSY]);
}
#[test]
fn malformed_and_late_privacy_fail_closed() {
    let mut e = Fake::new();
    e.malformed = true;
    assert_eq!(p::service(&mut e, 99), SendResult::Error);
    assert!(!e.trace.contains(&SUBMIT));
    let mut e = Fake::new();
    e.gate = 1;
    assert_eq!(p::service(&mut e, 99), SendResult::Empty);
    assert!(!e.trace.contains(&SUBMIT));
    assert_eq!(e.consumed, 0);
}
