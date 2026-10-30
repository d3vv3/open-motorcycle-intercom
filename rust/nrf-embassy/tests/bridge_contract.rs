//! Requirements for the nRF SPI bridge owner: queue priority and bounds, sequence
//! accounting, keepalive cadence, ingress validation/dedupe and command dispatch.
use nrf_embassy::bridge::{Bridge, Consumer, Entry, Prepared, Snapshot, EAGAIN, EINVAL, EMSGSIZE};
use openhelmet_wire::bridge as frame;
use std::fmt::Write as _;

#[derive(Default)]
struct Sink {
    trace: String,
    admission: i32,
}

impl Consumer for Sink {
    fn accept_audio_v2(&mut self, payload: &[u8]) -> i32 {
        write!(self.trace, " AUDIO{}:{}", self.admission, payload.len()).unwrap();
        self.admission
    }
    fn request_start(&mut self, generation: u8, group: u8) {
        write!(self.trace, " START{generation},{group}").unwrap();
    }
    fn request_stop(&mut self, generation: u8) {
        write!(self.trace, " STOP{generation}").unwrap();
    }
    fn request_pause(&mut self, generation: u8) {
        write!(self.trace, " PAUSE{generation}").unwrap();
    }
    fn request_resume(&mut self, generation: u8) {
        write!(self.trace, " RESUME{generation}").unwrap();
    }
    fn request_status(&mut self) {
        self.trace.push_str(" STATUS");
    }
    fn pulse_ack(&mut self) {
        self.trace.push_str(" ACK");
    }
}

fn bundle(seed: u8, predecessors: usize) -> Vec<u8> {
    let flags = [1, 7, 31][predecessors];
    let mut bytes = vec![
        2,
        20,
        seed,
        flags,
        255,
        seed,
        48,
        if predecessors == 0 { 0 } else { 48 },
    ];
    bytes.extend((0..48 * (predecessors + 1)).map(|i| seed.wrapping_add(i as u8)));
    bytes
}

fn rx(seq: u8, packet_type: u8, payload: &[u8]) -> [u8; 256] {
    let mut bytes = [0; 256];
    assert_ne!(frame::encode(&mut bytes, 253, seq, packet_type, payload), 0);
    bytes
}

const IDLE: [u8; 256] = [0; 256];

/// A new owner announces itself with one status frame (sequence 0); drain it.
fn fresh(sink: &mut Sink) -> Bridge {
    let mut bridge = Bridge::new();
    let (kind, seq, payload) = send_idle(&mut bridge, sink).unwrap();
    assert_eq!((kind, seq, payload[3]), (2, 0, 6));
    bridge
}

fn decode(token: &Prepared<'_>) -> (u8, u8, Vec<u8>) {
    let view = frame::decode(token.bytes(), 208).unwrap();
    (view.frame_type, view.seq, view.payload.to_vec())
}

fn send_idle(bridge: &mut Bridge, sink: &mut Sink) -> Option<(u8, u8, Vec<u8>)> {
    let token = bridge.prepare();
    let out = frame::decode(token.bytes(), 208)
        .ok()
        .map(|v| (v.frame_type, v.seq, v.payload.to_vec()));
    token.finish(true, &IDLE, sink);
    out
}

#[test]
fn control_precedes_audio_and_each_ring_drains_fifo_with_one_shared_sequence() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    bridge.send_audio_v2(1, &bundle(10, 0)).unwrap();
    bridge.send_audio_v2(2, &bundle(11, 1)).unwrap();
    bridge.send_log(b"abc").unwrap();
    bridge.send_event(8, &[9]).unwrap();
    bridge.send_audio_v2(3, &bundle(12, 2)).unwrap();
    // Sequence is allocated at enqueue in call order, selection is control-first.
    let expected = [
        (5, 3, b"abc".to_vec()),
        (3, 4, vec![8, 9]),
        (6, 1, [vec![1], bundle(10, 0)].concat()),
        (6, 2, [vec![2], bundle(11, 1)].concat()),
        (6, 5, [vec![3], bundle(12, 2)].concat()),
    ];
    for want in expected {
        assert_eq!(send_idle(&mut bridge, &mut sink), Some(want));
    }
    assert_eq!(send_idle(&mut bridge, &mut sink), None);
    let s = bridge.snapshot();
    assert_eq!(
        (s.tx_control, s.tx_audio, s.tx_idle, s.tx_fail),
        (3, 3, 1, 0)
    );
    assert_eq!(s.tx_seq, 6);
    assert!(sink.trace.is_empty());
}

#[test]
fn rings_overwrite_oldest_and_count_status_drops() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    for i in 0..20u8 {
        bridge.send_audio_v2(i, &bundle(i, 0)).unwrap();
    }
    // 16-slot ring holds 15 entries: five oldest overwritten.
    assert_eq!(bridge.snapshot().audio_overwrite, 5);
    let mut sources = vec![];
    while let Some((6, _, payload)) = send_idle(&mut bridge, &mut sink) {
        sources.push(payload[0]);
    }
    assert_eq!(sources, (5..20).collect::<Vec<u8>>());

    bridge.send_status(3, 2, 255, 2, -1, 1).unwrap();
    bridge.send_log(b"x").unwrap();
    bridge.send_event(1, &[]).unwrap();
    bridge.send_log(b"y").unwrap(); // 4-slot ring holds 3: status dropped
    let s = bridge.snapshot();
    // Boot status plus this one enqueued; the boot one already went out.
    assert_eq!(
        (s.status_enqueue, s.status_drop, s.control_overwrite),
        (2, 1, 1)
    );
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().2, b"x");
    // Ring now [event, log y]; two statuses and a log overwrite event then log y:
    // non-status overwrites never count as status drops.
    bridge.send_status(0, 0, 0, 0, 0, 0).unwrap();
    bridge.send_status(0, 0, 0, 0, 0, 0).unwrap();
    bridge.send_log(b"z").unwrap();
    let s = bridge.snapshot();
    assert_eq!(
        (s.status_enqueue, s.status_drop, s.control_overwrite),
        (4, 1, 3)
    );
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().0, 2);
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().0, 2);
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().2, b"z");
}

#[test]
fn invalid_sends_consume_no_sequence_and_status_payload_is_fixed_layout() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    assert_eq!(bridge.send_audio_v2(2, &[]), Err(EINVAL));
    assert_eq!(bridge.send_audio_v2(2, &[2, 20]), Err(EINVAL));
    assert_eq!(bridge.send_event(2, &[0; 208]), Err(EMSGSIZE));
    assert_eq!(bridge.send_log(&[]), Err(EINVAL));
    assert_eq!(bridge.snapshot().tx_seq, 1);
    assert_eq!(
        bridge.snapshot().control_head,
        bridge.snapshot().control_tail
    );
    bridge.send_event(3, &[0xa5; 207]).unwrap();
    bridge.send_log(&[0xff; 255]).unwrap(); // truncated to the payload limit
    bridge.send_status(3, 2, 255, 2, -1, 1).unwrap();
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().2.len(), 208);
    assert_eq!(
        send_idle(&mut bridge, &mut sink).unwrap().2,
        vec![0xff; 208]
    );
    assert_eq!(
        send_idle(&mut bridge, &mut sink).unwrap().2,
        vec![2, 255, 2, 6, 3, 0xff, 1, 0xa5, 2, 20]
    );
    assert_eq!(bridge.snapshot().tx_seq, 4);
}

#[test]
fn sequence_wraps_and_failed_transfers_retain_selection_without_rx_parsing() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    for _ in 0..254 {
        bridge.send_log(b"k").unwrap();
        send_idle(&mut bridge, &mut sink);
    }
    bridge.send_log(b"w").unwrap();
    let token = bridge.prepare();
    assert_eq!(decode(&token).1, 255);
    token.finish(false, &rx(1, 4, &[1, 9, 1]), &mut sink);
    assert!(
        !sink.trace.contains("START"),
        "failed transfer must not decode RX"
    );
    let s = bridge.snapshot();
    assert_eq!((s.tx_fail, s.tx_control, s.attempts), (1, 255, 256));
    assert_eq!(decode(&bridge.prepare()).1, 255);
    bridge.send_log(b"n").unwrap();
    assert_eq!(bridge.snapshot().tx_seq, 1);
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().1, 255);
    assert_eq!(send_idle(&mut bridge, &mut sink).unwrap().1, 0);
}

#[test]
fn keepalive_requests_status_every_200_attempts_including_failures_and_idle() {
    let mut bridge = Bridge::new();
    let mut sink = Sink::default();
    for i in 0..399 {
        bridge.prepare().finish(i % 2 == 0, &IDLE, &mut sink);
        assert_eq!(sink.trace.matches("STATUS").count(), usize::from(i >= 199));
    }
    bridge.prepare().finish(true, &IDLE, &mut sink);
    assert_eq!(sink.trace, " STATUS STATUS");
    assert_eq!(bridge.snapshot().keepalive, 0);
}

#[test]
fn ingress_tracks_sequence_gaps_crc_and_dedupes_audio_by_sequence() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    let audio = bundle(7, 2);
    // Admission failure counts reject, no ack, no dedupe anchor.
    sink.admission = -11;
    bridge
        .prepare()
        .finish(true, &rx(254, 6, &audio), &mut sink);
    sink.admission = 0;
    bridge
        .prepare()
        .finish(true, &rx(254, 6, &audio), &mut sink);
    bridge
        .prepare()
        .finish(true, &rx(254, 6, &audio), &mut sink); // duplicate: acked, not delivered
    bridge
        .prepare()
        .finish(true, &rx(255, 6, &audio), &mut sink);
    bridge.prepare().finish(true, &rx(0, 6, &audio), &mut sink); // wrap, in order
    bridge.prepare().finish(true, &rx(5, 6, &audio), &mut sink); // gap
    bridge.prepare().finish(true, &rx(5, 1, &audio), &mut sink); // legacy audio rejected
    let mut bad = rx(6, 6, &audio);
    bad[10] ^= 0xff;
    bridge.prepare().finish(true, &bad, &mut sink);
    let s = bridge.snapshot();
    assert_eq!(
        (
            s.ingress_ok,
            s.ingress_reject,
            s.ingress_duplicate,
            s.ack_pulses
        ),
        (4, 2, 1, 5)
    );
    assert_eq!((s.rx_seq_gap, s.rx_crc_fail, s.rx_seq), (1, 1, 5));
    assert_eq!(
        sink.trace,
        " AUDIO-11:152 AUDIO0:152 ACK ACK AUDIO0:152 ACK AUDIO0:152 ACK AUDIO0:152 ACK"
    );
    // Delivery gating flushes pending audio and the dedupe anchor.
    bridge.send_audio_v2(1, &audio).unwrap();
    bridge.set_audio_delivery_enabled(false);
    assert!(!bridge.snapshot().last_audio_seq_valid);
    assert_eq!(bridge.send_audio_v2(1, &audio), Err(EAGAIN));
    assert_eq!(send_idle(&mut bridge, &mut sink), None);
    sink.trace.clear();
    bridge.prepare().finish(true, &rx(5, 6, &audio), &mut sink);
    assert_eq!(
        sink.trace, " AUDIO0:152 ACK",
        "same seq accepted after flush"
    );
}

#[test]
fn commands_dispatch_only_well_formed_requests() {
    let mut sink = Sink::default();
    let mut bridge = fresh(&mut sink);
    let cases: &[(&[u8], &str)] = &[
        (&[3], " STATUS"),
        (&[1, 9, 1], " START9,1"),
        (&[1, 9, 3], " START9,3"),
        (&[1, 9, 0], ""),
        (&[1, 9, 4], ""),
        (&[1, 9], ""),
        (&[2, 7, 0], " STOP7"),
        (&[4, 6, 0], " PAUSE6"),
        (&[5, 5, 99], " RESUME5"),
        (&[6, 1, 1], ""),
        (&[], ""),
    ];
    for (i, (payload, want)) in cases.iter().enumerate() {
        sink.trace.clear();
        bridge
            .prepare()
            .finish(true, &rx(i as u8, 4, payload), &mut sink);
        assert_eq!(sink.trace, *want, "{payload:?}");
    }
    assert_eq!(bridge.snapshot().rx_seq_gap, 0);
}

fn finish_idle(bridge: &mut Bridge, sink: &mut Sink) {
    bridge.prepare().finish(true, &[0; 256], sink);
}

#[test]
fn prepared_drop_retains_selection_and_does_not_count_poll() {
    let mut bridge = Bridge::new();
    let before = bridge.snapshot();
    let selected = *bridge.prepare().bytes();
    {
        let token = bridge.prepare();
        assert_eq!(*token.bytes(), selected);
        // No finish: ownership is released without dequeue or keepalive progress.
    }
    assert_eq!(bridge.snapshot(), before);
    assert_eq!(*bridge.prepare().bytes(), selected);
    let mut sink = Sink::default();
    bridge.prepare().finish(false, &rx(7, 4, &[3]), &mut sink);
    assert!(sink.trace.is_empty());
    assert_eq!(*bridge.prepare().bytes(), selected);
    finish_idle(&mut bridge, &mut sink);
    assert_eq!(bridge.snapshot().status_tx, 1);
    assert_eq!(*bridge.prepare().bytes(), [0; 256]);
}

#[test]
fn api_bounds_gate_and_wire_payloads() {
    let mut bridge = Bridge::new();
    let mut sink = Sink::default();
    assert_eq!(
        frame::decode(bridge.prepare().bytes(), 208)
            .unwrap()
            .payload,
        &[0, 0, 0, 6, 0, 255, 0, 165, 2, 20]
    );
    finish_idle(&mut bridge, &mut sink);
    assert_eq!(bridge.send_event(8, &[0; 208]), Err(EMSGSIZE));
    assert_eq!(bridge.send_log(&[]), Err(EINVAL));
    assert_eq!(bridge.send_audio_v2(1, &[]), Err(EINVAL));
    let audio = bundle(0, 2);
    bridge.set_audio_delivery_enabled(false);
    assert_eq!(bridge.send_audio_v2(1, &audio), Err(EAGAIN));
    bridge.set_audio_delivery_enabled(true);
    bridge.send_audio_v2(77, &audio).unwrap();
    bridge.send_command_ack(5, 255, -19).unwrap();
    {
        let token = bridge.prepare();
        let view = frame::decode(token.bytes(), 208).unwrap();
        assert_eq!(view.frame_type, 3);
        assert_eq!(view.payload, &[7, 5, 255, 255]);
        token.finish(true, &[0; 256], &mut sink);
    }
    {
        let token = bridge.prepare();
        let view = frame::decode(token.bytes(), 208).unwrap();
        assert_eq!(view.frame_type, 6);
        assert_eq!(view.payload[0], 77);
        assert_eq!(&view.payload[1..], audio);
    }
    bridge.set_audio_delivery_enabled(true);
    assert_eq!(*bridge.prepare().bytes(), [0; 256]);
    eprintln!(
        "bridge host storage: owner={} prepared={} entry={} snapshot={} bytes; not ARM stack qualification",
        std::mem::size_of::<Bridge>(),
        std::mem::size_of::<Prepared<'_>>(),
        std::mem::size_of::<Entry>(),
        std::mem::size_of::<Snapshot>()
    );
}
