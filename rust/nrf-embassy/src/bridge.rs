//! Portable owner of the queues and RX policy in `nrf_mesh/src/uart_bridge.c`.
//! Hardware performs one fixed-size transfer while holding a [`Prepared`] token.
//! `detached-bridge-transfer` additionally supports a coordinator-owned attempt
//! whose bytes outlive the synchronous owner loan.

use openhelmet_wire::{audio_bundle::audio_bundle_parse, bridge as frame};

pub const TRANSFER_SIZE: usize = 256;
pub const MAX_PAYLOAD: usize = 208;
pub const EINVAL: i32 = -22;
pub const EMSGSIZE: i32 = -90;
pub const EAGAIN: i32 = -11;
pub type Result = core::result::Result<(), i32>;

const AUDIO: u8 = 1;
const STATUS: u8 = 2;
const EVENT: u8 = 3;
const COMMAND: u8 = 4;
const LOG: u8 = 5;
const AUDIO_V2: u8 = 6;

/// External admission/command publication. Implementations must not borrow the
/// bridge: runtime dispatch and response enqueue happen after `finish` returns.
pub trait Consumer {
    fn accept_audio_v2(&mut self, payload: &[u8]) -> i32;
    fn request_start(&mut self, generation: u8, group: u8);
    fn request_stop(&mut self, generation: u8);
    fn request_pause(&mut self, generation: u8);
    fn request_resume(&mut self, generation: u8);
    fn request_status(&mut self);
    /// Hardware adapter must drive ACK high, wait 20 us, then drive it low.
    fn pulse_ack(&mut self);
}

/// Stored bytes include inactive/stale slots, for diagnostic oracle inspection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Entry {
    pub bytes: [u8; TRANSFER_SIZE],
    pub len: u16,
    pub packet_type: u8,
}

impl Entry {
    const EMPTY: Self = Self {
        bytes: [0; TRANSFER_SIZE],
        len: 0,
        packet_type: 0,
    };
}

/// Semantic state; elapsed-time jitter and diagnostic printing are hardware-owned.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Snapshot {
    pub audio_head: u8,
    pub audio_tail: u8,
    pub control_head: u8,
    pub control_tail: u8,
    pub delivery_enabled: bool,
    pub tx_seq: u8,
    pub rx_seq: u8,
    pub rx_seq_initialized: bool,
    pub last_audio_seq: u8,
    pub last_audio_seq_valid: bool,
    pub audio_overwrite: u32,
    pub control_overwrite: u32,
    pub ingress_ok: u32,
    pub ingress_reject: u32,
    pub ingress_duplicate: u32,
    pub tx_audio: u32,
    pub tx_control: u32,
    pub tx_idle: u32,
    pub tx_fail: u32,
    pub status_enqueue: u32,
    pub status_tx: u32,
    pub status_drop: u32,
    pub rx_seq_gap: u32,
    pub rx_crc_fail: u32,
    pub ack_pulses: u32,
    pub attempts: u32,
    pub keepalive: u32,
}

pub struct Bridge {
    audio: [Entry; 16],
    control: [Entry; 4],
    state: Snapshot,
    #[cfg(feature = "detached-bridge-transfer")]
    audio_generation: u64,
    #[cfg(feature = "detached-bridge-transfer")]
    control_generation: u64,
    #[cfg(feature = "detached-bridge-transfer")]
    active: Option<Option<(bool, u8, u64)>>,
}

impl Default for Bridge {
    fn default() -> Self {
        Self::new()
    }
}

impl Bridge {
    /// Initialized portable owner, including C's initial IDLE/NONE probe status.
    /// Device initialization errors belong to the hardware adapter, not this owner.
    pub fn new() -> Self {
        let mut bridge = Self {
            audio: [Entry::EMPTY; 16],
            control: [Entry::EMPTY; 4],
            state: Snapshot {
                delivery_enabled: true,
                ..Snapshot::default()
            },
            #[cfg(feature = "detached-bridge-transfer")]
            audio_generation: 0,
            #[cfg(feature = "detached-bridge-transfer")]
            control_generation: 0,
            #[cfg(feature = "detached-bridge-transfer")]
            active: None,
        };
        let _ = bridge.send_status(0, 0, 0, 0, -1, 0);
        bridge
    }

    pub fn snapshot(&self) -> Snapshot {
        self.state
    }

    pub fn audio_storage(&self) -> &[Entry; 16] {
        &self.audio
    }

    pub fn control_storage(&self) -> &[Entry; 4] {
        &self.control
    }

    fn encode(&mut self, packet_type: u8, payload: &[u8]) -> Entry {
        let mut entry = Entry::EMPTY;
        entry.len = frame::encode(
            &mut entry.bytes,
            MAX_PAYLOAD,
            self.state.tx_seq,
            packet_type,
            payload,
        ) as u16;
        entry.packet_type = packet_type;
        if entry.len != 0 {
            self.state.tx_seq = self.state.tx_seq.wrapping_add(1);
        }
        entry
    }

    fn queue_control(&mut self, packet_type: u8, payload: &[u8]) {
        let entry = self.encode(packet_type, payload);
        let next = (self.state.control_head + 1) % 4;
        if next == self.state.control_tail {
            if self.control[usize::from(self.state.control_tail)].packet_type == STATUS {
                self.state.status_drop = self.state.status_drop.wrapping_add(1);
            }
            self.state.control_overwrite = self.state.control_overwrite.wrapping_add(1);
            self.state.control_tail = (self.state.control_tail + 1) % 4;
            self.invalidate_control_tail();
        }
        let stored = &mut self.control[usize::from(self.state.control_head)];
        let len = usize::from(entry.len);
        // C copies only encoded bytes into control slots, preserving stale suffixes.
        stored.bytes[..len].copy_from_slice(&entry.bytes[..len]);
        stored.len = entry.len;
        stored.packet_type = packet_type;
        self.state.control_head = next;
        if packet_type == STATUS {
            self.state.status_enqueue = self.state.status_enqueue.wrapping_add(1);
        }
    }

    pub fn send_audio_v2(&mut self, source: u8, payload: &[u8]) -> Result {
        if audio_bundle_parse(payload).is_none() {
            return Err(EINVAL);
        }
        if payload.len() + 1 > MAX_PAYLOAD {
            return Err(EMSGSIZE);
        }
        if !self.state.delivery_enabled {
            return Err(EAGAIN);
        }
        let next = (self.state.audio_head + 1) % 16;
        if next == self.state.audio_tail {
            self.state.audio_tail = (self.state.audio_tail + 1) % 16;
            self.invalidate_audio_tail();
            self.state.audio_overwrite = self.state.audio_overwrite.wrapping_add(1);
        }
        let mut prefixed = [0; MAX_PAYLOAD];
        prefixed[0] = source;
        prefixed[1..payload.len() + 1].copy_from_slice(payload);
        let entry = self.encode(AUDIO_V2, &prefixed[..payload.len() + 1]);
        self.audio[usize::from(self.state.audio_head)] = entry;
        self.state.audio_head = next;
        Ok(())
    }

    pub fn send_event(&mut self, event: u8, data: &[u8]) -> Result {
        if data.len() > MAX_PAYLOAD - 1 {
            return Err(EMSGSIZE);
        }
        let mut payload = [0; MAX_PAYLOAD];
        payload[0] = event;
        payload[1..data.len() + 1].copy_from_slice(data);
        self.queue_control(EVENT, &payload[..data.len() + 1]);
        Ok(())
    }

    pub fn send_log(&mut self, data: &[u8]) -> Result {
        if data.is_empty() {
            return Err(EINVAL);
        }
        self.queue_control(LOG, &data[..data.len().min(MAX_PAYLOAD)]);
        Ok(())
    }

    pub fn send_status(
        &mut self,
        state: u8,
        role: u8,
        peers: u8,
        node: u8,
        slot: i8,
        coord: u8,
    ) -> Result {
        self.queue_control(
            STATUS,
            &[role, peers, node, 6, state, slot as u8, coord, 0xa5, 2, 20],
        );
        Ok(())
    }

    pub fn send_command_ack(&mut self, command: u8, generation: u8, result: i32) -> Result {
        self.send_event(
            7,
            &[command, generation, if result == 0 { 0 } else { 0xff }],
        )
    }

    pub fn discard_pending_audio(&mut self) {
        self.invalidate_audio_tail();
        self.state.audio_tail = self.state.audio_head;
        self.state.last_audio_seq_valid = false;
    }

    /// Every call flushes and invalidates ingress dedupe, even if unchanged.
    pub fn set_audio_delivery_enabled(&mut self, enabled: bool) {
        self.discard_pending_audio();
        self.state.delivery_enabled = enabled;
    }

    /// Selection cannot race with enqueue/purge: this token exclusively borrows
    /// the owner for the entire transfer (including any hardware await).
    /// Dropping an unfinished token retains the queue and does not count a poll.
    ///
    /// ```compile_fail
    /// use nrf_embassy::bridge::Bridge;
    /// let mut bridge = Bridge::new();
    /// let prepared = bridge.prepare();
    /// bridge.discard_pending_audio();
    /// let _ = prepared.bytes();
    /// ```
    pub fn prepare(&mut self) -> Prepared<'_> {
        #[cfg(feature = "detached-bridge-transfer")]
        assert!(self.active.is_none(), "detached transfer active");
        let selected = if self.state.control_head != self.state.control_tail {
            Some((true, self.control[usize::from(self.state.control_tail)]))
        } else if self.state.audio_head != self.state.audio_tail {
            Some((false, self.audio[usize::from(self.state.audio_tail)]))
        } else {
            None
        };
        let mut bytes = [0; TRANSFER_SIZE];
        if let Some((_, entry)) = selected {
            let len = usize::from(entry.len);
            bytes[..len].copy_from_slice(&entry.bytes[..len]);
        }
        Prepared {
            bridge: self,
            bytes,
            selected: selected.map(|(control, entry)| (control, entry.packet_type)),
        }
    }

    fn invalidate_audio_tail(&mut self) {
        #[cfg(feature = "detached-bridge-transfer")]
        {
            self.audio_generation = self
                .audio_generation
                .checked_add(1)
                .expect("audio identity exhausted");
        }
    }

    fn invalidate_control_tail(&mut self) {
        #[cfg(feature = "detached-bridge-transfer")]
        {
            self.control_generation = self
                .control_generation
                .checked_add(1)
                .expect("control identity exhausted");
        }
    }

    /// Copy only the selected transfer bytes. The rings remain mutable during
    /// DMA. Exactly one detached attempt may be active; its owner must finish or
    /// cancel it (normally via a coordinator's RAII lease).
    #[cfg(feature = "detached-bridge-transfer")]
    pub fn begin_detached(&mut self) -> Option<[u8; TRANSFER_SIZE]> {
        if self.active.is_some() {
            return None;
        }
        let selected = if self.state.control_head != self.state.control_tail {
            Some((
                true,
                self.control[usize::from(self.state.control_tail)],
                self.control_generation,
            ))
        } else if self.state.audio_head != self.state.audio_tail {
            Some((
                false,
                self.audio[usize::from(self.state.audio_tail)],
                self.audio_generation,
            ))
        } else {
            None
        };
        let mut bytes = [0; TRANSFER_SIZE];
        if let Some((_, entry, _)) = selected {
            bytes[..usize::from(entry.len)].copy_from_slice(&entry.bytes[..usize::from(entry.len)]);
        }
        self.active = Some(
            selected.map(|(control, entry, generation)| (control, entry.packet_type, generation)),
        );
        Some(bytes)
    }

    /// Cancellation records no attempt and never requeues saved bytes.
    #[cfg(feature = "detached-bridge-transfer")]
    pub fn cancel_detached(&mut self) {
        assert!(self.active.take().is_some(), "no detached transfer");
    }

    #[cfg(feature = "detached-bridge-transfer")]
    pub fn finish_detached(
        &mut self,
        ok: bool,
        rx: &[u8; TRANSFER_SIZE],
        consumer: &mut impl Consumer,
    ) {
        let selected = self.active.take().expect("no detached transfer");
        self.record_attempt(consumer);
        if !ok {
            self.state.tx_fail = self.state.tx_fail.wrapping_add(1);
            return;
        }
        match selected {
            Some((true, packet_type, generation)) => {
                if generation == self.control_generation {
                    self.state.control_tail = (self.state.control_tail + 1) % 4;
                    self.invalidate_control_tail();
                }
                self.state.tx_control = self.state.tx_control.wrapping_add(1);
                if packet_type == STATUS {
                    self.state.status_tx = self.state.status_tx.wrapping_add(1);
                }
            }
            Some((false, _, generation)) => {
                if generation == self.audio_generation {
                    self.state.audio_tail = (self.state.audio_tail + 1) % 16;
                    self.invalidate_audio_tail();
                }
                self.state.tx_audio = self.state.tx_audio.wrapping_add(1);
            }
            None => self.state.tx_idle = self.state.tx_idle.wrapping_add(1),
        }
        self.parse_rx(rx, consumer);
    }

    fn record_attempt(&mut self, consumer: &mut impl Consumer) {
        self.state.attempts = self.state.attempts.wrapping_add(1);
        self.state.keepalive += 1;
        if self.state.keepalive >= 200 {
            self.state.keepalive = 0;
            consumer.request_status();
        }
    }

    fn parse_rx(&mut self, rx: &[u8; TRANSFER_SIZE], consumer: &mut impl Consumer) {
        let decoded = match frame::decode(rx, MAX_PAYLOAD) {
            Ok(decoded) => decoded,
            Err(frame::DecodeError::BadCrc) => {
                self.state.rx_crc_fail = self.state.rx_crc_fail.wrapping_add(1);
                return;
            }
            Err(_) => return,
        };
        let seq = decoded.seq;
        if !self.state.rx_seq_initialized {
            self.state.rx_seq = seq;
            self.state.rx_seq_initialized = true;
        } else if seq == self.state.rx_seq.wrapping_add(1) {
            self.state.rx_seq = seq;
        } else if seq != self.state.rx_seq {
            self.state.rx_seq_gap = self.state.rx_seq_gap.wrapping_add(1);
            self.state.rx_seq = seq;
        }
        match decoded.frame_type {
            AUDIO => self.state.ingress_reject = self.state.ingress_reject.wrapping_add(1),
            AUDIO_V2 => {
                if self.state.last_audio_seq_valid && seq == self.state.last_audio_seq {
                    self.state.ingress_duplicate = self.state.ingress_duplicate.wrapping_add(1);
                } else if consumer.accept_audio_v2(decoded.payload) == 0 {
                    self.state.ingress_ok = self.state.ingress_ok.wrapping_add(1);
                    self.state.last_audio_seq = seq;
                    self.state.last_audio_seq_valid = true;
                } else {
                    self.state.ingress_reject = self.state.ingress_reject.wrapping_add(1);
                    return;
                }
                consumer.pulse_ack();
                self.state.ack_pulses = self.state.ack_pulses.wrapping_add(1);
            }
            COMMAND => match decoded.payload {
                [3] => consumer.request_status(),
                [1, generation, group @ 1..=3] => consumer.request_start(*generation, *group),
                [2, generation, _] => consumer.request_stop(*generation),
                [4, generation, _] => consumer.request_pause(*generation),
                [5, generation, _] => consumer.request_resume(*generation),
                _ => {}
            },
            _ => {}
        }
    }
}

#[must_use = "finish records a transfer attempt; dropping retains the selected queue entry"]
pub struct Prepared<'a> {
    bridge: &'a mut Bridge,
    bytes: [u8; TRANSFER_SIZE],
    selected: Option<(bool, u8)>,
}

impl Prepared<'_> {
    pub fn bytes(&self) -> &[u8; TRANSFER_SIZE] {
        &self.bytes
    }

    /// One completed hardware attempt, including failures. Keepalive publication
    /// precedes RX callbacks, as in C, and counts every 200 `finish` calls.
    /// Failure retains selection and never decodes RX. Success dequeues first.
    pub fn finish(self, transfer_ok: bool, rx: &[u8; TRANSFER_SIZE], consumer: &mut impl Consumer) {
        let bridge = self.bridge;
        bridge.record_attempt(consumer);
        if !transfer_ok {
            bridge.state.tx_fail = bridge.state.tx_fail.wrapping_add(1);
            return;
        }
        match self.selected {
            Some((true, packet_type)) => {
                bridge.state.control_tail = (bridge.state.control_tail + 1) % 4;
                bridge.invalidate_control_tail();
                bridge.state.tx_control = bridge.state.tx_control.wrapping_add(1);
                if packet_type == STATUS {
                    bridge.state.status_tx = bridge.state.status_tx.wrapping_add(1);
                }
            }
            Some((false, _)) => {
                bridge.state.audio_tail = (bridge.state.audio_tail + 1) % 16;
                bridge.invalidate_audio_tail();
                bridge.state.tx_audio = bridge.state.tx_audio.wrapping_add(1);
            }
            None => bridge.state.tx_idle = bridge.state.tx_idle.wrapping_add(1),
        }
        bridge.parse_rx(rx, consumer);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn failed_encoding_does_not_advance_wrapping_sequence() {
        let mut bridge = Bridge::new();
        bridge.state.tx_seq = 255;
        let rejected = bridge.encode(LOG, &[0; MAX_PAYLOAD + 1]);
        assert_eq!(rejected.len, 0);
        assert_eq!(bridge.state.tx_seq, 255);
        let accepted = bridge.encode(LOG, &[42]);
        assert_eq!(accepted.bytes[2], 255);
        assert_eq!(bridge.state.tx_seq, 0);
    }

    #[cfg(not(feature = "detached-bridge-transfer"))]
    #[test]
    fn default_owner_keeps_original_layout() {
        assert_eq!(
            core::mem::size_of::<Bridge>(),
            core::mem::size_of::<[Entry; 20]>() + core::mem::size_of::<Snapshot>()
        );
    }

    #[cfg(feature = "detached-bridge-transfer")]
    mod detached {
        use super::*;
        struct Sink;
        impl Consumer for Sink {
            fn accept_audio_v2(&mut self, _: &[u8]) -> i32 {
                0
            }
            fn request_start(&mut self, _: u8, _: u8) {}
            fn request_stop(&mut self, _: u8) {}
            fn request_pause(&mut self, _: u8) {}
            fn request_resume(&mut self, _: u8) {}
            fn request_status(&mut self) {}
            fn pulse_ack(&mut self) {}
        }
        fn complete(bridge: &mut Bridge, ok: bool) {
            bridge.finish_detached(ok, &[0; TRANSFER_SIZE], &mut Sink);
        }
        fn empty() -> Bridge {
            let mut bridge = Bridge::new();
            bridge.begin_detached().unwrap();
            complete(&mut bridge, true);
            bridge
        }
        fn audio(bridge: &mut Bridge, value: u8) {
            use openhelmet_wire::audio_bundle::{audio_bundle_encode, BundleView};
            let mut bytes = [0; 56];
            assert_eq!(
                audio_bundle_encode(
                    &BundleView {
                        current_data: &[value; 48],
                        previous1_data: &[],
                        previous2_data: &[],
                        current_seq: u16::from(value),
                        stream_id: 1,
                        flags: 1,
                        codec: 2,
                    },
                    &mut bytes
                ),
                56
            );
            bridge.send_audio_v2(value, &bytes).unwrap();
        }

        #[test]
        fn seven_audio_admissions_survive_active_transfer_and_preserve_sequence() {
            let mut bridge = empty();
            bridge.state.tx_seq = 253;
            audio(&mut bridge, 0);
            let first = bridge.begin_detached().unwrap();
            assert!(bridge.begin_detached().is_none());
            for value in 1..=7 {
                audio(&mut bridge, value);
            }
            assert_eq!(bridge.state.audio_overwrite, 0);
            assert_eq!(frame::decode(&first, MAX_PAYLOAD).unwrap().seq, 253);
            complete(&mut bridge, true);
            for value in 1..=7 {
                let bytes = bridge.begin_detached().unwrap();
                let packet = frame::decode(&bytes, MAX_PAYLOAD).unwrap();
                assert_eq!(packet.payload[0], value);
                assert_eq!(packet.seq, 253u8.wrapping_add(value));
                complete(&mut bridge, true);
            }
            assert_eq!(bridge.state.audio_head, bridge.state.audio_tail);
        }

        #[test]
        fn overflow_full_ring_and_wire_wrap_cannot_pop_successor() {
            for control in [false, true] {
                for ok in [false, true] {
                    let mut bridge = empty();
                    for value in 0..if control { 3 } else { 15 } {
                        if control {
                            bridge.send_event(1, &[value]).unwrap();
                        } else {
                            audio(&mut bridge, value);
                        }
                    }
                    let selected = bridge.begin_detached().unwrap();
                    let selected_tail = if control {
                        bridge.state.control_tail
                    } else {
                        bridge.state.audio_tail
                    };
                    // More than a full wire-sequence and ring-index cycle.
                    for value in 0..=255 {
                        if control {
                            bridge.send_event(1, &[value]).unwrap();
                        } else {
                            audio(&mut bridge, value);
                        }
                    }
                    let before = bridge.snapshot();
                    assert_eq!(
                        if control {
                            before.control_tail
                        } else {
                            before.audio_tail
                        },
                        selected_tail
                    );
                    complete(&mut bridge, ok);
                    let after = bridge.snapshot();
                    assert_eq!(
                        (after.audio_tail, after.control_tail),
                        (before.audio_tail, before.control_tail)
                    );
                    assert_eq!(after.tx_fail, u32::from(!ok));
                    assert_eq!(after.tx_audio, u32::from(ok && !control));
                    assert_eq!(after.tx_control, 1 + u32::from(ok && control));
                    let next = bridge.begin_detached().unwrap();
                    assert_eq!(
                        frame::decode(&selected, MAX_PAYLOAD).unwrap().seq,
                        frame::decode(&next, MAX_PAYLOAD).unwrap().seq
                    );
                    assert_ne!(selected, next);
                    bridge.cancel_detached();
                    assert_eq!(bridge.snapshot(), after);
                }
            }
        }

        #[test]
        fn empty_selection_then_admission_purge_and_failure_preserve_current_tail() {
            let mut bridge = empty();
            assert_eq!(bridge.begin_detached().unwrap(), [0; TRANSFER_SIZE]);
            audio(&mut bridge, 1);
            complete(&mut bridge, true);
            assert_eq!(bridge.state.tx_idle, 1);
            let first = bridge.begin_detached().unwrap();
            complete(&mut bridge, false);
            assert_eq!(bridge.begin_detached().unwrap(), first); // existing retry policy
            bridge.discard_pending_audio();
            audio(&mut bridge, 2);
            let tail = bridge.state.audio_tail;
            complete(&mut bridge, true);
            assert_eq!(bridge.state.audio_tail, tail);
            let successor = bridge.begin_detached().unwrap();
            assert_eq!(
                frame::decode(&successor, MAX_PAYLOAD).unwrap().payload[0],
                2
            );
            bridge.cancel_detached();
            assert_eq!(bridge.begin_detached().unwrap(), successor);
            bridge.cancel_detached();
        }
    }
}
