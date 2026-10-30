#![no_std]
#![forbid(unsafe_code)]

pub mod actor_policy;
#[cfg(feature = "nonblocking-bridge-delivery")]
pub mod bridge_delivery;
#[cfg(all(test, feature = "nonblocking-bridge-delivery"))]
mod bridge_delivery_actor_tests;
pub mod bridge_rpc;
pub mod cancel;
pub mod clock;
pub mod deadline;
pub mod diagnostics;
pub mod receiver;
#[cfg(feature = "reset-diagnostics")]
pub mod reset_diagnostics;
#[cfg(feature = "rx-blocking-trace")]
pub mod rx_blocking_trace;
#[cfg(all(test, feature = "same-slot-audio-refit"))]
mod same_slot_refit_tests;
#[cfg(feature = "slot-entry-trace")]
pub mod slot_entry_trace;
#[cfg(feature = "sync-trace")]
pub mod sync_trace;
pub mod timing;
pub mod timing_clock;
pub mod transport;
#[cfg(feature = "sync-trace")]
pub mod tx_diagnostics;
#[cfg(feature = "sync-trace")]
pub mod tx_end_capture;
#[cfg(feature = "sync-trace")]
pub mod tx_failure_trace;

use nrf_embassy::bridge::{Bridge, MAX_PAYLOAD, TRANSFER_SIZE};
use nrf_embassy::platform::{Effect, Integration, Platform, Reply};
use nrf_embassy::runtime::ENOTSUP;
use openhelmet_wire::bridge as frame;

/// Preserve framing, sequence, v6 membership and padding, but do not advertise
/// LC3 readiness before the radio/protocol/audio adapters actually exist.
pub fn unsupported_status_tx(input: &[u8; TRANSFER_SIZE]) -> [u8; TRANSFER_SIZE] {
    let mut output = *input;
    if let Ok(decoded) = frame::decode(input, MAX_PAYLOAD)
        && decoded.frame_type == 2
        && decoded.payload.len() >= 10
        && decoded.payload[3] == 6
    {
        let mut payload = [0; MAX_PAYLOAD];
        let len = decoded.payload.len();
        payload[..len].copy_from_slice(decoded.payload);
        payload[7..10].fill(0);
        frame::encode(
            &mut output,
            MAX_PAYLOAD,
            decoded.seq,
            decoded.frame_type,
            &payload[..len],
        );
    }
    output
}

/// Requests only publish Runtime-owned work. This backend cannot touch Bridge
/// while a Prepared token holds its exclusive borrow.
pub struct SubmitOnlyBackend;

impl Platform for SubmitOnlyBackend {
    fn integration(&self) -> Integration {
        Integration::NotIntegrated
    }

    fn now_us(&self) -> i64 {
        0
    }

    fn apply(&mut self, effect: Effect) -> Reply {
        match effect {
            Effect::SubmitCommand => Reply::None,
            _ => Reply::Value(ENOTSUP),
        }
    }
}

/// Synchronous post-transfer effects only. No RF, membership or audio admission
/// is simulated; unsupported queries and adapter operations fail closed.
pub struct HardwareBackend<'a> {
    pub bridge: &'a mut Bridge,
    pub now_us: i64,
}

impl Platform for HardwareBackend<'_> {
    fn integration(&self) -> Integration {
        Integration::NotIntegrated
    }

    fn now_us(&self) -> i64 {
        self.now_us
    }

    fn apply(&mut self, effect: Effect) -> Reply {
        let result = match effect {
            Effect::Status(status) => self.bridge.send_status(
                status.state as u8,
                status.role as u8,
                status.peer_count,
                status.node_id,
                status.slot_index,
                status.coordinator_id,
            ),
            Effect::Ack { command, result } => {
                self.bridge
                    .send_command_ack(command.kind as u8, command.generation, result)
            }
            Effect::DiscardBridgeAudio | Effect::PurgeTx => {
                self.bridge.discard_pending_audio();
                Ok(())
            }
            Effect::BridgeDelivery(enabled) => {
                self.bridge.set_audio_delivery_enabled(enabled);
                Ok(())
            }
            _ => return Reply::Value(ENOTSUP),
        };
        Reply::Value(result.err().unwrap_or(0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nrf_embassy::runtime::{Event, Runtime};

    #[test]
    fn unsupported_status_crc_valid_and_other_fields_preserved() {
        for seq in [0, 1, 254, 255] {
            let mut input = [0x5a; TRANSFER_SIZE];
            let payload = [2, 255, 7, 6, 3, 255, 9, 0xa5, 2, 20, 42];
            let len = frame::encode(&mut input, MAX_PAYLOAD, seq, 2, &payload);
            let output = unsupported_status_tx(&input);
            let decoded = frame::decode(&output, MAX_PAYLOAD).unwrap();
            assert_eq!(decoded.seq, seq);
            assert_eq!(decoded.frame_type, 2);
            assert_eq!(&decoded.payload[..7], &payload[..7]);
            assert_eq!(&decoded.payload[7..10], &[0, 0, 0]);
            assert_eq!(decoded.payload[10], 42);
            assert_eq!(output[1], input[1]);
            assert_eq!(&output[len..], &input[len..]);
            assert_ne!(decoded.payload[7..10], [0xa5, 2, 20]);
            assert_eq!(unsupported_status_tx(&output), output);
        }
    }

    #[test]
    fn non_status_idle_and_invalid_crc_are_not_rewritten() {
        let idle = [0; TRANSFER_SIZE];
        assert_eq!(unsupported_status_tx(&idle), idle);
        let mut input = [0; TRANSFER_SIZE];
        let len = frame::encode(&mut input, MAX_PAYLOAD, 8, 3, &[7, 1, 9, 255]);
        assert_eq!(unsupported_status_tx(&input), input);
        input[len - 1] ^= 1;
        assert_eq!(unsupported_status_tx(&input), input);
    }

    #[test]
    fn initial_and_start_failure_status_never_advertise_ready() {
        let mut bridge = Bridge::new();
        let initial = unsupported_status_tx(bridge.prepare().bytes());
        let status = frame::decode(&initial, MAX_PAYLOAD).unwrap();
        assert_eq!(status.payload, &[0, 0, 0, 6, 0, 255, 0, 0, 0, 0]);

        let mut runtime = Runtime::default();
        let mut backend = HardwareBackend {
            bridge: &mut bridge,
            now_us: 0,
        };
        assert_eq!(runtime.init(&mut backend), ENOTSUP);
        runtime.request_start(93, 2, &mut SubmitOnlyBackend);
        runtime.handle(Event::DispatchCommands, &mut backend);
        let entries = backend.bridge.control_storage();
        let ack = frame::decode(&entries[1].bytes, MAX_PAYLOAD).unwrap();
        assert_eq!(ack.frame_type, 3);
        assert_eq!(ack.payload, &[7, 1, 93, 255]);
        let output = unsupported_status_tx(&entries[2].bytes);
        let status = frame::decode(&output, MAX_PAYLOAD).unwrap();
        assert_eq!(status.payload[4], 0);
        assert_eq!(&status.payload[7..10], &[0, 0, 0]);
        assert_eq!(backend.integration(), Integration::NotIntegrated);
    }

    #[test]
    fn passive_rx_diagnostics_do_not_touch_protocol_or_bridge_fifo() {
        use crate::receiver::{ReceiverMetadata, ReceiverRecord, ReceiverSnapshot, drain};
        let mut bridge = Bridge::new();
        let mut runtime = Runtime::default();
        let mut backend = HardwareBackend {
            bridge: &mut bridge,
            now_us: 1234,
        };
        assert_eq!(runtime.init(&mut backend), ENOTSUP);
        let before = *backend.bridge.prepare().bytes();
        let mut received = ReceiverSnapshot::default();
        drain(&mut received, || {
            Some(ReceiverRecord {
                bytes: 252,
                metadata: Some(ReceiverMetadata {
                    channel: 40,
                    epoch: 1,
                    timestamp_us: Some(1234),
                }),
            })
        });
        assert_eq!(received.received_packets, 8);
        assert_eq!(*backend.bridge.prepare().bytes(), before);
        assert_eq!(backend.integration(), Integration::NotIntegrated);
        let output = unsupported_status_tx(&before);
        assert_eq!(
            &frame::decode(&output, MAX_PAYLOAD).unwrap().payload[7..10],
            &[0, 0, 0]
        );
        runtime.request_start(93, 1, &mut SubmitOnlyBackend);
        runtime.handle(Event::DispatchCommands, &mut backend);
        let entries = backend.bridge.control_storage();
        assert_eq!(
            frame::decode(&entries[1].bytes, MAX_PAYLOAD)
                .unwrap()
                .payload,
            &[7, 1, 93, 255]
        );
    }
}
