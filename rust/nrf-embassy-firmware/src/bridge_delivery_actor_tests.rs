//! Real portable actor RX/slot composition; no hardware or executor model.
extern crate std;
use crate::{
    bridge_delivery::Coordinator,
    bridge_rpc::{Request, State as Rpc},
};
use core::{
    future::{Future, pending},
    pin::pin,
    task::{Context, Waker},
};
use nrf_embassy::{
    platform::{Effect, Integration, Platform, Reply},
    protocol_adaptive::{AdaptiveEffect, AdaptiveIo, AdaptivePlatform, AdaptiveReply},
    protocol_audio::{AudioEffect, AudioPlatform},
    protocol_membership::{
        Input, MembershipEffect, MembershipOutput, MembershipPlatform, ProtocolState,
    },
    protocol_runtime::FullProtocolPlatform,
    protocol_rx::{RxEffect, RxPlatform},
    runtime::{Role, Runtime, Timer},
};
use openhelmet_wire::{
    audio_bundle::{BundleView, audio_bundle_encode},
    mesh as w,
};
use std::rc::Rc;

struct Io {
    now: i64,
    bridge: Rc<Coordinator>,
    blocking: bool,
    rx_outputs: bool,
    slot_serviced: bool,
}
impl Platform for Io {
    fn integration(&self) -> Integration {
        Integration::ActorOracle
    }
    fn now_us(&self) -> i64 {
        self.now
    }
    fn apply(&mut self, effect: Effect) -> Reply {
        match effect {
            Effect::GetAddress => Reply::Address([1, 2, 3, 4, 5]),
            _ => Reply::Value(0),
        }
    }
}
impl MembershipPlatform for Io {
    fn membership_effect(&mut self, _: MembershipEffect, _: &mut ProtocolState) -> bool {
        true
    }
    async fn membership_output_async(&mut self, output: MembershipOutput) -> bool {
        let request = match output {
            MembershipOutput::Status(status) => Ok(Request::Status(status)),
            MembershipOutput::Event {
                id,
                node: Some(node),
            } => Request::event(id, &[node]),
            MembershipOutput::Event { id, node: None } => Request::event(id, &[]),
            MembershipOutput::Log { bytes, len } => Request::log(&bytes[..usize::from(len)]),
        };
        if crate::bridge_delivery::direct_rx(self.rx_outputs, &request) {
            if self.blocking {
                return pending().await;
            }
            return self.bridge.apply(request) == 0;
        }
        true
    }
}
impl AdaptivePlatform for Io {
    async fn adaptive_status_async(
        &mut self,
        status: nrf_embassy::runtime::Membership,
    ) -> AdaptiveReply {
        let request = Ok(Request::Status(status));
        if crate::bridge_delivery::direct_rx(self.rx_outputs, &request) && !self.blocking {
            AdaptiveReply::Accepted(self.bridge.apply(request))
        } else {
            pending().await
        }
    }
    fn adaptive_effect(&mut self, _: AdaptiveEffect, _: AdaptiveIo<'_>) -> AdaptiveReply {
        AdaptiveReply::Applied
    }
    fn audio_effect(&mut self, effect: AudioEffect<'_>) -> i64 {
        self.audio_io(effect)
    }
}
impl AudioPlatform for Io {
    fn audio_io(&mut self, effect: AudioEffect<'_>) -> i64 {
        match effect {
            AudioEffect::NowUs => self.now_us(),
            AudioEffect::SlotRemainingUs => 2_000,
            _ => 0,
        }
    }
    async fn audio_io_async(&mut self, effect: AudioEffect<'_>) -> i64 {
        if let AudioEffect::BridgeDeliver { source, bundle } = effect {
            if self.blocking {
                return pending().await;
            }
            i64::from(self.bridge.apply(Request::audio_bundle(source, bundle)))
        } else {
            self.audio_io(effect)
        }
    }
}
impl RxPlatform for Io {
    fn rx_io(&mut self, _: RxEffect) -> i32 {
        1
    }
}

fn setup(blocking: bool) -> (Runtime, Io, [u8; 65]) {
    let mut runtime = Runtime::default();
    let mut io = Io {
        now: 100_000,
        bridge: Rc::new(Coordinator::new()),
        blocking,
        rx_outputs: false,
        slot_serviced: false,
    };
    io.bridge.initialize();
    assert_eq!(runtime.init(&mut FullProtocolPlatform(&mut io)), 0);
    assert_eq!(runtime.start(&mut FullProtocolPlatform(&mut io)), 0);
    let timer = runtime.timer(Timer::Scan).unwrap();
    io.now = timer.deadline_us;
    runtime
        .membership_handle(Input::Timer(timer), &mut FullProtocolPlatform(&mut io))
        .unwrap();
    assert_eq!(runtime.snapshot().membership.role, Role::Coordinator);
    let join = w::JoinV3 {
        origin: w::Identity {
            address_len: 5,
            address: [3, 3, 3, 3, 3, 0],
        },
        capabilities: 2,
        target: w::Identity {
            address_len: 5,
            address: [1, 2, 3, 4, 5, 0],
        },
    };
    let mut payload = [0; w::JoinV3::SIZE];
    join.encode(&mut payload).unwrap();
    runtime
        .membership_handle(
            Input::Packet {
                header: w::Header {
                    version: w::VERSION,
                    packet_type: 15,
                    src_id: 0,
                    seq: 1,
                    ttl: 1,
                    flags: 0,
                    talk_channel: runtime.snapshot().talk_channel,
                    payload_len: payload.len() as u16,
                },
                payload: &payload,
            },
            &mut FullProtocolPlatform(&mut io),
        )
        .unwrap();
    assert_eq!(runtime.snapshot().membership.peer_count, 1);
    let mut packet = [0; 65];
    assert_eq!(
        audio_bundle_encode(
            &BundleView {
                current_data: &[42; 48],
                previous1_data: &[],
                previous2_data: &[],
                current_seq: 7,
                stream_id: 1,
                flags: 1,
                codec: 2,
            },
            &mut packet[9..]
        ),
        56
    );
    packet[..9].copy_from_slice(&[
        w::VERSION,
        13,
        2,
        9,
        2,
        0,
        runtime.snapshot().talk_channel,
        56,
        0,
    ]);
    (runtime, io, packet)
}

#[test]
fn seven_packet_real_rx_pass_survives_while_spi_lease_is_active() {
    for active in [false, true] {
        let (mut runtime, mut io, mut packet) = setup(false);
        let owner = Rc::clone(&io.bridge);
        let transfer = active.then(|| owner.begin().unwrap());
        let mut rpc = Rpc::inactive();
        rpc.initialize().unwrap();
        let ticket = rpc.reserve(Request::DiscardAudio).unwrap();
        for sequence in 7u8..14 {
            packet[3] = sequence;
            packet[13..15].copy_from_slice(&u16::from(sequence).to_be_bytes());
            assert_eq!(
                runtime.rx_publish(&packet, -40, io.now, &mut FullProtocolPlatform(&mut io)),
                Ok(true)
            );
        }
        {
            let mut turn = pin!(async {
                io.rx_outputs = true;
                runtime
                    .rx_work_handler_async(&mut FullProtocolPlatform(&mut io))
                    .await
                    .unwrap();
                io.rx_outputs = false;
                io.slot_serviced = true;
            });
            assert!(
                turn.as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_ready()
            );
        }
        assert!(io.slot_serviced);
        assert_eq!(runtime.audio_state().counters().audio_fwd, 7);
        assert_eq!(owner.snapshot().audio_head, 7);
        assert_eq!(owner.snapshot().audio_overwrite, 0);
        assert_eq!(rpc.take_result(ticket), Ok(None));
        drop(transfer);
    }
}

#[test]
fn adaptive_status_uses_shared_rx_dispatch_with_mailbox_unserved() {
    let (_, mut io, _) = setup(false);
    let mut rpc = Rpc::inactive();
    rpc.initialize().unwrap();
    let ticket = rpc.reserve(Request::DiscardAudio).unwrap();
    io.rx_outputs = true;
    let before = io.bridge.snapshot().status_enqueue;
    {
        let mut status = pin!(io.adaptive_status_async(Default::default()));
        assert!(matches!(
            status
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop())),
            core::task::Poll::Ready(AdaptiveReply::Accepted(0))
        ));
    }
    assert_eq!(io.bridge.snapshot().status_enqueue, before + 1);
    assert_eq!(rpc.take_result(ticket), Ok(None));
}

#[test]
fn real_rx_work_returns_ready_and_reaches_slot_with_bridge_unserved() {
    for blocking in [false, true] {
        let (mut runtime, mut io, packet) = setup(blocking);
        // The actual rendezvous remains occupied and never serviced throughout.
        let mut rpc = Rpc::inactive();
        rpc.initialize().unwrap();
        let ticket = rpc.reserve(Request::DiscardAudio).unwrap();
        runtime
            .rx_publish(&packet, -40, io.now, &mut FullProtocolPlatform(&mut io))
            .unwrap();
        {
            let mut turn = pin!(async {
                io.rx_outputs = true;
                runtime
                    .rx_work_handler_async(&mut FullProtocolPlatform(&mut io))
                    .await
                    .unwrap();
                io.rx_outputs = false;
                runtime
                    .audio_slot_tx_handler_async(0, 1, &mut FullProtocolPlatform(&mut io))
                    .await;
                io.slot_serviced = true;
            });
            let result = turn.as_mut().poll(&mut Context::from_waker(Waker::noop()));
            assert_eq!(result.is_pending(), blocking);
        }
        assert_eq!(io.slot_serviced, !blocking);
        assert_eq!(rpc.take_result(ticket), Ok(None));
        let counters = runtime.audio_state().counters();
        assert_eq!(counters.rf_rx_audio_ok, 1);
        assert_eq!(counters.audio_fwd, u32::from(!blocking));
        if !blocking {
            assert_eq!(io.bridge.snapshot().audio_head, 1);
        }
    }
}
