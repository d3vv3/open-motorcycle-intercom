//! Receive-only metadata. No fields are part of the RADIO DMA buffer or wire format.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::Channel;
use crate::esb::TxFault;
use crate::payload::PacketPool;

/// Opaque receive-session generation, independent of TX operation generations.
/// A facade never wraps or reuses a generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxEpoch(u64);

impl RxEpoch {
    /// Read the generation without permitting construction of a session.
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Snapshot captured at ISR handling time after normal RADIO quiescence.
/// This is not a calibrated RADIO END timestamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxMetadata {
    /// Nonnegative shared-clock microseconds, or None when the clock is unavailable.
    pub timestamp_us: Option<i64>,
    /// Configured RF channel for this receive session.
    pub rf_channel: u8,
    /// Receive session that accepted this packet.
    pub receiver_epoch: RxEpoch,
}

/// Owned by the canonical facade, not by a TX role or operation.
pub(crate) struct SharedReceiverContext {
    clock: Option<fn() -> Option<i64>>,
    epoch: u64,
    channel: u8,
    overflowed: bool,
}

impl SharedReceiverContext {
    pub(crate) const fn new(clock: Option<fn() -> Option<i64>>) -> Self {
        Self {
            clock,
            epoch: 0,
            channel: 0,
            overflowed: false,
        }
    }

    /// Dormant reconfiguration changes pending channel only. Activation and
    /// active reconstruction request a new session before moving/starting owners.
    /// This policy never invokes the clock accessor.
    pub(crate) fn prepare_session(
        &mut self,
        channel: Channel,
        request_rx: bool,
    ) -> Result<Option<RxMetadataContext>, TxFault> {
        if self.overflowed {
            return Err(TxFault::RxEpochOverflow);
        }
        if !request_rx {
            self.channel = channel.0;
            return Ok(None);
        }
        self.advance(channel)?;
        Ok(self.snapshot())
    }

    fn advance(&mut self, channel: Channel) -> Result<(), TxFault> {
        if self.overflowed {
            return Err(TxFault::RxEpochOverflow);
        }
        let Some(next) = self.epoch.checked_add(1) else {
            self.overflowed = true;
            return Err(TxFault::RxEpochOverflow);
        };
        self.epoch = next;
        self.channel = channel.0;
        Ok(())
    }

    pub(crate) fn overflowed(&self) -> bool {
        self.overflowed
    }

    /// Session capability is independent of the optional packet metadata clock.
    pub(crate) fn current_epoch(&self) -> Option<RxEpoch> {
        (self.epoch > 0 && !self.overflowed).then_some(RxEpoch(self.epoch))
    }

    pub(crate) fn snapshot(&self) -> Option<RxMetadataContext> {
        if self.epoch == 0 || self.overflowed {
            return None;
        }
        self.clock.map(|clock| RxMetadataContext {
            clock,
            epoch: RxEpoch(self.epoch),
            channel: self.channel,
        })
    }
}

/// Desired PRX state, independent of IRQ activation and the temporary TX role.
/// Callbacks keep the production policy testable without fabricated MMIO owners.
pub(crate) struct ReceiveControl {
    requested: bool,
}

impl ReceiveControl {
    pub(crate) const fn new() -> Self {
        Self { requested: false }
    }

    pub(crate) fn wanted(&self, activated: bool) -> bool {
        activated && self.requested
    }

    pub(crate) fn epoch(&self, receiver: &SharedReceiverContext) -> Option<RxEpoch> {
        self.requested.then(|| receiver.current_epoch()).flatten()
    }

    pub(crate) fn prepare_session(
        &mut self,
        receiver: &mut SharedReceiverContext,
        channel: Channel,
        activated: bool,
    ) -> Result<Option<RxMetadataContext>, TxFault> {
        let result = receiver.prepare_session(channel, self.wanted(activated));
        if result.is_err() {
            self.requested = false;
        }
        result
    }

    pub(crate) fn start(
        &mut self,
        receiver: &mut SharedReceiverContext,
        channel: Channel,
        activated: bool,
        tx_active: bool,
        start: impl FnOnce(Option<RxMetadataContext>) -> Result<(), TxFault>,
    ) -> Result<RxEpoch, TxFault> {
        if tx_active {
            return Err(TxFault::Busy);
        }
        if !activated {
            return Err(TxFault::NotReady);
        }
        if receiver.overflowed() {
            self.requested = false;
            return Err(TxFault::RxEpochOverflow);
        }
        let context = if self.requested {
            receiver.snapshot()
        } else {
            self.requested = true;
            self.prepare_session(receiver, channel, activated)?
        };
        // Allocation/start failure retains desire for same-session service recovery.
        start(context)?;
        receiver.current_epoch().ok_or(TxFault::NotReady)
    }

    pub(crate) fn stop(
        &mut self,
        tx_active: bool,
        cleanup: impl FnOnce() -> Result<(), TxFault>,
    ) -> Result<(), TxFault> {
        if tx_active {
            return Err(TxFault::Busy);
        }
        self.requested = false;
        cleanup()
    }
}

#[derive(Clone, Copy)]
pub(crate) struct RxMetadataContext {
    clock: fn() -> Option<i64>,
    epoch: RxEpoch,
    channel: u8,
}

/// Called only for valid, nonduplicate NoAck RX after normal stop. The caller
/// still owns the DMA claim. No thread-context path may invoke the clock.
pub(crate) fn publish_new_rx<const N: usize, const SIZE: usize>(
    pool: &PacketPool<N, SIZE>,
    index: usize,
    context: Option<RxMetadataContext>,
) {
    let metadata = context.map(|context| RxMetadata {
        timestamp_us: (context.clock)().filter(|time| *time >= 0),
        rf_channel: context.channel,
        receiver_epoch: context.epoch,
    });
    pool.rx_complete(index, metadata);
}

/// Split words need no AtomicU64 or new UnsafeCell. The exclusive slot claim
/// prevents modification while queued/held; pool publication supplies release/acquire.
pub(crate) struct MetadataSlot {
    time_low: AtomicU32,
    time_high: AtomicU32,
    epoch_low: AtomicU32,
    epoch_high: AtomicU32,
    channel_flags: AtomicU32,
}

impl MetadataSlot {
    pub(crate) const fn new() -> Self {
        Self {
            time_low: AtomicU32::new(0),
            time_high: AtomicU32::new(0),
            epoch_low: AtomicU32::new(0),
            epoch_high: AtomicU32::new(0),
            channel_flags: AtomicU32::new(0),
        }
    }

    pub(crate) fn write(&self, metadata: Option<RxMetadata>) {
        let (time, epoch, flags) = match metadata {
            Some(metadata) => (
                metadata.timestamp_us.unwrap_or(0) as u64,
                metadata.receiver_epoch.0,
                u32::from(metadata.rf_channel)
                    | (1 << 8)
                    | (u32::from(metadata.timestamp_us.is_some()) << 9),
            ),
            None => (0, 0, 0),
        };
        self.time_low.store(time as u32, Ordering::Relaxed);
        self.time_high.store((time >> 32) as u32, Ordering::Relaxed);
        self.epoch_low.store(epoch as u32, Ordering::Relaxed);
        self.epoch_high
            .store((epoch >> 32) as u32, Ordering::Relaxed);
        self.channel_flags.store(flags, Ordering::Relaxed);
    }

    pub(crate) fn read(&self) -> Option<RxMetadata> {
        let flags = self.channel_flags.load(Ordering::Relaxed);
        if flags & (1 << 8) == 0 {
            return None;
        }
        let time = u64::from(self.time_low.load(Ordering::Relaxed))
            | (u64::from(self.time_high.load(Ordering::Relaxed)) << 32);
        let epoch = u64::from(self.epoch_low.load(Ordering::Relaxed))
            | (u64::from(self.epoch_high.load(Ordering::Relaxed)) << 32);
        Some(RxMetadata {
            timestamp_us: (flags & (1 << 9) != 0).then_some(time as i64),
            rf_channel: flags as u8,
            receiver_epoch: RxEpoch(epoch),
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::header::EsbHeader;
    use crate::isr::ReceivedPacket;
    use core::future::Future;
    use core::sync::atomic::AtomicUsize;
    use core::task::{Context, Poll};
    use std::sync::Arc;
    use std::task::{Wake, Waker};

    fn clock() -> Option<i64> {
        Some(0x1234_5678_90ab_cdef)
    }

    fn context(channel: u8) -> SharedReceiverContext {
        let mut context = SharedReceiverContext::new(Some(clock));
        context.prepare_session(Channel(channel), true).unwrap();
        context
    }

    fn packet<const N: usize, const SIZE: usize>(
        pool: &PacketPool<N, SIZE>,
    ) -> ReceivedPacket<'_, N, SIZE> {
        ReceivedPacket::from_pool(pool, pool.try_receive_rx().unwrap())
    }

    #[test]
    fn stopped_activation_and_configuration_spend_no_epoch_or_clock() {
        fn forbidden() -> Option<i64> {
            panic!("session policy called clock")
        }
        let mut receiver = SharedReceiverContext::new(Some(forbidden));
        let mut control = ReceiveControl::new();
        for activated in [false, true] {
            for channel in [7, 42, 65] {
                assert!(
                    control
                        .prepare_session(&mut receiver, Channel(channel), activated)
                        .unwrap()
                        .is_none()
                );
                assert!(!control.wanted(activated));
                assert_eq!(control.epoch(&receiver), None);
                assert_eq!(receiver.epoch, 0);
            }
        }
        assert_eq!(
            control.start(&mut receiver, Channel(65), false, false, |_| {
                panic!("start before activation")
            }),
            Err(TxFault::NotReady)
        );
        assert_eq!(
            control
                .start(&mut receiver, Channel(65), true, false, |_| Ok(()))
                .unwrap()
                .value(),
            1
        );
        assert_eq!(receiver.current_epoch().unwrap().value(), 1);
    }

    #[test]
    fn repeated_stop_start_keeps_queued_and_held_bytes_and_metadata() {
        let pool = PacketPool::<3, 16>::new();
        let mut receiver = SharedReceiverContext::new(Some(clock));
        let mut control = ReceiveControl::new();
        let first = control
            .start(&mut receiver, Channel(7), true, false, |_| Ok(()))
            .unwrap();
        for index in [0, 1] {
            assert!(pool.rx_to_dma(index));
            unsafe {
                pool.header_mut(index).length = 1;
                pool.buf_mut(index)[4] = 0xabu8;
            }
            publish_new_rx(&pool, index, receiver.snapshot());
        }
        let held = packet(&pool);
        let expected = held.metadata();
        for _ in 0..3 {
            control.stop(false, || Ok(())).unwrap();
            assert!(!control.wanted(true));
            assert_eq!(control.epoch(&receiver), None);
            // Config/recovery/normal TX restore all use this production gate.
            assert!(
                control
                    .prepare_session(&mut receiver, Channel(42), true)
                    .unwrap()
                    .is_none()
            );
            assert_eq!(receiver.current_epoch(), Some(first));
            assert_eq!(held.metadata(), expected);
            assert_eq!(held.payload(), &[0xab]);
        }
        let second = control
            .start(&mut receiver, Channel(42), true, false, |_| Ok(()))
            .unwrap();
        assert_eq!(first.value(), 1);
        assert_eq!(second.value(), 2);
        let queued = packet(&pool);
        assert_eq!(queued.metadata(), expected);
        assert_eq!(queued.payload(), held.payload());
        assert_eq!(held.metadata(), expected);
        assert!(pool.rx_to_dma(2));
        publish_new_rx(&pool, 2, receiver.snapshot());
        assert_eq!(packet(&pool).metadata().unwrap().receiver_epoch, second);
    }

    #[test]
    fn normal_tx_restore_only_advances_when_receive_is_wanted() {
        for wanted in [false, true] {
            let mut receiver = SharedReceiverContext::new(None);
            let mut control = ReceiveControl::new();
            if wanted {
                assert_eq!(
                    control
                        .start(&mut receiver, Channel(7), true, false, |_| Ok(()))
                        .unwrap()
                        .value(),
                    1
                );
            }
            // Role restoration uses the same policy before owner consumption.
            let metadata = control
                .prepare_session(&mut receiver, Channel(7), true)
                .unwrap();
            assert!(metadata.is_none()); // Legacy metadata is off, not the epoch.
            assert_eq!(control.wanted(true), wanted);
            assert_eq!(
                control.epoch(&receiver).map(RxEpoch::value),
                wanted.then_some(2)
            );
            assert_eq!(receiver.epoch, if wanted { 2 } else { 0 });
        }
    }

    #[test]
    fn busy_receive_controls_never_touch_callbacks_or_desire() {
        for wanted in [false, true] {
            let mut receiver = SharedReceiverContext::new(None);
            let mut control = ReceiveControl::new();
            if wanted {
                control
                    .start(&mut receiver, Channel(7), true, false, |_| Ok(()))
                    .unwrap();
            }
            let original = receiver.current_epoch();
            assert_eq!(
                control.start(&mut receiver, Channel(42), true, true, |_| {
                    panic!("Busy touched guard/start")
                }),
                Err(TxFault::Busy)
            );
            assert_eq!(
                control.stop(true, || panic!("Busy touched disarm/stop")),
                Err(TxFault::Busy)
            );
            assert_eq!(control.wanted(true), wanted);
            assert_eq!(receiver.current_epoch(), original);
            assert_eq!(receiver.channel, if wanted { 7 } else { 0 });
        }
    }

    #[test]
    fn unavailable_start_keeps_desire_and_retry_uses_same_context() {
        let mut receiver = SharedReceiverContext::new(Some(clock));
        let mut control = ReceiveControl::new();
        assert_eq!(
            control.start(&mut receiver, Channel(7), true, false, |context| {
                assert_eq!(context.unwrap().epoch.value(), 1);
                Err(TxFault::ReceiverUnavailable)
            }),
            Err(TxFault::ReceiverUnavailable)
        );
        assert!(control.wanted(true));
        for _ in 0..3 {
            assert_eq!(
                control
                    .start(&mut receiver, Channel(7), true, false, |context| {
                        assert_eq!(context.unwrap().epoch.value(), 1);
                        Ok(())
                    })
                    .unwrap()
                    .value(),
                1
            );
        }
        control
            .stop(false, || Err(TxFault::PowerCycled))
            .unwrap_err();
        assert!(!control.wanted(true));
        assert_eq!(control.epoch(&receiver), None);
        assert!(
            control
                .prepare_session(&mut receiver, Channel(42), true)
                .unwrap()
                .is_none()
        );
        assert_eq!(receiver.epoch, 1);
    }

    #[test]
    fn requested_overflow_is_fail_closed_before_start_callback() {
        let mut receiver = SharedReceiverContext::new(None);
        receiver.epoch = u64::MAX;
        let mut control = ReceiveControl::new();
        // A stopped owner at MAX can still configure without requesting RX.
        assert!(
            control
                .prepare_session(&mut receiver, Channel(7), true)
                .unwrap()
                .is_none()
        );
        for _ in 0..2 {
            assert_eq!(
                control.start(&mut receiver, Channel(7), true, false, |_| {
                    panic!("overflow consumed owner/claimed pool/started RX")
                }),
                Err(TxFault::RxEpochOverflow)
            );
            assert!(!control.wanted(true));
            assert_eq!(control.epoch(&receiver), None);
            assert_eq!(receiver.current_epoch(), None);
            assert_eq!(receiver.epoch, u64::MAX);
            assert!(receiver.overflowed());
        }
    }

    #[test]
    fn current_epoch_is_independent_of_optional_metadata_clock() {
        fn missing() -> Option<i64> {
            None
        }
        fn zero() -> Option<i64> {
            Some(0)
        }
        for clock_fn in [
            None,
            Some(missing as fn() -> Option<i64>),
            Some(zero as fn() -> Option<i64>),
        ] {
            let mut receiver = SharedReceiverContext::new(clock_fn);
            let mut control = ReceiveControl::new();
            let epoch = control
                .start(&mut receiver, Channel(7), true, false, |_| Ok(()))
                .unwrap();
            assert_eq!(epoch.value(), 1);
            assert_eq!(control.epoch(&receiver), Some(epoch));
            let pool = PacketPool::<1, 16>::new();
            assert!(pool.rx_to_dma(0));
            publish_new_rx(&pool, 0, receiver.snapshot());
            let metadata = packet(&pool).metadata();
            assert_eq!(metadata.is_some(), clock_fn.is_some());
            if let Some(metadata) = metadata {
                assert_eq!(metadata.receiver_epoch, epoch);
                assert_eq!(metadata.timestamp_us, clock_fn.unwrap()());
                let copy = metadata;
                control.stop(false, || Ok(())).unwrap();
                assert_eq!(copy.receiver_epoch, epoch);
            }
        }
    }

    #[test]
    fn initial_activation_generation_is_one() {
        use crate::addresses::EsbAddresses;
        use crate::config::EsbConfig;

        // Exercise the production dormant/activation policy, not MMIO setup or
        // full facade transitions. Pending pipe/address changes do not spend epochs.
        for configure_count in 0..=3 {
            let mut receiver = SharedReceiverContext::new(Some(clock));
            let mut config = EsbConfig::default();
            let mut addresses = EsbAddresses::default();
            for (channel, pipes) in [(7, 1), (42, 4), (65, 2)].into_iter().take(configure_count) {
                config = EsbConfig {
                    channel: Channel(channel),
                    ..config
                };
                addresses = EsbAddresses::new([0xe7; 4], [0xc2; 4], [0xe7; 8], pipes).unwrap();
                assert!(
                    receiver
                        .prepare_session(config.channel, false)
                        .unwrap()
                        .is_none()
                );
                assert_eq!(receiver.epoch, 0);
                assert_eq!(receiver.channel, config.channel.0);
                assert!(receiver.snapshot().is_none());
            }
            let snapshot = receiver.prepare_session(config.channel, true).unwrap();
            assert_eq!(snapshot.unwrap().epoch.value(), 1);
            assert_eq!(snapshot.unwrap().channel, config.channel.0);
            let pool = PacketPool::<1, 16>::new();
            assert!(pool.rx_to_dma(0));
            unsafe {
                pool.header_mut(0).pipe = addresses.pipe_count() - 1;
            }
            publish_new_rx(&pool, 0, snapshot);
            let held = packet(&pool);
            assert_eq!(held.pipe(), addresses.pipe_count() - 1);
            assert_eq!(held.metadata().unwrap().rf_channel, config.channel.0);
            assert_eq!(held.metadata().unwrap().receiver_epoch.value(), 1);
        }
    }

    #[test]
    fn receive_sessions_advance_independently_of_tx() {
        use crate::config::EsbConfig;

        // Address-only reinit (same channel) and channel reinit share the policy.
        for channel in [7, 42] {
            let mut receiver = context(7);
            let config = EsbConfig {
                channel: Channel(channel),
                ..Default::default()
            };
            // Same production request policy as active configure, then restore after TX.
            // Hardware role conversion is compiled separately, not executed here.
            let configured = receiver.prepare_session(config.channel, true).unwrap();
            assert_eq!(configured.unwrap().epoch.value(), 2);
            assert_eq!(configured.unwrap().channel, config.channel.0);
            let pool = PacketPool::<2, 16>::new();
            assert!(pool.rx_to_dma(0));
            publish_new_rx(&pool, 0, configured);
            let held = packet(&pool);
            let expected = held.metadata();
            let restored = receiver.prepare_session(config.channel, true).unwrap();
            assert_eq!(restored.unwrap().epoch.value(), 3);
            assert_eq!(restored.unwrap().channel, config.channel.0);
            // Ordinary packet restart publishes within the installed PRX session;
            // like start_receiving, it does not request a new session from the owner.
            for _ in 0..2 {
                assert!(pool.rx_to_dma(1));
                publish_new_rx(&pool, 1, receiver.snapshot());
                assert_eq!(packet(&pool).metadata().unwrap().receiver_epoch.value(), 3);
                assert_eq!(receiver.epoch, 3);
                assert_eq!(held.metadata(), expected);
            }
        }
    }

    #[test]
    fn epoch_overflow_is_checked_and_sticky() {
        let mut context = context(7);
        // Valid scalar seed only: no hardware owner or MMIO is fabricated.
        context.epoch = u64::MAX - 1;
        context.prepare_session(Channel(42), true).unwrap();
        assert_eq!(context.snapshot().unwrap().epoch.value(), u64::MAX);
        assert!(matches!(
            context.prepare_session(Channel(43), true),
            Err(TxFault::RxEpochOverflow)
        ));
        assert!(context.overflowed());
        assert!(context.snapshot().is_none());
        assert!(matches!(
            context.prepare_session(Channel(44), true),
            Err(TxFault::RxEpochOverflow)
        ));
        assert_eq!(context.epoch, u64::MAX);
        assert_eq!(context.channel, 42);

        let mut dormant = SharedReceiverContext::new(Some(clock));
        dormant.epoch = u64::MAX;
        for channel in [7, 42, 65] {
            assert!(
                dormant
                    .prepare_session(Channel(channel), false)
                    .unwrap()
                    .is_none()
            );
            assert_eq!(dormant.epoch, u64::MAX);
            assert_eq!(dormant.channel, channel);
            assert!(!dormant.overflowed());
        }
        assert!(matches!(
            dormant.prepare_session(Channel(65), true),
            Err(TxFault::RxEpochOverflow)
        ));
        assert!(dormant.overflowed());
        assert!(dormant.snapshot().is_none());
        assert!(matches!(
            dormant.prepare_session(Channel(7), false),
            Err(TxFault::RxEpochOverflow)
        ));
        assert_eq!(dormant.epoch, u64::MAX);
        assert_eq!(dormant.channel, 65);
    }

    #[test]
    fn legacy_context_never_supplies_metadata() {
        let mut context = SharedReceiverContext::new(None);
        assert!(context.snapshot().is_none());
        context.advance(Channel(7)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context.snapshot());
        assert_eq!(packet(&pool).metadata(), None);
    }

    #[test]
    fn copied_context_keeps_original_channel_and_epoch() {
        let mut context = context(7);
        let captured = context.snapshot();
        context.advance(Channel(42)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, captured);
        let metadata = packet(&pool).metadata().unwrap();
        assert_eq!(metadata.rf_channel, 7);
        assert_eq!(metadata.receiver_epoch.value(), 1);
    }

    #[test]
    fn valid_publication_calls_real_context_clock_once() {
        static CALLS: AtomicUsize = AtomicUsize::new(0);
        fn counted() -> Option<i64> {
            CALLS.fetch_add(1, Ordering::Relaxed);
            Some(123)
        }
        let mut context = SharedReceiverContext::new(Some(counted));
        context.advance(Channel(65)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context.snapshot());
        let held = packet(&pool);
        assert_eq!(held.metadata().unwrap().timestamp_us, Some(123));
        assert_eq!(held.metadata().unwrap().rf_channel, 65);
        assert_eq!(held.metadata().unwrap().receiver_epoch.value(), 1);
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn unavailable_clock_keeps_metadata_present() {
        static CALLS: AtomicUsize = AtomicUsize::new(0);
        fn unavailable() -> Option<i64> {
            CALLS.fetch_add(1, Ordering::Relaxed);
            None
        }
        let mut receiver = SharedReceiverContext::new(Some(unavailable));
        receiver.advance(Channel(7)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context(42).snapshot());
        assert!(packet(&pool).metadata().unwrap().timestamp_us.is_some());
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, receiver.snapshot());
        let metadata = packet(&pool).metadata().unwrap();
        assert_eq!(metadata.timestamp_us, None);
        assert_eq!(metadata.rf_channel, 7);
        assert_eq!(metadata.receiver_epoch.value(), 1);
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn negative_shared_clock_is_unavailable_not_zero() {
        fn negative() -> Option<i64> {
            Some(-1)
        }
        let mut context = SharedReceiverContext::new(Some(negative));
        context.advance(Channel(7)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context.snapshot());
        assert_eq!(packet(&pool).metadata().unwrap().timestamp_us, None);
    }

    #[test]
    fn task_session_management_never_invokes_clock() {
        fn forbidden() -> Option<i64> {
            panic!("clock called by session management")
        }
        let mut context = SharedReceiverContext::new(Some(forbidden));
        assert!(context.snapshot().is_none());
        for channel in [7, 42, 65] {
            assert!(
                context
                    .prepare_session(Channel(channel), false)
                    .unwrap()
                    .is_none()
            );
            assert_eq!(context.epoch, 0);
        }
        for channel in [7, 7, 42] {
            context.prepare_session(Channel(channel), true).unwrap();
            assert!(context.snapshot().is_some());
        }
    }

    #[test]
    fn context_helper_model_only_publishes_valid_new_rx() {
        static CALLS: AtomicUsize = AtomicUsize::new(0);
        fn counted() -> Option<i64> {
            CALLS.fetch_add(1, Ordering::Relaxed);
            Some(9)
        }
        let mut context = SharedReceiverContext::new(Some(counted));
        context.advance(Channel(7)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        // This models the ISR branch gate, not hardware check_packet/stop.
        // Actual ISR branches are compiled but never executed on the host.
        for (valid, duplicate, normal_stop) in [
            (false, false, true),
            (true, true, true),
            (true, false, false),
            (true, false, true),
        ] {
            if valid && !duplicate && normal_stop {
                assert!(pool.rx_to_dma(0));
                publish_new_rx(&pool, 0, context.snapshot());
            }
        }
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(packet(&pool).metadata().unwrap().timestamp_us, Some(9));
        assert!(pool.try_receive_rx().is_none());
    }

    #[test]
    fn split_signed_time_and_high_epoch_words_round_trip() {
        let pool = PacketPool::<1, 16>::new();
        for time in [i64::MIN, -0x1234_5678_90ab_cdef, 0, i64::MAX] {
            let expected = RxMetadata {
                timestamp_us: Some(time),
                rf_channel: 100,
                receiver_epoch: RxEpoch(0xfedc_ba98_7654_3210),
            };
            assert!(pool.rx_to_dma(0));
            // Raw sidecar encoding preserves signed bits; the ISR clock helper
            // separately rejects negative shared-clock values.
            pool.rx_complete(0, Some(expected));
            assert_eq!(packet(&pool).metadata(), Some(expected));
        }
    }

    #[test]
    fn real_zero_clock_is_distinct_from_unavailable() {
        fn zero() -> Option<i64> {
            Some(0)
        }
        let mut context = SharedReceiverContext::new(Some(zero));
        context.advance(Channel(0)).unwrap();
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context.snapshot());
        assert_eq!(packet(&pool).metadata().unwrap().timestamp_us, Some(0));
    }

    #[test]
    fn legacy_reuse_clears_last_slot_metadata() {
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context(7).snapshot());
        assert!(packet(&pool).metadata().is_some());
        let tx = pool.alloc_tx().unwrap();
        pool.claim_allocated_tx(tx);
        pool.release_tx(tx);
        assert!(pool.rx_to_dma(0));
        pool.rx_complete(0, None);
        assert_eq!(packet(&pool).metadata(), None);
    }

    #[test]
    fn held_and_queued_records_survive_other_claims_and_sessions() {
        let pool = PacketPool::<3, 16>::new();
        let mut context = context(7);
        for index in [0, 1] {
            assert!(pool.rx_to_dma(index));
            publish_new_rx(&pool, index, context.snapshot());
        }
        let held = packet(&pool);
        let expected = held.metadata();
        context.advance(Channel(42)).unwrap();
        let tx = pool.alloc_tx().unwrap();
        assert_eq!(tx, 2);
        pool.claim_allocated_tx(tx);
        pool.release_tx(tx);
        assert!(!pool.rx_to_dma(0));
        assert!(!pool.rx_to_dma(1));
        assert!(pool.rx_to_dma(2));
        publish_new_rx(&pool, 2, context.snapshot());
        assert_eq!(held.metadata(), expected);
        assert_eq!(packet(&pool).metadata(), expected);
        let newest = packet(&pool).metadata().unwrap();
        assert_eq!(newest.rf_channel, 42);
        assert_eq!(newest.receiver_epoch.value(), 2);
    }

    #[test]
    fn metadata_is_outside_unchanged_dma_header_and_payload() {
        let pool = PacketPool::<1, 256>::new();
        assert_eq!(core::mem::size_of::<EsbHeader>(), 4);
        assert_eq!(EsbHeader::DMA_OFFSET, 2);
        assert_eq!(EsbHeader::PAYLOAD_OFFSET, 4);
        assert_eq!(core::mem::size_of::<crate::payload::Packet<256>>(), 256);
        assert!(pool.rx_to_dma(0));
        let mut expected = [0xabu8; 256];
        expected[2] = 252;
        unsafe {
            pool.buf_mut(0).copy_from_slice(&expected);
            assert_eq!(pool.dma_ptr(0), pool.buf_mut(0).as_mut_ptr().add(2));
        }
        publish_new_rx(&pool, 0, context(7).snapshot());
        let held = packet(&pool);
        assert_eq!(held.len(), 252);
        assert_eq!(held.payload(), &expected[4..]);
        assert_eq!(unsafe { pool.buf(0) }, &expected);
        assert!(held.metadata().is_some());
    }

    #[test]
    fn complete_metadata_precedes_queue_wake_and_consumer() {
        struct Observer {
            pool: Arc<PacketPool<1, 16>>,
            observed: AtomicUsize,
        }
        impl Wake for Observer {
            fn wake(self: Arc<Self>) {
                self.wake_by_ref();
            }
            fn wake_by_ref(self: &Arc<Self>) {
                // Inspect publication without reentering the channel mutex or
                // polling a task. Actual firmware wakers must defer polling.
                let metadata = self.pool.rx_metadata(0).unwrap();
                assert_eq!(metadata.timestamp_us, clock());
                assert_eq!(metadata.rf_channel, 7);
                assert_eq!(metadata.receiver_epoch.value(), 1);
                self.observed.fetch_add(1, Ordering::Relaxed);
            }
        }
        let pool = Arc::new(PacketPool::<1, 16>::new());
        let observer = Arc::new(Observer {
            pool: pool.clone(),
            observed: AtomicUsize::new(0),
        });
        let waker = Waker::from(observer.clone());
        let mut cx = Context::from_waker(&waker);
        let receive = pool.receive_rx();
        let mut receive = core::pin::pin!(receive);
        assert!(receive.as_mut().poll(&mut cx).is_pending());
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context(7).snapshot());
        assert_eq!(observer.observed.load(Ordering::Relaxed), 1);
        let Poll::Ready(index) = receive.as_mut().poll(&mut cx) else {
            panic!("not published")
        };
        let held = ReceivedPacket::from_pool(&pool, index);
        assert_eq!(held.metadata().unwrap().timestamp_us, clock());
    }

    #[test]
    fn raii_drop_not_metadata_copy_releases_slot() {
        let pool = PacketPool::<1, 16>::new();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context(7).snapshot());
        let held = packet(&pool);
        let copy = held.metadata().unwrap();
        assert!(pool.alloc_tx().is_none());
        drop(held);
        assert!(pool.rx_to_dma(0));
        pool.rx_complete(0, None);
        assert_eq!(packet(&pool).metadata(), None);
        assert_eq!(copy.rf_channel, 7);
        assert_eq!(copy.receiver_epoch.value(), 1);
    }
}
