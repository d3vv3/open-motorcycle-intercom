//! ESB packet buffer pool with word-aligned DMA storage.
//!
//! Each packet is a fixed-size buffer preceded by a 4-byte [`EsbHeader`].
//! The layout is `[rssi, pipe, length, pid_no_ack, payload...]` where bytes 2+ are the DMA region.
//!
//! Buffer states are tracked with atomic state machines:
//! `free → tx_allocated → tx_queued → in_dma → free` (TX path)
//! `free → in_dma → rx_queued → free` (RX path)
//!
//! # Safety
//!
//! `Packet` is `pub(crate)` — it cannot be constructed or accessed outside this module.
//! All access goes through `PacketPool` methods, which enforce the state machine invariants.
//! This ensures no data race between the ISR and application context.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU8, Ordering, compiler_fence};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::header::EsbHeader;
use crate::rx_metadata::{MetadataSlot, RxMetadata};

/// Maximum ESB payload length.
pub const MAX_PAYLOAD: usize = 252;

/// Wrapping branch/attempt counters, not estimates of lost packets.
#[cfg(feature = "rx-blocking-trace")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxDiagnostics {
    pub bad_crc: u32,
    pub duplicate: u32,
    pub start_alloc_failed: u32,
    pub post_delivery_starved: u32,
    pub queue_rejected: u32,
}

/// Packet buffer states (atomic state machine).
mod state {
    pub const FREE: u8 = 0;
    pub const TX_ALLOCATED: u8 = 1;
    pub const TX_QUEUED: u8 = 2;
    pub const IN_DMA: u8 = 3;
    pub const RX_QUEUED: u8 = 4;
}

/// A single word-aligned packet buffer.
///
/// Layout: `[EsbHeader (4 bytes) | payload (SIZE bytes)]`.
/// DMA pointer points to offset 2 (the `length` field) per ESB hardware convention.
///
/// This type is `pub(crate)` to prevent external construction. All access is mediated
/// through `PacketPool`.
#[repr(C, align(4))]
pub(crate) struct Packet<const SIZE: usize> {
    data: UnsafeCell<[u8; SIZE]>,
}

#[allow(dead_code)]
impl<const SIZE: usize> Packet<SIZE> {
    /// Create a zeroed packet buffer.
    pub(crate) const fn new() -> Self {
        Self {
            data: UnsafeCell::new([0u8; SIZE]),
        }
    }

    /// Get a pointer to the DMA region (offset 2 from start).
    pub(crate) fn dma_ptr(&self) -> *mut u8 {
        unsafe { (*self.data.get()).as_mut_ptr().add(EsbHeader::DMA_OFFSET) }
    }

    /// Access the full buffer as a mutable slice.
    ///
    /// # Safety
    ///
    /// Caller must hold exclusive access (guaranteed by pool state machine).
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn buf_mut(&self) -> &mut [u8; SIZE] {
        unsafe { &mut *self.data.get() }
    }

    /// Access the full buffer as a slice.
    ///
    /// # Safety
    ///
    /// Caller must ensure no concurrent mutable access.
    pub(crate) unsafe fn buf(&self) -> &[u8; SIZE] {
        unsafe { &*self.data.get() }
    }

    /// Access the header portion.
    ///
    /// # Safety
    ///
    /// Caller must hold exclusive access.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn header_mut(&self) -> &mut EsbHeader {
        unsafe {
            let ptr = self.data.get() as *mut EsbHeader;
            &mut *ptr
        }
    }

    /// Read-only access to the header portion.
    ///
    /// # Safety
    ///
    /// Caller must ensure no concurrent mutable access.
    pub(crate) unsafe fn header(&self) -> &EsbHeader {
        unsafe {
            let ptr = self.data.get() as *const EsbHeader;
            &*ptr
        }
    }
}

// SAFETY: Packet is pub(crate), only constructed by PacketPool::new().
// The pool's atomic state machine ensures no concurrent access to any Packet.
unsafe impl<const SIZE: usize> Send for Packet<SIZE> {}
unsafe impl<const SIZE: usize> Sync for Packet<SIZE> {}

const _: () = assert!(core::mem::align_of::<Packet<1>>() >= 4);

/// A pool of packet buffers with atomic state tracking and async queues.
///
/// `N` = number of packets, `SIZE` = buffer size per packet (including 4-byte header).
/// Typical sizes: `SIZE = 256` (4-byte header + 252-byte payload).
///
/// Must be placed in a `static` (e.g. via `static_cell::make_static`).
pub struct PacketPool<const N: usize, const SIZE: usize> {
    storage: [Packet<SIZE>; N],
    state: [AtomicU8; N],
    metadata: [MetadataSlot; N],
    rx_queue: Channel<CriticalSectionRawMutex, usize, N>,
    #[cfg(feature = "rx-blocking-trace")]
    rx_diagnostics: embassy_sync::blocking_mutex::Mutex<
        CriticalSectionRawMutex,
        core::cell::RefCell<RxDiagnostics>,
    >,
}

// SAFETY: PacketPool is thread-safe because all access is mediated by the
// atomic state machine. The embassy-sync Channels are Send+Sync.
unsafe impl<const N: usize, const SIZE: usize> Send for PacketPool<N, SIZE> {}
unsafe impl<const N: usize, const SIZE: usize> Sync for PacketPool<N, SIZE> {}

impl<const N: usize, const SIZE: usize> Default for PacketPool<N, SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const SIZE: usize> PacketPool<N, SIZE> {
    /// Create a zeroed pool.
    ///
    /// Must be placed in a `static` (e.g. via `static_cell`).
    pub const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const NEW: AtomicU8 = AtomicU8::new(state::FREE);
        Self {
            storage: [const { Packet::new() }; N],
            state: [NEW; N],
            metadata: [const { MetadataSlot::new() }; N],
            rx_queue: Channel::new(),
            #[cfg(feature = "rx-blocking-trace")]
            rx_diagnostics: embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(
                RxDiagnostics {
                    bad_crc: 0,
                    duplicate: 0,
                    start_alloc_failed: 0,
                    post_delivery_starved: 0,
                    queue_rejected: 0,
                },
            )),
        }
    }

    /// Allocate a packet for TX: transitions `free → tx_allocated`.
    ///
    /// Returns an index into the pool. Caller should fill the buffer then call
    /// `enqueue_tx()` or `release_tx()`.
    #[allow(clippy::manual_find)]
    pub(crate) fn alloc_tx(&self) -> Option<usize> {
        for i in 0..N {
            if self.state[i]
                .compare_exchange(
                    state::FREE,
                    state::TX_ALLOCATED,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                return Some(i);
            }
        }
        None
    }

    /// Enqueue a TX packet for the ISR to send.
    pub(crate) async fn enqueue_tx(&self, index: usize) {
        self.enqueue_tx_now(index);
    }

    pub(crate) fn enqueue_tx_now(&self, index: usize) {
        debug_assert!(index < N);
        let queued = self.state[index]
            .compare_exchange(
                state::TX_ALLOCATED,
                state::TX_QUEUED,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_ok();
        debug_assert!(queued, "enqueue_tx called for a non-allocated packet");
        compiler_fence(Ordering::Release);
    }

    pub(crate) fn claim_allocated_tx(&self, index: usize) {
        assert!(
            self.state[index]
                .compare_exchange(
                    state::TX_ALLOCATED,
                    state::IN_DMA,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
        );
        compiler_fence(Ordering::Release);
    }

    /// Dequeue the next TX packet (called from ISR).
    pub(crate) fn try_dequeue_tx(&self) -> Option<usize> {
        (0..N).find(|&index| self.try_claim_queued_tx(index))
    }

    /// Dequeue a queued TX packet for a specific pipe (called from ISR).
    ///
    /// This is used by PRX ACK payload handling so payloads queued for one
    /// pipe cannot be consumed by packets received on another pipe. Stale
    /// queue entries are skipped later by `try_dequeue_tx()`.
    pub(crate) fn try_dequeue_tx_for_pipe(&self, pipe: u8) -> Option<usize> {
        for index in 0..N {
            if self.state[index].load(Ordering::Acquire) != state::TX_QUEUED {
                continue;
            }

            let header = unsafe { self.header(index) };
            if header.pipe == pipe && self.try_claim_queued_tx(index) {
                return Some(index);
            }
        }
        None
    }

    /// Transition a TX packet to DMA ownership: `tx_queued → in_dma`.
    ///
    /// Called by the ISR before setting PACKETPTR.
    #[cfg(test)]
    pub(crate) fn tx_to_dma(&self, index: usize) {
        debug_assert!(index < N);
        compiler_fence(Ordering::Release);
        if self.state[index]
            .compare_exchange(
                state::TX_QUEUED,
                state::IN_DMA,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_err()
        {
            debug_assert_eq!(self.state[index].load(Ordering::Acquire), state::IN_DMA);
        }
    }

    /// Release a TX packet after DMA completes: `in_dma → free`.
    pub(crate) fn release_tx(&self, index: usize) {
        debug_assert!(index < N);
        compiler_fence(Ordering::Acquire);
        self.state[index].store(state::FREE, Ordering::Release);
    }

    /// Cancel a TX allocation: `tx_allocated/tx_queued → free`.
    ///
    /// Use when a packet was allocated but will not be sent.
    #[cfg(test)]
    pub(crate) fn cancel_tx(&self, index: usize) {
        debug_assert!(index < N);
        self.state[index].store(state::FREE, Ordering::Release);
    }

    fn try_claim_queued_tx(&self, index: usize) -> bool {
        debug_assert!(index < N);
        let ok = self.state[index]
            .compare_exchange(
                state::TX_QUEUED,
                state::IN_DMA,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_ok();
        if ok {
            compiler_fence(Ordering::Acquire);
        }
        ok
    }

    /// Transition an RX packet to DMA ownership: `free → in_dma`.
    ///
    /// Returns false if the slot was not free.
    pub(crate) fn rx_to_dma(&self, index: usize) -> bool {
        debug_assert!(index < N);
        let ok = self.state[index]
            .compare_exchange(
                state::FREE,
                state::IN_DMA,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_ok();
        if ok {
            compiler_fence(Ordering::Release);
        }
        ok
    }

    /// Complete RX: transition `in_dma → rx_queued` and enqueue for app.
    pub(crate) fn rx_complete(&self, index: usize, metadata: Option<RxMetadata>) {
        debug_assert!(index < N);
        debug_assert_eq!(self.state[index].load(Ordering::Acquire), state::IN_DMA);
        compiler_fence(Ordering::Acquire);
        self.metadata[index].write(metadata);
        self.state[index].store(state::RX_QUEUED, Ordering::Release);
        if self.rx_queue.try_send(index).is_err() {
            #[cfg(feature = "rx-blocking-trace")]
            self.record_rx(|c| c.queue_rejected = c.queue_rejected.wrapping_add(1));
            self.release_rx(index);
        }
    }

    pub(crate) fn rx_metadata(&self, index: usize) -> Option<RxMetadata> {
        let slot_state = self.state[index].load(Ordering::Acquire);
        debug_assert_eq!(slot_state, state::RX_QUEUED);
        self.metadata[index].read()
    }

    #[cfg(feature = "rx-blocking-trace")]
    pub(crate) fn record_rx(&self, update: impl FnOnce(&mut RxDiagnostics)) {
        self.rx_diagnostics.lock(|c| update(&mut c.borrow_mut()));
    }

    #[cfg(feature = "rx-blocking-trace")]
    pub(crate) fn rx_diagnostics(&self) -> RxDiagnostics {
        self.rx_diagnostics.lock(|c| *c.borrow())
    }

    /// Receive the next RX packet (async, called from app context).
    pub(crate) async fn receive_rx(&self) -> usize {
        self.rx_queue.receive().await
    }

    /// Try to receive an RX packet without blocking.
    pub(crate) fn try_receive_rx(&self) -> Option<usize> {
        self.rx_queue.try_receive().ok()
    }

    /// Release an RX packet back to the pool: `rx_queued → free`.
    pub(crate) fn release_rx(&self, index: usize) {
        debug_assert!(index < N);
        compiler_fence(Ordering::Acquire);
        self.state[index].store(state::FREE, Ordering::Release);
    }

    /// Discard all queued RX packets and return their buffers to the free pool.
    pub(crate) fn discard_received(&self) -> usize {
        let mut count = 0;
        while let Some(index) = self.try_receive_rx() {
            self.release_rx(index);
            count += 1;
        }
        count
    }
}

#[allow(dead_code)]
impl<const N: usize, const SIZE: usize> PacketPool<N, SIZE> {
    /// Get the DMA pointer for a packet by index.
    ///
    /// # Safety
    ///
    /// Caller must ensure the packet is in IN_DMA state.
    pub(crate) unsafe fn dma_ptr(&self, index: usize) -> *mut u8 {
        self.storage[index].dma_ptr()
    }

    /// Get mutable buffer access for a packet by index.
    ///
    /// # Safety
    ///
    /// Caller must ensure exclusive access per state machine.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn buf_mut(&self, index: usize) -> &mut [u8; SIZE] {
        unsafe { self.storage[index].buf_mut() }
    }

    /// Get the header for a packet by index.
    ///
    /// # Safety
    ///
    /// Caller must ensure exclusive access per state machine.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn header_mut(&self, index: usize) -> &mut EsbHeader {
        unsafe { self.storage[index].header_mut() }
    }

    /// Get read-only access to a packet header by index.
    ///
    /// # Safety
    ///
    /// Caller must ensure no concurrent mutable access.
    pub(crate) unsafe fn header(&self, index: usize) -> &EsbHeader {
        unsafe { self.storage[index].header() }
    }

    /// Get read access to a packet buffer by index.
    ///
    /// # Safety
    ///
    /// Caller must ensure no concurrent mutable access.
    pub(crate) unsafe fn buf(&self, index: usize) -> &[u8; SIZE] {
        unsafe { self.storage[index].buf() }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::{PacketPool, state};
    use core::future::Future;
    use core::task::{Context, Poll};
    use std::sync::Arc;
    use std::task::{Wake, Waker};

    struct NoopWaker;

    impl Wake for NoopWaker {
        fn wake(self: Arc<Self>) {}
    }

    fn assert_ready<F: Future<Output = ()>>(future: F) {
        let waker = Waker::from(Arc::new(NoopWaker));
        let mut cx = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Ready(())));
    }

    #[test]
    fn tx_allocation_is_not_visible_until_enqueued() {
        let pool = PacketPool::<2, 16>::new();

        let idx = pool.alloc_tx().expect("free tx slot");
        assert_eq!(
            pool.state[idx].load(core::sync::atomic::Ordering::Acquire),
            state::TX_ALLOCATED
        );
        assert_eq!(pool.try_dequeue_tx(), None);

        unsafe {
            let header = pool.header_mut(idx);
            header.pipe = 3;
            header.length = 4;
        }

        assert_ready(pool.enqueue_tx(idx));

        let claimed = pool.try_dequeue_tx().expect("queued tx slot");
        assert_eq!(claimed, idx);
        pool.tx_to_dma(claimed);
        assert_eq!(
            pool.state[idx].load(core::sync::atomic::Ordering::Acquire),
            state::IN_DMA
        );

        pool.release_tx(idx);
        assert_eq!(
            pool.state[idx].load(core::sync::atomic::Ordering::Acquire),
            state::FREE
        );
    }

    #[test]
    fn pipe_filtered_dequeue_claims_only_matching_pipe() {
        let pool = PacketPool::<3, 16>::new();

        let pipe0 = pool.alloc_tx().expect("pipe0 slot");
        let pipe1 = pool.alloc_tx().expect("pipe1 slot");

        unsafe {
            pool.header_mut(pipe0).pipe = 0;
            pool.header_mut(pipe1).pipe = 1;
        }

        assert_ready(pool.enqueue_tx(pipe0));
        assert_ready(pool.enqueue_tx(pipe1));

        assert_eq!(pool.try_dequeue_tx_for_pipe(2), None);

        let claimed_pipe1 = pool.try_dequeue_tx_for_pipe(1).expect("pipe1 queued slot");
        assert_eq!(claimed_pipe1, pipe1);
        assert_eq!(
            pool.state[pipe1].load(core::sync::atomic::Ordering::Acquire),
            state::IN_DMA
        );
        assert_eq!(
            pool.state[pipe0].load(core::sync::atomic::Ordering::Acquire),
            state::TX_QUEUED
        );

        let claimed_any = pool.try_dequeue_tx().expect("remaining queued slot");
        assert_eq!(claimed_any, pipe0);

        pool.release_tx(pipe0);
        pool.release_tx(pipe1);
    }

    #[test]
    fn pipe_filtered_dequeue_skips_allocated_or_wrong_pipe_slots() {
        let pool = PacketPool::<3, 16>::new();

        let allocated = pool.alloc_tx().expect("allocated slot");
        let wrong_pipe = pool.alloc_tx().expect("wrong pipe slot");
        let target_pipe = pool.alloc_tx().expect("target pipe slot");

        unsafe {
            pool.header_mut(allocated).pipe = 1;
            pool.header_mut(wrong_pipe).pipe = 2;
            pool.header_mut(target_pipe).pipe = 1;
        }

        assert_ready(pool.enqueue_tx(wrong_pipe));
        assert_ready(pool.enqueue_tx(target_pipe));

        let claimed = pool
            .try_dequeue_tx_for_pipe(1)
            .expect("target pipe queued slot");
        assert_eq!(claimed, target_pipe);
        assert_eq!(
            pool.state[allocated].load(core::sync::atomic::Ordering::Acquire),
            state::TX_ALLOCATED
        );
        assert_eq!(
            pool.state[wrong_pipe].load(core::sync::atomic::Ordering::Acquire),
            state::TX_QUEUED
        );

        pool.cancel_tx(allocated);
        pool.release_tx(target_pipe);
        let claimed_wrong_pipe = pool
            .try_dequeue_tx_for_pipe(2)
            .expect("wrong pipe slot remains queued");
        assert_eq!(claimed_wrong_pipe, wrong_pipe);
        pool.release_tx(wrong_pipe);
    }

    #[test]
    fn cancel_tx_releases_allocated_or_queued_slots() {
        let pool = PacketPool::<1, 16>::new();

        let idx = pool.alloc_tx().expect("free tx slot");
        pool.cancel_tx(idx);
        assert_eq!(
            pool.state[idx].load(core::sync::atomic::Ordering::Acquire),
            state::FREE
        );

        let idx = pool.alloc_tx().expect("reused tx slot");
        assert_ready(pool.enqueue_tx(idx));
        pool.cancel_tx(idx);
        assert_eq!(
            pool.state[idx].load(core::sync::atomic::Ordering::Acquire),
            state::FREE
        );
    }

    #[test]
    fn rx_dma_claim_rejects_busy_packet_and_preserves_contents() {
        let pool = PacketPool::<1, 16>::new();

        assert!(pool.rx_to_dma(0));
        assert!(!pool.rx_to_dma(0));
        assert_eq!(
            pool.state[0].load(core::sync::atomic::Ordering::Acquire),
            state::IN_DMA
        );

        unsafe {
            pool.header_mut(0).pipe = 2;
            pool.header_mut(0).length = 3;
            let buf = pool.buf_mut(0);
            buf[4..7].copy_from_slice(&[0xA0, 0xB1, 0xC2]);
        }

        unsafe {
            assert_eq!(pool.header(0).pipe, 2);
            assert_eq!(pool.header(0).length, 3);
            assert_eq!(&pool.buf(0)[4..7], &[0xA0, 0xB1, 0xC2]);
        }
    }

    #[test]
    fn rx_complete_queues_packet_until_app_releases_it() {
        let pool = PacketPool::<1, 16>::new();

        assert!(pool.rx_to_dma(0));
        unsafe {
            pool.header_mut(0).pipe = 3;
            pool.header_mut(0).length = 2;
            pool.buf_mut(0)[4..6].copy_from_slice(&[0x11, 0x22]);
        }

        pool.rx_complete(0, None);
        assert_eq!(
            pool.state[0].load(core::sync::atomic::Ordering::Acquire),
            state::RX_QUEUED
        );
        assert_eq!(pool.try_receive_rx(), Some(0));
        assert_eq!(pool.try_receive_rx(), None);
        assert!(!pool.rx_to_dma(0));

        unsafe {
            assert_eq!(pool.header(0).pipe, 3);
            assert_eq!(pool.header(0).length, 2);
            assert_eq!(&pool.buf(0)[4..6], &[0x11, 0x22]);
        }

        pool.release_rx(0);
        assert_eq!(
            pool.state[0].load(core::sync::atomic::Ordering::Acquire),
            state::FREE
        );
        assert!(pool.rx_to_dma(0));
    }

    #[test]
    fn discard_received_drains_rx_queue_and_releases_buffers() {
        let pool = PacketPool::<3, 16>::new();

        assert!(pool.rx_to_dma(0));
        assert!(pool.rx_to_dma(1));
        pool.rx_complete(0, None);
        pool.rx_complete(1, None);

        assert_eq!(pool.discard_received(), 2);
        assert_eq!(pool.discard_received(), 0);
        assert!(pool.rx_to_dma(0));
        assert!(pool.rx_to_dma(1));
    }

    #[test]
    fn queue_full_publication_releases_claim_and_reuse_clears_metadata() {
        use crate::config::Channel;
        use crate::rx_metadata::{SharedReceiverContext, publish_new_rx};

        let pool = PacketPool::<1, 16>::new();
        // With N queue entries and N exclusive claims, full cannot occur in
        // normal operation. Inject queue pressure to exercise fail-closed cleanup.
        pool.rx_queue.try_send(0).unwrap();
        let mut context = SharedReceiverContext::new(Some(|| Some(123)));
        context.prepare_session(Channel(7), true).unwrap();
        assert!(pool.rx_to_dma(0));
        publish_new_rx(&pool, 0, context.snapshot());
        #[cfg(feature = "rx-blocking-trace")]
        assert_eq!(pool.rx_diagnostics().queue_rejected, 1);
        assert_eq!(
            pool.state[0].load(core::sync::atomic::Ordering::Acquire),
            state::FREE
        );
        assert_eq!(pool.try_receive_rx(), Some(0));
        assert!(pool.rx_to_dma(0));
        pool.rx_complete(0, None);
        let held = crate::isr::ReceivedPacket::from_pool(&pool, pool.try_receive_rx().unwrap());
        assert_eq!(held.metadata(), None);
        drop(held);
        assert!(pool.rx_to_dma(0));
    }

    #[cfg(feature = "rx-blocking-trace")]
    #[test]
    fn diagnostics_wrap_and_are_owned_by_pool_not_role() {
        let pool = PacketPool::<1, 16>::new();
        pool.record_rx(|c| c.bad_crc = u32::MAX);
        pool.record_rx(|c| c.bad_crc = c.bad_crc.wrapping_add(1));
        pool.record_rx(|c| c.duplicate = 17);
        assert_eq!(pool.discard_received(), 0);
        assert_eq!(pool.rx_diagnostics().bad_crc, 0);
        assert_eq!(pool.rx_diagnostics().duplicate, 17);
    }
}
