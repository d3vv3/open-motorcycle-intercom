//! Abort a pending DMA operation before destroying its pinned child future.

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use pin_project_lite::pin_project;

/// The implementation must quiesce DMA before returning, or never return or unwind.
/// It must not release the child's buffers or restart the peripheral.
pub trait AbortOperations {
    fn abort(&mut self);
}

pin_project! {
    pub struct GuardedFuture<'a, F, A: AbortOperations> {
        #[pin]
        child: F,
        abort: &'a mut A,
        armed: bool,
    }

    impl<F, A: AbortOperations> PinnedDrop for GuardedFuture<'_, F, A> {
        fn drop(this: Pin<&mut Self>) {
            let this = this.project();
            if *this.armed {
                // PinnedDrop runs before any field (including child) is dropped.
                this.abort.abort();
            }
        }
    }
}

impl<'a, F, A: AbortOperations> GuardedFuture<'a, F, A> {
    pub fn new(child: F, abort: &'a mut A) -> Self {
        Self {
            child,
            abort,
            armed: false,
        }
    }
}

impl<F: Future, A: AbortOperations> Future for GuardedFuture<'_, F, A> {
    type Output = F::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.project();
        // Arm before the child can start DMA, including on its first poll.
        *this.armed = true;
        let result = this.child.poll(cx);
        if result.is_ready() {
            *this.armed = false;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::marker::PhantomPinned;
    use std::boxed::Box;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    use std::task::Waker;
    use std::vec::Vec;

    struct OwnedBuffer {
        alive: Rc<Cell<bool>>,
        log: Rc<RefCell<Vec<&'static str>>>,
    }

    impl Drop for OwnedBuffer {
        fn drop(&mut self) {
            self.alive.set(false);
            self.log.borrow_mut().push("buffer-drop");
        }
    }

    pin_project! {
        struct Child {
            buffer: OwnedBuffer,
            ready: Option<Result<(), ()>>,
            #[pin]
            _pinned: PhantomPinned,
        }
    }

    impl Future for Child {
        type Output = Result<(), ()>;

        fn poll(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
            let this = self.project();
            assert!(this.buffer.alive.get());
            this.buffer.log.borrow_mut().push("child-poll");
            match *this.ready {
                Some(result) => Poll::Ready(result),
                None => Poll::Pending,
            }
        }
    }

    struct FakeAbort {
        alive: Rc<Cell<bool>>,
        log: Rc<RefCell<Vec<&'static str>>>,
    }

    impl AbortOperations for FakeAbort {
        fn abort(&mut self) {
            for event in ["STOP", "STOPPED", "fence"] {
                assert!(
                    self.alive.get(),
                    "child buffer must survive all abort steps"
                );
                self.log.borrow_mut().push(event);
            }
        }
    }

    fn drop_case(poll: bool, ready: Option<Result<(), ()>>) -> Vec<&'static str> {
        let log = Rc::new(RefCell::new(Vec::new()));
        let alive = Rc::new(Cell::new(true));
        let mut abort = FakeAbort {
            alive: alive.clone(),
            log: log.clone(),
        };
        let child = Child {
            buffer: OwnedBuffer {
                alive: alive.clone(),
                log: log.clone(),
            },
            ready,
            _pinned: PhantomPinned,
        };
        let mut guarded = Box::pin(GuardedFuture::new(child, &mut abort));
        if poll {
            let mut cx = Context::from_waker(Waker::noop());
            assert_eq!(
                guarded.as_mut().poll(&mut cx),
                ready.map_or(Poll::Pending, Poll::Ready)
            );
        }
        drop(guarded);
        assert!(!alive.get());
        log.borrow().clone()
    }

    #[test]
    fn unpolled_does_not_abort() {
        assert_eq!(drop_case(false, None), ["buffer-drop"]);
    }

    #[test]
    fn ready_success_and_parameter_error_do_not_abort() {
        for ready in [Ok(()), Err(())] {
            assert_eq!(drop_case(true, Some(ready)), ["child-poll", "buffer-drop"]);
        }
    }

    #[test]
    fn pending_aborts_before_owned_buffer_destruction() {
        assert_eq!(
            drop_case(true, None),
            ["child-poll", "STOP", "STOPPED", "fence", "buffer-drop"]
        );
    }
}
