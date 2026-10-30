//! Single-poll bridge for canonical implementations with synchronous IO selected.
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

pub(crate) fn ready<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(value) => value,
        Poll::Pending => unreachable!("synchronous implementation suspended"),
    }
}

pub(crate) async fn radio_send<const ASYNC_SEND: bool>(
    p: &mut impl crate::protocol_adaptive::AdaptivePlatform,
    packet: crate::protocol_tx::RadioSend<'_>,
) -> i32 {
    if ASYNC_SEND {
        p.radio_send_async(packet).await
    } else {
        p.radio_send(packet)
    }
}

pub(crate) async fn radio_send_classified<const ASYNC_SEND: bool>(
    p: &mut impl crate::protocol_adaptive::AdaptivePlatform,
    packet: crate::protocol_tx::RadioSend<'_>,
) -> crate::protocol_tx::RadioSendResult {
    if ASYNC_SEND {
        p.radio_send_classified_async(packet).await
    } else {
        p.radio_send_classified(packet)
    }
}
