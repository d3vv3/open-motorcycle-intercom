#![no_std]
#![forbid(unsafe_code)]

#[cfg(test)]
extern crate std;

pub mod bridge;
mod corefuture;
pub mod platform;
pub mod protocol_adaptive;
pub mod protocol_audio;
pub mod protocol_membership;
pub mod protocol_runtime;
pub mod protocol_rx;
pub mod protocol_tx;
pub mod runtime;
pub mod tdma;
pub mod ws;

#[cfg(test)]
mod async_tests;

#[cfg(test)]
mod async_audio_rx_tests;

#[cfg(test)]
mod async_bridge_tests;

#[cfg(test)]
mod external_ingress_tests;
