//! Allocation-free wire codecs; no firmware runtime or transport integration.
#![no_std]
#![forbid(unsafe_code)]

pub mod audio_bundle;
pub mod bridge;
pub mod mesh;
