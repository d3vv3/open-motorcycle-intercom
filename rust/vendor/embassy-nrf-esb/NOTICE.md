# Third-Party Notices

This file lists third-party works whose source code, patterns, or binaries are
incorporated into or linked with `embassy-nrf-esb`. Each entry preserves the
attribution required by the upstream license.

## esb-ng (source-level derivation)

Portions of the following files are adapted from the `esb-ng` crate:

- `src/timer.rs` — TIMER peripheral abstraction (esb-ng `src/peripherals.rs` lines 500–684)
- `src/radio.rs` — RADIO peripheral register layer (esb-ng `src/peripherals.rs`)
- `src/isr.rs` — RADIO ISR handler / TIMER ISR race-prevention pattern (esb-ng `irq.rs`)

Upstream:

    esb-ng — https://github.com/jamesmunns/esb
    Copyright (c) James Munns and esb-ng contributors
    Licensed under MIT OR Apache-2.0

The adapted code has been ported to `nrf-pac` 0.3 and `embassy-nrf` 0.10 APIs;
register sequences and timing semantics follow the upstream implementation.

## Nordic Semiconductor binary libraries (linked under the `mpsl` feature)

Builds enabling the `mpsl` Cargo feature link against pre-compiled libraries
distributed by Nordic Semiconductor under `LicenseRef-Nordic-5-Clause`:

- `libmpsl.a` — Multiprotocol Service Layer
  (shipped via the `nrf-mpsl-sys` crate, `third_party/nordic/nrfxlib/mpsl/`)

If the dependent application additionally uses `nrf-sdc` for Bluetooth Low
Energy coexistence:

- `libsoftdevice_controller.a` — SoftDevice Controller
  (shipped via the `nrf-sdc-sys` crate)

The Nordic-5-Clause license restricts use of these binaries to Nordic
Semiconductor integrated circuits (e.g., the nRF52 series). Downstream users
are responsible for ensuring compliance when the `mpsl` feature is enabled.
See the upstream LICENSE files in the respective crates for full terms.

## Nordic header files (source-level dependency through `nrf-mpsl-sys`)

`nrf-mpsl-sys` includes Nordic Semiconductor C headers from `nrfxlib` and
`nrfx`, distributed under `BSD-3-Clause`. These are used at build time by
`bindgen` and are not redistributed by `embassy-nrf-esb` itself.

## ESB protocol / Trademark notice

"Enhanced ShockBurst" and "ESB" are trademarks of Nordic Semiconductor ASA.
This library is an independent, clean-room-by-reference Rust implementation
compatible with Nordic's ESB radio protocol. It is **not** affiliated with,
endorsed by, or maintained by Nordic Semiconductor ASA.
