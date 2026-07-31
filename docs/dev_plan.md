# Implementation Status

This file records implemented capabilities and validation that has not been
established. It is not a promise of unmeasured range, latency, power, or ride
stability.

## Implemented

- ESP32-S3 audio capture/playback, VOX/DTX, Opus, loopback, two-source remote decode/mix, PLC/jitter handling, notifications, and synchronized teardown
- ESP-NOW mesh with election, join/leave, fixed TDMA slots, scheduled control windows, liveness, bounded relay, and telemetry
- nRF52840 ESB mesh at 1 Mbps with election backoff, identity-bearing JOIN/LEAVE, TDMA/SYNC discipline, and SPI audio/control bridge
- Shared packed wire definitions and transport-neutral `mesh_core` logic with host tests
- Generated, ACKed START/STOP commands and explicit bridge readiness state
- Benchmark parsing with reset-aware deltas and conservative correlated-delivery reporting
- CI builds both firmware targets and runs Python/C host tests

## Not Established by the Repository

- Field range, rider-count stability, and end-to-end latency guarantees
- Eight-hour battery life or measured power budgets
- Collision-free RF behavior or sub-100-us over-the-air synchronization
- Mesh authentication or encryption
- Bluetooth Classic phone/intercom integration
- Production PCB, enclosure, and environmental validation
