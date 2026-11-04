# OpenHelmet Documentation

See the [project README](../README.md) for an overview and project progress.

## Start Here

- [Getting started](getting_started.md): development setup, firmware builds, and initial checks.
- [Wiring](wiring.md): CoreBoard audio, XIAO SPI connections, power arrangements, and flashing preparation.
- [Button controls](buttons.md): three external buttons and voice prompts; BOOT is for ROM download only on new S31 firmware.

## System Design

- [Architecture](architecture.md): component responsibilities and transport selection.
- [Audio pipeline](audio.md): capture, codecs, playback, and Bluetooth audio routes.
- [Mesh protocol](protocol.md): packet formats, mesh control messages, and SPI flow control.
- [Adaptive mesh](mesh.md): topology, relay and handover contracts, limits, and hardware verification plan.
- [TDMA scheduling](tdma.md): voice slots, control windows, and timing constraints.

## Testing and Development

- [Pipeline telemetry](pipeline_telemetry.md): counters and tools for tracing audio delivery and drops.
- CPU profiling: optional task profiling in `-D S31_TASK_CPU_PROFILE=ON` builds; see [pipeline telemetry](pipeline_telemetry.md#capture-and-reports).

The documentation covers LC3, mesh protocol v5, bridge protocol v6, and the Rust nRF52840 firmware; NCS and Zephyr are no longer used.
