# OpenHelmet Documentation

See the [project README](../README.md) for an overview and project progress.

## Start Here

- [Getting started](getting_started.md): development setup, firmware builds, and initial checks.
- [Wiring](wiring.md): CoreBoard audio, XIAO SPI connections, power arrangements, and flashing preparation.
- [Button gestures](buttons.md): BOOT button actions and notification sounds.

## System Design

- [Architecture](architecture.md): component responsibilities and transport selection.
- [Audio pipeline](audio.md): capture, codecs, playback, and Bluetooth audio routes.
- [Mesh protocol](protocol.md): packet formats, mesh control messages, and SPI flow control.
- [TDMA scheduling](tdma.md): voice slots, control windows, and timing constraints.
- [Power and battery](power.md): power management and battery measurement status.

## Testing and Development

- [Pipeline telemetry](pipeline_telemetry.md): counters and tools for tracing audio delivery and drops.
- [CPU profiling](s31-cpu-profiling.md): optional task profiling, timing measurements, and interpretation limits.

## Investigation History

- [S31 Bluetooth / ESP-NOW coexistence](s31-bluetooth-coexistence.md): September 2026 experiments using the S31's shared radio.
  This report predates the separate nRF ESB setup and does not describe its performance.

The documentation is being updated for LC3, protocol v3, and NCS 3.4.1.
Some linked guides still describe earlier versions; dated reports retain their original test context.
