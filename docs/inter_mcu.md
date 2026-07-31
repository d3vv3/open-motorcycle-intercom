# Inter-MCU Contract

This document records the implemented ESP32-S3/nRF52840 contract.

## Ownership

The ESP32-S3 owns audio I/O and cadence, Opus encode/decode, VOX, source mixing,
notifications, user policy, and the bridge SPI-slave task. The nRF52840 owns ESB,
mesh state, RF slot/control deadlines, and the bridge SPI-master polling loop.

`shared/bridge_protocol_defs.h` is the source of truth for bridge packet IDs,
commands, command ACKs, lifecycle/topology events, readiness states, and status
payload. `shared/mesh_protocol_defs.h` and `shared/mesh_core.c` own common on-air
layouts and transport-neutral mesh rules.

## Physical Link

- 4 MHz SPI, nRF master and ESP slave
- one full-duplex 256-byte transaction every 5 ms
- CRC8 and an 8-bit frame sequence in each framed packet
- separate nRF-to-ESP GPIO pulse acknowledging durable ESP audio admission
- ESP 16 kHz I2S WS connected to the nRF counter input for clock discipline

The frame format is:

```text
| Sync 0xAA | Length | Sequence | Type | Payload | CRC8 |
```

## Audio Transfer

ESP-to-nRF audio uses a bounded queue and one retained in-flight frame. While
waiting for the GPIO ACK, the ESP re-presents the same frame on subsequent SPI
transactions. A 50 ms timeout releases it so a lost pulse cannot deadlock the
pipeline; stale queued audio is dropped rather than sent after its useful window.

Lifecycle control can pass while audio awaits ACK. Control has a separate pending
slot, so it does not consume an audio queue entry.

The nRF maintains separate bounded outbound control and audio queues, selects
control first, and advances the selected queue only after a successful SPI
transfer. A failed transaction therefore leaves that packet available for retry.

## Lifecycle Commands

START and STOP carry an 8-bit generation. The ESP permits one lifecycle command
at a time and retries the same command/generation for a bounded number of 300 ms
ACK waits. It accepts only a matching command and generation ACK.

The nRF applies the requested transition in its command worker, then sends the
ACK and a status snapshot. The ACK reports command application, not mesh RF
readiness.

Status carries protocol version and explicit `IDLE`, `SCANNING`, `JOINING`, or
`ACTIVE` state plus role, node ID, slot, coordinator ID, and peer count.
MESH_READY/MESH_STOPPED report lifecycle readiness. PEER_JOINED/PEER_LEFT report
topology changes only; a node becoming coordinator is not a peer-join event.

## Timing

Audio remains fixed at 16 kHz mono, 16-bit samples, and 20 ms frames. The nRF owns
RF scheduling. When it is coordinator, valid ESP WS edge samples discipline its
local TDMA period. Participants use coordinator SYNC for frame phase and rate.
The Zephyr timer has 100 us correction resolution in the current configuration.

## Telemetry

Firmware periodically emits bridge admission/drop counters, retry/ACK timeouts,
TDMA deadlines and correction state, ESB outcomes, and audio queue/decode
counters. See `pipeline_telemetry.md` for reporting limitations.
