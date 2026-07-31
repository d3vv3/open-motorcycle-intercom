# Architecture Overview

This document describes the implemented ESP32-S3 and ESP32-S3+nRF52840 architecture.

## Configurations

The ESP32-S3 always owns audio capture/playback, Opus, VOX, remote-source mixing,
notifications, user policy, and application telemetry.

- In the single-MCU configuration, ESP-NOW provides the radio transport and the ESP32-S3 also runs mesh and TDMA scheduling.
- In the dual-MCU configuration, the nRF52840 runs the 1 Mbps ESB transport, RF mesh state, and TDMA scheduling. The ESP32-S3 and nRF52840 exchange audio and control over 4 MHz SPI.

The ESP32-S3 selects the nRF transport when the SPI probe receives status;
otherwise it initializes ESP-NOW. The two radio transports are not interoperable.

## Shared Ownership

Wire-visible mesh constants, enums, headers, and common payloads are defined once
in `shared/mesh_protocol_defs.h`. Transport-specific SYNC and ESB identity fields
remain in their platform headers because ESP-NOW uses six-byte MAC addresses and
ESB uses five-byte addresses.

Bridge packet IDs, START/STOP generations and ACKs, lifecycle events, readiness
states, and status layout are owned by `shared/bridge_protocol_defs.h`.

`shared/mesh_core.c` owns transport-neutral operations used by both mesh targets:

- node/slot and bitmap validation
- packet deduplication and wrap-aware sequence classification
- coordinator address comparison
- JOIN assignment and SLOT_MAP validation
- bounded relay-mask calculation

The shared packed layouts have compile-time size checks and host compile/tests in CI.

## Audio Flow

1. The ESP32-S3 captures fixed-format 16 kHz mono, 16-bit, 20 ms PCM frames.
2. VOX and Opus produce an active frame or a periodic DTX comfort frame.
3. The selected transport queues the frame for the local TDMA slot.
4. Received frames are admitted by source ID into one of two bounded remote-source slots.
5. Each source has its own jitter state, queue, and Opus decoder.
6. Decoded sources and queued notification tones are summed in a 32-bit accumulator, saturated to 16-bit PCM, and written by the audio task to I2S.

Audio teardown is synchronized: stop wakes the audio task, waits for its explicit
completion signal, and then resets decoders and queues or deletes resources.

## Mesh Lifecycle

Both transports expose `IDLE`, `SCANNING`, `JOINING`, and `ACTIVE`. `ACTIVE` means
a coordinator has formed a schedule or a participant has accepted an assignment;
participants suppress slot transmission until valid SYNC establishes timing.

In dual-MCU mode, ESP START/STOP commands are single-flight and generation-tagged.
The nRF ACKs after applying the command. Readiness is reported separately by
status and MESH_READY/MESH_STOPPED. PEER_JOINED and PEER_LEFT describe topology
changes and are not lifecycle completion signals.

## Scheduling and Control

Each 20 ms frame has eight 2 ms voice slots followed by a 2 ms control window.
The 500 us guard is part of each slot deadline. Joined-node control ownership
rotates by frame; every tenth frame reserves the window for coordinator SYNC.

ESP-NOW stores non-SYNC control in a bounded priority queue. Replaceable periodic
records are coalesced, lifecycle/topology records outrank periodic records, and
age prevents indefinite starvation. Discovery JOIN requests use bounded
randomized contention before assignment. Discovery JOIN and shutdown LEAVE are
immediate exceptions because those operations occur outside stable ownership.

Late slot/control work is dropped. The design reduces contention but does not
claim collision-free RF behavior or guaranteed latency.

## Relaying and Liveness

Ordinary audio is single-hop. The coordinator can grant at most two active
speakers for relay. Only granted audio selected by a relay mask is forwarded;
TTL and duplicate suppression bound amplification. KEEPALIVE and STATUS maintain
liveness independently of voice, and peers time out after 3 seconds without a
refreshing packet.

## Timing Discipline

The nRF coordinator counts ESP I2S WS rising edges using GPIOTE/PPI and a hardware
counter. Valid samples discipline the local frame period. Participants compare
coordinator SYNC against bounded frame history and apply phase/rate correction or
reacquire after a large frame difference.

The nRF Zephyr timer runs at 10 kHz, so period corrections have 100 us resolution
and are applied at no more than one tick per frame. This is timer discipline, not
a claim of 100 us over-the-air synchronization accuracy.

## Bounded Failure Behavior

Audio source slots, bridge queues, control queues, relay queues, and jitter queues
are bounded. Excess, stale, malformed, duplicate, or late work is rejected and
counted. Participant sync loss returns the node to scanning/election; there is no
CSMA voice fallback.
