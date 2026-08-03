# TDMA Scheduling

This document describes the current OMI TDMA implementation.
It does not specify unimplemented behavior.

[TOC]

---

## 1. Scope

OMI uses fixed windows for joined-node voice and control traffic.
The scheduler bounds transmit opportunities.
It drops work that misses its guarded deadline.

The scheduler does not guarantee collision-free RF operation.
Discovery traffic can contend.
Radio retries and interference can also affect timing.

## 2. Fixed Parameters

The shared protocol constants apply to ESP-NOW and ESB.

| Item | Current value |
|---|---:|
| Maximum nodes | exactly 8 |
| Frame length | 20 ms |
| Voice slots | 8 |
| Slot length | 2 ms |
| Guard | 500 us |
| Control window | 2 ms |
| SYNC interval | every 10 frames, about 200 ms |

The guard is part of the slot deadline.
For example, slot 0 starts at 0 us and has a 1500 us usable deadline.
The same 500 us reduction applies to the control deadline.

```text
0 ms                                                        20 ms
| S0 | S1 | S2 | S3 | S4 | S5 | S6 | S7 | CTRL |   TAIL   |
0    2    4    6    8    10   12   14   16     18         20
```

The voice region runs from 0 ms to 16 ms.
The control region runs from 16 ms to 18 ms.
The frame tail runs from 18 ms to 20 ms.
The current scheduler does not use the frame tail.

## 3. Slot Ownership

Slot ownership is fixed.
It is not optimized or negotiated independently of the node ID.

```text
slot_index = node_id - 1
```

The coordinator forms with node ID 1 and slot 0.
It assigns the first free node ID to a new participant.
The assigned slot follows from that node ID.

The current limit is exactly 8 nodes.
There is no configuration for 8 or more additional riders.

The SLOT_MAP reports node IDs by slot.
Participants validate the map before they apply it.
The implementation does not ACK a SLOT_MAP.
It does not perform slot optimization.
It does not negotiate a future frame boundary for map activation.

## 4. Voice Transmission

Each joined node gets one slot opportunity per frame.
The slot can carry local audio or one queued relay packet.
It cannot carry both as separate transmissions.

The scheduler checks the current generation and guarded deadline.
It drops stale or late slot work.
Silence suppression can leave the slot unused.

The ESP32-S3 always encodes audio with Opus.
Active audio and eligible DTX comfort updates enter the transport.
Small inactive DTX frames are suppressed.

In the dual-MCU path, an audio bundle can contain the current Opus frame and up
to two predecessor frames.
The nRF strips predecessor data when the remaining slot airtime is too short.
It drops the bundle when the current frame cannot fit.

## 5. Relay Use of a Slot

Ordinary audio is not relayed.
The coordinator can grant relay service to at most 2 active speakers.
This limit is independent of the ESP mixer's 3 remote-source slots.

A node queues relay audio only when all these conditions are true:

- the source has a current coordinator grant
- the packet requests relay
- the node is present in the source relay mask
- the remaining TTL is nonzero
- duplicate suppression accepts the packet

Source audio starts with TTL 2.
The selected relay queues it with TTL 1.
The next receiver computes TTL 0.
The queue rejects TTL 0.
The current model therefore permits one relay transmission only.

Local and relay traffic share one slot.
The nRF path alternates preference when both are pending.
The ESP-NOW path sends local audio first and uses an empty local slot for relay.

## 6. Control Ownership

Joined-node control ownership rotates by frame.
On normal frames, the owner slot is:

```text
owner_slot = frame_counter modulo 8
```

Every tenth frame is a SYNC frame.
The coordinator owns that control window.
It sends SYNC at about 200 ms intervals.

ESP-NOW keeps non-SYNC control in a bounded priority queue.
It coalesces replaceable periodic records.
Lifecycle and topology records have higher priority.
Old records can bypass normal priority to prevent indefinite starvation.
At most one queued record is submitted in an owned control window.

The nRF path keeps a bounded control ring.
It sends the next record only in its owned control window.

## 7. Discovery Exceptions

An unassigned node has no node ID and no slot.
It cannot use joined-node control ownership.

ESP-NOW discovery JOIN uses bounded randomized contention.
It applies a minimum interval and random jitter.
ESB discovery uses direct JOIN retries and address-derived scan backoff.
Graceful LEAVE during shutdown can also bypass normal queued ownership.

These are lifecycle exceptions.
They are not a general CSMA control plane.
There is no CSMA voice or CSMA PTT mode.

## 8. Coordinator Duties

The coordinator performs these scheduling duties:

- establishes the frame epoch
- uses slot 0
- assigns the first free node ID and its fixed slot
- sends targeted JOIN acknowledgements
- publishes SLOT_MAP records
- reserves every tenth control window for SYNC
- removes peers after liveness timeout
- selects up to 2 relay-granted speakers
- publishes relay masks and grant changes

The coordinator also resolves a merged-partition conflict.
The lower radio address remains coordinator.
The higher-address coordinator stops its schedule and joins the winner.

## 9. Synchronization

### Initial Acquisition

A participant learns the candidate coordinator during scanning.
A successful assignment makes the participant active.
The ESP-NOW path waits for SYNC before it arms its frame schedule.
The nRF path starts its timer in an unsynchronized state.
A valid coordinator SYNC establishes the frame epoch.
Only then can either path transmit in its slot.

### ESP-NOW Correction

The ESP-NOW participant estimates frame start from the SYNC receive timestamp.
It includes a fixed receive-latency allowance.
On each accepted SYNC, it stops pending slot and control timers.
It sets the frame counter and frame epoch from the received SYNC.
It then arms the next frame timer from that epoch.

The code also calculates a drift value and bounds its reported value to 500 us.
That value is telemetry.
It is not a slow phase-lock correction.

### nRF52840 Correction

The nRF participant keeps a 16-entry history of local frame boundaries.
It compares a SYNC frame counter with that history.
Small phase error becomes a bounded pending period correction.
Advertised drift also contributes a bounded rate correction.

The pending correction is limited to 2 ms.
The frame period changes by at most one Zephyr timer tick per frame.
The current 10 kHz timer gives a 100 us correction step.
This is timer resolution.
It is not measured over-the-air synchronization accuracy.

The nRF stops its timers and reacquires the epoch when either condition is true:

- the frame-counter difference is greater than 4 frames
- the phase error is greater than the 500 us guard

A small frame difference with no matching history entry is deferred.
The implementation does not invent an epoch for that case.

### WS Discipline

An nRF coordinator also measures the ESP 16 kHz I2S WS signal.
Hardware counts the rising edges.
The code rejects absent or implausible samples.
Valid filtered error contributes to the pending period correction.

## 10. Loss and Reconfiguration

KEEPALIVE and STATUS continue when voice is silent.
A peer is removed after 3 seconds without a refreshing packet.
The coordinator then publishes an updated SLOT_MAP.

The nRF participant declares coordinator loss after 5 seconds without SYNC.
The ESP-NOW participant uses coordinator liveness timeout handling.
In both cases, the node stops its active schedule and returns to scanning.
It does not fall back to CSMA PTT.

A network partition can create independent coordinators.
The partitions can merge when the coordinators hear each other again.
The lower radio address wins the conflict.
The other coordinator demotes and joins it.

## 11. Transport Differences

| Area | ESP-NOW path | nRF52840 path |
|---|---|---|
| Radio | ESP32-S3 Wi-Fi radio | Nordic ESB |
| Address | 6-byte Wi-Fi MAC | 5-byte ESB address |
| RF setting | ESP-NOW configuration | 2 Mbps, +8 dBm |
| TDMA timer | ESP high-resolution timers | Zephyr timers at 10 kHz |
| SYNC correction | epoch re-anchor on accepted SYNC | bounded phase correction and explicit reacquisition; advertised rate correction is currently zero |
| Audio packet | basic Opus audio packet | current frame plus up to two predecessor frames |
| Inter-MCU link | none | 4 MHz SPI, ACK GPIO, and WS signal |

The two transports share the frame layout and mesh protocol constants.
They do not interoperate over the air.

## 12. Timing and Telemetry Status

The firmware reports slot due, late, and submit-drop counts.
It reports control deadline counts and queue drops.
The nRF reports timer jitter, pending correction, applied correction, skipped
frames, SYNC acquisition, and reacquisition.
The radio and audio paths report additional queue and sequence counters.

These counters show internal behavior.
They do not prove collision-free operation or end-to-end latency.
Current audio latency is not externally measured.
The buffering model makes latency below 80 ms unlikely in the current build.

## 13. Targets and Future Work

These items are not current verified behavior:

- externally verify end-to-end latency below 80 ms
- measure over-the-air synchronization error
- validate operation with 8 nodes under interference and mobility
- evaluate use of the 18 ms to 20 ms frame tail
- evaluate a relay model with more than one relay transmission

The current rationale is to bound work and memory.
Fixed slots simplify membership and timing.
Rotating control ownership limits joined-node contention.
Late-drop rules prevent old audio from extending the queue without limit.
