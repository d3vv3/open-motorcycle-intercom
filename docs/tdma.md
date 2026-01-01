# TDMA Design & Scheduling

This document defines the **Time Division Multiple Access (TDMA)** system used by the Open Motorcycle Intercom (OMI).

TDMA is the core mechanism that enables **deterministic latency, collision-free voice transport, and linear scalability**.

This is not a generic TDMA design. It is purpose-built for **small, mobile, real-time voice meshes**.

[TOC]

---

## 1. Why TDMA (and Not CSMA)

Voice intercoms fail when latency spikes or packets collide.

CSMA-based systems (Wi‑Fi, BLE) suffer from:
- Random backoff delays
- Collision probability increasing with nodes
- Unbounded worst-case latency

TDMA solves this by design:
- Fixed transmission windows
- No collisions
- Predictable timing

Motorcycle groups are **small and slow-changing**, which makes TDMA viable and robust.

---

## 2. Core TDMA Parameters

### Fixed Frame Duration

- **Frame length:** 20 ms
- Aligned with Opus frame duration
- One voice opportunity per node per frame

### Slot Duration

- Nominal slot length: **2 ms**
- Configurable range: 1.5–2.5 ms
- Per-slot guard time: ~100–200 µs (included in slot duration)

### Capacity Example

20 ms frame budget allocation:

| Component | Duration | Purpose |
|-----------|----------|---------|
| Voice slots | 16 ms | 8 × 2 ms slots |
| Control window | 2 ms | CSMA for join/leave/sync |
| Frame guard | 2 ms | Clock drift absorption |

Slot capacity by configuration:

| Slot Length | Voice Slots | Control + Guard | Max Nodes |
|-------------|-------------|-----------------|-----------|
| 1.5 ms | 12 ms | 4 ms + 4 ms | ~8 |
| 2 ms | 16 ms | 2 ms + 2 ms | ~8 |
| 2.5 ms | 15 ms | 2.5 ms + 2.5 ms | ~6 |

Default configuration targets **6–8 nodes** with 2 ms slots.

---

## 3. Time Model

Each node maintains:
- Local high-resolution timer
- TDMA frame counter
- Slot index

Time is expressed as:
```
T = FrameCounter * 20 ms + SlotOffset
```

All nodes aim to transmit at the same frame boundaries.

---

## 4. Time Master Concept

One node acts as the **Time Master**.

Responsibilities:
- Defines frame start
- Broadcasts SYNC packets
- Maintains slot map

The master **does not route more traffic** than others. It only coordinates time.

---

## 5. Time Synchronization

### SYNC Transmission

- Sent periodically (e.g. every 10 frames)
- Broadcast during control (CSMA) window

### Synchronization Fields

- FrameCounter
- Estimated drift

### Slave Behavior

- Phase-lock local timer to master
- Apply slow drift correction
- Never jump time abruptly

Target sync accuracy: **<100 µs**

---

## 6. Slot Ownership

- Each node owns exactly **one slot**
- Slot index is assigned by the master
- Slot order defines transmission order

Slots are **logical**, not tied to Node IDs.

---

## 7. Transmission Rules

### Voice Transmission

- Transmit **at most one AUDIO packet per frame**
- Only inside own slot
- Silence = do not transmit

### Forwarding

- Relayed packets are transmitted in the node’s own slot
- No packet forwarding outside slot

This guarantees bounded latency and prevents amplification.

---

## 8. Guard Time

Guard time absorbs:
- Clock drift
- Radio turnaround
- Processing jitter

Typical guard time:
- 100–300 µs

Guard time is included in slot duration.

---

## 9. Join Procedure (TDMA Perspective)

1. New node listens passively
2. Learns frame timing from SYNC
3. Sends JOIN request (CSMA)
4. Master assigns slot
5. SLOT_MAP broadcast
6. Node begins transmitting in assigned slot

Until slot assignment, node **must not transmit voice**.

---

## 10. Leave Procedure

### Graceful Leave

- Node sends LEAVE
- Master updates slot map
- SLOT_MAP broadcast

### Unplanned Loss

- KEEPALIVE timeout
- Slot reclaimed automatically

Slot reuse is immediate but synchronized.

---

## 11. Slot Map Reconfiguration

Triggered when:
- Node joins
- Node leaves
- Slot optimization needed

Rules:
- Reconfiguration only at frame boundaries
- Old map remains valid until new map acknowledged

This avoids mid-frame chaos.

---

## 12. Master Election

### Initial Master

- First node becomes master

### Master Loss Detection

- Missing SYNC for N frames

### Election Rule

- Lowest Node ID becomes provisional master
- Provisional master issues SYNC
- Others accept unless conflict detected

This avoids complex elections.

---

## 13. Handling Mobility & Topology Changes

Motorcycle topology is typically linear.

TDMA design assumes:
- Small hop count
- Gradual topology changes

When partitions occur:
- Each partition elects its own master
- Independent TDMA schedules form

No attempt is made to merge partitions automatically.

---

## 14. Failure Modes & Degradation

| Failure | Behavior |
|------|---------|
| Missed slot | Frame skipped |
| Clock drift | Absorbed by guard |
| Packet loss | Opus PLC |
| Sync loss | Temporary CSMA fallback |

Silence is preferred over distortion.

---

## 15. Debug & Instrumentation Hooks

Recommended metrics:
- Slot miss count
- Sync error (µs)
- Frame jitter
- Late packet count

These are critical for tuning.

---

## 16. Design Rationale Summary

- Fixed 20 ms frame aligns with audio
- One packet per slot simplifies logic
- No retransmissions avoids latency explosion
- Simple master election reduces edge cases

TDMA is intentionally boring. That is its strength.

---

## 17. Status

This document defines the **authoritative TDMA behavior** for OMI.

Any deviation must justify increased complexity or improved determinism.

