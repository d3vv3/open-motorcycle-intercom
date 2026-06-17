# TDMA Design & Scheduling

This document defines the **Time Division Multiple Access (TDMA)** system used by the Open Motorcycle Intercom (OMI).

TDMA is the core mechanism that enables **deterministic latency, collision-free voice transport, and linear scalability**.

This is not a generic TDMA design. It is purpose-built for **small, mobile, real-time voice meshes**.

[TOC]

---

## Why TDMA (and not CSMA)

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

## TDMA Parameters

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

## Time Model

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

## Time Master Concept

One node acts as the **Time Master**.

Responsibilities:
- Defines frame start
- Broadcasts SYNC packets
- Maintains slot map

The master **does not route more traffic** than others. It only coordinates time.

---

## Time Synchronization

### SYNC Transmission

- Sent periodically by the master. Cadence differs by transport: ESP-NOW sends
  every `MESH_SYNC_INTERVAL_FRAMES` (10 frames ~= 200 ms); the nRF52840/ESB build
  sends on alternating status ticks (~= 1 s).
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

## Slot Ownership

- Each node owns exactly **one slot**
- Slot index is assigned by the master
- Slot order defines transmission order

Slots are **logical**, not tied to Node IDs.

---

## Transmission Rules

### Voice Transmission

- Transmit **at most one AUDIO packet per frame**
- Only inside own slot
- Silence = suppressed via Opus DTX: only periodic comfort-noise frames are sent;
  pure 1-2 byte DTX frames are dropped before TX (see audio.md §5.1)

### Forwarding

- **Single-hop by default**: ordinary (ungranted) audio is not relayed.
- Only a currently-active, coordinator-**granted** speaker's audio is relayed, and
  only by nodes in that speaker's relay mask (the nodes that heard it). Relays are
  TTL-bounded (`MESH_AUDIO_TTL_DEFAULT` = 2) and de-duplicated by (Type, SrcID,
  Seq) - see protocol.md "Relaying & Multi-Hop".

Bounded TTL, the active-speaker grant, and the never-relay rule for ungranted
audio keep latency bounded and prevent broadcast amplification.

---

## Guard Time

Guard time absorbs:
- Clock drift
- Radio turnaround
- Processing jitter

Typical guard time:
- 100–300 µs

Guard time is included in slot duration.

---

## Join Procedure (TDMA Perspective)

1. New node listens passively
2. Learns frame timing from SYNC
3. Sends JOIN request (CSMA)
4. Master assigns slot
5. SLOT_MAP broadcast
6. Node begins transmitting in assigned slot

Until slot assignment, node **must not transmit voice**.

---

## Leave Procedure

### Graceful Leave

- Node sends LEAVE
- Master updates slot map
- SLOT_MAP broadcast

### Unplanned Loss

- Each peer has a `last_seen` timestamp refreshed by any packet (AUDIO / KEEPALIVE
  / STATUS / JOIN), not KEEPALIVE alone.
- After `MESH_NODE_TIMEOUT_MS` (3000 ms) with no packets, the peer is dropped and
  its slot reclaimed (implemented on both ESP-NOW and nRF52840/ESB).
- A node that is merely silent keeps sending KEEPALIVE/STATUS, so silence does not
  cause loss (see protocol.md "Node Loss").

Slot reuse is immediate but synchronized.

---

## Slot Map Reconfiguration

Triggered when:
- Node joins
- Node leaves
- Slot optimization needed

Rules:
- Reconfiguration only at frame boundaries
- Old map remains valid until new map acknowledged

This avoids mid-frame chaos.

---

## Master Election

### Initial Master

- First node becomes master

### Master Loss Detection

- Missing SYNC for N frames

### Election Rule

- If two coordinators are detected, the one with the **lower radio address wins**
  (lexicographic `memcmp` of the address carried in SYNC); the other demotes to
  participant.
- The winning coordinator continues issuing SYNC; demoted nodes phase-lock to it.

This avoids complex elections.

---

## Handling Mobility & Topology Changes

Motorcycle topology is typically linear.

TDMA design assumes:
- Small hop count
- Gradual topology changes

When partitions occur:
- Each partition elects its own master
- Independent TDMA schedules form

No attempt is made to merge partitions automatically.

---

## Failure Modes & Degradation

| Failure | Behavior |
|------|---------|
| Missed slot | Frame skipped |
| Clock drift | Absorbed by guard |
| Packet loss | Opus PLC |
| Sync loss | Temporary CSMA fallback |

Silence is preferred over distortion.

---

## Debug & Instrumentation Hooks

Recommended metrics:
- Slot miss count
- Sync error (µs)
- Frame jitter
- Late packet count

These are critical for tuning.

---

## Design Rationale Summary

- Fixed 20 ms frame aligns with audio
- One packet per slot simplifies logic
- No retransmissions avoids latency explosion
- Simple master election reduces edge cases

TDMA is intentionally boring. That is its strength.

---

## Status

This document defines the **authoritative TDMA behavior** for OMI.

Any deviation must justify increased complexity or improved determinism.
