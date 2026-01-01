# OMI Protocol Specification (v0.1)

This document specifies the **on-air and inter-MCU protocol** for the Open Motorcycle Intercom (OMI).

The protocol is intentionally minimal, deterministic, and optimized for **real-time voice**, not general data transfer.

This specification covers:
- Packet types and formats
- Audio encapsulation
- TDMA frame usage
- Control messages
- Routing and relaying rules

[TOC]

---

## 1. Design Principles

- Deterministic latency beats throughput
- Fixed-size packets where possible
- Stateless forwarding
- Small headers, explicit fields
- Audio is first-class; everything else is secondary

---

## 2. Terminology

- **Node**: One intercom unit (one rider)
- **Frame**: One TDMA cycle (aligned to Opus frame)
- **Slot**: Time window allocated to one node
- **Hop**: One wireless relay
- **Audio Frame**: One Opus-encoded 20 ms chunk

---

## 3. Network Model

- Small, bounded mesh (2–10 nodes)
- Single logical group per mesh
- One active time master
- Max hop count: 2 (configurable)

Nodes outside hop limit are ignored.

---

## 4. Packet Overview

All packets share a common header.

### Common Packet Header (8 bytes)

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 1 | Version | Protocol version (0x01) |
| 1 | 1 | Type | Packet type |
| 2 | 1 | SrcID | Source node ID |
| 3 | 1 | Seq | Sequence number |
| 4 | 1 | Hop | Remaining hop count |
| 5 | 1 | Flags | Control flags |
| 6 | 2 | PayloadLen | Bytes following header |

All fields are little-endian.

---

## 5. Packet Types

| Type | Name | Purpose |
|----|------|--------|
| 0x01 | AUDIO | Opus audio data |
| 0x02 | JOIN | Request to join mesh |
| 0x03 | JOIN_ACK | Join response |
| 0x04 | LEAVE | Graceful leave |
| 0x05 | SYNC | TDMA timing sync |
| 0x06 | SLOT_MAP | Slot assignment |
| 0x07 | STATUS | Battery / health |
| 0x08 | KEEPALIVE | Presence check |

---

## 6. AUDIO Packet

### Purpose
Carries exactly **one Opus frame**.

### Payload Format

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 1 | Codec | 0x01 = Opus |
| 1 | 1 | FrameMs | Frame duration (20) |
| 2 | 1 | StreamID | Logical audio stream |
| 3 | 1 | Reserved | Future use |
| 4 | N | OpusData | Encoded audio |

Typical payload size:
- 20–40 bytes @ 8–16 kbps

### Rules
- AUDIO packets **must only be sent in TDMA slots**
- Packets received outside slot are dropped
- If Hop > 0, packet may be forwarded

---

## 7. TDMA Frame Usage

### Frame Parameters

- Frame duration: 20 ms
- Slot duration: configurable (1–3 ms)
- Guard time: implementation-defined

### Slot Ownership

- Each node owns exactly one slot
- Slot ID is implicit by position

### Transmission Rules

- One AUDIO packet per slot
- Silence = no transmission
- No retransmissions

---

## 8. Routing & Relaying

### Forwarding Rules

Upon receiving an AUDIO packet:

1. If Hop == 0 → do not forward
2. If SrcID == local ID → drop
3. If packet already seen → drop
4. Decrement Hop
5. Forward in own slot

### Duplicate Detection

- Maintain rolling window of Seq per SrcID
- Window size: implementation-defined (e.g. 8)

---

## 9. Control Plane (CSMA)

Control packets are transmitted **outside TDMA slots** using contention-based access.

### JOIN Packet

Payload:

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 1 | RequestedHop | Max hops supported |
| 1 | 1 | Capabilities | Codec, features |

### JOIN_ACK Packet

Payload:

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 1 | AssignedID | Node ID |
| 1 | 1 | SlotIndex | TDMA slot |
| 2 | 1 | MasterID | Time master |

---

## 10. Time Synchronization (SYNC)

### SYNC Packet

Payload:

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 4 | FrameCounter | TDMA frame number |
| 4 | 2 | DriftPPM | Estimated clock drift |

### Behavior

- Master sends SYNC periodically
- Slaves phase-lock
- Loss of SYNC triggers re-election

---

## 11. Slot Map (SLOT_MAP)

Broadcast by master when:
- Node joins/leaves
- Reconfiguration needed

Payload:

| Offset | Size | Field | Description |
|------|----|------|------------|
| 0 | 1 | SlotCount | Number of slots |
| 1 | N | SlotIDs | Ordered list of node IDs |

---

## 12. Failure Handling

### Audio Loss

- Missing frames are concealed by Opus PLC
- No retransmission

### Node Loss

- Missing KEEPALIVE → slot reclaimed

### Master Loss

- Lowest Node ID becomes provisional master
- New SYNC issued

---

## 13. ESP32 ↔ nRF54 Inter-MCU Protocol (Phase 2)

### Transport
- SPI preferred
- UART acceptable for early prototypes

### Message Types

| Type | Direction | Purpose |
|----|----------|--------|
| AUDIO_IN | ESP → nRF | Send encoded audio |
| AUDIO_OUT | nRF → ESP | Received audio |
| CTRL | Bidirectional | Control messages |

### Audio Message

| Offset | Size | Field |
|------|----|------|
| 0 | 1 | StreamID |
| 1 | 1 | Seq |
| 2 | 1 | Hop |
| 3 | N | OpusData |

---

## 14. Security (Deferred)

Initial versions focus on correctness and performance.

Planned:
- Group pre-shared key
- Packet authentication
- Optional encryption

Security must not compromise latency.

---

## 15. Versioning

- Major version increments break compatibility
- Minor version adds optional fields
- Version field in header enforces negotiation

---

## 16. Status

This document defines **Protocol v0.1**.

It is expected to evolve, but backward compatibility should be preserved where practical.
