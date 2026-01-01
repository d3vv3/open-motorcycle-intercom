# Inter-MCU Architecture & Contract

This document defines the **strict contract** between MCUs in the Open Motorcycle Intercom (OMI).

The goal is not performance maximalism — it is **fault isolation, power efficiency, and timing determinism**.

[TOC]

---

## 1. Why Dual MCU?

A single MCU *can* do everything — until it can’t.

Problems avoided by splitting:
- RF stack jitter breaking audio timing
- Bluetooth firmware crashes killing mesh
- Power-hungry radios preventing deep sleep
- Vendor lock-in

**Rule:** Audio timing must never depend on Bluetooth or mesh firmware behavior.

---

## 2. MCU Roles

### 2.1 Application MCU (ESP32-S3 initially)

Responsibilities:
- Audio I/O (ADC/DAC)
- Opus encode/decode
- VOX / PTT logic
- Audio mixing
- TDMA scheduler (logical)
- User interface
- Bluetooth Classic (HFP/A2DP)

This MCU owns *time*.

---

### 2.2 Radio MCU (nRF54)

Responsibilities:
- Mesh radio (ESB / custom PHY)
- Slot-level TX/RX
- Time sync enforcement
- Low-power sleep scheduling
- Encryption / authentication

This MCU owns *RF correctness*.

---

## 3. Clock Ownership & Time Model

- Application MCU defines **audio frame cadence** (20 ms)
- Radio MCU aligns slots to this cadence

Radio MCU is **not allowed** to invent timing.

Time flows:
```
Audio frame tick → TDMA slot → RF TX/RX
```

---

## 4. Interconnect Options

Preferred:
- SPI (10–20 MHz)

Fallback:
- UART (DMA only, no interrupts)

I2C is explicitly rejected.

---

## 5. Message Framing

All messages use fixed headers:

```
| Type | Length | Seq | Payload | CRC |
```

Rules:
- No dynamic allocation
- No variable-length queues
- Explicit backpressure

---

## 6. Data Classes

### 6.1 Real-Time Audio Frames

- Exactly one Opus frame per message
- Must arrive before slot deadline

Late frames are dropped silently.

---

### 6.2 Control Messages

Examples:
- Slot assignment
- Peer join/leave
- RSSI reports

Lower priority than audio.

---

### 6.3 Management Messages

Examples:
- Firmware update
- Debug logging
- Diagnostics

May be throttled aggressively.

---

## 7. Backpressure & Flow Control

Rules:
- Radio MCU advertises available TX slots
- Application MCU may only enqueue when slots exist

If queue is full:
- Drop oldest non-audio
- Never block audio pipeline

---

## 8. Failure Isolation

### Radio MCU Failure

Symptoms:
- Slot loss
- No RF ACKs

Response:
- Application MCU continues local audio
- Rejoin attempt after timeout

---

### Application MCU Failure

Symptoms:
- No frames
- No control messages

Response:
- Radio MCU enters low-power idle

---

## 9. Power Coordination

Radio MCU controls RF sleep states.

Application MCU signals:
- Active audio
- Idle
- Deep sleep request

Radio MCU may reject deep sleep if mesh timing requires wake.

---

## 10. Security Boundary

- Keys stored only on Radio MCU
- Application MCU never sees mesh keys

This prevents Bluetooth compromise from leaking mesh credentials.

---

## 11. Debug & Telemetry

Mandatory counters:
- Late audio frames
- Slot misses
- SPI underruns
- Clock drift

Telemetry must be pull-based.

---

## 12. Migration Path

| Phase | Configuration | Notes |
|-------|---------------|-------|
| Phase 1–5 | ESP32-S3 only | Single MCU, ESP-NOW transport |
| Phase 6 | ESP32-S3 + nRF54 | Dual MCU, ESB transport |
| Future | nRF54 dual-core | Optional single-chip solution |

The inter-MCU contract remains unchanged across configurations.

---

## 13. Design Rationale Summary

- Audio owns time
- RF owns correctness
- Backpressure is explicit
- Failures degrade gracefully

This contract is intentionally strict.

---

## 14. Status

This document defines the **non-negotiable MCU interface** for OMI.

