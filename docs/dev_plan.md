# Development Plan (Authoritative)

This document defines the **execution plan** for the Open Motorcycle Intercom (OMI).

It is intentionally concrete. Each phase has **exit criteria**. If a phase does not meet its criteria, the project does not advance.

[TOC]

---

## Guiding Rules

1. No feature without a power budget
2. No RF work without instrumentation
3. No audio feature without latency measurement
4. No mesh scaling before 2-node stability
5. Docs define behavior — code must follow

---

## Phase 0 – Tooling & Baseline (1–2 weeks)

### Goals
- Establish reproducible dev environment
- Avoid SDK chaos later

### Tasks
- Repo structure
- Build system (ESP-IDF + CMake)
- Opus build integration
- Logging framework with timestamps
- Hardware selection (ESP32-S3 dev board)

### Deliverables
- Firmware skeleton boots
- Audio task runs idle
- Power measurement setup ready

### Exit Criteria
- Can flash reliably
- Logs include monotonic timestamps

---

## Phase 1 – Audio Bring-Up (2–3 weeks)

### Goals
- Prove audio pipeline works locally

### Tasks
- Mic input + ADC
- HPF implementation
- Opus encode/decode loopback
- DAC / headphone output
- VOX gating

### Measurements
- Encode/decode timing
- End-to-end latency (loopback)

### Exit Criteria
- <50 ms local latency
- No audio glitches over 30 min

---

## Phase 2 – Single-Hop RF Link (2–3 weeks)

### Goals
- Send voice between two devices

### Tasks
- TDMA frame timer
- Slot TX/RX over ESP-NOW
- Packet loss stats
- Jitter buffer

### Notes
- ESP-NOW provides the transport layer (PHY/MAC)
- OMI implements TDMA scheduling and routing on top
- This allows focusing on protocol logic before RF complexity
- True custom PHY control comes with nRF54 in Phase 6

### Measurements
- Packet loss vs distance
- Latency distribution

### Exit Criteria
- Clear voice at 50–100 m LOS
- Packet loss <10% sustained

---

## Phase 3 – Power Discipline (1–2 weeks)

### Goals
- Enforce power model

### Tasks
- Power states implementation
- Duty cycling radio
- VOX-driven TX
- Idle sleep validation

### Measurements
- Current per state
- 8h continuous usage test

### Exit Criteria
- Meets power.md estimates
- No unexpected wakeups

---

## Phase 4 – Mesh MVP (3–4 weeks)

### Goals
- 3–4 rider mesh

### Tasks
- Slot assignment
- Join/leave handling
- Master election
- Forwarding logic

### Measurements
- End-to-end latency (multi-hop)
- Slot drift

### Exit Criteria
- Stable 4-node mesh for 1h
- Latency <100 ms

---

## Phase 5 – Bluetooth Integration (2–3 weeks)

### Goals
- Phone compatibility

### Tasks
- HFP audio path
- Mixing + ducking
- Call priority logic

### Exit Criteria
- Phone calls usable at speed
- Mesh audio unaffected

---

## Phase 6 – Dual-MCU Prototype (nRF54) (4–6 weeks)

### Goals
- Validate split architecture

### Tasks
- SPI protocol implementation
- Time sync enforcement
- Failure injection tests

### Exit Criteria
- Identical audio behavior vs single MCU
- RF crashes do not affect audio

---

## Phase 7 – Security Implementation (2–3 weeks)

### Goals
- Secure mesh communication

### Tasks
- Group pre-shared key distribution
- Packet authentication (HMAC)
- Optional payload encryption (AES-CCM)
- Key rotation mechanism

### Measurements
- Encryption latency overhead
- Memory usage impact

### Exit Criteria
- No measurable latency regression (< 5 ms added)
- Authenticated packets only accepted
- Key compromise does not affect other groups

---

## Phase 8 – Scaling & Stress (Ongoing)

### Goals
- 6–8 rider stability

### Tasks
- Slot pressure tests
- Packet loss injection
- Long-duration rides

### Exit Criteria
- 8 nodes stable for 2h
- No cascading failures

---

## Phase 9 – Hardware Refinement (Optional)

### Goals
- Prepare for real-world usage

### Tasks
- Custom PCB
- Battery optimization
- Waterproof enclosure

---

## Anti-Goals (Explicit)

- Music streaming
- Stereo audio
- Cloud connectivity
- App-first UX

These kill power and reliability.

---

## Kill Switch Conditions

Project stops or pivots if:
- Latency exceeds 120 ms
- Battery <8h continuous
- RF instability persists beyond Phase 4

---

## Status

This plan is the **single source of truth** for execution.

Skipping phases is forbidden.

