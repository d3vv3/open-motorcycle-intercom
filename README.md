# Open Motorcycle Intercom (OMI)

OMI is an **open‑source, motorcycle intercom system**
designed to outperform current proprietary solutions
in **latency, scalability, transparency, and hackability**.

[TOC]

## Key goals
- Open protocol and open firmware
- Real‑time full‑duplex voice
- Mesh networking for 4–8+ riders
- 8–16 hours of active riding per charge
- Interoperability with legacy Bluetooth intercoms (Cardo, Sena, etc.) via standard profiles
- Commodity, off‑the‑shelf hardware

This project explicitly avoids reverse‑engineering proprietary intercom protocols.
Interoperability is achieved **only through standard Bluetooth audio profiles**.

---

## Quick Start

### Prerequisites

- [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/)
- ESP32-S3-DevKitC-1-N8R8 (must have PSRAM)
- USB-C cable
- [Audio Hardware Wiring Guide](docs/wiring.md)

### Build

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build firmware
cd omi
idf.py build
```

### Flash

```bash
# Find your serial port
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Flash firmware (replace PORT with your device)
idf.py -p /dev/ttyACM0 flash
```

### Monitor Serial Output

```bash
# Option 1: Using idf.py (interactive, requires TTY)
idf.py -p /dev/ttyACM0 monitor

# Option 2: Using stty + cat (non-interactive)
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0

# Option 3: Flash and monitor combined
idf.py -p /dev/ttyACM0 flash monitor
```

**Exit monitor:** `Ctrl+]`

### Expected Boot Log

```
I (xxx) omi: ========================================
I (xxx) omi: OMI - Open Motorcycle Intercom
I (xxx) omi: Boot time: 0 ms
I (xxx) omi: Opus version: libopus x.x.x
I (xxx) omi: Free heap: 275432 bytes
I (xxx) omi: ========================================
I (xxx) audio: Initializing audio subsystem
I (xxx) audio: Audio task heartbeat: loops=0, encoded=0, decoded=0
```

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Permission denied on serial port | `sudo usermod -a -G dialout $USER` then log out/in |
| Device not found | Use a data cable (not charge-only), try different USB port |
| Garbage characters in monitor | Wrong baud rate, use 115200 |
| PSRAM not detected | Ensure you have the N8R8 variant board |

---

## Why

- Cardo/Sena mesh protocols are proprietary and closed
- True mesh interoperability with them is impossible today
- Voice communication requires **deterministic latency**, not best‑effort networking
- 2.4 GHz spectrum is congested; protocol efficiency matters more than raw bandwidth

As a result, OMI is designed around **custom real‑time audio transport**,
not existing consumer mesh standards.

---

## High‑Level Architecture

### Phase 1 (Single‑MCU)
- **ESP32‑S3**
- TDMA mesh protocol over ESP-NOW (2.4 GHz)
- Opus low‑bitrate voice
- Bluetooth Classic gateway

### Phase 2 (Dual‑MCU, Production‑Grade)
- **Nordic nRF54**: ESB radio with custom PHY control
- **ESP32‑S3**: Bluetooth Classic + audio mixing
- UART/SPI audio & control bridge between MCUs

Phase 1 uses ESP-NOW as the transport layer for rapid prototyping.
"Custom mesh" refers to the TDMA scheduling and routing protocol built on top.
True low-level PHY control comes with nRF54's Enhanced ShockBurst in Phase 2.

---

## Audio Codec Choice: Opus (Low Bitrate)

### Why Opus
Opus is mandatory for this project.
No alternative matches its efficiency for real‑time voice.

| Codec | Typical Bitrate | Quality | Packet Loss Resilience |
|------|---------------|---------|------------------------|
| CVSD | ~64 kbps | Poor | Very poor |
| SBC | 48–64 kbps | Fair | Poor |
| **Opus** | **8–16 kbps** | **Good** | **Excellent** |

### Target Configuration
- Mode: VoIP
- Sample rate: 16 kHz
- Channels: Mono
- Bitrate: 8–16 kbps
- Frame size: 20 ms
- FEC: Enabled

This balances clarity, wind tolerance, latency, and bandwidth.

---

## Transport Protocol Design

### Why Not Bluetooth Mesh / BLE Audio
- High latency
- Unpredictable scheduling
- Poor scaling for continuous audio

These technologies are unsuitable for real‑time group voice.

---

## Custom Mesh Strategy

### Hybrid TDMA + CSMA

OMI uses a **hybrid MAC design**:

- **TDMA** for voice frames
- **CSMA** for control traffic

#### TDMA (Voice)
- Fixed time slots per node
- Deterministic latency
- Zero collisions
- Linear scaling with rider count

Frame structure (20 ms frame, 8 riders):
```
| Slot 1 | Slot 2 | Slot 3 | ... | Slot 8 | Control | Guard |
|  2 ms  |  2 ms  |  2 ms  | ... |  2 ms  |   2 ms  |  2 ms |
```

- Voice slots: 8 × 2 ms = 16 ms
- Control window: 2 ms (CSMA for join/leave/sync)
- Guard time: 2 ms (absorbs clock drift)

Motorcycle groups are small and topology changes slowly, making TDMA practical.

#### CSMA (Control)
Used for:
- Joining/leaving the group
- Topology updates
- Slot renegotiation

---

## 7. Nordic ESB (Enhanced ShockBurst)

### What It Is
Nordic ESB is a **lightweight proprietary 2.4 GHz packet radio protocol**
with minimal overhead and very low latency.

### Why It Matters
- Sub‑millisecond packet latency
- Small stack footprint
- Predictable timing
- Excellent power efficiency

### Tradeoffs
- Nordic‑only
- Requires implementing:
  - Addressing
  - Routing
  - Security

For OMI, these are acceptable and desirable tradeoffs.

---

## Hardware Choices Explained

### ESP32‑S3 (Phase 1 Backbone)

**Why it is used initially:**
- Bluetooth Classic (HFP/A2DP) support
- Sufficient DSP for Opus
- Large community and low cost
- Easy prototyping

**Limitations:**
- Higher power consumption
- Bluetooth stack complexity
- Less deterministic radio control

**Typical Power Use:**
- Bluetooth audio active: ~60–90 mA
- Average system: ~120–150 mW

**Battery Life Estimate:**
- 1000 mAh @ 3.7 V ≈ 3.7 Wh
- ~24 hours mixed use achievable

---

### Nordic nRF52 (Reference / Legacy)

- Excellent low‑power radio
- Mature SDK
- Limited RAM for Opus + mesh
- Bluetooth Classic mostly unavailable

Good for experimentation, not ideal for full system.

---

### Nordic nRF54 (Target Mesh MCU)

**Why nRF54:**
- Much more RAM and CPU
- Designed for real‑time wireless
- Extremely low power consumption
- Ideal for ESB‑based mesh

**Why not yet:**
- New platform
- Higher cost
- Smaller ecosystem (for now)

---

## Dual‑MCU Architecture (Final Target)

### Responsibilities Split

**nRF54**
- ESB mesh
- TDMA scheduling
- Routing
- Ultra‑low power operation

**ESP32‑S3**
- Bluetooth Classic interop
- Audio mixing
- User interface

This keeps Bluetooth complexity isolated and preserves battery life.

---

## Interoperability Strategy

OMI **does not join proprietary meshes**.

Instead:
- Acts as Bluetooth headset or phone
- Uses HFP profile
- Bridges one legacy intercom into the open mesh

This is the only realistic interoperability model.

---

## Power & Battery Targets

### Design Targets
- Active riding: 8–16 hours
- Standby: multiple days

### Battery
- Li‑Po 1000–1500 mAh
- USB‑C charging

### Average Power Budget
- Target: <150 mW
- Mesh duty cycling
- VOX‑based audio gating

---

## Scalability Model

- 2–4 riders: full mesh
- 5–8 riders: TDMA with relays
- 8–10 riders: diminishing returns

Linear motorcycle formations are explicitly exploited.

---

## Hardware BOM (Development)

### Recommended Dev Board

**ESP32-S3-DevKitC-1-N8R8**

| Spec | Value |
|------|-------|
| MCU | ESP32-S3 (dual-core Xtensa LX7 @ 240 MHz) |
| Flash | 8 MB |
| PSRAM | 8 MB (required for Opus + mesh buffers) |
| Bluetooth | Classic + BLE 5.0 |
| WiFi | 802.11 b/g/n (ESP-NOW capable) |

**Important:** Get the N8R8 variant (with PSRAM). Boards without PSRAM will not have enough RAM.

### Audio Components (Phase 1)

| Component | Part Number | Purpose | Qty |
|-----------|-------------|---------|-----|
| I2S Microphone | INMP441 | Audio capture | 1 |
| I2S Amplifier | MAX98357A | Speaker output | 1 |
| Speaker | 4 ohm / 3W | Audio playback | 1 |

### Minimum Development Kit

| Item | Est. Cost |
|------|-----------|
| ESP32-S3-DevKitC-1-N8R8 | $12 |
| INMP441 breakout | $4 |
| MAX98357A breakout | $4 |
| Small speaker | $2 |
| Breadboard + jumpers | $5 |
| **Total** | **~$27** |

### Multi-Node Testing (Phase 2+)

For mesh development, obtain **2–3 identical setups**.

### Boards to Avoid

| Board | Reason |
|-------|--------|
| ESP32-S3 without PSRAM | Insufficient RAM |
| ESP32 (original) | Less RAM, no native USB debug |
| ESP32-C3 | Single core, no Bluetooth Classic |
| ESP32-S2 | No Bluetooth |

### Future Hardware (Phase 4+)

| Component | Purpose |
|-----------|---------|
| nRF54 DK | ESB mesh radio MCU |
| Custom PCB | Production prototype |
| Li-Po 1000–1500 mAh | Battery |
| TP4056 or BQ24072 | USB-C charging IC |

---

## Project Roadmap

### Phase 0 – Research & Validation
- Opus benchmarks on ESP32‑S3
- Latency measurements
- Power profiling

### Phase 1 – MVP (Single MCU)
- ESP32‑S3 firmware
- Opus voice
- Simple TDMA mesh
- Push‑to‑talk

### Phase 2 – Mesh Hardening
- Dynamic slot allocation
- Packet loss recovery
- Group management

### Phase 3 – Bluetooth Gateway
- HFP implementation
- Audio mixing
- Legacy intercom bridge

### Phase 4 – Dual‑MCU Prototype
- nRF54 mesh firmware
- ESP32 bridge firmware
- SPI/UART protocol

### Phase 5 – Hardware Refinement
- Custom PCB
- Power optimization
- RF tuning

---

## Guiding Principles

- Determinism over throughput
- Simplicity over standards
- Open over proprietary
- Physics always wins

---

## Status

This README defines the **technical north star** of the project.
Implementation details will evolve,
but the architectural decisions documented here
are intentional and grounded in real‑world constraints.

