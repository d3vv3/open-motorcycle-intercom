# OpenHelmet

<p align="center">
    <img src="assets/logo.svg" alt="OpenHelmet Logo" width="200"/>
    <br/>
    <b>Open Source Mesh and Bluetooth Motorcycle Intercom</b>
    <br/>
    <a href="https://openhelmet.devve.space">website</a>
</p>

OpenHelmet is an **open-source, motorcycle intercom system**
designed to outperform current proprietary solutions
in **latency, scalability, transparency, and hackability**.

## Key goals

- Open protocol and open firmware
- Commodity, off-the-shelf hardware
- Use any microphone and any speakers from your existing helmet setup
- Mesh networking for 4-8+ riders
- Real-time full-duplex voice
- 8-16 hours of active riding per charge
- Interoperability with legacy Bluetooth intercoms (Cardo, Sena, etc.) via standard profiles

This project explicitly avoids reverse-engineering proprietary intercom protocols.
Interoperability is achieved **only through standard Bluetooth audio profiles**.

---

## Why

- Cardo/Sena mesh protocols are proprietary and closed
- True mesh interoperability with them is impossible today
- Voice communication requires **deterministic latency**, not best-effort networking

As a result, OpenHelmet is designed around **custom real-time audio transport**.

---

## Roadmap

### Single-MCU

- [x] **ESP32-S3**
- [x] TDMA mesh protocol over [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) (2.4 GHz)
- [x] Opus low-bitrate voice
- [x] Analog microphone via headphone jack
- [x] Speaker output via headphone jack
- [x] VOX for voice activation
- [x] Configure ESP32 tx power to 20 dBm (100 mW) for ESP-NOW
- [x] Silence suppression (Opus DTX) - no radio TX during silence

### Dual-MCU

- [x] **ESP32-S3 + Nordic nRF52 52840**
- [x] TDMA mesh protocol over [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) radio with custom PHY control
  > ESB runs at 250 kbps with 8 dBm TX power (`OMI_ESB_BITRATE` / `OMI_ESB_TX_POWER_DBM`).
- [x] SPI audio & control bridge between MCUs
  > Needed to make the SPI handling to take place only in 1 core of the ESP32
- [x] Noise supression, echo cancellation
- [ ] Allow user to choose between [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) and [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html)
  > ESP-NOW is not interoperable with nRF52' ESB.
- [ ] Add [nRF21540 RF FEM](https://www.nordicsemi.com/Products/nRF21540) to boost range
  > [Validation](https://www.nordicsemi.com/Nordic-news/2023/01/Aptener-Mechatronics-BluArmor-C30-Helmet-Comms-Unit-employs-nRF52840-SoC-and-nRF21540-RF-FEM)

### Triple-MCU

- [ ] **FSC-BT1026D** (Qualcom QCC3034) BT5.1 audio SoC
- [ ] Phone interop via HFP profile
- [ ] Audio mixing for legacy intercom bridging

### Custom PCB

- [ ] PCB design with integrated audio codec and power management
- [ ] Battery power with charging circuit
- [ ] USB-C for firmware flashing
- [ ] Buttons
- [ ] Open-source manufacturing files
- [ ] Testing on real motorcycle rides

### Niceties

- [ ] Good configuration menu buttons
- [ ] LED status indicators
- [ ] Channels for multiple groups
  > Channels are represented as an LED color, e.g. green=1, red=2, blue=3, etc.

---

## Transport Protocol Design

### Why Not Bluetooth Mesh / BLE Audio

- High latency
- Unpredictable scheduling
- Poor scaling for continuous audio

These technologies are unsuitable for real-time group voice.

---

## Custom Mesh Strategy

### Hybrid TDMA + CSMA

OpenHelmet uses a **hybrid MAC design**:

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

## Quick Start

### Prerequisites

- [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/)
- ESP32-S3-DevKitC-1-N8R8
- (Optional) XIAO nRF52840
- USB-C data cables
- [Audio Hardware Wiring Guide](docs/wiring.md)
- (Optional) [KiCad](https://www.kicad.org/)

### Build

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build firmware
idf.py build
```

### Flash

```bash
# Find your serial port
ls /dev/ttyACM* /dev/ttyUSB*

# Flash firmware (replace PORT with your device)
idf.py -p /dev/ttyACM0 flash
```

### Monitor Serial Output

```bash
# Option 1: Using idf.py (interactive, requires TTY)
idf.py -p /dev/ttyACM0 monitor

# Option 2: Using stty + cat (non-interactive)
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
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
| Device not found | Use a data cable (not charge-only), try different USB port |
| Garbage characters in monitor | Wrong baud rate, use 115200 |
