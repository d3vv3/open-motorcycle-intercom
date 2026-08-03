# OpenHelmet

<p align="center">
    <img src="assets/logo.svg" alt="OpenHelmet Logo" width="200"/>
    <br/>
    <b>Open Source Mesh and Bluetooth Motorcycle Intercom</b>
    <br/>
    <a href="https://openhelmet.devve.space">website</a> | <a href="https://discord.gg/XxBSnwSDst">discord</a>
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

Enough to get you started! Check [the wiring guide](docs/wiring.md) for details on a single-MCU prototype. It needs an ESP32-S3, an amplified microphone input, and an I2S DAC or amplifier for audio output.

- [x] **ESP32-S3**
- [x] TDMA mesh protocol over [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) (2.4 GHz)
- [x] Opus low-bitrate voice
- [x] Analog microphone via headphone jack
- [x] Speaker output via headphone jack
- [x] VOX for voice activation
- [x] Configure ESP32 tx power to 20 dBm (100 mW) for ESP-NOW
- [x] Silence suppression (Opus DTX) - most silent audio frames are suppressed; control traffic and occasional comfort updates remain
- [x] Three-source receive mixer with first-speaker retention; a new active source can replace the longest-silent source after 400 ms

### Dual-MCU

Development and validation only.
Check [the wiring guide](docs/wiring.md) for details on how to build a dual-MCU intercom with an ESP32-S3, nRF52840, and speaker/mic hardware.

The nRF52840 firmware uses the radio's +8 dBm output directly. An nRF21540 is optional for additional range, not required for basic operation. Rev2 includes the nRF21540 in its hardware design, but firmware control of the FEM and its range benefit have not yet been validated.

- [x] **ESP32-S3 + Nordic nRF52840**
- [x] TDMA mesh protocol over [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) radio with custom PHY control
  > ESB is configured for 2 Mbps with +8 dBm TX power (`OMI_ESB_BITRATE` / `OMI_ESB_TX_POWER_DBM`).
- [x] SPI audio & control bridge between MCUs
  > The nRF52840 is the SPI master and polls the ESP32 bridge every 2 ms.
- [x] Noise suppression and echo cancellation paths
- [ ] Allow user to choose between [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) and [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html)
  > ESP-NOW is not interoperable with nRF52 ESB. The firmware currently selects ESB when the bridge is detected and otherwise falls back to ESP-NOW; comparative range, power, and latency remain to be measured.
- [x] Include an optional [nRF21540 RF FEM](https://www.nordicsemi.com/Products/nRF21540) in the Rev2 design
  > Hardware bring-up and firmware control of `FEM_TX_EN`, `FEM_RX_EN`, `FEM_MODE`, `FEM_PDN`, and the SPI control interface are still outstanding.
- [x] Three-source receive mixer with first-speaker retention and 400 ms silent-source eviction
- [x] Relay grants limited to at most two active speakers

### Triple-MCU

- [ ] **FSC-BT1026D** (Qualcom QCC3034) BT5.1 audio SoC
- [ ] Phone interop via HFP profile
- [ ] Audio mixing for legacy intercom bridging

### Custom PCB

- [x] Rev2 PCB design files with integrated audio, dual MCUs, RF FEM, USB-C, charger, and regulator
- [x] Close schematic ERC findings
- [ ] Close PCB DRC, routing, and manufacturing-rule findings
- [ ] Validate battery charging and regulated power rails on assembled hardware
- [ ] Validate USB recovery and flashing
- [ ] Bring up buttons, audio, RF, and FEM on Rev2 hardware
- [ ] Open-source manufacturing files
- [ ] Testing on real motorcycle rides

Rev2 is a design in progress, not completed hardware. DRC closure, manufacturing output generation, assembly, and board bring-up are unfinished.

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

### Scheduled Voice and Control

OpenHelmet divides each 20 ms frame into scheduled voice slots and a control window:

- **TDMA** for voice frames
- A rotating owner for joined-node control traffic; coordinator SYNC frames have a reserved window
- Bounded randomized contention for unjoined ESP-NOW JOIN requests

#### TDMA (Voice)

- Fixed time slots per node
- One bounded transmission opportunity per active node and frame
- Deadline checks drop late work rather than transmitting outside its window

Frame structure (20 ms frame, 8 riders):

```
| Slot 1 | Slot 2 | Slot 3 | ... | Slot 8 | Control | Margin |
|  2 ms  |  2 ms  |  2 ms  | ... |  2 ms  |   2 ms  |  2 ms |
```

- Voice slots: 8 × 2 ms = 16 ms
- Control window: 2 ms (scheduled sync/topology/status traffic)
- Frame margin: 2 ms; voice and control deadlines also reserve guard time

Motorcycle groups are small and topology changes slowly, making TDMA practical.

#### Control

Used for:
- Joining/leaving the group
- Topology updates
- Slot maps and synchronization

Joined-node control packets use a bounded queue and scheduled ownership. Unassigned ESP-NOW JOIN requests use a minimum interval plus randomized jitter. These rules reduce contention, but they do not guarantee collision-free RF operation.

---

## Quick Start

### Prerequisites

- [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/)
- ESP32-S3 development board with at least 8 MB flash (the DevKitC-1 N8R8 is supported, but this firmware does not currently enable or require PSRAM)
- (Optional) XIAO nRF52840
- USB-C data cables
- [Development environment and ESP32 setup](docs/getting_started.md)
- [Audio and dual-MCU wiring guide](docs/wiring.md)
- (Optional) [KiCad](https://www.kicad.org/)

### Build

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build firmware
idf.py build
```

For the optional XIAO nRF52840 firmware, use nRF Connect SDK v2.7.0 and the same board target as CI:

```bash
west build -b xiao_ble/nrf52840 ./nrf_mesh -d ./build-nrf
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
I (xxx) omi: Phase 2: Single-Hop RF Link
I (xxx) omi: ========================================
I (xxx) omi: Boot time: <milliseconds> ms
I (xxx) omi: IDF version: v5.5.x
I (xxx) omi: Free heap: <bytes> bytes
I (xxx) omi: ========================================
I (xxx) omi: nRF52840 not detected - using ESP-NOW transport
I (xxx) omi: Mesh networking ready (desired state: disabled)
I (xxx) omi: System running!
```

With a working nRF bridge, the transport line is currently logged as `nRF52840 detected on UART - using ESB transport`; the `UART` name is legacy wording for the SPI bridge. Timestamps and heap values vary by build and are not setup pass/fail criteria.

On a fresh device, mesh intent defaults to disabled. Hold the ESP Boot button for
two seconds to enable it. The setting is stored in NVS for later boots.

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Device not found | Use a data cable (not charge-only), try different USB port |
| Garbage characters in monitor | Wrong baud rate, use 115200 |
