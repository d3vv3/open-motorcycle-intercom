# OpenHelmet

<p align="center">
    <img src="assets/logo.svg" alt="OpenHelmet Logo" width="200"/>
    <br/>
    <b>Open Source Mesh and Bluetooth Motorcycle Intercom</b>
    <br/>
    <br/>
    <a href="https://openhelmet.devve.space">website</a> | <a href="https://discord.gg/XxBSnwSDst">discord</a>
</p>

OpenHelmet is an **open-source, motorcycle intercom system**
designed to outperform current proprietary solutions in **latency, scalability, transparency, and hackability**.

## Key goals

- Open protocol and open firmware
- Commodity, off-the-shelf hardware
- Use any microphone and any speakers from your existing helmet setup
- Mesh networking for 4-8+ riders
- Real-time full-duplex voice
- 8-16 hours of active riding per charge
- Bluetooth phone and media audio (calls, GPS, music, etc)
- Interoperability with legacy Bluetooth intercoms (Cardo, Sena, etc.) via standard profiles

This project explicitly avoids reverse-engineering proprietary intercom protocols.
Interoperability is achieved **only through standard Bluetooth audio profiles**.

---

## Why

- Cardo/Sena mesh protocols are proprietary and closed
- True mesh interoperability with and between them is impossible today
- Voice communication requires **deterministic latency**, not best-effort networking

As a result, OpenHelmet is designed around **custom real-time audio transport**.

---

## Roadmap

### Single-MCU

See [the wiring guide](docs/wiring.md) for the ESP32-S31 Function CoreBoard-1.

- [x] **ESP32-S31 Function CoreBoard-1** with 16 MB flash and 16 MB PSRAM
- [x] Onboard ES8311 capture/codec and NS4150B/J9 output
- [x] TDMA mesh protocol over [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) (2.4 GHz)
- [x] LC3 low-bitrate voice
- [x] Classic Bluetooth A2DP sink, AVRCP, and HFP HF client
- [x] VOX for voice activation
- [x] Configure ESP32 tx power to 20 dBm (100 mW) for ESP-NOW
- [x] Silence suppression - most silent audio frames are suppressed; control traffic and occasional comfort updates remain
- [x] Receive mixing with first-speaker retention; a new active source can replace the longest-silent source after 400 ms

Limitations:
- ESP-NOW is not interoperable with nRF52 ESB.
- The ESP-NOW and Bluetooth stacks share the same radio, so mesh cannot reach realtime performance when Bluetooth is active.

### Dual-MCU

Development and validation only. Check [the wiring guide](docs/wiring.md) for
the S31/nRF52840 SPI wiring.

The nRF52840 firmware uses the radio's +8 dBm output directly.
An nRF21540 is needed for additional range, efficiency and antenna radio distribution. See "Custom PCB" section below.

> The nRF21540 is needed for actual range improvements.
> The nRF52840 alone is not a range improvement over the ESP32-S31.

- [x] **ESP32-S31 + Nordic nRF52840**
- [x] TDMA mesh protocol over [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) radio with custom PHY control
  > ESB is configured for 2 Mbps with +8 dBm TX power (`OMI_ESB_BITRATE` / `OMI_ESB_TX_POWER_DBM`).
- [x] SPI audio & control bridge between MCUs
  > The nRF52840 is the SPI master and polls the ESP32 bridge every 2 ms.
- [x] Noise suppression and echo cancellation paths
- [ ] Allow user to choose between [ESP-NOW](https://www.espressif.com/en/solutions/low-power-solutions/esp-now) and [ESB](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html)
  > ESP-NOW is not interoperable with nRF52 ESB. The firmware currently selects ESB when the bridge is detected and otherwise falls back to ESP-NOW; comparative range, power, and latency remain to be measured.
- [x] Relay grants limited to at most two active speakers
- [x] Three-source receive mixer with first-speaker retention and 400 ms silent-source eviction
- [x] Silence suppression - most silent audio frames are suppressed; control traffic and occasional comfort updates remain
- [x] Bluetooth media playback (music, GPS, etc.) during mesh operation.

### Custom PCB

- [x] Schematic
- [ ] PCB layout
- [x] Include [nRF21540 RF FEM](https://www.nordicsemi.com/Products/nRF21540) in the schematics and PCB design

See the [wiring guide](docs/wiring.md) for the current assembled prototype.

### Ongoing

- [ ] Drop mesh during phone call; resume mesh after call ends
- [ ] Mesh channels with RGB LED channel indicator by color (e.g. green=1, red=2, blue=3, etc.)
- [ ] Buttons for channel selection, volume, mesh toggle, and Bluetooth pairing
- [ ] Replace beeps with voice prompts for button operations
- [ ] Power budgeting and optimization for 8-16 hours of active riding per charge
- [ ] Testing on real motorcycle rides
- [ ] E2EE

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

OpenHelmet divides each 20 ms TDMA frame into scheduled voice slots and a control window:

- **TDMA** for mesh audio packets
- A rotating owner for joined-node control traffic; coordinator SYNC frames have a reserved window
- Bounded randomized contention for unjoined ESP-NOW JOIN requests

#### TDMA (Voice)

Each current mesh audio payload represents 20 ms of sound: two 10 ms LC3 frames, encoded as 24 bytes each. These 48 bytes travel together in one packet, not in separate TDMA slots.

On the nRF transport, a bundle can also carry previous audio frames for loss recovery. This increases packet size without changing the 20 ms transmission cadence.

- Fixed time slots per node
- One bounded transmission opportunity per node per 20 ms TDMA frame
- Deadline checks reject work that is already late; they do not guarantee exact RF completion timing

Configured frame structure (20 ms frame, up to 8 nodes; durations include guard time):

```
| Slot 1 | Slot 2 | Slot 3 | ... | Slot 8 | Control | Margin |
|  2 ms  |  2 ms  |  2 ms  | ... |  2 ms  |   2 ms  |  2 ms |
```

- Voice slots: 8 × 2 ms = 16 ms
- Control window: 2 ms (scheduled sync/topology/status traffic)
- Frame margin: 2 ms
- Voice and control deadlines reserve the last 500 µs of each nominal 2 ms window, leaving 1.5 ms before the deadline

VOX suppresses local LC3 encoding and audio transmission during silence, after its hangover period. Capture and voice detection continue. The TDMA schedule remains unchanged, and synchronization, keepalives, and required relay/control traffic continue.

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

- [ESP-IDF v6.1 preview](https://docs.espressif.com/projects/esp-idf/en/release-v6.1/esp32s31/get-started/)
- [ESP32-S31 Function CoreBoard-1](https://docs.espressif.com/projects/esp-idf/en/release-v6.1/esp32s31/get-started/) with 16 MB flash and 16 MB PSRAM
- ESP32-S31 support in ESP-IDF v6.1 is preview
- CI pins a tested ESP-IDF image that includes the upstream S31 Bluetooth
  sniff-subrate fix. The mutable `v6.1` tag does not contain that fix.
- (Optional) XIAO nRF52840
- USB-C data cables
- [Development environment and ESP32 setup](docs/getting_started.md)
- [Audio and dual-MCU wiring guide](docs/wiring.md)
- (Optional) [KiCad](https://www.kicad.org/)

### Build

```bash
# Optional local environment example (path is machine-specific; this installation
# may use the unpatched v6.1 image and does not include the S31 sniff-subrate fix)
source "$HOME/.espressif/tools/activate_idf_v6.1.sh"

# Select the ESP32-S31 preview target
idf.py --preview set-target esp32s31

# Build firmware
idf.py build
```

For the tested SDK image with the tested fix, use the same immutable image as CI
instead of a local v6.1 installation:

```bash
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/project" -w /project \
  espressif/idf@sha256:8ac794c57fd4cac246cb8d2ada4002fa26337ac7df683047b5b83743dbedb6b7 \
  bash -lc 'idf.py --preview set-target esp32s31 && idf.py build'
```

That image reports ESP-IDF commit `9a97f6c54ec638111ce55cd36581b3c192f15207`
and S31 Bluetooth binary commit `3d8cee57c4ccff603d4f52fe16be1f04ece78ede`;
the latter includes the upstream sniff-subrate fix. The stable `espressif/idf:v6.1`
tag does not include this fix. A delayed BOOT guard alone does not fix
phone-originated transitions: cooldown handling and the external Bluetooth
controller fix are also required.

For the optional XIAO nRF52840 firmware, use nRF Connect SDK v3.4.1 and the same board target as CI:

```bash
west build -b xiao_ble/nrf52840 ./nrf_mesh -d ./build-nrf
```

### Recovery and Flashing

```bash
# Find your serial port
ls /dev/ttyACM* /dev/ttyUSB*

# Native USB JTAG/OpenOCD with board/esp32s31-builtin.cfg is the validated
# recovery path on the current setup. Serial flashing has proved unreliable here.
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
I (xxx) omi: IDF version: v6.1
I (xxx) omi: Free heap: <bytes> bytes
I (xxx) omi: ========================================
I (xxx) omi: nRF52840 not detected - using ESP-NOW transport
I (xxx) omi: Mesh networking ready (desired state: disabled)
I (xxx) omi: System running!
```


### Troubleshooting

| Issue | Solution |
|-------|----------|
| Device not found | Use a data cable (not charge-only), try different USB port |
| Garbage characters in monitor | Wrong baud rate, use 115200 |
