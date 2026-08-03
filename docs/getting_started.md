# Getting Started

This guide covers setting up a development environment for the Open Motorcycle Intercom (OMI) project.

[TOC]

---

## Prerequisites

### Hardware

This will get you started, together with **cables** and **soldering tools**:

| Item | Notes |
|------|-------|
| ESP32-S3-DevKitC-1-N8 or compatible | 8 MB flash; PSRAM is not required or enabled |
| PCM5102A breakout | Recommended I2S audio output for a prototype |
| MAX9814 breakout | Recommended analog microphone input for a prototype |
| CTIA TRRS 3.5mm jack | For headset audio and microphone connections |
| A TRRS/TRS microphone headset | For testing audio |
| Powered headphones or an amplified speaker | A passive speaker needs a separate amplifier |
| USB-C cable | For flashing and debugging |
| A power source | 5V USB power supply or battery pack |

Check the [Wiring Guide](wiring.md) for prototype wiring and the differences in
the Rev2 PCB schematic.

### Software

| Tool | Version | Notes |
|------|---------|-------|
| ESP-IDF | v5.5 (v5.5.2 recommended) | [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/) |
| nRF Connect SDK | v2.7.0 | Required to build the optional XIAO nRF52840 firmware |
| Git | Any recent | Source control |

---

## Build

```bash
# Activate ESP-IDF environment. Adjust the path for your installation.
source ~/esp/esp-idf/export.sh

# Clone the repository
git clone https://github.com/d3vv3/open-motorcycle-intercom.git
cd open-motorcycle-intercom

# Set target to ESP32-S3
idf.py set-target esp32s3

# Build
idf.py build
```

First build takes several minutes. Subsequent builds are faster.

### Build the XIAO nRF52840 Firmware

Use an nRF Connect SDK v2.7.0 workspace, then build from that workspace with the
exact board target used by CI:

```bash
west build -b xiao_ble/nrf52840 /path/to/open-motorcycle-intercom/nrf_mesh \
  -d /path/to/open-motorcycle-intercom/build-nrf
```

The output includes `build-nrf/zephyr/zephyr.hex` and
`build-nrf/zephyr/zephyr.uf2`.

---

## Flash and Monitor

```bash
# Flash firmware
idf.py -p /dev/ttyACM0 flash

# Monitor serial output
idf.py -p /dev/ttyACM0 monitor

# Or combined
idf.py -p /dev/ttyACM0 flash monitor
```

Identify the actual port on your system and replace `/dev/ttyACM0` as needed
(`/dev/ttyUSB0`, `/dev/cu.usbmodem*`, and similar names are also common).

**Exit monitor:** `Ctrl+]`

### Boot Checks

Boot output changes as firmware evolves, so do not depend on exact timestamps or
heap values. A healthy boot reports the OMI banner, ESP-IDF version, NVS, power,
button, audio, and transport initialization. During `Detecting mesh transport...`,
the ESP32 probes the nRF52840 over the SPI bridge. A successful SPI probe selects
the nRF/ESB transport and disables ESP Wi-Fi/ESP-NOW; otherwise the current
firmware reports `nRF52840 not detected - using ESP-NOW transport` and continues
with the fallback transport.

Fresh NVS defaults mesh intent to disabled. Hold the ESP Boot button for two
seconds to enable mesh operation. The firmware stores this setting for later
boots.

---

## Project Structure

```
open-motorcycle-intercom/
├── CMakeLists.txt          # Root project file
├── sdkconfig.defaults      # Default SDK configuration
├── partitions.csv          # Flash partition table
│
├── main/
│   ├── CMakeLists.txt
│   └── main.c              # Application entry point
│
├-- components/
|   ├-- audio/              # Audio capture, Opus (DTX), VOX, jitter buffer, playback
|   ├-- mesh/               # ESP-NOW transport, TDMA, relay/routing
|   ├-- uart_bridge/        # SPI bridge to the nRF52840 radio MCU
|   ├-- power/              # Power state machine
|   ├-- button/             # Boot-button UI handler
|   └-- hwtest/             # Hardware bring-up / test utilities
|
├-- nrf_mesh/               # nRF52840 (Zephyr) radio firmware: ESB, TDMA, mesh
├-- shared/                 # Shared wire formats and transport-neutral mesh code
├-- scripts/                # Acceptance and development utilities
├-- tests/
|   ├-- c/                  # Host-side C unit and compile tests
|   └-- python/             # Python acceptance and telemetry tests
|
└-- docs/                   # Documentation
```

---

## Common Commands

| Command | Purpose |
|---------|---------|
| `idf.py build` | Build the project |
| `idf.py flash` | Flash to device |
| `idf.py monitor` | Open serial monitor |
| `idf.py flash monitor` | Flash and monitor |
| `idf.py menuconfig` | Configure SDK options |
| `idf.py fullclean` | Clean all build artifacts |
| `idf.py size-components` | Show size by component |

---

## Troubleshooting

### Permission Denied on Serial Port (Linux)

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Device Not Found

- Use a data cable (not charge-only)
- Try a different USB port
- Check `dmesg | tail -20` for connection events

---

## LSP Support

After building, symlink `compile_commands.json` for clangd:

```bash
ln -sf build/compile_commands.json .
```

---

## Next Steps

See the [Development Plan](dev_plan.md) for the roadmap and exit criteria.
