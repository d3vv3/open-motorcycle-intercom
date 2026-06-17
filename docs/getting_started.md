# Getting Started

This guide covers setting up a development environment for the Open Motorcycle Intercom (OMI) project.

[TOC]

---

## Prerequisites

### Hardware

| Item | Notes |
|------|-------|
| ESP32-S3-DevKitC-1-N8R8 | Must have PSRAM (N8R8 variant) |
| USB-C cable | For flashing and debugging |

See [Hardware BOM](../README.md#hardware-bom-development) for full component list.

### Software

| Tool | Version | Notes |
|------|---------|-------|
| ESP-IDF | v5.1+ (v5.2.2 recommended) | [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/get-started/) |
| Git | Any recent | Source control |

---

## Build

```bash
# Activate ESP-IDF environment
get_idf

# Clone the repository
git clone https://github.com/your-org/omi.git
cd omi

# Set target to ESP32-S3
idf.py set-target esp32s3

# Build
idf.py build
```

First build takes several minutes. Subsequent builds are faster.

---

## Flash and Monitor

```bash
# Flash firmware
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or combined
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port (`/dev/ttyACM0`, `/dev/cu.usbmodem*`, etc.).

**Exit monitor:** `Ctrl+]`

### Expected Boot Log

```
I (123) omi: ========================================
I (123) omi: OMI - Open Motorcycle Intercom
I (123) omi: IDF version: v5.5.2
I (123) omi: Free heap: 275432 bytes
I (123) omi: ========================================
I (130) omi: [7 ms] NVS initialized
I (132) omi: [9 ms] Power management initialized
I (134) omi: [11 ms] Button handler initialized
I (140) omi: nRF52840 not detected - using ESP-NOW transport
I (160) omi: [37 ms] Audio initialized
I (185) omi: [62 ms] Mesh initialized
I (190) omi: [62 ms] System ready
```

Timestamps and heap are illustrative. When an nRF52840 is wired up over the SPI
bridge you'll instead see `nRF52840 detected on UART - using ESB transport`, and
the ESP-NOW mesh init is skipped in favor of the nRF/ESB transport.

---

## Project Structure

```
omi/
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

### PSRAM Not Detected

Verify boot log shows:
```
I (xxx) esp_psram: Found 8MB PSRAM
```

If missing, ensure you have the N8R8 variant board.

---

## LSP Support

After building, symlink `compile_commands.json` for clangd:

```bash
ln -sf build/compile_commands.json .
```

---

## Next Steps

See the [Development Plan](dev_plan.md) for the roadmap and exit criteria.
