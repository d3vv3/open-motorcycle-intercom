# Getting Started

The tested setup uses two ESP32-S31 Function CoreBoard-1 / XIAO nRF52840 Sense pairs.
The S31 handles audio and Bluetooth; the nRF provides the ESB mesh radio.

## Hardware and Tools

For each pair, use:

- An ESP32-S31 Function CoreBoard-1 with 16 MB flash and 16 MB PSRAM.
- A XIAO nRF52840 Sense and a passive 4 ohm, 3 W speaker.
- Three normally-open momentary buttons, data-capable USB cables, and the connections in [wiring.md](wiring.md).

The CoreBoard already contains the microphone, codec, and speaker amplifier.
Do not add the external audio modules from the older ESP32-S3 prototype.

The commands below assume Linux, Docker access, and a cloned repository.
Install `uv` for the benchmark and Python tests, and `rustup` for the nRF build.
The nRF crate's `rust-toolchain.toml` pins Rust 1.95.0 with the `thumbv7em-none-eabihf` target; rustup installs it on first use.

## Build the S31 Firmware

Run from the repository root. Use the pinned ESP-IDF preview image that supports S31:

```bash
export IDF_IMAGE=espressif/idf@sha256:8ac794c57fd4cac246cb8d2ada4002fa26337ac7df683047b5b83743dbedb6b7

docker run --rm --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -v "$PWD:/project" -w /project "$IDF_IMAGE" \
  bash -lc '. "$IDF_PATH/export.sh" >/dev/null &&
    idf.py --preview -B build-s31 \
      -D SDKCONFIG=/project/build-s31/sdkconfig \
      -D IDF_TARGET=esp32s31 build'
```

Use a fresh `build-s31` directory for this configuration.
Its separate `sdkconfig` uses the project's defaults instead of an existing root configuration.
LC3 is always built in. The codec options `S31_LC3_SERIALIZE`, `S31_LC3_INTERNAL_BUFFERS`, and
`S31_LC3_SINGLE_OWNER` default ON and `S31_TASK_CPU_PROFILE` defaults OFF, so a plain build is the release image.
`tools/release/build.py` passes the same values explicitly.

The pinned image contains ESP-IDF commit `9a97f6c54ec638111ce55cd36581b3c192f15207`.
Do not substitute a moving SDK tag when comparing results.

## Build the nRF Firmware

The nRF runs the Rust Embassy firmware in `rust/nrf-embassy-firmware`. Run from the repository root:

```bash
(cd rust/nrf-embassy-firmware &&
  cargo build --locked --release --target thumbv7em-none-eabihf)

python3 tools/migration/nrf_proof.py \
  rust/nrf-embassy-firmware/target/thumbv7em-none-eabihf/release/nrf-embassy-firmware \
  rust/nrf-embassy-firmware/target/thumbv7em-none-eabihf/release/nrf-embassy-firmware.uf2
```

Building inside the crate directory picks up its pinned toolchain.
Use the default Cargo features; they are the configuration `tools/release/build.py` builds and checks.
The release check rejects the experimental `early-timing-service` and `same-slot-audio-refit` features.
`nrf_proof.py` checks the ELF layout and writes the UF2; it does not prove the firmware boots.
The application image is `nrf-embassy-firmware.uf2`.
Both ends need matching LC3 firmware with bridge protocol 6; update the S31 and nRF together.

## Flash the Boards

### XIAO nRF52840

Power off the pair and disconnect the shared `3V3` supply wire before connecting XIAO USB.
Follow the [USB power and flashing procedure](wiring.md#usb-flashing-or-nrf-serial-capture).

Double-press XIAO RESET to mount its `XIAO-SENSE` UF2 drive, then copy `nrf-embassy-firmware.uf2` onto it.
The board restarts after the copy. Use the application UF2, not a whole-chip erase or bootloader replacement.
Repeat for the other nRF.

### ESP32-S31

Native USB JTAG/OpenOCD is the tested flashing path; serial flashing was unreliable on this setup.
Connect the S31's native USB port and select its USB serial identity explicitly.
Run from the repository root, using `IDF_IMAGE` from the build step:

```bash
export S31_SERIAL=YOUR_S31_USB_SERIAL

docker run --rm --user 0:0 \
  --device-cgroup-rule='c 189:* rwm' \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$PWD:/project:ro" -w /project \
  -e S31_SERIAL "$IDF_IMAGE" \
  bash -lc '. "$IDF_PATH/export.sh" >/dev/null &&
    openocd -f board/esp32s31-builtin.cfg \
      -c "adapter serial $S31_SERIAL" \
      -c "program_esp_bins /project/build-s31 flasher_args.json verify reset exit"'
```

This uses the generated image list and offsets for the bootloader, partition table, and application.
An app-only update is not sufficient for initial board setup. Check the programming and verification results before continuing.
Repeat with the other S31's serial identity.

## First Checks

1. Let each nRF boot, then reset its S31 so the SPI probe can find it.
2. Confirm nRF ESB selection, bridge protocol 6, LC3, and 20 ms audio in the S31 logs.
3. On each fresh S31, hold the external center/play button for 2 to under 6 seconds, then release it to enable mesh.
4. Confirm that the two nodes join the same mesh and report a peer.
5. Speak near each microphone and listen at the opposite speaker.
6. Use the [external button controls](buttons.md) to pair a phone, select a talk channel, and test music alongside mesh voice.

With VOX enabled, silence stops LC3 encoding and audio transmission after the hangover period.
Capture and control traffic continue; quiet audio counters alone are not a fault.
There is no pre-roll buffer, so the start of quiet speech can be clipped.

An absent or incompatible nRF bridge causes ESP-NOW fallback at startup.
Check the selected transport before comparing results. With nRF selected, S31 Wi-Fi is off and Bluetooth remains enabled.
With ESP-NOW, mesh and Bluetooth are [mutually exclusive](buttons.md#esp-now-mesh-and-bluetooth), so music and mesh voice do not run together.

For normal one-cable operation, follow the power transition in [wiring.md](wiring.md) before reconnecting the shared supply.

## Capture Logs and Run Tests

From the repository root, select the actual serial ports:

```bash
uv run benchmark.py --duration 120 --baud 115200 \
  --ports /dev/ttyACM0 /dev/ttyACM1 --out-dir logs/benchmark

uv run --frozen pytest
```

Prefer stable `/dev/serial/by-id/` paths. Capture both S31s together; nRF USB consoles are optional additional inputs.
Disconnect the shared supply before adding nRF USB power. The nRF firmware exposes one USB CDC ACM console (`if00`).

Each run produces `raw/`, `report.txt`, and `summary.json`.
See [pipeline telemetry](pipeline_telemetry.md) for counter interpretation. Task CPU profiling is an optional `-D S31_TASK_CPU_PROFILE=ON` build.
Standalone C checks and the S31 build are listed in [the CI workflow](../.github/workflows/build.yml);
the Rust nRF jobs are in [the nRF workflow](../.github/workflows/rust-nrf.yml).
