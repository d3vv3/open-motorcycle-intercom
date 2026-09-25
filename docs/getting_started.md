# Getting Started

The tested setup uses two ESP32-S31 Function CoreBoard-1 / XIAO nRF52840 Sense pairs.
The S31 handles audio and Bluetooth; the nRF provides the ESB mesh radio.

## Hardware and Tools

For each pair, use:

- An ESP32-S31 Function CoreBoard-1 with 16 MB flash and 16 MB PSRAM.
- A XIAO nRF52840 Sense and a passive 4 ohm, 3 W speaker.
- Data-capable USB cables and the connections in [wiring.md](wiring.md).

The CoreBoard already contains the microphone, codec, and speaker amplifier.
Do not add the external audio modules from the older ESP32-S3 prototype.

The commands below assume Linux, Docker access, and a cloned repository.
Install `uv` for the benchmark and Python tests, and NCS 3.4.1 with its matching toolchain for the nRF build.

## Build the S31 Firmware

Run from the repository root. Use the pinned ESP-IDF preview image that supports S31:

```bash
export IDF_IMAGE=espressif/idf@sha256:8ac794c57fd4cac246cb8d2ada4002fa26337ac7df683047b5b83743dbedb6b7

docker run --rm --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e OMI_ESP_LC3_BENCH=1 \
  -v "$PWD:/project" -w /project "$IDF_IMAGE" \
  bash -lc '. "$IDF_PATH/export.sh" >/dev/null &&
    idf.py --preview -B build-s31 \
      -D SDKCONFIG=/project/build-s31/sdkconfig \
      -D IDF_TARGET=esp32s31 \
      -D ESP_LC3_BENCH=ON \
      -D S31_LC3_WIRE=ON \
      -D S31_LC3_SELFTEST=ON \
      -D S31_LC3_SERIALIZE=ON \
      -D S31_LC3_INTERNAL_BUFFERS=ON \
      -D S31_LC3_SINGLE_OWNER=ON \
      -D S31_LC3_SPLIT_CORES=OFF \
      -D S31_LC3_SKIP_RX=OFF \
      -D S31_MESH_COEX_PREFER_WIFI=OFF \
      -D S31_MESH_PAIR_UNICAST=OFF \
      -D S31_TASK_CPU_PROFILE=OFF build'
```

Use a fresh `build-s31` directory for this configuration.
Its separate `sdkconfig` uses the project's defaults instead of an existing root configuration.
The environment variable and CMake options are both required for the LC3 dependency and codec path.
Plain `idf.py build` does not enable this configuration by default.

The pinned image contains ESP-IDF commit `9a97f6c54ec638111ce55cd36581b3c192f15207`.
Do not substitute a moving SDK tag when comparing results.

## Build the nRF Firmware

Open an NCS 3.4.1 toolchain terminal. From its SDK workspace, set the absolute path to this repository:

```bash
export OMI_APP_DIR=/absolute/path/to/omi

west build --sysbuild -b xiao_ble/nrf52840 \
  "$OMI_APP_DIR/nrf_mesh" -d "$OMI_APP_DIR/build-nrf341" \
  -- -DCONFIG_BUILD_OUTPUT_UF2=y
```

The tested target is also used for our XIAO Sense boards.
The application image is `build-nrf341/nrf_mesh/zephyr/zephyr.uf2`.
Both ends need matching LC3 protocol-v3 firmware; an older Opus bridge is not compatible.

## Flash the Boards

### XIAO nRF52840

Power off the pair and disconnect the shared `3V3` supply wire before connecting XIAO USB.
Follow the [USB power and flashing procedure](wiring.md#usb-flashing-or-nrf-serial-capture).

Double-press XIAO RESET and copy the application UF2 to its bootloader drive.
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
2. Confirm nRF ESB selection, bridge protocol 3, LC3, and 20 ms audio in the S31 logs.
3. On each fresh S31, hold BOOT for 2 to under 6 seconds, then release it to enable mesh.
4. Confirm that the two nodes join the same mesh and report a peer.
5. Speak near each microphone and listen at the opposite speaker.
6. Use the [button gestures](buttons.md) to pair a phone and test music alongside mesh voice.

Mesh starts disabled on fresh boards. The setting persists across restarts; the same gesture toggles it on existing installations.

With VOX enabled, silence stops LC3 encoding and audio transmission after the hangover period.
Capture and control traffic continue; quiet audio counters alone are not a fault.
There is no pre-roll buffer, so the start of quiet speech can be clipped.

An absent or incompatible nRF bridge causes ESP-NOW fallback at startup.
Check the selected transport before comparing results. With nRF selected, S31 Wi-Fi is off and Bluetooth remains enabled.

Two-pair music and voice operation has been tested. Larger groups and sustained calls still need validation.
For normal one-cable operation, follow the power transition in [wiring.md](wiring.md) before reconnecting the shared supply.

## Capture Logs and Run Tests

From the repository root, select the actual serial ports:

```bash
uv run benchmark.py --duration 120 --baud 115200 \
  --ports /dev/ttyACM0 /dev/ttyACM1 --out-dir logs/benchmark

uv run --frozen pytest
```

Prefer stable `/dev/serial/by-id/` paths. Capture both S31s together; nRF USB consoles are optional additional inputs.
Disconnect the shared supply before adding nRF USB power. The current nRF console uses CDC interface `if02`.

Each run produces `raw/`, `report.txt`, and `summary.json`.
See [pipeline telemetry](pipeline_telemetry.md) for counter interpretation and [CPU profiling](s31-cpu-profiling.md) for optional diagnostics.
Standalone C checks and firmware build jobs are listed in [the CI workflow](../.github/workflows/build.yml).
