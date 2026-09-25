# ESP32-S31 Function CoreBoard Wiring

This guide describes the wiring tested on two prototype pairs:

- ESP32-S31-Function-CoreBoard-1
- Seeed XIAO nRF52840 Sense running the `nrf_mesh` firmware
- One passive 4 ohm speaker on the CoreBoard speaker connector

The older ESP32-S3, MAX9814, PCM5102A, and MAX98357A prototype uses different
pins. Do not use its GPIO map with this firmware.

## Onboard Audio

The CoreBoard already contains the microphone, ES8311 codec, and NS4150B speaker
amplifier. Do not connect external microphone, codec, or I2S amplifier modules.

Connect a passive 4 ohm, 3 W speaker to the CoreBoard speaker output.
The connector pitch is 1.25 mm. The amplifier output is differential, so neither
speaker terminal connects to ground.

<details>
<summary>Onboard audio GPIOs and WS clock details</summary>

The onboard audio signals are not exposed on `J2`:

| Function | ESP32-S31 GPIO |
|---|---:|
| ES8311 I2C SCL | 50 |
| ES8311 I2C SDA | 51 |
| I2S MCLK | 52 |
| I2S BCLK | 53 |
| I2S microphone data to ESP | 54 |
| I2S WS/LRCLK | 55 |
| I2S playback data to codec | 56 |
| NS4150B amplifier enable | 57 |

The firmware mirrors physical 48 kHz I2S WS to GPIO47, providing 960 rising edges
per 20 ms frame. This is separate from the 16 kHz mono LC3 audio rate.
Two 10 ms LC3 frames form each 20 ms mesh audio payload.

Connect the mirror to the nRF as shown below, not to the onboard codec WS net.
The wire provides a clock reference; its presence alone does not establish that
WS-based clock correction is enabled.

</details>

## nRF52840 Bridge

The nRF52840 is the SPI master. The ESP32-S31 is the SPI slave.
The bridge uses SPI mode 0 at 4 MHz with 256-byte full-duplex transactions.

Make these six signal connections:

| Function | XIAO nRF52840 | CoreBoard J2 | ESP32-S31 |
|---|---|---:|---:|
| ACK, nRF to ESP | D10 / P1.15 | pin 20 | GPIO42 |
| SPI MISO, ESP to nRF | D7 / P1.12 | pin 17 | GPIO43 |
| SPI MOSI, nRF to ESP | D6 / P1.11 | pin 18 | GPIO44 |
| SPI clock, nRF to ESP | D8 / P1.13 | pin 15 | GPIO45 |
| SPI chip select, nRF to ESP, active low | D9 / P1.14 | pin 16 | GPIO46 |
| I2S WS sync, ESP to nRF | D0 / P0.02 | pin 13 | GPIO47 (48 kHz mirror) |

Also connect ground:

| XIAO nRF52840 | CoreBoard J2 |
|---|---|
| GND | any `G` pin: 1, 2, 8, 33, 34, 37, or 38 |

The `D` labels identify XIAO pads; the J2 numbers identify CoreBoard connector positions.
Use the connector's pin-1 marking and the vendor header table to orient J2.

<details>
<summary>MISO direction and connection diagram</summary>

Signal direction matters. In particular, the wire named `MISO` carries data
from ESP GPIO43 to XIAO P1.12 because the nRF is the bus master.

```text
ESP32-S31 Function CoreBoard-1                 XIAO nRF52840 Sense

J2 pin 20 / GPIO42  <------------------------- D10 / P1.15   ACK
J2 pin 17 / GPIO43  --------------------------> D7 / P1.12    MISO
J2 pin 18 / GPIO44  <-------------------------- D6 / P1.11    MOSI
J2 pin 15 / GPIO45  <-------------------------- D8 / P1.13    SCLK
J2 pin 16 / GPIO46  <-------------------------- D9 / P1.14    CS
J2 pin 13 / GPIO47  --------------------------> D0 / P0.02    WS sync
J2 GND              --------------------------- GND
```

</details>

### Power

Both boards use 3.3 V logic. No level shifter is required.

#### Normal prototype operation: one USB cable per pair

The tested setup powers the XIAO from the USB-powered CoreBoard:

| CoreBoard | XIAO nRF52840 Sense |
|---|---|
| J2 pin 35 or 36, `3V3` | `3V3` pad |
| J2 `G` pin | GND |

This feeds the XIAO's 3.3 V rail directly; Seeed labels that pad `3V3_OUT`.
It describes the tested bench arrangement, not a dedicated XIAO power-input connector.

Power off both boards before connecting the supply wire. Leave XIAO USB and battery
power disconnected while this wire is fitted. Never connect USB `5V` or `VBUS` to a `3V3` pad.

The nRF runs without a USB host. Its USB connection is not required for SPI or mesh operation.

#### USB flashing or nRF serial capture

1. Power off the pair and disconnect any XIAO battery.
2. Disconnect the `3V3` wire between the boards. Keep the signal and ground wires connected.
3. Power each board from its own USB connector.
4. Double-press the XIAO's RESET button to enter its UF2 bootloader, then copy the application UF2.
5. Wait for the XIAO to restart before checking the SPI bridge.

For serial capture, use the nRF application console after it restarts, not the UF2 drive.
The current nRF firmware exposes two CDC interfaces; its console is on `if02`.

To return to one-cable operation, remove USB power from both boards before restoring the `3V3` wire.
Then connect USB only to the CoreBoard.

<details>
<summary>CoreBoard Pins to Avoid</summary>

- GPIO61 is the BOOT button and a strapping pin.
- GPIO60 drives the onboard RGB LED and is a strapping pin.
- GPIO37 is a strapping pin.
- GPIO58 and GPIO59 are UART0 through the onboard USB-to-UART bridge.
- GPIO20 through GPIO25 are the fixed SDIO slot-0 group.
- GPIO35 through GPIO40 are the fixed SDIO slot-1 group.
- GPIO33 and GPIO34 are the fixed USB Serial/JTAG data pins and are not on J2.
- GPIO26 through GPIO32 are reserved for flash and must not be used.

The bridge map uses GPIO42 through GPIO47 and avoids these conflicts.

</details>

For BOOT button gestures and notification sounds, see [Button gestures](buttons.md).

## Bring-up Order

1. With power disconnected, connect ground, ACK, SPI, WS, and the speaker as shown above.
2. Use the separate-USB arrangement for flashing; leave the shared `3V3` wire disconnected.
3. Flash matching S31 and nRF firmware with bridge protocol v3 and LC3 support.
4. Let the XIAO boot, then reset the S31 so it can probe the bridge.
5. Confirm the S31 log selects nRF ESB and reports bridge protocol 3, LC3, and 20 ms audio frames.
6. Repeat for the second pair and confirm that the mesh reports a peer.
7. Speak near each microphone in turn and check the opposite speaker.

With VOX enabled, silent microphones do not produce LC3 audio packets. Control traffic
and mesh membership continue, so quiet audio counters alone do not indicate a wiring fault.

If the S31 selects ESP-NOW, check the nRF's power, firmware compatibility, and all six signal wires.
Also check common ground. An absent or incompatible bridge causes ESP-NOW fallback at startup.
When nRF ESB is selected, ESP Wi-Fi and ESP-NOW are disabled; Bluetooth remains available.

## Source References

- [ESP32-S31 Function CoreBoard-1 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-function-coreboard-1/user_guide.html)
- [ESP32-S31 Function CoreBoard-1 schematic](https://dl.espressif.com/schematics/esp32-s31-function-coreboard-1-schematics.pdf)
- [Seeed XIAO nRF52840 Sense pin map](https://wiki.seeedstudio.com/XIAO_BLE/#xiao-nrf52840-sense-front)
- [S31 firmware pin definitions](../components/board/include/omi_board_pins.h)
- [nRF SPI pin configuration](../nrf_mesh/boards/xiao_ble.overlay)
- [nRF ACK and bridge configuration](../nrf_mesh/include/uart_bridge.h)
- [nRF WS input configuration](../nrf_mesh/src/ws_sync.c)
