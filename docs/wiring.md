# Wiring Guide

This guide covers a development-board prototype built from breakout modules and
also records how those functions appear on the current Rev2 PCB schematic. The
Rev2 PCB integrates the PCM5102A, MAX9814, headphone amplifier, nRF52840, and CTIA
jack; do not wire breakout modules on top of those integrated circuits. Rev2 is a
development design, not a claim of production readiness.

## CTIA Headset Jack

A CTIA TRRS headset uses this contact order:

| Contact | Signal |
|---------|--------|
| Tip | Left headphone audio |
| Ring 1 | Right headphone audio |
| Ring 2 | Ground |
| Sleeve | Microphone |

The Rev2 `J2` jack follows this CTIA mapping. OMTP headsets swap microphone and
ground and require an adapter.

## Prototype Microphone Input

### (Recommended) Analog Microphone via TRRS (ADC)

Use a MAX9814 breakout to amplify an analog electret microphone before the
ESP32-S3 ADC. Common MAX9814 breakouts include an onboard electret microphone:
`OUT` is the amplified analog output, not a microphone input. To use a headset
microphone, connect the CTIA sleeve to the breakout's microphone input/bias point
only if that point is exposed and follow the breakout vendor's instructions for
disconnecting its onboard microphone. Connect CTIA ring 2 to ground. The jack tip
and ring 1 are left and right headphone outputs and do not connect to the
MAX9814.

| MAX9814 Pin | ESP32-S3                                |
| ----------- | --------------------------------------- |
| VDD         | 3.3V                                    |
| GND         | GND                                     |
| OUT         | GPIO1 (ADC)                             |
| AR          | Leave floating (default attack/release) |
| GAIN        | VDD (40 dB), GND (50 dB), or floating (60 dB) |

**ADC Configuration:**
- Channel: ADC1_CHANNEL_0 (GPIO1)
- Sample rate: 16 kHz
- Bit width: 12-bit (ESP32-S3 native)
- Attenuation: **12 dB**

On Rev2, the MAX9814 circuit and headset microphone bias/coupling are integrated;
the amplified `MIC_ADC` signal reaches ESP32-S3 GPIO1.

### I2S Digital Microphone (INMP441)

The original design planned for an INMP441 I2S digital microphone. If switching to I2S digital:

| INMP441 Pin | ESP32-S3 Pin | Function |
|-------------|--------------|----------|
| **SCK**     | **GPIO 4**   | BCLK (Bit Clock) |
| **WS**      | **GPIO 5**   | WS (Word Select) |
| **SD**      | **GPIO 6**   | DIN (Data In) |
| **L/R**     | **GND**      | Left channel select |
| **VDD**     | **3.3V**     | Power |
| **GND**     | **GND**      | Ground |

**Note:** Code changes required to switch from ADC to I2S RX mode.


## Prototype Audio Output

### (Recommended) PCM5102A 3.5mm Audio Jack

The PCM5102A is an I2S DAC for a line/headphone signal path. It is not a power
amplifier for a passive speaker. Use powered headphones/speakers or a suitable
headphone or speaker amplifier after the DAC. Rev2 routes the PCM5102A outputs
through a TPA6132A2 headphone amplifier to the CTIA jack.

| PCM5102A            | ESP32-S3 |
| ------------------- | -------- |
| **BCK**             | GPIO4    |
| **LCK (LRCK / WS)** | GPIO5    |
| **DIN**             | GPIO7    |
| **VIN**             | **3.3V** |
| **GND**             | GND      |


### MAX98357A I2S Amplifier (Alternative Speaker Output)

The MAX98357A is a mono I2S Class-D amplifier. It connects to the ESP32-S3 via the I2S interface.

| MAX98357A Pin | ESP32-S3 Pin | Function | Notes |
|---------------|--------------|----------|-------|
| **LRC**       | **GPIO 5**   | WS (Word Select) | Left/Right Clock |
| **BCLK**      | **GPIO 4**   | SCK (Bit Clock) | Serial Clock |
| **DIN**       | **GPIO 7**   | DOUT (Data Out) | Audio Data |
| **GAIN**      | **GND**      | Gain Setting | Sets +9dB gain (adjust as needed) |
| **SD**        | **Floating** | Shutdown / Channel | Left + Right / 2 (Mono Mix) |
| **GND**       | **GND**      | Ground | Common Ground |
| **VIN**       | **5V** or 3.3V | Power | 5V recommended for full 3W power |

**Notes:**
- If your board has a "5V" pin, use it for VIN to get louder audio.
- If using 3.3V, audio power will be limited to ~0.5W.
- The **SD** pin can be connected to VCC or GND to change channel selection, but leaving it floating selects "Left/2 + Right/2" which is perfect for mono mix.

Connect SPK+ and SPK- to your speaker. Speaker impedance should be 4-8 Ohms,
and the speaker should be 0.5W to 3W for best results.

The current firmware emits standard Philips-format I2S on the same BCLK, WS, and
DOUT pins used by both modules. A standard-I2S MAX98357A therefore needs no
firmware pin-mode change; configure only the module's gain/channel hardware as
needed.

## (Optional, Recommended) nRF52840 Mesh Radio (XIAO)

The Seeed XIAO nRF52840 handles mesh networking via ESB (Enhanced ShockBurst).
Audio packets received from the mesh are sent to ESP32-S3 via SPI for mixing.

### SPI Connection

The nRF52840 is the SPI **master** and the ESP32-S3 is the SPI **slave**.
The bridge uses SPI mode 0 at 4 MHz. The nRF polls every 2 ms with a full-duplex
256-byte transaction and drives chip select manually because hardware CS is too
short for reliable ESP32 SPI-slave detection.

| XIAO nRF52840 | ESP32-S3 | Function |
|---------------|----------|----------|
| **Pin 6** (P1.11 / MOSI) | **GPIO10** | nRF MOSI -> ESP MOSI |
| **Pin 7** (P1.12 / MISO) | **GPIO9**  | ESP MISO -> nRF MISO |
| **Pin 8** (P1.13 / SCK)  | **GPIO11** | SPI Clock |
| **Pin 9** (P1.14 / CS)   | **GPIO12** | Chip Select (active low) |
| **D10** (P1.15) | **GPIO2** | Audio-admission ACK pulse, nRF -> ESP |
| **3V3** | **3V3** | Common 3.3 V supply/reference |
| **GND** | **GND** | Common ground |

Use one regulated 3.3 V source for the shared 3V3 rail; do not tie together two
independently powered regulator outputs.

### I2S WS Sync Wire (Clock Discipline)

The ESP32's I2S Word Select (WS) output is also fed to the nRF52840. The nRF counts
WS edges and uses them to discipline its local TDMA period. This reduces relative
clock drift but does not by itself prove over-the-air synchronization accuracy.

| ESP32-S3 | XIAO nRF52840 | Function |
|----------|---------------|----------|
| **GPIO 5** (I2S WS/LCK) | **Pin D0** (P0.02) | Sync clock (16 kHz) |

> **Note:** GPIO5 is already wired to the speaker amplifier (MAX98357A/PCM5102A LCK).
> Simply add a second wire from the same GPIO5 pad to XIAO D0.
> The nRF input is high-impedance and will not affect the amplifier signal.

### Wiring Diagram

The compact diagram shows the SPI bus and WS sync. The ACK and shared supply
connections are listed in the complete table above.

```
ESP32-S3                    XIAO nRF52840
┌──────────┐                ┌──────────┐
│          │                │          │
│   GPIO10 ├────────────────┤ Pin 6    │  (MOSI)
│   GPIO9  ├────────────────┤ Pin 7    │  (MISO)
│   GPIO11 ├────────────────┤ Pin 8    │  (SCK)
│   GPIO12 ├────────────────┤ Pin 9    │  (CS)
│   GPIO5  ├────────────────┤ Pin D0   │  (I2S WS Sync)
│          │                │          │
│   GND    ├────────────────┤ GND      │
│          │                │          │
└──────────┘                └──────────┘
```

> **Note:** Both boards run at 3.3V logic, no level shifter needed.
