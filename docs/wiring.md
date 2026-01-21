# Wiring Guide

## Microphone Input

### Current Implementation: Analog Microphone via TRRS (ADC)

This configuration uses an **analog electret microphone** (like a standard headset mic) connected through the ESP32-S3's ADC:

```
ESP32 3.3V ---- 2.2kΩ ---- TRRS Sleeve (MIC) ----|| 100nF ----> ESP32 ADC1 (GPIO1)

ESP32 ADC1 (GPIO1) ---- 100kΩ ---- GND

TRRS Ring2 (GND) ------------> ESP32 GND
```

**Component Values:**
- 2.2kΩ: Bias resistor for electret microphone
- 100nF: DC blocking capacitor
- 100kΩ: Pull-down resistor to set ADC mid-point

**ADC Configuration:**
- Channel: ADC1_CHANNEL_0 (GPIO1)
- Sample rate: 16 kHz
- Bit width: 12-bit (ESP32-S3 native)
- Attenuation: 11dB (full 0-3.3V range)

### Alternative: I2S Digital Microphone (INMP441)

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


## Speaker

### MAX98357A I2S Amplifier (Speaker Output)

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
