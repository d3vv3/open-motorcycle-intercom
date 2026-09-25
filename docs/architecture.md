# Architecture Overview

OMI currently targets the ESP32-S31 Function CoreBoard-1. It has 16 MB flash,
16 MB PSRAM, onboard ES8311 audio capture/codec, and NS4150B/J9 speaker output.

## Component Map

| Area | Current owner |
|---|---|
| Board | S31 pins, ES8311/NS4150B, GPIO47 WS mirror |
| Audio core | 16 kHz mono PCM, Opus, VOX/DTX, four-source decode/mix |
| Audio route | A2DP resample/downmix, HFP 8/16 kHz route, call priority |
| Voice cleanup | Mic-only noise suppression; ESP-SR AEC disabled (122 ms per 20 ms frame) |
| Phone audio | Classic Bluetooth A2DP sink, AVRCP, HFP HF client |
| Mesh | ESP-NOW or nRF ESB selected at startup |
| Inter-MCU | `uart_bridge` legacy API name; SPI nRF master/S31 slave |
| UI | BOOT release gestures and distinct notification beeps |

The runtime requires PSRAM for audio routes, four Opus decoders, and AEC state.
Audio initialization fails if the expected 16 MB PSRAM is missing. Task stacks,
DMA buffers, and controller allocations are internal implementation details.

## Bluetooth and Coexistence

Classic Bluetooth is implemented locally: A2DP sink with internal SBC PCM,
AVRCP, and HFP Hands-Free client with internal CVSD/mSBC over HCI. HSP is not
implemented. Pairing is closed by default; the bonded phone is retained, and a
replacement-phone pairing window explicitly forgets existing bonds before
opening discoverability.

ESP-NOW and Classic Bluetooth are both enabled with software coexistence. This
does not make ESP-NOW impossible, but product-quality simultaneous traffic and
SCO latency are not yet validated. When the nRF bridge is selected, Wi-Fi is
deinitialized and nRF ESB is preferred for phone-call reliability. Calls
suspend or preempt A2DP locally.

## Mesh and Audio Ownership

Without the bridge, the S31 owns ESP-NOW, TDMA, audio, and application policy.
With the bridge, the nRF52840 owns ESB membership and TDMA while the S31 keeps
all audio and phone processing. The nRF is SPI master and the S31 is slave.
See [SPI flow control and ownership](protocol.md#spi-flow-control) and [TDMA scheduling](tdma.md).

The mesh format remains 20 ms, 16 kHz mono Opus. Four remote decoder slots are
independent of the maximum two relay grants. Audio route mixing gives calls
exclusive priority; outside calls, mesh, music, and notifications may mix.

## Button Policy

On release, 50 ms to less than 2 seconds is a contextual short press: it
answers an incoming call, ends an ongoing call, or toggles A2DP play/pause when
both media profiles are connected. It emits no notification tone. From 2 to
less than 6 seconds, the button toggles mesh and emits a distinct mesh beep. A
release at 6 seconds or longer opens a 120-second Bluetooth pairing window and
emits a distinct pairing beep. Short-press behavior has not been hardware-
tested. Do not hold BOOT while resetting or applying power, because that enters
download mode.

## Validation Caveats

A2DP pairing/playback hardware validation passed, including a short music check
and the telemetry playout fix. After scalar far-reference subtraction was
removed, a normal-volume call was reported to sound good with no apparent
microphone leakage. This does not establish production acoustic AEC: ESP-SR AEC
is disabled and mic-only noise suppression remains active. Longer SCO stress
testing remains outstanding. The physical nRF WS connection has not yet been
validated with the S31.
