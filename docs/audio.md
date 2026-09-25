# Audio Pipeline

This page describes the tested ESP32-S31 LC3 mesh configuration. The S31 handles all audio processing;
the nRF52840 transports encoded audio over ESB.

## Formats and Signal Path

| Domain | Format | Samples per 20 ms |
|---|---|---:|
| Hardware capture and playback | 48 kHz, mono, signed 16-bit PCM | 960 |
| Mesh voice | 16 kHz, mono, signed 16-bit PCM | 320 |
| Encoded mesh audio | Two 10 ms LC3 frames, 24 bytes each | 48 bytes total |

The LC3 payload rate is 19.2 kbit/s, excluding headers and redundancy.

```text
Microphone -> 48-to-16 kHz conversion -> voice cleanup -> VOX -> LC3 -> mesh
Mesh -> packet recovery -> LC3 decode -> voice mix -> 16-to-48 kHz conversion -> speaker
Bluetooth music -> stereo downmix and rate conversion -> speaker mix
```

The onboard ES8311 codec and NS4150B amplifier handle physical audio I/O.
See [wiring](wiring.md) for the speaker connection and pin details.

## Codec Ownership and Memory

The tested build enables `ESP_LC3_BENCH`, `S31_LC3_WIRE`, `S31_LC3_SERIALIZE`, and `S31_LC3_SINGLE_OWNER`.
These CMake options default to OFF; the benchmark and serialization options are prerequisites for this path.
Capture, playout, and the dedicated LC3 owner task run on core 1.
All vendor LC3 operations run through the owner task, including resets. Preserve this arrangement when changing task placement.

Mesh mode uses LC3; local codec loopback still uses Opus.
The nRF does not encode, decode, or mix audio.

Audio initialization requires the CoreBoard's expected 16 MB PSRAM.
Route buffers and Speex resampler state and scratch buffers use PSRAM in the S31 configuration.
Speex uses fixed-point quality-3 resampling, with a full sinc table to reduce conversion work.

## VOX and Silence

VOX measures microphone RMS once per 20 ms frame. Current defaults are:

- Activation threshold: `0.03` of full scale; deactivation threshold: `0.010`.
- Minimum active time: 500 ms; hangover: 500 ms.
- Forced continuous transmission: disabled.

During VOX silence, the LC3 mesh path skips encoding and local audio transmission.
Microphone capture, voice detection, Bluetooth microphone routing, and mesh control traffic continue.
Local silence does not stop reception of other speakers.

There is no pre-roll buffer. Quiet speech before activation can lose its opening milliseconds;
that behavior is currently accepted. RMS detection can also respond to noise or music.

See [protocol.md](protocol.md#lc3-audio-bundles) for quiet-slot sequencing and predecessor handling.

## Receive and Playback

Up to three remote sources have independent packet, decoder, and resampler state.
Predecessor frames can recover missing audio; remaining gaps use concealment before an empty stream becomes idle.
These decoder slots are separate from the mesh's two active-speaker relay grants.

Packet and PCM buffers provide startup prefill and absorb timing variation.
Their presence means codec frame duration alone does not describe end-to-end latency.

Outside phone calls, remote voice, Bluetooth music, and notification tones can mix locally.
An active HFP call takes playback priority and suppresses those other sources.
Mesh microphone transmission can still continue during a call, subject to VOX.

## Bluetooth and Voice Cleanup

- **A2DP:** The S31 decodes SBC music, downmixes stereo, and resamples to the 48 kHz speaker domain.
  Hardware playback tests used 44.1 kHz stereo input.
- **AVRCP:** Provides media controls and metadata.
- **HFP:** Hands-Free call audio uses CVSD or mSBC, with 8/16 kHz microphone and playback routes.

Microphone processing includes a high-pass filter and mic-only noise cleanup.
ESP-SR acoustic echo cancellation is disabled. Noise cleanup is not acoustic echo cancellation.
The disabled AEC path also skips unnecessary playback-reference conversion.

See [button gestures](buttons.md) for media, call, mesh, and pairing controls.

## Validation and Diagnostics

Bluetooth music and two-way LC3 mesh voice have worked together on two nRF-equipped pairs.
Quiet-state tests confirmed continued capture with zero LC3 encoding or audio handoff.
Larger groups, sustained calls, and broader acoustic conditions still need testing.

Codec initialization includes a reset sequence that restored speaker output after MCU-only resets.
`AUDIO_OUT` diagnostics report submitted PCM levels, write results, and codec/amplifier register state.
Successful writes and nonzero samples do not prove audible output.

See [pipeline telemetry](pipeline_telemetry.md) for delivery and dropout counters,
or [architecture](architecture.md) for the system overview.
