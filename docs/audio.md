# Audio Pipeline & Voice Processing

This document defines the **audio architecture** of the Open Motorcycle Intercom (OMI).

Audio is the product. Everything else exists to serve it. The system is optimized for **clear speech, low latency, and robustness under packet loss**, not hi‑fi quality.

[TOC]

---

## 1. Audio Design Goals

- End‑to‑end latency < 80 ms
- Intelligible speech at highway speeds
- Graceful degradation under packet loss
- Minimal CPU and power usage
- Simple, deterministic processing pipeline

Music playback and stereo quality are explicitly out of scope.

---

## 2. Audio Signal Chain (Local)

```
[ Microphone ]
      |
[ Preamp + HPF ]
      |
[ ADC @ 16 kHz ]
      |
[ VOX / PTT ]
      |
[ Opus Encoder ]
      |
[ TDMA Slot TX ]
```

The reverse chain applies for received audio.

---

## 3. Sampling Parameters

| Parameter | Value | Rationale |
|---------|------|-----------|
| Sample rate | 16 kHz | Speech‑optimized, low CPU |
| Channels | Mono | No spatial benefit in helmets |
| Bit depth | 16‑bit | Internal processing |

16 kHz is the lowest rate that preserves consonant clarity.

---

## 4. Microphone Considerations

Recommended:
- Electret or MEMS mic with good wind tolerance
- Directional or noise‑canceling if possible

Hardware assumptions:
- Wind noise is unavoidable
- DSP must tolerate broadband noise

A simple **high‑pass filter (~120 Hz)** removes engine rumble.

---

## 5. VOX & Push‑To‑Talk (PTT)

### VOX Behavior

- Default activation method
- Energy‑based threshold
- Hangover timer (200–500 ms)

Rules:
- VOX gates Opus encoding
- No speech → no encoding → no TX

### PTT Behavior

- Overrides VOX
- Forces continuous TX while pressed

Both methods feed the same pipeline.

---

## 6. Opus Codec Configuration

Opus is mandatory. No fallback codecs are planned.

### Encoder Settings

| Setting | Value |
|------|------|
| Mode | VoIP |
| Sample rate | 16 kHz |
| Channels | 1 |
| Frame size | 20 ms |
| Bitrate | 8–16 kbps |
| VBR | Enabled |
| FEC | Enabled |
| Complexity | Medium |

Lower bitrates reduce range reliability; higher bitrates waste power.

---

## 7. Opus Framing & Packetization

- Exactly **one Opus frame per TDMA frame**
- No fragmentation
- No aggregation

This enforces deterministic timing and simplifies buffering.

---

## 8. Jitter Buffer Design

### Purpose

Absorbs:
- Slot timing jitter
- Multi‑hop delay variation

### Configuration

- Target depth: 1–2 frames
- Max depth: 3 frames

### Behavior

- Late packets are dropped
- Missing packets invoke Opus PLC

Buffer growth is tightly bounded to protect latency.

---

## 9. Packet Loss Concealment (PLC)

Opus PLC is relied upon heavily.

Expected behavior:
- Single packet loss: barely audible
- Burst loss: brief muffling, not silence

No retransmissions are attempted.

---

## 10. Audio Mixing Model

### Mesh Audio

- One active speaker at a time (typical usage)
- Multiple streams possible but discouraged

Mixing strategy:
- Simple summation
- Soft limiter to prevent clipping

### Bluetooth Audio Integration

Bluetooth audio appears as an additional input stream.

Rules:
- Bluetooth stream is mixed at lower priority
- Mesh audio always takes precedence

---

## 11. Priority & Ducking

When mesh audio is active:
- Bluetooth audio is attenuated or muted

When Bluetooth call is active:
- Mesh audio may be mixed or paused (configurable)

This avoids cognitive overload.

---

## 12. Latency Budget (Audio Path)

| Stage | Typical Latency |
|-----|----------------|
| ADC + prefilter | 2–3 ms |
| VOX decision | <1 ms |
| Opus encode | 5–10 ms |
| TDMA wait | 0–20 ms |
| Opus decode | 5–8 ms |
| DAC + output | 2–3 ms |
| **Total** | **<80 ms** |

Worst‑case latency is bounded by design.

---

## 13. CPU Load Estimates

### ESP32‑S3

- Opus encode: ~10–15% CPU
- Opus decode: ~8–12% CPU
- Mixing + VOX: <5%

### nRF54

- Significantly lower relative load

Audio processing fits comfortably within budget.

---

## 14. Failure Modes & Degradation

| Condition | Result |
|--------|-------|
| High packet loss | PLC smoothing |
| CPU overload | Audio drop (preferred) |
| Buffer overrun | Packet drop |

Distortion is avoided even if silence occurs.

---

## 15. Audio Debugging Hooks

Recommended metrics:
- Encode time per frame
- Jitter buffer depth
- Packet loss rate
- PLC activation count

These are essential for field tuning.

---

## 16. Design Rationale Summary

- 16 kHz mono is sufficient and efficient
- One frame per slot simplifies everything
- PLC beats retransmission
- Mixing is intentionally simple

Clarity beats cleverness.

---

## 17. Status

This document defines the **authoritative audio behavior** for OMI.

Any changes must preserve latency and power guarantees.

