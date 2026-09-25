# Audio Pipeline Telemetry

Firmware emits `PIPE` records through ESP-IDF and Zephyr logs. These examples omit other fields:

```text
PIPE v=1 dev=esp stage=audio part=tx epoch_id=0x1234 uptime_ms=20000 capture_ok=1000 encode_ok=100 vox_skip=900 tx_handoff=100
PIPE v=1 dev=esp stage=espnow part=tx epoch_id=0x5678 uptime_ms=20000 node_mac=aa:bb:cc:dd:ee:ff tx_offer=100 tx_queue_full=2
PIPE v=1 dev=esp stage=transport node=2 source=100 spi_ok=98
PIPE v=1 dev=nrf stage=mesh node=2 ingress_ok=98 rf_tx_ok=96
```

The parser requires `v=1`, `dev`, and `stage`. Most event counts are cumulative.
Queue depths, maxima, current settings, and identity fields are not event counts.

## Stages

| Device | Stage | Boundary |
|---|---|---|
| S31 | `audio` | Separate `tx`, `rx`, `playout`, and `bt` parts for capture, codecs, queues, and playback |
| S31 | `audio_timing` | Music and notification processing time |
| S31 | `transport` | nRF audio handoff, SPI admission, redundancy, and playback queue |
| S31 | `spi` | SPI queue, framing, parser, and ACK handling |
| S31 | `espnow` | Separate `tx` and `rx` parts for ESP-NOW admission, completion, and receive handling |
| S31 | `espnow_timing` | Slot timing, queue age, radio completion, send errors, and internal heap |
| nRF | `spi` | SPI ingress and nRF-to-S31 transactions |
| nRF | `mesh` | Ingress, TDMA, ESB audio, relay, and SPI egress |
| nRF | `tdma` | Due work, coalescing, and late execution |
| nRF | `rf` | ESB completion, timeout, FIFO, and RX restart |

The capture boundary is PCM read from the audio codec, not a direct ADC measurement.
The tested mesh configuration uses LC3; local codec loopback still uses Opus.
Only inspect stages relevant to the selected transport.

## Useful Counters

- **Capture and encoding:** `capture_ok`, `encode_ok`, and `tx_handoff` track successive audio boundaries.
- **VOX silence:** `vox_skip` counts intentionally suppressed LC3 frames, not failures.
  Capture can continue at 50 frames/s while encoding and audio transmission stay at zero.
- **Local discards:** Examples include `tx_queue_full`, `jitter_late`, and `rx_store_reject`.
  Some counters overlap; do not add every drop or gap counter into one loss total.
- **Reception and playback:** `rx_store_pop`, `decode_ok`, `conceal`, and `play_ok` distinguish received audio from concealment and output writes.
- **Radio completion:** nRF `rf_tx_ok` and ESP-NOW `tx_radio_ok` describe local completion, not remote application delivery.

Control traffic and required relaying can continue while the local microphone is silent.
Retries and duplicate rejection also need separate interpretation from lost audio.

## Compare Intervals Carefully

Use changes between samples, not lifetime totals, to assess a test window.

- Audio records carry `epoch_id`, `uptime_ms`, and `part`; they do not carry a MAC address. Keep serial-port identity attached.
- ESP-NOW records also carry `node_mac`. Keep different stages, parts, and epochs separate.
- S31 nRF-transport and nRF records lack those epoch and uptime fields. Use capture timestamps and boot boundaries.
  Node IDs alone are not stable device identities.

Derived ESP rates require at least two samples with increasing uptime in one epoch.
Each counter must cover the full interval without decreasing. Missing or reset counters are unavailable, not zero.
Generic counter summaries handle decreases as resets, but cannot establish continuity across an unobserved reboot.

Sender and receiver windows must refer to the same traffic and time period.
Aggregate counters alone do not establish an exact end-to-end delivery percentage.

## Capture and Reports

From the repository root, substitute your serial ports in this example:

```sh
uv run benchmark.py --duration 120 --baud 115200 \
  --ports /dev/ttyACM0 /dev/ttyACM1 --out-dir logs/benchmark
```

Prefer stable `/dev/serial/by-id/` paths when available. Each run writes `raw/`, `summary.json`, and `report.txt`.

The report adds `WARN` for measured ESP discard increases and marks missing interval coverage as unavailable.
Missing ESP coverage can change otherwise-OK health to `UNKNOWN`; existing warnings or failures remain.
Health is a diagnostic summary, not proof of clean audio or successful radio delivery.

Optional `cpu` records report task runtime; `cpu_span` records measure elapsed wall time, including waits and preemption.
See [CPU profiling](s31-cpu-profiling.md) for interpretation and overhead.

Counters and successful I2S writes do not prove audible output, acoustic latency, coexistence quality, or battery life.
See [audio.md](audio.md) for the pipeline and its limits.
