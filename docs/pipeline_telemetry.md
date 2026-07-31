# Audio Pipeline Telemetry

Firmware emits cumulative pipeline counters through the normal ESP-IDF and
Zephyr logging systems. The message body starts with `PIPE` and uses a stable
key-value schema:

```text
PIPE v=1 dev=esp stage=transport node=2 source=1000 gate_drop=0 spi_ok=998
PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok=998 rf_tx_ok=995 rf_tx_fail=3
```

Required fields are `v`, `dev`, and `stage`. All metric values are unsigned
cumulative integers. Counter names ending in `_drop`, `_fail`, `_err`, or
`_timeout` identify a loss reason. Queue depths and maximum durations are gauges;
do not calculate deltas for those fields.

`benchmark.py` accepts records with ESP-IDF prefixes, Zephyr prefixes, or no
prefix. It groups records by device, stage, node, and any explicit session/link
identity. Reports include first/last values, reset-aware deltas, and per-counter
`reset_epochs`; a counter decrease starts a new epoch instead of producing a
negative delta.

## Stages

| Device | Stage | Boundary |
|---|---|---|
| ESP | `audio` | ADC capture through Opus and I2S playback |
| ESP | `transport` | Audio callback through bridge RX and playback queue |
| ESP | `spi` | ESP bridge queue, framing, parser, and ACK handling |
| nRF | `spi` | SPI ingress admission and nRF-to-ESP transactions |
| nRF | `mesh` | Ingress queue, TDMA ring, RF audio, and SPI egress |
| nRF | `tdma` | Timer due, work coalescing, and late execution |
| nRF | `rf` | ESB driver completion, timeout, FIFO, and RX restart |

## Finding The First Loss

Compare deltas from one talk interval. Start at the sender and stop at the first
boundary where accepted output is lower than input after subtracting explicit
drops.

1. `esp:transport source` counts frames offered by the codec callback.
2. `esp:transport spi_ok` counts frames admitted to the ESP SPI queue.
3. `nrf:spi ingress_ok` counts unique frames durably admitted by nRF. The GPIO
   ACK is now sent only at this boundary or for a known duplicate.
4. `nrf:mesh ingress_ok` counts frames moved into the owner-context TDMA ring.
5. `nrf:mesh rf_tx_ok` counts successful local ESB completion. ESB broadcasts
   have no receiver ACK, so this does not prove over-the-air delivery.
6. Receiver `nrf:mesh rf_rx_ok` counts accepted RF audio.
7. Receiver `nrf:mesh spi_out_ok` counts frames admitted to its outbound SPI
   queue.
8. `esp:transport play_q_ok` counts frames admitted to ESP playback.
9. `esp:audio play_ok` counts complete I2S writes.

Use `spi_gap` and the nRF E2E gap fields to estimate missing frame counts. These
are stage-local loss estimates, not end-to-end delivery percentages.

Delivery is reported only when separate TX and RX records have matching explicit
`session`, sender, receiver, and stage semantics. A single port's unrelated TX
and RX counters are never treated as correlated delivery. Missing identity yields
`insufficient correlated data`; reset epochs or RX greater than TX yield
`inconsistent correlated data` and suppress the percentage. This avoids a false
delivery claim but cannot infer correlation for legacy logs that lack link
identity.

## Commands

```bash
python benchmark.py --duration 120
python -m pytest tests/python
```
