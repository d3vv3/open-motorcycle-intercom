# ESP32-S3/nRF52840 Inter-MCU Contract

This document describes the current dual-MCU implementation. Wire values and
payload layouts are specified in [protocol.md](protocol.md#esp32-s3nrf52840-bridge).

## Ownership

The ESP32-S3 owns:

- 16 kHz mono audio I/O and 20 ms frame cadence
- Opus encode/decode, VOX, playout recovery, and source mixing
- user policy and lifecycle requests
- the SPI slave and application-side bridge state

The nRF52840 owns:

- ESB radio operation and mesh membership
- the complete RF TDMA scheduler, including voice slots and control windows
- coordinator SYNC generation and participant synchronization
- the SPI master polling loop

The nRF owns dual-MCU TDMA. RF timing is not derived from bridge queue depth.
When the node is coordinator, the nRF samples the ESP I2S word-select (`WS`)
edges and applies bounded correction to its local TDMA period. Participants
apply phase correction or reacquire from coordinator `SYNC` timestamps. The
coordinator currently advertises zero drift, so participant rate correction is
not active.

There is no runtime Bluetooth implementation in the current firmware. The
dual-MCU path also does not implement cross-MCU low-power coordination,
authentication, encryption, key storage, or firmware update.

## Physical Link

The interconnect is SPI only:

- nRF52840 master, ESP32-S3 slave
- 4 MHz, mode 0 (`CPOL=0`, `CPHA=0`), 8-bit words, MSB first
- one full-duplex poll every 2 ms
- exactly 256 bytes transferred per transaction
- manual active-low CS, with 100 us setup and 50 us hold on the nRF
- a separate nRF-to-ESP GPIO pulse for accepted ESP audio
- ESP `WS` wired to an nRF counter input for coordinator clock discipline

UART and I2C are not implemented bridge transports.

## Frame Codec

Each 256-byte transaction contains one frame at offset zero or an idle frame of
zeros:

```text
| 0xAA | LEN | SEQ | TYPE | PAYLOAD... | CRC8 | zero padding... |
```

`LEN = payload_length + 2` and covers `SEQ`, `TYPE`, and payload. CRC8 covers
`LEN`, `SEQ`, `TYPE`, and payload; it uses polynomial `0x07` with initial value
zero. A decoded frame occupies `payload_length + 5` bytes. Invalid sync, length,
truncation, and CRC are rejected. The 8-bit sequence is diagnostic and detects
gaps; audio admission deduplication also recognizes a re-presented sequence.

The implemented bridge packet types are `AUDIO 0x01`, `STATUS 0x02`,
`MESH_EVENT 0x03`, `CONTROL 0x04`, `LOG 0x05`, and `AUDIO_V2 0x06`. Normal
dual-MCU audio uses `AUDIO_V2`; `AUDIO` is reserved for the RTT diagnostic.

`AUDIO_V2` carries an intact redundant bundle and can contain predecessor
audio. ESP-to-nRF payloads contain only the bundle. nRF-to-ESP payloads prefix
the bundle with the RF source node ID. The receiver offers `previous2`, then
`previous1`, then the current frame to sequence-aware playout recovery.

## Priority and Flow Control

Control has priority over audio on both SPI endpoints. A lifecycle control frame
may pass while ESP audio is retained for ACK. The nRF selects its outbound
control queue before its outbound audio queue and removes the selected entry
only after a successful SPI transaction.

ESP-to-nRF audio uses GPIO-ACK stop-and-wait. The ESP retains one frame and
re-presents it on each poll until the nRF has durably admitted it and pulses the
ACK GPIO. The nRF treats a repeated bridge sequence as a duplicate and pulses
ACK again without admitting it twice. A missing pulse releases the ESP frame
after 50 ms. This ACK is bridge flow control, not an RF delivery ACK.

## Bounded Queues

All bridge and nRF audio queues are statically bounded:

- ESP outbound audio ring: 15 usable entries. Enqueue is nonblocking; when no
  slot or mutex is immediately available, the new frame is rejected.
- ESP pending control: one replaceable slot, independent of the audio ring.
- ESP in-flight audio: one retained stop-and-wait entry.
- nRF bridge audio ingress: 16 entries. On full, it drops the oldest entry and
  attempts to admit the newest.
- nRF local RF audio ring: 15 usable entries. On full, it drops the oldest and
  keeps the newest.
- nRF outbound SPI audio ring: 15 usable entries. On full, it drops the oldest.
- nRF outbound SPI control ring: 3 usable entries. On full, it drops the oldest.

The nRF on-air relay ring also has 15 usable entries and drops the oldest on
overflow. Its on-air control ring has 31 usable entries and rejects a new item
when full.

Audio queued on the ESP for more than 120 ms is stale and dropped before
transfer. The ESP also purges or gates audio unless a fresh bridge status reports
an active mesh and a nonzero local node ID.

## Status and Events

The nRF requests a pushed eight-byte status snapshot every 200 SPI transactions,
about every 400 ms at the current poll cadence, and pushes another snapshot
after lifecycle command handling. An explicit status request can also trigger a
snapshot. Fields are:

| Offset | Field | Meaning |
|---:|---|---|
| 0 | `role` | none, coordinator, or participant |
| 1 | `peer_count` | active peers, or `0xFF` when unknown |
| 2 | `node_id` | `0` while unassigned |
| 3 | `version` | bridge protocol `2` |
| 4 | `mesh_state` | `IDLE`, `SCANNING`, `JOINING`, or `ACTIVE` |
| 5 | signed `slot_index` | `-1` while unassigned |
| 6 | `coordinator_id` | `0` while unknown |
| 7 | `marker` | `0xA5` |

The ESP considers status fresh for 5 seconds. Expiry marks the bridge
disconnected, blocks new mesh audio, and purges queued and in-flight audio.

Defined event IDs are:

| Value | Event | Current meaning |
|---:|---|---|
| `0x01` | `MESH_READY` | Mesh reached usable coordinator or participant state |
| `0x02` | `PEER_JOINED` | Topology gained a peer |
| `0x03` | `PEER_LEFT` | An announced peer left or timed out |
| `0x04` | `BECAME_COORDINATOR` | Local nRF won or completed election |
| `0x05` | `SYNC_LOST` | Participant lost coordinator timing |
| `0x06` | `MESH_STOPPED` | Mesh stop completed |
| `0x07` | `COMMAND_ACK` | Command, generation, and result |
| `0x08` | `AUDIO_V2_READY` | Defined, but not emitted by the current firmware |

`PEER_JOINED` and `PEER_LEFT` are topology events, not lifecycle readiness.
`COMMAND_ACK` means the nRF command worker applied the transition request; mesh
readiness is reported separately by status and readiness events.

Telemetry is pushed, not pull-only. Both MCUs periodically log queue admission
and drops, ACK and retry timeouts, frame sequence/CRC failures, TDMA deadlines
and correction state, ESB outcomes, and audio recovery counters.

## Lifecycle Commands

START and STOP carry an 8-bit generation. The ESP permits one lifecycle command
at a time. It sends the same command and generation up to three times, waiting
300 ms for a matching `COMMAND_ACK` after each attempt. A status transition can
serve as the implemented legacy ACK fallback. Unrelated or stale generations do
not complete the command.

The nRF coalesces pending lifecycle requests in its command worker, applies the
latest requested state, then pushes a command ACK and status snapshot.

## Failure Behavior

- A failed nRF SPI transaction leaves the selected outbound queue entry pending
  for the next poll.
- A lost ESP audio ACK is bounded by the 50 ms timeout.
- Audio older than 120 ms is discarded instead of extending latency.
- Status older than 5 seconds disables dual-MCU mesh audio until fresh status
  arrives.
- RF packet loss is not retried across SPI; predecessor recovery and Opus PLC
  handle audio loss.

The dual-MCU ESP32-S3/nRF52840 path is the current implementation. Firmware
update over this interface is not implemented.
