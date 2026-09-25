# Mesh and Bridge Protocol v3

The current LC3 firmware uses mesh version `0x03` and SPI bridge version `3`.
The name `AUDIO_V2` identifies a packet format; it does not mean protocol version 2.

## Mesh Radio Header

Every mesh packet starts with this packed 8-byte header:

| Offset | Field | Bytes | Meaning |
|---:|---|---:|---|
| 0 | `version` | 1 | `0x03` |
| 1 | `type` | 1 | Message ID below |
| 2 | `src_id` | 1 | Source node; `0` for an unassigned joining node |
| 3 | `seq` | 1 | Radio packet sequence |
| 4 | `ttl` | 1 | Remaining hop limit |
| 5 | `flags` | 1 | Relay request, relayed, and speaker-granted bits |
| 6 | `payload_len` | 2 | Payload length, little-endian |

Assigned node IDs are 1-8, with TDMA slot `node_id - 1`.
Receivers reject mismatched protocol versions and invalid packet lengths.

| ID | Message | Purpose |
|---:|---|---|
| `0x01` | `AUDIO` | ESP-NOW audio; rejected by the current nRF audio path |
| `0x02` / `0x03` | `JOIN` / `JOIN_ACK` | Membership request and assignment |
| `0x04` | `LEAVE` | Departure |
| `0x05` / `0x06` | `SYNC` / `SLOT_MAP` | Timing and slot assignments |
| `0x07` / `0x08` | `STATUS` / `KEEPALIVE` | Node status and liveness |
| `0x09` / `0x0A` | `SPEAKER_GRANT` / `SPEAKER_RELEASE` | Speaker-control IDs |
| `0x0B` / `0x0C` | `JOIN_V2` / `JOIN_ACK_V2` | Extended membership IDs |
| `0x0D` | `AUDIO_V2` | LC3 bundle used by nRF ESB |

These are shared IDs; not every transport uses every message.
The nRF coordinator requires the LC3 capability bit in `JOIN_V2` requests.
ESP-NOW membership does not enforce the same capability check; use matching firmware on all nodes.

## LC3 Audio Bundles

Mesh voice is 16 kHz mono. Each 20 ms audio unit contains two 10 ms LC3 frames, 24 bytes each.
The nRF forwards these encoded bytes; the S31 performs encoding and decoding.

An `AUDIO_V2` payload begins with an 8-byte header:

```text
codec | frame_ms | stream_id | flags | current_seq (2 bytes) | current_len | previous1_len
```

All fields are one byte except `current_seq`, which is **big-endian**.
The codec is `0x02` (LC3), duration is `20`, and every present audio unit must contain exactly 48 bytes.
`stream_id` identifies the source, not a new speech segment. The S31 accepts zero or the matching source node ID.

Audio data follows in this order: oldest predecessor, immediate predecessor, current audio.
The oldest predecessor's length is inferred from the remaining payload size.

| Flag | Meaning |
|---:|---|
| `0x01` | Current audio active |
| `0x02` / `0x04` | Immediate predecessor present / active |
| `0x08` / `0x10` | Oldest predecessor present / active |

Other flag bits are rejected. The S31 currently attaches at most one predecessor; the format supports two.

| Audio units | Bundle bytes | Bytes with mesh header |
|---|---:|---:|
| Current only | 56 | 64 |
| Current + one predecessor | 104 | 112 |
| Current + two predecessors | 152 | 160 |

The 16-bit audio sequence is separate from the 8-bit radio packet sequence.
During VOX silence, no LC3 audio is encoded or sent. There is no explicit end-of-speech packet.
The sender clears cached predecessors and advances through at most five quiet sequence slots before holding the sequence.
The receiver uses bounded concealment before treating an empty stream as idle.

ESP-NOW uses its separate `AUDIO` payload format. The S31 LC3 build requires LC3 there too; it does not negotiate codecs.

## Control and Delivery

Coordinator SYNC is scheduled every 200 ms. The nRF queues STATUS and KEEPALIVE every second;
ESP-NOW schedules keepalives every 500 ms. Peer timeout is 3 seconds.
These control messages continue during VOX silence.

The coordinator grants relay service to at most two active speakers.
Audio starts with TTL 2; the implemented relay path permits one forwarding hop.
See [TDMA scheduling](tdma.md) for slot ownership and deadlines.

nRF ESB sends with RF acknowledgments disabled. Lost audio is handled through predecessors and receiver concealment, not RF retransmission.
ESP-NOW completion callbacks have separate MAC-layer semantics; they do not prove application delivery.

## S31/nRF SPI Bridge

The nRF is SPI master; the S31 is slave. Transfers are 256 bytes, mode 0, at 4 MHz.
The polling loop has a nominal 2 ms interval, not a guaranteed transaction deadline.
See [wiring](wiring.md) for connections. The `uart_bridge` name is historical; this link uses SPI.

```text
0xAA | LEN | SEQ | TYPE | PAYLOAD... | CRC8 | zero padding...
```

`LEN = payload_length + 2`. CRC8 covers `LEN` through the payload, using polynomial `0x07` and initial value zero.
The 256-byte transfer allows at most 251 payload bytes. SPI sequence numbers are separate from radio and audio sequences.

| ID | Message | Direction |
|---:|---|---|
| `0x01` | Legacy `AUDIO` | Rejected by current nRF firmware |
| `0x02` | `STATUS` | nRF to S31 |
| `0x03` | `MESH_EVENT` | nRF to S31 |
| `0x04` | `CONTROL` | S31 to nRF |
| `0x05` | `LOG` | nRF to S31 |
| `0x06` | `AUDIO_V2` | Both directions |

S31-to-nRF audio carries the bundle directly. nRF-to-S31 audio prefixes the bundle with one source-ID byte.
Receive timestamps are assigned locally; they are not included in this payload.

### Status and Compatibility

The v3 STATUS payload is 10 bytes, in this order:

```text
role | peer_count | node_id | version | mesh_state | slot_index |
coordinator_id | marker | audio_codec | audio_frame_ms
```

Each field is one byte; `slot_index` is signed. The marker is `0xA5`.
Startup selects nRF only after a fresh status advertises version 3, LC3, and 20 ms audio.
Audio transfer additionally requires an ACTIVE mesh and an assigned node ID.
Older 3-byte and 8-byte statuses remain readable but do not establish LC3 compatibility.

### Two Different Acknowledgments

- **Audio admission:** The nRF pulses GPIO ACK for 20 µs after accepting an audio packet or recognizing its admitted duplicate.
  The S31 repeats unacknowledged SPI audio until ACK or a 50 ms timeout. This does not confirm radio delivery.
- **Commands:** MESH_START and MESH_STOP carry a command byte and generation byte.
  A COMMAND_ACK event returns the command, matching generation, and signed result: zero for success, minus one for failure.

See [the inter-MCU contract](inter_mcu.md) for lifecycle and queue handling.

## Security

CRC detects accidental corruption; it does not authenticate the sender.
On-air authentication, encryption, key management, and secure firmware update are not implemented.

## Source Definitions

- [Mesh header and message IDs](../shared/mesh_protocol_defs.h)
- [LC3 bundle encoder and parser](../shared/audio_bundle.c)
- [Bridge messages and status layout](../shared/bridge_protocol_defs.h)
- [SPI framing and CRC](../shared/bridge_frame.c)
