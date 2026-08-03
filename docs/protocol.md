# OMI Protocol v2

This document describes the implemented on-air mesh protocol and the
ESP32-S3/nRF52840 bridge protocol. The current on-air version is `0x02`.

## On-Air Envelope

Every mesh packet starts with this packed 8-byte header:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `version` | Must be `0x02` |
| 1 | 1 | `type` | Packet type |
| 2 | 1 | `src_id` | Source node ID; `0` means unassigned |
| 3 | 1 | `seq` | Per-sender 8-bit packet sequence |
| 4 | 1 | `ttl` | Remaining relay hops |
| 5 | 1 | `flags` | Relay and speaker flags |
| 6 | 2 | `payload_len` | Bytes after the header, little-endian |

Node IDs are `1` through `8`. The coordinator normally owns ID `1` and slot
`0`; other assignments map node ID `n` to slot `n - 1`.

Header flags are:

| Value | Name | Meaning |
|---:|---|---|
| `0x01` | `RELAY_REQUEST` | Packet may be considered for relay |
| `0x02` | `RELAYED` | Packet has traversed a relay |
| `0x04` | `SPEAKER_GRANTED` | Source has a current coordinator relay grant |

Receivers fail closed by rejecting packets with another protocol version.
There is no version negotiation or optional-field negotiation.

## Packet Types

| Value | Name | Purpose |
|---:|---|---|
| `0x01` | `AUDIO` | Legacy single-frame audio envelope |
| `0x02` | `JOIN` | ESP-NOW join request |
| `0x03` | `JOIN_ACK` | ESP-NOW join assignment |
| `0x04` | `LEAVE` | Graceful departure |
| `0x05` | `SYNC` | Coordinator timing |
| `0x06` | `SLOT_MAP` | Membership and relay map |
| `0x07` | `STATUS` | Peer health and relay observations |
| `0x08` | `KEEPALIVE` | Presence during audio silence |
| `0x09` | `SPEAKER_GRANT` | Active-speaker and relay assignment |
| `0x0A` | `SPEAKER_RELEASE` | Active-speaker release |
| `0x0B` | `JOIN_V2` | nRF/ESB identity-bearing join request |
| `0x0C` | `JOIN_ACK_V2` | nRF/ESB identity-targeted assignment |
| `0x0D` | `AUDIO_V2` | Redundant Opus bundle |

Normal nRF/ESB microphone audio must use `AUDIO_V2`. On that transport,
legacy `AUDIO` is accepted only for the fixed-format RTT diagnostic. ESP-NOW
still uses legacy `AUDIO` for normal audio.

## Audio V2 Bundle

The `AUDIO_V2` payload has an exact 8-byte fixed header followed by zero to two
predecessor frames and the current Opus frame:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `codec` | `0x01` for Opus |
| 1 | 1 | `frame_ms` | Must be `20` |
| 2 | 1 | `stream_id` | Logical source; `0` is allowed on receive |
| 3 | 1 | `flags` | Bundle flags below |
| 4 | 2 | `current_seq` | Current frame sequence, big-endian |
| 6 | 1 | `current_len` | Current frame bytes, `1..64` |
| 7 | 1 | `previous1_len` | Immediate predecessor bytes, `0..64` |
| 8 | N | frame data | `previous2`, then `previous1`, then `current` |

There is no `previous2_len` field. When `PREVIOUS2_PRESENT` is set, its length
is inferred as:

```text
payload_len - 8 - previous1_len - current_len
```

Bundle flags are:

| Value | Name |
|---:|---|
| `0x01` | `CURRENT_ACTIVE` |
| `0x02` | `PREVIOUS1_PRESENT` |
| `0x04` | `PREVIOUS1_ACTIVE` |
| `0x08` | `PREVIOUS2_PRESENT` |
| `0x10` | `PREVIOUS2_ACTIVE` |

Unknown flag bits, inconsistent presence/activity flags, empty current frames,
and malformed lengths are rejected. `previous2` is valid only when
`previous1` is also present.

Each Opus frame is at most 64 bytes. The three-frame data area is at most 192
bytes, the complete bundle is at most 200 bytes, and the complete on-air packet
is at most 208 bytes including the mesh header.

The ESP sender currently attaches only `previous1`, and only when the cached
frame was active, eligible, and immediately precedes `current_seq`. Parsers and
playout recovery support both predecessors. Relays preserve the bundle when it
fits the remaining slot airtime and strip the oldest predecessor first when it
does not.

## Legacy Audio

The legacy `AUDIO` payload is:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `codec` | `0x01` for Opus |
| 1 | 1 | `frame_ms` | `20` |
| 2 | 1 | `stream_id` | Source stream |
| 3 | 1 | `audio_flags` | Bit `0x01` means active audio |
| 4 | N | data | Opus data, or the nRF RTT diagnostic payload |

## TDMA and RF Delivery

A frame is 20 ms. It contains eight fixed 2 ms voice slots followed by a 2 ms
control window. Each voice or control window closes 500 us before its nominal
end, providing the fixed guard interval. SYNC is sent every ten frames; that
control window is reserved for the coordinator. Other control-window ownership
rotates by frame and assigned slot.

Audio is attempted only in the sender's voice slot. Late work is dropped. RF
audio has no delivery ACK and no protocol retry. nRF ESB transmissions set the
no-ACK flag; ESP-NOW completion callbacks account for local send completion but
do not trigger audio retransmission. Loss is handled by V2 predecessor recovery
and Opus packet-loss concealment.

## Relaying

Ordinary audio remains one hop. The coordinator may grant at most two active
speakers and publishes a relay mask for each. A packet is relay-eligible only
when it has `RELAY_REQUEST`, `SPEAKER_GRANTED`, a positive TTL, and the receiving
node is selected in that speaker's relay mask. Relays deduplicate by packet type,
source ID, and sequence. The default audio TTL is `2`; each relay decrements it
and sets `RELAYED`.

Relay arbitration is transport-specific:

- ESP-NOW always sends queued local audio first and uses an otherwise idle local
  slot for one relay.
- nRF alternates local and relay traffic when both are pending. It may defer an
  active local V2 frame and send its successor on the next local turn only when
  that successor proves recovery by carrying the deferred frame as `previous1`.

There is no cross-transport claim of identical relay scheduling.

## Join Identity

ESP-NOW obtains the sender's 6-byte MAC from receive metadata, so its payloads
do not carry an address:

| Packet | Size | Payload |
|---|---:|---|
| `JOIN` | 2 | `capabilities`, `reserved` |
| `JOIN_ACK` | 3 | `assigned_id`, `slot_index`, `coordinator_id` |
| `LEAVE` | 0 | Sender identity comes from receive metadata |

The coordinator unicasts `JOIN_ACK` to the requester MAC. Repeated joins from a
known MAC refresh that member and receive the same assignment.

ESB receive delivery does not provide a transmitter identity. nRF therefore
uses identity-bearing payloads:

| Packet | Size | Payload |
|---|---:|---|
| `JOIN_V2` | 7 | `capabilities`, `reserved`, `requester_addr[5]` |
| `JOIN_ACK_V2` | 8 | `assigned_id`, `slot_index`, `coordinator_id`, `target_addr[5]` |
| `LEAVE` | 5 | `sender_addr[5]`; this is the current transmitted form |

`JOIN_V2` is sent with source ID `0`. Its stable device-derived ESB address lets
the coordinator deduplicate retries. `JOIN_ACK_V2` is broadcast, but only the
requester whose local address matches `target_addr` accepts it. nRF ignores the
legacy `JOIN` and `JOIN_ACK` forms.

The nRF receiver also accepts a zero-length legacy `LEAVE` for compatibility.
That form identifies the departing peer only by `SrcID`. New senders use the
five-byte identity-bearing form.

## Sync, Membership, and Status

`SYNC` differs only in coordinator address width:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | `frame_counter`, little-endian |
| 4 | 2 | signed `drift_ppm`, little-endian |
| 6 | 6 or 5 | ESP-NOW MAC or nRF ESB coordinator address |

The address is also the coordinator-election tie-breaker: the lower
lexicographic address wins. nRF coordinators currently send `drift_ppm = 0`;
participants currently apply phase correction or reacquire timing from received
SYNC timestamps. The advertised rate-correction term is inactive while
`drift_ppm` remains zero.

`SLOT_MAP` is always 14 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `slot_count` |
| 1 | 8 | `slot_ids`; zero marks an unused slot |
| 9 | 1 | `active_speaker_count` |
| 10 | 2 | `active_speaker_ids` |
| 12 | 2 | `relay_masks` |

Current senders publish `slot_count = 8`. `SPEAKER_GRANT` is five bytes
(`speaker_count`, two IDs, two masks); `SPEAKER_RELEASE` is three bytes
(`speaker_count`, two IDs).

`STATUS` is exactly 8 bytes:

| Offset | Size | Field | Sentinel or meaning |
|---:|---:|---|---|
| 0 | 1 | `battery_pct` | `0..100`; `255` unknown |
| 1 | 1 | signed `rssi_dbm` | `127` unknown |
| 2 | 1 | `peer_count` | Active peers |
| 3 | 1 | `fw_version` | Currently `0x02` |
| 4 | 1 | signed `temperature_c` | `127` unknown |
| 5 | 1 | `heard_bitmap` | Sources heard in the reporting interval |
| 6 | 1 | `relay_bitmap` | Sources relayed in the reporting interval |
| 7 | 1 | `active_speakers` | Current active/granted count |

`KEEPALIVE` is two bytes: `battery_pct` and `reserved`.

## Liveness and Coordinator Loss

The ordinary peer timeout is 3000 ms without accepted traffic. Audio, join
retries, keepalives, and status reports refresh the relevant peer records; SYNC
refreshes coordinator presence. Silence therefore does not depend on audio
traffic.

The transports differ in coordinator handling:

- ESP-NOW sends KEEPALIVE every 500 ms and STATUS every 1000 ms. Its general
  3000 ms peer timeout also removes a missing coordinator and returns the node
  to scanning.
- nRF sends STATUS and KEEPALIVE together every 1000 ms. It excludes the
  coordinator from the 3000 ms peer reaper and instead returns to scanning after
  5000 ms without accepted coordinator SYNC.

## ESP32-S3/nRF52840 Bridge

The bridge is SPI only. The nRF52840 is master and the ESP32-S3 is slave. It
runs at 4 MHz, mode 0, with manual active-low chip select. The master starts one
full-duplex transaction every 2 ms. Every transaction clocks exactly 256 bytes;
unused bytes are zero and an all-zero first byte is idle.

One framed message may occupy the start of a transaction:

```text
0xAA | LEN | SEQ | TYPE | PAYLOAD... | CRC8 | zero padding...
```

`LEN` is `2 + payload_length` and covers `SEQ`, `TYPE`, and payload. CRC8 covers
`LEN` through the final payload byte, uses polynomial `0x07`, and starts at zero.
The frame length before padding is `payload_length + 5`. The bridge constrains
application payloads to 208 bytes even though the generic frame codec can
represent up to 253.

Bridge packet types are:

| Value | Name | Direction |
|---:|---|---|
| `0x01` | `AUDIO` | Bidirectional legacy RTT diagnostic |
| `0x02` | `STATUS` | nRF to ESP |
| `0x03` | `MESH_EVENT` | nRF to ESP |
| `0x04` | `CONTROL` | ESP to nRF |
| `0x05` | `LOG` | nRF to ESP |
| `0x06` | `AUDIO_V2` | Bidirectional normal audio |

Audio payload direction is significant:

- ESP to nRF `AUDIO_V2`: the intact V2 bundle.
- nRF to ESP `AUDIO_V2`: `src_id` followed by the intact V2 bundle.
- ESP to nRF legacy `AUDIO`: `audio_flags` followed by the RTT payload.
- nRF to ESP legacy `AUDIO`: `src_id`, `audio_flags`, then the RTT payload.

ESP-to-nRF audio uses GPIO-ACK stop-and-wait across SPI: the ESP retains and
re-presents one in-flight audio frame until nRF admission produces an ACK pulse
or the 50 ms timeout expires. Control can pass while audio is waiting. This
bridge reliability mechanism does not apply to RF transmission.

The bridge status payload is eight bytes: `role`, `peer_count`, `node_id`,
`version`, `mesh_state`, signed `slot_index`, `coordinator_id`, and marker
`0xA5`. Bridge protocol version is `2`. START and STOP control payloads contain
`command` and 8-bit `generation`; command ACK events contain `command`, matching
`generation`, and signed result.

See [inter_mcu.md](inter_mcu.md) for ownership, queueing, status freshness, and
failure behavior.

## Security

On-air authentication and encryption are not implemented. Key management and
secure firmware update are outside protocol v2 and remain deferred.
