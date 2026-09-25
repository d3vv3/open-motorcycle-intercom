# TDMA Scheduling

The implemented mesh schedule is shared by ESP-NOW and nRF ESB.

| Item | Value |
|---|---:|
| Maximum nodes | 8 |
| Frame | 20 ms |
| Voice slots | 8 x 2 ms |
| Guard within each voice/control window | 500 µs |
| Control window | 2 ms, from 16 to 18 ms |
| Frame margin | 2 ms, from 18 to 20 ms |
| SYNC | Every 10 frames, about 200 ms |

Each 20 ms audio payload contains two 10 ms LC3 frames: 24 bytes each, 48 bytes total.
They travel together in one packet.
Redundant previous audio increases packet size, not the transmission cadence.

Slot ownership is fixed: `slot_index = node_id - 1`.
Local and relay audio share one transmission opportunity per node per frame.
Relay grants are limited to two active speakers.

The guard leaves 1.5 ms before each slot's deadline.
Deadline checks reject late work but cannot guarantee RF completion timing.

During VOX silence, local LC3 encoding and audio transmission stop after the hangover period.
Capture and voice detection continue. Slots may remain unused, but synchronization, keepalives, and required relay/control traffic continue.

ESP-NOW discovery uses bounded randomized contention.
ESB discovery uses bounded join retries and scan backoff.
These are discovery exceptions, not a general CSMA voice mode.
The schedule does not guarantee collision-free RF operation.

The nRF owns ESB timing when the bridge is active.
Participants synchronize to the coordinator and reacquire SYNC after loss.
S31 GPIO47 provides a physical 48 kHz WS reference, separate from the 16 kHz mono LC3 audio.
The current runtime does not enable WS-based TDMA clock correction.

Two-board operation has been tested; eight-node operation still needs validation.
Timing counters alone do not prove end-to-end audio latency or RF delivery.

See [protocol.md](protocol.md) for packet and bridge formats.
