# TDMA Scheduling

OMI uses fixed transmission windows for joined-node voice and control. The
implementation bounds opportunities and drops late work; it does not claim
collision-free RF behavior.

## Frame Layout

The compiled protocol constants are shared by ESP-NOW and ESB:

| Item | Value |
|---|---:|
| Frame | 20 ms |
| Voice slots | 8 |
| Slot | 2 ms |
| Per-slot guard | 500 us, included in the slot |
| Control window | 2 ms after the voice slots |
| SYNC cadence | every 10 frames |

Slot 0 belongs to the coordinator. Participants receive a nonzero slot in
JOIN_ACK and validate subsequent SLOT_MAP records before applying changes.

Each node sends at most one local or relayed audio packet in its slot. A slot
callback outside its generation or deadline is dropped. Ordinary audio is not
relayed; coordinator-granted active audio may be relayed in the relay node's own
slot, with TTL and duplicate suppression.

## Control Windows

For joined nodes, ownership rotates by frame. Every tenth frame reserves the
control window for coordinator SYNC; otherwise the owner is derived from the
frame counter and slot index.

ESP-NOW queues non-SYNC control in a bounded priority queue. It coalesces
replaceable status/topology records, prefers lifecycle and topology over periodic
traffic, and lets sufficiently old records bypass priority. At most one queued
record is submitted in an owned control window.

Unassigned ESP-NOW JOIN requests cannot use slot ownership. They transmit
directly with a minimum interval plus random jitter. Graceful ESP-NOW LEAVE also
bypasses the queue during synchronized shutdown. These are bounded lifecycle
exceptions, not a general CSMA control path.

## Election and Join

A node starts in `SCANNING`. A valid coordinator SYNC moves it to `JOINING`; a
successful targeted assignment moves it to `ACTIVE`. If no mesh is found before
the scan timeout, it becomes coordinator.

The nRF scan timeout adds 0-500 ms of deterministic address-derived backoff.
JOIN_V2 carries the requester's five-byte ESB identity, allowing a coordinator to
reuse an assignment for retries and target JOIN_ACK_V2. JOIN attempts are bounded;
exhaustion clears the candidate coordinator and returns to a backed-off scan.

If active coordinators hear each other, the lexicographically lower radio address
wins. The higher-address nRF coordinator stops TDMA/audio admission, clears stale
assignment and queued state, enters `JOINING`, and sends identity-bearing JOIN
directly to the winner. This also lets previously separated coordinators converge.

nRF LEAVE carries the sender's five-byte identity. A receiver removes a peer only
when node ID and stored identity match, preventing a stale/reused node ID from
removing the wrong peer.

## Synchronization

The coordinator sends SYNC in its reserved control window. A participant acquires
the frame epoch from valid coordinator SYNC and suppresses slot transmission until
that acquisition. Later SYNC packets are compared with a 16-entry history of
local frame boundaries. Small phase error and advertised drift become bounded
period corrections; a frame-counter difference greater than four causes an
explicit timer quiesce and reacquisition.

The nRF coordinator also counts rising edges from the ESP's 16 kHz I2S WS using
GPIOTE/PPI and a hardware counter. A sample is rejected when no signal is present,
its frame span is invalid, or its edge count is outside 75-125% of expectation.
Valid error is filtered and contributes to pending correction.

Pending nRF correction is capped at 2 ms. The frame period changes by at most one
Zephyr timer tick per frame. With `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`, the
actual correction resolution is **100 us**. Diagnostics report requested,
pending, applied, and clamped corrections plus callback jitter and sync
acquisition. This does not establish 100 us over-the-air synchronization accuracy.

## Loss and Reconfiguration

KEEPALIVE and STATUS continue independently of voice. A peer with no refreshing
packet for 3 seconds is removed and the coordinator queues a new SLOT_MAP.
Participant coordinator/SYNC timeout stops TDMA and returns to scanning/election;
there is no CSMA voice fallback.

Participants reject SLOT_MAP records with the wrong coordinator, invalid counts,
duplicate or missing assignments, or invalid speaker grants. The protocol does
not ACK SLOT_MAP records.
