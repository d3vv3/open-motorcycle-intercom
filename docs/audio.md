# Audio Pipeline

The ESP32-S3 owns capture, processing, Opus, remote-source playback,
notifications, and I2S output in both radio configurations.

## Supported Format

The PCM/frame format is fixed:

| Parameter | Value |
|---|---:|
| Sample rate | 16 kHz |
| Channels | mono |
| Sample width | 16-bit |
| Frame duration | 20 ms / 320 samples |
| Opus mode | VoIP, VBR, in-band FEC, DTX, complexity 5 |
| Opus bitrate | 12 kbps default, configurable |

Initialization rejects other sample rates, channel counts, widths, and frame
durations. Encoded packets are bounded to 64 Opus bytes and are not fragmented.

## Capture and Transmission

Each complete ADC frame passes through DC blocking, low-pass and optional 80 Hz
high-pass filtering. Mesh mode also applies the implemented voice-cleanup stage
using the previous final speaker mix as its far-end reference. VOX uses RMS
hysteresis, minimum-active time, and hangover.

During active speech, each 20 ms frame is Opus encoded and offered to the selected
mesh transport with the active flag. During silence, the encoder remains in DTX
mode: periodic comfort-noise updates larger than two bytes are offered with the
active flag clear, while one- and two-byte DTX frames are suppressed. Mesh
KEEPALIVE/STATUS maintain liveness independently of this audio gating.

## Remote Playback

The playback path admits at most **two remote source IDs**. Each source owns an
independent bounded eight-frame queue, jitter state, and Opus decoder. Decoder
state is never shared between remote talkers or the dedicated loopback decoder.
A third concurrent source is rejected until a source slot becomes idle.

Playout begins after two queued frames. Backlog above the configured target is
trimmed, a bounded hold can refill a shallow queue, and short active-stream gaps
use Opus PLC. Intentional DTX silence does not count as an underrun. An idle empty
source slot is released and its decoder reset.

Each 20 ms cycle decodes admitted sources into a 32-bit accumulator. The result
is saturated to signed 16-bit PCM, which bounds both source count and output
range.

## Notifications

Startup, mesh lifecycle, and peer tones are non-blocking requests in a bounded
queue. The audio task owns tone generation and mixes notifications into the same
saturating playback path. Mesh and bridge callbacks do not write I2S directly.

Peer notifications are separate from lifecycle notifications: PEER_JOINED and
PEER_LEFT enqueue peer tones, while MESH_READY and user disable enqueue mesh
lifecycle tones. Transient SYNC_LOST does not immediately produce a peer-leave
tone.

## Teardown

Audio stop clears the run flag, wakes the task, and waits for its explicit
completion semaphore. Only after the task exits does stop reset source queues,
notification state, and Opus state. Deinitialization performs the synchronized
stop before deleting queues, semaphores, codecs, ADC, or I2S resources.

## Telemetry

Emitted counters cover capture completion/timeouts, encode/decode failures,
suppressed DTX frames, source admission rejection, queue overflow/underrun,
PLC/hold/trim behavior, active sources, notification overflow, I2S completion,
and capture-to-transport / receive-to-playback latency.
