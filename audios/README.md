# Voice prompts

These clips use [Kokoro TTS](https://github.com/nazdridoy/kokoro-tts) version 2.3.2 and its v1.0 model.
The voice is `af_sarah`, Kokoro's default.
Language is US English (`en-us`), and speed is `1.0`.
Files contain mono, 24 kHz, 16-bit PCM WAV audio.

| File | Spoken text |
| --- | --- |
| `mesh_on.wav` | Mesh On |
| `mesh_off.wav` | Mesh Off |
| `phone_pairing.wav` | Phone Pairing |
| `channel_green.wav` | Channel Green |
| `channel_blue.wav` | Channel Blue |
| `channel_red.wav` | Channel Red |
| `you_are_coordinator.wav` | You are coordinator |
| `you_are_participant.wav` | You are participant |
| `startup.wav` | Ready |
| `peer_join.wav` | Peer joined |
| `peer_leave.wav` | Peer left |

## Setup

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and `curl` first.
Run these commands from the repository root in Bash or Zsh.
They store the Python environment and model files outside the repository.
The model downloads total approximately 350 MB.

```sh
KOKORO_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/kokoro-tts"
mkdir -p "$KOKORO_DIR"
uv venv --python 3.12 "$KOKORO_DIR/.venv"
uv pip install --python "$KOKORO_DIR/.venv/bin/python" 'kokoro-tts==2.3.2'

curl --fail --location --retry 3 \
  https://github.com/nazdridoy/kokoro-tts/releases/download/v1.0.0/kokoro-v1.0.onnx \
  --output "$KOKORO_DIR/kokoro-v1.0.onnx"
curl --fail --location --retry 3 \
  https://github.com/nazdridoy/kokoro-tts/releases/download/v1.0.0/voices-v1.0.bin \
  --output "$KOKORO_DIR/voices-v1.0.bin"
```

## Create more audio

Run this block from the repository root.
Define the function again when you open a new shell.
The first argument is the spoken text. The second is the output filename.
Use a new filename to keep existing clips.

```sh
KOKORO_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/kokoro-tts"
mkdir -p audios

create_audio() {
  printf '%s' "$1" | "$KOKORO_DIR/.venv/bin/kokoro-tts" - "audios/$2" \
    --model "$KOKORO_DIR/kokoro-v1.0.onnx" \
    --voices "$KOKORO_DIR/voices-v1.0.bin" \
    --voice af_sarah --lang en-us --speed 1.0 --format wav
}

create_audio 'Battery Low' battery_low.wav
```

To regenerate the supplied clips, use the same function:

```sh
create_audio 'Mesh On' mesh_on.wav
create_audio 'Mesh Off' mesh_off.wav
create_audio 'Phone Pairing' phone_pairing.wav
create_audio 'Channel Green' channel_green.wav
create_audio 'Channel Blue' channel_blue.wav
create_audio 'Channel Red' channel_red.wav
create_audio 'You are coordinator' you_are_coordinator.wav
create_audio 'You are participant' you_are_participant.wav
```

The additional startup and peer notifications use the same command and voice:

```sh
create_audio 'Ready' startup.wav
create_audio 'Peer joined' peer_join.wav
create_audio 'Peer left' peer_leave.wav
```

Firmware builds regenerate compact 16 kHz IMA ADPCM C data from these WAVs via
`tools/encode_notifications.py` (Python standard library only). The firmware never
loads WAVs at runtime. Run `python3 tools/encode_notifications.py --output /tmp/audio_prompts_data.c`
to inspect the generated data independently of the build.
