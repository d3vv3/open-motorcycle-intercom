import math
import re
import struct
import wave

import pytest

from tools.encode_notifications import IDS, encode, generate, read_wav, resample


def test_format_validation(tmp_path):
    path = tmp_path / "invalid.wav"
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(24000)
        wav.writeframes(struct.pack("<hh", 0, 0))
    with pytest.raises(ValueError, match="mono 24 kHz"):
        read_wav(path)


def test_resampling_and_encoding():
    assert resample([100] * 24) == [100] * 16
    assert encode([0, 11, 41]) == bytes([0x70, 0x07])
    assert len(encode([32767] * 5)) == 3
    high = [round(10000 * math.sin(2 * math.pi * 10000 * i / 24000)) for i in range(2400)]
    low = [round(10000 * math.sin(2 * math.pi * 2000 * i / 24000)) for i in range(2400)]
    assert max(map(abs, resample(high)[16:-16])) < 2000
    assert max(map(abs, resample(low)[16:-16])) > 8000


def test_all_clips_reproducible():
    from pathlib import Path

    directory = Path(__file__).resolve().parents[2] / "audios"
    first = generate(directory)
    assert first == generate(directory)
    assert first.count("static const uint8_t prompt_") == 11
    designators = re.findall(r"^    \[AUDIO_NOTIFY_([A-Z_]+)\] =", first, re.MULTILINE)
    assert designators == list(IDS)
    assert "COUNT" not in designators
