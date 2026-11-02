"""Opus and the retired LC3 build variants must not come back.

LC3 is the only audio codec; the mesh wire IDs MESH_AUDIO_CODEC_OPUS,
MESH_CAP_OPUS and MESH_MAX_OPUS_BYTES in shared/ stay because they are wire ABI.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
FORBIDDEN = re.compile(r"opus_encode|opus_decode|esp-opus|S31_LC3_WIRE|S31_LC3_SELFTEST")


def _sources():
    for top in ("components", "main", "tools"):
        for path in sorted((ROOT / top).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts:
                yield path


def test_no_opus_or_retired_lc3_variants_in_firmware_or_tools():
    hits = []
    for path in _sources():
        try:
            text = path.read_text()
        except UnicodeDecodeError:
            continue
        for number, line in enumerate(text.splitlines(), 1):
            if FORBIDDEN.search(line):
                hits.append(f"{path.relative_to(ROOT)}:{number}: {line.strip()}")
    assert not hits, "\n".join(hits)


def test_shared_wire_codec_ids_are_kept():
    defs = (ROOT / "shared/mesh_protocol_defs.h").read_text()
    for name in ("MESH_AUDIO_CODEC_OPUS", "MESH_CAP_OPUS", "MESH_MAX_OPUS_BYTES", "MESH_AUDIO_CODEC_LC3"):
        assert re.search(rf"#define {name}\b", defs), name
