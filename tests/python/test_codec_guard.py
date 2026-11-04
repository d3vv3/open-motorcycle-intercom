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


def _top_level_keys(text, section):
    """Keys of a top-level YAML mapping, or items of a top-level list (stdlib only)."""
    lines = text.splitlines()
    start = lines.index(f"{section}:") + 1
    keys = []
    for line in lines[start:]:
        if line and not line.startswith((" ", "- ")):
            break
        match = re.match(r"^(?:  ([^\s:][^:]*):$|- (\S+)$)", line)
        if match:
            keys.append(match[1] or match[2])
    return keys


def test_tracked_component_lock_has_no_opus_or_esp_sr_closure():
    """dependencies.lock is tracked for releases; it must match the LC3-only manifests."""
    lock = (ROOT / "dependencies.lock").read_text()
    retired = {"78/esp-opus", "espressif/esp-sr", "espressif/esp-dl", "espressif/dl_fft",
               "espressif/esp-dsp", "espressif/cjson", "espressif/esp_new_jpeg"}
    resolved = set(_top_level_keys(lock, "dependencies"))
    assert resolved and not retired & resolved
    direct = set()
    for manifest in sorted((ROOT / "components").glob("*/idf_component.yml")):
        direct |= set(_top_level_keys(manifest.read_text(), "dependencies"))
    assert set(_top_level_keys(lock, "direct_dependencies")) == direct - {"idf"}
