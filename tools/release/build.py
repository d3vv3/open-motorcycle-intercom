#!/usr/bin/env python3
"""Build paired firmware from a git archive; never flash, tag, or publish."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import struct
import subprocess
import tarfile
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[2]
LEGACY_SHA = "9b3376b11009f66f74fd34a01f65635a117dd591"
ESP_IMAGE = "espressif/idf@sha256:8ac794c57fd4cac246cb8d2ada4002fa26337ac7df683047b5b83743dbedb6b7"
LEGACY_ESP_IMAGE = "espressif/idf@sha256:88a65f412e730fdbe134cd7427dd840eb1a6923479499c9c233699f804a3096b"
NCS_IMAGE = "ghcr.io/nrfconnect/sdk-nrf-toolchain:v3.4.1@sha256:45b97cad97a9967c52d77d1d1a0f7dd8fe027edd17c05c3eda2eeadc23729418"
LEGACY_NCS_DIGEST = "b481467f39ed524a2cb23fed8b9dcdc1e4f60972988b913c386538953c929247"
ON_OPTIONS = ("ESP_LC3_BENCH", "S31_LC3_WIRE", "S31_LC3_SELFTEST", "S31_LC3_SERIALIZE", "S31_LC3_INTERNAL_BUFFERS", "S31_LC3_SINGLE_OWNER")
OFF_OPTIONS = ("S31_LC3_SPLIT_CORES", "S31_LC3_SKIP_RX", "S31_TASK_CPU_PROFILE", "S31_MESH_COEX_PREFER_WIFI", "S31_MESH_PAIR_UNICAST")
FREEZE_ACTIVE = """if west manifest --help | grep -q -- --active-only; then
    west manifest --freeze --active-only > /work/provenance/west-frozen.yml
else
    python3 - <<'PY'
from pathlib import Path
import yaml
from west.manifest import Manifest, ManifestProject

manifest = Manifest.from_topdir()
resolved = manifest.as_dict()
active = {project.name: project for project in manifest.projects
          if not isinstance(project, ManifestProject) and manifest.is_active(project)}
projects = []
for entry in resolved['manifest']['projects']:
    project = active.get(entry['name'])
    if project is not None:
        entry['revision'] = project.sha('manifest-rev')
        projects.append(entry)
resolved['manifest']['projects'] = projects
Path('/work/provenance/west-frozen.yml').write_text(yaml.safe_dump(resolved, sort_keys=False))
PY
fi
"""


def run(args: list[str], **kwargs) -> str:
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, **kwargs).stdout.strip()


def required(path: Path) -> bytes:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"Missing or empty release input: {path}")
    return path.read_bytes()


def flash_path(build: Path, name: str) -> Path:
    relative = PurePosixPath(name)
    if not name or "\\" in name or relative.is_absolute() or any(p in ("..", ".") for p in name.split("/")):
        raise ValueError(f"Unsafe flash path: {name!r}")
    path = build / name
    if not path.resolve().is_relative_to(build.resolve()):
        raise ValueError(f"Flash path escapes build: {name!r}")
    required(path)
    return path


def validate_uf2(data: bytes) -> dict:
    if not data or len(data) % 512:
        raise ValueError("UF2 must contain complete 512-byte blocks")
    count = len(data) // 512
    seen = set()
    ranges = []
    for offset in range(0, len(data), 512):
        magic0, magic1, flags, address, size, number, total, family = struct.unpack_from("<8I", data, offset)
        end_magic = struct.unpack_from("<I", data, offset + 508)[0]
        if (magic0, magic1, end_magic) != (0x0A324655, 0x9E5D5157, 0x0AB16F30):
            raise ValueError("Invalid UF2 magic")
        if flags != 0x2000 or family != 0xADA52840:
            raise ValueError("UF2 must be nRF52840 family flash blocks")
        if total != count or number >= count or number in seen:
            raise ValueError("UF2 block sequence is incomplete or duplicated")
        if not 0 < size <= 476 or address < 0x27000 or address + size > 0xEC000:
            raise ValueError("UF2 payload crosses application boundary [0x27000, 0xec000)")
        seen.add(number)
        ranges.append((address, address + size))
    ranges.sort()
    if any(right[0] < left[1] for left, right in zip(ranges, ranges[1:])):
        raise ValueError("UF2 payload addresses overlap")
    return {"family": "0xADA52840", "blocks": count, "start": hex(ranges[0][0]), "end_exclusive": hex(ranges[-1][1])}


def factory_partition(data: bytes, table_offset: int, capacity: int) -> tuple[tuple[int, int], list[tuple[int, int]]]:
    if len(data) % 32 or len(data) > 0x1000:
        raise ValueError("Invalid ESP partition table length")
    partitions = []
    factories = []
    checksum_seen = False
    for offset in range(0, len(data), 32):
        entry = data[offset:offset + 32]
        if entry == b"\xff" * 32:
            if data[offset:] != b"\xff" * (len(data) - offset):
                raise ValueError("Invalid ESP partition table padding")
            break
        magic = struct.unpack_from("<H", entry)[0]
        if magic == 0xEBEB:
            if checksum_seen or entry[:16] != b"\xeb\xeb" + b"\xff" * 14 or entry[16:] != hashlib.md5(data[:offset]).digest():
                raise ValueError("ESP partition table checksum mismatch")
            checksum_seen = True
            continue
        if magic != 0x50AA or checksum_seen:
            raise ValueError("Invalid ESP partition table entry")
        _, kind, subtype, address, size, _, _ = struct.unpack("<HBBII16sI", entry)
        if size == 0 or address < table_offset + 0x1000 or address + size > capacity:
            raise ValueError("ESP partition outside flash layout")
        partitions.append((address, address + size))
        if kind == 0 and subtype == 0:
            factories.append((address, size))
    partitions.sort()
    if any(right[0] < left[1] for left, right in zip(partitions, partitions[1:])):
        raise ValueError("Overlapping ESP partitions")
    if len(factories) != 1 or factories[0][0] % 0x10000:
        raise ValueError("ESP partition table must have one aligned factory application")
    return factories[0], partitions


def validate_esp(build: Path, profile: str, version: str, sha: str) -> tuple[dict, dict]:
    description = json.loads(required(build / "project_description.json"))
    if description.get("target") != profile or description.get("project_version") != version:
        raise ValueError("ESP target/version metadata mismatch")
    flasher = json.loads(required(build / "flasher_args.json"))
    if flasher.get("extra_esptool_args", {}).get("chip") != profile:
        raise ValueError("ESP flasher target mismatch")
    images = flasher.get("flash_files")
    if not isinstance(images, dict) or not images:
        raise ValueError("ESP flash_files is missing or empty")
    # Use only the generated configuration captured inside isolated build scratch.
    config = dict(re.findall(r"^(CONFIG_[A-Z0-9_]+)=(.*)$", required(build / "sdkconfig").decode(), re.M))
    if json.loads(config.get("CONFIG_IDF_TARGET", '""')) != profile:
        raise ValueError("ESP sdkconfig target mismatch")
    configured_size = json.loads(config.get("CONFIG_ESPTOOLPY_FLASHSIZE", '""'))
    flash_size = flasher.get("flash_settings", {}).get("flash_size")
    match = re.fullmatch(r"([1-9][0-9]*)MB", str(flash_size))
    if not match or flash_size != configured_size:
        raise ValueError("ESP flash capacity missing or mismatched with sdkconfig")
    capacity = int(match[1]) * 1024 * 1024
    try:
        boot_offset = int(config["CONFIG_BOOTLOADER_OFFSET_IN_FLASH"], 0)
        table_offset = int(config["CONFIG_PARTITION_TABLE_OFFSET"], 0)
    except (KeyError, ValueError) as error:
        raise ValueError("ESP sdkconfig flash offsets missing or invalid") from error
    if not 0 <= boot_offset < table_offset or table_offset % 0x1000 or table_offset + 0x1000 > capacity:
        raise ValueError("Invalid ESP sdkconfig flash layout")
    ranges = []
    resolved_images = {}
    for address, name in images.items():
        offset = int(address, 0)
        path = flash_path(build, name)
        if offset < 0 or offset + path.stat().st_size > capacity:
            raise ValueError("ESP flash image exceeds declared flash capacity")
        if offset in resolved_images or name in resolved_images.values():
            raise ValueError("Duplicate ESP flash offset or image")
        resolved_images[offset] = name
        ranges.append((offset, offset + path.stat().st_size))
    ranges.sort()
    if any(right[0] < left[1] for left, right in zip(ranges, ranges[1:])):
        raise ValueError("Overlapping ESP flash images")
    roles = {}
    for role in ("bootloader", "partition-table", "app"):
        entry = flasher.get(role)
        if not isinstance(entry, dict) or not isinstance(entry.get("file"), str) or not isinstance(entry.get("offset"), str):
            raise ValueError(f"Missing or invalid ESP flasher role: {role}")
        address = int(entry["offset"], 0)
        if resolved_images.get(address) != entry["file"]:
            raise ValueError(f"ESP {role} role disagrees with flash_files")
        roles[role] = (address, flash_path(build, entry["file"]))
    if roles["bootloader"][0] != boot_offset or roles["bootloader"][1].stat().st_size > table_offset - boot_offset:
        raise ValueError("ESP bootloader offset/size disagrees with sdkconfig")
    if roles["partition-table"][0] != table_offset:
        raise ValueError("ESP partition-table offset disagrees with sdkconfig")
    (app_offset, app_size), partitions = factory_partition(required(roles["partition-table"][1]), table_offset, capacity)
    app_name = roles["app"][1].relative_to(build).as_posix()
    if app_name != description.get("app_bin", "omi.bin"):
        raise ValueError("ESP application role disagrees with project metadata")
    if roles["app"][0] != app_offset or roles["app"][1].stat().st_size > app_size:
        raise ValueError("ESP application offset/size exceeds factory partition")
    role_offsets = {entry[0] for entry in roles.values()}
    for start, end in ranges:
        if start not in role_offsets and not any(start == address and end <= limit for address, limit in partitions):
            raise ValueError("Additional ESP flash image is not at a partition boundary")
    app = required(flash_path(build, app_name))
    if len(app) < 80 or struct.unpack_from("<I", app, 32)[0] != 0xABCD5432:
        raise ValueError("ESP app descriptor missing")
    if app[48:80].split(b"\0", 1)[0].decode() != version:
        raise ValueError("ESP application binary version mismatch")
    if profile == "esp32s31":
        validate_identity(required(build / "omi.elf"), version, sha)
        validate_identity(app, version, sha)
    return description, flasher


def validate_identity(data: bytes, version: str, sha: str) -> None:
    if version.encode() + b"\0" not in data or sha.encode() + b"\0" not in data:
        raise ValueError("Firmware version/source SHA mismatch")


def uf2_payload(data: bytes) -> bytes:
    """Reassemble contiguous flash regions without UF2 transport headers."""
    blocks = []
    for offset in range(0, len(data), 512):
        address, size = struct.unpack_from("<2I", data, offset + 12)
        blocks.append((address, data[offset + 32:offset + 32 + size]))
    result = bytearray()
    end = None
    for address, payload in sorted(blocks):
        if end is not None and address != end:
            result.extend(b"\0")
        result.extend(payload)
        end = address + len(payload)
    return bytes(result)


def protocols(source: Path) -> dict:
    result = {}
    for name in ("bridge", "mesh"):
        text = required(source / "shared" / f"{name}_protocol_defs.h").decode()
        match = re.search(rf"^#define\s+{name.upper()}_PROTOCOL_VERSION\s+(0x[0-9a-fA-F]+|[0-9]+)\b", text, re.M)
        if not match:
            raise ValueError(f"Missing {name} protocol version")
        result[name] = int(match[1], 0)
    return result


def write_zip(path: Path, entries: dict[str, bytes]) -> None:
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(entries.items()):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)


def package(source: Path, esp: Path, nrf: Path, provenance: Path, output: Path,
            version: str, sha: str, profile: str, images: dict) -> list[Path]:
    description, flasher = validate_esp(esp, profile, version, sha)
    uf2 = required(nrf / "zephyr.uf2")
    uf2_info = validate_uf2(uf2)
    nrf_elf = required(nrf / "zephyr.elf")
    if profile == "esp32s31":
        validate_identity(nrf_elf, version, sha)
        validate_identity(uf2_payload(uf2), version, sha)
    else:
        identity = f"*** Booting Zephyr OS build {version}+{sha} ***\n\0".encode()
        if identity not in nrf_elf or identity not in uf2_payload(uf2):
            raise ValueError("Legacy Zephyr build version/source SHA mismatch")
    wire = protocols(source)
    expected = {"bridge": 6, "mesh": 5} if profile == "esp32s31" else {"bridge": 2, "mesh": 2}
    if wire != expected:
        raise ValueError(f"Unexpected source protocol versions: {wire}")
    files = {name: required(flash_path(esp, name)) for name in flasher["flash_files"].values()}
    files["flasher_args.json"] = required(esp / "flasher_args.json")
    pairs = " ".join(f"{address} {shlex.quote(name)}" for address, name in sorted(flasher["flash_files"].items(), key=lambda item: int(item[0], 0)))
    settings = flasher.get("flash_settings", {})
    options = " ".join(f"--{key} {shlex.quote(str(settings[key]))}" for key in ("flash_mode", "flash_freq", "flash_size") if key in settings)
    extra = flasher["extra_esptool_args"]
    reset = " ".join(f"--{key} {shlex.quote(str(extra[key]))}" for key in ("before", "after") if key in extra)
    readme = f"# OMI {version} ({profile})\n\nFrom this extracted directory, using the matching ESP-IDF tools:\n\n```sh\npython -m esptool --chip {profile} --port PORT {reset} write_flash {options} {pairs}\n```\n\nNo erase_flash is required.\n"
    if profile == "esp32s31":
        openocd = "; ".join(f"program_esp {shlex.quote(name)} {address} verify" for address, name in sorted(flasher["flash_files"].items(), key=lambda item: int(item[0], 0)))
        readme += f"\nESP32-S31 built-in USB JTAG (matching preview IDF OpenOCD):\n\n```sh\nopenocd -f board/esp32s31-builtin.cfg -c '{openocd}; reset run; shutdown'\n```\n"
    else:
        readme += "\nExperimental legacy ESP32-S3: no Classic Bluetooth and no ESP32-S31 LC3 interoperability.\n"
    readme += "\nCopy the paired application UF2 to the XIAO bootloader drive. Do not erase or replace the nRF bootloader.\n"
    files["README.md"] = readme.encode()
    debug = {"esp/omi.elf": required(esp / "omi.elf"), "nrf/zephyr.elf": nrf_elf,
             "provenance/project_description.json": required(esp / "project_description.json")}
    for path in sorted(provenance.rglob("*")):
        if path.is_file():
            debug["provenance/" + path.relative_to(provenance).as_posix()] = required(path)
    for name in ("west-frozen.yml", "west-resolved.txt", "west-input.yml", "release-tools-source-sha.txt", "idf-commit.txt", "idf-version.txt", "sdkconfig", "dependencies.lock"):
        required(provenance / name)
    driver_sha = required(provenance / "release-tools-source-sha.txt").decode().strip()
    if not re.fullmatch(r"[0-9a-f]{40}", driver_sha):
        raise ValueError("Invalid release tools source SHA provenance")
    idf_version = required(provenance / "idf-version.txt").decode().strip()
    if not idf_version:
        raise ValueError("Empty ESP-IDF version provenance")
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("Output directory must be empty")
    stem = f"omi-{version}"
    write_zip(output / f"{stem}-{profile}.zip", files)
    (output / f"{stem}-nrf52840.uf2").write_bytes(uf2)
    write_zip(output / f"{stem}-debug.zip", debug)
    manifest = {"schema_version": 1, "version": version, "source_sha": sha, "profile": profile,
                "release_tools_source_sha": driver_sha,
                "ncs_input_manifest": {"archive_path": "provenance/west-input.yml", "sha256": hashlib.sha256(required(provenance / "west-input.yml")).hexdigest()},
                "protocol_versions": wire, "builder_images": images, "uf2": uf2_info,
                "experimental": profile == "esp32s3", "classic_bluetooth": profile == "esp32s31",
                "s31_lc3_interoperable": profile == "esp32s31", "source_patches": [],
                "reproducibility": "Archived source and pinned builders; resolved dependencies captured, not a claim of historical bit reproducibility",
                "provenance_archive": f"{stem}-debug.zip", "idf_version": idf_version,
                "assets": {p.name: {"sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "size": p.stat().st_size} for p in sorted(output.iterdir())}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    assets = sorted(output.iterdir())
    (output / "SHA256SUMS").write_text("".join(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}\n" for p in assets))
    return sorted(output.iterdir())


def snapshot(repo: Path, sha: str, scratch: Path) -> Path:
    archive = scratch / "source.tar"
    subprocess.run(["git", "-C", str(repo), "archive", "--format=tar", "-o", str(archive), sha], check=True)
    source = scratch / "source"
    source.mkdir()
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            path = PurePosixPath(member.name)
            if path.is_absolute() or ".." in path.parts or not (member.isfile() or member.isdir()):
                raise ValueError(f"Unsafe git archive entry: {member.name}")
        tar.extractall(source, filter="data")
    return source


def pinned(image: str) -> str:
    if not re.fullmatch(r"[a-zA-Z0-9./:_-]+@sha256:[0-9a-f]{64}", image):
        raise ValueError("Builder image must be pinned with @sha256:<64 hex digits>")
    return image


def docker(image: str, scratch: Path, script: str, env: dict | None = None) -> None:
    args = ["docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}",
            "--volume", f"{scratch}:/work", "--workdir", "/work", "--env", "HOME=/work/home",
            "--entrypoint", "/bin/bash"]
    for key, value in (env or {}).items():
        args += ["--env", f"{key}={value}"]
    # Disable image startup hooks before bash runs; initialize SDKs explicitly below.
    args += ["--env", "BASH_ENV="]
    # NCS images have ENTRYPOINT ["/bin/bash", "-c"], which swallows a nested bash command.
    subprocess.run(args + [pinned(image), "-euc", script], check=True)


def build(scratch: Path, version: str, sha: str, profile: str, esp_image: str, ncs_image: str) -> None:
    legacy = profile == "esp32s3"
    for name in ("home", "provenance"):
        (scratch / name).mkdir()
    sdk = "v2.7.0" if legacy else "v3.4.1"
    # Release-driver inputs are intentionally separate from the archived firmware source.
    sdk_manifest = required(ROOT / "tools/release" / f"ncs-{sdk}.yml")
    manifest_dir = scratch / "ncs/release-manifest"
    manifest_dir.mkdir(parents=True)
    (manifest_dir / "west.yml").write_bytes(sdk_manifest)
    (scratch / "provenance/west-input.yml").write_bytes(sdk_manifest)
    driver_sha = run(["git", "-C", str(ROOT), "rev-parse", "HEAD"])
    (scratch / "provenance/release-tools-source-sha.txt").write_text(driver_sha + "\n")
    q = shlex.quote
    definitions = [f"-DPROJECT_VER={version}", f"-DIDF_TARGET={profile}", "-DSDKCONFIG=/work/esp-sdkconfig"]
    if not legacy:
        definitions += [f"-DOMI_FIRMWARE_VERSION={version}", f"-DOMI_GIT_SHA={sha}", "-DSDKCONFIG_DEFAULTS=/work/source/sdkconfig.defaults;/work/source/sdkconfig.release.esp32s31.defaults"]
        definitions += [f"-D{key}=ON" for key in ON_OPTIONS] + [f"-D{key}=OFF" for key in OFF_OPTIONS]
    else:
        definitions += ["-DSDKCONFIG_DEFAULTS=/work/source/sdkconfig.defaults"]
    command = " ".join(q(arg) for arg in definitions)
    # Explicitly initialize IDF because docker() bypasses the image's entrypoint.
    script = '. "${IDF_PATH:-/opt/esp/idf}/export.sh"\n'
    script += 'idf.py --version > /work/provenance/idf-version.txt\ngit -C "$IDF_PATH" rev-parse HEAD > /work/provenance/idf-commit.txt\n'
    if legacy:
        script += "grep -Eq '^ESP-IDF v5\\.5\\.2($|[^0-9])' /work/provenance/idf-version.txt\n"
    script += f"idf.py {'--preview ' if not legacy else ''}-C /work/source -B /work/esp {command} build\n"
    script += "cp /work/esp-sdkconfig /work/provenance/sdkconfig\ncp /work/esp-sdkconfig /work/esp/sdkconfig\ncp /work/source/dependencies.lock /work/provenance/dependencies.lock\ncp /work/esp/CMakeCache.txt /work/provenance/esp-CMakeCache.txt\n"
    (scratch / "provenance/esp-build.sh").write_text(script)
    docker(esp_image, scratch, script, {"OMI_ESP_LC3_BENCH": "0" if legacy else "1"})
    nrf_defs = "-Dnrf_mesh_CONFIG_BUILD_OUTPUT_UF2=y"
    if legacy:
        nrf_defs += " -Dnrf_mesh_CONFIG_NCS_BOOT_BANNER=n -Dnrf_mesh_CONFIG_BOOT_BANNER=y"
    else:
        nrf_defs += f" -Dnrf_mesh_OMI_FIRMWARE_VERSION={q(version)} -Dnrf_mesh_OMI_GIT_SHA={q(sha)}"
    script = 'export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"\n. /opt/toolchain-env.sh\n'
    script += "west init -l /work/ncs/release-manifest\n"
    script += "cd /work/ncs\nwest update --narrow -o=--depth=1\nwest zephyr-export\n"
    script += FREEZE_ACTIVE
    # west list defaults to active projects; --all would include uncloned private repos.
    script += "west list -f '{name}' > /work/provenance/west-active-projects.txt\n"
    # The copied wrapper is not a git repository, so do not ask west for its SHA.
    script += "while IFS= read -r project; do\n    if [ \"$project\" != manifest ]; then\n        west list -f '{name} {revision} {sha} {url}' \"$project\"\n    fi\ndone < /work/provenance/west-active-projects.txt > /work/provenance/west-resolved.txt\n"
    script += f"west build --sysbuild {'--cmake-only ' if legacy else ''}-b xiao_ble/nrf52840 /work/source/nrf_mesh -d /work/nrf -- {nrf_defs}\n"
    if legacy:
        script += f"cmake -S /work/source/nrf_mesh -B /work/nrf/nrf_mesh -DBUILD_VERSION:STRING={q(version + '+' + sha)}\ncmake --build /work/nrf\n"
    script += "cp /work/nrf/nrf_mesh/zephyr/.config /work/provenance/nrf.config\ncp /work/nrf/nrf_mesh/CMakeCache.txt /work/provenance/nrf-CMakeCache.txt\n"
    (scratch / "provenance/nrf-build.sh").write_text(script)
    docker(ncs_image, scratch, script)
    for name in ("sdkconfig.defaults", "sdkconfig.release.esp32s31.defaults"):
        path = scratch / "source" / name
        if path.is_file():
            shutil.copyfile(path, scratch / "provenance" / name)


@contextmanager
def scratch_directory(work_dir: Path | None):
    if work_dir is None:
        with tempfile.TemporaryDirectory(prefix="omi-release-") as temporary:
            yield Path(temporary)
    else:
        work_dir = work_dir.resolve()
        if work_dir.exists() and (not work_dir.is_dir() or any(work_dir.iterdir())):
            raise ValueError("Work directory must be absent or empty; existing scratch is never reused automatically")
        work_dir.mkdir(parents=True, exist_ok=True)
        yield work_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="Firmware SemVer, without leading v")
    parser.add_argument("--source-sha", required=True, help="Full git commit SHA")
    parser.add_argument("--profile", choices=("esp32s31", "esp32s3"), default="esp32s31")
    parser.add_argument("--output-dir", type=Path, default=Path("dist/release"))
    parser.add_argument("--esp-image", help="Override the profile's digest-pinned ESP-IDF image")
    parser.add_argument("--work-dir", type=Path, help="Empty scratch directory to retain after success or failure; no automatic reuse")
    args = parser.parse_args(argv)
    try:
        if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.-]+)?", args.version) or len(args.version.encode()) > 31:
            raise ValueError("Version must be a SemVer fitting ESP's 31-byte metadata field")
        if not re.fullmatch(r"[0-9a-f]{40}", args.source_sha):
            raise ValueError("source-sha must be a full lowercase commit SHA")
        resolved = run(["git", "-C", str(ROOT), "rev-parse", f"{args.source_sha}^{{commit}}"])
        if resolved != args.source_sha:
            raise ValueError("Source commit mismatch")
        legacy = args.profile == "esp32s3"
        if legacy and (args.source_sha != LEGACY_SHA or args.version != "0.1.0"):
            raise ValueError("Legacy requires version 0.1.0 and the historical SHA")
        esp_image = pinned(args.esp_image or (LEGACY_ESP_IMAGE if legacy else ESP_IMAGE))
        ncs_image = NCS_IMAGE
        if legacy:
            workflow = run(["git", "-C", str(ROOT), "show", f"{args.source_sha}:.github/workflows/build.yml"])
            match = re.search(r"image:\s*(\S+@sha256:" + LEGACY_NCS_DIGEST + r")", workflow)
            if not match:
                raise ValueError("Historical NCS image pin not found")
            ncs_image = pinned(match[1])
        output = args.output_dir.resolve()
        if output.exists() and (not output.is_dir() or any(output.iterdir())):
            raise ValueError("Output directory must be absent or empty")
        if args.work_dir:
            work_dir = args.work_dir.resolve()
            if work_dir.is_relative_to(output) or output.is_relative_to(work_dir):
                raise ValueError("Work and output directories must be separate, not nested")
        with scratch_directory(args.work_dir) as scratch:
            source = snapshot(ROOT, args.source_sha, scratch)
            build(scratch, args.version, args.source_sha, args.profile, esp_image, ncs_image)
            staging = scratch / "assets"
            package(source, scratch / "esp", scratch / "nrf/nrf_mesh/zephyr", scratch / "provenance", staging,
                    args.version, args.source_sha, args.profile, {"esp": esp_image, "nrf": ncs_image})
            output.parent.mkdir(parents=True, exist_ok=True)
            if output.exists():
                output.rmdir()
            shutil.copytree(staging, output)
        print(f"Validated release assets: {output}")
        return 0
    except (ValueError, OSError, subprocess.CalledProcessError, tarfile.TarError) as error:
        retained = f"\nScratch retained when created: {args.work_dir.resolve()}" if args.work_dir else ""
        parser.exit(1, f"Release build failed: {error}{retained}\n")


if __name__ == "__main__":
    raise SystemExit(main())
