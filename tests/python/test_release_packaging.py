"""Release packaging safety checks using synthetic firmware, never Docker."""

import hashlib
import json
import os
import struct
import subprocess
import zipfile

import pytest

from tools.release import build


VERSION = "0.2.0-alpha.1"
SHA = "a" * 40


def uf2(address=0x27000, size=256, family=0xADA52840, number=0, total=1):
    block = bytearray(512)
    struct.pack_into("<8I", block, 0, 0x0A324655, 0x9E5D5157, 0x2000, address, size, number, total, family)
    block[32:32 + len(VERSION) + 1] = VERSION.encode() + b"\0"
    block[96:137] = SHA.encode() + b"\0"
    struct.pack_into("<I", block, 508, 0x0AB16F30)
    return bytes(block)


def partition_table(factory_offset=0x10000, factory_size=0x300000):
    entries = b"".join(struct.pack("<HBBII16sI", 0x50AA, kind, subtype, address, size, name.encode(), 0)
                       for kind, subtype, address, size, name in (
                           (1, 2, 0x9000, 0x6000, "nvs"),
                           (1, 1, 0xF000, 0x1000, "phy_init"),
                           (0, 0, factory_offset, factory_size, "factory"),
                           (1, 0x82, 0x310000, 0xF0000, "storage")))
    checksum = b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(entries).digest()
    return (entries + checksum).ljust(0xC00, b"\xff")


@pytest.fixture
def inputs(tmp_path):
    source, esp, nrf, provenance = (tmp_path / name for name in ("source", "esp", "nrf", "provenance"))
    for directory in (source / "shared", esp / "bootloader", esp / "partition_table", esp / "extra", nrf, provenance):
        directory.mkdir(parents=True)
    for name, version in (("bridge", 6), ("mesh", 5)):
        (source / "shared" / f"{name}_protocol_defs.h").write_text(f"#define {name.upper()}_PROTOCOL_VERSION {version}\n")
    app = bytearray(256)
    struct.pack_into("<I", app, 32, 0xABCD5432)
    app[48:48 + len(VERSION)] = VERSION.encode()
    app[128:169] = SHA.encode() + b"\0"
    (esp / "omi.bin").write_bytes(app)
    for name in ("bootloader/bootloader.bin", "extra/phy.bin"):
        (esp / name).write_bytes(b"image")
    (esp / "partition_table/partition-table.bin").write_bytes(partition_table())
    (esp / "sdkconfig").write_text('CONFIG_IDF_TARGET="esp32s31"\nCONFIG_BOOTLOADER_OFFSET_IN_FLASH=0x2000\nCONFIG_PARTITION_TABLE_OFFSET=0x8000\nCONFIG_ESPTOOLPY_FLASHSIZE="16MB"\n')
    elf = b"ELF\0" + VERSION.encode() + b"\0" + SHA.encode() + b"\0"
    (esp / "omi.elf").write_bytes(elf)
    (nrf / "zephyr.elf").write_bytes(elf)
    (nrf / "zephyr.uf2").write_bytes(uf2())
    (esp / "project_description.json").write_text(json.dumps({"target": "esp32s31", "project_version": VERSION, "app_bin": "omi.bin"}))
    (esp / "flasher_args.json").write_text(json.dumps({"extra_esptool_args": {"chip": "esp32s31", "before": "default_reset", "after": "hard_reset"}, "flash_settings": {"flash_mode": "dio", "flash_freq": "80m", "flash_size": "16MB"}, "flash_files": {"0x2000": "bootloader/bootloader.bin", "0x8000": "partition_table/partition-table.bin", "0xf000": "extra/phy.bin", "0x10000": "omi.bin"}, "bootloader": {"offset": "0x2000", "file": "bootloader/bootloader.bin"}, "partition-table": {"offset": "0x8000", "file": "partition_table/partition-table.bin"}, "app": {"offset": "0x10000", "file": "omi.bin"}}))
    for name in ("west-frozen.yml", "west-resolved.txt", "west-input.yml", "idf-commit.txt", "idf-version.txt", "sdkconfig", "dependencies.lock"):
        (provenance / name).write_text("captured\n")
    (provenance / "release-tools-source-sha.txt").write_text("c" * 40 + "\n")
    (provenance / "idf-version.txt").write_text("ESP-IDF v6.1-dev\n")
    return source, esp, nrf, provenance


def package(inputs, output):
    return build.package(*inputs, output, VERSION, SHA, "esp32s31", {"esp": build.ESP_IMAGE, "nrf": build.NCS_IMAGE})


@pytest.fixture
def legacy_inputs(inputs):
    source, esp, nrf, provenance = inputs
    for name in ("bridge", "mesh"):
        (source / "shared" / f"{name}_protocol_defs.h").write_text(f"#define {name.upper()}_PROTOCOL_VERSION 2\n")
    description_path = esp / "project_description.json"
    description = json.loads(description_path.read_text())
    description.update(target="esp32s3", project_version="0.1.0")
    description_path.write_text(json.dumps(description))
    (provenance / "idf-version.txt").write_text("ESP-IDF v5.5.2\n")
    app = bytearray((esp / "omi.bin").read_bytes())
    app[48:80] = b"0.1.0".ljust(32, b"\0")
    (esp / "omi.bin").write_bytes(app)
    (esp / "sdkconfig").write_text('CONFIG_IDF_TARGET="esp32s3"\nCONFIG_BOOTLOADER_OFFSET_IN_FLASH=0x0\nCONFIG_PARTITION_TABLE_OFFSET=0x8000\nCONFIG_ESPTOOLPY_FLASHSIZE="8MB"\n')
    path = esp / "flasher_args.json"
    flasher = json.loads(path.read_text())
    flasher["extra_esptool_args"]["chip"] = "esp32s3"
    flasher["flash_settings"]["flash_size"] = "8MB"
    flasher["bootloader"]["offset"] = "0x0"
    flasher["flash_files"]["0x0"] = flasher["flash_files"].pop("0x2000")
    path.write_text(json.dumps(flasher))
    identity = f"*** Booting Zephyr OS build 0.1.0+{SHA} ***\n\0".encode()
    (nrf / "zephyr.elf").write_bytes(b"ELF\0" + identity)
    block = bytearray(uf2())
    block[32:288] = identity.ljust(256, b"\0")
    (nrf / "zephyr.uf2").write_bytes(block)
    return inputs


def test_legacy_package_preserves_identity_and_experimental_status(legacy_inputs, tmp_path):
    output = tmp_path / "legacy"
    build.package(*legacy_inputs, output, "0.1.0", SHA, "esp32s3", {"esp": build.LEGACY_ESP_IMAGE})
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["version"] == "0.1.0"
    assert manifest["source_sha"] == SHA
    assert manifest["idf_version"] == "ESP-IDF v5.5.2"
    assert manifest["experimental"] is True
    assert manifest["classic_bluetooth"] is False
    assert manifest["s31_lc3_interoperable"] is False


def test_manifest_idf_version_uses_stripped_provenance_not_project_description(inputs, tmp_path):
    path = inputs[1] / "project_description.json"
    description = json.loads(path.read_text())
    description["idf_ver"] = "stale-description-value"
    path.write_text(json.dumps(description))
    (inputs[3] / "idf-version.txt").write_text("  ESP-IDF v6.1-dev\n\n")
    output = tmp_path / "release"
    package(inputs, output)
    assert json.loads((output / "manifest.json").read_text())["idf_version"] == "ESP-IDF v6.1-dev"


def test_reject_whitespace_only_idf_version_provenance(inputs, tmp_path):
    (inputs[3] / "idf-version.txt").write_text(" \n\n")
    with pytest.raises(ValueError, match="Empty ESP-IDF version provenance"):
        package(inputs, tmp_path / "release")
    assert not (tmp_path / "release").exists()


@pytest.mark.parametrize("name", ["zephyr.elf", "zephyr.uf2"])
def test_legacy_rejects_identity_mismatch_in_elf_and_uf2(legacy_inputs, tmp_path, name):
    path = legacy_inputs[2] / name
    path.write_bytes(path.read_bytes().replace(SHA.encode(), b"b" * 40))
    with pytest.raises(ValueError, match="Legacy Zephyr build version/source SHA mismatch"):
        build.package(*legacy_inputs, tmp_path / "legacy", "0.1.0", SHA, "esp32s3", {})


@pytest.mark.parametrize("name", ["zephyr.elf", "zephyr.uf2"])
@pytest.mark.parametrize("replacement", [
    f"0.1.0+{SHA}\0".encode(),
    f"*** Booting Zephyr OS build 0.1.0+{SHA}x ***\n\0".encode(),
    f"*** Booting Zephyr OS build 0.1.0+{SHA} ***\0".encode(),
])
def test_legacy_requires_complete_standard_banner(legacy_inputs, tmp_path, name, replacement):
    banner = f"*** Booting Zephyr OS build 0.1.0+{SHA} ***\n\0".encode()
    path = legacy_inputs[2] / name
    # Keep the UF2 block length unchanged while replacing its embedded string.
    if name.endswith(".uf2"):
        data = bytearray(path.read_bytes())
        data[32:288] = replacement.ljust(256, b"\0")
        path.write_bytes(data)
    else:
        path.write_bytes(path.read_bytes().replace(banner, replacement))
    with pytest.raises(ValueError, match="Legacy Zephyr build version/source SHA mismatch"):
        build.package(*legacy_inputs, tmp_path / "legacy", "0.1.0", SHA, "esp32s3", {})


@pytest.mark.parametrize("role", ["bootloader", "partition-table", "app"])
@pytest.mark.parametrize("omission", ["role", "flash_files", "both"])
def test_reject_omitted_required_esp_image(inputs, tmp_path, role, omission):
    path = inputs[1] / "flasher_args.json"
    flasher = json.loads(path.read_text())
    address = flasher[role]["offset"]
    if omission in ("role", "both"):
        del flasher[role]
    if omission in ("flash_files", "both"):
        del flasher["flash_files"][address]
    path.write_text(json.dumps(flasher))
    with pytest.raises(ValueError, match="flasher role|disagrees with flash_files"):
        package(inputs, tmp_path / "release")


@pytest.mark.parametrize("role,address", [("bootloader", "0x0"), ("partition-table", "0x7000"), ("app", "0x20000")])
def test_reject_wrong_offsets_even_when_role_and_flash_files_agree(inputs, tmp_path, role, address):
    path = inputs[1] / "flasher_args.json"
    flasher = json.loads(path.read_text())
    original = flasher[role]["offset"]
    flasher[role]["offset"] = address
    flasher["flash_files"][address] = flasher["flash_files"].pop(original)
    path.write_text(json.dumps(flasher))
    with pytest.raises(ValueError, match="offset/size|offset disagrees"):
        package(inputs, tmp_path / "release")


@pytest.mark.parametrize("address,message", [("0x1000000", "flash capacity"), ("0xf100", "partition boundary")])
def test_reject_offchip_or_arbitrary_extra_image_offset(inputs, tmp_path, address, message):
    path = inputs[1] / "flasher_args.json"
    flasher = json.loads(path.read_text())
    flasher["flash_files"][address] = flasher["flash_files"].pop("0xf000")
    path.write_text(json.dumps(flasher))
    with pytest.raises(ValueError, match=message):
        package(inputs, tmp_path / "release")


def test_reject_extra_image_crossing_flash_capacity(inputs, tmp_path):
    path = inputs[1] / "flasher_args.json"
    flasher = json.loads(path.read_text())
    flasher["flash_files"]["0xffffff"] = flasher["flash_files"].pop("0xf000")
    path.write_text(json.dumps(flasher))
    with pytest.raises(ValueError, match="flash capacity"):
        package(inputs, tmp_path / "release")


def test_reject_mismatched_flash_capacity(inputs, tmp_path):
    path = inputs[1] / "flasher_args.json"
    flasher = json.loads(path.read_text())
    flasher["flash_settings"]["flash_size"] = "8MB"
    path.write_text(json.dumps(flasher))
    with pytest.raises(ValueError, match="capacity missing or mismatched"):
        package(inputs, tmp_path / "release")


def test_reject_application_larger_than_factory_partition(inputs, tmp_path):
    (inputs[1] / "partition_table/partition-table.bin").write_bytes(partition_table(factory_size=128))
    with pytest.raises(ValueError, match="exceeds factory partition"):
        package(inputs, tmp_path / "release")


@pytest.mark.parametrize("factory_offset,factory_size", [(0x20000, 0x2F0000), (0x10000, 0x1000000)])
def test_factory_partition_offsets_and_capacity_are_authoritative(inputs, tmp_path, factory_offset, factory_size):
    (inputs[1] / "partition_table/partition-table.bin").write_bytes(partition_table(factory_offset, factory_size))
    with pytest.raises(ValueError, match="factory partition|outside flash layout"):
        package(inputs, tmp_path / "release")


def test_reject_corrupt_partition_checksum(inputs, tmp_path):
    path = inputs[1] / "partition_table/partition-table.bin"
    data = bytearray(path.read_bytes())
    data[12] ^= 1
    path.write_bytes(data)
    with pytest.raises(ValueError, match="checksum mismatch"):
        package(inputs, tmp_path / "release")


def test_package_preserves_all_flash_paths_offsets_provenance_and_hashes(inputs, tmp_path):
    output = tmp_path / "release"
    assets = package(inputs, output)
    assert len(assets) == 5
    with zipfile.ZipFile(output / f"omi-{VERSION}-esp32s31.zip") as archive:
        for name in ("bootloader/bootloader.bin", "partition_table/partition-table.bin", "extra/phy.bin", "omi.bin", "flasher_args.json"):
            assert archive.read(name) == (inputs[1] / name).read_bytes()
        readme = archive.read("README.md").decode()
        assert "0xf000 extra/phy.bin" in readme
        assert "program_esp omi.bin 0x10000 verify" in readme
        assert "erase_flash is required" in readme
    with zipfile.ZipFile(output / f"omi-{VERSION}-debug.zip") as archive:
        assert archive.read("nrf/zephyr.elf")
        assert archive.read("esp/omi.elf")
        assert archive.read("provenance/west-frozen.yml")
        assert archive.read("provenance/dependencies.lock")
        assert archive.read("provenance/west-input.yml") == (inputs[3] / "west-input.yml").read_bytes()
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["source_sha"] == SHA
    assert manifest["release_tools_source_sha"] == "c" * 40
    assert manifest["ncs_input_manifest"]["sha256"] == hashlib.sha256((inputs[3] / "west-input.yml").read_bytes()).hexdigest()
    assert manifest["protocol_versions"] == {"bridge": 6, "mesh": 5}
    for name, metadata in manifest["assets"].items():
        assert metadata["sha256"] == hashlib.sha256((output / name).read_bytes()).hexdigest()
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ")
        assert digest == hashlib.sha256((output / name).read_bytes()).hexdigest()


@pytest.mark.parametrize("empty", [False, True])
def test_reject_missing_or_empty_flash_image(inputs, tmp_path, empty):
    path = inputs[1] / "extra/phy.bin"
    if empty:
        path.write_bytes(b"")
    else:
        path.unlink()
    with pytest.raises(ValueError, match="Missing or empty"):
        package(inputs, tmp_path / "release")
    assert not (tmp_path / "release").exists()


@pytest.mark.parametrize("name", ["../secret.bin", "/secret.bin", "extra/../../secret.bin", "extra\\secret.bin", "./omi.bin"])
def test_reject_traversal(inputs, name):
    with pytest.raises(ValueError, match="Unsafe flash path"):
        build.flash_path(inputs[1], name)


def test_reject_symlink_escape(inputs, tmp_path):
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"secret")
    (inputs[1] / "escape.bin").symlink_to(outside)
    with pytest.raises(ValueError, match="escapes build"):
        build.flash_path(inputs[1], "escape.bin")


@pytest.mark.parametrize("kwargs", [{"address": 0x26FFF}, {"address": 0xEBF80}, {"address": 0xEC000}, {"size": 0}, {"size": 477}, {"family": 0}, {"total": 2}, {"number": 1}])
def test_uf2_rejects_unsafe_or_incomplete_blocks(kwargs):
    with pytest.raises(ValueError):
        build.validate_uf2(uf2(**kwargs))


def test_uf2_boundaries_and_sequence():
    assert build.validate_uf2(uf2(address=0xEBF00))["end_exclusive"] == "0xec000"
    data = uf2(number=1, total=2, address=0x27100) + uf2(number=0, total=2)
    assert build.validate_uf2(data)["blocks"] == 2
    with pytest.raises(ValueError, match="sequence"):
        build.validate_uf2(uf2(total=2) * 2)
    with pytest.raises(ValueError, match="overlap"):
        build.validate_uf2(uf2(total=2) + uf2(number=1, total=2))


def test_uf2_identity_spanning_transport_blocks():
    first = bytearray(uf2(total=2))
    second = bytearray(uf2(number=1, total=2, address=0x27100))
    identity = SHA.encode() + b"\0"
    first[96:137] = b"\0" * 41
    second[96:137] = b"\0" * 41
    first[32 + 240:32 + 256] = identity[:16]
    second[32:32 + 25] = identity[16:]
    data = bytes(second + first)
    build.validate_uf2(data)
    build.validate_identity(build.uf2_payload(data), VERSION, SHA)


@pytest.mark.parametrize("offset", [0, 4, 508])
def test_bad_uf2_magic(offset):
    data = bytearray(uf2())
    struct.pack_into("<I", data, offset, 0)
    with pytest.raises(ValueError, match="magic"):
        build.validate_uf2(bytes(data))


@pytest.mark.parametrize("data", [b"", uf2()[:-1]])
def test_truncated_uf2(data):
    with pytest.raises(ValueError, match="complete"):
        build.validate_uf2(data)


@pytest.mark.parametrize("field,value", [("target", "esp32s3"), ("project_version", "wrong")])
def test_wrong_esp_metadata(inputs, tmp_path, field, value):
    path = inputs[1] / "project_description.json"
    description = json.loads(path.read_text())
    description[field] = value
    path.write_text(json.dumps(description))
    with pytest.raises(ValueError, match="metadata mismatch"):
        package(inputs, tmp_path / "release")


def test_wrong_binary_version(inputs, tmp_path):
    path = inputs[1] / "omi.bin"
    data = bytearray(path.read_bytes())
    data[48] = ord("9")
    path.write_bytes(data)
    with pytest.raises(ValueError, match="binary version"):
        package(inputs, tmp_path / "release")


@pytest.mark.parametrize("index,name", [(1, "omi.elf"), (2, "zephyr.elf"), (2, "zephyr.uf2")])
def test_wrong_current_source_sha(inputs, tmp_path, index, name):
    path = inputs[index] / name
    path.write_bytes(path.read_bytes().replace(SHA.encode(), b"b" * 40))
    with pytest.raises(ValueError, match="source SHA mismatch"):
        package(inputs, tmp_path / "release")


def test_missing_provenance(inputs, tmp_path):
    (inputs[3] / "dependencies.lock").unlink()
    with pytest.raises(ValueError, match="Missing or empty"):
        package(inputs, tmp_path / "release")


def test_snapshot_excludes_dirty_and_ignored_inputs(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    def git(*args):
        return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout.strip()
    git("init")
    (repo / "tracked").write_text("committed")
    (repo / ".gitignore").write_text("ignored\n")
    git("add", "tracked", ".gitignore")
    git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-m", "fixture")
    sha = git("rev-parse", "HEAD")
    (repo / "tracked").write_text("dirty")
    (repo / "ignored").write_text("ignored")
    (repo / "untracked").write_text("untracked")
    scratch = tmp_path / "scratch"
    scratch.mkdir()
    source = build.snapshot(repo, sha, scratch)
    assert (source / "tracked").read_text() == "committed"
    assert not (source / "ignored").exists()
    assert not (source / "untracked").exists()


def test_build_commands_have_sysbuild_child_metadata_and_fresh_sdkconfig(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(build, "docker", lambda *args: calls.append(args))
    monkeypatch.setattr(build, "run", lambda *args: "c" * 40)
    build.build(tmp_path, VERSION, SHA, "esp32s31", build.ESP_IMAGE, build.NCS_IMAGE)
    esp_script, nrf_script = calls[0][2], calls[1][2]
    assert esp_script.startswith('. "${IDF_PATH:-/opt/esp/idf}/export.sh"\n')
    assert "-DSDKCONFIG=/work/esp-sdkconfig" in esp_script
    assert "sdkconfig.release.esp32s31.defaults" in esp_script
    for name in build.ON_OPTIONS:
        assert f"-D{name}=ON" in esp_script
    for name in build.OFF_OPTIONS:
        assert f"-D{name}=OFF" in esp_script
    for name in ("S31_TASK_CPU_PROFILE", "S31_MESH_COEX_PREFER_WIFI", "S31_MESH_PAIR_UNICAST"):
        assert f"-D{name}=OFF" in esp_script
    assert calls[0][3]["OMI_ESP_LC3_BENCH"] == "1"
    assert "west update --narrow -o=--depth=1" in nrf_script
    assert "west init -l /work/ncs/release-manifest" in nrf_script
    assert nrf_script.startswith('export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"\n. /opt/toolchain-env.sh\n')
    assert "west manifest --freeze --active-only" in nrf_script
    assert "west list -f '{name}'" in nrf_script
    assert 'if [ "$project" != manifest ]' in nrf_script
    assert "west list --all" not in nrf_script
    assert "west build --sysbuild -b xiao_ble/nrf52840" in nrf_script
    assert f"-Dnrf_mesh_OMI_FIRMWARE_VERSION={VERSION}" in nrf_script
    assert f"-Dnrf_mesh_OMI_GIT_SHA={SHA}" in nrf_script
    assert "-Dnrf_mesh_CONFIG_BUILD_OUTPUT_UF2=y" in nrf_script
    assert "CONFIG_NCS_BOOT_BANNER" not in nrf_script
    assert "-Dnrf_mesh_CONFIG_BOOT_BANNER=y" not in nrf_script
    expected_manifest = (build.ROOT / "tools/release/ncs-v3.4.1.yml").read_bytes()
    assert (tmp_path / "ncs/release-manifest/west.yml").read_bytes() == expected_manifest
    assert (tmp_path / "provenance/west-input.yml").read_bytes() == expected_manifest
    assert (tmp_path / "provenance/release-tools-source-sha.txt").read_text() == "c" * 40 + "\n"


def test_legacy_commands_generate_metadata_without_source_patch(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(build, "docker", lambda *args: calls.append(args))
    monkeypatch.setattr(build, "run", lambda *args: "c" * 40)
    build.build(tmp_path, "0.1.0", build.LEGACY_SHA, "esp32s3", build.ESP_IMAGE, build.NCS_IMAGE)
    assert "-DPROJECT_VER=0.1.0" in calls[0][2]
    assert "-DESP_LC3_BENCH=ON" not in calls[0][2]
    script = calls[1][2]
    assert script.index('export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"') < script.index(". /opt/toolchain-env.sh") < script.index("west init")
    configure = "west build --sysbuild --cmake-only -b xiao_ble/nrf52840 /work/source/nrf_mesh -d /work/nrf -- -Dnrf_mesh_CONFIG_BUILD_OUTPUT_UF2=y -Dnrf_mesh_CONFIG_NCS_BOOT_BANNER=n -Dnrf_mesh_CONFIG_BOOT_BANNER=y"
    reconfigure = f"cmake -S /work/source/nrf_mesh -B /work/nrf/nrf_mesh -DBUILD_VERSION:STRING=0.1.0+{build.LEGACY_SHA}"
    compile = "cmake --build /work/nrf"
    assert script.index(configure) < script.index(reconfigure) < script.index(compile)
    assert "nrf_mesh_BUILD_VERSION" not in script
    assert "CONFIG_BUILD_VERSION" not in script
    assert not (tmp_path / "extra.conf").exists()
    expected_manifest = (build.ROOT / "tools/release/ncs-v2.7.0.yml").read_bytes()
    assert (tmp_path / "ncs/release-manifest/west.yml").read_bytes() == expected_manifest
    assert (tmp_path / "provenance/west-input.yml").read_bytes() == expected_manifest


def test_driver_manifest_is_used_even_when_archive_contains_different_input(tmp_path, monkeypatch):
    archived_manifest = tmp_path / "source/tools/release/ncs-v2.7.0.yml"
    archived_manifest.parent.mkdir(parents=True)
    archived_manifest.write_text("not a release-driver input\n")
    monkeypatch.setattr(build, "docker", lambda *args: None)
    monkeypatch.setattr(build, "run", lambda *args: "c" * 40)
    build.build(tmp_path, "0.1.0", build.LEGACY_SHA, "esp32s3", build.LEGACY_ESP_IMAGE, build.NCS_IMAGE)
    assert (tmp_path / "provenance/west-input.yml").read_bytes() == (build.ROOT / "tools/release/ncs-v2.7.0.yml").read_bytes()


@pytest.mark.parametrize("profile,version,sha", [("esp32s31", VERSION, SHA), ("esp32s3", "0.1.0", build.LEGACY_SHA)])
def test_generated_nrf_initialization_exposes_tools_with_unset_library_path(tmp_path, monkeypatch, profile, version, sha):
    calls = []
    monkeypatch.setattr(build, "docker", lambda *args: calls.append(args))
    monkeypatch.setattr(build, "run", lambda *args: "c" * 40)
    build.build(tmp_path, version, sha, profile, build.ESP_IMAGE, build.NCS_IMAGE)
    prefix = calls[1][2].split("west init", 1)[0]
    tools = tmp_path / "fixture-tools"
    tools.mkdir()
    west = tools / "west"
    west.write_text("#!/bin/sh\nprintf '%s' fixture-west\n")
    west.chmod(0o755)
    setup = tmp_path / "toolchain-env.sh"
    setup.write_text(f'test "${{LD_LIBRARY_PATH+x}}" = x\nexport LD_LIBRARY_PATH="${{LD_LIBRARY_PATH}}:/fixture/lib"\nexport PATH="{tools}:$PATH"\n')
    # Substitute only the fixture location; execute the generated initialization itself.
    script = prefix.replace("/opt/toolchain-env.sh", str(setup)) + "west --version"
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env["BASH_ENV"] = ""
    result = subprocess.run(["/bin/bash", "-euc", script], env=env, capture_output=True, text=True, check=True)
    assert result.stdout == "fixture-west"


def test_explicit_work_directory_is_retained_on_failure_and_not_reused(tmp_path):
    work_dir = tmp_path / "work"
    with pytest.raises(RuntimeError, match="build failed"):
        with build.scratch_directory(work_dir) as scratch:
            (scratch / "diagnostic.log").write_text("failure details\n")
            raise RuntimeError("build failed")
    assert (work_dir / "diagnostic.log").read_text() == "failure details\n"
    with pytest.raises(ValueError, match="absent or empty"):
        with build.scratch_directory(work_dir):
            pytest.fail("Existing scratch must not be reused")


def test_default_scratch_is_removed(tmp_path):
    with build.scratch_directory(None) as scratch:
        assert scratch.is_dir()
    assert not scratch.exists()


@pytest.mark.parametrize("name", ["west-input.yml", "release-tools-source-sha.txt"])
def test_missing_driver_provenance_fails_before_packaging(inputs, tmp_path, name):
    (inputs[3] / name).unlink()
    with pytest.raises(ValueError, match="Missing or empty"):
        package(inputs, tmp_path / "release")
    assert not (tmp_path / "release").exists()


@pytest.mark.parametrize("image", ["espressif/idf:v5.5.2", "espressif/idf@sha256:abc"])
def test_require_digest_pinned_builder(image):
    with pytest.raises(ValueError, match="pinned"):
        build.pinned(image)


@pytest.fixture
def ncs_docker_fixture(monkeypatch, tmp_path):
    """Execute the effective shell argv using the reported NCS image entrypoint."""
    real_run = subprocess.run
    results = []
    startup = tmp_path / "non-interactive-setup.sh"
    startup.write_text("exit 24\n")
    monkeypatch.setenv("BASH_ENV", str(startup))

    def fake_docker_run(args, **kwargs):
        assert args[:2] == ["docker", "run"]
        image_index = next(i for i, arg in enumerate(args) if "@sha256:" in arg)
        options, command = args[2:image_index], args[image_index + 1:]
        entrypoint = ["/bin/bash", "-c"]
        if "--entrypoint" in options:
            entrypoint = [options[options.index("--entrypoint") + 1]]
        # Do not generate a replacement script: execute the caller's actual argv.
        env = os.environ.copy()
        for index, option in enumerate(options):
            if option == "--env":
                key, value = options[index + 1].split("=", 1)
                env[key] = value
        result = real_run(entrypoint + command, env=env, capture_output=True, text=True)
        results.append(result)
        if kwargs.get("check"):
            result.check_returncode()
        return result

    monkeypatch.setattr(build.subprocess, "run", fake_docker_run)
    return results


@pytest.mark.parametrize("image", [build.NCS_IMAGE, f"ghcr.io/nrfconnect/sdk-nrf-toolchain:v2.7.0@sha256:{build.LEGACY_NCS_DIGEST}"])
def test_ncs_docker_executes_script_and_propagates_failure(tmp_path, ncs_docker_fixture, image):
    build.docker(image, tmp_path, "printf '%s' release-script-executed")
    assert ncs_docker_fixture[-1].stdout == "release-script-executed"
    with pytest.raises(subprocess.CalledProcessError) as failure:
        build.docker(image, tmp_path, "exit 23")
    assert failure.value.returncode == 23


@pytest.mark.parametrize("image", [build.ESP_IMAGE, build.LEGACY_ESP_IMAGE])
def test_esp_docker_uses_explicit_shell_entrypoint(tmp_path, monkeypatch, image):
    calls = []
    monkeypatch.setattr(build.subprocess, "run", lambda args, **kwargs: calls.append((args, kwargs)))
    script = '. "${IDF_PATH:-/opt/esp/idf}/export.sh"\nidf.py --version'
    build.docker(image, tmp_path, script)
    args, kwargs = calls[0]
    assert args[args.index("--entrypoint") + 1] == "/bin/bash"
    assert args[args.index(image) + 1:] == ["-euc", script]
    assert any(args[index:index + 2] == ["--env", "BASH_ENV="] for index in range(len(args) - 1))
    assert kwargs["check"] is True


@pytest.mark.parametrize("override", [None, build.ESP_IMAGE])
def test_legacy_cli_defaults_to_verified_idf_pin_and_allows_override(tmp_path, monkeypatch, override):
    class BuildReached(Exception):
        pass

    def fake_run(args, **kwargs):
        if "rev-parse" in args:
            return build.LEGACY_SHA
        assert "show" in args
        return f"image: ghcr.io/nrfconnect/sdk-nrf-toolchain:v2.7.0@sha256:{build.LEGACY_NCS_DIGEST}"

    def fake_build(scratch, version, sha, profile, esp_image, ncs_image):
        assert esp_image == (override or build.LEGACY_ESP_IMAGE)
        assert profile == "esp32s3"
        assert version == "0.1.0"
        assert sha == build.LEGACY_SHA
        raise BuildReached

    monkeypatch.setattr(build, "run", fake_run)
    monkeypatch.setattr(build, "snapshot", lambda repo, sha, scratch: scratch / "source")
    monkeypatch.setattr(build, "build", fake_build)
    args = ["--version", "0.1.0", "--source-sha", build.LEGACY_SHA, "--profile", "esp32s3", "--output-dir", str(tmp_path / "release")]
    if override:
        args += ["--esp-image", override]
    with pytest.raises(BuildReached):
        build.main(args)
