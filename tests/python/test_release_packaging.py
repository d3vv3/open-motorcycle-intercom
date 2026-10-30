"""Release packaging safety checks using synthetic firmware, never Docker."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import zipfile

import pytest

from tools.release import build


VERSION = "0.2.0-alpha.1"
SHA = "a" * 40
# build.rs rows rendered into .integration_metadata from OMI_FIRMWARE_VERSION/OMI_GIT_SHA.
NRF_IDENTITY = f"version={VERSION}\nsource_sha={SHA}\n".encode()


def uf2(address=0x27000, size=256, family=0xADA52840, number=0, total=1):
    block = bytearray(512)
    struct.pack_into("<8I", block, 0, 0x0A324655, 0x9E5D5157, 0x2000, address, size, number, total, family)
    block[32:32 + len(NRF_IDENTITY)] = NRF_IDENTITY
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
    (nrf / "zephyr.elf").write_bytes(b"ELF\0" + NRF_IDENTITY)
    (nrf / "zephyr.uf2").write_bytes(uf2())
    (esp / "project_description.json").write_text(json.dumps({"target": "esp32s31", "project_version": VERSION, "app_bin": "omi.bin"}))
    (esp / "flasher_args.json").write_text(json.dumps({"extra_esptool_args": {"chip": "esp32s31", "before": "default_reset", "after": "hard_reset"}, "flash_settings": {"flash_mode": "dio", "flash_freq": "80m", "flash_size": "16MB"}, "flash_files": {"0x2000": "bootloader/bootloader.bin", "0x8000": "partition_table/partition-table.bin", "0xf000": "extra/phy.bin", "0x10000": "omi.bin"}, "bootloader": {"offset": "0x2000", "file": "bootloader/bootloader.bin"}, "partition-table": {"offset": "0x8000", "file": "partition_table/partition-table.bin"}, "app": {"offset": "0x10000", "file": "omi.bin"}}))
    for name in ("rustc-version.txt", "Cargo.lock", "nrf-build-argv.json", "idf-commit.txt", "idf-version.txt", "sdkconfig", "dependencies.lock"):
        (provenance / name).write_text("captured\n")
    (provenance / "release-tools-source-sha.txt").write_text("c" * 40 + "\n")
    (provenance / "idf-version.txt").write_text("ESP-IDF v6.1-dev\n")
    return source, esp, nrf, provenance


def package(inputs, output):
    return build.package(*inputs, output, VERSION, SHA, "esp32s31", {"esp": build.ESP_IMAGE})


def uf2_with_policy_field(field, clock=True):
    """Put the policy key itself across two contiguous 256-byte payloads."""
    first = bytearray(uf2(total=2))
    second = bytearray(uf2(number=1, total=2, address=0x27100))
    second[32:288] = b"\0" * 256
    prefix = b"identity clock_origin=shared-timer0 " if clock else b"identity "
    row = prefix + field + b"\n"
    start = 248 - len(prefix)
    # Pad short implicit-policy rows to preserve the complete UF2 block.
    first[32 + start:288] = row[:256 - start].ljust(256 - start, b"\0")
    tail = row[256 - start:]
    second[32:32 + len(tail)] = tail
    # Reverse transport order to exercise address-based reconstruction too.
    return bytes(second + first)


@pytest.mark.parametrize("clock", [False, True])
@pytest.mark.parametrize("policy", [b"RetainGuardedCompletion", b"FutureExperimentalPolicy"])
def test_package_gates_reconstructed_nrf_policy(inputs, tmp_path, clock, policy):
    field = b"completion_policy=" + policy
    data = uf2_with_policy_field(field, clock)
    assert field not in data
    assert field in build.uf2_payload(data)
    (inputs[2] / "zephyr.uf2").write_bytes(data)
    output = tmp_path / "release"
    if clock and policy == b"FutureExperimentalPolicy":
        with pytest.raises(ValueError, match="^experimental nRF completion policy is not release-qualified$"):
            package(inputs, output)
        assert not output.exists()
    else:
        package(inputs, output)


@pytest.mark.parametrize("field", [
    b"completion_policy=StrictSoftwareDeadline",
    b"firmware_version=RetainGuardedCompletion completion_policy=StrictSoftwareDeadline",
    b"",  # Historical identity with implicit policy.
    b"RetainGuardedCompletion",  # An enum name is not a policy field.
])
def test_package_accepts_strict_or_implicit_nrf_policy(inputs, tmp_path, field):
    (inputs[2] / "zephyr.uf2").write_bytes(uf2_with_policy_field(field))
    # Debug ELF strings do not describe the executing image's policy.
    with (inputs[2] / "zephyr.elf").open("ab") as elf:
        elf.write(b"\0completion_policy=RetainGuardedCompletion\nRetainGuardedCompletion\0")
    package(inputs, tmp_path / "release")


def test_prebuilt_packages_bench_artifacts_without_source_identity(inputs, tmp_path):
    """Bench images keep their own identity rows; Zephyr/IDF provenance is not required."""
    source, esp, nrf, provenance = inputs
    for name in ("omi.bin", "omi.elf"):
        (esp / name).write_bytes((esp / name).read_bytes().replace(SHA.encode(), b"9" * 40))
    (nrf / "zephyr.elf").write_bytes(b"ELF\0version=0.1.0 source_HEAD=unavailable\0")
    (nrf / "zephyr.uf2").write_bytes(uf2_with_policy_field(b"completion_policy=RetainGuardedCompletion"))
    for path in provenance.iterdir():
        if path.name != "release-tools-source-sha.txt":
            path.unlink()
    with pytest.raises(ValueError, match="^nRF firmware version/source SHA mismatch$"):
        package(inputs, tmp_path / "strict")
    prebuilt = {"esp": esp, "nrf": nrf}
    build.package(*inputs, tmp_path / "release", VERSION, SHA, "esp32s31", {}, prebuilt)
    manifest = json.loads((tmp_path / "release/manifest.json").read_text())
    assert manifest["prebuilt"] == {"esp": str(esp), "nrf": str(nrf)}
    assert manifest["nrf_toolchain"] is None and manifest["idf_version"] == "prebuilt"
    assert manifest["reproducibility"].startswith("Prebuilt bench artifacts")
    assert (tmp_path / "release" / f"omi-{VERSION}-nrf52840.uf2").read_bytes() == (nrf / "zephyr.uf2").read_bytes()
    # Experimental gates still apply to prebuilt images.
    (nrf / "zephyr.uf2").write_bytes(uf2_with_policy_field(b"completion_policy=FutureExperimentalPolicy"))
    with pytest.raises(ValueError, match="^experimental nRF completion policy is not release-qualified$"):
        build.package(*inputs, tmp_path / "rejected", VERSION, SHA, "esp32s31", {}, prebuilt)


@pytest.mark.parametrize("kwargs,message", [
    ({"family": 0}, "family flash blocks"),
    ({"address": 0x26FFF}, "application boundary"),
    ({"total": 3}, "sequence"),
])
def test_uf2_validation_precedes_completion_policy_gate(inputs, tmp_path, kwargs, message):
    data = bytearray(uf2_with_policy_field(b"completion_policy=FutureExperimentalPolicy"))
    for name, value in kwargs.items():
        offset = {"family": 28, "address": 12, "total": 24}[name]
        struct.pack_into("<I", data, offset, value)
    (inputs[2] / "zephyr.uf2").write_bytes(data)
    with pytest.raises(ValueError, match=message):
        package(inputs, tmp_path / "release")
    assert not (tmp_path / "release").exists()


EARLY_TIMING_FIELD = b"early_timing_service=EarlyTimingServiceExperimental"
SAME_SLOT_REFIT_FIELD = b"same_slot_audio_refit=SameSlotAudioRefitExperimental"
NONBLOCKING_BRIDGE_FIELD = b"nonblocking_bridge_delivery=NonblockingBridgeDeliveryExperimental"
UNMASKED_BRIDGE_FIELD = b"unmasked_bridge_owner=UnmaskedBridgeOwnerExperimental"
ADMISSION_HEADROOM_FIELD = b"admission_headroom=AdmissionHeadroomExperimental"
STRICT_COMPLETION_ROW = b"clock_origin=shared-timer0 completion_policy=StrictSoftwareDeadline\n"


def uf2_with_experimental_field(field, split=False):
    if split:
        data = bytearray(uf2_with_policy_field(field))
        first = 512  # The lower-address block is last in transport order.
    else:
        data = bytearray(uf2())
        first = 0
        assert len(field) + 1 <= 256 - 184
        data[32 + 184:32 + 184 + len(field) + 1] = field + b"\n"
    data[first + 32 + 112:first + 32 + 112 + len(STRICT_COMPLETION_ROW)] = STRICT_COMPLETION_ROW
    return bytes(data)


@pytest.mark.parametrize("location,split", [
    ("elf", False), ("payload", False), ("both", False),
    ("payload", True), ("both", True),
])
@pytest.mark.parametrize("marker,message", [
    (EARLY_TIMING_FIELD, "early timing service"),
    (SAME_SLOT_REFIT_FIELD, "same-slot audio refit"),
])
def test_package_rejects_experimental_marker_with_strict_completion(inputs, tmp_path, location, split, marker, message):
    elf = inputs[2] / "zephyr.elf"
    elf.write_bytes(elf.read_bytes() + STRICT_COMPLETION_ROW
                    + (marker + b"\n" if location in ("elf", "both") else b""))
    field = marker if location in ("payload", "both") else b""
    data = uf2_with_experimental_field(field, split)
    if split:
        assert marker not in data
        assert marker in build.uf2_payload(data)
    (inputs[2] / "zephyr.uf2").write_bytes(data)
    output = tmp_path / "release"
    with pytest.raises(ValueError, match=f"^experimental nRF {message} is not release-qualified$"):
        package(inputs, output)
    assert not output.exists()


@pytest.mark.parametrize("field", [
    b"",  # Default OFF: strict completion without the experimental marker.
    NONBLOCKING_BRIDGE_FIELD,  # Bench-proven; release-qualified.
    UNMASKED_BRIDGE_FIELD,
    ADMISSION_HEADROOM_FIELD,
    b"EarlyTimingServiceExperimental",  # A debug enum name is not a field.
    b"other=EarlyTimingServiceExperimental",
    b"other=" + EARLY_TIMING_FIELD,
    b"prefix_" + EARLY_TIMING_FIELD,
    EARLY_TIMING_FIELD + b"Suffix",
    EARLY_TIMING_FIELD[:-1],
    b"SameSlotAudioRefitExperimental",
    b"other=SameSlotAudioRefitExperimental",
    b"other=" + SAME_SLOT_REFIT_FIELD,
    b"prefix_" + SAME_SLOT_REFIT_FIELD,
    SAME_SLOT_REFIT_FIELD + b"Suffix",
    SAME_SLOT_REFIT_FIELD[:-1],
    b"NonblockingBridgeDeliveryExperimental",
    b"other=NonblockingBridgeDeliveryExperimental",
    b"other=" + NONBLOCKING_BRIDGE_FIELD,
    b"prefix_" + NONBLOCKING_BRIDGE_FIELD,
    NONBLOCKING_BRIDGE_FIELD + b"Suffix",
    NONBLOCKING_BRIDGE_FIELD[:-1],
    b"UnmaskedBridgeOwnerExperimental",
    b"other=UnmaskedBridgeOwnerExperimental",
    b"other=" + UNMASKED_BRIDGE_FIELD,
    b"prefix_" + UNMASKED_BRIDGE_FIELD,
    UNMASKED_BRIDGE_FIELD + b"Suffix",
    UNMASKED_BRIDGE_FIELD[:-1],
    b"AdmissionHeadroomExperimental",
    b"other=AdmissionHeadroomExperimental",
    b"other=" + ADMISSION_HEADROOM_FIELD,
    b"prefix_" + ADMISSION_HEADROOM_FIELD,
    ADMISSION_HEADROOM_FIELD + b"Suffix",
    ADMISSION_HEADROOM_FIELD[:-1],
])
def test_package_accepts_strict_completion_without_exact_experimental_marker(inputs, tmp_path, field):
    elf = inputs[2] / "zephyr.elf"
    elf.write_bytes(elf.read_bytes() + STRICT_COMPLETION_ROW + field + b"\n")
    (inputs[2] / "zephyr.uf2").write_bytes(uf2_with_experimental_field(field, split=True))
    assert len(package(inputs, tmp_path / "release")) == 5


@pytest.mark.parametrize("marker", [EARLY_TIMING_FIELD, SAME_SLOT_REFIT_FIELD])
def test_experimental_marker_is_not_joined_across_flash_address_gap(inputs, tmp_path, marker):
    data = bytearray(uf2_with_experimental_field(marker, split=True))
    struct.pack_into("<I", data, 12, 0x27200)
    assert marker not in build.uf2_payload(data)
    (inputs[2] / "zephyr.uf2").write_bytes(data)
    package(inputs, tmp_path / "release")


def test_policy_field_is_not_joined_across_flash_address_gap(inputs, tmp_path):
    data = bytearray(uf2_with_policy_field(b"completion_policy=FutureExperimentalPolicy", clock=False))
    # The first transport block is the higher-address payload. Leave a flash gap
    # where the policy key was split, so it is no longer a complete field.
    struct.pack_into("<I", data, 12, 0x27200)
    assert b"completion_policy=FutureExperimentalPolicy" not in build.uf2_payload(data)
    (inputs[2] / "zephyr.uf2").write_bytes(data)
    package(inputs, tmp_path / "release")


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
        assert archive.read("provenance/Cargo.lock")
        assert archive.read("provenance/dependencies.lock")
        assert archive.read("provenance/rustc-version.txt") == (inputs[3] / "rustc-version.txt").read_bytes()
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["source_sha"] == SHA
    assert manifest["release_tools_source_sha"] == "c" * 40
    assert manifest["esp_identity"] == "app-descriptor-version-only"
    assert manifest["nrf_toolchain"]["sha256"] == hashlib.sha256((inputs[3] / "rustc-version.txt").read_bytes()).hexdigest()
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
    first[32:32 + len(NRF_IDENTITY)] = (VERSION.encode() + b"\0").ljust(len(NRF_IDENTITY), b"\0")
    first[32 + 240:32 + 256] = identity[:16]
    second[32:32 + 25] = identity[16:]
    data = bytes(second + first)
    build.validate_uf2(data)
    payload = build.uf2_payload(data)
    assert VERSION.encode() + b"\0" in payload and identity in payload


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


@pytest.mark.parametrize("index,name", [(2, "zephyr.elf"), (2, "zephyr.uf2")])
def test_wrong_current_source_sha(inputs, tmp_path, index, name):
    path = inputs[index] / name
    path.write_bytes(path.read_bytes().replace(SHA.encode(), b"b" * 40))
    with pytest.raises(ValueError, match="source SHA mismatch"):
        package(inputs, tmp_path / "release")


def test_c_esp_image_needs_no_embedded_source_sha(inputs, tmp_path):
    """The pure-C S31 image carries only the app-descriptor version."""
    for name in ("omi.bin", "omi.elf"):
        path = inputs[1] / name
        path.write_bytes(path.read_bytes().replace(SHA.encode(), b"\0" * 40))
    package(inputs, tmp_path / "release")


def test_reject_unsupported_profile(inputs, tmp_path):
    with pytest.raises(ValueError, match="Unsupported release profile"):
        build.package(*inputs, tmp_path / "release", VERSION, SHA, "esp32s3", {})


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


def test_build_commands_have_fresh_sdkconfig_and_host_cargo_nrf(tmp_path, monkeypatch):
    calls, cargo = [], []
    monkeypatch.setattr(build, "docker", lambda *args: calls.append(args))
    monkeypatch.setattr(build, "run", lambda *args: "c" * 40)
    monkeypatch.setattr(build, "cargo_nrf", lambda *args: cargo.append(args))
    build.build(tmp_path, VERSION, SHA, build.ESP_IMAGE)
    assert len(calls) == 1
    esp_script = calls[0][2]
    assert esp_script.startswith('. "${IDF_PATH:-/opt/esp/idf}/export.sh"\n')
    assert "-DSDKCONFIG=/work/esp-sdkconfig" in esp_script
    assert "-DSDKCONFIG_DEFAULTS=/work/source/sdkconfig.defaults " in esp_script
    assert "-DIDF_TARGET=esp32s31" in esp_script and "idf.py --preview " in esp_script
    assert "OMI_GIT_SHA" not in esp_script and "rust_embassy" not in esp_script
    for name in build.ON_OPTIONS:
        assert f"-D{name}=ON" in esp_script
    for name in build.OFF_OPTIONS:
        assert f"-D{name}=OFF" in esp_script
    for name in ("ESP_LC3_BENCH", "S31_LC3_WIRE", "S31_LC3_SELFTEST", "S31_LC3_SERIALIZE",
                 "S31_LC3_INTERNAL_BUFFERS", "S31_LC3_SINGLE_OWNER"):
        assert f"-D{name}=ON" in esp_script
    assert "-DS31_TASK_CPU_PROFILE=OFF" in esp_script
    for name in ("S31_LC3_SPLIT_CORES", "S31_LC3_SKIP_RX", "S31_MESH_COEX_PREFER_WIFI",
                 "S31_MESH_PAIR_UNICAST"):
        assert name not in esp_script
    assert calls[0][3]["OMI_ESP_LC3_BENCH"] == "1"
    assert cargo == [(tmp_path, VERSION, SHA)]
    assert (tmp_path / "provenance/release-tools-source-sha.txt").read_text() == "c" * 40 + "\n"


def test_cargo_nrf_builds_archived_crate_with_release_identity_and_records_provenance(tmp_path, monkeypatch):
    crate = tmp_path / "source" / build.NRF_FIRMWARE
    crate.mkdir(parents=True)
    (crate / "Cargo.lock").write_text("locked\n")
    (tmp_path / "provenance").mkdir()
    runs = []

    def fake_run(argv, **kwargs):
        runs.append((argv, kwargs))
        if argv[0] == "cargo":
            out = Path(kwargs["env"]["CARGO_TARGET_DIR"]) / build.NRF_TARGET / "release"
            out.mkdir(parents=True)
            (out / "nrf-embassy-firmware").write_bytes(b"ELF\0" + NRF_IDENTITY)
            return None
        return subprocess.CompletedProcess(argv, 0, stdout="rustc 1.95.0\n")

    monkeypatch.setattr(build.subprocess, "run", fake_run)
    monkeypatch.setattr(build.nrf_proof, "elf_to_uf2", lambda elf: b"uf2:" + elf)
    build.cargo_nrf(tmp_path, VERSION, SHA)
    cargo_argv, cargo_kwargs = runs[0]
    assert cargo_argv == ["cargo", "build", "--locked", "--release", "--target", build.NRF_TARGET]
    assert cargo_kwargs["cwd"] == crate and cargo_kwargs["check"]
    assert cargo_kwargs["env"]["OMI_FIRMWARE_VERSION"] == VERSION and cargo_kwargs["env"]["OMI_GIT_SHA"] == SHA
    assert cargo_kwargs["env"]["CARGO_TARGET_DIR"] == str(tmp_path / "nrf-target")
    assert runs[1][0] == ["rustc", "-vV"] and runs[1][1]["cwd"] == crate
    assert (tmp_path / "nrf/zephyr.elf").read_bytes() == b"ELF\0" + NRF_IDENTITY
    assert (tmp_path / "nrf/zephyr.uf2").read_bytes() == b"uf2:ELF\0" + NRF_IDENTITY
    assert (tmp_path / "provenance/rustc-version.txt").read_text() == "rustc 1.95.0\n"
    assert (tmp_path / "provenance/Cargo.lock").read_text() == "locked\n"
    assert json.loads((tmp_path / "provenance/nrf-build-argv.json").read_text()) == cargo_argv


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


@pytest.mark.parametrize("name", ["rustc-version.txt", "release-tools-source-sha.txt"])
def test_missing_driver_provenance_fails_before_packaging(inputs, tmp_path, name):
    (inputs[3] / name).unlink()
    with pytest.raises(ValueError, match="Missing or empty"):
        package(inputs, tmp_path / "release")
    assert not (tmp_path / "release").exists()


@pytest.mark.parametrize("image", ["espressif/idf:v5.5.2", "espressif/idf@sha256:abc"])
def test_require_digest_pinned_builder(image):
    with pytest.raises(ValueError, match="pinned"):
        build.pinned(image)


def test_esp_docker_uses_explicit_shell_entrypoint(tmp_path, monkeypatch):
    image = build.ESP_IMAGE
    calls = []
    monkeypatch.setattr(build.subprocess, "run", lambda args, **kwargs: calls.append((args, kwargs)))
    script = '. "${IDF_PATH:-/opt/esp/idf}/export.sh"\nidf.py --version'
    build.docker(image, tmp_path, script)
    args, kwargs = calls[0]
    assert args[args.index("--entrypoint") + 1] == "/bin/bash"
    assert args[args.index(image) + 1:] == ["-euc", script]
    assert any(args[index:index + 2] == ["--env", "BASH_ENV="] for index in range(len(args) - 1))
    assert kwargs["check"] is True


@pytest.mark.parametrize("override", [None, "espressif/idf@sha256:" + "0" * 64])
def test_cli_defaults_to_pinned_idf_and_allows_override(tmp_path, monkeypatch, override):
    class BuildReached(Exception):
        pass

    def fake_run(args, **kwargs):
        assert "rev-parse" in args
        return SHA

    def fake_build(scratch, version, sha, esp_image):
        assert esp_image == (override or build.ESP_IMAGE)
        assert version == VERSION and sha == SHA
        raise BuildReached

    monkeypatch.setattr(build, "run", fake_run)
    monkeypatch.setattr(build, "snapshot", lambda repo, sha, scratch: scratch / "source")
    monkeypatch.setattr(build, "build", fake_build)
    args = ["--version", VERSION, "--source-sha", SHA, "--output-dir", str(tmp_path / "release")]
    if override:
        args += ["--esp-image", override]
    with pytest.raises(BuildReached):
        build.main(args)
