"""Synthetic ELF safety cases; no compiler, hardware, or deployment required."""

import struct

import pytest

from tools.migration import nrf_proof as proof
from tools.release.build import uf2_payload, validate_uf2


START = 0x27000
END = 0xEA000
RAM = 0x20000000
TOP = 0x20040000
ENTRY = START + 9


def segment(payload=b"data", *, vaddr=START + 0x100, paddr=None, memsz=None,
            flags=4, kind=1, align=1):
    return dict(payload=payload, vaddr=vaddr, paddr=vaddr if paddr is None else paddr,
                memsz=len(payload) if memsz is None else memsz, flags=flags,
                kind=kind, align=align)


def vector(*, sp=TOP, reset=ENTRY, address=START, flags=5):
    return segment(struct.pack("<2I", sp, reset) + b"\x00\xbf" * 4,
                   vaddr=address, flags=flags)


def elf(segments=None, **overrides):
    if segments is None:
        segments = [vector()]
    header = dict(ident=b"\x7fELF\x01\x01\x01" + b"\0" * 9,
                  kind=2, machine=40, version=1, entry=ENTRY, phoff=52,
                  shoff=0, flags=0x05000200, ehsize=52, phentsize=32,
                  phnum=len(segments), shentsize=0, shnum=0, shstrndx=0)
    header.update(overrides)
    result = bytearray(proof.ELF_HEADER.pack(*header.values()))
    offset = 52 + len(segments) * 32
    payloads = bytearray()
    for item in segments:
        payload = item["payload"]
        result.extend(struct.pack("<8I", item["kind"], offset, item["vaddr"],
                                  item["paddr"], len(payload), item["memsz"],
                                  item["flags"], item["align"]))
        payloads.extend(payload)
        offset += len(payload)
    return bytes(result + payloads)


def change_word(data, offset, value):
    result = bytearray(data)
    struct.pack_into("<I", result, offset, value)
    return bytes(result)


def test_valid_vector_and_uf2_compatibility():
    assert proof.APP_START == START and proof.APP_END == END
    source = elf()
    result = proof.elf_to_uf2(source)
    assert validate_uf2(result) == dict(family="0xADA52840", blocks=1,
                                      start="0x27000", end_exclusive="0x27100")
    assert uf2_payload(result) == vector()["payload"].ljust(256, b"\xff")
    assert proof.elf_to_uf2(source) == result


@pytest.mark.parametrize("flags", [0x05000000, 0x05000200, 0x05000400])
@pytest.mark.parametrize("osabi", [0, 3, 97])
def test_supported_arm_eabi_flags_and_identification(flags, osabi):
    ident = b"\x7fELF\x01\x01\x01" + bytes([osabi]) + b"\0" * 8
    validate_uf2(proof.elf_to_uf2(elf(flags=flags, ident=ident)))


def test_adjacent_loads_and_ram_runtime_top_boundary():
    initialized = segment(b"INIT", vaddr=TOP - 4, paddr=START + 16, flags=6)
    result = proof.elf_to_uf2(elf([vector(), initialized]))
    validate_uf2(result)
    assert uf2_payload(result)[:20] == vector()["payload"] + b"INIT"


def test_ram_data_initialized_at_physical_flash_address_and_bss_not_emitted():
    data = segment(b"INIT", vaddr=RAM, paddr=START + 0x204, memsz=32, flags=6)
    bss = segment(b"", vaddr=RAM + 32, paddr=RAM + 32, memsz=64, flags=6)
    result = proof.elf_to_uf2(elf([bss, data, vector()]))
    assert validate_uf2(result)["blocks"] == 3
    image = uf2_payload(result)
    assert image[:16] == vector()["payload"]
    assert image[16:0x204] == b"\xff" * (0x204 - 16)
    assert image[0x204:0x208] == b"INIT"
    assert image[0x208:] == b"\xff" * (0x300 - 0x208)
    for number in range(3):
        fields = struct.unpack_from("<8I", result, number * 512)
        assert fields[2:] == (0x2000, START + number * 256, 256, number, 3, 0xADA52840)


@pytest.mark.parametrize("address,size", [(END - 256, 256), (END - 1, 1),
                                          (START + 256, 1), (START + 256, 256)])
def test_padding_boundaries(address, size):
    result = proof.elf_to_uf2(elf([vector(), segment(b"x" * size, vaddr=address)]))
    rounded = START + ((address + size - START + 255) // 256) * 256
    assert int(validate_uf2(result)["end_exclusive"], 16) == rounded <= END
    image = uf2_payload(result)
    assert image[address - START:address - START + size] == b"x" * size
    assert image[address - START + size:] == b"\xff" * (rounded - address - size)


@pytest.mark.parametrize("address,size", [(END, 1), (END - 1, 2), (0xEBFFF, 1)])
@pytest.mark.parametrize("runtime", ["flash", "ram-data"])
def test_contracted_boundary_rejects_flash_and_ram_initializers(address, size, runtime):
    load = segment(b"x" * size, vaddr=address if runtime == "flash" else RAM,
                   paddr=address, flags=4 if runtime == "flash" else 6)
    with pytest.raises(ValueError, match="runtime memory|physical bytes") as failure:
        proof.elf_to_uf2(elf([vector(), load]))
    if runtime == "ram-data":
        assert f"[{hex(START)}, {hex(END)})" in str(failure.value)


def test_flash_runtime_cannot_extend_past_contracted_boundary():
    load = segment(b"x", vaddr=END - 1, memsz=2)
    with pytest.raises(ValueError, match="runtime memory"):
        proof.elf_to_uf2(elf([vector(), load]))


@pytest.mark.parametrize("address", [END, END + 0x100, 0xEBFFE])
@pytest.mark.parametrize("field", ["entry", "reset"])
def test_entry_and_reset_reject_newly_excluded_tail(address, field):
    source = elf(entry=address | 1) if field == "entry" else elf([vector(reset=address | 1)])
    with pytest.raises(ValueError, match="ELF entry|Reset vector"):
        proof.elf_to_uf2(source)


@pytest.mark.parametrize("overrides", [
    {"kind": 1}, {"kind": 3}, {"machine": 3}, {"version": 0},
    {"entry": ENTRY & ~1}, {"entry": END + 1}, {"entry": START - 1},
    {"flags": 0}, {"flags": 0x05000600}, {"flags": 0x05000001},
    {"ehsize": 51}, {"phentsize": 31}, {"phnum": 0}, {"phnum": 0xFFFF},
    {"phoff": 0}, {"phoff": 0xFFFFFFF0}, {"phnum": 20},
    {"shoff": 52}, {"shstrndx": 0xFFFF},
    {"shnum": 1, "shoff": 0xFFFFFFF0, "shentsize": 40},
    {"shnum": 1, "shoff": 52, "shentsize": 39},
])
def test_invalid_headers(overrides):
    with pytest.raises(ValueError):
        proof.elf_to_uf2(elf(**overrides))


@pytest.mark.parametrize("offset,value", [(0, 0), (4, 2), (5, 2), (6, 0),
                                          (8, 1), (15, 1)])
def test_invalid_identification(offset, value):
    source = bytearray(elf())
    source[offset] = value
    with pytest.raises(ValueError, match="identification"):
        proof.elf_to_uf2(bytes(source))


@pytest.mark.parametrize("osabi", [1, 2, 4, 9, 96, 98, 255])
def test_unsupported_osabi(osabi):
    ident = b"\x7fELF\x01\x01\x01" + bytes([osabi]) + b"\0" * 8
    with pytest.raises(ValueError, match="identification"):
        proof.elf_to_uf2(elf(ident=ident))


@pytest.mark.parametrize("size", [0, 4, 51, 52, 83, 84, 99])
def test_truncated_elf(size):
    with pytest.raises(ValueError):
        proof.elf_to_uf2(elf()[:size])


@pytest.mark.parametrize("offset,value", [(56, 0xFFFFFFF0), (68, 0xFFFFFFFF),
                                          (72, 0), (72, 15), (76, 8),
                                          (76, 1), (80, 3), (80, 4096)])
def test_malformed_program_headers(offset, value):
    with pytest.raises(ValueError):
        proof.elf_to_uf2(change_word(elf(), offset, value))


@pytest.mark.parametrize("segments", [
    [], [segment(kind=4)], [segment(b"", vaddr=RAM, memsz=16)],
    [vector(), segment(b"")],
    [vector(), segment(vaddr=0x20000)],
    [vector(), segment(vaddr=RAM, paddr=0x20000, memsz=16)],
    [vector(), segment(vaddr=RAM, paddr=END, memsz=16)],
    [vector(), segment(vaddr=RAM, paddr=END - 2, memsz=16)],
    [vector(), segment(vaddr=END - 2)],
    [vector(), segment(vaddr=0xFFFFFFFF, memsz=0xFFFFFFFF)],
    [vector(), segment(vaddr=RAM - 1, paddr=START + 256, memsz=16)],
    [vector(), segment(vaddr=TOP - 4, paddr=START + 256, memsz=8)],
    [vector(), segment(vaddr=RAM, paddr=START + 256, memsz=0xFFFFFFFF)],
    [vector(), segment(b"", vaddr=RAM, paddr=RAM, memsz=0xFFFFFFFF)],
    [vector(), segment(vaddr=START + 256, memsz=8)],
    [vector(), segment(vaddr=RAM, paddr=START + 256, flags=5)],
    [vector(), segment(vaddr=START + 256, paddr=START + 512)],
    [vector(), segment(vaddr=START + 8)],
    [vector(), segment(vaddr=RAM, paddr=START + 8)],
    [vector(), segment(vaddr=RAM, paddr=START + 256, memsz=16),
     segment(vaddr=RAM + 8, paddr=START + 512)],
])
def test_reject_missing_zero_overflow_out_of_range_and_overlapping_loads(segments):
    with pytest.raises(ValueError):
        proof.elf_to_uf2(elf(segments))


def test_rogue_elf_header_load_below_application_is_not_ignored():
    source = bytearray(elf([vector(), segment(b"header", vaddr=0x20000)]))
    struct.pack_into("<I", source, 52 + 32 + 4, 0)
    struct.pack_into("<2I", source, 52 + 32 + 16, 52, 52)
    with pytest.raises(ValueError):
        proof.elf_to_uf2(bytes(source))


@pytest.mark.parametrize("sp", [0, RAM - 8, TOP + 8, TOP - 4])
def test_bad_stack_pointer(sp):
    with pytest.raises(ValueError, match="stack pointer"):
        proof.elf_to_uf2(elf([vector(sp=sp)]))


@pytest.mark.parametrize("sp", [RAM, RAM + 8, TOP - 8, TOP])
def test_stack_pointer_valid_boundaries(sp):
    validate_uf2(proof.elf_to_uf2(elf([vector(sp=sp)])))


@pytest.mark.parametrize("reset", [ENTRY & ~1, START - 1, END + 1, RAM + 1, ENTRY + 2])
def test_bad_reset_vector(reset):
    with pytest.raises(ValueError, match="Reset vector"):
        proof.elf_to_uf2(elf([vector(reset=reset)]))


@pytest.mark.parametrize("segments,entry", [
    ([vector(address=START + 256)], ENTRY),
    ([segment(b"1234567", vaddr=START, flags=5)], ENTRY),
    ([vector(flags=4)], ENTRY),
    ([vector(reset=START + 0x101), segment(flags=4)], START + 0x101),
    ([vector(reset=START + 0x201)], START + 0x201),
    ([segment(vector(reset=START + 15)["payload"][:15], vaddr=START, flags=5)], START + 15),
])
def test_vector_location_completeness_and_executable_entry(segments, entry):
    with pytest.raises(ValueError):
        proof.elf_to_uf2(elf(segments, entry=entry))


@pytest.mark.parametrize("prefix", [0b11101, 0b11110, 0b11111])
@pytest.mark.parametrize("tail", [b"", b"\0", b"\0\0"])
def test_thumb32_entry_requires_complete_instruction(prefix, tail):
    payload = struct.pack("<2IH", TOP, ENTRY, prefix << 11) + tail
    source = elf([segment(payload, vaddr=START, flags=5)])
    if len(tail) < 2:
        with pytest.raises(ValueError, match="Thumb instruction is truncated"):
            proof.elf_to_uf2(source)
    else:
        validate_uf2(proof.elf_to_uf2(source))


def test_thumb16_entry_needs_only_two_loaded_bytes():
    payload = struct.pack("<2IH", TOP, ENTRY, 0xBF00)
    validate_uf2(proof.elf_to_uf2(elf([segment(payload, vaddr=START, flags=5)])))


@pytest.mark.parametrize("existing", [False, True])
@pytest.mark.parametrize("source", [b"not an ELF", elf(phoff=0xFFFFFFF0),
                                    elf([vector(), segment(vaddr=0x20000)])])
def test_invalid_cli_input_never_creates_or_changes_output(tmp_path, existing, source):
    input_path = tmp_path / "proof.elf"
    output = tmp_path / "proof.uf2"
    input_path.write_bytes(source)
    if existing:
        output.write_bytes(b"existing output")
    with pytest.raises(SystemExit) as failure:
        proof.main([str(input_path), str(output)])
    assert failure.value.code == 1
    if existing:
        assert output.read_bytes() == b"existing output"
    else:
        assert not output.exists()


def test_cli_valid_conversion_and_execution_disclaimer(tmp_path, capsys):
    input_path = tmp_path / "proof.elf"
    output = tmp_path / "proof.uf2"
    source = elf()
    input_path.write_bytes(source)
    assert proof.main([str(input_path), str(output)]) == 0
    validate_uf2(output.read_bytes())
    assert input_path.read_bytes() == source
    assert "execution not verified" in capsys.readouterr().out


def test_cli_refuses_to_overwrite_input(tmp_path):
    path = tmp_path / "proof.elf"
    source = elf()
    path.write_bytes(source)
    with pytest.raises(SystemExit):
        proof.main([str(path), str(path)])
    assert path.read_bytes() == source


@pytest.mark.parametrize("alias", ["hardlink", "symlink"])
def test_cli_refuses_input_output_alias(tmp_path, alias):
    path = tmp_path / "proof.elf"
    output = tmp_path / "proof.uf2"
    source = elf()
    path.write_bytes(source)
    if alias == "hardlink":
        output.hardlink_to(path)
    else:
        output.symlink_to(path)
    with pytest.raises(SystemExit) as failure:
        proof.main([str(path), str(output)])
    assert failure.value.code == 1
    assert path.read_bytes() == output.read_bytes() == source
    assert set(tmp_path.iterdir()) == {path, output}


@pytest.mark.parametrize("existing", [False, True])
@pytest.mark.parametrize("failure_point", ["write", "short_write", "fsync", "replace"])
def test_atomic_output_failure_preserves_destination_and_cleans_temp(tmp_path, monkeypatch,
                                                                   existing, failure_point):
    path = tmp_path / "proof.elf"
    output = tmp_path / "proof.uf2"
    source = elf()
    path.write_bytes(source)
    if existing:
        output.write_bytes(b"previous output")

    if failure_point in ("write", "short_write"):
        original_temporary_file = proof.tempfile.NamedTemporaryFile

        def temporary_file(*args, **kwargs):
            stream = original_temporary_file(*args, **kwargs)
            assert kwargs["dir"] == output.parent
            original_write = stream.write

            def incomplete_write(data):
                written = original_write(data[:17])
                if failure_point == "write":
                    raise OSError("Injected write failure")
                return written

            monkeypatch.setattr(stream, "write", incomplete_write)
            return stream

        monkeypatch.setattr(proof.tempfile, "NamedTemporaryFile", temporary_file)
    else:
        def fail(*args):
            raise OSError(f"Injected {failure_point} failure")

        monkeypatch.setattr(proof.os, failure_point, fail)

    with pytest.raises(SystemExit) as failure:
        proof.main([str(path), str(output)])
    assert failure.value.code == 1
    assert path.read_bytes() == source
    if existing:
        assert output.read_bytes() == b"previous output"
    else:
        assert not output.exists()
    assert set(tmp_path.iterdir()) == ({path, output} if existing else {path})


def test_atomic_output_is_synced_before_replacing_existing_destination(tmp_path, monkeypatch):
    path = tmp_path / "proof.elf"
    output = tmp_path / "proof.uf2"
    source = elf()
    path.write_bytes(source)
    output.write_bytes(b"previous output")
    original_fsync = proof.os.fsync
    original_replace = proof.os.replace
    synced = []

    def fsync(fd):
        original_fsync(fd)
        synced.append(fd)

    def replace(temporary, destination):
        assert synced
        assert temporary.parent == output.parent
        assert destination == output
        assert output.read_bytes() == b"previous output"
        assert temporary.read_bytes() == proof.elf_to_uf2(source)
        original_replace(temporary, destination)

    monkeypatch.setattr(proof.os, "fsync", fsync)
    monkeypatch.setattr(proof.os, "replace", replace)
    assert proof.main([str(path), str(output)]) == 0
    validate_uf2(output.read_bytes())
    assert set(tmp_path.iterdir()) == {path, output}
