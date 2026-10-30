#!/usr/bin/env python3
"""Validate a compiled nRF52840 proof ELF and convert it to UF2, without flashing.

These structural checks do not establish that the firmware boots or executes.
Only the Python standard library is used; production release tooling is untouched.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import struct
import tempfile


APP_START = 0x27000
# Contracted prototype profile from bootloader source and observed CURRENT.UF2,
# not installed-binary attestation or a universal nRF52840 board layout.
APP_END = 0xEA000
RAM_START = 0x20000000
RAM_END = 0x20040000
ELF_HEADER = struct.Struct("<16sHHIIIIIHHHHHH")
PROGRAM_HEADER = struct.Struct("<8I")
PAYLOAD_SIZE = 256


def _bounded(start: int, size: int, low: int, high: int) -> bool:
    return low <= start < high and size > 0 and start + size <= high


def _no_overlaps(ranges: list[tuple[int, int]], kind: str) -> None:
    ranges.sort()
    if any(right[0] < left[1] for left, right in zip(ranges, ranges[1:])):
        raise ValueError(f"Overlapping {kind} ranges")


def validate_elf(data: bytes) -> bytes:
    """Return initialized flash from APP_START, with gaps and tail padded to 256.

    PT_LOAD physical addresses locate initialized bytes, while virtual addresses
    describe runtime memory. Zero-fill tails are permitted only in RAM.
    """
    if len(data) < ELF_HEADER.size:
        raise ValueError("Truncated ELF header")
    (ident, kind, machine, version, entry, phoff, shoff, flags, ehsize,
     phentsize, phnum, shentsize, shnum, shstrndx) = ELF_HEADER.unpack_from(data)
    # rustc can emit GNU OSABI (3) for bare-metal Cortex-M; OSABI metadata
    # is not a runtime qualification. Machine, EABI, layout, and vectors govern.
    if (ident[:7] != b"\x7fELF\x01\x01\x01" or ident[7] not in (0, 3, 97)
            or ident[8:] != b"\0" * 8):
        raise ValueError("Invalid ELF identification: require ARM ELF32 little endian")
    if kind != 2 or machine != 40 or version != 1:
        raise ValueError("Require executable ARM ELF version 1")
    # ARM EABI5, optionally declaring soft- or hard-float ABI, not both.
    if (flags & ~0x600) != 0x05000000 or (flags & 0x600) == 0x600:
        raise ValueError("Unsupported ARM ELF flags (require EABI5)")
    if ehsize != ELF_HEADER.size or phentsize != PROGRAM_HEADER.size:
        raise ValueError("Invalid ELF header/program header size")
    if not phnum or phnum == 0xFFFF or phoff < ehsize or phoff + phnum * phentsize > len(data):
        raise ValueError("Missing or truncated program header table")
    if shnum:
        if (shentsize != 40 or shoff < ehsize or shoff + shnum * shentsize > len(data)
                or shstrndx >= shnum):
            raise ValueError("Invalid or truncated section header table")
    elif shoff or shstrndx:
        raise ValueError("Unsupported extended section header numbering")
    if not entry & 1 or not APP_START <= (entry & ~1) < APP_END:
        raise ValueError("ELF entry must be Thumb in application flash")

    loads = []
    runtime = []
    executable = []
    for index in range(phnum):
        ptype, offset, vaddr, paddr, filesz, memsz, pflags, align = PROGRAM_HEADER.unpack_from(
            data, phoff + index * phentsize)
        if offset > len(data) or filesz > len(data) - offset:
            raise ValueError("Truncated segment file range")
        if ptype != 1:
            continue
        if not memsz or filesz > memsz:
            raise ValueError("Invalid PT_LOAD zero size or file/memory size")
        if pflags & ~7 or not pflags & 4:
            raise ValueError("Invalid PT_LOAD flags: readable segment required")
        if align not in (0, 1) and (align & (align - 1) or vaddr % align != offset % align):
            raise ValueError("Invalid PT_LOAD alignment")
        in_ram = _bounded(vaddr, memsz, RAM_START, RAM_END)
        in_flash = _bounded(vaddr, memsz, APP_START, APP_END)
        if not (in_ram or in_flash) or (memsz > filesz and not in_ram):
            raise ValueError("PT_LOAD runtime memory outside RAM/application or flash zero-fill")
        if pflags & 1 and (not in_flash or not filesz or vaddr != paddr):
            raise ValueError("PT_LOAD execution must be file-backed application flash")
        if in_flash and vaddr != paddr:
            raise ValueError("Flash runtime and physical load addresses must agree")
        runtime.append((vaddr, vaddr + memsz))
        if filesz:
            if not _bounded(paddr, filesz, APP_START, APP_END):
                raise ValueError(f"PT_LOAD physical bytes outside application [{hex(APP_START)}, {hex(APP_END)})")
            loads.append((paddr, data[offset:offset + filesz]))
            if pflags & 1:
                executable.append((vaddr, data[offset:offset + filesz]))
    if not loads:
        raise ValueError("No file-backed PT_LOAD segments")
    _no_overlaps(runtime, "runtime memory")
    _no_overlaps([(address, address + len(payload)) for address, payload in loads], "physical load")
    loads.sort()
    if loads[0][0] != APP_START or len(loads[0][1]) < 8:
        raise ValueError("Initial vector must be loaded exactly at 0x27000 with eight bytes")
    sp, reset = struct.unpack_from("<2I", loads[0][1])
    if sp % 8 or not RAM_START <= sp <= RAM_END:
        raise ValueError("Initial stack pointer must be 8-byte aligned within RAM (top inclusive)")
    if not reset & 1 or not APP_START <= (reset & ~1) < APP_END:
        raise ValueError("Reset vector must be Thumb in application flash")
    if reset != entry:
        raise ValueError("Reset vector and ELF entry disagree")
    instruction_address = entry & ~1
    instruction = next((payload[instruction_address - start:]
                        for start, payload in executable
                        if start <= instruction_address and instruction_address + 2 <= start + len(payload)), None)
    if instruction is None:
        raise ValueError("Entry/reset must point to file-backed executable flash")
    halfword = struct.unpack_from("<H", instruction)[0]
    instruction_size = 4 if halfword >> 11 in (0b11101, 0b11110, 0b11111) else 2
    if len(instruction) < instruction_size:
        raise ValueError("Entry/reset Thumb instruction is truncated in executable flash")

    end = loads[-1][0] + len(loads[-1][1])
    rounded_end = APP_START + ((end - APP_START + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE) * PAYLOAD_SIZE
    if rounded_end > APP_END:
        raise ValueError("Rounded UF2 payload exceeds application boundary")
    image = bytearray(b"\xff" * (rounded_end - APP_START))
    for address, payload in loads:
        image[address - APP_START:address - APP_START + len(payload)] = payload
    return bytes(image)


def elf_to_uf2(data: bytes) -> bytes:
    """Validate the entire ELF before constructing family-tagged flash blocks."""
    image = validate_elf(data)
    count = len(image) // PAYLOAD_SIZE
    result = bytearray()
    for number in range(count):
        block = bytearray(512)
        struct.pack_into("<8I", block, 0, 0x0A324655, 0x9E5D5157, 0x2000,
                         APP_START + number * PAYLOAD_SIZE, PAYLOAD_SIZE,
                         number, count, 0xADA52840)
        block[32:32 + PAYLOAD_SIZE] = image[number * PAYLOAD_SIZE:(number + 1) * PAYLOAD_SIZE]
        struct.pack_into("<I", block, 508, 0x0AB16F30)
        result.extend(block)
    return bytes(result)


def _write_atomic(destination: Path, data: bytes) -> None:
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=destination.parent,
                                         prefix=f".{destination.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            if stream.write(data) != len(data):
                raise OSError("Incomplete UF2 write")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path, help="Compiled ARM ELF proof input")
    parser.add_argument("uf2", type=Path, help="Output UF2 file (no deployment)")
    args = parser.parse_args(argv)
    try:
        if (args.elf.resolve() == args.uf2.resolve()
                or (args.uf2.exists() and args.elf.samefile(args.uf2))):
            raise ValueError("Input ELF and output UF2 must be different files")
        converted = elf_to_uf2(args.elf.read_bytes())
        _write_atomic(args.uf2, converted)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Proof conversion failed: {error}\n")
    print(f"Validated ELF structure and wrote UF2: {args.uf2}; execution not verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
