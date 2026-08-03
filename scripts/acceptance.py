#!/usr/bin/env python3
"""Acceptance test for the two-unit OMI desk rig.

Flow (default): optionally force TX in ESP firmware, flash both ESP ports,
run benchmark.py, assert metrics from summary.json, always restore firmware.

Assert-only: python scripts/acceptance.py --assert-only RUN_DIR
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCHMARK = os.path.join(REPO_ROOT, "benchmark.py")
MAIN_C = os.path.join(REPO_ROOT, "main", "main.c")
IDF_EXPORT = "/home/devve/esp/esp-idf/export.sh"

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM2", "/dev/ttyACM4", "/dev/ttyACM5"]
DEFAULT_ESP_PORTS = ["/dev/ttyACM0", "/dev/ttyACM5"]

DEFINE_RE = re.compile(r"(#define\s+FORCE_TX_ALWAYS_FOR_TEST\s+)(\d+)")

FLASH_SETTLE_S = 15


# ---------------------------------------------------------------------------
# Firmware define handling
# ---------------------------------------------------------------------------


def set_force_tx_define(value: int) -> None:
    """Set FORCE_TX_ALWAYS_FOR_TEST in main.c. Verify exactly one substitution."""
    with open(MAIN_C, "r", encoding="utf-8") as fh:
        src = fh.read()
    new_src, count = DEFINE_RE.subn(lambda m: m.group(1) + str(value), src)
    if count != 1:
        raise RuntimeError(
            f"FORCE_TX_ALWAYS_FOR_TEST: expected 1 match in {MAIN_C}, found {count}"
        )
    if new_src != src:
        with open(MAIN_C, "w", encoding="utf-8") as fh:
            fh.write(new_src)
        print(f"SET   FORCE_TX_ALWAYS_FOR_TEST={value} in {MAIN_C}")
    else:
        print(f"KEEP  FORCE_TX_ALWAYS_FOR_TEST already {value}")


def flash_esp(port: str, build_dir: str) -> None:
    """Build (if needed) and flash one ESP port via idf.py."""
    cmd = (
        f"source {IDF_EXPORT} && "
        f"idf.py -B {build_dir} -p {port} flash"
    )
    print(f"FLASH {port} (build dir {build_dir})")
    subprocess.run(["bash", "-c", cmd], cwd=REPO_ROOT, check=True)


def flash_all_esp(esp_ports: list[str], build_dir: str) -> None:
    for port in esp_ports:
        flash_esp(port, build_dir)
    print(f"WAIT  {FLASH_SETTLE_S} s for boards to settle")
    time.sleep(FLASH_SETTLE_S)


def restore_all_esp(esp_ports: list[str], build_dir: str) -> None:
    errors: list[tuple[str, Exception]] = []
    for port in esp_ports:
        try:
            flash_esp(port, build_dir)
        except Exception as exc:
            errors.append((port, exc))
    if errors:
        detail = ", ".join(f"{port}: {exc}" for port, exc in errors)
        raise RuntimeError(f"restore flash failed ({detail})")
    print(f"WAIT  {FLASH_SETTLE_S} s for boards to settle")
    time.sleep(FLASH_SETTLE_S)


# ---------------------------------------------------------------------------
# Benchmark run
# ---------------------------------------------------------------------------


def run_benchmark(duration: int, ports: list[str], out_dir: str) -> str:
    """Run benchmark.py, return the newest run dir with a summary.json."""
    before = set(_run_dirs(out_dir))
    cmd = [
        sys.executable,
        BENCHMARK,
        "--duration",
        str(duration),
        "--out-dir",
        out_dir,
        "--ports",
        *ports,
    ]
    print(f"RUN   {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    new_dirs = [d for d in _run_dirs(out_dir) if d not in before]
    candidates = new_dirs or _run_dirs(out_dir)
    if not candidates:
        raise RuntimeError(f"no run dir with summary.json under {out_dir}")
    return max(candidates, key=os.path.getmtime)


def _run_dirs(out_dir: str) -> list[str]:
    if not os.path.isdir(out_dir):
        return []
    return [
        os.path.join(out_dir, name)
        for name in os.listdir(out_dir)
        if os.path.isfile(os.path.join(out_dir, name, "summary.json"))
    ]


# ---------------------------------------------------------------------------
# Assertions
# ---------------------------------------------------------------------------


def _fmt(value: object) -> str:
    return "n/a" if value is None else str(value)


def _is_finite_number(value: object) -> bool:
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return isinstance(value, float) and math.isfinite(value)


def _is_finite_integer(value: object) -> bool:
    return _is_finite_number(value) and (
        isinstance(value, int) or value.is_integer()
    )


def _nested_value(value: object, *keys: str) -> object:
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def assert_run(run_dir: str, esp_ports: list[str], args: argparse.Namespace) -> bool:
    """Check ESP-port metrics in summary.json. Return True when all pass."""
    summary_path = os.path.join(run_dir, "summary.json")
    rows: list[tuple[str, str, str, str, str]] = []
    ok = True

    try:
        with open(summary_path, "r", encoding="utf-8") as fh:
            summary = json.load(fh)
    except (OSError, ValueError) as exc:
        rows.append(("summary", "summary_json", "valid JSON", str(exc), "FAIL"))
        summary = {}
        ok = False

    if not isinstance(summary, dict):
        rows.append(
            ("summary", "summary_shape", "object", type(summary).__name__, "FAIL")
        )
        ports = []
        ok = False
    else:
        ports = summary.get("ports")
        if not isinstance(ports, list):
            rows.append(
                ("summary", "ports_shape", "array", type(ports).__name__, "FAIL")
            )
            ports = []
            ok = False

    by_port: dict[str, dict[str, object]] = {}
    for index, entry in enumerate(ports):
        if not isinstance(entry, dict) or not isinstance(entry.get("port"), str):
            rows.append(
                ("summary", f"ports[{index}]", "port object", type(entry).__name__, "FAIL")
            )
            ok = False
            continue
        port_name = entry["port"]
        if port_name in by_port:
            rows.append((port_name, "port_in_summary", "unique", "duplicate", "FAIL"))
            ok = False
            continue
        by_port[port_name] = entry

    for port in esp_ports:
        entry = by_port.get(port)
        if entry is None:
            rows.append((port, "port_in_summary", "present", "absent", "FAIL"))
            ok = False
            continue

        open_ok = entry.get("open_ok")
        if open_ok is not True:
            rows.append((port, "open_ok", "true", _fmt(open_ok), "FAIL"))
            ok = False

        # Glitches
        glitches = _nested_value(entry, "glitch_delta", "glitches")
        if _is_finite_integer(glitches) and glitches >= 0:
            passed = glitches <= args.max_glitches
            rows.append(
                (port, "glitches", f"<= {args.max_glitches}", str(glitches),
                 "PASS" if passed else "FAIL")
            )
            ok = ok and passed
        else:
            rows.append(
                (port, "glitches", f"<= {args.max_glitches}", _fmt(glitches), "FAIL")
            )
            ok = False

        # Effective RX gap percentage
        gap = _nested_value(entry, "hop_pct", "esp_e2e_effective_gap_pct")
        if _is_finite_number(gap) and 0 <= gap <= 100:
            passed = gap <= args.max_effective_gap_pct
            rows.append(
                (port, "effective_gap_pct", f"<= {args.max_effective_gap_pct}",
                 str(gap), "PASS" if passed else "FAIL")
            )
            ok = ok and passed
        else:
            rows.append(
                (port, "effective_gap_pct", f"<= {args.max_effective_gap_pct}",
                 _fmt(gap), "FAIL")
            )
            ok = False

        # PCM underrun delta (from PIPE pipeline records)
        pipeline = entry.get("pipeline")
        underruns: list[object] = []
        pipeline_valid = isinstance(pipeline, dict)
        if pipeline_valid:
            for rec in pipeline.values():
                if not isinstance(rec, dict) or not isinstance(rec.get("delta"), dict):
                    pipeline_valid = False
                    continue
                if "pcm_underrun" in rec["delta"]:
                    underruns.append(rec["delta"]["pcm_underrun"])

        values_valid = bool(underruns) and all(
            _is_finite_integer(value) and value >= 0 for value in underruns
        )
        if pipeline_valid and values_valid:
            total = sum(underruns)
            passed = total == 0
            rows.append(
                (port, "pcm_underrun_delta", "== 0", str(total),
                 "PASS" if passed else "FAIL")
            )
            ok = ok and passed
        else:
            rows.append(
                (port, "pcm_underrun_delta", "== 0",
                 _fmt(underruns if underruns else None), "FAIL")
            )
            ok = False

    print(f"\nRun dir: {run_dir}")
    _print_table(rows)
    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
    return ok


def _print_table(rows: list[tuple[str, str, str, str, str]]) -> None:
    header = ("PORT", "METRIC", "LIMIT", "VALUE", "STATUS")
    widths = [
        max(len(header[i]), max((len(r[i]) for r in rows), default=0))
        for i in range(len(header))
    ]
    def line(cells: tuple[str, ...]) -> str:
        return "  ".join(cell.ljust(widths[i]) for i, cell in enumerate(cells))
    print(line(header))
    print(line(tuple("-" * w for w in widths)))
    for row in rows:
        print(line(row))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Two-unit desk-rig acceptance test: flash, benchmark, assert."
    )
    parser.add_argument("--duration", type=int, default=60,
                        help="Benchmark capture duration in seconds (default: 60)")
    parser.add_argument("--ports", nargs="*", default=DEFAULT_PORTS,
                        help=f"Serial ports to capture (default: {' '.join(DEFAULT_PORTS)})")
    parser.add_argument("--esp-ports", nargs="*", default=DEFAULT_ESP_PORTS,
                        help="ESP32-S3 console ports to flash and assert "
                             f"(default: {' '.join(DEFAULT_ESP_PORTS)})")
    parser.add_argument("--out-dir", default="/home/devve/projects/omi-benchmarks",
                        help="Benchmark output directory")
    parser.add_argument("--force-tx", choices=["keep", "on"], default="keep",
                        help="on: set FORCE_TX_ALWAYS_FOR_TEST=1, flash, test, "
                             "restore to 0 and reflash (default: keep)")
    parser.add_argument("--esp-build-dir", default="/home/devve/projects/omi-build-esp",
                        help="idf.py build directory")
    parser.add_argument("--skip-flash", action="store_true",
                        help="Skip firmware edit/build/flash; run benchmark and assert")
    parser.add_argument("--assert-only", metavar="RUN_DIR", default=None,
                        help="Assert against an existing run dir; no flash, no capture")
    parser.add_argument("--max-effective-gap-pct", type=float, default=0.75,
                        help="Max esp_e2e_effective_gap_pct per ESP port (default: 0.75)")
    parser.add_argument("--max-glitches", type=int, default=0,
                        help="Max glitches delta per ESP port (default: 0)")
    args = parser.parse_args()

    if args.assert_only:
        run_dir = args.assert_only
        if not os.path.isfile(os.path.join(run_dir, "summary.json")):
            print(f"ERROR: no summary.json in {run_dir}")
            return 2
        return 0 if assert_run(run_dir, args.esp_ports, args) else 1

    do_flash = not args.skip_flash
    force_tx = args.force_tx == "on" and do_flash

    if args.force_tx == "on" and args.skip_flash:
        print("NOTE  --skip-flash given: --force-tx on has no effect")

    restore_error: Exception | None = None
    try:
        if force_tx:
            set_force_tx_define(1)
        if do_flash:
            flash_all_esp(args.esp_ports, args.esp_build_dir)
        run_dir = run_benchmark(args.duration, args.ports, args.out_dir)
        return 0 if assert_run(run_dir, args.esp_ports, args) else 1
    finally:
        operation_failed = sys.exc_info()[0] is not None
        if force_tx:
            print("RESTORE FORCE_TX_ALWAYS_FOR_TEST=0 and reflash")
            try:
                set_force_tx_define(0)
                restore_all_esp(args.esp_ports, args.esp_build_dir)
            except Exception as exc:
                restore_error = exc
                print(f"ERROR during restore: {exc}")
                print(f"MANUAL ACTION: verify define in {MAIN_C} and reflash ESPs")
        if restore_error is not None and not operation_failed:
            raise RuntimeError("failed to restore normal ESP firmware") from restore_error


if __name__ == "__main__":
    raise SystemExit(main())
