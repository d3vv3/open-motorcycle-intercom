#!/usr/bin/env python3
"""Acceptance test for the two-unit OMI desk rig.

Flow (default): optionally force TX in ESP firmware, flash both ESP ports,
run benchmark.py, assert metrics from summary.json, always restore firmware.

Assert-only: python scripts/acceptance.py --assert-only RUN_DIR
"""

from __future__ import annotations

import argparse
import json
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


def assert_run(run_dir: str, esp_ports: list[str], args: argparse.Namespace) -> bool:
    """Check ESP-port metrics in summary.json. Return True when all pass."""
    summary_path = os.path.join(run_dir, "summary.json")
    with open(summary_path, "r", encoding="utf-8") as fh:
        summary = json.load(fh)

    by_port = {entry.get("port"): entry for entry in summary.get("ports", [])}
    rows: list[tuple[str, str, str, str, str]] = []
    ok = True

    for port in esp_ports:
        entry = by_port.get(port)
        if entry is None:
            rows.append((port, "port_in_summary", "present", "absent", "FAIL"))
            ok = False
            continue

        if not entry.get("open_ok", False):
            rows.append((port, "open_ok", "true", "false", "FAIL"))
            ok = False

        # Glitches
        glitches = entry.get("glitch_delta", {}).get("glitches")
        if glitches is None:
            rows.append((port, "glitches", f"<= {args.max_glitches}", "n/a", "SKIP"))
        else:
            passed = glitches <= args.max_glitches
            rows.append(
                (port, "glitches", f"<= {args.max_glitches}", str(glitches),
                 "PASS" if passed else "FAIL")
            )
            ok = ok and passed

        # Effective RX gap percentage
        gap = entry.get("hop_pct", {}).get("esp_e2e_effective_gap_pct")
        if gap is None:
            rows.append(
                (port, "effective_gap_pct", f"<= {args.max_effective_gap_pct}",
                 "n/a", "SKIP")
            )
        else:
            passed = gap <= args.max_effective_gap_pct
            rows.append(
                (port, "effective_gap_pct", f"<= {args.max_effective_gap_pct}",
                 str(gap), "PASS" if passed else "FAIL")
            )
            ok = ok and passed

        # PCM underrun delta (from PIPE pipeline records, when present)
        underruns = [
            (name, rec["delta"]["pcm_underrun"])
            for name, rec in entry.get("pipeline", {}).items()
            if "pcm_underrun" in rec.get("delta", {})
        ]
        if not underruns:
            rows.append((port, "pcm_underrun_delta", "== 0", "n/a", "SKIP"))
        else:
            total = sum(v for _, v in underruns)
            passed = total == 0
            rows.append(
                (port, "pcm_underrun_delta", "== 0", str(total),
                 "PASS" if passed else "FAIL")
            )
            ok = ok and passed

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

    try:
        if force_tx:
            set_force_tx_define(1)
        if do_flash:
            flash_all_esp(args.esp_ports, args.esp_build_dir)
        run_dir = run_benchmark(args.duration, args.ports, args.out_dir)
        return 0 if assert_run(run_dir, args.esp_ports, args) else 1
    finally:
        if force_tx:
            print("RESTORE FORCE_TX_ALWAYS_FOR_TEST=0 and reflash")
            try:
                set_force_tx_define(0)
                flash_all_esp(args.esp_ports, args.esp_build_dir)
            except Exception as exc:
                print(f"ERROR during restore: {exc}")
                print(f"MANUAL ACTION: verify define in {MAIN_C} and reflash ESPs")


if __name__ == "__main__":
    raise SystemExit(main())
