#!/usr/bin/env -S uv run
"""OMI Serial Benchmark - capture serial logs from all boards and summarize mesh metrics.

Run with uv: `uv run benchmark.py --duration 90`
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from datetime import datetime

from benchtool.capture import (
    PortReader,
    _discover_ports,
    _reconnect_candidates,
    _sanitize_port,
)
from benchtool.health import _health_line
from benchtool.parsers import parse_pipeline_logfmt
from benchtool.report import (
    _build_port_json,
    _report_lines_for_port,
    print_quick_summary,
    write_human_report,
    write_summary_json,
)
from benchtool.series import _u32_cumulative_series_delta
from benchtool.stats import (
    PortStats,
    _nrf_starvation_delta,
    compute_correlated_delivery,
    compute_hop_pct,
)

__all__ = [
    "PortReader",
    "PortStats",
    "_build_port_json",
    "_health_line",
    "_nrf_starvation_delta",
    "_reconnect_candidates",
    "_report_lines_for_port",
    "_u32_cumulative_series_delta",
    "compute_correlated_delivery",
    "compute_hop_pct",
    "parse_pipeline_logfmt",
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture serial logs from all boards and summarize mesh metrics."
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=90,
        help="Capture duration in seconds (default: 90)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for all ports (default: 115200)",
    )
    parser.add_argument(
        "--ports",
        nargs="*",
        help="Explicit serial ports; auto-discovers /dev/ttyACM* and /dev/ttyUSB* "
        "if omitted",
    )
    parser.add_argument(
        "--out-dir",
        default="benchmark_runs",
        help="Output directory for benchmark runs (default: benchmark_runs)",
    )
    args = parser.parse_args()

    ports = args.ports or _discover_ports()
    if not ports:
        print("No serial ports found under /dev/ttyACM* or /dev/ttyUSB*.")
        return 1

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.out_dir, run_id)
    raw_dir = os.path.join(run_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    stop_event = threading.Event()
    all_stats = [PortStats(port=p) for p in ports]
    readers = []

    for s in all_stats:
        out_path = os.path.join(raw_dir, f"{_sanitize_port(s.port)}.log")
        reader = PortReader(s.port, args.baud, stop_event, out_path, s)
        readers.append(reader)
        reader.start()

    started_at = time.time()
    print(f"Capturing {len(ports)} ports for {args.duration}s at {args.baud} baud...")
    print("Ports:")
    for p in ports:
        print(f"  - {p}")

    time.sleep(args.duration)
    stop_event.set()
    for reader in readers:
        reader.join(timeout=2.0)
    ended_at = time.time()

    summary_json = os.path.join(run_dir, "summary.json")
    report_txt = os.path.join(run_dir, "report.txt")
    write_summary_json(summary_json, started_at, ended_at, all_stats, args.duration)
    write_human_report(report_txt, run_dir, all_stats, args.duration)

    print("Capture complete.")
    print(f"Run directory: {run_dir}")
    print(f"Summary JSON: {summary_json}")
    print(f"Report: {report_txt}")
    print_quick_summary(all_stats, args.duration)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
