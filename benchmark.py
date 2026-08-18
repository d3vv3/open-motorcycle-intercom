#!/usr/bin/env python3
"""OMI Serial Benchmark — capture serial logs from all boards and summarize mesh metrics."""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shlex
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

import serial
from serial.tools import list_ports

_PIPE_GAUGE_KEYS = {
    "v",
    "node",
    "q_depth",
    "poll_max_us",
    "tx_wait_avg_us",
    "tx_wait_max_us",
    "rx_pause_max_us",
    "correction_applied_us",
    "correction_pending_us",
    "last_correction_us",
    "commanded_period_us",
    "measured_interval_us",
    "callback_jitter_us",
    "callback_jitter_max_us",
    "sync_frame_diff",
    "sync_phase_us",
    "age_ms",
    "max_age_ms",
    "gen",
    "state",
    "exp_gen",
    "exp_state",
    "rx_sources",
    "asrc_ppm",
    "asrc_abs_max_ppm",
    "asrc_recovery",
    "bundle_max_bytes",
    "tx_duration_max_us",
}

_PIPE_SIGNED_KEYS = {
    "correction_applied_us",
    "correction_pending_us",
    "last_correction_us",
    "callback_jitter_us",
    "sync_frame_diff",
    "sync_phase_us",
    "asrc_ppm",
}

_IDENTITY_KEYS = {
    "v",
    "node",
    "id",
    "session",
    "src",
    "src_node",
    "dst",
    "dst_node",
    "peer",
    "peer_node",
}

_PIPE_TX_COUNTERS = ("sent", "tx", "tx_ok", "rf_tx_ok", "spi_out_ok")
_PIPE_RX_COUNTERS = ("received", "rx", "rx_ok", "rf_rx_ok", "ingress_ok")

_SNAPSHOT_GAUGE_KEYS = {
    "mesh": {"q"},
    "spi": {"poll_min", "poll_avg", "poll_max"},
    "adaptive": {"sources"},
    "latency": {"lat_avg_ms", "lat_max_ms"},
    "encode": {"enc_avg_us", "enc_max_us"},
    "decode": {"dec_avg_us", "dec_max_us"},
    "tx_pipeline": {"tx_pipe_avg_us", "tx_pipe_max_us"},
    "rx_pipeline": {"rx_pipe_avg_us", "rx_pipe_max_us"},
    "vox": {"vox_active"},
    "rx_depth": {"rx_q_min", "rx_q_avg", "rx_q_max", "rx_q_total"},
    "e2e_esp": {"effective_gap"},
    "atune": {
        "r",
        "q",
        "under_d",
        "skip_pct",
        "ws_delta",
        "ws_corr",
        "ws_drift",
        "td_sum",
        "td_pend",
        "td_last",
        "td_cmd",
        "td_meas",
        "td_jit",
        "td_jit_max",
    },
}

# =============================================================================
# Regex patterns
# =============================================================================

MESH_RE = re.compile(
    r"tx=(?P<tx>\d+)\(err=(?P<tx_err>\d+)\) rx=(?P<rx>\d+) drop=(?P<drop>\d+) "
    r"fwd=(?P<fwd>\d+) \| spi_in=(?P<spi_in>\d+) overwr=(?P<overwr>\d+) "
    r"(?:(?:starve=(?P<starve>\d+)(?: drain=(?P<drain>\d+))?)|under=(?P<under>\d+)) "
    r"q=(?P<q>\d+)"
)

SPI_RE = re.compile(
    r"poll_us min/avg/max=(?P<poll_min>\d+)/(?P<poll_avg>\d+)/(?P<poll_max>\d+) "
    r"q_over=(?P<q_over>\d+) seq_gap=(?P<seq_gap>\d+) crc_fail=(?P<crc_fail>\d+)"
)

ESP_AUDIO_PIPE_RE = re.compile(
    r"Audio pipe: tx_queued=(?P<tx_queued>\d+) tx_overwr=(?P<tx_overwr>\d+) "
    r"rx_from_nrf=(?P<rx_from_nrf>\d+)"
    r"(?: tx_q=(?P<tx_q>\d+) ctrl_pending=(?P<ctrl_pending>\d+) "
    r"bad_sync=(?P<bad_sync>\d+) bad_len=(?P<bad_len>\d+) "
    r"trunc=(?P<trunc>\d+) crc_fail=(?P<crc_fail>\d+) seq_gap=(?P<seq_gap>\d+))?"
)

ESP_CRC_WARN_RE = re.compile(r"crc_fail=(?P<crc_fail>\d+)")

ESP_AUDIO_GLITCH_RE = re.compile(
    r"Glitches:\s*(?P<glitches>\d+)\s*\(rx_und=(?P<rx_und>\d+)\s+"
    r"i2s_inc=(?P<i2s_inc>\d+)\),\s*ADC overruns:\s*(?P<adc_overruns>\d+)"
)

ESP_AUDIO_CONCEAL_RE = re.compile(
    r"Concealment:\s*plc=(?P<plc>\d+)\s*grace_empty=(?P<grace_empty>\d+)"
    r"(?:\s*conceal=(?P<conceal>\d+)\s*seq_gap=(?P<seq_gap_frames>\d+)\s*"
    r"seq_reset=(?P<seq_reset>\d+)\s*seq_stale=(?P<seq_stale>\d+))?"
)

ESP_AUDIO_ADAPTIVE_RE = re.compile(
    r"Adaptive playout:\s*hold=(?P<hold>\d+)\s*catchup=(?P<catchup>\d+)\s*"
    r"sources=(?P<sources>\d+)"
)

ESP_AUDIO_LATENCY_RE = re.compile(
    r"Latency:\s*avg=(?P<lat_avg_ms>\d+)\s*ms,\s*max=(?P<lat_max_ms>\d+)\s*ms"
)

ESP_AUDIO_ENCODE_TIME_RE = re.compile(
    r"Encode time:\s*avg=(?P<enc_avg_us>\d+)\s*us,\s*max=(?P<enc_max_us>\d+)\s*us"
)

ESP_AUDIO_DECODE_TIME_RE = re.compile(
    r"Decode time:\s*avg=(?P<dec_avg_us>\d+)\s*us,\s*max=(?P<dec_max_us>\d+)\s*us"
)

ESP_AUDIO_TX_PIPE_RE = re.compile(
    r"TX pipeline:\s*avg=(?P<tx_pipe_avg_us>\d+)\s*us,\s*max=(?P<tx_pipe_max_us>\d+)\s*us"
)

ESP_AUDIO_RX_PIPE_RE = re.compile(
    r"RX pipeline:\s*avg=(?P<rx_pipe_avg_us>\d+)\s*us,\s*max=(?P<rx_pipe_max_us>\d+)\s*us"
)

ESP_AUDIO_VOX_RE = re.compile(
    r"VOX activations:\s*(?P<vox_activations>\d+)\s*"
    r"\(active:\s*(?P<vox_active>YES|yes|no)\)"
)

ESP_AUDIO_RX_DEPTH_RE = re.compile(
    r"RX queue depth/source:\s*min=(?P<rx_q_min>\d+)\s*avg=(?P<rx_q_avg>\d+)\s*"
    r"max=(?P<rx_q_max>\d+)\s*\(total now=(?P<rx_q_total>\d+)\)"
)

ESP_AUDIO_ENCODED_RE = re.compile(r"Encoded:\s*(?P<encoded_frames>\d+)\s*frames")
ESP_AUDIO_DECODED_RE = re.compile(r"Decoded:\s*(?P<decoded_frames>\d+)\s*frames")

NRF_UFLOW_RE = re.compile(
    r"\[UFLOW\].*under=(?P<under>\d+).*reason=(?P<reason>[A-Za-z0-9_\-]+)"
)

NRF_TXSTARVE_RE = re.compile(r"\[TXSTARVE\]\s+total=(?P<total>\d+)\b")

NRF_AUDIO_RX_RE = re.compile(r"uart_bridge: Audio:\s*(?P<rx_pkts>\d+)\s*pkts received")

NRF_ATUNE_RE = re.compile(
    r"\[ATUNE\]\s+r=(?P<r>\d+)\s+id=(?P<id>\d+)\s+q=(?P<q>\d+)\s+"
    r"(?:under_d=(?P<under_d>\d+)\s+)?skip=(?P<skip>\d+)/(?P<ticks>\d+)\s+"
    r"ws_e=(?P<ws_edges>\d+)\s+ws_n=(?P<ws_samples>\d+)\s+"
    r"ws_ok=(?P<ws_valid>\d+)\s+ws_no=(?P<ws_no_signal>\d+)\s+"
    r"ws_rej=(?P<ws_rejected>\d+)\s+ws_delta=(?P<ws_delta>\d+)\s+"
    r"ws_c=(?P<ws_corr>-?\d+)\s+ws_d=(?P<ws_drift>-?\d+)\s+"
    r"td_req=(?P<td_req>\d+)\s+td_app=(?P<td_app>\d+)\s+"
    r"td_sum=(?P<td_sum>-?\d+)\s+td_pend=(?P<td_pend>-?\d+)\s+"
    r"td_last=(?P<td_last>-?\d+)\s+td_cmd=(?P<td_cmd>\d+)\s+"
    r"td_meas=(?P<td_meas>\d+)\s+td_jit=(?P<td_jit>-?\d+)\s+"
    r"td_jit_max=(?P<td_jit_max>\d+)"
)

ESP_E2E_RE = re.compile(
    r"\[E2E_ESP\]\s*tx=(?P<tx>\d+)\s*rx=(?P<rx>\d+)\s*gap_evt=(?P<gap_evt>\d+)\s*"
    r"gap_fr=(?P<gap_fr>\d+)\s*reset_evt=(?P<reset_evt>\d+)"
    r"(?:\s*recovered=(?P<recovered>\d+)\s*effective_gap=(?P<effective_gap>\d+))?"
)

NRF_E2E_RE = re.compile(
    r"\[E2E_NRF\]\s*id=(?P<id>\d+)\s*spi_in=(?P<spi_in>\d+)\s*"
    r"spi_gap=(?P<spi_gap_evt>\d+)/(?P<spi_gap_fr>\d+)\s*"
    r"spi_reset=(?P<spi_reset_evt>\d+)\s*rf_tx=(?P<rf_tx>\d+)\s*"
    r"rf_rx=(?P<rf_rx>\d+)\s*rf_gap=(?P<rf_gap_evt>\d+)/(?P<rf_gap_fr>\d+)\s*"
    r"rf_reset=(?P<rf_reset_evt>\d+)\s*spi_out=(?P<spi_out>\d+)"
)

# All regex + field-name pairs for simple "first/last int dict" parsing.
_SIMPLE_PARSERS: list[tuple[re.Pattern, str]] = [
    (MESH_RE, "mesh"),
    (NRF_TXSTARVE_RE, "txstarve"),
    (SPI_RE, "spi"),
    (ESP_AUDIO_PIPE_RE, "audio_pipe"),
    (ESP_AUDIO_GLITCH_RE, "glitch"),
    (ESP_AUDIO_CONCEAL_RE, "conceal"),
    (ESP_AUDIO_ADAPTIVE_RE, "adaptive"),
    (ESP_AUDIO_LATENCY_RE, "latency"),
    (ESP_AUDIO_ENCODE_TIME_RE, "encode"),
    (ESP_AUDIO_DECODE_TIME_RE, "decode"),
    (ESP_AUDIO_TX_PIPE_RE, "tx_pipeline"),
    (ESP_AUDIO_RX_PIPE_RE, "rx_pipeline"),
    (ESP_AUDIO_RX_DEPTH_RE, "rx_depth"),
    (ESP_E2E_RE, "e2e_esp"),
    (NRF_E2E_RE, "e2e_nrf"),
]

# =============================================================================
# Helpers
# =============================================================================


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _sanitize_port(port: str) -> str:
    return port.replace("/", "_")


def parse_pipeline_logfmt(line: str) -> dict[str, int | str] | None:
    """Parse one versioned PIPE logfmt record from a prefixed serial line."""
    marker = "PIPE "
    start = line.find(marker)
    if start < 0:
        return None
    record: dict[str, int | str] = {}
    try:
        tokens = shlex.split(line[start + len(marker) :])
    except ValueError:
        return None
    for token in tokens:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if not key:
            continue
        try:
            parsed = int(value, 10)
            if parsed < 0 and key not in _PIPE_SIGNED_KEYS:
                return None
            record[key] = parsed
        except ValueError:
            record[key] = value
    if record.get("v") != 1 or "dev" not in record or "stage" not in record:
        return None
    return record


def _reset_aware_delta(
    samples: list[dict], excluded: set[str] | frozenset[str] = frozenset()
) -> tuple[dict[str, int], dict[str, int]]:
    """Sum cumulative-counter increments without spanning counter resets."""
    if len(samples) < 2:
        return {}, {}
    out: dict[str, int] = {}
    resets: dict[str, int] = {}
    keys = set.union(*(set(sample) for sample in samples)) - excluded
    for key in sorted(keys):
        try:
            values = [int(sample[key]) for sample in samples if key in sample]
        except (TypeError, ValueError):
            continue
        if len(values) < 2:
            continue
        total = 0
        reset_count = 0
        for previous, current in zip(values, values[1:]):
            if current >= previous:
                total += current - previous
            else:
                # The new epoch starts at zero; count only its observed value.
                total += current
                reset_count += 1
        out[key] = total
        if reset_count:
            resets[key] = reset_count
    return out, resets


def _dict_delta(first: dict, last: dict) -> dict:
    """Compute a reset-aware delta between two cumulative snapshots."""
    if not first or not last:
        return {}
    return _reset_aware_delta([first, last], _IDENTITY_KEYS)[0]


def _pipeline_delta(first: dict, last: dict) -> dict:
    return _reset_aware_delta(
        [first, last], _PIPE_GAUGE_KEYS | _IDENTITY_KEYS
    )[0]


def _scalar_delta(first: int | None, last: int | None) -> int | None:
    if first is None or last is None:
        return None
    return last - first if last >= first else last


def _scalar_series_delta(values: list[int]) -> int | None:
    if len(values) < 2:
        return None
    return _reset_aware_delta([{"value": value} for value in values])[0]["value"]


_U32_MAX = (1 << 32) - 1
_U32_ROLLOVER_HIGH_WATER = 0xF0000000
_U32_ROLLOVER_LOW_WATER = 0x0FFFFFFF


def _u32_cumulative_series_delta(values: list[int]) -> tuple[int | None, int]:
    """Sum a uint32 cumulative series and return its true reset count."""
    if len(values) < 2:
        return None, 0
    total = 0
    reset_count = 0
    for previous, current in zip(values, values[1:]):
        if current >= previous:
            total += current - previous
        elif (
            previous >= _U32_ROLLOVER_HIGH_WATER
            and current <= _U32_ROLLOVER_LOW_WATER
        ):
            total += (_U32_MAX - previous) + 1 + current
        else:
            # A non-boundary decrease starts a new counter epoch.
            total += current
            reset_count += 1
    return total, reset_count


def _rate_per_min(value: int | None, duration_s: int) -> float | None:
    if value is None or duration_s <= 0:
        return None
    return round((value * 60.0) / float(duration_s), 2)


def _pct(numerator: float, denominator: float) -> float | None:
    """Safe percentage: returns None when denominator is non-positive."""
    try:
        d = float(denominator)
        return round(float(numerator) * 100.0 / d, 2) if d > 0 else None
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def _discover_ports() -> list[str]:
    return sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))


def _serial_identity(port: str) -> tuple[str | None, int | None, int | None]:
    """Return stable USB identity fields for a serial device when available."""
    for info in list_ports.comports():
        if info.device == port:
            return info.serial_number, info.vid, info.pid
    return None, None, None


def _reconnect_candidates(
    original_port: str, identity: tuple[str | None, int | None, int | None]
) -> list[str]:
    """Find the original path or a re-enumerated path for the same USB device."""
    serial_number, vid, pid = identity
    if not serial_number:
        return [original_port]
    infos = list(list_ports.comports())
    original_info = next((info for info in infos if info.device == original_port), None)
    candidates = []
    if original_info is None or (
        original_info.serial_number == serial_number
        and original_info.vid == vid
        and original_info.pid == pid
    ):
        candidates.append(original_port)
    for info in infos:
        if (
            info.device != original_port
            and info.serial_number == serial_number
            and info.vid == vid
            and info.pid == pid
        ):
            candidates.append(info.device)
    return candidates


# =============================================================================
# Data model
# =============================================================================


@dataclass
class PortStats:
    """Accumulator for a single serial port's parsed metrics."""

    port: str

    # Connection
    open_ok: bool = False
    open_error: str | None = None
    reconnects: int = 0
    lines: int = 0
    bytes_rx: int = 0

    # Sample counts
    mesh_samples: int = 0
    spi_samples: int = 0
    audio_pipe_samples: int = 0
    glitch_samples: int = 0
    conceal_samples: int = 0
    adaptive_samples: int = 0
    latency_samples: int = 0
    encode_samples: int = 0
    decode_samples: int = 0
    tx_pipeline_samples: int = 0
    rx_pipeline_samples: int = 0
    vox_samples: int = 0
    rx_depth_samples: int = 0
    frame_counts_samples: int = 0
    txstarve_samples: int = 0
    uflow_events: int = 0
    nrf_audio_samples: int = 0
    atune_samples: int = 0
    e2e_esp_samples: int = 0
    e2e_nrf_samples: int = 0

    # First / last snapshots (dict[str, int])
    first_mesh: dict = field(default_factory=dict)
    last_mesh: dict = field(default_factory=dict)
    first_spi: dict = field(default_factory=dict)
    last_spi: dict = field(default_factory=dict)
    first_audio_pipe: dict = field(default_factory=dict)
    last_audio_pipe: dict = field(default_factory=dict)
    first_glitch: dict = field(default_factory=dict)
    last_glitch: dict = field(default_factory=dict)
    first_conceal: dict = field(default_factory=dict)
    last_conceal: dict = field(default_factory=dict)
    first_adaptive: dict = field(default_factory=dict)
    last_adaptive: dict = field(default_factory=dict)
    first_latency: dict = field(default_factory=dict)
    last_latency: dict = field(default_factory=dict)
    first_encode: dict = field(default_factory=dict)
    last_encode: dict = field(default_factory=dict)
    first_decode: dict = field(default_factory=dict)
    last_decode: dict = field(default_factory=dict)
    first_tx_pipeline: dict = field(default_factory=dict)
    last_tx_pipeline: dict = field(default_factory=dict)
    first_rx_pipeline: dict = field(default_factory=dict)
    last_rx_pipeline: dict = field(default_factory=dict)
    first_vox: dict = field(default_factory=dict)
    last_vox: dict = field(default_factory=dict)
    first_rx_depth: dict = field(default_factory=dict)
    last_rx_depth: dict = field(default_factory=dict)
    first_frame_counts: dict = field(default_factory=dict)
    last_frame_counts: dict = field(default_factory=dict)
    first_txstarve: dict = field(default_factory=dict)
    last_txstarve: dict = field(default_factory=dict)
    first_e2e_esp: dict = field(default_factory=dict)
    last_e2e_esp: dict = field(default_factory=dict)
    first_e2e_nrf: dict = field(default_factory=dict)
    last_e2e_nrf: dict = field(default_factory=dict)
    first_atune: dict = field(default_factory=dict)
    last_atune: dict = field(default_factory=dict)
    first_pipe: dict[str, dict] = field(default_factory=dict)
    last_pipe: dict[str, dict] = field(default_factory=dict)
    pipe_samples: dict[str, int] = field(default_factory=dict)
    pipe_history: dict[str, list[dict]] = field(default_factory=dict)
    pipe_identity: dict[str, dict[str, int | str]] = field(default_factory=dict)
    sample_history: dict[str, list[dict]] = field(default_factory=dict)

    # Scalars
    latency_max_peak_ms: int = 0
    first_uflow_under: int | None = None
    last_uflow_under: int | None = None
    first_nrf_audio_rx_pkts: int | None = None
    last_nrf_audio_rx_pkts: int | None = None
    uflow_under_history: list[int] = field(default_factory=list)
    nrf_audio_rx_history: list[int] = field(default_factory=list)
    last_crc_warn: int | None = None

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    def _record(self, name: str, values: dict) -> None:
        """Record a first/last sample and bump the sample counter."""
        first_attr = f"first_{name}"
        last_attr = f"last_{name}"
        count_attr = f"{name}_samples"
        if not getattr(self, first_attr):
            setattr(self, first_attr, values)
        setattr(self, last_attr, values)
        setattr(self, count_attr, getattr(self, count_attr) + 1)
        self.sample_history.setdefault(name, []).append(values.copy())

    def delta(self, name: str) -> dict:
        """Return the integer delta between first and last for *name*."""
        history = self.sample_history.get(name)
        excluded = _IDENTITY_KEYS | _SNAPSHOT_GAUGE_KEYS.get(name, set())
        if history:
            return _reset_aware_delta(history, excluded)[0]
        return _reset_aware_delta(
            [getattr(self, f"first_{name}"), getattr(self, f"last_{name}")],
            excluded,
        )[0]

    def resets(self, name: str) -> dict[str, int]:
        history = self.sample_history.get(name)
        excluded = _IDENTITY_KEYS | _SNAPSHOT_GAUGE_KEYS.get(name, set())
        if history:
            return _reset_aware_delta(history, excluded)[1]
        first = getattr(self, f"first_{name}")
        last = getattr(self, f"last_{name}")
        return _reset_aware_delta([first, last], excluded)[1]


def _cumulative_samples(s: PortStats, name: str, key: str) -> list[dict]:
    history = s.sample_history.get(name)
    if history:
        return [sample for sample in history if key in sample]
    return [
        sample
        for sample in (getattr(s, f"first_{name}"), getattr(s, f"last_{name}"))
        if key in sample
    ]


def _nrf_starvation_source(s: PortStats) -> tuple[list[dict], str] | None:
    """Select a cumulative series with enough samples to calculate a delta."""
    mesh_samples = _cumulative_samples(s, "mesh", "starve")
    if len(mesh_samples) >= 2:
        return mesh_samples, "starve"

    txstarve_samples = _cumulative_samples(s, "txstarve", "total")
    if len(txstarve_samples) >= 2:
        return txstarve_samples, "total"

    return None


def _nrf_starvation_delta(s: PortStats) -> int | None:
    source = _nrf_starvation_source(s)
    if source is None:
        return None
    samples, key = source
    delta, _ = _u32_cumulative_series_delta([int(sample[key]) for sample in samples])
    return delta


def _mesh_starvation_series(s: PortStats) -> tuple[int | None, int]:
    samples = _cumulative_samples(s, "mesh", "starve")
    return _u32_cumulative_series_delta([int(sample["starve"]) for sample in samples])


def _nrf_starvation_total(s: PortStats) -> int | None:
    source = _nrf_starvation_source(s)
    if source is not None:
        samples, key = source
        return int(samples[-1][key])
    if "starve" in s.last_mesh:
        return s.last_mesh["starve"]
    if s.last_txstarve:
        return s.last_txstarve["total"]
    return None


# =============================================================================
# Hop-percentage computation
# =============================================================================


def compute_hop_pct(s: PortStats) -> dict[str, float | None]:
    """Derive stage-local loss percentages; TX/RX delivery needs two endpoints."""
    out: dict[str, float | None] = {}

    e2e_esp_d = s.delta("e2e_esp")
    e2e_nrf_d = s.delta("e2e_nrf")
    # These gap counters are local to the same receive stage and denominator.
    if e2e_esp_d:
        raw_gaps = e2e_esp_d.get("gap_fr", 0)
        effective_gaps = max(raw_gaps - e2e_esp_d.get("recovered", 0), 0)
        received = e2e_esp_d.get("rx", 0)
        out["esp_e2e_raw_gap_pct"] = _pct(raw_gaps, received + raw_gaps)
        out["esp_e2e_effective_gap_pct"] = _pct(
            effective_gaps, received + effective_gaps
        )
        out["esp_e2e_gap_pct"] = out["esp_e2e_effective_gap_pct"]

    # nRF per-hop
    if e2e_nrf_d:
        spi_in = e2e_nrf_d.get("spi_in", 0)
        spi_gaps = e2e_nrf_d.get("spi_gap_fr", 0)
        rf_gaps = e2e_nrf_d.get("rf_gap_fr", 0)
        out["nrf_spi_gap_pct"] = _pct(spi_gaps, spi_in + spi_gaps)
        out["nrf_rf_gap_pct"] = _pct(
            rf_gaps, e2e_nrf_d.get("rf_rx", 0) + rf_gaps
        )

    return out


def _pipe_key(record: dict[str, int | str]) -> str:
    key = f"{record['dev']}:{record['stage']}:{record.get('node', 'na')}"
    for field_name in ("session", "src_node", "dst_node", "peer", "peer_node"):
        if field_name in record:
            key += f":{field_name}={record[field_name]}"
    return key


def _endpoint(record: dict, direction: str) -> tuple[Any, Any] | None:
    node = record.get("node")
    peer = record.get("peer_node", record.get("peer"))
    source = record.get("src_node", record.get("src"))
    destination = record.get("dst_node", record.get("dst"))
    if direction == "tx":
        source = source if source is not None else node
        destination = destination if destination is not None else peer
    else:
        source = source if source is not None else peer
        destination = destination if destination is not None else node
    if source is None or destination is None:
        return None
    if source == destination:
        return None
    return source, destination


def _counter_value(delta: dict[str, int], aliases: tuple[str, ...]) -> tuple[str, int] | None:
    for name in aliases:
        if name in delta:
            return name, delta[name]
    return None


def compute_correlated_delivery(all_stats: list[PortStats]) -> dict[str, Any]:
    """Correlate PIPE TX/RX counters only with explicit session and endpoints."""
    transmitters: dict[tuple[Any, Any, Any, str], list[dict]] = {}
    receivers: dict[tuple[Any, Any, Any, str], list[dict]] = {}

    for stats in all_stats:
        for key, identity in stats.pipe_identity.items():
            if "session" not in identity:
                continue
            stage = str(identity.get("stage", ""))
            semantic = re.sub(r"(?:[_-](?:tx|rx))$", "", stage)
            history = stats.pipe_history.get(key, [])
            delta, resets = _reset_aware_delta(
                history, _PIPE_GAUGE_KEYS | _IDENTITY_KEYS
            )
            tx = _counter_value(delta, _PIPE_TX_COUNTERS)
            rx = _counter_value(delta, _PIPE_RX_COUNTERS)
            if tx is not None:
                endpoints = _endpoint(identity, "tx")
                if endpoints:
                    link = (identity["session"], endpoints[0], endpoints[1], semantic)
                    transmitters.setdefault(link, []).append(
                        {"port": stats.port, "counter": tx[0], "value": tx[1], "resets": resets}
                    )
            if rx is not None:
                endpoints = _endpoint(identity, "rx")
                if endpoints:
                    link = (identity["session"], endpoints[0], endpoints[1], semantic)
                    receivers.setdefault(link, []).append(
                        {"port": stats.port, "counter": rx[0], "value": rx[1], "resets": resets}
                    )

    links: list[dict[str, Any]] = []
    for link in sorted(set(transmitters) & set(receivers), key=str):
        tx_entries = transmitters[link]
        rx_entries = receivers[link]
        if len(tx_entries) != 1 or len(rx_entries) != 1:
            continue
        tx, rx = tx_entries[0], rx_entries[0]
        status = "ok"
        percentage = _pct(rx["value"], tx["value"])
        reset_in_window = (
            tx["counter"] in tx["resets"] or rx["counter"] in rx["resets"]
        )
        if reset_in_window or (percentage is not None and percentage > 100.0):
            status = "inconsistent correlated data"
            percentage = None
        links.append(
            {
                "session": link[0],
                "sender_node": link[1],
                "receiver_node": link[2],
                "stage": link[3],
                "sender_port": tx["port"],
                "receiver_port": rx["port"],
                "sent": tx["value"],
                "received": rx["value"],
                "delivery_pct": percentage,
                "status": status,
            }
        )

    if not links:
        return {
            "status": "insufficient correlated data",
            "reason": "PIPE records require matching session, sender, receiver, and stage semantics",
            "links": [],
        }
    overall_status = "ok" if all(link["status"] == "ok" for link in links) else "inconsistent correlated data"
    return {"status": overall_status, "links": links}


# =============================================================================
# Serial reader thread
# =============================================================================


class PortReader(threading.Thread):
    """Read serial data in background, parse metrics into PortStats."""

    def __init__(
        self,
        port: str,
        baud: int,
        stop_event: threading.Event,
        out_path: str,
        stats: PortStats,
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_event = stop_event
        self.out_path = out_path
        self.stats = stats

    def run(self) -> None:
        identity = _serial_identity(self.port)
        ser = None
        connected_once = False
        with open(self.out_path, "w", encoding="utf-8", errors="replace") as fh:
            while not self.stop_event.is_set():
                if ser is None:
                    last_error = None
                    for candidate in _reconnect_candidates(self.port, identity):
                        try:
                            ser = serial.Serial(candidate, self.baud, timeout=0.2)
                            self.stats.open_ok = True
                            self.stats.open_error = None
                            if connected_once:
                                self.stats.reconnects += 1
                            connected_once = True
                            break
                        except Exception as exc:
                            last_error = exc
                    if ser is None:
                        self.stats.open_error = f"Open error: {last_error}"
                        self.stop_event.wait(0.25)
                        continue
                try:
                    raw = ser.readline()
                except Exception as exc:
                    self.stats.open_error = f"Read error: {exc}"
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                    self.stop_event.wait(0.1)
                    continue
                if not raw:
                    continue

                self.stats.bytes_rx += len(raw)
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                self.stats.lines += 1
                fh.write(f"{_now_iso()} {line}\n")
                self._parse_line(line)
        if ser is not None:
            ser.close()

    # ------------------------------------------------------------------
    # Line parsing
    # ------------------------------------------------------------------

    def _parse_line(self, line: str) -> None:
        s = self.stats

        pipe = parse_pipeline_logfmt(line)
        if pipe is not None:
            name = _pipe_key(pipe)
            values = {k: v for k, v in pipe.items() if isinstance(v, int)}
            if name not in s.first_pipe:
                s.first_pipe[name] = values.copy()
            s.last_pipe[name] = values
            s.pipe_samples[name] = s.pipe_samples.get(name, 0) + 1
            s.pipe_history.setdefault(name, []).append(values.copy())
            s.pipe_identity[name] = {
                key: value
                for key, value in pipe.items()
                if key in _IDENTITY_KEYS or key in {"dev", "stage"}
            }

        # Simple first/last int-dict parsers
        for regex, name in _SIMPLE_PARSERS:
            m = regex.search(line)
            if m:
                vals = {k: int(v) for k, v in m.groupdict().items() if v is not None}
                s._record(name, vals)

        # VOX (needs bool coercion)
        m = ESP_AUDIO_VOX_RE.search(line)
        if m:
            vals = {
                "vox_activations": int(m.group("vox_activations")),
                "vox_active": 1 if m.group("vox_active").lower() == "yes" else 0,
            }
            s._record("vox", vals)

        # Frame counts (incremental merge — encoded/decoded on separate lines)
        enc_m = ESP_AUDIO_ENCODED_RE.search(line)
        dec_m = ESP_AUDIO_DECODED_RE.search(line)
        if enc_m or dec_m:
            vals = dict(s.last_frame_counts) if s.last_frame_counts else {}
            if enc_m:
                vals["encoded_frames"] = int(enc_m.group("encoded_frames"))
            if dec_m:
                vals["decoded_frames"] = int(dec_m.group("decoded_frames"))
            if vals:
                if not s.first_frame_counts:
                    s.first_frame_counts = vals.copy()
                s.last_frame_counts = vals
                s.frame_counts_samples += 1
                s.sample_history.setdefault("frame_counts", []).append(vals.copy())

        # Latency peak tracking
        if s.last_latency:
            s.latency_max_peak_ms = max(
                s.latency_max_peak_ms, s.last_latency.get("lat_max_ms", 0)
            )

        # nRF underflow events
        m = NRF_UFLOW_RE.search(line)
        if m:
            under = int(m.group("under"))
            if s.first_uflow_under is None:
                s.first_uflow_under = under
            s.last_uflow_under = under
            s.uflow_under_history.append(under)
            s.uflow_events += 1

        # nRF audio RX packet count
        m = NRF_AUDIO_RX_RE.search(line)
        if m:
            rx_pkts = int(m.group("rx_pkts"))
            if s.first_nrf_audio_rx_pkts is None:
                s.first_nrf_audio_rx_pkts = rx_pkts
            s.last_nrf_audio_rx_pkts = rx_pkts
            s.nrf_audio_rx_history.append(rx_pkts)
            s.nrf_audio_samples += 1

        # nRF auto-tune (adds computed skip_pct)
        m = NRF_ATUNE_RE.search(line)
        if m:
            vals = {k: int(v) for k, v in m.groupdict().items() if v is not None}
            ticks = vals.get("ticks", 0)
            if ticks > 0:
                vals["skip_pct"] = round(vals.get("skip", 0) * 100.0 / ticks, 2)
            s._record("atune", vals)

        # ESP bridge CRC warning
        if "Bridge RX CRC mismatch" in line:
            m = ESP_CRC_WARN_RE.search(line)
            if m:
                s.last_crc_warn = int(m.group("crc_fail"))


# =============================================================================
# Report / summary output
# =============================================================================

# Names of all first/last/delta snapshot groups (order matters for JSON output).
_SNAPSHOT_NAMES = [
    "mesh",
    "spi",
    "audio_pipe",
    "glitch",
    "conceal",
    "adaptive",
    "latency",
    "encode",
    "decode",
    "tx_pipeline",
    "rx_pipeline",
    "vox",
    "rx_depth",
    "frame_counts",
    "txstarve",
    "e2e_esp",
    "e2e_nrf",
    "atune",
]


def _snapshot_json(s: PortStats, name: str) -> dict[str, Any]:
    """Return first / last / delta dict entries for one snapshot group."""
    first = getattr(s, f"first_{name}")
    last = getattr(s, f"last_{name}")
    history = s.sample_history.get(name, [first, last] if first and last else [])
    excluded = _IDENTITY_KEYS | _SNAPSHOT_GAUGE_KEYS.get(name, set())
    delta, resets = _reset_aware_delta(history, excluded)
    if name == "mesh":
        starve_delta, starve_resets = _mesh_starvation_series(s)
        if starve_delta is None:
            delta.pop("starve", None)
            resets.pop("starve", None)
        else:
            delta["starve"] = starve_delta
            if starve_resets:
                resets["starve"] = starve_resets
            else:
                resets.pop("starve", None)
    if name == "e2e_esp" and delta:
        delta["effective_gap"] = max(
            delta.get("gap_fr", 0) - delta.get("recovered", 0), 0
        )
    return {
        f"first_{name}": first,
        f"last_{name}": last,
        f"{name}_delta": delta,
        f"{name}_reset_epochs": resets,
    }


def _build_port_json(s: PortStats) -> dict[str, Any]:
    """Build the JSON-serialisable dict for one port."""
    d: dict[str, Any] = {
        "port": s.port,
        "open_ok": s.open_ok,
        "open_error": s.open_error,
        "reconnects": s.reconnects,
        "lines": s.lines,
        "bytes": s.bytes_rx,
        "mesh_samples": s.mesh_samples,
        "spi_samples": s.spi_samples,
        "audio_pipe_samples": s.audio_pipe_samples,
        "glitch_samples": s.glitch_samples,
        "conceal_samples": s.conceal_samples,
        "adaptive_samples": s.adaptive_samples,
        "latency_samples": s.latency_samples,
        "encode_samples": s.encode_samples,
        "decode_samples": s.decode_samples,
        "tx_pipeline_samples": s.tx_pipeline_samples,
        "rx_pipeline_samples": s.rx_pipeline_samples,
        "vox_samples": s.vox_samples,
        "rx_depth_samples": s.rx_depth_samples,
        "frame_counts_samples": s.frame_counts_samples,
        "txstarve_samples": s.txstarve_samples,
        "uflow_events": s.uflow_events,
        "nrf_audio_samples": s.nrf_audio_samples,
        "atune_samples": s.atune_samples,
        "e2e_esp_samples": s.e2e_esp_samples,
        "e2e_nrf_samples": s.e2e_nrf_samples,
        "pipeline_samples": s.pipe_samples,
        "pipeline": {
            name: {
                "first": s.first_pipe[name],
                "last": s.last_pipe.get(name, {}),
                "delta": _reset_aware_delta(
                    s.pipe_history.get(name)
                    or [s.first_pipe[name], s.last_pipe.get(name, {})],
                    _PIPE_GAUGE_KEYS | _IDENTITY_KEYS,
                )[0],
                "reset_epochs": _reset_aware_delta(
                    s.pipe_history.get(name)
                    or [s.first_pipe[name], s.last_pipe.get(name, {})],
                    _PIPE_GAUGE_KEYS | _IDENTITY_KEYS,
                )[1],
            }
            for name in sorted(s.first_pipe)
        },
    }
    for name in _SNAPSHOT_NAMES:
        d.update(_snapshot_json(s, name))
    d.update(
        {
            "latency_max_peak_ms": s.latency_max_peak_ms,
            "first_uflow_under": s.first_uflow_under,
            "last_uflow_under": s.last_uflow_under,
            "uflow_under_delta": _scalar_series_delta(
                s.uflow_under_history
                or [value for value in (s.first_uflow_under, s.last_uflow_under) if value is not None]
            ),
            "first_nrf_audio_rx_pkts": s.first_nrf_audio_rx_pkts,
            "last_nrf_audio_rx_pkts": s.last_nrf_audio_rx_pkts,
            "nrf_audio_rx_delta": _scalar_series_delta(
                s.nrf_audio_rx_history
                or [
                    value
                    for value in (s.first_nrf_audio_rx_pkts, s.last_nrf_audio_rx_pkts)
                    if value is not None
                ]
            ),
            "hop_pct": compute_hop_pct(s),
            "last_crc_warn": s.last_crc_warn,
        }
    )
    return d


def write_summary_json(
    path: str,
    started_at: float,
    ended_at: float,
    all_stats: list[PortStats],
    duration: int,
) -> None:
    """Write structured JSON summary to *path*."""
    result = {
        "started_at": datetime.fromtimestamp(started_at, tz=timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "ended_at": datetime.fromtimestamp(ended_at, tz=timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "requested_duration_s": duration,
        "actual_duration_s": round(ended_at - started_at, 3),
        "ports": [_build_port_json(s) for s in all_stats],
        "correlated_delivery": compute_correlated_delivery(all_stats),
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)


# ------------------------------------------------------------------
# Human-readable report
# ------------------------------------------------------------------


def _report_lines_for_port(s: PortStats, duration: int) -> list[str]:
    """Generate human-readable report lines for one port."""
    out: list[str] = []

    # Header
    out.append(f"Port: {s.port}")
    out.append(f"  Open: {'yes' if s.open_ok else 'no'}")
    if s.open_error:
        out.append(f"  Error: {s.open_error}")
    out.append(f"  Lines: {s.lines}")
    out.append(f"  Bytes: {s.bytes_rx}")
    out.append(
        f"  Samples: mesh={s.mesh_samples} spi={s.spi_samples} "
        f"audio_pipe={s.audio_pipe_samples} glitch={s.glitch_samples} "
        f"latency={s.latency_samples} enc={s.encode_samples} "
        f"dec={s.decode_samples} txp={s.tx_pipeline_samples} rxp={s.rx_pipeline_samples} "
        f"vox={s.vox_samples} depth={s.rx_depth_samples} "
        f"txstarve={s.txstarve_samples} legacy_uflow={s.uflow_events} "
        f"nrf_audio={s.nrf_audio_samples} "
        f"atune={s.atune_samples} e2e_esp={s.e2e_esp_samples} "
        f"e2e_nrf={s.e2e_nrf_samples}"
    )
    for name in sorted(s.first_pipe):
        delta, resets = _reset_aware_delta(
            s.pipe_history.get(name)
            or [s.first_pipe[name], s.last_pipe.get(name, {})],
            _PIPE_GAUGE_KEYS | _IDENTITY_KEYS,
        )
        fields = " ".join(f"{key}={value}" for key, value in delta.items() if value != 0)
        if resets:
            fields += " resets=" + ",".join(
                f"{key}:{count}" for key, count in sorted(resets.items())
            )
        out.append(f"  PIPE {name} samples={s.pipe_samples.get(name, 0)} {fields}".rstrip())

    # Mesh
    if s.last_mesh:
        m, d = s.last_mesh, s.delta("mesh")
        if "starve" in m:
            mesh_starve_delta, _ = _mesh_starvation_series(s)
            if mesh_starve_delta is not None:
                d = dict(d)
                d["starve"] = mesh_starve_delta
        starvation_field = (
            f"starve={m['starve']}" if "starve" in m else f"legacy_under={m.get('under', 0)}"
        )
        starvation_delta_field = (
            f"starve={d.get('starve', 0)}"
            if "starve" in m
            else f"legacy_under={d.get('under', 0)}"
        )
        drain = m.get("drain")
        out.append(
            f"  Last MESH: tx={m['tx']} err={m['tx_err']} rx={m['rx']} "
            f"drop={m['drop']} fwd={m['fwd']} spi_in={m['spi_in']} "
            f"overwr={m['overwr']} {starvation_field}"
            f"{' drain=' + str(drain) if drain is not None else ''} q={m['q']}"
        )
        if d:
            out.append(
                f"  Delta MESH: tx={d.get('tx', 0)} err={d.get('tx_err', 0)} "
                f"rx={d.get('rx', 0)} drop={d.get('drop', 0)} fwd={d.get('fwd', 0)} "
                f"spi_in={d.get('spi_in', 0)} overwr={d.get('overwr', 0)} "
                f"{starvation_delta_field} drain={d.get('drain', 0)}"
            )

    # SPI
    if s.last_spi:
        m, d = s.last_spi, s.delta("spi")
        out.append(
            f"  Last SPI: poll_us(min/avg/max)={m['poll_min']}/{m['poll_avg']}"
            f"/{m['poll_max']} q_over={m['q_over']} seq_gap={m['seq_gap']} "
            f"crc_fail={m['crc_fail']}"
        )
        if d:
            out.append(
                f"  Delta SPI: q_over={d.get('q_over', 0)} "
                f"seq_gap={d.get('seq_gap', 0)} crc_fail={d.get('crc_fail', 0)}"
            )

    # Audio pipe
    if s.last_audio_pipe:
        m, d = s.last_audio_pipe, s.delta("audio_pipe")
        out.append(
            f"  Last AudioPipe: tx_queued={m['tx_queued']} "
            f"tx_overwr={m['tx_overwr']} rx_from_nrf={m['rx_from_nrf']}"
        )
        if d:
            out.append(
                f"  Delta AudioPipe: tx_queued={d.get('tx_queued', 0)} "
                f"tx_overwr={d.get('tx_overwr', 0)} rx_from_nrf={d.get('rx_from_nrf', 0)}"
            )

    # Glitch
    if s.last_glitch:
        m, d = s.last_glitch, s.delta("glitch")
        out.append(
            f"  Last GlitchStats: glitches={m['glitches']} rx_und={m['rx_und']} "
            f"i2s_inc={m['i2s_inc']} adc_overruns={m['adc_overruns']}"
        )
        if d:
            out.append(
                f"  Delta GlitchStats: glitches={d.get('glitches', 0)} "
                f"rx_und={d.get('rx_und', 0)} i2s_inc={d.get('i2s_inc', 0)} "
                f"adc_overruns={d.get('adc_overruns', 0)} "
                f"(glitches/min={_rate_per_min(d.get('glitches', 0), duration)})"
            )

    # Concealment
    if s.last_conceal:
        m, d = s.last_conceal, s.delta("conceal")
        out.append(
            f"  Last Concealment: plc={m['plc']} grace_empty={m['grace_empty']} "
            f"conceal={m.get('conceal', 0)} seq_gap={m.get('seq_gap_frames', 0)} "
            f"seq_reset={m.get('seq_reset', 0)} seq_stale={m.get('seq_stale', 0)}"
        )
        if d:
            out.append(
                f"  Delta Concealment: plc={d.get('plc', 0)} "
                f"grace_empty={d.get('grace_empty', 0)} "
                f"conceal={d.get('conceal', 0)} seq_gap={d.get('seq_gap_frames', 0)} "
                f"seq_reset={d.get('seq_reset', 0)} seq_stale={d.get('seq_stale', 0)}"
            )

    # Adaptive playout
    if s.last_adaptive:
        m, d = s.last_adaptive, s.delta("adaptive")
        out.append(
            f"  Last Adaptive: hold={m['hold']} catchup={m['catchup']} "
            f"sources={m['sources']}"
        )
        if d:
            out.append(
                f"  Delta Adaptive: hold={d.get('hold', 0)} catchup={d.get('catchup', 0)}"
            )

    # Latency / encode / decode
    if s.last_latency:
        m = s.last_latency
        out.append(
            f"  Last Latency: avg={m['lat_avg_ms']}ms max={m['lat_max_ms']}ms "
            f"peak_max={s.latency_max_peak_ms}ms"
        )
    if s.last_encode:
        m = s.last_encode
        out.append(f"  Last Encode: avg={m['enc_avg_us']}us max={m['enc_max_us']}us")
    if s.last_decode:
        m = s.last_decode
        out.append(f"  Last Decode: avg={m['dec_avg_us']}us max={m['dec_max_us']}us")
    if s.last_tx_pipeline:
        m = s.last_tx_pipeline
        out.append(
            f"  Last TX pipeline: avg={m['tx_pipe_avg_us']}us max={m['tx_pipe_max_us']}us"
        )
    if s.last_rx_pipeline:
        m = s.last_rx_pipeline
        out.append(
            f"  Last RX pipeline: avg={m['rx_pipe_avg_us']}us max={m['rx_pipe_max_us']}us"
        )

    # Consolidated latency summary
    if s.last_latency or s.last_tx_pipeline or s.last_rx_pipeline:
        lat_summary: list[str] = []
        if s.last_latency:
            lat_summary.append(
                f"total={s.last_latency['lat_avg_ms']}/{s.latency_max_peak_ms}ms"
            )
        if s.last_tx_pipeline:
            lat_summary.append(
                f"TX={round(s.last_tx_pipeline['tx_pipe_avg_us'] / 1000, 1)}"
                f"/{round(s.last_tx_pipeline['tx_pipe_max_us'] / 1000, 1)}ms"
            )
        if s.last_rx_pipeline:
            lat_summary.append(
                f"RX={round(s.last_rx_pipeline['rx_pipe_avg_us'] / 1000, 1)}"
                f"/{round(s.last_rx_pipeline['rx_pipe_max_us'] / 1000, 1)}ms"
            )
        if s.last_encode:
            lat_summary.append(
                f"enc={round(s.last_encode['enc_avg_us'] / 1000, 1)}"
                f"/{round(s.last_encode['enc_max_us'] / 1000, 1)}ms"
            )
        if s.last_decode:
            lat_summary.append(
                f"dec={round(s.last_decode['dec_avg_us'] / 1000, 1)}"
                f"/{round(s.last_decode['dec_max_us'] / 1000, 1)}ms"
            )
        out.append(f"  Latency (avg/max): {' | '.join(lat_summary)}")

    # VOX
    if s.last_vox:
        d = s.delta("vox")
        out.append(
            f"  Last VOX: activations={s.last_vox['vox_activations']} "
            f"active={bool(s.last_vox.get('vox_active', 0))}"
        )
        if d:
            out.append(f"  Delta VOX: activations={d.get('vox_activations', 0)}")

    # RX depth
    if s.last_rx_depth:
        m = s.last_rx_depth
        out.append(
            f"  Last RX depth: min={m['rx_q_min']} avg={m['rx_q_avg']} "
            f"max={m['rx_q_max']} total={m['rx_q_total']}"
        )

    # Frame counts
    if s.last_frame_counts:
        m, d = s.last_frame_counts, s.delta("frame_counts")
        out.append(
            f"  Last FrameCounts: encoded={m.get('encoded_frames')} "
            f"decoded={m.get('decoded_frames')}"
        )
        if d:
            out.append(
                f"  Delta FrameCounts: encoded={d.get('encoded_frames', 0)} "
                f"decoded={d.get('decoded_frames', 0)}"
            )

    # nRF local slot starvation
    starve_delta = _nrf_starvation_delta(s)
    starve_total = _nrf_starvation_total(s)
    if starve_total is not None:
        out.append(
            f"  nRF TX starvation: total={starve_total} "
            f"delta={starve_delta if starve_delta is not None else 'n/a'} "
            f"(events/min={_rate_per_min(starve_delta, duration)})"
        )
    if s.last_uflow_under is not None:
        legacy_delta = _scalar_series_delta(s.uflow_under_history)
        out.append(
            f"  nRF legacy UFLOW: total={s.last_uflow_under} "
            f"delta={legacy_delta if legacy_delta is not None else 'n/a'} (not health)"
        )

    # nRF bridge audio RX
    if s.last_nrf_audio_rx_pkts is not None:
        rx_d = _scalar_series_delta(s.nrf_audio_rx_history)
        out.append(
            f"  nRF Bridge Audio RX: last_pkts={s.last_nrf_audio_rx_pkts} "
            f"delta={rx_d if rx_d is not None else 'n/a'} "
            f"(pkts/min={_rate_per_min(rx_d, duration)})"
        )

    # nRF auto-tune
    if s.last_atune:
        m, d = s.last_atune, s.delta("atune")
        out.append(
            f"  nRF AutoTune: last_q={m.get('q')} "
            f"skip={m.get('skip', 0)}/{m.get('ticks', 0)} "
            f"skip_pct={m.get('skip_pct', 'n/a')}% "
            f"ws_edges={m.get('ws_edges', 'n/a')} "
            f"ws_corr={m.get('ws_corr', 'n/a')} ws_drift={m.get('ws_drift', 'n/a')}"
        )
        if d:
            out.append(
                f"  Delta AutoTune: q={d.get('q', 0)} skip={d.get('skip', 0)}"
            )

    # E2E ESP
    if s.last_e2e_esp:
        m, d = s.last_e2e_esp, s.delta("e2e_esp")
        recovered = m.get("recovered", 0)
        effective_gap = max(m["gap_fr"] - recovered, 0)
        out.append(
            f"  E2E ESP: tx={m['tx']} rx={m['rx']} "
            f"gap_evt={m['gap_evt']} raw_gap={m['gap_fr']} "
            f"recovered={recovered} effective_gap={effective_gap}"
        )
        if d:
            raw_gap = d.get("gap_fr", 0)
            recovered = d.get("recovered", 0)
            effective_gap = max(raw_gap - recovered, 0)
            out.append(
                f"  Delta E2E ESP: tx={d.get('tx', 0)} rx={d.get('rx', 0)} "
                f"gap_evt={d.get('gap_evt', 0)} raw_gap={raw_gap} "
                f"recovered={recovered} effective_gap={effective_gap}"
            )
            hop = compute_hop_pct(s)
            raw_pct = hop.get("esp_e2e_raw_gap_pct")
            effective_pct = hop.get("esp_e2e_effective_gap_pct")
            if raw_pct is not None and effective_pct is not None:
                out.append(
                    f"  Stage-local ESP RX gap: raw={raw_pct}% effective={effective_pct}%"
                )

    # E2E nRF
    if s.last_e2e_nrf:
        m, d = s.last_e2e_nrf, s.delta("e2e_nrf")
        out.append(
            f"  E2E nRF: id={m['id']} spi_in={m['spi_in']} "
            f"spi_gap={m['spi_gap_evt']}/{m['spi_gap_fr']} "
            f"rf_tx={m['rf_tx']} rf_rx={m['rf_rx']} "
            f"rf_gap={m['rf_gap_evt']}/{m['rf_gap_fr']} spi_out={m['spi_out']}"
        )
        if d:
            out.append(
                f"  Delta E2E nRF: spi_in={d.get('spi_in', 0)} "
                f"spi_gap_evt={d.get('spi_gap_evt', 0)} "
                f"spi_gap_fr={d.get('spi_gap_fr', 0)} "
                f"rf_tx={d.get('rf_tx', 0)} rf_rx={d.get('rf_rx', 0)} "
                f"rf_gap_evt={d.get('rf_gap_evt', 0)} "
                f"rf_gap_fr={d.get('rf_gap_fr', 0)} spi_out={d.get('spi_out', 0)}"
            )
            hop = compute_hop_pct(s)
            gap_parts = []
            if hop.get("nrf_spi_gap_pct") is not None:
                gap_parts.append(f"spi_rx_gap={hop['nrf_spi_gap_pct']}%")
            if hop.get("nrf_rf_gap_pct") is not None:
                gap_parts.append(f"rf_rx_gap={hop['nrf_rf_gap_pct']}%")
            if gap_parts:
                out.append(f"  Stage-local nRF: {' '.join(gap_parts)}")

    if s.last_crc_warn is not None:
        out.append(f"  Last ESP bridge CRC warn counter: {s.last_crc_warn}")

    # Health assessment
    out.append(f"  Health: {_health_line(s)}")
    out.append("")
    return out


def _health_line(s: PortStats) -> str:
    """Produce a one-line health assessment."""
    if not s.open_ok:
        return "FAIL (port could not be opened)"
    if s.lines == 0:
        return "NO DATA (port opened but no output)"

    issues: list[str] = []

    def _check_delta(name: str, keys: list[str]) -> None:
        d = s.delta(name)
        for k in keys:
            v = d.get(k, 0)
            if v > 0:
                issues.append(f"{name}_{k}+{v}")

    _check_delta("mesh", ["tx_err", "drop"])
    _check_delta("spi", ["crc_fail", "q_over"])

    audio_d = s.delta("audio_pipe")
    if audio_d.get("tx_overwr", 0) > 0:
        issues.append(f"esp_tx_overwr+{audio_d['tx_overwr']}")

    glitch_d = s.delta("glitch")
    for k in ("glitches", "rx_und", "adc_overruns"):
        if glitch_d.get(k, 0) > 0:
            issues.append(f"esp_{k}+{glitch_d[k]}")

    # High underrun ratio
    frame_d = s.delta("frame_counts")
    decoded = frame_d.get("decoded_frames", 0)
    rx_und = glitch_d.get("rx_und", 0)
    if decoded > 0 and rx_und > 0 and (rx_und / decoded) > 0.1:
        issues.append(f"esp_und_per_decoded={rx_und / decoded:.2f}")

    starve_d = _nrf_starvation_delta(s)
    if starve_d is not None and starve_d > 0:
        issues.append(f"nrf_starve+{starve_d}")

    e2e_esp_d = s.delta("e2e_esp")
    effective_gap = max(
        e2e_esp_d.get("gap_fr", 0) - e2e_esp_d.get("recovered", 0), 0
    )
    if effective_gap > 0:
        issues.append(f"e2e_esp_effective_gap+{effective_gap}")

    e2e_nrf_d = s.delta("e2e_nrf")
    if e2e_nrf_d.get("spi_gap_fr", 0) > 0:
        issues.append(f"e2e_nrf_spi_gap_fr+{e2e_nrf_d['spi_gap_fr']}")
    if e2e_nrf_d.get("rf_gap_fr", 0) > 0:
        issues.append(f"e2e_nrf_rf_gap_fr+{e2e_nrf_d['rf_gap_fr']}")

    if not issues:
        return "OK (no error/drops/CRC growth observed)"
    return "WARN (" + ", ".join(issues) + ")"


def write_human_report(
    path: str, run_dir: str, all_stats: list[PortStats], duration: int
) -> None:
    """Write human-readable report to *path*."""
    lines = [
        "OMI Serial Benchmark",
        f"Duration: {duration}s",
        f"Run directory: {run_dir}",
        "",
    ]
    for s in all_stats:
        lines.extend(_report_lines_for_port(s, duration))

    correlation = compute_correlated_delivery(all_stats)
    lines.append(f"Correlated delivery: {correlation['status']}")
    if correlation.get("reason"):
        lines.append(f"  {correlation['reason']}")
    for link in correlation["links"]:
        delivery = (
            f"{link['delivery_pct']}%" if link["delivery_pct"] is not None else "not reported"
        )
        lines.append(
            f"  session={link['session']} {link['sender_node']}->{link['receiver_node']} "
            f"stage={link['stage']} sent={link['sent']} received={link['received']} "
            f"delivery={delivery} status={link['status']}"
        )

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines).rstrip() + "\n")


# =============================================================================
# Quick stdout summary
# =============================================================================


def print_quick_summary(all_stats: list[PortStats], duration: int) -> None:
    """Print compact glitch + hop-% metrics to stdout for quick comparison."""
    print("\n--- Quick Summary ---")
    printed = False

    for s in all_stats:
        glitch_d = s.delta("glitch")
        hop = compute_hop_pct(s)

        # ESP port: glitches + stage-local gaps + latency
        is_esp = glitch_d or (hop and "esp_e2e_effective_gap_pct" in hop)
        if is_esp:
            parts: list[str] = []
            if glitch_d:
                gl = glitch_d.get("glitches", 0)
                gpm = _rate_per_min(gl, duration)
                parts.append(f"glitches={gl} ({gpm}/min)")
            raw_gap = hop.get("esp_e2e_raw_gap_pct")
            effective_gap = hop.get("esp_e2e_effective_gap_pct")
            if raw_gap is not None and effective_gap is not None:
                parts.append(f"rx_gap_raw={raw_gap}% rx_gap_effective={effective_gap}%")
            if parts:
                print(f"  {s.port} (ESP): {' | '.join(parts)}")
                printed = True

            # Latency breakdown
            lat_parts: list[str] = []
            if s.last_latency:
                lat_parts.append(
                    f"total avg={s.last_latency['lat_avg_ms']}ms "
                    f"max={s.latency_max_peak_ms}ms"
                )
            if s.last_tx_pipeline:
                lat_parts.append(
                    f"TX avg={round(s.last_tx_pipeline['tx_pipe_avg_us'] / 1000, 1)}ms "
                    f"max={round(s.last_tx_pipeline['tx_pipe_max_us'] / 1000, 1)}ms"
                )
            if s.last_rx_pipeline:
                lat_parts.append(
                    f"RX avg={round(s.last_rx_pipeline['rx_pipe_avg_us'] / 1000, 1)}ms "
                    f"max={round(s.last_rx_pipeline['rx_pipe_max_us'] / 1000, 1)}ms"
                )
            if s.last_encode:
                lat_parts.append(
                    f"enc avg={round(s.last_encode['enc_avg_us'] / 1000, 1)}ms"
                )
            if s.last_decode:
                lat_parts.append(
                    f"dec avg={round(s.last_decode['dec_avg_us'] / 1000, 1)}ms"
                )
            if lat_parts:
                print(f"    Latency: {' | '.join(lat_parts)}")
                printed = True

        # nRF port: stage-local receive gap ratios
        if hop and ("nrf_spi_gap_pct" in hop or "nrf_rf_gap_pct" in hop):
            parts = []
            for label, key in [
                ("spi_gap", "nrf_spi_gap_pct"),
                ("rf_gap", "nrf_rf_gap_pct"),
            ]:
                if hop.get(key) is not None:
                    parts.append(f"{label}={hop[key]}%")
            if parts:
                print(f"  {s.port} (nRF): {' | '.join(parts)}")
                printed = True

    if not printed:
        print("  (no metrics captured)")
    correlation = compute_correlated_delivery(all_stats)
    print(f"  Correlated delivery: {correlation['status']}")
    print()


# =============================================================================
# Main
# =============================================================================


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
