"""Per-port accumulator and derived metrics."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any

from benchtool.parsers import (
    _IDENTITY_KEYS,
    _PIPE_GAUGE_KEYS,
    _PIPE_RX_COUNTERS,
    _PIPE_TX_COUNTERS,
    _SNAPSHOT_GAUGE_KEYS,
)
from benchtool.series import (
    _pct,
    _reset_aware_delta,
    _u32_cumulative_series_delta,
)


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
        out["nrf_rf_gap_pct"] = _pct(rf_gaps, e2e_nrf_d.get("rf_rx", 0) + rf_gaps)

    return out


def _pipe_key(record: dict[str, int | str]) -> str:
    key = f"{record['dev']}:{record['stage']}:{record.get('node', 'na')}"
    for field_name in (
        "session", "src_node", "dst_node", "peer", "peer_node",
        "node_mac", "epoch_id", "part",
    ):
        if field_name in record:
            key += f":{field_name}={record[field_name]}"
    return key


_ESP_STAGE_FIELDS = {
    "audio": {
        "frames": ("capture_ok", "encode_ok", "tx_handoff", "rx_store_ok", "rx_store_pop", "decode_ok", "play_ok"),
        "discards": ("tx_no_cb", "rx_store_reject", "rx_store_purge", "rx_q_drop", "rx_lock_drop", "rx_src_drop", "jitter_drop", "pcm_overflow", "bt_music_overflow", "bt_call_overflow", "bt_mic_overflow"),
        "attempts_filters": ("capture_short", "capture_timeout", "capture_err", "encode_err", "rx_invalid", "rx_inactive", "decode_err", "packet_dup", "packet_late", "packet_future", "notify_drop", "i2s_err", "dtx_drop"),
        "diagnostics": ("capture_fifo_discard_samples", "rx_src_evict", "seq_gap", "seq_reset", "seq_stale", "conceal", "plc", "glitch", "pcm_underrun", "bt_music_underrun", "bt_call_underrun", "bt_mic_underrun", "bt_music_lock", "bt_call_lock", "bt_playout_lock", "bt_mic_write_lock", "bt_mic_read_lock"),
        "gauges": ("rx_store_depth", "rx_store_depth_valid", "rx_sources"),
    },
    "espnow": {
        "frames": ("tx_offer", "tx_enqueue", "tx_submit_ok", "tx_radio_ok", "rx_audio_raw", "rx_audio_queued", "rx_audio_seen", "rx_audio_accept", "jitter_pop", "rx_deliver"),
        "discards": ("tx_queue_full", "tx_purge", "tx_radio_fail", "tx_abandoned", "relay_overwrite", "rx_audio_queue_full", "rx_audio_purge", "jitter_overwrite", "jitter_late", "jitter_purge"),
        "attempts_filters": ("tx_reject_state", "tx_reject_invalid", "tx_submit_err", "rx_short", "rx_audio_bad_header", "rx_audio_disabled", "rx_audio_invalid", "rx_audio_self", "rx_audio_dup"),
        "diagnostics": ("seq_gap", "slot_misses", "control_queue_drops", "rx_queue_overflows"),
        "gauges": ("tx_depth", "tx_inflight", "jitter_depth", "rx_depth", "control_queue_depth"),
    },
}


def compute_esp_stage_rows(s: PortStats) -> list[dict[str, Any]]:
    """Measured, stage-local intervals; never join audio and mesh epoch tokens."""
    rows: list[dict[str, Any]] = []
    for key, identity in sorted(s.pipe_identity.items()):
        if identity.get("dev") != "esp" or identity.get("stage") not in _ESP_STAGE_FIELDS:
            continue
        history = s.pipe_history[key]
        fields = _ESP_STAGE_FIELDS[str(identity["stage"])]
        uptime = [sample.get("uptime_ms") for sample in history]
        elapsed = uptime[-1] - uptime[0] if len(uptime) >= 2 and all(isinstance(x, int) for x in uptime) else 0
        valid = ("epoch_id" in identity and elapsed > 0 and
                 all(b > a for a, b in zip(uptime, uptime[1:])))
        # Count only counters covering the full row window. Sparse or reset series
        # cannot be divided by this row's elapsed time.
        counters: dict[str, dict[str, int]] = {}
        rates: dict[str, float | None] = {}
        discard_unavailable: list[str] = []
        for category, names in fields.items():
            if category == "gauges":
                continue
            counters[category] = {}
            for name in names:
                present = any(name in sample for sample in history)
                values = [sample[name] for sample in history if name in sample]
                covered = (valid and len(values) == len(history) and
                           all(b >= a for a, b in zip(values, values[1:])))
                if covered:
                    counters[category][name] = values[-1] - values[0]
                elif category == "discards" and present:
                    discard_unavailable.append(name)
                if category == "frames" and present:
                    rates[name] = round((values[-1] - values[0]) * 1000 / elapsed, 2) if covered else None
        measured = valid and any(counters.values())
        rows.append({
            "series": key,
            "identity": identity,
            "samples": len(history),
            "status": "measured" if measured else "unavailable",
            "elapsed_ms": elapsed if valid else None,
            "counters": counters,
            "discard_counters_unavailable": discard_unavailable,
            "rates_per_s": rates,
            "gauges": {name: history[-1][name] for name in fields["gauges"] if name in history[-1]},
        })
    return rows


def compute_esp_discard_health(s: PortStats) -> dict[str, Any]:
    """Report confirmed local discard growth separately, never as a sum or RF loss."""
    warnings: list[str] = []
    unavailable: list[str] = []
    rows = compute_esp_stage_rows(s)
    split_groups: dict[tuple[str, str, str | int | None], set[str]] = {}
    for row in rows:
        identity = row["identity"]
        if "epoch_id" in identity and "part" in identity:
            group = (str(identity["stage"]), str(identity["epoch_id"]), identity.get("node_mac"))
            split_groups.setdefault(group, set()).add(str(identity["part"]))
    for (stage, epoch, mac), parts in sorted(split_groups.items(), key=lambda item: str(item[0])):
        expected = {"audio": {"tx", "rx", "playout", "bt"}, "espnow": {"tx", "rx"}}[stage]
        for part in sorted(expected - parts):
            unavailable.append(f"{stage}/{part} epoch={epoch}" + (f" mac={mac}" if mac else "") + " (missing part)")
    for row in rows:
        identity = row["identity"]
        label = f"{identity['stage']}/{identity.get('part', 'all')} epoch={identity.get('epoch_id', 'unknown')}"
        if (row["status"] != "measured" or not row["counters"]["discards"] or
                row["discard_counters_unavailable"]):
            unavailable.append(label)
        if row["status"] != "measured":
            continue
        for name, count in row["counters"]["discards"].items():
            if count > 0:
                warnings.append(f"{label} {name}+{count}")
    return {"warnings": warnings, "unavailable": unavailable}


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


def _counter_value(
    delta: dict[str, int], aliases: tuple[str, ...]
) -> tuple[str, int] | None:
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
                        {
                            "port": stats.port,
                            "counter": tx[0],
                            "value": tx[1],
                            "resets": resets,
                        }
                    )
            if rx is not None:
                endpoints = _endpoint(identity, "rx")
                if endpoints:
                    link = (identity["session"], endpoints[0], endpoints[1], semantic)
                    receivers.setdefault(link, []).append(
                        {
                            "port": stats.port,
                            "counter": rx[0],
                            "value": rx[1],
                            "resets": resets,
                        }
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
        reset_in_window = tx["counter"] in tx["resets"] or rx["counter"] in rx["resets"]
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
    overall_status = (
        "ok"
        if all(link["status"] == "ok" for link in links)
        else "inconsistent correlated data"
    )
    return {"status": overall_status, "links": links}
