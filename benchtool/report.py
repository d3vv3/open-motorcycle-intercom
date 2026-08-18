"""report.txt, summary.json, and stdout rendering."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any

from benchtool.health import _health_line
from benchtool.parsers import _IDENTITY_KEYS, _PIPE_GAUGE_KEYS, _SNAPSHOT_GAUGE_KEYS
from benchtool.series import (
    _rate_per_min,
    _reset_aware_delta,
    _scalar_series_delta,
)
from benchtool.stats import (
    PortStats,
    _mesh_starvation_series,
    _nrf_starvation_delta,
    _nrf_starvation_total,
    compute_correlated_delivery,
    compute_hop_pct,
)

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


def _pipe_delta_and_resets(s: PortStats, name: str) -> tuple[dict, dict]:
    return _reset_aware_delta(
        s.pipe_history.get(name) or [s.first_pipe[name], s.last_pipe.get(name, {})],
        _PIPE_GAUGE_KEYS | _IDENTITY_KEYS,
    )


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
                "delta": _pipe_delta_and_resets(s, name)[0],
                "reset_epochs": _pipe_delta_and_resets(s, name)[1],
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


def _header_lines(s: PortStats) -> list[str]:
    out = [f"Port: {s.port}", f"  Open: {'yes' if s.open_ok else 'no'}"]
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
        delta, resets = _pipe_delta_and_resets(s, name)
        fields = " ".join(f"{key}={value}" for key, value in delta.items() if value != 0)
        if resets:
            fields += " resets=" + ",".join(
                f"{key}:{count}" for key, count in sorted(resets.items())
            )
        out.append(f"  PIPE {name} samples={s.pipe_samples.get(name, 0)} {fields}".rstrip())
    return out


def _mesh_lines(s: PortStats) -> list[str]:
    if not s.last_mesh:
        return []
    out = []
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
    return out


def _bridge_lines(s: PortStats) -> list[str]:
    out = []
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
    return out


def _playback_lines(s: PortStats, duration: int) -> list[str]:
    out = []
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
    return out


def _latency_lines(s: PortStats) -> list[str]:
    out = []
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
    return out


def _capture_lines(s: PortStats) -> list[str]:
    out = []
    if s.last_vox:
        d = s.delta("vox")
        out.append(
            f"  Last VOX: activations={s.last_vox['vox_activations']} "
            f"active={bool(s.last_vox.get('vox_active', 0))}"
        )
        if d:
            out.append(f"  Delta VOX: activations={d.get('vox_activations', 0)}")
    if s.last_rx_depth:
        m = s.last_rx_depth
        out.append(
            f"  Last RX depth: min={m['rx_q_min']} avg={m['rx_q_avg']} "
            f"max={m['rx_q_max']} total={m['rx_q_total']}"
        )
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
    return out


def _nrf_lines(s: PortStats, duration: int) -> list[str]:
    out = []
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
    if s.last_nrf_audio_rx_pkts is not None:
        rx_d = _scalar_series_delta(s.nrf_audio_rx_history)
        out.append(
            f"  nRF Bridge Audio RX: last_pkts={s.last_nrf_audio_rx_pkts} "
            f"delta={rx_d if rx_d is not None else 'n/a'} "
            f"(pkts/min={_rate_per_min(rx_d, duration)})"
        )
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
    return out


def _e2e_lines(s: PortStats) -> list[str]:
    out = []
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
    return out


def _report_lines_for_port(s: PortStats, duration: int) -> list[str]:
    """Generate human-readable report lines for one port."""
    out = _header_lines(s)
    out.extend(_mesh_lines(s))
    out.extend(_bridge_lines(s))
    out.extend(_playback_lines(s, duration))
    out.extend(_latency_lines(s))
    out.extend(_capture_lines(s))
    out.extend(_nrf_lines(s, duration))
    out.extend(_e2e_lines(s))
    if s.last_crc_warn is not None:
        out.append(f"  Last ESP bridge CRC warn counter: {s.last_crc_warn}")
    out.append(f"  Health: {_health_line(s)}")
    out.append("")
    return out


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
