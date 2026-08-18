"""Log-line regexes, key tables, and the PIPE logfmt parser."""

from __future__ import annotations

import re
import shlex

# Keys that report a current value rather than a cumulative count.
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

# All regex + snapshot-name pairs for simple "first/last int dict" parsing.
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
