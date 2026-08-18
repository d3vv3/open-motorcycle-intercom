"""Serial port discovery, identity tracking, and the reader thread."""

from __future__ import annotations

import glob
import threading
from datetime import datetime, timezone

import serial
from serial.tools import list_ports

from benchtool.parsers import (
    _IDENTITY_KEYS,
    _SIMPLE_PARSERS,
    ESP_AUDIO_DECODED_RE,
    ESP_AUDIO_ENCODED_RE,
    ESP_AUDIO_VOX_RE,
    ESP_CRC_WARN_RE,
    NRF_ATUNE_RE,
    NRF_AUDIO_RX_RE,
    NRF_UFLOW_RE,
    parse_pipeline_logfmt,
)
from benchtool.stats import PortStats, _pipe_key


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _sanitize_port(port: str) -> str:
    return port.replace("/", "_")


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

        # Frame counts (incremental merge - encoded/decoded on separate lines)
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
