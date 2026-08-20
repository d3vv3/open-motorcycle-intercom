"""Per-port health assessment rules."""

from __future__ import annotations

from benchtool.stats import PortStats, _nrf_starvation_delta


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
    effective_gap = max(e2e_esp_d.get("gap_fr", 0) - e2e_esp_d.get("recovered", 0), 0)
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
