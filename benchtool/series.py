"""Pure delta and rollover math for cumulative counter series."""

from __future__ import annotations

_U32_MAX = (1 << 32) - 1
_U32_ROLLOVER_HIGH_WATER = 0xF0000000
_U32_ROLLOVER_LOW_WATER = 0x0FFFFFFF


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


def _scalar_series_delta(values: list[int]) -> int | None:
    if len(values) < 2:
        return None
    return _reset_aware_delta([{"value": value} for value in values])[0]["value"]


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
            previous >= _U32_ROLLOVER_HIGH_WATER and current <= _U32_ROLLOVER_LOW_WATER
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
