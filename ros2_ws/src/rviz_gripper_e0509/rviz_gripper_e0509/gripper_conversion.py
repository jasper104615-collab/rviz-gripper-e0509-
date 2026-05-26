"""Real gripper pulse -> URDF joint angle [rad]."""

from __future__ import annotations


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def pulse_to_rad(
    pulse: int,
    *,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
    pulse_open: int = 0,
    pulse_closed: int = 700,
) -> float:
    span = pulse_closed - pulse_open
    if span <= 0:
        return open_rad
    t = clamp((pulse - pulse_open) / float(span), 0.0, 1.0)
    return open_rad + t * (closed_rad - open_rad)
