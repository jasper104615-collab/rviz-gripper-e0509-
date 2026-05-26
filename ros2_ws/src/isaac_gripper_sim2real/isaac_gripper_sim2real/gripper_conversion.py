"""Isaac gripper joint [rad] <-> real command units (stroke 0~700 or pulse)."""

from __future__ import annotations


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalized_closure(
    angle_rad: float,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
) -> float:
    """0 = fully open, 1 = fully closed (URDF upper ~63 deg)."""
    span = closed_rad - open_rad
    if span <= 0.0:
        return 0.0
    return clamp((angle_rad - open_rad) / span, 0.0, 1.0)


def rad_to_stroke(
    angle_rad: float,
    *,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
    stroke_max: int = 700,
) -> int:
    """Map master joint [rad] to e0509 Modbus stroke (0=open, stroke_max=close)."""
    t = normalized_closure(angle_rad, open_rad, closed_rad)
    return int(round(t * stroke_max))


def rad_to_pulse(
    angle_rad: float,
    *,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
    pulse_open: int = 0,
    pulse_closed: int = 700,
) -> int:
    """Map master joint [rad] to rh_p12 set_position pulse (linear)."""
    t = normalized_closure(angle_rad, open_rad, closed_rad)
    return int(round(pulse_open + t * (pulse_closed - pulse_open)))


def stroke_to_rad(
    stroke: int,
    *,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
    stroke_max: int = 700,
) -> float:
    if stroke_max <= 0:
        return open_rad
    t = clamp(stroke / float(stroke_max), 0.0, 1.0)
    return open_rad + t * (closed_rad - open_rad)


def pulse_to_rad(
    pulse: int,
    *,
    open_rad: float = 0.0,
    closed_rad: float = 1.101,
    pulse_open: int = 0,
    pulse_closed: int = 700,
) -> float:
    """Map rh_p12 pulse feedback to master joint angle [rad] for RViz."""
    span = pulse_closed - pulse_open
    if span <= 0:
        return open_rad
    t = clamp((pulse - pulse_open) / float(span), 0.0, 1.0)
    return open_rad + t * (closed_rad - open_rad)
