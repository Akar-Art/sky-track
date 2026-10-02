"""Bounded TB2 flight path inside the 1 km² area (ascend + descend)."""

from __future__ import annotations

import math

from .targets import BAYRAKTAR_TB2, TargetTruth
from .world import DEFAULT_WORLD, WorldConfig


# Keep airframe centre inside the square with margin for 12 m wingspan.
_MARGIN_M = 40.0
# Orbit radius chosen so path stays well inside ±(half_size - margin)
_ORBIT_RADIUS_M = 320.0
# Horizontal lap period (seconds)
_ORBIT_PERIOD_S = 140.0
# Altitude oscillation (metres AGL) — inside useful HQ+16mm band
_ALT_CENTER_M = 2700.0
_ALT_AMPLITUDE_M = 450.0  # 2250 … 3150 m
_ALT_PERIOD_S = 100.0


def _clamp_horizontal(x: float, z: float, world: WorldConfig) -> tuple[float, float]:
    lim = world.half_size_m - _MARGIN_M
    return (
        max(-lim, min(lim, x)),
        max(-lim, min(lim, z)),
    )


def truth_at(t: float, world: WorldConfig = DEFAULT_WORLD) -> TargetTruth:
    """
    Moving operator truth at simulation time t (seconds).

    Horizontal: circular orbit inside the square.
    Vertical: smooth ascend / descend (sine).
    Never leaves the designated area (clamped with margin).
    """
    omega = 2.0 * math.pi / _ORBIT_PERIOD_S
    ang = omega * t
    x = _ORBIT_RADIUS_M * math.cos(ang)
    z = _ORBIT_RADIUS_M * math.sin(ang)
    x, z = _clamp_horizontal(x, z, world)

    y = _ALT_CENTER_M + _ALT_AMPLITUDE_M * math.sin(2.0 * math.pi * t / _ALT_PERIOD_S)

    # Ground-track heading: tangent to orbit (clockwise from north = +Z)
    # velocity ~ (-R sin, 0, R cos) * omega for ang = omega t with x=R cos, z=R sin
    vx = -_ORBIT_RADIUS_M * omega * math.sin(ang)
    vz = _ORBIT_RADIUS_M * omega * math.cos(ang)
    vy = (
        _ALT_AMPLITUDE_M
        * (2.0 * math.pi / _ALT_PERIOD_S)
        * math.cos(2.0 * math.pi * t / _ALT_PERIOD_S)
    )
    # Heading 0 = +Z north, clockwise toward +X east
    heading_deg = (math.degrees(math.atan2(vx, vz)) + 360.0) % 360.0
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    return TargetTruth(
        target_id="T-01",
        kind="uav",
        airframe=BAYRAKTAR_TB2,
        x_m=x,
        y_m=y,
        z_m=z,
        heading_deg=heading_deg,
        speed_mps=speed,
    )


def motion_summary() -> dict:
    world = DEFAULT_WORLD
    lim = world.half_size_m - _MARGIN_M
    return {
        "path": "circular orbit inside 1 km²",
        "orbit_radius_m": _ORBIT_RADIUS_M,
        "orbit_period_s": _ORBIT_PERIOD_S,
        "altitude_min_m": _ALT_CENTER_M - _ALT_AMPLITUDE_M,
        "altitude_max_m": _ALT_CENTER_M + _ALT_AMPLITUDE_M,
        "altitude_period_s": _ALT_PERIOD_S,
        "horizontal_limit_m": lim,
        "stays_in_area": True,
        "ascends_and_descends": True,
    }


# Snapshot at t=0 for static imports / back-compat
PHASE3_STATIC_TB2 = truth_at(0.0)
PHASE3_STATIC_DRONE = PHASE3_STATIC_TB2
