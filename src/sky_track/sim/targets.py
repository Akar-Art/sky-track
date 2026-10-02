"""Simulated aerial targets. Ground truth is operator-only — never fed to the tracker."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AirframeModel:
    """Physical platform used for visuals and later detection sizing."""

    name: str
    manufacturer: str
    length_m: float
    wingspan_m: float
    height_m: float
    mtow_kg: float
    cruise_speed_mps: float  # ~70 KTAS
    max_speed_mps: float  # ~110–120 KTAS
    operational_altitude_m: float  # published operational altitude
    service_ceiling_m: float
    notes: str = ""

    @property
    def characteristic_length_m(self) -> float:
        """Size used for rough angular / pixel estimates (wingspan)."""
        return self.wingspan_m

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "manufacturer": self.manufacturer,
            "length_m": self.length_m,
            "wingspan_m": self.wingspan_m,
            "height_m": self.height_m,
            "mtow_kg": self.mtow_kg,
            "cruise_speed_mps": self.cruise_speed_mps,
            "cruise_speed_kmh": round(self.cruise_speed_mps * 3.6, 1),
            "max_speed_mps": self.max_speed_mps,
            "max_speed_kmh": round(self.max_speed_mps * 3.6, 1),
            "operational_altitude_m": round(self.operational_altitude_m, 1),
            "service_ceiling_m": round(self.service_ceiling_m, 1),
            "notes": self.notes,
        }


# Public Baykar specs (baykartech.com): 12 m span, 6.5 m length, 2.2 m height,
# 700 kg MTOW, 90–110 KTAS travel, 16,000 ft operational / 22,000 ft ceiling.
BAYRAKTAR_TB2 = AirframeModel(
    name="Bayraktar TB2",
    manufacturer="Baykar",
    length_m=6.5,
    wingspan_m=12.0,
    height_m=2.2,
    mtow_kg=700.0,
    cruise_speed_mps=70.0 * 0.514444,  # 70 KTAS
    max_speed_mps=110.0 * 0.514444,  # 110 KTAS
    operational_altitude_m=16000.0 * 0.3048,  # 16,000 ft ≈ 4877 m
    service_ceiling_m=22000.0 * 0.3048,  # 22,000 ft ≈ 6706 m
    notes=(
        "MALE tactical UAV. Inverted V-tail, pusher prop. "
        "Specs from Baykar public data; used for size/speed/altitude in the sandbox."
    ),
)


@dataclass(frozen=True)
class TargetTruth:
    """
    Authoritative world state for the operator / scoring.

    The tracking core must never read these fields. In the live system this
    does not exist; in the sandbox it is used only for visualization and
    private operator readout.
    """

    target_id: str
    kind: str  # e.g. "uav"
    airframe: AirframeModel
    # East, up, north (meters), origin at area center
    x_m: float
    y_m: float
    z_m: float
    # Heading: 0 = +Z north, degrees clockwise toward +X east (aviation-style from north)
    heading_deg: float = 45.0
    speed_mps: float = 0.0

    def as_dict(self) -> dict:
        return {
            "id": self.target_id,
            "type": self.kind,
            "model": self.airframe.name,
            "x": self.x_m,
            "y": self.y_m,
            "z": self.z_m,
            "heading_deg": self.heading_deg,
            "speed_mps": self.speed_mps,
            "speed_kmh": round(self.speed_mps * 3.6, 1),
            "airframe": self.airframe.as_dict(),
        }


def pixel_span_estimate(fx_px: float, size_m: float, range_m: float) -> float:
    """Approximate image span in pixels for an object of length size_m at range_m."""
    if range_m <= 1e-6:
        return float("inf")
    return fx_px * size_m / range_m
