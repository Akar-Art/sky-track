"""Default simulation world: 1 km × 1 km flat area."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WorldConfig:
    """Ground plane centered at the origin; corners at ±half_size meters."""

    size_m: float = 1000.0  # full side length
    grid_step_m: float = 100.0

    @property
    def half_size_m(self) -> float:
        return self.size_m / 2.0

    @property
    def corners_m(self) -> tuple[tuple[float, float, float], ...]:
        h = self.half_size_m
        return (
            (-h, 0.0, -h),
            (h, 0.0, -h),
            (h, 0.0, h),
            (-h, 0.0, h),
        )


DEFAULT_WORLD = WorldConfig()
