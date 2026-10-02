"""Shared tracking schemas."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Detection:
    """Pixel observation from one camera. No world coordinates."""

    camera_id: str
    t: float
    u: float
    v: float
    confidence: float = 1.0

    def as_dict(self) -> dict:
        return asdict(self)
