"""Simulation world, targets, and synthetic camera projections."""

from .cameras import (
    DEFAULT_CAMERA_MODEL,
    CameraModel,
    CameraMount,
    corner_cameras,
    summarize_model,
)
from .motion import PHASE3_STATIC_DRONE, PHASE3_STATIC_TB2, motion_summary, truth_at
from .scenario import build_sim_config
from .targets import BAYRAKTAR_TB2, AirframeModel, TargetTruth
from .world import DEFAULT_WORLD, WorldConfig

__all__ = [
    "DEFAULT_WORLD",
    "WorldConfig",
    "DEFAULT_CAMERA_MODEL",
    "CameraModel",
    "CameraMount",
    "corner_cameras",
    "summarize_model",
    "build_sim_config",
    "BAYRAKTAR_TB2",
    "PHASE3_STATIC_DRONE",
    "PHASE3_STATIC_TB2",
    "AirframeModel",
    "TargetTruth",
    "truth_at",
    "motion_summary",
]
