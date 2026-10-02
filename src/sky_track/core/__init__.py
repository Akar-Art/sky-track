"""Shared tracking core."""

from .camera import (
    CameraIntrinsics,
    CameraPose,
    image_corner_rays,
    intrinsics_from_diagonal_fov,
    intrinsics_from_sensor,
    is_visible,
    look_at_rotation,
    pixel_to_ray,
    project_point,
)
from .schemas import Detection
from .triangulation import TrackEstimate, triangulate_detections, triangulate_rays

__all__ = [
    "CameraIntrinsics",
    "CameraPose",
    "Detection",
    "TrackEstimate",
    "image_corner_rays",
    "intrinsics_from_diagonal_fov",
    "intrinsics_from_sensor",
    "is_visible",
    "look_at_rotation",
    "pixel_to_ray",
    "project_point",
    "triangulate_detections",
    "triangulate_rays",
]
