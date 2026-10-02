"""Realistic pinhole camera projection (OpenCV convention), stdlib only."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _sub(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _norm(a)
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return _scale(a, 1.0 / n)


def _mat_vec(
    R: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def _transpose(
    R: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return (
        (R[0][0], R[1][0], R[2][0]),
        (R[0][1], R[1][1], R[2][1]),
        (R[0][2], R[1][2], R[2][2]),
    )


Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


@dataclass(frozen=True)
class CameraIntrinsics:
    """
    Pinhole intrinsics from a real sensor + lens.

    OpenCV convention: +X right, +Y down, +Z forward (into the scene).
    """

    width_px: int
    height_px: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(self.width_px / (2.0 * self.fx)))

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(self.height_px / (2.0 * self.fy)))

    @property
    def dfov_deg(self) -> float:
        half_w = self.width_px / 2.0
        half_h = self.height_px / 2.0
        # Use fx (≈ fy) with image diagonal in pixel space
        return math.degrees(2.0 * math.atan(math.hypot(half_w, half_h) / self.fx))

    def as_dict(self) -> dict:
        return {
            "width_px": self.width_px,
            "height_px": self.height_px,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "k1": self.k1,
            "k2": self.k2,
            "p1": self.p1,
            "p2": self.p2,
            "hfov_deg": self.hfov_deg,
            "vfov_deg": self.vfov_deg,
            "dfov_deg": self.dfov_deg,
        }


def intrinsics_from_sensor(
    *,
    width_px: int,
    height_px: int,
    sensor_width_mm: float,
    sensor_height_mm: float,
    focal_length_mm: float,
) -> CameraIntrinsics:
    fx = focal_length_mm * width_px / sensor_width_mm
    fy = focal_length_mm * height_px / sensor_height_mm
    return CameraIntrinsics(
        width_px=width_px,
        height_px=height_px,
        fx=fx,
        fy=fy,
        cx=(width_px - 1) / 2.0,
        cy=(height_px - 1) / 2.0,
    )


def intrinsics_from_diagonal_fov(
    *,
    width_px: int,
    height_px: int,
    sensor_diagonal_mm: float,
    diagonal_fov_deg: float,
) -> CameraIntrinsics:
    """
    Derive focal length from marketed diagonal FOV + sensor diagonal,
    then build pinhole intrinsics (common for consumer wide modules).
    """
    aspect = width_px / height_px
    sensor_height = sensor_diagonal_mm / math.hypot(aspect, 1.0)
    sensor_width = sensor_height * aspect
    f_mm = (sensor_diagonal_mm / 2.0) / math.tan(math.radians(diagonal_fov_deg) / 2.0)
    return intrinsics_from_sensor(
        width_px=width_px,
        height_px=height_px,
        sensor_width_mm=sensor_width,
        sensor_height_mm=sensor_height,
        focal_length_mm=f_mm,
    )


@dataclass(frozen=True)
class CameraPose:
    """World pose: center C and R_world_cam (columns = cam axes in world)."""

    camera_id: str
    center_m: Vec3
    R_world_cam: Mat3

    @property
    def forward_world(self) -> Vec3:
        # column 2
        R = self.R_world_cam
        return (R[0][2], R[1][2], R[2][2])

    def world_to_cam(self, point_world: Vec3) -> Vec3:
        d = _sub(point_world, self.center_m)
        return _mat_vec(_transpose(self.R_world_cam), d)


def look_at_rotation(center: Vec3, target: Vec3, world_up: Vec3 | None = None) -> Mat3:
    """
    Build R_world_cam so camera +Z looks toward target.
    OpenCV: +X right, +Y down — align -Y with world_up when possible.
    """
    if world_up is None:
        world_up = (0.0, 1.0, 0.0)
    forward = _unit(_sub(target, center))
    down_hint = _scale(world_up, -1.0)
    right = _cross(down_hint, forward)
    if _norm(right) < 1e-8:
        down_hint = (-1.0, 0.0, 0.0)
        right = _cross(down_hint, forward)
    right = _unit(right)
    down = _unit(_cross(forward, right))
    # columns: right, down, forward
    return (
        (right[0], down[0], forward[0]),
        (right[1], down[1], forward[1]),
        (right[2], down[2], forward[2]),
    )


def project_point(
    intrinsics: CameraIntrinsics,
    pose: CameraPose,
    point_world: Vec3,
) -> tuple[float, float, float] | None:
    """Return (u, v, depth_cam_z) or None if behind the camera."""
    p_cam = pose.world_to_cam(point_world)
    if p_cam[2] <= 1e-6:
        return None
    u = intrinsics.fx * (p_cam[0] / p_cam[2]) + intrinsics.cx
    v = intrinsics.fy * (p_cam[1] / p_cam[2]) + intrinsics.cy
    return float(u), float(v), float(p_cam[2])


def is_visible(
    intrinsics: CameraIntrinsics,
    pose: CameraPose,
    point_world: Vec3,
    *,
    margin_px: float = 0.0,
) -> bool:
    proj = project_point(intrinsics, pose, point_world)
    if proj is None:
        return False
    u, v, _ = proj
    return (
        -margin_px <= u < intrinsics.width_px + margin_px
        and -margin_px <= v < intrinsics.height_px + margin_px
    )


def pixel_to_ray(
    intrinsics: CameraIntrinsics,
    pose: CameraPose,
    u: float,
    v: float,
) -> tuple[Vec3, Vec3]:
    """Return (origin_world, direction_world_unit) for a pixel ray."""
    x = (u - intrinsics.cx) / intrinsics.fx
    y = (v - intrinsics.cy) / intrinsics.fy
    dir_cam = _unit((x, y, 1.0))
    dir_world = _unit(_mat_vec(pose.R_world_cam, dir_cam))
    return pose.center_m, dir_world


def image_corner_rays(
    intrinsics: CameraIntrinsics,
    pose: CameraPose,
    depth: float,
) -> list[Vec3]:
    corners = [
        (0.0, 0.0),
        (float(intrinsics.width_px - 1), 0.0),
        (float(intrinsics.width_px - 1), float(intrinsics.height_px - 1)),
        (0.0, float(intrinsics.height_px - 1)),
    ]
    points: list[Vec3] = []
    for u, v in corners:
        origin, direction = pixel_to_ray(intrinsics, pose, u, v)
        points.append(_add(origin, _scale(direction, depth)))
    return points
