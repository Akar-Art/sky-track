"""Multi-view triangulation from camera rays (no ground-truth access)."""

from __future__ import annotations

from dataclasses import dataclass

from .camera import (
    _norm,
    _sub,
    CameraIntrinsics,
    CameraPose,
    Vec3,
    pixel_to_ray,
)
from .schemas import Detection


@dataclass(frozen=True)
class Ray:
    origin: Vec3
    direction: Vec3  # unit


def detection_to_ray(
    detection: Detection,
    intrinsics: CameraIntrinsics,
    pose: CameraPose,
) -> Ray:
    origin, direction = pixel_to_ray(intrinsics, pose, detection.u, detection.v)
    return Ray(origin=origin, direction=direction)


def triangulate_rays(rays: list[Ray]) -> Vec3 | None:
    """
    Closest-point least squares for N≥2 skew rays.

    Solves for X minimizing sum_i || (X - O_i) × d_i ||^2
    which is equivalent to the linear system from expanding
    (I - d d^T)(X - O) = 0 stacked.
    """
    if len(rays) < 2:
        return None

    # Accumulate 3x3 system A X = b
    a00 = a01 = a02 = 0.0
    a11 = a12 = 0.0
    a22 = 0.0
    b0 = b1 = b2 = 0.0

    for ray in rays:
        ox, oy, oz = ray.origin
        dx, dy, dz = ray.direction
        # P = I - d d^T
        p00 = 1.0 - dx * dx
        p01 = -dx * dy
        p02 = -dx * dz
        p11 = 1.0 - dy * dy
        p12 = -dy * dz
        p22 = 1.0 - dz * dz
        a00 += p00
        a01 += p01
        a02 += p02
        a11 += p11
        a12 += p12
        a22 += p22
        # b += P O
        b0 += p00 * ox + p01 * oy + p02 * oz
        b1 += p01 * ox + p11 * oy + p12 * oz
        b2 += p02 * ox + p12 * oy + p22 * oz

    # Solve symmetric 3x3 via Cramer's rule / explicit inverse
    # Matrix:
    # [a00 a01 a02]
    # [a01 a11 a12]
    # [a02 a12 a22]
    det = (
        a00 * (a11 * a22 - a12 * a12)
        - a01 * (a01 * a22 - a12 * a02)
        + a02 * (a01 * a12 - a11 * a02)
    )
    if abs(det) < 1e-12:
        return None

    inv00 = (a11 * a22 - a12 * a12) / det
    inv01 = (a02 * a12 - a01 * a22) / det
    inv02 = (a01 * a12 - a02 * a11) / det
    inv11 = (a00 * a22 - a02 * a02) / det
    inv12 = (a01 * a02 - a00 * a12) / det
    inv22 = (a00 * a11 - a01 * a01) / det

    x = inv00 * b0 + inv01 * b1 + inv02 * b2
    y = inv01 * b0 + inv11 * b1 + inv12 * b2
    z = inv02 * b0 + inv12 * b1 + inv22 * b2
    return (float(x), float(y), float(z))


def mean_ray_residual_m(point: Vec3, rays: list[Ray]) -> float:
    """Average perpendicular distance from point to each ray (metres)."""
    if not rays:
        return float("inf")
    total = 0.0
    for ray in rays:
        # || (X - O) × d ||
        w = _sub(point, ray.origin)
        cx = w[1] * ray.direction[2] - w[2] * ray.direction[1]
        cy = w[2] * ray.direction[0] - w[0] * ray.direction[2]
        cz = w[0] * ray.direction[1] - w[1] * ray.direction[0]
        total += _norm((cx, cy, cz))
    return total / len(rays)


@dataclass(frozen=True)
class TrackEstimate:
    """Tracker output — estimated world position only (never ground truth)."""

    track_id: str
    type: str
    x: float
    y: float
    z: float
    cameras: int
    residual_m: float
    status: str = "live"
    speed: float = 0.0

    def as_dict(self) -> dict:
        return {
            "id": self.track_id,
            "type": self.type,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "z": round(self.z, 2),
            "cameras": self.cameras,
            "residual_m": round(self.residual_m, 3),
            "status": self.status,
            "speed": self.speed,
        }


def triangulate_detections(
    detections: list[Detection],
    cameras_by_id: dict[str, tuple[CameraIntrinsics, CameraPose]],
    *,
    track_id: str = "TRK-01",
    target_type: str = "uav",
) -> TrackEstimate | None:
    """
    Build rays from pixel detections and triangulate.

    `cameras_by_id` maps camera_id → (intrinsics, pose).
    Does not accept or use any ground-truth XYZ.
    """
    rays: list[Ray] = []
    for det in detections:
        cam = cameras_by_id.get(det.camera_id)
        if cam is None:
            continue
        intrinsics, pose = cam
        rays.append(detection_to_ray(det, intrinsics, pose))

    if len(rays) < 2:
        return None

    point = triangulate_rays(rays)
    if point is None:
        return None

    return TrackEstimate(
        track_id=track_id,
        type=target_type,
        x=point[0],
        y=point[1],
        z=point[2],
        cameras=len(rays),
        residual_m=mean_ray_residual_m(point, rays),
        status="live",
    )
