"""Corner mounts: Raspberry Pi HQ Camera + 16 mm C-mount (track-class, looks up)."""

from __future__ import annotations

from dataclasses import dataclass
import math

from sky_track.core.camera import (
    CameraIntrinsics,
    CameraPose,
    Vec3,
    image_corner_rays,
    intrinsics_from_sensor,
    is_visible,
    look_at_rotation,
    project_point,
)

from .world import DEFAULT_WORLD, WorldConfig


def _look_up_inward(
    corner_x: float,
    corner_z: float,
    tilt_from_zenith_deg: float,
) -> Vec3:
    """Near-zenith optical axis with a small lean toward the area center."""
    tilt = math.radians(tilt_from_zenith_deg)
    hx, hz = -corner_x, -corner_z
    hlen = math.hypot(hx, hz) or 1.0
    hx /= hlen
    hz /= hlen
    dx = hx * math.sin(tilt)
    dy = math.cos(tilt)
    dz = hz * math.sin(tilt)
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    return (dx / n, dy / n, dz / n)


@dataclass(frozen=True)
class CameraModel:
    """
    Raspberry Pi High Quality Camera (IMX477) + 16 mm C-mount lens.

    Built for resolving a TB2-class UAV at MALE altitude with upward-looking
    corner mounts. Narrower FOV than a "wide" module; much longer effective focal length.

    Street prices (USD, approx. retail 2025–2026 — camera + lens only, no Pi/compute):
      - HQ module: ~$55–65
      - 16 mm C-mount (Arducam-class): ~$50–65
      - 16 mm official/CGL telephoto: ~$100–125
      → ~$115–190 per corner; ×4 corners ≈ $460–760 for optics
    """

    name: str = "Raspberry Pi HQ + 16mm C-mount"
    # Cost bands (optics only)
    price_camera_usd: tuple[float, float] = (55.0, 65.0)
    price_lens_budget_usd: tuple[float, float] = (50.0, 65.0)
    price_lens_official_usd: tuple[float, float] = (100.0, 125.0)
    # IMX477 full still resolution
    image_width_px: int = 4056
    image_height_px: int = 3040
    # Active area from 1.55 µm pixels (Raspberry Pi HQ / IMX477)
    sensor_width_mm: float = 6.287
    sensor_height_mm: float = 4.712
    focal_length_mm: float = 16.0
    mount_height_m: float = 1.5
    tilt_from_zenith_deg: float = 12.0

    @property
    def price_band(self) -> str:
        lo = self.price_camera_usd[0] + self.price_lens_budget_usd[0]
        hi = self.price_camera_usd[1] + self.price_lens_official_usd[1]
        return f"~${lo:.0f}–{hi:.0f} / corner (optics)"

    @property
    def price_four_corners_band(self) -> str:
        lo = 4 * (self.price_camera_usd[0] + self.price_lens_budget_usd[0])
        hi = 4 * (self.price_camera_usd[1] + self.price_lens_official_usd[1])
        return f"~${lo:.0f}–{hi:.0f} for 4 corners (optics only)"

    def intrinsics(self) -> CameraIntrinsics:
        return intrinsics_from_sensor(
            width_px=self.image_width_px,
            height_px=self.image_height_px,
            sensor_width_mm=self.sensor_width_mm,
            sensor_height_mm=self.sensor_height_mm,
            focal_length_mm=self.focal_length_mm,
        )


DEFAULT_CAMERA_MODEL = CameraModel()


@dataclass(frozen=True)
class CameraMount:
    camera_id: str
    corner: str
    pose: CameraPose
    intrinsics: CameraIntrinsics
    model: CameraModel = DEFAULT_CAMERA_MODEL

    @property
    def position_m(self) -> Vec3:
        return self.pose.center_m

    @property
    def look_dir(self) -> Vec3:
        return self.pose.forward_world

    def sees(self, point_world: Vec3) -> bool:
        return is_visible(self.intrinsics, self.pose, point_world)

    def project(self, point_world: Vec3) -> tuple[float, float, float] | None:
        return project_point(self.intrinsics, self.pose, point_world)

    def frustum_corners_world(self, depth_m: float = 6000.0) -> list[Vec3]:
        return image_corner_rays(self.intrinsics, self.pose, depth_m)

    def as_dict(self) -> dict:
        K = self.intrinsics
        R = self.pose.R_world_cam
        return {
            "camera_id": self.camera_id,
            "corner": self.corner,
            "position_m": list(self.position_m),
            "look_dir": list(self.look_dir),
            "R_world_cam": [list(row) for row in R],
            "intrinsics": K.as_dict(),
            "tilt_from_zenith_deg": self.model.tilt_from_zenith_deg,
            "frustum_depth_m": 6000.0,
            "frustum_corners_m": [list(c) for c in self.frustum_corners_world(6000.0)],
        }


def corner_cameras(
    world: WorldConfig = DEFAULT_WORLD,
    model: CameraModel = DEFAULT_CAMERA_MODEL,
) -> tuple[CameraMount, ...]:
    h = world.half_size_m
    y = model.mount_height_m
    K = model.intrinsics()

    specs = (
        ("cam_sw", "SW", -h, -h),
        ("cam_se", "SE", h, -h),
        ("cam_ne", "NE", h, h),
        ("cam_nw", "NW", -h, h),
    )
    mounts: list[CameraMount] = []
    for cam_id, corner, x, z in specs:
        center: Vec3 = (x, y, z)
        forward = _look_up_inward(x, z, model.tilt_from_zenith_deg)
        target: Vec3 = (
            center[0] + forward[0] * 1000.0,
            center[1] + forward[1] * 1000.0,
            center[2] + forward[2] * 1000.0,
        )
        R = look_at_rotation(center, target)
        pose = CameraPose(camera_id=cam_id, center_m=center, R_world_cam=R)
        mounts.append(
            CameraMount(
                camera_id=cam_id,
                corner=corner,
                pose=pose,
                intrinsics=K,
                model=model,
            )
        )
    return tuple(mounts)


def summarize_model(model: CameraModel = DEFAULT_CAMERA_MODEL) -> dict:
    K = model.intrinsics()
    return {
        "name": model.name,
        "role": "track (narrow FOV, long focal length)",
        "aim": "upward (near zenith)",
        "tilt_from_zenith_deg": model.tilt_from_zenith_deg,
        "mount_height_m": model.mount_height_m,
        "focal_length_mm": model.focal_length_mm,
        "sensor": "Sony IMX477 (Raspberry Pi HQ), 1/2.3\"",
        "price_band": model.price_band,
        "price_four_corners_band": model.price_four_corners_band,
        "bill_of_materials_usd": {
            "hq_camera_module": {
                "low": model.price_camera_usd[0],
                "high": model.price_camera_usd[1],
                "note": "Adafruit/SparkFun-class retail",
            },
            "lens_16mm_budget": {
                "low": model.price_lens_budget_usd[0],
                "high": model.price_lens_budget_usd[1],
                "note": "Arducam-class C-mount 16mm",
            },
            "lens_16mm_official": {
                "low": model.price_lens_official_usd[0],
                "high": model.price_lens_official_usd[1],
                "note": "CGL / official telephoto 16mm",
            },
            "per_corner_optics": {
                "budget": model.price_camera_usd[0] + model.price_lens_budget_usd[0],
                "premium": model.price_camera_usd[1] + model.price_lens_official_usd[1],
            },
            "four_corners_optics": {
                "budget": 4
                * (model.price_camera_usd[0] + model.price_lens_budget_usd[0]),
                "premium": 4
                * (model.price_camera_usd[1] + model.price_lens_official_usd[1]),
            },
            "not_included": "Raspberry Pi SBC, PoE, enclosure, mast, cabling (~$50–120+ extra per site)",
        },
        "intrinsics": K.as_dict(),
        "derived_hfov_deg": round(K.hfov_deg, 2),
        "derived_vfov_deg": round(K.vfov_deg, 2),
        "derived_dfov_deg": round(K.dfov_deg, 2),
        "focal_length_px": {"fx": K.fx, "fy": K.fy},
        "notes": (
            "Pinhole from IMX477 active area + 16 mm focal length. "
            "Looks up with slight inward tilt. "
            "Visibility = project inside the image."
        ),
    }
