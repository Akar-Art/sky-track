"""Synthetic detections from simulation truth — used only inside the sim layer."""

from __future__ import annotations

from sky_track.core.schemas import Detection
from sky_track.sim.cameras import CameraMount
from sky_track.sim.targets import TargetTruth


def project_truth_to_detections(
    truth: TargetTruth,
    mounts: tuple[CameraMount, ...],
    *,
    t: float = 0.0,
) -> list[Detection]:
    """
    Sim-only: project operator truth into per-camera pixels.

    The tracker must receive only the returned Detection list — never truth XYZ.
    """
    point = (truth.x_m, truth.y_m, truth.z_m)
    out: list[Detection] = []
    for mount in mounts:
        if not mount.sees(point):
            continue
        proj = mount.project(point)
        if proj is None:
            continue
        u, v, _depth = proj
        out.append(
            Detection(
                camera_id=mount.camera_id,
                t=t,
                u=u,
                v=v,
                confidence=1.0,
            )
        )
    return out
