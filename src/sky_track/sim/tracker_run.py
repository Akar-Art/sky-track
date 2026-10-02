"""Run one tracking cycle: detections → triangulation → track list."""

from __future__ import annotations

import time

from sky_track.core.triangulation import triangulate_detections
from sky_track.sim.cameras import corner_cameras
from sky_track.sim.motion import motion_summary, truth_at
from sky_track.sim.projector import project_truth_to_detections

# Wall-clock origin so motion is continuous across requests
_T0 = time.monotonic()


def sim_time_s() -> float:
    return time.monotonic() - _T0


def run_tracker_once(*, t: float | None = None) -> dict:
    """
    Simulation harness: generate detections from hidden moving truth, then track.

    Returns tracker-facing payload (no truth XYZ in `tracks`), plus a separate
    operator_truth block for the private HUD / scoring only.
    """
    if t is None:
        t = sim_time_s()

    mounts = corner_cameras()
    truth = truth_at(t)

    detections = project_truth_to_detections(truth, mounts, t=t)
    cameras_by_id = {m.camera_id: (m.intrinsics, m.pose) for m in mounts}

    estimate = triangulate_detections(
        detections,
        cameras_by_id,
        track_id="TRK-01",
        target_type=truth.kind,
    )

    tracks: list[dict] = []
    if estimate is not None:
        # Attach measured ground speed from truth motion for monitor display
        # (derived from track deltas later; for now use sim speed on estimate)
        d = estimate.as_dict()
        d["speed"] = round(truth.speed_mps, 2)
        tracks.append(d)

    error_m = None
    if estimate is not None:
        dx = estimate.x - truth.x_m
        dy = estimate.y - truth.y_m
        dz = estimate.z - truth.z_m
        error_m = (dx * dx + dy * dy + dz * dz) ** 0.5

    return {
        "t": round(t, 3),
        "detections": [d.as_dict() for d in detections],
        "tracks": tracks,
        "operator_truth": {
            **truth.as_dict(),
            "private": True,
            "note": "Operator-only — not an input to triangulation.",
        },
        "operator_score": {
            "track_id": estimate.track_id if estimate else None,
            "error_m": None if error_m is None else round(error_m, 4),
            "note": "Scoring only — tracker never receives truth XYZ as input.",
        },
        "motion": motion_summary(),
    }
