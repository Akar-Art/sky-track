"""Build the shared simulation config consumed by the dashboard."""

from __future__ import annotations

import math

from sky_track.sim.cameras import DEFAULT_CAMERA_MODEL, corner_cameras, summarize_model
from sky_track.sim.motion import motion_summary, truth_at
from sky_track.sim.targets import pixel_span_estimate
from sky_track.sim.tracker_run import run_tracker_once
from sky_track.sim.world import DEFAULT_WORLD


def build_sim_config() -> dict:
    mounts = corner_cameras()
    # Static scene setup uses t=0 pose; live motion comes from /api/tracks.json
    truth = truth_at(0.0)
    truth_pt = (truth.x_m, truth.y_m, truth.z_m)
    fx = DEFAULT_CAMERA_MODEL.intrinsics().fx
    span = truth.airframe.wingspan_m

    cameras = []
    seeing = []
    for m in mounts:
        d = m.as_dict()
        # Draw frustums far enough to reach MALE altitudes
        d["frustum_depth_m"] = 6000.0
        d["frustum_corners_m"] = [
            list(c) for c in m.frustum_corners_world(6000.0)
        ]
        vis = m.sees(truth_pt)
        d["sees_operator_truth"] = vis
        proj = m.project(truth_pt)
        if proj is None:
            d["truth_projection"] = None
            d["truth_range_m"] = None
            d["truth_wingspan_px"] = None
        else:
            range_m = proj[2]
            # Approximate slant range from camera center
            dx = truth.x_m - m.position_m[0]
            dy = truth.y_m - m.position_m[1]
            dz = truth.z_m - m.position_m[2]
            slant = math.sqrt(dx * dx + dy * dy + dz * dz)
            d["truth_projection"] = {"u": proj[0], "v": proj[1], "depth_m": proj[2]}
            d["truth_range_m"] = slant
            d["truth_wingspan_px"] = pixel_span_estimate(fx, span, slant)
        if vis:
            seeing.append(m.corner)
        cameras.append(d)

    ranges = [c["truth_range_m"] for c in cameras if c["truth_range_m"] is not None]
    px_spans = [
        c["truth_wingspan_px"] for c in cameras if c["truth_wingspan_px"] is not None
    ]

    return {
        "phase": "realistic-camera",
        "units": "metric",
        "world": {
            "size_m": DEFAULT_WORLD.size_m,
            "grid_step_m": DEFAULT_WORLD.grid_step_m,
            "axes": "X east, Y up, Z north (metres)",
        },
        "camera_model": summarize_model(DEFAULT_CAMERA_MODEL),
        "cameras": cameras,
        "operator_truth": {
            **truth.as_dict(),
            "seen_by_corners": seeing,
            "private": True,
            "note": "Not available to the tracker / Sky Monitor.",
            "slant_range_m_per_cam": {
                c["corner"]: c["truth_range_m"] for c in cameras
            },
            "wingspan_px_per_cam": {
                c["corner"]: None
                if c["truth_wingspan_px"] is None
                else round(c["truth_wingspan_px"], 2)
                for c in cameras
            },
            "mean_slant_range_m": None
            if not ranges
            else round(sum(ranges) / len(ranges), 1),
            "mean_wingspan_px": None
            if not px_spans
            else round(sum(px_spans) / len(px_spans), 2),
        },
        "detection_envelope_tb2": {
            "target": "Bayraktar TB2 (12 m wingspan)",
            "camera": DEFAULT_CAMERA_MODEL.name,
            "assumptions": (
                "Upward corner cams over 1 km²; detect ≈ wingspan ≥10 px; "
                "track/triangulate needs ≥2 cameras."
            ),
            "triangulation_min_altitude_m": 2000,
            "triangulation_good_from_m": 2500,
            "comfortable_max_altitude_m": 6000,
            "usable_max_altitude_m": 10000,
            "marginal_max_altitude_m": 12000,
            "blind_below_m": 1200,
            "summary_m": "2 000–10 000 m AGL (best 2 500–6 000)",
        },
        "motion": motion_summary(),
        "coverage": {
            "slice_y_m": truth.y_m,
            "cell_m": 50.0,
            "method": "pinhole project-to-pixel",
        },
        "tracker": run_tracker_once(t=0.0),
    }
