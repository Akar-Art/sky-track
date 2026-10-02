"""Coverage / blind-spot analysis using realistic pinhole projection."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from sky_track.sim.cameras import DEFAULT_CAMERA_MODEL, corner_cameras, summarize_model
from sky_track.sim.motion import truth_at
from sky_track.sim.targets import pixel_span_estimate
from sky_track.sim.world import DEFAULT_WORLD


def sample_counts(mounts, alt: float, half: float, step: float):
    xs = []
    x = -half
    while x <= half + 1e-9:
        xs.append(x)
        x += step
    counts = [0, 0, 0, 0, 0]
    for xi in xs:
        for zi in xs:
            nsee = sum(1 for m in mounts if m.sees((xi, alt, zi)))
            counts[nsee] += 1
    n = len(xs) * len(xs)
    return counts, n, step


def main() -> None:
    mounts = corner_cameras()
    summary = summarize_model(DEFAULT_CAMERA_MODEL)
    half = DEFAULT_WORLD.half_size_m
    step = 25.0
    tb2 = truth_at(0.0)
    fx = DEFAULT_CAMERA_MODEL.intrinsics().fx

    print("Camera:", summary["name"])
    print(
        f"  focal={summary['focal_length_mm']} mm  "
        f"HFOV={summary['derived_hfov_deg']}°  "
        f"VFOV={summary['derived_vfov_deg']}°  "
        f"DFOV={summary['derived_dfov_deg']}°"
    )
    print(
        f"  resolution={DEFAULT_CAMERA_MODEL.image_width_px}x{DEFAULT_CAMERA_MODEL.image_height_px}  "
        f"fx={summary['intrinsics']['fx']:.1f}px  fy={summary['intrinsics']['fy']:.1f}px"
    )
    print(f"  cost: {summary['price_band']}  |  {summary['price_four_corners_band']}")
    print("  visibility = pinhole project-to-pixel (inside image bounds)")
    print()
    print("Coverage over 1 km² (sample every 25 m):")
    print(
        f"{'alt_m':>6} {'blind%':>7} {'1cam%':>7} {'2+%':>7} "
        f"{'3+%':>7} {'4cam%':>7} {'blind_km2':>9}"
    )

    alts = [50, 100, 180, 300, 500, 800, 1500, 3000, int(round(tb2.y_m)), 6000]
    for alt in alts:
        counts, n, st = sample_counts(mounts, float(alt), half, step)
        pct = lambda c: 100.0 * c / n
        blind_km2 = counts[0] * st * st / 1e6
        print(
            f"{alt:6d} {pct(counts[0]):6.1f}% {pct(counts[1]):6.1f}% "
            f"{pct(sum(counts[2:])):6.1f}% {pct(sum(counts[3:])):6.1f}% "
            f"{pct(counts[4]):6.1f}% {blind_km2:9.3f}"
        )

    pt = (tb2.x_m, tb2.y_m, tb2.z_m)
    print()
    print(
        f"{tb2.airframe.name} {tb2.target_id} at "
        f"({tb2.x_m}, {tb2.y_m:.1f} m, {tb2.z_m})"
    )
    print(
        f"  {tb2.airframe.wingspan_m} m span × {tb2.airframe.length_m} m, "
        f"cruise {tb2.speed_mps:.1f} m/s ({tb2.speed_mps * 3.6:.0f} km/h)"
    )
    for m in mounts:
        proj = m.project(pt)
        dx = tb2.x_m - m.position_m[0]
        dy = tb2.y_m - m.position_m[1]
        dz = tb2.z_m - m.position_m[2]
        slant = (dx * dx + dy * dy + dz * dz) ** 0.5
        wings_px = pixel_span_estimate(fx, tb2.airframe.wingspan_m, slant)
        if proj is None:
            print(f"  {m.corner}: behind camera")
        elif m.sees(pt):
            print(
                f"  {m.corner}: SEES  u={proj[0]:.1f} v={proj[1]:.1f}  "
                f"slant={slant:.0f}m  wingspan≈{wings_px:.2f}px"
            )
        else:
            print(
                f"  {m.corner}: outside image  u={proj[0]:.1f} v={proj[1]:.1f}  "
                f"slant={slant:.0f}m  wingspan≈{wings_px:.2f}px"
            )


if __name__ == "__main__":
    main()
