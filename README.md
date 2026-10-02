# Sky Track

Multi-camera aerial tracking sandbox (Bayraktar TB2–class targets).

## Cameras (current)

**Raspberry Pi HQ (IMX477) + 16 mm C-mount**, looking up.

| | |
|--|--|
| Role | Track cam (narrow FOV, long focal length) |
| Resolution | 4056 × 3040 |
| Focal length | 16 mm |
| Derived HFOV / VFOV | ~22° / ~17° |
| Aim | Upward (~12° from zenith, inward) |

### Cost (optics only, USD retail approx.)

| Item | Budget | Premium |
|------|--------|---------|
| HQ camera module | ~$55 | ~$65 |
| 16 mm C-mount lens | ~$50–65 (Arducam-class) | ~$100–125 (official/CGL) |
| **Per corner** | **~$105–130** | **~$165–190** |
| **4 corners** | **~$420–520** | **~$660–760** |

Not included: Raspberry Pi SBC, PoE, enclosure, mast, cabling (~$50–120+ per site).

Optional later: add a cheap wide “cue” cam (~$25–40) beside each track cam.

## How to run

```bash
cd sky-track
python3 scripts/run_sim.py
xdg-open http://127.0.0.1:8765/
```

Config: `http://127.0.0.1:8765/api/sim_config.json`

```bash
PYTHONPATH=src python3 scripts/analyze_coverage.py
```
