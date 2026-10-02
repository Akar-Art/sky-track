#!/usr/bin/env python3
"""Start the Sky Track sandbox dashboard."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sky_track.viz.server import main

if __name__ == "__main__":
    main()
