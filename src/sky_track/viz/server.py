"""Minimal HTTP server for the Sky Track dashboard + sim config API."""

from __future__ import annotations

import argparse
import functools
import http.server
import json
import shutil
import socketserver
import subprocess
import webbrowser
from pathlib import Path
from urllib.parse import urlparse

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765

STATIC_DIR = Path(__file__).resolve().parent / "static"


class SkyTrackHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, directory: str | None = None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, format: str, *args) -> None:
        print(f"[sky-track] {args[0]}")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/api/sim_config.json", "/sim_config.json"):
            self._serve_sim_config()
            return
        if parsed.path in ("/api/tracks.json", "/tracks.json"):
            self._serve_tracks()
            return
        super().do_GET()

    def _serve_json(self, payload: dict, *, ok: bool = True) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(200 if ok else 500)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_sim_config(self) -> None:
        try:
            from sky_track.sim.scenario import build_sim_config

            self._serve_json(build_sim_config())
        except Exception as exc:  # noqa: BLE001
            self._serve_json({"error": str(exc)}, ok=False)

    def _serve_tracks(self) -> None:
        try:
            from sky_track.sim.tracker_run import run_tracker_once

            self._serve_json(run_tracker_once())
        except Exception as exc:  # noqa: BLE001
            self._serve_json({"error": str(exc)}, ok=False)


def _open_external_browser(url: str) -> None:
    """Prefer the OS default browser so Cursor does not swallow localhost links."""
    for opener in ("xdg-open", "gio"):
        path = shutil.which(opener)
        if path:
            try:
                subprocess.Popen(
                    [path, url],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                return
            except OSError:
                pass
    try:
        webbrowser.open(url, new=2)
    except Exception:
        pass


def serve(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, open_browser: bool = True) -> None:
    if not (STATIC_DIR / "index.html").is_file():
        raise FileNotFoundError(f"Missing dashboard at {STATIC_DIR / 'index.html'}")

    handler = functools.partial(SkyTrackHandler, directory=str(STATIC_DIR))
    socketserver.TCPServer.allow_reuse_address = True

    with socketserver.TCPServer((host, port), handler) as httpd:
        url = f"http://{host}:{port}/"
        print("Sky Track sandbox (realistic pinhole cameras)")
        print(f"Open: {url}")
        print(f"Config: http://{host}:{port}/api/sim_config.json")
        print("Press Ctrl+C to stop.")
        if open_browser:
            _open_external_browser(url)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run the Sky Track sandbox dashboard")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open a browser tab automatically",
    )
    args = parser.parse_args(argv)
    serve(host=args.host, port=args.port, open_browser=not args.no_browser)


if __name__ == "__main__":
    main()
