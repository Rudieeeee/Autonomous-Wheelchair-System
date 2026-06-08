"""
map_server.py — Local HTTP server for the JARVIS map point editor.
==================================================================

Converts the LiDAR PGM to a colour PNG, serves jarvis_map.html, and
exposes two REST endpoints so the browser tool can read and write
map_points.json without touching the filesystem directly.

Usage
-----
    python map_server.py

Then open http://localhost:5757/ in any browser.
Press Ctrl+C to stop.

Endpoints
---------
GET  /                 → jarvis_map.html
GET  /lidar_map.png    → colour-coded LiDAR occupancy map (same palette as GUI)
GET  /map_meta.json    → {"width":…,"height":…,"resolution":…,"origin_x":…,"origin_y":…}
GET  /points           → contents of map_points.json as JSON array
POST /points           → replace map_points.json with the posted JSON array
"""

from __future__ import annotations

import ast
import json
import os
import struct
import threading
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Paths (all relative to this file)
# ---------------------------------------------------------------------------
_HERE        = os.path.dirname(os.path.abspath(__file__))
_HTML_PATH   = os.path.join(_HERE, "jarvis_map.html")
_PGM_PATH    = os.path.join(_HERE, "Lidar Map", "my_map.pgm")
_YAML_PATH   = os.path.join(_HERE, "Lidar Map", "my_map.yaml")
_POINTS_PATH = os.path.join(_HERE, "map_points.json")
_PORT        = 5757

# ---------------------------------------------------------------------------
# Map metadata (populated from YAML at startup)
# ---------------------------------------------------------------------------
MAP_ORIGIN_X   = -47.471
MAP_ORIGIN_Y   = -30.341
MAP_RESOLUTION = 0.050
FREE_THRESH    = 0.196
OCC_THRESH     = 0.65
NEGATE         = 0

_PNG_BYTES:  bytes = b""
_IMG_WIDTH:  int   = 0
_IMG_HEIGHT: int   = 0


# ---------------------------------------------------------------------------
# YAML parser (no PyYAML dependency)
# ---------------------------------------------------------------------------
def _parse_yaml() -> None:
    global MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION, FREE_THRESH, OCC_THRESH, NEGATE
    if not os.path.exists(_YAML_PATH):
        return
    with open(_YAML_PATH, encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(":")
            key   = key.strip()
            value = value.strip()
            if key == "resolution":
                MAP_RESOLUTION = float(value)
            elif key == "origin":
                origin = ast.literal_eval(value)
                MAP_ORIGIN_X = float(origin[0])
                MAP_ORIGIN_Y = float(origin[1])
            elif key == "negate":
                NEGATE = int(value)
            elif key == "free_thresh":
                FREE_THRESH = float(value)
            elif key == "occupied_thresh":
                OCC_THRESH = float(value)


# ---------------------------------------------------------------------------
# PGM reader
# ---------------------------------------------------------------------------
def _read_pgm(path: str) -> Tuple[np.ndarray, int, int]:
    """Read a binary PGM (P5) and return (grayscale_array, width, height)."""
    with open(path, "rb") as fh:
        raw = fh.read()

    idx = 0

    def next_token() -> str:
        nonlocal idx
        # Skip whitespace and comment lines
        while idx < len(raw):
            ch = raw[idx:idx + 1]
            if ch == b"#":
                while idx < len(raw) and raw[idx:idx + 1] != b"\n":
                    idx += 1
            elif ch in (b" ", b"\t", b"\r", b"\n"):
                idx += 1
            else:
                break
        start = idx
        while idx < len(raw) and raw[idx:idx + 1] not in (b" ", b"\t", b"\r", b"\n"):
            idx += 1
        return raw[start:idx].decode("ascii")

    magic  = next_token()   # noqa: F841 — should be "P5"
    width  = int(next_token())
    height = int(next_token())
    maxval = int(next_token())
    idx += 1  # single whitespace byte that follows maxval

    bpp = 1 if maxval <= 255 else 2
    dtype = np.uint8 if bpp == 1 else np.dtype(">u2")
    gray = (
        np.frombuffer(raw[idx: idx + height * width * bpp], dtype=dtype)
        .reshape(height, width)
        .astype(np.uint8)
    )
    return gray, width, height


# ---------------------------------------------------------------------------
# Colour map — identical palette to jarvis_gui._pgm_to_rgb
# ---------------------------------------------------------------------------
def _pgm_to_rgb(gray: np.ndarray) -> np.ndarray:
    occ_prob = gray.astype(np.float32) / 255.0
    if NEGATE == 0:
        occ_prob = 1.0 - occ_prob

    height, width = gray.shape
    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    occupied = occ_prob > OCC_THRESH
    free     = occ_prob < FREE_THRESH
    unknown  = ~occupied & ~free

    rgb[unknown] = [10, 20, 40]   # dark blue  — unknown / unexplored
    rgb[free]    = [28, 48, 80]   # grey-blue  — navigable free space

    if occupied.any():
        v = occ_prob[occupied]
        rgb[occupied, 0] = np.clip(40  + v * 110, 0, 255).astype(np.uint8)
        rgb[occupied, 1] = np.clip(100 + v * 110, 0, 255).astype(np.uint8)
        rgb[occupied, 2] = np.clip(160 + v * 95,  0, 255).astype(np.uint8)

    return rgb


# ---------------------------------------------------------------------------
# Minimal PNG encoder (stdlib only)
# ---------------------------------------------------------------------------
def _encode_png(rgb: np.ndarray) -> bytes:
    """Encode H×W×3 uint8 array as a valid PNG file (no Pillow needed)."""
    height, width = rgb.shape[:2]

    def chunk(tag: bytes, data: bytes) -> bytes:
        payload = tag + data
        return (
            struct.pack(">I", len(data))
            + payload
            + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)
        )

    sig  = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    # Filter byte 0 (no filter) prepended to each scanline
    raw  = b"".join(b"\x00" + rgb[r].tobytes() for r in range(height))
    idat = zlib.compress(raw, 6)

    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")


# ---------------------------------------------------------------------------
# Map generation (runs once at startup)
# ---------------------------------------------------------------------------
def _generate_map() -> None:
    global _PNG_BYTES, _IMG_WIDTH, _IMG_HEIGHT
    _parse_yaml()
    if not os.path.exists(_PGM_PATH):
        print(f"[map_server] WARNING — PGM not found: {_PGM_PATH}")
        return
    print("[map_server] Reading PGM …")
    gray, w, h = _read_pgm(_PGM_PATH)
    _IMG_WIDTH, _IMG_HEIGHT = w, h
    print(f"[map_server] Map size: {w} × {h} px  |  resolution: {MAP_RESOLUTION} m/px")
    print("[map_server] Applying colour map …")
    rgb = _pgm_to_rgb(gray)
    print("[map_server] Encoding PNG …")
    _PNG_BYTES = _encode_png(rgb)
    print(f"[map_server] PNG ready ({len(_PNG_BYTES) // 1024} kB).")


# ---------------------------------------------------------------------------
# map_points.json helpers
# ---------------------------------------------------------------------------
def _load_points() -> list:
    if os.path.exists(_POINTS_PATH):
        try:
            with open(_POINTS_PATH, encoding="utf-8") as fh:
                return json.load(fh)
        except Exception:
            pass
    return []


def _save_points(data: list) -> None:
    with open(_POINTS_PATH, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)


# ---------------------------------------------------------------------------
# HTTP request handler
# ---------------------------------------------------------------------------
class _Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass  # suppress default access log

    # CORS headers so fetch() works from file:// as well as localhost
    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        path = self.path.split("?")[0]

        if path in ("/", "/index.html"):
            self._serve_file(_HTML_PATH, "text/html; charset=utf-8")

        elif path == "/lidar_map.png":
            if not _PNG_BYTES:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type",   "image/png")
            self.send_header("Content-Length", str(len(_PNG_BYTES)))
            self._cors()
            self.end_headers()
            self.wfile.write(_PNG_BYTES)

        elif path == "/map_meta.json":
            meta = {
                "width":      _IMG_WIDTH,
                "height":     _IMG_HEIGHT,
                "resolution": MAP_RESOLUTION,
                "origin_x":   MAP_ORIGIN_X,
                "origin_y":   MAP_ORIGIN_Y,
            }
            self._json_response(meta)

        elif path == "/points":
            self._json_response(_load_points())

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self) -> None:
        if self.path.split("?")[0] != "/points":
            self.send_response(404)
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length)
        try:
            data = json.loads(body)
            _save_points(data)
            self._json_response({"ok": True})
            print(f"[map_server] Saved {len(data)} point(s) to map_points.json")
        except Exception as exc:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(str(exc).encode())

    # ------------------------------------------------------------------
    def _json_response(self, obj) -> None:
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _serve_file(self, file_path: str, mime: str) -> None:
        if not os.path.exists(file_path):
            self.send_response(404)
            self.end_headers()
            return
        with open(file_path, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type",   mime)
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    _generate_map()
    server = HTTPServer(("localhost", _PORT), _Handler)
    url = f"http://localhost:{_PORT}/"
    print(f"[map_server] Serving at {url}")
    print("[map_server] Press Ctrl+C to stop.\n")
    threading.Timer(0.8, webbrowser.open, [url]).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[map_server] Stopped.")


if __name__ == "__main__":
    main()
