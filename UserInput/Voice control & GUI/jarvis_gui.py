"""
jarvis_gui.py — Cinematic GUI for the Smart Wheelchair Voice Subsystem
======================================================================

A PyQt6 full-screen GUI styled after the Jarvis / Cortana orb from the
demo reference video: deep navy background, an animated glowing blue
ring with rotating arcs, a prominent digital clock, a status label, a
live transcript, an addressing chip ("SIR" / "MADAME"), and a slide-in
map overlay that displays the LiDAR occupancy map on demand.

Decoupling
----------
This file is intentionally I/O-free. It exposes a `JarvisWindow` whose
slots (`set_state`, `set_partial`, `set_final`, `set_reply`,
`set_address`, `show_map`, `hide_map`) are driven by the voice
subsystem from another thread via Qt signals. That keeps the audio
loop free of GUI dependencies and the GUI free of audio dependencies.
"""

from __future__ import annotations

import ast
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import queue
import threading
from pathlib import Path, PurePosixPath
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from PyQt6.QtCore import (
    QEvent,
    QPoint,
    QPointF,
    QPropertyAnimation,
    QRectF,
    Qt,
    QTimer,
    pyqtProperty,
    pyqtSignal,
)
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontDatabase,
    QImage,
    QKeyEvent,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QRadialGradient,
)
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QGraphicsBlurEffect,
    QGraphicsOpacityEffect,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


WINDOWS_PROJECT_ROOT = Path(__file__).resolve().parents[0]

WSL_PROJECT_ROOT = PurePosixPath("/home/rudrh/Autonomous-Wheelchair-System")

PROJECT_ROOT = WSL_PROJECT_ROOT if os.name == "nt" else Path(__file__).resolve().parents[2]

MAPPING_WS = PROJECT_ROOT / "Positioning" / "MapGeneration" / "Mapping"
LOCALIZATION_WS = PROJECT_ROOT / "Positioning" / "Localization" / "Localization"
NAVIGATION_WS = PROJECT_ROOT / "Navigation" / "Pathfinding" / "Navigation"

ROS_SETUP = PurePosixPath("/opt/ros/jazzy/setup.bash") if os.name == "nt" else Path("/opt/ros/jazzy/setup.bash")
LEFT_LIDAR_PORT = "/dev/left_lidar"
RIGHT_LIDAR_PORT = "/dev/right_lidar"
ARDUINO_PORT = "/dev/arduino_wheelchair"

def q(value):
    return shlex.quote(str(value))

def source(path):
    return f"source {q(path)}"

# ─── ROS2 system launch commands ──────────────────────────────────────────────
_MAPPING_CMD = (
    f"cd {q(MAPPING_WS)} && "
    f"{source(ROS_SETUP)} && "
    f"{source(MAPPING_WS / 'install' / 'setup.bash')} && "
    "ros2 launch map_generator mapping.launch.py "
    f"left_lidar_port:={q(LEFT_LIDAR_PORT)} "
    f"right_lidar_port:={q(RIGHT_LIDAR_PORT)} "
    f"arduino_port:={q(ARDUINO_PORT)}"
)
_LOCALIZATION_CMD = (
    f"cd {q(LOCALIZATION_WS)} && "
    f"{source(ROS_SETUP)} && "
    f"{source(MAPPING_WS / 'install' / 'setup.bash')} && "
    f"{source(LOCALIZATION_WS / 'install' / 'setup.bash')} && "
    "ros2 launch localization localization.launch.py "
    f"left_lidar_port:={q(LEFT_LIDAR_PORT)} "
    f"right_lidar_port:={q(RIGHT_LIDAR_PORT)} "
    f"arduino_port:={q(ARDUINO_PORT)}"
)
_NAVIGATION_CMD = (
    f"cd {q(NAVIGATION_WS)} && "
    f"{source(ROS_SETUP)} && "
    f"{source(MAPPING_WS / 'install' / 'setup.bash')} && "
    f"{source(LOCALIZATION_WS / 'install' / 'setup.bash')} && "
    f"{source(NAVIGATION_WS / 'install' / 'setup.bash')} && "
    "ros2 launch navigation navigation.launch.py "
    f"left_lidar_port:={q(LEFT_LIDAR_PORT)} "
    f"right_lidar_port:={q(RIGHT_LIDAR_PORT)} "
    f"arduino_port:={q(ARDUINO_PORT)}"
)
_NAVIGATION_MAPPING_CMD = (
    f"cd {q(NAVIGATION_WS)} && "
    f"{source(ROS_SETUP)} && "
    f"{source(MAPPING_WS / 'install' / 'setup.bash')} && "
    f"{source(NAVIGATION_WS / 'install' / 'setup.bash')} && "
    "ros2 launch navigation navigation_mapping.launch.py "
    f"left_lidar_port:={q(LEFT_LIDAR_PORT)} "
    f"right_lidar_port:={q(RIGHT_LIDAR_PORT)} "
    f"arduino_port:={q(ARDUINO_PORT)} "
    "use_rviz:=true"
)
_MAPPING_WITH_LOG_CMD = (
    f"cd {q(MAPPING_WS)} && "
    f"{source(ROS_SETUP)} && "
    f"{source(MAPPING_WS / 'install' / 'setup.bash')} && "
    "ros2 launch map_generator mapping_with_log.launch.py "
    f"left_lidar_port:={q(LEFT_LIDAR_PORT)} "
    f"right_lidar_port:={q(RIGHT_LIDAR_PORT)} "
    f"arduino_port:={q(ARDUINO_PORT)} "
    "use_rviz:=true "
    f"log_file:={q(PROJECT_ROOT / 'Other-Files' / 'GeneralData' / 'Logs' / 'mapping_log.txt')}"
)

# ─── Developer tool commands ──────────────────────────────────────────────────
# TOF: actual calibration is firmware-triggered via CAN ID 0x011 on the ESP32.
# From the PC side, the closest thing is reading live sensor data via ToF.py.
_TOF_CALIB_CMD = (
    f"python3 {q(PROJECT_ROOT / 'Integration' / 'Sensors' / 'ToF' / 'ToF.py')}"
)

# IMU: calibration runs automatically in firmware setup() on the Arduino/BNO055.
# No Python entry point exists — this just prints an explanation to the log.
_IMU_CALIB_CMD = (
    "echo 'IMU calibration is handled by the Arduino firmware (BNO055 setup).' "
    "&& echo 'Check getCalStatus() in DFRobot_BNO055.cpp for calibration state.'"
)

# CANBUS: read-only status check — shows whether the can0 interface is up.
_CANBUS_CMD = "sudo ip link show can0"

# Sensor readouts: live 8x8 ToF matrix visualised as ASCII/matplotlib plot.
_SENSOR_CMD = (
    f"python3 {q(PROJECT_ROOT / 'Integration' / 'Sensors' / 'ToF' / 'ToF_ascii_plot.py')}"
)


def _launch_proc(cmd: str, name: str):
    """Start a ROS2 launch command in a new process group and return the process.

    Works on:
      - Linux/WSL directly
      - Windows Python launching ROS2 commands inside WSL through wsl.exe

    stdout and stderr are merged and piped through a daemon reader thread so
    the LogPanel widget can display them in real time without blocking the GUI.
    Output is also echoed to the terminal.
    """
    if os.name == "nt":
        # Windows Python: run the Linux ROS2 command inside WSL.
        # os.setsid does not exist on Windows, so use CREATE_NEW_PROCESS_GROUP.
        proc = subprocess.Popen(
            ["wsl.exe", "bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
    else:
        # Linux / WSL Python: normal bash launch with a separate process group.
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1,
        )

    def _reader() -> None:
        try:
            for line in proc.stdout:
                stripped = line.rstrip()
                LogPanel.feed(f"[{name}] {stripped}")
                print(f"[{name}] {stripped}")
        except Exception:
            pass

    threading.Thread(target=_reader, daemon=True, name=f"log_{name}").start()
    print(f"[Launcher] {name} started (pid={proc.pid})")
    return proc


def _kill_proc(proc, name: str) -> None:
    """Stop a launched process.

    Linux/WSL uses SIGINT -> SIGTERM -> SIGKILL on the process group.
    Windows uses terminate/kill because os.killpg and os.setsid are unavailable.
    """
    if proc is None or proc.poll() is not None:
        return

    if os.name == "nt":
        try:
            proc.terminate()
            print(f"[Launcher] {name} stopping on Windows...")
        except Exception as exc:
            print(f"[Launcher] Could not stop {name}: {exc}")
            return

        def _escalate_windows() -> None:
            import time as _time
            _time.sleep(3)
            if proc.poll() is not None:
                return
            try:
                proc.kill()
                print(f"[Launcher] {name} did not exit after terminate. Killing...")
            except Exception as exc:
                print(f"[Launcher] Could not kill {name}: {exc}")

        threading.Thread(
            target=_escalate_windows,
            daemon=True,
            name=f"kill_{name}",
        ).start()
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        print(f"[Launcher] {name} stopping (SIGINT)…")
    except ProcessLookupError:
        return
    except Exception as exc:
        print(f"[Launcher] Could not stop {name}: {exc}")
        return

    def _escalate() -> None:
        import time as _time
        _time.sleep(3)
        if proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            print(f"[Launcher] {name} did not exit after SIGINT. Sending SIGTERM…")
        except ProcessLookupError:
            return
        _time.sleep(3)
        if proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            print(f"[Launcher] {name} did not exit after SIGTERM. Sending SIGKILL…")
        except ProcessLookupError:
            return

    threading.Thread(target=_escalate, daemon=True, name=f"kill_{name}").start()


# =============================================================================
# Theme
# =============================================================================
class Theme:
    BG_TOP = QColor(4, 10, 24)
    BG_BOTTOM = QColor(0, 0, 0)
    ACCENT = QColor(80, 170, 255)
    ACCENT_BRIGHT = QColor(150, 210, 255)
    ACCENT_DIM = QColor(40, 90, 150)
    TEXT_PRIMARY = QColor(220, 235, 255)
    TEXT_DIM = QColor(120, 150, 190)
    PANEL_BG = QColor(10, 20, 40, 200)
    CHIP_BG = QColor(20, 50, 90, 220)


class State(str, Enum):
    """Top-level lifecycle of the voice subsystem, mirrored on screen."""
    IDLE = "STANDING BY"
    LISTENING = "LISTENING"
    AWAKE = "AWAITING COMMAND"
    THINKING = "THINKING"
    SPEAKING = "SPEAKING"


# ─── Map coordinate parameters ────────────────────────────────────────────────
# Fallbacks until Lidar Map/my_map.yaml is loaded (values match that file).
MAP_ORIGIN_X   = -47.471
MAP_ORIGIN_Y   = -30.341
MAP_RESOLUTION = 0.050

_LIDAR_MAP_DIR  = "Lidar Map"
_LIDAR_MAP_YAML = "my_map.yaml"


def default_map_yaml_path() -> str:
    """Absolute path to the ROS map_server YAML in the project Lidar Map folder."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, _LIDAR_MAP_DIR, _LIDAR_MAP_YAML)


def _parse_ros_map_yaml(yaml_path: str) -> dict:
    """Parse map_server metadata from my_map.yaml (no PyYAML dependency)."""
    meta: dict = {}
    with open(yaml_path, encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(":")
            key = key.strip()
            value = value.strip()
            if key == "image":
                meta["image"] = value
            elif key == "resolution":
                meta["resolution"] = float(value)
            elif key == "origin":
                origin = ast.literal_eval(value)
                meta["origin_x"] = float(origin[0])
                meta["origin_y"] = float(origin[1])
            elif key == "negate":
                meta["negate"] = int(value)
            elif key == "occupied_thresh":
                meta["occupied_thresh"] = float(value)
            elif key == "free_thresh":
                meta["free_thresh"] = float(value)
    return meta


def _pgm_to_rgb(
    gray,
    *,
    negate: int,
    free_thresh: float,
    occupied_thresh: float,
) -> bytes:
    """Convert a ROS map_server PGM (grayscale) to packed RGB bytes.

    Uses the same Jarvis palette as ros2_bridge._occupancy_to_rgb.
    """
    import numpy as np

    occ_prob = gray.astype(np.float32) / 255.0
    if negate == 0:
        occ_prob = 1.0 - occ_prob

    height, width = gray.shape
    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    occupied = occ_prob > occupied_thresh
    free     = occ_prob < free_thresh
    unknown  = ~occupied & ~free

    rgb[unknown]  = [10, 20, 40]
    rgb[free]     = [28, 48, 80]
    if occupied.any():
        occ_vals = occ_prob[occupied]
        rgb[occupied, 0] = np.clip(40  + occ_vals * 110, 0, 255).astype(np.uint8)
        rgb[occupied, 1] = np.clip(100 + occ_vals * 110, 0, 255).astype(np.uint8)
        rgb[occupied, 2] = np.clip(160 + occ_vals * 95,  0, 255).astype(np.uint8)

    return rgb.tobytes()


def load_lidar_map(yaml_path: str) -> tuple[QPixmap, float, float, float]:
    """Load Lidar Map/my_map.yaml + its PGM into a QPixmap and transform params."""
    if not os.path.exists(yaml_path):
        return QPixmap(), MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION

    meta = _parse_ros_map_yaml(yaml_path)
    pgm_name = meta.get("image", "my_map.pgm")
    pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_name)
    if not os.path.exists(pgm_path):
        return QPixmap(), MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION

    img = QImage(pgm_path)
    if img.isNull():
        return QPixmap(), MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION

    import numpy as np

    img = img.convertToFormat(QImage.Format.Format_Grayscale8)
    width, height = img.width(), img.height()
    bpl = img.bytesPerLine()
    ptr = img.constBits()
    ptr.setsize(bpl * height)
    gray = np.frombuffer(ptr, dtype=np.uint8).reshape(height, bpl)[:, :width].copy()

    rgb_bytes = _pgm_to_rgb(
        gray,
        negate=meta.get("negate", 0),
        free_thresh=meta.get("free_thresh", 0.196),
        occupied_thresh=meta.get("occupied_thresh", 0.65),
    )
    qimg = QImage(rgb_bytes, width, height, width * 3, QImage.Format.Format_RGB888)
    origin_x   = meta.get("origin_x", MAP_ORIGIN_X)
    origin_y   = meta.get("origin_y", MAP_ORIGIN_Y)
    resolution = meta.get("resolution", MAP_RESOLUTION)
    return QPixmap.fromImage(qimg), origin_x, origin_y, resolution


# =============================================================================
# Animated orb widget
# =============================================================================
class JarvisOrb(QWidget):
    """A pulsating glowing ring with rotating arcs and an inner core.

    Three behaviours we want from the orb:
      • Steady idle pulse (slow breathing).
      • Faster, brighter motion when LISTENING/THINKING.
      • Wider radius and stronger amplitude when SPEAKING.

    Implemented as a single custom-painted widget so we avoid layered
    QWidgets that flicker.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._phase = 0.0
        self._arc_phase = 0.0
        self._arc_phase2 = 0.0     # independent phase for the outer counter-rotating arcs
        self._pulse_phase = 0.0    # drives the whole-orb size pulse
        self._pulse_scale = 1.0    # current scale multiplier applied to radius
        self._intensity = 0.6      # 0..1, animated toward _target_intensity
        self._target_intensity = 0.6
        self._state: State = State.IDLE
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)  # ~30 fps — plenty for an orb

    def set_state(self, state: State) -> None:
        self._state = state
        self._target_intensity = {
            State.IDLE: 0.55,
            State.LISTENING: 0.85,
            State.AWAKE: 1.0,
            State.THINKING: 0.95,
            State.SPEAKING: 1.0,
        }.get(state, 0.6)

    def _tick(self) -> None:
        # Breathing speed depends on state.
        speed = {
            State.IDLE: 0.020,
            State.LISTENING: 0.045,
            State.AWAKE: 0.060,
            State.THINKING: 0.070,
            State.SPEAKING: 0.080,
        }.get(self._state, 0.03)
        self._phase = (self._phase + speed) % (2 * math.pi)
        self._arc_phase = (self._arc_phase + speed * 1.7) % (2 * math.pi)
        # Independent phase for the outer counter-rotating arcs — wraps at 2π so
        # degrees stay in 0–360 and there is no visible jump between cycles.
        self._arc_phase2 = (self._arc_phase2 + speed * 1.7 * 0.8) % (2 * math.pi)
        # Whole-orb size pulse: active in THINKING (faster) and SPEAKING (slower).
        if self._state in (State.THINKING, State.SPEAKING):
            pulse_spd = 0.13 if self._state == State.THINKING else 0.09
            self._pulse_phase = (self._pulse_phase + pulse_spd) % (2 * math.pi)
            pulse_target = 1.0 + 0.12 * math.sin(self._pulse_phase)
        else:
            self._pulse_phase = 0.0
            pulse_target = 1.0
        # Smooth approach so the size change never snaps.
        self._pulse_scale += (pulse_target - self._pulse_scale) * 0.2
        # Smooth approach to target intensity.
        self._intensity += (self._target_intensity - self._intensity) * 0.08
        self.update()

    def paintEvent(self, _ev) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        side = min(w, h)
        base_radius = side * 0.30

        breath = 0.5 + 0.5 * math.sin(self._phase)  # 0..1
        # _pulse_scale drives the whole-orb size animation; the small breath factor
        # stays on top of it so the inner breathing is not lost during a pulse.
        radius = base_radius * self._pulse_scale * (1.0 + 0.04 * breath * self._intensity)

        # 1. Outer halo — soft radial gradient.
        halo = QRadialGradient(QPointF(cx, cy), radius * 2.4)
        halo_color = QColor(Theme.ACCENT)
        halo_color.setAlpha(int(60 * self._intensity))
        halo.setColorAt(0.0, halo_color)
        halo.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(halo))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), radius * 2.4, radius * 2.4)

        # 2. Inner glow disc.
        disc = QRadialGradient(QPointF(cx, cy), radius)
        disc_color_in = QColor(Theme.ACCENT_BRIGHT)
        disc_color_in.setAlpha(int(110 * self._intensity))
        disc_color_out = QColor(Theme.ACCENT)
        disc_color_out.setAlpha(0)
        disc.setColorAt(0.0, disc_color_in)
        disc.setColorAt(1.0, disc_color_out)
        painter.setBrush(QBrush(disc))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # 3. Main ring.
        ring_pen = QPen(Theme.ACCENT_BRIGHT)
        ring_pen.setWidthF(max(2.5, side * 0.006))
        ring_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(ring_pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        rect = QRectF(cx - radius, cy - radius, 2 * radius, 2 * radius)
        painter.drawEllipse(rect)

        # 4. Rotating broken arcs — sense of motion.
        arc_pen = QPen(Theme.ACCENT)
        arc_pen.setWidthF(max(2.0, side * 0.005))
        arc_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(arc_pen)
        arc_rect = QRectF(
            cx - radius * 1.18, cy - radius * 1.18,
            2 * radius * 1.18, 2 * radius * 1.18,
        )
        # Two arcs rotating in opposite directions.
        deg = math.degrees(self._arc_phase)
        painter.drawArc(arc_rect, int(deg * 16), int(70 * 16))
        painter.drawArc(arc_rect, int((180 + deg) * 16), int(50 * 16))

        arc_rect2 = QRectF(
            cx - radius * 1.35, cy - radius * 1.35,
            2 * radius * 1.35, 2 * radius * 1.35,
        )
        # Use _arc_phase2 (wraps independently at 2π) so degrees stay in 0–360
        # and the negative direction gives the counter-rotation without a jump.
        deg2 = -math.degrees(self._arc_phase2) + 30
        thin_pen = QPen(Theme.ACCENT_DIM)
        thin_pen.setWidthF(max(1.5, side * 0.003))
        painter.setPen(thin_pen)
        painter.drawArc(arc_rect2, int(deg2 * 16), int(40 * 16))
        painter.drawArc(arc_rect2, int((140 + deg2) * 16), int(30 * 16))

        # 5. JARVIS wordmark in the centre.
        font = QFont("Segoe UI", int(side * 0.045), QFont.Weight.DemiBold)
        font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 6.0)
        painter.setFont(font)
        text_color = QColor(Theme.ACCENT_BRIGHT)
        text_color.setAlpha(int(220 * (0.6 + 0.4 * breath)))
        painter.setPen(text_color)
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "JARVIS")

        painter.end()


# =============================================================================
# Pose dot — "you are here" marker drawn over the map
# =============================================================================
class PoseDot(QWidget):
    """Animated green pose marker showing the wheelchair's estimated position.

    Styled to match the blue waypoint markers (staggered pulsing rings, a soft
    halo, a core dot and a heading arrow) but in green so the live AMCL pose is
    instantly distinguishable from the placed destinations.  The heading arrow
    is drawn from the AMCL yaw using the same convention as MarkerLayer.

    Positioned by MapOverlay.set_pose() whenever a new /amcl_pose message
    arrives.  Lives as a child widget of MapOverlay so it is automatically
    shown/hidden with the map.  The widget is deliberately oversized (so the
    arrow and outer rings are never clipped) and fully mouse-transparent so it
    never steals clicks from the markers underneath.
    """

    # Box is large enough to contain the longest ring + arrow without clipping.
    _BOX = 80

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setFixedSize(self._BOX, self._BOX)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._phase = 0.0
        self._yaw: Optional[float] = None   # AMCL heading in radians, None = unknown
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)   # 25 fps — matches the blue marker pulse
        self.hide()

    def set_heading(self, yaw: Optional[float]) -> None:
        """Update the heading arrow direction (radians, ROS convention)."""
        self._yaw = yaw
        self.update()

    def _tick(self) -> None:
        self._phase = (self._phase + 0.08) % (2 * math.pi)
        self.update()

    def paintEvent(self, _ev) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        t  = self._phase
        cx = self.width() / 2.0
        cy = self.height() / 2.0

        # Green palette mirroring the blue/red scheme used in MarkerLayer.
        ring_col = QColor(80,  230, 130)
        halo_col = QColor(80,  230, 130, 110)
        dot_col  = QColor(170, 255, 200)

        # Three staggered concentric rings — each ring at a different phase.
        for j in range(3):
            phase    = (t + j * 0.75) % (2 * math.pi)
            progress = (math.sin(phase) + 1) / 2
            r        = 7 + progress * 24
            c = QColor(ring_col)
            c.setAlpha(int((1 - progress) * 160))
            painter.setPen(QPen(c, 1.5))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), r, r)

        # Soft radial glow halo.
        grd = QRadialGradient(QPointF(cx, cy), 16)
        grd.setColorAt(0.0, halo_col)
        grd.setColorAt(1.0, QColor(halo_col.red(), halo_col.green(), halo_col.blue(), 0))
        painter.setBrush(QBrush(grd))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), 16, 16)

        # Heading arrow from the AMCL yaw.
        # ros_yaw: right=0, CCW positive (ROS convention).
        # Display angle: right=0, CW positive (Y-down screen) → negate.
        if self._yaw is not None:
            arrow_len  = 26 + math.sin(t * 2) * 3     # gentle length pulse
            disp_angle = -self._yaw
            tip_x = cx + math.cos(disp_angle) * arrow_len
            tip_y = cy + math.sin(disp_angle) * arrow_len
            head_len = 8
            head_ang = 0.45
            arrow_pen = QPen(ring_col, 2)
            arrow_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(arrow_pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawLine(QPointF(cx, cy), QPointF(tip_x, tip_y))
            for sign in (-1, 1):
                hx = tip_x - head_len * math.cos(disp_angle + sign * head_ang)
                hy = tip_y - head_len * math.sin(disp_angle + sign * head_ang)
                painter.drawLine(QPointF(tip_x, tip_y), QPointF(hx, hy))

        # Core dot.
        painter.setBrush(QBrush(dot_col))
        painter.setPen(QPen(QColor(255, 255, 255, 230), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4.5, 4.5)
        painter.end()


# =============================================================================
# Marker layer — pulsing blue waypoint dots painted over the map
# =============================================================================
class MarkerLayer(QWidget):
    """Transparent child widget of MapOverlay that animates named waypoint markers.

    Purely visual — mouse events pass through so MapOverlay can handle clicks.
    """

    def __init__(self, overlay: "MapOverlay"):
        super().__init__(overlay)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._overlay = overlay
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)   # 25 fps — smooth pulse without burning CPU

    def _tick(self) -> None:
        self._phase = (self._phase + 0.08) % (2 * math.pi)
        self.update()

    def paintEvent(self, _ev) -> None:
        ov = self._overlay

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        t = self._phase

        # During placement mode, overlay the navigable-area highlight mask so
        # the operator knows exactly where markers can be placed.
        if ov._placement_mode and ov._nav_mask_scaled is not None and ov._scaled_w > 0:
            label_origin = ov._map_label.mapTo(ov, QPoint(0, 0))
            x_pad = (ov._map_label.width()  - ov._scaled_w) / 2
            y_pad = (ov._map_label.height() - ov._scaled_h) / 2
            painter.drawPixmap(
                int(label_origin.x() + x_pad),
                int(label_origin.y() + y_pad),
                ov._nav_mask_scaled,
            )

        if not ov._map_points:
            painter.end()
            return

        for pt in ov._map_points:
            pos = ov._ros_to_display(pt["ros_x"], pt["ros_y"])
            if pos is None:
                continue
            cx, cy = pos

            is_sel = (pt["name"] == ov._selected_point)

            # When a destination is locked in, fade the other markers back so the
            # active red target stands out. Full opacity when nothing is selected.
            if ov._selected_point is not None and not is_sel:
                painter.setOpacity(0.35)
            else:
                painter.setOpacity(1.0)

            # Colour scheme — blue for normal, red for selected destination.
            if is_sel:
                ring_col  = QColor(255, 70,  70)
                halo_col  = QColor(255, 70,  70, 110)
                dot_col   = QColor(255, 140, 140)
                label_col = QColor(255, 180, 180, 240)
            else:
                ring_col  = QColor(80,  170, 255)
                halo_col  = QColor(80,  170, 255, 105)
                dot_col   = QColor(205, 232, 255)
                label_col = QColor(160, 215, 255, 240)

            # Three staggered concentric rings — each ring at a different phase.
            for j in range(3):
                phase    = (t + j * 0.75) % (2 * math.pi)
                progress = (math.sin(phase) + 1) / 2
                r        = 7 + progress * 24
                alpha    = int((1 - progress) * (185 if is_sel else 145))
                c = QColor(ring_col)
                c.setAlpha(alpha)
                painter.setPen(QPen(c, 1.5))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(QPointF(cx, cy), r, r)

            # Soft radial glow halo
            grd = QRadialGradient(QPointF(cx, cy), 16)
            grd.setColorAt(0.0, halo_col)
            grd.setColorAt(1.0, QColor(halo_col.red(), halo_col.green(), halo_col.blue(), 0))
            painter.setBrush(QBrush(grd))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QPointF(cx, cy), 16, 16)

            # Core dot
            painter.setBrush(QBrush(dot_col))
            painter.setPen(QPen(QColor(255, 255, 255, 230), 1.5))
            painter.drawEllipse(QPointF(cx, cy), 4.5, 4.5)

            # Heading arrow from ros_yaw.
            # ros_yaw: right=0, CCW positive (ROS convention).
            # Display angle: right=0, CW positive (Y-down screen) → negate.
            ros_yaw = pt.get("ros_yaw", None)
            if ros_yaw is not None:
                arrow_len  = 26 + math.sin(t * 2) * 3     # gentle length pulse
                disp_angle = -ros_yaw
                tip_x = cx + math.cos(disp_angle) * arrow_len
                tip_y = cy + math.sin(disp_angle) * arrow_len
                head_len = 8
                head_ang = 0.45

                arrow_pen = QPen(ring_col, 2)
                arrow_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
                painter.setPen(arrow_pen)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawLine(QPointF(cx, cy), QPointF(tip_x, tip_y))

                # Arrowhead — two short lines from the tip
                for sign in (-1, 1):
                    hx = tip_x - head_len * math.cos(disp_angle + sign * head_ang)
                    hy = tip_y - head_len * math.sin(disp_angle + sign * head_ang)
                    painter.drawLine(QPointF(tip_x, tip_y), QPointF(hx, hy))

            # Name label with drop-shadow
            font = QFont("Segoe UI", 10, QFont.Weight.DemiBold)
            font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 2.0)
            painter.setFont(font)
            fm  = painter.fontMetrics()
            txt = pt["name"].upper()
            tw  = fm.horizontalAdvance(txt)
            lx  = cx - tw / 2
            ly  = cy - 18
            painter.setPen(QColor(0, 5, 20, 190))
            painter.drawText(QPointF(lx + 1, ly + 1), txt)
            painter.setPen(label_col)
            painter.drawText(QPointF(lx, ly), txt)

        painter.end()


# =============================================================================
# Path layer — planned path from /plan drawn over the map
# =============================================================================
class PathLayer(QWidget):
    """Transparent child of MapOverlay that draws the Nav2 planned path.

    Updated via set_path() whenever the ROS2 bridge fires on_path with a new
    list of (ros_x, ros_y) tuples.  Animated at 25 fps so the begin/end dots
    pulse.  Stacked below MarkerLayer so waypoint markers stay on top.
    """

    def __init__(self, overlay: "MapOverlay"):
        super().__init__(overlay)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._overlay = overlay
        self._path_points: list = []    # list of (ros_x, ros_y) tuples
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)           # 25 fps — smooth pulse without burning CPU

    def _tick(self) -> None:
        self._phase = (self._phase + 0.09) % (2 * math.pi)
        if self._path_points:
            self.update()

    def set_path(self, points: list) -> None:
        """Replace the displayed path.  Pass an empty list to clear."""
        self._path_points = list(points)
        self.update()

    def paintEvent(self, _ev) -> None:
        if not self._path_points:
            return
        ov = self._overlay
        coords = []
        for rx, ry in self._path_points:
            pos = ov._ros_to_display(rx, ry)
            if pos is not None:
                coords.append(QPointF(*pos))
        if len(coords) < 2:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        t     = self._phase
        pulse = (math.sin(t) + 1) / 2  # 0 → 1, smooth

        # Build the path once — shared by all draw passes.
        qt_path = QPainterPath()
        qt_path.moveTo(coords[0])
        for pt in coords[1:]:
            qt_path.lineTo(pt)

        # Outer glow — wide, very faint cyan halo.
        glow_outer = QPen(QColor(0, 220, 255, 30))
        glow_outer.setWidthF(20.0)
        glow_outer.setCapStyle(Qt.PenCapStyle.RoundCap)
        glow_outer.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.setPen(glow_outer)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(qt_path)

        # Inner glow — tighter, brighter cyan ring.
        glow_inner = QPen(QColor(0, 220, 255, 80))
        glow_inner.setWidthF(8.0)
        glow_inner.setCapStyle(Qt.PenCapStyle.RoundCap)
        glow_inner.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.setPen(glow_inner)
        painter.drawPath(qt_path)

        # Main line — bright sci-fi cyan.
        line_pen = QPen(QColor(0, 230, 255, 235))
        line_pen.setWidthF(3.5)
        line_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        line_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.setPen(line_pen)
        painter.drawPath(qt_path)

        # ── Begin dot ────────────────────────────────────────────────────────
        cx0, cy0 = coords[0].x(), coords[0].y()
        r_begin  = 9.0
        b_col    = QColor(80, 255, 140)     # bright green

        # Expanding glow ring
        glow_r0     = r_begin + 6 + pulse * 12
        glow_alpha0 = int((1 - pulse) * 130)
        grd0 = QRadialGradient(QPointF(cx0, cy0), glow_r0)
        grd0.setColorAt(0.0, QColor(80, 255, 140, glow_alpha0))
        grd0.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(grd0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx0, cy0), glow_r0, glow_r0)

        ring_r0 = r_begin + 3 + pulse * 13
        c0 = QColor(80, 255, 140, int((1 - pulse * 0.7) * 190))
        painter.setPen(QPen(c0, 1.8))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx0, cy0), ring_r0, ring_r0)

        # Solid core dot
        painter.setBrush(QBrush(b_col))
        painter.setPen(QPen(QColor(255, 255, 255, 230), 2.0))
        painter.drawEllipse(QPointF(cx0, cy0), r_begin, r_begin)

        # ── End dot ──────────────────────────────────────────────────────────
        cx1, cy1 = coords[-1].x(), coords[-1].y()
        r_end    = 9.0
        is_sel   = bool(getattr(ov, "_selected_point", None))

        if is_sel:
            e_col    = QColor(255, 55, 55)
            e_glow   = QColor(255, 55, 55)
        else:
            e_col    = QColor(255, 215, 60)     # gold when idle
            e_glow   = QColor(255, 200, 40)

        # Expanding glow
        glow_r1     = r_end + 7 + pulse * 14
        glow_alpha1 = int((1 - pulse) * (170 if is_sel else 110))
        grd1 = QRadialGradient(QPointF(cx1, cy1), glow_r1)
        grd1.setColorAt(0.0, QColor(e_glow.red(), e_glow.green(), e_glow.blue(), glow_alpha1))
        grd1.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(grd1))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx1, cy1), glow_r1, glow_r1)

        # One ring when idle, two staggered when selected — more urgency.
        ring_count = 2 if is_sel else 1
        for j in range(ring_count):
            phase_j  = (t + j * 1.3) % (2 * math.pi)
            prog_j   = (math.sin(phase_j) + 1) / 2
            ring_r1  = r_end + 3 + prog_j * 15
            ring_a1  = int((1 - prog_j) * (210 if is_sel else 170))
            ce = QColor(e_glow)
            ce.setAlpha(ring_a1)
            painter.setPen(QPen(ce, 1.8))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QPointF(cx1, cy1), ring_r1, ring_r1)

        # Solid core dot
        painter.setBrush(QBrush(e_col))
        painter.setPen(QPen(QColor(255, 255, 255, 230), 2.0))
        painter.drawEllipse(QPointF(cx1, cy1), r_end, r_end)

        # Crosshair reticle when selected — makes the target read as "locked".
        if is_sel:
            gap  = r_end + 4
            rlen = 11
            pen_x = QPen(QColor(255, 80, 80, 190), 1.8)
            pen_x.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(pen_x)
            painter.drawLine(QPointF(cx1 + gap, cy1), QPointF(cx1 + gap + rlen, cy1))
            painter.drawLine(QPointF(cx1 - gap, cy1), QPointF(cx1 - gap - rlen, cy1))
            painter.drawLine(QPointF(cx1, cy1 + gap), QPointF(cx1, cy1 + gap + rlen))
            painter.drawLine(QPointF(cx1, cy1 - gap), QPointF(cx1, cy1 - gap - rlen))

        painter.end()


# =============================================================================
# Map overlay
# =============================================================================
class MapOverlay(QWidget):
    """Full-screen translucent panel that fades in to show the LiDAR occupancy map."""

    closed           = pyqtSignal()
    location_placed  = pyqtSignal(str, float, float)         # (name, ros_x, ros_y)
    nav_requested    = pyqtSignal(str, float, float, float)  # (name, ros_x, ros_y, ros_yaw)

    def __init__(self, map_yaml_path: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            "background-color: rgba(2, 8, 20, 235);"
        )
        self._map_yaml_path = map_yaml_path
        pixmap, ox, oy, res = load_lidar_map(map_yaml_path)
        self._pixmap = pixmap
        # Navigable-area assets — built once from the source pixmap.
        self._map_image: Optional[QImage] = None
        self._nav_mask: Optional[QPixmap] = None
        self._nav_mask_scaled: Optional[QPixmap] = None
        self._opacity_effect = QGraphicsOpacityEffect(self)
        self._opacity_effect.setOpacity(0.0)
        self.setGraphicsEffect(self._opacity_effect)
        self._anim = QPropertyAnimation(self._opacity_effect, b"opacity", self)
        self._anim.setDuration(380)

        # Map coordinate transform — updated automatically when a live /map arrives.
        # Initialized from Lidar Map/my_map.yaml until then.
        self._map_origin_x   = ox
        self._map_origin_y   = oy
        self._map_resolution = res

        # Top bar: title + close hint.
        self._title_label = QLabel("LIDAR MAP")
        title = self._title_label
        title.setStyleSheet(
            f"color: {Theme.ACCENT_BRIGHT.name()}; "
            "font-family: 'Segoe UI'; font-size: 22px; font-weight: 600; "
            "letter-spacing: 4px;"
        )
        hint = QLabel("Click a marker to navigate  ·  ESC to close")
        hint.setStyleSheet(
            f"color: {Theme.TEXT_DIM.name()}; "
            "font-family: 'Segoe UI'; font-size: 13px; letter-spacing: 2px;"
        )
        hint.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._hint_label = hint

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(40, 20, 40, 10)
        top_bar.addWidget(title)
        top_bar.addStretch()
        top_bar.addWidget(hint)

        self._map_label = QLabel()
        self._map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._map_label.setStyleSheet("background: transparent;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 30)
        layout.setSpacing(0)
        layout.addLayout(top_bar)
        layout.addWidget(self._map_label, 1)

        # Pose dot — child widget so it is hidden with the overlay automatically.
        self._pose_dot = PoseDot(self)
        self._scaled_w = 0
        self._scaled_h = 0
        self._last_pose: Optional[tuple] = None   # (ros_x, ros_y, ros_yaw) of last known pose

        # Waypoint state — placement mode, stored named map points, selection.
        self._placement_mode: bool = False
        self._pending_pos: Optional[tuple] = None   # (ros_x, ros_y) awaiting name input
        self._map_points: list[dict] = []
        self._selected_point: Optional[str] = None  # name of currently active destination
        self._load_points()
        self._build_nav_assets()

        # Path drawing layer — below the marker layer so waypoints stay on top.
        self._path_layer = PathLayer(self)
        self._path_layer.setGeometry(self.rect())
        self._path_layer.show()

        # Marker drawing layer — transparent, mouse-passthrough, always on top.
        self._marker_layer = MarkerLayer(self)
        self._marker_layer.setGeometry(self.rect())
        self._marker_layer.raise_()
        self._marker_layer.show()

        # Intercept clicks on the map label so we get map-space coordinates.
        self._map_label.installEventFilter(self)

        self.hide()

    def has_map(self) -> bool:
        return not self._pixmap.isNull()

    def fade_in(self) -> None:
        self.show()
        self.raise_()
        self._update_pixmap()
        self._anim.stop()
        self._anim.setStartValue(self._opacity_effect.opacity())
        self._anim.setEndValue(1.0)
        self._anim.start()
        # Restore the pose dot if we already had a position before the map was hidden.
        if self._last_pose is not None:
            self.set_pose(*self._last_pose)

    def fade_out(self) -> None:
        self._anim.stop()
        self._anim.setStartValue(self._opacity_effect.opacity())
        self._anim.setEndValue(0.0)
        try:
            self._anim.finished.disconnect()
        except TypeError:
            pass
        self._anim.finished.connect(self._after_hide)
        self._anim.start()

    def _after_hide(self) -> None:
        if self._opacity_effect.opacity() < 0.01:
            self.hide()
            self.closed.emit()

    def set_pose(self, ros_x: float, ros_y: float, ros_yaw: Optional[float] = None) -> None:
        """Position the pose dot from a ROS2 map-frame coordinate (metres).

        ros_yaw (radians, ROS convention) drives the heading arrow.  It is
        optional so older callers still work; pass it whenever an /amcl_pose
        message provides an orientation.

        The conversion chain is:
          ROS map metres → source-image pixel → scaled-image pixel
          → label-local pixel (accounting for centre-alignment padding)
          → overlay pixel (accounting for label's position inside the overlay)

        Transform parameters are updated automatically when set_map() is called
        with a live OccupancyGrid.  Before that they fall back to the module
        constants (MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION).
        """
        self._last_pose = (ros_x, ros_y, ros_yaw)
        self._pose_dot.set_heading(ros_yaw)
        if self._pixmap.isNull() or self._scaled_w == 0 or self._scaled_h == 0:
            return

        src_w = self._pixmap.width()
        src_h = self._pixmap.height()

        # 1. ROS map metres → pixel in the original source image.
        #    ROS Y axis points up; image row 0 is at the top, so we flip Y.
        src_px = (ros_x - self._map_origin_x)   / self._map_resolution
        src_py = src_h - (ros_y - self._map_origin_y) / self._map_resolution

        # 2. Scale to the displayed (fitted) pixmap size.
        scale  = self._scaled_w / src_w
        disp_x = src_px * scale
        disp_y = src_py * scale

        # 3. The label centres the pixmap; compute the padding offsets.
        x_pad = (self._map_label.width()  - self._scaled_w) / 2
        y_pad = (self._map_label.height() - self._scaled_h) / 2

        # 4. Convert label-local coordinates to overlay coordinates.
        label_origin = self._map_label.mapTo(self, QPoint(0, 0))
        dot_cx = label_origin.x() + x_pad + disp_x
        dot_cy = label_origin.y() + y_pad + disp_y

        # Place the dot so its centre sits on the pose point.
        half = self._pose_dot.width() / 2
        self._pose_dot.move(int(dot_cx - half), int(dot_cy - half))
        self._pose_dot.show()
        self._pose_dot.raise_()

    def set_map(self, payload: dict) -> None:
        """Replace the static PGM map with a live OccupancyGrid rendered as a pixmap.

        Parameters
        ----------
        payload : dict
            Produced by ros2_bridge._on_map_msg / _on_map_ws.
            Keys: width (int), height (int), data (bytes, packed RGB),
                  origin_x (float), origin_y (float), resolution (float).

        Must be called on the GUI thread (connect via request_set_map signal).
        """
        width      = payload["width"]
        height     = payload["height"]
        rgb_bytes  = payload["data"]
        origin_x   = payload["origin_x"]
        origin_y   = payload["origin_y"]
        resolution = payload["resolution"]

        # Build QPixmap from the pre-computed RGB bytes.
        # QImage.Format_RGB888 = 3 bytes per pixel, no alpha, no padding.
        img = QImage(rgb_bytes, width, height, width * 3, QImage.Format.Format_RGB888)
        self._pixmap = QPixmap.fromImage(img)

        # Store the correct coordinate transform for the pose dot.
        self._map_origin_x   = origin_x
        self._map_origin_y   = origin_y
        self._map_resolution = resolution

        # Update the title to reflect that this is the live sensor map.
        self._title_label.setText("LIVE LIDAR MAP")

        # Clear the "map not found" error text if it was showing.
        self._map_label.setStyleSheet("background: transparent;")

        # Rebuild the navigable-area highlight mask for the new map data.
        self._build_nav_assets()

        # Refresh the displayed image.
        self._update_pixmap()

        # Re-apply the pose dot with the new transform.
        if self._last_pose is not None:
            self.set_pose(*self._last_pose)

        # Keep the marker layer raised after the pixmap swap so markers stay visible.
        self._marker_layer.raise_()
        self._marker_layer.update()

    def set_path(self, points: list) -> None:
        """Draw the Nav2 planned path on the map.  Pass [] to clear."""
        self._path_layer.set_path(points)

    # ------------------------------------------------------------------
    # Waypoint placement
    # ------------------------------------------------------------------
    def enter_placement_mode(self) -> None:
        """Switch cursor to crosshair and arm the next click as a new marker."""
        self._placement_mode = True
        self.setCursor(Qt.CursorShape.CrossCursor)
        self._hint_label.setText(
            "Click anywhere on the map to place a waypoint marker  ·  ESC to cancel"
        )

    def exit_placement_mode(self) -> None:
        """Return to normal navigation-click mode."""
        self._placement_mode = False
        self._pending_pos = None
        self.setCursor(Qt.CursorShape.ArrowCursor)
        self._hint_label.setText("Click a marker to navigate  ·  ESC to close")

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------
    def _ros_to_display(self, ros_x: float, ros_y: float) -> Optional[tuple]:
        """ROS map-frame metres → overlay widget pixel coordinates.

        This is the forward transform used by the marker layer and the pose
        dot.  Returns None when the map pixmap has not loaded yet or the
        displayed size is not known.
        """
        if self._pixmap.isNull() or self._scaled_w == 0 or self._scaled_h == 0:
            return None
        src_px = (ros_x - self._map_origin_x) / self._map_resolution
        src_py = self._pixmap.height() - (ros_y - self._map_origin_y) / self._map_resolution
        scale  = self._scaled_w / self._pixmap.width()
        disp_x = src_px * scale
        disp_y = src_py * scale
        label_origin = self._map_label.mapTo(self, QPoint(0, 0))
        x_pad = (self._map_label.width()  - self._scaled_w) / 2
        y_pad = (self._map_label.height() - self._scaled_h) / 2
        return label_origin.x() + x_pad + disp_x, label_origin.y() + y_pad + disp_y

    def _build_nav_assets(self) -> None:
        """Pre-compute the navigable-area cache and highlight mask from the source pixmap.

        Navigable pixels are rgb(28, 48, 80) — the free-space colour produced by
        _pgm_to_rgb.  A semi-transparent cyan QPixmap is built so MarkerLayer can
        paint it as a placement guide without per-frame pixel iteration.
        """
        if self._pixmap.isNull():
            self._map_image = None
            self._nav_mask = None
            self._nav_mask_scaled = None
            return

        import numpy as np

        img = self._pixmap.toImage().convertToFormat(QImage.Format.Format_RGB888)
        self._map_image = img   # cached for _is_navigable_pixel fast lookup

        w, h = img.width(), img.height()
        bpl = img.bytesPerLine()
        ptr = img.constBits()
        ptr.setsize(bpl * h)
        rgb = np.frombuffer(ptr, dtype=np.uint8).reshape(h, bpl)[:, :w * 3].reshape(h, w, 3).copy()

        nav = (
            (np.abs(rgb[:, :, 0].astype(np.int16) - 28) < 20) &
            (np.abs(rgb[:, :, 1].astype(np.int16) - 48) < 20) &
            (np.abs(rgb[:, :, 2].astype(np.int16) - 80) < 20)
        )

        # Build RGBA mask: navigable pixels get a soft cyan tint, rest transparent.
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        rgba[nav, 0] = 100   # R
        rgba[nav, 1] = 220   # G
        rgba[nav, 2] = 180   # B
        rgba[nav, 3] = 55    # A — subtle so the map stays readable

        mask_bytes = rgba.tobytes()
        mask_img = QImage(mask_bytes, w, h, w * 4, QImage.Format.Format_RGBA8888)
        self._nav_mask = QPixmap.fromImage(mask_img.copy())
        self._nav_mask_scaled = None   # will be built on next _update_pixmap call

    def _is_navigable_pixel(self, overlay_x: float, overlay_y: float) -> bool:
        """Return True if the overlay coordinate falls on a navigable (free-space) pixel."""
        if self._map_image is None or self._scaled_w == 0:
            return True   # fail open when map not yet loaded
        label_origin = self._map_label.mapTo(self, QPoint(0, 0))
        x_pad = (self._map_label.width()  - self._scaled_w) / 2
        y_pad = (self._map_label.height() - self._scaled_h) / 2
        img_x = overlay_x - label_origin.x() - x_pad
        img_y = overlay_y - label_origin.y() - y_pad
        if img_x < 0 or img_y < 0 or img_x >= self._scaled_w or img_y >= self._scaled_h:
            return False
        scale = self._scaled_w / self._map_image.width()
        src_x = max(0, min(self._map_image.width()  - 1, int(img_x / scale)))
        src_y = max(0, min(self._map_image.height() - 1, int(img_y / scale)))
        c = self._map_image.pixel(src_x, src_y)
        r = (c >> 16) & 0xFF
        g = (c >> 8)  & 0xFF
        b = c         & 0xFF
        return abs(r - 28) < 20 and abs(g - 48) < 20 and abs(b - 80) < 20

    def _pixel_to_ros(self, overlay_x: float, overlay_y: float) -> Optional[tuple]:
        """Overlay widget pixel → ROS map-frame metres (inverse of _ros_to_display).

        Returns None when the click falls outside the displayed image area.
        """
        if self._pixmap.isNull() or self._scaled_w == 0 or self._scaled_h == 0:
            return None
        label_origin = self._map_label.mapTo(self, QPoint(0, 0))
        x_pad = (self._map_label.width()  - self._scaled_w) / 2
        y_pad = (self._map_label.height() - self._scaled_h) / 2
        img_x = overlay_x - label_origin.x() - x_pad
        img_y = overlay_y - label_origin.y() - y_pad
        if img_x < 0 or img_y < 0 or img_x > self._scaled_w or img_y > self._scaled_h:
            return None
        scale  = self._scaled_w / self._pixmap.width()
        src_px = img_x / scale
        src_py = img_y / scale
        ros_x  = self._map_origin_x + src_px * self._map_resolution
        ros_y  = self._map_origin_y + (self._pixmap.height() - src_py) * self._map_resolution
        return ros_x, ros_y

    # ------------------------------------------------------------------
    # Mouse / event filter
    # ------------------------------------------------------------------
    def eventFilter(self, obj, event) -> bool:
        """Intercept clicks inside _map_label for placement and navigation."""
        if obj is self._map_label and event.type() == QEvent.Type.MouseButtonPress:
            # Convert label-local coords to overlay-widget coords.
            label_origin = self._map_label.mapTo(self, QPoint(0, 0))
            overlay_x = event.position().x() + label_origin.x()
            overlay_y = event.position().y() + label_origin.y()
            self._handle_map_click(overlay_x, overlay_y)
            return True
        return super().eventFilter(obj, event)

    def set_selected_destination(self, name: Optional[str]) -> None:
        """Highlight the named marker as the active navigation destination (turns it red).

        The voice path passes a lower-cased name (main.py stores waypoints lower-cased),
        while the markers keep their original casing. Resolve to the marker's canonical
        name so a case mismatch never silently skips the highlight.
        """
        if name is not None:
            for pt in self._map_points:
                if pt["name"].lower() == name.lower():
                    name = pt["name"]
                    break
        self._selected_point = name
        self._marker_layer.update()

    def _handle_map_click(self, overlay_x: float, overlay_y: float) -> None:
        """Route a click: navigate to an existing marker or place a new one."""
        # Priority 1 — click on an existing marker (15 px hit radius).
        for pt in self._map_points:
            pos = self._ros_to_display(pt["ros_x"], pt["ros_y"])
            if pos is None:
                continue
            cx, cy = pos
            if (overlay_x - cx) ** 2 + (overlay_y - cy) ** 2 <= 225:
                # Toggle selection: clicking an already-selected marker deselects it.
                if self._selected_point == pt["name"]:
                    self._selected_point = None
                else:
                    self._selected_point = pt["name"]
                    self.nav_requested.emit(
                        pt["name"], pt["ros_x"], pt["ros_y"], pt.get("ros_yaw", 0.0)
                    )
                self._marker_layer.update()
                return

        # Priority 2 — placement mode active: create a new marker here.
        if self._placement_mode:
            if not self._is_navigable_pixel(overlay_x, overlay_y):
                self._hint_label.setText(
                    "Click on a navigable area (highlighted in teal)  ·  ESC to cancel"
                )
                return
            ros = self._pixel_to_ros(overlay_x, overlay_y)
            if ros is not None:
                self._pending_pos = ros
                self._ask_point_name_and_heading()

    def _ask_point_name_and_heading(self) -> None:
        """Custom dialog that collects waypoint name and heading in one step."""
        _ss = (
            "QDialog, QWidget {"
            f"  background-color: {Theme.BG_TOP.name()}; "
            f"  color: {Theme.ACCENT_BRIGHT.name()}; "
            "}"
            "QLabel {"
            f"  color: {Theme.ACCENT_BRIGHT.name()}; "
            "  font-family: 'Segoe UI'; font-size: 13px; letter-spacing: 2px; "
            "}"
            "QLineEdit, QSpinBox {"
            f"  background: rgba(10,20,40,220); color: {Theme.ACCENT_BRIGHT.name()}; "
            f"  border: 1px solid {Theme.ACCENT.name()}; padding: 6px; font-size: 13px; "
            "}"
            "QPushButton {"
            f"  background: rgba(20,50,90,220); color: {Theme.ACCENT_BRIGHT.name()}; "
            f"  border: 1px solid {Theme.ACCENT.name()}; padding: 6px 20px; font-size: 12px; "
            "}"
            "QPushButton:hover { background: rgba(30,70,120,240); }"
        )

        dlg = QDialog(self)
        dlg.setWindowTitle("New Waypoint")
        dlg.setStyleSheet(_ss)
        dlg.setMinimumWidth(340)

        layout = QVBoxLayout(dlg)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        # Name row
        layout.addWidget(QLabel("Location name:"))
        name_edit = QLineEdit()
        name_edit.setPlaceholderText("e.g. Lab A, Entrance, Cafeteria…")
        layout.addWidget(name_edit)

        # Heading row
        layout.addWidget(QLabel("Heading (degrees):"))
        spin = QSpinBox()
        spin.setRange(0, 359)
        spin.setValue(90)       # default: North (up on map)
        spin.setSuffix("°")
        spin.setToolTip("0 = East · 90 = North · 180 = West · 270 = South")
        layout.addWidget(spin)
        layout.addWidget(QLabel("0 = East  ·  90 = North  ·  180 = West  ·  270 = South"))

        # Buttons
        btn_row = QHBoxLayout()
        ok_btn  = QPushButton("CONFIRM")
        can_btn = QPushButton("CANCEL")
        btn_row.addWidget(ok_btn)
        btn_row.addWidget(can_btn)
        layout.addLayout(btn_row)

        ok_btn.clicked.connect(dlg.accept)
        can_btn.clicked.connect(dlg.reject)
        name_edit.returnPressed.connect(dlg.accept)

        accepted = dlg.exec() == QDialog.DialogCode.Accepted
        name = name_edit.text().strip()
        heading_deg = spin.value()

        if accepted and name and self._pending_pos is not None:
            ros_x, ros_y = self._pending_pos
            # Convert compass heading to ROS yaw: 0=East, 90=North, CCW positive
            ros_yaw = math.radians(heading_deg)
            self._map_points.append({
                "name":    name,
                "ros_x":   ros_x,
                "ros_y":   ros_y,
                "ros_yaw": ros_yaw,
            })
            self._save_points()
            self.location_placed.emit(name, ros_x, ros_y)
        self.exit_placement_mode()

    # ------------------------------------------------------------------
    # Persistence — map_points.json lives next to the project root
    # ------------------------------------------------------------------
    def _points_file_path(self) -> str:
        root = os.path.abspath(os.path.join(os.path.dirname(self._map_yaml_path), ".."))
        return os.path.join(root, "map_points.json")

    def _load_points(self) -> None:
        path = self._points_file_path()
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as fh:
                    data = json.load(fh)
                if isinstance(data, list):
                    self._map_points = data
            except Exception:
                self._map_points = []

    def _save_points(self) -> None:
        try:
            with open(self._points_file_path(), "w", encoding="utf-8") as fh:
                json.dump(self._map_points, fh, indent=2)
        except Exception as exc:
            print(f"[MapOverlay] Could not save map_points.json: {exc}")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()
        if hasattr(self, "_path_layer"):
            self._path_layer.setGeometry(self.rect())
        if hasattr(self, "_marker_layer"):
            self._marker_layer.setGeometry(self.rect())
            self._marker_layer.raise_()

    def _update_pixmap(self) -> None:
        if self._pixmap.isNull():
            self._map_label.setText(
                f"Map not found — expected {_LIDAR_MAP_DIR}/{_LIDAR_MAP_YAML} "
                f"and its PGM (looked in {self._map_yaml_path!r})"
            )
            self._map_label.setStyleSheet(
                f"color: {Theme.TEXT_DIM.name()}; font-size: 16px;"
            )
            return
        avail = self._map_label.size()
        if avail.width() <= 1 or avail.height() <= 1:
            return
        scaled = self._pixmap.scaled(
            avail,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._map_label.setPixmap(scaled)
        self._scaled_w = scaled.width()
        self._scaled_h = scaled.height()
        # Rescale the navigable highlight mask to match the new display size.
        if self._nav_mask is not None:
            self._nav_mask_scaled = self._nav_mask.scaled(
                self._scaled_w, self._scaled_h,
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        # Re-position the pose dot now that the image has been re-scaled.
        if self._last_pose is not None:
            self.set_pose(*self._last_pose)


# =============================================================================
# Address chip (SIR / MADAME)
# =============================================================================
class AddressChip(QLabel):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_address(None)

    def set_address(self, address: Optional[str]) -> None:
        if address is None:
            label = "AWAITING VOICE"
            color = Theme.TEXT_DIM
        else:
            label = "MASTER"
            color = Theme.ACCENT_BRIGHT
        self.setText(f"  ◆  {label}  ◆  ")
        self.setStyleSheet(
            f"color: {color.name()};"
            f"background-color: rgba(20,50,90,160);"
            "border: 1px solid rgba(80,170,255,160);"
            "border-radius: 14px;"
            "padding: 6px 14px;"
            "font-family: 'Segoe UI';"
            "font-size: 13px;"
            "font-weight: 600;"
            "letter-spacing: 4px;"
        )


# =============================================================================
# ROS2 Status Chip
# =============================================================================
class Ros2StatusChip(QLabel):
    """Small connection indicator shown in the top bar.

    Green dot  = ROS2 live (bridge connected).
    Red dot    = ROS2 offline / not started.
    Yellow dot = connecting / transitioning.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_connected(False)

    def set_connected(self, connected: bool) -> None:
        if connected:
            dot, label, color = "●", "ROS2  LIVE", QColor(80, 220, 120)
        else:
            dot, label, color = "●", "ROS2  OFF", QColor(200, 60, 60)
        self.setText(f"  {dot}  {label}  ")
        self.setStyleSheet(
            f"color: {color.name()};"
            "background-color: rgba(10,20,35,180);"
            "border: 1px solid rgba(80,170,255,80);"
            "border-radius: 14px;"
            "padding: 5px 12px;"
            "font-family: 'Segoe UI';"
            "font-size: 11px;"
            "font-weight: 600;"
            "letter-spacing: 3px;"
        )


class LocCovChip(QLabel):
    """Live AMCL localisation-quality readout for Developer Mode.

    Shows the covariance metric (worst of the X, Y and yaw variances) that
    gates goal publishing in main.py.  Green when it is below the gate, so the
    chair will drive; amber when it is at or above the gate, so goals are held
    until the pose tightens; dim when no /amcl_pose has arrived yet.
    """

    _GATE = 0.5

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_cov(float("inf"))

    def set_cov(self, cov: float) -> None:
        if cov == float("inf"):
            text, color = "LOC  σ² —", QColor(150, 160, 175)
        elif cov < self._GATE:
            text, color = f"LOC  σ² {cov:.3f}  OK", QColor(80, 220, 120)
        else:
            text, color = f"LOC  σ² {cov:.3f}  HOLD", QColor(235, 170, 60)
        self.setText(f"  ●  {text}  ")
        self.setStyleSheet(
            f"color: {color.name()};"
            "background-color: rgba(10,20,35,180);"
            "border: 1px solid rgba(80,170,255,80);"
            "border-radius: 14px;"
            "padding: 5px 12px;"
            "font-family: 'Segoe UI';"
            "font-size: 11px;"
            "font-weight: 600;"
            "letter-spacing: 3px;"
        )


# =============================================================================
# Developer log panel
# =============================================================================
class LogPanel(QWidget):
    """Scrollable process-output viewer with live text filter for Developer Mode.

    All launched processes write their stdout/stderr into a class-level queue
    via ``LogPanel.feed()``.  A QTimer drains that queue at 5 Hz and updates
    the text area, optionally filtered by the user's search string.
    """

    _queue: queue.Queue = queue.Queue()

    @classmethod
    def feed(cls, line: str) -> None:
        """Push a log line from any thread.  Non-blocking."""
        cls._queue.put_nowait(line)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._all_lines: list[str] = []

        header = QLabel("SYSTEM LOGS")
        header.setStyleSheet(
            f"color: {Theme.ACCENT_BRIGHT.name()}; "
            "font-family: 'Segoe UI'; font-size: 11px; font-weight: 600; "
            "letter-spacing: 4px; background: transparent;"
        )

        self._filter_edit = QLineEdit()
        self._filter_edit.setPlaceholderText("Filter logs…")
        self._filter_edit.textChanged.connect(self._refresh_view)
        self._filter_edit.setStyleSheet(
            f"background: rgba(10,20,40,200); color: {Theme.TEXT_PRIMARY.name()}; "
            "border: 1px solid rgba(80,170,255,80); border-radius: 4px; "
            "padding: 4px 8px; font-family: 'Segoe UI'; font-size: 11px;"
        )

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setMaximumHeight(130)
        self._text.setStyleSheet(
            "background: rgba(4,10,24,220); "
            f"color: {Theme.TEXT_DIM.name()}; "
            "border: 1px solid rgba(80,170,255,60); border-radius: 4px; "
            "font-family: 'Consolas', 'Courier New'; font-size: 11px; "
            "padding: 4px;"
        )

        filter_row = QHBoxLayout()
        filter_row.setContentsMargins(0, 0, 0, 0)
        filter_row.setSpacing(12)
        filter_row.addWidget(header)
        filter_row.addWidget(self._filter_edit, 1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addLayout(filter_row)
        layout.addWidget(self._text)

        self._poll = QTimer(self)
        self._poll.timeout.connect(self._drain)
        self._poll.start(200)   # 5 Hz — fast enough for log updates

    def _drain(self) -> None:
        """Drain the shared queue and refresh the view if new lines arrived."""
        added = False
        while not LogPanel._queue.empty():
            try:
                line = LogPanel._queue.get_nowait()
                self._all_lines.append(line)
                added = True
            except queue.Empty:
                break
        if added:
            self._refresh_view(self._filter_edit.text())

    def _refresh_view(self, text: str) -> None:
        lf = text.lower()
        lines = (
            self._all_lines if not lf
            else [l for l in self._all_lines if lf in l.lower()]
        )
        self._text.setPlainText("\n".join(lines[-500:]))
        sb = self._text.verticalScrollBar()
        sb.setValue(sb.maximum())


# =============================================================================
# Main window
# =============================================================================
class JarvisWindow(QMainWindow):
    """Top-level Jarvis interface.

    The voice subsystem connects to these signals:
        request_set_state(State)
        request_set_partial(str)
        request_set_final(str)
        request_set_reply(str)
        request_set_address(str)
        request_show_map()
        request_hide_map()

    Emit them from any thread; Qt's queued connections will marshal to
    the GUI thread.
    """

    # External-facing signals (thread-safe entry points).
    request_set_state = pyqtSignal(object)
    request_set_partial = pyqtSignal(str)
    request_set_final = pyqtSignal(str)
    request_set_reply = pyqtSignal(str)
    request_set_address = pyqtSignal(str)
    request_show_map = pyqtSignal()
    request_hide_map = pyqtSignal()

    # ROS2 signals (driven from the bridge thread → GUI thread via queued conn).
    request_set_ros2_connected = pyqtSignal(bool)
    request_set_robot_state    = pyqtSignal(str)
    request_set_pose           = pyqtSignal(float, float, float)   # (ros_x, ros_y, ros_yaw)
    request_set_loc_cov        = pyqtSignal(float)          # AMCL covariance metric (dev chip)
    request_set_map            = pyqtSignal(object)         # occupancy grid payload dict
    request_set_path           = pyqtSignal(object)         # list of (x, y) tuples from /plan

    # User-driven signals out (e.g., ESC pressed, E-stop clicked).
    map_dismissed    = pyqtSignal()
    quit_requested   = pyqtSignal()
    estop_requested  = pyqtSignal(bool)   # True = activate, False = release

    # Map waypoint signals — voice thread → GUI thread → main.py.
    request_enter_placement_mode     = pyqtSignal()                        # arm the next click
    map_location_placed              = pyqtSignal(str, float, float)       # (name, ros_x, ros_y)
    map_point_nav_requested          = pyqtSignal(str, float, float, float)  # (name, ros_x, ros_y, ros_yaw)
    request_set_selected_destination = pyqtSignal(str)                     # highlight marker red
    request_dev_command              = pyqtSignal(str)                     # voice dev commands

    def __init__(self, map_yaml_path: Optional[str] = None):
        super().__init__()
        self.setWindowTitle("Jarvis — Smart Wheelchair Voice Console")
        self.resize(1280, 800)
        self.setMinimumSize(900, 600)

        # ROS2 system process handles (None = not running)
        self._mapping_proc      = None
        self._localization_proc = None
        self._navigation_proc   = None
        self._tof_proc          = None
        self._imu_proc          = None
        self._canbus_proc       = None
        self._sensor_proc       = None

        # Current view mode: "user" | "developer"
        self._mode: str = "developer"

        if map_yaml_path is None:
            map_yaml_path = default_map_yaml_path()
        self._build_ui(map_yaml_path)
        self._wire_signals()

        # 1 Hz clock tick.
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)
        self._update_clock()

        self.set_state(State.IDLE)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_ui(self, map_yaml_path: str) -> None:
        central = QWidget(self)
        central.setObjectName("central")
        central.setStyleSheet(
            "QWidget#central {"
            "  background-color: qlineargradient("
            "    x1:0, y1:0, x2:0, y2:1,"
            f"   stop:0 {Theme.BG_TOP.name()},"
            f"   stop:1 {Theme.BG_BOTTOM.name()});"
            "}"
        )
        self.setCentralWidget(central)

        # Top strip: title left, address chip right.
        self._title_label = QLabel("J A R V I S")
        self._title_label.setStyleSheet(
            f"color: {Theme.ACCENT_BRIGHT.name()}; "
            "font-family: 'Segoe UI'; font-size: 16px; font-weight: 600; "
            "letter-spacing: 8px;"
        )

        self._subtitle_label = QLabel("SMART WHEELCHAIR · VOICE CONSOLE")
        self._subtitle_label.setStyleSheet(
            f"color: {Theme.TEXT_DIM.name()}; "
            "font-family: 'Segoe UI'; font-size: 11px; letter-spacing: 4px;"
        )

        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(self._title_label)
        title_box.addWidget(self._subtitle_label)

        self._chip = AddressChip()
        self._ros2_chip = Ros2StatusChip()

        # Mode toggle buttons — shown in the top bar, voice-controllable too.
        self._mode_user_btn = QPushButton("USER")
        self._mode_user_btn.setMinimumHeight(28)
        self._mode_user_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._mode_user_btn.clicked.connect(lambda: self._set_mode("user"))

        self._mode_dev_btn = QPushButton("DEV")
        self._mode_dev_btn.setMinimumHeight(28)
        self._mode_dev_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._mode_dev_btn.clicked.connect(lambda: self._set_mode("developer"))

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(28, 22, 28, 0)
        top_bar.addLayout(title_box)
        top_bar.addStretch()
        top_bar.addWidget(self._mode_user_btn)
        top_bar.addSpacing(4)
        top_bar.addWidget(self._mode_dev_btn)
        top_bar.addSpacing(10)
        self._loc_cov_chip = LocCovChip()
        top_bar.addWidget(self._chip)
        top_bar.addSpacing(10)
        top_bar.addWidget(self._ros2_chip)
        top_bar.addSpacing(10)
        top_bar.addWidget(self._loc_cov_chip)

        # Centre: orb.
        self._orb = JarvisOrb()

        # Clock.
        self._clock_label = QLabel("00:00")
        self._clock_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._clock_label.setStyleSheet(
            f"color: {Theme.ACCENT_BRIGHT.name()}; "
            "font-family: 'Consolas', 'Segoe UI'; "
            "font-size: 64px; font-weight: 300; letter-spacing: 14px;"
            "background: transparent;"
        )

        # Status (LISTENING / SPEAKING / ...).
        self._state_label = QLabel(State.IDLE.value)
        self._state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state_label.setStyleSheet(
            f"color: {Theme.ACCENT.name()}; "
            "font-family: 'Segoe UI'; font-size: 13px; "
            "font-weight: 600; letter-spacing: 8px;"
        )

        # Robot navigation state (updated from ROS2 /wheelchair/status).
        self._robot_state_label = QLabel("")
        self._robot_state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._robot_state_label.setStyleSheet(
            "color: rgba(80, 220, 120, 210); "
            "font-family: 'Segoe UI'; font-size: 11px; "
            "font-weight: 500; letter-spacing: 5px;"
            "background: transparent;"
        )
        self._robot_state_label.setVisible(False)

        # Transcript (live partial / final).
        self._transcript_label = QLabel("")
        self._transcript_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._transcript_label.setStyleSheet(
            f"color: {Theme.TEXT_PRIMARY.name()}; "
            "font-family: 'Segoe UI'; font-size: 18px; font-weight: 300;"
            "background: transparent;"
        )
        self._transcript_label.setWordWrap(True)
        self._transcript_label.setMinimumHeight(56)

        # Reply.
        self._reply_label = QLabel("")
        self._reply_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._reply_label.setStyleSheet(
            f"color: {Theme.ACCENT_BRIGHT.name()}; "
            "font-family: 'Segoe UI'; font-size: 16px; font-weight: 400;"
            "font-style: italic;"
            "background: transparent;"
        )
        self._reply_label.setWordWrap(True)
        self._reply_label.setMinimumHeight(46)

        # Map + emergency-stop buttons.
        self._map_btn = QPushButton("  SHOW MAP")
        self._map_btn.setMinimumHeight(46)
        self._map_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._map_btn.clicked.connect(self._on_map_btn_clicked)
        self._apply_map_btn_style(open=False)

        self._estop_active = False
        self._estop_btn = QPushButton("  EMERGENCY STOP")
        self._estop_btn.setMinimumHeight(46)
        self._estop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._estop_btn.clicked.connect(self._on_estop_clicked)
        self._apply_estop_style(active=False)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)
        btn_row.addWidget(self._map_btn)
        btn_row.addWidget(self._estop_btn)

        # Launcher row — Start/Stop Mapping / Localization / Navigation + Stop All.
        self._mapping_btn = QPushButton("  START MAPPING")
        self._mapping_btn.setMinimumHeight(36)
        self._mapping_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._mapping_btn.clicked.connect(self._on_mapping_clicked)
        self._apply_launcher_btn_style(self._mapping_btn, running=False)

        self._localization_btn = QPushButton("  START LOCALIZATION")
        self._localization_btn.setMinimumHeight(36)
        self._localization_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._localization_btn.clicked.connect(self._on_localization_clicked)
        self._apply_launcher_btn_style(self._localization_btn, running=False)

        self._navigation_btn = QPushButton("  START NAVIGATION")
        self._navigation_btn.setMinimumHeight(36)
        self._navigation_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._navigation_btn.clicked.connect(self._on_navigation_clicked)
        self._apply_launcher_btn_style(self._navigation_btn, running=False)

        self._start_all_btn = QPushButton("  START ALL")
        self._start_all_btn.setMinimumHeight(36)
        self._start_all_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._start_all_btn.clicked.connect(self._on_start_all_clicked)
        self._start_all_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: rgba(10, 70, 30, 210);"
            "  color: rgba(100,230,140,230);"
            "  border: 1px solid rgba(60,200,100,150);"
            "  border-radius: 6px;"
            "  font-family: 'Segoe UI'; font-size: 11px; font-weight: 600;"
            "  letter-spacing: 4px; padding: 6px 20px;"
            "}"
            "QPushButton:hover { background-color: rgba(15, 100, 45, 230); }"
            "QPushButton:pressed { background-color: rgba(8, 55, 20, 255); }"
        )

        self._stop_all_btn = QPushButton("  STOP ALL")
        self._stop_all_btn.setMinimumHeight(36)
        self._stop_all_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._stop_all_btn.clicked.connect(self._on_stop_all_clicked)
        self._stop_all_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: rgba(100, 30, 10, 200);"
            "  color: rgba(255,180,140,230);"
            "  border: 1px solid rgba(255,120,60,150);"
            "  border-radius: 6px;"
            "  font-family: 'Segoe UI'; font-size: 11px; font-weight: 600;"
            "  letter-spacing: 4px; padding: 6px 20px;"
            "}"
            "QPushButton:hover { background-color: rgba(150, 50, 15, 230); }"
            "QPushButton:pressed { background-color: rgba(200, 60, 10, 255); }"
        )

        _launcher_inner = QHBoxLayout()
        _launcher_inner.setSpacing(8)
        _launcher_inner.setContentsMargins(0, 0, 0, 0)
        _launcher_inner.addWidget(self._mapping_btn)
        _launcher_inner.addWidget(self._localization_btn)
        _launcher_inner.addWidget(self._navigation_btn)
        _launcher_inner.addWidget(self._start_all_btn)
        _launcher_inner.addWidget(self._stop_all_btn)
        self._launcher_widget = QWidget()
        self._launcher_widget.setLayout(_launcher_inner)

        # ── Developer tool buttons (row 2) ──────────────────────────────────
        self._tof_btn = QPushButton("  TOF CALIBRATION")
        self._tof_btn.setMinimumHeight(36)
        self._tof_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tof_btn.clicked.connect(self._on_tof_clicked)
        self._apply_launcher_btn_style(self._tof_btn, running=False)

        self._imu_btn = QPushButton("  IMU CALIBRATION")
        self._imu_btn.setMinimumHeight(36)
        self._imu_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._imu_btn.clicked.connect(self._on_imu_clicked)
        self._apply_launcher_btn_style(self._imu_btn, running=False)

        self._canbus_btn = QPushButton("  NODE STATUS (CANBUS)")
        self._canbus_btn.setMinimumHeight(36)
        self._canbus_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._canbus_btn.clicked.connect(self._on_canbus_clicked)
        self._apply_launcher_btn_style(self._canbus_btn, running=False)

        self._sensor_btn = QPushButton("  SENSOR READOUTS")
        self._sensor_btn.setMinimumHeight(36)
        self._sensor_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._sensor_btn.clicked.connect(self._on_sensor_clicked)
        self._apply_launcher_btn_style(self._sensor_btn, running=False)

        self._create_marker_btn = QPushButton("  CREATE MARKER")
        self._create_marker_btn.setMinimumHeight(36)
        self._create_marker_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._create_marker_btn.clicked.connect(self.enter_placement_mode)
        self._apply_launcher_btn_style(self._create_marker_btn, running=False)

        _dev_inner = QHBoxLayout()
        _dev_inner.setSpacing(8)
        _dev_inner.setContentsMargins(0, 0, 0, 0)
        _dev_inner.addWidget(self._tof_btn)
        _dev_inner.addWidget(self._imu_btn)
        _dev_inner.addWidget(self._canbus_btn)
        _dev_inner.addWidget(self._sensor_btn)
        _dev_inner.addWidget(self._create_marker_btn)
        self._dev_tools_widget = QWidget()
        self._dev_tools_widget.setLayout(_dev_inner)

        # ── Log panel ────────────────────────────────────────────────────────
        self._log_panel = LogPanel()

        # Footer hint.
        self._hint_label = QLabel(
            'Say "Jarvis" to wake  ·  SHOW MAP button or "show me the map"  ·  ESC to exit'
        )
        self._hint_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._hint_label.setStyleSheet(
            f"color: {Theme.TEXT_DIM.name()}; "
            "font-family: 'Segoe UI'; font-size: 11px; letter-spacing: 3px;"
            "background: transparent;"
        )

        # Layout assembly.
        body = QVBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(8)
        body.addLayout(top_bar)
        body.addWidget(self._orb, 1)
        body.addWidget(self._clock_label)
        body.addSpacing(4)
        body.addWidget(self._state_label)
        body.addWidget(self._robot_state_label)
        body.addSpacing(2)
        body.addWidget(self._transcript_label)
        body.addWidget(self._reply_label)
        body.addSpacing(8)
        body.addWidget(self._hint_label)
        body.addSpacing(6)
        body.addWidget(self._launcher_widget)
        body.addSpacing(4)
        body.addWidget(self._dev_tools_widget)
        body.addSpacing(4)
        body.addWidget(self._log_panel)
        body.addSpacing(8)
        body.addLayout(btn_row)
        body.addSpacing(18)
        central.setLayout(body)

        # Start in Developer Mode by default.
        self._apply_mode("developer")

        # Map overlay covers central widget.
        self._map = MapOverlay(map_yaml_path, self)
        self._map.setGeometry(self.rect())
        self._map.closed.connect(self._on_map_overlay_closed)
        self._map.closed.connect(self.map_dismissed.emit)
        self._map.location_placed.connect(self._on_location_placed)
        self._map.nav_requested.connect(self._on_map_nav_requested)  # (name, x, y, yaw)

    def _wire_signals(self) -> None:
        # Use queued connections so any thread can drive the GUI safely.
        self.request_set_state.connect(self.set_state, Qt.ConnectionType.QueuedConnection)
        self.request_set_partial.connect(self.set_partial, Qt.ConnectionType.QueuedConnection)
        self.request_set_final.connect(self.set_final, Qt.ConnectionType.QueuedConnection)
        self.request_set_reply.connect(self.set_reply, Qt.ConnectionType.QueuedConnection)
        self.request_set_address.connect(self.set_address, Qt.ConnectionType.QueuedConnection)
        self.request_show_map.connect(self.show_map, Qt.ConnectionType.QueuedConnection)
        self.request_hide_map.connect(self.hide_map, Qt.ConnectionType.QueuedConnection)
        # ROS2 signals.
        self.request_set_ros2_connected.connect(
            self.set_ros2_connected, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_robot_state.connect(
            self.set_robot_state, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_pose.connect(
            self.set_pose, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_loc_cov.connect(
            self.set_loc_cov, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_map.connect(
            self.set_map, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_path.connect(
            self.set_path, Qt.ConnectionType.QueuedConnection
        )
        self.request_enter_placement_mode.connect(
            self.enter_placement_mode, Qt.ConnectionType.QueuedConnection
        )
        self.request_set_selected_destination.connect(
            self.set_selected_destination, Qt.ConnectionType.QueuedConnection
        )
        self.request_dev_command.connect(
            self._on_dev_command, Qt.ConnectionType.QueuedConnection
        )

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------
    def set_state(self, state) -> None:
        if isinstance(state, str):
            try:
                state = State(state)
            except ValueError:
                return
        self._orb.set_state(state)
        self._state_label.setText(state.value)

    def set_partial(self, text: str) -> None:
        if not text:
            self._transcript_label.setText("")
            return
        self._transcript_label.setText(f"“{text}…”")

    def set_final(self, text: str) -> None:
        if text:
            self._transcript_label.setText(f"“{text}”")

    def set_reply(self, text: str) -> None:
        self._reply_label.setText(text or "")

    def set_address(self, address: Optional[str]) -> None:
        self._chip.set_address(address)

    def show_map(self) -> None:
        self._map.setGeometry(self.rect())
        self._map.fade_in()
        self._apply_map_btn_style(open=True)

    def hide_map(self) -> None:
        self._map.fade_out()
        self._apply_map_btn_style(open=False)

    # ROS2 slots -------------------------------------------------------
    def set_ros2_connected(self, connected: bool) -> None:
        """Update the ROS2 status chip in the top bar."""
        self._ros2_chip.set_connected(connected)

    def set_loc_cov(self, cov: float) -> None:
        """Update the Developer-mode AMCL localisation-quality chip."""
        self._loc_cov_chip.set_cov(cov)

    def set_robot_state(self, state: str) -> None:
        """
        Show the wheelchair's navigation state below the voice state label.

        Coloured by urgency:
          NAVIGATING / ARRIVED  → green
          IDLE / empty          → hidden
          OBSTACLE / ESTOP / ERROR → orange/red
        """
        if not state:
            self._robot_state_label.setVisible(False)
            return
        upper = state.upper()
        color_map = {
            "NAVIGATING": "rgba(80, 220, 120, 210)",
            "ARRIVED":    "rgba(120, 255, 140, 240)",
            "IDLE":       "rgba(120, 150, 190, 180)",
            "OBSTACLE":   "rgba(255, 180, 50, 230)",
            "ESTOP":      "rgba(255, 70, 70, 240)",
            "ERROR":      "rgba(255, 70, 70, 240)",
        }
        color = color_map.get(upper, "rgba(120, 150, 190, 180)")
        self._robot_state_label.setStyleSheet(
            f"color: {color}; "
            "font-family: 'Segoe UI'; font-size: 11px; "
            "font-weight: 500; letter-spacing: 5px;"
            "background: transparent;"
        )
        self._robot_state_label.setText(f"ROBOT  ·  {upper}")
        self._robot_state_label.setVisible(True)

    def set_pose(self, ros_x: float, ros_y: float, ros_yaw: float = 0.0) -> None:
        """Update the 'you are here' marker on the map overlay.

        Safe to call at any time — the map overlay stores the pose and
        re-applies it whenever the map is next opened or the window resizes.
        ros_yaw (radians) drives the green heading arrow.
        """
        self._map.set_pose(ros_x, ros_y, ros_yaw)

    def set_map(self, payload: object) -> None:
        """Replace the static EEMCS image with the live sensor map.

        Called automatically when an OccupancyGrid arrives on /map.
        Also updates the coordinate transform used by the pose dot.
        """
        self._map.set_map(payload)

    def set_path(self, points: list) -> None:
        """Draw the Nav2 planned path on the map overlay.

        Called automatically when a new path arrives on /plan.
        Pass an empty list to clear the overlay.
        """
        self._map.set_path(points)

    # Waypoint slots ---------------------------------------------------
    def enter_placement_mode(self) -> None:
        """Open the map (if closed) and arm the next click as a new waypoint.

        Called from the GUI thread via request_enter_placement_mode signal, which
        the voice observer emits on the on_create_point callback.
        """
        if not self._map.isVisible():
            self.show_map()
        self._map.enter_placement_mode()
        self.set_reply("Tap the map to place your marker, Master.")

    @property
    def map_points(self) -> list:
        """Return the list of stored waypoint dicts ({name, ros_x, ros_y})."""
        return self._map._map_points

    def _on_location_placed(self, name: str, ros_x: float, ros_y: float) -> None:
        self.set_reply(f"Location '{name}' saved, Master.")
        self.map_location_placed.emit(name, ros_x, ros_y)

    def _on_map_nav_requested(self, name: str, ros_x: float, ros_y: float, ros_yaw: float) -> None:
        """Forward a click-on-marker event to main.py for ROS2 dispatch."""
        self._map.set_selected_destination(name)
        self.map_point_nav_requested.emit(name, ros_x, ros_y, ros_yaw)

    def set_selected_destination(self, name: Optional[str]) -> None:
        """Highlight a named marker red — called by voice when a navigate command is confirmed."""
        self._map.set_selected_destination(name)

    # Map button -------------------------------------------------------
    def _on_map_btn_clicked(self) -> None:
        """Toggle the LiDAR map overlay (same as the voice 'show me the map' command)."""
        if self._map.isVisible():
            self.hide_map()
        else:
            self.show_map()

    def _on_map_overlay_closed(self) -> None:
        """Keep the map button label in sync when the overlay is dismissed (ESC / voice)."""
        self._apply_map_btn_style(open=False)

    def _apply_map_btn_style(self, *, open: bool) -> None:
        if open:
            self._map_btn.setText("  HIDE MAP")
            self._map_btn.setStyleSheet(
                "QPushButton {"
                f"  background-color: rgba({Theme.ACCENT.red()}, {Theme.ACCENT.green()}, "
                f"  {Theme.ACCENT.blue()}, 200);"
                "  color: white;"
                f"  border: 2px solid rgba({Theme.ACCENT_BRIGHT.red()}, "
                f"  {Theme.ACCENT_BRIGHT.green()}, {Theme.ACCENT_BRIGHT.blue()}, 200);"
                "  border-radius: 8px;"
                "  font-family: 'Segoe UI';"
                "  font-size: 13px;"
                "  font-weight: 600;"
                "  letter-spacing: 5px;"
                "  padding: 8px 40px;"
                "}"
                "QPushButton:hover {"
                f"  background-color: rgba({Theme.ACCENT_BRIGHT.red()}, "
                f"  {Theme.ACCENT_BRIGHT.green()}, {Theme.ACCENT_BRIGHT.blue()}, 220);"
                "}"
                "QPushButton:pressed {"
                f"  background-color: rgba({Theme.ACCENT_DIM.red()}, "
                f"  {Theme.ACCENT_DIM.green()}, {Theme.ACCENT_DIM.blue()}, 230);"
                "}"
            )
        else:
            self._map_btn.setText("  SHOW MAP")
            self._map_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: rgba(20, 50, 90, 220);"
                f"  color: {Theme.TEXT_PRIMARY.name()};"
                f"  border: 2px solid rgba({Theme.ACCENT.red()}, {Theme.ACCENT.green()}, "
                f"  {Theme.ACCENT.blue()}, 160);"
                "  border-radius: 8px;"
                "  font-family: 'Segoe UI';"
                "  font-size: 13px;"
                "  font-weight: 600;"
                "  letter-spacing: 5px;"
                "  padding: 8px 40px;"
                "}"
                "QPushButton:hover {"
                "  background-color: rgba(30, 70, 120, 240);"
                f"  border-color: rgba({Theme.ACCENT_BRIGHT.red()}, "
                f"  {Theme.ACCENT_BRIGHT.green()}, {Theme.ACCENT_BRIGHT.blue()}, 200);"
                "}"
                "QPushButton:pressed {"
                "  background-color: rgba(15, 40, 75, 255);"
                "}"
            )

    # E-stop -----------------------------------------------------------
    def _on_estop_clicked(self) -> None:
        """Toggle emergency stop and emit the signal."""
        self._estop_active = not self._estop_active
        self._apply_estop_style(self._estop_active)
        self.estop_requested.emit(self._estop_active)
        if self._estop_active:
            self.set_reply("Emergency stop activated.")
            self.set_robot_state("ESTOP")
        else:
            self.set_reply("Emergency stop released.")
            self.set_robot_state("IDLE")

    def _apply_estop_style(self, active: bool) -> None:
        if active:
            self._estop_btn.setText("  RELEASE E-STOP")
            self._estop_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: rgba(255, 140, 0, 230);"
                "  color: white;"
                "  border: 2px solid rgba(255, 200, 80, 200);"
                "  border-radius: 8px;"
                "  font-family: 'Segoe UI';"
                "  font-size: 13px;"
                "  font-weight: 700;"
                "  letter-spacing: 5px;"
                "  padding: 8px 40px;"
                "}"
                "QPushButton:hover {"
                "  background-color: rgba(255, 170, 20, 255);"
                "}"
            )
        else:
            self._estop_btn.setText("  EMERGENCY STOP")
            self._estop_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: rgba(160, 25, 25, 210);"
                "  color: white;"
                "  border: 2px solid rgba(240, 70, 70, 180);"
                "  border-radius: 8px;"
                "  font-family: 'Segoe UI';"
                "  font-size: 13px;"
                "  font-weight: 700;"
                "  letter-spacing: 5px;"
                "  padding: 8px 40px;"
                "}"
                "QPushButton:hover {"
                "  background-color: rgba(210, 40, 40, 240);"
                "  border-color: rgba(255, 100, 100, 220);"
                "}"
                "QPushButton:pressed {"
                "  background-color: rgba(255, 20, 20, 255);"
                "}"
            )

    # Launcher -----------------------------------------------------------
    def _apply_launcher_btn_style(self, btn: QPushButton, *, running: bool) -> None:
        if running:
            btn.setStyleSheet(
                "QPushButton {"
                "  background-color: rgba(15, 60, 30, 210);"
                "  color: rgba(100, 230, 140, 230);"
                "  border: 1px solid rgba(80, 200, 110, 160);"
                "  border-radius: 6px;"
                "  font-family: 'Segoe UI'; font-size: 11px; font-weight: 600;"
                "  letter-spacing: 4px; padding: 6px 20px;"
                "}"
                "QPushButton:hover { background-color: rgba(20, 80, 40, 240); }"
                "QPushButton:pressed { background-color: rgba(10, 45, 20, 255); }"
            )
        else:
            btn.setStyleSheet(
                "QPushButton {"
                "  background-color: rgba(15, 30, 55, 200);"
                f" color: {Theme.TEXT_DIM.name()};"
                "  border: 1px solid rgba(80,170,255,80);"
                "  border-radius: 6px;"
                "  font-family: 'Segoe UI'; font-size: 11px; font-weight: 600;"
                "  letter-spacing: 4px; padding: 6px 20px;"
                "}"
                "QPushButton:hover { background-color: rgba(25, 50, 90, 230); }"
                "QPushButton:pressed { background-color: rgba(10, 25, 50, 255); }"
            )

    def _on_mapping_clicked(self) -> None:
        if self._mapping_proc is not None and self._mapping_proc.poll() is None:
            _kill_proc(self._mapping_proc, "Mapping")
            self._mapping_proc = None
            self._apply_launcher_btn_style(self._mapping_btn, running=False)
            self._mapping_btn.setText("  START MAPPING")
        else:
            self._mapping_proc = _launch_proc(_MAPPING_CMD, "Mapping")
            self._apply_launcher_btn_style(self._mapping_btn, running=True)
            self._mapping_btn.setText("  STOP MAPPING")

    def _on_localization_clicked(self) -> None:
        if self._localization_proc is not None and self._localization_proc.poll() is None:
            _kill_proc(self._localization_proc, "Localization")
            self._localization_proc = None
            self._apply_launcher_btn_style(self._localization_btn, running=False)
            self._localization_btn.setText("  START LOCALIZATION")
        else:
            self._localization_proc = _launch_proc(_LOCALIZATION_CMD, "Localization")
            self._apply_launcher_btn_style(self._localization_btn, running=True)
            self._localization_btn.setText("  STOP LOCALIZATION")

    def _on_navigation_clicked(self) -> None:
        if self._navigation_proc is not None and self._navigation_proc.poll() is None:
            _kill_proc(self._navigation_proc, "Navigation")
            self._navigation_proc = None
            self._apply_launcher_btn_style(self._navigation_btn, running=False)
            self._navigation_btn.setText("  START NAVIGATION")
        else:
            self._navigation_proc = _launch_proc(_NAVIGATION_CMD, "Navigation")
            self._apply_launcher_btn_style(self._navigation_btn, running=True)
            self._navigation_btn.setText("  STOP NAVIGATION")

    def _on_start_all_clicked(self) -> None:
        """Launch Mapping, Localization, and Navigation in one shot."""
        if self._mapping_proc is None or self._mapping_proc.poll() is not None:
            self._mapping_proc = _launch_proc(_MAPPING_CMD, "Mapping")
            self._apply_launcher_btn_style(self._mapping_btn, running=True)
            self._mapping_btn.setText("  STOP MAPPING")

        if self._localization_proc is None or self._localization_proc.poll() is not None:
            self._localization_proc = _launch_proc(_LOCALIZATION_CMD, "Localization")
            self._apply_launcher_btn_style(self._localization_btn, running=True)
            self._localization_btn.setText("  STOP LOCALIZATION")

        if self._navigation_proc is None or self._navigation_proc.poll() is not None:
            self._navigation_proc = _launch_proc(_NAVIGATION_CMD, "Navigation")
            self._apply_launcher_btn_style(self._navigation_btn, running=True)
            self._navigation_btn.setText("  STOP NAVIGATION")

    def _on_stop_all_clicked(self) -> None:
        for proc, name in (
            (self._navigation_proc,   "Navigation"),
            (self._localization_proc, "Localization"),
            (self._mapping_proc,      "Mapping"),
            (self._tof_proc,          "TOF Cal"),
            (self._imu_proc,          "IMU Cal"),
            (self._canbus_proc,       "CANBUS"),
            (self._sensor_proc,       "Sensors"),
        ):
            _kill_proc(proc, name)
        self._mapping_proc = self._localization_proc = self._navigation_proc = None
        self._tof_proc = self._imu_proc = self._canbus_proc = self._sensor_proc = None
        for btn, label in (
            (self._mapping_btn,      "  START MAPPING"),
            (self._localization_btn, "  START LOCALIZATION"),
            (self._navigation_btn,   "  START NAVIGATION"),
            (self._tof_btn,          "  TOF CALIBRATION"),
            (self._imu_btn,          "  IMU CALIBRATION"),
            (self._canbus_btn,       "  NODE STATUS (CANBUS)"),
            (self._sensor_btn,       "  SENSOR READOUTS"),
        ):
            self._apply_launcher_btn_style(btn, running=False)
            btn.setText(label)

    # Mode switching ---------------------------------------------------
    def _set_mode(self, mode: str) -> None:
        """Switch between 'user' and 'developer' view modes."""
        if mode == self._mode:
            return
        self._apply_mode(mode)

    def _apply_mode(self, mode: str) -> None:
        """Show / hide widgets based on the selected mode and refresh button styles."""
        self._mode = mode
        is_dev = mode == "developer"

        # Developer-only widgets.
        self._ros2_chip.setVisible(is_dev)
        self._loc_cov_chip.setVisible(is_dev)
        self._transcript_label.setVisible(is_dev)
        self._launcher_widget.setVisible(is_dev)
        self._dev_tools_widget.setVisible(is_dev)
        self._log_panel.setVisible(is_dev)

        # Subtitle reflects the active mode.
        self._subtitle_label.setText(
            "SMART WHEELCHAIR · DEVELOPER CONSOLE"
            if is_dev else
            "SMART WHEELCHAIR · VOICE CONSOLE"
        )

        self._apply_mode_btn_styles()

    def _apply_mode_btn_styles(self) -> None:
        is_dev = self._mode == "developer"
        _active = (
            "QPushButton {"
            "  background-color: rgba(80,170,255,180);"
            "  color: white;"
            "  border: 1px solid rgba(150,210,255,200);"
            "  border-radius: 5px;"
            "  font-family: 'Segoe UI'; font-size: 10px; font-weight: 700;"
            "  letter-spacing: 3px; padding: 4px 16px;"
            "}"
        )
        _inactive = (
            "QPushButton {"
            "  background-color: rgba(15,30,55,200);"
            f" color: {Theme.TEXT_DIM.name()};"
            "  border: 1px solid rgba(80,170,255,50);"
            "  border-radius: 5px;"
            "  font-family: 'Segoe UI'; font-size: 10px; font-weight: 600;"
            "  letter-spacing: 3px; padding: 4px 16px;"
            "}"
            "QPushButton:hover { background-color: rgba(25,50,90,220); }"
        )
        self._mode_user_btn.setStyleSheet(_inactive if is_dev else _active)
        self._mode_dev_btn.setStyleSheet(_active if is_dev else _inactive)

    # Developer tool handlers ------------------------------------------
    def _on_tof_clicked(self) -> None:
        if self._tof_proc is not None and self._tof_proc.poll() is None:
            _kill_proc(self._tof_proc, "TOF Cal")
            self._tof_proc = None
            self._apply_launcher_btn_style(self._tof_btn, running=False)
            self._tof_btn.setText("  TOF CALIBRATION")
        else:
            self._tof_proc = _launch_proc(_TOF_CALIB_CMD, "TOF Cal")
            self._apply_launcher_btn_style(self._tof_btn, running=True)
            self._tof_btn.setText("  STOP TOF CAL")

    def _on_imu_clicked(self) -> None:
        if self._imu_proc is not None and self._imu_proc.poll() is None:
            _kill_proc(self._imu_proc, "IMU Cal")
            self._imu_proc = None
            self._apply_launcher_btn_style(self._imu_btn, running=False)
            self._imu_btn.setText("  IMU CALIBRATION")
        else:
            self._imu_proc = _launch_proc(_IMU_CALIB_CMD, "IMU Cal")
            self._apply_launcher_btn_style(self._imu_btn, running=True)
            self._imu_btn.setText("  STOP IMU CAL")

    def _on_canbus_clicked(self) -> None:
        if self._canbus_proc is not None and self._canbus_proc.poll() is None:
            _kill_proc(self._canbus_proc, "CANBUS")
            self._canbus_proc = None
            self._apply_launcher_btn_style(self._canbus_btn, running=False)
            self._canbus_btn.setText("  NODE STATUS (CANBUS)")
        else:
            self._canbus_proc = _launch_proc(_CANBUS_CMD, "CANBUS")
            self._apply_launcher_btn_style(self._canbus_btn, running=True)
            self._canbus_btn.setText("  STOP CANBUS")

    def _on_sensor_clicked(self) -> None:
        if self._sensor_proc is not None and self._sensor_proc.poll() is None:
            _kill_proc(self._sensor_proc, "Sensors")
            self._sensor_proc = None
            self._apply_launcher_btn_style(self._sensor_btn, running=False)
            self._sensor_btn.setText("  SENSOR READOUTS")
        else:
            self._sensor_proc = _launch_proc(_SENSOR_CMD, "Sensors")
            self._apply_launcher_btn_style(self._sensor_btn, running=True)
            self._sensor_btn.setText("  STOP SENSORS")

    # Voice dev commands -----------------------------------------------
    def _on_dev_command(self, command: str) -> None:
        """Dispatch a voice-issued developer command to the correct handler.

        Called via request_dev_command signal (queued, always on GUI thread).
        Each command string maps directly to the button that would otherwise
        be clicked in Developer Mode.
        """
        if command == "start_mapping":
            if self._mapping_proc is None or self._mapping_proc.poll() is not None:
                self._on_mapping_clicked()
        elif command == "stop_mapping":
            if self._mapping_proc is not None and self._mapping_proc.poll() is None:
                self._on_mapping_clicked()
        elif command == "start_localization":
            if self._localization_proc is None or self._localization_proc.poll() is not None:
                self._on_localization_clicked()
        elif command == "stop_localization":
            if self._localization_proc is not None and self._localization_proc.poll() is None:
                self._on_localization_clicked()
        elif command == "start_navigation":
            if self._navigation_proc is None or self._navigation_proc.poll() is not None:
                self._on_navigation_clicked()
        elif command == "stop_navigation":
            if self._navigation_proc is not None and self._navigation_proc.poll() is None:
                self._on_navigation_clicked()
        elif command == "start_all":
            self._on_start_all_clicked()
        elif command == "stop_all":
            self._on_stop_all_clicked()
        elif command == "mode_developer":
            self._set_mode("developer")
        elif command == "mode_user":
            self._set_mode("user")
        elif command == "emergency_stop":
            self._on_estop_clicked()
        elif command == "create_marker":
            self.enter_placement_mode()

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------
    def _update_clock(self) -> None:
        from datetime import datetime
        now = datetime.now()
        self._clock_label.setText(now.strftime("%H:%M"))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._map.setGeometry(self.rect())

    def closeEvent(self, event) -> None:
        """Ensure all launched processes are killed when the window closes."""
        self._on_stop_all_clicked()
        super().closeEvent(event)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_Escape:
            if self._map.isVisible():
                self.hide_map()
            else:
                self.quit_requested.emit()
                self.close()
        elif event.key() == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        else:
            super().keyPressEvent(event)


# =============================================================================
# Stand-alone preview (so the GUI can be eyeballed without the audio stack)
# =============================================================================
def _preview() -> None:
    """Run the GUI on its own with a scripted demo loop."""
    app = QApplication(sys.argv)
    here = os.path.dirname(os.path.abspath(__file__))
    win = JarvisWindow(map_yaml_path=os.path.join(here, _LIDAR_MAP_DIR, _LIDAR_MAP_YAML))
    win.show()

    # Walk through states so you can see the orb and ROS2 panel react.
    script = [
        (1500,  lambda: win.set_state(State.IDLE)),
        (3000,  lambda: win.set_state(State.LISTENING)),
        (4000,  lambda: win.set_partial("jarvis")),
        (4500,  lambda: (win.set_final("jarvis"), win.set_state(State.AWAKE))),
        (5000,  lambda: win.set_reply("What do you want, Master?")),
        (6500,  lambda: win.set_state(State.LISTENING)),
        (7500,  lambda: win.set_partial("take me to lab a")),
        (8200,  lambda: (win.set_final("take me to lab a"),
                         win.set_state(State.SPEAKING),
                         win.set_reply("Navigating to Lab A, Master."))),
        (8500,  lambda: (win.set_ros2_connected(True),
                         win.set_robot_state("NAVIGATING"))),
        (11000, lambda: win.set_robot_state("ARRIVED")),
        (11500, lambda: win.set_reply("Arrived at Lab A.")),
        (13000, lambda: win.set_state(State.LISTENING)),
        (14000, lambda: win.set_partial("show me the map")),
        (14700, lambda: (win.set_final("show me the map"),
                         win.set_state(State.SPEAKING),
                         win.set_reply("Of course, Master."))),
        (15500, lambda: win.show_map()),
        (19500, lambda: win.hide_map()),
        (20000, lambda: (win.set_address("Master"),
                         win.set_reply("Of course, Master."))),
        (22000, lambda: win.set_state(State.IDLE)),
    ]
    for delay, fn in script:
        QTimer.singleShot(delay, fn)
    sys.exit(app.exec())


if __name__ == "__main__":
    _preview()
