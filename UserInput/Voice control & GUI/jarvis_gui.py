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
import math
import os
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from PyQt6.QtCore import (
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
    QGraphicsBlurEffect,
    QGraphicsOpacityEffect,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


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
        radius = base_radius * (1.0 + 0.04 * breath * self._intensity)

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
        deg2 = -math.degrees(self._arc_phase * 0.8) + 30
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
    """Small animated green circle showing the wheelchair's estimated position.

    Positioned by MapOverlay.set_pose() whenever a new /amcl_pose message
    arrives.  Lives as a child widget of MapOverlay so it is automatically
    shown/hidden with the map.
    """

    _DOT_RADIUS = 5.0

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)   # 20 fps — enough for a subtle pulse
        self.hide()

    def _tick(self) -> None:
        self._phase = (self._phase + 0.12) % (2 * math.pi)
        self.update()

    def paintEvent(self, _ev) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pulse = 0.5 + 0.5 * math.sin(self._phase)
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        r  = self._DOT_RADIUS

        # Outer glow ring — fades in and out with the pulse.
        glow = QRadialGradient(QPointF(cx, cy), r * 2.4)
        glow_col = QColor(80, 220, 120)
        glow_col.setAlpha(int(90 * pulse))
        glow.setColorAt(0.0, glow_col)
        glow.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(glow))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), r * 2.4, r * 2.4)

        # Solid inner dot.
        painter.setBrush(QBrush(QColor(100, 240, 140)))
        painter.setPen(QPen(QColor(255, 255, 255, 200), 1.5))
        painter.drawEllipse(QPointF(cx, cy), r, r)
        painter.end()


# =============================================================================
# Map overlay
# =============================================================================
class MapOverlay(QWidget):
    """Full-screen translucent panel that fades in to show the LiDAR occupancy map."""

    closed = pyqtSignal()

    def __init__(self, map_yaml_path: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            "background-color: rgba(2, 8, 20, 235);"
        )
        self._map_yaml_path = map_yaml_path
        pixmap, ox, oy, res = load_lidar_map(map_yaml_path)
        self._pixmap = pixmap
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
        hint = QLabel("Say 'thank you' or press ESC to close")
        hint.setStyleSheet(
            f"color: {Theme.TEXT_DIM.name()}; "
            "font-family: 'Segoe UI'; font-size: 13px; letter-spacing: 2px;"
        )
        hint.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

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
        self._last_pose: Optional[tuple] = None   # (ros_x, ros_y) of last known pose

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

    def set_pose(self, ros_x: float, ros_y: float) -> None:
        """Position the pose dot from a ROS2 map-frame coordinate (metres).

        The conversion chain is:
          ROS map metres → source-image pixel → scaled-image pixel
          → label-local pixel (accounting for centre-alignment padding)
          → overlay pixel (accounting for label's position inside the overlay)

        Transform parameters are updated automatically when set_map() is called
        with a live OccupancyGrid.  Before that they fall back to the module
        constants (MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION).
        """
        self._last_pose = (ros_x, ros_y)
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

        # Refresh the displayed image.
        self._update_pixmap()

        # Re-apply the pose dot with the new transform.
        if self._last_pose is not None:
            self.set_pose(*self._last_pose)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

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
        elif address.lower() == "madame":
            label = "MADAME"
            color = QColor(255, 150, 200)
        else:
            label = "SIR"
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
    request_set_pose           = pyqtSignal(float, float)   # (ros_x, ros_y) metres
    request_set_map            = pyqtSignal(object)         # occupancy grid payload dict

    # User-driven signals out (e.g., ESC pressed, E-stop clicked).
    map_dismissed = pyqtSignal()
    quit_requested = pyqtSignal()
    estop_requested = pyqtSignal(bool)   # True = activate, False = release

    def __init__(self, map_yaml_path: Optional[str] = None):
        super().__init__()
        self.setWindowTitle("Jarvis — Smart Wheelchair Voice Console")
        self.resize(1280, 800)
        self.setMinimumSize(900, 600)

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

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(28, 22, 28, 0)
        top_bar.addLayout(title_box)
        top_bar.addStretch()
        top_bar.addWidget(self._chip)
        top_bar.addSpacing(10)
        top_bar.addWidget(self._ros2_chip)

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
        body.addSpacing(8)
        body.addLayout(btn_row)
        body.addSpacing(18)
        central.setLayout(body)

        # Map overlay covers central widget.
        self._map = MapOverlay(map_yaml_path, self)
        self._map.setGeometry(self.rect())
        self._map.closed.connect(self._on_map_overlay_closed)
        self._map.closed.connect(self.map_dismissed.emit)

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
        self.request_set_map.connect(
            self.set_map, Qt.ConnectionType.QueuedConnection
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

    def set_pose(self, ros_x: float, ros_y: float) -> None:
        """Update the 'you are here' dot on the map overlay.

        Safe to call at any time — the map overlay stores the pose and
        re-applies it whenever the map is next opened or the window resizes.
        """
        self._map.set_pose(ros_x, ros_y)

    def set_map(self, payload: object) -> None:
        """Replace the static EEMCS image with the live sensor map.

        Called automatically when an OccupancyGrid arrives on /map.
        Also updates the coordinate transform used by the pose dot.
        """
        self._map.set_map(payload)

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
        (5000,  lambda: win.set_reply("What do you want, sir?")),
        (6500,  lambda: win.set_state(State.LISTENING)),
        (7500,  lambda: win.set_partial("take me to lab a")),
        (8200,  lambda: (win.set_final("take me to lab a"),
                         win.set_state(State.SPEAKING),
                         win.set_reply("Navigating to Lab A, sir."))),
        (8500,  lambda: (win.set_ros2_connected(True),
                         win.set_robot_state("NAVIGATING"))),
        (11000, lambda: win.set_robot_state("ARRIVED")),
        (11500, lambda: win.set_reply("You have arrived at Lab A.")),
        (13000, lambda: win.set_state(State.LISTENING)),
        (14000, lambda: win.set_partial("show me the map")),
        (14700, lambda: (win.set_final("show me the map"),
                         win.set_state(State.SPEAKING),
                         win.set_reply("Of course, sir."))),
        (15500, lambda: win.show_map()),
        (19500, lambda: win.hide_map()),
        (20000, lambda: (win.set_address("madame"),
                         win.set_reply("Of course, madame."))),
        (22000, lambda: win.set_state(State.IDLE)),
    ]
    for delay, fn in script:
        QTimer.singleShot(delay, fn)
    sys.exit(app.exec())


if __name__ == "__main__":
    _preview()
