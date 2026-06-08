"""
main.py — Entry point: Jarvis GUI + Voice Control + ROS2 Bridge wired together.
================================================================================

Architecture
------------
                    ┌──────────────────────────────────────────┐
                    │              main thread                   │
                    │  PyQt6 event loop  ←  JarvisWindow        │
                    └───────────────┬──────────────────────────┘
                                    │  Qt signals (queued)
                    ┌───────────────▼──────────────────────────┐
                    │          QtVoiceObserver (bridge)          │
                    └───────────────┬──────────────────────────┘
                                    │  callbacks
           ┌────────────────────────▼──────────────────────────┐
           │   VoiceController  (daemon thread)                 │
           │   — Vosk ASR → keyword spotting → TTS              │
           │   — on_navigate(payload) → ROS2 bridge             │
           └──────────────────────┬────────────────────────────┘
                                  │  publish_nav_goal / publish_estop
           ┌──────────────────────▼────────────────────────────┐
           │   Ros2Bridge  (daemon thread)                       │
           │   — Native rclpy  OR  rosbridge WebSocket           │
           │   — on_status(data) → GUI robot state label         │
           │   — on_connection_changed(bool) → GUI status chip   │
           └───────────────────────────────────────────────────┘

Transport selection
-------------------
Set ROS2_TRANSPORT in the environment (or edit the constant below):

  ROS2_TRANSPORT=native    — rclpy (requires sourced ROS2 installation)
  ROS2_TRANSPORT=websocket — rosbridge WebSocket (requires websocket-client)

WebSocket host/port can be overridden with:
  ROSBRIDGE_HOST=192.168.1.100
  ROSBRIDGE_PORT=9090

Run
---
  # Native (single machine):
  source /opt/ros/jazzy/setup.bash
  python main.py

  # WebSocket (robot on separate PC):
  ROSBRIDGE_HOST=<robot-ip> ROS2_TRANSPORT=websocket python main.py

  Press F11 for full-screen, ESC to dismiss map / quit.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import time

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QApplication

from jarvis_gui import JarvisWindow, State
from ros2_bridge import Ros2BridgeBase, create_bridge
from voice_control import VoiceConfig, VoiceController, VoiceObserver

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ─── Configuration (override via environment variables) ────────────────────────
_TRANSPORT    = os.environ.get("ROS2_TRANSPORT",  "native")     # "native" | "websocket"
_ROS_HOST     = os.environ.get("ROSBRIDGE_HOST",  "localhost")
_ROS_PORT     = int(os.environ.get("ROSBRIDGE_PORT", "9090"))


# =============================================================================
# Bridge: VoiceObserver → Qt signals → GUI slots
# =============================================================================
class QtVoiceObserver(QObject, VoiceObserver):
    """Translates VoiceController events into Qt signals (thread-safe)."""

    state    = pyqtSignal(str)
    partial  = pyqtSignal(str)
    final    = pyqtSignal(str)
    reply    = pyqtSignal(str)
    address  = pyqtSignal(str)
    show_map     = pyqtSignal()
    hide_map     = pyqtSignal()
    navigate     = pyqtSignal(dict)
    create_point = pyqtSignal()

    def __init__(self) -> None:
        QObject.__init__(self)
        VoiceObserver.__init__(self)

    # VoiceObserver hooks — fire the matching Qt signal.
    def on_state(self, state: str) -> None:        self.state.emit(state)
    def on_partial(self, text: str) -> None:       self.partial.emit(text)
    def on_final(self, text: str) -> None:         self.final.emit(text)
    def on_reply(self, text: str) -> None:         self.reply.emit(text)
    def on_address(self, address: str) -> None:    self.address.emit(address)
    def on_show_map(self) -> None:                 self.show_map.emit()
    def on_hide_map(self) -> None:                 self.hide_map.emit()
    def on_create_point(self) -> None:             self.create_point.emit()
    def on_navigate(self, payload: dict) -> None:  self.navigate.emit(payload)


def _state_for_gui(s: str) -> State:
    """Map controller state strings → GUI State enum."""
    table = {
        "STANDING BY":     State.IDLE,
        "LISTENING":       State.LISTENING,
        "AWAITING COMMAND": State.AWAKE,
        "THINKING":        State.THINKING,
        "SPEAKING":        State.SPEAKING,
    }
    return table.get(s, State.IDLE)


def _robot_state_color_hint(state: str) -> str:
    """Return a short human-readable label for the given robot state."""
    labels = {
        "IDLE":       "",         # hide when idle
        "NAVIGATING": "NAVIGATING",
        "ARRIVED":    "ARRIVED",
        "OBSTACLE":   "OBSTACLE DETECTED",
        "ESTOP":      "E-STOP ACTIVE",
        "ERROR":      "ERROR",
    }
    return labels.get(state.upper(), state.upper())


# =============================================================================
# Main
# =============================================================================
def main() -> None:
    here     = os.path.dirname(os.path.abspath(__file__))
    # ── GUI ──────────────────────────────────────────────────────────────────
    app = QApplication(sys.argv)
    win = JarvisWindow()
    win.show()

    # ── Voice bridge (VoiceObserver → Qt signals) ─────────────────────────
    voice_bridge = QtVoiceObserver()
    voice_bridge.state.connect(lambda s: win.set_state(_state_for_gui(s)))
    voice_bridge.partial.connect(win.set_partial)
    voice_bridge.final.connect(win.set_final)
    voice_bridge.reply.connect(win.set_reply)
    voice_bridge.address.connect(win.set_address)
    voice_bridge.show_map.connect(win.show_map)
    voice_bridge.hide_map.connect(win.hide_map)
    voice_bridge.create_point.connect(win.request_enter_placement_mode)
    # navigate signal is wired to ROS2 below.

    # ── Voice controller ─────────────────────────────────────────────────────
    config = VoiceConfig(
        tts_provider="edge",
        edge_voice="en-GB-ThomasNeural",
        enable_gender_detection=True,
    )

    # Seed destinations from map_points.json so voice navigation works for
    # any location the operator placed via the map editor before this session.
    for pt in win.map_points:
        name_key = pt["name"].lower().strip()
        loc_id   = f"LOC_{name_key.upper().replace(' ', '_').replace('-', '_')}"
        config.destinations[name_key] = loc_id
        logger.info("Pre-seeded destination from map: '%s' → %s", name_key, loc_id)

    controller = VoiceController(config=config, observer=voice_bridge)

    audio_thread = threading.Thread(
        target=controller.run, name="VoiceControllerThread", daemon=True
    )
    audio_thread.start()

    # ── ROS2 bridge ──────────────────────────────────────────────────────────
    logger.info("Creating ROS2 bridge (transport=%s)", _TRANSPORT)
    ros2: Ros2BridgeBase = create_bridge(
        _TRANSPORT,
        host=_ROS_HOST,
        port=_ROS_PORT,
    )

    # Connection status → ROS2 chip in the GUI top bar.
    def _on_ros2_connection(connected: bool) -> None:
        win.request_set_ros2_connected.emit(connected)
        state_str = "LIVE" if connected else "OFFLINE"
        logger.info("ROS2 bridge %s.", state_str)

    # Status messages from the pathfinding team → robot state label in GUI.
    def _on_robot_status(data: dict) -> None:
        state   = data.get("state", "")
        message = data.get("message", "")
        label   = _robot_state_color_hint(state)
        win.request_set_robot_state.emit(label)
        # If the robot has arrived, also update the reply text.
        if state.upper() == "ARRIVED":
            dest = data.get("destination", "")
            reply_text = f"Arrived at {dest.replace('_', ' ').title()}." if dest else "Arrived."
            win.request_set_reply.emit(reply_text)
        elif state.upper() in ("OBSTACLE", "ERROR"):
            win.request_set_reply.emit(message or f"Navigation issue: {state}.")

    # Pose updates from the localisation team → "you are here" dot on the map.
    def _on_pose(x: float, y: float) -> None:
        win.request_set_pose.emit(x, y)

    # Live OccupancyGrid from the mapping team → replaces static EEMCS PNG.
    def _on_map(payload: dict) -> None:
        win.request_set_map.emit(payload)

    # Planned path from Nav2 /plan → path overlay drawn on the map.
    def _on_path(points: list) -> None:
        win.request_set_path.emit(points)

    ros2.on_connection_changed = _on_ros2_connection
    ros2.on_status             = _on_robot_status
    ros2.on_pose               = _on_pose
    ros2.on_map                = _on_map
    ros2.on_path               = _on_path

    # Map waypoint store — seeded from JSON at startup, updated as points are placed.
    # Maps lower-cased point name → (ros_x, ros_y, ros_yaw) for voice/click navigation.
    _map_points: dict[str, tuple[float, float, float]] = {
        pt["name"].lower(): (pt["ros_x"], pt["ros_y"], pt.get("ros_yaw", 0.0))
        for pt in win.map_points
    }

    def _on_location_placed(name: str, ros_x: float, ros_y: float) -> None:
        name_lower = name.lower().strip()
        # Yaw is 0 for points placed via the GUI (without the heading step) — fine for now.
        _map_points[name_lower] = (ros_x, ros_y, 0.0)
        loc_id = f"LOC_{name_lower.upper().replace(' ', '_').replace('-', '_')}"
        controller.add_destination(name_lower, loc_id)
        logger.info("Waypoint stored: '%s' → (%.3f, %.3f)", name, ros_x, ros_y)

    win.map_location_placed.connect(_on_location_placed)

    # Click-on-marker → navigate immediately (no voice confirmation needed —
    # the user explicitly tapped the destination).
    def _on_map_point_nav(name: str, ros_x: float, ros_y: float, ros_yaw: float = 0.0) -> None:
        payload = {
            "mode": "map_click",
            "destination": name,
            "ros_x": ros_x,
            "ros_y": ros_y,
            "ros_yaw": ros_yaw,
            "confidence": 1.0,
            "confirmed": True,
            "timestamp": time.time(),
        }
        logger.info("Map-click navigation: '%s' → (%.3f, %.3f) yaw=%.3f", name, ros_x, ros_y, ros_yaw)
        ros2.publish_nav_goal(payload)
        ros2.publish_goal_pose(ros_x, ros_y, ros_yaw)
        win.request_set_reply.emit(f"Navigating to {name}, sir.")
        win.request_set_robot_state.emit("NAVIGATING")

    win.map_point_nav_requested.connect(_on_map_point_nav)

    # Voice → ROS2: when the voice controller detects a confirmed destination,
    # enrich with ROS2 coords if the phrase matches a stored map waypoint.
    def _on_navigate(payload: dict) -> None:
        phrase = payload.get("destination_phrase", "").lower()
        ros_yaw = 0.0
        for name, (rx, ry, ryaw) in _map_points.items():
            if name in phrase or phrase in name:
                ros_yaw = ryaw
                payload = {**payload, "ros_x": rx, "ros_y": ry, "ros_yaw": ryaw, "mode": "map_point"}
                logger.info(
                    "Voice navigate matched map waypoint '%s': (%.3f, %.3f) yaw=%.3f",
                    name, rx, ry, ryaw,
                )
                break
        logger.info("Navigate command: %s", payload)
        ros2.publish_nav_goal(payload)
        if "ros_x" in payload:
            ros2.publish_goal_pose(payload["ros_x"], payload["ros_y"], ros_yaw)

    voice_bridge.navigate.connect(_on_navigate)

    # GUI E-stop button → ROS2 estop publisher.
    def _on_estop(active: bool) -> None:
        ros2.publish_estop(active)

    win.estop_requested.connect(_on_estop)

    ros2.start()

    # ── Shutdown ─────────────────────────────────────────────────────────────
    def _on_quit() -> None:
        logger.info("Shutting down …")
        controller.stop()
        ros2.stop()

    win.quit_requested.connect(_on_quit)
    app.aboutToQuit.connect(_on_quit)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
