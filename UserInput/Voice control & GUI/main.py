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

import argparse
import json
import logging
import os
import socket as _socket
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

# ── MEMORY PROFILING — remove this block after testing (main.py ~line 73) ──
try:
    import psutil as _psutil
    _mem_proc = _psutil.Process()
    def _mem_snap(label: str) -> None:
        rss = _mem_proc.memory_info().rss / (1024 * 1024)
        print(f"[MEM_STAGE] {label}: {rss:.0f} MB", flush=True)
except ImportError:
    def _mem_snap(label: str) -> None:
        pass
# ── END MEMORY PROFILING ────────────────────────────────────────────────────


# ─── Configuration (override via environment variables) ────────────────────────
_TRANSPORT    = os.environ.get("ROS2_TRANSPORT",  "native")     # "native" | "websocket"
_ROS_HOST     = os.environ.get("ROSBRIDGE_HOST",  "localhost")
_ROS_PORT     = int(os.environ.get("ROSBRIDGE_PORT", "9090"))

# ─── Test socket broadcaster (active only when --test-port is passed) ─────────
_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument("--test-port", type=int, default=0)
_parser.add_argument("--no-fast-nav", action="store_true", default=False)
_parsed_args      = _parser.parse_known_args()[0]
_TEST_PORT: int   = _parsed_args.test_port
_NO_FAST_NAV: bool = _parsed_args.no_fast_nav

_test_conn:  _socket.socket | None = None
_test_lock:  threading.Lock         = threading.Lock()


def _test_server_thread(port: int) -> None:
    """Accept one test-runner connection and store it in _test_conn."""
    global _test_conn
    srv = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
    srv.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(60)
    try:
        conn, addr = srv.accept()
        logger.info("Test runner connected from %s", addr)
        with _test_lock:
            _test_conn = conn
    except _socket.timeout:
        logger.warning("Test runner did not connect within 60 s — test port idle.")
    finally:
        srv.close()


def _test_send(data: dict) -> None:
    """Send one JSON line to the test runner (no-op if not connected)."""
    with _test_lock:
        if _test_conn is None:
            return
        try:
            _test_conn.sendall((json.dumps(data) + "\n").encode())
        except Exception:
            pass


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
    dev_command  = pyqtSignal(str)

    def __init__(self) -> None:
        QObject.__init__(self)
        VoiceObserver.__init__(self)
        self._voice_state: str = ""

    def is_confirming(self) -> bool:
        """Return True while the voice controller is waiting for a verbal yes/no."""
        return self._voice_state == "CONFIRMING"

    # VoiceObserver hooks — fire the matching Qt signal.
    def on_state(self, state: str) -> None:
        self._voice_state = state
        self.state.emit(state)
    def on_partial(self, text: str) -> None:             self.partial.emit(text)
    def on_final(self, text: str) -> None:               self.final.emit(text)
    def on_reply(self, text: str) -> None:               self.reply.emit(text)
    def on_address(self, address: str) -> None:          self.address.emit(address)
    def on_show_map(self) -> None:                       self.show_map.emit()
    def on_hide_map(self) -> None:                       self.hide_map.emit()
    def on_create_point(self) -> None:                   self.create_point.emit()
    def on_navigate(self, payload: dict) -> None:        self.navigate.emit(payload)
    def on_dev_command(self, command: str) -> None:      self.dev_command.emit(command)


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
    here = os.path.dirname(os.path.abspath(__file__))

    # Start test socket server before anything else so the test runner can
    # connect while the rest of the system is still initialising.
    if _TEST_PORT:
        threading.Thread(
            target=_test_server_thread, args=(_TEST_PORT,),
            name="TestSocketServer", daemon=True,
        ).start()
        logger.info("Test socket server started on port %d", _TEST_PORT)

    # ── GUI ──────────────────────────────────────────────────────────────────
    app = QApplication(sys.argv)
    win = JarvisWindow()
    win.show()
    _mem_snap("Qt + GUI")                            # MEMORY PROFILING

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
    voice_bridge.dev_command.connect(win.request_dev_command)
    # navigate signal is wired to ROS2 below.

    # ── Voice controller ─────────────────────────────────────────────────────
    config = VoiceConfig(
        tts_provider="edge",
        edge_voice="en-GB-ThomasNeural",
        fast_navigate=not _NO_FAST_NAV,
    )

    # Seed destinations from map_points.json so voice navigation works for
    # any location the operator placed via the map editor before this session.
    for pt in win.map_points:
        name_key = pt["name"].lower().strip()
        loc_id   = f"LOC_{name_key.upper().replace(' ', '_').replace('-', '_')}"
        config.destinations[name_key] = loc_id
        logger.info("Pre-seeded destination from map: '%s' → %s", name_key, loc_id)

    controller = VoiceController(config=config, observer=voice_bridge)
    _mem_snap("+ VoiceController constructed (pre-Vosk thread)")  # MEMORY PROFILING

    audio_thread = threading.Thread(
        target=controller.run, name="VoiceControllerThread", daemon=True
    )
    audio_thread.start()

    # Delayed snap: Vosk model finishes loading ~10s after thread start.  MEMORY PROFILING
    def _snap_post_vosk():                                               # MEMORY PROFILING
        time.sleep(12)                                                   # MEMORY PROFILING
        _mem_snap("+ Vosk fully loaded (12 s after thread start)")       # MEMORY PROFILING
    threading.Thread(target=_snap_post_vosk, daemon=True).start()       # MEMORY PROFILING

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

    # ── Localisation-gated goal dispatch ──────────────────────────────────────
    # The pathfinding team can only plan a correct path once AMCL is confident
    # about where the chair actually is.  Every nav_goal / goal_pose is gated on
    # the latest /amcl_pose covariance metric (the worst of the X, Y and yaw
    # variances).  A goal that arrives while the pose is still loose is *held*,
    # not dropped, and released automatically the instant localisation tightens
    # below the threshold — so the chair waits for a good fix, then drives.
    _COV_GATE = 0.5
    _loc = {"cov": float("inf")}            # latest covariance metric (inf = unknown)
    _pending: dict | None = None            # held goal awaiting a confident fix
    _pending_lock = threading.Lock()

    def _do_publish(payload: dict, ros_yaw: float) -> None:
        ros2.publish_nav_goal(payload)
        if "ros_x" in payload:
            ros2.publish_goal_pose(payload["ros_x"], payload["ros_y"], ros_yaw)

    def _dispatch_goal(payload: dict, ros_yaw: float, name: str) -> None:
        """Confirmed goal: start the nav stack, show the map, then publish.

        The moment a destination is confirmed we fire the same start_navigation
        command as the Developer-mode button (idempotent — it no-ops if the
        stack is already up) and bring up the map so the user sees where they
        are headed.  The ROS2 nav_goal / goal_pose itself is still gated on a
        confident AMCL fix: it goes out now if covariance is already good, else
        it is held and released automatically once the pose tightens below the
        threshold.
        """
        nonlocal _pending
        win.request_dev_command.emit("start_navigation")
        win.request_show_map.emit()
        with _pending_lock:
            cov = _loc["cov"]
            if cov < _COV_GATE:
                _do_publish(payload, ros_yaw)
                logger.info("Goal '%s' sent — AMCL covariance %.3f < %.2f.",
                            name, cov, _COV_GATE)
                win.request_set_reply.emit(f"Navigating to {name}, Master.")
                win.request_set_robot_state.emit("NAVIGATING")
            else:
                _pending = {"payload": payload, "yaw": ros_yaw, "name": name}
                logger.warning(
                    "Goal '%s' held — AMCL covariance %.3f >= %.2f, waiting for a tighter fix.",
                    name, cov, _COV_GATE,
                )
                win.request_set_reply.emit(
                    f"Holding {name} until I am sure of our position, Master."
                )

    # Pose updates from the localisation team → green pose marker + gate state.
    def _on_pose(x: float, y: float, yaw: float = 0.0, cov: float = float("inf")) -> None:
        nonlocal _pending
        win.request_set_pose.emit(x, y, yaw)
        win.request_set_loc_cov.emit(cov)          # dev-mode localisation quality chip
        released = None
        with _pending_lock:
            _loc["cov"] = cov
            if _pending is not None and cov < _COV_GATE:
                released, _pending = _pending, None
        if released is not None:
            _do_publish(released["payload"], released["yaw"])
            logger.info("Held goal '%s' released — AMCL covariance %.3f < %.2f.",
                        released["name"], cov, _COV_GATE)
            win.request_set_reply.emit(
                f"Position confirmed. Navigating to {released['name']}, Master."
            )
            win.request_set_robot_state.emit("NAVIGATING")

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
        if voice_bridge.is_confirming():
            logger.info(
                "Map-click to '%s' rejected: voice controller is in CONFIRMING state.", name
            )
            return
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
        _dispatch_goal(payload, ros_yaw, name)

    win.map_point_nav_requested.connect(_on_map_point_nav)

    # Voice → ROS2: when the voice controller detects a confirmed destination,
    # enrich with ROS2 coords if the phrase matches a stored map waypoint.
    def _on_navigate(payload: dict) -> None:
        _test_send({"type": "nav", "payload": payload, "t": time.time()})
        phrase = payload.get("destination_phrase", "").lower()
        ros_yaw = 0.0
        matched_name = None
        for name, (rx, ry, ryaw) in _map_points.items():
            if name in phrase or phrase in name:
                ros_yaw = ryaw
                payload = {**payload, "ros_x": rx, "ros_y": ry, "ros_yaw": ryaw, "mode": "map_point"}
                matched_name = name
                logger.info(
                    "Voice navigate matched map waypoint '%s': (%.3f, %.3f) yaw=%.3f",
                    name, rx, ry, ryaw,
                )
                break
        logger.info("Navigate command: %s", payload)
        # Final safety gate: only ever drive to a marker that exists on the map.
        # If the phrase did not resolve to a stored waypoint, refuse rather than
        # sending the pathfinding team a goal with no coordinates.
        if not matched_name:
            logger.warning("Navigate rejected — '%s' is not a map marker.", phrase)
            win.request_set_reply.emit("That location is not on the map, Master.")
            return
        # Highlight the destination marker red BEFORE sending to ROS2 so the
        # GUI is always up-to-date regardless of when the map overlay is open.
        win.request_set_selected_destination.emit(matched_name)
        _dispatch_goal(payload, ros_yaw, matched_name)

    voice_bridge.navigate.connect(_on_navigate)

    # Broadcast speech-final, reply, and state events so the test runner
    # can measure latency and verify destinations.
    #
    # IMPORTANT: these callbacks are invoked directly on the voice thread
    # (inside VoiceObserver hooks) so the timestamps reflect actual
    # processing time, not Qt event-loop delay.  The previous approach
    # routed through Qt signals (QueuedConnection) and the main thread's
    # animation timers added ~1 s of phantom latency to every measurement.
    if _TEST_PORT:
        _orig_on_final = voice_bridge.on_final
        def _test_on_final(text: str) -> None:
            _test_send({"type": "final", "text": text, "t": time.time()})
            _orig_on_final(text)
        voice_bridge.on_final = _test_on_final

        _orig_on_reply = voice_bridge.on_reply
        def _test_on_reply(text: str) -> None:
            _test_send({"type": "reply", "text": text, "t": time.time()})
            _orig_on_reply(text)
        voice_bridge.on_reply = _test_on_reply

        _orig_on_state = voice_bridge.on_state
        def _test_on_state(state: str) -> None:
            _test_send({"type": "state", "state": state, "t": time.time()})
            _orig_on_state(state)
        voice_bridge.on_state = _test_on_state

    # GUI E-stop button → ROS2 estop publisher.
    def _on_estop(active: bool) -> None:
        ros2.publish_estop(active)

    win.estop_requested.connect(_on_estop)

    ros2.start()
    _mem_snap("+ ROS2 bridge started")               # MEMORY PROFILING

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
