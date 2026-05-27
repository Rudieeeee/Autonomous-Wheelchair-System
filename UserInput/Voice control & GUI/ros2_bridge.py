"""
ros2_bridge.py — ROS2 Integration Bridge for the Smart Wheelchair Voice Subsystem
==================================================================================

Provides two transport implementations for sending/receiving ROS2 messages,
both behind a single abstract interface (Ros2BridgeBase):

  ┌─────────────────────┬──────────────────────────────────────────────────────┐
  │ Ros2NativeNode      │ Uses rclpy directly. Both GUI and robot share the     │
  │                     │ same machine / same ROS2 installation. Simplest       │
  │                     │ option for development on a single laptop.            │
  ├─────────────────────┼──────────────────────────────────────────────────────┤
  │ Ros2WebsocketBridge │ Uses rosbridge_server over WebSocket (port 9090).     │
  │                     │ GUI laptop and wheelchair onboard PC are separate     │
  │                     │ machines on the same network. Preferred for the       │
  │                     │ actual demo where the laptop is off the robot.        │
  └─────────────────────┴──────────────────────────────────────────────────────┘

ROS2 Topics
-----------
  PUBLISH   /wheelchair/nav_goal    std_msgs/msg/String   (JSON payload)
  PUBLISH   /wheelchair/estop       std_msgs/msg/Bool     (True = stop)
  SUBSCRIBE /wheelchair/status      std_msgs/msg/String   (JSON payload)
  SUBSCRIBE /amcl_pose              geometry_msgs/msg/PoseWithCovarianceStamped
  SUBSCRIBE /map                    nav_msgs/msg/OccupancyGrid  (latched)

Navigation goal JSON schema
---------------------------
  {
    "mode":        "voice" | "map_click" | "button",
    "destination": "<location_key>",          # e.g. "lab_a", "cafeteria"
    "confidence":  <float 0.0–1.0>,           # ASR confidence score
    "confirmed":   <bool>,                    # True after undo window expires
    "timestamp":   <float>                    # time.time()
  }

Status JSON schema (from pathfinding team)
------------------------------------------
  {
    "state":       "IDLE" | "NAVIGATING" | "ARRIVED" | "OBSTACLE" | "ESTOP" | "ERROR",
    "destination": "<location_key>",
    "progress":    <float 0.0–1.0>,           # 0 = just started, 1 = arrived
    "message":     "<human readable string>"
  }

Quick-start (native rclpy)
--------------------------
  # Source ROS2 first:  source /opt/ros/jazzy/setup.bash
  from ros2_bridge import Ros2NativeNode
  bridge = Ros2NativeNode()
  bridge.on_status = lambda data: print("Robot says:", data)
  bridge.on_connection_changed = lambda ok: print("ROS2 connected:", ok)
  bridge.start()
  bridge.publish_nav_goal({"mode": "voice", "destination": "lab_a",
                           "confidence": 0.98, "confirmed": True,
                           "timestamp": time.time()})
  bridge.publish_estop(True)   # stop
  bridge.publish_estop(False)  # release
  bridge.stop()

Quick-start (rosbridge WebSocket)
----------------------------------
  # On the wheelchair PC:
  #   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  from ros2_bridge import Ros2WebsocketBridge
  bridge = Ros2WebsocketBridge(host="192.168.1.100", port=9090)
  bridge.on_status = lambda data: print("Robot says:", data)
  bridge.start()
  ...
"""

from __future__ import annotations

import json
import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# ─── ROS2 topic / message type constants ──────────────────────────────────────
TOPIC_NAV_GOAL  = "/wheelchair/nav_goal"
TOPIC_ESTOP     = "/wheelchair/estop"
TOPIC_STATUS    = "/wheelchair/status"
TOPIC_AMCL_POSE = "/amcl_pose"          # localisation team — robot pose in map frame
TOPIC_MAP       = "/map"                # mapping/localisation team — OccupancyGrid

MSG_STRING    = "std_msgs/msg/String"
MSG_BOOL      = "std_msgs/msg/Bool"
MSG_POSE_COV  = "geometry_msgs/msg/PoseWithCovarianceStamped"
MSG_OCC_GRID  = "nav_msgs/msg/OccupancyGrid"

# Robot state constants (as published by the pathfinding team).
class RobotState:
    IDLE       = "IDLE"
    NAVIGATING = "NAVIGATING"
    ARRIVED    = "ARRIVED"
    OBSTACLE   = "OBSTACLE"
    ESTOP      = "ESTOP"
    ERROR      = "ERROR"


# =============================================================================
# Abstract base
# =============================================================================
class Ros2BridgeBase(ABC):
    """
    Common interface for all ROS2 transport implementations.

    Subclass and override the abstract methods.  Set the callback attributes
    before calling start().

    Attributes
    ----------
    on_status : Callable[[dict], None]
        Called (on an arbitrary thread) whenever a new status message arrives
        from the pathfinding team.  The argument is the parsed JSON dict.
    on_connection_changed : Callable[[bool], None]
        Called whenever the connection state changes.  True = connected.
    """

    def __init__(self) -> None:
        self.on_status: Callable[[dict], None] = lambda _data: None
        self.on_connection_changed: Callable[[bool], None] = lambda _ok: None
        # Called whenever AMCL publishes a new estimated robot pose.
        # Arguments are (x_metres, y_metres) in the ROS2 map frame.
        self.on_pose: Callable[[float, float], None] = lambda _x, _y: None
        # Called whenever a new OccupancyGrid map arrives.
        # Argument is a dict: {width, height, data (RGB bytes), origin_x,
        # origin_y, resolution} — ready to pass to MapOverlay.set_map().
        self.on_map: Callable[[dict], None] = lambda _m: None
        self._connected = False

    # ------------------------------------------------------------------
    # Connection state
    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._connected

    def _set_connected(self, value: bool) -> None:
        if value != self._connected:
            self._connected = value
            try:
                self.on_connection_changed(value)
            except Exception:
                logger.exception("on_connection_changed callback raised")

    # ------------------------------------------------------------------
    # Abstract API
    # ------------------------------------------------------------------
    @abstractmethod
    def start(self) -> None:
        """Start the bridge in a background daemon thread (non-blocking)."""

    @abstractmethod
    def stop(self) -> None:
        """Gracefully shut down the bridge and its background thread."""

    @abstractmethod
    def publish_nav_goal(self, payload: dict) -> None:
        """
        Send a navigation goal to the pathfinding team.

        Parameters
        ----------
        payload : dict
            Must contain: mode, destination, confidence, confirmed, timestamp.
            See module docstring for the full schema.
        """

    @abstractmethod
    def publish_estop(self, active: bool) -> None:
        """
        Send an emergency stop command.

        Parameters
        ----------
        active : bool
            True  → stop the wheelchair immediately.
            False → release the emergency stop.
        """

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def build_nav_payload(
        self,
        destination: str,
        *,
        mode: str = "voice",
        confidence: float = 1.0,
        confirmed: bool = True,
    ) -> dict:
        """Convenience: build a well-formed nav_goal payload dict."""
        return {
            "mode":        mode,
            "destination": destination,
            "confidence":  round(float(confidence), 4),
            "confirmed":   bool(confirmed),
            "timestamp":   time.time(),
        }


# =============================================================================
# Occupancy grid → RGB conversion  (runs in the bridge thread, not the GUI)
# =============================================================================
def _occupancy_to_rgb(data, width: int, height: int) -> bytes:
    """Convert a flat OccupancyGrid data array to packed RGB bytes.

    Cell values:  -1 = unknown, 0 = free, 1-100 = occupied (% probability).
    The ROS grid is stored row-major from the bottom-left corner, so we flip
    vertically so that row 0 of the output image is the top of the map.

    Uses numpy when available (fast, <5 ms for a 400×400 map).
    Falls back to pure Python otherwise (~1–2 s for the same size).
    """
    try:
        import numpy as np
        arr = np.array(data, dtype=np.int16).reshape(height, width)
        rgb = np.zeros((height, width, 3), dtype=np.uint8)

        unknown  = arr < 0
        free     = arr == 0
        occupied = arr > 0

        # Colour scheme: dark navy background, dim blue-grey for free space,
        # bright accent blue for walls — matches the Jarvis GUI palette.
        rgb[unknown]  = [10,  20,  40]   # very dark navy
        rgb[free]     = [28,  48,  80]   # dim blue (driveable floor)

        if occupied.any():
            # Scale occupancy 1-100 → brightness 80-255
            occ_vals = arr[occupied].astype(np.float32) / 100.0
            rgb[occupied, 0] = np.clip(40  + occ_vals * 110, 0, 255).astype(np.uint8)
            rgb[occupied, 1] = np.clip(100 + occ_vals * 110, 0, 255).astype(np.uint8)
            rgb[occupied, 2] = np.clip(160 + occ_vals * 95,  0, 255).astype(np.uint8)

        # ROS stores rows bottom-up; flip so image row 0 is the map top.
        rgb = np.flipud(rgb)
        return rgb.tobytes()

    except ImportError:
        # Pure Python fallback — noticeably slower but requires no extra deps.
        out = bytearray(width * height * 3)
        for ros_idx, val in enumerate(data):
            ros_row = ros_idx // width
            col     = ros_idx % width
            img_row = height - 1 - ros_row      # flip Y
            i = (img_row * width + col) * 3
            if val < 0:                          # unknown
                out[i], out[i+1], out[i+2] = 10, 20, 40
            elif val == 0:                       # free
                out[i], out[i+1], out[i+2] = 28, 48, 80
            else:                               # occupied
                s = int(val * 1.7)
                out[i] = min(255, 40  + s)
                out[i+1] = min(255, 100 + s)
                out[i+2] = min(255, 160 + s)
        return bytes(out)


# =============================================================================
# Native rclpy implementation
# =============================================================================
class Ros2NativeNode(Ros2BridgeBase):
    """
    Communicates with ROS2 via rclpy directly.

    Requirements
    ------------
    - A sourced ROS2 Jazzy/Iron environment before launching Python:
          source /opt/ros/jazzy/setup.bash   # Linux
          (Windows: call C:\\opt\\ros\\jazzy\\setup.bat)
    - The std_msgs package (included in any desktop install).

    The node spins in a dedicated daemon thread so it never blocks the
    PyQt6 event loop.
    """

    def __init__(self, node_name: str = "jarvis_voice_node") -> None:
        super().__init__()
        self._node_name = node_name
        self._node       = None
        self._executor   = None
        self._thread: Optional[threading.Thread] = None
        self._pub_nav    = None
        self._pub_estop  = None
        self._sub_status = None
        self._rclpy      = None   # cached import

    # ------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._spin, daemon=True, name="ROS2NativeThread"
        )
        self._thread.start()

    # ------------------------------------------------------------------
    def _spin(self) -> None:
        # Lazy-import rclpy so the rest of the app works even without ROS2.
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid
            from std_msgs.msg import Bool, String
        except ImportError as exc:
            logger.error(
                "rclpy import failed — is ROS2 Jazzy sourced?\n  %s", exc
            )
            self._set_connected(False)
            return

        try:
            rclpy.init()
        except Exception as exc:
            logger.error("rclpy.init() failed: %s", exc)
            self._set_connected(False)
            return

        self._rclpy = rclpy
        self._node = rclpy.create_node(self._node_name)

        # Publishers
        self._pub_nav   = self._node.create_publisher(String, TOPIC_NAV_GOAL, qos_profile=10)
        self._pub_estop = self._node.create_publisher(Bool,   TOPIC_ESTOP,    qos_profile=10)

        # Subscribers
        self._sub_status = self._node.create_subscription(
            String, TOPIC_STATUS, self._on_status_msg, qos_profile=10
        )
        self._sub_pose = self._node.create_subscription(
            PoseWithCovarianceStamped, TOPIC_AMCL_POSE, self._on_amcl_pose, qos_profile=10
        )
        # /map is a latched topic — use transient_local so we receive the last
        # published map immediately on connect, even if SLAM finished earlier.
        _map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._sub_map = self._node.create_subscription(
            OccupancyGrid, TOPIC_MAP, self._on_map_msg, qos_profile=_map_qos
        )

        self._set_connected(True)
        logger.info("ROS2 native node '%s' started and spinning.", self._node_name)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        try:
            self._executor.spin()
        except Exception as exc:
            logger.error("ROS2 executor error: %s", exc)
        finally:
            self._set_connected(False)
            try:
                self._node.destroy_node()
                rclpy.try_shutdown()
            except Exception:
                pass
            logger.info("ROS2 native node shut down.")

    # ------------------------------------------------------------------
    def stop(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception as exc:
                logger.warning("ROS2 executor shutdown error: %s", exc)

    # ------------------------------------------------------------------
    def publish_nav_goal(self, payload: dict) -> None:
        if not self._connected or self._pub_nav is None:
            logger.warning("publish_nav_goal skipped — ROS2 not connected.")
            return
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps(payload)
            self._pub_nav.publish(msg)
            logger.info("→ nav_goal published: destination=%s mode=%s confirmed=%s",
                        payload.get("destination"), payload.get("mode"), payload.get("confirmed"))
        except Exception as exc:
            logger.error("publish_nav_goal error: %s", exc)

    # ------------------------------------------------------------------
    def publish_estop(self, active: bool) -> None:
        if not self._connected or self._pub_estop is None:
            logger.warning("publish_estop skipped — ROS2 not connected.")
            return
        try:
            from std_msgs.msg import Bool
            msg = Bool()
            msg.data = bool(active)
            self._pub_estop.publish(msg)
            logger.info("→ E-STOP published: active=%s", active)
        except Exception as exc:
            logger.error("publish_estop error: %s", exc)

    # ------------------------------------------------------------------
    def _on_status_msg(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"state": msg.data, "raw": True}
        try:
            self.on_status(data)
        except Exception:
            logger.exception("on_status callback raised")

    def _on_amcl_pose(self, msg) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        try:
            self.on_pose(x, y)
        except Exception:
            logger.exception("on_pose callback raised")

    def _on_map_msg(self, msg) -> None:
        info     = msg.info
        width    = info.width
        height   = info.height
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution
        logger.info(
            "Map received: %dx%d cells, %.4f m/px, origin=(%.2f, %.2f)",
            width, height, resolution, origin_x, origin_y,
        )
        rgb_bytes = _occupancy_to_rgb(msg.data, width, height)
        payload = {
            "width":      width,
            "height":     height,
            "data":       rgb_bytes,
            "origin_x":   origin_x,
            "origin_y":   origin_y,
            "resolution": resolution,
        }
        try:
            self.on_map(payload)
        except Exception:
            logger.exception("on_map callback raised")


# =============================================================================
# Rosbridge WebSocket implementation
# =============================================================================
class Ros2WebsocketBridge(Ros2BridgeBase):
    """
    Communicates with ROS2 via rosbridge_server over a WebSocket.

    Use this when:
    - The ROS2 stack runs on the wheelchair's onboard PC.
    - The Jarvis GUI laptop connects to the wheelchair over Wi-Fi.

    Requirements (on the wheelchair PC)
    ------------------------------------
      sudo apt install ros-jazzy-rosbridge-server
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml
      # WebSocket now listens on port 9090

    Requirements (on the GUI laptop)
    ---------------------------------
      pip install websocket-client

    The bridge reconnects automatically if the connection drops.
    """

    RETRY_DELAY_S = 3.0

    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        super().__init__()
        self._url  = f"ws://{host}:{port}"
        self._ws   = None
        self._ws_lock      = threading.Lock()
        self._stop_event   = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._connect_loop, daemon=True, name="RosbridgeWSThread"
        )
        self._thread.start()

    # ------------------------------------------------------------------
    def _connect_loop(self) -> None:
        """Outer reconnect loop — keeps retrying until stop() is called."""
        try:
            import websocket as _ws_mod
        except ImportError:
            logger.error(
                "websocket-client not installed.\n"
                "  Run:  pip install websocket-client"
            )
            return

        while not self._stop_event.is_set():
            logger.info("Connecting to rosbridge at %s …", self._url)
            try:
                ws = _ws_mod.WebSocketApp(
                    self._url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                with self._ws_lock:
                    self._ws = ws
                ws.run_forever(ping_interval=20, ping_timeout=10)
            except Exception as exc:
                logger.error("Rosbridge connection exception: %s", exc)
            finally:
                self._set_connected(False)
                with self._ws_lock:
                    self._ws = None

            if not self._stop_event.is_set():
                logger.info(
                    "Rosbridge disconnected — retrying in %.1fs …",
                    self.RETRY_DELAY_S,
                )
                self._stop_event.wait(self.RETRY_DELAY_S)

    # ------------------------------------------------------------------
    def _on_open(self, ws) -> None:
        logger.info("Rosbridge WebSocket connected.")
        # Advertise our publishers so the server knows the message types.
        self._send(ws, {"op": "advertise", "topic": TOPIC_NAV_GOAL, "type": MSG_STRING})
        self._send(ws, {"op": "advertise", "topic": TOPIC_ESTOP,    "type": MSG_BOOL})
        # Subscribe to incoming topics.
        self._send(ws, {"op": "subscribe", "topic": TOPIC_STATUS,    "type": MSG_STRING})
        self._send(ws, {"op": "subscribe", "topic": TOPIC_AMCL_POSE, "type": MSG_POSE_COV})
        # rosbridge replays the last latched /map message automatically on subscribe.
        self._send(ws, {"op": "subscribe", "topic": TOPIC_MAP,       "type": MSG_OCC_GRID})
        self._set_connected(True)

    def _on_message(self, ws, raw: str) -> None:
        try:
            envelope = json.loads(raw)
        except json.JSONDecodeError as exc:
            logger.warning("Rosbridge non-JSON message: %s", exc)
            return
        topic = envelope.get("topic")
        op    = envelope.get("op")

        if op == "publish" and topic == TOPIC_STATUS:
            inner = envelope.get("msg", {}).get("data", "{}")
            try:
                data = json.loads(inner) if isinstance(inner, str) else inner
            except json.JSONDecodeError:
                data = {"state": inner, "raw": True}
            try:
                self.on_status(data)
            except Exception:
                logger.exception("on_status callback raised")

        elif op == "publish" and topic == TOPIC_AMCL_POSE:
            # rosbridge serialises PoseWithCovarianceStamped as nested dicts.
            pos = (
                envelope.get("msg", {})
                        .get("pose", {})
                        .get("pose", {})
                        .get("position", {})
            )
            x = float(pos.get("x", 0.0))
            y = float(pos.get("y", 0.0))
            try:
                self.on_pose(x, y)
            except Exception:
                logger.exception("on_pose callback raised")

        elif op == "publish" and topic == TOPIC_MAP:
            msg  = envelope.get("msg", {})
            info = msg.get("info", {})
            width      = int(info.get("width",      0))
            height     = int(info.get("height",     0))
            resolution = float(info.get("resolution", 0.05))
            origin     = info.get("origin", {}).get("position", {})
            origin_x   = float(origin.get("x", 0.0))
            origin_y   = float(origin.get("y", 0.0))
            data       = msg.get("data", [])
            if width > 0 and height > 0 and data:
                logger.info(
                    "Map received (WS): %dx%d cells, %.4f m/px, origin=(%.2f, %.2f)",
                    width, height, resolution, origin_x, origin_y,
                )
                rgb_bytes = _occupancy_to_rgb(data, width, height)
                payload = {
                    "width":      width,
                    "height":     height,
                    "data":       rgb_bytes,
                    "origin_x":   origin_x,
                    "origin_y":   origin_y,
                    "resolution": resolution,
                }
                try:
                    self.on_map(payload)
                except Exception:
                    logger.exception("on_map callback raised")

    def _on_error(self, ws, error) -> None:
        logger.error("Rosbridge WS error: %s", error)

    def _on_close(self, ws, close_status_code, close_msg) -> None:
        logger.info(
            "Rosbridge WS closed (code=%s, msg=%s).", close_status_code, close_msg
        )
        self._set_connected(False)

    # ------------------------------------------------------------------
    def stop(self) -> None:
        self._stop_event.set()
        with self._ws_lock:
            if self._ws is not None:
                try:
                    self._ws.close()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    def publish_nav_goal(self, payload: dict) -> None:
        self._publish_string_topic(TOPIC_NAV_GOAL, json.dumps(payload))
        logger.info("→ nav_goal published: destination=%s mode=%s confirmed=%s",
                    payload.get("destination"), payload.get("mode"), payload.get("confirmed"))

    def publish_estop(self, active: bool) -> None:
        with self._ws_lock:
            ws = self._ws
        if ws is None or not self._connected:
            logger.warning("publish_estop skipped — rosbridge not connected.")
            return
        msg = {"op": "publish", "topic": TOPIC_ESTOP, "msg": {"data": bool(active)}}
        self._send(ws, msg)
        logger.info("→ E-STOP published: active=%s", active)

    # ------------------------------------------------------------------
    def _publish_string_topic(self, topic: str, data: str) -> None:
        with self._ws_lock:
            ws = self._ws
        if ws is None or not self._connected:
            logger.warning("Publish skipped (not connected): topic=%s", topic)
            return
        msg = {"op": "publish", "topic": topic, "msg": {"data": data}}
        self._send(ws, msg)

    @staticmethod
    def _send(ws, obj: dict) -> None:
        try:
            ws.send(json.dumps(obj))
        except Exception as exc:
            logger.error("Rosbridge send error: %s", exc)


# =============================================================================
# Factory helper
# =============================================================================
def create_bridge(
    transport: str = "native",
    *,
    node_name: str = "jarvis_voice_node",
    host: str = "localhost",
    port: int = 9090,
) -> Ros2BridgeBase:
    """
    Factory function — create a bridge by name.

    Parameters
    ----------
    transport : "native" | "websocket"
    node_name : ROS2 node name (native only)
    host      : rosbridge host (websocket only)
    port      : rosbridge port (websocket only, default 9090)

    Example
    -------
    bridge = create_bridge("native")
    bridge = create_bridge("websocket", host="192.168.1.100")
    """
    if transport == "native":
        return Ros2NativeNode(node_name=node_name)
    elif transport == "websocket":
        return Ros2WebsocketBridge(host=host, port=port)
    else:
        raise ValueError(f"Unknown transport: {transport!r}. Use 'native' or 'websocket'.")
