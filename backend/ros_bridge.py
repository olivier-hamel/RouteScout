from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import roslibpy


def _truthy(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _truthy_default(value: str | None, default: bool) -> bool:
    if value is None:
        return default
    return _truthy(value)


@dataclass(frozen=True)
class RosBridgeConfig:
    host: str
    port: int
    topic: str
    message_type: str
    map_topic: str
    map_message_type: str
    odom_topic: str
    odom_message_type: str
    tf_topic: str
    tf_message_type: str
    tf_static_topic: str
    tf_static_message_type: str
    cmd_vel_topic: str
    cmd_vel_message_type: str
    cmd_vel_stream_enabled: bool
    cmd_vel_stream_hz: float
    cmd_vel_idle_timeout_s: float
    subscribe_map: bool
    subscribe_odom: bool
    subscribe_tf: bool
    subscribe_tf_static: bool
    activity_timeout_s: float

    @staticmethod
    def from_env() -> "RosBridgeConfig":
        host = os.getenv("ROSBRIDGE_HOST", "127.0.0.1")
        port = int(os.getenv("ROSBRIDGE_PORT", "9090"))
        topic = os.getenv("ROSBRIDGE_TOPIC", "/chatter")
        # ROS2 rosbridge generally uses pkg/msg/Type naming.
        message_type = os.getenv("ROSBRIDGE_MESSAGE_TYPE", "std_msgs/msg/String")

        map_topic = os.getenv("ROSBRIDGE_MAP_TOPIC", "/map")
        map_message_type = os.getenv(
            "ROSBRIDGE_MAP_MESSAGE_TYPE", "nav_msgs/msg/OccupancyGrid"
        )
        odom_topic = os.getenv("ROSBRIDGE_ODOM_TOPIC", "/odom")
        odom_message_type = os.getenv(
            "ROSBRIDGE_ODOM_MESSAGE_TYPE", "nav_msgs/msg/Odometry"
        )
        tf_topic = os.getenv("ROSBRIDGE_TF_TOPIC", "/tf")
        tf_message_type = os.getenv("ROSBRIDGE_TF_MESSAGE_TYPE", "tf2_msgs/msg/TFMessage")
        tf_static_topic = os.getenv("ROSBRIDGE_TF_STATIC_TOPIC", "/tf_static")
        tf_static_message_type = os.getenv(
            "ROSBRIDGE_TF_STATIC_MESSAGE_TYPE", "tf2_msgs/msg/TFMessage"
        )

        subscribe_map = _truthy_default(os.getenv("ROSBRIDGE_SUBSCRIBE_MAP"), True)
        subscribe_odom = _truthy_default(os.getenv("ROSBRIDGE_SUBSCRIBE_ODOM"), True)
        # TF is high-rate and can be very heavy over rosbridge.
        # Default it off to keep teleop responsive; enable explicitly when needed.
        subscribe_tf = _truthy_default(os.getenv("ROSBRIDGE_SUBSCRIBE_TF"), False)
        subscribe_tf_static = _truthy_default(os.getenv("ROSBRIDGE_SUBSCRIBE_TF_STATIC"), False)

        cmd_vel_topic = os.getenv("ROSBRIDGE_CMD_VEL_TOPIC", "/cmd_vel")
        cmd_vel_message_type = os.getenv(
            "ROSBRIDGE_CMD_VEL_MESSAGE_TYPE", "geometry_msgs/msg/Twist"
        )

        # Default to enabled so the frontend's press-and-hold teleop works out of the box.
        # You can still disable it explicitly with ROSBRIDGE_CMD_VEL_STREAM=false.
        cmd_vel_stream_enabled = _truthy_default(
            os.getenv("ROSBRIDGE_CMD_VEL_STREAM"), True
        )
        cmd_vel_stream_hz = float(os.getenv("ROSBRIDGE_CMD_VEL_STREAM_HZ", "15"))
        cmd_vel_idle_timeout_s = float(os.getenv("ROSBRIDGE_CMD_VEL_IDLE_TIMEOUT_S", "0.25"))

        activity_timeout_s = float(os.getenv("ROSBRIDGE_ACTIVITY_TIMEOUT_S", "1.0"))
        return RosBridgeConfig(
            host=host,
            port=port,
            topic=topic,
            message_type=message_type,
            map_topic=map_topic,
            map_message_type=map_message_type,
            odom_topic=odom_topic,
            odom_message_type=odom_message_type,
            tf_topic=tf_topic,
            tf_message_type=tf_message_type,
            tf_static_topic=tf_static_topic,
            tf_static_message_type=tf_static_message_type,
            cmd_vel_topic=cmd_vel_topic,
            cmd_vel_message_type=cmd_vel_message_type,
            cmd_vel_stream_enabled=cmd_vel_stream_enabled,
            cmd_vel_stream_hz=cmd_vel_stream_hz,
            cmd_vel_idle_timeout_s=cmd_vel_idle_timeout_s,
            subscribe_map=subscribe_map,
            subscribe_odom=subscribe_odom,
            subscribe_tf=subscribe_tf,
            subscribe_tf_static=subscribe_tf_static,
            activity_timeout_s=activity_timeout_s,
        )


@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float

    def compose(self, other: "Transform2D") -> "Transform2D":
        """Return self ∘ other (i.e. parent<-mid composed with mid<-child)."""
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        x = self.x + cy * other.x - sy * other.y
        y = self.y + sy * other.x + cy * other.y
        return Transform2D(x=x, y=y, yaw=self.yaw + other.yaw)

    def inverse(self) -> "Transform2D":
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        # Inverse of p_parent = R p_child + t is p_child = R^T (p_parent - t)
        x = -(cy * self.x + sy * self.y)
        y = -(-sy * self.x + cy * self.y)
        return Transform2D(x=x, y=y, yaw=-self.yaw)


def _normalize_frame(frame: str | None) -> str | None:
    if frame is None:
        return None
    f = frame.strip()
    if not f:
        return None
    return f[1:] if f.startswith("/") else f


def _yaw_from_quat(q: dict[str, Any] | None) -> float:
    if not q:
        return 0.0
    try:
        x = float(q.get("x", 0.0))
        y = float(q.get("y", 0.0))
        z = float(q.get("z", 0.0))
        w = float(q.get("w", 1.0))
    except (TypeError, ValueError):
        return 0.0

    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosBridgeClient:
    def __init__(self, config: RosBridgeConfig):
        self._config = config
        self._lock = threading.Lock()

        self._ros: roslibpy.Ros | None = None
        self._topic: roslibpy.Topic | None = None

        self._map_topic: roslibpy.Topic | None = None
        self._odom_topic: roslibpy.Topic | None = None
        self._tf_topic: roslibpy.Topic | None = None
        self._tf_static_topic: roslibpy.Topic | None = None

        self._cmd_vel_sub: roslibpy.Topic | None = None
        self._cmd_vel_pub: roslibpy.Topic | None = None

        self._last_message: dict[str, Any] | None = None
        self._last_received_at: str | None = None

        self._last_map: dict[str, Any] | None = None
        self._last_map_received_at: str | None = None

        self._last_odom: dict[str, Any] | None = None
        self._last_odom_received_at: str | None = None

        self._last_tf: dict[str, Any] | None = None
        self._last_tf_received_at: str | None = None

        self._last_tf_static: dict[str, Any] | None = None
        self._last_tf_static_received_at: str | None = None

        # 2D TF buffer: stores both directions for quick lookup.
        self._tf_edges: dict[tuple[str, str], Transform2D] = {}

        self._last_cmd_vel_message: dict[str, Any] | None = None
        self._last_cmd_vel_received_at: str | None = None
        self._last_cmd_vel_published_at: str | None = None
        self._last_cmd_vel_published_message: dict[str, Any] | None = None
        self._last_cmd_vel_published_packet: dict[str, Any] | None = None

        self._cmd_vel_command: tuple[float, float] = (0.0, 0.0)
        self._cmd_vel_commanded_at: datetime | None = None
        self._cmd_vel_latched = False
        self._cmd_vel_stream_thread: threading.Thread | None = None
        self._cmd_vel_stream_stop = threading.Event()
        self._connected_once = False

        self._reconnect_thread: threading.Thread | None = None
        self._reconnect_stop = threading.Event()
        self._last_reconnect_attempt_monotonic = 0.0

    @property
    def config(self) -> RosBridgeConfig:
        return self._config

    def connect_and_subscribe(self) -> None:
        if self._ros is not None:
            return

        ros = roslibpy.Ros(host=self._config.host, port=self._config.port)
        topic = roslibpy.Topic(ros, self._config.topic, self._config.message_type)

        map_topic = (
            roslibpy.Topic(ros, self._config.map_topic, self._config.map_message_type)
            if self._config.subscribe_map
            else None
        )
        odom_topic = (
            roslibpy.Topic(ros, self._config.odom_topic, self._config.odom_message_type)
            if self._config.subscribe_odom
            else None
        )
        tf_topic = (
            roslibpy.Topic(ros, self._config.tf_topic, self._config.tf_message_type)
            if self._config.subscribe_tf
            else None
        )
        tf_static_topic = (
            roslibpy.Topic(ros, self._config.tf_static_topic, self._config.tf_static_message_type)
            if self._config.subscribe_tf_static
            else None
        )

        cmd_vel_sub = roslibpy.Topic(
            ros, self._config.cmd_vel_topic, self._config.cmd_vel_message_type
        )
        cmd_vel_pub = roslibpy.Topic(
            ros, self._config.cmd_vel_topic, self._config.cmd_vel_message_type
        )

        def _on_message(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_message = message
                self._last_received_at = datetime.now(timezone.utc).isoformat()

        def _on_map(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_map = message
                self._last_map_received_at = datetime.now(timezone.utc).isoformat()

        def _on_odom(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_odom = message
                self._last_odom_received_at = datetime.now(timezone.utc).isoformat()

        def _update_tf_from_message(message: dict[str, Any]) -> None:
            transforms = message.get("transforms")
            if not isinstance(transforms, list):
                return

            edges: dict[tuple[str, str], Transform2D] = {}
            for t in transforms:
                if not isinstance(t, dict):
                    continue
                header = t.get("header") or {}
                parent = _normalize_frame((header.get("frame_id") if isinstance(header, dict) else None))
                child = _normalize_frame(t.get("child_frame_id"))
                if not parent or not child:
                    continue

                tr = t.get("transform") or {}
                if not isinstance(tr, dict):
                    continue
                trans = tr.get("translation") or {}
                rot = tr.get("rotation") or {}
                if not isinstance(trans, dict) or not isinstance(rot, dict):
                    continue
                try:
                    x = float(trans.get("x", 0.0))
                    y = float(trans.get("y", 0.0))
                except (TypeError, ValueError):
                    continue
                yaw = _yaw_from_quat(rot)
                tf2d = Transform2D(x=x, y=y, yaw=yaw)

                edges[(parent, child)] = tf2d
                edges[(child, parent)] = tf2d.inverse()

            if not edges:
                return

            with self._lock:
                self._tf_edges.update(edges)

        def _on_tf(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_tf = message
                self._last_tf_received_at = datetime.now(timezone.utc).isoformat()
            _update_tf_from_message(message)

        def _on_tf_static(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_tf_static = message
                self._last_tf_static_received_at = datetime.now(timezone.utc).isoformat()
            _update_tf_from_message(message)

        def _on_cmd_vel(message: dict[str, Any]) -> None:
            with self._lock:
                self._last_cmd_vel_message = message
                self._last_cmd_vel_received_at = datetime.now(timezone.utc).isoformat()

        def _on_ready() -> None:
            self._connected_once = True
            topic.subscribe(_on_message)
            if map_topic is not None:
                map_topic.subscribe(_on_map)
            if odom_topic is not None:
                odom_topic.subscribe(_on_odom)
            if tf_topic is not None:
                tf_topic.subscribe(_on_tf)
            if tf_static_topic is not None:
                tf_static_topic.subscribe(_on_tf_static)
            cmd_vel_sub.subscribe(_on_cmd_vel)

        ros.on_ready(_on_ready, run_in_thread=True)

        self._ros = ros
        self._topic = topic

        self._map_topic = map_topic
        self._odom_topic = odom_topic
        self._tf_topic = tf_topic
        self._tf_static_topic = tf_static_topic

        self._cmd_vel_sub = cmd_vel_sub
        self._cmd_vel_pub = cmd_vel_pub

        # Non-blocking in roslibpy; uses an internal thread/loop.
        ros.run()

        # Keep the connection alive: rosbridge/ws can drop under load.
        self._start_reconnect_watchdog()

        # Optional cmd_vel keepalive stream (useful for base controllers expecting 10-20Hz).
        if self._config.cmd_vel_stream_enabled:
            self._start_cmd_vel_stream()

    def _start_reconnect_watchdog(self) -> None:
        if self._reconnect_thread is not None and self._reconnect_thread.is_alive():
            return

        self._reconnect_stop.clear()

        def _loop() -> None:
            # Try fast, but with backoff to avoid thrashing.
            check_period_s = 1.0
            min_retry_s = 1.0
            max_retry_s = 10.0
            backoff_s = min_retry_s

            while not self._reconnect_stop.is_set():
                time.sleep(check_period_s)
                if self._reconnect_stop.is_set():
                    break

                ros = self._ros
                if ros is None:
                    continue

                if self.is_connected():
                    backoff_s = min_retry_s
                    continue

                # If we never connected successfully, don't spam reconnect attempts.
                if not self._connected_once:
                    continue

                now_m = time.monotonic()
                if now_m - self._last_reconnect_attempt_monotonic < backoff_s:
                    continue

                self._last_reconnect_attempt_monotonic = now_m

                try:
                    self._reconnect()
                    backoff_s = min_retry_s
                except Exception:
                    backoff_s = min(max_retry_s, backoff_s * 1.5)

        t = threading.Thread(target=_loop, name="rosbridge-reconnect", daemon=True)
        self._reconnect_thread = t
        t.start()

    def _reconnect(self) -> None:
        # Tear down existing connection and create a fresh one.
        with self._lock:
            ros = self._ros
            topic = self._topic
            map_topic = self._map_topic
            odom_topic = self._odom_topic
            tf_topic = self._tf_topic
            tf_static_topic = self._tf_static_topic
            cmd_vel_sub = self._cmd_vel_sub
            cmd_vel_pub = self._cmd_vel_pub

            self._ros = None
            self._topic = None
            self._map_topic = None
            self._odom_topic = None
            self._tf_topic = None
            self._tf_static_topic = None
            self._cmd_vel_sub = None
            self._cmd_vel_pub = None

        # Best-effort unsub/terminate outside the lock.
        try:
            if topic is not None:
                topic.unsubscribe()
            if map_topic is not None:
                map_topic.unsubscribe()
            if odom_topic is not None:
                odom_topic.unsubscribe()
            if tf_topic is not None:
                tf_topic.unsubscribe()
            if tf_static_topic is not None:
                tf_static_topic.unsubscribe()
            if cmd_vel_sub is not None:
                cmd_vel_sub.unsubscribe()
        finally:
            if ros is not None:
                try:
                    ros.terminate()
                except Exception:
                    pass

        # Recreate and subscribe.
        self.connect_and_subscribe()

    def _start_cmd_vel_stream(self) -> None:
        if self._cmd_vel_stream_thread is not None and self._cmd_vel_stream_thread.is_alive():
            return

        # Clear any previous stop signal.
        self._cmd_vel_stream_stop.clear()

        def _loop() -> None:
            hz = max(1.0, float(self._config.cmd_vel_stream_hz))
            period_s = 1.0 / hz
            idle_timeout_s = max(0.0, float(self._config.cmd_vel_idle_timeout_s))

            # Light backoff while disconnected.
            disconnected_sleep_s = min(0.5, period_s)

            while not self._cmd_vel_stream_stop.is_set():
                if not self.is_connected() or self._cmd_vel_pub is None:
                    time.sleep(disconnected_sleep_s)
                    continue

                now = datetime.now(timezone.utc)
                with self._lock:
                    linear_x, angular_z = self._cmd_vel_command
                    commanded_at = self._cmd_vel_commanded_at
                    latched = self._cmd_vel_latched

                if (not latched) and (commanded_at is None or (now - commanded_at).total_seconds() > idle_timeout_s):
                    linear_x, angular_z = 0.0, 0.0

                try:
                    # Publish directly without updating the "command" timestamp.
                    self._publish_cmd_vel_no_command_update(linear_x, angular_z)
                except Exception:
                    # Keep running; transient rosbridge disconnects are expected.
                    pass

                # Sleep in small chunks so stop signal is responsive.
                deadline = time.monotonic() + period_s
                while not self._cmd_vel_stream_stop.is_set():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(0.05, remaining))

        t = threading.Thread(target=_loop, name="cmd_vel_stream", daemon=True)
        self._cmd_vel_stream_thread = t
        t.start()

    def is_connected(self) -> bool:
        ros = self._ros
        if ros is None:
            return False
        return bool(getattr(ros, "is_connected", False))

    def status(self) -> dict[str, Any]:
        cmd_vel_status = self.cmd_vel_status()
        tf_status = self.tf_status()
        map_status = self.map_status()
        odom_status = self.odom_status()
        return {
            "enabled": True,
            "host": self._config.host,
            "port": self._config.port,
            "topic": self._config.topic,
            "message_type": self._config.message_type,
            "is_connected": self.is_connected(),
            "connected_once": self._connected_once,
            "last_received_at": self._last_received_at,
            "last_message": self._last_message,
            "map": {
                "enabled": self._config.subscribe_map,
                "topic": self._config.map_topic,
                "message_type": self._config.map_message_type,
                "last_received_at": map_status["last_received_at"],
                "has_message": map_status["has_message"],
            },
            "odom": {
                "enabled": self._config.subscribe_odom,
                "topic": self._config.odom_topic,
                "message_type": self._config.odom_message_type,
                "last_received_at": odom_status["last_received_at"],
                "has_message": odom_status["has_message"],
            },
            "tf": {
                "enabled": self._config.subscribe_tf,
                "static_enabled": self._config.subscribe_tf_static,
                "topic": self._config.tf_topic,
                "static_topic": self._config.tf_static_topic,
                "last_received_at": tf_status["last_received_at"],
                "last_static_received_at": tf_status["last_static_received_at"],
                "edge_count": tf_status["edge_count"],
            },
            "cmd_vel": {
                "topic": self._config.cmd_vel_topic,
                "message_type": self._config.cmd_vel_message_type,
                "stream_enabled": self._config.cmd_vel_stream_enabled,
                "stream_hz": self._config.cmd_vel_stream_hz,
                "idle_timeout_s": self._config.cmd_vel_idle_timeout_s,
                "latched": cmd_vel_status.get("latched"),
                "active": cmd_vel_status["active"],
                "activity_timeout_s": self._config.activity_timeout_s,
                "last_received_at": cmd_vel_status["last_received_at"],
                "last_published_at": cmd_vel_status["last_published_at"],
                "last_message": cmd_vel_status["last_message"],
            },
        }

    def latest(self) -> dict[str, Any]:
        with self._lock:
            return {
                "topic": self._config.topic,
                "message_type": self._config.message_type,
                "received_at": self._last_received_at,
                "message": self._last_message,
            }

    def latest_map(self) -> dict[str, Any]:
        with self._lock:
            return {
                "topic": self._config.map_topic,
                "message_type": self._config.map_message_type,
                "received_at": self._last_map_received_at,
                "message": self._last_map,
            }

    def latest_odom(self) -> dict[str, Any]:
        with self._lock:
            return {
                "topic": self._config.odom_topic,
                "message_type": self._config.odom_message_type,
                "received_at": self._last_odom_received_at,
                "message": self._last_odom,
            }

    def latest_tf(self) -> dict[str, Any]:
        with self._lock:
            return {
                "topic": self._config.tf_topic,
                "message_type": self._config.tf_message_type,
                "received_at": self._last_tf_received_at,
                "message": self._last_tf,
            }

    def latest_tf_static(self) -> dict[str, Any]:
        with self._lock:
            return {
                "topic": self._config.tf_static_topic,
                "message_type": self._config.tf_static_message_type,
                "received_at": self._last_tf_static_received_at,
                "message": self._last_tf_static,
            }

    def map_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "has_message": self._last_map is not None,
                "last_received_at": self._last_map_received_at,
            }

    def odom_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "has_message": self._last_odom is not None,
                "last_received_at": self._last_odom_received_at,
            }

    def tf_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "last_received_at": self._last_tf_received_at,
                "last_static_received_at": self._last_tf_static_received_at,
                "edge_count": len(self._tf_edges),
            }

    def _lookup_tf_2d(self, parent: str, child: str) -> Transform2D | None:
        """Find Transform2D mapping child->parent (i.e. T_parent_child)."""
        parent_n = _normalize_frame(parent)
        child_n = _normalize_frame(child)
        if not parent_n or not child_n:
            return None
        if parent_n == child_n:
            return Transform2D(0.0, 0.0, 0.0)

        with self._lock:
            edges = dict(self._tf_edges)

        # BFS over frames, composing transforms.
        from collections import deque

        q: deque[tuple[str, Transform2D]] = deque()
        q.append((child_n, Transform2D(0.0, 0.0, 0.0)))
        visited: set[str] = set()

        while q:
            frame, acc = q.popleft()
            if frame in visited:
                continue
            visited.add(frame)
            if frame == parent_n:
                return acc

            for (p, c), tf2d in edges.items():
                if c != frame:
                    continue
                if p in visited:
                    continue
                q.append((p, tf2d.compose(acc)))

        return None

    def robot_pose_2d(self) -> dict[str, Any]:
        """Return robot pose in map frame when TF is available; otherwise in odom frame."""
        with self._lock:
            odom = self._last_odom
            map_msg = self._last_map

        if not odom or not isinstance(odom, dict):
            return {
                "ok": False,
                "reason": "no_odom",
            }

        header = odom.get("header") or {}
        odom_frame = _normalize_frame(header.get("frame_id") if isinstance(header, dict) else None) or "odom"
        base_frame = _normalize_frame(odom.get("child_frame_id")) or "base_link"

        pose = odom.get("pose") or {}
        pose_pose = pose.get("pose") if isinstance(pose, dict) else None
        if not isinstance(pose_pose, dict):
            return {"ok": False, "reason": "odom_missing_pose"}

        pos = pose_pose.get("position") or {}
        ori = pose_pose.get("orientation") or {}
        if not isinstance(pos, dict) or not isinstance(ori, dict):
            return {"ok": False, "reason": "odom_bad_pose"}

        try:
            ox = float(pos.get("x", 0.0))
            oy = float(pos.get("y", 0.0))
        except (TypeError, ValueError):
            return {"ok": False, "reason": "odom_bad_position"}

        oyaw = _yaw_from_quat(ori)
        t_odom_base = Transform2D(x=ox, y=oy, yaw=oyaw)

        map_frame = "map"
        if isinstance(map_msg, dict):
            mh = map_msg.get("header") or {}
            mf = _normalize_frame(mh.get("frame_id") if isinstance(mh, dict) else None)
            if mf:
                map_frame = mf

        t_map_odom = self._lookup_tf_2d(map_frame, odom_frame)
        if t_map_odom is None:
            return {
                "ok": True,
                "tf_available": False,
                "frame": odom_frame,
                "base_frame": base_frame,
                "x": t_odom_base.x,
                "y": t_odom_base.y,
                "yaw": t_odom_base.yaw,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "note": "TF unavailable; showing odom pose in odom frame",
            }

        t_map_base = t_map_odom.compose(t_odom_base)
        return {
            "ok": True,
            "tf_available": True,
            "frame": map_frame,
            "base_frame": base_frame,
            "x": t_map_base.x,
            "y": t_map_base.y,
            "yaw": t_map_base.yaw,
            "map_frame": map_frame,
            "odom_frame": odom_frame,
        }

    def publish(self, message: dict[str, Any]) -> None:
        if self._topic is None or self._ros is None:
            raise RuntimeError("ROS bridge is not initialized")
        if not self.is_connected():
            raise RuntimeError("ROS bridge is not connected")

        self._topic.publish(roslibpy.Message(message))

    def _publish_cmd_vel_no_command_update(self, linear_x: float, angular_z: float) -> None:
        if self._cmd_vel_pub is None or self._ros is None:
            raise RuntimeError("ROS bridge is not initialized")
        if not self.is_connected():
            raise RuntimeError("ROS bridge is not connected")

        msg: dict[str, Any] = {
            "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
        }

        packet: dict[str, Any] = {
            "op": "publish",
            "topic": self._config.cmd_vel_topic,
            "type": self._config.cmd_vel_message_type,
            "msg": msg,
        }

        self._cmd_vel_pub.publish(roslibpy.Message(msg))
        with self._lock:
            self._last_cmd_vel_published_at = datetime.now(timezone.utc).isoformat()
            self._last_cmd_vel_published_message = msg
            self._last_cmd_vel_published_packet = packet

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        # Record the desired command for the streaming loop.
        with self._lock:
            self._cmd_vel_command = (float(linear_x), float(angular_z))
            self._cmd_vel_commanded_at = datetime.now(timezone.utc)

        # Also publish immediately for responsiveness.
        self._publish_cmd_vel_no_command_update(linear_x, angular_z)

    def cmd_vel_status(self) -> dict[str, Any]:
        with self._lock:
            last_received_at = self._last_cmd_vel_received_at
            last_published_at = self._last_cmd_vel_published_at
            last_message = self._last_cmd_vel_message
            last_published_message = self._last_cmd_vel_published_message
            last_published_packet = self._last_cmd_vel_published_packet
            latched = self._cmd_vel_latched

        def _parse(ts: str | None) -> datetime | None:
            if not ts:
                return None
            try:
                return datetime.fromisoformat(ts)
            except ValueError:
                return None

        now = datetime.now(timezone.utc)
        timeout = self._config.activity_timeout_s
        last_rx = _parse(last_received_at)
        last_tx = _parse(last_published_at)

        def _recent(dt: datetime | None) -> bool:
            if dt is None:
                return False
            return (now - dt).total_seconds() <= timeout

        active = bool(self.is_connected() and (_recent(last_rx) or _recent(last_tx)))
        return {
            "active": active,
            "last_received_at": last_received_at,
            "last_published_at": last_published_at,
            "last_message": last_message,
            "last_published_message": last_published_message,
            "last_published_packet": last_published_packet,
            "latched": latched,
        }

    def latch_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        with self._lock:
            self._cmd_vel_latched = True
            self._cmd_vel_command = (float(linear_x), float(angular_z))
            self._cmd_vel_commanded_at = datetime.now(timezone.utc)
        self._publish_cmd_vel_no_command_update(linear_x, angular_z)

    def release_cmd_vel(self) -> None:
        with self._lock:
            self._cmd_vel_latched = False
            self._cmd_vel_command = (0.0, 0.0)
            self._cmd_vel_commanded_at = datetime.now(timezone.utc)
        try:
            self._publish_cmd_vel_no_command_update(0.0, 0.0)
        except Exception:
            pass

    def close(self) -> None:
        self._reconnect_stop.set()
        self._cmd_vel_stream_stop.set()
        try:
            if self._topic is not None:
                self._topic.unsubscribe()
            if self._map_topic is not None:
                self._map_topic.unsubscribe()
            if self._odom_topic is not None:
                self._odom_topic.unsubscribe()
            if self._tf_topic is not None:
                self._tf_topic.unsubscribe()
            if self._tf_static_topic is not None:
                self._tf_static_topic.unsubscribe()
            if self._cmd_vel_sub is not None:
                self._cmd_vel_sub.unsubscribe()
        finally:
            if self._ros is not None:
                self._ros.terminate()


def is_rosbridge_enabled() -> bool:
    return _truthy(os.getenv("ENABLE_ROSBRIDGE"))
