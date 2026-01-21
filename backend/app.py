from __future__ import annotations

import argparse
import os
import threading
from datetime import datetime, timezone

from flask import Flask, jsonify, request
from flask_cors import CORS

from ros_bridge import RosBridgeClient, RosBridgeConfig, is_rosbridge_enabled


def create_app() -> Flask:
    app = Flask(__name__)

    # In dev we typically access the backend via the Vite proxy, so CORS isn't strictly
    # required. Keeping it enabled makes direct API calls from other origins easier.
    CORS(app)

    ros_bridge: RosBridgeClient | None = None
    if is_rosbridge_enabled():
        ros_bridge = RosBridgeClient(RosBridgeConfig.from_env())
        try:
            ros_bridge.connect_and_subscribe()
        except Exception:
            # Keep the Flask app running even if ROS isn't available.
            pass

    app.extensions["ros_bridge"] = ros_bridge

    def _truthy(value: str | None) -> bool:
        if value is None:
            return False
        return value.strip().lower() in {"1", "true", "yes", "on"}

    def _truthy_default(value: str | None, default: bool) -> bool:
        if value is None:
            return default
        return _truthy(value)

    # Enabled by default; set ENABLE_POTHOLES=false to disable.
    potholes_enabled = _truthy_default(os.getenv("ENABLE_POTHOLES"), True)

    potholes_lock = threading.Lock()
    potholes: list[dict] = []
    potholes_next_seq = 1
    potholes_max = 500

    def _append_pothole(item: dict, source: str) -> dict:
        nonlocal potholes_next_seq

        record = dict(item)
        record["seq"] = potholes_next_seq
        record["received_at"] = datetime.now(timezone.utc).isoformat()
        record["source"] = source
        potholes_next_seq += 1
        potholes.append(record)
        if len(potholes) > potholes_max:
            del potholes[: max(0, len(potholes) - potholes_max)]
        return record

    @app.get("/api/health")
    def health():
        return jsonify(
            {
                "status": "ok",
                "service": "flask-backend",
                "time": datetime.now(timezone.utc).isoformat(),
            }
        )

    @app.get("/api/ros/status")
    def ros_status():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"enabled": False})
        return jsonify(bridge.status())

    @app.get("/api/ros/latest")
    def ros_latest():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400
        return jsonify(bridge.latest())

    @app.get("/api/ros/map")
    def ros_map_latest():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400

        since = request.args.get("since")
        packet = bridge.latest_map()
        received_at = packet.get("received_at")

        # If the client already has the latest map, avoid sending a massive JSON payload.
        # We return 204 (no content) with a hint header.
        if since and received_at and str(since) == str(received_at):
            resp = app.response_class(status=204)
            resp.headers["X-Map-Received-At"] = str(received_at)
            return resp

        return jsonify(packet)

    @app.get("/api/ros/odom")
    def ros_odom_latest():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400
        return jsonify(bridge.latest_odom())

    @app.get("/api/ros/tf")
    def ros_tf_latest():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400
        return jsonify(
            {
                "tf": bridge.latest_tf(),
                "tf_static": bridge.latest_tf_static(),
                "status": bridge.tf_status(),
            }
        )

    @app.get("/api/ros/robot_pose_2d")
    def ros_robot_pose_2d():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400
        return jsonify(bridge.robot_pose_2d())

    @app.get("/api/ros/cmd_vel/status")
    def ros_cmd_vel_status():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"enabled": False, "topic": "/cmd_vel", "active": False})

        status = bridge.cmd_vel_status()
        app.logger.debug(
            "cmd_vel status active=%s connected=%s last_rx=%s last_tx=%s",
            status.get("active"),
            bridge.is_connected(),
            status.get("last_received_at"),
            status.get("last_published_at"),
        )
        return jsonify(
            {
                "enabled": True,
                "is_connected": bridge.is_connected(),
                "topic": bridge.config.cmd_vel_topic,
                "message_type": bridge.config.cmd_vel_message_type,
                "active": status["active"],
                "last_received_at": status["last_received_at"],
                "last_published_at": status["last_published_at"],
                "last_published_message": status.get("last_published_message"),
                "last_published_packet": status.get("last_published_packet"),
            }
        )

    @app.post("/api/ros/cmd_vel")
    def ros_cmd_vel_publish():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400

        body = request.get_json(silent=True) or {}
        linear_x = body.get("linear_x")
        angular_z = body.get("angular_z")
        if linear_x is None or angular_z is None:
            return (
                jsonify(
                    {
                        "error": "Expected JSON body with {linear_x: number, angular_z: number}",
                    }
                ),
                400,
            )

        try:
            linear_x_f = float(linear_x)
            angular_z_f = float(angular_z)
            payload = {
                "op": "publish",
                "topic": bridge.config.cmd_vel_topic,
                "type": bridge.config.cmd_vel_message_type,
                "msg": {
                    "linear": {"x": linear_x_f, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": angular_z_f},
                },
            }
            app.logger.info(
                "Publishing /cmd_vel payload=%s",
                payload,
            )
            bridge.publish_cmd_vel(linear_x_f, angular_z_f)
        except Exception as exc:
            app.logger.exception("Failed to publish cmd_vel")
            return jsonify({"error": str(exc)}), 500

        return jsonify({"ok": True})

    @app.post("/api/ros/cmd_vel/hold")
    def ros_cmd_vel_hold():
        """Latch a cmd_vel command until released.

        Intended for press-and-hold UI controls to avoid per-tick HTTP calls.
        """
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400

        body = request.get_json(silent=True) or {}
        linear_x = body.get("linear_x")
        angular_z = body.get("angular_z")
        if linear_x is None or angular_z is None:
            return (
                jsonify({"error": "Expected JSON body with {linear_x: number, angular_z: number}"}),
                400,
            )

        try:
            bridge.latch_cmd_vel(float(linear_x), float(angular_z))
        except Exception as exc:
            app.logger.exception("Failed to latch cmd_vel")
            return jsonify({"error": str(exc)}), 500

        return jsonify({"ok": True})

    @app.post("/api/ros/cmd_vel/release")
    def ros_cmd_vel_release():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400

        try:
            bridge.release_cmd_vel()
        except Exception as exc:
            app.logger.exception("Failed to release cmd_vel")
            return jsonify({"error": str(exc)}), 500

        return jsonify({"ok": True})

    @app.post("/api/ros/publish")
    def ros_publish():
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        if bridge is None:
            return jsonify({"error": "ROS bridge is disabled"}), 400

        body = request.get_json(silent=True) or {}
        message = body.get("message")
        if not isinstance(message, dict):
            return jsonify({"error": "Expected JSON body with {message: {...}}"}), 400

        try:
            bridge.publish(message)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500

        return jsonify({"ok": True})

    @app.post("/api/potholes")
    def potholes_ingest():
        if not potholes_enabled:
            return jsonify({"error": "potholes disabled (set ENABLE_POTHOLES=true)"}), 404

        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            body = {}

        source = body.get("source")
        source = str(source) if source is not None else "unknown"

        items = body.get("potholes")
        if items is None:
            single = body.get("pothole")
            if isinstance(single, dict):
                items = [single]

        if not isinstance(items, list):
            return (
                jsonify(
                    {
                        "error": "Expected JSON body with {potholes: [{...}]} (or {pothole: {...}})",
                    }
                ),
                400,
            )

        # Snapshot robot pose at ingest time so the UI can place markers reliably.
        bridge: RosBridgeClient | None = app.extensions.get("ros_bridge")
        pose_snapshot: dict | None = None
        if bridge is not None:
            try:
                pose = bridge.robot_pose_2d()
                if isinstance(pose, dict) and pose.get("ok") is True:
                    pose_snapshot = {
                        "x": pose.get("x"),
                        "y": pose.get("y"),
                        "yaw": pose.get("yaw"),
                        "frame": pose.get("frame"),
                        "tf_available": pose.get("tf_available"),
                        "map_frame": pose.get("map_frame"),
                        "odom_frame": pose.get("odom_frame"),
                    }
            except Exception:
                pose_snapshot = None

        ingested = 0
        with potholes_lock:
            for item in items:
                if not isinstance(item, dict):
                    continue
                if pose_snapshot is not None:
                    item = dict(item)
                    item["robot_pose"] = pose_snapshot
                _append_pothole(item, source)
                ingested += 1

        return jsonify({"ok": True, "ingested": ingested})

    @app.get("/api/potholes")
    def potholes_list():
        if not potholes_enabled:
            return jsonify({"error": "potholes disabled (set ENABLE_POTHOLES=true)"}), 404

        limit_raw = request.args.get("limit")
        since_raw = request.args.get("since_seq")

        try:
            limit = int(limit_raw) if limit_raw is not None else 100
        except ValueError:
            limit = 100
        limit = max(1, min(500, limit))

        since_seq = None
        if since_raw is not None:
            try:
                since_seq = int(since_raw)
            except ValueError:
                since_seq = None

        with potholes_lock:
            items = list(potholes)

        if since_seq is not None:
            items = [p for p in items if int(p.get("seq", 0)) > since_seq]
        if len(items) > limit:
            items = items[-limit:]

        next_seq = (items[-1].get("seq") if items else None)
        return jsonify({"items": items, "count": len(items), "next_since_seq": next_seq})

    return app


app = create_app()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flask backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5000, type=int)
    parser.add_argument("--no-debug", action="store_true")
    args = parser.parse_args()

    # Threaded dev server helps keep teleop responsive even when large payload
    # endpoints (like /map) are being polled.
    # Disable the Flask reloader: it spawns a second process in debug mode, which can
    # create multiple RosBridge clients/streams and lead to cmd_vel being reset to zero.
    app.run(
        host=args.host,
        port=args.port,
        debug=not args.no_debug,
        threaded=True,
        use_reloader=False,
    )
