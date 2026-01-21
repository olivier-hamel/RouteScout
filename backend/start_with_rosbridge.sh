#!/usr/bin/env bash
set -euo pipefail

# Starts:
# 1) rosbridge websocket (ROS2 Jazzy-safe params)
# 2) Flask backend with ENABLE_ROSBRIDGE=true
#
# Usage:
#   ./start_with_rosbridge.sh
#
# Optional env overrides:
#   ROS_SETUP=/opt/ros/jazzy/setup.bash
#   ROSBRIDGE_BIND_ADDRESS=0.0.0.0
#   ROSBRIDGE_HOST=127.0.0.1
#   ROSBRIDGE_PORT=9090
#   ROSBRIDGE_TOPIC=/chatter
#   ROSBRIDGE_MESSAGE_TYPE=std_msgs/String
#   FLASK_HOST=0.0.0.0
#   FLASK_PORT=5000

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROSBRIDGE_PARAMS_FILE="${ROSBRIDGE_PARAMS_FILE:-$SCRIPT_DIR/rosbridge_params.yaml}"

VENV_ACTIVATE=""
if [[ -f "$REPO_ROOT/.venv/bin/activate" ]]; then
  VENV_ACTIVATE="$REPO_ROOT/.venv/bin/activate"
elif [[ -f "$SCRIPT_DIR/.venv/bin/activate" ]]; then
  VENV_ACTIVATE="$SCRIPT_DIR/.venv/bin/activate"
fi

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ROS_SETUP"
  set -u
else
  echo "WARN: ROS setup file not found at '$ROS_SETUP'." >&2
  echo "      If ros2 is not on PATH, set ROS_SETUP=/path/to/setup.bash" >&2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: 'ros2' not found in PATH. Did you source ROS2?" >&2
  exit 1
fi

if [[ ! -f "$ROSBRIDGE_PARAMS_FILE" ]]; then
  echo "ERROR: rosbridge params file not found at '$ROSBRIDGE_PARAMS_FILE'" >&2
  exit 1
fi

if [[ -z "$VENV_ACTIVATE" ]]; then
  echo "ERROR: Could not find a venv activate script." >&2
  echo "       Expected '$REPO_ROOT/.venv/bin/activate' (recommended)." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${ROSBRIDGE_PID:-}" ]]; then
    kill "$ROSBRIDGE_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting rosbridge websocket..."
ROSBRIDGE_BIND_ADDRESS="${ROSBRIDGE_BIND_ADDRESS:-0.0.0.0}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"

# If the port is already bound, rosbridge will spam retries and the backend
# won't be able to reliably connect. Fail fast with actionable info.
if ss -ltnp 2>/dev/null | grep -qE "[:.]${ROSBRIDGE_PORT}\\s"; then
  echo "ERROR: ROSBRIDGE_PORT=${ROSBRIDGE_PORT} is already in use." >&2
  echo "       Listener(s):" >&2
  ss -ltnp 2>/dev/null | grep -E "[:.]${ROSBRIDGE_PORT}\\s" >&2 || true
  echo "" >&2
  echo "Fix options:" >&2
  echo "  - Stop the existing rosbridge process (or whatever owns the port), then re-run." >&2
  echo "  - Or choose a different port, e.g. 'ROSBRIDGE_PORT=9091 ./start_with_rosbridge.sh'" >&2
  exit 1
fi

ros2 run rosbridge_server rosbridge_websocket --ros-args \
  --params-file "$ROSBRIDGE_PARAMS_FILE" \
  -p address:="$ROSBRIDGE_BIND_ADDRESS" \
  -p port:="$ROSBRIDGE_PORT" &
ROSBRIDGE_PID=$!

# Give rosbridge a moment to initialize.
sleep 1

echo "Starting Flask backend..."
# shellcheck disable=SC1090
source "$VENV_ACTIVATE"

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  else
    echo "ERROR: Could not find python/python3 after activating the venv." >&2
    exit 1
  fi
fi

export ENABLE_ROSBRIDGE=true
export ROSBRIDGE_HOST="${ROSBRIDGE_HOST:-127.0.0.1}"
export ROSBRIDGE_PORT="$ROSBRIDGE_PORT"
export ROSBRIDGE_TOPIC="${ROSBRIDGE_TOPIC:-/chatter}"
export ROSBRIDGE_MESSAGE_TYPE="${ROSBRIDGE_MESSAGE_TYPE:-std_msgs/msg/String}"

# TF is required to compute map->odom and show the robot pose aligned with /map.
# If this causes lag on your setup, set these to false.
export ROSBRIDGE_SUBSCRIBE_TF="${ROSBRIDGE_SUBSCRIBE_TF:-true}"
export ROSBRIDGE_SUBSCRIBE_TF_STATIC="${ROSBRIDGE_SUBSCRIBE_TF_STATIC:-true}"

# cmd_vel keepalive stream (10-20Hz is common for mobile bases). When idle, the
# backend will publish zeros after a short timeout.
export ROSBRIDGE_CMD_VEL_STREAM="${ROSBRIDGE_CMD_VEL_STREAM:-true}"
export ROSBRIDGE_CMD_VEL_STREAM_HZ="${ROSBRIDGE_CMD_VEL_STREAM_HZ:-15}"
export ROSBRIDGE_CMD_VEL_IDLE_TIMEOUT_S="${ROSBRIDGE_CMD_VEL_IDLE_TIMEOUT_S:-0.25}"

FLASK_HOST="${FLASK_HOST:-0.0.0.0}"
FLASK_PORT="${FLASK_PORT:-5000}"

cd "$SCRIPT_DIR"
"$PYTHON_BIN" app.py --host "$FLASK_HOST" --port "$FLASK_PORT"
