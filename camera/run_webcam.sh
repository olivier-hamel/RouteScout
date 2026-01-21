#!/usr/bin/env bash
set -euo pipefail

# Runs camera_video.py with a minimal environment.
# This avoids Snap-injected GTK/Qt paths (common when VS Code is installed via snap)
# that can cause native library conflicts like:
#   /snap/core20/.../libpthread.so.0: undefined symbol: __libc_pthread_init

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

if [[ -n "${PYTHON_BIN:-}" ]]; then
  :
elif [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
  PYTHON_BIN="${VIRTUAL_ENV}/bin/python"
else
  PYTHON_BIN="python3"
fi

# Preserve only the essentials for GUI + user context.
# (If you are on Wayland, you may need WAYLAND_DISPLAY too.)
ENV_VARS=(
  "HOME=${HOME:-}"
  "USER=${USER:-}"
  "PATH=/usr/bin:/bin"
)

if [[ -n "${DISPLAY:-}" ]]; then ENV_VARS+=("DISPLAY=$DISPLAY"); fi
if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then ENV_VARS+=("XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"); fi
if [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]]; then ENV_VARS+=("DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS"); fi
if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then ENV_VARS+=("WAYLAND_DISPLAY=$WAYLAND_DISPLAY"); fi
if [[ -n "${LANG:-}" ]]; then ENV_VARS+=("LANG=$LANG"); fi
if [[ -n "${LC_ALL:-}" ]]; then ENV_VARS+=("LC_ALL=$LC_ALL"); fi

echo "[run_webcam] python=$(command -v "$PYTHON_BIN" || echo "$PYTHON_BIN")" >&2
exec env -i "${ENV_VARS[@]}" "$PYTHON_BIN" camera_video.py "$@"
