#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$REPO_ROOT/ros_ws"
UI_DIR="$REPO_ROOT/ui"
VENV_DIR="$REPO_ROOT/venv"

err() { echo "ERROR: $*" >&2; }
note() { echo "[run.sh] $*"; }

note "Repo: $REPO_ROOT"

# Basic commands
command -v python3 >/dev/null || { err "python3 not found"; exit 1; }
command -v npm >/dev/null || { err "npm not found (install Node.js + npm)"; exit 1; }
command -v ros2 >/dev/null || { err "ros2 not found (install ROS 2 Humble)"; exit 1; }
command -v colcon >/dev/null || { err "colcon not found (sudo apt install python3-colcon-common-extensions)"; exit 1; }

# Check ROS Humble install path exists
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  err "ROS 2 Humble not found at /opt/ros/humble/setup.bash"
  exit 1
fi

# Check venv
if [[ ! -d "$VENV_DIR" ]]; then
  err "Python venv not found at $VENV_DIR"
  err "Create it with:"
  err "  cd $REPO_ROOT && python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt"
  exit 1
fi

# Check UI deps
if [[ ! -d "$UI_DIR/node_modules" ]]; then
  err "UI dependencies not installed (missing $UI_DIR/node_modules)"
  err "Install with:"
  err "  cd $UI_DIR && npm install"
  exit 1
fi

# Check ROS workspace exists
if [[ ! -d "$ROS_WS/src/ros_supervisor" ]]; then
  err "ROS wrapper package not found at $ROS_WS/src/ros_supervisor"
  exit 1
fi

note "Sourcing ROS 2 Humble..."
# ROS setup scripts sometimes reference unset variables; temporarily disable nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

note "Building ROS workspace (merge-install)..."
cd "$ROS_WS"
rm -rf build install log
colcon build --merge-install

note "Sourcing workspace overlay..."
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

note "Launching ROS Supervisor (backend + UI)..."
exec ros2 launch ros_supervisor supervisor.launch.py
