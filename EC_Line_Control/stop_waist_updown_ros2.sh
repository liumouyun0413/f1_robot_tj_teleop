#!/usr/bin/env bash
# Stop the ROS 2 waist up/down controller started by start_waist_updown_ros2.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_SCRIPT="$SCRIPT_DIR/dist/waist_updown_ros2.py"
PID_FILE="$SCRIPT_DIR/waist_updown_ros2.pid"

if [[ ! -f "$PID_FILE" ]]; then
    echo "Controller is not running: PID file not found."
    exit 0
fi

pid="$(cat "$PID_FILE")"
if ! kill -0 "$pid" 2>/dev/null; then
    echo "Controller is not running (stale PID: $pid)."
    rm -f "$PID_FILE"
    exit 0
fi

command="$(ps -p "$pid" -o args= 2>/dev/null || true)"
if [[ "$command" != *"$NODE_SCRIPT"* ]]; then
    echo "Error: PID $pid does not belong to $NODE_SCRIPT; refusing to stop it." >&2
    exit 1
fi

kill "$pid"
rm -f "$PID_FILE"
echo "Controller stop signal sent (PID: $pid)."
