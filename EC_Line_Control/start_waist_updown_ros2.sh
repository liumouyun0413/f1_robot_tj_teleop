#!/usr/bin/env bash
# Start the ROS 2 waist up/down controller in the background.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_SCRIPT="$SCRIPT_DIR/dist/waist_updown_ros2.py"
PID_FILE="$SCRIPT_DIR/waist_updown_ros2.pid"
LOG_FILE="$SCRIPT_DIR/waist_updown_ros2.log"

if [[ ! -f "$NODE_SCRIPT" ]]; then
    echo "Error: controller not found: $NODE_SCRIPT" >&2
    exit 1
fi

if [[ -f "$PID_FILE" ]]; then
    pid="$(cat "$PID_FILE")"
    if kill -0 "$pid" 2>/dev/null; then
        command="$(ps -p "$pid" -o args= 2>/dev/null || true)"
        if [[ "$command" == *"$NODE_SCRIPT"* ]]; then
            echo "Controller is already running (PID: $pid)."
            exit 0
        fi
    fi
    rm -f "$PID_FILE"
fi

ros_setup="$(compgen -G '/opt/ros/*/setup.bash' | head -n 1 || true)"
if [[ -z "$ros_setup" ]]; then
    echo "Error: ROS 2 setup.bash was not found under /opt/ros." >&2
    exit 1
fi

nohup bash -lc "source '$ros_setup' && exec python3 '$NODE_SCRIPT'" \
    > "$LOG_FILE" 2>&1 < /dev/null &
echo $! > "$PID_FILE"
echo "Controller started (PID: $(cat "$PID_FILE"))."
echo "Log: $LOG_FILE"
