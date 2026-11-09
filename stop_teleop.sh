#!/bin/bash
# 停止 F1 遥操作相关进程（遥操作主程序 / RoboticsService / 腰部节点）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 腰部节点属于驱动栈（~/workspace/start_all.sh），默认不停
STOP_WAIST_NODE="${STOP_WAIST_NODE:-0}"

echo "=== 停止 F1 teleop ==="
pkill -INT -f "teleop_f1_hardware.py" 2>/dev/null
sleep 2
pkill -9 -f "teleop_f1_hardware.py" 2>/dev/null
pkill -f "runService.sh" 2>/dev/null
pkill -f "RoboticsServiceProcess" 2>/dev/null
rm -f "${SCRIPT_DIR}/log/teleop.pid"

if [[ "${STOP_WAIST_NODE}" == "1" && -f "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" ]]; then
    bash "${SCRIPT_DIR}/EC_Line_Control/stop_waist_updown_ros2.sh" || true
fi

sleep 1
if ps aux | grep -E 'teleop_f1_hardware|runService|RoboticsServiceProcess' | grep -v grep; then
    echo "⚠️ 仍有残留进程，请再执行一次"
else
    echo "✅ 已全部停止"
fi
