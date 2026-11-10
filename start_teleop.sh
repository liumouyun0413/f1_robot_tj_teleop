#!/bin/bash
# ============================================================================
# F1 遥操作一键启动（替代 lxl 版的 start_all_pico2.sh）
#   1) 腰部升降节点（EC_Line_Control，可选）
#   2) PICO 数据服务 RoboticsService
#   3) 遥操作主程序 teleop_f1_hardware.py（单进程：双臂/夹爪/头/腰/底盘/安全门/数采）
# 日志按时间戳写入 ./log/，重启不覆盖。
# 注意：不要开 set -u，ROS/conda 的 setup 脚本会引用未定义变量。
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
TS="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

# ---------------- 可按现场环境修改（2026-11-09 按 ms-thor 实机核实） ----------------
ROS_SETUP="/opt/ros/jazzy/setup.bash"
# interface_20260623 编译产物（move/srv 等）：优先用机器人上已编译的，其次用本目录自带的
IFACE_SETUP_CANDIDATES=(
    "${HOME}/lxl/interface_20260623/install/setup.bash"
    "${SCRIPT_DIR}/interface_20260623/install/setup.bash"
)
F1_SETUP="${HOME}/workspace/f1/install/setup.bash"       # f1_msgs / motor_ctl（非必需）
CONDA_BASE="${HOME}/miniconda3"
CONDA_ENV="pico4ultra_py312"
ROBOTICS_SERVICE_DIR="/opt/apps/roboticsservice"
# 腰部节点已由驱动栈 ~/workspace/start_all.sh 拉起（/waist_updown_controller），默认不重复启动
START_WAIST_NODE="${START_WAIST_NODE:-0}"
TELEOP_ARGS="${TELEOP_ARGS:-}" # 例如 TELEOP_ARGS="--visualize-placo"
# --------------------------------------------------

echo "=== F1 teleop 启动 (${TS}) ==="

# ROS2 环境：与 ~/.bashrc / 驱动栈保持一致（nohup 非交互 shell 不会读 bashrc，必须显式导出）
source "${ROS_SETUP}"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"enP3p1s0f1\"/></Interfaces></General></Domain></CycloneDDS>"
export RCUTILS_LOGGING_MAX_SEVERITY=2      # 抑制 DDS type-hash WARN 刷屏

IFACE_OK=0
for f in "${IFACE_SETUP_CANDIDATES[@]}"; do
    if [[ -f "$f" ]]; then source "$f"; IFACE_OK=1; echo "已 source 接口包: $f"; break; fi
done
[[ "${IFACE_OK}" == "1" ]] || echo "⚠️ 未找到 interface_20260623 编译产物，move/srv 无法导入（底盘使能会失败）"
[[ -f "${F1_SETUP}" ]] && source "${F1_SETUP}"

# 1) 腰部节点（通常已由驱动栈启动，仅 START_WAIST_NODE=1 且未运行时拉起）
if [[ "${START_WAIST_NODE}" == "1" ]]; then
    if pgrep -f waist_updown_ros2.py >/dev/null; then
        echo "腰部节点已在运行"
    elif [[ -f "${SCRIPT_DIR}/EC_Line_Control/start_waist_updown_ros2.sh" ]]; then
        bash "${SCRIPT_DIR}/EC_Line_Control/start_waist_updown_ros2.sh" || echo "⚠️ 腰部节点启动失败"
    fi
elif ! pgrep -f waist_updown_ros2.py >/dev/null; then
    echo "⚠️ 未检测到腰部节点 waist_updown_ros2.py，Y/X 升降腰将无响应（可 START_WAIST_NODE=1 重跑）"
fi

# 2) RoboticsService（PICO 手柄数据）
if [[ -d "${ROBOTICS_SERVICE_DIR}" ]]; then
    if pgrep -f RoboticsServiceProcess >/dev/null; then
        echo "RoboticsService 已在运行"
    else
        (cd "${ROBOTICS_SERVICE_DIR}" && nohup ./runService.sh > "${LOG_DIR}/roboticsservice_${TS}.log" 2>&1 &)
        echo "RoboticsService 已启动，等待就绪..."
        sleep 3
    fi
else
    echo "⚠️ 未找到 ${ROBOTICS_SERVICE_DIR}"
fi

# 3) 遥操作主程序（conda 环境；实测该环境下 conda python 可直接导入 rclpy，无需清理库路径）
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}" || { echo "❌ conda 环境 ${CONDA_ENV} 激活失败"; exit 1; }

if pgrep -f teleop_f1_hardware.py >/dev/null; then
    echo "❌ teleop_f1_hardware.py 已在运行，请先 ./stop_teleop.sh"; exit 1
fi
if pgrep -f "teleop_lxl.py|all_remote_lxl.py" >/dev/null; then
    echo "❌ 旧版 lxl 遥操进程仍在运行，会与本程序争抢 /motion_ctl/joint_ctl，请先 ~/lxl/stop_all_pico.sh"; exit 1
fi

cd "${SCRIPT_DIR}"
nohup python -u scripts/teleop_f1_hardware.py ${TELEOP_ARGS} > "${LOG_DIR}/teleop_${TS}.log" 2>&1 &
echo $! > "${LOG_DIR}/teleop.pid"
sleep 2

echo ""
echo "进程检查："
ps aux | grep -E 'teleop_f1_hardware|runService|RoboticsServiceProcess|waist_updown_ros2' | grep -v grep
echo ""
echo "日志：tail -f ${LOG_DIR}/teleop_${TS}.log"
echo "停止：./stop_teleop.sh"
