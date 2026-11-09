"""
F1ArmController —— 双臂 + 夹爪 ROS2 接口（对应官方示例的 A1XController）。

职责：
  * 订阅 /hal/joint_states（度，按关节名索引），缓存全身关节状态；
  * 发布 /motion_ctl/joint_ctl（双臂 14 关节，度）；
  * 发布 /motion_ctl/gripper/{left,right}（GripperCommand）；
  * 提供 move_to(目标位姿, 时长) 的直线插值运动（启动回零位 / X+Y 复位）；
  * 记录最近一次下发的双臂指令 last_cmd_deg（供数采镜像与安全检查使用）。
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Sequence

import numpy as np
from control_msgs.msg import GripperCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from f1_teleop.hardware.interface.ros_context import RosContext


class F1ArmController:
    def __init__(
        self,
        ros: RosContext,
        left_joint_names: Sequence[str],
        right_joint_names: Sequence[str],
        joint_cmd_topic: str,
        joint_state_topic: str,
        gripper_left_cmd_topic: str,
        gripper_right_cmd_topic: str,
        gripper_max_effort: float = 15.0,
    ) -> None:
        self.ros = ros
        self.left_joint_names = list(left_joint_names)
        self.right_joint_names = list(right_joint_names)
        self.arm_joint_names = self.left_joint_names + self.right_joint_names
        self.gripper_max_effort = float(gripper_max_effort)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        node = ros.node
        self._joint_pub = node.create_publisher(JointState, joint_cmd_topic, reliable)
        self._grip_left_pub = node.create_publisher(GripperCommand, gripper_left_cmd_topic, reliable)
        self._grip_right_pub = node.create_publisher(GripperCommand, gripper_right_cmd_topic, reliable)
        self._state_sub = node.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, qos_profile_sensor_data
        )

        self._lock = threading.Lock()
        self._state_deg: Dict[str, float] = {}
        self._state_stamp: float = -1.0
        self.last_cmd_deg: Optional[np.ndarray] = None     # 最近一次下发的 14 维双臂指令（度）
        self.last_gripper_cmd: List[float] = [0.0, 0.0]

    # ------------------------------------------------------------------ #
    # 状态
    # ------------------------------------------------------------------ #
    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._state_deg[name] = float(pos)
            self._state_stamp = time.time()

    @property
    def has_state(self) -> bool:
        return self._state_stamp > 0

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.has_state:
                return True
            time.sleep(0.05)
        return False

    def get_joint_deg(self, names: Sequence[str], default: float = 0.0) -> np.ndarray:
        """按关节名读取最新反馈（度）；缺失的关节用 default 填充。"""
        with self._lock:
            return np.array([self._state_deg.get(n, default) for n in names], dtype=float)

    def get_missing_joints(self, names: Sequence[str]) -> List[str]:
        with self._lock:
            return [n for n in names if n not in self._state_deg]

    def get_arm_state_deg(self) -> np.ndarray:
        return self.get_joint_deg(self.arm_joint_names)

    # ------------------------------------------------------------------ #
    # 指令
    # ------------------------------------------------------------------ #
    def publish_arm_cmd(self, arm_deg: Sequence[float]) -> None:
        arm = np.asarray(arm_deg, dtype=float)
        if arm.shape != (14,):
            raise ValueError(f"双臂指令应为 14 维，实际 {arm.shape}")
        msg = JointState()
        msg.header.stamp = self.ros.now_msg()
        msg.name = self.arm_joint_names
        msg.position = arm.tolist()
        self._joint_pub.publish(msg)
        self.last_cmd_deg = arm

    def publish_gripper_cmd(self, left: float, right: float) -> None:
        for pub, value in ((self._grip_left_pub, left), (self._grip_right_pub, right)):
            msg = GripperCommand()
            msg.position = float(value)
            msg.max_effort = self.gripper_max_effort
            pub.publish(msg)
        self.last_gripper_cmd = [float(left), float(right)]

    def move_to(self, target_deg: Sequence[float], duration_sec: float, rate_hz: float = 50.0) -> bool:
        """从真机当前反馈位置线性插值到目标（阻塞）。反馈缺失时从 last_cmd_deg 起步。"""
        target = np.asarray(target_deg, dtype=float)
        if target.shape != (14,):
            raise ValueError("move_to 目标应为 14 维")

        if self.has_state:
            start = self.get_arm_state_deg()
        elif self.last_cmd_deg is not None:
            start = self.last_cmd_deg.copy()
        else:
            print("[Arm] 无真机反馈且无历史指令，无法安全插值，跳过运动")
            return False

        steps = max(1, int(duration_sec * rate_hz))
        dt = 1.0 / rate_hz
        print(f"[Arm] 插值运动 {duration_sec:.1f}s: {np.round(start, 1)} → {np.round(target, 1)}")
        for i in range(steps + 1):
            alpha = i / steps
            self.publish_arm_cmd(start + (target - start) * alpha)
            time.sleep(dt)
        return True
