"""
F1HeadController —— 头部三关节增量位置控制。

目标角（度）由摇杆每帧增量更新并钳位，有变化时发布 JointState（仅 head_j1~3）
到 /motion_ctl/joint_ctl。初始目标取自 /hal/joint_states 的真实头部角度。
"""

from __future__ import annotations

import threading
from typing import Sequence

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from f1_teleop.hardware.interface.ros_context import RosContext


class F1HeadController:
    def __init__(
        self,
        ros: RosContext,
        joint_names: Sequence[str],
        joint_cmd_topic: str,
        limits_deg: Sequence[Sequence[float]],
        step_deg_per_frame: float,
        frame_id: str = "base_link",
    ) -> None:
        self.ros = ros
        self.joint_names = list(joint_names)
        self.frame_id = frame_id
        self.limits = np.asarray(limits_deg, dtype=float)
        self.step = float(step_deg_per_frame)
        if self.limits.shape != (len(self.joint_names), 2):
            raise ValueError("头部 limits_deg 维度应为 (关节数, 2)")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._pub = ros.node.create_publisher(JointState, joint_cmd_topic, qos)
        self._lock = threading.Lock()
        self._target = np.zeros(len(self.joint_names), dtype=float)

    def set_target(self, target_deg: Sequence[float]) -> None:
        with self._lock:
            self._target = np.clip(np.asarray(target_deg, dtype=float), self.limits[:, 0], self.limits[:, 1])

    def get_target(self) -> list[float]:
        with self._lock:
            return self._target.tolist()

    def step_by(self, axis_values: Sequence[float]) -> bool:
        """按 [-1,1] 轴输入做一帧增量；返回目标是否变化。"""
        axes = np.asarray(axis_values, dtype=float)
        if not np.any(axes):
            return False
        with self._lock:
            new = np.clip(self._target + axes * self.step, self.limits[:, 0], self.limits[:, 1])
            changed = not np.array_equal(new, self._target)
            self._target = new
        return changed

    def publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.ros.now_msg()
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = self.get_target()
        self._pub.publish(msg)
