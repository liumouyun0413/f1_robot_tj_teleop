"""
ExternalPoseInput —— 外部位姿回放输入（可选）。

当 /lead_tcp/replay_mode 为 True 时，双臂输入不再来自 PICO 手柄，而是来自
/lead/{left,right}_tcp_control（JointState.position = [x,y,z,rx,ry,rz]，欧拉角度；
header.frame_id == "press_pico" 表示该臂激活）。与 LeadDataPublisher 发布的
/lead/*_tcp 格式一致，可直接回放录制数据。
"""

from __future__ import annotations

import threading
from typing import Dict, Optional, Tuple

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from f1_teleop.hardware.interface.ros_context import RosContext
from f1_teleop.utils.geometry import euler_deg_to_quat_wxyz


class ExternalPoseInput:
    def __init__(self, ros: RosContext, replay_mode_topic: str, left_topic: str, right_topic: str) -> None:
        node = ros.node
        node.create_subscription(Bool, replay_mode_topic, self._on_mode, 10)
        node.create_subscription(JointState, left_topic, lambda m: self._on_pose("left", m), 10)
        node.create_subscription(JointState, right_topic, lambda m: self._on_pose("right", m), 10)
        self._lock = threading.Lock()
        self.enabled = False
        self._latest: Dict[str, Tuple[bool, Optional[np.ndarray]]] = {"left": (False, None), "right": (False, None)}

    def _on_mode(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        print(f"[Replay] 外部位姿回放模式 {'开启' if self.enabled else '关闭'}")

    def _on_pose(self, side: str, msg: JointState) -> None:
        p = list(msg.position)
        if len(p) < 6:
            return
        q = euler_deg_to_quat_wxyz(p[3:6])          # [w,x,y,z]
        pose = np.array([p[0], p[1], p[2], q[1], q[2], q[3], q[0]], dtype=float)   # [x,y,z,qx,qy,qz,qw]
        with self._lock:
            self._latest[side] = (msg.header.frame_id == "press_pico", pose)

    def read(self, side: str) -> Tuple[bool, Optional[np.ndarray]]:
        with self._lock:
            return self._latest[side]
