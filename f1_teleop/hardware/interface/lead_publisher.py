"""
LeadDataPublisher —— 主端（遥操作端）数据镜像发布，供 ros2 bag 录制对齐主从数据。

  * /lead/joint_states : 17 关节 = 双臂最近下发指令(14, 度) + 头部目标(3, 度)
  * /lead/left_tcp, /lead/right_tcp : PICO 手柄位姿 [x,y,z,rx,ry,rz]，
    frame_id = press_pico（握把按下）/ un_press_pico
"""

from __future__ import annotations

import threading
from typing import Dict, Optional, Sequence

import numpy as np
from sensor_msgs.msg import JointState

from f1_teleop.hardware.interface.ros_context import RosContext
from f1_teleop.utils.geometry import quat_wxyz_to_euler_deg

TCP_FIELD_NAMES = ["x", "y", "z", "rx", "ry", "rz"]


class LeadDataPublisher:
    def __init__(
        self,
        ros: RosContext,
        joint_state_topic: str,
        left_tcp_topic: str,
        right_tcp_topic: str,
        lead_joint_names: Sequence[str],
    ) -> None:
        self.ros = ros
        self.lead_joint_names = list(lead_joint_names)
        self._joint_pub = ros.node.create_publisher(JointState, joint_state_topic, 10)
        self._tcp_pubs = {
            "left": ros.node.create_publisher(JointState, left_tcp_topic, 10),
            "right": ros.node.create_publisher(JointState, right_tcp_topic, 10),
        }
        self._lock = threading.Lock()
        self._tcp: Dict[str, Optional[tuple[list[float], bool]]] = {"left": None, "right": None}

    def update_controller_pose(self, side: str, xr_pose: Optional[np.ndarray], active: bool) -> None:
        """记录手柄位姿（[x,y,z,qx,qy,qz,qw]）；xr_pose 为 None 时仅更新激活标志。"""
        with self._lock:
            if xr_pose is not None:
                euler = quat_wxyz_to_euler_deg([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]])
                self._tcp[side] = ([float(xr_pose[0]), float(xr_pose[1]), float(xr_pose[2]), *euler], active)
            elif self._tcp[side] is not None:
                self._tcp[side] = (self._tcp[side][0], active)

    def publish(self, arm_cmd_deg: Optional[np.ndarray], head_target_deg: Sequence[float]) -> None:
        stamp = self.ros.now_msg()
        if arm_cmd_deg is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = self.lead_joint_names
            msg.position = [*np.asarray(arm_cmd_deg, dtype=float).tolist(), *[float(v) for v in head_target_deg]]
            self._joint_pub.publish(msg)

        with self._lock:
            snapshot = dict(self._tcp)
        for side, data in snapshot.items():
            if data is None:
                continue
            pose, active = data
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "press_pico" if active else "un_press_pico"
            msg.name = TCP_FIELD_NAMES
            msg.position = pose
            self._tcp_pubs[side].publish(msg)
