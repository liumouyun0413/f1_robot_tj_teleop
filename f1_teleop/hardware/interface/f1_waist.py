"""
F1WaistController —— 腰部升降（对应官方示例的 R1LiteTorsoController）。

向 EC_Line_Control 的 waist_updown_ros2 节点发布 std_msgs/Bool：
True = 上升一个步长，False = 下降一个步长（步长由该节点的 ini 配置决定）。
"""

from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool

from f1_teleop.hardware.interface.ros_context import RosContext


class F1WaistController:
    def __init__(self, ros: RosContext, waist_cmd_topic: str) -> None:
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._pub = ros.node.create_publisher(Bool, waist_cmd_topic, qos)

    def step(self, up: bool) -> None:
        msg = Bool()
        msg.data = bool(up)
        self._pub.publish(msg)
        print(f"[Waist] {'上升' if up else '下降'}")
