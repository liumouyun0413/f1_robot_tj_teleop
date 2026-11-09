"""
进程内唯一的 rclpy 节点 + 后台 executor 线程。

所有硬件接口类共用一个 Node（与官方 R1 Lite 示例共用一个 rospy 节点等价），
避免多节点 / 多 executor 互相争抢。
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class RosContext:
    def __init__(self, node_name: str = "f1_teleop") -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node(node_name)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._running = True
        self._thread = threading.Thread(target=self._spin, name="ros_spin", daemon=True)
        self._thread.start()
        print(f"[ROS] 节点 {node_name} 已启动")

    def _spin(self) -> None:
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception as e:
                print(f"[ROS] spin 异常: {e!r}")

    def now_msg(self):
        return self.node.get_clock().now().to_msg()

    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        print("[ROS] 已关闭")
