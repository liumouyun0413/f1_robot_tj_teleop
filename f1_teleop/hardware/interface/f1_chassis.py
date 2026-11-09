"""
F1ChassisController —— 底盘速度接口（对应官方示例的 R1LiteChassisController）。

  * set_velocity_command(vx, vy, wz) 设定目标速度；
  * publish() 以固定频率发布 TwistStamped 到 /move/ManualMoveCmd；
  * enable() 调用 /move/SetControlMode 切换到手动模式；disable() 发零速刹车。
"""

from __future__ import annotations

from geometry_msgs.msg import TwistStamped

from f1_teleop.hardware.interface.ros_context import RosContext
from f1_teleop.utils.control_utils import RateLimiter

try:
    from move.srv import SetControlMode
except ImportError as exc:  # interface_20260623 未 source
    raise ImportError(
        "无法导入 move.srv.SetControlMode，请先 source interface_20260623 的 install/setup.bash"
    ) from exc


class F1ChassisController:
    def __init__(
        self,
        ros: RosContext,
        move_topic: str,
        control_mode_service: str,
        control_mode_value: int,
        frame_id: str = "base_link",
        publish_rate_hz: float = 30.0,
    ) -> None:
        self.ros = ros
        self.frame_id = frame_id
        self.control_mode_value = int(control_mode_value)
        self._pub = ros.node.create_publisher(TwistStamped, move_topic, 10)
        self._mode_client = ros.node.create_client(SetControlMode, control_mode_service)
        self._limiter = RateLimiter(publish_rate_hz)
        self._cmd = [0.0, 0.0, 0.0]
        self.enabled = False

    def set_velocity_command(self, vx: float, vy: float, wz: float) -> None:
        self._cmd = [float(vx), float(vy), float(wz)]

    def get_velocity_command(self) -> list[float]:
        return list(self._cmd)

    def enable(self) -> None:
        if not self._mode_client.wait_for_service(timeout_sec=2.0):
            print(f"[Chassis] 服务 {self._mode_client.srv_name} 不可用，底盘无法激活")
            return
        req = SetControlMode.Request()
        req.mode = self.control_mode_value
        future = self._mode_client.call_async(req)

        def _done(fut):
            try:
                fut.result()
                print(f"[Chassis] SetControlMode(mode={self.control_mode_value}) 成功")
            except Exception as e:
                print(f"[Chassis] SetControlMode 失败: {e}")

        future.add_done_callback(_done)
        self.enabled = True
        print("[Chassis] 底盘控制已激活")

    def disable(self) -> None:
        self.enabled = False
        self.set_velocity_command(0.0, 0.0, 0.0)
        self._publish_now()
        print("[Chassis] 底盘控制已停止")

    def publish(self) -> None:
        """使能状态下按固定频率发布当前速度指令。"""
        if self.enabled and self._limiter.ready():
            self._publish_now()

    def _publish_now(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.ros.now_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x, msg.twist.linear.y = self._cmd[0], self._cmd[1]
        msg.twist.angular.z = self._cmd[2]
        self._pub.publish(msg)

    def stop(self) -> None:
        if self.enabled:
            self.disable()
