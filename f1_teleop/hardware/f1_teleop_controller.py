"""
F1TeleopController —— F1 轮式人形机器人真机遥操作控制器。

对应官方示例 GalaxeaR1LiteTeleopController 的角色：把通用 IK 骨架
（HardwareTeleopController）与 F1 的硬件接口（双臂 / 夹爪 / 头 / 腰 / 底盘 /
安全门 / 数采）组装起来。

手柄按键一览
------------
  握把(grip) > 0.9          激活对应臂，手柄位姿增量 → 末端位姿
  扳机(trigger)             夹爪开合（0 → 张开指令，1 → 闭合指令）
  left_axis_click           末端模式 = position（纯位置）
  right_axis_click          末端模式 = orientation（纯姿态）
  两摇杆同时按压            末端模式 = pose（位置+姿态，默认）
  X + Y                     双臂复位到 reset_pose（插值），Placo 同步
  Y 长按 0.3s               腰部上升一步        X 长按 0.3s   腰部下降一步
  A                         开始数采            B             停止数采
  右扳机按住 + 双握把松开   头部模式：左摇杆 X/Y → roll/pitch，右摇杆 X → yaw
  右摇杆上拨                底盘使能            右摇杆下拨    底盘停止
  底盘使能后：左摇杆 → 前后/横移，右摇杆 X → 旋转（头部模式期间挂起）

设计说明
--------
* Placo 是开环"虚拟机器人"：启动 / 复位时用真机位姿同步一次，之后只由手柄驱动，
  不每帧回读真机反馈（避免与滞后的真机位置控制互相拉扯）。
* Placo 弧度 ↔ 真机度 的映射集中在 JointMap，正反两向共用同一组参数。
* 双臂指令只在至少一臂激活时下发；复位期间由复位线程独占双臂通道。
"""

from __future__ import annotations

import threading
from types import SimpleNamespace
from typing import Dict, Optional, Tuple

import numpy as np

from f1_teleop.common.hardware_teleop_controller import HardwareTeleopController
from f1_teleop.hardware.interface import (
    BagRecorder,
    ExternalPoseInput,
    F1ArmController,
    F1ChassisController,
    F1HeadController,
    F1WaistController,
    LeadDataPublisher,
    RecordingManager,
    RosContext,
    SafetyGuard,
    TianjiRecorderClient,
)
from f1_teleop.utils.control_utils import HoldTrigger, JointMap, RateLimiter, RisingEdge

ARM_ORDER = ("left_arm", "right_arm")
_SIDE = {"left_arm": "left", "right_arm": "right"}


def build_manipulator_config(cfg: SimpleNamespace) -> Dict[str, dict]:
    r, g = cfg.robot, cfg.teleop.gripper
    return {
        "left_arm": {
            "link_name": r.left_ee_link,
            "joint_names": list(r.urdf_left_joint_names),
            "shoulder_link": getattr(r, "left_shoulder_link", None),
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "gripper_config": {"gripper_trigger": "left_trigger", "open_pos": g.open_cmd, "close_pos": g.close_cmd},
        },
        "right_arm": {
            "link_name": r.right_ee_link,
            "joint_names": list(r.urdf_right_joint_names),
            "shoulder_link": getattr(r, "right_shoulder_link", None),
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {"gripper_trigger": "right_trigger", "open_pos": g.open_cmd, "close_pos": g.close_cmd},
        },
    }


class F1TeleopController(HardwareTeleopController):
    def __init__(self, cfg: SimpleNamespace, visualize_placo: bool = False) -> None:
        self.cfg = cfg
        r, t = cfg.robot, cfg.teleop

        self.joint_map: Dict[str, JointMap] = {
            "left_arm": JointMap.from_lists(r.joint_map.left.sign, r.joint_map.left.offset_deg),
            "right_arm": JointMap.from_lists(r.joint_map.right.sign, r.joint_map.right.offset_deg),
        }
        self.home_pose_deg = np.array([*r.home_pose.left, *r.home_pose.right], dtype=float)
        self.reset_pose_deg = np.array([*r.reset_pose.left, *r.reset_pose.right], dtype=float)

        super().__init__(
            robot_urdf_path=r.urdf_path,
            manipulator_config=build_manipulator_config(cfg),
            scale_factor=t.scale_factor,
            visualize_placo=visualize_placo,
            control_rate_hz=int(t.control_rate_hz),
            aux_rate_hz=float(cfg.ros.lead_publish_rate_hz),
            grip_activate_threshold=t.grip_activate_threshold,
            frame_task_weight=t.ik.frame_task_weight,
            manipulability_weight=t.ik.manipulability_weight,
            joint_regularization_weight=t.ik.joint_regularization_weight,
            workspace_radius=t.workspace_radius,
            max_ee_speed=getattr(t, "max_ee_speed_m_s", None),
            max_ee_ang_speed=getattr(t, "max_ee_ang_speed_rad_s", None),
            enable_joint_velocity_limits=bool(getattr(t.ik, "enable_joint_velocity_limits", False)),
        )
        self._apply_driver_joint_limits(getattr(r, "arm_joint_limits_deg", None))
        self.sync_arm_joints(self._deg14_to_placo(self.home_pose_deg))

        # 运行时状态
        self._resetting = threading.Event()
        self._head_active = False
        self._xy_edge = RisingEdge()
        self._y_hold = HoldTrigger(t.waist_button_hold_sec)
        self._x_hold = HoldTrigger(t.waist_button_hold_sec)
        self._a_edge = RisingEdge()
        self._b_edge = RisingEdge()
        self._base_up_edge = RisingEdge()
        self._base_down_edge = RisingEdge()
        self._gripper_limiter = RateLimiter(cfg.teleop.gripper.publish_rate_hz)
        self._max_step_deg = float(getattr(t, "max_joint_speed_deg_s", 180.0)) / self.control_rate_hz

        # 硬件接口在 _robot_setup 中创建
        self.ros: Optional[RosContext] = None
        self.arm: Optional[F1ArmController] = None
        self.head: Optional[F1HeadController] = None
        self.chassis: Optional[F1ChassisController] = None
        self.waist: Optional[F1WaistController] = None
        self.lead_pub: Optional[LeadDataPublisher] = None
        self.external_input: Optional[ExternalPoseInput] = None
        self.recorder: Optional[RecordingManager] = None
        self.safety: Optional[SafetyGuard] = None

    # ------------------------------------------------------------------ #
    # 关节映射辅助
    # ------------------------------------------------------------------ #
    def _apply_driver_joint_limits(self, limits_cfg) -> None:
        """把真机驱动的软限位（度，驱动坐标）收紧到 Placo 模型，避免 IK 解出驱动会裁剪的角度。"""
        if not limits_cfg:
            return
        r = self.cfg.robot
        name_to_arm = {}
        for i, n in enumerate(r.left_joint_names):
            name_to_arm[n] = ("left_arm", i, r.urdf_left_joint_names[i])
        for i, n in enumerate(r.right_joint_names):
            name_to_arm[n] = ("right_arm", i, r.urdf_right_joint_names[i])
        for real_name, (lo_deg, hi_deg) in vars(limits_cfg).items():
            if real_name not in name_to_arm:
                print(f"[Placo] ⚠️ arm_joint_limits_deg 未知关节 {real_name}")
                continue
            arm, idx, urdf_name = name_to_arm[real_name]
            jm = self.joint_map[arm]
            q_off = self.placo_robot.get_joint_offset(urdf_name)
            urdf_lo = float(self.placo_robot.model.lowerPositionLimit[q_off])
            urdf_hi = float(self.placo_robot.model.upperPositionLimit[q_off])

            def to_deg(rad):
                full = np.zeros(7); full[idx] = rad
                return float(jm.rad_to_deg(full)[idx])

            def to_rad(deg):
                full = np.zeros(7); full[idx] = deg
                return float(jm.deg_to_rad(full)[idx])

            # None 的一侧沿用 URDF 限位（先换算到驱动坐标，sign=-1 时上下界互换）
            urdf_deg = sorted((to_deg(urdf_lo), to_deg(urdf_hi)))
            d_lo = urdf_deg[0] if lo_deg is None else max(urdf_deg[0], float(lo_deg))
            d_hi = urdf_deg[1] if hi_deg is None else min(urdf_deg[1], float(hi_deg))
            lo, hi = sorted((to_rad(d_lo), to_rad(d_hi)))
            self.placo_robot.set_joint_limits(urdf_name, lo, hi)
            print(f"[Placo] {real_name}({urdf_name}) 限位收紧为 驱动[{d_lo:.1f}, {d_hi:.1f}]° = URDF[{np.rad2deg(lo):.1f}, {np.rad2deg(hi):.1f}]°")
        self.solver.enable_joint_limits(True)

    def _deg14_to_placo(self, arm_deg: np.ndarray) -> Dict[str, np.ndarray]:
        return {
            "left_arm": self.joint_map["left_arm"].deg_to_rad(arm_deg[:7]),
            "right_arm": self.joint_map["right_arm"].deg_to_rad(arm_deg[7:14]),
        }

    def _placo_to_deg14(self) -> np.ndarray:
        return np.concatenate([self.joint_map[n].rad_to_deg(self.get_arm_q(n)) for n in ARM_ORDER])

    def _sync_to_robot_pose(self, arm_deg: np.ndarray) -> None:
        """Placo 与安全门同时对齐到给定真机位姿（度）。"""
        self.sync_arm_joints(self._deg14_to_placo(arm_deg))
        if self.safety:
            self.safety.reset_last_safe(arm_deg)

    def _body_state_deg(self) -> np.ndarray:
        r = self.cfg.robot
        return self.arm.get_joint_deg([*r.head_joint_names, *r.body_joint_names])

    # ------------------------------------------------------------------ #
    # 初始化 / 停机
    # ------------------------------------------------------------------ #
    def _robot_setup(self) -> None:
        cfg, r, ros_cfg = self.cfg, self.cfg.robot, self.cfg.ros
        self.ros = RosContext(ros_cfg.node_name)

        self.arm = F1ArmController(
            self.ros,
            left_joint_names=r.left_joint_names,
            right_joint_names=r.right_joint_names,
            joint_cmd_topic=ros_cfg.joint_cmd_topic,
            joint_state_topic=ros_cfg.joint_state_topic,
            gripper_left_cmd_topic=ros_cfg.gripper_left_cmd_topic,
            gripper_right_cmd_topic=ros_cfg.gripper_right_cmd_topic,
            gripper_max_effort=cfg.teleop.gripper.max_effort,
        )
        self.head = F1HeadController(
            self.ros,
            joint_names=r.head_joint_names,
            joint_cmd_topic=ros_cfg.joint_cmd_topic,
            limits_deg=cfg.head.limits_deg,
            step_deg_per_frame=cfg.head.step_deg_per_frame,
            frame_id=cfg.head.frame_id,
        )
        self.chassis = F1ChassisController(
            self.ros,
            move_topic=cfg.chassis.move_topic,
            control_mode_service=cfg.chassis.control_mode_service,
            control_mode_value=cfg.chassis.control_mode_value,
            frame_id=cfg.chassis.frame_id,
            publish_rate_hz=cfg.chassis.publish_rate_hz,
        )
        self.waist = F1WaistController(self.ros, ros_cfg.waist_cmd_topic)
        self.lead_pub = LeadDataPublisher(
            self.ros,
            joint_state_topic=ros_cfg.lead_joint_state_topic,
            left_tcp_topic=ros_cfg.lead_left_tcp_topic,
            right_tcp_topic=ros_cfg.lead_right_tcp_topic,
            lead_joint_names=[*r.urdf_left_joint_names, *r.urdf_right_joint_names, *r.head_joint_names],
        )
        self.external_input = ExternalPoseInput(
            self.ros, ros_cfg.replay_mode_topic, ros_cfg.replay_left_tcp_topic, ros_cfg.replay_right_tcp_topic
        )
        rec = cfg.recording
        self.recorder = RecordingManager(
            bag=BagRecorder(rec.bag_dir, rec.topics, rec.storage, rec.validate) if rec.enable else None,
            tianji=TianjiRecorderClient(self.ros, ros_cfg.tianji_start_topic, ros_cfg.tianji_stop_topic),
        )

        # 1. 等真机状态
        print("[Setup] 等待 /hal/joint_states ...")
        if not self.arm.wait_for_state(timeout=5.0):
            raise RuntimeError(f"未收到 {ros_cfg.joint_state_topic}，驱动未启动？")
        missing = self.arm.get_missing_joints([*self.arm.arm_joint_names, *r.head_joint_names, *r.body_joint_names])
        if missing:
            print(f"[Setup] ⚠️ 反馈中缺少关节 {missing}，相关功能将按 0 处理")

        # 2. 头部目标从真实角度起步
        self.head.set_target(self.arm.get_joint_deg(r.head_joint_names))

        # 3. 双臂回零位，Placo 同步
        self.arm.move_to(self.home_pose_deg, r.home_duration_sec, r.move_rate_hz)
        self._sync_to_robot_pose(self.home_pose_deg)
        self._self_check(self.home_pose_deg)

        # 4. 安全门（质心 + 碰撞）
        if cfg.safety.enable:
            init_cmd = np.zeros(24)
            init_cmd[:14] = self.home_pose_deg
            init_cmd[14:22] = self._body_state_deg()
            self.safety = SafetyGuard(
                module_dir=cfg.safety.module_dir,
                body_state_deg_getter=self._body_state_deg,
                initial_cmd_deg=init_cmd,
                enable_com_guard=cfg.safety.enable_com_guard,
                enable_collision_guard=cfg.safety.enable_collision_guard,
                check_rate_hz=cfg.safety.check_rate_hz,
                warn_interval_sec=cfg.safety.warn_interval_sec,
            )
        else:
            print("[Setup] ⚠️ 安全门已禁用（无质心/碰撞保护）")

    def _self_check(self, expected_deg: np.ndarray) -> None:
        fb = self.arm.get_arm_state_deg()
        diff = np.max(np.abs(fb - expected_deg))
        if diff > 10.0:
            print(f"[Setup] ⚠️ 真机反馈与目标位姿最大偏差 {diff:.1f}°（>10°），请检查零位/方向标定")
        else:
            print(f"[Setup] 真机反馈与目标位姿最大偏差 {diff:.1f}°，一致")

    def _should_keep_running(self) -> bool:
        return super()._should_keep_running() and (self.ros is None or self.ros.ok())

    def _shutdown_robot(self) -> None:
        if self.recorder:
            self.recorder.stop()
        if self.chassis:
            self.chassis.stop()
        if self.safety:
            self.safety.stop()
        if self.ros:
            self.ros.shutdown()

    # ------------------------------------------------------------------ #
    # 输入
    # ------------------------------------------------------------------ #
    def _update_robot_state(self) -> None:
        """开环虚拟机器人：不回读真机反馈。"""

    def _read_arm_input(self, name: str) -> Tuple[bool, Optional[np.ndarray]]:
        if self._resetting.is_set():
            return False, None
        side = _SIDE[name]
        if self.external_input and self.external_input.enabled:
            return self.external_input.read(side)
        active, pose = super()._read_arm_input(name)
        if self.lead_pub:
            self.lead_pub.update_controller_pose(side, pose, active)
        return active, pose

    def _mode_switch_allowed(self) -> bool:
        return not self._head_active

    def _pre_ik_update(self) -> None:
        if self._resetting.is_set():
            return
        self._handle_buttons()
        self._update_head()
        self._update_chassis()

    # ------------------------------------------------------------------ #
    # 按键：复位 / 腰
    # ------------------------------------------------------------------ #
    def _handle_buttons(self) -> None:
        x = bool(self.xr_client.get_button_state_by_name("X"))
        y = bool(self.xr_client.get_button_state_by_name("Y"))
        if self._xy_edge(x and y):
            self._start_reset()
            return
        if self._y_hold(y and not x):
            self.waist.step(True)
        if self._x_hold(x and not y):
            self.waist.step(False)

    def _start_reset(self) -> None:
        if self._resetting.is_set():
            return
        self._resetting.set()
        threading.Thread(target=self._do_reset, name="reset", daemon=True).start()

    def _do_reset(self) -> None:
        try:
            print("[Reset] X+Y → 双臂复位")
            r = self.cfg.robot
            self.arm.move_to(self.reset_pose_deg, r.reset_duration_sec, r.move_rate_hz)
            self._sync_to_robot_pose(self.reset_pose_deg)
            self._self_check(self.reset_pose_deg)
            print("[Reset] 完成")
        except Exception as e:
            print(f"[Reset] 异常: {e!r}")
        finally:
            self._resetting.clear()

    # ------------------------------------------------------------------ #
    # 头部
    # ------------------------------------------------------------------ #
    @staticmethod
    def _axis(raw: float, deadzone: float, invert: bool) -> float:
        if abs(raw) < deadzone:
            return 0.0
        return -raw if invert else raw

    def _update_head(self) -> None:
        h = self.cfg.head
        rt = self.xr_client.get_key_value_by_name("right_trigger")
        lg = self.xr_client.get_key_value_by_name("left_grip")
        rg = self.xr_client.get_key_value_by_name("right_grip")
        self._head_active = rt >= h.trigger_threshold and lg < h.grip_release_threshold and rg < h.grip_release_threshold
        if not self._head_active:
            return
        lx, ly = self.xr_client.get_joystick_state("left")
        rx, _ = self.xr_client.get_joystick_state("right")
        axes = [
            self._axis(rx, h.deadzone, h.invert_yaw),     # head_j1 yaw
            self._axis(ly, h.deadzone, h.invert_pitch),   # head_j2 pitch
            self._axis(lx, h.deadzone, h.invert_roll),    # head_j3 roll
        ]
        if self.head.step_by(axes):
            self.head.publish()

    # ------------------------------------------------------------------ #
    # 底盘
    # ------------------------------------------------------------------ #
    def _update_chassis(self) -> None:
        c = self.cfg.chassis
        lx, ly = self.xr_client.get_joystick_state("left")
        rx, ry = self.xr_client.get_joystick_state("right")

        # 头部模式占用摇杆：底盘忽略使能/解除边沿并保持零速
        if self._head_active:
            self._base_up_edge(ry >= c.trigger_up)
            self._base_down_edge(ry <= -c.trigger_down)
            self.chassis.set_velocity_command(0.0, 0.0, 0.0)
            return

        if self._base_up_edge(ry >= c.trigger_up):
            self.chassis.enable()
        if self._base_down_edge(ry <= -c.trigger_down):
            self.chassis.disable()

        if self.chassis.enabled:
            vx = self._axis(ly, c.deadzone, c.invert_linear_x) * c.max_linear
            vy = self._axis(lx, c.deadzone, c.invert_linear_y) * c.max_linear
            wz = self._axis(-rx, c.deadzone, c.invert_angular_z) * c.max_angular   # 左拨 = 逆时针为正
            self.chassis.set_velocity_command(vx, vy, wz)

    # ------------------------------------------------------------------ #
    # 下发
    # ------------------------------------------------------------------ #
    def _limit_step(self, arm_deg: np.ndarray) -> np.ndarray:
        prev = self.arm.last_cmd_deg
        if prev is None:
            return arm_deg
        delta = arm_deg - prev
        clipped = np.clip(delta, -self._max_step_deg, self._max_step_deg)
        if self.prof_ctrl.enabled:
            self.prof_ctrl.note("max_step_deg", f"{np.max(np.abs(delta)):.3f}")
            self.prof_ctrl.note("clipped", int(np.any(clipped != delta)))
        return prev + clipped

    def _send_command(self) -> None:
        if not self._resetting.is_set() and any(self.active.values()):
            arm_deg = self._placo_to_deg14()
            if self.safety:
                arm_deg = self.safety.filter(arm_deg)
            self.arm.publish_arm_cmd(self._limit_step(arm_deg))
            self.prof_ctrl.note("pub", 1)
            if self._gripper_limiter.ready():
                self.arm.publish_gripper_cmd(
                    self.gripper_pos_target["left_arm"], self.gripper_pos_target["right_arm"]
                )
        self.chassis.publish()

    # ------------------------------------------------------------------ #
    # 辅助线程（30Hz）：数采按键 + 主端数据镜像
    # ------------------------------------------------------------------ #
    def _aux_update(self) -> None:
        if self._a_edge(bool(self.xr_client.get_button_state_by_name("A"))):
            self.recorder.start()
        if self._b_edge(bool(self.xr_client.get_button_state_by_name("B"))):
            self.recorder.stop()
        self.lead_pub.publish(self.arm.last_cmd_deg, self.head.get_target())
