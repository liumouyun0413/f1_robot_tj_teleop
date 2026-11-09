"""
BaseTeleopController —— 与机器人无关的遥操作核心（Placo IK）。

结构沿用 XRoboToolkit-Teleop-Sample-Python 的 BaseTeleopController，并吸收
lxl_0918 在 F1 上验证过的几项改动：
  * 三种末端控制模式（position / orientation / pose），摇杆按压切换；
  * 每帧刷新关节正则化目标 + 非臂关节硬约束固定，抑制 IK 漂移与跳解；
  * 双臂输入来源抽象为 ``_read_arm_input``，便于用外部位姿（回放）替代手柄；
  * ``sync_arm_joints`` 把 Placo 构型对齐到真机并重置所有参考，供启动/复位使用。

子类需实现：_robot_setup / _update_robot_state / _send_command / run。
"""

from __future__ import annotations

import abc
import threading
import webbrowser
from typing import Any, Dict, Optional, Tuple

import meshcat.transformations as tf
import numpy as np
import placo

from f1_teleop.common.xr_client import XrClient
from f1_teleop.utils.control_utils import RisingEdge, calc_parallel_gripper_position
from f1_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)

MODE_POSITION = "position"
MODE_ORIENTATION = "orientation"
MODE_POSE = "pose"


class BaseTeleopController(abc.ABC):
    """
    manipulator_config 结构（每个末端一项）::

        {
          "left_arm": {
            "link_name":       "arm_L_Link7",          # 末端 link
            "joint_names":     ["arm_L_joint1", ...],  # 该臂在 URDF 中的关节（IK 可动关节）
            "shoulder_link":   "arm_L_Link1",          # 可选，球形工作空间球心
            "pose_source":     "left_controller",
            "control_trigger": "left_grip",
            "gripper_config":  {"gripper_trigger": "left_trigger", "open_pos": 100.0, "close_pos": 0.0},
          },
          ...
        }
    """

    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: Dict[str, Dict[str, Any]],
        floating_base: bool = False,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        q_init: Optional[np.ndarray] = None,
        dt: float = 0.01,
        grip_activate_threshold: float = 0.9,
        frame_task_weight: float = 10.0,
        manipulability_weight: float = 1e-4,
        joint_regularization_weight: float = 1.0,
        workspace_radius: Optional[float] = None,
    ):
        self.robot_urdf_path = robot_urdf_path
        self.manipulator_config = manipulator_config
        self.floating_base = floating_base
        self.R_headset_world = R_headset_world
        self.scale_factor = scale_factor
        self.q_init = q_init
        self.dt = dt
        self.grip_activate_threshold = grip_activate_threshold
        self.frame_task_weight = frame_task_weight
        self.manipulability_weight = manipulability_weight
        self.joint_regularization_weight = joint_regularization_weight
        self.workspace_radius = workspace_radius

        self.xr_client = XrClient()

        names = list(manipulator_config.keys())
        self.ref_ee_xyz: Dict[str, Optional[np.ndarray]] = {n: None for n in names}
        self.ref_ee_quat: Dict[str, Optional[np.ndarray]] = {n: None for n in names}
        self.ref_controller_xyz: Dict[str, Optional[np.ndarray]] = {n: None for n in names}
        self.ref_controller_quat: Dict[str, Optional[np.ndarray]] = {n: None for n in names}
        self.active: Dict[str, bool] = {n: False for n in names}
        self.control_mode: Dict[str, str] = {n: MODE_POSE for n in names}
        self.effector_task: Dict[str, Any] = {}
        self.arm_q_slice: Dict[str, slice] = {}

        self.gripper_pos_target: Dict[str, float] = {}
        for name, config in manipulator_config.items():
            gc = config.get("gripper_config")
            if gc:
                self.gripper_pos_target[name] = float(gc["open_pos"])

        self._mode_left_edge = RisingEdge()
        self._mode_right_edge = RisingEdge()
        self._stop_event = threading.Event()
        self._placo_lock = threading.Lock()   # 保护跨线程读写 placo state.q

        self._placo_setup()

    # ------------------------------------------------------------------ #
    # Placo
    # ------------------------------------------------------------------ #
    def _placo_setup(self) -> None:
        self.placo_robot = placo.RobotWrapper(self.robot_urdf_path)
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.dt

        if self.q_init is not None:
            if self.floating_base:
                self.placo_robot.state.q = self.q_init.copy()
            else:
                self.solver.mask_fbase(True)
                self.placo_robot.state.q[7:] = self.q_init.copy()
        else:
            if not self.floating_base:
                self.solver.mask_fbase(True)
            self.placo_robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])
        self.placo_robot.update_kinematics()

        arm_q_indices: set[int] = set()
        for name, config in self.manipulator_config.items():
            jn = config["joint_names"]
            offsets = [self.placo_robot.get_joint_offset(j) for j in jn]
            if offsets != list(range(offsets[0], offsets[0] + len(offsets))):
                raise RuntimeError(f"{name} 的关节在 q 中不连续: {dict(zip(jn, offsets))}")
            self.arm_q_slice[name] = slice(offsets[0], offsets[-1] + 1)
            arm_q_indices.update(offsets)

            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            ee_target = tf.quaternion_matrix(ee_quat)
            ee_target[:3, 3] = ee_xyz
            task = self.solver.add_frame_task(config["link_name"], ee_target)
            task.configure(name, "soft", self.frame_task_weight)
            self.effector_task[name] = task

            manip = self.solver.add_manipulability_task(config["link_name"], "both", 1.0)
            manip.configure(f"manipulability_{name}", "soft", self.manipulability_weight)

        # 关节正则化：每帧刷新到当前构型，抑制 solver 跳到另一组解
        self.joints_regularization_task = self.solver.add_joints_task()
        self.joints_regularization_task.set_joints(self._get_joint_values_dict())
        self.joints_regularization_task.configure(
            "joints_regularization", "soft", self.joint_regularization_weight
        )

        # 非臂关节（腰、头、轮）硬约束固定，IK 只动双臂
        fixed = {}
        for joint_id in range(self.placo_robot.model.njoints):
            q_idx = self.placo_robot.model.idx_qs[joint_id]
            if q_idx >= 7 and q_idx not in arm_q_indices:
                fixed[self.placo_robot.model.names[joint_id]] = float(self.placo_robot.state.q[q_idx])
        if fixed:
            self.fixed_joints_task = self.solver.add_joints_task()
            self.fixed_joints_task.set_joints(fixed)
            self.fixed_joints_task.configure("fixed_joints", "hard", 1.0)

        self.placo_robot.update_kinematics()
        print(f"[Placo] 已加载 {self.robot_urdf_path}")
        for name, sl in self.arm_q_slice.items():
            print(f"[Placo] {name}: q[{sl.start}:{sl.stop}] → {self.manipulator_config[name]['link_name']}")
        print(f"[Placo] 固定非臂关节 {len(fixed)} 个")

    def _get_joint_values_dict(self) -> Dict[str, float]:
        values = {}
        for joint_id in range(self.placo_robot.model.njoints):
            q_idx = self.placo_robot.model.idx_qs[joint_id]
            if q_idx >= 7:
                values[self.placo_robot.model.names[joint_id]] = float(self.placo_robot.state.q[q_idx])
        return values

    def _get_link_pose(self, link_name: str) -> Tuple[np.ndarray, np.ndarray]:
        T = self.placo_robot.get_T_world_frame(link_name)
        return T[:3, 3].copy(), tf.quaternion_from_matrix(T)

    def get_arm_q(self, name: str) -> np.ndarray:
        with self._placo_lock:
            return self.placo_robot.state.q[self.arm_q_slice[name]].copy()

    def sync_arm_joints(self, q_rad_by_arm: Dict[str, np.ndarray]) -> None:
        """把 Placo 双臂构型对齐到给定关节角（弧度），并重置所有遥操参考。"""
        with self._placo_lock:
            for name, q in q_rad_by_arm.items():
                self.placo_robot.state.q[self.arm_q_slice[name]] = np.asarray(q, dtype=float)
            self.placo_robot.update_kinematics()
            for name, config in self.manipulator_config.items():
                ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                T = tf.quaternion_matrix(ee_quat)
                T[:3, 3] = ee_xyz
                self.effector_task[name].T_world_frame = T
                self.ref_ee_xyz[name] = None
                self.ref_ee_quat[name] = None
                self.ref_controller_xyz[name] = None
                self.ref_controller_quat[name] = None
            self.joints_regularization_task.set_joints(self._get_joint_values_dict())
        print("[Placo] 构型已同步，参考位姿已重置")

    # ------------------------------------------------------------------ #
    # 输入处理
    # ------------------------------------------------------------------ #
    def _read_arm_input(self, name: str) -> Tuple[bool, Optional[np.ndarray]]:
        """返回 (是否激活, 手柄位姿 [x,y,z,qx,qy,qz,qw])。子类可覆盖以替换输入源。"""
        config = self.manipulator_config[name]
        grip = self.xr_client.get_key_value_by_name(config["control_trigger"])
        if grip <= self.grip_activate_threshold:
            return False, None
        return True, np.asarray(self.xr_client.get_pose_by_name(config["pose_source"]), dtype=float)

    def _process_xr_pose(self, xr_pose: np.ndarray, name: str) -> Tuple[np.ndarray, np.ndarray]:
        controller_xyz = np.array(xr_pose[0:3], dtype=float)
        controller_quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]], dtype=float)
        norm = np.linalg.norm(controller_quat)
        controller_quat = controller_quat / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0, 0.0])

        controller_xyz = self.R_headset_world @ controller_xyz
        R4 = np.eye(4)
        R4[:3, :3] = self.R_headset_world
        R_quat = tf.quaternion_from_matrix(R4)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat), tf.quaternion_conjugate(R_quat)
        )

        if self.ref_controller_xyz[name] is None:
            self.ref_controller_xyz[name] = controller_xyz
            self.ref_controller_quat[name] = controller_quat
            return np.zeros(3), np.zeros(3)

        delta_xyz = (controller_xyz - self.ref_controller_xyz[name]) * self.scale_factor
        delta_rot = quat_diff_as_angle_axis(self.ref_controller_quat[name], controller_quat)
        return delta_xyz, delta_rot

    # ------------------------------------------------------------------ #
    # 控制模式
    # ------------------------------------------------------------------ #
    def _mode_switch_allowed(self) -> bool:
        """子类可覆盖：某些状态（如头部控制占用摇杆）下禁止切换模式。"""
        return True

    def _update_control_mode(self) -> None:
        lc = bool(self.xr_client.get_button_state_by_name("left_axis_click"))
        rc = bool(self.xr_client.get_button_state_by_name("right_axis_click"))
        left_edge = self._mode_left_edge(lc)
        right_edge = self._mode_right_edge(rc)
        if not (left_edge or right_edge) or not self._mode_switch_allowed():
            return

        new_mode = MODE_POSE if (lc and rc) else (MODE_POSITION if lc else MODE_ORIENTATION)
        old_mode = next(iter(self.control_mode.values()))
        if new_mode == old_mode:
            return
        for name in self.control_mode:
            self.control_mode[name] = new_mode
        print(f"[Mode] 控制模式切换: {old_mode} → {new_mode}")
        # 切换瞬间重新锚定，避免 delta 被新模式误读导致末端跳变
        for name, config in self.manipulator_config.items():
            if self.ref_ee_xyz[name] is not None:
                self.ref_ee_xyz[name], self.ref_ee_quat[name] = self._get_link_pose(config["link_name"])
            self.ref_controller_xyz[name] = None
            self.ref_controller_quat[name] = None

    def _apply_effector_target(self, name: str, delta_xyz: np.ndarray, delta_rot: np.ndarray) -> None:
        mode = self.control_mode[name]
        ref_xyz, ref_quat = self.ref_ee_xyz[name], self.ref_ee_quat[name]

        if mode == MODE_POSITION:
            target_xyz, target_quat = ref_xyz + delta_xyz, ref_quat
        elif mode == MODE_ORIENTATION:
            target_xyz = ref_xyz.copy()
            _, target_quat = apply_delta_pose(ref_xyz, ref_quat, np.zeros(3), delta_rot)
        else:
            target_xyz, target_quat = apply_delta_pose(ref_xyz, ref_quat, delta_xyz, delta_rot)

        target_xyz = self._clip_to_workspace(name, np.asarray(target_xyz, dtype=float))

        q_norm = np.linalg.norm(target_quat)
        if not np.isfinite(q_norm) or abs(q_norm - 1.0) > 0.01:
            target_quat = ref_quat

        T = tf.quaternion_matrix(target_quat)
        T[:3, 3] = target_xyz
        self.effector_task[name].T_world_frame = T

    def _clip_to_workspace(self, name: str, target_xyz: np.ndarray) -> np.ndarray:
        """以肩部 link 为球心的球形工作空间限制（workspace_radius 为 None 时不限制）。"""
        if self.workspace_radius is None:
            return target_xyz
        shoulder = self.manipulator_config[name].get("shoulder_link")
        if not shoulder:
            return target_xyz
        center = self.placo_robot.get_T_world_frame(shoulder)[:3, 3]
        rel = target_xyz - center
        dist = np.linalg.norm(rel)
        if dist > self.workspace_radius:
            return center + rel / dist * self.workspace_radius
        return target_xyz

    # ------------------------------------------------------------------ #
    # IK 主流程
    # ------------------------------------------------------------------ #
    def _update_ik(self) -> None:
        with self._placo_lock:
            self._update_robot_state()
            self.placo_robot.update_kinematics()
            self._update_control_mode()

            for name, config in self.manipulator_config.items():
                active, xr_pose = self._read_arm_input(name)
                self.active[name] = active

                if active:
                    if self.ref_ee_xyz[name] is None:
                        print(f"[Teleop] {name} 激活")
                        self.ref_ee_xyz[name], self.ref_ee_quat[name] = self._get_link_pose(config["link_name"])
                    delta_xyz, delta_rot = self._process_xr_pose(xr_pose, name)
                    self._apply_effector_target(name, delta_xyz, delta_rot)
                else:
                    if self.ref_ee_xyz[name] is not None:
                        print(f"[Teleop] {name} 松开")
                        self.ref_ee_xyz[name] = None
                        self.ref_ee_quat[name] = None
                        self.ref_controller_xyz[name] = None
                        self.ref_controller_quat[name] = None
                        # 末端任务回到当前位姿，松开后不再漂移
                        ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                        T = tf.quaternion_matrix(ee_quat)
                        T[:3, 3] = ee_xyz
                        self.effector_task[name].T_world_frame = T

            self.joints_regularization_task.set_joints(self._get_joint_values_dict())
            try:
                self.solver.solve(True)
            except RuntimeError as e:
                print(f"[Placo] IK solver failed: {e}")

    def _update_gripper_target(self) -> None:
        for name, config in self.manipulator_config.items():
            gc = config.get("gripper_config")
            if not gc:
                continue
            trigger = self.xr_client.get_key_value_by_name(gc["gripper_trigger"])
            self.gripper_pos_target[name] = calc_parallel_gripper_position(
                float(gc["open_pos"]), float(gc["close_pos"]), trigger
            )

    # ------------------------------------------------------------------ #
    # 可视化
    # ------------------------------------------------------------------ #
    def _init_placo_viz(self) -> None:
        from placo_utils.visualization import frame_viz, robot_frame_viz, robot_viz

        self._viz_frame = frame_viz
        self._viz_robot_frame = robot_frame_viz
        self.placo_vis = robot_viz(self.placo_robot)
        url = self.placo_vis.viewer.url()
        print(f"[Viz] Meshcat: {url}")
        webbrowser.open(url)
        self._update_placo_viz()

    def _update_placo_viz(self) -> None:
        with self._placo_lock:
            self.placo_vis.display(self.placo_robot.state.q)
            for name, config in self.manipulator_config.items():
                self._viz_robot_frame(self.placo_robot, config["link_name"])
                self._viz_frame(f"vis_target_{name}", self.effector_task[name].T_world_frame)

    # ------------------------------------------------------------------ #
    # 子类必须实现
    # ------------------------------------------------------------------ #
    @abc.abstractmethod
    def _robot_setup(self) -> None:
        """初始化具体后端（连接机器人 / 启动仿真等）。"""

    @abc.abstractmethod
    def _update_robot_state(self) -> None:
        """把机器人当前状态写入 self.placo_robot.state.q（开环虚拟机器人可为空实现）。"""

    @abc.abstractmethod
    def _send_command(self) -> None:
        """把 self.placo_robot.state.q 中的目标下发到机器人。"""

    @abc.abstractmethod
    def run(self) -> None:
        """主入口。"""
