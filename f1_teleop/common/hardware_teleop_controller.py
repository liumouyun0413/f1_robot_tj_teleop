"""
HardwareTeleopController —— 真机遥操作多线程骨架。

与 XRoboToolkit-Teleop-Sample-Python 的 HardwareTeleopController 对应：
  * _ik_thread      : 读输入 → _pre_ik_update → IK → 可视化
  * _control_thread : _send_command 下发
  * _aux_thread     : 低频辅助（数采按键、镜像发布等），由子类通过 _aux_update 实现
  * run()           : _robot_setup → 启动线程 → 监控 → 收尾 _shutdown_robot
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from f1_teleop.common.base_teleop_controller import BaseTeleopController
from f1_teleop.utils.control_utils import RateLimiter, sleep_until_period
from f1_teleop.utils.geometry import R_HEADSET_TO_WORLD


class HardwareTeleopController(BaseTeleopController, ABC):
    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        floating_base: bool = False,
        scale_factor: float = 1.0,
        q_init: Optional[np.ndarray] = None,
        visualize_placo: bool = False,
        control_rate_hz: int = 100,
        aux_rate_hz: float = 30.0,
        **ik_kwargs,
    ):
        super().__init__(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            floating_base=floating_base,
            R_headset_world=R_headset_world,
            scale_factor=scale_factor,
            q_init=q_init,
            dt=1.0 / control_rate_hz,
            **ik_kwargs,
        )
        self.control_rate_hz = control_rate_hz
        self.aux_rate_hz = aux_rate_hz
        self.visualize_placo = visualize_placo
        self._viz_limiter = RateLimiter(30.0)
        if self.visualize_placo:
            self._init_placo_viz()

    # ------------------------------------------------------------------ #
    # 子类钩子
    # ------------------------------------------------------------------ #
    @abstractmethod
    def _shutdown_robot(self) -> None:
        """安全停机（停底盘、停线程等）。"""

    def _pre_ik_update(self) -> None:
        """IK 之前的逻辑（按键、底盘/头部等），默认空。"""

    def _aux_update(self) -> None:
        """辅助线程每拍调用一次（低频发布 / 数采按键），默认空。"""

    def _should_keep_running(self) -> bool:
        return not self._stop_event.is_set()

    # ------------------------------------------------------------------ #
    # 线程
    # ------------------------------------------------------------------ #
    def _ik_thread(self, stop_event: threading.Event) -> None:
        period = 1.0 / self.control_rate_hz
        while not stop_event.is_set():
            t0 = time.perf_counter()
            try:
                self._update_gripper_target()
                self._pre_ik_update()
                self._update_ik()
                if self.visualize_placo and self._viz_limiter.ready():
                    self._update_placo_viz()
            except Exception as e:  # 单帧异常不终止线程
                print(f"[IK] 异常: {e!r}")
            sleep_until_period(t0, period)
        print("[IK] 线程退出")

    def _control_thread(self, stop_event: threading.Event) -> None:
        period = 1.0 / self.control_rate_hz
        while not stop_event.is_set():
            t0 = time.perf_counter()
            try:
                self._send_command()
            except Exception as e:
                print(f"[Control] 异常: {e!r}")
            sleep_until_period(t0, period)
        print("[Control] 线程退出")

    def _aux_thread(self, stop_event: threading.Event) -> None:
        period = 1.0 / self.aux_rate_hz
        while not stop_event.is_set():
            t0 = time.perf_counter()
            try:
                self._aux_update()
            except Exception as e:
                print(f"[Aux] 异常: {e!r}")
            sleep_until_period(t0, period)
        print("[Aux] 线程退出")

    # ------------------------------------------------------------------ #
    # 主入口
    # ------------------------------------------------------------------ #
    def run(self) -> None:
        self._robot_setup()
        self._stop_event.clear()

        threads = [
            threading.Thread(name="ik", target=self._ik_thread, args=(self._stop_event,), daemon=True),
            threading.Thread(name="control", target=self._control_thread, args=(self._stop_event,), daemon=True),
            threading.Thread(name="aux", target=self._aux_thread, args=(self._stop_event,), daemon=True),
        ]
        for t in threads:
            t.start()

        print("[Teleop] 运行中，Ctrl+C 退出")
        try:
            while self._should_keep_running():
                if not all(t.is_alive() for t in threads):
                    print("[Teleop] 有线程异常退出，准备停机")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[Teleop] 收到中断")
        finally:
            self._stop_event.set()
            for t in threads:
                t.join(timeout=2.0)
            self._shutdown_robot()
            print("[Teleop] 已退出")
