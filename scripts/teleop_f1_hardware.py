#!/usr/bin/env python3
"""
F1 轮式人形机器人 PICO 遥操作入口（对应官方 scripts/hardware/teleop_r1lite_hardware.py）。

    python scripts/teleop_f1_hardware.py
    python scripts/teleop_f1_hardware.py --config /path/to/f1_teleop.yaml --visualize-placo
    python scripts/teleop_f1_hardware.py --no-safety        # 调试：关闭质心/碰撞保护
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import tyro  # noqa: E402

from f1_teleop.config import load_config  # noqa: E402
from f1_teleop.hardware.f1_teleop_controller import F1TeleopController  # noqa: E402


def main(
    config: str | None = None,
    visualize_placo: bool = False,
    scale_factor: float | None = None,
    control_rate_hz: int | None = None,
    no_safety: bool = False,
    no_recording: bool = False,
) -> None:
    """
    Args:
        config:           配置文件路径，默认 f1_teleop/config/f1_teleop.yaml
        visualize_placo:  打开 Meshcat 可视化 Placo 模型
        scale_factor:     覆盖配置中的手柄位移放大倍数
        control_rate_hz:  覆盖配置中的控制频率
        no_safety:        禁用质心/碰撞安全门（仅调试）
        no_recording:     禁用 ros2 bag 录制
    """
    cfg = load_config(config)
    if scale_factor is not None:
        cfg.teleop.scale_factor = scale_factor
    if control_rate_hz is not None:
        cfg.teleop.control_rate_hz = control_rate_hz
    if no_safety:
        cfg.safety.enable = False
    if no_recording:
        cfg.recording.enable = False

    print(f"[Main] 配置: {cfg.config_path}")
    print(f"[Main] URDF: {cfg.robot.urdf_path}")
    controller = F1TeleopController(cfg, visualize_placo=visualize_placo)
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
