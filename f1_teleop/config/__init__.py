"""
配置加载：把 f1_teleop.yaml 解析为嵌套的只读命名空间对象。

用法::

    cfg = load_config()                 # 默认 f1_teleop/config/f1_teleop.yaml
    cfg.teleop.control_rate_hz          # 属性式访问
    cfg.robot.joint_map.left.sign       # 嵌套访问
    cfg.robot.urdf_path                 # 已解析为绝对路径
"""

from __future__ import annotations

import os
from types import SimpleNamespace
from typing import Any

import yaml

from f1_teleop.utils.path_utils import PROJECT_ROOT, DEFAULT_CONFIG_PATH


class ConfigError(RuntimeError):
    pass


def _to_namespace(obj: Any) -> Any:
    if isinstance(obj, dict):
        return SimpleNamespace(**{k: _to_namespace(v) for k, v in obj.items()})
    if isinstance(obj, list):
        return [_to_namespace(v) for v in obj]
    return obj


def _require(ns: SimpleNamespace, path: str) -> None:
    cur: Any = ns
    for key in path.split("."):
        if not hasattr(cur, key):
            raise ConfigError(f"配置缺少必需项: {path}")
        cur = getattr(cur, key)


_REQUIRED = (
    "robot.urdf_path",
    "robot.left_joint_names",
    "robot.right_joint_names",
    "robot.head_joint_names",
    "robot.body_joint_names",
    "robot.urdf_left_joint_names",
    "robot.urdf_right_joint_names",
    "robot.left_ee_link",
    "robot.right_ee_link",
    "robot.joint_map.left.sign",
    "robot.joint_map.left.offset_deg",
    "robot.joint_map.right.sign",
    "robot.joint_map.right.offset_deg",
    "robot.home_pose.left",
    "robot.home_pose.right",
    "robot.reset_pose.left",
    "robot.reset_pose.right",
    "ros.joint_cmd_topic",
    "ros.joint_state_topic",
    "teleop.control_rate_hz",
    "head.limits_deg",
    "chassis.move_topic",
    "chassis.control_mode_service",
    "safety.module_dir",
    "recording.topics",
)


def load_config(path: str | None = None) -> SimpleNamespace:
    path = os.path.abspath(os.path.expanduser(path or DEFAULT_CONFIG_PATH))
    if not os.path.isfile(path):
        raise ConfigError(f"配置文件不存在: {path}")
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ConfigError(f"配置文件格式非法（应为键值映射）: {path}")

    cfg = _to_namespace(raw)
    for item in _REQUIRED:
        _require(cfg, item)

    # 相对路径按项目根目录解析
    urdf = os.path.expanduser(cfg.robot.urdf_path)
    if not os.path.isabs(urdf):
        urdf = os.path.join(PROJECT_ROOT, urdf)
    cfg.robot.urdf_path = os.path.abspath(urdf)
    if not os.path.isfile(cfg.robot.urdf_path):
        raise ConfigError(f"URDF 不存在: {cfg.robot.urdf_path}")

    bag_dir = os.path.expanduser(cfg.recording.bag_dir)
    if not os.path.isabs(bag_dir):
        bag_dir = os.path.join(PROJECT_ROOT, bag_dir)
    cfg.recording.bag_dir = os.path.abspath(bag_dir)
    cfg.safety.module_dir = os.path.abspath(os.path.expanduser(cfg.safety.module_dir))
    cfg.config_path = path
    return cfg
