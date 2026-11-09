"""小工具：平行夹爪映射、关节映射、按键边沿检测、定频循环。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def calc_parallel_gripper_position(open_pos: float, close_pos: float, percentage: float) -> float:
    """trigger 百分比 (0~1) → 夹爪指令值（线性插值）。"""
    percentage = float(np.clip(percentage, 0.0, 1.0))
    return open_pos + (close_pos - open_pos) * percentage


@dataclass(frozen=True)
class JointMap:
    """Placo 弧度 ↔ 真机度 的逐关节线性映射：deg = sign * rad2deg(rad) + offset。"""

    sign: np.ndarray
    offset_deg: np.ndarray

    @classmethod
    def from_lists(cls, sign: Sequence[float], offset_deg: Sequence[float]) -> "JointMap":
        s = np.asarray(sign, dtype=float)
        o = np.asarray(offset_deg, dtype=float)
        if s.shape != o.shape or not np.all(np.abs(s) == 1.0):
            raise ValueError("JointMap: sign 必须全为 ±1 且与 offset_deg 等长")
        return cls(sign=s, offset_deg=o)

    def rad_to_deg(self, q_rad: Sequence[float]) -> np.ndarray:
        return self.sign * np.rad2deg(np.asarray(q_rad, dtype=float)) + self.offset_deg

    def deg_to_rad(self, q_deg: Sequence[float]) -> np.ndarray:
        return np.deg2rad((np.asarray(q_deg, dtype=float) - self.offset_deg) / self.sign)


class RisingEdge:
    """布尔信号上升沿检测器。"""

    def __init__(self) -> None:
        self._prev = False

    def __call__(self, now: bool) -> bool:
        edge = bool(now) and not self._prev
        self._prev = bool(now)
        return edge


class HoldTrigger:
    """按住超过 hold_sec 才触发一次；松开后复位。用于区分单键长按与组合键。"""

    def __init__(self, hold_sec: float) -> None:
        self.hold_sec = hold_sec
        self._t0: float | None = None
        self._fired = False

    def __call__(self, pressed: bool) -> bool:
        if not pressed:
            self._t0 = None
            self._fired = False
            return False
        if self._t0 is None:
            self._t0 = time.time()
            return False
        if not self._fired and time.time() - self._t0 >= self.hold_sec:
            self._fired = True
            return True
        return False


class RateLimiter:
    """按固定频率放行：在高频循环中把某些发布降到较低频率。"""

    def __init__(self, hz: float) -> None:
        self.period = 1.0 / float(hz) if hz > 0 else 0.0
        self._next = 0.0

    def ready(self) -> bool:
        now = time.monotonic()
        if now >= self._next:
            self._next = now + self.period
            return True
        return False


def sleep_until_period(loop_start: float, period: float) -> None:
    remaining = period - (time.perf_counter() - loop_start)
    if remaining > 0:
        time.sleep(remaining)
