"""
SafetyGuard —— 质心 / 碰撞安全门（主进程侧客户端）。

工作方式：
  * 控制线程每拍调用 filter(arm_cmd_deg)：
      - 把原始目标记为"待检查"；
      - 若当前处于冻结状态，用最近一次通过检查的指令替代目标；
      - 否则原样放行。
  * 内部线程以 check_rate_hz 把"待检查目标 + 真机头/腰反馈"组成 24 维命令
    送子进程检查；通过则更新"最近安全指令"，否则置冻结标志。
    因此冻结时回退到的是【经过验证的】安全位置，而非最后一帧未验证指令。
  * 质心越界无法归因到单臂，任一不安全即冻结双臂。

24 维命令布局（与 cyb SafetyProtector 一致，弧度）：
    [0:7] 左臂  [7:14] 右臂  [14:17] 头 j1..3  [17:22] 腰 j1..5  [22:24] 夹爪（不消费）
"""

from __future__ import annotations

import multiprocessing
import threading
import time
from typing import Callable, Optional, Sequence

import numpy as np

from f1_teleop.hardware.interface.safety_worker import worker_main


class SafetyGuard:
    def __init__(
        self,
        module_dir: str,
        body_state_deg_getter: Callable[[], np.ndarray],
        initial_cmd_deg: Sequence[float],
        enable_com_guard: bool = True,
        enable_collision_guard: bool = True,
        check_rate_hz: float = 15.0,
        warn_interval_sec: float = 3.0,
    ) -> None:
        """
        body_state_deg_getter : 返回 8 维 [head_j1..3, body_j1..5]（度）
        initial_cmd_deg       : 24 维初始命令（度），用于子进程首帧安全回退
        """
        self._get_body_deg = body_state_deg_getter
        self.check_rate_hz = check_rate_hz
        self.warn_interval_sec = warn_interval_sec

        self._conn, worker_conn = multiprocessing.Pipe()
        self._proc = multiprocessing.Process(
            target=worker_main,
            args=(worker_conn, module_dir, enable_com_guard, enable_collision_guard),
            daemon=True,
            name="SafetyWorker",
        )
        self._proc.start()
        self._conn.send(("init", np.deg2rad(np.asarray(initial_cmd_deg, dtype=float))))
        kind, payload = self._conn.recv()       # 阻塞等子进程加载 MuJoCo 模型（数秒）
        if kind == "error":
            raise RuntimeError(payload)
        print(f"[Safety] 子进程就绪 | 质心保护={enable_com_guard} 碰撞检测={enable_collision_guard}")

        self._lock = threading.Lock()
        self._pending: Optional[np.ndarray] = None     # 待检查的 14 维原始目标（度）
        self._last_safe: Optional[np.ndarray] = None   # 最近一次放行的 14 维指令（度）
        self.frozen = False
        self._last_warn = 0.0
        self.reject_count = 0

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="safety_check", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    def filter(self, arm_cmd_deg: Sequence[float]) -> np.ndarray:
        """返回实际应下发的双臂指令：冻结时为最近安全指令，否则原样放行。"""
        raw = np.asarray(arm_cmd_deg, dtype=float)
        with self._lock:
            self._pending = raw.copy()
            if self._last_safe is None:
                self._last_safe = raw.copy()
            if self.frozen:
                return self._last_safe.copy()
            return raw

    def reset_last_safe(self, arm_cmd_deg: Sequence[float]) -> None:
        """复位/同步后重置安全回退点，避免解冻时跳回旧位置。"""
        with self._lock:
            self._last_safe = np.asarray(arm_cmd_deg, dtype=float).copy()
            self._pending = None
            self.frozen = False

    # ------------------------------------------------------------------ #
    def _loop(self) -> None:
        period = 1.0 / self.check_rate_hz
        while not self._stop.is_set():
            t0 = time.perf_counter()
            self._check_once()
            remaining = period - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)

    def _check_once(self) -> None:
        with self._lock:
            pending = None if self._pending is None else self._pending.copy()
        if pending is None:
            return

        cmd = np.zeros(24, dtype=float)
        cmd[:14] = pending
        cmd[14:22] = self._get_body_deg()

        try:
            self._conn.send(("run", np.deg2rad(cmd)))
            _, (is_safe, indices) = self._conn.recv()
        except Exception as e:
            print(f"[Safety] 检查异常，暂时放行: {e!r}")
            with self._lock:
                self.frozen = False
            return

        with self._lock:
            self.frozen = not is_safe
            if is_safe:
                self._last_safe = pending
        if not is_safe:
            self.reject_count += 1
            now = time.time()
            if now - self._last_warn >= self.warn_interval_sec:
                self._last_warn = now
                print(f"[Safety] 危险（碰撞或质心越界），双臂冻结 | 碰撞关节索引: {indices}")

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._conn.send(None)
        except Exception:
            pass
        self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            self._proc.terminate()
