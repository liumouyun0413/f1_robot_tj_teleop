"""
轻量级阶段计时器（F1_PROFILE=1 时启用），用于定位遥操链路卡顿。

每个线程在循环里调用 ``prof.mark(name)`` 打点，``prof.end_loop()`` 收尾；
周期性把 CSV 行写到 ``F1_PROFILE_DIR``（默认 /tmp/f1_profile）。
未启用时所有方法为空操作，开销可忽略。
"""

from __future__ import annotations

import os
import threading
import time
from typing import Dict, List

_ENABLED = os.environ.get("F1_PROFILE", "0") == "1"
_DIR = os.environ.get("F1_PROFILE_DIR", "/tmp/f1_profile")


class StageProfiler:
    def __init__(self, name: str, stages: List[str], flush_every: int = 200) -> None:
        self.enabled = _ENABLED
        self.name = name
        self.stages = stages
        self._rows: List[str] = []
        self._flush_every = flush_every
        self._lock = threading.Lock()
        self._t_loop_start = 0.0
        self._t_prev = 0.0
        self._t_last_loop = 0.0
        self._cur: Dict[str, float] = {}
        self._extra: Dict[str, str] = {}
        if self.enabled:
            os.makedirs(_DIR, exist_ok=True)
            self._path = os.path.join(_DIR, f"{name}.csv")
            with open(self._path, "w") as f:
                f.write("t_wall,period_ms," + ",".join(f"{s}_ms" for s in stages) + ",extra\n")
            print(f"[Profile] {name} → {self._path}")

    def begin_loop(self) -> None:
        if not self.enabled:
            return
        now = time.perf_counter()
        self._t_loop_start = now
        self._t_prev = now
        self._cur = {}
        self._extra = {}

    def mark(self, stage: str) -> None:
        """记录从上一个 mark（或 begin_loop）到现在的耗时，归到 stage。"""
        if not self.enabled:
            return
        now = time.perf_counter()
        self._cur[stage] = self._cur.get(stage, 0.0) + (now - self._t_prev) * 1e3
        self._t_prev = now

    def note(self, key: str, value) -> None:
        if self.enabled:
            self._extra[key] = str(value)

    def end_loop(self) -> None:
        if not self.enabled:
            return
        now = time.perf_counter()
        period = (now - self._t_last_loop) * 1e3 if self._t_last_loop else 0.0
        self._t_last_loop = now
        row = (
            f"{time.time():.6f},{period:.3f},"
            + ",".join(f"{self._cur.get(s, 0.0):.3f}" for s in self.stages)
            + ","
            + ";".join(f"{k}={v}" for k, v in self._extra.items())
        )
        with self._lock:
            self._rows.append(row)
            if len(self._rows) >= self._flush_every:
                self._flush()

    def _flush(self) -> None:
        if not self._rows:
            return
        with open(self._path, "a") as f:
            f.write("\n".join(self._rows) + "\n")
        self._rows.clear()

    def close(self) -> None:
        if self.enabled:
            with self._lock:
                self._flush()
