"""
质心保护 + 碰撞检测子进程入口（持有 cyb SafetyProtector）。

SafetyProtector 单次检查约 100ms 且是纯 Python 计算，放在遥操进程内会长期占用
GIL 拖慢 IK；因此由独立子进程持有，主进程通过 Pipe 发送目标、阻塞等待结果
（recv 期间释放 GIL）。

Pipe 协议（父 → 子）：
    ("init", cmd24_rad) → ("init_ok", None)
    ("run",  cmd24_rad) → ("result", (is_safe: bool, arm_collision_indices: list[int]))
    None                → 退出
初始化失败 → ("error", str)
"""

from __future__ import annotations

import os
import sys


def worker_main(conn, module_dir: str, use_com_guard: bool, use_collision_guard: bool) -> None:
    try:
        os.nice(5)
    except Exception:
        pass

    try:
        if module_dir not in sys.path:
            sys.path.insert(0, module_dir)
        from com_and_collision_protect_modules import SafetyProtector  # type: ignore

        protector = SafetyProtector(
            use_com_guard=use_com_guard,
            use_collision_guard=use_collision_guard,
            verbose=False,
        )
        # SafetyProtector 加载失败时不抛异常而是静默直通，这里显式拦截
        missing = []
        if use_com_guard and protector._com_guard is None:
            missing.append("CoMGuard")
        if use_collision_guard and protector._collision_guard is None:
            missing.append("CollisionGuard")
        if missing:
            raise RuntimeError(f"{'/'.join(missing)} 未能创建，请检查 [SafetyModules] 日志（配置/URDF 路径）")
    except Exception as exc:
        try:
            conn.send(("error", f"安全检测子进程初始化失败: {exc}"))
        except Exception:
            pass
        return

    while True:
        try:
            msg = conn.recv()
        except (EOFError, OSError):
            break
        if msg is None:
            break
        kind, payload = msg
        if kind == "init":
            protector.init_safe_cmd(payload)
            conn.send(("init_ok", None))
        elif kind == "run":
            result = protector.run(payload)   # 不同版本返回 4 或 6 元组，只取前两项
            conn.send(("result", (bool(result[0]), list(result[1]))))

    try:
        conn.close()
    except Exception:
        pass
