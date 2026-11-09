"""
数据采集：ros2 bag 录制管理 + 外部相机录制器（tianji）启停信号。

A 键开始 / B 键停止（由 F1TeleopController 调用 start()/stop()）。
录制开始后异步检查各 topic 是否有发布者；结束后异步校验 bag 中各 topic 消息数。
检查失败只打印告警，不终止遥操作。
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import time
from typing import List, Optional, Sequence

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from f1_teleop.hardware.interface.ros_context import RosContext


class BagRecorder:
    def __init__(self, bag_dir: str, topics: Sequence[str], storage: str = "mcap", validate: bool = True) -> None:
        self.bag_dir = bag_dir
        self.topics = list(topics)
        self.storage = storage
        self.validate = validate
        self._proc: Optional[subprocess.Popen] = None
        self.current_bag_path: Optional[str] = None
        os.makedirs(self.bag_dir, exist_ok=True)

    @property
    def is_recording(self) -> bool:
        return self._proc is not None

    def _next_episode_index(self) -> int:
        idx = -1
        for name in os.listdir(self.bag_dir):
            m = re.fullmatch(r"episode_(\d{6})", name)
            if m:
                idx = max(idx, int(m.group(1)))
        return idx + 1

    def start(self) -> bool:
        if self.is_recording:
            print("[Bag] 已在录制，忽略")
            return False
        episode = self._next_episode_index()
        self.current_bag_path = os.path.join(self.bag_dir, f"episode_{episode:06d}")
        cmd = ["ros2", "bag", "record", "--storage", self.storage, "-o", self.current_bag_path, *self.topics]
        try:
            self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"[Bag] 启动失败: {e}")
            self._proc = None
            return False
        print(f"[Bag] 开始录制 episode_{episode:06d} → {self.current_bag_path}")
        if self.validate:
            threading.Thread(target=self._check_publishers, daemon=True).start()
        return True

    def stop(self) -> None:
        if not self.is_recording:
            return
        proc, self._proc = self._proc, None
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        print("[Bag] 录制已停止")
        if self.validate and self.current_bag_path:
            threading.Thread(target=self._validate_bag, args=(self.current_bag_path,), daemon=True).start()

    # ---------------------------------------------------------------- #
    def _check_publishers(self) -> None:
        try:
            out = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, timeout=10).stdout
            active = {line.strip() for line in out.splitlines() if line.strip()}
        except Exception as e:
            print(f"[Bag] topic 列表获取失败: {e}")
            return
        missing = [t for t in self.topics if t not in active]
        no_pub: List[str] = []
        for t in self.topics:
            if t in active:
                try:
                    info = subprocess.run(["ros2", "topic", "info", t], capture_output=True, text=True, timeout=5).stdout
                    m = re.search(r"Publisher count:\s*(\d+)", info)
                    if m and int(m.group(1)) == 0:
                        no_pub.append(t)
                except Exception:
                    no_pub.append(t)
        if missing or no_pub:
            print("[Bag] ⚠️ topic 检查未通过（录制继续，但数据可能不完整）")
            for t in missing:
                print(f"       不存在: {t}")
            for t in no_pub:
                print(f"       无发布者: {t}")
        else:
            print("[Bag] topic 检查通过")

    def _validate_bag(self, bag_path: str) -> None:
        time.sleep(2.5)   # 等文件系统刷新
        mcaps = [f for f in os.listdir(bag_path) if f.endswith(".mcap")] if os.path.isdir(bag_path) else []
        if not mcaps:
            print(f"[Bag] ⚠️ 未找到 mcap 文件: {bag_path}")
            return
        try:
            out = subprocess.run(
                ["ros2", "bag", "info", os.path.join(bag_path, mcaps[0])],
                capture_output=True, text=True, timeout=15,
            ).stdout
        except Exception as e:
            print(f"[Bag] bag info 失败: {e}")
            return
        counts = {}
        for line in out.splitlines():
            m = re.search(r"Topic:\s*(\S+)\s*\|.*\|\s*Count:\s*(\d+)", line)
            if m:
                counts[m.group(1)] = int(m.group(2))
        missing = [t for t in self.topics if t not in counts]
        zero = [t for t in self.topics if counts.get(t, -1) == 0]
        if missing or zero:
            print(f"[Bag] ⚠️ {os.path.basename(bag_path)} 校验未通过（数据已保留）")
            for t in missing:
                print(f"       未写入: {t}")
            for t in zero:
                print(f"       0 条消息: {t}")
        else:
            print(f"[Bag] {os.path.basename(bag_path)} 校验通过")


class TianjiRecorderClient:
    """向外部相机录制器发布 start/stop 字符串信号。"""

    def __init__(self, ros: RosContext, start_topic: str, stop_topic: str) -> None:
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._start_pub = ros.node.create_publisher(String, start_topic, qos)
        self._stop_pub = ros.node.create_publisher(String, stop_topic, qos)

    def _send(self, pub, text: str) -> None:
        msg = String()
        msg.data = text
        for _ in range(3):   # 多发几次防丢
            pub.publish(msg)
            time.sleep(0.001)

    def start(self) -> None:
        self._send(self._start_pub, "start from teleop")

    def stop(self) -> None:
        self._send(self._stop_pub, "stop from teleop")


class RecordingManager:
    """把 bag 录制与外部录制器合并为一个开关。"""

    def __init__(self, bag: Optional[BagRecorder], tianji: Optional[TianjiRecorderClient]) -> None:
        self.bag = bag
        self.tianji = tianji
        self.is_recording = False

    def start(self) -> None:
        if self.is_recording:
            return
        self.is_recording = True
        if self.tianji:
            self.tianji.start()
        if self.bag:
            self.bag.start()
        print("[Record] 数据采集开始")

    def stop(self) -> None:
        if not self.is_recording:
            return
        self.is_recording = False
        if self.tianji:
            self.tianji.stop()
        if self.bag:
            self.bag.stop()
        print("[Record] 数据采集停止")
