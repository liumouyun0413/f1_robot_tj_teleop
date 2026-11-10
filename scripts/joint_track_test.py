#!/usr/bin/env python3
"""
单关节周期跟踪测试（模拟遥操的连续插值指令）：
以当前 /hal/joint_states 为基线，对指定关节发布 q0 → q0+amp → q0 的余弦平滑往复轨迹，
周期 period 秒、重复 cycles 次，100 Hz 发布 14 关节指令，同时录 /hal/joint_states。
输出 CSV，并打印 跟踪滞后(互相关) / 幅值比 / 最大、RMS 跟踪误差 / 峰值速度。

用法: python3 joint_track_test.py --joint arm_l_j4 --amp 10 --period 2 --cycles 3 --out /tmp/track_L4.csv
"""
import argparse, csv, sys, time, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

ARM = [f"arm_l_j{i}" for i in range(1, 8)] + [f"arm_r_j{i}" for i in range(1, 8)]
LIMITS = {"arm_l_j1": (-178, 178), "arm_l_j2": (-94, 120), "arm_l_j3": (-178, 178), "arm_l_j4": (-145, 60),
          "arm_l_j5": (-178, 178), "arm_l_j6": (-60, 60), "arm_l_j7": (-90, 90),
          "arm_r_j1": (-178, 178), "arm_r_j2": (-94, 120), "arm_r_j3": (-178, 178), "arm_r_j4": (-145, 60),
          "arm_r_j5": (-178, 178), "arm_r_j6": (-60, 60), "arm_r_j7": (-90, 90)}
CTRL_DT = 0.01


class TrackTest(Node):
    def __init__(self, joint, amp, period, cycles, out):
        super().__init__("track_test")
        self.joint, self.amp, self.period, self.cycles, self.out = joint, amp, period, cycles, out
        self.idx = ARM.index(joint)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(JointState, "/motion_ctl/joint_ctl", rel)
        self.create_subscription(JointState, "/hal/joint_states", self.cb, qos_profile_sensor_data)
        self.lock = threading.Lock(); self.hal = None
        self.hal_rows = []; self.cmd_rows = []; self.recording = False

    def cb(self, m):
        d = dict(zip(m.name, m.position))
        q = np.array([d.get(n, np.nan) for n in ARM])
        with self.lock:
            self.hal = q
            if self.recording:
                self.hal_rows.append((time.time(), float(q[self.idx])))

    def send(self, q):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM; msg.position = [float(v) for v in q]; self.pub.publish(msg)
        if self.recording:
            self.cmd_rows.append((time.time(), float(q[self.idx])))

    def profile(self, t):
        """余弦平滑往复: 0 → amp → 0，周期 period；超出总时长后保持 0。"""
        if t < 0.0 or t >= self.cycles * self.period:
            return 0.0
        return self.amp * 0.5 * (1.0 - np.cos(2.0 * np.pi * t / self.period))

    def run(self):
        t0 = time.time()
        while self.hal is None or np.isnan(self.hal).any():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 5: print("no /hal/joint_states"); return False
        q0 = self.hal.copy()
        lo, hi = LIMITS[self.joint]
        if not (lo + 2 <= q0[self.idx] + self.amp <= hi - 2):
            print(f"target {q0[self.idx] + self.amp:.1f} out of driver limit {LIMITS[self.joint]}"); return False
        vmax_cmd = self.amp * np.pi / self.period
        print(f"baseline {self.joint} = {q0[self.idx]:.2f} deg, amp {self.amp:+.1f}, period {self.period}s x{self.cycles}, "
              f"cmd peak speed {vmax_cmd:.0f} deg/s")

        # 保持 1 s → 周期运动 → 再保持 1 s
        self.recording = True
        total = 1.0 + self.cycles * self.period + 1.0
        t_start = time.time(); next_tick = t_start
        while True:
            t = time.time() - t_start
            if t >= total: break
            q = q0.copy(); q[self.idx] = q0[self.idx] + self.profile(t - 1.0)
            self.send(q); rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += CTRL_DT
            time.sleep(max(0.0, next_tick - time.time()))
        self.recording = False

        with open(self.out, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["src", "t", "deg"])
            w.writerows(("cmd", t, v) for t, v in self.cmd_rows)
            w.writerows(("hal", t, v) for t, v in self.hal_rows)
        self.analyze()
        return True

    def analyze(self):
        tc = np.array([r[0] for r in self.cmd_rows]); c = np.array([r[1] for r in self.cmd_rows])
        th = np.array([r[0] for r in self.hal_rows]); h = np.array([r[1] for r in self.hal_rows])
        t0 = tc[0]; tc -= t0; th -= t0
        grid = np.arange(0.0, min(tc[-1], th[-1]), 0.001)           # 1 kHz 统一时间轴
        cg = np.interp(grid, tc, c) - c[0]; hg = np.interp(grid, th, h) - c[0]

        # 互相关求滞后：hal 相对 cmd 向后平移多少 ms 残差最小
        lags = np.arange(0, 400)
        err = np.array([np.sqrt(np.mean((hg[l:] - cg[:len(cg) - l]) ** 2)) for l in lags])
        lag_ms = int(lags[int(np.argmin(err))])
        e = cg - hg
        moving = np.abs(np.gradient(cg, 0.001)) > 1.0                 # 仅在指令运动段统计
        amp_ratio = (hg.max() - hg.min()) / (cg.max() - cg.min())
        v_hal = np.max(np.abs(np.diff(h) / np.diff(th)))
        print(f"\n=== {self.joint} 周期跟踪 amp {self.amp:+.0f} deg, T={self.period}s x{self.cycles}  "
              f"(cmd {len(tc)} @ {len(tc)/tc[-1]:.0f} Hz, hal {len(th)} @ {len(th)/th[-1]:.0f} Hz) ===")
        print(f"  跟踪滞后(互相关)  {lag_ms:4d} ms   |  对齐后残差 RMS {err.min():.2f} deg")
        print(f"  幅值比 hal/cmd    {amp_ratio:5.3f}  |  峰值速度 hal {v_hal:.0f} deg/s")
        print(f"  运动段跟踪误差    max {np.abs(e[moving]).max():.2f} deg | RMS {np.sqrt(np.mean(e[moving]**2)):.2f} deg")
        print(f"  静止段稳态误差    {np.mean(e[-500:]):+.2f} deg (最后 0.5 s)")
        sgn = np.sign(self.amp)
        for k in range(self.cycles):
            a = 1.0 + k * self.period; m = (grid >= a) & (grid < a + self.period)
            if not m.any(): continue
            ic = np.argmax(sgn * cg[m]); ih = np.argmax(sgn * hg[m])
            tpc = grid[m][ic]; tph = grid[m][ih]
            print(f"  周期{k+1}: cmd 峰 {cg[m][ic]:6.2f}° @ {tpc-a:.3f}s, hal 峰 {hg[m][ih]:6.2f}° @ {tph-a:.3f}s"
                  f" → 峰滞后 {(tph-tpc)*1e3:4.0f} ms")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="arm_l_j4"); ap.add_argument("--amp", type=float, default=10.0)
    ap.add_argument("--period", type=float, default=2.0); ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--out", default="/tmp/track_test.csv")
    a = ap.parse_args()
    rclpy.init(); n = TrackTest(a.joint, a.amp, a.period, a.cycles, a.out)
    try:
        ok = n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)
