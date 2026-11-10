#!/usr/bin/env python3
"""
外部观测探针：订阅关节指令与真机反馈话题，逐条记录接收时刻 / 消息时间戳 / 14 个臂关节角(度)。
不干预被测程序，可用于对比不同遥操程序的下发周期、卡顿与真机滞后。

用法: python3 ros_topic_probe.py --out /tmp/probe.csv --duration 180
CSV 列: topic,t_recv,t_stamp,n,q0..q13   (topic: cmd=/motion_ctl/joint_ctl, hal=/hal/joint_states)
"""
import argparse, csv, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

ARM = [f"arm_l_j{i}" for i in range(1, 8)] + [f"arm_r_j{i}" for i in range(1, 8)]


class Probe(Node):
    def __init__(self, out, cmd_topic):
        super().__init__("teleop_probe")
        self.f = open(out, "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(["topic", "t_recv", "t_stamp", "n"] + [f"q{i}" for i in range(14)])
        self.count = {"cmd": 0, "hal": 0}
        rel = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        be = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        # 指令话题同时挂可靠/尽力两种订阅，兼容不同发布者 QoS；同一发布者只会匹配其中一个
        self.create_subscription(JointState, cmd_topic, lambda m: self.cb("cmd", m), rel)
        self.create_subscription(JointState, cmd_topic, lambda m: self.cb("cmd", m), be)
        self.create_subscription(JointState, "/hal/joint_states", lambda m: self.cb("hal", m), qos_profile_sensor_data)

    def cb(self, tag, m):
        t = time.time()
        d = dict(zip(m.name, m.position))
        q = [d.get(n, float("nan")) for n in ARM]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.w.writerow([tag, f"{t:.6f}", f"{stamp:.6f}", len(m.name)] + [f"{v:.4f}" for v in q])
        self.count[tag] += 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/probe.csv"); ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--cmd-topic", default="/motion_ctl/joint_ctl")
    a = ap.parse_args()
    rclpy.init(); n = Probe(a.out, a.cmd_topic)
    t0 = time.time(); last = t0
    try:
        while time.time() - t0 < a.duration:
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - last >= 10:
                last = time.time(); n.f.flush()
                print(f"[{last - t0:5.0f}s] cmd {n.count['cmd']}  hal {n.count['hal']}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close(); n.destroy_node(); rclpy.shutdown()
        print(f"saved {a.out}: cmd {n.count['cmd']} hal {n.count['hal']}")
