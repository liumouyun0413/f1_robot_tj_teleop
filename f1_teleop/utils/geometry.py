"""几何工具：与 XRoboToolkit-Teleop-Sample-Python 的 utils/geometry.py 保持一致。"""

import meshcat.transformations as tf
import numpy as np

# PICO 头显坐标系 → 机器人世界坐标系
R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)


def is_valid_quaternion(quat, tol=1e-4):
    if not isinstance(quat, (list, tuple, np.ndarray)) or len(quat) != 4:
        return False
    if not np.all(np.isfinite(quat)):
        return False
    magnitude = np.sqrt(np.sum(np.square(quat)))
    return abs(magnitude - 1.0) <= tol


def quaternion_to_angle_axis(quat: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """四元数 (w,x,y,z) → 角轴向量 [ax*angle, ay*angle, az*angle]。"""
    q = np.array(quat, dtype=np.float64, copy=True)
    if q[0] < 0.0:
        q = -q
    w = q[0]
    vec_part = q[1:]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    if angle < eps:
        return np.zeros(3, dtype=np.float64)
    sin_half_angle = np.sin(angle / 2.0)
    if sin_half_angle < eps:
        return np.zeros(3, dtype=np.float64)
    axis = vec_part / sin_half_angle
    return axis * angle


def quat_diff_as_angle_axis(q1: np.ndarray, q2: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """q1 → q2 的旋转，以角轴向量表示（q2 = ΔQ * q1）。非法输入返回零向量。"""
    if not (is_valid_quaternion(q1, tol=eps) and is_valid_quaternion(q2, tol=eps)):
        print("Warning: Invalid quaternion input to quat_diff_as_angle_axis, returning zero delta.")
        return np.zeros(3, dtype=np.float64)
    q1_inv = tf.quaternion_inverse(q1)
    delta_q = tf.quaternion_multiply(q2, q1_inv)
    return quaternion_to_angle_axis(delta_q, eps)


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot: np.ndarray,
    eps: float = 1.0e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """在源位姿上叠加增量：位置直接相加，姿态用角轴转四元数左乘。"""
    target_pos = np.asarray(source_pos, dtype=np.float64) + np.asarray(delta_pos, dtype=np.float64)
    angle = float(np.linalg.norm(delta_rot))
    if angle > eps:
        axis = np.asarray(delta_rot) / angle
        rot_delta_quat = tf.quaternion_about_axis(angle, axis)
    else:
        rot_delta_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_rot = tf.quaternion_multiply(rot_delta_quat, source_rot)
    return target_pos, target_rot


def quat_wxyz_to_euler_deg(quat_wxyz, axes="sxyz") -> list[float]:
    q_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
    return np.rad2deg(tf.euler_from_quaternion(q_xyzw, axes=axes)).tolist()


def euler_deg_to_quat_wxyz(euler_deg, axes="sxyz") -> list[float]:
    r = np.deg2rad(euler_deg)
    q_xyzw = tf.quaternion_from_euler(r[0], r[1], r[2], axes=axes)
    return [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
