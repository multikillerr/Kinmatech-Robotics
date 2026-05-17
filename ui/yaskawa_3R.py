"""
Minimal Yaskawa 3R position IK with quaternion orientation helpers.

- solve_position_3r(x, y, z) → analytic J1–J3 (deg)
- fk_orientation_quat(j1, j2, j3) → unit quaternion of the 3R chain
- quat_mul / quat_normalize utilities for composing tool orientation on top
"""

import numpy as np
from typing import Tuple


# ── Geometry (mm / rad) ──────────────────────────────────────────────────────
d1 = 450.0
a1 = 155.0
a2 = 614.0
a3 = float(np.hypot(200.0, 640.0))
alpha = float(np.arctan2(200.0, 640.0))  # link twist used in analytic solution


# ── Quaternion helpers ───────────────────────────────────────────────────────
def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, q = q1 * q2 (w,x,y,z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion (w, x, y, z)."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        idx = np.argmax(np.diag(R))
        if idx == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif idx == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return quat_normalize(np.array([w, x, y, z], dtype=float))


# ── Analytic IK (position only) ──────────────────────────────────────────────
def solve_position_3r(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Analytic 3R position IK in Yaskawa convention (degrees)."""
    theta1 = np.arctan2(y, x)
    exd = (x / np.cos(theta1)) - a1
    ezd = z - d1

    v2 = (exd * exd + ezd * ezd) - (a2 * a2 + a3 * a3)
    v3 = 2.0 * a2 * a3
    cos_theta3 = v2 / v3
    reach_eps = 1e-6
    if cos_theta3 < -1.0 - reach_eps or cos_theta3 > 1.0 + reach_eps:
        raise ValueError(
            "Position outside 3R workspace: "
            f"x={float(x):.3f}, y={float(y):.3f}, z={float(z):.3f}, "
            f"cos(theta3)={float(cos_theta3):.6f}"
        )
    v4 = np.clip(cos_theta3, -1.0, 1.0)

    theta3 = -np.arccos(v4)
    theta2 = np.arctan2(ezd, exd) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))

    theta1_y = np.degrees(theta1)
    theta2_y = np.degrees(theta2)
    theta3_y = np.degrees(theta3 + (np.pi / 2.0 - alpha))
    return theta1_y, theta2_y, theta3_y


# ── FK orientation (quaternion) for the 3R chain ─────────────────────────────
def fk_orientation_quat(theta1_deg: float, theta2_deg: float, theta3_deg: float) -> np.ndarray:
    """Return orientation quaternion (w,x,y,z) for the 3R chain."""
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    t3 = np.radians(theta3_deg)
    # Approximated chain: Rz(t1) * Ry(t2) * Ry(t3 - (np.pi/2 - alpha))
    R = rot_z(t1) @ rot_y(t2) @ rot_y(t3 - (np.pi / 2.0 - alpha))
    return rot_to_quat(R)


# ── Example ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    target = (500.0, 240.0, 712.0)
    j1, j2, j3 = solve_position_3r(*target)
    q = fk_orientation_quat(j1, j2, j3)
    print("J1,J2,J3 (deg):", j1, j2, j3)
    print("FK orientation quaternion (w,x,y,z):", q)

    # If you have a desired tool quaternion q_tool, compose: q_total = q_base * q_tool
    q_tool = quat_normalize(np.array([1, 0, 0, 0], dtype=float))  # identity example
    q_total = quat_mul(q, q_tool)
    print("Combined EE quaternion:", q_total)
