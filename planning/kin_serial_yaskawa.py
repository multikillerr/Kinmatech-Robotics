"""
Lightweight kin_serial for the Yaskawa 3R analytic base with a spherical wrist.

J1–J3 are solved analytically (ui.yaskawa_3R). J4–J6 are derived by matching
the desired tool orientation quaternion relative to the base 3R orientation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from ui.yaskawa_3R import solve_position_3r, fk_orientation_quat


def _quat_wxyz_from_xyzw(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z], dtype=float)


def _quat_mul_wxyz(q1, q2):
    """Hamilton product for wxyz quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def kin_engine(x, y, z, a, b, c):
    """
    IK interface compatible with the existing kin_engine signature.

    Parameters
    ----------
    x, y, z : mm
    a, b, c : deg
        Legacy engine convention used by the UI: (rz, ry, rx).
    """
    rx = float(c)
    ry = float(b)
    rz = float(a)

    # Base position IK
    try:
        j1, j2, j3 = solve_position_3r(x, y, z)
    except ValueError:
        return None

    # Orientation decomposition
    q_base = fk_orientation_quat(j1, j2, j3)          # wxyz
    q_des = _quat_wxyz_from_xyzw(R.from_euler('xyz', [rx, ry, rz], degrees=True).as_quat())

    # Wrist quaternion to reach desired orientation: q_des = q_base * q_wrist
    q_base_conj = np.array([q_base[0], -q_base[1], -q_base[2], -q_base[3]], dtype=float)
    q_wrist = _quat_mul_wxyz(q_base_conj, q_des)
    q_wrist_rot = R.from_quat([q_wrist[1], q_wrist[2], q_wrist[3], q_wrist[0]])

    j4, j5, j6 = q_wrist_rot.as_euler('xyz', degrees=True)

    return (
        float(j1), float(j2), float(j3),
        float(j4), float(j5), float(j6),
    )


def run_f_kin(j1, j2, j3, j4, j5, j6):
    """
    Forward kinematics: position from analytic 3R, orientation from composed wrist.
    Returns (x, y, z, rx, ry, rz) with angles in degrees, position in mm.
    """
    # Position FK from 3R geometry (approximate using the same parameters)
    t1 = np.radians(j1)
    t2 = np.radians(j2)
    t3 = np.radians(j3)
    # simple planar forward model consistent with solve_position_3r assumptions
    d1 = 450.0
    a1 = 155.0
    a2 = 614.0
    a3 = float(np.hypot(200.0, 640.0))
    alpha = float(np.arctan2(200.0, 640.0))
    x = (a1 + a2 * np.cos(t2) + a3 * np.cos(t2 + t3 - (np.pi / 2.0 - alpha))) * np.cos(t1)
    y = (a1 + a2 * np.cos(t2) + a3 * np.cos(t2 + t3 - (np.pi / 2.0 - alpha))) * np.sin(t1)
    z = d1 + a2 * np.sin(t2) + a3 * np.sin(t2 + t3 - (np.pi / 2.0 - alpha))

    q_base_wxyz = fk_orientation_quat(j1, j2, j3)
    q_base = R.from_quat([q_base_wxyz[1], q_base_wxyz[2], q_base_wxyz[3], q_base_wxyz[0]])
    q_wrist = R.from_euler('xyz', [j4, j5, j6], degrees=True)
    q_total = q_base * q_wrist
    rx, ry, rz = q_total.as_euler('xyz', degrees=True)

    # Match the historical engine convention: (x, y, z, rz, ry, rx).
    return (
        round(x, 3), round(y, 3), round(z, 3),
        round(rz, 3), round(ry, 3), round(rx, 3),
    )
