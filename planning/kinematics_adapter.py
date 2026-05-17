"""
Single-entry kinematics adapter for GUI pose convention.

The rest of the app should only use poses in GUI order:
    (x, y, z, rx, ry, rz)

This module is the only place that translates to the backend engine order:
    (x, y, z, rz, ry, rx)
"""

import numpy as np
from typing import Optional, Sequence, Tuple

from planning.kin_serial import kin_engine, run_f_kin, robot, _KIN_LOCK

Pose6 = Tuple[float, float, float, float, float, float]

# Thread-local storage for the last solved joints, used as IK seed
import threading
_ik_seed = threading.local()
KINEMATICS_SIGNATURE = (
    "dh_visual_kinematics_v1|"
    "gui_pose=x,y,z,rx,ry,rz|"
    "engine_pose=x,y,z,rz,ry,rx"
)


def _coerce_pose(pose: Sequence[float]) -> Pose6:
    if len(pose) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(pose)}")
    return tuple(float(v) for v in pose)


def _coerce_joints(joints: Sequence[float]) -> Pose6:
    if len(joints) != 6:
        raise ValueError(f"Expected 6 joint values, got {len(joints)}")
    return tuple(float(v) for v in joints)


def gui_pose_to_engine(pose: Sequence[float]) -> Pose6:
    x, y, z, rx, ry, rz = _coerce_pose(pose)
    return (x, y, z, rz, ry, rx)


def engine_pose_to_gui(pose: Sequence[float]) -> Pose6:
    x, y, z, rz, ry, rx = _coerce_pose(pose)
    return (x, y, z, rx, ry, rz)


def solve_ik_gui(pose: Sequence[float], seed: Optional[Sequence[float]] = None, max_iter: Optional[int] = None) -> Optional[Tuple[float, float, float, float, float, float]]:
    engine_pose = gui_pose_to_engine(pose)
    if seed is None:
        seed = getattr(_ik_seed, 'joints', None)
    result = kin_engine(*engine_pose, seed=seed, max_iter=max_iter)
    if result is None:
        return None
    joints = _coerce_joints(result)
    _ik_seed.joints = joints
    return joints


def solve_fk_gui(joints: Sequence[float]) -> Pose6:
    result = run_f_kin(*_coerce_joints(joints))
    return engine_pose_to_gui(result)


def solve_visual_chain_gui(joints: Sequence[float]):
    """
    Return DH-consistent link positions for the 3D visualizer.

    Uses axis_frames from the visual_kinematics RobotSerial to obtain
    per-joint positions directly from the DH chain.

    Returns
    -------
    positions_m : ndarray, shape (7, 3)
        Base origin + six joint/TCP positions, in metres.
    orientations : list of ndarray, each shape (3,)
        Per-link orientation (only the last entry is meaningful).
        The last entry is the TCP orientation in GUI order (rx, ry, rz) degrees.
    """
    joint_values = _coerce_joints(joints)
    thetas = np.radians(joint_values)

    with _KIN_LOCK:
        f = robot.forward(thetas)
        frames = list(robot.axis_frames)

    # Build positions: base (origin) + 6 joint frames
    positions = [np.array([0.0, 0.0, 0.0], dtype=float)]
    for frame in frames:
        positions.append(frame.t_3_1.flatten().copy())
    positions_m = np.array(positions, dtype=float)

    # TCP orientation from FK result: euler_3 is ZYX radians → (rz, ry, rx)
    euler_zyx_deg = np.degrees(f.euler_3)  # [rz, ry, rx]
    tcp_orient_gui = np.array([euler_zyx_deg[2], euler_zyx_deg[1], euler_zyx_deg[0]], dtype=float)

    orientations = [np.zeros(3, dtype=float) for _ in range(len(frames) - 1)] + [tcp_orient_gui]
    return positions_m, orientations
