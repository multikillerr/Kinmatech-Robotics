from planning.visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
import threading

dh_params = np.array([[0.575, 0.175, 0.5 * np.pi, 0.],
                      [0., 0.890, np.pi, 0.5 * np.pi],
                      [0., 0.050, -0.5 * np.pi, 0.],
                      [1.035, 0., -0.5 * np.pi, 0.0],
                      [0.0, 0., 0.5 * np.pi, 0.],
                      [0.185, 0., 0., 0.]])

robot = RobotSerial(dh_params, max_iter=100)
_KIN_LOCK = threading.RLock()

# ── Wrist-singularity helpers ────────────────────────────────────────
# When J5 is near 0° (or 180°), J4 and J6 become collinear.
# Only their *sum* (J4+J6) matters physically.  An unconstrained IK
# solver can freely redistribute the sum, causing sudden flips.
#
# Strategy:
#   1. Detect: |sin(J5)| < threshold
#   2. Freeze: pin J4 to its seed value, let J6 absorb the remainder
#   3. Select: among valid IK results, prefer minimum wrapped joint distance

_WRIST_SING_THRESHOLD_DEG = 8.0  # J5 within ±8° of 0° or 180°

# ── Shoulder/overhead-singularity helpers ────────────────────────────────────
# When the TCP (or wrist centre) lies close to the J1 rotation axis the
# Jacobian row for J1 approaches zero — J1 has no effect on TCP position so
# the numerical IK can assign any value.  We detect this from the FK of the
# seed (computed anyway before robot.inverse) and pin J1 to the seed value.
# 200 mm is a conservative radius; this robot's links are ~2 m total.
_SHOULDER_SING_RADIUS_MM = 200.0


def _wrap_angle_deg(a: float) -> float:
    """Wrap angle to [-180, +180]."""
    return (a + 180.0) % 360.0 - 180.0


def _nearest_angle_deg(angle: float, reference: float) -> float:
    """Return the value of `angle` (mod 360) that is closest to `reference`."""
    diff = _wrap_angle_deg(angle - reference)
    return reference + diff


def _wrapped_joint_distance(a, b) -> float:
    """Sum of absolute wrapped differences across all joints (degrees)."""
    return sum(abs(_wrap_angle_deg(ai - bi)) for ai, bi in zip(a, b))


def _is_wrist_singular(j5_deg: float) -> bool:
    """True when J5 is close enough to 0° or 180° to cause wrist flip."""
    j5_mod = abs(_wrap_angle_deg(j5_deg))
    return j5_mod < _WRIST_SING_THRESHOLD_DEG or abs(j5_mod - 180.0) < _WRIST_SING_THRESHOLD_DEG


def _enforce_wrist_continuity(joints_deg, seed_deg):
    """Near wrist singularity: pin J4 to seed, push remainder into J6.

    When J5 ≈ 0, the physical wrist rotation is fully described by
    (J4 + J6).  Any split of that sum into J4/J6 is kinematically
    equivalent.  We pin J4 to the seed value for continuity and
    let J6 absorb the rest.

    Returns a *new* list (degrees) with corrected J4/J6.
    """
    j = list(joints_deg)
    s = list(seed_deg)
    if not _is_wrist_singular(j[4]):
        return j
    # Preserve the physical wrist sum
    wrist_sum = j[3] + j[5]
    j[3] = s[3]                                     # pin J4
    j[5] = _wrap_angle_deg(wrist_sum - j[3])         # J6 absorbs rest
    return j


def _snap_to_seed(joints_deg, seed_deg):
    """Shift each joint to the ±360° representative closest to its seed value.

    The numerical IK solver's simplify_angles wraps to [-180, +180],
    which can jump 360° from the seed.  This undoes that wrapping.
    """
    out = list(joints_deg)
    for i in range(len(out)):
        out[i] = _nearest_angle_deg(out[i], seed_deg[i])
    return out


# Joint limits (degrees): same as the Teensy firmware limits
_JOINT_LIMITS = [
    (-170.0, 170.0),   # J1
    (-90.0,  130.0),   # J2
    (-90.0,  180.0),   # J3
    (-200.0, 200.0),   # J4
    (-120.0, 120.0),   # J5
    (-360.0, 360.0),   # J6
]


def _normalize_to_limits(joints_deg):
    """Shift joints by multiples of ±360° to stay within hardware limits.

    For revolute joints any ±360° representative is the same physical
    configuration.  After _snap_to_seed the joint may have wound beyond
    the limit, so we try shifting by ±N×360° to land inside the limit.
    If no representative fits, the value is returned unchanged (the
    downstream joint-limit validator will flag it).
    """
    out = list(joints_deg)
    for i, (lo, hi) in enumerate(_JOINT_LIMITS):
        if lo <= out[i] <= hi:
            continue
        # Try ±N×360 shifts (up to 3 full turns) to land inside limits
        best = out[i]
        for n in range(1, 4):
            for sign in (-1, 1):
                candidate = out[i] + sign * n * 360.0
                if lo <= candidate <= hi:
                    best = candidate
                    break
            if lo <= best <= hi:
                break
        out[i] = best
    return out

#xyz = np.array([[1.395], [0.], [1.515]])
#abc = np.array([0.5 * pi, 0.5 * pi, 0.5 * pi])
#end = Frame.from_euler_3(abc, xyz)
#robot.inverse(end)

def kin_engine(x,y,z,a,b,c, seed=None, max_iter=None):
    x=x/1000
    y=y/1000
    z=z/1000
    a=a/180*pi
    b=b/180*pi
    c=c/180*pi
    xyz=np.array([[x],[y],[z]])
    abc=np.array([a,b,c])
    end=Frame.from_euler_3(abc,xyz)
    with _KIN_LOCK:
        saved_max_iter = None
        try:
            if max_iter is not None:
                saved_max_iter = robot.max_iter
                robot.max_iter = max_iter
            if seed is not None:
                f_seed = robot.forward(np.array(seed, dtype=float) / 57.2958)
                # Detect shoulder singularity from the seed FK position.
                try:
                    pos_m = f_seed.t_3_1.reshape(3)
                    horiz_r_mm = float(np.sqrt(float(pos_m[0])**2 + float(pos_m[1])**2)) * 1000.0
                    _near_j1_axis = horiz_r_mm < _SHOULDER_SING_RADIUS_MM
                except Exception:
                    _near_j1_axis = False
            else:
                _near_j1_axis = False
            robot.inverse(end)
            raw = [float(robot.axis_values[i] * 57.2958) for i in range(6)]

            if seed is not None:
                # 1. Snap each joint to the ±360° value closest to seed
                #    (undoes simplify_angles wrapping)
                raw = _snap_to_seed(raw, seed)
                # 2. Near wrist singularity: pin J4, let J6 absorb
                raw = _enforce_wrist_continuity(raw, seed)
                # 3. Final snap after wrist correction
                raw = _snap_to_seed(raw, seed)
                # 4. Near shoulder singularity: pin J1 to seed — J1 is
                #    undefined when the arm passes through the J1 axis, so
                #    any IK result is as valid as the seed value; keeping it
                #    at seed prevents solver-noise-driven base rotation.
                if _near_j1_axis:
                    raw[0] = float(seed[0])

            j1, j2, j3, j4, j5, j6 = raw
            return round(j1, 3), round(j2, 3), round(j3, 3), round(j4, 3), round(j5, 3), round(j6, 3)
        except Exception:
            return None
        finally:
            if saved_max_iter is not None:
                robot.max_iter = saved_max_iter

def run_f_kin(angle1, angle2, angle3, angle4, angle5, angle6):
    # Convert angles from degrees to radians
    angle1 = angle1 / 57.2958
    angle2 = angle2 / 57.2958
    angle3 = angle3 / 57.2958
    angle4 = angle4 / 57.2958
    angle5 = angle5 / 57.2958
    angle6 = angle6 / 57.2958
    #print(angle1,angle2,angle3,angle4,angle5,angle6)
    # Store all angles as an array
    thetas = np.array([angle1, angle2, angle3, angle4, angle5, angle6])

    with _KIN_LOCK:
        # Forward kinematics calculation
        f = robot.forward(thetas)

        # Extract position and orientation
        position = f.t_3_1.reshape([3, ])  # 3x1 to 1x3 array
        orientation = f.euler_3  # Assuming euler_3 is a 3-element array

        # Correct the np.concatenate usage
        array_of_orientation_and_position = np.concatenate((position, orientation))
        array_of_orientation_and_position[:3] *= 1000
        array_of_orientation_and_position[3:] *=57.2958

        array_of_orientation_and_position = np.round(array_of_orientation_and_position, 2)
        return array_of_orientation_and_position

#result = run_f_kin(30.77,0.00,2.02,-110.41,-40.1,-56.1)
#print(result)
#robot.show()
#print(kin_engine(150,0.00,300,0,90,0))